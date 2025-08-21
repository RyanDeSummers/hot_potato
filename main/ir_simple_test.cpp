/// Optimized IR Test - Sender + Receiver on same device
// Receiver always running, button press sends "Z" signal
// Features: CPU-minimal RX, prebuilt TX symbols, validation-based decoding

#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include "esp_rom_crc.h"
#include "M5GFX.h"
#include "led_strip.h"
#include "ir_simple_test.h"

#define IR_TX_GPIO GPIO_NUM_26
#define IR_RX_GPIO GPIO_NUM_36

#define CARRIER_FREQ 38000 // 38 kHz

#define TX_BYTE 'Z' // 0x5A

#ifndef IR_RX_BACKEND_UART
#define IR_RX_BACKEND_UART 1   // 1 = use UART2 for IR RX, 0 = use existing RMT RX
#endif

// UART RX config (only used when IR_RX_BACKEND_UART == 1)
#define IR_UART_PORT       UART_NUM_2
#define IR_UART_RX_GPIO    IR_RX_GPIO   // your TSOP pin is already GPIO36

static const char *TAG = "IR_SIMPLE_TEST_OPT";

// Use external display instance from main.cpp
extern M5GFX display;
extern led_strip_handle_t led_strip;

// RMT handles
static rmt_channel_handle_t rmt_tx_chan = NULL;
static rmt_channel_handle_t rmt_rx_chan = NULL;
static rmt_encoder_handle_t tx_copy_encoder = NULL;

// RX state
static volatile bool rx_inflight = false;
static volatile size_t rx_symbol_count = 0;
static SemaphoreHandle_t rx_done_sem = NULL;
static QueueHandle_t rx_byte_queue = NULL;

// Button functionality removed - using automatic beacon instead

// Display state
static bool display_needs_update = true;
static uint8_t last_sent_byte = 0;
static uint8_t last_received_byte = 0;
static bool has_received = false;

// UI hex strings
static char g_tx_hex[128];      // e.g., "5A 54"
static char g_rx_work_hex[256]; // RECEIVING (this capture)
static char g_rx_last_hex[256]; // LAST RECEIVED

// MAC address for display
static char g_mac_line[32] = {0};

// Our MAC for self-filtering
static uint8_t g_self_mac[6] = {0};

// ---- Role state ----
typedef enum { ROLE_HOLDER, ROLE_RUNNER, ROLE_PENDING } role_t; // +PENDING
static volatile role_t g_role = START_AS_HOLDER ? ROLE_HOLDER : ROLE_RUNNER;
static uint16_t g_ir_seq = 0;          // holder++ each beacon
static uint16_t g_pending_seq = 0;     // runner's claimed seq
static uint8_t g_no_tagback_peer[6] = {0};
static int64_t g_no_tagback_until_us = 0;
static int64_t g_holder_startup_until_us = 0;
static uint8_t g_last_peer_mac[6] = {0};

// Per-seq grant guard (holder-side sequence gating)
static bool     g_granted_seq_valid = false;
static uint16_t g_granted_seq = 0;
static uint16_t g_last_beacon_seq_tx = 0;

// Forward declarations
static void role_beacon_task(void *arg);
static esp_err_t espnow_send_ack_to(const uint8_t dst_mac[6], uint8_t seq);
static esp_err_t espnow_send_pass_req(const uint8_t* holder_mac, uint16_t seq);
static esp_err_t espnow_send_pass_grant(const uint8_t* winner_mac, uint16_t seq);
static esp_err_t espnow_send_state_sync(uint16_t seq, const uint8_t* new_holder_mac);
static void update_led_state(void);

// PASS_REQ timer callbacks
static void pass_req_timer_callback(void* arg);
static void retry_timer_callback(void* arg);

// Slot picker (deterministic)
typedef struct { uint8_t slot; uint16_t jitter_us; } slot_pick_t;
static inline slot_pick_t pick_slot(uint16_t seq, const uint8_t holder[6], const uint8_t mine[6]) {
    // If seq==0 (old payload with no seq), fold in time so it's not constant
    if (seq == 0) {
        uint16_t t = (uint16_t)(esp_timer_get_time() & 0xFFFF);
        seq = (uint16_t)(seq + t);
    }
    uint8_t buf[2+6+6];
    buf[0] = (uint8_t)(seq & 0xFF); buf[1] = (uint8_t)(seq >> 8);
    memcpy(&buf[2], holder, 6);
    memcpy(&buf[8], mine,   6);
    uint16_t h16 = esp_rom_crc16_le(0, buf, sizeof(buf));
    uint8_t  h8  = esp_rom_crc8_le(0, buf, sizeof(buf));
    slot_pick_t r;
    r.slot = (uint8_t)(h16 % SLOTS);
    r.jitter_us = (uint16_t)(h8 % SLOT_JITTER_MAX_US);
    return r;
}

// ---- Dedupe state ----
typedef struct {
    uint8_t  mac[6];
    uint16_t last_seq;   // valid only if a SEQ is present; otherwise ignored
    int64_t  last_us;    // last accepted time (µs)
    bool     used;
} dedupe_entry_t;
static dedupe_entry_t g_dedupe[IR_DEDUPE_CAPACITY] = {};

// Per-holder dedupe (runner, IR decode path)
typedef struct { 
    uint8_t mac[6]; 
    uint16_t last_seq; 
    bool used; 
} seq_entry_t;

#ifndef DEDUPE_SLOTS
#define DEDUPE_SLOTS 12   // a bit > max players
#endif
static seq_entry_t g_seq_seen[DEDUPE_SLOTS] = {};

static inline bool dedupe_seen_and_update(const uint8_t holder[6], uint16_t seq) {
    int free_i = -1;
    for (int i=0;i<DEDUPE_SLOTS;i++){
        if (g_seq_seen[i].used && memcmp(g_seq_seen[i].mac, holder, 6)==0) {
            // treat seq as uint16 wrap-around: accept strictly newer
            uint16_t prev = g_seq_seen[i].last_seq;
            bool newer = (uint16_t)(seq - prev) != 0;   // not equal → newer or wrapped
            g_seq_seen[i].last_seq = seq;
            return !newer; // true => drop duplicate
        }
        if (!g_seq_seen[i].used && free_i<0) free_i=i;
    }
    if (free_i>=0) {
        memcpy(g_seq_seen[free_i].mac, holder, 6);
        g_seq_seen[free_i].last_seq = seq;
        g_seq_seen[free_i].used = true;
    } else {
        // simple LRU: overwrite index 0
        memcpy(g_seq_seen[0].mac, holder, 6);
        g_seq_seen[0].last_seq = seq;
        g_seq_seen[0].used = true;
    }
    return false;
}

static int dedupe_find(const uint8_t mac[6]) {
    for (int i = 0; i < IR_DEDUPE_CAPACITY; ++i)
        if (g_dedupe[i].used && memcmp(g_dedupe[i].mac, mac, 6) == 0) return i;
    return -1;
}

static int dedupe_oldest_or_free(void) {
    int idx = -1; int64_t oldest = INT64_MAX;
    for (int i = 0; i < IR_DEDUPE_CAPACITY; ++i) {
        if (!g_dedupe[i].used) return i;
        if (g_dedupe[i].last_us < oldest) { oldest = g_dedupe[i].last_us; idx = i; }
    }
    return (idx < 0) ? 0 : idx;
}

// Returns true if this frame should be dropped as duplicate.
// If a SEQ is present, drop exact (MAC,SEQ) repeats. Otherwise use time window per MAC.
static bool dedupe_should_drop_and_update(const uint8_t mac[6], bool has_seq, uint16_t seq) {
    const int64_t now = esp_timer_get_time(); // µs
    int idx = dedupe_find(mac);
    if (idx < 0) {
        idx = dedupe_oldest_or_free();
        memcpy(g_dedupe[idx].mac, mac, 6);
        g_dedupe[idx].used = true;
        g_dedupe[idx].last_seq = seq;
        g_dedupe[idx].last_us  = now;
        return false;
    }
    if (has_seq) {
        if (g_dedupe[idx].last_seq == seq) return true; // exact repeat
        g_dedupe[idx].last_seq = seq;
        g_dedupe[idx].last_us  = now;
        return false;
    } else {
        if ((now - g_dedupe[idx].last_us) < (int64_t)IR_DEDUPE_WINDOW_MS * 1000) return true;
        g_dedupe[idx].last_us = now;
        return false;
    }
}

// ESP-NOW ACK globals
static bool g_espnow_ready = false;
static QueueHandle_t g_ack_queue = nullptr;   // posts sender MACs on ACK rx
static char g_ack_hex[64] = {0};              // "aa bb cc dd ee ff"

// ESP-NOW message types
#define MSG_PASS_REQ   0xA1  // runner->holder (you already use 0xA1)
#define MSG_PASS_GRANT 0xA2  // holder->winner
#define MSG_STATE_SYNC 0xA3  // holder->all (optional now)

// PASS_REQ slotting
typedef struct {
    uint16_t seq;
    uint8_t holder_mac[6];
} pass_req_data_t;

static QueueHandle_t g_pass_req_queue = nullptr;  // posts PASS_REQ data
static esp_timer_handle_t g_pass_req_timer = nullptr;
static esp_timer_handle_t g_retry_timer = nullptr;
static pass_req_data_t g_pending_pass_req = {{0}, {0}};

// One-shot timer to send the delayed ACK/claim
static esp_timer_handle_t g_ack_slot_timer = NULL;
static uint8_t  g_ack_slot_dst[6] = {0};
static uint8_t  g_ack_slot_seq    = 0;

// Beacon cadence (esp_timer) → notifies role_beacon_task to send
static TaskHandle_t        g_beacon_task  = NULL;
static esp_timer_handle_t  g_beacon_timer = NULL;

// -------- IR RX gating (role-driven) --------
static volatile bool g_rx_enabled = true;   // RUNNER listens, HOLDER not

void ir_rx_set_enabled(bool en) {
    g_rx_enabled = en;
    ESP_LOGI("IR_RX", "%s", en ? "ENABLED (listening)" : "DISABLED (holder; not listening)");
}

// PASS_REQ timer callback implementations
static void pass_req_timer_callback(void* arg) {
    // Post to queue for processing in main task
    uint8_t dummy = 1;
    xQueueSend(g_pass_req_queue, &dummy, 0);
}

static void retry_timer_callback(void* arg) {
    // Post to queue for retry processing
    uint8_t dummy = 1;
    xQueueSend(g_pass_req_queue, &dummy, 0);
}

static void ack_slot_timer_cb(void *arg) {
    // Copy out in case send triggers logs that preempt this task
    uint8_t mac[6]; memcpy(mac, g_ack_slot_dst, 6);
    uint8_t seq = g_ack_slot_seq;
    // Send existing 0xA1 claim/ACK (peer mgmt already in espnow_send_ack_to)
    (void)espnow_send_ack_to(mac, seq);
}

static void beacon_timer_cb(void* arg) {
    // Runs in esp_timer task context; don't block here.
    if (g_beacon_task) {
        xTaskNotifyGive(g_beacon_task);  // wake the beacon task
    }
}

// Prebuilt buffer for current TX byte (e.g., 'Z')
static rmt_symbol_word_t tx_uart_buf[16];
static size_t tx_uart_len = 0;

// Forward declarations
static void hex_append(char *dst, size_t cap, const uint8_t *data, size_t len);

// Helper to ensure NVS is ready (handles "no free pages" / version change)
static void ensure_nvs_ready(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(err);
    }
}

// Initialize Wi-Fi STA and ESP-NOW on a fixed channel
static esp_err_t espnow_init_once(void) {
    if (g_espnow_ready) return ESP_OK;
    
    // NVS must be ready before any Wi-Fi calls
    ensure_nvs_ready();
    
    // Netif + default loop might already exist (M5 libs, etc.)—tolerate that.
    esp_err_t err;
    err = esp_netif_init(); 
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(err);
    err = esp_event_loop_create_default(); 
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(err);
    // Create default STA netif if not created already
    if (esp_netif_create_default_wifi_sta() == NULL) {
        // It may already exist—ignore.
    }

    wifi_init_config_t wic = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wic));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_start());
    // Lower latency a bit
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    // Ensure all devices are on the same channel
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE("ESPNOW", "esp_now_init failed: %s", esp_err_to_name(err));
        return err;
    }
    
    // Optional: set a PMK later if you want encryption
    // ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t*)"0123456789abcdef0123456789abcdef"));

    // Send status callback (logs OK/FAIL)
    err = esp_now_register_send_cb([](const uint8_t* mac, esp_now_send_status_t st){
        ESP_LOGI("ESPNOW", "send to %02X:%02X:%02X:%02X:%02X:%02X -> %s",
                 mac[0],mac[1],mac[2],mac[3],mac[4],mac[5],
                 st==ESP_NOW_SEND_SUCCESS ? "OK":"FAIL");
    });
    if (err != ESP_OK) {
        ESP_LOGE("ESPNOW", "esp_now_register_send_cb failed: %s", esp_err_to_name(err));
        return err;
    }

    // Receive callback (post sender MAC to UI queue when ACK arrives)
    err = esp_now_register_recv_cb([](const esp_now_recv_info* info, const uint8_t* data, int len){
        if (len < 1) return;
        
        switch (data[0]) {
            case MSG_PASS_REQ: {
                // PASS_REQ: [0xA1 | seq_lo | seq_hi | holder_mac(6)]
                if (len >= 10 && g_role == ROLE_HOLDER) {
                    int64_t now = esp_timer_get_time();
                    
                    // Check cooldown
                    if (memcmp(info->src_addr, g_no_tagback_peer, 6) == 0 && 
                        now < g_no_tagback_until_us) {
                        int32_t remaining_ms = (int32_t)((g_no_tagback_until_us - now) / 1000);
                        ESP_LOGI("HOLDER", "Ignoring PASS_REQ from cooldown peer, remaining %ld ms", remaining_ms);
                        return;
                    }
                    
                    uint16_t seq = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
                    uint8_t holder_mac[6];
                    memcpy(holder_mac, &data[3], 6);
                    
                    // Only accept PASS_REQ aimed at me
                    if (memcmp(holder_mac, g_self_mac, 6) != 0) {
                        ESP_LOGD("NET", "drop PASS_REQ: not for me");
                        return;
                    }
                    
                    // Holder-side sequence gating: only accept PASS_REQ for current beacon seq
                    if (seq != g_last_beacon_seq_tx) {
                        ESP_LOGD("NET", "drop PASS_REQ: seq=%u != current=%u", seq, g_last_beacon_seq_tx);
                        return;
                    }
                    
                    // First valid request per seq wins
                    if (g_granted_seq_valid && g_granted_seq == seq) {
                        ESP_LOGD("NET", "drop PASS_REQ: already granted seq=%u", seq);
                        return;
                    }
                    
                    ESP_LOGI("HOLDER", "PASS_REQ from %02X:%02X:%02X:%02X:%02X:%02X seq=%u",
                             info->src_addr[0],info->src_addr[1],info->src_addr[2],
                             info->src_addr[3],info->src_addr[4],info->src_addr[5], seq);
                    
                    // Mark this sequence as granted
                    g_granted_seq_valid = true;
                    g_granted_seq = seq;
                    
                    // Send PASS_GRANT to winner
                    espnow_send_pass_grant(info->src_addr, seq);
                    
                    // Broadcast STATE_SYNC
                    espnow_send_state_sync(seq, info->src_addr);
                    
                    // Flip to RUNNER
                    memcpy(g_last_peer_mac, info->src_addr, 6);
                    memcpy(g_no_tagback_peer, info->src_addr, 6);
                    g_no_tagback_until_us = now + (int64_t)PASS_COOLDOWN_MS * 1000;
                    ESP_LOGI("CORE", "cooldown with %02X:%02X:%02X:%02X:%02X:%02X for %d ms",
                             info->src_addr[0], info->src_addr[1], info->src_addr[2],
                             info->src_addr[3], info->src_addr[4], info->src_addr[5], PASS_COOLDOWN_MS);
                    g_role = ROLE_RUNNER;
                    ir_rx_set_enabled(true);
                    update_led_state();
                    display_needs_update = true;
                    
                    ESP_LOGI("HOLDER", "GRANT -> %02X:%02X:%02X:%02X:%02X:%02X seq=%u",
                             info->src_addr[0],info->src_addr[1],info->src_addr[2],
                             info->src_addr[3],info->src_addr[4],info->src_addr[5], seq);
                }
                break;
            }
            
            case MSG_PASS_GRANT: {
                // PASS_GRANT: [0xA2 | seq_lo | seq_hi | new_holder_mac(6)]
                if (len >= 10 && g_role == ROLE_PENDING) {
                    uint16_t seq = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
                    uint8_t new_holder_mac[6];
                    memcpy(new_holder_mac, &data[3], 6);
                    
                    if (seq == g_pending_seq && memcmp(new_holder_mac, g_self_mac, 6) == 0) {
                        // We won! Commit to HOLDER
                        esp_timer_stop(g_pass_req_timer);
                        esp_timer_stop(g_retry_timer);
                        int64_t now = esp_timer_get_time();
                        g_role = ROLE_HOLDER;
                        ir_rx_set_enabled(false);
                        g_holder_startup_until_us = now + (int64_t)HOLDER_STARTUP_DELAY_MS * 1000;
                        display_needs_update = true;
                        
                        update_led_state();
                        ESP_LOGI("CORE", "ROLE -> HOLDER (PASS_GRANT won seq=%u)", seq);
                    }
                }
                break;
            }
            
            case MSG_STATE_SYNC: {
                // STATE_SYNC: [0xA3 | seq_lo | seq_hi | new_holder_mac(6) | cooldown_ms(2)]
                if (len >= 12 && g_role == ROLE_PENDING) {
                    uint16_t seq = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
                    uint8_t new_holder_mac[6];
                    memcpy(new_holder_mac, &data[3], 6);
                    
                    if (g_role == ROLE_PENDING && seq == g_pending_seq) {
                        if (memcmp(new_holder_mac, g_self_mac, 6) == 0) {
                            // winner path (likely already handled): also stop timers
                            esp_timer_stop(g_pass_req_timer);
                            esp_timer_stop(g_retry_timer);
                            int64_t now = esp_timer_get_time();
                            g_role = ROLE_HOLDER;
                            ir_rx_set_enabled(false);
                            g_holder_startup_until_us = now + (int64_t)HOLDER_STARTUP_DELAY_MS * 1000;
                            display_needs_update = true;
                            
                            update_led_state();
                            ESP_LOGI("CORE", "ROLE -> HOLDER (STATE_SYNC won seq=%u)", seq);
                        } else {
                            // lost: cleanly exit pending
                            esp_timer_stop(g_pass_req_timer);
                            esp_timer_stop(g_retry_timer);
                            g_role = ROLE_RUNNER;
                            ir_rx_set_enabled(true);
                            update_led_state();
                            display_needs_update = true;
                            ESP_LOGI("CORE","ROLE -> RUNNER (lost seq=%u)", seq);
                        }
                    }
                    
                    ESP_LOGI("SYNC", "new_holder=%02X:%02X:%02X:%02X:%02X:%02X seq=%u",
                             new_holder_mac[0],new_holder_mac[1],new_holder_mac[2],
                             new_holder_mac[3],new_holder_mac[4],new_holder_mac[5], seq);
                }
                break;
            }
            
            default: {
                // Legacy ACK - backward compatibility
                if (data[0] == 0xA1 && g_ack_queue) {
                    // Role transition: HOLDER → RUNNER on legacy ACK
                    int64_t now = esp_timer_get_time();
                    if (g_role == ROLE_HOLDER && now >= g_no_tagback_until_us) {
                        memcpy(g_last_peer_mac, info->src_addr, 6);
                        g_no_tagback_until_us = now + (int64_t)PASS_COOLDOWN_MS * 1000;
                        ESP_LOGI("CORE", "cooldown with %02X:%02X:%02X:%02X:%02X:%02X for %d ms",
                                 info->src_addr[0], info->src_addr[1], info->src_addr[2],
                                 info->src_addr[3], info->src_addr[4], info->src_addr[5], PASS_COOLDOWN_MS);
                        g_role = ROLE_RUNNER;
                        ir_rx_set_enabled(true);
                        update_led_state();
                        display_needs_update = true;
                        ESP_LOGI("CORE", "ROLE -> RUNNER (legacy ACK from %02X:%02X:%02X:%02X:%02X:%02X)",
                                 info->src_addr[0],info->src_addr[1],info->src_addr[2],
                                 info->src_addr[3],info->src_addr[4],info->src_addr[5]);
                    }
                    // Non-blocking post of 6-byte MAC
                    xQueueSend(g_ack_queue, info->src_addr, 0);
                }
                break;
            }
        }
    });
    if (err != ESP_OK) {
        ESP_LOGE("ESPNOW", "esp_now_register_recv_cb failed: %s", esp_err_to_name(err));
        return err;
    }

    g_espnow_ready = true;
    ESP_LOGI("ESPNOW", "init OK, channel=%d", ESPNOW_CHANNEL);
    return ESP_OK;
}

// Unicast a tiny ACK to a peer MAC (adds peer on-demand)
static esp_err_t espnow_send_ack_to(const uint8_t dst_mac[6], uint8_t seq /*0 if none*/) {
    if (!g_espnow_ready) {
        esp_err_t err = espnow_init_once();
        if (err != ESP_OK) {
            ESP_LOGE("ESPNOW", "init failed: %s", esp_err_to_name(err));
            return err;
        }
    }
    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, dst_mac, 6);
    p.ifidx = WIFI_IF_STA;
    p.channel = ESPNOW_CHANNEL;
    p.encrypt = false;
    esp_err_t err = esp_now_add_peer(&p);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGW("ESPNOW", "add_peer %s", esp_err_to_name(err));
        return err;
    }
    uint8_t ack[2] = {0xA1, seq};   // echo seq if provided
    return esp_now_send(dst_mac, ack, 2);
}

// ESP-NOW helper functions
static esp_err_t ensure_peer(const uint8_t* mac) {
    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, mac, 6);
    p.ifidx = WIFI_IF_STA;
    p.channel = ESPNOW_CHANNEL;
    p.encrypt = false;
    esp_err_t err = esp_now_add_peer(&p);
    if (err == ESP_ERR_ESPNOW_EXIST) {
        return ESP_OK;  // Already exists
    }
    return err;
}

static esp_err_t espnow_send_pass_req(const uint8_t* holder_mac, uint16_t seq) {
    if (!g_espnow_ready) {
        esp_err_t err = espnow_init_once();
        if (err != ESP_OK) return err;
    }
    
    esp_err_t err = ensure_peer(holder_mac);
    if (err != ESP_OK) return err;
    
    uint8_t payload[10] = {
        MSG_PASS_REQ,
        (uint8_t)(seq & 0xFF),      // seq_lo
        (uint8_t)((seq >> 8) & 0xFF), // seq_hi
        holder_mac[0], holder_mac[1], holder_mac[2],
        holder_mac[3], holder_mac[4], holder_mac[5]
    };
    
    return esp_now_send(holder_mac, payload, sizeof(payload));
}

static esp_err_t espnow_send_pass_grant(const uint8_t* winner_mac, uint16_t seq) {
    if (!g_espnow_ready) return ESP_ERR_INVALID_STATE;
    
    esp_err_t err = ensure_peer(winner_mac);
    if (err != ESP_OK) return err;
    
    uint8_t payload[10] = {
        MSG_PASS_GRANT,
        (uint8_t)(seq & 0xFF),      // seq_lo
        (uint8_t)((seq >> 8) & 0xFF), // seq_hi
        winner_mac[0], winner_mac[1], winner_mac[2],
        winner_mac[3], winner_mac[4], winner_mac[5]
    };
    
    return esp_now_send(winner_mac, payload, sizeof(payload));
}

static esp_err_t espnow_send_state_sync(uint16_t seq, const uint8_t* new_holder_mac) {
    if (!g_espnow_ready) return ESP_ERR_INVALID_STATE;
    
    uint8_t payload[12] = {
        MSG_STATE_SYNC,
        (uint8_t)(seq & 0xFF),      // seq_lo
        (uint8_t)((seq >> 8) & 0xFF), // seq_hi
        new_holder_mac[0], new_holder_mac[1], new_holder_mac[2],
        new_holder_mac[3], new_holder_mac[4], new_holder_mac[5],
        (uint8_t)(PASS_COOLDOWN_MS & 0xFF),      // cooldown_lo
        (uint8_t)((PASS_COOLDOWN_MS >> 8) & 0xFF) // cooldown_hi
    };
    
    // Broadcast to all peers
    uint8_t broadcast_addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    return esp_now_send(broadcast_addr, payload, sizeof(payload));
}

// CRC-8 Dallas/Maxim (poly 0x31), init 0x00, table-less
static inline uint8_t crc8_maxim(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

// ---- TX helpers ----
// mark = logic 0 (carrier ON), space = logic 1 (carrier OFF)
static inline void set_mark(rmt_symbol_word_t* s, uint32_t us) {
    s->level0 = 1; s->duration0 = us; s->level1 = 0; s->duration1 = 1; // padding half
}
static inline void set_space(rmt_symbol_word_t* s, uint32_t us) {
    s->level0 = 0; s->duration0 = us; s->level1 = 0; s->duration1 = 1;
}

// Append start+8 data+stop (10 symbols) for 'b' into 'dst' at index '*idx'
static inline void ir_append_byte_syms(uint8_t b, rmt_symbol_word_t *dst, size_t *idx) {
    // start (0) = mark
    set_mark(&dst[(*idx)++], IR_BIT_US);
    // data LSB-first
    for (int i = 0; i < 8; ++i)
        ((b >> i) & 1) ? set_space(&dst[(*idx)++], IR_BIT_US)
                       : set_mark (&dst[(*idx)++], IR_BIT_US);
    // stop (1) = space
    set_space(&dst[(*idx)++], IR_BIT_US);
}

static void ir_send_ZT_oneshot(void) {
    rmt_symbol_word_t syms[32];
    size_t n = 0;
    ir_append_byte_syms(0x5A, syms, &n); // 'Z'
    ir_append_byte_syms(0x54, syms, &n); // 'T'

    rmt_transmit_config_t cfg = { .loop_count = 0, .flags = {0} };
    ESP_ERROR_CHECK(rmt_transmit(rmt_tx_chan, tx_copy_encoder, syms, n * sizeof(syms[0]), &cfg));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(rmt_tx_chan, pdMS_TO_TICKS(1000)));

    IR_LOGI(TAG, "Sent ZT preamble: 0x5A 0x54");
}

static void ir_send_ZT_MAC_oneshot(void) {
    // Build contiguous UART symbols for: 0x5A, 0x54, then our 6-byte MAC
    rmt_symbol_word_t syms[128];
    size_t n = 0;

    // Preamble
    ir_append_byte_syms(IR_PREAMBLE0, syms, &n);
    ir_append_byte_syms(IR_PREAMBLE1, syms, &n);

    // Source MAC
    uint8_t mac[IR_MAC_LEN];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    for (int i = 0; i < IR_MAC_LEN; ++i) {
        ir_append_byte_syms(mac[i], syms, &n);
    }

    // Transmit in one shot
    rmt_transmit_config_t cfg = { .loop_count = 0, .flags = {0} };
    ESP_ERROR_CHECK(rmt_transmit(rmt_tx_chan, tx_copy_encoder, syms, n * sizeof(syms[0]), &cfg));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(rmt_tx_chan, pdMS_TO_TICKS(1000)));

    IR_LOGI(TAG, "Sent ZT+MAC (%u symbols)", (unsigned)n);
}

static void ir_send_ZT_LEN_MAC_CRC_oneshot(void) {
    // Build the payload: LEN + [SEQ] + MAC
    uint8_t mac[IR_MAC_LEN];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    uint8_t len = IR_MAC_LEN;

#if IR_ENABLE_SEQ_FUTURE
    // Include 16-bit sequence number
    len = IR_MAC_LEN + 2;  // SEQ_LO + SEQ_HI + MAC
    uint16_t seq = g_ir_seq;
    g_last_beacon_seq_tx = seq;
    uint8_t seq_lo = (uint8_t)(seq & 0xFF);
    uint8_t seq_hi = (uint8_t)(seq >> 8);
#endif

    // Compute CRC over [LEN || [SEQ_LO|SEQ_HI] || MAC]
    uint8_t crc = 0;
    {
#if IR_ENABLE_SEQ_FUTURE
        uint8_t tmp[1 + IR_MAC_LEN + 2];  // LEN + SEQ_LO + SEQ_HI + MAC
        tmp[0] = len;
        tmp[1] = seq_lo;
        tmp[2] = seq_hi;
        memcpy(tmp + 3, mac, IR_MAC_LEN);
        crc = crc8_maxim(tmp, sizeof(tmp));
#else
        uint8_t tmp[1 + IR_MAC_LEN];
        tmp[0] = len;
        memcpy(tmp + 1, mac, IR_MAC_LEN);
        crc = crc8_maxim(tmp, sizeof(tmp));
#endif
    }

    // Build contiguous UART symbols for all bytes
    rmt_symbol_word_t syms[160];
    size_t n = 0;
    ir_append_byte_syms(IR_PREAMBLE0, syms, &n);
    ir_append_byte_syms(IR_PREAMBLE1, syms, &n);
    ir_append_byte_syms(len,          syms, &n);
#if IR_ENABLE_SEQ_FUTURE
    ir_append_byte_syms(seq_lo,       syms, &n);
    ir_append_byte_syms(seq_hi,       syms, &n);
#endif
    for (int i = 0; i < IR_MAC_LEN; ++i) ir_append_byte_syms(mac[i], syms, &n);
    ir_append_byte_syms(crc,          syms, &n);

    // Transmit one shot
    rmt_transmit_config_t cfg = { .loop_count = 0, .flags = {0} };
    ESP_ERROR_CHECK(rmt_transmit(rmt_tx_chan, tx_copy_encoder, syms, n * sizeof(syms[0]), &cfg));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(rmt_tx_chan, pdMS_TO_TICKS(1000)));

#if IR_ENABLE_SEQ_FUTURE
    // Reset grant guard before incrementing sequence
    g_granted_seq_valid = false;
    // Increment sequence number after successful transmit
    g_ir_seq++;
    IR_LOGI(TAG, "Sent ZT+LEN+SEQ+MAC+CRC (len=%u, seq=%u)", (unsigned)len, (unsigned)g_ir_seq-1);
#else
    IR_LOGI(TAG, "Sent ZT+LEN+MAC+CRC (len=%u)", (unsigned)len);
#endif
}

// Build a UART byte (start 0, 8 data LSB, stop 1) into symbols
static size_t build_uart_byte(uint8_t b, rmt_symbol_word_t* out) {
    size_t i = 0;
    
    // Remove pre-idle gap - start immediately with start bit
    set_mark(&out[i++], IR_BIT_US);                 // start (0)
    for (int bit = 0; bit < 8; ++bit) {
        if ((b >> bit) & 1) set_space(&out[i++], IR_BIT_US); else set_mark(&out[i++], IR_BIT_US);
    }
    set_space(&out[i++], IR_BIT_US);                // stop (1)
    return i; // number of symbols
}

static void tx_prebuild_byte(uint8_t b) {
    tx_uart_len = build_uart_byte(b, tx_uart_buf);
    IR_LOGI(TAG, "TX prebuilt 0x%02X into %u symbols", b, (unsigned)tx_uart_len);
}

// Button interrupt handler removed - using automatic beacon instead

// ----------------------
// Setup RMT TX
// ----------------------
static void setup_rmt_tx() {
    rmt_tx_channel_config_t tx_chan_cfg = {
        .gpio_num = IR_TX_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000, // 1 tick = 1 µs
        .mem_block_symbols = 128,
        .trans_queue_depth = 4,
        .intr_priority = 0,
        .flags = {0}
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_cfg, &rmt_tx_chan));

    // Apply 38 kHz carrier for marks
    rmt_carrier_config_t carrier_cfg = {
        .frequency_hz = CARRIER_FREQ,
        .duty_cycle = 0.33,                // was 0.5 → use ~33%
        .flags = { .polarity_active_low = 0 }
    };
    ESP_ERROR_CHECK(rmt_apply_carrier(rmt_tx_chan, &carrier_cfg));

    // Copy encoder for raw symbol sending
    rmt_copy_encoder_config_t copy_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_cfg, &tx_copy_encoder));

    ESP_ERROR_CHECK(rmt_enable(rmt_tx_chan));
    IR_LOGI(TAG, "RMT TX initialized");
    
    // Prebuild the TX byte
    tx_prebuild_byte(TX_BYTE);
}

#if IR_RX_BACKEND_UART
// ----------------------
// Setup UART RX (TSOP -> UART2 @ 2400-8N1)
// ----------------------
static void setup_uart_rx() {
    uart_config_t cfg = {
        .baud_rate  = IR_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(IR_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_driver_install(IR_UART_PORT, IR_UART_RX_BUF_SZ, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_set_pin(IR_UART_PORT,
                                 UART_PIN_NO_CHANGE,   // TX unused
                                 IR_UART_RX_GPIO,      // RX = GPIO36 (TSOP)
                                 UART_PIN_NO_CHANGE,   // RTS
                                 UART_PIN_NO_CHANGE)); // CTS
    ESP_LOGI("IR-UART", "UART RX initialized on GPIO%d at %d-8N1", (int)IR_UART_RX_GPIO, IR_BAUD);
}

// Tiny parser for Stage-2a: detect ZT and update UI/logs
static void uart_rx_task(void *pv) {
    ESP_LOGI("IR-UART", "UART RX task started");
    enum { WAIT_Z, WAIT_T, READ_LEN, READ_PAYLOAD, READ_CRC } st = WAIT_Z;
    uint8_t buf[64];
    uint8_t payload[IR_MAX_PAYLOAD];
    uint8_t len = 0;
    size_t  pi  = 0;

    while (true) {
        // If RX is disabled (e.g., we're HOLDER in beacon mode), don't arm UART
        if (!g_rx_enabled) {
            vTaskDelay(pdMS_TO_TICKS(20));  // light sleep; no busy wait
            continue;
        }
        
        int n = uart_read_bytes(IR_UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(20));
        if (n <= 0) continue;

        for (int i = 0; i < n; ++i) {
            uint8_t b = buf[i];
#if IR_DEBUG
            ESP_LOGI("IR-UART", "0x%02X", b);
#endif
            switch (st) {
                case WAIT_Z:
                    if (b == IR_PREAMBLE0) {
                        g_rx_work_hex[0] = 0;
                        hex_append(g_rx_work_hex, sizeof(g_rx_work_hex), &b, 1);
                        st = WAIT_T;
                    }
                    break;

                case WAIT_T:
                    hex_append(g_rx_work_hex, sizeof(g_rx_work_hex), &b, 1);
                    if (b == IR_PREAMBLE1) {
                        st = READ_LEN;
                    } else {
                        ESP_LOGW("RX", "Unexpected second byte after Z: 0x%02X", b);
                        g_rx_work_hex[0] = 0; // Clear the in-progress line
                        st = WAIT_Z;
                    }
                    break;

                case READ_LEN:
                    len = b;
                    hex_append(g_rx_work_hex, sizeof(g_rx_work_hex), &b, 1);
                    if (len == 0 || len > IR_MAX_PAYLOAD) {
                        ESP_LOGW("RX", "LEN invalid: %u", (unsigned)len);
                        g_rx_work_hex[0] = 0; // Clear the in-progress line
                        st = WAIT_Z;
                        break;
                    }
                    pi = 0;
                    st = READ_PAYLOAD;
                    break;

                case READ_PAYLOAD:
                    payload[pi++] = b;
                    hex_append(g_rx_work_hex, sizeof(g_rx_work_hex), &b, 1);
                    if (pi >= len) {
                        st = READ_CRC;
                    }
                    break;

                case READ_CRC: {
                    // Verify CRC over [LEN || PAYLOAD]
                    uint8_t calc;
                    {
                        uint8_t tmp[1 + IR_MAX_PAYLOAD];
                        tmp[0] = len;
                        memcpy(tmp + 1, payload, len);
                        calc = crc8_maxim(tmp, 1 + len);
                    }
                    hex_append(g_rx_work_hex, sizeof(g_rx_work_hex), &b, 1); // append CRC to RECEIVING
                    bool ok = (calc == b);

                    if (ok) {
                        // Interpret payload form
                        bool has_seq = (IR_ENABLE_SEQ_FUTURE && len == (IR_MAC_LEN + 2));
                        uint16_t seq = has_seq ? ((uint16_t)payload[0] | ((uint16_t)payload[1] << 8)) : 0;
                        const uint8_t *mac_ptr = has_seq ? (payload + 2) : payload; // sender MAC
#if IR_SELF_FILTER
                        bool is_self = (memcmp(mac_ptr, g_self_mac, IR_MAC_LEN) == 0);
                        if (is_self) {
                            // Ignore our own frame; don't promote
                            ESP_LOGI("RX", "CRC OK but ignored self frame (our MAC)");
                            // Clear the in-progress line so the UI doesn't show stale bytes
                            g_rx_work_hex[0] = 0;
                            st = WAIT_Z;
                            break;
                        }
#endif
                        // Dedupe (drops fast repeats or identical SEQ)
                        if (dedupe_should_drop_and_update(mac_ptr, has_seq, seq)) {
                            ESP_LOGI("RX", "Duplicate frame suppressed (MAC%s, %s)",
                                     "", has_seq ? "SEQ" : "time-window");
                            g_rx_work_hex[0] = 0;
                            st = WAIT_Z;
                            break;
                        }

                        // Not self and not duplicate — send ESP-NOW ACK back to sender MAC
#if IR_USE_ESPNOW_ACK
                        // Optional safety: cancel PENDING if a different holder's IR is seen
                        if (g_role == ROLE_PENDING && has_seq && memcmp(mac_ptr, g_pending_pass_req.holder_mac, 6) != 0) {
                            esp_timer_stop(g_pass_req_timer);
                            esp_timer_stop(g_retry_timer);
                            g_role = ROLE_RUNNER;
                            ir_rx_set_enabled(true);
                            ESP_LOGI("CORE","cancel PENDING: new holder IR seen %02X:%02X", mac_ptr[0], mac_ptr[1]);
                        }
                        
                        // Re-slot PENDING if same holder emits newer sequence
                        if (g_role == ROLE_PENDING && has_seq && memcmp(mac_ptr, g_pending_pass_req.holder_mac, 6) == 0 && seq != g_pending_seq) {
                            esp_timer_stop(g_pass_req_timer);
                            esp_timer_stop(g_retry_timer);
                            g_pending_seq = seq;
                            // recompute slot and start once
                            slot_pick_t sp = pick_slot(seq, mac_ptr, g_self_mac);
                            uint32_t delay_us = (uint32_t)sp.slot * SLOT_US + sp.jitter_us;
                            // (re)arm the pass-req timer to send for this seq
                            esp_timer_stop(g_pass_req_timer);
                            esp_timer_start_once(g_pass_req_timer, delay_us);
                            ESP_LOGI("CLAIM","re-slot same-holder newer seq=%u (slot=%u,jit=%u)", seq, sp.slot, sp.jitter_us);
                            // keep ROLE_PENDING; return (don't schedule another immediate path)
                            goto done_crc_ok;
                        }
                        
                        // Role transition: RUNNER → HOLDER on valid IR
                        if (g_role == ROLE_RUNNER) {
                            int64_t now = esp_timer_get_time();
                            if (now >= g_no_tagback_until_us) {
                                // Check if we have sequence numbers (new protocol)
                                if (has_seq) {
                                    // Per-holder dedupe check
                                    if (dedupe_seen_and_update(mac_ptr, seq)) { 
                                        ESP_LOGD("IR","dup (h=%02X:%02X seq=%u)",mac_ptr[0],mac_ptr[1],(unsigned)seq); 
                                        break; 
                                    }
                                    
                                    // New protocol: use slotting
                                    slot_pick_t sp = pick_slot(seq, mac_ptr, g_self_mac);
                                    uint32_t delay_us = sp.slot * SLOT_US + sp.jitter_us;
                                    
                                    // Store PASS_REQ data
                                    g_pending_pass_req.seq = seq;
                                    memcpy(g_pending_pass_req.holder_mac, mac_ptr, 6);
                                    
                                    // Schedule PASS_REQ (defensive timer stop)
                                    esp_timer_stop(g_pass_req_timer);
                                    esp_timer_start_once(g_pass_req_timer, delay_us);
                                    
                                    ESP_LOGI("CLAIM", "slot=%u jitter=%u send_in=%lu seq=%u", 
                                             sp.slot, sp.jitter_us, (unsigned long)delay_us, (unsigned)seq);
                                    
                                    if (PASS_COMMIT_MODE == PASS_COMMIT_ON_GRANT) {
                                        g_role = ROLE_PENDING;
                                        g_pending_seq = seq;
                                        update_led_state();
                                        // Schedule retry timer (60ms) - defensive stop
                                        esp_timer_stop(g_retry_timer);
                                        esp_timer_start_once(g_retry_timer, 60000);
                                    }
                                } else {
                                    // Legacy protocol: use IR backoff if enabled
#if ENABLE_IR_BACKOFF
                                    // Deterministic slot + micro-jitter to avoid collisions
                                    slot_pick_t sp = pick_slot((uint16_t)seq, mac_ptr, g_self_mac);
                                    uint32_t delay_us = (uint32_t)sp.slot * (uint32_t)SLOT_US + (uint32_t)sp.jitter_us;
                                    memcpy(g_ack_slot_dst, mac_ptr, 6);
                                    g_ack_slot_seq = seq;
                                    esp_timer_stop(g_ack_slot_timer);           // hygiene: stop if armed
                                    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_start_once(g_ack_slot_timer, delay_us));
                                    ESP_LOGD("CLAIM","slot=%u jitter=%u send_in=%lu us", sp.slot, sp.jitter_us, (unsigned long)delay_us);
#else
                                    (void)espnow_send_ack_to(mac_ptr, seq);     // legacy immediate
#endif
                                    memcpy(g_last_peer_mac, mac_ptr, 6);
                                    g_no_tagback_until_us = now + (int64_t)PASS_COOLDOWN_MS * 1000;
                                    ESP_LOGI("CORE", "cooldown with %02X:%02X:%02X:%02X:%02X:%02X for %d ms",
                                             mac_ptr[0], mac_ptr[1], mac_ptr[2], mac_ptr[3], mac_ptr[4], mac_ptr[5], PASS_COOLDOWN_MS);
                                    g_holder_startup_until_us = now + (int64_t)HOLDER_STARTUP_DELAY_MS * 1000;
                                    g_role = ROLE_HOLDER;
                                    ir_rx_set_enabled(false);
                                    update_led_state();
                                    display_needs_update = true;
                                    ESP_LOGI("CORE", "ROLE -> HOLDER (legacy, from %02X:%02X:%02X:%02X:%02X:%02X) - startup delay %dms",
                                             mac_ptr[0],mac_ptr[1],mac_ptr[2],mac_ptr[3],mac_ptr[4],mac_ptr[5], HOLDER_STARTUP_DELAY_MS);
                                }
                            }
                        }
#endif
done_crc_ok:
                        // Promote to LAST RECEIVED
                        strlcpy(g_rx_last_hex, g_rx_work_hex, sizeof(g_rx_last_hex));
                        g_rx_work_hex[0] = 0;
                        ESP_LOGI("RX", "CRC OK (len=%u)", (unsigned)len);
                        if (len == IR_MAC_LEN) {
                            ESP_LOGI("RX", "MAC = %02X:%02X:%02X:%02X:%02X:%02X",
                                     mac_ptr[0], mac_ptr[1], mac_ptr[2],
                                     mac_ptr[3], mac_ptr[4], mac_ptr[5]);
                        } else if (has_seq) {
                            ESP_LOGI("RX", "SEQ=%u MAC=%02X:%02X:%02X:%02X:%02X:%02X",
                                     (unsigned)seq, mac_ptr[0], mac_ptr[1], mac_ptr[2],
                                     mac_ptr[3], mac_ptr[4], mac_ptr[5]);
                        }
                    } else {
                        // CRC fail — log it but don't update display
                        strlcpy(g_rx_last_hex, g_rx_work_hex, sizeof(g_rx_last_hex));
                        g_rx_work_hex[0] = 0;
                        ESP_LOGW("RX", "CRC FAIL: calc=0x%02X recv=0x%02X", calc, b);
                    }
                    st = WAIT_Z;
                    break;
                }
            }
        }
    }
}
#endif // IR_RX_BACKEND_UART

// ----------------------
// Setup RMT RX
// ----------------------
static void setup_rmt_rx() {
    // Create RX channel
    rmt_rx_channel_config_t rx_chan_cfg = {
        .gpio_num = IR_RX_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000, // 1 tick = 1 µs
        .mem_block_symbols = 256,   // room for long bursts and future packets
        .intr_priority = 0,
        .flags = {0}
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_cfg, &rmt_rx_chan));

    // Create semaphore and queue
    rx_done_sem = xSemaphoreCreateBinary();
    rx_byte_queue = xQueueCreate(16, sizeof(uint8_t));

    // Enable RX channel
    ESP_ERROR_CHECK(rmt_enable(rmt_rx_chan));
    
    IR_LOGI(TAG, "RMT RX initialized");
}

// ----------------------
// RMT RX done callback (ISR)
// ----------------------
static bool IRAM_ATTR rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data) {
    rx_symbol_count = edata->num_symbols;  // true number of symbols captured
    BaseType_t hp = pdFALSE;
    xSemaphoreGiveFromISR(rx_done_sem, &hp);
    return hp == pdTRUE;
}

// ----------------------
// Debug symbol dump
// ----------------------
static void dump_symbols(const rmt_symbol_word_t *buf, size_t n) {
    if (!IR_DEBUG) return;
    
    size_t m = (n < 20) ? n : 20;
    IR_LOGI("RX", "Dump %u symbols:", (unsigned)m);
    for (size_t i = 0; i < m; ++i) {
        IR_LOGI("RX", "[%02u] l0=%d d0=%u | l1=%d d1=%u",
                (unsigned)i, buf[i].level0, buf[i].duration0,
                buf[i].level1, buf[i].duration1);
    }
}

// ---- Robust start + mid-bit sampling with validity ----
static int find_start_t0_strict(const rmt_symbol_word_t *syms, size_t cnt,
                                uint32_t bit_us, uint32_t *t0_us, int *mark_level_out) {
    const uint32_t MIN1 = (bit_us * 7) / 10;   // 0.7*BIT
    const uint32_t MAX1 = (bit_us * 13) / 10;  // 1.3*BIT
    const uint32_t MIN2 = (bit_us * 17) / 10;  // 1.7*BIT
    const uint32_t MAX2 = (bit_us * 23) / 10;  // 2.3*BIT
    // was: const uint32_t IDLE_MIN = (bit_us * 5) / 2;  // 2.5*BIT (too strict for inside the stream)
    // now let us detect the next byte's start after just one stop bit of HIGH
    const uint32_t IDLE_MIN = (bit_us * 8) / 10;        // ~0.8*BIT

    uint32_t t=0, high_accum=0; int prev=1;

    // Positive "merged start+bit0" fast-path ONLY if we can confirm the shape:
    // LOW ~2*BIT at buffer start followed by HIGH ~1*BIT.
    if (cnt) {
        int l0 = syms[0].level0; uint32_t d0 = syms[0].duration0;
        int l1 = syms[0].level1; uint32_t d1 = syms[0].duration1;
        if (l0==0 && d0>=MIN2 && d0<=MAX2 && l1==1 && d1>=MIN1 && d1<=MAX1) {
            *t0_us = 0;
            *mark_level_out = 0; // active-low demod: mark=LOW
            return 1;
        }
    }

    // Otherwise, require an actual falling edge after enough idle HIGH
    for (size_t i=0;i<cnt;++i) {
        int l0=syms[i].level0, l1=syms[i].level1;
        uint32_t d0=syms[i].duration0, d1=syms[i].duration1;

        if (d0) {
            if (prev==1 && l0==0 && high_accum>=IDLE_MIN &&
                ((d0>=MIN1 && d0<=MAX1) || (d0>=MIN2 && d0<=MAX2))) {
                *t0_us = t; *mark_level_out = 0; return 1;
            }
            high_accum = (l0==1) ? high_accum+d0 : 0;
            t += d0; prev = l0;
        }
        if (d1) {
            if (prev==1 && l1==0 && high_accum>=IDLE_MIN &&
                ((d1>=MIN1 && d1<=MAX1) || (d1>=MIN2 && d1<=MAX2))) {
                *t0_us = t; *mark_level_out = 0; return 1;
            }
            high_accum = (l1==1) ? high_accum+d1 : 0;
            t += d1; prev = l1;
        }
    }
    return 0; // no start edge found
}

static uint16_t sample9_from_t0(const rmt_symbol_word_t *syms, size_t cnt,
                                uint32_t t0_us, uint32_t bit_us, uint32_t phase_us,
                                int mark_level){
    uint32_t half=bit_us/2, sample_t[9];
    for(int n=0;n<9;++n) sample_t[n]=t0_us+bit_us+half+phase_us+n*bit_us; // t0 + 1.5*BIT + n*BIT
    uint32_t t=0; int next=0; uint16_t out=0;
    for(size_t i=0;i<cnt && next<9;++i){
        int l0=syms[i].level0,l1=syms[i].level1;
        uint32_t d0=syms[i].duration0,d1=syms[i].duration1;
        if (d0){ uint32_t tend=t+d0; while(next<9 && sample_t[next]<=tend){ if(l0!=mark_level) out|=(1u<<next); next++; } t=tend; }
        if (d1){ uint32_t tend=t+d1; while(next<9 && sample_t[next]<=tend){ if(l1!=mark_level) out|=(1u<<next); next++; } t=tend; }
    }
    return out; // [b0..b7, stop]
}

static void hex_append(char *dst, size_t cap, const uint8_t *data, size_t len) {
    size_t used = strlen(dst);
    for (size_t i=0; i<len && used + 3 < cap; ++i) {
        int n = snprintf(dst + used, cap - used, i ? " %02X" : "%02X", data[i]);
        if (n < 0) break; 
        used += n;
    }
}

static size_t advance_to_time(const rmt_symbol_word_t *syms, size_t cnt, uint32_t target_us) {
    uint32_t acc=0; size_t i=0;
    while (i < cnt && acc < target_us) { acc += syms[i].duration0 + syms[i].duration1; ++i; }
    return i;
}

void process_capture(const rmt_symbol_word_t *buf, size_t count) {
    size_t off = 0;
    // Clear current "RECEIVING:" string here
    g_rx_work_hex[0] = 0;
    uint8_t rx_bytes[16];
    size_t rx_count = 0;

    while (off + 1 < count && rx_count < sizeof(rx_bytes)) {
        uint32_t t0=0; int mark=0;
        if (!find_start_t0_strict(&buf[off], count - off, IR_BIT_US, &t0, &mark)) break;

        uint16_t s = sample9_from_t0(&buf[off], count - off, t0, IR_BIT_US, 0, mark);
        if (((s >> 8) & 1) == 0) {
            s = sample9_from_t0(&buf[off], count - off, t0, IR_BIT_US, IR_BIT_US/2, mark);
            if (((s >> 8) & 1) == 0) break; // no valid stop -> end
        }

        uint8_t b = (uint8_t)(s & 0xFF);
        rx_bytes[rx_count++] = b;
        
        // append 'b' to RECEIVING string (e.g., "5A 54 ...")
        hex_append(g_rx_work_hex, sizeof(g_rx_work_hex), &b, 1);

        // move past this framed byte (start+8+stop ≈ 10*BIT from t0)
        size_t used = advance_to_time(&buf[off], count - off, (uint32_t)(t0 + 10 * IR_BIT_US));
        if (!used) break;
        off += used;
    }

    // Check for ZT preamble recognition
    if (rx_count >= 2 && rx_bytes[0] == 0x5A && rx_bytes[1] == 0x54) {
        IR_LOGI(TAG, "ZT preamble seen: 0x%02X 0x%02X", rx_bytes[0], rx_bytes[1]);
    }

    // Promote RECEIVING -> LAST RECEIVED here (no display update needed)
    if (rx_count > 0) {
        strlcpy(g_rx_last_hex, g_rx_work_hex, sizeof(g_rx_last_hex));
        g_rx_work_hex[0] = 0; // clear working buffer
        
        // Queue the first byte for compatibility
        if (xQueueSend(rx_byte_queue, &rx_bytes[0], 0) == pdTRUE) {
            last_received_byte = rx_bytes[0];
            has_received = true;
        }
    }
}

// Return 1 on success and fill *out_byte; return 0 if no valid frame found.
static int decode_symbols_to_byte_valid(const rmt_symbol_word_t *syms, size_t cnt, uint8_t *out_byte) {
    const uint32_t HALF = IR_BIT_US / 2;
    uint32_t t0 = 0;
    int mark_level = 0;

    // 1) Try to find start edge, but allow fallback to t0=0 if needed (like original)
    if (!find_start_t0_strict(syms, cnt, IR_BIT_US, &t0, &mark_level)) {
        t0 = 0; mark_level = 0; // fallback like original
    }

    // 2) Sample mid-bit from t0; require stop==1, else try half-bit phase
    uint16_t a = sample9_from_t0(syms, cnt, t0, IR_BIT_US, 0,    mark_level);
    if (((a >> 8) & 1) == 1) { *out_byte = (uint8_t)(a & 0xFF); return 1; }

    uint16_t b = sample9_from_t0(syms, cnt, t0, IR_BIT_US, HALF, mark_level);
    if (((b >> 8) & 1) == 1) { *out_byte = (uint8_t)(b & 0xFF); return 1; }

    // 3) If stop bit still invalid, return the first attempt anyway (like original)
    *out_byte = (uint8_t)(a & 0xFF);
    return 1;
}

// Keep the C API: return 0x00 only when there's truly a decoded zero
// and otherwise don't update the screen/queue unless decode returns 1.
static uint8_t decode_symbols_to_byte(const rmt_symbol_word_t *syms, size_t cnt) {
    uint8_t b = 0;
    if (!decode_symbols_to_byte_valid(syms, cnt, &b)) {
        return 0; // caller must check validity to avoid "all zeros" UI
    }
    return b;
}

// ----------------------
// RX Task - Fully blocking, always armed
// ----------------------
static void rx_task(void *pvParameters) {
    rmt_symbol_word_t rx_buffer[256]; // Increased buffer size for multi-byte
    
    IR_LOGI(TAG, "RX task started");
    
    // Arm immediately
    rmt_receive_config_t rx_cfg = {
        .signal_range_min_ns = IR_RX_MIN_NS,
        .signal_range_max_ns = IR_RX_MAX_NS,
        .flags = {0}
    };

    while (true) {
        if (!rx_inflight) {
            ESP_ERROR_CHECK(rmt_receive(rmt_rx_chan, rx_buffer, sizeof(rx_buffer), &rx_cfg));
            rx_inflight = true;
        }

        // Block until the ISR posts completion (no timeouts/spin)
        xSemaphoreTake(rx_done_sem, portMAX_DELAY);

        // Snapshot count from ISR and mark not inflight
        size_t count = rx_symbol_count;
        rx_inflight = false;

        if (IR_DEBUG) {
            size_t m = count < 20 ? count : 20;
            IR_LOGI("RX", "Dump %u symbols:", (unsigned)m);
            for (size_t i = 0; i < m; ++i) {
                IR_LOGI("RX", "[%02u] l0=%d d0=%u | l1=%d d1=%u",
                        (unsigned)i, rx_buffer[i].level0, rx_buffer[i].duration0,
                        rx_buffer[i].level1, rx_buffer[i].duration1);
            }
        }

        // Process the entire capture for multiple bytes
        process_capture(rx_buffer, count);

        // Loop re-arms at top (no polling / no sleep)
    }
}

// ----------------------
// Send a byte using prebuilt symbols
// ----------------------
static void send_byte(uint8_t byte) {
    if (tx_uart_len) {
        rmt_transmit_config_t tx_cfg = {
            .loop_count = 0,
            .flags = {0}
        };
        
        ESP_ERROR_CHECK(rmt_transmit(rmt_tx_chan, tx_copy_encoder, tx_uart_buf, tx_uart_len * sizeof(rmt_symbol_word_t), &tx_cfg));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(rmt_tx_chan, portMAX_DELAY)); // optional: ensure back-to-back presses queue nicely
        
        last_sent_byte = byte;
        display_needs_update = true;
        IR_LOGI(TAG, "Sent byte: 0x%02X", byte);
    }
}

// ----------------------
// Public wrapper functions
// ----------------------
extern "C" void ir_send_byte(uint8_t byte) {
    // Prebuild symbols for this byte
    tx_prebuild_byte(byte);
    send_byte(byte);
}

extern "C" void ir_send_frame(const uint8_t* data, size_t len) {
    if (!data || len == 0) return;
    
    // For now, send each byte individually
    // TODO: Optimize to build symbols for entire frame at once
    for (size_t i = 0; i < len; i++) {
        ir_send_byte(data[i]);
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay between bytes
    }
}

extern "C" uint8_t decode_symbols_to_byte(const void* symbols, size_t count) {
    uint8_t b = 0;
    if (::decode_symbols_to_byte_valid(static_cast<const rmt_symbol_word_t*>(symbols), count, &b)) {
        return b;
    }
    return 0; // caller must check validity to avoid "all zeros" UI
}

// ---- Beacon task for HOLDER role ----
static void role_beacon_task(void *arg) {
    // Wakeups are driven by esp_timer → vTaskNotifyGive()
    for (;;) {
        // Block until the next beacon tick
        ulTaskNotifyTake(pdTRUE /*clear on exit*/, portMAX_DELAY);
        
        // Only the holder transmits, and only after startup guard
        if (g_role != ROLE_HOLDER) continue;
        
        int64_t now = esp_timer_get_time();
        if (now < g_holder_startup_until_us) continue;
        
        // One-shot send of the beacon frame (ZT | holderMAC | seq)
        ir_send_ZT_LEN_MAC_CRC_oneshot();
    }
}

// ----------------------
// Print binary representation
// ----------------------
static void print_binary(uint8_t val, const char *label, int x, int y) {
    display.setCursor(x, y);
    display.print(label);
    display.print(": ");
    
    // Print binary
    for (int b = 7; b >= 0; b--) {
        display.print((val & (1 << b)) ? '1' : '0');
    }
    
    // Print hex
    display.printf(" (0x%02X)", val);
    
    // Print ASCII if printable
    if (val >= 32 && val <= 126) {
        display.printf(" '%c'", val);
    }
}

// ----------------------
// Update display
// ----------------------
static void update_display() {
    display.fillScreen(TFT_BLACK);
    display.setTextSize(2);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    
    // Show current role prominently
    display.setCursor(0, 60);
    display.setTextColor(TFT_YELLOW, TFT_BLACK);
    const char* role_str = (g_role == ROLE_HOLDER) ? "HOLDER" : 
                          (g_role == ROLE_RUNNER) ? "RUNNER" : "PENDING";
    display.printf("ROLE: %s", role_str);
    
    display.setTextSize(1);
    display.setCursor(0, 120);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.println("Hot Potato Demo");
}

// TX Task removed - using automatic beacon instead

// Button task removed - using automatic beacon instead

// Button setup function removed - using automatic beacon instead

// ----------------------
// Main test function
// ----------------------
extern "C" void ir_simple_test_main(void) {
    IR_LOGI(TAG, "Starting IR Simple Test");
    
    // Initialize watchdog timer (may already be initialized by ESP-IDF)
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 10000,  // 10 second timeout
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,    // Bitmask of all idle cores to reset
        .trigger_panic = true // Trigger panic when watchdog timeout occurs
    };
    esp_err_t wdt_err = esp_task_wdt_init(&wdt_config);
    if (wdt_err != ESP_OK && wdt_err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(wdt_err);
    }
    // Add current task to watchdog (may already be added)
    wdt_err = esp_task_wdt_add(NULL);
    if (wdt_err != ESP_OK && wdt_err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(wdt_err);
    }
    
    // Get our MAC address for display
    uint8_t my_mac[6];
    esp_read_mac(my_mac, ESP_MAC_WIFI_STA);
    snprintf(g_mac_line, sizeof(g_mac_line), "MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             my_mac[0], my_mac[1], my_mac[2], my_mac[3], my_mac[4], my_mac[5]);
    ESP_LOGI(TAG, "%s", g_mac_line);
    
    // Initialize display
    display.begin();
    display.setTextSize(1);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.fillScreen(TFT_BLACK);
    display.setCursor(0, 0);
    display.println("Initializing...");
    
    // Button setup removed - using automatic beacon instead
    
    // Setup TX and selected RX backend
    setup_rmt_tx();
#if IR_SELF_FILTER
    esp_read_mac(g_self_mac, ESP_MAC_WIFI_STA);
    ESP_LOGI("CFG", "Self MAC = %02X:%02X:%02X:%02X:%02X:%02X",
             g_self_mac[0], g_self_mac[1], g_self_mac[2],
             g_self_mac[3], g_self_mac[4], g_self_mac[5]);
#endif
#if IR_USE_ESPNOW_ACK
    if (!g_ack_queue) g_ack_queue = xQueueCreate(4, 6); // 4 MACs
    if (!g_pass_req_queue) g_pass_req_queue = xQueueCreate(2, sizeof(uint8_t)); // PASS_REQ events
    
    // Create timers for PASS_REQ slotting
    esp_timer_create_args_t pass_req_timer_args = {
        .callback = pass_req_timer_callback,
        .arg = nullptr,
        .name = "pass_req_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&pass_req_timer_args, &g_pass_req_timer));
    
    esp_timer_create_args_t retry_timer_args = {
        .callback = retry_timer_callback,
        .arg = nullptr,
        .name = "retry_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&retry_timer_args, &g_retry_timer));
    
    // Create ACK slot timer for IR backoff
    if (g_ack_slot_timer == NULL) {
        const esp_timer_create_args_t a = {
            .callback = &ack_slot_timer_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "ack_slot"
        };
        ESP_ERROR_CHECK(esp_timer_create(&a, &g_ack_slot_timer));
    }
    
    espnow_init_once();
#endif
#if IR_RX_BACKEND_UART
    setup_uart_rx();
#else
    setup_rmt_rx();
#endif
    
#if IR_RX_BACKEND_UART
    xTaskCreate(uart_rx_task, "uart_rx_task", 8192, NULL, 5, NULL);
#else
    // Register RX callback
    rmt_rx_event_callbacks_t rx_cbs = {
        .on_recv_done = rmt_rx_done_callback
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rmt_rx_chan, &rx_cbs, NULL));
    
    // Start RX task
    xTaskCreate(rx_task, "rx_task", 8192, NULL, 5, NULL);
#endif

    // Start with RX enabled only if we're not the holder
    ir_rx_set_enabled(START_AS_HOLDER ? false : true);
    
    // M5Stack FIRE side LEDs reset (WS2812/SK6812 style on GPIO 15)
    static led_strip_handle_t s_reset_strip = NULL;
    led_strip_config_t strip_cfg = {
        .strip_gpio_num = HP_LED_GPIO,
        .max_leds       = HP_LED_COUNT,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = 10 * 1000 * 1000,  // 10 MHz
        .mem_block_symbols = 64,
    };
    if (led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_reset_strip) == ESP_OK) {
        // Send all zeros a couple of times just to be sure
        led_strip_clear(s_reset_strip);
        led_strip_refresh(s_reset_strip);
        vTaskDelay(pdMS_TO_TICKS(10));
        led_strip_clear(s_reset_strip);
        led_strip_refresh(s_reset_strip);
        ESP_LOGI(TAG, "M5Stack FIRE side LEDs reset - should be OFF now");
        
        // Keep the handle for potential future use
        led_strip = s_reset_strip;
    } else {
        ESP_LOGW(TAG, "led_strip init failed; cannot clear LEDs");
        led_strip = NULL;
    }
    

    
    // Set initial startup delay for HOLDER role
    if (g_role == ROLE_HOLDER) {
        g_holder_startup_until_us = esp_timer_get_time() + (int64_t)HOLDER_STARTUP_DELAY_MS * 1000;
        ESP_LOGI(TAG, "Initial HOLDER - startup delay %dms", HOLDER_STARTUP_DELAY_MS);
    }
    
    // Start beacon task for role-based potato passing
    xTaskCreate(role_beacon_task, "role_beacon", 4096, NULL, 4, &g_beacon_task);
    
    // Create & start periodic esp_timer for beacon cadence
    if (g_beacon_timer == NULL) {
        const esp_timer_create_args_t beacon_args = {
            .callback = &beacon_timer_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "beacon_tick"
        };
        ESP_ERROR_CHECK(esp_timer_create(&beacon_args, &g_beacon_timer));
    }
    // Always run the periodic tick; the task will gate on role/startup
    ESP_ERROR_CHECK(esp_timer_start_periodic(g_beacon_timer, (uint64_t)BEACON_PERIOD_MS * 1000ULL));
    
    IR_LOGI(TAG, "IR Simple Test (Stage-2a) ready - potato passing demo");
    
    // Initial display update
    update_display();
    
    // Initial LED state update
    update_led_state();
    
    // Main loop - only update display when role changes
    while (1) {
        if (display_needs_update) {
            update_display();
            display_needs_update = false;
        }
        
#if IR_USE_ESPNOW_ACK
        uint8_t ack_mac[6];
        if (g_ack_queue && xQueueReceive(g_ack_queue, ack_mac, 0) == pdTRUE) {
            // Format "aa bb cc dd ee ff" for the LCD line
            snprintf(g_ack_hex, sizeof(g_ack_hex),
                     "%02X %02X %02X %02X %02X %02X",
                     ack_mac[0], ack_mac[1], ack_mac[2],
                     ack_mac[3], ack_mac[4], ack_mac[5]);
            ESP_LOGI("ESPNOW", "ACK received from %s", g_ack_hex);
        }
        
        // Process PASS_REQ timer events
        uint8_t dummy;
        if (g_pass_req_queue && xQueueReceive(g_pass_req_queue, &dummy, 0) == pdTRUE) {
            if (g_role == ROLE_RUNNER || g_role == ROLE_PENDING) {
                // Send PASS_REQ
                esp_err_t err = espnow_send_pass_req(g_pending_pass_req.holder_mac, g_pending_pass_req.seq);
                if (err == ESP_OK) {
                    ESP_LOGI("PASS_REQ", "Sent seq=%u to %02X:%02X:%02X:%02X:%02X:%02X",
                             g_pending_pass_req.seq,
                             g_pending_pass_req.holder_mac[0], g_pending_pass_req.holder_mac[1],
                             g_pending_pass_req.holder_mac[2], g_pending_pass_req.holder_mac[3],
                             g_pending_pass_req.holder_mac[4], g_pending_pass_req.holder_mac[5]);
                    
                    if (PASS_COMMIT_MODE == PASS_COMMIT_IMMEDIATE) {
                        // Legacy mode: flip immediately
                        int64_t now = esp_timer_get_time();
                        memcpy(g_last_peer_mac, g_pending_pass_req.holder_mac, 6);
                        g_no_tagback_until_us = now + (int64_t)PASS_COOLDOWN_MS * 1000;
                        g_holder_startup_until_us = now + (int64_t)HOLDER_STARTUP_DELAY_MS * 1000;
                        g_role = ROLE_HOLDER;
                        ir_rx_set_enabled(false);
                        update_led_state();
                        display_needs_update = true;
                        ESP_LOGI("CORE", "ROLE -> HOLDER (immediate commit seq=%u)", g_pending_pass_req.seq);
                    }
                } else {
                    ESP_LOGW("PASS_REQ", "Send failed: %s", esp_err_to_name(err));
                }
            }
        }
#endif
        
        // Feed watchdog
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// LED control function - light up red when HOLDER, off otherwise
static void update_led_state(void) {
    if (!led_strip) {
        ESP_LOGW(TAG, "LED strip not initialized!");
        return;
    }
    
    ESP_LOGI(TAG, "update_led_state: role=%d", g_role);
    
    if (g_role == ROLE_HOLDER) {
        // Aggressive reset sequence to ensure clean state before turning on
        ESP_LOGI(TAG, "Resetting LEDs before turning on for HOLDER role");
        
        // Multiple clear attempts to ensure clean state
        for (int i = 0; i < 3; i++) {
            led_strip_clear(led_strip);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        
        // Then turn on red LEDs for all side LEDs
        for (int i = 0; i < HP_LED_COUNT; i++) {
            led_strip_set_pixel(led_strip, i, 255, 0, 0);  // Red for all side LEDs
        }
        led_strip_refresh(led_strip);
        ESP_LOGI(TAG, "LED: ON (HOLDER) - all %d side LEDs red", HP_LED_COUNT);
    } else {
        // For RUNNER and PENDING roles, turn off LED with multiple attempts
        ESP_LOGI(TAG, "Turning off LED for role %d", g_role);
        
        // Very aggressive LED off sequence
        for (int i = 0; i < 5; i++) {
            led_strip_clear(led_strip);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(10));
            
            // Also explicitly set pixel to black
            led_strip_set_pixel(led_strip, 0, 0, 0, 0);  // Black/off
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        // Final clear and refresh
        led_strip_clear(led_strip);
        led_strip_refresh(led_strip);
        
        if (g_role == ROLE_RUNNER) {
            ESP_LOGI(TAG, "LED: OFF (RUNNER - disabled) - aggressive clear completed");
        } else {
            ESP_LOGI(TAG, "LED: OFF (PENDING) - aggressive clear completed");
        }
    }
}
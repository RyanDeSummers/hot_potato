// Optimized IR Test - Sender + Receiver on same device
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
#include "M5GFX.h"
#include "ir_simple_test.h"

#define IR_TX_GPIO GPIO_NUM_26
#define IR_RX_GPIO GPIO_NUM_36
#define BUTTON_A_GPIO GPIO_NUM_39

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

// RMT handles
static rmt_channel_handle_t rmt_tx_chan = NULL;
static rmt_channel_handle_t rmt_rx_chan = NULL;
static rmt_encoder_handle_t tx_copy_encoder = NULL;

// RX state
static volatile bool rx_inflight = false;
static volatile size_t rx_symbol_count = 0;
static SemaphoreHandle_t rx_done_sem = NULL;
static QueueHandle_t rx_byte_queue = NULL;

// Button interrupt state
static TaskHandle_t tx_task_handle = NULL;
static bool use_button_interrupt = false;

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

// ---- Dedupe state ----
typedef struct {
    uint8_t  mac[6];
    uint8_t  last_seq;   // valid only if a SEQ is present; otherwise ignored
    int64_t  last_us;    // last accepted time (µs)
    bool     used;
} dedupe_entry_t;
static dedupe_entry_t g_dedupe[IR_DEDUPE_CAPACITY] = {};

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
static bool dedupe_should_drop_and_update(const uint8_t mac[6], bool has_seq, uint8_t seq) {
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
        if (len >= 1 && data[0] == 0xA1 && g_ack_queue) {
            // Non-blocking post of 6-byte MAC
            xQueueSend(g_ack_queue, info->src_addr, 0);
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

    // Update "LAST SENT" line with "5A 54"
    last_sent_byte = 0x54; // keep old field for compatibility
    g_tx_hex[0] = 0; 
    uint8_t zt[2] = {0x5A, 0x54}; 
    hex_append(g_tx_hex, sizeof(g_tx_hex), zt, 2);
    display_needs_update = true;
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

    // Update "LAST SENT" line: "5A 54 xx xx xx xx xx xx"
    g_tx_hex[0] = 0;
    uint8_t head[2] = { IR_PREAMBLE0, IR_PREAMBLE1 };
    hex_append(g_tx_hex, sizeof(g_tx_hex), head, 2);
    hex_append(g_tx_hex, sizeof(g_tx_hex), mac, IR_MAC_LEN);
    display_needs_update = true;
    IR_LOGI(TAG, "Sent ZT+MAC (%u symbols)", (unsigned)n);
}

static void ir_send_ZT_LEN_MAC_CRC_oneshot(void) {
    // Build the payload: LEN + MAC
    uint8_t mac[IR_MAC_LEN];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    const uint8_t len = IR_MAC_LEN;

    // Compute CRC over [LEN || MAC]
    uint8_t crc = 0;
    {
        uint8_t tmp[1 + IR_MAC_LEN];
        tmp[0] = len;
        memcpy(tmp + 1, mac, IR_MAC_LEN);
        crc = crc8_maxim(tmp, sizeof(tmp));
    }

    // Build contiguous UART symbols for all bytes
    // Total bytes: 2 (ZT) + 1 (LEN) + 6 (MAC) + 1 (CRC) = 10 -> 100 symbols
    rmt_symbol_word_t syms[160];
    size_t n = 0;
    ir_append_byte_syms(IR_PREAMBLE0, syms, &n);
    ir_append_byte_syms(IR_PREAMBLE1, syms, &n);
    ir_append_byte_syms(len,          syms, &n);
    for (int i = 0; i < IR_MAC_LEN; ++i) ir_append_byte_syms(mac[i], syms, &n);
    ir_append_byte_syms(crc,          syms, &n);

    // Transmit one shot
    rmt_transmit_config_t cfg = { .loop_count = 0, .flags = {0} };
    ESP_ERROR_CHECK(rmt_transmit(rmt_tx_chan, tx_copy_encoder, syms, n * sizeof(syms[0]), &cfg));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(rmt_tx_chan, pdMS_TO_TICKS(1000)));

    // Update LAST SENT: "5A 54 06 <MAC...> <CRC>"
    g_tx_hex[0] = 0;
    uint8_t head[3] = { IR_PREAMBLE0, IR_PREAMBLE1, len };
    hex_append(g_tx_hex, sizeof(g_tx_hex), head, 3);
    hex_append(g_tx_hex, sizeof(g_tx_hex), mac, IR_MAC_LEN);
    hex_append(g_tx_hex, sizeof(g_tx_hex), &crc, 1);
    display_needs_update = true;
    IR_LOGI(TAG, "Sent ZT+LEN+MAC+CRC (len=%u)", (unsigned)len);
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

// ----------------------
// Button interrupt handler (ISR)
// ----------------------
static void IRAM_ATTR button_isr(void* arg) {
    if (tx_task_handle == NULL) return;  // guard against early IRQ before task exists
    BaseType_t hp = pdFALSE;
    xTaskNotifyFromISR(tx_task_handle, 1, eSetBits, &hp);
    if (hp) portYIELD_FROM_ISR();
}

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
                        display_needs_update = true;
                        st = WAIT_T;
                    }
                    break;

                case WAIT_T:
                    hex_append(g_rx_work_hex, sizeof(g_rx_work_hex), &b, 1);
                    display_needs_update = true;
                    if (b == IR_PREAMBLE1) {
                        st = READ_LEN;
                    } else {
                        ESP_LOGW("RX", "Unexpected second byte after Z: 0x%02X", b);
                        g_rx_work_hex[0] = 0; // Clear the in-progress line
                        display_needs_update = true;
                        st = WAIT_Z;
                    }
                    break;

                case READ_LEN:
                    len = b;
                    hex_append(g_rx_work_hex, sizeof(g_rx_work_hex), &b, 1);
                    display_needs_update = true;
                    if (len == 0 || len > IR_MAX_PAYLOAD) {
                        ESP_LOGW("RX", "LEN invalid: %u", (unsigned)len);
                        g_rx_work_hex[0] = 0; // Clear the in-progress line
                        display_needs_update = true;
                        st = WAIT_Z;
                        break;
                    }
                    pi = 0;
                    st = READ_PAYLOAD;
                    break;

                case READ_PAYLOAD:
                    payload[pi++] = b;
                    hex_append(g_rx_work_hex, sizeof(g_rx_work_hex), &b, 1);
                    display_needs_update = true;
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
                        bool has_seq = (IR_ENABLE_SEQ_FUTURE && len == (IR_MAC_LEN + 1));
                        uint8_t seq = has_seq ? payload[0] : 0;
                        const uint8_t *mac_ptr = has_seq ? (payload + 1) : payload; // sender MAC
#if IR_SELF_FILTER
                        bool is_self = (memcmp(mac_ptr, g_self_mac, IR_MAC_LEN) == 0);
                        if (is_self) {
                            // Ignore our own frame; don't promote
                            ESP_LOGI("RX", "CRC OK but ignored self frame (our MAC)");
                            // Clear the in-progress line so the UI doesn't show stale bytes
                            g_rx_work_hex[0] = 0;
                            display_needs_update = true;
                            st = WAIT_Z;
                            break;
                        }
#endif
                        // Dedupe (drops fast repeats or identical SEQ)
                        if (dedupe_should_drop_and_update(mac_ptr, has_seq, seq)) {
                            ESP_LOGI("RX", "Duplicate frame suppressed (MAC%s, %s)",
                                     "", has_seq ? "SEQ" : "time-window");
                            g_rx_work_hex[0] = 0;
                            display_needs_update = true;
                            st = WAIT_Z;
                            break;
                        }

                        // Not self and not duplicate — send ESP-NOW ACK back to sender MAC
#if IR_USE_ESPNOW_ACK
                        (void)espnow_send_ack_to(mac_ptr, seq);
#endif
                        // Promote to LAST RECEIVED
                        strlcpy(g_rx_last_hex, g_rx_work_hex, sizeof(g_rx_last_hex));
                        g_rx_work_hex[0] = 0;
                        display_needs_update = true;
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
                        // CRC fail — show what we saw but call it out in logs
                        strlcpy(g_rx_last_hex, g_rx_work_hex, sizeof(g_rx_last_hex));
                        g_rx_work_hex[0] = 0;
                        display_needs_update = true;
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

    // Promote RECEIVING -> LAST RECEIVED here (single redraw)
    if (rx_count > 0) {
        strlcpy(g_rx_last_hex, g_rx_work_hex, sizeof(g_rx_last_hex));
        g_rx_work_hex[0] = 0; // clear working buffer
        display_needs_update = true;
        
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
    display.setTextSize(1);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    
    display.setCursor(0, 0);
    display.println("IR Simple Test (Stage-2a)");
    
    // Show MAC address
    display.setCursor(0, 16);
    display.setTextColor(TFT_CYAN, TFT_BLACK);
    display.println(g_mac_line);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    
    // Show LAST SENT
    display.setCursor(0, 32);
    display.setTextColor(TFT_GREEN, TFT_BLACK);
    display.print("LAST SENT: ");
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.println(g_tx_hex[0] ? g_tx_hex : "-");
    
    // Show RECEIVING (if any)
    if (g_rx_work_hex[0]) {
        display.setCursor(0, 48);
        display.setTextColor(TFT_YELLOW, TFT_BLACK);
        display.print("RECEIVING: ");
        display.setTextColor(TFT_WHITE, TFT_BLACK);
        display.println(g_rx_work_hex);
    }
    
    // Show LAST RECEIVED
    display.setCursor(0, 64);
    display.setTextColor(TFT_MAGENTA, TFT_BLACK);
    display.print("LAST RECEIVED: ");
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.println(g_rx_last_hex[0] ? g_rx_last_hex : "-");
    
#if IR_USE_ESPNOW_ACK
    display.setCursor(0, 80);
    display.setTextColor(TFT_YELLOW, TFT_BLACK);
    display.print("LAST ACK FROM: ");
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.println(g_ack_hex[0] ? g_ack_hex : "-");
#endif
    
    display.setCursor(0, 100);
    display.println("Press Button A to send ZT+LEN+MAC+CRC");
}

// ----------------------
// TX Task (for interrupt-driven button)
// ----------------------
static void tx_task(void *pvParameters) {
    IR_LOGI(TAG, "TX task started");
    
    while (1) {
        // Wait for button interrupt notification
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        // Send ZT + LEN + MAC + CRC in one shot
        ir_send_ZT_LEN_MAC_CRC_oneshot();
        
        // Small delay to prevent rapid-fire
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ----------------------
// Button task (polling fallback)
// ----------------------
static void button_task(void *pvParameters) {
    bool last_button_state = true;
    
    while (1) {
        bool current_button_state = (gpio_get_level(BUTTON_A_GPIO) == 0);
        
        if (current_button_state && !last_button_state) {
            // Button pressed - send ZT+LEN+MAC+CRC in one shot
            ir_send_ZT_LEN_MAC_CRC_oneshot();
            
            // Debounce
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        
        last_button_state = current_button_state;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ----------------------
// Setup button (try interrupt, fallback to polling)
// ----------------------
static void setup_button() {
    // Try to setup button interrupt first
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << BUTTON_A_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, // GPIO34-39 have no internal pull-ups
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,    // press -> falling edge (if externally pulled up)
    };
    
    esp_err_t ret = gpio_config(&io);
    if (ret == ESP_OK) {
        ret = gpio_install_isr_service(0);
        if (ret == ESP_OK) {
            ret = gpio_isr_handler_add(BUTTON_A_GPIO, button_isr, NULL);
            if (ret == ESP_OK) {
                use_button_interrupt = true;
                IR_LOGI(TAG, "Button interrupt setup successful");
            } else {
                IR_LOGI(TAG, "Button interrupt handler add failed, using polling");
            }
        } else {
            IR_LOGI(TAG, "Button interrupt service install failed, using polling");
        }
    } else {
        IR_LOGI(TAG, "Button interrupt config failed, using polling");
    }
    
    // Fallback to polling if interrupt setup failed
    if (!use_button_interrupt) {
        gpio_config_t btn_cfg = {};
        btn_cfg.pin_bit_mask = 1ULL << BUTTON_A_GPIO;
        btn_cfg.mode = GPIO_MODE_INPUT;
        btn_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
        gpio_config(&btn_cfg);
        IR_LOGI(TAG, "Button polling mode enabled");
    }
}

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
    
    // Setup button (interrupt or polling)
    setup_button();
    
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
    
    // Start appropriate button task
    if (use_button_interrupt) {
        xTaskCreate(tx_task, "tx_task", 8192, NULL, 4, &tx_task_handle);
    } else {
        xTaskCreate(button_task, "button_task", 8192, NULL, 4, NULL);
    }
    
    IR_LOGI(TAG, "IR Simple Test (Stage-2a) ready - press Button A to send ZT");
    
    // Initial display update
    update_display();
    
    // Main display loop
    while (1) {
        if (display_needs_update) {
            update_display();
            display_needs_update = false;
        }
        
#if !IR_RX_BACKEND_UART
        // Check for received bytes (for compatibility)
        uint8_t received_byte;
        if (xQueueReceive(rx_byte_queue, &received_byte, 0) == pdTRUE) {
            last_received_byte = received_byte;
            has_received = true;
            // Note: display update is handled by process_capture now
        }
#endif

#if IR_USE_ESPNOW_ACK
        uint8_t ack_mac[6];
        if (g_ack_queue && xQueueReceive(g_ack_queue, ack_mac, 0) == pdTRUE) {
            // Format "aa bb cc dd ee ff" for the LCD line
            snprintf(g_ack_hex, sizeof(g_ack_hex),
                     "%02X %02X %02X %02X %02X %02X",
                     ack_mac[0], ack_mac[1], ack_mac[2],
                     ack_mac[3], ack_mac[4], ack_mac[5]);
            display_needs_update = true;
            ESP_LOGI("ESPNOW", "ACK received from %s", g_ack_hex);
        }
#endif
        
        // Feed watchdog
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

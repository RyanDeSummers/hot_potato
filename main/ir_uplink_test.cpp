// Mode B: HOLDER receives IR; RUNNERS transmit IR using slotted IR backoff.
// Reuses your UART IR decoder and RMT TX patterns; uses ESP-NOW to announce
// contention windows and to deliver GRANT/STATE_SYNC.
//
// ESP-IDF 5.3.3-only APIs, no deprecated functions.

// Build configuration: ZT_IS_HOST determines host vs peer behavior
// Must be defined via build flag: -DCMAKE_CXX_FLAGS="-DZT_IS_HOST=1" for HOST
// or -DCMAKE_CXX_FLAGS="-DZT_IS_HOST=0" for PEER

#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_rom_crc.h"
#include "driver/uart.h"
#include "driver/rmt_tx.h"
// IR constants (local definitions to avoid header conflicts)
#define SLOTS 6
#define SLOT_US 52000   // 6 slots * ~52ms ≈ 312ms window; fits 12-byte frame @2400 bps with guard
#define SLOT_JITTER_MAX_US 200
#define BEACON_PERIOD_MS 300
#define PASS_COOLDOWN_MS 1000
// Match the configured UART baud (2400-8N1): ~416.7us/bit.
// Use 417 to stay slightly on the slow side for tolerance.
#define IR_BIT_US 417
#define IR_MAC_LEN 6
#define IR_PREAMBLE0 0x5A
#define IR_PREAMBLE1 0x54
#define IR_MAX_PAYLOAD 32
#define ESPNOW_CHANNEL 1
#define HP_LED_GPIO 15
#define HP_LED_COUNT 12
#define HOLDER_STARTUP_DELAY_MS 100
#include "zt_test_mode.h"

// External access to global variables from zt_test_mode.cpp
extern zt_roster_t g_roster;
extern int g_last_granted_idx;    // for g_T0_us and zt_* helpers

#ifndef MODE_B_USE_ACK2
#define MODE_B_USE_ACK2 1
#endif

static void zt_render_countdown(int secs, const uint8_t initial_holder[6], const zt_roster_t* roster);
static void ir_uplink_recv_cb(const esp_now_recv_info* info, const uint8_t* data, int len);

extern "C" {
  #include "led_strip.h"
}
#include "M5GFX.h"

// ---- External singletons (already defined in main.cpp & ir_simple_test.cpp) ----
extern M5GFX display;
extern led_strip_handle_t led_strip;

// ---- GPIOs (match your existing wiring) ----
#define IR_TX_GPIO       GPIO_NUM_26
#define IR_RX_GPIO       GPIO_NUM_36  // Reverted back - GPIO36 is fine for IR RX
#define IR_UART_PORT     UART_NUM_2

// ---- Task and timer configuration ----
#define TASK_STACK_SIZE          4096
#define TASK_PRIORITY_HIGH       6
#define TASK_PRIORITY_MEDIUM     5
#define TASK_PRIORITY_LOW        4

// ---- IR transmission configuration ----
#define IR_CARRIER_FREQ_HZ       38000
#define IR_CARRIER_DUTY_CYCLE    0.33
#define IR_CARRIER_POLARITY      0
#define IR_TX_WAIT_TIMEOUT_MS   1000

// ---- UART configuration ----
#define UART_BAUD_RATE           2400
#define UART_DATA_BITS           UART_DATA_8_BITS
#define UART_PARITY              UART_PARITY_DISABLE
#define UART_STOP_BITS           UART_STOP_BITS_1
#define UART_FLOW_CTRL           UART_HW_FLOWCTRL_DISABLE
#define UART_RX_BUFFER_SIZE      1024
#define UART_TX_BUFFER_SIZE      0
#define UART_QUEUE_SIZE          0
#define UART_TIMEOUT_MS          20

// ---- LED configuration ----
#define LED_COUNT                HP_LED_COUNT
#define LED_COLOR_RED           255
#define LED_COLOR_GREEN         0
#define LED_COLOR_BLUE          0

static const char* TAG = "IR_UPLINK_TEST";

// ---- Game state and role management ----
typedef enum { ROLE_HOLDER, ROLE_RUNNER } role_t;
static volatile role_t g_role = ZT_IS_HOST ? ROLE_HOLDER : ROLE_RUNNER;
static uint8_t g_self_mac[6] = {0};
static uint8_t g_last_peer_mac[6] = {0};

bool g_game_started = false;  // Track when countdown is complete (non-static for access from zt_test_mode.cpp)

// ---- Cooldown and timing management ----
static int64_t g_no_tagback_until_us = 0;      // Prevents rapid passbacks
static int64_t g_holder_startup_until_us = 0;  // Holder startup delay

// -----------------------------------------------------------------------------
// Fair candidate selection (using centralized logic from zt_test_mode.cpp)
// -----------------------------------------------------------------------------
static int g_window_target_idx = -1;          // index of target for this window
static uint8_t g_window_target_mac[6] = {0};  // MAC of target for this window

// Adaptive candidate sweep with strike tracking
static uint8_t g_unresp_strikes[ZT_MAX_PEERS];
#define MAX_UNRESP_STRIKES  8   // ~8 windows (~1.8–2.1s) before rotating


/**
 * @brief Reset unresponsive strike counters for a new round
 */
static void round_reset_unresp() {
  memset(g_unresp_strikes, 0, sizeof(g_unresp_strikes));
}

/**
 * @brief Public interface to reset unresponsive strike counters
 * 
 * Called from zt_test_mode.cpp when a new round starts
 */
void zt_round_reset_unresp(void) {
  round_reset_unresp();
}

/**
 * @brief Pick next unseen+alive candidate for fair sweep
 * 
 * @param out_idx Output index of selected candidate
 * @param out_mac Output MAC of selected candidate
 * @return true if a candidate was found
 */
static bool pick_next_unseen_alive(int* out_idx, uint8_t out_mac[6]) {
  const int n = g_roster.count;
  if (n <= 1) return false;
  
  int start = (g_last_granted_idx >= 0) ? (g_last_granted_idx + 1) % n : 0;
  
  for (int off = 0; off < n; ++off) {
    int i = (start + off) % n;
    const uint8_t* mac = g_roster.macs[i];
    
    // Don't pass to self (use actual self MAC, not roster[0])
    if (memcmp(mac, g_self_mac, 6) == 0) continue;      // don't pass to self
    if (zt_distinct_already_seen(i)) continue;          // already seen this round
    if (!zt_peer_alive(i)) continue;                    // not alive
    if (!zt_cooldown_ok(mac)) continue;                 // cooldown check
    
    memcpy(out_mac, mac, 6);
    *out_idx = i;
    return true;
  }
  return false;
}

// ---- RMT TX (IR) ----
static rmt_channel_handle_t rmt_tx_chan = nullptr;
static rmt_encoder_handle_t tx_copy_encoder = nullptr;

// --- Soft wait helper (non-fatal, with proper timeout for 2400bps frames) ---
static bool rmt_soft_wait_done(rmt_channel_handle_t ch, uint32_t ms_soft) {
    esp_err_t err = rmt_tx_wait_all_done(ch, ms_soft);
    if (err == ESP_OK) return true;
    if (err == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "RMT: soft timeout after %u ms (continuing)", (unsigned)ms_soft);
        return false;
    }
    // Any other error: log and continue (non-fatal)
    ESP_LOGE(TAG, "RMT: wait_all_done err=%d (continuing)", (int)err);
    return false;
}

// ---- UART RX (IR demod as 2400-8N1) ----
static bool g_rx_enabled = true;
static void ir_rx_set_enabled(bool en) { g_rx_enabled = en; }

// ---- ESP-NOW infrastructure ----
static bool g_espnow_ready = false;

// ---- Task and timer handles ----
// (These are declared later in the file)

// ---- Window and slotting management ----
// We reuse SLOTS/SLOT_US/SLOT_JITTER_MAX_US from ir_simple_test.h
// For IR uplink, slot length must cover a full 12-byte 2400bps frame (~50ms) + guard.
#ifndef UPLINK_SLOT_US
#define UPLINK_SLOT_US 52000
#endif
#ifndef UPLINK_K
#define UPLINK_K SLOTS    // default to same macro
#endif

// ---- Window state tracking ----
static uint16_t g_window_seq = 0;           // Current contention sequence
static bool g_granted_seq_valid = false;    // Whether we have a valid granted sequence
static uint16_t g_granted_seq = 0;         // Last granted sequence number
static uint32_t g_empty_windows = 0;        // Count consecutive empty windows

// Note: IR collision tracking variables removed - were making the game worse

// ---- Handshake state management ----
// Holder side: waiting for ACK2 from runner
static bool g_wait_ack2 = false;
static uint8_t g_wait_ack2_mac[6] = {0};
static uint16_t g_wait_ack2_seq = 0;
static esp_timer_handle_t g_ack2_timer = nullptr;

// Runner side: tracking IR transmission (waiting for ACK1)
static bool g_wait_ack1 = false;
static uint16_t g_wait_ack1_seq = 0;

// ---- Handshake timing ----
#ifndef ACK2_TIMEOUT_MS
#define ACK2_TIMEOUT_MS 30
#endif

// --- Message types (ESP-NOW) ---
#define MSG_PASS_GRANT   0xA2  // (same as Mode A)
#define MSG_STATE_SYNC   0xA3  // (same as Mode A)
#define MSG_IR_WINDOW    0xA4  // holder -> all runners: start an IR uplink window
#define MSG_ACK1_H2R     0xB1  // holder -> runner: IR seen
#define MSG_ACK2_R2H     0xB2  // runner -> holder: confirm

// Forward declarations
static esp_err_t espnow_send_ack1_h2r(const uint8_t* runner_mac, uint16_t seq);
static esp_err_t espnow_send_ack2_r2h(const uint8_t* holder_mac, uint16_t seq);
static void holder_commit_after_ack2(const uint8_t* winner_mac, uint16_t seq);
static void ack2_timer_cb(void*);

// Collision detection and learning functions
// Note: Collision handling functions removed - were making the game worse

// MSG_IR_WINDOW payload:
// [0]=0xA4, [1]=seq_lo, [2]=seq_hi, [3..8]=holder_mac, [9]=K, [10..13]=delta_us (T0-now)

// --- Small helpers reused from your code style ---
static inline uint8_t crc8_maxim(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i=0;i<len;++i) {
    crc ^= data[i];
    for (int b=0;b<8;++b) crc = (crc & 0x80) ? (uint8_t)((crc<<1)^0x31) : (uint8_t)(crc<<1);
  }
  return crc;
}

static uint16_t crc16_le_buf(const uint8_t* p, size_t n) {
  // Cheap CRC16 (same as esp_rom_crc16_le)
  return esp_rom_crc16_le(0, p, (uint32_t)n);
}

typedef struct { uint8_t slot; uint16_t jitter_us; } slot_pick_t;
static inline slot_pick_t pick_slot(uint16_t seq, const uint8_t holder[6], const uint8_t mine[6], uint8_t K) {
  uint8_t buf[2+6+6];
  buf[0]=(uint8_t)(seq&0xFF); buf[1]=(uint8_t)(seq>>8);
  memcpy(&buf[2], holder, 6);
  memcpy(&buf[8], mine,   6);
  uint16_t h16 = crc16_le_buf(buf, sizeof(buf));
  uint8_t  h8  = 0; for (int i=0;i<sizeof(buf);++i) h8 ^= buf[i];
  
  // Start with deterministic slot based on hash
  uint8_t base_slot = (uint8_t)(h16 % (K?K:1));
  
  // Note: Complex collision avoidance removed - was making the game worse
  // Keep simple deterministic slot selection for better performance
  
  // Simple jitter to break synchronization
  uint32_t base_jitter = SLOT_JITTER_MAX_US / 4;  // 25% of max jitter as base
  uint32_t random_offset = rand() % (base_jitter / 2);
  uint32_t final_jitter = base_jitter + random_offset;
  
  slot_pick_t r;
  r.slot = base_slot;
  r.jitter_us = (uint16_t)(final_jitter % SLOT_JITTER_MAX_US);
  
  // Note: Slot tracking removed - no longer needed
  
  return r;
}

/**
 * @brief Sets up the RMT (Remote Control) transmitter for IR communication
 * 
 * Configures the RMT peripheral to transmit IR signals on the specified GPIO pin.
 * Sets up carrier frequency, duty cycle, and encoder for UART-style IR transmission.
 * 
 * @return void
 * 
 * @note This function must be called before any IR transmission can occur.
 * @note Uses ESP_ERROR_CHECK for critical errors - will abort on failure.
 */
static void setup_rmt_tx() {
  ESP_LOGI(TAG, "Setting up RMT TX on GPIO %d", IR_TX_GPIO);
  
  // Configure RMT TX channel
  rmt_tx_channel_config_t txc = {
    .gpio_num = IR_TX_GPIO,
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .resolution_hz = 1000000, // 1us resolution
    .mem_block_symbols = 128,
    .trans_queue_depth = 4,
    .intr_priority = 0
  };
  
  esp_err_t err = rmt_new_tx_channel(&txc, &rmt_tx_chan);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create RMT TX channel: %s", esp_err_to_name(err));
    return;
  }
  
  // Configure IR carrier (38kHz with 33% duty cycle)
  rmt_carrier_config_t carrier = { 
    .frequency_hz = IR_CARRIER_FREQ_HZ, 
    .duty_cycle = IR_CARRIER_DUTY_CYCLE, 
    .flags = { 
      .polarity_active_low = IR_CARRIER_POLARITY
    } 
  };
  
  err = rmt_apply_carrier(rmt_tx_chan, &carrier);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to apply carrier: %s", esp_err_to_name(err));
    return;
  }
  
  // Create copy encoder for UART-style transmission
  rmt_copy_encoder_config_t cc = {};
  err = rmt_new_copy_encoder(&cc, &tx_copy_encoder);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create copy encoder: %s", esp_err_to_name(err));
    return;
  }
  
  // Enable the RMT channel
  err = rmt_enable(rmt_tx_chan);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable RMT channel: %s", esp_err_to_name(err));
    return;
  }
  
  ESP_LOGI(TAG, "RMT TX setup complete on GPIO %d", IR_TX_GPIO);
}

// Make 10 UART bits of 'b' into RMT symbols (mark=0, space=1 @ IR_BIT_US)
static size_t build_uart_byte(uint8_t b, rmt_symbol_word_t* out) {
  auto set_mark  = [](rmt_symbol_word_t* s, uint32_t us){ s->level0=1; s->duration0=us; s->level1=0; s->duration1=1; };
  auto set_space = [](rmt_symbol_word_t* s, uint32_t us){ s->level0=0; s->duration0=us; s->level1=0; s->duration1=1; };
  size_t i=0;
  set_mark(&out[i++], IR_BIT_US);                 // start (0)
  for (int bit=0; bit<8; ++bit) ((b>>bit)&1) ? set_space(&out[i++],IR_BIT_US) : set_mark(&out[i++],IR_BIT_US);
  set_space(&out[i++], IR_BIT_US);                // stop (1)
  return i;
}

// Runner IR frame: ZT | LEN | [SEQ_LO|SEQ_HI] | RUNNER_MAC(6) | CRC
static void runner_send_ir_uplink(uint16_t seq) {
  // Early self-gate: prevent already-seen runners from transmitting
  const int self_idx = zt_roster_index_of(g_self_mac);
  if (self_idx >= 0 && zt_distinct_already_seen(self_idx)) {
    ESP_LOGI(TAG, "RUNNER: already seen this round; skipping IR transmission");
    return;
  }
  
  uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_WIFI_STA);
  const uint8_t len = (uint8_t)(IR_MAC_LEN + 2);
  uint8_t crc_src[1 + 2 + IR_MAC_LEN];
  crc_src[0] = len;
  crc_src[1] = (uint8_t)(seq & 0xFF);
  crc_src[2] = (uint8_t)(seq >> 8);
  memcpy(&crc_src[3], mac, IR_MAC_LEN);
  const uint8_t crc = crc8_maxim(crc_src, sizeof(crc_src));

  rmt_symbol_word_t syms[160]; size_t n=0;
  n += build_uart_byte(IR_PREAMBLE0, &syms[n]);
  n += build_uart_byte(IR_PREAMBLE1, &syms[n]);
  n += build_uart_byte(len,         &syms[n]);
  n += build_uart_byte((uint8_t)(seq & 0xFF), &syms[n]);
  n += build_uart_byte((uint8_t)(seq >> 8),   &syms[n]);
  for (int i=0;i<IR_MAC_LEN;++i) n += build_uart_byte(mac[i], &syms[n]);
  n += build_uart_byte(crc,       &syms[n]);

  rmt_transmit_config_t cfg = { .loop_count = 0 };
  esp_err_t tx_err = rmt_transmit(rmt_tx_chan, tx_copy_encoder, syms, n*sizeof(syms[0]), &cfg);
  if (tx_err != ESP_OK) {
    ESP_LOGE(TAG, "RMT transmit failed: %s", esp_err_to_name(tx_err));
    return;
  }
  
  // Brief wait; a full frame at 2400 bps is ~50 ms, so 60 ms avoids false timeouts
  bool completed = rmt_soft_wait_done(rmt_tx_chan, 60);
  if (completed) {
    ESP_LOGI(TAG, "IR uplink sent successfully (SEQ=%u, %d symbols)", (unsigned)seq, n);
  } else {
    ESP_LOGI(TAG, "IR uplink transmission started (SEQ=%u, %d symbols) - may not have completed", (unsigned)seq, n);
  }
  // Now expect ACK1 for this seq
  g_wait_ack1 = true;
  g_wait_ack1_seq = seq;
}

/**
 * @brief Sets up the UART receiver for IR demodulation
 * 
 * Configures UART2 to receive IR signals demodulated to 2400-8N1 format.
 * This is used by the holder to receive IR uplinks from runners.
 * 
 * @return void
 * 
 * @note This function must be called before any IR reception can occur.
 * @note Uses ESP_ERROR_CHECK for critical errors - will abort on failure.
 */
static void setup_uart_rx() {
  ESP_LOGI(TAG, "Setting up UART RX on GPIO %d", IR_RX_GPIO);
  
  // Configure UART parameters for IR demodulation
  uart_config_t cfg = {
    .baud_rate = UART_BAUD_RATE,
    .data_bits = UART_DATA_BITS,
    .parity = UART_PARITY,
    .stop_bits = UART_STOP_BITS,
    .flow_ctrl = UART_FLOW_CTRL,
    .source_clk = UART_SCLK_DEFAULT
  };
  
  esp_err_t err = uart_param_config(IR_UART_PORT, &cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(err));
    return;
  }
  
  // Install UART driver with RX buffer only
  err = uart_driver_install(IR_UART_PORT, UART_RX_BUFFER_SIZE, UART_TX_BUFFER_SIZE, 
                           UART_QUEUE_SIZE, NULL, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(err));
    return;
  }
  
  // Set UART pins (RX only, TX not used for IR reception)
  err = uart_set_pin(IR_UART_PORT, UART_PIN_NO_CHANGE, IR_RX_GPIO, 
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(err));
    return;
  }
  
  ESP_LOGI(TAG, "UART RX ready on GPIO %d @ %d-8N1", IR_RX_GPIO, UART_BAUD_RATE);
}

/**
 * @brief Ensures NVS (Non-Volatile Storage) is properly initialized
 * 
 * Initializes the NVS flash storage system. If there are issues with
 * free pages or version compatibility, erases and reinitializes the storage.
 * 
 * @return void
 * 
 * @note This function must be called before using any NVS functionality.
 * @note Uses ESP_ERROR_CHECK for critical errors - will abort on failure.
 */
static void ensure_nvs() {
  ESP_LOGI(TAG, "Initializing NVS flash storage");
  
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(TAG, "NVS issues detected, erasing and reinitializing");
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_LOGI(TAG, "NVS reinitialization complete");
  } else if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(err));
    ESP_ERROR_CHECK(err);
  } else {
    ESP_LOGI(TAG, "NVS initialization successful");
  }
}

/**
 * @brief Initializes ESP-NOW communication system once
 * 
 * Sets up WiFi, ESP-NOW, and registers callbacks for Mode B IR uplink communication.
 * This function is idempotent - subsequent calls return immediately if already initialized.
 * 
 * @return esp_err_t ESP_OK on success, error code on failure
 * 
 * @note This function must be called before any ESP-NOW communication can occur.
 * @note Initializes WiFi in station mode with power saving disabled.
 */
static esp_err_t espnow_init_once(void) {
  if (g_espnow_ready) {
    ESP_LOGD(TAG, "ESP-NOW already initialized, skipping");
    return ESP_OK;
  }
  
  ESP_LOGI(TAG, "Initializing ESP-NOW communication system");
  
  // Initialize NVS and networking stack
  ensure_nvs();
  esp_netif_init();
  esp_event_loop_create_default();
  
  // Create default WiFi station interface
  esp_netif_t* netif = esp_netif_create_default_wifi_sta();
  if (!netif) {
    ESP_LOGE(TAG, "Failed to create default WiFi station interface");
    return ESP_FAIL;
  }
  
  // Initialize and configure WiFi
  wifi_init_config_t w = WIFI_INIT_CONFIG_DEFAULT();
  esp_err_t err = esp_wifi_init(&w);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize WiFi: %s", esp_err_to_name(err));
    return err;
  }
  
  err = esp_wifi_set_mode(WIFI_MODE_STA);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(err));
    return err;
  }
  
  err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set WiFi storage: %s", esp_err_to_name(err));
    return err;
  }
  
  err = esp_wifi_start();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(err));
    return err;
  }
  
  err = esp_wifi_set_ps(WIFI_PS_NONE);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to disable WiFi power saving: %s", esp_err_to_name(err));
    return err;
  }
  
  err = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set WiFi channel: %s", esp_err_to_name(err));
    return err;
  }

  // Initialize ESP-NOW
  err = esp_now_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize ESP-NOW: %s", esp_err_to_name(err));
    return err;
  }
   
  // Add broadcast peer for IR window announcements
  uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_peer_info_t bcast_peer = {};
  memcpy(bcast_peer.peer_addr, bcast, 6);
  bcast_peer.ifidx = WIFI_IF_STA;
  bcast_peer.channel = ESPNOW_CHANNEL;
  bcast_peer.encrypt = false;
  
  err = esp_now_add_peer(&bcast_peer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add broadcast peer: %s", esp_err_to_name(err));
    return err;
  }
  
  // Register ESP-NOW callbacks
  err = esp_now_register_send_cb([](const uint8_t* mac, esp_now_send_status_t st){
    ESP_LOGD("ESPNOW","tx %02X:%02X:%02X:%02X:%02X:%02X -> %s",
      mac[0],mac[1],mac[2],mac[3],mac[4],mac[5], st==ESP_NOW_SEND_SUCCESS?"OK":"FAIL");
  });
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register send callback: %s", esp_err_to_name(err));
    return err;
  }
  
  err = esp_now_register_recv_cb(ir_uplink_recv_cb);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register receive callback: %s", esp_err_to_name(err));
    return err;
  }
  
  g_espnow_ready = true;
  ESP_LOGI(TAG, "ESP-NOW initialization complete");
  return ESP_OK;
}

// Reinstall Mode-B ESPNOW recv callback after test harness
extern "C" void ir_uplink_install_recv_cb(void) {
  ESP_ERROR_CHECK(esp_now_register_recv_cb(ir_uplink_recv_cb));
}

/**
 * @brief Ensures a peer device is registered with ESP-NOW
 * 
 * Adds a peer device to the ESP-NOW peer list if it doesn't already exist.
 * This is required before sending ESP-NOW messages to a specific device.
 * 
 * @param mac MAC address of the peer device (6 bytes)
 * @return esp_err_t ESP_OK on success, error code on failure
 * 
 * @note If the peer already exists, this function returns ESP_OK.
 * @note Uses the global ESPNOW_CHANNEL for peer configuration.
 */
static esp_err_t ensure_peer(const uint8_t* mac) {
  if (!mac) {
    ESP_LOGE(TAG, "ensure_peer: invalid MAC address");
    return ESP_ERR_INVALID_ARG;
  }
  
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, mac, 6);
  p.ifidx = WIFI_IF_STA;
  p.channel = ESPNOW_CHANNEL;
  p.encrypt = false;
  
  esp_err_t err = esp_now_add_peer(&p);
  if (err == ESP_ERR_ESPNOW_EXIST) {
    ESP_LOGD(TAG, "Peer %02X:%02X:%02X:%02X:%02X:%02X already exists", 
              mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return ESP_OK;
  } else if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add peer %02X:%02X:%02X:%02X:%02X:%02X: %s", 
              mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], esp_err_to_name(err));
  }
  
  return err;
}

/**
 * @brief Sends a pass grant message to the winning runner
 * 
 * Sends an ESP-NOW message granting the potato to a specific runner.
 * This message includes the sequence number and the winner's MAC address.
 * 
 * @param winner_mac MAC address of the winning runner (6 bytes)
 * @param seq Sequence number of the IR window where the winner was selected
 * @return esp_err_t ESP_OK on success, error code on failure
 * 
 * @note This function must be called when the holder decides to grant the potato.
 * @note Automatically ensures the peer is registered before sending.
 */
static esp_err_t espnow_send_pass_grant(const uint8_t* winner_mac, uint16_t seq) {
  if (!g_espnow_ready) {
    ESP_LOGE(TAG, "espnow_send_pass_grant: ESP-NOW not ready");
    return ESP_ERR_INVALID_STATE;
  }
  
  if (!winner_mac) {
    ESP_LOGE(TAG, "espnow_send_pass_grant: invalid winner MAC");
    return ESP_ERR_INVALID_ARG;
  }
  
  esp_err_t err = ensure_peer(winner_mac);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "espnow_send_pass_grant: failed to ensure peer: %s", esp_err_to_name(err));
    return err;
  }
  
  uint8_t payload[10] = { 
    MSG_PASS_GRANT,
    (uint8_t)(seq & 0xFF), (uint8_t)(seq >> 8),
    winner_mac[0], winner_mac[1], winner_mac[2], 
    winner_mac[3], winner_mac[4], winner_mac[5] 
  };
  
  err = esp_now_send(winner_mac, payload, sizeof(payload));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "espnow_send_pass_grant: failed to send: %s", esp_err_to_name(err));
  } else {
    ESP_LOGI(TAG, "Pass grant sent to %02X:%02X:%02X:%02X:%02X:%02X (seq=%u)", 
              winner_mac[0], winner_mac[1], winner_mac[2], 
              winner_mac[3], winner_mac[4], winner_mac[5], seq);
  }
  
  return err;
}

/**
 * @brief Broadcasts a state synchronization message to all devices
 * 
 * Sends a broadcast ESP-NOW message to synchronize game state across all devices.
 * This message includes the sequence number, new holder's MAC, and cooldown period.
 * 
 * @param seq Sequence number of the IR window where the state change occurred
 * @param new_holder_mac MAC address of the new potato holder (6 bytes)
 * @return esp_err_t ESP_OK on success, error code on failure
 * 
 * @note This function broadcasts to all devices to ensure state consistency.
 * @note Includes cooldown information to prevent rapid passbacks.
 */
static esp_err_t espnow_send_state_sync(uint16_t seq, const uint8_t* new_holder_mac) {
  if (!g_espnow_ready) {
    ESP_LOGE(TAG, "espnow_send_state_sync: ESP-NOW not ready");
    return ESP_ERR_INVALID_STATE;
  }
  
  if (!new_holder_mac) {
    ESP_LOGE(TAG, "espnow_send_state_sync: invalid new holder MAC");
    return ESP_ERR_INVALID_ARG;
  }
  
  uint8_t payload[12] = { 
    MSG_STATE_SYNC,
    (uint8_t)(seq & 0xFF), (uint8_t)(seq >> 8),
    new_holder_mac[0], new_holder_mac[1], new_holder_mac[2],
    new_holder_mac[3], new_holder_mac[4], new_holder_mac[5],
    (uint8_t)(PASS_COOLDOWN_MS & 0xFF), (uint8_t)((PASS_COOLDOWN_MS >> 8) & 0xFF) 
  };
  
  uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_err_t err = esp_now_send(bcast, payload, sizeof(payload));
  
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "espnow_send_state_sync: failed to broadcast: %s", esp_err_to_name(err));
  } else {
    ESP_LOGI(TAG, "State sync broadcast: new holder %02X:%02X:%02X:%02X:%02X:%02X (seq=%u)", 
              new_holder_mac[0], new_holder_mac[1], new_holder_mac[2], 
              new_holder_mac[3], new_holder_mac[4], new_holder_mac[5], seq);
  }
  
  return err;
}

/**
 * @brief Broadcasts an IR window announcement to all runners
 * 
 * Sends a broadcast ESP-NOW message announcing the start of an IR uplink window.
 * This message includes the sequence number, holder's MAC, slot count, and timing.
 * 
 * @param seq Sequence number for this IR window
 * @param K Number of available slots in this window
 * @param delta_us Time delay until window start (microseconds)
 * @return esp_err_t ESP_OK on success, error code on failure
 * 
 * @note This function broadcasts to all devices to coordinate IR transmission.
 * @note Runners use this information to pick their transmission slots.
 */
static esp_err_t espnow_broadcast_ir_window(uint16_t seq, uint8_t K, uint32_t delta_us) {
  if (!g_espnow_ready) {
    ESP_LOGE(TAG, "espnow_broadcast_ir_window: ESP-NOW not ready");
    return ESP_ERR_INVALID_STATE;
  }
  
  // Get this device's MAC address
  uint8_t holder[6];
  esp_err_t err = esp_read_mac(holder, ESP_MAC_WIFI_STA);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "espnow_broadcast_ir_window: failed to read MAC: %s", esp_err_to_name(err));
    return err;
  }
  
  // Build the IR window announcement payload
  uint8_t payload[14] = { 
    MSG_IR_WINDOW,
    (uint8_t)(seq & 0xFF), (uint8_t)(seq >> 8),
    holder[0], holder[1], holder[2], holder[3], holder[4], holder[5],
    K,
    (uint8_t)(delta_us & 0xFF), (uint8_t)((delta_us >> 8) & 0xFF),
    (uint8_t)((delta_us >> 16) & 0xFF), (uint8_t)((delta_us >> 24) & 0xFF)
  };
  
  // Broadcast to all devices
  uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  err = esp_now_send(bcast, payload, sizeof(payload));
  
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "espnow_broadcast_ir_window: failed to broadcast: %s", esp_err_to_name(err));
  } else {
    ESP_LOGI(TAG, "IR window broadcast: seq=%u K=%u delta=%lu us", seq, K, delta_us);
  }
  
  return err;
}

/**
 * @brief Sends ACK1 message from holder to runner
 * 
 * Sends an acknowledgment that the holder has received the runner's IR transmission.
 * This is the first step in the two-phase handshake for granting the potato.
 * 
 * @param runner_mac MAC address of the runner to acknowledge (6 bytes)
 * @param seq Sequence number of the IR window where the transmission occurred
 * @return esp_err_t ESP_OK on success, error code on failure
 * 
 * @note This function initiates the handshake process.
 * @note Automatically ensures the peer is registered before sending.
 */
static esp_err_t espnow_send_ack1_h2r(const uint8_t* runner_mac, uint16_t seq) {
  if (!g_espnow_ready) {
    ESP_LOGE(TAG, "espnow_send_ack1_h2r: ESP-NOW not ready");
    return ESP_ERR_INVALID_STATE;
  }
  
  if (!runner_mac) {
    ESP_LOGE(TAG, "espnow_send_ack1_h2r: invalid runner MAC");
    return ESP_ERR_INVALID_ARG;
  }
  
  esp_err_t err = ensure_peer(runner_mac);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "espnow_send_ack1_h2r: failed to ensure peer: %s", esp_err_to_name(err));
    return err;
  }
  
  uint8_t p[3] = { MSG_ACK1_H2R, (uint8_t)(seq & 0xFF), (uint8_t)(seq >> 8) };
  err = esp_now_send(runner_mac, p, sizeof(p));
  
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "espnow_send_ack1_h2r: failed to send: %s", esp_err_to_name(err));
  } else {
    ESP_LOGI(TAG, "ACK1 sent to %02X:%02X:%02X:%02X:%02X:%02X (seq=%u)", 
              runner_mac[0], runner_mac[1], runner_mac[2], 
              runner_mac[3], runner_mac[4], runner_mac[5], seq);
  }
  
  return err;
}

/**
 * @brief Sends ACK2 message from runner to holder
 * 
 * Sends a confirmation that the runner has received the holder's ACK1.
 * This completes the two-phase handshake and allows the holder to commit the potato grant.
 * 
 * @param holder_mac MAC address of the holder to confirm to (6 bytes)
 * @param seq Sequence number of the IR window where the transmission occurred
 * @return esp_err_t ESP_OK on success, error code on failure
 * 
 * @note This function completes the handshake process.
 * @note Automatically ensures the peer is registered before sending.
 */
static esp_err_t espnow_send_ack2_r2h(const uint8_t* holder_mac, uint16_t seq) {
  if (!g_espnow_ready) {
    ESP_LOGE(TAG, "espnow_send_ack2_r2h: ESP-NOW not ready");
    return ESP_ERR_INVALID_STATE;
  }
  
  if (!holder_mac) {
    ESP_LOGE(TAG, "espnow_send_ack2_r2h: invalid holder MAC");
    return ESP_ERR_INVALID_ARG;
  }
  
  esp_err_t err = ensure_peer(holder_mac);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "espnow_send_ack2_r2h: failed to ensure peer: %s", esp_err_to_name(err));
    return err;
  }
  
  uint8_t p[3] = { MSG_ACK2_R2H, (uint8_t)(seq & 0xFF), (uint8_t)(seq >> 8) };
  err = esp_now_send(holder_mac, p, sizeof(p));
  
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "espnow_send_ack2_r2h: failed to send: %s", esp_err_to_name(err));
  } else {
    ESP_LOGI(TAG, "ACK2 sent to %02X:%02X:%02X:%02X:%02X:%02X (seq=%u)", 
              holder_mac[0], holder_mac[1], holder_mac[2], 
              holder_mac[3], holder_mac[4], holder_mac[5], seq);
    
    // Update liveness for the holder
    zt_note_heard_mac(holder_mac);
  }
  
  return err;
}

/**
 * @brief Commits the potato grant after receiving ACK2 from the winning runner
 * 
 * This function is called when the holder receives ACK2, completing the handshake.
 * It updates the game state, sends final messages, and transitions the holder to runner role.
 * 
 * @param winner_mac MAC address of the winning runner (6 bytes)
 * @param seq Sequence number of the IR window where the grant occurred
 * @return void
 * 
 * @note This function completes the potato passing process.
 * @note Updates distinct-before-repeat state and applies cooldown.
 */
static void holder_commit_after_ack2(const uint8_t* winner_mac, uint16_t seq) {
  if (!winner_mac) {
    ESP_LOGE(TAG, "holder_commit_after_ack2: invalid winner MAC");
    return;
  }
  
  ESP_LOGI(TAG, "Committing potato grant to %02X:%02X:%02X:%02X:%02X:%02X (seq=%u)", 
           winner_mac[0], winner_mac[1], winner_mac[2], 
           winner_mac[3], winner_mac[4], winner_mac[5], seq);
  
  // Update distinct-before-repeat round state
  ESP_LOGI(TAG, "Recording grant for %02X:%02X:%02X:%02X:%02X:%02X in distinct-before-repeat", 
           winner_mac[0], winner_mac[1], winner_mac[2], winner_mac[3], winner_mac[4], winner_mac[5]);
  zt_record_grant(winner_mac);
  
  // Mark self as seen immediately on commit (old holder)
  const int self_idx = zt_roster_index_of(g_self_mac);
  if (self_idx >= 0) {
    zt_distinct_mark_seen(self_idx);
    ESP_LOGI(TAG, "ROUND: self marked seen on commit (idx=%d)", self_idx);
  }
  
  // Track the grant using centralized logic
  if (g_window_target_idx >= 0) {
    zt_note_grant_to_idx(g_window_target_idx);
    zt_on_grant_committed_idx(g_window_target_idx);
  }
  // Clear the per-window target; next window will pick again
  g_window_target_idx = -1;
  memset(g_window_target_mac, 0, sizeof(g_window_target_mac));
  
  // Send final messages to complete the pass
  esp_err_t err = espnow_send_pass_grant(winner_mac, seq);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to send pass grant: %s", esp_err_to_name(err));
    return;
  }
  
  err = espnow_send_state_sync(seq, winner_mac);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to send state sync: %s", esp_err_to_name(err));
    return;
  }
  
  // Update local state
  memcpy(g_last_peer_mac, winner_mac, 6);
  g_granted_seq_valid = true;
  g_granted_seq = seq;
  g_wait_ack2 = false;
  
  // Apply cooldown to prevent rapid passbacks
  g_no_tagback_until_us = esp_timer_get_time() + (int64_t)PASS_COOLDOWN_MS * 1000;
  
  // Transition from holder to runner role
  g_role = ROLE_RUNNER;
  ir_rx_set_enabled(false);
  
  // Update LEDs for RUNNER (off)
  if (led_strip) {
    err = led_strip_clear(led_strip);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to clear LEDs: %s", esp_err_to_name(err));
    } else {
      err = led_strip_refresh(led_strip);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to refresh LEDs: %s", esp_err_to_name(err));
      }
    }
  }
  
  // Update display
  display.fillScreen(TFT_BLACK);
  display.setCursor(0, 60);
  display.printf("ROLE: RUNNER");
  
  ESP_LOGI(TAG, "Role transition: HOLDER -> RUNNER (commit seq=%u)", seq);
}

/**
 * @brief Timer callback for ACK2 timeout
 * 
 * Called when the holder doesn't receive ACK2 from a runner within the timeout period.
 * This indicates a communication failure or the runner is no longer responding.
 * 
 * @param arg Timer argument (unused)
 * @return void
 * 
 * @note This function resets the handshake state to allow the next window to proceed.
 * @note Called by the ESP timer system when ACK2_TIMEOUT_MS expires.
 */
static void ack2_timer_cb(void* arg) {
  (void)arg; // Unused parameter
  
  if (!g_wait_ack2) {
    ESP_LOGD(TAG, "ACK2 timer expired but not waiting for ACK2");
    return;
  }
  
  ESP_LOGW(TAG, "ACK2 timeout (seq=%u from %02X:%02X:%02X:%02X:%02X:%02X)",
           g_wait_ack2_seq,
           g_wait_ack2_mac[0], g_wait_ack2_mac[1], g_wait_ack2_mac[2],
           g_wait_ack2_mac[3], g_wait_ack2_mac[4], g_wait_ack2_mac[5]);
  
  // Note: Collision detection removed - ACK2 timeouts can happen for many reasons
  // and aggressive collision handling was making the game worse
  
  // Reset handshake state to allow next window to proceed
  g_wait_ack2 = false;
  ESP_LOGI(TAG, "Handshake abandoned due to ACK2 timeout, next window will proceed");
}

// Note: All collision handling functions removed - were making the game worse

// ---------- Holder window task ----------
static TaskHandle_t g_window_task = nullptr;
static esp_timer_handle_t g_window_tick = nullptr;

/**
 * @brief Timer callback for IR window ticks
 * 
 * Called periodically by the ESP timer to trigger IR window processing.
 * This function notifies the holder window task to process the next window.
 * 
 * @param arg Timer argument (unused)
 * @return void
 * 
 * @note This function is called every BEACON_PERIOD_MS milliseconds.
 * @note Uses task notification to wake up the holder window task.
 */
static void window_tick_cb(void* arg) {
  (void)arg; // Unused parameter
  
  // Only notify if we're the holder
  if (g_role != ROLE_HOLDER) {
    return;
  }
  
  if (g_window_task) {
    xTaskNotifyGive(g_window_task);
  } else {
    ESP_LOGW(TAG, "Window tick received but window task not created");
  }
}

/**
 * @brief Main task for the holder to manage IR uplink windows
 * 
 * This task runs continuously, waiting for window ticks from the timer.
 * When a tick occurs, it broadcasts an IR window announcement and manages
 * the window state for runners to transmit their IR uplinks.
 * 
 * @param arg Task argument (unused)
 * @return void
 * 
 * @note This task only runs when the device is in HOLDER role.
 * @note Waits for countdown completion and startup delay before processing windows.
 */
static void holder_window_task(void* arg) {
  (void)arg; // Unused parameter
  
  ESP_LOGI(TAG, "Holder window task started");
  
  for (;;) {
    // Only run holder logic if we're actually the holder
    if (g_role != ROLE_HOLDER) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    
    // Wait for window tick notification from timer
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    // Only process windows when in holder role
    if (g_role != ROLE_HOLDER) {
      ESP_LOGD(TAG, "Window tick received but not in holder role, skipping");
      continue;
    }
    
    // Wait for countdown to complete
    if (!g_game_started) {
      ESP_LOGD(TAG, "Window tick received but game not started, skipping");
      continue;
    }
    
    // Check startup delay
    int64_t now = esp_timer_get_time();
    if (now < g_holder_startup_until_us) {
      ESP_LOGD(TAG, "Window tick received but startup delay not expired, skipping");
      continue;
    }

    // Increment window sequence and reset state
    g_window_seq++;
    g_granted_seq_valid = false;
    g_empty_windows++;  // Track empty windows for monitoring (no longer used for round relaxation)
    
    // Adaptive strike tracking: if no uplink was accepted this window, increment strikes for current target
    if (g_window_target_idx >= 0) {
      if (g_unresp_strikes[g_window_target_idx] < 255) {
        ++g_unresp_strikes[g_window_target_idx];
      }
      
      if (g_unresp_strikes[g_window_target_idx] >= MAX_UNRESP_STRIKES) {
        // Rotate fair pointer past this peer (still unseen)
        g_last_granted_idx = g_window_target_idx;
        ESP_LOGW(TAG, "FAIR: rotating past idx=%d after %u empty windows",
                g_window_target_idx, g_unresp_strikes[g_window_target_idx]);
        
        // Reset strike counter for the device we just rotated past
        g_unresp_strikes[g_window_target_idx] = 0;
        
        // Pick a new unseen+alive peer
        int next_idx = -1; 
        uint8_t next_mac[6];
        if (pick_next_unseen_alive(&next_idx, next_mac)) {
          // Update target for the next window
          g_window_target_idx = next_idx;
          memcpy(g_window_target_mac, next_mac, 6);
          ESP_LOGI(TAG, "FAIR: new target idx=%d %02X:%02X:%02X:%02X:%02X:%02X",
                   g_window_target_idx,
                   g_window_target_mac[0], g_window_target_mac[1], g_window_target_mac[2], 
                   g_window_target_mac[3], g_window_target_mac[4], g_window_target_mac[5]);
        } else {
          ESP_LOGW(TAG, "FAIR: no unseen+alive candidates; waiting (liveness may drop a peer)");
          g_window_target_idx = -1;
          memset(g_window_target_mac, 0, sizeof(g_window_target_mac));
        }
      }
    }
    
    // Dynamic K calculation to fit slots within period
    const uint32_t period_us = (uint32_t)BEACON_PERIOD_MS * 1000u;
    const uint32_t guard_us = 5000;  // Room for ACKs and timing
    uint32_t maxK = (period_us > guard_us) ? (period_us - guard_us) / UPLINK_SLOT_US : 1;
    uint8_t K = (uint8_t)((UPLINK_K < maxK) ? UPLINK_K : maxK);
    if (K == 0) K = 1;
    
    // Note: Adaptive slot width removed - was making the game worse
    // Keep fixed K for consistent, predictable behavior
    
    // Broadcast IR window announcement
    const uint32_t delta_us = 7000; // Window T0 = now + 7ms (reduced from 10ms)
    esp_err_t err = espnow_broadcast_ir_window(g_window_seq, K, delta_us);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to broadcast IR window: %s", esp_err_to_name(err));
      continue;
    }
    
    // Choose who we're willing to ACK1 in this window, using adaptive fair sweep
    g_window_target_idx = -1;
    if (pick_next_unseen_alive(&g_window_target_idx, g_window_target_mac)) {
      ESP_LOGI(TAG, "IR_WINDOW: fair target idx=%d %02X:%02X:%02X:%02X:%02X:%02X",
               g_window_target_idx,
               g_window_target_mac[0], g_window_target_mac[1], g_window_target_mac[2], 
               g_window_target_mac[3], g_window_target_mac[4], g_window_target_mac[5]);
    } else {
      g_window_target_idx = -1;
      memset(g_window_target_mac, 0, sizeof(g_window_target_mac)); // no target
      ESP_LOGI(TAG, "IR_WINDOW: no fair target available this window");
    }
    
    ESP_LOGI(TAG, "IR_WINDOW seq=%u K=%u/%lu T0=now+%lu us window_len=%lu us empty_count=%lu", 
             g_window_seq, K, (unsigned long)UPLINK_K, delta_us, (unsigned long)(K * UPLINK_SLOT_US), g_empty_windows);
    
    // Note: Round relaxation removed - distinct-before-repeat will always be enforced
    // The system must work through all devices before allowing repeats, regardless of IR collisions
    
    // Note: Periodic collision history reset removed - no longer needed
  }
}

/**
 * @brief UART RX task for receiving IR uplinks from runners
 * 
 * This task continuously reads from UART2 to receive IR signals demodulated to UART format.
 * It implements a state machine to parse IR packets and process valid transmissions.
 * Only runs when the device is in HOLDER role and RX is enabled.
 * 
 * @param arg Task argument (unused)
 * @return void
 * 
 * @note Implements UART packet parsing with CRC verification.
 * @note Handles both ACK2 handshake and immediate grant modes.
 * @note Applies cooldown and distinct-before-repeat logic.
 */
static void uart_rx_task(void* arg) {
  (void)arg; // Unused parameter
  
  ESP_LOGI(TAG, "UART RX task started for IR uplink reception");
  
  // UART packet parsing state machine
  enum { WAIT_Z, WAIT_T, READ_LEN, READ_PAYLOAD, READ_CRC } st = WAIT_Z;
  uint8_t buf[64], payload[IR_MAX_PAYLOAD];
  uint8_t len = 0;
  size_t pi = 0;
  
  for (;;) {
    // Only process when RX is enabled (holder role)
    if (!g_rx_enabled) {
      vTaskDelay(pdMS_TO_TICKS(UART_TIMEOUT_MS));
      continue;
    }
    
    // Read UART data
    int n = uart_read_bytes(IR_UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(UART_TIMEOUT_MS));
    if (n <= 0) continue;
    
    // Process each received byte
    for (int i = 0; i < n; ++i) {
      uint8_t b = buf[i];
      
      switch (st) {
        case WAIT_Z:
          if (b == IR_PREAMBLE0) {
            st = WAIT_T;
          }
          break;
          
        case WAIT_T:
          if (b == IR_PREAMBLE1) {
            st = READ_LEN;
          } else {
            st = WAIT_Z;
          }
          break;
          
        case READ_LEN:
          len = b;
          if (len == 0 || len > IR_MAX_PAYLOAD) {
            st = WAIT_Z;
            break;
          }
          pi = 0;
          st = READ_PAYLOAD;
          break;
          
        case READ_PAYLOAD:
          payload[pi++] = b;
          if (pi >= len) {
            st = READ_CRC;
          }
          break;
          
        case READ_CRC: {
          // Verify CRC
          uint8_t tmp[1 + IR_MAX_PAYLOAD];
          tmp[0] = len;
          memcpy(tmp + 1, payload, len);
          
          if (crc8_maxim(tmp, 1 + len) == b) {
            bool has_seq = (len == (IR_MAC_LEN + 2));
            uint16_t seq = has_seq ? (uint16_t)payload[0] | ((uint16_t)payload[1] << 8) : 0;
            const uint8_t* src = has_seq ? (payload + 2) : payload;
            
            if (g_role == ROLE_HOLDER && has_seq) {
              // Accept only once per sequence
              if (g_granted_seq_valid && g_granted_seq == seq) {
                ESP_LOGD(TAG, "IR drop: duplicate sequence %u", seq);
                st = WAIT_Z;
                break;
              }
              
              // Check cooldown and sequence gating
              int64_t now = esp_timer_get_time();
              if (now < g_no_tagback_until_us) {
                ESP_LOGD(TAG, "IR drop: cooldown remaining %lld us", (long long)(g_no_tagback_until_us - now));
                st = WAIT_Z;
                break;
              }
              
              if (seq != g_window_seq) {
                ESP_LOGD(TAG, "IR drop: sequence mismatch got=%u want=%u", seq, g_window_seq);
                st = WAIT_Z;
                break;
              }  // Must match current window
              
              // Fair target selection: only ACK the pre-selected peer for this window
              if (g_window_target_idx < 0 || memcmp(g_window_target_mac, src, 6) != 0) {
                ESP_LOGD(TAG, "HOLDER drop IR: not window target %02X:%02X:%02X:%02X:%02X:%02X (target_idx=%d)",
                         src[0], src[1], src[2], src[3], src[4], src[5], g_window_target_idx);
                st = WAIT_Z;
                break;
              }
              
              // Distinct-before-repeat: only accept not-yet-seen candidates in this round
              if (!zt_is_candidate_allowed(src)) {
                ESP_LOGD(TAG, "IR drop: not distinct %02X:%02X:%02X - distinct-before-repeat active", 
                         src[3], src[4], src[5]);
                st = WAIT_Z;
                break;
              }
              
              // IR received successfully - reset empty counter
              g_empty_windows = 0;
              
              // Update liveness for the sender
              zt_note_heard_mac(src);
              
#if MODE_B_USE_ACK2
              // Begin handshake: ACK1 → wait for ACK2 → then commit
              memcpy(g_wait_ack2_mac, src, 6);
              g_wait_ack2_seq = seq;
              g_wait_ack2 = true;
              
              if (!g_ack2_timer) {
                const esp_timer_create_args_t args = { 
                  .callback = &ack2_timer_cb, 
                  .arg = nullptr, 
                  .name = "ack2_to"
                };
                esp_err_t err = esp_timer_create(&args, &g_ack2_timer);
                if (err != ESP_OK) {
                  ESP_LOGE(TAG, "Failed to create ACK2 timer: %s", esp_err_to_name(err));
                  st = WAIT_Z;
                  break;
                }
              }
              
              esp_timer_stop(g_ack2_timer);
              esp_err_t err = esp_timer_start_once(g_ack2_timer, (uint64_t)ACK2_TIMEOUT_MS * 1000ULL);
              if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to start ACK2 timer: %s", esp_err_to_name(err));
                st = WAIT_Z;
                break;
              }
              
              err = espnow_send_ack1_h2r(src, seq);
              if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send ACK1: %s", esp_err_to_name(err));
                st = WAIT_Z;
                break;
              }
              
              ESP_LOGI(TAG, "ACK1 sent to %02X:%02X:%02X:%02X:%02X:%02X (seq=%u)",
                       src[0], src[1], src[2], src[3], src[4], src[5], seq);
              
              // Update liveness for the runner
              zt_note_heard_mac(src);
              
              // Note: Collision history reset removed - no longer needed
#else
              // Immediate grant (faster but less robust)
              esp_err_t err = espnow_send_pass_grant(src, seq);
              if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send pass grant: %s", esp_err_to_name(err));
                st = WAIT_Z;
                break;
              }
              
              err = espnow_send_state_sync(seq, src);
              if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send state sync: %s", esp_err_to_name(err));
                st = WAIT_Z;
                break;
              }
              
              memcpy(g_last_peer_mac, src, 6);
              g_granted_seq_valid = true;
              g_granted_seq = seq;
              g_no_tagback_until_us = esp_timer_get_time() + (int64_t)PASS_COOLDOWN_MS * 1000;
              g_role = ROLE_RUNNER;
              ir_rx_set_enabled(false);
              
              // Update distinct-before-repeat round state
              zt_record_grant(src);
              
              // Update LEDs for RUNNER (off)
              if (led_strip) {
                err = led_strip_clear(led_strip);
                if (err != ESP_OK) {
                  ESP_LOGE(TAG, "Failed to clear LEDs: %s", esp_err_to_name(err));
                } else {
                  err = led_strip_refresh(led_strip);
                  if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to refresh LEDs: %s", esp_err_to_name(err));
                  }
                }
              }
              
              // Update display
              display.fillScreen(TFT_BLACK);
              display.setCursor(0, 60);
              display.printf("ROLE: RUNNER");
              
              ESP_LOGI(TAG,"Immediate GRANT to %02X:%02X:%02X:%02X:%02X:%02X (seq=%u)",
                       src[0],src[1],src[2],src[3],src[4],src[5], (unsigned)seq);
              
              // Note: Collision history reset removed - no longer needed
#endif
            }
          }
          st=WAIT_Z; break;
        }
      }
    }
  }
}

/**
 * @brief Main entry point for Mode B IR Uplink Test
 * 
 * Initializes the Mode B game mode where holders receive IR uplinks from runners.
 * Sets up hardware, joins the game, assigns roles, and starts the game loop.
 * 
 * @return void
 * 
 * @note This function initializes all hardware and starts the game.
 * @note Uses the test harness for device discovery and role assignment.
 */
extern "C" void ir_uplink_test_main(void) {
  ESP_LOGI(TAG, "=== MODE B: IR UPLINK TEST (holder RX / runners IR TX) ===");
  ESP_LOGI(TAG, "Starting IR Uplink Test (Mode B)");
  
  // Read this device's MAC address
  esp_err_t err = esp_read_mac(g_self_mac, ESP_MAC_WIFI_STA);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read MAC address: %s", esp_err_to_name(err));
    return;
  }

  // Initialize LED strip if not already done
  if (led_strip == nullptr) {
    led_strip_config_t strip_config = {
      .strip_gpio_num = HP_LED_GPIO,
      .max_leds = HP_LED_COUNT
    };
    
    led_strip_rmt_config_t rmt_config = {
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = 10 * 1000 * 1000, // 10MHz
      .mem_block_symbols = 64,
      .flags = {}
    };
    
    err = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to create LED strip: %s", esp_err_to_name(err));
      return;
    }
  }
  
  // Aggressive clear for WS2812 reset (multiple times to ensure they're off)
  if (led_strip) {
    err = led_strip_clear(led_strip);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to clear LEDs: %s", esp_err_to_name(err));
    } else {
      err = led_strip_refresh(led_strip);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to refresh LEDs: %s", esp_err_to_name(err));
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(10)); // Small delay for WS2812 to process
    
    err = led_strip_clear(led_strip);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to clear LEDs (second time): %s", esp_err_to_name(err));
    } else {
      err = led_strip_refresh(led_strip);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to refresh LEDs (second time): %s", esp_err_to_name(err));
      }
    }
    
    ESP_LOGI(TAG, "M5Stack FIRE side LEDs reset - should be OFF now");
  }

  // Display (minimal) - will be updated after role assignment
  display.begin();
  display.setTextSize(2);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.fillScreen(TFT_BLACK);
  display.setCursor(0, 20);
  display.printf("MODE B");
  display.setCursor(0, 60);
  display.printf("ROLE: %s", ZT_IS_HOST ? "HOST" : "PEER");

  // Hardware initialization
  ESP_LOGI(TAG, "Initializing hardware components");
  setup_rmt_tx();
  setup_uart_rx();
  
  err = espnow_init_once();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize ESP-NOW: %s", esp_err_to_name(err));
    return;
  }
  
  // 1) Join + countdown → get initial role & roster
  zt_role_t init_role;
  
  // LCD countdown hook so all devices show the same timer
  zt_set_countdown_cb(zt_render_countdown);
  
  zt_join_and_countdown(&init_role, nullptr, ZT_IS_HOST == 1); // HOST if ZT_IS_HOST == 1

  // 2) Apply role to Mode B (uplink mode): holder RX, runners TX
  g_role = (init_role == ZT_ROLE_HOLDER) ? ROLE_HOLDER : ROLE_RUNNER;
  ir_rx_set_enabled(g_role == ROLE_HOLDER);
  
  ESP_LOGI(TAG, "Build config: ZT_IS_HOST=%d", ZT_IS_HOST);
  ESP_LOGI(TAG, "Test harness: initial role = %s", (init_role == ZT_ROLE_HOLDER) ? "HOLDER" : "RUNNER");
  ESP_LOGI(TAG, "Test harness: using global roster from test harness");

  // Harness installed its own recv cb. Restore Mode-B's handler for gameplay:
  ir_uplink_install_recv_cb();

  // Distinct-before-repeat: initialize round with the initial holder
  uint8_t init_holder[6]; zt_get_initial_holder(init_holder);
  ESP_LOGI(TAG, "Distinct-before-repeat: initializing with holder %02X:%02X:%02X:%02X:%02X:%02X",
           init_holder[0], init_holder[1], init_holder[2], init_holder[3], init_holder[4], init_holder[5]);
  zt_distinct_init(nullptr, init_holder);  // Use global roster from test harness
  ESP_LOGI(TAG, "Distinct-before-repeat: initialization complete");

  // Update display with actual role
  display.fillScreen(TFT_BLACK);
  display.setCursor(0, 20);
  display.printf("MODE B");
  display.setCursor(0, 60);
  display.printf("ROLE: %s", (g_role == ROLE_HOLDER) ? "HOLDER" : "RUNNER");
  display.setCursor(0, 100);
  display.printf("BUILD: %s", ZT_IS_HOST ? "HOST" : "PEER");
  ESP_LOGI(TAG, "Display updated: ROLE = %s, BUILD = %s", (g_role == ROLE_HOLDER) ? "HOLDER" : "RUNNER", ZT_IS_HOST ? "HOST" : "PEER");

  // LED already initialized at startup

  // Role-driven RX gating (Mode B: holder listens)
  ir_rx_set_enabled(g_role == ROLE_HOLDER);

  // Handshake init
  g_wait_ack1 = false; g_wait_ack1_seq = 0;
  g_wait_ack2 = false; g_wait_ack2_seq = 0;

  // Initial LED state based on role
  if (led_strip) {
    if (g_role == ROLE_HOLDER) {
      // HOLDER: red LEDs
      for (int i = 0; i < LED_COUNT; i++) {
        err = led_strip_set_pixel(led_strip, i, LED_COLOR_RED, LED_COLOR_GREEN, LED_COLOR_BLUE);
        if (err != ESP_OK) {
          ESP_LOGE(TAG, "Failed to set LED %d: %s", i, esp_err_to_name(err));
        }
      }
      
      err = led_strip_refresh(led_strip);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to refresh LEDs: %s", esp_err_to_name(err));
      } else {
        ESP_LOGI(TAG, "LED: ON (HOLDER) - all %d side LEDs red", LED_COUNT);
      }
    } else {
      // RUNNER: LEDs off (already cleared above)
      ESP_LOGI(TAG, "LED: OFF (RUNNER)");
    }
  }

  // Task and timer creation
  ESP_LOGI(TAG, "Creating game tasks and timers");
  
  // Holder periodic window ticker (~ every BEACON_PERIOD_MS, matching Mode A cadence)
  if (g_window_task == nullptr) {
    BaseType_t task_created = xTaskCreate(holder_window_task, "holder_window_task", 
                                         TASK_STACK_SIZE, nullptr, TASK_PRIORITY_MEDIUM, &g_window_task);
    if (task_created != pdPASS) {
      ESP_LOGE(TAG, "Failed to create holder window task");
      return;
    }
    
    const esp_timer_create_args_t a = { 
      .callback = &window_tick_cb, 
      .arg = nullptr, 
      .name = "window_tick"
    };
    
    err = esp_timer_create(&a, &g_window_tick);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to create window tick timer: %s", esp_err_to_name(err));
      return;
    }
    
    // Start the timer but the task will wait for countdown
    err = esp_timer_start_periodic(g_window_tick, (uint64_t)BEACON_PERIOD_MS * 1000ULL);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to start window tick timer: %s", esp_err_to_name(err));
      return;
    }
    
    ESP_LOGI(TAG, "Holder window task and timer created successfully");
  }
  
  // UART RX task (decodes runner IR when we are holder)
  BaseType_t rx_task_created = xTaskCreate(uart_rx_task, "uart_rx_task", 
                                          TASK_STACK_SIZE, nullptr, TASK_PRIORITY_HIGH, nullptr);
  if (rx_task_created != pdPASS) {
    ESP_LOGE(TAG, "Failed to create UART RX task");
    return;
  }
  
  ESP_LOGI(TAG, "UART RX task created successfully");
  
  // Wait for countdown to complete before starting game
  ESP_LOGI(TAG, "Waiting for countdown to complete...");
  // The countdown completion is now handled by zt_test_mode.cpp
  // g_game_started will be set to true when the countdown actually completes
}

/**
 * @brief Renders the synchronized countdown display
 * 
 * Displays a countdown timer that all devices show simultaneously.
 * Shows the current countdown value and a hint about the initial holder.
 * 
 * @param secs Current countdown value in seconds
 * @param initial_holder MAC address of the initial holder (6 bytes)
 * @param roster Device roster (unused but required by callback signature)
 * @return void
 * 
 * @note This function is called by the test harness during countdown.
 * @note Uses centered text layout for a clean appearance.
 */
static void zt_render_countdown(int secs, const uint8_t initial_holder[6], const zt_roster_t* roster) {
  (void)roster; // Unused parameter
  
  // Minimalist centered banner
  display.fillScreen(TFT_BLACK);
  display.setTextDatum(textdatum_t::middle_center);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.setTextSize(3);
  display.drawString("SYNC START", display.width()/2, display.height()/2 - 28);
  display.setTextSize(5);
  char buf[16]; snprintf(buf, sizeof(buf), "%d", secs);
  display.drawString(buf, display.width()/2, display.height()/2 + 8);
  // Small footer: initial holder hint
  display.setTextSize(2);
  char macs[32];
  snprintf(macs, sizeof(macs), "%02X:%02X:%02X", initial_holder[3], initial_holder[4], initial_holder[5]);
  display.drawString(macs, display.width()/2, display.height()/2 + 48);
}

// ESPNOW receive callback for Mode B
static void ir_uplink_recv_cb(const esp_now_recv_info* info, const uint8_t* data, int len){
  if (len<=0 || !data) return;
  switch (data[0]) {
    case MSG_IR_WINDOW: {
      if (g_role != ROLE_RUNNER) break;
      if (!g_game_started) break;  // Wait for countdown to complete
      if (len < 14) break;
      uint16_t seq = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
      uint8_t holder[6]; memcpy(holder, &data[3], 6);
      uint8_t K = data[9];
      uint32_t delta_us = (uint32_t)data[10] | ((uint32_t)data[11]<<8) | ((uint32_t)data[12]<<16) | ((uint32_t)data[13]<<24);
      
      ESP_LOGI(TAG, "IR_WINDOW received: seq=%u holder=%02X:%02X:%02X K=%u delta=%lu us", 
               seq, holder[3], holder[4], holder[5], K, delta_us);

      // Early self-gate: prevent already-seen runners from transmitting
      const int self_idx = zt_roster_index_of(g_self_mac);
      if (self_idx >= 0 && zt_distinct_already_seen(self_idx)) {
        ESP_LOGI(TAG, "RUNNER: already seen this round; skipping window");
        break;
      }

      // Cooldown check: never block unseen runners
      int64_t now = esp_timer_get_time();
      if (now < g_no_tagback_until_us) { 
        ESP_LOGI(TAG,"cooldown; skip window"); 
        break; 
      }

      // Skip if we've already held this round (distinct-before-repeat, local view)
      if (!zt_is_candidate_allowed(g_self_mac)) { 
        ESP_LOGD(TAG,"ineligible this round; skip window - distinct-before-repeat active"); 
        break;
      }

      // Pick slot deterministically and schedule IR uplink
      slot_pick_t sp = pick_slot(seq, holder, g_self_mac, K ? K : UPLINK_K);
      uint64_t delay = (uint64_t)delta_us + (uint64_t)sp.slot * UPLINK_SLOT_US + sp.jitter_us;
      esp_timer_handle_t t = nullptr;
      const esp_timer_create_args_t a = { 
        .callback = [](void* p){
          uint16_t s = (uint16_t)(uintptr_t)p;
          runner_send_ir_uplink(s);
        }, 
        .arg = (void*)(uintptr_t)seq, 
        .name="ir_uplink_tx" 
      };
      ESP_ERROR_CHECK(esp_timer_create(&a, &t));
      ESP_ERROR_CHECK(esp_timer_start_once(t, delay));
      ESP_LOGI("IRBK","seq=%u slot=%u/%u jit=%uus send_in=%lluus", (unsigned)seq, sp.slot, K?K:UPLINK_K, sp.jitter_us, (unsigned long long)delay);
      break;
    }
    case MSG_PASS_GRANT: {
      if (g_role != ROLE_RUNNER) break;
      if (len < 10) break;
      uint16_t seq = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
      uint8_t new_holder[6]; memcpy(new_holder, &data[3], 6);
      if (memcmp(new_holder, g_self_mac, 6) == 0) {
                   // we won → become HOLDER
         g_role = ROLE_HOLDER;
         ir_rx_set_enabled(true);                   // holder listens in Mode B
         g_holder_startup_until_us = esp_timer_get_time() + (int64_t)HOLDER_STARTUP_DELAY_MS * 1000;
         g_granted_seq_valid = false;
         
         // Mark self as seen immediately on grant reception (new holder)
         const int self_idx = zt_roster_index_of(g_self_mac);
         if (self_idx >= 0) {
           zt_distinct_mark_seen(self_idx);
           ESP_LOGI(TAG, "ROUND: self marked seen on grant reception (idx=%d)", self_idx);
         }
         
         // Update LEDs for HOLDER
         if (led_strip) {
           ESP_ERROR_CHECK(led_strip_clear(led_strip));
           ESP_ERROR_CHECK(led_strip_refresh(led_strip));
           for (int i = 0; i < HP_LED_COUNT; i++) {
             ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 255, 0, 0)); // Red
           }
           ESP_ERROR_CHECK(led_strip_refresh(led_strip));
         }
         
         // Update display
         display.fillScreen(TFT_BLACK);
         display.setCursor(0, 60);
         display.printf("ROLE: HOLDER");
         
         ESP_LOGI("CORE","ROLE -> HOLDER (GRANT seq=%u)", (unsigned)seq);
      }
      break;
    }
    case MSG_ACK1_H2R: {
      // Runner got ACK1 → send ACK2 back to holder
      if (g_role != ROLE_RUNNER) break;
      if (len < 3) break;
      uint16_t seq = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
      if (!g_wait_ack1 || g_wait_ack1_seq != seq) break; // not ours
      espnow_send_ack2_r2h(info->src_addr, seq);
      g_wait_ack1 = false;
      ESP_LOGI(TAG,"ACK1 rx → ACK2 tx (seq=%u)", (unsigned)seq);
      break;
    }
    case MSG_ACK2_R2H: {
      // Holder got ACK2 → commit pass
      if (g_role != ROLE_HOLDER) break;
      if (len < 3) break;
      uint16_t seq = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
      if (!g_wait_ack2 || seq != g_wait_ack2_seq) break;
      if (memcmp(info->src_addr, g_wait_ack2_mac, 6) != 0) break;
      if (g_ack2_timer) esp_timer_stop(g_ack2_timer);
      holder_commit_after_ack2(g_wait_ack2_mac, seq);
      break;
    }
         case MSG_STATE_SYNC: {
       if (len < 12) break;
       uint8_t new_holder[6]; memcpy(new_holder, &data[3], 6);
       ESP_LOGI(TAG, "STATE_SYNC: recording grant for %02X:%02X:%02X:%02X:%02X:%02X in distinct-before-repeat", 
                new_holder[0], new_holder[1], new_holder[2], new_holder[3], new_holder[4], new_holder[5]);
       zt_record_grant(new_holder);  // keep local round state in sync
       break;
     }
    default: break;
  }
}

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
#include "ir_simple_test.h"  // reuse IR constants/macros (SLOTS, SLOT_US, etc.)
#include "zt_test_mode.h"    // for g_T0_us

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
#define IR_TX_GPIO   GPIO_NUM_26
#define IR_RX_GPIO   GPIO_NUM_36  // Reverted back - GPIO36 is fine for IR RX
#define IR_UART_PORT UART_NUM_2

static const char* TAG = "IR_UPLINK_TEST";

// ---- Role state ----
typedef enum { ROLE_HOLDER, ROLE_RUNNER } role_t;
static volatile role_t g_role = ZT_IS_HOST ? ROLE_HOLDER : ROLE_RUNNER;
static uint8_t  g_self_mac[6] = {0};
static uint8_t  g_last_peer_mac[6] = {0};
static int64_t  g_no_tagback_until_us = 0;
static int64_t  g_holder_startup_until_us = 0;
static bool     g_game_started = false;  // Track when countdown is complete

// ---- RMT TX (IR) ----
static rmt_channel_handle_t rmt_tx_chan = nullptr;
static rmt_encoder_handle_t tx_copy_encoder = nullptr;

// ---- UART RX (IR demod as 2400-8N1) ----
static bool g_rx_enabled = true;
void ir_rx_set_enabled(bool en) { g_rx_enabled = en; }

// ---- ESP-NOW infra ----
static bool g_espnow_ready = false;

// ---- Window & slotting ----
// We reuse SLOTS/SLOT_US/SLOT_JITTER_MAX_US from ir_simple_test.h
// For IR uplink, slot length should be ~frame-time+guard ≈ 45ms @2400bps.
#ifndef UPLINK_SLOT_US
#define UPLINK_SLOT_US 45000
#endif
#ifndef UPLINK_K
#define UPLINK_K SLOTS    // default to same macro
#endif

// Current contention seq issued by the holder:
static uint16_t g_window_seq = 0;
static bool     g_granted_seq_valid = false;
static uint16_t g_granted_seq = 0;
static uint32_t g_empty_windows = 0;  // Count consecutive windows with no IR

// Handshake state (holder waits for ACK2 from this runner/seq)
static bool     g_wait_ack2 = false;
static uint8_t  g_wait_ack2_mac[6] = {0};
static uint16_t g_wait_ack2_seq = 0;
static esp_timer_handle_t g_ack2_timer = nullptr;

// Runner side: track IR we just sent (waiting for ACK1)
static bool     g_wait_ack1 = false;
static uint16_t g_wait_ack1_seq = 0;

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
  slot_pick_t r;
  r.slot = (uint8_t)(h16 % (K?K:1));
  r.jitter_us = (uint16_t)(h8 % SLOT_JITTER_MAX_US);
  return r;
}

// ---------- RMT TX setup ----------
static void setup_rmt_tx() {
  rmt_tx_channel_config_t txc = {
    .gpio_num = IR_TX_GPIO,
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .resolution_hz = 1000000, // 1us
    .mem_block_symbols = 128,
    .trans_queue_depth = 4,
    .intr_priority = 0,
    .flags = {0}
  };
  ESP_ERROR_CHECK(rmt_new_tx_channel(&txc, &rmt_tx_chan));
  rmt_carrier_config_t carrier = { .frequency_hz = 38000, .duty_cycle = 0.33, .flags = { .polarity_active_low = 0 } };
  ESP_ERROR_CHECK(rmt_apply_carrier(rmt_tx_chan, &carrier));
  rmt_copy_encoder_config_t cc = {};
  ESP_ERROR_CHECK(rmt_new_copy_encoder(&cc, &tx_copy_encoder));
  ESP_ERROR_CHECK(rmt_enable(rmt_tx_chan));
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

  rmt_transmit_config_t cfg = { .loop_count = 0, .flags = {0} };
  ESP_ERROR_CHECK(rmt_transmit(rmt_tx_chan, tx_copy_encoder, syms, n*sizeof(syms[0]), &cfg));
  ESP_ERROR_CHECK(rmt_tx_wait_all_done(rmt_tx_chan, pdMS_TO_TICKS(1000)));
  ESP_LOGI(TAG, "IR uplink sent (SEQ=%u)", (unsigned)seq);
  // Now expect ACK1 for this seq
  g_wait_ack1 = true;
  g_wait_ack1_seq = seq;
}

// ---------- UART RX setup (holder only listens in Mode B) ----------
static void setup_uart_rx() {
  uart_config_t cfg = {
    .baud_rate = IR_BAUD, .data_bits = UART_DATA_8_BITS, .parity=UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT
  };
  ESP_ERROR_CHECK(uart_param_config(IR_UART_PORT, &cfg));
  ESP_ERROR_CHECK(uart_driver_install(IR_UART_PORT, IR_UART_RX_BUF_SZ, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_set_pin(IR_UART_PORT, UART_PIN_NO_CHANGE, IR_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_LOGI(TAG, "UART RX ready on GPIO%d @ %d-8N1", (int)IR_RX_GPIO, IR_BAUD);
}

// ---------- ESP-NOW ----------
static void ensure_nvs() {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  } else {
    ESP_ERROR_CHECK(err);
  }
}

static esp_err_t espnow_init_once(void) {
  if (g_espnow_ready) return ESP_OK;
  ensure_nvs();
  esp_netif_init();
  esp_event_loop_create_default();
  esp_netif_create_default_wifi_sta();
  wifi_init_config_t w = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&w));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

  ESP_ERROR_CHECK(esp_now_init());
  
  // Add broadcast peer for IR window announcements
  uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_peer_info_t bcast_peer = {};
  memcpy(bcast_peer.peer_addr, bcast, 6);
  bcast_peer.ifidx = WIFI_IF_STA;
  bcast_peer.channel = ESPNOW_CHANNEL;
  bcast_peer.encrypt = false;
  ESP_ERROR_CHECK(esp_now_add_peer(&bcast_peer));
  
  ESP_ERROR_CHECK(esp_now_register_send_cb([](const uint8_t* mac, esp_now_send_status_t st){
    ESP_LOGD("ESPNOW","tx %02X:%02X:%02X:%02X:%02X:%02X -> %s",
      mac[0],mac[1],mac[2],mac[3],mac[4],mac[5], st==ESP_NOW_SEND_SUCCESS?"OK":"FAIL");
  }));
  ESP_ERROR_CHECK(esp_now_register_recv_cb(ir_uplink_recv_cb));
  g_espnow_ready = true;
  return ESP_OK;
}

// Reinstall Mode-B ESPNOW recv callback after test harness
extern "C" void ir_uplink_install_recv_cb(void) {
  ESP_ERROR_CHECK(esp_now_register_recv_cb(ir_uplink_recv_cb));
}

static esp_err_t ensure_peer(const uint8_t* mac) {
  esp_now_peer_info_t p = {}; memcpy(p.peer_addr, mac, 6);
  p.ifidx = WIFI_IF_STA; p.channel = ESPNOW_CHANNEL; p.encrypt = false;
  esp_err_t err = esp_now_add_peer(&p);
  return (err == ESP_ERR_ESPNOW_EXIST) ? ESP_OK : err;
}

static esp_err_t espnow_send_pass_grant(const uint8_t* winner_mac, uint16_t seq) {
  if (!g_espnow_ready) return ESP_ERR_INVALID_STATE;
  ESP_ERROR_CHECK(ensure_peer(winner_mac));
  uint8_t payload[10] = { MSG_PASS_GRANT,
    (uint8_t)(seq & 0xFF), (uint8_t)(seq >> 8),
    winner_mac[0],winner_mac[1],winner_mac[2],winner_mac[3],winner_mac[4],winner_mac[5] };
  return esp_now_send(winner_mac, payload, sizeof(payload));
}

static esp_err_t espnow_send_state_sync(uint16_t seq, const uint8_t* new_holder_mac) {
  if (!g_espnow_ready) return ESP_ERR_INVALID_STATE;
  uint8_t payload[12] = { MSG_STATE_SYNC,
    (uint8_t)(seq & 0xFF), (uint8_t)(seq >> 8),
    new_holder_mac[0],new_holder_mac[1],new_holder_mac[2],
    new_holder_mac[3],new_holder_mac[4],new_holder_mac[5],
    (uint8_t)(PASS_COOLDOWN_MS & 0xFF), (uint8_t)((PASS_COOLDOWN_MS>>8)&0xFF) };
  uint8_t bcast[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  return esp_now_send(bcast, payload, sizeof(payload));
}

static esp_err_t espnow_broadcast_ir_window(uint16_t seq, uint8_t K, uint32_t delta_us) {
  uint8_t holder[6]; esp_read_mac(holder, ESP_MAC_WIFI_STA);
  uint8_t payload[14] = { MSG_IR_WINDOW,
    (uint8_t)(seq & 0xFF), (uint8_t)(seq >> 8),
    holder[0],holder[1],holder[2],holder[3],holder[4],holder[5],
    K,
    (uint8_t)(delta_us & 0xFF), (uint8_t)((delta_us>>8)&0xFF),
    (uint8_t)((delta_us>>16)&0xFF),(uint8_t)((delta_us>>24)&0xFF)
  };
  uint8_t bcast[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  return esp_now_send(bcast, payload, sizeof(payload));
}

static esp_err_t espnow_send_ack1_h2r(const uint8_t* runner_mac, uint16_t seq) {
  if (!g_espnow_ready) return ESP_ERR_INVALID_STATE;
  ESP_ERROR_CHECK(ensure_peer(runner_mac));
  uint8_t p[3] = { MSG_ACK1_H2R, (uint8_t)(seq & 0xFF), (uint8_t)(seq >> 8) };
  return esp_now_send(runner_mac, p, sizeof(p));
}

static esp_err_t espnow_send_ack2_r2h(const uint8_t* holder_mac, uint16_t seq) {
  if (!g_espnow_ready) return ESP_ERR_INVALID_STATE;
  ESP_ERROR_CHECK(ensure_peer(holder_mac));
  uint8_t p[3] = { MSG_ACK2_R2H, (uint8_t)(seq & 0xFF), (uint8_t)(seq >> 8) };
  return esp_now_send(holder_mac, p, sizeof(p));
}

static void holder_commit_after_ack2(const uint8_t* winner_mac, uint16_t seq) {
  // Update distinct-before-repeat round state
  ESP_LOGI(TAG, "Recording grant for %02X:%02X:%02X:%02X:%02X:%02X in distinct-before-repeat", 
           winner_mac[0], winner_mac[1], winner_mac[2], winner_mac[3], winner_mac[4], winner_mac[5]);
  zt_record_grant(winner_mac);
  
  // Commit: GRANT + STATE_SYNC, cooldown, flip role
  ESP_ERROR_CHECK(espnow_send_pass_grant(winner_mac, seq));
  ESP_ERROR_CHECK(espnow_send_state_sync(seq, winner_mac));
  memcpy(g_last_peer_mac, winner_mac, 6);
  g_granted_seq_valid = true; g_granted_seq = seq;
  g_wait_ack2 = false;
  g_no_tagback_until_us = esp_timer_get_time() + (int64_t)PASS_COOLDOWN_MS*1000;
  g_role = ROLE_RUNNER;
  ir_rx_set_enabled(false);
  
  // Update LEDs for RUNNER (off)
  if (led_strip) {
    ESP_ERROR_CHECK(led_strip_clear(led_strip));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
  }
  
  // Update display
  display.fillScreen(TFT_BLACK);
  display.setCursor(0, 60);
  display.printf("ROLE: RUNNER");
  
  ESP_LOGI("CORE","ROLE -> RUNNER (commit seq=%u)", (unsigned)seq);
}

static void ack2_timer_cb(void*) {
  if (!g_wait_ack2) return;
  ESP_LOGW(TAG,"ACK2 timeout (seq=%u from %02X:%02X:%02X:%02X:%02X:%02X)",
           (unsigned)g_wait_ack2_seq,
           g_wait_ack2_mac[0],g_wait_ack2_mac[1],g_wait_ack2_mac[2],
           g_wait_ack2_mac[3],g_wait_ack2_mac[4],g_wait_ack2_mac[5]);
  g_wait_ack2 = false; // give up; next window will run
}

// ---------- Holder window task ----------
static TaskHandle_t g_window_task = nullptr;
static esp_timer_handle_t g_window_tick = nullptr;

static void window_tick_cb(void*){
  if (g_window_task) xTaskNotifyGive(g_window_task);
}

static void holder_window_task(void*){
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (g_role != ROLE_HOLDER) continue;
    if (!g_game_started) continue;  // Wait for countdown to complete
    int64_t now = esp_timer_get_time();
    if (now < g_holder_startup_until_us) continue;

    g_window_seq++;
    g_granted_seq_valid = false;
    g_empty_windows++;  // Increment empty window counter
    
    // Dynamic K calculation to fit slots within period
    const uint32_t period_us = (uint32_t)BEACON_PERIOD_MS * 1000u;
    const uint32_t guard_us  = 5000;              // room for ACKs and timing
    uint32_t maxK = (period_us > guard_us) ? (period_us - guard_us) / UPLINK_SLOT_US : 1;
    uint8_t  K    = (uint8_t)((UPLINK_K < maxK) ? UPLINK_K : maxK);
    if (K == 0) K = 1;
    
    const uint32_t delta_us = 7000; // window T0 = now + 7ms (reduced from 10ms)
    ESP_ERROR_CHECK(espnow_broadcast_ir_window(g_window_seq, K, delta_us));
    ESP_LOGI(TAG, "IR_WINDOW seq=%u K=%u/%u T0=now+%u us window_len=%u us empty_count=%u", 
             (unsigned)g_window_seq, (unsigned)K, (unsigned)UPLINK_K, (unsigned)delta_us, (unsigned)(K * UPLINK_SLOT_US), (unsigned)g_empty_windows);
    zt_round_maybe_relax(g_empty_windows, g_self_mac);
    if (g_empty_windows >= 10) {
      ESP_LOGW(TAG, "WARNING: No IR received in %u consecutive windows - possible starvation", (unsigned)g_empty_windows);
    }
  }
}

// ---------- Simple IR UART RX (holder path) ----------
static void uart_rx_task(void*){
  enum { WAIT_Z, WAIT_T, READ_LEN, READ_PAYLOAD, READ_CRC } st = WAIT_Z;
  uint8_t buf[64], payload[IR_MAX_PAYLOAD]; uint8_t len=0; size_t pi=0;
  for(;;){
    if (!g_rx_enabled){ vTaskDelay(pdMS_TO_TICKS(20)); continue; }
    int n = uart_read_bytes(IR_UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(20));
    if (n<=0) continue;
    for (int i=0;i<n;++i){
      uint8_t b=buf[i];
      switch(st){
        case WAIT_Z: if (b==IR_PREAMBLE0){ st=WAIT_T; } break;
        case WAIT_T:
          if (b==IR_PREAMBLE1){ st=READ_LEN; } else { st=WAIT_Z; }
          break;
        case READ_LEN:
          len=b; if (len==0 || len>IR_MAX_PAYLOAD){ st=WAIT_Z; break; }
          pi=0; st=READ_PAYLOAD; break;
        case READ_PAYLOAD:
          payload[pi++]=b; if (pi>=len) st=READ_CRC; break;
        case READ_CRC: {
          // verify CRC
          uint8_t tmp[1+IR_MAX_PAYLOAD]; tmp[0]=len; memcpy(tmp+1,payload,len);
          if (crc8_maxim(tmp, 1+len) == b){
            bool has_seq = (len == (IR_MAC_LEN+2));
            uint16_t seq = has_seq ? (uint16_t)payload[0] | ((uint16_t)payload[1]<<8) : 0;
            const uint8_t* src = has_seq ? (payload+2) : payload;
            if (g_role == ROLE_HOLDER && has_seq) {
              // accept only once per seq
              if (g_granted_seq_valid && g_granted_seq == seq) { 
                ESP_LOGD(TAG, "IR drop: dup_seq=%u", (unsigned)seq);
                st=WAIT_Z; break; 
              }
              // no-tagback and seq gating
              int64_t now = esp_timer_get_time();
              if (now < g_no_tagback_until_us){ 
                ESP_LOGD(TAG, "IR drop: cooldown remaining=%lld us", (long long)(g_no_tagback_until_us - now));
                st=WAIT_Z; break; 
              }
              if (seq != g_window_seq){ 
                ESP_LOGD(TAG, "IR drop: seq_mismatch got=%u want=%u", (unsigned)seq, (unsigned)g_window_seq);
                st=WAIT_Z; break; 
              }  // must match current window
              
              // Distinct-before-repeat: only accept not-yet-seen candidates in this round
              if (!zt_is_candidate_allowed(src)) { 
                ESP_LOGI(TAG, "IR drop: not_distinct %02X:%02X:%02X - distinct-before-repeat active", src[3], src[4], src[5]);
                st=WAIT_Z; break; 
              }
              
              // IR received successfully - reset empty counter
              g_empty_windows = 0;
#if MODE_B_USE_ACK2
              // Begin handshake: ACK1 → wait for ACK2 → then commit
              memcpy(g_wait_ack2_mac, src, 6);
              g_wait_ack2_seq = seq;
              g_wait_ack2 = true;
              if (!g_ack2_timer) {
                const esp_timer_create_args_t args = { .callback = &ack2_timer_cb, .arg = nullptr, .name = "ack2_to" };
                ESP_ERROR_CHECK(esp_timer_create(&args, &g_ack2_timer));
              }
              esp_timer_stop(g_ack2_timer);
              ESP_ERROR_CHECK(esp_timer_start_once(g_ack2_timer, (uint64_t)ACK2_TIMEOUT_MS*1000ULL));
              ESP_ERROR_CHECK(espnow_send_ack1_h2r(src, seq));
              ESP_LOGI(TAG,"ACK1 sent to %02X:%02X:%02X:%02X:%02X:%02X (seq=%u)",
                       src[0],src[1],src[2],src[3],src[4],src[5], (unsigned)seq);
#else
              // Immediate grant (faster but less robust)
              ESP_ERROR_CHECK(espnow_send_pass_grant(src, seq));
              ESP_ERROR_CHECK(espnow_send_state_sync(seq, src));
              memcpy(g_last_peer_mac, src, 6);
              g_granted_seq_valid = true; g_granted_seq = seq;
              g_no_tagback_until_us = esp_timer_get_time() + (int64_t)PASS_COOLDOWN_MS*1000;
              g_role = ROLE_RUNNER;
              ir_rx_set_enabled(false);
              
                             // Update distinct-before-repeat round state
               ESP_LOGI(TAG, "Recording grant for %02X:%02X:%02X:%02X:%02X:%02X in distinct-before-repeat (immediate)", 
                        src[0], src[1], src[2], src[3], src[4], src[5]);
               zt_record_grant(src);
              
              // Update LEDs for RUNNER (off)
              if (led_strip) {
                ESP_ERROR_CHECK(led_strip_clear(led_strip));
                ESP_ERROR_CHECK(led_strip_refresh(led_strip));
              }
              
              // Update display
              display.fillScreen(TFT_BLACK);
              display.setCursor(0, 60);
              display.printf("ROLE: RUNNER");
              
              ESP_LOGI(TAG,"Immediate GRANT to %02X:%02X:%02X:%02X:%02X:%02X (seq=%u)",
                       src[0],src[1],src[2],src[3],src[4],src[5], (unsigned)seq);
#endif
            }
          }
          st=WAIT_Z; break;
        }
      }
    }
  }
}

extern "C" void ir_uplink_test_main(void) {
  ESP_LOGI(TAG, "=== MODE B: IR UPLINK TEST (holder RX / runners IR TX) ===");
  ESP_LOGI(TAG, "Starting IR Uplink Test (Mode B)");
  esp_read_mac(g_self_mac, ESP_MAC_WIFI_STA);

  // Immediate LED clear on startup (before anything else)
  if (led_strip == nullptr) {
    led_strip_config_t strip_config = {
      .strip_gpio_num = HP_LED_GPIO,
      .max_leds = HP_LED_COUNT,
    };
    led_strip_rmt_config_t rmt_config = {
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = 10 * 1000 * 1000, // 10MHz
      .mem_block_symbols = 64,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
  }
  
  // Aggressive clear for WS2812 reset (multiple times to ensure they're off)
  if (led_strip) {
    ESP_ERROR_CHECK(led_strip_clear(led_strip));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
    vTaskDelay(pdMS_TO_TICKS(10)); // Small delay for WS2812 to process
    ESP_ERROR_CHECK(led_strip_clear(led_strip));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
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

  // HW init
  setup_rmt_tx();
  setup_uart_rx();
  ESP_ERROR_CHECK(espnow_init_once());
  
  // 1) Join + countdown → get initial role & roster
  zt_role_t init_role; zt_roster_t roster;
  
  // LCD countdown hook so all devices show the same timer
  zt_set_countdown_cb(zt_render_countdown);
  
  zt_join_and_countdown(&init_role, &roster, ZT_IS_HOST == 1); // HOST if ZT_IS_HOST=1

  // 2) Apply role to Mode B (uplink mode): holder RX, runners TX
  g_role = (init_role == ZT_ROLE_HOLDER) ? ROLE_HOLDER : ROLE_RUNNER;
  ir_rx_set_enabled(g_role == ROLE_HOLDER);
  
  ESP_LOGI(TAG, "Build config: ZT_IS_HOST=%d", ZT_IS_HOST);
  ESP_LOGI(TAG, "Test harness: initial role = %s", (init_role == ZT_ROLE_HOLDER) ? "HOLDER" : "RUNNER");
  ESP_LOGI(TAG, "Test harness: roster count = %d", roster.count);

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
      for (int i = 0; i < HP_LED_COUNT; i++) {
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 255, 0, 0)); // Red
      }
      ESP_ERROR_CHECK(led_strip_refresh(led_strip));
      ESP_LOGI(TAG, "LED: ON (HOLDER) - all %d side LEDs red", HP_LED_COUNT);
    } else {
      // RUNNER: LEDs off (already cleared above)
      ESP_LOGI(TAG, "LED: OFF (RUNNER)");
    }
  }

  // Tasks/timers
  // Holder periodic window ticker (~ every BEACON_PERIOD_MS, matching Mode A cadence)
  if (g_window_task == nullptr) {
    xTaskCreate(holder_window_task, "holder_window_task", 4096, nullptr, 5, &g_window_task);
    const esp_timer_create_args_t a = { .callback = &window_tick_cb, .arg=nullptr, .name="window_tick" };
    ESP_ERROR_CHECK(esp_timer_create(&a, &g_window_tick));
    // Start the timer but the task will wait for countdown
    ESP_ERROR_CHECK(esp_timer_start_periodic(g_window_tick, (uint64_t)BEACON_PERIOD_MS * 1000ULL));
  }
  // UART RX task (decodes runner IR when we are holder)
  xTaskCreate(uart_rx_task, "uart_rx_task", 4096, nullptr, 6, nullptr);
  
  // Wait for countdown to complete before starting game
  ESP_LOGI(TAG, "Waiting for countdown to complete...");
  // The countdown is already handled by the test harness, so we just need to wait a bit
  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_LOGI(TAG, "Countdown complete! Game starting...");
  g_game_started = true;  // Enable game logic
}

static void zt_render_countdown(int secs, const uint8_t initial_holder[6], const zt_roster_t* roster) {
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

      // no-tagback check
      int64_t now = esp_timer_get_time();
      if (now < g_no_tagback_until_us) { ESP_LOGI(TAG,"cooldown; skip window"); break; }

      // Skip if we've already held this round (distinct-before-repeat, local view)
      if (!zt_is_candidate_allowed(g_self_mac)) { 
        ESP_LOGI(TAG,"ineligible this round; skip window - distinct-before-repeat active"); 
        break; 
      }

      // Pick slot deterministically and schedule IR uplink
      slot_pick_t sp = pick_slot(seq, holder, g_self_mac, K ? K : UPLINK_K);
      uint64_t delay = (uint64_t)delta_us + (uint64_t)sp.slot * UPLINK_SLOT_US + sp.jitter_us;
      esp_timer_handle_t t = nullptr;
      const esp_timer_create_args_t a = { .callback = [](void* p){
          uint16_t s = (uint16_t)(uintptr_t)p;
          runner_send_ir_uplink(s);
        }, .arg = (void*)(uintptr_t)seq, .name="ir_uplink_tx" };
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

// Mode A (downlink): HOLDER transmits IR beacons; RUNNERS receive IR and send PASS_REQ.
// Mirrored from Mode B skeleton to stay harness-compatible.
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/rmt_tx.h"
#include "ir_simple_test.h"   // reuse IR constants/macros (IR_BIT_US, etc.)
#include "zt_test_mode.h"
#include "M5GFX.h"            // for display

extern "C" {
  #include "led_strip.h"
}

// ---- External singletons (already defined in main.cpp & ir_simple_test.cpp) ----
extern M5GFX display;
extern led_strip_handle_t led_strip;

#define IR_TX_GPIO   GPIO_NUM_26
#define IR_RX_GPIO   GPIO_NUM_36      // UART RX pin (same as Mode B wiring)
#define IR_UART_PORT UART_NUM_2

// Host/peer selection - same as Mode B
#ifndef ZT_IS_HOST
#define ZT_IS_HOST 0   // build: -DZT_IS_HOST=1 for HOST
#endif

static const char* TAG = "IR_DOWNLINK_TEST";

typedef enum { ROLE_HOLDER, ROLE_RUNNER } role_t;
static volatile role_t g_role = ROLE_RUNNER;
static uint8_t  g_self_mac[6] = {0};
static uint16_t g_seq = 0;
static uint16_t g_beacon_seq_curr = 0;
static uint16_t g_beacon_seq_prev = 0;
static int64_t  g_beacon_sent_at_us = 0;
static int64_t  g_no_tagback_until_us = 0;  // Cooldown timer to prevent immediate passback
static bool     g_espnow_ready = false;
static bool     g_rx_enabled = false;
static bool     g_game_started = false;  // Track when countdown is complete

// Accept a PASS_REQ that matches the previous beacon if it arrives within this window.
#ifndef MODEA_ACCEPT_PREV_BEACON_US
#define MODEA_ACCEPT_PREV_BEACON_US 90000   // 90 ms (tune: <= BEACON_PERIOD_MS)
#endif

// --- RMT TX (IR) ---
static rmt_channel_handle_t rmt_tx = nullptr;
static rmt_encoder_handle_t tx_copy = nullptr;
static TaskHandle_t  s_beacon_task = nullptr;
static esp_timer_handle_t s_beacon_timer = nullptr;
static void beacon_timer_cb(void*) { if (s_beacon_task) xTaskNotifyGive(s_beacon_task); }

// ---------- UI helpers ----------
static void draw_role_ui() {
  display.fillScreen(TFT_BLACK);
  display.setTextSize(2);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.setCursor(8, 12);
  display.printf("MODE A (Downlink)");
  display.setCursor(8, 40);
  display.printf("ROLE: %s", (g_role==ROLE_HOLDER)?"HOLDER":"RUNNER");
  // Simple holder "LED" banner on LCD (replace with your side-LED hook if desired)
  if (g_role == ROLE_HOLDER) {
    display.fillRect(0, 70, display.width(), 30, TFT_RED);
    display.setCursor(8, 76);
    display.setTextColor(TFT_BLACK, TFT_RED);
    display.printf("HOLDER ACTIVE");
  } else {
    display.fillRect(0, 70, display.width(), 30, TFT_DARKGREEN);
    display.setCursor(8, 76);
    display.setTextColor(TFT_WHITE, TFT_DARKGREEN);
    display.printf("RUNNER LISTENING");
  }
}

static void countdown_draw_cb(int secs_remaining, const uint8_t initial_holder_mac[6], const zt_roster_t* roster) {
  // Big T-N overlay during harness countdown, like Mode B
  display.fillScreen(TFT_BLACK);
  display.setTextDatum(textdatum_t::middle_center);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.setTextSize(3);
  display.drawString("SYNC START", display.width()/2, display.height()/2 - 28);
  display.setTextSize(5);
  char buf[16]; snprintf(buf, sizeof(buf), "%d", secs_remaining);
  display.drawString(buf, display.width()/2, display.height()/2 + 8);
  // Small footer: initial holder hint
  display.setTextSize(2);
  char macs[32];
  snprintf(macs, sizeof(macs), "%02X:%02X:%02X", initial_holder_mac[3], initial_holder_mac[4], initial_holder_mac[5]);
  display.drawString(macs, display.width()/2, display.height()/2 + 48);
}

static void setup_rmt_tx() {
  rmt_tx_channel_config_t txc = { 
    .gpio_num=IR_TX_GPIO, 
    .clk_src=RMT_CLK_SRC_DEFAULT,
    .resolution_hz=1000000, 
    .mem_block_symbols=128, 
    .trans_queue_depth=4,
    .intr_priority = 0,
    .flags = {0}
  };
  ESP_LOGI(TAG, "Setting up RMT TX for IR on GPIO %d", IR_TX_GPIO);
  ESP_ERROR_CHECK(rmt_new_tx_channel(&txc, &rmt_tx));
  rmt_carrier_config_t c = { 
    .frequency_hz=38000, 
    .duty_cycle=0.33, 
    .flags={ .polarity_active_low=0 } 
  };
  ESP_ERROR_CHECK(rmt_apply_carrier(rmt_tx, &c));
  rmt_copy_encoder_config_t cc = {};
  ESP_ERROR_CHECK(rmt_new_copy_encoder(&cc, &tx_copy));
  ESP_ERROR_CHECK(rmt_enable(rmt_tx));
}

// Build one UART byte into IR symbols (LSB first; mark=0, space=1)
static size_t build_uart_byte(uint8_t b, rmt_symbol_word_t* out){
  auto mark=[](rmt_symbol_word_t* s,uint32_t us){s->level0=1;s->duration0=us;s->level1=0;s->duration1=1;};
  auto spac=[](rmt_symbol_word_t* s,uint32_t us){s->level0=0;s->duration0=us;s->level1=0;s->duration1=1;};
  size_t i=0; mark(&out[i++],IR_BIT_US); for(int k=0;k<8;++k)((b>>k)&1)?spac(&out[i++],IR_BIT_US):mark(&out[i++],IR_BIT_US); spac(&out[i++],IR_BIT_US); return i;
}

static void holder_send_ir_beacon(uint16_t seq){
  uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_WIFI_STA);
  const uint8_t len = (uint8_t)(IR_MAC_LEN + 2); // SEQ(2)+MAC(6)
  uint8_t crc_src[1+2+IR_MAC_LEN]; crc_src[0]=len; crc_src[1]=(uint8_t)(seq&0xFF); crc_src[2]=(uint8_t)(seq>>8); memcpy(&crc_src[3],mac,6);
  uint8_t crc = 0; { uint8_t c=0; for(size_t i=0;i<sizeof(crc_src);++i){ c^=crc_src[i]; for(int b=0;b<8;++b) c=(c&0x80)?(uint8_t)((c<<1)^0x31):(uint8_t)(c<<1);} crc=c; }
  rmt_symbol_word_t syms[200]; size_t n=0;
  n+=build_uart_byte(IR_PREAMBLE0,&syms[n]); n+=build_uart_byte(IR_PREAMBLE1,&syms[n]);
  n+=build_uart_byte(len,&syms[n]); n+=build_uart_byte((uint8_t)(seq&0xFF),&syms[n]); n+=build_uart_byte((uint8_t)(seq>>8),&syms[n]);
  for(int i=0;i<IR_MAC_LEN;++i) { n+=build_uart_byte(mac[i],&syms[n]); } n+=build_uart_byte(crc,&syms[n]);
  rmt_transmit_config_t cfg={.loop_count=0, .flags = {0}};
  esp_err_t err = rmt_transmit(rmt_tx, tx_copy, syms, n*sizeof(syms[0]), &cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "RMT transmit failed: %s", esp_err_to_name(err));
    return;
  }
  err = rmt_tx_wait_all_done(rmt_tx, pdMS_TO_TICKS(1000));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "RMT wait failed: %s", esp_err_to_name(err));
    return;
  }
  g_beacon_seq_prev   = g_beacon_seq_curr;
  g_beacon_seq_curr   = seq;
  g_beacon_sent_at_us = esp_timer_get_time();
  ESP_LOGI(TAG,"IR beacon sent (SEQ=%u)", (unsigned)seq);
}

// --- UART RX (runners) ---
static void setup_uart_rx(){
  uart_config_t cfg={ 
    .baud_rate=IR_BAUD, 
    .data_bits=UART_DATA_8_BITS, 
    .parity=UART_PARITY_DISABLE,
    .stop_bits=UART_STOP_BITS_1, 
    .flow_ctrl=UART_HW_FLOWCTRL_DISABLE, 
    .source_clk=UART_SCLK_DEFAULT
  };
  ESP_ERROR_CHECK(uart_param_config(IR_UART_PORT,&cfg));
  ESP_ERROR_CHECK(uart_driver_install(IR_UART_PORT, IR_UART_RX_BUF_SZ, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_set_pin(IR_UART_PORT, UART_PIN_NO_CHANGE, IR_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  
#ifdef IR_UART_RX_INVERT
  ESP_ERROR_CHECK(uart_set_line_inverse(IR_UART_PORT, UART_SIGNAL_RXD_INV));
#endif
  
  ESP_LOGI(TAG,"UART RX ready on GPIO%d @ %d-8N1", (int)IR_RX_GPIO, IR_BAUD);
}
void ir_rx_set_enabled(bool en){ g_rx_enabled = en; }

// --- ESPNOW infra ---
static void ensure_nvs(){ auto e=nvs_flash_init(); if(e==ESP_ERR_NVS_NO_FREE_PAGES||e==ESP_ERR_NVS_NEW_VERSION_FOUND){ESP_ERROR_CHECK(nvs_flash_erase()); ESP_ERROR_CHECK(nvs_flash_init());} else ESP_ERROR_CHECK(e);}
static esp_err_t espnow_init_once(){
  static bool ready=false; if(ready) return ESP_OK; ensure_nvs();
  esp_netif_init(); esp_event_loop_create_default(); esp_netif_create_default_wifi_sta();
  wifi_init_config_t w=WIFI_INIT_CONFIG_DEFAULT(); ESP_ERROR_CHECK(esp_wifi_init(&w));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA)); ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
  ESP_ERROR_CHECK(esp_now_init());
  // broadcast peer
  uint8_t bcast[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; esp_now_peer_info_t p={}; memcpy(p.peer_addr,bcast,6); p.ifidx=WIFI_IF_STA; p.channel=ESPNOW_CHANNEL; p.encrypt=false; esp_now_add_peer(&p);
  ESP_ERROR_CHECK(esp_now_register_send_cb([](const uint8_t* m, esp_now_send_status_t st){ ESP_LOGD("ESPNOW","tx %02X:%02X:%02X:%02X:%02X:%02X -> %s", m[0],m[1],m[2],m[3],m[4],m[5], st==ESP_NOW_SEND_SUCCESS?"OK":"FAIL"); }));
  ready=true; g_espnow_ready=true; return ESP_OK;
}

// Messages: keep Mode-A codes
#define MSG_PASS_REQ   0xA1
#define MSG_PASS_GRANT 0xA2
#define MSG_STATE_SYNC 0xA3

static esp_err_t ensure_peer(const uint8_t* mac){ esp_now_peer_info_t p={}; memcpy(p.peer_addr,mac,6); p.ifidx=WIFI_IF_STA; p.channel=ESPNOW_CHANNEL; p.encrypt=false; auto e=esp_now_add_peer(&p); return (e==ESP_ERR_ESPNOW_EXIST)?ESP_OK:e; }
static esp_err_t send_pass_req(const uint8_t* holder, uint16_t seq){
  ESP_ERROR_CHECK(ensure_peer(holder));
  uint8_t pl[10]={ MSG_PASS_REQ, (uint8_t)(seq&0xFF),(uint8_t)(seq>>8), holder[0],holder[1],holder[2],holder[3],holder[4],holder[5] };
  return esp_now_send(holder, pl, sizeof(pl));
}
static esp_err_t send_pass_grant(const uint8_t* winner, uint16_t seq){
  ESP_ERROR_CHECK(ensure_peer(winner));
  uint8_t pl[10]={ MSG_PASS_GRANT, (uint8_t)(seq&0xFF),(uint8_t)(seq>>8), winner[0],winner[1],winner[2],winner[3],winner[4],winner[5] };
  return esp_now_send(winner, pl, sizeof(pl));
}
static esp_err_t send_state_sync(uint16_t seq, const uint8_t* new_holder){
  uint8_t pl[12]={ MSG_STATE_SYNC, (uint8_t)(seq&0xFF),(uint8_t)(seq>>8),
    new_holder[0],new_holder[1],new_holder[2],new_holder[3],new_holder[4],new_holder[5],
    (uint8_t)(PASS_COOLDOWN_MS&0xFF),(uint8_t)((PASS_COOLDOWN_MS>>8)&0xFF) };
  uint8_t bcast[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; return esp_now_send(bcast, pl, sizeof(pl));
}

// --- Recv callback ---
static void recv_cb(const esp_now_recv_info* info, const uint8_t* data, int len){
  if(len<1) return;
  switch(data[0]){
    case MSG_PASS_REQ: { // holder receives claims
      if(g_role!=ROLE_HOLDER || len<10) break;
      uint16_t seq=(uint16_t)data[1] | ((uint16_t)data[2]<<8);
      uint8_t  holder[6]; memcpy(holder, &data[3], 6);
      if(memcmp(holder,g_self_mac,6)!=0) break;                    // aimed at me
      // Accept current beacon, or (briefly) the previous one to absorb ESPNOW latency
      int64_t now = esp_timer_get_time();
      bool ok = (seq == g_beacon_seq_curr) ||
                ((seq == g_beacon_seq_prev) && (now - g_beacon_sent_at_us) <= MODEA_ACCEPT_PREV_BEACON_US);
      if(!ok) break;
      
      // Check cooldown - prevent immediate passback
      if (now < g_no_tagback_until_us) {
        ESP_LOGD(TAG, "PASS_REQ drop: cooldown remaining=%lld us", (long long)(g_no_tagback_until_us - now));
        break;
      }
      
      // First valid wins: grant + sync + flip roles
      send_pass_grant(info->src_addr, seq);
      send_state_sync(seq, info->src_addr);
      
      // Set cooldown to prevent immediate passback
      g_no_tagback_until_us = esp_timer_get_time() + (int64_t)PASS_COOLDOWN_MS*1000;
      
      g_role=ROLE_RUNNER;
      ir_rx_set_enabled(true);
      uart_flush_input(IR_UART_PORT);  // drop any partial junk before we start parsing
      
      // Update LEDs for RUNNER (off)
      if (led_strip) {
        ESP_ERROR_CHECK(led_strip_clear(led_strip));
        ESP_ERROR_CHECK(led_strip_refresh(led_strip));
      }
      
      draw_role_ui();
      ESP_LOGI(TAG,"GRANT -> %02X:%02X:%02X:%02X:%02X:%02X seq=%u",
        info->src_addr[0],info->src_addr[1],info->src_addr[2],info->src_addr[3],info->src_addr[4],info->src_addr[5], (unsigned)seq);
      break;
    }
    case MSG_PASS_GRANT: // runner becomes holder
    case MSG_STATE_SYNC: {
      if(len<10) break;
      uint16_t seq=(uint16_t)data[1] | ((uint16_t)data[2]<<8);
      uint8_t new_holder[6]; memcpy(new_holder,&data[3],6);
      if(memcmp(new_holder,g_self_mac,6)==0){
        g_role=ROLE_HOLDER;
        ir_rx_set_enabled(false);
        // next timer tick will send beacons; update UI immediately
        
        // Update LEDs for HOLDER (red)
        if (led_strip) {
          ESP_ERROR_CHECK(led_strip_clear(led_strip));
          ESP_ERROR_CHECK(led_strip_refresh(led_strip));
          for (int i = 0; i < HP_LED_COUNT; i++) {
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 255, 0, 0)); // Red
          }
          ESP_ERROR_CHECK(led_strip_refresh(led_strip));
        }
        
        draw_role_ui();
        // small guard; next timer tick will send beacons
        ESP_LOGI(TAG,"ROLE -> HOLDER (seq=%u)", (unsigned)seq);
      }else{
        g_role=ROLE_RUNNER;
        ir_rx_set_enabled(true);
        uart_flush_input(IR_UART_PORT);
        
        // Update LEDs for RUNNER (off)
        if (led_strip) {
          ESP_ERROR_CHECK(led_strip_clear(led_strip));
          ESP_ERROR_CHECK(led_strip_refresh(led_strip));
        }
        
        draw_role_ui();
      }
      break;
    }
    default: break;
  }
}
extern "C" void ir_downlink_install_recv_cb(void){ ESP_ERROR_CHECK(esp_now_register_recv_cb(recv_cb)); }

// --- Tasks ---
static void beacon_task(void*){
  s_beacon_task = xTaskGetCurrentTaskHandle();
  // periodic timer
  if(!s_beacon_timer){ 
    const esp_timer_create_args_t a={ 
      .callback=&beacon_timer_cb, 
      .arg = nullptr,
      .dispatch_method = ESP_TIMER_TASK,
      .name="downlink_beacon",
      .skip_unhandled_events = false
    }; 
    ESP_ERROR_CHECK(esp_timer_create(&a,&s_beacon_timer)); 
  }
  ESP_ERROR_CHECK(esp_timer_start_periodic(s_beacon_timer, (uint64_t)BEACON_PERIOD_MS*1000ULL));
  for(;;){
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if(g_role!=ROLE_HOLDER || !g_game_started) continue;
    holder_send_ir_beacon(g_seq++);
  }
}

static void uart_rx_task(void*){
  uint8_t rb[256]; int rblen = 0;
  uint8_t buf[128];
  for(;;){
    if(!g_rx_enabled || !g_game_started){ vTaskDelay(pdMS_TO_TICKS(10)); continue; }
    int n = uart_read_bytes(IR_UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(20));
    if(n<=0) continue;
    // append to ring
    if(n > (int)sizeof(rb)-rblen) { rblen = 0; } // overflow guard: reset
    memcpy(&rb[rblen], buf, n); rblen += n;
    // scan ring: look for ZT, then LEN==8, then SEQ+MAC+CRC; on success send PASS_REQ
    int i = 0;
    while (i + 4 < rblen) {
      if(rb[i]==IR_PREAMBLE0 && rb[i+1]==IR_PREAMBLE1){
        uint8_t len = rb[i+2];
        if(len==(IR_MAC_LEN+2) && (i+3+len) < rblen){
          uint16_t seq = (uint16_t)rb[i+3] | ((uint16_t)rb[i+4]<<8);
          uint8_t  holder[6]; memcpy(holder,&rb[i+5],6);
          uint8_t  crc   = rb[i+11];
          // crc check (same as TX)
          uint8_t src[1+2+6]; src[0]=len; src[1]=rb[i+3]; src[2]=rb[i+4]; memcpy(&src[3],holder,6);
          uint8_t c=0; for(size_t k=0;k<sizeof(src);++k){ c^=src[k]; for(int b=0;b<8;++b) c=(c&0x80)?(uint8_t)((c<<1)^0x31):(uint8_t)(c<<1); }
                     if(c==crc){
             // Check cooldown before sending PASS_REQ
             int64_t now = esp_timer_get_time();
             if (now < g_no_tagback_until_us) {
               ESP_LOGD(TAG, "BEACON drop: cooldown remaining=%lld us", (long long)(g_no_tagback_until_us - now));
               i += 3+len; // jump past this packet
               continue;
             }
             ESP_LOGI(TAG,"BEACON ok: seq=%u from %02X:%02X:%02X:%02X:%02X:%02X",(unsigned)seq,holder[0],holder[1],holder[2],holder[3],holder[4],holder[5]);
             send_pass_req(holder, seq);
             i += 3+len; // jump past this packet
             continue;
           }
        }
      }
      // no valid frame at i: advance
      i++;
    }
    // compact leftover tail
    if (i > 0 && i < rblen) { memmove(rb, &rb[i], rblen - i); rblen -= i; }
    else if (i >= rblen) { rblen = 0; }
  }
}

// --- Entry ---
extern "C" void ir_downlink_test_main(void){
  ESP_LOGI(TAG, "=== MODE A: IR DOWNLINK TEST (holder TX / runners IR RX) ===");
  ESP_LOGI(TAG, "Starting IR Downlink Test (Mode A)");
  ESP_ERROR_CHECK(espnow_init_once()); esp_read_mac(g_self_mac, ESP_MAC_WIFI_STA);

  // Immediate LED clear on startup (before anything else)
  if (led_strip == nullptr) {
    led_strip_config_t strip_config = {
      .strip_gpio_num = HP_LED_GPIO,
      .max_leds = HP_LED_COUNT,
      .led_model = LED_MODEL_WS2812,
      .flags = {0}
    };
    led_strip_rmt_config_t rmt_config = {
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = 10 * 1000 * 1000, // 10MHz
      .mem_block_symbols = 64,
      .flags = {0}
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
  
  // Display setup
  display.begin();
  display.setTextSize(2);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.fillScreen(TFT_BLACK);
  display.setCursor(0, 20);
  display.printf("MODE A");
  display.setCursor(0, 60);
  display.printf("ROLE: %s", ZT_IS_HOST ? "HOST" : "PEER");

  // HW init
  setup_rmt_tx();
  setup_uart_rx();
  
  // Harness: discover + countdown + initial role
  // Register a countdown draw callback so all devices show the same T-minus UI
  zt_set_countdown_cb(countdown_draw_cb);
  zt_role_t r; zt_roster_t roster={0};
  zt_join_and_countdown(&r, &roster,
    /*is_host=*/(bool)ZT_IS_HOST);       // the host opens join & broadcasts START
  g_role = (r==ZT_ROLE_HOLDER)?ROLE_HOLDER:ROLE_RUNNER;
  
  // Update display with actual role (post-countdown)
  draw_role_ui();
  
  // Reinstall our recv handler after the harness
  ir_downlink_install_recv_cb();

  // IR setup
  ir_rx_set_enabled(g_role==ROLE_RUNNER);

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

  // tasks
  xTaskCreatePinnedToCore(beacon_task, "beacon", 4096, NULL, 4, NULL, tskNO_AFFINITY);
  xTaskCreatePinnedToCore(uart_rx_task,"ir_rx", 4096, NULL, 4, NULL, tskNO_AFFINITY);

  // Wait for countdown to complete before starting game
  ESP_LOGI(TAG, "Waiting for countdown to complete...");
  // The countdown is already handled by the test harness, so we just need to wait a bit
  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_LOGI(TAG, "Countdown complete! Game starting...");
  g_game_started = true;  // Enable game logic

  ESP_LOGI(TAG,"Start as %s", g_role==ROLE_HOLDER?"HOLDER":"RUNNER");
  // idle: tasks do all the work
  for(;;) vTaskDelay(pdMS_TO_TICKS(1000));
}
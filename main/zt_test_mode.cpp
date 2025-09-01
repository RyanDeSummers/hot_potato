#include "zt_test_mode.h"
#include <string.h>
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

static const char* TAG = "ZT_TEST";

// simple roster set
static zt_roster_t g_roster = {};
static uint64_t    g_last_hello_us[ZT_MAX_PEERS] = {0};
static uint32_t    g_alive_timeout_us = 2000000; // 2s default
static esp_timer_handle_t g_hello_timer = nullptr;
static bool roster_has(const uint8_t mac[6]) {
  for (int i=0;i<g_roster.count;i++) if (!memcmp(g_roster.macs[i], mac, 6)) return true;
  return false;
}
static void roster_add(const uint8_t mac[6]) {
  if (g_roster.count >= ZT_MAX_PEERS) return;
  if (!roster_has(mac)) memcpy(g_roster.macs[g_roster.count++], mac, 6);
  // mark as seen now
  for (int i=0;i<g_roster.count;i++) if (!memcmp(g_roster.macs[i], mac, 6)) { g_last_hello_us[i] = esp_timer_get_time(); break; }
}
static void roster_sort() { // lexicographic sort for determinism
  for (int i=0;i<g_roster.count;i++)
    for (int j=i+1;j<g_roster.count;j++)
      if (memcmp(g_roster.macs[i], g_roster.macs[j], 6) > 0) {
        uint8_t tmp[6]; memcpy(tmp,g_roster.macs[i],6);
        memcpy(g_roster.macs[i],g_roster.macs[j],6);
        memcpy(g_roster.macs[j],tmp,6);
      }
}
static void sort_and_pick_leader(uint8_t out_leader[6]) {
  roster_sort();
  memcpy(out_leader, g_roster.macs[0], 6); // lowest MAC = leader
}

// temporary recv handler plumbing
static bool g_started = false;
static bool g_have_roster = false;
static uint8_t g_initial_holder[6] = {0};
static uint64_t g_T0_us = 0;
static zt_countdown_render_cb_t g_cd_cb = nullptr;

static esp_now_recv_cb_t g_prev_recv = nullptr;

static void send_hello() {
  uint8_t p = ZT_MSG_HELLO;
  uint8_t bcast[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  esp_now_send(bcast, &p, 1);
}
static void hello_tick(void*) { send_hello(); }

static void send_roster_and_start_as_host() {
  // ROSTER
  uint8_t buf[1 + 1 + ZT_MAX_PEERS*6] = {0};
  buf[0] = ZT_MSG_ROSTER;
  buf[1] = g_roster.count;
  for (int i=0;i<g_roster.count;i++) memcpy(&buf[2 + i*6], g_roster.macs[i], 6);
  uint8_t bcast[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  esp_now_send(bcast, buf, 2 + g_roster.count*6);

  // Choose initial holder deterministically: hash of roster bytes mod N
  uint32_t h=2166136261u;
  for (int i=0;i<g_roster.count;i++)
    for (int b=0;b<6;b++){ h^=g_roster.macs[i][b]; h*=16777619u; }
  uint8_t idx = (g_roster.count>0) ? (h % g_roster.count) : 0;
  memcpy(g_initial_holder, g_roster.macs[idx], 6);

  // START with a relative delta (so peers compute local T0)
  uint32_t delta_us = (uint32_t)(ZT_COUNTDOWN_S * 1000000u);
  g_T0_us = esp_timer_get_time() + delta_us; // local copy for our own countdown/logs
  uint8_t s[1+6+4];
  s[0] = ZT_MSG_START;
  memcpy(&s[1], g_initial_holder, 6);
  // pack delta_us little-endian
  s[7]  = (uint8_t)(delta_us & 0xFF);
  s[8]  = (uint8_t)((delta_us >> 8) & 0xFF);
  s[9]  = (uint8_t)((delta_us >> 16) & 0xFF);
  s[10] = (uint8_t)((delta_us >> 24) & 0xFF);
  esp_now_send(bcast, s, sizeof(s));
  ESP_LOGI(TAG,"Host sent ROSTER(count=%d) and START (delta=%u us)", g_roster.count, (unsigned)delta_us);
}

static void test_recv_cb(const esp_now_recv_info* info, const uint8_t* data, int len) {
  if (!data || len<=0) { if (g_prev_recv) g_prev_recv(info,data,len); return; }
  switch (data[0]) {
    case ZT_MSG_HELLO: {
      roster_add(info->src_addr);
      // refresh alive timestamp
      for (int i=0;i<g_roster.count;i++) if (!memcmp(g_roster.macs[i], info->src_addr, 6)) { g_last_hello_us[i] = esp_timer_get_time(); break; }
      break;
    }
    case ZT_MSG_ROSTER: {
      if (len < 2) break;
      g_roster.count = (data[1] <= ZT_MAX_PEERS) ? data[1] : ZT_MAX_PEERS;
      for (int i=0;i<g_roster.count;i++) memcpy(g_roster.macs[i], &data[2+i*6], 6);
      g_have_roster = true;
      break;
    }
    case ZT_MSG_START: {
      if (len < 1+6+4) break;
      memcpy(g_initial_holder, &data[1], 6);
      uint32_t delta_us = (uint32_t)data[7] |
                          ((uint32_t)data[8] << 8) |
                          ((uint32_t)data[9] << 16) |
                          ((uint32_t)data[10] << 24);
      g_T0_us = esp_timer_get_time() + delta_us; // local T0
      g_started = true;
      break;
    }
    default: break;
  }
  // chain to previous handler so Mode A/B still see their messages
  if (g_prev_recv) g_prev_recv(info,data,len);
}

void zt_join_and_countdown(zt_role_t* out_initial_role, zt_roster_t* out_roster, bool is_host) {
  uint8_t self[6]; esp_read_mac(self, ESP_MAC_WIFI_STA);
  memcpy(g_roster.macs[g_roster.count++], self, 6);

  // hook our recv (chain old one)
  esp_now_register_recv_cb(nullptr); // get current? API doesn't expose, so we just replace.
  g_prev_recv = nullptr;             // both modes register after us; or re-register us after they did
  esp_now_register_recv_cb(test_recv_cb);

  if (is_host) {
    // HOST: Wait for button press, then start the game
    ESP_LOGI(TAG, "HOST MODE: Waiting for button press on GPIO %d...", ZT_HOST_BUTTON_GPIO);
    
    // Setup button GPIO (M5Stack FIRE: GPIO 34-39 have no internal pulls, board has external PU)
    gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << ZT_HOST_BUTTON_GPIO),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,   // GPIO 34-39 have no internal pulls; board has external PU
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // Wait for button press (active low for M5Stack FIRE)
    while (gpio_get_level(ZT_HOST_BUTTON_GPIO) == 1) {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    ESP_LOGI(TAG, "Button pressed! Starting game...");
    
    // Brief join window to collect peers AFTER button press
    ESP_LOGI(TAG, "Starting %d ms join window to collect peers...", ZT_JOIN_MS);
    int64_t end = esp_timer_get_time() + (int64_t)ZT_JOIN_MS*1000;
    while (esp_timer_get_time() < end) { 
      send_hello(); 
      vTaskDelay(pdMS_TO_TICKS(200)); 
    }
    ESP_LOGI(TAG, "Join window complete. Found %d peers.", g_roster.count);
    
    // Send ROSTER and START as host
    ESP_LOGI(TAG, "Host: sending ROSTER and START at time %lld us", (long long)esp_timer_get_time());
    send_roster_and_start_as_host();
    
  } else {
    // PEER: Wait for host to start the game
    ESP_LOGI(TAG, "PEER MODE: Waiting for host to start...");
    
    // Wait for host's HELLO messages and respond to them
    ESP_LOGI(TAG, "Waiting for host HELLO messages...");
    while (!g_started) {
      // Send HELLO periodically to announce presence
      send_hello();
      vTaskDelay(pdMS_TO_TICKS(500)); // Send every 500ms until host starts
    }
    ESP_LOGI(TAG, "Received START from host!");
  }

  // Optional: show countdown in logs with LCD callback
  int last_secs = -1;
  for (;;) {
    int64_t now = esp_timer_get_time();
    if (now >= (int64_t)g_T0_us) break;
    int secs = (int)((g_T0_us - now + 999999) / 1000000);
    if (secs != last_secs) {
      if (g_cd_cb) g_cd_cb(secs, g_initial_holder, &g_roster);
      ESP_LOGI(TAG,"Start in %d...", secs);
      last_secs = secs;
    }
    vTaskDelay(pdMS_TO_TICKS(250));
  }
  ESP_LOGI(TAG,"GO!");
  // Start periodic HELLO so liveness stays fresh during the game
  if (!g_hello_timer) {
    const esp_timer_create_args_t a = { .callback = &hello_tick, .arg = nullptr, .dispatch_method = ESP_TIMER_TASK, .name = "zt_hello" };
    ESP_ERROR_CHECK(esp_timer_create(&a, &g_hello_timer));
  }
  esp_timer_stop(g_hello_timer);
  ESP_ERROR_CHECK(esp_timer_start_periodic(g_hello_timer, 1000000)); // 1s

  // Decide our initial role
  zt_role_t role = ZT_ROLE_RUNNER;
  if (!memcmp(g_initial_holder, self, 6)) role = ZT_ROLE_HOLDER;

  ESP_LOGI(TAG, "Role assignment: self=%02X:%02X:%02X:%02X:%02X:%02X, initial_holder=%02X:%02X:%02X:%02X:%02X:%02X, role=%s", 
           self[0], self[1], self[2], self[3], self[4], self[5],
           g_initial_holder[0], g_initial_holder[1], g_initial_holder[2], g_initial_holder[3], g_initial_holder[4], g_initial_holder[5],
           (role == ZT_ROLE_HOLDER) ? "HOLDER" : "RUNNER");

  if (out_initial_role) *out_initial_role = role;
  if (out_roster) *out_roster = g_roster;

  // keep our recv handler in place so late joiners are ignored; Mode A/B handlers still run via chaining
}

void zt_set_countdown_cb(zt_countdown_render_cb_t cb) { g_cd_cb = cb; }
void zt_get_initial_holder(uint8_t out_mac[6]) { if (out_mac) memcpy(out_mac, g_initial_holder, 6); }

// ---------------- Distinct-before-repeat (holder policy) ----------------
static uint32_t g_seen_bits = 0;  // bit i == player i has held this round
static int      g_seen_count = 0;

static int idx_of(const uint8_t mac[6]) {
  for (int i=0;i<g_roster.count;i++) if (!memcmp(g_roster.macs[i], mac, 6)) return i;
  return -1;
}

static int alive_count_now(void){
  int n=0; uint64_t now=esp_timer_get_time();
  for (int i=0;i<g_roster.count;i++) if (g_last_hello_us[i] && (now - g_last_hello_us[i] <= g_alive_timeout_us)) n++;
  if (n==0) n=1; // at least self
  return n;
}

void zt_distinct_init(const zt_roster_t* roster, const uint8_t* current_holder_mac) {
  if (roster) g_roster = *roster;
  g_seen_bits = 0; g_seen_count = 0;
  int idx = idx_of(current_holder_mac);
  if (idx >= 0) { g_seen_bits |= (1u << idx); g_seen_count = 1; }
}

bool zt_is_candidate_allowed(const uint8_t* candidate_mac) {
  int alive = alive_count_now();
  if (alive <= 2) return true; // "unless only 2 ztaggers…"
  int idx = idx_of(candidate_mac);
  if (idx < 0) return false; // unknown
  if (g_seen_count < alive) {
    // Still mid-round: only allow those not yet seen
    return ((g_seen_bits & (1u << idx)) == 0);
  }
  // Round complete (shouldn't really happen because we rotate on record_grant)
  return true;
}

void zt_record_grant(const uint8_t* new_holder_mac) {
  int idx = idx_of(new_holder_mac);
  if (idx < 0) return;
  if ((g_seen_bits & (1u << idx)) == 0) {
    g_seen_bits |= (1u << idx);
    g_seen_count++;
  }
  if (g_seen_count >= alive_count_now()) {
    // Start a new round: clear all and mark current holder as first in the new round
    g_seen_bits = (1u << idx);
    g_seen_count = 1;
  }
}

void zt_set_alive_timeout_ms(uint32_t ms){ g_alive_timeout_us = ms * 1000u; }

void zt_round_maybe_relax(uint32_t empty_windows, const uint8_t* current_holder_mac){
  // If we've had too many empty windows in a row, reset the round to unblock
  const uint32_t THRESH = 6; // windows
  if (empty_windows < THRESH) return;
  int idx = idx_of(current_holder_mac);
  g_seen_bits = (idx>=0) ? (1u<<idx) : 0;
  g_seen_count = (idx>=0) ? 1 : 0;
  ESP_LOGW(TAG, "Round relaxed after %lu empty windows; keeping only current holder as seen", (unsigned long)empty_windows);
}

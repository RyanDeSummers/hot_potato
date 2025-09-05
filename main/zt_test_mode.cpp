#include "zt_test_mode.h"
#include <string.h>
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Constants needed for ESP-NOW peer setup
#define ESPNOW_CHANNEL 1
#define WIFI_IF_STA WIFI_IF_STA
#include "driver/gpio.h"

// ============================================================================
// CONSTANTS AND CONFIGURATION
// ============================================================================

static const char* TAG = "ZT_TEST";

// Timing constants
static const uint32_t BUTTON_POLL_INTERVAL_MS = 50;      // 50ms button polling interval
static const uint32_t JOIN_WINDOW_POLL_INTERVAL_MS = 200; // 200ms during join window
static const uint32_t PEER_WAIT_INTERVAL_MS = 500;       // 500ms between peer wait intervals
static const uint32_t COUNTDOWN_POLL_INTERVAL_MS = 250;  // 250ms countdown update interval

// Game logic constants
static const uint32_t ROUND_RELAX_THRESHOLD = 0xFFFFFFFF; // Disable round relaxation (effectively infinite)
static const uint64_t ZT_LIVENESS_TMO_US = 30000000; // 30 seconds liveness timeout

// Hash constants for deterministic holder selection
static const uint32_t HASH_INITIAL = 2166136261u;
static const uint32_t HASH_MULTIPLIER = 16777619u;

/**
 * @brief Simple CRC-16 (poly 0x1021) over bytes for roster integrity checking
 * 
 * @param d Data buffer to checksum
 * @param n Number of bytes to checksum
 * @return uint16_t CRC-16 checksum value
 * 
 * Implements CRC-16-CCITT polynomial 0x1021 for roster data integrity.
 * Used to detect roster corruption or tampering during transmission.
 */
static uint16_t crc16_ccitt(const uint8_t* d, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= (uint16_t)d[i] << 8;
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
  }
  return crc;
}

// ============================================================================
// GLOBAL STATE VARIABLES
// ============================================================================

// Device roster management
zt_roster_t g_roster = {};  // Non-static for access from other files

// Game state tracking
static bool g_started = false;
static bool g_host_announced = false;  // New flag for host announcement
static bool g_have_roster = false;
static bool g_join_closed = false;     // Flag to freeze roster after join window
static uint8_t g_initial_holder[6] = {0};
static uint64_t g_T0_us = 0;

// Callback and chaining
static zt_countdown_render_cb_t g_cd_cb = nullptr;
static esp_now_recv_cb_t g_prev_recv = nullptr;

// Distinct-before-repeat game logic (centralized here for both uplink/downlink)
static uint32_t g_seen_bits = 0;  // Bit i == player i has held this round
static int      g_seen_count = 0;
int             g_last_granted_idx = -1;  // Global last granted index for fair sweep (non-static for external access)

// Liveness tracking
static uint64_t g_last_heard_us[ZT_MAX_PEERS];

// Roster versioning and synchronization
static uint32_t g_roster_version = 0;
static uint16_t g_roster_checksum = 0;

// ============================================================================
// ROSTER MANAGEMENT FUNCTIONS
// ============================================================================

/**
 * @brief Checks if a MAC address is already in the roster
 * 
 * @param mac MAC address to check (6 bytes)
 * @return true if MAC is found in roster, false otherwise
 * 
 * Linear search through roster for exact MAC match.
 * Used to prevent duplicate entries and track device presence.
 */
static bool roster_has(const uint8_t mac[6]) {
  if (!mac) return false;
  
  for (int i = 0; i < g_roster.count; i++) {
    if (!memcmp(g_roster.macs[i], mac, 6)) {
      return true;
    }
  }
  return false;
}

/**
 * @brief Adds a new MAC address to the roster if not already present
 * 
 * @param mac MAC address to add (6 bytes)
 * 
 * Adds MAC to roster if:
 * - Roster has space available
 * - MAC is not already present
 * Updates last seen timestamp for the device.
 */
static void roster_add(const uint8_t mac[6]) {
  if (!mac || g_roster.count >= ZT_MAX_PEERS) {
    return;
  }
  
  // Only add if not already present
  if (!roster_has(mac)) {
    memcpy(g_roster.macs[g_roster.count], mac, 6);
    g_roster.count++;
  }
}

/**
 * @brief Sorts roster by MAC address for deterministic ordering
 * 
 * Uses bubble sort to arrange MAC addresses in lexicographic order.
 * This ensures consistent leader selection across all devices.
 * Called before selecting initial holder to guarantee determinism.
 */
static void roster_sort() {
  for (int i = 0; i < g_roster.count; i++) {
    for (int j = i + 1; j < g_roster.count; j++) {
      if (memcmp(g_roster.macs[i], g_roster.macs[j], 6) > 0) {
        // Swap MAC addresses
        uint8_t tmp[6];
        memcpy(tmp, g_roster.macs[i], 6);
        memcpy(g_roster.macs[i], g_roster.macs[j], 6);
        memcpy(g_roster.macs[j], tmp, 6);
      }
    }
  }
}

/**
 * @brief Sorts roster and selects the leader (lowest MAC address)
 * 
 * @param out_leader Output buffer for leader's MAC address (6 bytes)
 * 
 * Sorts roster lexicographically and copies the lowest MAC address
 * to out_leader. This ensures all devices select the same leader.
 */
static void sort_and_pick_leader(uint8_t out_leader[6]) {
  if (!out_leader) return;
  
  roster_sort();
  memcpy(out_leader, g_roster.macs[0], 6); // Lowest MAC = leader
}

// temporary recv handler plumbing

// ============================================================================
// COMMUNICATION FUNCTIONS
// ============================================================================

// Note: send_hello function removed - liveness tracking disabled for simplicity

/**
 * @brief Checks if ESP-NOW is ready for communication
 * 
 * @return true if ESP-NOW is initialized and ready, false otherwise
 * 
 * Verifies that the ESP-NOW system is properly initialized
 * before attempting to send messages.
 */
static bool espnow_is_ready() {
  // Basic check - could be enhanced with more sophisticated validation
  return true; // ESP-NOW should be ready by the time this is called
}

// Note: HELLO timer callback removed - liveness tracking disabled for simplicity

/**
 * @brief Sends roster and start messages as the host device
 * 
 * This function performs two critical operations:
 * 1. Broadcasts the complete device roster to all peers
 * 2. Sends the START message with initial holder and countdown timing
 * 
 * Called by the host after the join window closes to synchronize all devices.
 */
static void send_roster_and_start_as_host() {
  roster_sort(); // Make roster order deterministic before hashing/sending
  
  uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  
  // Build sorted roster bytes first (already sorted above)
  const int roster_bytes = g_roster.count * 6;
  uint8_t roster_body[ZT_MAX_PEERS * 6] = {0};
  for (int i = 0; i < g_roster.count; i++) {
    memcpy(&roster_body[i * 6], g_roster.macs[i], 6);
  }

  // Bump version and compute checksum over the MAC list
  g_roster_version++;
  g_roster_checksum = crc16_ccitt(roster_body, roster_bytes);

  // ROSTER: [MSG][COUNT][VER(4)][CHK(2)][MAC1..MACN]
  uint8_t roster_buffer[1 + 1 + 4 + 2 + ZT_MAX_PEERS * 6] = {0};
  roster_buffer[0] = ZT_MSG_ROSTER;
  roster_buffer[1] = g_roster.count;
  roster_buffer[2] = (uint8_t)(g_roster_version & 0xFF);
  roster_buffer[3] = (uint8_t)((g_roster_version >> 8) & 0xFF);
  roster_buffer[4] = (uint8_t)((g_roster_version >> 16) & 0xFF);
  roster_buffer[5] = (uint8_t)((g_roster_version >> 24) & 0xFF);
  roster_buffer[6] = (uint8_t)(g_roster_checksum & 0xFF);
  roster_buffer[7] = (uint8_t)((g_roster_checksum >> 8) & 0xFF);
  memcpy(&roster_buffer[8], roster_body, roster_bytes);

  esp_err_t err = esp_now_send(broadcast_mac, roster_buffer, 8 + roster_bytes);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to send ROSTER message: %s", esp_err_to_name(err));
    return; // Cannot proceed without roster broadcast
  }

  // Choose initial holder deterministically using hash of roster bytes
  uint32_t hash = HASH_INITIAL;
  for (int i = 0; i < g_roster.count; i++) {
    for (int b = 0; b < 6; b++) {
      hash ^= g_roster.macs[i][b];
      hash *= HASH_MULTIPLIER;
    }
  }
  
  uint8_t holder_index = (g_roster.count > 0) ? (hash % g_roster.count) : 0;
  memcpy(g_initial_holder, g_roster.macs[holder_index], 6);

  // Send START message: [MSG_TYPE][INITIAL_HOLDER_MAC][DELTA_US][ROSTER_VER(4)]
  uint32_t delta_us = (uint32_t)(ZT_COUNTDOWN_S * 1000000u);
  g_T0_us = esp_timer_get_time() + delta_us; // Local copy for our own countdown/logs
  
  // START: [MSG][INITIAL_HOLDER(6)][DELTA_US(4 LE)][ROSTER_VER(4 LE)]
  uint8_t start_buffer[1 + 6 + 4 + 4] = {0};
  start_buffer[0] = ZT_MSG_START;
  memcpy(&start_buffer[1], g_initial_holder, 6);
  start_buffer[7]  = (uint8_t)(delta_us & 0xFF);
  start_buffer[8]  = (uint8_t)((delta_us >> 8) & 0xFF);
  start_buffer[9]  = (uint8_t)((delta_us >> 16) & 0xFF);
  start_buffer[10] = (uint8_t)((delta_us >> 24) & 0xFF);
  start_buffer[11] = (uint8_t)(g_roster_version & 0xFF);
  start_buffer[12] = (uint8_t)((g_roster_version >> 8) & 0xFF);
  start_buffer[13] = (uint8_t)((g_roster_version >> 16) & 0xFF);
  start_buffer[14] = (uint8_t)((g_roster_version >> 24) & 0xFF);
  
  err = esp_now_send(broadcast_mac, start_buffer, sizeof(start_buffer));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to send START message: %s", esp_err_to_name(err));
    return;
  }
  
  ESP_LOGI(TAG, "Host sent ROSTER(count=%d, ver=%u, chk=0x%04X) and START (delta=%u us, holder=%02X:%02X:%02X:%02X:%02X:%02X, ver=%u)", 
           g_roster.count, (unsigned)g_roster_version, (unsigned)g_roster_checksum, (unsigned)delta_us,
           g_initial_holder[0], g_initial_holder[1], g_initial_holder[2],
           g_initial_holder[3], g_initial_holder[4], g_initial_holder[5], (unsigned)g_roster_version);
}

// ============================================================================
// MESSAGE RECEPTION AND PROCESSING
// ============================================================================

/**
 * @brief Processes incoming ESP-NOW messages for test harness functionality
 * 
 * @param info ESP-NOW receive information including source MAC
 * @param data Message payload data
 * @param len Length of message data
 * 
 * Handles three message types:
 * - HELLO: Device discovery and liveness tracking
 * - ROSTER: Complete device list from host
 * - START: Game start signal with timing and initial holder
 * 
 * Chains to previous callback handler to maintain game mode functionality.
 */
static void test_recv_cb(const esp_now_recv_info* info, const uint8_t* data, int len) {
  // Validate input parameters
  if (!info || !data || len <= 0) {
    if (g_prev_recv) g_prev_recv(info, data, len);
    return;
  }
  
  // Debug: log all incoming messages to see what we're receiving
  ESP_LOGI(TAG, "DEBUG: Received message type 0x%02X from %02X:%02X:%02X:%02X:%02X:%02X, len=%d", 
           data[0], info->src_addr[0], info->src_addr[1], info->src_addr[2], 
           info->src_addr[3], info->src_addr[4], info->src_addr[5], len);
  
  switch (data[0]) {
    case ZT_MSG_HELLO: {
      // HELLO messages no longer used for roster building
      // Countdown system will automatically build roster
      ESP_LOGD(TAG, "HELLO from %02X:%02X:%02X:%02X:%02X:%02X - ignored (using countdown roster)", 
               info->src_addr[0], info->src_addr[1], info->src_addr[2], 
               info->src_addr[3], info->src_addr[4], info->src_addr[5]);
      break;
    }
    
    case ZT_MSG_ANNOUNCE: {
      // ANNOUNCE message: host is ready for countdown
      ESP_LOGI(TAG, "ANNOUNCE from %02X:%02X:%02X:%02X:%02X:%02X - host ready for countdown!", 
               info->src_addr[0], info->src_addr[1], info->src_addr[2], 
               info->src_addr[3], info->src_addr[4], info->src_addr[5]);
      g_host_announced = true;  // Set flag so peer knows to send JOIN
      break;
    }
    
    case ZT_MSG_JOIN: {
      // JOIN message: peer wants to join countdown
      if (g_join_closed) {
        ESP_LOGD(TAG, "JOIN from %02X:%02X:%02X:%02X:%02X:%02X - ignored (join window closed)", 
                 info->src_addr[0], info->src_addr[1], info->src_addr[2], 
                 info->src_addr[3], info->src_addr[4], info->src_addr[5]);
        break;
      }
      
      int old_count = g_roster.count;
      roster_add(info->src_addr);
      
      // Update liveness for this peer
      zt_note_heard_mac(info->src_addr);
      
      if (g_roster.count > old_count) {
        ESP_LOGI(TAG, "JOIN from %02X:%02X:%02X:%02X:%02X:%02X - NEW PEER ADDED (roster: %d -> %d)", 
                 info->src_addr[0], info->src_addr[1], info->src_addr[2], 
                 info->src_addr[3], info->src_addr[4], info->src_addr[5],
                 old_count, g_roster.count);
      } else {
        // Already in roster - log at DEBUG to avoid spam during re-JOINs
        ESP_LOGD(TAG, "JOIN from %02X:%02X:%02X:%02X:%02X:%02X - already in roster (count: %d)", 
                 info->src_addr[0], info->src_addr[1], info->src_addr[2], 
                 info->src_addr[3], info->src_addr[4], info->src_addr[5],
                 g_roster.count);
      }
      break;
    }
    
    case ZT_MSG_ROSTER: {
      if (len < 1 + 1 + 4 + 2) { ESP_LOGW(TAG, "ROSTER too short: %d", len); break; }
      uint8_t count = data[1];
      if (count > ZT_MAX_PEERS) { ESP_LOGW(TAG, "ROSTER count %d > max, trunc", count); count = ZT_MAX_PEERS; }

      uint32_t ver = (uint32_t)data[2] | ((uint32_t)data[3] << 8)
                   | ((uint32_t)data[4] << 16) | ((uint32_t)data[5] << 24);
      uint16_t chk = (uint16_t)data[6] | ((uint16_t)data[7] << 8);

      const int need = 1 + 1 + 4 + 2 + count*6;
      if (len < need) { ESP_LOGW(TAG, "ROSTER len %d < need %d", len, need); break; }

      // Validate checksum
      uint16_t calc = crc16_ccitt(&data[8], count*6);
      if (calc != chk) { ESP_LOGW(TAG, "ROSTER checksum mismatch calc=%04X got=%04X", calc, chk); break; }

      // Only upgrade
      if (ver <= g_roster_version) { ESP_LOGD(TAG, "ROSTER ver %u <= local %u, ignore", (unsigned)ver, (unsigned)g_roster_version); break; }

      // Apply roster
      g_roster.count = count;
      for (int i = 0; i < g_roster.count; i++) {
        memcpy(g_roster.macs[i], &data[8 + i*6], 6);
      }
      g_roster_version = ver;
      g_roster_checksum = chk;
      g_have_roster = true;
      ESP_LOGI(TAG, "ROSTER applied: count=%d ver=%u", g_roster.count, (unsigned)g_roster_version);
      break;
    }
    
    case ZT_MSG_START: {
      if (len < 1 + 6 + 4) { ESP_LOGW(TAG, "START too short: %d", len); break; }
      memcpy(g_initial_holder, &data[1], 6);
      uint32_t delta_us = (uint32_t)data[7] | ((uint32_t)data[8] << 8) |
                          ((uint32_t)data[9] << 16) | ((uint32_t)data[10] << 24);
      uint32_t ver = 0;
      if (len >= 1 + 6 + 4 + 4) {
        ver = (uint32_t)data[11] | ((uint32_t)data[12] << 8) |
              ((uint32_t)data[13] << 16) | ((uint32_t)data[14] << 24);
      }
      if (ver && ver < g_roster_version) {
        ESP_LOGW(TAG, "START roster ver %u < local %u; ignoring", (unsigned)ver, (unsigned)g_roster_version);
        break;
      }
      if (ver && ver > g_roster_version) {
        ESP_LOGW(TAG, "START bumps roster ver %u -> %u (no ROSTER?)", (unsigned)g_roster_version, (unsigned)ver);
        g_roster_version = ver; // allow START to bump forward if needed
      }
      g_T0_us = esp_timer_get_time() + delta_us;
      g_started = true;
      ESP_LOGI(TAG, "START: holder=%02X:%02X:%02X:%02X:%02X:%02X delta=%u ver=%u",
               g_initial_holder[0],g_initial_holder[1],g_initial_holder[2],
               g_initial_holder[3],g_initial_holder[4],g_initial_holder[5],
               (unsigned)delta_us, (unsigned)g_roster_version);
      break;
    }
    
    default:
      // Unknown message type - ignore
      break;
  }
  
  // Note: No callback chaining - we fully replace the callback
  // Mode A/B handlers work via the test_recv_cb switch statement
}

// ============================================================================
// HOST MODE FUNCTIONS
// ============================================================================

/**
 * @brief Configures GPIO button for host mode operation
 * 
 * Sets up the button GPIO pin for input with appropriate pull-up/down configuration.
 * M5Stack FIRE uses GPIO 39 which has no internal pull resistors.
 */
static void setup_host_button() {
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << ZT_HOST_BUTTON_GPIO),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,   // GPIO 34-39 have no internal pulls; board has external PU
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };
  
  esp_err_t err = gpio_config(&io_conf);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure host button GPIO: %s", esp_err_to_name(err));
  }
}

/**
 * @brief Waits for host button press to start the game
 * 
 * Polls the button GPIO until pressed (active low for M5Stack FIRE).
 * Provides visual feedback through logs during waiting.
 */
static void wait_for_host_button() {
  ESP_LOGI(TAG, "HOST MODE: Waiting for button press on GPIO %d...", ZT_HOST_BUTTON_GPIO);
  
  while (gpio_get_level(ZT_HOST_BUTTON_GPIO) == 1) {
    vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_INTERVAL_MS));
  }
  
  ESP_LOGI(TAG, "Button pressed! Starting game...");
}

/**
 * @brief Runs the join window to collect peer devices
 * 
 * Sends HELLO messages periodically for ZT_JOIN_MS duration
 * to allow peer devices to discover and join the game.
 */
static void run_join_window() {
  ESP_LOGI(TAG, "Starting %d ms join window to collect peers...", ZT_JOIN_MS);
  
  int64_t end_time = esp_timer_get_time() + (int64_t)ZT_JOIN_MS * 1000;
  int initial_roster_count = g_roster.count;
  const uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  
  // Broadcast ANNOUNCE every ~200ms to tell peers to JOIN
  while (esp_timer_get_time() < end_time) { 
    uint8_t ann[1] = {ZT_MSG_ANNOUNCE};
    esp_now_send(bcast, ann, sizeof(ann));  // Tell peers to JOIN
    vTaskDelay(pdMS_TO_TICKS(JOIN_WINDOW_POLL_INTERVAL_MS)); 
  }
  
  ESP_LOGI(TAG, "Join window complete. Roster: %d -> %d peers.", 
           initial_roster_count, g_roster.count);
  
  // Remove the known_peers backfill entirely (see Fix C)
  // Note: roster_sort() called in send_roster_and_start_as_host() before sending
  
  // Freeze roster - no more JOINs accepted after this point
  g_join_closed = true;
  
  ESP_LOGI(TAG, "Final roster for countdown: %d devices", g_roster.count);
}

// ============================================================================
// PEER MODE FUNCTIONS
// ============================================================================

/**
 * @brief Waits for host to start the game
 * 
 * Sends periodic HELLO messages while waiting for the host's START signal.
 * Continues until g_started flag is set by receiving START message.
 */
// Note: wait_for_host_start() function removed - no longer used
// Peers now wait for ANNOUNCE and send JOIN responses instead

// ============================================================================
// COUNTDOWN AND ROLE ASSIGNMENT FUNCTIONS
// ============================================================================

/**
 * @brief Runs the synchronized countdown across all devices
 * 
 * Displays countdown with optional LCD callback and logs progress.
 * Blocks until countdown reaches zero.
 */
static void run_synchronized_countdown() {
  ESP_LOGI(TAG, "Starting synchronized countdown...");
  
  int last_seconds = -1;
  for (;;) {
    int64_t now = esp_timer_get_time();
    if (now >= (int64_t)g_T0_us) break;
    
    int seconds_remaining = (int)((g_T0_us - now + 999999) / 1000000);
    if (seconds_remaining != last_seconds) {
      // Update display if callback is registered
      if (g_cd_cb) {
        g_cd_cb(seconds_remaining, g_initial_holder, &g_roster);
      }
      
      ESP_LOGI(TAG, "Start in %d...", seconds_remaining);
      last_seconds = seconds_remaining;
    }
    
    vTaskDelay(pdMS_TO_TICKS(COUNTDOWN_POLL_INTERVAL_MS));
  }
  
  ESP_LOGI(TAG, "GO!");
  
  // Notify all devices that the countdown is complete
  extern bool g_game_started; // from ir_uplink_test.cpp
  g_game_started = true;
}

// Note: Liveness tracking functions removed - disabled for simplicity

/**
 * @brief Determines the initial role for this device
 * 
 * @param self_mac This device's MAC address
 * @return zt_role_t The assigned role (HOLDER or RUNNER)
 * 
 * Compares this device's MAC with the initial holder MAC
 * to determine if we start as HOLDER or RUNNER.
 */
static zt_role_t determine_initial_role(const uint8_t* self_mac) {
  if (!self_mac) return ZT_ROLE_RUNNER;
  
  zt_role_t role = ZT_ROLE_RUNNER;
  if (!memcmp(g_initial_holder, self_mac, 6)) {
    role = ZT_ROLE_HOLDER;
  }
  
  return role;
}

/**
 * @brief Logs the role assignment for debugging purposes
 * 
 * @param self_mac This device's MAC address
 * @param role The assigned role
 * 
 * Provides detailed logging of role assignment including
 * MAC addresses and final role determination.
 */
static void log_role_assignment(const uint8_t* self_mac, zt_role_t role) {
  if (!self_mac) return;
  
  ESP_LOGI(TAG, "Role assignment: self=%02X:%02X:%02X:%02X:%02X:%02X, initial_holder=%02X:%02X:%02X:%02X:%02X:%02X, role=%s", 
           self_mac[0], self_mac[1], self_mac[2], self_mac[3], self_mac[4], self_mac[5],
           g_initial_holder[0], g_initial_holder[1], g_initial_holder[2], 
           g_initial_holder[3], g_initial_holder[4], g_initial_holder[5],
           (role == ZT_ROLE_HOLDER) ? "HOLDER" : "RUNNER");
}

// ============================================================================
// MAIN COUNTDOWN FUNCTION
// ============================================================================

/**
 * @brief Main function for device discovery and synchronized countdown
 * 
 * @param out_initial_role Output parameter for assigned initial role
 * @param out_roster Output parameter for device roster
 * @param is_host true if this device is the host, false if peer
 * 
 * This function orchestrates the complete startup sequence:
 * 1. Device discovery and roster building
 * 2. Synchronized countdown across all devices
 * 3. Initial role assignment (HOLDER or RUNNER)
 * 
 * Blocks until countdown completes and role is assigned.
 */
void zt_join_and_countdown(zt_role_t* out_initial_role, zt_roster_t* out_roster, bool is_host) {
  // Read this device's MAC address and add to roster
  uint8_t self_mac[6];
  esp_err_t err = esp_read_mac(self_mac, ESP_MAC_WIFI_STA);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read MAC address: %s", esp_err_to_name(err));
    return;
  }
  
  roster_add(self_mac);  // Use safe path with bounds and de-dup

  // Ensure broadcast peer exists for ESP-NOW communication
  uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_peer_info_t bcast_peer = {};
  memcpy(bcast_peer.peer_addr, broadcast_mac, 6);
  bcast_peer.ifidx = WIFI_IF_STA;
  bcast_peer.channel = ESPNOW_CHANNEL;
  bcast_peer.encrypt = false;
  
  esp_err_t peer_err = esp_now_add_peer(&bcast_peer);
  if (peer_err != ESP_OK && peer_err != ESP_ERR_ESPNOW_EXIST) {
    ESP_LOGW(TAG, "Failed to add broadcast peer: %s", esp_err_to_name(peer_err));
  }
  
  // Hook our receive callback (fully replace previous one)
  // Note: We're not chaining callbacks - this fully replaces any existing callback
  // Mode A/B handlers will still work via the test_recv_cb switch statement
  esp_now_register_recv_cb(test_recv_cb);

  if (is_host) {
    // HOST: Set up button and wait for press
    setup_host_button();
    
    // Wait for button press to start countdown
    ESP_LOGI(TAG, "HOST: Waiting for button press to start countdown...");
    wait_for_host_button();
    
    // Brief join window to collect peers
    run_join_window();
    
      // Self in roster sanity check
  if (!roster_has(self_mac)) {
    ESP_LOGE(TAG, "Host not in roster! Adding self...");
    roster_add(self_mac);
  }
  
  // Send ROSTER and START as host
  ESP_LOGI(TAG, "Host: sending ROSTER and START at time %lld us", (long long)esp_timer_get_time());
  send_roster_and_start_as_host();
    
  } else {
    // PEER: Wait for host announcement, then send JOIN response
    ESP_LOGI(TAG, "PEER MODE: Waiting for host announcement...");
    
    // Wait for host's announcement signal
    while (!g_host_announced) {
      vTaskDelay(pdMS_TO_TICKS(200)); // Check every 200ms
    }
    
    // Host announced countdown, send JOIN response
    ESP_LOGI(TAG, "PEER: Host announced countdown! Sending JOIN response...");
    const uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    
    while (!g_started) {
      uint8_t join_msg[1] = {ZT_MSG_JOIN};
      esp_now_send(bcast, join_msg, sizeof(join_msg));  // Actually JOIN the roster
      vTaskDelay(pdMS_TO_TICKS(PEER_WAIT_INTERVAL_MS));
    }
    
    ESP_LOGI(TAG, "PEER: Received START message from host!");
  }

  // Ensure roster presence on peers before countdown
  if (!is_host && !g_have_roster) {
    ESP_LOGW(TAG, "No ROSTER yet; waiting up to 3 seconds...");
    int tries = 0;
    while (!g_have_roster && tries++ < 30) vTaskDelay(pdMS_TO_TICKS(100)); // 3 seconds total
    if (!g_have_roster) ESP_LOGE(TAG, "Starting without roster; distinct-before-repeat will be wrong");
  }
  
  // Run synchronized countdown across all devices
  run_synchronized_countdown();
  
  // Note: Liveness tracking disabled for simplicity
  
  // Determine and assign initial role
  zt_role_t role = determine_initial_role(self_mac);
  
  // Log role assignment for debugging
  log_role_assignment(self_mac, role);
  
  // Return results to caller
  if (out_initial_role) *out_initial_role = role;
  if (out_roster) *out_roster = g_roster;

  // Keep our receive handler in place so late joiners are ignored
  // Mode A/B handlers still run via chaining
}



// ============================================================================
// COUNTDOWN ROSTER BUILDING FUNCTIONS
// ============================================================================

/**
 * @brief Allows a device to join the countdown and be added to the roster
 * 
 * @param device_mac MAC address of the device joining countdown
 * 
 * This function is called by devices that want to participate in the countdown.
 * It automatically adds them to the roster if they're not already present.
 * This replaces the old HELLO-based roster building system.
 */
void zt_join_countdown(const uint8_t* device_mac) {
  if (!device_mac) return;
  
  int old_count = g_roster.count;
  roster_add(device_mac);
  
  if (g_roster.count > old_count) {
    ESP_LOGI(TAG, "Device %02X:%02X:%02X:%02X:%02X:%02X joined countdown (roster: %d -> %d)", 
             device_mac[0], device_mac[1], device_mac[2], 
             device_mac[3], device_mac[4], device_mac[5],
             old_count, g_roster.count);
  } else {
    ESP_LOGD(TAG, "Device %02X:%02X:%02X:%02X:%02X:%02X already in countdown roster (count: %d)", 
             device_mac[0], device_mac[1], device_mac[2], 
             device_mac[3], device_mac[4], device_mac[5],
             g_roster.count);
  }
}

// ============================================================================
// PUBLIC API FUNCTIONS
// ============================================================================

/**
 * @brief Sets the countdown display callback function
 * 
 * @param cb Function pointer for countdown rendering
 * 
 * Registers a callback that will be called during countdown to update
 * the display with current countdown status and device information.
 */
void zt_set_countdown_cb(zt_countdown_render_cb_t cb) {
  g_cd_cb = cb;
}

/**
 * @brief Retrieves the initial holder's MAC address
 * 
 * @param out_mac Output buffer for the initial holder's MAC (6 bytes)
 * 
 * Copies the MAC address of the device selected as initial holder
 * to the provided output buffer.
 */
void zt_get_initial_holder(uint8_t out_mac[6]) {
  if (out_mac) {
    memcpy(out_mac, g_initial_holder, 6);
  }
}

// ============================================================================
// DISTINCT-BEFORE-REPEAT GAME LOGIC
// ============================================================================

/**
 * @brief Finds the index of a MAC address in the roster
 * 
 * @param mac MAC address to search for (6 bytes)
 * @return int Index in roster (0-based) or -1 if not found
 * 
 * Linear search through roster to find matching MAC address.
 * Used for tracking which devices have held the potato in each round.
 */
static int idx_of(const uint8_t mac[6]) {
  if (!mac) return -1;
  
  for (int i = 0; i < g_roster.count; i++) {
    if (!memcmp(g_roster.macs[i], mac, 6)) {
      return i;
    }
  }
  return -1;
}

/**
 * @brief Counts currently alive devices based on recent HELLO messages
 * 
 * @return int Number of devices that have sent HELLO within timeout period
 * 
 * Checks last_hello_us timestamps against current time and alive_timeout_us.
 * Returns at least 1 (self) even if no other devices are responding.
 */
static int alive_count_now(void) {
  int alive_count = zt_alive_count();
  ESP_LOGI(TAG, "Alive count: %d (liveness enabled)", alive_count);
  return alive_count;
}

/**
 * @brief Initializes the distinct-before-repeat game logic
 * 
 * @param roster Device roster to use for tracking
 * @param current_holder_mac MAC address of the current potato holder
 * 
 * Sets up the game state for a new round, marking the current holder
 * as the first device to have held the potato in this round.
 */
void zt_distinct_init(const zt_roster_t* roster, const uint8_t* current_holder_mac) {
  if (roster) {
    g_roster = *roster;
  }
  
  // Initialize liveness tracking for all peers
  zt_liveness_init_all(zt_now_us());
  
  // Reset round tracking
  g_seen_bits = 0;
  g_seen_count = 0;
  
  // Mark current holder as seen in this round
  int idx = idx_of(current_holder_mac);
  if (idx >= 0) {
    g_seen_bits |= (1u << idx);
    g_seen_count = 1;
  }
}

/**
 * @brief Checks if a candidate device is allowed to receive the potato
 * 
 * @param candidate_mac MAC address of the candidate device
 * @return true if candidate is allowed, false otherwise
 * 
 * Implements the "distinct-before-repeat" rule:
 * - If only 2 devices alive: always allow (prevents deadlock)
 * - Otherwise: only allow devices that haven't held the potato this round
 * - Once all alive devices have held it, a new round begins
 */
bool zt_is_candidate_allowed(const uint8_t* candidate_mac) {
  if (!candidate_mac) return false;
  
  int alive_count = alive_count_now();
  int idx = idx_of(candidate_mac);
  
  ESP_LOGI(TAG, "Candidate check: MAC=%02X:%02X:%02X:%02X:%02X:%02X, idx=%d, alive_count=%d, seen_count=%d, seen_bits=0x%08lX", 
           candidate_mac[0], candidate_mac[1], candidate_mac[2], 
           candidate_mac[3], candidate_mac[4], candidate_mac[5], 
           idx, alive_count, g_seen_count, (unsigned long)g_seen_bits);
  
  // Special case: if only 2 devices alive, always allow to prevent deadlock
  if (alive_count <= 2) {
    ESP_LOGI(TAG, "Candidate ALLOWED: only %d devices alive (deadlock prevention)", alive_count);
    return true;
  }
  
  if (idx < 0) {
    ESP_LOGE(TAG, "Candidate REJECTED: device not found in roster (idx=%d)", idx);
    return false; // Unknown device
  }
  
  if (g_seen_count < alive_count) {
    // Still mid-round: only allow those not yet seen
    bool already_seen = ((g_seen_bits & (1u << idx)) != 0);
    if (already_seen) {
      ESP_LOGI(TAG, "Candidate REJECTED: device already held potato this round (idx=%d)", idx);
    } else {
      ESP_LOGI(TAG, "Candidate ALLOWED: device hasn't held potato this round (idx=%d)", idx);
    }
    return !already_seen;
  }
  
  // Round complete (shouldn't really happen because we rotate on record_grant)
  ESP_LOGI(TAG, "Candidate ALLOWED: round complete (seen_count=%d >= alive_count=%d)", g_seen_count, alive_count);
  return true;
}

/**
 * @brief Fair candidate picker that sweeps from last granted index
 * 
 * @param out_idx Output parameter for the selected candidate index
 * @param out_mac Output parameter for the selected candidate MAC
 * @return true if a candidate was found, false otherwise
 * 
 * Picks the next eligible candidate starting from g_last_granted_idx + 1,
 * ensuring fair rotation through the roster.
 */
bool zt_pick_next_candidate(int* out_idx, uint8_t out_mac[6]) {
  const int n = g_roster.count;
  if (n <= 1) return false;

  int start = (g_last_granted_idx >= 0) ? (g_last_granted_idx + 1) % n : 0;
  for (int off = 0; off < n; ++off) {
    int i = (start + off) % n;
    const uint8_t* mac = g_roster.macs[i];
    
    // Get self MAC from the roster (first entry is always self)
    const uint8_t* self_mac = g_roster.macs[0];
    if (memcmp(mac, self_mac, 6) == 0) continue;        // don't pass to self
    if (g_seen_bits & (1u << i)) continue;                 // already seen this round
    
    // Use the existing candidate allowed check which includes liveness and cooldown
    if (!zt_is_candidate_allowed(mac)) continue;
    
    memcpy(out_mac, mac, 6);
    *out_idx = i;
    return true;
  }
  return false;
}

/**
 * @brief Notes that a grant was made to a specific index
 * 
 * @param idx Index of the device that received the grant
 * 
 * Updates the last granted index and marks the device as seen in the current round.
 */
void zt_note_grant_to_idx(int idx) {
  g_last_granted_idx = idx;
  g_seen_bits |= (1u << idx);
}

/**
 * @brief Call this whenever you successfully commit a grant
 * 
 * @param idx Index of the device that received the grant
 * 
 * Updates the last granted index for fair sweep behavior.
 */
void zt_on_grant_committed_idx(int idx) {
  g_last_granted_idx = idx;
}

/**
 * @brief Records that a device has been granted the potato
 * 
 * @param new_holder_mac MAC address of the new potato holder
 * 
 * Updates the round tracking to mark this device as having held the potato.
 * If all alive devices have held it, starts a new round with only the
 * current holder marked as seen.
 */
void zt_record_grant(const uint8_t* new_holder_mac) {
  if (!new_holder_mac) return;
  
  int idx = idx_of(new_holder_mac);
  if (idx < 0) return;
  
  ESP_LOGI(TAG, "Recording grant: MAC=%02X:%02X:%02X:%02X:%02X:%02X, idx=%d, seen_bits=0x%08lX, seen_count=%d", 
           new_holder_mac[0], new_holder_mac[1], new_holder_mac[2], 
           new_holder_mac[3], new_holder_mac[4], new_holder_mac[5], 
           idx, (unsigned long)g_seen_bits, g_seen_count);
  
  // Mark this device as seen in current round if not already
  if ((g_seen_bits & (1u << idx)) == 0) {
    g_seen_bits |= (1u << idx);
    g_seen_count++;
    ESP_LOGI(TAG, "Device marked as seen, new seen_count=%d", g_seen_count);
  } else {
    ESP_LOGI(TAG, "Device already seen this round, seen_count unchanged=%d", g_seen_count);
  }
  
  // Check if round is complete (all alive devices have held the potato)
  int alive_count = alive_count_now();
  if (g_seen_count >= alive_count) {
    // Log the completion BEFORE resetting the counters
    ESP_LOGI(TAG, "ROUND COMPLETE! Starting new round with holder %02X:%02X:%02X:%02X:%02X:%02X (seen_count=%d >= alive_count=%d)",
             new_holder_mac[0], new_holder_mac[1], new_holder_mac[2],
             new_holder_mac[3], new_holder_mac[4], new_holder_mac[5], g_seen_count, alive_count);
    
    // Start a new round: clear all and mark current holder as first in the new round
    g_seen_bits = (1u << idx);
    g_seen_count = 1;
    
    // Reset the fair-rotation pointer to start next sweep after the new holder
    g_last_granted_idx = idx;
    ESP_LOGI(TAG, "FAIR: reset last_granted to new holder idx=%d", g_last_granted_idx);
    
    // Reset unresponsive strike counters for the new round
    zt_round_reset_unresp();
  } else {
    ESP_LOGI(TAG, "Round continues: seen_count=%d < alive_count=%d", g_seen_count, alive_count);
  }
}

/**
 * @brief Makes round completion authoritative and synchronous
 * 
 * Called when a round is detected as complete. This function is called from
 * Mode A/B when they detect round completion to ensure all devices reset
 * their distinct-before-repeat state synchronously.
 */
void zt_round_complete_authoritative(void) {
  // This function is called from Mode A/B when they detect round completion
  // The actual round completion logic is handled in the Mode A/B files
  // This is just a placeholder to maintain the API
  ESP_LOGI(TAG, "Round completion detected by Mode A/B - state will be reset via STATE_SYNC");
}

// Note: zt_set_alive_timeout_ms removed - liveness tracking disabled for simplicity

/**
 * @brief Relaxes round restrictions after too many empty windows
 * 
 * @param empty_windows Number of consecutive empty windows
 * @param current_holder_mac MAC address of the current holder
 * 
 * NOTE: Round relaxation is currently DISABLED (threshold set to effectively infinite)
 * to ensure strict enforcement of distinct-before-repeat rules.
 * 
 * If too many empty windows occur in a row, this function would reset
 * the round to unblock the game, keeping only the current holder
 * as seen in the new round.
 */
void zt_round_maybe_relax(uint32_t empty_windows, const uint8_t* current_holder_mac) {
  if (empty_windows < ROUND_RELAX_THRESHOLD) {
    return;
  }
  
  int idx = idx_of(current_holder_mac);
  g_seen_bits = (idx >= 0) ? (1u << idx) : 0;
  g_seen_count = (idx >= 0) ? 1 : 0;
  
  ESP_LOGW(TAG, "Round relaxed after %lu empty windows; keeping only current holder as seen", 
           (unsigned long)empty_windows);
}

// ============================================================================
// ADDITIONAL HELPER FUNCTIONS FOR STRICT DISTINCT-BEFORE-REPEAT
// ============================================================================

/**
 * @brief Find the index of a MAC address in the roster
 * 
 * @param mac MAC address to find
 * @return int Index in roster, or -1 if not found
 */
int zt_roster_index_of(const uint8_t* mac) {
  for (int i = 0; i < g_roster.count; i++) {
    if (memcmp(g_roster.macs[i], mac, 6) == 0) {
      return i;
    }
  }
  return -1;
}

/**
 * @brief Check if a device has already been seen this round
 * 
 * @param idx Index of the device in the roster
 * @return true if device has already been seen this round
 */
bool zt_distinct_already_seen(int idx) {
  if (idx < 0 || idx >= g_roster.count) return false;
  return (g_seen_bits & (1u << idx)) != 0;
}

/**
 * @brief Mark a device as seen this round
 * 
 * @param idx Index of the device in the roster
 */
void zt_distinct_mark_seen(int idx) {
  if (idx < 0 || idx >= g_roster.count) return;
  if ((g_seen_bits & (1u << idx)) == 0) {
    g_seen_bits |= (1u << idx);
    g_seen_count++;
    ESP_LOGI(TAG, "Device marked as seen: idx=%d, seen_count=%d", idx, g_seen_count);
  }
}


/**
 * @brief Check if cooldown allows transmission for a device
 * 
 * @param mac MAC address of the device
 * @return true if cooldown allows transmission
 */
bool zt_cooldown_ok(const uint8_t* mac) {
  // For now, always allow transmission (cooldown logic can be added later)
  // This ensures unseen devices are never blocked by cooldown
  return true;
}

// ============================================================================
// LIVENESS TRACKING FUNCTIONS
// ============================================================================

/**
 * @brief Initialize liveness tracking for all peers
 * 
 * @param now_us Current timestamp in microseconds
 */
void zt_liveness_init_all(uint64_t now_us) {
  for (int i = 0; i < g_roster.count; ++i) {
    g_last_heard_us[i] = now_us;
  }
}

/**
 * @brief Get current timestamp in microseconds
 * 
 * @return uint64_t Current timestamp
 */
uint64_t zt_now_us() {
  return esp_timer_get_time();
}

/**
 * @brief Note that we heard from a specific MAC address
 * 
 * @param mac MAC address that we heard from
 */
void zt_note_heard_mac(const uint8_t* mac) {
  int i = zt_roster_index_of(mac);
  if (i >= 0) {
    g_last_heard_us[i] = zt_now_us();
  }
}

/**
 * @brief Check if a peer is alive based on liveness timeout
 * 
 * @param i Index of the device in the roster
 * @return true if peer is considered alive
 */
bool zt_peer_alive(int i) {
  if (i < 0 || i >= g_roster.count) return false;
  const uint64_t now = zt_now_us();
  const uint64_t dt = now - g_last_heard_us[i];
  return dt < ZT_LIVENESS_TMO_US;
}

/**
 * @brief Count how many peers are currently alive
 * 
 * @return int Number of alive peers
 */
int zt_alive_count() {
  int n = 0;
  for (int i = 0; i < g_roster.count; ++i) {
    if (zt_peer_alive(i)) ++n;
  }
  return n;
}

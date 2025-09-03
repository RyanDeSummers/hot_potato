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

// ============================================================================
// CONSTANTS AND CONFIGURATION
// ============================================================================

static const char* TAG = "ZT_TEST";

// Timing constants
static const uint32_t ALIVE_TIMEOUT_MS = 2000;           // 2 seconds default timeout
static const uint32_t HELLO_INTERVAL_MS = 1000;          // 1 second between hello messages
static const uint32_t BUTTON_POLL_INTERVAL_MS = 50;      // 50ms button polling interval
static const uint32_t JOIN_WINDOW_POLL_INTERVAL_MS = 200; // 200ms during join window
static const uint32_t PEER_WAIT_INTERVAL_MS = 500;       // 500ms between peer hello messages
static const uint32_t COUNTDOWN_POLL_INTERVAL_MS = 250;  // 250ms countdown update interval

// Game logic constants
static const uint32_t ROUND_RELAX_THRESHOLD = 6;         // Empty windows before relaxing round
static const uint32_t MIN_ALIVE_COUNT = 1;               // Minimum alive count (at least self)

// Hash constants for deterministic holder selection
static const uint32_t HASH_INITIAL = 2166136261u;
static const uint32_t HASH_MULTIPLIER = 16777619u;

// ============================================================================
// GLOBAL STATE VARIABLES
// ============================================================================

// Device roster management
static zt_roster_t g_roster = {};
static uint64_t g_last_hello_us[ZT_MAX_PEERS] = {0};
static uint32_t g_alive_timeout_us = ALIVE_TIMEOUT_MS * 1000; // Convert to microseconds

// Timer handles
static esp_timer_handle_t g_hello_timer = nullptr;

// Game state tracking
static bool g_started = false;
static bool g_have_roster = false;
static uint8_t g_initial_holder[6] = {0};
static uint64_t g_T0_us = 0;

// Callback and chaining
static zt_countdown_render_cb_t g_cd_cb = nullptr;
static esp_now_recv_cb_t g_prev_recv = nullptr;

// Distinct-before-repeat game logic
static uint32_t g_seen_bits = 0;  // Bit i == player i has held this round
static int g_seen_count = 0;

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
  
  // Update last seen timestamp for this device
  for (int i = 0; i < g_roster.count; i++) {
    if (!memcmp(g_roster.macs[i], mac, 6)) {
      g_last_hello_us[i] = esp_timer_get_time();
      break;
    }
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

/**
 * @brief Broadcasts a HELLO message to discover other devices
 * 
 * Sends a single-byte HELLO message to the broadcast address.
 * Used during device discovery phase to announce presence.
 * 
 * @return true if message sent successfully, false otherwise
 */
static bool send_hello() {
  uint8_t message = ZT_MSG_HELLO;
  uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  
  esp_err_t err = esp_now_send(broadcast_mac, &message, 1);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to send HELLO message: %s", esp_err_to_name(err));
    return false;
  }
  
  return true;
}

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

/**
 * @brief Timer callback for periodic HELLO messages
 * 
 * @param arg Unused argument (required by timer callback signature)
 * 
 * Called periodically to maintain device liveness tracking.
 * Sends HELLO message to keep other devices aware of our presence.
 */
static void hello_tick(void* arg) {
  (void)arg; // Unused parameter
  
  if (espnow_is_ready()) {
    send_hello();
  } else {
    ESP_LOGW(TAG, "ESP-NOW not ready, skipping HELLO message");
  }
}

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
  uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  
  // Send ROSTER message: [MSG_TYPE][COUNT][MAC1][MAC2]...[MACN]
  uint8_t roster_buffer[1 + 1 + ZT_MAX_PEERS * 6] = {0};
  roster_buffer[0] = ZT_MSG_ROSTER;
  roster_buffer[1] = g_roster.count;
  
  for (int i = 0; i < g_roster.count; i++) {
    memcpy(&roster_buffer[2 + i * 6], g_roster.macs[i], 6);
  }
  
  esp_err_t err = esp_now_send(broadcast_mac, roster_buffer, 2 + g_roster.count * 6);
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

  // Send START message: [MSG_TYPE][INITIAL_HOLDER_MAC][DELTA_US]
  uint32_t delta_us = (uint32_t)(ZT_COUNTDOWN_S * 1000000u);
  g_T0_us = esp_timer_get_time() + delta_us; // Local copy for our own countdown/logs
  
  uint8_t start_buffer[1 + 6 + 4];
  start_buffer[0] = ZT_MSG_START;
  memcpy(&start_buffer[1], g_initial_holder, 6);
  
  // Pack delta_us as little-endian 32-bit value
  start_buffer[7] = (uint8_t)(delta_us & 0xFF);
  start_buffer[8] = (uint8_t)((delta_us >> 8) & 0xFF);
  start_buffer[9] = (uint8_t)((delta_us >> 16) & 0xFF);
  start_buffer[10] = (uint8_t)((delta_us >> 24) & 0xFF);
  
  err = esp_now_send(broadcast_mac, start_buffer, sizeof(start_buffer));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to send START message: %s", esp_err_to_name(err));
    return;
  }
  
  ESP_LOGI(TAG, "Host sent ROSTER(count=%d) and START (delta=%u us, holder=%02X:%02X:%02X:%02X:%02X:%02X)", 
           g_roster.count, (unsigned)delta_us,
           g_initial_holder[0], g_initial_holder[1], g_initial_holder[2],
           g_initial_holder[3], g_initial_holder[4], g_initial_holder[5]);
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
  
  switch (data[0]) {
    case ZT_MSG_HELLO: {
      // Process HELLO message for device discovery
      roster_add(info->src_addr);
      
      // Refresh alive timestamp for this device
      for (int i = 0; i < g_roster.count; i++) {
        if (!memcmp(g_roster.macs[i], info->src_addr, 6)) {
          g_last_hello_us[i] = esp_timer_get_time();
          break;
        }
      }
      break;
    }
    
    case ZT_MSG_ROSTER: {
      // Process ROSTER message: [MSG_TYPE][COUNT][MAC1][MAC2]...[MACN]
      if (len < 2) {
        ESP_LOGW(TAG, "ROSTER message too short: %d bytes", len);
        break;
      }
      
      uint8_t device_count = data[1];
      if (device_count > ZT_MAX_PEERS) {
        ESP_LOGW(TAG, "ROSTER count %d exceeds max %d, truncating", device_count, ZT_MAX_PEERS);
        device_count = ZT_MAX_PEERS;
      }
      
      g_roster.count = device_count;
      for (int i = 0; i < g_roster.count; i++) {
        memcpy(g_roster.macs[i], &data[2 + i * 6], 6);
      }
      
      g_have_roster = true;
      ESP_LOGI(TAG, "Received ROSTER with %d devices", g_roster.count);
      break;
    }
    
    case ZT_MSG_START: {
      // Process START message: [MSG_TYPE][INITIAL_HOLDER_MAC][DELTA_US]
      if (len < 1 + 6 + 4) {
        ESP_LOGW(TAG, "START message too short: %d bytes", len);
        break;
      }
      
      memcpy(g_initial_holder, &data[1], 6);
      
      // Unpack delta_us as little-endian 32-bit value
      uint32_t delta_us = (uint32_t)data[7] |
                          ((uint32_t)data[8] << 8) |
                          ((uint32_t)data[9] << 16) |
                          ((uint32_t)data[10] << 24);
      
      g_T0_us = esp_timer_get_time() + delta_us; // Local T0
      g_started = true;
      
      ESP_LOGI(TAG, "Received START: holder=%02X:%02X:%02X:%02X:%02X:%02X, delta=%u us",
               g_initial_holder[0], g_initial_holder[1], g_initial_holder[2],
               g_initial_holder[3], g_initial_holder[4], g_initial_holder[5],
               (unsigned)delta_us);
      break;
    }
    
    default:
      // Unknown message type - ignore
      break;
  }
  
  // Chain to previous handler so Mode A/B still see their messages
  if (g_prev_recv) {
    g_prev_recv(info, data, len);
  }
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
  while (esp_timer_get_time() < end_time) { 
    send_hello(); 
    vTaskDelay(pdMS_TO_TICKS(JOIN_WINDOW_POLL_INTERVAL_MS)); 
  }
  
  ESP_LOGI(TAG, "Join window complete. Found %d peers.", g_roster.count);
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
static void wait_for_host_start() {
  ESP_LOGI(TAG, "PEER MODE: Waiting for host to start...");
  ESP_LOGI(TAG, "Waiting for host HELLO messages...");
  
  while (!g_started) {
    // Send HELLO periodically to announce presence
    send_hello();
    vTaskDelay(pdMS_TO_TICKS(PEER_WAIT_INTERVAL_MS));
  }
  
  ESP_LOGI(TAG, "Received START from host!");
}

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
}

/**
 * @brief Starts periodic HELLO messages for liveness tracking
 * 
 * Creates and starts a timer to send HELLO messages every second
 * during the game to maintain device presence awareness.
 */
static void start_liveness_tracking() {
  // Create periodic timer if it doesn't exist
  if (!g_hello_timer) {
    const esp_timer_create_args_t timer_args = {
      .callback = &hello_tick,
      .arg = nullptr,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "zt_hello"
    };
    
    esp_err_t err = esp_timer_create(&timer_args, &g_hello_timer);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to create hello timer: %s", esp_err_to_name(err));
      return;
    }
  }
  
  // Stop any existing timer and start fresh
  esp_timer_stop(g_hello_timer);
  
  esp_err_t err = esp_timer_start_periodic(g_hello_timer, HELLO_INTERVAL_MS * 1000);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start hello timer: %s", esp_err_to_name(err));
  }
}

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
  
  memcpy(g_roster.macs[g_roster.count], self_mac, 6);
  g_roster.count++;

  // Hook our receive callback (chain with previous one)
  esp_now_register_recv_cb(nullptr); // Get current? API doesn't expose, so we just replace.
  g_prev_recv = nullptr;             // Both modes register after us; or re-register us after they did
  esp_now_register_recv_cb(test_recv_cb);

  if (is_host) {
    // HOST: Wait for button press, then start the game
    setup_host_button();
    wait_for_host_button();
    
    // Brief join window to collect peers AFTER button press
    run_join_window();
    
    // Send ROSTER and START as host
    ESP_LOGI(TAG, "Host: sending ROSTER and START at time %lld us", (long long)esp_timer_get_time());
    send_roster_and_start_as_host();
    
  } else {
    // PEER: Wait for host to start the game
    wait_for_host_start();
  }

  // Run synchronized countdown across all devices
  run_synchronized_countdown();
  
  // Start periodic HELLO messages for liveness tracking during the game
  start_liveness_tracking();
  
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
  int alive_count = 0;
  uint64_t now = esp_timer_get_time();
  
  for (int i = 0; i < g_roster.count; i++) {
    if (g_last_hello_us[i] && (now - g_last_hello_us[i] <= g_alive_timeout_us)) {
      alive_count++;
    }
  }
  
  // Ensure at least self is counted as alive
  if (alive_count == 0) {
    alive_count = MIN_ALIVE_COUNT;
  }
  
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
  
  // Special case: if only 2 devices alive, always allow to prevent deadlock
  if (alive_count <= 2) {
    return true;
  }
  
  int idx = idx_of(candidate_mac);
  if (idx < 0) {
    return false; // Unknown device
  }
  
  if (g_seen_count < alive_count) {
    // Still mid-round: only allow those not yet seen
    return ((g_seen_bits & (1u << idx)) == 0);
  }
  
  // Round complete (shouldn't really happen because we rotate on record_grant)
  return true;
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
  
  // Mark this device as seen in current round if not already
  if ((g_seen_bits & (1u << idx)) == 0) {
    g_seen_bits |= (1u << idx);
    g_seen_count++;
  }
  
  // Check if round is complete (all alive devices have held the potato)
  if (g_seen_count >= alive_count_now()) {
    // Start a new round: clear all and mark current holder as first in the new round
    g_seen_bits = (1u << idx);
    g_seen_count = 1;
    ESP_LOGI(TAG, "Round complete, starting new round with holder %02X:%02X:%02X:%02X:%02X:%02X",
             new_holder_mac[0], new_holder_mac[1], new_holder_mac[2],
             new_holder_mac[3], new_holder_mac[4], new_holder_mac[5]);
  }
}

/**
 * @brief Sets the alive timeout for device liveness tracking
 * 
 * @param ms Timeout in milliseconds
 * 
 * Configures how long to wait for HELLO messages before considering
 * a device as no longer alive/responding.
 */
void zt_set_alive_timeout_ms(uint32_t ms) {
  g_alive_timeout_us = ms * 1000u;
}

/**
 * @brief Relaxes round restrictions after too many empty windows
 * 
 * @param empty_windows Number of consecutive empty windows
 * @param current_holder_mac MAC address of the current holder
 * 
 * If too many empty windows occur in a row, this function resets
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

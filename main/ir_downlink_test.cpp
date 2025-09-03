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

#define IR_TX_GPIO   GPIO_NUM_26   // IR LED
#define IR_RX_GPIO   GPIO_NUM_36   // IR Receiver
#define IR_UART_PORT UART_NUM_2  // UART Peripheral (IR Receiver)

// Host/peer selection - same as Mode B
#ifndef ZT_IS_HOST
#define ZT_IS_HOST 0   // build: -DZT_IS_HOST=1 for HOST
#endif

static const char* TAG = "IR_DOWNLINK_TEST";

// ============================================================================
// GAME STATE AND ROLE MANAGEMENT
// ============================================================================

typedef enum { ROLE_HOLDER, ROLE_RUNNER } role_t;
static volatile role_t g_role = ROLE_RUNNER;                    // Current game role
static uint8_t g_self_mac[6] = {0};                            // This device's MAC address

// ============================================================================
// BEACON SEQUENCE MANAGEMENT
// ============================================================================

static uint16_t g_beacon_sequence = 0;                         // Next beacon sequence number to send
static uint16_t g_current_beacon_seq = 0;                      // Most recently sent beacon sequence
static uint16_t g_previous_beacon_seq = 0;                     // Previously sent beacon sequence
static int64_t g_last_beacon_timestamp_us = 0;                 // When the last beacon was transmitted

// ============================================================================
// GAME TIMING AND STATE FLAGS
// ============================================================================

static int64_t g_cooldown_until_timestamp_us = 0;              // No-tagback cooldown timer
static bool g_espnow_ready = false;                            // ESP-NOW initialization status
static bool g_ir_reception_enabled = false;                    // IR reception enable/disable state
static bool g_game_active = false;                             // Game has started and is active
static uint32_t g_empty_window_count = 0;                      // Count of consecutive windows with no eligible candidates

// ============================================================================
// ERROR HANDLING HELPERS
// ============================================================================

/**
 * @brief Logs an error and returns the error code
 * 
 * @param tag Log tag for the error
 * @param operation Description of the operation that failed
 * @param error_code ESP error code
 * @return The error code (for chaining)
 * 
 * This function provides consistent error logging format for non-critical errors
 * that don't require immediate termination.
 */
static esp_err_t log_error(const char* tag, const char* operation, esp_err_t error_code) {
  ESP_LOGE(tag, "%s failed: %s (%s)", operation, esp_err_to_name(error_code), esp_err_to_name(error_code));
  return error_code;
}

/**
 * @brief Logs a warning and continues execution
 * 
 * @param tag Log tag for the warning
 * @param operation Description of the operation that had issues
 * @param warning_code ESP warning code
 * 
 * This function logs warnings for non-critical issues that don't stop execution.
 */
static void log_warning(const char* tag, const char* operation, esp_err_t warning_code) {
  ESP_LOGW(tag, "%s warning: %s (%s)", operation, esp_err_to_name(warning_code), esp_err_to_name(warning_code));
}

/**
 * @brief Safely executes an operation with error logging
 * 
 * @param tag Log tag for the operation
 * @param operation Description of the operation
 * @param func Function to execute
 * @return ESP_OK on success, error code on failure
 * 
 * This function executes an operation and provides consistent error handling.
 * Use for operations where failure is not critical.
 */
#define SAFE_EXECUTE(tag, operation, func) do { \
  esp_err_t err = (func); \
  if (err != ESP_OK) { \
    log_error(tag, operation, err); \
  } \
} while(0)

// ============================================================================
// GAME TIMING CONSTANTS
// ============================================================================

// Accept a PASS_REQ that matches the previous beacon if it arrives within this window.
#ifndef MODEA_ACCEPT_PREV_BEACON_US
#define MODEA_ACCEPT_PREV_BEACON_US 90000   // 90 ms (tune: <= BEACON_PERIOD_MS)
#endif

// Task configuration constants
#define TASK_STACK_SIZE 4096                 // Stack size for background tasks
#define TASK_PRIORITY 4                      // Priority level for game tasks
#define GAME_START_DELAY_MS 100             // Delay after countdown before starting game

// Buffer and memory constants
#define RMT_MEM_BLOCK_SYMBOLS 128           // RMT memory block size for IR transmission
#define IR_SYMBOL_BUFFER_SIZE 200           // Maximum IR symbols per beacon packet
#define UART_RING_BUFFER_SIZE 256           // Ring buffer size for UART reception
#define UART_READ_BUFFER_SIZE 128           // Single read buffer size for UART
#define UART_READ_TIMEOUT_MS 20             // Timeout for UART read operations
#define UART_TASK_DELAY_MS 10               // Delay between UART task iterations

// LED and display constants
#define LED_RED_R 255                        // Red LED color component
#define LED_RED_G 0                          // Green LED color component  
#define LED_RED_B 0                          // Blue LED color component
#define LED_OFF_R 0                          // LED off color component
#define LED_OFF_G 0                          // LED off color component
#define LED_OFF_B 0                          // LED off color component

// RMT transmission constants
#define RMT_TX_WAIT_TIMEOUT_MS 1000         // Timeout for RMT transmission completion
#define RMT_RESOLUTION_HZ 1000000            // RMT resolution (1MHz = 1us per tick)
#define RMT_TX_QUEUE_DEPTH 4                 // RMT transmit queue depth
#define RMT_TX_INTR_PRIORITY 0              // RMT interrupt priority

// IR carrier constants
#define IR_CARRIER_FREQ_HZ 38000            // IR carrier frequency (38kHz)
#define IR_CARRIER_DUTY_CYCLE 0.33          // IR carrier duty cycle (33%)

// Packet structure constants
#define PACKET_HEADER_SIZE 3                // Preamble(2) + Length(1)
#define PACKET_SEQ_SIZE 2                   // Sequence number (2 bytes)
#define PACKET_CRC_SIZE 1                   // CRC-8 checksum (1 byte)
#define PACKET_TOTAL_OVERHEAD (PACKET_HEADER_SIZE + PACKET_SEQ_SIZE + PACKET_CRC_SIZE)
#define EXPECTED_PACKET_LEN (IR_MAC_LEN + PACKET_SEQ_SIZE)  // MAC(6) + SEQ(2)

// LED strip constants
#define LED_STRIP_RESOLUTION_HZ 10000000    // LED strip RMT resolution (10MHz)
#define LED_STRIP_MEM_BLOCK_SYMBOLS 64      // LED strip RMT memory block size
#define LED_STRIP_RESET_DELAY_MS 10         // Delay for WS2812 reset processing

// Display positioning constants
#define DISPLAY_MODE_HEADER_Y 12            // Y position for "MODE A" header
#define DISPLAY_ROLE_Y 40                   // Y position for "ROLE:" text
#define DISPLAY_BANNER_Y 70                 // Y position for role banner
#define DISPLAY_BANNER_HEIGHT 30            // Height of role banner
#define DISPLAY_BANNER_TEXT_Y 76            // Y position for banner text
#define DISPLAY_LEFT_MARGIN 8               // Left margin for text
#define DISPLAY_COUNTDOWN_HEADER_Y_OFFSET -28  // Y offset for "SYNC START" text
#define DISPLAY_COUNTDOWN_NUMBER_Y_OFFSET 8     // Y offset for countdown number
#define DISPLAY_COUNTDOWN_FOOTER_Y_OFFSET 48    // Y offset for MAC address footer

// --- RMT TX (IR) ---
static rmt_channel_handle_t rmt_tx = nullptr;
static rmt_encoder_handle_t tx_copy = nullptr;
static TaskHandle_t  s_beacon_task = nullptr;
static esp_timer_handle_t s_beacon_timer = nullptr;
static void beacon_timer_cb(void*) { if (s_beacon_task) xTaskNotifyGive(s_beacon_task); }

// ---------- UI helpers ----------

/**
 * @brief Updates the display to show current game role and status
 * 
 * This function renders the main gameplay UI showing:
 * - Game mode (MODE A Downlink)
 * - Current role (HOLDER or RUNNER)
 * - Visual banner indicating role status
 * 
 * HOLDER: Red banner with "HOLDER ACTIVE"
 * RUNNER: Green banner with "RUNNER LISTENING"
 * 
 * Called whenever the role changes during gameplay.
 */
static void draw_role_ui() {
  display.fillScreen(TFT_BLACK);
  display.setTextSize(2);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.setCursor(DISPLAY_LEFT_MARGIN, DISPLAY_MODE_HEADER_Y);
  display.printf("MODE A (Downlink)");
  display.setCursor(DISPLAY_LEFT_MARGIN, DISPLAY_ROLE_Y);
  display.printf("ROLE: %s", (g_role==ROLE_HOLDER)?"HOLDER":"RUNNER");
  
  // Simple holder "LED" banner on LCD (replace with your side-LED hook if desired)
  if (g_role == ROLE_HOLDER) {
    display.fillRect(0, DISPLAY_BANNER_Y, display.width(), DISPLAY_BANNER_HEIGHT, TFT_RED);
    display.setCursor(DISPLAY_LEFT_MARGIN, DISPLAY_BANNER_TEXT_Y);
    display.setTextColor(TFT_BLACK, TFT_RED);
    display.printf("HOLDER ACTIVE");
  } else {
    display.fillRect(0, DISPLAY_BANNER_Y, display.width(), DISPLAY_BANNER_HEIGHT, TFT_DARKGREEN);
    display.setCursor(DISPLAY_LEFT_MARGIN, DISPLAY_BANNER_TEXT_Y);
    display.setTextColor(TFT_WHITE, TFT_DARKGREEN);
    display.printf("RUNNER LISTENING");
  }
}

/**
 * @brief Callback function for rendering the synchronized countdown UI
 * 
 * @param secs_remaining Number of seconds left in countdown (3, 2, 1)
 * @param initial_holder_mac MAC address of the device that will start as holder
 * @param roster Pointer to roster information (unused in current implementation)
 * 
 * This function renders the countdown screen that all devices show simultaneously:
 * - "SYNC START" header
 * - Large countdown number in center
 * - MAC address hint at bottom showing initial holder
 * 
 * Called automatically by the test harness during countdown phase.
 */
static void countdown_draw_cb(int secs_remaining, const uint8_t initial_holder_mac[6], const zt_roster_t* roster) {
  // Big T-N overlay during harness countdown, like Mode B
  display.fillScreen(TFT_BLACK);
  display.setTextDatum(textdatum_t::middle_center);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.setTextSize(3);
  display.drawString("SYNC START", display.width()/2, display.height()/2 + DISPLAY_COUNTDOWN_HEADER_Y_OFFSET);
  display.setTextSize(5);
  char buf[16]; snprintf(buf, sizeof(buf), "%d", secs_remaining);
  display.drawString(buf, display.width()/2, display.height()/2 + DISPLAY_COUNTDOWN_NUMBER_Y_OFFSET);
  
  // Small footer: initial holder hint
  display.setTextSize(2);
  char macs[32];
  snprintf(macs, sizeof(macs), "%02X:%02X:%02X", initial_holder_mac[3], initial_holder_mac[4], initial_holder_mac[5]);
  display.drawString(macs, display.width()/2, display.height()/2 + DISPLAY_COUNTDOWN_FOOTER_Y_OFFSET);
}

/**
 * @brief Configures RMT (Remote Control) peripheral for IR LED transmission
 * 
 * This function sets up the RMT hardware for transmitting IR signals:
 * - Configures RMT channel on GPIO 26
 * - Sets up 38kHz carrier frequency with 33% duty cycle
 * - Creates copy encoder for IR symbol generation
 * - Enables the RMT channel for transmission
 * 
 * Called once at startup before any IR transmission begins.
 */
static bool setup_rmt_tx() {
  rmt_tx_channel_config_t txc = { 
    .gpio_num=IR_TX_GPIO, 
    .clk_src=RMT_CLK_SRC_DEFAULT,
    .resolution_hz=RMT_RESOLUTION_HZ, 
    .mem_block_symbols=RMT_MEM_BLOCK_SYMBOLS, 
    .trans_queue_depth=RMT_TX_QUEUE_DEPTH,
    .intr_priority = RMT_TX_INTR_PRIORITY,
    .flags = {0}
  };
  ESP_LOGI(TAG, "Setting up RMT TX for IR on GPIO %d", IR_TX_GPIO);
  
  // Create RMT TX channel with error handling
  esp_err_t err = rmt_new_tx_channel(&txc, &rmt_tx);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create RMT TX channel: %s", esp_err_to_name(err));
    return false;
  }
  
  // Configure carrier frequency
  rmt_carrier_config_t c = { 
    .frequency_hz=IR_CARRIER_FREQ_HZ, 
    .duty_cycle=IR_CARRIER_DUTY_CYCLE, 
    .flags={ .polarity_active_low=0 } 
  };
  err = rmt_apply_carrier(rmt_tx, &c);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to apply RMT carrier: %s", esp_err_to_name(err));
    rmt_del_channel(rmt_tx);
    rmt_tx = nullptr;
    return false;
  }
  
  // Create copy encoder
  rmt_copy_encoder_config_t cc = {};
  err = rmt_new_copy_encoder(&cc, &tx_copy);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create RMT copy encoder: %s", esp_err_to_name(err));
    rmt_del_channel(rmt_tx);
    rmt_tx = nullptr;
    return false;
  }
  
  // Enable RMT channel
  err = rmt_enable(rmt_tx);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable RMT channel: %s", esp_err_to_name(err));
    rmt_del_channel(rmt_tx);
    rmt_tx = nullptr;
    return false;
  }
  
  ESP_LOGI(TAG, "RMT TX setup completed successfully");
  return true;
} 

/**
 * @brief Converts a UART byte into IR transmission symbols
 * 
 * @param b The UART byte to convert (0-255)
 * @param out Array of RMT symbols to fill with IR representation
 * @return Number of RMT symbols generated
 * 
 * This function converts a single UART byte into IR transmission symbols:
 * - LSB first transmission order
 * - Mark = 0 (IR LED on), Space = 1 (IR LED off)
 * - Each bit gets IR_BIT_US microseconds duration
 * - Adds start and stop bits around the 8 data bits
 * 
 * Used by holder_send_ir_beacon() to build complete IR packets.
 */
/**
 * @brief Converts a UART byte into IR transmission symbols
 * 
 * @param b The UART byte to convert (0-255)
 * @param out Array of RMT symbols to fill with IR representation
 * @return Number of RMT symbols generated
 * 
 * This function converts a single UART byte into IR transmission symbols:
 * - LSB first transmission order
 * - Mark = 0 (IR LED on), Space = 1 (IR LED off)
 * - Each bit gets IR_BIT_US microseconds duration
 * - Adds start and stop bits around the 8 data bits
 * 
 * Used by build_beacon_packet() to build complete IR packets.
 */
static size_t build_uart_byte(uint8_t b, rmt_symbol_word_t* out) {
  // Input validation
  if (out == nullptr) {
    ESP_LOGE(TAG, "build_uart_byte: out parameter is null");
    return 0;
  }
  
  // Lambda functions for creating mark/space symbols
  auto create_mark = [](rmt_symbol_word_t* s, uint32_t us) {
    s->level0 = 1;      // IR LED on
    s->duration0 = us;  // Duration in microseconds
    s->level1 = 0;      // IR LED off
    s->duration1 = 1;   // Minimum space
  };
  
  auto create_space = [](rmt_symbol_word_t* s, uint32_t us) {
    s->level0 = 0;      // IR LED off
    s->duration0 = us;  // Duration in microseconds
    s->level1 = 0;      // IR LED off
    s->duration1 = 1;   // Minimum space
  };
  
  size_t symbol_index = 0;
  
  // Start bit (mark)
  create_mark(&out[symbol_index++], IR_BIT_US);
  
  // 8 data bits, LSB first
  for (int bit = 0; bit < 8; bit++) {
    if ((b >> bit) & 1) {
      // Bit is 1: create space
      create_space(&out[symbol_index++], IR_BIT_US);
    } else {
      // Bit is 0: create mark
      create_mark(&out[symbol_index++], IR_BIT_US);
    }
  }
  
  // Stop bit (space)
  create_space(&out[symbol_index++], IR_BIT_US);
  
  return symbol_index;
}

/**
 * @brief Calculates CRC-8 checksum for beacon packet data
 * 
 * @param data Pointer to data array
 * @param len Length of data array
 * @return 8-bit CRC value
 * 
 * This function implements CRC-8 with polynomial 0x31 (x^8 + x^5 + x^4 + 1).
 * Used to verify beacon packet integrity during transmission and reception.
 */
static uint8_t calculate_crc8(const uint8_t* data, size_t len) {
  // Input validation
  if (data == nullptr) {
    ESP_LOGE(TAG, "calculate_crc8: data parameter is null");
    return 0;
  }
  if (len == 0) {
    ESP_LOGW(TAG, "calculate_crc8: length is 0");
    return 0;
  }
  
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x80) {
        crc = (uint8_t)((crc << 1) ^ 0x31);
      } else {
        crc = (uint8_t)(crc << 1);
      }
    }
  }
  return crc;
}

/**
 * @brief Builds complete IR beacon packet from components
 * 
 * @param seq Sequence number for this beacon
 * @param mac MAC address of the holder device
 * @param symbols Array to fill with IR transmission symbols
 * @return Number of symbols generated
 * 
 * This function constructs the complete IR beacon packet:
 * - Preamble (ZT bytes)
 * - Length field
 * - Sequence number (2 bytes, little-endian)
 * - MAC address (6 bytes)
 * - CRC-8 checksum
 * 
 * Each byte is converted to IR symbols using build_uart_byte().
 */
static size_t build_beacon_packet(uint16_t seq, const uint8_t* mac, rmt_symbol_word_t* symbols) {
  // Input validation
  if (mac == nullptr) {
    ESP_LOGE(TAG, "build_beacon_packet: mac parameter is null");
    return 0;
  }
  if (symbols == nullptr) {
    ESP_LOGE(TAG, "build_beacon_packet: symbols parameter is null");
    return 0;
  }
  
  size_t n = 0;
  
  // Preamble
  n += build_uart_byte(IR_PREAMBLE0, &symbols[n]);
  n += build_uart_byte(IR_PREAMBLE1, &symbols[n]);
  
  // Length field
  const uint8_t len = (uint8_t)EXPECTED_PACKET_LEN; // SEQ(2) + MAC(6)
  n += build_uart_byte(len, &symbols[n]);
  
  // Sequence number (little-endian)
  n += build_uart_byte((uint8_t)(seq & 0xFF), &symbols[n]);      // LSB
  n += build_uart_byte((uint8_t)(seq >> 8), &symbols[n]);        // MSB
  
  // MAC address
  for (int i = 0; i < IR_MAC_LEN; i++) {
    n += build_uart_byte(mac[i], &symbols[n]);
  }
  
  // CRC-8 checksum
  uint8_t crc_src[1 + PACKET_SEQ_SIZE + IR_MAC_LEN];
  crc_src[0] = len;
  crc_src[1] = (uint8_t)(seq & 0xFF);
  crc_src[2] = (uint8_t)(seq >> 8);
  memcpy(&crc_src[3], mac, IR_MAC_LEN);
  
  uint8_t crc = calculate_crc8(crc_src, sizeof(crc_src));
  n += build_uart_byte(crc, &symbols[n]);
  
  return n;
}

/**
 * @brief Transmits an IR beacon packet from the holder device
 * 
 * @param seq Sequence number for this beacon (increments with each transmission)
 * 
 * This function builds and transmits a complete IR beacon packet:
 * - Creates packet with preamble, length, sequence, MAC address, and CRC
 * - Converts packet to IR symbols using build_uart_byte()
 * - Transmits via RMT peripheral on GPIO 26
 * - Updates beacon tracking variables (current/previous sequence, timing)
 * - Logs transmission for debugging
 * 
 * Called periodically by beacon_task() when device is in HOLDER role.
 * Runners receive these beacons and use them to request passes.
 */
static void holder_send_ir_beacon(uint16_t seq) {
  // Get this device's MAC address
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  
  // Build complete beacon packet
  rmt_symbol_word_t symbols[IR_SYMBOL_BUFFER_SIZE];
  size_t symbol_count = build_beacon_packet(seq, mac, symbols);
  
  // Transmit via RMT
  rmt_transmit_config_t cfg = {
    .loop_count = 0,
    .flags = {0}
  };
  
  esp_err_t err = rmt_transmit(rmt_tx, tx_copy, symbols, symbol_count * sizeof(symbols[0]), &cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "RMT transmit failed: %s", esp_err_to_name(err));
    return;
  }
  
  // Wait for transmission to complete
  err = rmt_tx_wait_all_done(rmt_tx, pdMS_TO_TICKS(RMT_TX_WAIT_TIMEOUT_MS));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "RMT wait failed: %s", esp_err_to_name(err));
    return;
  }
  
  // Update beacon tracking state
  g_previous_beacon_seq = g_current_beacon_seq;
  g_current_beacon_seq = seq;
  g_last_beacon_timestamp_us = esp_timer_get_time();
  
  ESP_LOGI(TAG, "IR beacon sent (SEQ=%u)", (unsigned)seq);
}

// --- UART RX (runners) ---
/**
 * @brief Configures UART peripheral for receiving demodulated IR signals
 * 
 * This function sets up UART2 for IR reception:
 * - Configures UART parameters (2400 baud, 8N1 format)
 * - Installs UART driver with receive buffer
 * - Binds GPIO 36 as RX pin for IR receiver input
 * - Optionally inverts signal if IR_UART_RX_INVERT is defined
 * - Logs configuration for debugging
 * 
 * Called once at startup before any IR reception begins.
 * IR receiver demodulates 38kHz signals into UART-compatible digital data.
 */
static bool setup_uart_rx(){
  uart_config_t cfg={ 
    .baud_rate=IR_BAUD, 
    .data_bits=UART_DATA_8_BITS, 
    .parity=UART_PARITY_DISABLE,
    .stop_bits=UART_STOP_BITS_1, 
    .flow_ctrl=UART_HW_FLOWCTRL_DISABLE, 
    .source_clk=UART_SCLK_DEFAULT
  };
  
  ESP_LOGI(TAG, "Setting up UART RX for IR on GPIO %d", IR_RX_GPIO);
  
  // Configure UART parameters
  esp_err_t err = uart_param_config(IR_UART_PORT, &cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(err));
    return false;
  }
  
  // Install UART driver
  err = uart_driver_install(IR_UART_PORT, IR_UART_RX_BUF_SZ, 0, 0, NULL, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(err));
    return false;
  }
  
  // Set UART pins
  err = uart_set_pin(IR_UART_PORT, UART_PIN_NO_CHANGE, IR_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(err));
    uart_driver_delete(IR_UART_PORT);
    return false;
  }
  
  // Optionally invert RX line (IR receiver outputs inverted signal)
#ifdef IR_UART_RX_INVERT
  err = uart_set_line_inverse(IR_UART_PORT, UART_SIGNAL_RXD_INV);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to set UART line inverse: %s (continuing anyway)", esp_err_to_name(err));
    // This is not critical, so we continue
  }
#endif
  
  ESP_LOGI(TAG, "UART RX setup completed successfully on GPIO%d @ %d-8N1", (int)IR_RX_GPIO, IR_BAUD);
  return true;
}

/**
 * @brief Enables or disables IR reception capability
 * 
 * @param en true to enable IR reception, false to disable
 * 
 * This function controls whether the device listens for IR beacons:
 * - HOLDER devices: IR reception disabled (don't receive own beacons)
 * - RUNNER devices: IR reception enabled (listen for holder beacons)
 * - Called during role transitions to update reception state
 * 
 * Simple setter function that controls the g_ir_reception_enabled flag.
 */
void ir_rx_set_enabled(bool en){ 
  g_ir_reception_enabled = en; 
}

// --- ESPNOW infrastructure ---

/**
 * @brief Ensures NVS (Non-Volatile Storage) flash is properly initialized
 * 
 * This function handles NVS flash initialization and error recovery:
 * - Attempts to initialize NVS flash system
 * - Handles common errors (no free pages, new version found)
 * - Erases and reinitializes if corruption detected
 * - Provides clean NVS state for WiFi operations
 * 
 * Called by espnow_init_once() before WiFi setup.
 * NVS is required for storing WiFi configuration data.
 */
static bool ensure_nvs(){
   esp_err_t e = nvs_flash_init(); 
   if(e==ESP_ERR_NVS_NO_FREE_PAGES||e==ESP_ERR_NVS_NEW_VERSION_FOUND){
    ESP_LOGW(TAG, "NVS needs cleanup: %s, erasing and reinitializing...", esp_err_to_name(e));
    e = nvs_flash_erase();
    if (e != ESP_OK) {
      ESP_LOGE(TAG, "Failed to erase NVS: %s", esp_err_to_name(e));
      return false;
    }
    e = nvs_flash_init();
    if (e != ESP_OK) {
      ESP_LOGE(TAG, "Failed to reinitialize NVS: %s", esp_err_to_name(e));
      return false;
    }
    ESP_LOGI(TAG, "NVS cleanup completed successfully");
  } 
  else if (e != ESP_OK) {
    ESP_LOGE(TAG, "NVS initialization failed: %s", esp_err_to_name(e));
    return false;
  }
  
  ESP_LOGI(TAG, "NVS initialization completed successfully");
  return true;
}

/**
 * @brief One-time initialization of ESP-NOW wireless communication system
 * 
 * @return ESP_OK on success, ESP_ERR_* on failure
 * 
 * This function sets up the complete ESP-NOW infrastructure:
 * - Initializes NVS flash (via ensure_nvs())
 * - Sets up WiFi in STA mode on channel 1
 * - Initializes ESP-NOW protocol
 * - Adds broadcast peer for sending to all devices
 * - Registers send callback for transmission logging
 * - Uses static flag to prevent multiple initializations
 * 
 * Called once at startup before any ESP-NOW communication begins.
 * Sets up foundation for PASS_REQ, PASS_GRANT, and STATE_SYNC messages.
 */
static esp_err_t espnow_init_once(){
  static bool ready = false; 
  if(ready) return ESP_OK; 
  
  // Initialize NVS first
  if (!ensure_nvs()) {
    ESP_LOGE(TAG, "NVS initialization failed, cannot proceed with ESP-NOW setup");
    return ESP_ERR_INVALID_STATE;
  }
  
  // Initialize networking components
  esp_err_t err = esp_netif_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize netif: %s", esp_err_to_name(err));
    return err;
  }
  
  err = esp_event_loop_create_default();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create event loop: %s", esp_err_to_name(err));
    return err;
  }
  
  esp_netif_t* netif = esp_netif_create_default_wifi_sta();
  if (netif == nullptr) {
    ESP_LOGE(TAG, "Failed to create default WiFi STA");
    return ESP_ERR_INVALID_STATE;
  }
  
  // Initialize WiFi
  wifi_init_config_t w = WIFI_INIT_CONFIG_DEFAULT(); 
  err = esp_wifi_init(&w);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize WiFi: %s", esp_err_to_name(err));
    return err;
  }
  
  err = esp_wifi_set_mode(WIFI_MODE_STA);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(err));
    return err;
  }
  
  err = esp_wifi_start();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(err));
    return err;
  }
  
  err = esp_wifi_set_ps(WIFI_PS_NONE);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to set WiFi power save: %s (continuing anyway)", esp_err_to_name(err));
    // This is not critical, so we continue
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
  
  // Add broadcast peer
  uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 
  esp_now_peer_info_t p = {}; 
  memcpy(p.peer_addr, bcast, 6); 
  p.ifidx = WIFI_IF_STA; 
  p.channel = ESPNOW_CHANNEL; 
  p.encrypt = false; 
  
  err = esp_now_add_peer(&p);
  if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
    ESP_LOGW(TAG, "Failed to add broadcast peer: %s (continuing anyway)", esp_err_to_name(err));
    // This is not critical, so we continue
  }
  
  // Register send callback
  err = esp_now_register_send_cb([](const uint8_t* m, esp_now_send_status_t st){ 
    ESP_LOGD("ESPNOW","tx %02X:%02X:%02X:%02X:%02X:%02X -> %s", m[0],m[1],m[2],m[3],m[4],m[5], st==ESP_NOW_SEND_SUCCESS?"OK":"FAIL"); 
  });
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to register ESP-NOW send callback: %s (continuing anyway)", esp_err_to_name(err));
    // This is not critical, so we continue
  }
  
  ready = true; 
  g_espnow_ready = true; 
  ESP_LOGI(TAG, "ESP-NOW initialization completed successfully");
  return ESP_OK;
}

// ============================================================================
// ESP-NOW MESSAGE CONSTANTS
// ============================================================================

// Message types (keep Mode-A codes)
#define MSG_PASS_REQ   0xA1
#define MSG_PASS_GRANT 0xA2
#define MSG_STATE_SYNC 0xA3

// Message structure constants
#define MSG_HEADER_SIZE 1                   // Message type byte
#define MSG_SEQ_SIZE 2                      // Sequence number (2 bytes)
#define MSG_MAC_SIZE 6                      // MAC address (6 bytes)
#define MSG_COOLDOWN_SIZE 2                 // Cooldown timer (2 bytes)

// Message length constants
#define PASS_REQ_MSG_LEN (MSG_HEADER_SIZE + MSG_SEQ_SIZE + MSG_MAC_SIZE)      // 9 bytes
#define PASS_GRANT_MSG_LEN (MSG_HEADER_SIZE + MSG_SEQ_SIZE + MSG_MAC_SIZE)     // 9 bytes
#define STATE_SYNC_MSG_LEN (MSG_HEADER_SIZE + MSG_SEQ_SIZE + MSG_MAC_SIZE + MSG_COOLDOWN_SIZE)  // 12 bytes

/**
 * @brief Ensures a peer device is added to ESP-NOW peer list
 * 
 * @param mac MAC address of the peer device to add
 * @return ESP_OK on success, ESP_ERR_* on failure
 * 
 * This function adds a peer device to the ESP-NOW peer list:
 * - Creates peer info structure with MAC address
 * - Sets WiFi interface and channel parameters
 * - Adds peer to ESP-NOW system
 * - Returns success if peer already exists or was added successfully
 * 
 * Called before sending any ESP-NOW message to ensure peer is available.
 */
static esp_err_t ensure_peer(const uint8_t* mac){ esp_now_peer_info_t p={}; memcpy(p.peer_addr,mac,6); p.ifidx=WIFI_IF_STA; p.channel=ESPNOW_CHANNEL; p.encrypt=false; auto e=esp_now_add_peer(&p); return (e==ESP_ERR_ESPNOW_EXIST)?ESP_OK:e; }
/**
 * @brief Sends a pass request message from runner to holder
 * 
 * @param holder MAC address of the holder device
 * @param seq Sequence number from the IR beacon that triggered this request
 * @return ESP_OK on success, ESP_ERR_* on failure
 * 
 * This function sends a PASS_REQ message when a runner detects an IR beacon:
 * - Ensures holder is in peer list
 * - Creates message with MSG_PASS_REQ type, sequence, and holder MAC
 * - Sends via ESP-NOW to the holder device
 * 
 * Called by uart_rx_task() when valid IR beacon is received.
 */
static esp_err_t send_pass_req(const uint8_t* holder, uint16_t seq){
  esp_err_t err = ensure_peer(holder);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to ensure peer for PASS_REQ: %s", esp_err_to_name(err));
    return err;
  }
  
  uint8_t pl[PASS_REQ_MSG_LEN]={ MSG_PASS_REQ, (uint8_t)(seq&0xFF),(uint8_t)(seq>>8), holder[0],holder[1],holder[2],holder[3],holder[4],holder[5] };
  err = esp_now_send(holder, pl, sizeof(pl));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to send PASS_REQ: %s", esp_err_to_name(err));
  }
  return err;
}
/**
 * @brief Sends a pass grant message from holder to winning runner
 * 
 * @param winner MAC address of the runner that won the pass
 * @param seq Sequence number from the pass request
 * @return ESP_OK on success, ESP_ERR_* on failure
 * 
 * This function sends a PASS_GRANT message when holder accepts a pass request:
 * - Ensures winner is in peer list
 * - Creates message with MSG_PASS_GRANT type, sequence, and winner MAC
 * - Sends via ESP-NOW to the winning runner
 * 
 * Called by recv_cb() when processing valid PASS_REQ messages.
 */
static esp_err_t send_pass_grant(const uint8_t* winner, uint16_t seq){
  esp_err_t err = ensure_peer(winner);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to ensure peer for PASS_GRANT: %s", esp_err_to_name(err));
    return err;
  }
  
  uint8_t pl[PASS_GRANT_MSG_LEN]={ MSG_PASS_GRANT, (uint8_t)(seq&0xFF),(uint8_t)(seq>>8), winner[0],winner[1],winner[2],winner[3],winner[4],winner[5] };
  err = esp_now_send(winner, pl, sizeof(pl));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to send PASS_GRANT: %s", esp_err_to_name(err));
  }
  return err;
}
/**
 * @brief Broadcasts state synchronization message to all devices
 * 
 * @param seq Sequence number from the pass that caused this role change
 * @param new_holder MAC address of the device that is now the holder
 * @return ESP_OK on success, ESP_ERR_* on failure
 * 
 * This function broadcasts a STATE_SYNC message when roles change:
 * - Creates message with MSG_STATE_SYNC type, sequence, new holder MAC, and cooldown
 * - Sends via ESP-NOW broadcast to all connected devices
 * - Informs all devices about the new holder and enables cooldown enforcement
 * 
 * Called by recv_cb() after granting a pass to synchronize all devices.
 */
static esp_err_t send_state_sync(uint16_t seq, const uint8_t* new_holder){
  uint8_t pl[STATE_SYNC_MSG_LEN]={ MSG_STATE_SYNC, (uint8_t)(seq&0xFF),(uint8_t)(seq>>8),
    new_holder[0],new_holder[1],new_holder[2],new_holder[3],new_holder[4],new_holder[5],
    (uint8_t)(PASS_COOLDOWN_MS&0xFF),(uint8_t)((PASS_COOLDOWN_MS>>8)&0xFF) };
  
  uint8_t bcast[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  esp_err_t err = esp_now_send(bcast, pl, sizeof(pl));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to send STATE_SYNC: %s", esp_err_to_name(err));
  }
  return err;
}

// ============================================================================
// MESSAGE PROCESSING HELPERS
// ============================================================================

/**
 * @brief Updates LED strip to show HOLDER status (red LEDs)
 * 
 * This function sets all side LEDs to red to indicate the device is now the holder.
 * Called when transitioning to HOLDER role.
 */
static void update_leds_for_holder() {
  if (led_strip) {
    SAFE_EXECUTE(TAG, "LED strip clear", led_strip_clear(led_strip));
    SAFE_EXECUTE(TAG, "LED strip refresh", led_strip_refresh(led_strip));
    
    for (int i = 0; i < HP_LED_COUNT; i++) {
      SAFE_EXECUTE(TAG, "LED strip set pixel", led_strip_set_pixel(led_strip, i, LED_RED_R, LED_RED_G, LED_RED_B));
    }
    
    SAFE_EXECUTE(TAG, "LED strip final refresh", led_strip_refresh(led_strip));
  }
}

/**
 * @brief Updates LED strip to show RUNNER status (LEDs off)
 * 
 * This function turns off all side LEDs to indicate the device is now a runner.
 * Called when transitioning to RUNNER role.
 */
static void update_leds_for_runner() {
  if (led_strip) {
    SAFE_EXECUTE(TAG, "LED strip clear", led_strip_clear(led_strip));
    SAFE_EXECUTE(TAG, "LED strip refresh", led_strip_refresh(led_strip));
  }
}

/**
 * @brief Transitions device from HOLDER to RUNNER role
 * 
 * @param seq Sequence number from the pass that caused this transition
 * @param winner_mac MAC address of the runner that won the pass
 * 
 * This function handles the complete transition from HOLDER to RUNNER:
 * - Updates role state
 * - Enables IR reception
 * - Clears UART input buffer
 * - Updates LEDs to runner state
 * - Updates display UI
 * - Logs the transition
 */
static void transition_to_runner(uint16_t seq, const uint8_t* winner_mac) {
  g_role = ROLE_RUNNER;
  ir_rx_set_enabled(true);
  uart_flush_input(IR_UART_PORT);  // drop any partial junk before we start parsing
  
  update_leds_for_runner();
  draw_role_ui();
  
  ESP_LOGI(TAG, "GRANT -> %02X:%02X:%02X:%02X:%02X:%02X seq=%u",
    winner_mac[0], winner_mac[1], winner_mac[2], 
    winner_mac[3], winner_mac[4], winner_mac[5], (unsigned)seq);
}

/**
 * @brief Transitions device to HOLDER role
 * 
 * @param seq Sequence number from the pass that caused this transition
 * 
 * This function handles the transition to HOLDER role:
 * - Updates role state
 * - Disables IR reception (don't receive own beacons)
 * - Updates LEDs to holder state (red)
 * - Updates display UI
 * - Logs the transition
 */
static void transition_to_holder(uint16_t seq) {
  g_role = ROLE_HOLDER;
  ir_rx_set_enabled(false);
  
  update_leds_for_holder();
  draw_role_ui();
  
  ESP_LOGI(TAG, "ROLE -> HOLDER (seq=%u)", (unsigned)seq);
}

/**
 * @brief Processes a PASS_REQ message from a runner
 * 
 * @param info ESP-NOW message information
 * @param data Message payload data
 * @param len Message length
 * @return true if message was processed successfully, false otherwise
 * 
 * This function handles pass requests from runners:
 * - Validates the request is for this holder
 * - Checks sequence number validity (current or recent beacon)
 * - Enforces cooldown to prevent immediate passback
 * - Grants the pass if valid
 * - Transitions this device to RUNNER role
 */
static bool process_pass_request(const esp_now_recv_info* info, const uint8_t* data, int len) {
  // Input validation - ensure all parameters are valid before processing
  if (info == nullptr) {
    ESP_LOGE(TAG, "process_pass_request: info parameter is null");
    return false;
  }
  if (data == nullptr) {
    ESP_LOGE(TAG, "process_pass_request: data parameter is null");
    return false;
  }
  if (len < 0) {
    ESP_LOGE(TAG, "process_pass_request: invalid length %d", len);
    return false;
  }
  
  // Only HOLDER devices can process pass requests, and message must be long enough
  if (g_role != ROLE_HOLDER || len < PASS_REQ_MSG_LEN) return false;
  
  // Extract sequence number from message (little-endian format: LSB first, then MSB)
  // This matches the ESP32's native byte order for better performance
  uint16_t seq = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
  
  // Extract the intended holder's MAC address from the message
  // This allows us to verify the request is actually meant for this device
  uint8_t holder[6]; 
  memcpy(holder, &data[3], 6);
  
  // Verify this request is aimed at us - prevents processing requests meant for other devices
  if (memcmp(holder, g_self_mac, 6) != 0) return false;
  
  // Sequence validation: accept current beacon or briefly accept previous one
  // This absorbs ESP-NOW latency where messages might arrive slightly out of order
  int64_t now = esp_timer_get_time();
  bool seq_valid = (seq == g_current_beacon_seq) ||
                   ((seq == g_previous_beacon_seq) && 
                    (now - g_last_beacon_timestamp_us) <= MODEA_ACCEPT_PREV_BEACON_US);
  if (!seq_valid) return false;
  
  // Check cooldown timer to prevent rapid passback (ping-pong effect)
  // Cooldown ensures at least PASS_COOLDOWN_MS milliseconds between passes
  if (now < g_cooldown_until_timestamp_us) {
    ESP_LOGD(TAG, "PASS_REQ drop: cooldown remaining=%lld us", 
              (long long)(g_cooldown_until_timestamp_us - now));
    return false;
  }
  
  // Distinct-before-repeat: check if this candidate is allowed to receive the potato
  if (!zt_is_candidate_allowed(info->src_addr)) {
    ESP_LOGD(TAG, "PASS_REQ drop: candidate %02X:%02X:%02X:%02X:%02X:%02X not eligible this round",
             info->src_addr[0], info->src_addr[1], info->src_addr[2],
             info->src_addr[3], info->src_addr[4], info->src_addr[5]);
    return false;
  }
  
  // First valid request wins: send grant + broadcast state sync + flip roles
  // Note: We continue even if these messages fail to ensure role transition happens
  esp_err_t grant_err = send_pass_grant(info->src_addr, seq);
  if (grant_err != ESP_OK) {
    ESP_LOGW(TAG, "PASS_GRANT failed, but continuing with role transition");
  }
  
  esp_err_t sync_err = send_state_sync(seq, info->src_addr);
  if (sync_err != ESP_OK) {
    ESP_LOGW(TAG, "STATE_SYNC failed, but continuing with role transition");
  }
  
  // Set cooldown timer to prevent immediate passback (ping-pong effect)
  // Convert milliseconds to microseconds for high-precision timing
  g_cooldown_until_timestamp_us = esp_timer_get_time() + (int64_t)PASS_COOLDOWN_MS * 1000;
  
  // Transition roles: this device becomes RUNNER, requester becomes HOLDER
  // This is the core game mechanic - passing the potato changes who has it
  transition_to_runner(seq, info->src_addr);
  
  // Record the grant in distinct-before-repeat tracking
  zt_record_grant(info->src_addr);
  ESP_LOGI(TAG, "Pass granted to %02X:%02X:%02X:%02X:%02X:%02X - distinct-before-repeat updated",
           info->src_addr[0], info->src_addr[1], info->src_addr[2],
           info->src_addr[3], info->src_addr[4], info->src_addr[5]);
  
  // Reset empty window counter since we successfully granted a pass
  g_empty_window_count = 0;
  
  return true;
}

/**
 * @brief Processes PASS_GRANT or STATE_SYNC messages
 * 
 * @param data Message payload data
 * @param len Message length
 * @return true if message was processed successfully, false otherwise
 * 
 * This function handles role synchronization messages:
 * - Extracts sequence number and new holder MAC
 * - Updates this device's role based on whether it's the new holder
 * - Updates LEDs and UI accordingly
 */
static bool process_role_sync(const uint8_t* data, int len) {
  // Input validation - ensure data pointer and length are valid
  if (data == nullptr) {
    ESP_LOGE(TAG, "process_role_sync: data parameter is null");
    return false;
  }
  if (len < 0) {
    ESP_LOGE(TAG, "process_role_sync: invalid length %d", len);
    return false;
  }
  
  // Message must be at least as long as a PASS_GRANT message
  if (len < PASS_GRANT_MSG_LEN) return false;
  
  // Extract sequence number (little-endian: LSB first, then MSB)
  uint16_t seq = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
  
  // Extract the MAC address of the new holder from the message
  uint8_t new_holder[6]; 
  memcpy(new_holder, &data[3], 6);
  
  // Check if this device is the new holder
  if (memcmp(new_holder, g_self_mac, 6) == 0) {
    // This device is becoming the new holder - transition to HOLDER role
    transition_to_holder(seq);
  } else {
    // This device remains a runner - update state accordingly
    g_role = ROLE_RUNNER;
    
    // Enable IR reception since runners need to detect beacons
    ir_rx_set_enabled(true);
    
    // Clear any stale UART data to prevent processing old beacons
    uart_flush_input(IR_UART_PORT);
    
    // Update visual indicators and UI for RUNNER role
    update_leds_for_runner();
    draw_role_ui();
  }
  return true;
}

// ============================================================================
// MAIN RECEIVE CALLBACK
// ============================================================================

/**
 * @brief Main ESP-NOW receive callback for processing game messages
 * 
 * @param info Information about the received message (source MAC, etc.)
 * @param data Pointer to the message payload data
 * @param len Length of the message data
 * 
 * This function is the main entry point for all incoming ESP-NOW messages.
 * It delegates processing to specialized functions based on message type:
 * - MSG_PASS_REQ: Delegate to process_pass_request()
 * - MSG_PASS_GRANT/STATE_SYNC: Delegate to process_role_sync()
 * 
 * Called automatically by ESP-NOW when messages are received.
 */
static void recv_cb(const esp_now_recv_info* info, const uint8_t* data, int len) {
  if (len < 1) return;
  
  switch (data[0]) {
    case MSG_PASS_REQ:
      process_pass_request(info, data, len);
      break;
      
    case MSG_PASS_GRANT:  // runner becomes holder
    case MSG_STATE_SYNC:  // role synchronization
      process_role_sync(data, len);
      break;
      
    default:
      // Unknown message type - ignore
      break;
  }
}
/**
 * @brief Installs the ESP-NOW receive callback for game message processing
 * 
 * This function registers the recv_cb() function as the ESP-NOW receive callback:
 * - Replaces any previous callback (like from test harness)
 * - Enables processing of PASS_REQ, PASS_GRANT, and STATE_SYNC messages
 * - Called after test harness completes to restore game functionality
 * 
 * Called from ir_downlink_test_main() after harness initialization.
 */
extern "C" void ir_downlink_install_recv_cb(void){ 
  esp_err_t err = esp_now_register_recv_cb(recv_cb);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register ESP-NOW receive callback: %s", esp_err_to_name(err));
    // This is critical - without this callback, the game cannot function
    // We could implement a retry mechanism here if needed
  } else {
    ESP_LOGI(TAG, "ESP-NOW receive callback registered successfully");
  }
}

// --- Background tasks ---

/**
 * @brief Background task for periodic IR beacon transmission
 * 
 * @param arg Task argument (unused)
 * 
 * This task handles periodic IR beacon transmission from holder devices:
 * - Creates periodic timer for beacon transmission
 * - Waits for timer notifications
 * - Calls holder_send_ir_beacon() every BEACON_PERIOD_MS (300ms)
 * - Only active when device is in HOLDER role and game has started
 * - Increments sequence number with each beacon
 * 
 * Created at startup and runs continuously in background.
 */
static void beacon_task(void*){
  // Store task handle for timer callback to wake this task
  s_beacon_task = xTaskGetCurrentTaskHandle();
  
  // Create periodic timer if it doesn't exist
  // This timer will wake the beacon task at regular intervals
  if(!s_beacon_timer){ 
    const esp_timer_create_args_t a={ 
      .callback=&beacon_timer_cb,  // Function to call when timer expires
      .arg = nullptr,              // No arguments needed
      .dispatch_method = ESP_TIMER_TASK,  // Use dedicated timer task
      .name="downlink_beacon",     // Human-readable name for debugging
      .skip_unhandled_events = false      // Don't skip events
    }; 
    
    esp_err_t err = esp_timer_create(&a, &s_beacon_timer);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to create beacon timer: %s", esp_err_to_name(err));
      // This is critical - without the timer, beacons won't be sent
      return;  // Exit the task if timer creation fails
    }
  }
  
  // Start periodic timer to wake beacon task every BEACON_PERIOD_MS milliseconds
  esp_err_t err = esp_timer_start_periodic(s_beacon_timer, (uint64_t)BEACON_PERIOD_MS*1000ULL);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start beacon timer: %s", esp_err_to_name(err));
    // This is critical - without the timer running, beacons won't be sent
    return;  // Exit the task if timer start fails
  }
  
  for(;;){
    // Wait for timer notification to wake this task
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
      // Only send beacons if we're the holder and the game is active
  if(g_role!=ROLE_HOLDER || !g_game_active) continue;
  
  // Send IR beacon with current sequence number, then increment
  // This ensures each beacon has a unique, increasing sequence number
  holder_send_ir_beacon(g_beacon_sequence++);
  
  // Increment empty window counter since no pass was granted in this window
  // This helps track when no eligible candidates are available
  g_empty_window_count++;
  
  // Check if we need to relax round restrictions due to empty windows
  // This prevents the game from getting stuck when no eligible candidates exist
  if (g_empty_window_count >= 10) { // Relax after 10 empty windows
    zt_round_maybe_relax(g_empty_window_count, g_self_mac);
    ESP_LOGI(TAG, "Round relaxed after %lu empty windows to prevent game stall", (unsigned long)g_empty_window_count);
    g_empty_window_count = 0; // Reset counter
  }
  
  // Small delay to prevent overwhelming the system if there are errors
  // This gives the RMT hardware time to complete transmission
  vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/**
 * @brief Background task for processing incoming IR beacon data
 * 
 * @param arg Task argument (unused)
 * 
 * This task handles IR beacon reception and processing:
 * - Continuously reads UART data from IR receiver
 * - Maintains ring buffer for packet assembly
 * - Parses IR packets for valid beacons (preamble, length, sequence, MAC, CRC)
 * - Sends PASS_REQ messages when valid beacons are detected
 * - Only active when IR reception is enabled and game has started
 * - Implements cooldown checking to prevent rapid pass requests
 * 
 * Created at startup and runs continuously in background.
 */
/**
 * @brief Validates CRC-8 checksum for received beacon packet
 * 
 * @param data Pointer to packet data starting at length field
 * @param len Length of packet data
 * @param received_crc CRC value received in packet
 * @return true if CRC matches, false otherwise
 * 
 * This function calculates the expected CRC-8 for the packet data
 * and compares it with the received CRC value.
 */
static bool validate_beacon_crc(const uint8_t* data, uint8_t len, uint8_t received_crc) {
  // Input validation
  if (data == nullptr) {
    ESP_LOGE(TAG, "validate_beacon_crc: data parameter is null");
    return false;
  }
  if (len != EXPECTED_PACKET_LEN) {
    ESP_LOGE(TAG, "validate_beacon_crc: invalid length %d, expected %d", len, EXPECTED_PACKET_LEN);
    return false;
  }
  
  uint8_t crc_src[1 + PACKET_SEQ_SIZE + IR_MAC_LEN];  // LEN + SEQ(2) + MAC(6)
  crc_src[0] = len;
  crc_src[1] = data[0];  // SEQ LSB
  crc_src[2] = data[1];  // SEQ MSB
  memcpy(&crc_src[3], &data[2], IR_MAC_LEN);  // MAC address
  
  uint8_t calculated_crc = calculate_crc8(crc_src, sizeof(crc_src));
  return calculated_crc == received_crc;
}

/**
 * @brief Processes a valid IR beacon packet and sends pass request
 * 
 * @param holder_mac MAC address of the holder device
 * @param seq Sequence number from the beacon
 * @return true if pass request was sent, false if blocked by cooldown
 * 
 * This function handles the complete processing of a valid beacon:
 * - Checks cooldown timer to prevent rapid pass requests
 * - Logs successful beacon reception
 * - Sends PASS_REQ message to the holder
 */
static bool process_valid_beacon(const uint8_t* holder_mac, uint16_t seq) {
  // Check cooldown timer before sending PASS_REQ to prevent rapid passback
  // This prevents the "ping-pong" effect where devices pass the potato back and forth
  int64_t now = esp_timer_get_time();
  if (now < g_cooldown_until_timestamp_us) {
    ESP_LOGD(TAG, "BEACON drop: cooldown remaining=%lld us", 
              (long long)(g_cooldown_until_timestamp_us - now));
    return false;
  }
  
  // Log successful beacon detection with sequence number and holder MAC
  ESP_LOGI(TAG, "BEACON ok: seq=%u from %02X:%02X:%02X:%02X:%02X:%02X",
            (unsigned)seq, holder_mac[0], holder_mac[1], holder_mac[2], 
            holder_mac[3], holder_mac[4], holder_mac[5]);
  
  // Send PASS_REQ message to request the potato from this holder
  // Note: We don't fail if the request fails - the beacon was still valid
  esp_err_t err = send_pass_req(holder_mac, seq);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "PASS_REQ failed for beacon seq=%u: %s", (unsigned)seq, esp_err_to_name(err));
    // Don't return false here - the beacon was valid, just the request failed
    // This allows the system to continue operating even with ESP-NOW issues
  }
  return true;
}

/**
 * @brief Scans ring buffer for valid beacon packets
 * 
 * @param ring_buffer Ring buffer containing UART data
 * @param buffer_length Current length of data in ring buffer
 * @return Number of bytes consumed (packets processed)
 * 
 * This function scans the ring buffer looking for valid beacon packets:
 * - Searches for preamble (IR_PREAMBLE0, IR_PREAMBLE1)
 * - Validates packet length and structure
 * - Checks CRC-8 integrity
 * - Processes valid beacons via process_valid_beacon()
 * - Returns number of bytes consumed for buffer compaction
 */
static int scan_for_beacons(uint8_t* ring_buffer, int buffer_length) {
  int scan_index = 0;
  
  // Scan through the ring buffer looking for valid beacon packets
  // We need at least 4 bytes to check for preamble + length
  while (scan_index + 4 < buffer_length) {
    // Look for IR packet preamble: two specific bytes that mark the start of a packet
    // This helps distinguish valid packets from noise or corrupted data
    if (ring_buffer[scan_index] == IR_PREAMBLE0 && 
        ring_buffer[scan_index + 1] == IR_PREAMBLE1) {
      
      // Extract the packet length from the third byte
      uint8_t len = ring_buffer[scan_index + 2];
      
      // Check if we have a complete packet in the buffer
      // Length must match expected size and we must have enough bytes for the full packet
      if (len == EXPECTED_PACKET_LEN && (scan_index + PACKET_HEADER_SIZE + len) <= buffer_length) {
        
        // Extract packet components for processing
        // Sequence number: little-endian format (LSB first, then MSB)
        uint16_t seq = (uint16_t)ring_buffer[scan_index + 3] | 
                       ((uint16_t)ring_buffer[scan_index + 4] << 8);
        
        // MAC address: 6 bytes starting after sequence number
        uint8_t holder_mac[6];
        memcpy(holder_mac, &ring_buffer[scan_index + 5], 6);
        
        // CRC checksum: last byte of the packet
        uint8_t received_crc = ring_buffer[scan_index + 11];
        
        // Validate packet integrity using CRC-8 checksum
        // This ensures the packet wasn't corrupted during IR transmission
        if (validate_beacon_crc(&ring_buffer[scan_index + 3], len, received_crc)) {
          // Process the valid beacon - this will send a PASS_REQ if cooldown allows
          process_valid_beacon(holder_mac, seq);
        }
        
        // Move past this packet regardless of validity to continue scanning
        // This prevents getting stuck on corrupted packets
        scan_index += PACKET_HEADER_SIZE + len;
        continue;
      }
    }
    
    // No valid frame found at this position, advance to next byte
    scan_index++;
  }
  
  return scan_index;
}

/**
 * @brief Background task for processing incoming IR beacon data
 * 
 * @param arg Task argument (unused)
 * 
 * This task handles IR beacon reception and processing:
 * - Continuously reads UART data from IR receiver
 * - Maintains ring buffer for packet assembly
 * - Scans for valid beacon packets using scan_for_beacons()
 * - Sends PASS_REQ messages when valid beacons are detected
 * - Only active when IR reception is enabled and game has started
 * - Implements cooldown checking to prevent rapid pass requests
 * 
 * Created at startup and runs continuously in background.
 */
static void uart_rx_task(void*) {
  // Ring buffer for accumulating UART data until we have complete packets
  uint8_t ring_buffer[UART_RING_BUFFER_SIZE];
  int buffer_length = 0;
  
  // Temporary buffer for reading from UART
  uint8_t read_buffer[UART_READ_BUFFER_SIZE];
  
  for (;;) {
    // Wait if IR reception is disabled or game hasn't started
    // This prevents unnecessary UART operations when not needed
    if (!g_ir_reception_enabled || !g_game_active) {
      vTaskDelay(pdMS_TO_TICKS(UART_TASK_DELAY_MS));
      continue;
    }
    
    // Read UART data with timeout to prevent blocking
    int bytes_read = uart_read_bytes(IR_UART_PORT, read_buffer, sizeof(read_buffer), pdMS_TO_TICKS(UART_READ_TIMEOUT_MS));
    if (bytes_read < 0) {
      ESP_LOGW(TAG, "UART read error: %d", bytes_read);
      vTaskDelay(pdMS_TO_TICKS(UART_TASK_DELAY_MS));
      continue;
    }
    if (bytes_read == 0) continue;  // No data available
    
    // Append new data to ring buffer with overflow protection
    // If we would overflow, reset the buffer to prevent memory corruption
    if (bytes_read > (int)sizeof(ring_buffer) - buffer_length) {
      ESP_LOGW(TAG, "Ring buffer overflow, resetting (had %d bytes, got %d)", buffer_length, bytes_read);
      buffer_length = 0;  // Reset on overflow
    }
    memcpy(&ring_buffer[buffer_length], read_buffer, bytes_read);
    buffer_length += bytes_read;
    
    // Scan for and process complete beacon packets in the buffer
    // This function returns the number of bytes consumed (processed)
    int consumed = scan_for_beacons(ring_buffer, buffer_length);
    
    // Compact the ring buffer by removing processed data
    // This keeps the buffer efficient and prevents it from growing indefinitely
    if (consumed > 0 && consumed < buffer_length) {
      // Move remaining data to the beginning of the buffer
      memmove(ring_buffer, &ring_buffer[consumed], buffer_length - consumed);
      buffer_length -= consumed;
    } else if (consumed >= buffer_length) {
      // All data was processed, reset buffer
      buffer_length = 0;
    }
  }
}

// --- Main entry point ---

/**
 * @brief Main entry point for Mode A (Downlink) Hot Potato game
 * 
 * This function initializes and runs the complete Hot Potato game:
 * 
 * Initialization Phase:
 * - Sets up ESP-NOW wireless communication
 * - Configures RMT for IR transmission (GPIO 26)
 * - Configures UART for IR reception (GPIO 36)
 * - Initializes LED strip and display
 * - Clears all LEDs to known state
 * 
 * Game Setup Phase:
 * - Runs test harness for device discovery and countdown
 * - Assigns initial roles (HOLDER/RUNNER) based on harness result
 * - Sets up IR reception based on role
 * - Creates background tasks for beacon transmission and IR reception
 * 
 * Game Execution:
 * - Enables game logic after countdown completion
 * - Background tasks handle IR communication and role transitions
 * - Main loop remains idle while tasks do the work
 * 
 * Called from main.cpp when HP_TEST_UPLINK is not defined.
 */
extern "C" void ir_downlink_test_main(void){
  ESP_LOGI(TAG, "=== MODE A: IR DOWNLINK TEST (holder TX / runners IR RX) ===");
  ESP_LOGI(TAG, "Starting IR Downlink Test (Mode A)");
  
  // Initialize ESP-NOW wireless communication for device-to-device messaging
  esp_err_t err = espnow_init_once();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "ESP-NOW initialization failed: %s - cannot proceed with game", esp_err_to_name(err));
    // Display error on screen
    display.fillScreen(TFT_RED);
    display.setTextColor(TFT_WHITE, TFT_RED);
    display.setCursor(DISPLAY_LEFT_MARGIN, 60);
    display.printf("ESP-NOW FAILED");
    display.setCursor(DISPLAY_LEFT_MARGIN, 100);
    display.printf("Check WiFi config");
    
    // Wait indefinitely - game cannot function without ESP-NOW
    ESP_LOGE(TAG, "Game cannot start without ESP-NOW - halting");
    for(;;) vTaskDelay(pdMS_TO_TICKS(1000));
  } 
  
  // Read this device's unique MAC address for identification
  esp_read_mac(g_self_mac, ESP_MAC_WIFI_STA);

  // Initialize LED strip if not already done
  // This provides visual feedback about the device's role (HOLDER = red, RUNNER = off)
  if (led_strip == nullptr) {
    led_strip_config_t strip_config = {
      .strip_gpio_num = HP_LED_GPIO,        // GPIO pin for LED data
      .max_leds = HP_LED_COUNT,             // Number of LEDs in the strip
      .led_model = LED_MODEL_WS2812,        // LED chip type (WS2812/SK6812)
      .flags = {0}                          // No special flags
    };
    led_strip_rmt_config_t rmt_config = {
      .clk_src = RMT_CLK_SRC_DEFAULT,       // Use default clock source
      .resolution_hz = LED_STRIP_RESOLUTION_HZ, // 10MHz for precise timing
      .mem_block_symbols = LED_STRIP_MEM_BLOCK_SYMBOLS, // RMT memory allocation
      .flags = {0}                          // No special flags
    };
    
    // Create LED strip device using RMT peripheral for precise timing
    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "LED strip initialization failed: %s - continuing without LED support", esp_err_to_name(err));
      led_strip = nullptr;  // Ensure it's null if initialization fails
    } else {
      ESP_LOGI(TAG, "LED strip initialized successfully");
    }
  }
  
    // Aggressive clear for WS2812 reset (multiple times to ensure they're off)
  if (led_strip) {
    SAFE_EXECUTE(TAG, "LED strip initial clear", led_strip_clear(led_strip));
    SAFE_EXECUTE(TAG, "LED strip initial refresh", led_strip_refresh(led_strip));
    
    vTaskDelay(pdMS_TO_TICKS(LED_STRIP_RESET_DELAY_MS)); // Small delay for WS2812 to process
    
    SAFE_EXECUTE(TAG, "LED strip final clear", led_strip_clear(led_strip));
    SAFE_EXECUTE(TAG, "LED strip final refresh", led_strip_refresh(led_strip));
    
    ESP_LOGI(TAG, "M5Stack FIRE side LEDs reset - should be OFF now");
  }
  
  // Display setup - initialize M5Stack display for user interface
  display.begin();
  display.setTextSize(2);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.fillScreen(TFT_BLACK);
  display.setCursor(DISPLAY_LEFT_MARGIN, 20);
  display.printf("MODE A");  // Show this is Mode A (Downlink)
  display.setCursor(DISPLAY_LEFT_MARGIN, 60);
  display.printf("ROLE: %s", ZT_IS_HOST ? "HOST" : "PEER");  // Show host/peer status

  // Hardware initialization - set up IR transmission and reception
  bool rmt_ok = setup_rmt_tx();   // Configure RMT for IR LED transmission (GPIO 26)
  if (!rmt_ok) {
    ESP_LOGE(TAG, "RMT TX setup failed - continuing without IR transmission capability");
    // Continue without IR transmission - device can still receive and process messages
  }
  
  bool uart_ok = setup_uart_rx();  // Configure UART for IR receiver (GPIO 36)
  if (!uart_ok) {
    ESP_LOGE(TAG, "UART RX setup failed - continuing without IR reception capability");
    // Continue without IR reception - device can still send messages and coordinate
  }
  
  // Log hardware status for debugging
  ESP_LOGI(TAG, "Hardware status: RMT_TX=%s, UART_RX=%s", 
           rmt_ok ? "OK" : "FAILED", uart_ok ? "OK" : "FAILED");
  
  // Test harness phase: device discovery + synchronized countdown + initial role assignment
  // Register a countdown draw callback so all devices show the same T-minus UI
  zt_set_countdown_cb(countdown_draw_cb);
  zt_role_t r;      // Variable to receive assigned role from harness
  zt_roster_t roster={0};  // Device roster (populated by harness)
  
  // For PEER devices: show waiting screen before joining
  if (!ZT_IS_HOST) {
    display.fillScreen(TFT_BLACK);
    display.setTextSize(2);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.setCursor(DISPLAY_LEFT_MARGIN, 20);
    display.printf("MODE A (Downlink)");
    display.setCursor(DISPLAY_LEFT_MARGIN, 60);
    display.printf("ROLE: PEER");
    display.setCursor(DISPLAY_LEFT_MARGIN, 100);
    display.printf("Waiting for HOST...");
    ESP_LOGI(TAG, "PEER: Showing waiting screen, waiting for host to start...");
    
    // Also log the build configuration to verify macros are working
    ESP_LOGI(TAG, "Build config: ZT_IS_HOST=%d, HP_TEST_UPLINK=%s", 
              ZT_IS_HOST, 
              #ifdef HP_TEST_UPLINK 
                "defined" 
              #else 
                "not defined" 
              #endif
              );
  }
  
  zt_join_and_countdown(&r, &roster,
    /*is_host=*/(bool)ZT_IS_HOST);       // Host opens join & broadcasts START
  g_role = (r==ZT_ROLE_HOLDER)?ROLE_HOLDER:ROLE_RUNNER;  // Convert harness role to game role
  
  // Update display with actual role (post-countdown)
  // Note: For PEER devices, this will show after the countdown completes
  draw_role_ui();
  
  // Initialize distinct-before-repeat game logic with the initial holder
  uint8_t initial_holder_mac[6];
  zt_get_initial_holder(initial_holder_mac);
  zt_distinct_init(&roster, initial_holder_mac);
  ESP_LOGI(TAG, "Distinct-before-repeat initialized with %d devices, initial holder %02X:%02X:%02X:%02X:%02X:%02X",
           roster.count, initial_holder_mac[0], initial_holder_mac[1], initial_holder_mac[2],
           initial_holder_mac[3], initial_holder_mac[4], initial_holder_mac[5]);
  
  // Log the distinct-before-repeat rule for clarity
  ESP_LOGI(TAG, "Game rule: Each device must hold the potato once before any device can hold it again");
  
  // Reinstall our ESP-NOW receive handler after the harness
  // The harness temporarily replaces it during discovery/countdown
  ir_downlink_install_recv_cb();

  // IR reception setup based on assigned role
  ir_rx_set_enabled(g_role==ROLE_RUNNER);  // Only runners need to receive IR beacons

    // Initial LED state based on role
  if (led_strip) {
    if (g_role == ROLE_HOLDER) {
      // HOLDER: red LEDs
      for (int i = 0; i < HP_LED_COUNT; i++) {
        SAFE_EXECUTE(TAG, "LED strip set pixel", led_strip_set_pixel(led_strip, i, LED_RED_R, LED_RED_G, LED_RED_B));
      }
      SAFE_EXECUTE(TAG, "LED strip refresh", led_strip_refresh(led_strip));
      ESP_LOGI(TAG, "LED: ON (HOLDER) - all %d side LEDs red", HP_LED_COUNT);
    } else {
      // RUNNER: LEDs off (already cleared above)
      ESP_LOGI(TAG, "LED: OFF (RUNNER)");
    }
  }

  // Create background tasks for game operation
  // beacon_task: sends IR beacons when this device is the holder
  xTaskCreatePinnedToCore(beacon_task, "beacon", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL, tskNO_AFFINITY);
  // uart_rx_task: processes incoming IR data and sends pass requests
  xTaskCreatePinnedToCore(uart_rx_task,"ir_rx", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL, tskNO_AFFINITY);

  // Wait for countdown to complete before starting game
  ESP_LOGI(TAG, "Waiting for countdown to complete...");
  // The countdown is already handled by the test harness, so we just need to wait a bit
  vTaskDelay(pdMS_TO_TICKS(GAME_START_DELAY_MS));
  ESP_LOGI(TAG, "Countdown complete! Game starting...");
  g_game_active = true;  // Enable game logic - this allows tasks to start processing

  ESP_LOGI(TAG,"Start as %s", g_role==ROLE_HOLDER?"HOLDER":"RUNNER");
  // Main loop: tasks do all the work, main thread just stays idle
  // This is a common pattern in embedded systems - main thread initializes, tasks run
  for(;;) vTaskDelay(pdMS_TO_TICKS(1000));
}
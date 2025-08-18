#pragma once

// ---- IR config ----
#define IR_BAUD         2400
#define IR_BIT_US       (1000000 / IR_BAUD)   // 416 at 2400 bps

// ---- Packet: ZT | LEN | PAYLOAD | CRC8(0x31 over LEN+PAYLOAD) ----
#define IR_PREAMBLE0    0x5A
#define IR_PREAMBLE1    0x54
#define IR_MAC_LEN      6
#define IR_MAX_PAYLOAD  32     // safety cap for future growth
#define IR_UART_RX_BUF_SZ 512  // more headroom now that we read longer frames

// ---- Duplicate suppression (for continuous "potato beacons") ----
#ifndef IR_DEDUPE_WINDOW_MS
#define IR_DEDUPE_WINDOW_MS 200   // ignore repeats from same MAC seen within this window
#endif
#ifndef IR_DEDUPE_CAPACITY
#define IR_DEDUPE_CAPACITY 8      // max distinct MACs to track (tiny LRU)
#endif

// ---- Future: enable 1-byte SEQ in payload (LEN=IR_MAC_LEN+1) ----
// When enabled on TX later, payload layout becomes: [SEQ][MAC(6)]
#ifndef IR_ENABLE_SEQ_FUTURE
#define IR_ENABLE_SEQ_FUTURE 1     // RX is SEQ-aware if a 7-byte payload arrives
#endif

// RX capture window bounds (ns)
// min ~ 2 us to ignore tiny glitches; max ≈ frame_len + margin (~20 ms like original)
#define IR_RX_MIN_NS    1000
#define IR_RX_MAX_NS   50000000UL // 50 ms (safe for 2+ bytes; we'll go bigger for full packet later) is plenty for 1 byte @ 2400 bps; bump later for multi-byte frames

// Debug gate (set to 1 while tuning; 0 for gameplay)
#ifndef IR_DEBUG
#define IR_DEBUG 1
#endif

#ifndef IR_SELF_FILTER
#define IR_SELF_FILTER 1   // 1=ignore frames whose MAC == our MAC
#endif

// ---- ESP-NOW ACK config ----
#ifndef IR_USE_ESPNOW_ACK
#define IR_USE_ESPNOW_ACK 1       // 1 = enable IR->ESP-NOW ack path
#endif
#ifndef ESPNOW_CHANNEL
#define ESPNOW_CHANNEL 1          // all devices must use the same channel
#endif

#if IR_DEBUG
  #define IR_LOGI(tag, fmt, ...) ESP_LOGI(tag, fmt, ##__VA_ARGS__)
#else
  #define IR_LOGI(tag, fmt, ...) do{}while(0)
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * IR test with both sender and receiver on same device.
 * Receiver is always running, button press sends "Z" signal.
 * Features: CPU-minimal RX, prebuilt TX symbols, validation-based decoding.
 */
void ir_simple_test_main(void);

/**
 * Send a single byte over IR (builds symbols once for the byte)
 */
void ir_send_byte(uint8_t byte);

/**
 * Send a frame of bytes over IR (builds symbols once for the whole frame)
 */
void ir_send_frame(const uint8_t* data, size_t len);

/**
 * Decode symbols to byte (pure function for unit testing)
 */
uint8_t decode_symbols_to_byte(const void* symbols, size_t count);

#ifdef __cplusplus
}
#endif

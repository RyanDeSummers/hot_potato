#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ZT_MAX_PEERS  12
#define ZT_JOIN_MS    3000   // join window
#define ZT_COUNTDOWN_S 5     // display sync countdown seconds

// Test messages (do NOT collide with your existing 0xA* codes)
#define ZT_MSG_HELLO   0xC1  // peer discovery
#define ZT_MSG_ROSTER  0xC2  // host → all: count + macs[]
#define ZT_MSG_START   0xC3  // host → all: delta_us + initial_holder_mac

// Button configuration
#define ZT_HOST_BUTTON_GPIO GPIO_NUM_39  // M5Stack FIRE button A (GPIO 39)

typedef enum { ZT_ROLE_HOLDER=0, ZT_ROLE_RUNNER=1 } zt_role_t;

typedef struct {
  uint8_t macs[ZT_MAX_PEERS][6];
  uint8_t count;
} zt_roster_t;

// Optional LCD countdown hook (called during COUNTDOWN, ~4Hz)
typedef void (*zt_countdown_render_cb_t)(int secs_remaining,
                                         const uint8_t initial_holder_mac[6],
                                         const zt_roster_t* roster);
void zt_set_countdown_cb(zt_countdown_render_cb_t cb);

// Call from Mode A/B main after ESPNOW init.
// Blocks until roster is sealed and START received/emitted.
// Returns chosen initial role and filled roster.
void zt_join_and_countdown(zt_role_t* out_initial_role, zt_roster_t* out_roster, bool is_host);

// Distinct-before-repeat helper API (holder-side policy)
void zt_distinct_init(const zt_roster_t* roster, const uint8_t* current_holder_mac);
bool zt_is_candidate_allowed(const uint8_t* candidate_mac);   // true if not yet seen this round
void zt_record_grant(const uint8_t* new_holder_mac);          // mark winner; rotate round if complete
void zt_get_initial_holder(uint8_t out_mac[6]);               // for UI or init convenience
// Liveness + failsafe
void zt_set_alive_timeout_ms(uint32_t ms);
void zt_round_maybe_relax(uint32_t empty_windows, const uint8_t* current_holder_mac);

#ifdef __cplusplus
}
#endif
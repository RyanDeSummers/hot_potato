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
#define ZT_MSG_HELLO   0xC1  // peer discovery (legacy, no longer used)
#define ZT_MSG_ANNOUNCE 0xC4 // host → all: "I'm ready for countdown, send JOIN"
#define ZT_MSG_JOIN    0xC5  // peer → host: "I want to join countdown"
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

// Countdown roster building (replaces HELLO system)
void zt_join_countdown(const uint8_t* device_mac);

// Distinct-before-repeat helper API (holder-side policy)
void zt_distinct_init(const zt_roster_t* roster, const uint8_t* current_holder_mac);
bool zt_is_candidate_allowed(const uint8_t* candidate_mac);   // true if not yet seen this round
void zt_record_grant(const uint8_t* new_holder_mac);          // mark winner; rotate round if complete

// Fair candidate selection
bool zt_pick_next_candidate(int* out_idx, uint8_t out_mac[6]);
void zt_note_grant_to_idx(int idx);
void zt_on_grant_committed_idx(int idx);
void zt_get_initial_holder(uint8_t out_mac[6]);               // for UI or init convenience

// Additional helper functions for strict distinct-before-repeat
int zt_roster_index_of(const uint8_t* mac);                   // find index of MAC in roster
bool zt_distinct_already_seen(int idx);                       // check if device already seen this round
void zt_distinct_mark_seen(int idx);                          // mark device as seen this round
bool zt_peer_alive(int idx);                                  // check if peer is alive
bool zt_cooldown_ok(const uint8_t* mac);                      // check if cooldown allows transmission
void zt_note_heard_mac(const uint8_t* mac);                   // note that we heard from a MAC address
void zt_round_reset_unresp(void);                             // reset unresponsive strike counters
int zt_alive_count(void);                                     // count alive peers
void zt_liveness_init_all(uint64_t now_us);                   // initialize liveness tracking
uint64_t zt_now_us(void);                                     // get current timestamp

// Authoritative round completion (called from Mode A/B when round completes)
void zt_round_complete_authoritative(void);

// Mode A/B round completion function (internal use)
void ir_downlink_round_complete_authoritative(void);
// Liveness + failsafe
void zt_set_alive_timeout_ms(uint32_t ms);
void zt_round_maybe_relax(uint32_t empty_windows, const uint8_t* current_holder_mac);

#ifdef __cplusplus
}
#endif

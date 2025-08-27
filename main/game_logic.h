// game_logic.h
#pragma once
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

#define MAX_HEALTH 11
#define HEALTH_TICK_RATE 1

typedef struct PlayerState {
    bool has_potato;
    int  health;
    bool alive;
    uint8_t last_sender_mac[6];
} PlayerState;

void game_init(PlayerState* state);
void game_tick(PlayerState* state);
void give_potato(PlayerState* state, uint8_t sender_mac[6]);
void pass_potato(PlayerState* state, uint8_t target_mac[6]);
bool can_pass_to(PlayerState* state, uint8_t target_mac[6]);
bool is_eliminated(PlayerState* state);

#ifdef __cplusplus
}
#endif



extern "C" {
#include "game_logic.h"   // C declarations, wrapped for C++ in the header
}

#include <string.h>
#include <stdio.h>

// Give the *definitions* C linkage too:
extern "C" {

void game_init(PlayerState* state) {
    state->has_potato = false;
    state->health = MAX_HEALTH;
    state->alive = true;
    memset(state->last_sender_mac, 0, sizeof(state->last_sender_mac));
}

void game_tick(PlayerState* state) {
    if (!state->alive) return;

    if (state->has_potato) {
        state->health -= HEALTH_TICK_RATE;
        if (state->health <= 0) {
            state->health = 0;
            state->alive = false;
            printf("Player eliminated!\n");
        }
    }
}

void give_potato(PlayerState* state, uint8_t sender_mac[6]) {
    if (!state->alive) return;

    state->has_potato = true;
    memcpy(state->last_sender_mac, sender_mac, 6);
    printf("Potato received!\n");
}

bool can_pass_to(PlayerState* state, uint8_t target_mac[6]) {
    // No tagbacks: can't pass to last sender
    return memcmp(state->last_sender_mac, target_mac, 6) != 0;
}

bool is_eliminated(PlayerState* state) {
    return !state->alive;
}

} // extern "C"



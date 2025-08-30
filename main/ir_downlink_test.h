#pragma once
#include "zt_test_mode.h"

#ifdef __cplusplus
extern "C" {
#endif

// Mode A (downlink): HOLDER TX IR beacons; RUNNERS RX and send PASS_REQ
void ir_downlink_test_main(void);
// Reinstall Mode-A ESPNOW recv callback after harness
void ir_downlink_install_recv_cb(void);

#ifdef __cplusplus
}
#endif


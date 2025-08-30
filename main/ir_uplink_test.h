#pragma once

#include "zt_test_mode.h"

#ifdef __cplusplus
extern "C" {
#endif

// Mode B entrypoint: holder receives IR; runners send IR with slotted backoff
void ir_uplink_test_main(void);
// Reinstall Mode-B ESPNOW recv callback after test harness
void ir_uplink_install_recv_cb(void);

#ifdef __cplusplus
}
#endif

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Mode B entrypoint: holder receives IR; runners send IR with slotted backoff
void ir_uplink_test_main(void);

#ifdef __cplusplus
}
#endif


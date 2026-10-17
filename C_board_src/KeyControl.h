#ifndef KEY_CONTROL_H
#define KEY_CONTROL_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    uint32_t pressed;
    uint32_t start_count;
    uint32_t stop_count;
    uint32_t rejected;
    uint32_t last_result; // 1=start accepted, 2=stop, 3=clear, 4=rejected
} KeyControlStatus;
extern volatile KeyControlStatus g_key_status;
void KeyControlInit(void);
void KeyControlPoll(void);
#ifdef __cplusplus
}
#endif
#endif

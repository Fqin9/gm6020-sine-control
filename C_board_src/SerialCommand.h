#ifndef SERIAL_COMMAND_H
#define SERIAL_COMMAND_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    uint32_t rx_bytes;
    uint32_t commands;
    uint32_t rejected;
    uint32_t rx_errors;
    uint32_t tx_dropped;
    uint32_t link_fault;
} SerialCommandStatus;
extern volatile SerialCommandStatus g_serial_status;
// USART6 = C board UART1. Call Init once after ControlTaskInit, Poll in main loop.
void SerialCommandInit(void);
void SerialCommandPoll(void);
uint32_t SerialCommandClearFaults(void);
#ifdef __cplusplus
}
#endif
#endif

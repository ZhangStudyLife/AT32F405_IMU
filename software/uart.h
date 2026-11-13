#ifndef UART_H
#define UART_H
#include <stdint.h>

/* Persistent record: reserved is always zero. Defaults: 2 Mbps / 1 kHz / Euler. */
typedef struct {
    uint32_t baud;
    uint16_t rate;
    uint8_t mask, reserved;
} UARTConfig;
extern UARTConfig uart_config;
enum { UART_BAD_ARGUMENT=9, UART_BANDWIDTH, UART_SAVE_FAILED, UART_NOT_READY };
int UART_ConfigCheck(const UARTConfig *config);
void UART_Init(void);
void UART_Process(void);
#endif

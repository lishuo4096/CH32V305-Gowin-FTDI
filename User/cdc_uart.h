/********************************** (C) COPYRIGHT *******************************
 * File Name          : cdc_uart.h
 * Description        : USART3 bridge for FTDI channel B (COM port).
 *                      PB10 = TX (pin 11), PB11 = RX (pin 12).
 *******************************************************************************/
#ifndef __CDC_UART_H
#define __CDC_UART_H

#include "ch32v30x.h"

void cdc_uart_init(void);
void cdc_uart_set_baud(uint32_t baud);
void cdc_uart_set_line(uint8_t data_bits, uint8_t parity, uint8_t stop_bits);
void cdc_uart_write(const uint8_t *p, uint16_t n);
uint16_t cdc_uart_tx_free(void);
void cdc_uart_tick_1ms(void);

#endif /* __CDC_UART_H */

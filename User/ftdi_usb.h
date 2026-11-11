/********************************** (C) COPYRIGHT *******************************
 * File Name          : ftdi_usb.h
 * Description        : FTDI FT2232D/FT232H compatible USB device layer for
 *                      CH32V305 (two interfaces: channel A = MPSSE/JTAG,
 *                      channel B = UART).
 *******************************************************************************/
#ifndef __FTDI_USB_H
#define __FTDI_USB_H

#include "ch32v30x.h"
#include "mpsse.h"

/* interface / port index (wIndex low byte) */
#define FTDI_PORT_A                 0
#define FTDI_PORT_B                 1

/* endpoint routing */
#define FTDI_EP_JTAG_IN             1       /* 0x81 : MPSSE results    */
#define FTDI_EP_JTAG_OUT            2       /* 0x02 : MPSSE commands   */
#define FTDI_EP_UART_IN             3       /* 0x83 : UART rx data     */
#define FTDI_EP_UART_OUT            4       /* 0x04 : UART tx data     */

/* FTDI vendor requests */
#define SIO_RESET                   0x00
#define SIO_MODEM_CTRL              0x01
#define SIO_SET_FLOW_CTRL           0x02
#define SIO_SET_BAUD_RATE           0x03
#define SIO_SET_DATA                0x04
#define SIO_POLL_MODEM              0x05
#define SIO_SET_EVENT_CHAR          0x06
#define SIO_SET_ERROR_CHAR          0x07
#define SIO_SET_LATENCY_TIMER       0x09
#define SIO_GET_LATENCY_TIMER       0x0A
#define SIO_SET_BITMODE             0x0B
#define SIO_READ_PINS               0x0C
#define SIO_READ_EEPROM             0x90
#define SIO_WRITE_EEPROM            0x91
#define SIO_ERASE_EEPROM            0x92

void      ftdi_init(void);
void      ftdi_bus_reset(void);
void      ftdi_tick_1ms(void);

/* returns number of IN bytes to send back, or -1 for STALL */
int16_t   ftdi_setup(uint8_t req_type, uint8_t req, uint16_t value,
                     uint16_t index, uint16_t length);
uint16_t  ftdi_ep0_in(void);

/* bulk OUT payloads from the host (called in interrupt context) */
void      ftdi_jtag_out(const uint8_t *p, uint16_t n);
void      ftdi_uart_out(const uint8_t *p, uint16_t n);

/* main loop: drain the MPSSE command fifo and push results out */
void      ftdi_jtag_poll(void);
uint16_t  ftdi_jtag_free(void);

/* channel B response stream (filled by the UART bridge) */
void      ft_uart_reset(void);
void      ft_uart_byte(uint8_t b);
void      ft_uart_mem(const uint8_t *p, uint16_t n);
uint16_t  ft_uart_len(void);
uint8_t   ft_uart_flush(void);
uint8_t  *ft_uart_buf(void);

uint8_t   ftdi_get_bitmode(uint8_t port);
uint8_t   ftdi_get_latency(uint8_t port);

#endif /* __FTDI_USB_H */

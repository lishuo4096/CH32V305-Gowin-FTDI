/********************************** (C) COPYRIGHT *******************************
 * File Name          : mpsse.h
 * Description        : FT232H/FT2232H style MPSSE engine for CH32V305.
 *                      JTAG byte shifts run on SPI1 hardware (TCK 18 MHz = 144/8),
 *                      bit/TMS/TCK-only operations run on GPIO bit-bang.
 *******************************************************************************/
#ifndef __MPSSE_H
#define __MPSSE_H

#include "ch32v30x.h"

/* ---- response stream (implemented in ftdi_usb.c) ------------------------- */
void      ft_resp_reset(void);              /* start a new packet: {0x02,0x60}   */
uint8_t   ft_resp_byte(uint8_t b);          /* append one byte (auto flush)      */
void      ft_resp_mem(const uint8_t *p, uint16_t n); /* append n bytes (auto flush) */
uint16_t  ft_resp_len(void);                /* bytes pending in current packet   */
uint8_t  *ft_resp_buf(void);                /* raw packet buffer (512 bytes)     */
uint8_t   ft_resp_flush(void);              /* push current packet to EP1        */
uint8_t   ft_resp_busy(void);               /* 1 = EP1 still busy                */

#define FT_PACKET_SIZE      512
#define FT_STATUS_BYTES     2

/* ---- MPSSE engine ------------------------------------------------------- */
void mpsse_init(void);                      /* config clocks/pins                */
void mpsse_reset(void);                     /* SIO_RESET: back to idle state     */
void mpsse_enter(void);                     /* SIO_SET_BITMODE(0x02): own the pins */
void mpsse_exit(void);                      /* leave MPSSE (release pins)        */
void mpsse_execute(const uint8_t *buf, uint16_t len);
uint8_t mpsse_active(void);                 /* 1 while bitmode == 0x02           */

/* set by the vendor-request layer */
void mpsse_set_divisor(uint16_t div);
uint16_t mpsse_get_divisor(void);

#endif /* __MPSSE_H */

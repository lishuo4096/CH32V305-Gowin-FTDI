/********************************** (C) COPYRIGHT *******************************
 * File Name          : ftdi_usb.c
 * Description        : FTDI FT2232D / FT232H compatible device layer for CH32V305.
 *
 *   Channel A (interface 0, class FF) : MPSSE  -> mpsse.c   (EP 0x81 / 0x02)
 *   Channel B (interface 1, class FF) : UART   -> cdc_uart.c (EP 0x83 / 0x04)
 *
 *   The USB descriptors declare VID 0x0403 / PID 0x6010 (FT2232D) with
 *   bcdDevice 0x0500, therefore Windows binds the signed FTDI driver
 *   (ftdibus.sys / ftdiport.sys) automatically and exposes
 *     - a D2XX / libftdi capable device on interface 0
 *     - a virtual COM port on interface 1
 *
 *   Response stream format (FTDI, MPSSE/245 mode):
 *     every USB packet starts with the two status bytes 0x02 0x60
 *******************************************************************************/
#include "debug.h"
#include "ftdi_usb.h"
#include "cdc_uart.h"
#include "ch32v30x_usbhs_device.h"
#include "string.h"

/* ------------------------------------------------------------------------- */
/* EEPROM image (93LC56B, 16 bit words).                                      */
/* Layout proven by tpunix/FT-LINK against ftdibus.sys + Gowin Programmer.    */
/* The visible USB strings come from usb_desc.c; this table mainly tells the  */
/* driver chip type, VID/PID and power data.                                  */
/* ------------------------------------------------------------------------- */
static const uint16_t ftdi_eeprom[64] = {
    0x0800,                                     /* 00 device / config      */
    0x0403,                                     /* 01 vendor id            */
    0x6010,                                     /* 02 product id           */
    0x0500,                                     /* 03 bcdDevice            */
    0x3280,                                     /* 04 bus powered, 100 mA  */
    0x0000,                                     /* 05                      */
    0x0200,                                     /* 06 bcdUSB 2.00          */
    0x1096,                                     /* 07 channel A settings   */
    0x1aa6,                                     /* 08 channel B settings   */
    0x0000,                                     /* 09                      */
    0x0046, 0x0310,                             /* 0A manufacturer string  */
    0x004f, 0x0070, 0x0065, 0x006e,             /* "Open"                  */
    0x002d, 0x0045, 0x0043,                     /* "-EC"                   */
    /* 0x13 product string: must contain the substring "RS232-HS".
     * Gowin's Programmer reads D2XX FT_GetDeviceInfoDetail().Description and,
     * when it does not contain "RS232-HS", falls back to the hard coded cable
     * name "USB Debugger A" (programmer_cli.exe +0x00C00B16).  The genuine
     * FT232HQ based Gowin cable carries the FTDI default string "Dual RS232-HS",
     * so matching it makes the programmer treat us exactly like that cable.
     * (0x031C = string descriptor: 28 bytes = 2 + 13 * 2.)                      */
    0x031c, 0x0044, 0x0075, 0x0061,             /* "Dual"                  */
    0x006c, 0x0020, 0x0052, 0x0053,             /* " RS2"                  */
    0x0032, 0x0033, 0x0032, 0x002d,             /* "32-H"                  */
    0x0048, 0x0053,                             /* "S"                     */
    0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000,
    0x1027                                      /* 3F checksum (FT-LINK)   */
};

/* ------------------------------------------------------------------------- */
/* per port state                                                             */
/* ------------------------------------------------------------------------- */
typedef struct {
    uint8_t  bitmode;
    uint8_t  latency;                           /* ms, 1..255 */
    uint16_t timeout;                           /* elapsed ms counter */
    uint8_t  rsp_pending;                       /* send a status packet after reset */
    uint8_t  flow;
} ftdi_port_t;

static ftdi_port_t m_port[2];

/* Host -> COM loopback, used to exercise the whole channel B data path without
 * a jumper: with this enabled the bytes the host writes to the serial channel
 * are pushed straight back into its IN stream.  Enabled from the host with the
 * debug vendor request 0x0D (no real FTDI device understands that code).  */
static volatile uint8_t m_loopback;
volatile uint32_t ft_loopback_cnt;              /* bytes echoed so far        */

/* Diagnostic switches, set with FT_SetBitMode(handleA, mode, 0x0D):
 *   1 = channel B loopback (bytes come straight back, no UART needed)
 *   3 = SIO_POLL_MODEM reports MSR 0x00 instead of 0x02
 *   5 = channel B heartbeat: answer every pending IN read with the 2 status
 *       bytes each latency period, instead of only once after a reset        */
static volatile uint8_t m_dbg_mode;

/* ------------------------------------------------------------------------- */
/* response streams                                                           */
/* ------------------------------------------------------------------------- */
typedef struct {
    uint8_t  buf[2][FT_PACKET_SIZE];            /* double buffered for DMA  */
    uint16_t len;
    uint8_t  cur;
    uint8_t  ep;
} __attribute__((aligned(4))) ft_stream_t;

static ft_stream_t m_jtag = { { {0} }, 0, 0, FTDI_EP_JTAG_IN };
static ft_stream_t m_uart = { { {0} }, 0, 0, FTDI_EP_UART_IN };

static uint8_t ep_busy(uint8_t ep)
{
    return (USBHS_Endp_Busy[ep] & DEF_UEP_BUSY) ? 1 : 0;
}

/* Start a fresh packet: two status bytes first. */
static void st_reset(ft_stream_t *s)
{
    s->buf[s->cur][0] = 0x02;
    s->buf[s->cur][1] = 0x60;
    s->len = FT_STATUS_BYTES;
}

/*
 * Hand the current packet to the endpoint.  Returns 0 on success, 1 if the
 * endpoint stayed busy (data is kept and retried on the next call).
 */
/*
 * Hand the buffer to the endpoint even when it only holds the two status
 * bytes.  The FTDI driver reads the IN endpoint straight after SIO_RESET and
 * hangs ("port busy") if nothing ever comes back; the reference FT-LINK
 * implementation solves this by arming a latency timer after the reset.
 */
static uint8_t st_send(ft_stream_t *s)
{
    uint32_t guard;

    /* Never spin for long here: this runs from the main loop, and a long
       stall makes the host's D2XX/COM timeouts expire (the "device is busy"
       symptom).  A short spin covers the normal case; otherwise keep the
       data and let the next tick retry.                                     */
    guard = 2000u;
    while (ep_busy(s->ep)) {
        if (--guard == 0) {
            return 1;                           /* retry later, data kept */
        }
    }
    USBHS_Endp_DataUp(s->ep, s->buf[s->cur], s->len, DEF_UEP_DMA_LOAD);
    s->len = 0;
    s->cur ^= 1;
    st_reset(s);
    return 0;
}

static uint8_t st_flush(ft_stream_t *s)
{
    if (s->len <= FT_STATUS_BYTES) {
        return 0;                               /* nothing but the header */
    }
    return st_send(s);
}

/*
 * Append one byte to the stream.  When the packet is full the old code called
 * st_flush() once and, if the endpoint was still busy, silently DROPPED the
 * byte -- that is exactly what a long programming transfer cannot survive
 * (the host then parses a shifted stream: "verify failed", "stuck at erase").
 *
 * A real FT2232H applies back-pressure here (the MPSSE clock stalls until the
 * host takes the packet), and the FT-LINK reference cable does the same
 * ("while(jtag_resp_size>2);").  So: wait for the endpoint to be handed over.
 * Only when the host has stopped reading altogether (~1 s) do we resync the
 * packet, so a dead reader can never wedge the main loop forever.
 */
#define ST_BP_ATTEMPTS 20000u

static void st_put(ft_stream_t *s, uint8_t b)
{
    if (s->len >= FT_PACKET_SIZE) {
        uint32_t tries = 0;

        while (st_flush(s) != 0) {
            if (++tries >= ST_BP_ATTEMPTS) {
                s->len = FT_STATUS_BYTES;       /* host stalled: resync */
                break;
            }
        }
    }
    if (s->len < FT_PACKET_SIZE) {
        s->buf[s->cur][s->len] = b;
        s->len++;
    }
}

static void st_put_mem(ft_stream_t *s, const uint8_t *p, uint16_t n)
{
    uint16_t i;

    for (i = 0; i < n; i++) {
        st_put(s, p[i]);
    }
}

/* ---- response trace (diagnostics only) ---------------------------------- *
 * Every byte that leaves this device on channel A (MPSSE) is recorded, so the
 * exact byte stream the host received can be reconstructed from the debugger
 * without any host-side capture:
 *      x/512bx &m_rsp_ring        (newest at m_rsp_cnt & 511)
 *      p m_rsp_cnt
 * This is what makes "the host reads the wrong ID / no data at all" decidable:
 * compare what we appended here against what the host printed.               */
volatile uint8_t  m_rsp_ring[512];
volatile uint32_t m_rsp_cnt;

static void rsp_trace(uint8_t b)
{
    m_rsp_ring[m_rsp_cnt & 511u] = b;
    m_rsp_cnt++;
}

/* ---- channel A (MPSSE) public stream API, used by mpsse.c --------------- */
void ft_resp_reset(void)
{
    st_reset(&m_jtag);
}

uint8_t ft_resp_byte(uint8_t b)
{
    rsp_trace(b);
    st_put(&m_jtag, b);
    return 0;
}

void ft_resp_mem(const uint8_t *p, uint16_t n)
{
    uint16_t i;

    for (i = 0; i < n; i++) {
        rsp_trace(p[i]);
    }
    st_put_mem(&m_jtag, p, n);
}

uint16_t ft_resp_len(void)
{
    return m_jtag.len;
}

uint8_t *ft_resp_buf(void)
{
    return m_jtag.buf[m_jtag.cur];
}

uint8_t ft_resp_flush(void)
{
    return st_flush(&m_jtag);
}

uint8_t ft_resp_busy(void)
{
    return ep_busy(m_jtag.ep);
}

/* ---- channel B (UART) public stream API, used by cdc_uart.c ------------- */
void ft_uart_reset(void)
{
    st_reset(&m_uart);
}

void ft_uart_byte(uint8_t b)
{
    st_put(&m_uart, b);
}

void ft_uart_mem(const uint8_t *p, uint16_t n)
{
    st_put_mem(&m_uart, p, n);
}

uint16_t ft_uart_len(void)
{
    return m_uart.len;
}

uint8_t ft_uart_flush(void)
{
    return st_flush(&m_uart);
}

uint8_t *ft_uart_buf(void)
{
    return m_uart.buf[m_uart.cur];
}

/* ------------------------------------------------------------------------- */
/* vendor request handling                                                    */
/* ------------------------------------------------------------------------- */
#define EP0B                USBHS_EP0_Buf

/* EEPROM read continuation state (the host may read it as one block) */
static uint16_t m_ee_off;
static uint16_t m_ee_remain;

/* Trace of the vendor requests the host driver sends.  The FTDI driver walks a
 * fixed sequence when it opens or closes a device (reset, read EEPROM, set
 * latency, purge, ...), so the fastest way to see what it is waiting for is to
 * read this ring back.  Two ways in:
 *   1. debugger:  x/256hx &ftdi_dbg          (eight words per slot)
 *   2. user mode: FT_ReadEE(handle, 0x1000 + i) returns ftdi_dbg[i]
 * Eight words per slot: {type<<8|req, value, index, length, sequence, result,
 * ms tick when the request arrived, 0}.                                     */
#define FTDI_DBG_SLOTS  32
#define FTDI_DBG_STRIDE 8
volatile uint16_t ftdi_dbg[FTDI_DBG_SLOTS * FTDI_DBG_STRIDE];
volatile uint32_t ftdi_dbg_cnt;
volatile uint32_t ftdi_ms_tick;                 /* incremented by tick_1ms    */

static uint8_t eeprom_byte(uint16_t off)
{
    uint16_t word;

    if (off & 1) {
        word = (off >> 1) < 64 ? ftdi_eeprom[off >> 1] : 0;
        return (uint8_t)(word >> 8);
    }
    word = (off >> 1) < 64 ? ftdi_eeprom[off >> 1] : 0;
    return (uint8_t)(word & 0xFF);
}

static void eeprom_fill(uint16_t n)
{
    uint16_t i;

    for (i = 0; i < n; i++) {
        EP0B[i] = eeprom_byte((uint16_t)(m_ee_off + i));
    }
}

/*
 * Called for every further EP0 IN token of a vendor request: continues a
 * block EEPROM read.  Returns the number of bytes placed in the EP0 buffer.
 */
uint16_t ftdi_ep0_in(void)
{
    uint16_t chunk = (m_ee_remain > 64) ? 64 : m_ee_remain;

    if (chunk == 0) {
        return 0;
    }
    eeprom_fill(chunk);
    m_ee_off += chunk;
    m_ee_remain -= chunk;
    return chunk;
}

uint8_t ftdi_get_bitmode(uint8_t port)
{
    return m_port[port & 1].bitmode;
}

uint8_t ftdi_get_latency(uint8_t port)
{
    return m_port[port & 1].latency;
}

/* ------------------------------------------------------------------------- */
/* FTDI baud divisor -> real baud rate                                        */
/* ------------------------------------------------------------------------- */
/*  The host never sends a baud rate: SIO_SET_BAUD_RATE (0x03) carries the FTDI
 *  *divisor* -- baud = 3000000 / divisor.  wValue holds the 14 bit integer
 *  divisor plus 2 fraction bits (0.5 / 0.25 / 0.125), wIndex is the channel.
 *  Passing the raw divisor to the USART would program 26 baud at 115200.
 */
static uint32_t ft_baud_from_divisor(uint16_t value)
{
    uint32_t d  = (uint32_t)(value & 0x3FFFu);
    uint32_t fr = (uint32_t)((value >> 14) & 0x03u);
    uint32_t x8;

    if (d == 0u) {
        return 3000000u;                    /* FTDI's encoding for 3 Mbaud */
    }
    x8 = d * 8u + ((fr == 1u) ? 4u : (fr == 2u) ? 2u : (fr == 3u) ? 1u : 0u);

    /* 3000000 * 8 = 24000000 still fits a uint32_t, so no 64 bit divide */
    return (24000000u + (x8 / 2u)) / x8;
}

static int16_t ftdi_setup_body(uint8_t req_type, uint8_t req, uint16_t value,
                               uint16_t index, uint16_t length)
{
    /*  The FTDI driver numbers the two ports from 1, exactly like the tpunix
     *  FT-LINK reference: wIndex 1 -> channel A (MPSSE), wIndex 2 -> channel B
     *  (the CDC serial port); wIndex 0 is treated as channel A.  Deriving the
     *  channel as (index & 1) instead would swap the two ports, so a serial
     *  port open would reconfigure the JTAG side and vice versa.             */
    uint8_t port = (uint8_t)(((index & 0xFFu) >= 2u) ? FTDI_PORT_B : FTDI_PORT_A);
    uint16_t word;

    (void)req_type;
    (void)length;

    switch (req) {
    case SIO_RESET:                             /* 0x00 */
        /* Only the channel this request arrived on may be disturbed: opening
           the VCP side (interface 1) must never clear channel A's MPSSE mode,
           otherwise the JTAG programmer loses its cable the moment a serial
           terminal touches the COM port.                                   */
        if (value == 0) {                       /* 0 = reset this channel */
            m_port[port].bitmode = 0;
            if (m_dbg_mode != 2) {
                m_port[port].rsp_pending = 1;
            }
            if (port == FTDI_PORT_A) {
                mpsse_exit();
                ft_resp_reset();
                USBHS_Endp_Busy[FTDI_EP_JTAG_IN] &= (uint8_t)(~DEF_UEP_BUSY);
            } else {
                ft_uart_reset();
                USBHS_Endp_Busy[FTDI_EP_UART_IN] &= (uint8_t)(~DEF_UEP_BUSY);
            }
        } else if (value == 1 || value == 3) {   /* 1 = purge rx, 3 = both */
            if (port == FTDI_PORT_A) {
                ft_resp_reset();
                USBHS_Endp_Busy[FTDI_EP_JTAG_IN] &= (uint8_t)(~DEF_UEP_BUSY);
            } else {
                ft_uart_reset();
                USBHS_Endp_Busy[FTDI_EP_UART_IN] &= (uint8_t)(~DEF_UEP_BUSY);
            }
        }
        /* 2 = purge tx, nothing buffered on our side */
        m_port[0].timeout = m_port[0].latency;
        m_port[1].timeout = m_port[1].latency;
        return 0;

    case SIO_MODEM_CTRL:                        /* 0x01 */
        return 0;

    case SIO_SET_FLOW_CTRL:                     /* 0x02 */
        m_port[port].flow = (uint8_t)(value >> 8);
        return 0;

    case SIO_SET_BAUD_RATE:                     /* 0x03 */
        cdc_uart_set_baud(ft_baud_from_divisor(value));
        return 0;

    case SIO_SET_DATA:                          /* 0x04 */
        cdc_uart_set_line((uint8_t)(value & 0x0F),
                          (uint8_t)((value >> 8) & 0x0F),
                          (uint8_t)((value >> 11) & 0x0F));
        return 0;

    case SIO_POLL_MODEM:                        /* 0x05 : 2 bytes */
        EP0B[0] = (m_dbg_mode == 3) ? 0x00 : 0x02;  /* MSR: DSR asserted     */
        EP0B[1] = 0x60;                         /* LSR: THRE | TEMT (idle)  */
        return 2;

    case SIO_SET_EVENT_CHAR:                    /* 0x06 */
    case SIO_SET_ERROR_CHAR:                    /* 0x07 */
        return 0;

    case SIO_SET_LATENCY_TIMER:                 /* 0x09 */
        m_port[port].latency = (uint8_t)value;
        if (m_port[port].latency == 0) m_port[port].latency = 1;
        m_port[port].timeout = m_port[port].latency;
        return 0;

    case SIO_GET_LATENCY_TIMER:                 /* 0x0A : 1 byte */
        EP0B[0] = m_port[port].latency;
        return 1;

    case SIO_SET_BITMODE: {                     /* 0x0B : channel A only */
        uint8_t mode = (uint8_t)(value >> 8);

        /* Debug hook: FT_SetBitMode(handle, flag, 0x0D) toggles the channel B
           loopback.  0x0D is not a real FTDI bit mode, so no normal host ever
           sends it, and D2XX is the only way to reach this from user mode
           (FTDI has no raw vendor request API).                             */
        if (mode == 0x0D) {
            m_dbg_mode = (uint8_t)(value & 0xFFu);
            m_loopback = (m_dbg_mode == 1) ? 1 : 0;
            return 0;
        }
        if (port != FTDI_PORT_A) {
            return 0;                           /* channel B is a plain UART */
        }
        if (mode == 0x02) {
            m_port[port].bitmode = 0x02;
            mpsse_enter();
        } else {
            m_port[port].bitmode = 0;
            mpsse_exit();
        }
        return 0;
    }

    case SIO_READ_PINS:                         /* 0x0C : 1 byte */
        EP0B[0] = (uint8_t)((GPIOA->INDR & 0x0F) | ((m_port[0].bitmode == 0x02) ? 0x00 : 0x00));
        return 1;

    case SIO_READ_EEPROM:                       /* 0x90 : 2 bytes */
        /* Word offsets from 0x1000 up read the request trace ring instead of
           the EEPROM, so a plain user mode test (FT_ReadEE) can dump it.  */
        if (index >= 0x1000 &&
            (uint32_t)(index - 0x1000) < (FTDI_DBG_SLOTS * FTDI_DBG_STRIDE)) {
            word = ftdi_dbg[index - 0x1000];
        } else if (index < 64) {
            word = ftdi_eeprom[index];
        } else {
            word = 0x0000;
        }
        EP0B[0] = (uint8_t)(word & 0xFF);
        EP0B[1] = (uint8_t)(word >> 8);
        return 2;

    case SIO_WRITE_EEPROM:                      /* 0x91 : accepted, ignored */
    case SIO_ERASE_EEPROM:                      /* 0x92 */
        return 0;

    default:
        break;
    }
    return -1;                                  /* STALL */
}

/*
 * Thin wrapper: runs the request and records {type,req,value,index,length,seq,
 * result,ms} in the trace ring so the host side can replay what the driver asked
 * for (FT_ReadEE(0x1000+i) or x/256hx &ftdi_dbg, then ftdi_dbg_cnt).
 */
int16_t ftdi_setup(uint8_t req_type, uint8_t req, uint16_t value,
                   uint16_t index, uint16_t length)
{
    int16_t r = ftdi_setup_body(req_type, req, value, index, length);
    uint32_t slot;

    /* Reading the trace ring must not overwrite the trace ring. */
    if (req == SIO_READ_EEPROM && index >= 0x1000) {
        return r;
    }

    slot = (ftdi_dbg_cnt & (FTDI_DBG_SLOTS - 1u)) * FTDI_DBG_STRIDE;

    ftdi_dbg[slot + 0] = (uint16_t)(((uint16_t)req_type << 8) | req);
    ftdi_dbg[slot + 1] = value;
    ftdi_dbg[slot + 2] = index;
    ftdi_dbg[slot + 3] = length;
    ftdi_dbg[slot + 4] = (uint16_t)(ftdi_dbg_cnt & 0xFFFFu);
    ftdi_dbg[slot + 5] = (uint16_t)r;
    ftdi_dbg[slot + 6] = (uint16_t)(ftdi_ms_tick & 0xFFFFu);
    ftdi_dbg[slot + 7] = 0;
    ftdi_dbg_cnt++;
    return r;
}

/* ------------------------------------------------------------------------- */
/* bulk endpoints                                                             */
/* ------------------------------------------------------------------------- */
/*
 * The MPSSE command stream is buffered here (append only, never blocks) and
 * executed from the main loop by ftdi_jtag_poll().  Executing a long command
 * list or waiting for the host to drain a response inside the USB interrupt
 * would stall the endpoint, so it is deliberately deferred.
 */
/* 8 KB: holds a full Gowin embFlash page (1754 B) plus MPSSE headers without
 * fragmenting across ftdi_jtag_poll() calls.  The 18 KB erase stream is
 * handled by the burst-clock path in mpsse.c and does not need to fit here.
 * CH32V305GBU6 has 32 KB SRAM with a 2 KB stack, so this is the ceiling. */
#define JTAG_CMD_FIFO_SIZE          8192

static uint8_t  jcmd[JTAG_CMD_FIFO_SIZE];
static volatile uint16_t jcmd_head, jcmd_tail;

static uint16_t jcmd_used(void)
{
    uint16_t h = jcmd_head;
    uint16_t t = jcmd_tail;

    return (uint16_t)((h >= t) ? (h - t) : (JTAG_CMD_FIFO_SIZE - t + h));
}

uint16_t ftdi_jtag_free(void)
{
    return (uint16_t)(JTAG_CMD_FIFO_SIZE - 1 - jcmd_used());
}

void ftdi_jtag_out(const uint8_t *p, uint16_t n)
{
    uint16_t i;

    if (m_port[FTDI_PORT_A].bitmode != 0x02) {
        return;                                 /* not in MPSSE mode */
    }
    for (i = 0; i < n; i++) {
        uint16_t nh = (uint16_t)((jcmd_head + 1) % JTAG_CMD_FIFO_SIZE);

        if (nh == jcmd_tail) {
            break;                              /* caller keeps EP NAKed */
        }
        jcmd[jcmd_head] = p[i];
        jcmd_head = nh;
    }
}

void ftdi_jtag_poll(void)
{
    uint16_t n;

    USBHS_Jtag_Out_Poll();

    /*
     * Drain the FIFO with an explicit cursor, never "tail = head".
     * USB IRQ can push more commands while mpsse_execute() runs (a long
     * SPI shift takes hundreds of microseconds), and assigning tail = head
     * would silently discard them.  That is exactly what breaks a large
     * SRAM program/verify or an embFlash erase sequence.
     */
    for (;;) {
        uint16_t head = jcmd_head;
        if (jcmd_tail == head) {
            break;
        }
        if (head > jcmd_tail) {
            n = (uint16_t)(head - jcmd_tail);
        } else {
            n = (uint16_t)(JTAG_CMD_FIFO_SIZE - jcmd_tail);
        }
        /* execute in place: mpsse_execute() only reads the buffer */
        mpsse_execute(&jcmd[jcmd_tail], n);
        jcmd_tail = (uint16_t)((jcmd_tail + n) % JTAG_CMD_FIFO_SIZE);
    }
    /* Keep the latency countdown running while idle, so the FTDI driver
       receives status packets during enumeration/open and can finish. */
    (void)ft_resp_flush();
}

void ftdi_uart_out(const uint8_t *p, uint16_t n)
{
    if (m_loopback) {
        ft_uart_mem(p, n);                      /* echo back, see 0x0D above */
        ft_loopback_cnt += n;
        return;
    }
    cdc_uart_write(p, n);
}

/* ------------------------------------------------------------------------- */
/* 1 ms housekeeping (latency timer, uart batching)                           */
/* ------------------------------------------------------------------------- */
void ftdi_tick_1ms(void)
{
    uint8_t i;

    ftdi_ms_tick++;

    for (i = 0; i < 2; i++) {
        if (m_port[i].timeout) {
            m_port[i].timeout--;
        }
    }

    /*
     * Latency heartbeat.  A real FT232 returns a 2-byte status packet every
     * latency period whenever the host has a pending IN read and there is no
     * payload.  Without that, ftdiport.sys's read URB sits on a NAK and
     * CloseHandle() blocks for the 5 s USB cancel timeout.  Keep answering
     * even when only the status header is armed; only clear rsp_pending once
     * the packet was actually handed to the endpoint.
     */
    if (ft_uart_len() > FT_STATUS_BYTES && m_port[FTDI_PORT_B].timeout == 0) {
        (void)ft_uart_flush();
        m_port[FTDI_PORT_B].timeout = m_port[FTDI_PORT_B].latency;
    } else if (m_port[FTDI_PORT_B].timeout == 0) {
        if (st_send(&m_uart) == 0) {
            m_port[FTDI_PORT_B].rsp_pending = 0;
        }
        m_port[FTDI_PORT_B].timeout = m_port[FTDI_PORT_B].latency;
    }

    /* channel A: same heartbeat so D2XX reads after SIO_RESET always unblock */
    if (ft_resp_len() > FT_STATUS_BYTES && m_port[FTDI_PORT_A].timeout == 0) {
        (void)ft_resp_flush();
        m_port[FTDI_PORT_A].timeout = m_port[FTDI_PORT_A].latency;
    } else if (m_port[FTDI_PORT_A].timeout == 0) {
        if (st_send(&m_jtag) == 0) {
            m_port[FTDI_PORT_A].rsp_pending = 0;
        }
        m_port[FTDI_PORT_A].timeout = m_port[FTDI_PORT_A].latency;
    }

    cdc_uart_tick_1ms();

    /* re-open the COM OUT endpoint once the UART fifo has room again */
    if( ( USBHSD->UEP4_RX_CTRL & USBHS_UEP_R_RES_MASK ) == USBHS_UEP_R_RES_NAK
        && cdc_uart_tx_free() > 64 )
    {
        USBHSD->UEP4_RX_CTRL = ( USBHSD->UEP4_RX_CTRL & ~USBHS_UEP_R_RES_MASK ) | USBHS_UEP_R_RES_ACK;
    }
}

/* ------------------------------------------------------------------------- */
/* init / reset                                                               */
/* ------------------------------------------------------------------------- */
void ftdi_init(void)
{
    m_port[0].latency = 16;
    m_port[0].timeout = 16;
    m_port[0].bitmode = 0;
    m_port[1].latency = 16;
    m_port[1].timeout = 16;
    m_port[1].bitmode = 0;

    mpsse_init();
    cdc_uart_init();

    jcmd_head = jcmd_tail = 0;
    st_reset(&m_jtag);
    st_reset(&m_uart);
}

void ftdi_bus_reset(void)
{
    uint8_t i;

    mpsse_exit();
    jcmd_head = jcmd_tail = 0;
    st_reset(&m_jtag);
    st_reset(&m_uart);
    /* an armed IN packet that the host never took must not keep the
       endpoint marked busy forever */
    USBHS_Endp_Busy[FTDI_EP_JTAG_IN] &= (uint8_t)(~DEF_UEP_BUSY);
    USBHS_Endp_Busy[FTDI_EP_UART_IN] &= (uint8_t)(~DEF_UEP_BUSY);
    for (i = 0; i < 2; i++) {
        m_port[i].bitmode = 0;
        m_port[i].timeout = m_port[i].latency;
    }
}

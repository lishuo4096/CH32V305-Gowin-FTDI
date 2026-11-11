/********************************** (C) COPYRIGHT *******************************
 * File Name          : mpsse.c
 * Description        : FT232H / FT2232H compatible MPSSE engine for CH32V305.
 *
 *   byte-mode data shifts  -> SPI1 hardware + DMA  (TCK = 18 MHz, PCLK2 = 144 MHz / 8)
 *   bit-mode / TMS / clock -> GPIO bit-bang
 *
 *   Reference:
 *     FTDI AN_108 "Command Processor for MPSSE"
 *     OpenOCD  src/jtag/drivers/mpsse.c        (bit semantics, authoritative)
 *     pyftdi   pyftdi/ftdi.py                  (opcode table, 0x8E/0x8F naming)
 *     tpunix/FT-LINK usbuser/jtag_mppse.c      (field proven against Gowin)
 *
 *   Uniform opcode bit layout (verified against AN_108 / OpenOCD / FT-LINK):
 *     bit7 bit6 bit5 bit4 bit3 bit2 bit1 bit0
 *      0    1=TMS  RD  WR  LSB  IN-  BIT  OUT-
 *   -> 0x10..0x1F write only, 0x20..0x2F read only, 0x30..0x3F read+write,
 *      0x4A/0x4B/0x6A/0x6B TMS (always carries one TMS data byte).
 *
 *   Pin map (CH32V305GBU6, QFN28):
 *     PA5  (pin 8)  TCK  <- SPI1_SCK
 *     PA6  (pin 9)  TDO  -> SPI1_MISO
 *     PA7  (pin 10) TDI  <- SPI1_MOSI
 *     PB12 (pin 13) TMS  <- GPIO push-pull
 *******************************************************************************/
#include "debug.h"
#include "mpsse.h"
#include "string.h"
#include "ch32v30x_usbhs_device.h"

/* ------------------------------------------------------------------------- */
/* pin / peripheral map                                                       */
/* ------------------------------------------------------------------------- */
#define JTAG_PORT           GPIOA
#define PIN_TCK             GPIO_Pin_5
#define PIN_TDO             GPIO_Pin_6
#define PIN_TDI             GPIO_Pin_7
#define PIN_JTAG_OUT        (PIN_TCK | PIN_TDI)

#define TMS_PORT            GPIOB
#define PIN_TMS             GPIO_Pin_12

#define SPI_TX_DMA          DMA1_Channel3
#define SPI_RX_DMA          DMA1_Channel2
#define DMA_FLAG_TX         DMA1_FLAG_TC3
#define DMA_FLAG_RX         DMA1_FLAG_TC2
#define DMA_FLAG_CLR        (DMA1_FLAG_GL2 | DMA1_FLAG_GL3)

/* ADBUS bit assignment seen by the host (FT2232 channel A low byte) */
#define ADB_TCK             0x01
#define ADB_TDI             0x02
#define ADB_TDO             0x04
#define ADB_TMS             0x08

/* MPSSE control register bits */
#define CTRL_LOOPBACK       0x01
#define CTRL_DIV5           0x02
#define CTRL_PHASE3         0x04
#define CTRL_RTCK           0x08

/* command attribute flags */
#define F_TMS               0x40    /* drive the TMS pin instead of TDI */
#define F_INN               0x20    /* TDO sampled on the -ve TCK edge  */
#define F_OUTN              0x10    /* data changed on the -ve TCK edge */
#define F_LSB               0x08    /* LSB first                        */
#define F_BIT               0x04    /* bit mode (1..8 bits)             */
#define F_RD                0x02    /* host consumes TDO data           */
#define F_WR                0x01    /* host supplies TDI data           */

/* parser stages */
#define ST_CMD              0
#define ST_PARAM            1
#define ST_DATA             2

/* ------------------------------------------------------------------------- */
/* module state                                                               */
/* ------------------------------------------------------------------------- */
static uint16_t m_div = 8;                  /* clock divisor from 0x86         */
static uint8_t  m_active;                   /* bitmode == 0x02                 */
static uint8_t  m_ctrl = CTRL_DIV5;         /* divide-by-5 clocking enabled    */
static uint8_t  m_spi_pins;                 /* 1 = PA5/6/7 in AF mode          */
static uint8_t  m_adbus_val, m_adbus_dir;
static uint8_t  m_acbus_val, m_acbus_dir;
static uint8_t  m_bb_loops = 8;             /* bit-bang delay loop count       */
static uint8_t  m_ff[FT_PACKET_SIZE] __attribute__((aligned(4)));   /* 0xFF source */
static uint8_t  m_rx[FT_PACKET_SIZE] __attribute__((aligned(4)));   /* SPI rx */
static uint8_t  m_tx[FT_PACKET_SIZE] __attribute__((aligned(4)));   /* SPI tx */

/* ------------------------------------------------------------------------- */
/* command trace (diagnostics only)                                           */
/* ------------------------------------------------------------------------- *
 * Every opcode the host sends is recorded together with its first two
 * parameter bytes, so a failing programming run can be decoded offline
 * instead of guessed at.  Read it from the debugger:
 *      x/256bx &m_trace_cmd        (opcodes, newest at m_trace_cnt & 255)
 *      x/256bx &m_trace_p0         (first parameter byte, 0 if none)
 *      x/256bx &m_trace_p1         (second parameter byte, 0 if none)
 *      x/256bx &m_trace_d0         (first data byte, 0 if none)
 * or print m_trace_cnt to see how many commands have been executed.          */
#define M_TRACE_ENTRIES     256
volatile uint8_t  m_trace_cmd[M_TRACE_ENTRIES];
volatile uint8_t  m_trace_p0[M_TRACE_ENTRIES];
volatile uint8_t  m_trace_p1[M_TRACE_ENTRIES];
volatile uint8_t  m_trace_d0[M_TRACE_ENTRIES];
volatile uint32_t m_trace_cnt;
static   uint32_t m_trace_cur;

/* command parser */
static uint8_t  m_stage = ST_CMD;
static uint8_t  m_cmd;
static uint8_t  m_flags;
static uint8_t  m_nparam;
static uint8_t  m_ngot;
static uint8_t  m_param[2];
static uint8_t  m_bits;                     /* bit count of current command    */
static uint32_t m_total;                    /* byte count of current command   */
static uint32_t m_done_b;                   /* bytes already shifted out       */
static uint8_t m_tap_known, m_tap_state, m_tap_ones;
static uint8_t m_ir_shift, m_ir_active, m_erase_burst;
static uint8_t m_zero __attribute__((aligned(4)));
volatile uint32_t m_erase_burst_count;

/* IEEE 1149.1 state tracking gates the Gowin erase Run-Test optimization.
   Never infer an erase from a transfer length: configuration data uses the
   same write opcodes and must always be shifted verbatim. */
static void tap_step(uint8_t tdi, uint8_t tms)
{
    static const uint8_t next[16][2] = {
        {1,0},{1,2},{3,9},{4,5},{4,5},{6,8},{6,7},{4,8},
        {1,2},{10,0},{11,12},{11,12},{13,15},{13,14},{11,15},{1,2}
    };
    m_tap_ones = tms ? (uint8_t)(m_tap_ones + (m_tap_ones < 5)) : 0;
    if (m_tap_ones >= 5) {
        m_tap_known = 1;
        m_tap_state = 0;
        m_ir_active = 0;
        return;
    }
    if (!m_tap_known) return;
    if (m_tap_state == 11) {
        m_ir_shift = (uint8_t)((m_ir_shift >> 1) | (tdi ? 0x80u : 0u));
    }
    m_tap_state = next[m_tap_state][tms ? 1 : 0];
    if (m_tap_state == 15) m_ir_active = m_ir_shift;
}

/* ------------------------------------------------------------------------- */
/* primitives                                                                 */
/* ------------------------------------------------------------------------- */
#define TCK_HI()            (JTAG_PORT->BSHR = PIN_TCK)
#define TCK_LO()            (JTAG_PORT->BCR  = PIN_TCK)
#define TDI_HI()            (JTAG_PORT->BSHR = PIN_TDI)
#define TDI_LO()            (JTAG_PORT->BCR  = PIN_TDI)
#define TMS_HI()            (TMS_PORT->BSHR  = PIN_TMS)
#define TMS_LO()            (TMS_PORT->BCR   = PIN_TMS)
#define TDO_GET()           ((JTAG_PORT->INDR & PIN_TDO) ? 1 : 0)

/* Buffer one EF-Program word including its following Run-Test clocks.
   Parsing and USB service take place while TCK is idle, outside the frame. */
#define WAVE_BITS 128u
static uint8_t m_wave[WAVE_BITS], m_wave_n, m_wave_active;
static uint8_t m_wave_tdi, m_wave_tms;
volatile uint32_t m_wave_frames;
volatile uint32_t m_wave_dma_errors;
static uint32_t m_wave_a[WAVE_BITS * 2u + 1u] __attribute__((aligned(4)));
static uint32_t m_wave_b[WAVE_BITS * 2u + 1u] __attribute__((aligned(4)));

static uint8_t current_tms(void)
{
    return m_wave_active ? m_wave_tms : ((TMS_PORT->OUTDR & PIN_TMS) != 0);
}

static void wave_flush(void)
{
    uint16_t i, count;
    uint32_t saved_cfgr, saved_paddr, rate, half_ticks, budget;
    DMA_InitTypeDef dma;
    TIM_TimeBaseInitTypeDef timer;
    TIM_OCInitTypeDef compare = {0};
    if (!m_wave_active) return;
    for (i = 0; i < m_wave_n; i++) {
        uint8_t value = m_wave[i];
        uint32_t data = (value & 1u) ? PIN_TDI : ((uint32_t)PIN_TDI << 16);
        uint32_t mode = (value & 2u) ? PIN_TMS : ((uint32_t)PIN_TMS << 16);
        m_wave_a[2*i] = data | ((uint32_t)PIN_TCK << 16);
        m_wave_a[2*i+1] = data | PIN_TCK;
        m_wave_b[2*i] = mode;
        m_wave_b[2*i+1] = mode;
    }
    count = 2u * m_wave_n + 1u;
    m_wave_a[count-1] = (uint32_t)PIN_TCK << 16;
    m_wave_b[count-1] = m_wave_tms ? PIN_TMS : ((uint32_t)PIN_TMS << 16);
    rate = ((m_ctrl & CTRL_DIV5) ? 6000000u : 30000000u) / (1u + m_div);
    if (rate > 4000000u) rate = 4000000u;
    half_ticks = rate ? (144000000u + rate) / (2u * rate) : 65535u;
    if (half_ticks < 18u) half_ticks = 18u;
    if (half_ticks > 65535u) half_ticks = 65535u;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    TIM_Cmd(TIM1, DISABLE);
    TIM_DMACmd(TIM1, TIM_DMA_Update | TIM_DMA_CC1, DISABLE);
    timer.TIM_Period = half_ticks - 1u;
    timer.TIM_Prescaler = 0;
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    timer.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &timer);
    compare.TIM_OCMode = TIM_OCMode_Timing;
    compare.TIM_OutputState = TIM_OutputState_Disable;
    compare.TIM_Pulse = 1;
    TIM_OC1Init(TIM1, &compare);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);
    TIM_GenerateEvent(TIM1, TIM_EventSource_Update);
    TIM_SetCounter(TIM1, half_ticks - 1u);
    TIM_ClearFlag(TIM1, TIM_FLAG_Update | TIM_FLAG_CC1);

    /* TIM1_UP -> DMA1 channel 5 drives TCK/TDI. TIM1_CH1 -> channel 2
       drives TMS one timer tick after each half-cycle starts. TMS thus
       settles during TCK low, an entire half-cycle before the next rise.
       SPI1 RX shares channel 2; save and restore its configuration. */
    DMA_Cmd(DMA1_Channel2, DISABLE);
    DMA_Cmd(DMA1_Channel5, DISABLE);
    saved_cfgr = DMA1_Channel2->CFGR;
    saved_paddr = DMA1_Channel2->PADDR;
    dma.DMA_DIR = DMA_DIR_PeripheralDST;
    dma.DMA_BufferSize = count;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_VeryHigh;
    dma.DMA_M2M = DMA_M2M_Disable;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&JTAG_PORT->BSHR;
    dma.DMA_MemoryBaseAddr = (uint32_t)m_wave_a;
    DMA_Init(DMA1_Channel5, &dma);
    dma.DMA_PeripheralBaseAddr = (uint32_t)&TMS_PORT->BSHR;
    dma.DMA_MemoryBaseAddr = (uint32_t)m_wave_b;
    DMA_Init(DMA1_Channel2, &dma);
    DMA_ClearFlag(DMA1_FLAG_GL2 | DMA1_FLAG_GL5);
    DMA_Cmd(DMA1_Channel2, ENABLE);
    DMA_Cmd(DMA1_Channel5, ENABLE);
    TIM_DMACmd(TIM1, TIM_DMA_Update | TIM_DMA_CC1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
    budget = 2000000u;
    while ((DMA1_Channel2->CNTR || DMA1_Channel5->CNTR) && --budget) { ; }
    TIM_Cmd(TIM1, DISABLE);
    TIM_DMACmd(TIM1, TIM_DMA_Update | TIM_DMA_CC1, DISABLE);
    DMA_Cmd(DMA1_Channel2, DISABLE);
    DMA_Cmd(DMA1_Channel5, DISABLE);
    if (!budget) m_wave_dma_errors++;
    DMA1_Channel2->CFGR = saved_cfgr;
    DMA1_Channel2->PADDR = saved_paddr;
    DMA_ClearFlag(DMA1_FLAG_GL2 | DMA1_FLAG_GL5);
    TCK_LO();
    m_wave_active = 0;
    m_wave_n = 0;
    m_wave_frames++;
}

static inline void bb_delay(void)
{
    uint8_t i = m_bb_loops;
    while (i--) {
        __asm volatile ("nop");
    }
}

static void pins_to_gpio(void)
{
    GPIO_InitTypeDef g;

    if (!m_spi_pins) {
        return;
    }
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) != RESET) {
        ;
    }
    g.GPIO_Speed = GPIO_Speed_2MHz;
    g.GPIO_Pin   = PIN_JTAG_OUT;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(JTAG_PORT, &g);

    g.GPIO_Pin  = PIN_TDO;
    g.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(JTAG_PORT, &g);

    /* Only force TCK low (mode-0 idle).  TDI keeps the level the SPI
       shifter left, so a 0x11 byte shift followed by a 0x13 bit shift
       (Gowin's split DR32) does not see a TDI glitch between commands. */
    JTAG_PORT->BCR = PIN_TCK;
    m_spi_pins = 0;
}

static void pins_to_spi(void)
{
    GPIO_InitTypeDef g;

    if (m_spi_pins) {
        return;
    }
    g.GPIO_Speed = GPIO_Speed_2MHz;
    g.GPIO_Pin   = PIN_JTAG_OUT;
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(JTAG_PORT, &g);

    g.GPIO_Pin  = PIN_TDO;
    g.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(JTAG_PORT, &g);

    m_spi_pins = 1;
}

/*
 * One MPSSE bit.  use_tms selects the TMS pin, otherwise TDI is driven.
 * Exactly one TCK pulse per bit.  Per AN_108 / OpenOCD (mpsse.h):
 *   bit0 = 1 -> NEG_EDGE_OUT: the data is changed on the falling edge
 *   bit0 = 0 -> POS_EDGE_OUT: the data is changed on the rising edge
 * OpenOCD's JTAG mode is (LSB_FIRST | POS_EDGE_IN | NEG_EDGE_OUT), i.e. the
 * classic mode-0 JTAG timing: change TDI/TMS while TCK is low, sample TDO
 * while TCK is high.
 *
 * F_INN (bit2) is deliberately ignored.  The old code inserted an extra
 * falling edge before sampling, which advanced TDO one bit and shifted the
 * whole read stream by +1 (REPORT_JTAG_READ_PATH_SIM §2.5).  FT-LINK's
 * jtag_ll.S never looks at bit2 and works against Gowin, so matching it is
 * the proven behaviour.
 */
static __attribute__((always_inline)) inline uint8_t bb_bit(uint8_t data, uint8_t tms, uint8_t use_tms)
{
    uint8_t tdo;

    if (m_wave_active) {
        if (use_tms) m_wave_tms = tms;
        else m_wave_tdi = data;
        if (m_wave_n == WAVE_BITS) wave_flush();
        if (m_wave_active) {
            m_wave[m_wave_n++] = m_wave_tdi | (m_wave_tms << 1);
            return 0;
        }
    }

    if ((m_flags & F_OUTN) || (!use_tms && !(m_flags & F_WR))) {
        TCK_LO();
        if (use_tms) {
            if (tms) {
                TMS_HI();
            } else {
                TMS_LO();
            }
        } else if (data) {
            TDI_HI();
        } else {
            TDI_LO();
        }
        bb_delay();
        TCK_HI();
        bb_delay();
        tdo = TDO_GET();
    } else {                                /* change on the rising edge */
        TCK_HI();
        bb_delay();
        if (use_tms) {
            if (tms) {
                TMS_HI();
            } else {
                TMS_LO();
            }
        } else if (data) {
            TDI_HI();
        } else {
            TDI_LO();
        }
        bb_delay();
        TCK_LO();
        bb_delay();
        tdo = TDO_GET();
    }
    return tdo;
}

/*
 * Shift 1..8 TDI bits with TMS held at its current level.
 * AN_108: LSB input shifts right into bit7; MSB input shifts left into bit0.
 * A partial LSB result is left aligned; a partial MSB result is right aligned.
 */
static uint8_t bb_bits(uint8_t wdata, uint8_t nbits)
{
    uint8_t i, r = 0, v = wdata;

    for (i = 0; i < nbits; i++) {
        uint8_t b;
        if (m_flags & F_LSB) {
            b = (uint8_t)(v & 0x01);
            v >>= 1;
        } else {
            b = (uint8_t)((v & 0x80) ? 1 : 0);
            v <<= 1;
        }
        uint8_t tdo = bb_bit(b, 0, 0);
        if (m_flags & F_LSB) {
            r = (uint8_t)((r >> 1) | (tdo ? 0x80u : 0u));
        } else {
            r = (uint8_t)((r << 1) | tdo);
        }
    }
    TCK_LO();
    /* Track the completed bits outside the timed GPIO clock loop. */
    if (!(m_tap_known && (m_tap_state == 1 || m_tap_state == 4) &&
          !current_tms())) {
        for (i = 0; i < nbits; i++) {
            tap_step((wdata >> ((m_flags & F_LSB) ? i : (7-i))) & 1u,
                     current_tms());
        }
    }
    TCK_LO();
    return r;
}

/*
 * Shift 1..8 TMS bits with TDI taken from bit7 of the data byte.
 * TMS bits are consumed LSB first (AN_108 / OpenOCD / FT-LINK agree).
 * Return value is MSB aligned.
 */
static uint8_t bb_tms(uint8_t data, uint8_t nbits)
{
    uint8_t i, r = 0;

    if (m_wave_active) {
        m_wave_tdi = (data >> 7) & 1u;
    } else if (data & 0x80) {
        TDI_HI();
    } else {
        TDI_LO();
    }
    for (i = 0; i < nbits; i++) {
        uint8_t tdo = bb_bit(0, (uint8_t)((data >> i) & 0x01), 1);
        r = (uint8_t)((r >> 1) | (tdo ? 0x80u : 0u));
    }
    TCK_LO();
    for (i = 0; i < nbits; i++) {
        tap_step((data >> 7) & 1u, (data >> i) & 1u);
    }
    TCK_LO();
    return r;
}

/* ------------------------------------------------------------------------- */
/* SPI1 + DMA                                                                 */
/* ------------------------------------------------------------------------- */
static void spi_run(const uint8_t *tx, uint8_t *rx, uint16_t n)
{
    if (n == 0) {
        return;
    }
    pins_to_spi();

    SPI_TX_DMA->CNTR = n;
    SPI_TX_DMA->MADDR = (uint32_t)tx;
    SPI_RX_DMA->CNTR = n;
    SPI_RX_DMA->MADDR = (uint32_t)rx;

    DMA_ClearFlag(DMA_FLAG_CLR);
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);
    /* RX first so the first shifted-in byte cannot hit an OVR before the
       RX DMA is armed */
    DMA_Cmd(SPI_RX_DMA, ENABLE);
    DMA_Cmd(SPI_TX_DMA, ENABLE);

    while (DMA_GetFlagStatus(DMA_FLAG_TX) == RESET) {
        ;
    }
    while (DMA_GetFlagStatus(DMA_FLAG_RX) == RESET) {
        ;
    }
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) != RESET) {
        ;
    }

    DMA_Cmd(SPI_TX_DMA, DISABLE);
    DMA_Cmd(SPI_RX_DMA, DISABLE);
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, DISABLE);
    DMA_ClearFlag(DMA_FLAG_CLR);

    (void)SPI1->STATR;
    (void)SPI1->DATAR;
}

/*
 * Byte mode transfer.  send != 0 -> the bytes in "in" are shifted out on TDI,
 * otherwise 0xFF is shifted (JTAG: TDI high while only clocking).
 * Received bytes are appended to the response stream when the host reads.
 */
static uint8_t bitrev8(uint8_t b)
{
    b = (uint8_t)(((b & 0xF0u) >> 4) | ((b & 0x0Fu) << 4));
    b = (uint8_t)(((b & 0xCCu) >> 2) | ((b & 0x33u) << 2));
    b = (uint8_t)(((b & 0xAAu) >> 1) | ((b & 0x55u) << 1));
    return b;
}

static void spi_bytes(const uint8_t *in, uint32_t n, uint8_t send)
{
    uint32_t off = 0;

    /* Select the clock engine by logical transaction, not USB fragment size.
       Keep short DR/IR shifts and residual bits on GPIO; Run-Test and bulk
       data retain SPI/DMA. This avoids a peripheral remap inside DR32. */
    if (m_wave_active || (m_total <= 8u && n <= 8u &&
        !(m_tap_known && m_tap_state == 1 && !current_tms()))) {
        pins_to_gpio();
        while (off < n) {
            uint8_t value = bb_bits(send ? in[off] : 0xFFu, 8);
            if (m_flags & F_RD) ft_resp_byte(value);
            off++;
        }
        return;
    }

    /*
     * SPI mode 0 (CPOL = CPHA = 0) reproduces NEG_EDGE_OUT + POS_EDGE_IN:
     * the data changes on the falling TCK edge and TDO is sampled on the
     * rising edge.  That is exactly the standard JTAG clocking and the mode
     * every MPSSE host uses (OpenOCD: JTAG_MODE = LSB_FIRST | POS_EDGE_IN |
     * NEG_EDGE_OUT), so this is the fast path.
     */
    /*
     * F_INN is ignored (see bb_bit): FT-LINK never samples on the falling
     * edge and works against Gowin, and the old falling-edge sample shifted
     * the TDO stream by +1 bit.  SPI mode 0 (CPOL = CPHA = 0) reproduces
     * NEG_EDGE_OUT + POS_EDGE_IN and is the only path we need.
     */
    while (off < n) {
        uint32_t left = n - off;
        uint16_t chunk = (left > FT_PACKET_SIZE) ? FT_PACKET_SIZE : (uint16_t)left;
        uint16_t i;
        const uint8_t *tx = send ? (in + off) : m_ff;

        if (m_flags & F_LSB) {
            /* the SPI hardware shifts MSB first, so reverse each byte */
            for (i = 0; i < chunk; i++) {
                m_tx[i] = bitrev8(tx[i]);
            }
            tx = m_tx;
        }

        spi_run(tx, m_rx, chunk);

        /* Byte shifts normally remain in Shift-DR or Run-Test-Idle.  Advance
           the tracker as well when a client uses bytes to shift an IR. */
        if (m_tap_known && m_tap_state != 1 && m_tap_state != 4) {
            uint16_t j;
            for (j = 0; j < chunk; j++) {
                uint8_t bit, value = send ? in[off+j] : 0xFF;
                for (bit = 0; bit < 8; bit++) {
                    tap_step((value >> ((m_flags & F_LSB) ? bit : (7-bit))) & 1u,
                             current_tms());
                }
            }
        }

        if (m_flags & F_RD) {
            if (m_flags & F_LSB) {
                for (i = 0; i < chunk; i++) {
                    m_tx[i] = bitrev8(m_rx[i]);
                }
                ft_resp_mem(m_tx, chunk);
            } else {
                ft_resp_mem(m_rx, chunk);
            }
        }
        off += chunk;
    }
}

/* ------------------------------------------------------------------------- */
/* clock divisor                                                              */
/* ------------------------------------------------------------------------- */
void mpsse_set_divisor(uint16_t div)
{
    uint32_t rate;
    uint16_t br;

    m_div = div;

    /* AN_108 / FT2232H clocking:
     *   opcode 0x8A = disable clock divide by 5 -> 60 MHz master clock
     *   opcode 0x8B = enable  clock divide by 5 -> 12 MHz master clock
     * One TCK period costs two master clocks, so
     *   TCK = 30 MHz / (1 + divisor)      (divide by 5 disabled)
     *   TCK =  6 MHz / (1 + divisor)      (divide by 5 enabled)
     * Sanity check against a real FT2232H: divisor 11 (what Gowin's own
     * programmer sends for 2.5 MHz, "86 0b 00") gives 30/12 = 2.5 MHz. The
     * previous version of this function had the two divide-by-5 branches
     * swapped and dropped the factor of two, so at divisor 11 it programmed
     * 1.125 MHz -- 2.2x slower than the host asked for, which is exactly the
     * kind of error that makes a slow operation (embFlash erase, post-program
     * verify) time out on the host side. */
    if (m_ctrl & CTRL_DIV5) {
        rate = 6000000u / (1u + (uint32_t)div);     /* divide by 5 enabled  */
    } else {
        rate = 30000000u / (1u + (uint32_t)div);    /* divide by 5 disabled */
    }

    /* SPI1 hangs off PCLK2 = 144 MHz (core 144 MHz, APB2 = HCLK/1).  The SPI
     * prescaler is a power of two, so the reachable TCK values are
     *     18 / 9 / 4.5 / 2.25 / 1.125 / 0.5625 MHz.
     * 18 MHz is the hardware ceiling here: the next step up (PCLK2/4 = 36 MHz)
     * is past a safe JTAG TCK for the GW1N target, so anything the host asks
     * for that is faster gets clamped.  Thresholds sit on the geometric
     * midpoints, i.e. the host's request is honoured to the nearest rate. */
    if (rate >= 13500000u) {
        br = SPI_BaudRatePrescaler_8;       /* 144 MHz / 8   = 18.00 MHz (ceiling) */
    } else if (rate >= 6750000u) {
        br = SPI_BaudRatePrescaler_16;      /* 144 MHz / 16  =  9.00 MHz */
    } else if (rate >= 3375000u) {
        br = SPI_BaudRatePrescaler_32;      /* 144 MHz / 32  =  4.50 MHz */
    } else if (rate >= 1687500u) {
        br = SPI_BaudRatePrescaler_64;      /* 144 MHz / 64  =  2.25 MHz */
    } else if (rate >= 843750u) {
        br = SPI_BaudRatePrescaler_128;     /* 144 MHz / 128 =  1.125 MHz */
    } else {
        br = SPI_BaudRatePrescaler_256;     /* 144 MHz / 256 =  0.5625 MHz */
    }

    SPI_Cmd(SPI1, DISABLE);
    SPI1->CTLR1 = (uint16_t)((SPI1->CTLR1 & (uint16_t)(~0x0038u)) | br);
    SPI_Cmd(SPI1, ENABLE);

    /* Measured 144 MHz core: the inlined GPIO bit path takes about
       36 core cycles without delay; each extra loop costs about 8 cycles
       per full clock. Keep arithmetic wide until the final clamp. */
    {
        uint32_t cycles = rate ? (144000000u / rate) : 144000000u;
        uint32_t loops = cycles > 36u ? (cycles - 36u + 4u) / 8u : 0u;
        if (loops > 250u) loops = 250u;
        m_bb_loops = (uint8_t)loops;
    }

}

uint16_t mpsse_get_divisor(void)
{
    return m_div;
}

/* ------------------------------------------------------------------------- */
/* command execution                                                          */
/* ------------------------------------------------------------------------- */
static void cmd_pin_write(void)
{
    m_adbus_val = m_param[0];
    m_adbus_dir = m_param[1];

    pins_to_gpio();
    if ((m_adbus_dir & ADB_TCK) != 0) {
        if (m_adbus_val & ADB_TCK) {
            TCK_HI();
        } else {
            TCK_LO();
        }
    }
    if ((m_adbus_dir & ADB_TDI) != 0) {
        if (m_adbus_val & ADB_TDI) {
            TDI_HI();
        } else {
            TDI_LO();
        }
    }
    if ((m_adbus_dir & ADB_TMS) != 0) {
        if (m_adbus_val & ADB_TMS) {
            TMS_HI();
        } else {
            TMS_LO();
        }
    }
}

/* 0x81 / 0x83 : one byte back, ADBUS or ACBUS depending on the opcode. */
static void cmd_pin_read(void)
{
    if (m_cmd == 0x83) {
        ft_resp_byte((uint8_t)(m_acbus_val & m_acbus_dir));
        return;
    }
    if ((m_adbus_dir & ADB_TDO) != 0) {
        ft_resp_byte((uint8_t)(m_adbus_val & m_adbus_dir));
    } else {
        uint8_t v = (uint8_t)(m_adbus_val & m_adbus_dir);
        v = (uint8_t)(v | (TDO_GET() ? ADB_TDO : 0));
        ft_resp_byte(v);
    }
}

/*
 * Decode a fresh opcode.
 *  1 = command finished
 *  0 = more bytes required (m_stage has been advanced)
 */
static uint8_t cmd_begin(uint8_t cmd)
{
    uint8_t f = 0;

    if (m_wave_active && cmd != 0x4b && cmd != 0x11 && cmd != 0x13 && cmd != 0x19)
        wave_flush();

    m_cmd = cmd;
    m_flags = 0;
    m_ngot = 0;
    m_total = 0;
    m_done_b = 0;
    m_erase_burst = 0;

    /* trace: opcode + room for its first two parameter bytes */
    m_trace_cur = m_trace_cnt & (M_TRACE_ENTRIES - 1u);
    m_trace_cmd[m_trace_cur] = cmd;
    m_trace_p0[m_trace_cur] = 0;
    m_trace_p1[m_trace_cur] = 0;
    m_trace_cnt++;

    switch (cmd) {
    case 0x80:
    case 0x82:                              /* set data bits low/high byte */
        m_nparam = 2;
        m_stage = ST_PARAM;
        return 0;

    case 0x81:
    case 0x83:                              /* get data bits low/high byte */
        cmd_pin_read();
        return 1;

    case 0x84:                              /* loopback on */
        m_ctrl |= CTRL_LOOPBACK;
        return 1;

    case 0x85:                              /* loopback off */
        m_ctrl &= (uint8_t)(~CTRL_LOOPBACK);
        return 1;

    case 0x86:                              /* set clock divisor: LO then HI byte
                                               (AN_108 / OpenOCD mpsse.c:726-728:
                                                buffer_write_byte(0x86);
                                                buffer_write_byte(div & 0xff);
                                                buffer_write_byte(div >> 8);)  */
        m_nparam = 2;
        m_stage = ST_PARAM;
        return 0;

    case 0x87:                              /* Send Immediate: flush the reply
                                               now so a host FT_Read issued
                                               right after this opcode does
                                               not wait for the latency timer. */
        ft_resp_flush();
        return 1;

    case 0x88:
    case 0x89:
    case 0x8C:
    case 0x8D:
    case 0x96:
    case 0x97:                              /* accepted, no side effect */
        return 1;

    case 0x8A:                              /* disable divide by 5 */
        m_ctrl &= (uint8_t)(~CTRL_DIV5);
        mpsse_set_divisor(m_div);
        return 1;

    case 0x8B:                              /* enable divide by 5 */
        m_ctrl |= CTRL_DIV5;
        mpsse_set_divisor(m_div);
        return 1;

    case 0x8E:                              /* clock N bits, no data */
        m_nparam = 1;
        m_stage = ST_PARAM;
        return 0;

    case 0x8F:                              /* clock N bytes, no data */
        m_nparam = 2;
        m_stage = ST_PARAM;
        return 0;

    default:
        break;
    }

    /* ---- TMS family: always carries one TMS data byte ---------------- */
    if ((cmd & 0x40) != 0) {
        f |= F_TMS;
        if (cmd & 0x20) {
            f |= F_RD;
        }
        if (cmd & 0x04) {
            f |= F_INN;
        }
        if (cmd & 0x01) {
            f |= F_OUTN;
        }
        m_flags = f;
        m_nparam = 1;                       /* bit count - 1 */
        m_stage = ST_PARAM;
        return 0;
    }

    /* ---- data shift opcodes 0x10..0x3F ------------------------------- */
    if ((cmd & 0xF0) == 0x10 || (cmd & 0xF0) == 0x20 || (cmd & 0xF0) == 0x30) {
        if (cmd & 0x10) {
            f |= F_WR;
        }
        if (cmd & 0x20) {
            f |= F_RD;
        }
        if (cmd & 0x08) {
            f |= F_LSB;
        }
        if (cmd & 0x04) {
            f |= F_INN;
        }
        if (cmd & 0x02) {
            f |= F_BIT;
        }
        if (cmd & 0x01) {
            f |= F_OUTN;
        }
        m_flags = f;
        m_nparam = (f & F_BIT) ? 1 : 2;
        m_stage = ST_PARAM;
        return 0;
    }

    /* ---- unsupported: reply 0xFA + opcode ---------------------------- */
    ft_resp_byte(0xFA);
    ft_resp_byte(cmd);
    return 1;
}

/* All fixed parameter bytes of the current command have been collected. */
static uint8_t cmd_param_done(void)
{
    switch (m_cmd) {
    case 0x80:
    case 0x82:
        cmd_pin_write();
        return 1;

    case 0x86:                              /* divisor = m_param[1]:m_param[0] */
        m_div = (uint16_t)((uint16_t)m_param[0] | ((uint16_t)m_param[1] << 8));
        mpsse_set_divisor(m_div);
        return 1;

    case 0x8E: {                            /* clock m_param[0]+1 bits (1..256) */
        uint16_t k;
        uint16_t n = (uint16_t)m_param[0] + 1u;

        /* FT-LINK only toggles TCK here and leaves TDI alone.  Gowin's
           embFlash erase asks for clocks with TDI held low, so forcing
           TDI high would corrupt the shift. */
        pins_to_gpio();
        for (k = 0; k < n; k++) {
            TCK_LO();
            bb_delay();
            TCK_HI();
            bb_delay();
        }
        TCK_LO();
        return 1;
    }

    case 0x8F:                              /* clock N+1 bytes */
        spi_bytes(NULL, (uint32_t)(m_param[0] | ((uint16_t)m_param[1] << 8)) + 1u, 0);
        return 1;

    default:
        break;
    }

    /* TMS family: pick up the length, then wait for the TMS data byte. */
    if (m_flags & F_TMS) {
        uint8_t nb = (uint8_t)(m_param[0] + 1);

        if (nb > 8) {
            nb = 8;
        }
        m_bits = nb;
        m_stage = ST_DATA;
        return 0;
    }

    if (m_flags & F_BIT) {
        uint8_t nb = (uint8_t)(m_param[0] + 1);

        if (nb > 8) {
            nb = 8;
        }
        m_bits = nb;
    } else {
        m_total = (uint32_t)(m_param[0] | ((uint16_t)m_param[1] << 8)) + 1u;
    }

    /* read only commands carry no host data: run them immediately */
    if ((m_flags & F_WR) == 0) {
        pins_to_gpio();
        if (m_flags & F_BIT) {
            uint8_t r = bb_bits(0xFF, m_bits);
            if (m_flags & F_RD) {
                ft_resp_byte(r);
            }
        } else {
            spi_bytes(NULL, m_total, 0);
        }
        return 1;
    }

    /* Gowin UG290: EFlash erase needs uninterrupted Run-Test clocks for
       >=120 ms. In Run-Test-Idle TDI is ignored by the target, so the clocks
       may run before the host's dummy payload has all arrived. A single
       constant-address DMA transfer avoids USB packet and 512-byte DMA gaps.
       The IR and TAP checks exclude every actual configuration-data shift. */
    if (m_tap_known && m_tap_state == 1 && m_ir_active == 0x75 &&
        !current_tms() &&
        (m_flags & (F_WR|F_RD|F_BIT|F_TMS)) == F_WR &&
        m_total >= 8000u && m_total <= 65535u) {
        SPI_TX_DMA->CFGR &= ~DMA_MemoryInc_Enable;
        SPI_RX_DMA->CFGR &= ~DMA_MemoryInc_Enable;
        spi_run(&m_zero, m_rx, (uint16_t)m_total);
        SPI_TX_DMA->CFGR |= DMA_MemoryInc_Enable;
        SPI_RX_DMA->CFGR |= DMA_MemoryInc_Enable;
        m_erase_burst = 1;
        m_erase_burst_count++;
    }
    m_stage = ST_DATA;
    return 0;
}

/* Consume data bytes of the current command. */
static void cmd_data(const uint8_t *p, uint32_t n)
{
    m_trace_d0[m_trace_cur] = p[0];         /* first data byte of this command */

    if (m_erase_burst) {
        m_done_b += n;
        return;
    }

    if ((m_flags & F_TMS) != 0) {
        uint8_t r;

        pins_to_gpio();
        if (!m_wave_active && m_ir_active == 0x71 && m_tap_known &&
            m_tap_state == 1 && m_cmd == 0x4b && m_bits == 3 && p[0] == 1) {
            m_wave_active = 1;
            m_wave_n = 0;
            m_wave_tdi = (JTAG_PORT->OUTDR & PIN_TDI) != 0;
            m_wave_tms = (TMS_PORT->OUTDR & PIN_TMS) != 0;
        }
        r = bb_tms(p[0], m_bits);
        if (m_flags & F_RD) {
            ft_resp_byte(r);
        }
        return;
    }

    if ((m_flags & F_BIT) != 0) {
        uint8_t r;

        pins_to_gpio();
        r = bb_bits(p[0], m_bits);
        if (m_flags & F_RD) {
            ft_resp_byte(r);
        }
        return;
    }

    spi_bytes(p, n, 1);
    m_done_b += n;
    if (m_wave_active && m_cmd == 0x19 && m_done_b == m_total &&
        m_tap_state == 1 && !current_tms()) wave_flush();
}

/* ------------------------------------------------------------------------- */
/* public entry points                                                        */
/* ------------------------------------------------------------------------- */
void mpsse_init(void)
{
    GPIO_InitTypeDef g;
    SPI_InitTypeDef s;
    DMA_InitTypeDef d;

    memset(m_ff, 0xFF, sizeof(m_ff));

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    g.GPIO_Pin   = PIN_TMS;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(TMS_PORT, &g);
    TMS_LO();

    m_spi_pins = 1;                         /* force the next call to re-init */
    pins_to_gpio();

    /* SPI1: master, 8 bit, mode 0 == JTAG timing */
    SPI_I2S_DeInit(SPI1);
    s.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    s.SPI_Mode              = SPI_Mode_Master;
    s.SPI_DataSize          = SPI_DataSize_8b;
    s.SPI_CPOL              = SPI_CPOL_Low;
    s.SPI_CPHA              = SPI_CPHA_1Edge;
    s.SPI_NSS               = SPI_NSS_Soft;
    s.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;      /* 144 MHz / 8 = 18 MHz */
    s.SPI_FirstBit          = SPI_FirstBit_MSB;
    s.SPI_CRCPolynomial     = 7;
    SPI_Init(SPI1, &s);
    SPI_Cmd(SPI1, ENABLE);

    /* DMA1 channel 2 = SPI1 RX, channel 3 = SPI1 TX */
    DMA_DeInit(SPI_RX_DMA);
    d.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DATAR;
    d.DMA_MemoryBaseAddr     = (uint32_t)m_rx;
    d.DMA_DIR                = DMA_DIR_PeripheralSRC;
    d.DMA_BufferSize         = FT_PACKET_SIZE;
    d.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    d.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    d.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    d.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    d.DMA_Mode               = DMA_Mode_Normal;
    d.DMA_Priority           = DMA_Priority_VeryHigh;
    d.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(SPI_RX_DMA, &d);

    DMA_DeInit(SPI_TX_DMA);
    d.DMA_MemoryBaseAddr     = (uint32_t)m_ff;
    d.DMA_DIR                = DMA_DIR_PeripheralDST;
    DMA_Init(SPI_TX_DMA, &d);

    mpsse_reset();
}

void mpsse_reset(void)
{
    m_wave_active = 0;
    m_wave_n = 0;
    m_active = 0;
    m_stage = ST_CMD;
    m_flags = 0;
    m_bits = 0;
    m_total = 0;
    m_done_b = 0;
    m_erase_burst = 0;
    m_tap_known = 0;
    m_tap_ones = 0;
    m_ngot = 0;
    m_nparam = 0;
    m_adbus_val = 0;
    m_adbus_dir = 0x0B;                     /* TCK/TDI/TMS out, TDO in */
    m_acbus_val = 0;
    m_acbus_dir = 0;
    m_ctrl = CTRL_DIV5;
    m_div = 8;
    pins_to_gpio();
    TCK_LO();
    TDI_LO();
    TMS_LO();
}

void mpsse_enter(void)
{
    mpsse_reset();
    m_active = 1;
}

void mpsse_exit(void)
{
    wave_flush();
    m_active = 0;
    pins_to_gpio();
}

uint8_t mpsse_active(void)
{
    return m_active;
}

void mpsse_execute(const uint8_t *buf, uint16_t len)
{
    uint16_t i = 0;

    while (i < len) {
        if (m_stage == ST_CMD) {
            (void)cmd_begin(buf[i++]);
            continue;
        }

        if (m_stage == ST_PARAM) {
            m_param[m_ngot] = buf[i++];
            if (m_ngot == 0) {
                m_trace_p0[m_trace_cur] = m_param[0];
            } else if (m_ngot == 1) {
                m_trace_p1[m_trace_cur] = m_param[1];
            }
            m_ngot++;
            if (m_ngot < m_nparam) {
                continue;
            }
            if (cmd_param_done()) {
                m_stage = ST_CMD;
            }
            continue;
        }

        /* ST_DATA */
        if ((m_flags & (F_TMS | F_BIT)) != 0) {
            cmd_data(&buf[i], 1);
            i++;
            m_stage = ST_CMD;
            continue;
        }

        {
            uint32_t avail = (uint32_t)(len - i);
            uint32_t left = m_total - m_done_b;

            if (avail > left) {
                avail = left;
            }
            if (avail != 0) {
                cmd_data(&buf[i], avail);
                i = (uint16_t)(i + avail);
            }
            if (m_done_b >= m_total) {
                m_stage = ST_CMD;
            }
        }
    }
}

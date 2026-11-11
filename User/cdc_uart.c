/********************************** (C) COPYRIGHT *******************************
 * File Name          : cdc_uart.c
 * Description        : FTDI channel B UART bridge for CH32V305.
 *
 *   The FT2232D compatible device exposes channel B as a UART.  The FTDI VCP
 *   driver creates the COM port on the PC side, this file bridges that stream
 *   to the on chip USART3 (PB10 = TX, PB11 = RX).
 *
 *     PC  -> EP 0x04 OUT -> tx ring -> USART3 TX (PB10)
 *     PC  <- EP 0x83 IN  <- rx ring <- USART3 RX (PB11)
 *******************************************************************************/
#include "debug.h"
#include "cdc_uart.h"
#include "ftdi_usb.h"

#define UART_RX_RING_SIZE       1024
#define UART_TX_RING_SIZE       1024

static uint8_t  rx_ring[UART_RX_RING_SIZE];
static volatile uint16_t rx_head, rx_tail;
static uint8_t  tx_ring[UART_TX_RING_SIZE];
static volatile uint16_t tx_head, tx_tail;

static uint32_t uart_baud = 115200;

/* ------------------------------------------------------------------------- */
/* ring helpers                                                               */
/* ------------------------------------------------------------------------- */
static uint16_t tx_used(void)
{
    uint16_t h = tx_head;
    uint16_t t = tx_tail;

    return (uint16_t)((h >= t) ? (h - t) : (UART_TX_RING_SIZE - t + h));
}

static uint16_t rx_used(void)
{
    uint16_t h = rx_head;
    uint16_t t = rx_tail;

    return (uint16_t)((h >= t) ? (h - t) : (UART_RX_RING_SIZE - t + h));
}

uint16_t cdc_uart_tx_free(void)
{
    return (uint16_t)(UART_TX_RING_SIZE - 1 - tx_used());
}

/* ------------------------------------------------------------------------- */
/* init                                                                       */
/* ------------------------------------------------------------------------- */
void cdc_uart_init(void)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef  nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    /* PB10 = USART3_TX, PB11 = USART3_RX */
    gpio.GPIO_Pin   = GPIO_Pin_10;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio);

    gpio.GPIO_Pin  = GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &gpio);

    usart.USART_BaudRate            = uart_baud;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3, &usart);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART3, ENABLE);

    nvic.NVIC_IRQChannel                   = USART3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 1;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    rx_head = rx_tail = 0;
    tx_head = tx_tail = 0;
}

/* ------------------------------------------------------------------------- */
/* PC -> USART3                                                               */
/* ------------------------------------------------------------------------- */
void cdc_uart_write(const uint8_t *p, uint16_t n)
{
    uint16_t i;

    for (i = 0; i < n; i++) {
        uint16_t nh = (uint16_t)((tx_head + 1) % UART_TX_RING_SIZE);

        if (nh == tx_tail) {
            break;                              /* caller keeps the endpoint NAKed */
        }
        tx_ring[tx_head] = p[i];
        tx_head = nh;
    }
    USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
}

/* ------------------------------------------------------------------------- */
/* USART3 -> PC                                                               */
/* ------------------------------------------------------------------------- */
static void rx_service(void)
{
    while (rx_tail != rx_head) {
        uint16_t n = rx_used();
        uint16_t chunk;

        /* keep the USB packet boundary logic in ftdi_usb.c happy */
        if (ft_uart_len() >= (FT_PACKET_SIZE - 1)) {
            (void)ft_uart_flush();
        }
        if (ft_uart_len() + 1 >= FT_PACKET_SIZE) {
            break;
        }
        chunk = 1;
        if (n > 64) {
            chunk = 64;
        }
        for (uint16_t i = 0; i < chunk && rx_tail != rx_head; i++) {
            ft_uart_byte(rx_ring[rx_tail]);
            rx_tail = (uint16_t)((rx_tail + 1) % UART_RX_RING_SIZE);
        }
    }
    /* small batches are pushed out by the latency timer */
    if (ft_uart_len() >= (FT_PACKET_SIZE - 1)) {
        (void)ft_uart_flush();
    }
}

void cdc_uart_tick_1ms(void)
{
    rx_service();
    /* flush eagerly once a decent batch accumulated, keeps throughput up */
    if (ft_uart_len() >= 128) {
        (void)ft_uart_flush();
    }
}

/* ------------------------------------------------------------------------- */
/* line settings from the FTDI vendor requests                                */
/* ------------------------------------------------------------------------- */
void cdc_uart_set_baud(uint32_t baud)
{
    USART_InitTypeDef usart;

    if (baud < 300 || baud > 6000000) {
        return;
    }
    uart_baud = baud;
    usart.USART_BaudRate            = baud;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3, &usart);
}

void cdc_uart_set_line(uint8_t data_bits, uint8_t parity, uint8_t stop_bits)
{
    uint16_t wl = USART_WordLength_8b;
    uint16_t par = USART_Parity_No;
    uint16_t sb = USART_StopBits_1;
    USART_InitTypeDef usart;

    switch (data_bits) {
    case 5: wl = USART_WordLength_8b; break;
    case 6: wl = USART_WordLength_8b; break;
    case 7: wl = USART_WordLength_8b; break;
    default: wl = USART_WordLength_8b; break;
    }
    switch (parity) {
    case 1: par = USART_Parity_Odd; break;
    case 2: par = USART_Parity_Even; break;
    case 3: par = USART_Parity_Odd; break;
    case 4: par = USART_Parity_Even; break;
    default: par = USART_Parity_No; break;
    }
    stop_bits = (uint8_t)(stop_bits & 0x0F);
    if (stop_bits == 2 || stop_bits == 3) {
        sb = USART_StopBits_2;
    } else if (stop_bits == 4) {
        sb = USART_StopBits_1_5;
    } else {
        sb = USART_StopBits_1;
    }

    usart.USART_BaudRate            = uart_baud;
    usart.USART_WordLength          = wl;
    usart.USART_StopBits            = sb;
    usart.USART_Parity              = par;
    usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3, &usart);
    (void)data_bits;
}

/* ------------------------------------------------------------------------- */
/* interrupt                                                                  */
/* ------------------------------------------------------------------------- */
void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void USART3_IRQHandler(void)
{
    uint16_t statr = USART3->STATR;

    if (statr & 0x0020) {                       /* RXNE */
        uint8_t d = (uint8_t)(USART3->DATAR & 0xFF);
        uint16_t nh = (uint16_t)((rx_head + 1) % UART_RX_RING_SIZE);

        if (nh != rx_tail) {
            rx_ring[rx_head] = d;
            rx_head = nh;
        }
    }
    if (statr & 0x0080) {                       /* TXE */
        if (tx_tail != tx_head) {
            USART3->DATAR = tx_ring[tx_tail];
            tx_tail = (uint16_t)((tx_tail + 1) % UART_TX_RING_SIZE);
        } else {
            USART3->CTLR1 &= (uint16_t)~0x0080;  /* TXEIE off */
        }
    }
}

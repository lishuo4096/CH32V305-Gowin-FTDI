/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Description        : CH32V305 as a drop-in FTDI FT2232D / FT232H replacement.
 *
 *   Interface 0 (channel A) : MPSSE  -> JTAG for Gowin Programmer / GAO
 *   Interface 1 (channel B) : UART   -> FTDI VCP COM port <-> USART3
 *
 *   Clock plan (16 MHz crystal on the board):
 *     HSE 16 MHz -> PLL x9 -> HCLK 144 MHz   (CH32V305 top speed, zero wait
 *                                             state, and one of the three
 *                                             clocks the RM allows with USB)
 *     PCLK2 = 144 MHz (APB2 = HCLK/1)  -> GPIO bit-bang runs at full speed
 *     PCLK1 =  72 MHz (APB1 = HCLK/2)  -> USART3 for the COM port
 *     SPI1 (APB2 = 144 MHz) / 8  ->  TCK = 18.0 MHz
 *       The SPI prescaler is only a power of two, so at PCLK2 = 144 MHz the
 *       reachable TCKs are 36 / 18 / 9 ... MHz; 36 MHz is too fast for a GW1N
 *       JTAG chain, so 18 MHz is the ceiling -- see mpsse_set_divisor().
 *     USBHS PLL is fed from HSE/4 = 4 MHz and is independent of the core
 *       clock, so USB stays at 480 MHz at either 96 or 144 MHz.
 *******************************************************************************/
#include "debug.h"
#include "ftdi_usb.h"
#include "cdc_uart.h"
#include "ch32v30x_usbhs_device.h"

/* ------------------------------------------------------------------------- */
/* 1 ms time base: TIM3 free running at 1 MHz, polled from the main loop.     */
/* Using a polled counter avoids stealing any interrupt vector.               */
/* ------------------------------------------------------------------------- */
static void tick_init(void)
{
    TIM_TimeBaseInitTypeDef tim;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* TIM3 is on APB1.  With PPRE1 = /2 the timer kernel clock is
       PCLK1 * 2 = 72 MHz * 2 = 144 MHz, so divide by 144 to get 1 MHz.     */
    tim.TIM_Prescaler         = 144 - 1;        /* 144 MHz -> 1 MHz         */
    tim.TIM_CounterMode       = TIM_CounterMode_Up;
    tim.TIM_Period            = 0xFFFF;
    tim.TIM_ClockDivision     = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &tim);
    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);
}

static uint8_t ms_elapsed(void)
{
    static uint16_t last = 0;
    uint16_t now = (uint16_t)TIM3->CNT;
    uint16_t delta = (uint16_t)(now - last);

    if (delta >= 1000) {
        last = (uint16_t)(last + (delta / 1000) * 1000);
        delta /= 1000;
        return (delta > 4) ? 4 : (uint8_t)delta;
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
int main(void)
{
    uint8_t i;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();

    /* bring up our own peripherals first: the USB endpoint DMA pointers are
       taken from the response buffers owned by the FTDI layer */
    ftdi_init();
    tick_init();

    /* USBHS: 480 Mbit/s device, internal PHY */
    USBHS_RCC_Init();
    USBHS_Device_Init(ENABLE);

    while (1) {
        uint8_t ms;

        ftdi_jtag_poll();

        ms = ms_elapsed();
        for (i = 0; i < ms; i++) {
            ftdi_tick_1ms();
        }
    }
}

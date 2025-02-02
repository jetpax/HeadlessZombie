/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/rtc.h>
#include "hwdefs.h"
#include "hwinit.h"
#include "params.h"
#include "iomatrix.h"
#include "digio.h"

extern volatile uint32_t rtc_counter;

extern "C"  void rtc_wkup_isr(void) {
    // Clear the wakeup timer interrupt flag
    RTC_ISR &= ~RTC_ISR_WUTF;
    exti_reset_request(EXTI22);  // clear irq
#ifdef STM32F4    
    /* emulate f1 rtc_get_counter_value*/
    rtc_counter++;
#endif
}

/**
* Start clocks of all needed peripherals
*/
void clock_setup(void)
{
    RCC_CLOCK_SETUP;

#ifdef STM32F1

    rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6);

#else

    rcc_set_ppre1(RCC_CFGR_PPRE_DIV4);  // APB1 = HCLK / 4 = 42MHz for (USART2,3,4)
    rcc_set_ppre2(RCC_CFGR_PPRE_DIV2);  // APB2 = HCLK = 84MHz for (USART1,6)

    ADC_CCR = (ADC_CCR & ~ADC_CCR_ADCPRE_MASK) | ADC_CCR_ADCPRE_BY4;        // Set ADC prescaler

#endif
    //The reset value for PRIGROUP (=0) is not actually a defined
    //value. Explicitly set 16 preemtion priorities
    SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_PRIGROUP_GROUP16_NOSUB;

    rcc_periph_clock_enable(RCC_GPIOA);  // GPIOA for ADC input
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_GPIOE);
    rcc_periph_clock_enable(RCC_USART3);
    rcc_periph_clock_enable(RCC_USART2);//GS450H Inverter Comms
	rcc_periph_clock_enable(RCC_USART1);//LIN Comms
    rcc_periph_clock_enable(RCC_TIM1); //GS450H oil pump pwm
    rcc_periph_clock_enable(RCC_TIM2); //GS450H 500khz usart clock
    rcc_periph_clock_enable(RCC_TIM3); //PWM outputs
    rcc_periph_clock_enable(RCC_TIM4); //Scheduler
    rcc_periph_clock_enable(RCC_DMA1);  //ADC, and UARTS
    rcc_periph_clock_enable(RCC_DMA2);
    rcc_periph_clock_enable(RCC_ADC1);  // Enable ADC1 clock
    rcc_periph_clock_enable(RCC_CRC);
#ifdef STM32F1
    rcc_periph_clock_enable(RCC_AFIO); //CAN AND USART3
#endif
    rcc_periph_clock_enable(RCC_CAN1); //CAN1
    rcc_periph_clock_enable(RCC_CAN2); //CAN2
    rcc_periph_clock_enable(RCC_SPI2);  //CAN3
    rcc_periph_clock_enable(RCC_SPI3);  //Digital POTS
}

#ifdef H_Z
void spi1_setup()  // spi 1 used for CAN3 on sparkfun_micromod_f405
{
    // Enable SPI1 clock
    rcc_periph_clock_enable(RCC_SPI1);

    // Configure SPI1
    spi_init_master(SPI1,
                    SPI_CR1_BAUDRATE_FPCLK_DIV_16,    // assume APB2=84MHz, /16
                                                        // gives 5.25MHz SPI clock
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,  // Clock polarity: idle low
                    SPI_CR1_CPHA_CLK_TRANSITION_1,    // Clock phase: data sampled
                                                        // on first clock transition
                    SPI_CR1_DFF_8BIT,                 // Data frame format: 8-bit
                    SPI_CR1_MSBFIRST);                // Data order: MSB first

    // Since we are manually managing the SS line, we need to move it to
    // software control here.
    spi_enable_software_slave_management(SPI1);

    // We also need to set the value of NSS high, so that our SPI peripheral
    // doesn't think it is itself in slave mode.
    spi_set_nss_high(SPI1);

    /* Configure MCP2515 CAN3 CS */
    gpio_mode_setup(CAN3_CS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, CAN3_CS_PIN);
    // Configure PA5, PA6, and PA7 as Alternate Function for SPI1
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5 | GPIO6 | GPIO7);
    // Set alternate function 5 (AF5) for PA5, PA6, PA7
    gpio_set_af(GPIOA, GPIO_AF5, GPIO5 | GPIO6 | GPIO7);
    spi_enable(SPI1);
}
#else
void spi2_setup()  // spi 2 used for CAN3
{
    spi_init_master(
        SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_32, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
        SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
    spi_set_standard_mode(SPI2, 0);  // set mode 0

    spi_enable_software_slave_management(SPI2);
    // spi_enable_ss_output(SPI2);
    spi_set_nss_high(SPI2);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                    GPIO15 | GPIO13);  // MOSI , CLK
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO14);  // MISO
    spi_enable(SPI2);
}
#endif

void spi3_setup()  // spi3 used for digi pots (fuel gauge etc)
{
  spi_init_master(
      SPI3, SPI_CR1_BAUDRATE_FPCLK_DIV_32, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
      SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

  spi_enable_software_slave_management(SPI3);
  spi_set_nss_high(SPI3);

#ifdef STM32F1
    // STM32F1-specific GPIO setup
    gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_SPI3_REMAP);

    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                    GPIO12 | GPIO10);  // MOSI, SCK
    gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO11);  // MISO

#elif defined(STM32F4)
    // STM32F4-specific GPIO setup
    rcc_periph_clock_enable(RCC_GPIOC);  // Enable GPIOC clock

    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE,
                    GPIO10 | GPIO11 | GPIO12);               // SCK, MISO, MOSI
    gpio_set_af(GPIOC, GPIO_AF6, GPIO10 | GPIO11 | GPIO12);  // SPI3 uses AF6
    #endif

    // Enable SPI3
    spi_enable(SPI3);
}


#ifndef H_Z      // Sync serial not supported on Headless Zombie, which uses USART2 for Terminal
/**
* Setup USART2 500000 8N1 for Toyota inverter comms
*/
void usart2_setup(void)
{
    /* Setup GPIO pin GPIO_USART2_TX and GPIO_USART2_RX. */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);
    usart_set_baudrate(USART2, 500000);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_enable(USART2);
}
#endif
/**
* Enable Timer refresh and break interrupts
*/
void nvic_setup(void)
{
#ifndef H_Z // Not supported on Headless Zombie
    // USART2_TX
    nvic_enable_irq(NVIC_DMA1_CHANNEL7_IRQ);
    nvic_set_priority(NVIC_DMA1_CHANNEL7_IRQ, 0xf0); // High priority for USART2_TX

    // USART2_RX
    nvic_enable_irq(NVIC_DMA1_CHANNEL6_IRQ);
    nvic_set_priority(NVIC_DMA1_CHANNEL6_IRQ, 0xf0); // Low priority for USART2_RX
#endif

    // nvic_enable_irq(NVIC_DMA1_CHANNEL3_IRQ);
    //nvic_set_priority(NVIC_DMA1_CHANNEL3_IRQ, 0x20);//usart3_RX high priority int

    nvic_enable_irq(NVIC_TIM4_IRQ); //Scheduler on tim4
    nvic_set_priority(NVIC_TIM4_IRQ, 0); //Highest priority

#ifdef STM32F1
    // CAN RX (STM32F103)
    nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
    nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ,
                      0xe << 4);  // Second lowest priority

    // CAN TX (STM32F103)
    nvic_enable_irq(NVIC_USB_HP_CAN_TX_IRQ);
    nvic_set_priority(NVIC_USB_HP_CAN_TX_IRQ,
                      0xe << 4);  // Second lowest priority

#elif defined(STM32F4)
    // CAN RX (STM32F405)
    nvic_enable_irq(NVIC_CAN1_RX0_IRQ);  // Use NVIC_CAN2_RX0_IRQ for CAN2
    nvic_set_priority(NVIC_CAN1_RX0_IRQ, 0xe << 4);  // Second lowest priority

    // CAN TX (STM32F405)
    nvic_enable_irq(NVIC_CAN1_TX_IRQ);  // Use NVIC_CAN2_TX_IRQ for CAN2
    nvic_set_priority(NVIC_CAN1_TX_IRQ, 0xe << 4);  // Second lowest priority
#endif



#ifdef STM32F1

    /* Configure RTC interrupt */
    nvic_enable_irq(NVIC_RTC_IRQ);          // RTC IRQ for STM32F103
    nvic_set_priority(NVIC_RTC_IRQ, 0x20);  // Second lowest priority
    /* Configure MCP2515 CAN3 IRQ */
    gpio_set_mode(CAN3_INT_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, CAN3_INT_PIN);

#else

    /* Enable GPIOC clock */
    rcc_periph_clock_enable(RCC_GPIOC);
    /* Configure MCP2515 CAN3 IRQ */
    gpio_mode_setup(CAN3_INT_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, CAN3_INT_PIN);

#endif

    nvic_enable_irq(CAN3_EXTI_VECTOR);                  // enable CAN3 EXTI vector
    exti_enable_request(CAN3_EXTI);                     // Enable CAN3 EXTI interrupts
    exti_set_trigger(CAN3_EXTI, EXTI_TRIGGER_FALLING);  // Trigger on falling edge
    exti_select_source(CAN3_EXTI, CAN3_INT_PORT);       // Map interrupt to port
}

void rtc_setup() {
#ifdef STM32F1
    // Base clock is HSE/128 = 8MHz/128 = 62.5kHz
    // 62.5kHz / (62499 + 1) = 1Hz
    rtc_auto_awake(RCC_HSE, 62499);  // 1s tick
    rtc_set_counter_val(0);
    //* Enable the RTC interrupt to occur off the SEC flag.
    rtc_clear_flag(RTC_SEC);
    rtc_interrupt_enable(RTC_SEC);
#elif defined(STM32F4)
    // Enable power interface clock
    rcc_periph_clock_enable(RCC_PWR);

    // Allow access to the backup domain
    pwr_disable_backup_domain_write_protect();

    // Enable the 32kHz LSE oscillator
    RCC_BDCR |= RCC_BDCR_LSEON;
    while (!(RCC_BDCR & RCC_BDCR_LSERDY));  // Wait for LSE to stabilize

    // Clear previous RTCSEL bits
    RCC_BDCR &= ~(RCC_BDCR_RTCSEL_MASK << RCC_BDCR_RTCSEL_SHIFT);

    // Set RTCSEL to LSE
    RCC_BDCR |= (RCC_BDCR_RTCSEL_LSE << RCC_BDCR_RTCSEL_SHIFT);  

    // Enable RTC
    RCC_BDCR |= RCC_BDCR_RTCEN;

    // Unlock RTC configuration
    rtc_unlock();

    // Set the RTC prescaler for a 1Hz clock
    rtc_set_prescaler(127, 255);  // (127 + 1) * (255 + 1) = 32,768 (LSE frequency)

    // Disable the wakeup timer during configuration
    rtc_disable_wakeup_timer();

    rtc_set_wakeup_time(2047, RTC_CR_WUCLKSEL_RTC_DIV16);

    // Setup PA15 as output for run LED
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15);
    gpio_set(GPIOA, GPIO15);

    // Enable the wakeup timer
    rtc_enable_wakeup_timer();

    // Enable the wakeup timer interrupt
    rtc_enable_wakeup_timer_interrupt();

    // Lock RTC configuration
    rtc_lock();

#endif
}

void tim_setup()
{
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Code for timer 1
    //oil pump control pwm for Toyota hybrid gearbox Needs to be 1khz
    //Variable frequency output
    ////////////////////////////////////////////////////////////////////////
#ifdef STM32F1
      /* STM32F103: Use GPIOE9 for TIM1.CH1 */
      /* Enable GPIOE clock */
      rcc_periph_clock_enable(RCC_GPIOE);

      /* Configure GPIOE9 as Alternate Function Push-Pull for TIM1.CH1 */
      gpio_set_mode(GPIOE, GPIO_MODE_OUTPUT_2_MHZ,
                    GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9);

#elif defined(STM32F4)
      /* STM32F405: Use GPIOC6 for TIM3.CH1 */
      /* Enable GPIOC clock */
      rcc_periph_clock_enable(RCC_GPIOC);

      /* Configure GPIOC6 as Alternate Function for TIM3.CH1 */
      gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);
      gpio_set_af(GPIOC, GPIO_AF2,
                  GPIO6);  // TIM3 uses Alternate Function 2 (AF2)
#endif

      /* Timer configuration */
#ifdef STM32F1
      /* TIM1 configuration for STM32F103 */
      timer_disable_counter(TIM1);
      /* Set timer mode */
      timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1,
                     TIM_CR1_DIR_UP);
      /* Set timer prescaler */
      timer_set_prescaler(TIM1, 32);  // Adjust prescaler value as needed
      /* Set PWM mode for TIM1.CH1 */
      timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM2);
      /* Enable output for TIM1.CH1 */
      timer_enable_oc_output(TIM1, TIM_OC1);
      /* Enable the main output for TIM1 (advanced timer requirement) */
      timer_enable_break_main_output(TIM1);
      /* Set PWM duty cycle (Output Compare value) */
      timer_set_oc_value(TIM1, TIM_OC1, 400);  // Example duty cycle
      /* Set timer period (frequency) */
      timer_set_period(TIM1, 1088);  // Example frequency
      /* Enable the timer */
      timer_enable_counter(TIM1);

#elif defined(STM32F4)
      /* TIM3 configuration for STM32F405 */
      timer_disable_counter(TIM3);
      /* Set timer mode */
      timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1,
                     TIM_CR1_DIR_UP);
      /* Set timer prescaler */
      timer_set_prescaler(TIM3, 32);  // Adjust prescaler value as needed
      /* Set PWM mode for TIM3.CH1 */
      timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM2);
      /* Enable output for TIM3.CH1 */
      timer_enable_oc_output(TIM3, TIM_OC1);
      /* Set PWM duty cycle (Output Compare value) */
      timer_set_oc_value(TIM3, TIM_OC1, 400);  // Example duty cycle
      /* Set timer period (frequency) */
      timer_set_period(TIM3, 1088);  // Example frequency
      /* Enable the timer */
      timer_enable_counter(TIM3);
#endif
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////

}

#ifndef H_Z // Not supported on Headless Zombie
void tim2_setup()
{
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Code for timer 2 to create usart 2 clock for Toyota hybrid comms
    ////////////////////////////////////////////////////////////////////////
    gpio_set_mode(GPIOA,GPIO_MODE_OUTPUT_50_MHZ,	// High speed
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,GPIO1);	// GPIOA1=TIM2.CH2 clock for usart2

    // TIM2:
    timer_disable_counter(TIM2);
//	timer_reset(TIM2);

    timer_set_mode(TIM2,
                   TIM_CR1_CKD_CK_INT,
                   TIM_CR1_CMS_EDGE,
                   TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM2,0);
    timer_enable_preload(TIM2);
    timer_continuous_mode(TIM2);
    timer_set_period(TIM2,143);//500khz

    timer_disable_oc_output(TIM2,TIM_OC2);
    timer_set_oc_mode(TIM2,TIM_OC2,TIM_OCM_PWM1);
    timer_enable_oc_output(TIM2,TIM_OC2);

    timer_set_oc_value(TIM2,TIM_OC2,72);//50% duty
    timer_enable_counter(TIM2);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
}


void tim3_setup()
{
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Setup all 3 PWM ports to output a 1khz 50% duty cycle PWM signal
    // General purpose pwm output. Push/pull driven to +12v/gnd. Timer 3 Chan 3 PB0.
    // General purpose pwm output. Push/pull driven to +12v/gnd. Timer 3 Chan 2 PA7.
    // General purpose pwm output. Push/pull driven to +12v/gnd. Timer 3 Chan 1 PA6.
    ////////////////////////////////////////////////////////////////////////
    bool CPspoofPres = 0;
    bool GS450hOil = 0;
///PWM3
    if (Param::GetInt(Param::PWM3Func) == IOMatrix::PWM_TIM3)
    {
        gpio_set_mode(GPIOB,GPIO_MODE_OUTPUT_2_MHZ,	// Low speed (only need 1khz)
                      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,GPIO0);	// GPIOB0=TIM3.CH3
    }
    else if (Param::GetInt(Param::PWM3Func) == IOMatrix::CP_SPOOF)
    {
        gpio_set_mode(GPIOB,GPIO_MODE_OUTPUT_2_MHZ,	// Low speed (only need 1khz)
                      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,GPIO0);	// GPIOB0=TIM3.CH3
        CPspoofPres = 1; //Running CP Spoof
    }
    else if (Param::GetInt(Param::PWM3Func) == IOMatrix::GS450HOIL)
    {
        gpio_set_mode(GPIOB,GPIO_MODE_OUTPUT_2_MHZ,	// Low speed (only need 1khz)
                      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,GPIO0);	// GPIOB0=TIM3.CH3
        GS450hOil = 1; //Running GS450h Oil Pump
    }
    else
    {
        DigIo::PWM3.Configure(GPIOB,GPIO0,PinMode::OUTPUT);
    }

    ///PWM2
    if (Param::GetInt(Param::PWM2Func) == IOMatrix::PWM_TIM3)
    {
        gpio_set_mode(GPIOA,GPIO_MODE_OUTPUT_2_MHZ,	// Low speed (only need 1khz)
                      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,GPIO7);	// GPIOE9=TIM3.CH2
    }
    else if (Param::GetInt(Param::PWM2Func) == IOMatrix::CP_SPOOF)
    {
        gpio_set_mode(GPIOA,GPIO_MODE_OUTPUT_2_MHZ,	// Low speed (only need 1khz)
                      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,GPIO7);	// GPIOE9=TIM3.CH2
        CPspoofPres = 1; //Running CP Spoof
    }
    else if (Param::GetInt(Param::PWM2Func) == IOMatrix::GS450HOIL)
    {
        gpio_set_mode(GPIOA,GPIO_MODE_OUTPUT_2_MHZ,	// Low speed (only need 1khz)
                      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,GPIO7);	// GPIOE9=TIM3.CH2
        GS450hOil = 1; //Running GS450h Oil Pump
    }
    else
    {
        DigIo::PWM2.Configure(GPIOA,GPIO7,PinMode::OUTPUT);
    }

    ///PWM1
    if (Param::GetInt(Param::PWM1Func) == IOMatrix::PWM_TIM3)
    {
        gpio_set_mode(GPIOA,GPIO_MODE_OUTPUT_2_MHZ,	// Low speed (only need 1khz)
                      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,GPIO6);	// GPIOA6=TIM3.CH1
    }
    else if (Param::GetInt(Param::PWM1Func) == IOMatrix::CP_SPOOF)
    {
        gpio_set_mode(GPIOA,GPIO_MODE_OUTPUT_2_MHZ,	// Low speed (only need 1khz)
                      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,GPIO6);	// GPIOA6=TIM3.CH1
        CPspoofPres = 1; //Running CP Spoof
    }
    else if (Param::GetInt(Param::PWM1Func) == IOMatrix::GS450HOIL)
    {
        gpio_set_mode(GPIOA,GPIO_MODE_OUTPUT_2_MHZ,	// Low speed (only need 1khz)
                      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,GPIO6);	// GPIOA6=TIM3.CH1
        GS450hOil = 1; //Running GS450h Oil Pump
    }
    else
    {
        DigIo::PWM1.Configure(GPIOA,GPIO6,PinMode::OUTPUT);
    }

    timer_disable_counter(TIM3);
    //edge aligned PWM
    timer_set_alignment(TIM3, TIM_CR1_CMS_EDGE);
    timer_enable_preload(TIM3);
    /* PWM mode 1 and preload enable */
    timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM1);
    timer_set_oc_mode(TIM3, TIM_OC3, TIM_OCM_PWM1);
    timer_enable_oc_preload(TIM3, TIM_OC1);
    timer_enable_oc_preload(TIM3, TIM_OC2);
    timer_enable_oc_preload(TIM3, TIM_OC3);

    timer_set_oc_polarity_high(TIM3, TIM_OC1);
    timer_set_oc_polarity_high(TIM3, TIM_OC2);
    timer_set_oc_polarity_high(TIM3, TIM_OC3);
    timer_enable_oc_output(TIM3, TIM_OC1);
    timer_enable_oc_output(TIM3, TIM_OC2);
    timer_enable_oc_output(TIM3, TIM_OC3);

    if (CPspoofPres == 0 && GS450hOil == 0) //No CP Spoof Selected or GS450h Oil pump
    {
        timer_set_period(TIM3, Param::GetInt(Param::Tim3_Period));
        timer_set_oc_value(TIM3, TIM_OC1, Param::GetInt(Param::Tim3_1_OC));
        timer_set_oc_value(TIM3, TIM_OC2, Param::GetInt(Param::Tim3_2_OC));
        timer_set_oc_value(TIM3, TIM_OC3, Param::GetInt(Param::Tim3_3_OC));
        timer_generate_event(TIM3, TIM_EGR_UG);
        timer_set_prescaler(TIM3,Param::GetInt(Param::Tim3_Presc));
        timer_enable_counter(TIM3);
    }
    else//No CP Spoof or GS450h Oil pump output selected locks TIM3 Force 1khz clock (996khz)
    {
        timer_set_period(TIM3, 6540);
        timer_set_oc_value(TIM3, TIM_OC1, 1);//No duty set here
        timer_set_oc_value(TIM3, TIM_OC2, 1);//No duty set here
        timer_set_oc_value(TIM3, TIM_OC3, 1);//No duty set here
        timer_generate_event(TIM3, TIM_EGR_UG);
        timer_set_prescaler(TIM3,10);
        timer_enable_counter(TIM3);
    }
}
#endif

#ifdef STM32F4
uint32_t rtc_get_counter_val(void) {
  /* emulate f1 rtc_get_counter_value*/
  return rtc_counter;
}

#endif


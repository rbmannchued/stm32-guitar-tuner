#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/cm3/cortex.h>

#include "arm_math.h"
#include <stdio.h>
#include <stdint.h>

#define OVER_FRAME_LEN 4096
#define FFT_SIZE 1024

static volatile float32_t samples[OVER_FRAME_LEN/2];
static volatile int full_samples_frame = 0;
static volatile uint16_t adc_value = 0;



void uart_init(void) {
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART2);

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2 | GPIO3);

    usart_set_baudrate(USART2, 115200);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

    usart_enable(USART2);
}

static void timer2_set_sampling_rate(void) {
    const int clock_div = 750;
    timer_set_prescaler(TIM2, clock_div-1);
    timer_set_period(TIM2, 1);

    timer_generate_event(TIM2, TIM_EGR_UG);
    timer_clear_flag(TIM2, TIM_EGR_UG);
}

static void timer_init(void) {
    rcc_periph_clock_enable(RCC_TIM2);

    timer_set_alignment(TIM2, TIM_CR1_CMS_EDGE);
    timer_direction_up(TIM2);
    timer_update_on_overflow(TIM2);
    timer_enable_update_event(TIM2);
    timer_set_master_mode(TIM2, TIM_CR2_MMS_UPDATE);
    timer2_set_sampling_rate();
}

void adc_isr(void) {
    static int i = 0;

    // Verifica se a interrupção foi causada pelo fim da conversão (EOC)
    if (ADC_SR(ADC1) & ADC_SR_EOC) {
        adc_value = adc_read_regular(ADC1); // Lê o valor do ADC
        samples[i] = (float32_t) adc_value;
        i++;

        if (i % OVER_FRAME_LEN == 0) {
            full_samples_frame = 1;
            if (i == OVER_FRAME_LEN * 2)
                i = 0;
        }

        // Limpa a flag EOC
        ADC_SR(ADC1) &= ~ADC_SR_EOC;
    }
}

static void adc_init(void) {
    const uint8_t adc_channel = 1;

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_ADC1);

    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);

    adc_set_clk_prescale(ADC_CCR_ADCPRE_BY8);
    adc_set_resolution(ADC1, ADC_CR1_RES_12BIT);
    adc_set_single_conversion_mode(ADC1);
    adc_set_regular_sequence(ADC1, 1, (uint8_t *)&adc_channel);
    adc_enable_eoc_interrupt(ADC1); // Habilita a interrupção de fim de conversão
    nvic_enable_irq(NVIC_ADC_IRQ);  // Habilita a interrupção do ADC no NVIC
    adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_TIM2_TRGO, ADC_CR2_EXTEN_RISING_EDGE);

    adc_power_on(ADC1);
    adc_start_conversion_regular(ADC1); // Inicia a primeira conversão
}

void usart_send_string(const char *str) {
    while (*str) {
        usart_send_blocking(USART2, *str++);
    }
}



int main(void) {
    rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
    uart_init();
    cm_enable_interrupts();
    timer_init();
    adc_init();
    timer_enable_counter(TIM2);




    usart_send_string("** inicio codigo ** \r\n");

    while (1) {
        if (full_samples_frame) {
            char msgBuff[50];
            snprintf(msgBuff, sizeof(msgBuff), "ADC Value: %d\r\n", adc_value);
            usart_send_string(msgBuff);

            full_samples_frame = 0;
        }
    }
}

// Vetor de interrupção do ADC
void adc_isr(void) __attribute__((interrupt));
void ADC_IRQHandler(void) {
    adc_isr();
}

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/systick.h>

#include "arm_math.h"

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include <libopencm3/stm32/usart.h>

#define FRAME_LEN 4096
#define SAMPLE_RATE 8000  // Defina a taxa de amostragem do seu ADC
#define NUM_TAPS 64

volatile uint32_t sample_count = 0;
volatile uint32_t last_time = 0;
volatile uint32_t ms_counter = 0;

static volatile uint16_t adc_buffer[FRAME_LEN];
static volatile int buffer_index = 0;
static volatile int frame_ready = 0;

float input_fft[FRAME_LEN * 2];  // Entrada complexa (real + imaginária)
float output_fft[FRAME_LEN];      // Magnitude da FFT
arm_rfft_fast_instance_f32 fft_instance;



const float fir_coeffs[64] = {-0.001192, -0.001687, -0.001821, -0.001461, -0.000716, -0.000047, -0.000156, -0.001574, -0.004129, -0.006719, -0.007750, -0.006167, -0.002503, 0.000897, 0.000955, -0.004113, -0.013037, -0.021367, -0.023597, -0.016692, -0.002979, 0.009778, 0.011936, -0.002176, -0.029273, -0.056255, -0.064940, -0.040747, 0.018804, 0.100441, 0.178891, 0.226882, 0.226882, 0.178891, 0.100441, 0.018804, -0.040747, -0.064940, -0.056255, -0.029273, -0.002176, 0.011936, 0.009778, -0.002979, -0.016692, -0.023597, -0.021367, -0.013037, -0.004113, 0.000955, 0.000897, -0.002503, -0.006167, -0.007750, -0.006719, -0.004129, -0.001574, -0.000156, -0.000047, -0.000716, -0.001461, -0.001821, -0.001687, -0.001192};

float fir_state[NUM_TAPS + FRAME_LEN];
arm_fir_instance_f32 fir;


void adc_isr(void) {
    if (adc_eoc(ADC1)) {
	
        adc_buffer[buffer_index++] = adc_read_regular(ADC1);
	sample_count++;
        if (buffer_index >= FRAME_LEN) {
	    
            buffer_index = 0;
            frame_ready = 1;
        }
    }
}

void sys_tick_handler(void) {
    ms_counter++;
}

void check_sample_rate(void) {
    static uint32_t last_time = 0;
    if ((ms_counter - last_time) >= 1000) {  // 1000ms = 1 segundo
        char msg[50];
        snprintf(msg, sizeof(msg), "Amostras por segundo: %lu\r\n", sample_count);
        usart_send_string(msg);
        sample_count = 0;  // Reseta o contador
        last_time = ms_counter;
    }
}


void systick_setup(void) {
    systick_set_reload(84000);  // 1ms (84MHz / 84.000)
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_counter_enable();
    systick_interrupt_enable();
}

void process_fft(void) {
    // Calcular a média do sinal (offset DC)
    char msg[50];
    float mean = 0.0f;
    for (int i = 0; i < FRAME_LEN; i++) {
        mean += (float)adc_buffer[i] / 4096.0f;
    }
    mean /= FRAME_LEN;

    // Remover o offset DC e normalizar o sinal
    for (int i = 0; i < FRAME_LEN; i++) {
        input_fft[i * 2] = ((float)adc_buffer[i] / 4096.0f) - mean;  // Remove DC
        input_fft[i * 2 + 1] = 0.0f; // Parte imaginária = 0
    }

    // Aplicar janela (Hamming)
    for (int i = 0; i < FRAME_LEN; i++) {
        float window = 0.54f - 0.46f * cosf(2 * PI * i / (FRAME_LEN - 1)); // Hamming window
        input_fft[i * 2] *= window;
    }
    //arm_fir_f32(&fir, input_fft, input_fft, FRAME_LEN);
    
    // Executar FFT

    arm_rfft_fast_f32(&fft_instance, input_fft, output_fft, 0);

    // Calcula magnitude da FFT (ignorando índice 0 e primeiros índices)
    float max_value = 0.0f;
    int max_index = 0;
    int start_index = 5;  // Ignorar os primeiros 5 índices
    for (int i = start_index; i < FRAME_LEN / 2; i++) {
        float magnitude = sqrtf(output_fft[i] * output_fft[i]);
        if (magnitude > max_value) {
            max_value = magnitude;
            max_index = i;
        }
    }

    // Ignorar picos pequenos
    if (max_value < 0.1f) {
        max_index = 0;  // Frequência zero
    }

    // Converter índice para frequência em Hz
    float frequency = (float)max_index * SAMPLE_RATE / FRAME_LEN;

    // Enviar para UART
    snprintf(msg, sizeof(msg), "Index: %d, Freq: %.2f Hz\r\n", max_index, frequency);
    
    //snprintf(msg, sizeof(msg), "Freq: %.2f Hz\r\n", frequency);
    usart_send_string(msg);
}

static void timer_init(void) {
    rcc_periph_clock_enable(RCC_TIM2);
    timer_set_prescaler(TIM2, 104);
    timer_set_period(TIM2, 99);
    timer_generate_event(TIM2, TIM_EGR_UG);
    timer_clear_flag(TIM2, TIM_EGR_UG);
    timer_set_master_mode(TIM2, TIM_CR2_MMS_UPDATE);
    timer_enable_counter(TIM2);
}

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

void usart_send_string(const char *str) {
    while (*str) {
        usart_send_blocking(USART2, *str++);
    }
}

static void adc_init(void) {
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_ADC1);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
    
    adc_power_off(ADC1);
    adc_set_clk_prescale(ADC_CCR_ADCPRE_BY8);
    adc_set_resolution(ADC1, ADC_CR1_RES_12BIT);
    adc_set_single_conversion_mode(ADC1);
    uint8_t channel = 1;
    adc_set_regular_sequence(ADC1, 1, &channel);
    adc_enable_eoc_interrupt(ADC1);
    nvic_enable_irq(NVIC_ADC_IRQ);
    adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_TIM2_TRGO, ADC_CR2_EXTEN_RISING_EDGE);
    adc_power_on(ADC1);
}

int main(void) {
    rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
    systick_setup();
    cm_enable_interrupts();
    
    adc_init();
    uart_init();
    timer_init();

    arm_rfft_fast_init_f32(&fft_instance, FRAME_LEN);
    arm_fir_init_f32(&fir, NUM_TAPS, fir_coeffs, fir_state, FRAME_LEN);
    usart_send_string("inicio do codigo\r\n");
    char msg[50];
    

    while (1) {
        check_sample_rate();  // Checar a taxa de amostragem
        if (frame_ready) {
            process_fft();
            frame_ready = 0;
        }
        __asm__("wfi");
    }
}

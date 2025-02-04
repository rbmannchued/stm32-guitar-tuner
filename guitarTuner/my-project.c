#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include "arm_math.h"
#include "arm_const_structs.h"

#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#define SAMPLE_RATE 8000
#define BUFFER_SIZE 1024


uint16_t adc_buffer[BUFFER_SIZE];
float fft_input[BUFFER_SIZE * 2]; // Interleaved real and imaginary parts
float fft_output[BUFFER_SIZE];
float magnitude[BUFFER_SIZE / 2]; 
float dominantFreq = 0;

arm_rfft_fast_instance_f32 fft_instance;

void adc_setup(void) {

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_ADC1);


    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);


    adc_power_off(ADC1);
    adc_set_resolution(ADC1, ADC_CR1_RES_12BIT);
    adc_disable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_3CYC);
    adc_power_on(ADC1);


    for (int i = 0; i < 800000; i++) {
        __asm__("nop");
    }
}


void usart_setup(void) {

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


void compute_fft(float *input, float *output, int buffer_size) {
    


    arm_rfft_fast_init_f32(&fft_instance, buffer_size);


    arm_rfft_fast_f32(&fft_instance, input, output, 0);
    

    arm_cmplx_mag_f32(output, magnitude, buffer_size/2);

    /* float maxValue; */
    /* uint32_t maxIndex; */
    /* arm_max_f32(magnitude, BUFFER_SIZE / 2, &maxValue, &maxIndex); */

    /* // Converte índice para frequência */

}


float find_fundamental_frequency(float *fft_output, int buffer_size) {
    float max_value = 0.0f;
    int max_index = 0;
    for (int i = 1; i < buffer_size / 2; i++) { 
        if (fft_output[i] > max_value) {
            max_value = fft_output[i];
            max_index = i;
        }
    }
    return (float)max_index * SAMPLE_RATE / buffer_size;
}


void capture_adc_data(uint16_t *buffer, int size) {
    for (int i = 0; i < size; i++) {
        adc_start_conversion_regular(ADC1);
        while (!adc_eoc(ADC1)); 
        buffer[i] = adc_read_regular(ADC1);
    }
}

int main(void) {

    rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);


    adc_setup();
    usart_setup();


    char output_buffer[50];


    while (1) {

        capture_adc_data(adc_buffer, BUFFER_SIZE);

        for (int i = 0; i < 10; i++) {
            snprintf(output_buffer, sizeof(output_buffer), "ADC[%d]: %d\r\n", i, adc_buffer[i]);
            usart_send_string(output_buffer);
        }


        for (int i = 0; i < BUFFER_SIZE; i++) {
	    float sample = ((float)adc_buffer[i] / 4096.0f) * 3.3f - 1.65f;
            fft_input[2 * i] = sample;
            fft_input[2 * i + 1] = 0.0f; 
        }

	

        compute_fft(fft_input, fft_output, BUFFER_SIZE);
	
	for (int i = 0; i < 10; i++) {
	    snprintf(output_buffer, sizeof(output_buffer), "MAG[%d]: %.2f\r\n", i, magnitude[i]);
	    usart_send_string(output_buffer);
	} 
        float fundamental_frequency = find_fundamental_frequency(magnitude, BUFFER_SIZE);


        snprintf(output_buffer, sizeof(output_buffer), "Freq: %.2f Hz\r\n", dominantFreq);
        usart_send_string(output_buffer);
	
    }

    return 0;
}

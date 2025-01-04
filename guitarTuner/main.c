#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include "libs/kiss_fft.h"
#include "libs/ssd1306_libopencm3/ssd1306.h"
#include "libs/ssd1306_libopencm3/ssd1306_fonts.h"
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define SAMPLE_RATE 8000
#define FFT_SIZE 256
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


void generate_signal(float *buffer, int size, float frequency, float amplitude) {
    for (int i = 0; i < size; i++) {
        buffer[i] = amplitude * sinf(2 * M_PI * frequency * i / SAMPLE_RATE);
    }
}

static void usart_setup(void) {
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_GPIOA);

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    usart_enable(USART1);
}

void usart_send_string(const char *str) {
    while (*str) {
        usart_send_blocking(USART1, *str++);
    }
}

int main(void) {

    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    usart_setup();
  float signal[FFT_SIZE];
    generate_signal(signal, FFT_SIZE, 440.0f, 1.0f);

    // Depurar valores gerados
    for (int i = 0; i < FFT_SIZE; i++) {
        char debug_buffer[50];
        snprintf(debug_buffer, sizeof(debug_buffer), "signal[%d]: %.2f\r\n", i, signal[i]);
        usart_send_string(debug_buffer);
          
  }

    kiss_fft_cfg cfg = kiss_fft_alloc(FFT_SIZE, 0, NULL, NULL);
    if (cfg == NULL) {
        usart_send_string("Error: kiss_fft_alloc failed\r\n");
        while (1);
    }

    kiss_fft_cpx in[FFT_SIZE], out[FFT_SIZE];
    for (int i = 0; i < FFT_SIZE; i++) {
        in[i].r = signal[i];
        in[i].i = 0.0f;
    }

    kiss_fft(cfg, in, out);

    float max_magnitude = 0;
    int dominant_index = 0;
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        float magnitude = sqrtf(out[i].r * out[i].r + out[i].i * out[i].i);
        char debug_buffer[50];
        snprintf(debug_buffer, sizeof(debug_buffer), "Index %d: Mag=%.2f\r\n", i, magnitude);
        usart_send_string(debug_buffer);

        if (magnitude > max_magnitude) {
            max_magnitude = magnitude;
            dominant_index = i;
        }
    }

    float fundamental_frequency = (float)dominant_index * SAMPLE_RATE / FFT_SIZE;

    char buffer[100];
    snprintf(buffer, sizeof(buffer), "Dominant Index: %d\r\n", dominant_index);
    usart_send_string(buffer);

    snprintf(buffer, sizeof(buffer), "Frequência Fundamental: %.2f Hz\r\n", fundamental_frequency);
    usart_send_string(buffer);
  if(fundamental_frequency >= 430){
    usart_send_string("maior que 430\r\n");
  }
    while (1) {
        __asm__("nop");
    }

    return 0;
}


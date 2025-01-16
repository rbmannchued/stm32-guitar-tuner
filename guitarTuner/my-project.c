#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <math.h>
#include <stdio.h>
#include "arm_math.h"  // CMSIS-DSP

// Tamanho da FFT (deve ser uma potência de 2)
#define FFT_SIZE 1024

// Dados de entrada e saída da FFT
float32_t input[FFT_SIZE];
float32_t output[FFT_SIZE];

// Função de inicialização do STM32
void setup(void) {
    // Ativar o clock para o GPIOA
    rcc_periph_clock_enable(RCC_GPIOA);

    // Configurar o LED no GPIOA (pino 5)
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);
}
void usart_setup(void) {
    // Ativar o clock para o GPIOA e USART2
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART2);

    // Configurar os pinos GPIOA2 e GPIOA3 para USART2
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2 | GPIO3);

    // Configurar USART2: Baudrate, 8N1
    usart_set_baudrate(USART2, 115200);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

    // Habilitar USART2
    usart_enable(USART2);
}
void usart_send_float(float value) {
    char buffer[32];
    int len = snprintf(buffer, sizeof(buffer), "%.4f\n", value);
    for (int i = 0; i < len; i++) {
        usart_send_blocking(USART2, buffer[i]);
    }
}

// Função principal
int main(void) {
    // Inicialização do STM32
    setup();
    usart_setup();

    // Gerar um sinal simples de seno para ser analisado pela FFT
    for (int i = 0; i < FFT_SIZE; i++) {
        input[i] = sinf(2.0f * PI * i / FFT_SIZE);  // Sinal seno
    }

    // Inicializar a estrutura da FFT CMSIS-DSP
    arm_cfft_radix4_instance_f32 S;
    arm_cfft_radix4_init_f32(&S, FFT_SIZE, 0, 1);  // Inicialização da FFT (radix 4)

    // Aplicar a FFT no sinal
    arm_cfft_radix4_f32(&S, input);

    // Calcular a magnitude dos resultados
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        // Os resultados da FFT são complexos. Calculamos a magnitude.
        output[i] = sqrtf(input[2 * i] * input[2 * i] + input[2 * i + 1] * input[2 * i + 1]);
    }

    for (int i = 0; i < FFT_SIZE / 2; i++) {
        usart_send_float(output[i]);
    }
    

    // Loop para depuração (LED piscando)
    while (1) {
        // Ligar e desligar o LED para indicar que o código está funcionando
        gpio_toggle(GPIOA, GPIO5);
        for (int i = 0; i < 1000000; i++) {
            __asm__("NOP");
        }
    }

    return 0;
}

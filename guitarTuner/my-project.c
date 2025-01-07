
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <arm_math.h>  // Cabeçalho principal da CMSIS-DSP

#define LED_PORT GPIOC
#define LED_PIN GPIO13 

void setup_led(void);
void delay(uint32_t ms);
void process_signal(void);

int main(void) {
    // Configuração do LED
    setup_led();

    // Exemplo simples de uso da CMSIS-DSP
    process_signal();

    while (1) {
        gpio_toggle(LED_PORT, LED_PIN); // Alterna o estado do LED
        delay(500);                    // Aguarda 500ms
    }

    return 0;
}

// Configuração do LED no GPIOA5 (STM32F411 - LED onboard)
void setup_led(void) {
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
}

// Função para delay (simples, com timer básico)
void delay(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 8000; i++) {
        __asm__("nop");
    }
}

// Processa um sinal usando a CMSIS-DSP
void process_signal(void) {
    // Exemplo: Calcular o valor RMS de um vetor de amostras
    float32_t signal[8] = {0.0, 1.0, -1.0, 2.0, -2.0, 3.0, -3.0, 0.0};
    float32_t rms = 0.0;

    // Usa a função da CMSIS-DSP para calcular o RMS
    arm_rms_f32(signal, 8, &rms);
    for(int i =0; i < 3; i++){
    }
    // O valor RMS é calculado e pode ser usado conforme necessário
    // Aqui você pode, por exemplo, enviá-lo por UART, usar em um filtro, etc.
}

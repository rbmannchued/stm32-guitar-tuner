#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <arm_math.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#define SAMPLE_RATE 44100
#define BUFFER_SIZE 1024

// Buffers para entrada e FFT
uint16_t adc_buffer[BUFFER_SIZE];
float fft_input[BUFFER_SIZE * 2]; // Interleaved real and imaginary parts
float fft_output[BUFFER_SIZE];

// Configuração do ADC
void adc_setup(void) {
    // Habilitar clocks necessários
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_DMA2);

    // Configurar GPIO PA0 como entrada analógica
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);

    // Configurar ADC
    adc_power_off(ADC1);
    adc_disable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28CYC);
    adc_power_on(ADC1);
    for (int i = 0; i < 800000; i++) {
        __asm__("nop");
    }

    // Configurar DMA para transferir os dados do ADC
    dma_stream_reset(DMA2, DMA_STREAM0);
    dma_set_peripheral_address(DMA2, DMA_STREAM0, (uint32_t)&ADC_DR(ADC1));
    dma_set_memory_address(DMA2, DMA_STREAM0, (uint32_t)adc_buffer);
    dma_set_number_of_data(DMA2, DMA_STREAM0, BUFFER_SIZE);
    dma_channel_select(DMA2, DMA_STREAM0, DMA_SxCR_CHSEL_0);
    dma_enable_memory_increment_mode(DMA2, DMA_STREAM0);
    dma_set_transfer_mode(DMA2, DMA_STREAM0, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
    dma_set_priority(DMA2, DMA_STREAM0, DMA_SxCR_PL_HIGH);
    dma_enable_circular_mode(DMA2, DMA_STREAM0);
    

    // Habilitar o ADC para usar DMA
    adc_enable_dma(ADC1);
    dma_enable_stream(DMA2, DMA_STREAM0);
    adc_start_conversion_regular(ADC1);
}

// Configuração da USART
void usart_setup(void) {
    // Habilitar clocks necessários
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART2);

    // Configurar GPIO PA2 (TX) e PA3 (RX) para USART
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2 | GPIO3);

    // Configurar USART
    usart_set_baudrate(USART2, 115200);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

    // Habilitar USART
    usart_enable(USART2);
}

// Enviar uma string pela USART
void usart_send_string(const char *str) {
    while (*str) {
        usart_send_blocking(USART2, *str++);
    }
}

// FFT computation
void compute_fft(float *input, float *output, int buffer_size) {
    arm_cfft_radix4_instance_f32 fft_instance;
    arm_cfft_radix4_init_f32(&fft_instance, buffer_size, 0, 1);
    arm_cfft_radix4_f32(&fft_instance, input);
    arm_cmplx_mag_f32(input, output, buffer_size);
}

float lowpass_filter(float input, float cutoff_freq, float sample_rate) {
    static float prev_output = 0.0f;
    float alpha = cutoff_freq / (cutoff_freq + sample_rate / (2.0f * PI));
    prev_output = alpha * input + (1 - alpha) * prev_output;
    return prev_output;
}

// Find fundamental frequency
float find_fundamental_frequency(float *fft_output, int buffer_size) {
    float max_value = 0.0f;
    int max_index = 0;
    for (int i = 1; i < buffer_size / 2; i++) { // Ignorar componente DC
        if (fft_output[i] > max_value) {
            max_value = fft_output[i];
            max_index = i;
        }
    }
    return (float)max_index * SAMPLE_RATE / buffer_size;
}

int main(void) {
    // Configuração do clock
    rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);

    // Configurar ADC e USART
    adc_setup();
    usart_setup();

    // Buffer para formatar a saída
    char output_buffer[50];

    // Captura e análise do sinal
    while (1) {
        // Copiar amostras do ADC para o buffer de entrada da FFT
        /* for (int i = 0; i < BUFFER_SIZE; i++) { */
        /*     fft_input[2 * i] = (float)adc_buffer[i] / 4096.0f - 0.5f; // Normalizar para [-0.5, 0.5] */
        /*     fft_input[2 * i + 1] = 0.0f;  // Parte imaginária */
        /* } */

	/* for (int i = 0; i < BUFFER_SIZE; i++) { */
	/*     fft_input[2 * i] = 0.5f * arm_sin_f32(2 * PI * 440.0f * i / SAMPLE_RATE); */
	/*     fft_input[2 * i + 1] = 0.0f; // Parte */
	/* } */
	for (int i = 0; i < 10; i++) {
	    char output_buffer[50];
	    snprintf(output_buffer, sizeof(output_buffer), "ADC[%d]: %d\r\n", i, adc_buffer[i]);
	    usart_send_string(output_buffer);
	}
	for (int i = 0; i < BUFFER_SIZE; i++) {
	    float sample = (float)adc_buffer[i] / 4096.0f - 0.5f; // Normalizar
	    fft_input[2 * i] = lowpass_filter(sample, 6000.0f, SAMPLE_RATE);
	    fft_input[2 * i + 1] = 0.0f; // Parte imaginária
	}
        // Calcular FFT
        compute_fft(fft_input, fft_output, BUFFER_SIZE);

        // Encontrar frequência fundamental
        float fundamental_frequency = find_fundamental_frequency(fft_output, BUFFER_SIZE);

        // Formatar e enviar a frequência pela USART
        snprintf(output_buffer, sizeof(output_buffer), "Freq: %.2f Hz\r\n", fundamental_frequency);
        usart_send_string(output_buffer);
    }

    return 0;
}

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <arm_math.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#define SAMPLE_RATE 44100
#define BUFFER_SIZE 1024

// Buffers for Karplus-Strong and FFT
float ks_buffer[BUFFER_SIZE];
float fft_input[BUFFER_SIZE * 2]; // Interleaved real and imaginary parts
float fft_output[BUFFER_SIZE];

// Karplus-Strong initialization
void karplus_strong_init(float *buffer, int buffer_size, float frequency) {
    int period = (int)(SAMPLE_RATE / frequency);
    for (int i = 0; i < buffer_size; i++) {
        if (i < period) {
            buffer[i] = (float)(rand() / (float)RAND_MAX) - 0.5f; // Inicializa com ruído
        } else {
            buffer[i] = 0.0f;
        }
    }
}

// Karplus-Strong update
float karplus_strong_step(float *buffer, int buffer_size, int *idx) {
    float output = buffer[*idx];
    int next_idx = (*idx + 1) % buffer_size;
    buffer[*idx] = 0.5f * (buffer[*idx] + buffer[next_idx]);
    *idx = next_idx;
    return output;
}

// FFT computation
void compute_fft(float *input, float *output, int buffer_size) {
    arm_cfft_radix4_instance_f32 fft_instance;
    arm_cfft_radix4_init_f32(&fft_instance, buffer_size, 0, 1);
    arm_cfft_radix4_f32(&fft_instance, input);
    arm_cmplx_mag_f32(input, output, buffer_size);
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

    // Simulação da corda
    float frequency = 440.0f; // Frequência inicial (A4)
    int ks_index = 0;
    karplus_strong_init(ks_buffer, BUFFER_SIZE, frequency);

    // Simulação e análise
    for (int i = 0; i < BUFFER_SIZE; i++) {
        float sample = karplus_strong_step(ks_buffer, BUFFER_SIZE, &ks_index);
        fft_input[2 * i] = sample;     // Parte real
        fft_input[2 * i + 1] = 0.0f;  // Parte imaginária
    }
    
    // Calcular FFT
    compute_fft(fft_input, fft_output, BUFFER_SIZE);

    // Encontrar frequência fundamental
    volatile float fundamental_frequency = find_fundamental_frequency(fft_output, BUFFER_SIZE);
 
    // Exibir resultado (depuração via UART ou debugger)
    while (1) {
        // A frequência fundamental pode ser visualizada no debugger
        __asm__("nop");
    }

    return 0;
}

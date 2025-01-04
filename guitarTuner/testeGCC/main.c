#include "../libs/kiss_fft.h"
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
    

int main(void) {

    float signal[FFT_SIZE];
    generate_signal(signal, FFT_SIZE, 440.0f, 1.0f);

    // Depurar valores gerados
    for (int i = 0; i < FFT_SIZE; i++) {
        char debug_buffer[50];
        snprintf(debug_buffer, sizeof(debug_buffer), "signal[%d]: %.2f\r\n", i, signal[i]);
        printf(debug_buffer);
    }

    kiss_fft_cfg cfg = kiss_fft_alloc(FFT_SIZE, 0, NULL, NULL);
    if (cfg == NULL) {
        printf("Error: kiss_fft_alloc failed\r\n");
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
        printf(debug_buffer);

        if (magnitude > max_magnitude) {
            max_magnitude = magnitude;
            dominant_index = i;
        }
    }

    float fundamental_frequency = (float)dominant_index * SAMPLE_RATE / FFT_SIZE;

    char buffer[100];
    snprintf(buffer, sizeof(buffer), "Dominant Index: %d\r\n", dominant_index);
    printf(buffer);

    snprintf(buffer, sizeof(buffer), "Frequência Fundamental: %.2f Hz\r\n", fundamental_frequency);
    printf(buffer);

    while (1) {
        __asm__("nop");
    }

    return 0;
}


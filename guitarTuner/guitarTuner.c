#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/cm3/cortex.h>


#include "arm_math.h"

#include "ssd1306.h"
#include "ssd1306_fonts.h"

#include <stdio.h>
#include <stdint.h>
#include <math.h>



#define FRAME_LEN 4096
#define SAMPLE_RATE 4000  // Defina a taxa de amostragem do seu ADC
#define NUM_TAPS 64

volatile uint32_t sample_count = 0;
volatile uint32_t last_time = 0;
volatile uint32_t ms_counter = 0;

static volatile uint16_t adc_buffer[FRAME_LEN];
static volatile int buffer_index = 0;
static volatile int frame_ready = 0;
float magnitude_fft[FRAME_LEN / 2];
float input_fft[FRAME_LEN * 2];  // Entrada complexa (real + imaginária)
float output_fft[FRAME_LEN];      // Magnitude da FFT
arm_rfft_fast_instance_f32 fft_instance;
arm_fir_instance_f32 fir_instance;
float32_t fir_state[FRAME_LEN + NUM_TAPS - 1];


const float32_t fir_coeffs[64] = {
    -0.00079823,
    -0.00041938,
    0.00030160,
    0.00076554,
    0.00032056,
    -0.00119156,
    -0.00299337,
    -0.00367625,
    -0.00240416,
    -0.00013388,
    0.00054591,
    -0.00255714,
    -0.00874998,
    -0.01400784,
    -0.01384406,
    -0.00772045,
    -0.00099430,
    -0.00158625,
    -0.01298392,
    -0.02934585,
    -0.03814044,
    -0.03005271,
    -0.00889455,
    0.00776666,
    0.00053342,
    -0.03447555,
    -0.07647799,
    -0.08885410,
    -0.04294674,
    0.05869062,
    0.17717949,
    0.25658406,
    0.25658406,
    0.17717949,
    0.05869062,
    -0.04294674,
    -0.08885410,
    -0.07647799,
    -0.03447555,
    0.00053342,
    0.00776666,
    -0.00889455,
    -0.03005271,
    -0.03814044,
    -0.02934585,
    -0.01298392,
    -0.00158625,
    -0.00099430,
    -0.00772045,
    -0.01384406,
    -0.01400784,
    -0.00874998,
    -0.00255714,
    0.00054591,
    -0.00013388,
    -0.00240416,
    -0.00367625,
    -0.00299337,
    -0.00119156,
    0.00032056,
    0.00076554,
    0.00030160,
    -0.00041938,
    -0.00079823
};

const char *noteNames[] = {"A","A#","B","C","C#","D","D#","E","F","F#","G","G#"};
const double noteFrequencies[60] = {
  55.00,  // A1
  58.27,  // A#1
  61.74,  // B1
  65.41,  // C2
  69.30,  // C#2
  73.42,  // D2
  77.78,  // D#2
  82.41,  // E2
  87.31,  // F2
  92.50,  // F#2
  98.00,  // G2
  103.83, // G#2
  110.00, // A2
  116.54, // A#2
  123.47, // B2
  130.81, // C3
  138.59, // C#3
  146.83, // D3
  155.56, // D#3
  164.81, // E3
  174.61, // F3
  185.00, // F#3
  196.00, // G3
  207.65, // G#3
  220.00, // A3
  233.08, // A#3
  246.94, // B3
  261.63, // C4
  277.18, // C#4
  293.66, // D4
  311.13, // D#4
  329.63, // E4
  349.23, // F4
  369.99, // F#4
  392.00, // G4
  415.30, // G#4
  440.00, // A4
  466.16, // A#4
  493.88, // B4
  523.25, // C5
  554.37, // C#5
  587.33, // D5
  622.25, // D#5
  659.26, // E5
  698.46, // F5
  739.99, // F#5
  783.99, // G5
  830.61, // G#5
  880.00, // A5
  932.33, // A#5
  987.77, // B5
  1046.50, // C6
  1108.73, // C#6
  1174.66, // D6
  1244.51, // D#6
  1318.51, // E6
  1396.91, // F6
  1479.98, // F#6
  1567.98, // G6
  1661.22, // G#6
  1760.00, // A6
};
 
const int getClosestNoteIndex(double frequency) {
    double minDiff = 1e9;
    int closestIndex = 0;
  
    for (int i = 1; i < sizeof(noteFrequencies) / sizeof(noteFrequencies[0]); i++) {
	double diff = fabs(frequency - noteFrequencies[i]);
	if (diff < minDiff) {
	    minDiff = diff;
	    closestIndex = i;
	}
    }
    return closestIndex;
    
}

const double getNoteDiff(double frequency, int closestIndex) {
    double noteGap;
    double frequencyGap;

    if (frequency < noteFrequencies[closestIndex]) {
        noteGap = noteFrequencies[closestIndex] - noteFrequencies[closestIndex - 1];
    } else {
        noteGap = noteFrequencies[closestIndex + 1] - noteFrequencies[closestIndex];
    }
  
    frequencyGap = fabs(frequency - noteFrequencies[closestIndex]) / noteGap * 100;
    if (frequency < noteFrequencies[closestIndex]) {
        frequencyGap = -frequencyGap;
    }
    return frequencyGap;
}

void displayResult(int noteDiff, double frequency, int noteIndex){
    char frequencyStr[15];
    
    ssd1306_SetCursor(0, 10);
    snprintf(frequencyStr,sizeof(frequencyStr),"%.2f hz \n",frequency);
	  
	     
    ssd1306_Fill(Black);
    ssd1306_Line(64,0,(64+(128*noteDiff/100)),0, White);
    ssd1306_Line(64,1,(64+(128*noteDiff/100)),1, White);
    ssd1306_Line(64,2,(64+(128*noteDiff/100)),2, White);
    ssd1306_Line(64,3,(64+(128*noteDiff/100)),3, White);
    ssd1306_Line(64,4,(64+(128*noteDiff/100)),4, White);
    ssd1306_Line(64,5,(64+(128*noteDiff/100)),5, White);
    ssd1306_Line(64,6,(64+(128*noteDiff/100)),6, White);
    ssd1306_Line(64,7,(64+(128*noteDiff/100)),7, White);
    ssd1306_Line(64,8,(64+(128*noteDiff/100)),8, White);
    ssd1306_Line(64,9,(64+(128*noteDiff/100)),9, White);
    ssd1306_SetCursor(30,20);
    ssd1306_WriteString("  ",Font_11x18, White);

    ssd1306_WriteString(noteNames[noteIndex % 12], Font_16x26, White);
    ssd1306_SetCursor(25,50);
    ssd1306_WriteString(frequencyStr,Font_7x10, White);
    ssd1306_UpdateScreen();
     
}	     
	
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

void apply_fir_filter(float32_t *input, float32_t *output, uint32_t block_size) {
    arm_fir_f32(&fir_instance, input, output, block_size);
}

void apply_hps(float *magnitude_fft, float *hps_result, int hps_len) {
    // Inicializa o resultado do HPS com a magnitude da FFT
    for (int i = 0; i < hps_len; i++) {
        hps_result[i] = magnitude_fft[i];
    }

    // Decimar e multiplicar
    for (int decimation = 2; decimation <= 2; decimation++) { 
        for (int i = 0; i < hps_len / decimation; i++) {
            hps_result[i] *= magnitude_fft[i * decimation];
        }
    }
}

// Função principal para processar a FFT
void process_fft(void) {
    // Calcular a média do sinal (offset DC)
    char msg[65];
    float mean = 0.0f;

    int lastNoteIndex = 0;
    int lastNoteDiff = 0;
    double lastFrequency = 0;

    for (int i = 0; i < FRAME_LEN; i++) {
        mean += (float)adc_buffer[i] / 4096.0f;
    }
    mean /= FRAME_LEN;

    // Remover o offset DC e normalizar o sinal
    float32_t input_signal[FRAME_LEN];
    for (int i = 0; i < FRAME_LEN; i++) {
        input_signal[i] = ((float)adc_buffer[i] / 4096.0f) - mean;  // Remove DC
    }
    float32_t filtered_signal[FRAME_LEN];
    apply_fir_filter(input_signal, filtered_signal, FRAME_LEN);

    // Aplicar janela (Hamming)
    for (int i = 0; i < FRAME_LEN; i++) {
        float window = 0.54f - 0.46f * cosf(2 * PI * i / (FRAME_LEN - 1)); // Hamming window
        filtered_signal[i] *= window;
    }

    // Executar FFT
    arm_rfft_fast_f32(&fft_instance, filtered_signal, output_fft, 0);

    // Calcula magnitude da FFT (ignorando índice 0 e primeiros índices)
    arm_cmplx_mag_f32(output_fft, magnitude_fft, FRAME_LEN / 2);

    // Aplicar Harmonic Product Spectrum (HPS)
    int hps_len = FRAME_LEN / 2;
    float hps_result[hps_len];
    apply_hps(magnitude_fft, hps_result, hps_len);

    // Encontrar o pico máximo
    float max_value = 0.0f;
    int max_index = 0;
    int start_index = 5;  // Ignorar os primeiros 5 índices

    for (int i = start_index; i < hps_len; i++) {
        if (hps_result[i] > max_value) {
            max_value = hps_result[i];
            max_index = i;
        }
    }

    // Ignorar picos pequenos
    if (max_value < 0.1f) {
        max_index = 0;  // Frequência zero
    }

    // Converter índice para frequência em Hz
    float frequency = (float)max_index * SAMPLE_RATE / (FRAME_LEN / 2);
    int noteIndex = getClosestNoteIndex(frequency);

    if (frequency == 0 || noteIndex == 60 || noteIndex == 0) {
        displayResult(lastNoteDiff, lastFrequency, lastNoteIndex);
    } else {
        int noteDiff = getNoteDiff(frequency, noteIndex);
        displayResult(noteDiff, frequency, noteIndex);
        lastNoteDiff = noteDiff;
        lastFrequency = frequency;
        lastNoteIndex = noteIndex;
    }

    // Enviar para UART
    snprintf(msg, sizeof(msg), "Index: %d, Freq: %.2f Hz, nota mais prox: %d \r\n", max_index, frequency, noteIndex);
    usart_send_string(msg);
}

void i2c_setup(void) {
    /* enable clock for GPIOB and I2C1 */
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_I2C1);


    /* configure pins PB6 (SCL) and PB7 (SDA) as Alternate Function */
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6|GPIO7);
    gpio_set_af(GPIOB, GPIO_AF4, GPIO6|GPIO7);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, GPIO6 | GPIO7);
    
    /* reset and config I2C */


    i2c_peripheral_disable(SSD1306_I2C_PORT);

    i2c_set_clock_frequency(SSD1306_I2C_PORT, I2C_CR2_FREQ_42MHZ);
    i2c_set_fast_mode(SSD1306_I2C_PORT); /* fast mode */
    i2c_set_ccr(SSD1306_I2C_PORT, 35); /* configure CCR */
    i2c_set_trise(SSD1306_I2C_PORT, 42);

    i2c_peripheral_enable(SSD1306_I2C_PORT);
}

static void timer_init(void) {
    rcc_periph_clock_enable(RCC_TIM2);
    timer_set_prescaler(TIM2, 209);
    timer_set_period(TIM2, 99);
    timer_generate_event(TIM2, TIM_EGR_UG);
    timer_clear_flag(TIM2, TIM_EGR_UG);
    timer_set_master_mode(TIM2, TIM_CR2_MMS_UPDATE);
    timer_enable_counter(TIM2);
}

void usart_init(void) {

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
    cm_enable_interrupts();
    
    adc_init();
    usart_init();
    timer_init();
    i2c_setup();
    
    arm_rfft_fast_init_f32(&fft_instance, FRAME_LEN);
    arm_fir_init_f32(&fir_instance, NUM_TAPS, (float32_t *)fir_coeffs, fir_state, FRAME_LEN);


    ssd1306_Init();
    ssd1306_WriteString("Inicio", Font_11x18,White);
    ssd1306_UpdateScreen();
    usart_send_string("inicio do codigo\r\n");
    char msg[50];
    

    while (1) {
        if (frame_ready) {
            process_fft();
            frame_ready = 0;
        }
        __asm__("wfi");
    }
}

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>

#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/cm3/cortex.h>


#include <dsp/filtering_functions.h>
#include <dsp/transform_functions.h>
#include <dsp/complex_math_functions.h>
#include <dsp/statistics_functions.h>

#include "arm_math.h"
#include <arm_math_types.h>
#include <stdint.h>
#include <stdio.h>

#define ADC_DR_DATA_MASK 0x00000ff

#define nr_taps 128
#define sampling_rate 4000
#define oversampling_factor 2
#define OVER_FRAME_LEN  (frame_len*oversampling_factor)

#define max_frame_len 4096
#define frame_len 4095
#define max_nr_bins 2048


static volatile float32_t *volatile full_samples_frame = NULL;
static arm_fir_decimate_instance_f32 fir_decimate_instance;
static arm_rfft_fast_instance_f32 fft_instance;

static volatile float32_t samples[OVER_FRAME_LEN*2];


#define ADC_UINT12_MAX 4095
#define ADC_UINT12_ZERO_VAL 2047.5
#define ADC_UINT12_MIN_NEG -2047.5
#define ADC_UINT12_MAX_POS  2047.5

struct note_freq {
	const char *note_name;
	float32_t frequency;
};


struct note_freq note_freqs[] = {
	{ "C0",  16.351 },
	{ "C#0", 17.324 },
	{ "D0",  18.354 },
	{ "D#0", 19.445 },
	{ "E0",  20.601 },
	{ "F0",  21.827 },
	{ "F#0", 23.124 },
	{ "G0",	 24.499 },
	{ "G#0", 25.956 },
	{ "A0",  27.5 },
	{ "A#0", 29.135 },
	{ "B0",  30.868 },

	{ "C1",  32.703 },
	{ "C#1", 34.648 },
	{ "D1",  36.708 },
	{ "D#1", 38.891 },
	{ "E1",  41.203 },
	{ "F1",  43.654 },
	{ "F#1", 46.249 },
	{ "G1",  48.999 },
	{ "G#1", 51.913 },
/* 
 * Starting at A1 is the lowest note covered by tests because it's the 
 * lowest you'd realistically tune down to (or not?). Notes above this
 * comment (but "lower" in pitch) are only handled (and with poor frequency 
 * resolution) to get a gist of the pitch when tuning up for the first 
 * time after putting on a new set of strings.
 */
	{ "A1",  55 },
	{ "A#1", 58.27 },
	{ "B1",  61.735 },

	{ "C2",  65.406 },
	{ "C#2", 69.296 },
	{ "D2",  73.416 },
	{ "D#2", 77.782 },
	{ "E2",  82.407 },
	{ "F2",  87.307 },
	{ "F#2", 92.499 },
	{ "G2",  97.999 },
	{ "G#2", 103.826 },
	{ "A2",  110 },
	{ "A#2", 116.541 },
	{ "B2",  123.471 },

	{ "C3",  130.813 },
	{ "C#3", 138.591 },
	{ "D3",  146.832 },
	{ "D#3", 155.563 },
	{ "E3",  164.814 },
	{ "F3",  174.614 },
	{ "F#3", 184.997 },
	{ "G3",  195.998 },
	{ "G#3", 207.652 },
	{ "A3",  220 },
	{ "A#3", 233.082 },
	{ "B3",  246.942 },

	{ "C4",  261.626 },
	{ "C#4", 277.183 },
	{ "D4",  293.665 },
	{ "D#4", 311.127 },
	{ "E4",  329.628 },
	{ "F4",  349.228 },
	{ "F#4", 369.994 },
/*
 * Ending at F#4 is the highest note covered by tests, chosen because it's a
 * a few semitones above the high open E string. See also the next below comment.
 */
	{ "G4",  391.995 },
	{ "G#4", 415.305 },
	{ "A4",  440 },
	{ "A#4", 466.164 },
	{ "B4",  493.883 },
/*
 * Ending at B4 is the highest note expected to be reliably handled because 
 * its 4th harmonic is below the 2000 Hz cutoff frequency defined by the anti-aliasing
 * filter implemented in gen_filter_coeffs.m, and HPS uses 4 harmonics (see NHARMONICS).
 *
 * This doesn't affect getting the open strings in tune, which is the main use
 * case of this tuner, but may cause issues if trying to test the intonation 
 * of the high E string e.g. when checking its pitch at fret 12.
 */

	{ "C5",  523.251 },
	{ "C#5", 554.365 },
	{ "D5",  587.33 },
	{ "D#5", 622.254 },
	{ "E5",  659.255 },
	{ "F5",  698.456 },
	{ "F#5", 739.989 },
	{ "G5",  783.991 },
	{ "G#5", 830.609 },
	{ "A5",  880 },
	{ "A#5", 932.328 },
	{ "B5",  987.767 },
					 
	{ "C6",  1046.502 },
	{ "C#6", 1108.731 },
	{ "D6",  1174.659 },
	{ "D#6", 1244.508 },
	{ "E6",  1318.51 },
	{ "F6",	 1396.913 },
	{ "F#6", 1479.978 },
	{ "G6",  1567.982 },
	{ "G#6", 1661.219 },
	{ "A6",  1760 },
	{ "A#6", 1864.655 },
	{ "B6",  1975.533 },
/*
 * Ending at B6 is the highest note with fundamental frequency / first harmonic below
 * the cutoff frequency.
 */
	{ 0 }
};

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

static void timer2_set_sampling_rate(void)
{
	/*
	 * The timer is running off APB1, which is 6 MHz, but because the APB1 prescaler is > 1,
	 * the timer clock frequencies are twice APB1, which is 12 MHz. This clock divider sets
	 * the counter clock frequency to twice the OVERSAMPLING_RATE (i.e. 12 MHz / clock_div = 2*OVERSAMPLING_RATE),
	 * because it takes a clock cycle to count from 0 to 1, and another clock cycle to overflow 
	 * from 1 back to 0: each update event takes 2 cycles, so with a clock rate of 2*OVERSAMPLING_RATE 
	 * there will be OVERSAMPLING_RATE update events. 
	 *
	 * Note also the counter clock is lowered rather than raising the timer period in order to save power.
	 */
	const int clock_div = 750;
	timer_set_prescaler(TIM2, clock_div-1);
	timer_set_period(TIM2, 1);

	/* Trigger update event to load "preload" prescaler value set above into preload register proper. */
	timer_generate_event(TIM2, TIM_EGR_UG);
	timer_clear_flag(TIM2, TIM_EGR_UG);
}

static void timer_init(void)
{
	rcc_periph_clock_enable(RCC_TIM2);

	/* Below 2 lines put the timer in upcounting mode. */
	timer_set_alignment(TIM2, TIM_CR1_CMS_EDGE);
	timer_direction_up(TIM2);
	/* Generate an update event when timer upcounts from 0 to the period set with timer_set_period(). */
	timer_update_on_overflow(TIM2);
	timer_enable_update_event(TIM2);
	/* Send update event as trigger output (TRGO).  */
	timer_set_master_mode(TIM2, TIM_CR2_MMS_UPDATE);
	timer2_set_sampling_rate();
}

static void adc_init(void)
{
	const uint8_t adc_channel = 1;

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_ADC1);

	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);

	adc_set_clk_prescale(ADC_CCR_ADCPRE_BY8);  /* Use slowest clock to save power. */
	adc_set_resolution(ADC1, ADC_CR1_RES_12BIT);
	adc_set_single_conversion_mode(ADC1);
	adc_set_regular_sequence(ADC1, 1, (uint8_t *)&adc_channel);
	adc_enable_eoc_interrupt(ADC1);
	nvic_enable_irq(NVIC_ADC_IRQ);
	adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_TIM2_TRGO, ADC_CR2_EXTEN_RISING_EDGE);

	adc_power_on(ADC1);
}

void fft_init_sla(){
    extern const float32_t filter_coefficients[nr_taps];
    

    static float32_t fir_state[128+(2*4096)-1];

    
    arm_fir_decimate_init_f32(&fir_decimate_instance, nr_taps, oversampling_factor, filter_coefficients,
			      fir_state, oversampling_factor * frame_len);
    arm_rfft_fast_init_f32(&fft_instance, frame_len);
    
}

float32_t *samples_to_freq_bin_magnitudes(float32_t *samples, int FRAME_LEN)
{
	/*
	 * Each processing step below interleaves between using `buf` and `samples` as input/output
	 * buffers instead of allocating memory for each, in order to save MCU RAM space.
	 */
	static float32_t buf[max_frame_len]; 
	float32_t *filtered_samples = buf;
	float32_t *fft_complex_nrs, *freq_bin_magnitudes;

	/* Apply band-pass filter and decimate down from the OVERSAMPLING_RATE to SAMPLING_RATE. */
	arm_fir_decimate_f32(&fir_decimate_instance, samples, filtered_samples, oversampling_factor*frame_len);
	/* Convert from time domain to frequency domain. */
	fft_complex_nrs = samples;
	arm_rfft_fast_f32(&fft_instance, filtered_samples, fft_complex_nrs, 0);
	/* 
	 * Zero the first complex number because it's the DC offset and value at the Nyquist frequency 
	 * masquerading as a complex number.
	 */
	fft_complex_nrs[0] = fft_complex_nrs[1] = 0;
	/* 
	 * Get the energy of the spectra. Use regular mag over mag squared because the numbers mag
	 * squared ouput are too big and cause the result of HPS to overflow and give wrong results. 
	 */
	freq_bin_magnitudes = buf;
	arm_cmplx_mag_f32(fft_complex_nrs, freq_bin_magnitudes, max_nr_bins);
	return freq_bin_magnitudes;
}

int freq_to_bin_index(float32_t frequency, float32_t binwidth)
{
	/*
	 * The calculation here to round to the nearest bin index comes from testing
	 * and seeing which bin sine waves at particular frequencies fall into. 
	 * See assert_sine_wave_freq_to_bin_index() for these tests (see also the sine wave 
	 * data that it tests on).
	 */
	return round(frequency/binwidth);
}

int max_bin_index(float32_t *freq_bin_magnitudes, int FRAME_LEN)
{
	float32_t max;
	uint32_t max_index;

	arm_max_f32(freq_bin_magnitudes, frame_len/2, &max, &max_index);
	return max_index;
}

float32_t bin_index_to_freq(int bin_index, float32_t binwidth)
{
	return bin_index*binwidth;
}

float32_t bin_width(int FRAME_LEN, int SAMPLING_RATE)
{
    return (sampling_rate/2)/(frame_len/2);
}

float32_t convert_adc_u12_sample_to_s16(uint16_t u12_sample)
{
	float32_t diff_from_zero, s16_sample;
	static const float32_t scale_to_neg_int16 = INT16_MIN/ADC_UINT12_MIN_NEG;
	static const float32_t scale_to_pos_int16 = INT16_MAX/ADC_UINT12_MAX_POS;
	
	/* 
	 * The microphone has a DC bias of VCC/2 = 3.3/2 = 1.65V, i.e. the 
	 * "zero" value is at 1.65V: all readings below 1.65V are negative 
	 * numbers, and all above are positive. The 12-bit encoded value
	 * of 3.3V is ADC_UINT12_MAX, so the 12-bit encoded value of 1.65V is
	 * half ADC_UINT12_MAX (or ADC_UINT12_ZERO_VAL).
	 */
	if (u12_sample < ADC_UINT12_ZERO_VAL) {
		/* Sample is a negative number. */
		diff_from_zero = -(ADC_UINT12_ZERO_VAL-u12_sample);
		s16_sample = diff_from_zero*scale_to_neg_int16;
	} else {
		/* Sample is a positive number. */
		diff_from_zero = u12_sample-ADC_UINT12_ZERO_VAL;
		s16_sample = diff_from_zero*scale_to_pos_int16;
	}
	return s16_sample;
}





void adc_isr(void) 
{
	static int i = 0;

	samples[i++] = convert_adc_u12_sample_to_s16(adc_read_regular(ADC1)&ADC_DR_DATA_MASK);
	char debug_buffer[50];
	snprintf(debug_buffer, sizeof(debug_buffer), "ADC: %d\r\n", samples[i]);
	usart_send_string(debug_buffer);
	/* If just finished filling a frame of samples. */
	if (i%OVER_FRAME_LEN == 0) {
		full_samples_frame = samples+(i-OVER_FRAME_LEN);
		if (i == OVER_FRAME_LEN*2)
			i = 0;
	}
}



void harmonic_product_spectrum(float32_t *freq_bin_magnitudes, int FRAME_LEN, int SAMPLING_RATE)
{
	const int nbins = frame_len/2;
	int i;  /* Bin index. */

	/* Skip the frequencies below the lowest note. */

	i = freq_to_bin_index(note_freqs[0].frequency, (sampling_rate/2)/(frame_len/2) );
	for (bool finished = false; !finished; ++i) {
		/* Start at 2 because the current bin is the 1st harmonic. */
		for (int harmonic = 2; harmonic <= 4; ++harmonic) {
			/* 
			 * The frequency of current bin, bin at index i, is i*bin_width(). The frequency of
			 * the kth harmonic is k times the frequency of the current bin, k*(i*bin_width()).
			 * But this is also the same as (k*i)*bin_width(), the frequency of bin at index k*i,
			 * so the bin index of the harmonic is k*i (or harmonic*i).
			 *
			 * Note this is equivalent to downsampling/compression at a factor k for each harmonic
			 * and then taking the product at index i because the compression is shifting the bin
			 * index of the harmonic from index k*i down to index i.
			 */
			int harmonic_bin_index = harmonic*i;
			if (harmonic_bin_index >= nbins) {
				/* 
				 * The frequency of the harmonic is higher than the total bandwidth and
				 * not in the array, so can't use it in product. If it's the 2nd harmonic,
				 * the even higher (3rd, 4th, etc.) harmonics won't be in the bandwidth either, 
				 * and because the bins that follow span higher frequencies than the current bin, 
				 * their 2nd harmonics also won't be in bounds, so terminate.
				 */
				if (harmonic == 2)
					finished = true;
				break;
			}
			freq_bin_magnitudes[i] *= freq_bin_magnitudes[harmonic_bin_index];
		}
	}
}

static void processing_start(){
    	float32_t *freq_bin_magnitudes;
	int max_bin_ind;
	float32_t frequency;

	char bufferUart[50];
	
	while(1){
	    do {
		__asm__("wfi");
	    } while (!full_samples_frame);

	    freq_bin_magnitudes = samples_to_freq_bin_magnitudes((float32_t *)full_samples_frame, frame_len);	
	    max_bin_ind = max_bin_index(freq_bin_magnitudes, frame_len);	    
	    
	    max_bin_ind = max_bin_index(freq_bin_magnitudes, frame_len);
	    
	    frequency = bin_index_to_freq(max_bin_ind, bin_width(frame_len, sampling_rate));

	    
	    snprintf(bufferUart,sizeof(bufferUart), "freq: %.2f \r\n", frequency);
	    //    usart_send_string(bufferUart);
	}
    
}



int main(){
    rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);



    usart_setup();
    usart_send_string(" ** Inicio do código. ** \r\n");    
    cm_enable_interrupts();
    timer_init();
    adc_init();
    fft_init_sla();
    timer_enable_counter(TIM2);
    processing_start();
    

    while(1){
	
    }
			

    return 0;
}

const float32_t filter_coefficients[nr_taps] = {
	-0.000509719772, -0.00040682542, 0.000174891917, 6.82590253e-05, -0.00058944081,
	-0.000499212358, 0.000237542612, 0.000113508642, -0.000798678841, -0.000700031873,
	0.000359211263, 0.000198285416, -0.00114902609, -0.00102236355, 0.000556307437,
	0.00034181308, -0.00165169628, -0.00148003211, 0.000848398195, 0.000567564217,
	-0.0023187655, -0.00208892883, 0.00125951727, 0.000904651009, -0.00316527905,
	-0.00286928075, 0.00182046858, 0.0013903433, -0.00421280973, -0.00384956924,
	0.00257297116, 0.00207473012, -0.00549566094, -0.00507353805, 0.00357740349,
	0.00302966405, -0.00707229087, -0.00661346922, 0.0049280934, 0.00436689844,
	-0.0090480689, -0.00859743077, 0.00678595714, 0.00627796911, -0.0116255134,
	-0.011271555, 0.00945620611, 0.00913267583, -0.0152315609, -0.0151648661,
	0.0136045003, 0.0137673197, -0.0209093001, -0.0216300618, 0.0210194718,
	0.0225875657, -0.0319615267, -0.0353795849, 0.038702812, 0.0465542451,
	-0.0668606013, -0.0913642719, 0.148400351, 0.447491318, 0.447491318,
	0.148400351, -0.0913642719, -0.0668606013, 0.0465542451, 0.038702812,
	-0.0353795849, -0.0319615267, 0.0225875657, 0.0210194718, -0.0216300618,
	-0.0209093001, 0.0137673197, 0.0136045003, -0.0151648661, -0.0152315609,
	0.00913267583, 0.00945620611, -0.011271555, -0.0116255134, 0.00627796911,
	0.00678595714, -0.00859743077, -0.0090480689, 0.00436689844, 0.0049280934,
	-0.00661346922, -0.00707229087, 0.00302966405, 0.00357740349, -0.00507353805,
	-0.00549566094, 0.00207473012, 0.00257297116, -0.00384956924, -0.00421280973,
	0.0013903433, 0.00182046858, -0.00286928075, -0.00316527905, 0.000904651009,
	0.00125951727, -0.00208892883, -0.0023187655, 0.000567564217, 0.000848398195,
	-0.00148003211, -0.00165169628, 0.00034181308, 0.000556307437, -0.00102236355,
	-0.00114902609, 0.000198285416, 0.000359211263, -0.000700031873, -0.000798678841,
	0.000113508642, 0.000237542612, -0.000499212358, -0.00058944081, 6.82590253e-05,
	0.000174891917, -0.00040682542, -0.000509719772, 
};


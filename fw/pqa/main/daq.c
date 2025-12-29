/*
 * Data acquisition module
 */

#include <errno.h>
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include <string.h>
#include <strings.h>
#include <sys/time.h>
#include <time.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_dsp.h"

#include "mcp3913.h"

#include "board.h"
#include "daq.h"

/* external variables */
extern SemaphoreHandle_t	send_data_semaphore;

/* global variables */
SemaphoreHandle_t	daq_rest_samples_mutex;

/* local constants */
/* ADC crystal frequency in Hz */
#define DAQ_ADC_FOSC	(4194304)
/* sampling frequency in Hz - no resolution is lost */
#define DAQ_FS		(DAQ_ADC_FOSC / (4 * 512))
/* Calculate sampling period - convert to float */
#define DAQ_TS		(1.0 / DAQ_FS)

/* number of FFT points */
#define	DAQ_FFT_N		(2048)
/* FFT frequency resolution - make it float */
#define	DAQ_FFT_RESF		(1.0 * DAQ_FS / DAQ_FFT_N)

/* ADC voltage reference in millivolts*/
#define	DAQ_ADC_VREF	1200.0

/* Direction of zero-crossing detector */
#define DAQ_ZC_DIR_RISING	1
#define DAQ_ZC_DIR_FALLING	-1
#define DAQ_ZC_DIR_BOTH		0

/* Calibration constants */
/* Current sensors - phase error; measured for f=50Hz */
#if MCP3913_BOARD_SN == 1
#define	DAQ_CT_PHASE_ERROR_1	0.0
#define	DAQ_CT_PHASE_ERROR_2	0.0
#define	DAQ_CT_PHASE_ERROR_3	0.0
#elif  MCP3913_BOARD_SN == 2
#define	DAQ_CT_PHASE_ERROR_1	-8.2
#define	DAQ_CT_PHASE_ERROR_2	-5.2
#define	DAQ_CT_PHASE_ERROR_3	-4.9
#else
#error "Board S/N not specified"
#endif /* MCP3913_BOARD_SN */

/* local macros */
/* convert degrees to radians */
#define	PI		3.141592654
#define	DEG2RAD(deg)		(PI * (deg) / 180.0)
/* Convert raw value from ADC to millivolts */
#define DAQ_ADC2MILLIVOLTS(adc) ((DAQ_ADC_VREF / 1.5 * adc) / (1LL << 23))

/* Active data set */
#define DAQ_ACTIVE_SET		(daq_channels[daq_cset])
/* Sample from active data set for given channel */
#define	DAQ_SAMPLE(ch, sn)	(daq_channels[daq_cset][ch][sn])

/* global variables */
uint32_t		int_cnt;
int64_t			int_min_time = 100000000LL;
int64_t			int_max_time = 0;
uint32_t		skipped_samples = 0;

/*
 * ADC samples per one measurement 
 * two sample sets are allocated - while one is being filled with samples
 * by MCP3913 module, the other one is being processed (pointed to via
 * daq_cset variable).
 */
int	daq_cset = 0;
EXT_RAM_BSS_ATTR int32_t	daq_channels[DAQ_NUM_SETS][DAQ_ADC_CHANNELS][DAQ_N];
EXT_RAM_BSS_ATTR int32_t	daq_data[DAQ_ADC_CHANNELS][DAQ_N];

/* raw samples served by REST server */
EXT_RAM_BSS_ATTR int32_t	daq_rest_samples[DAQ_ADC_CHANNELS][DAQ_N];

/* Calculated values */
int64_t	daq_dc[DAQ_ADC_CHANNELS];
float	daq_rms[DAQ_ADC_CHANNELS];
float	daq_amp[DAQ_ADC_CHANNELS];

/* DAQ calculated values */
daq_t	daq;

/* L1, L2, L3 voltages and currents */
/* Convert ADC millivolts to input Volts */ 
#define DAQ_CALC_U(ch)	((300.47 / 470.0) * daq_rms[DAQ_U_OFFSET + (ch)])
/* Convert ADC millivolts to input Amperes - 1000mV/20A current sensors */
#define DAQ_CALC_I(ch)	((20.0 / 1000.0) * daq_rms[DAQ_I_OFFSET + (ch)])

/* FFT  */
/* Hanning window */
//EXT_RAM_BSS_ATTR static float	daq_fft_window[DAQ_FFT_N];
__attribute__((aligned(16)))
//static float	daq_fft_window[DAQ_FFT_N];
/* input signal as well as calculated FFT result */
//__attribute__((aligned(16)))
EXT_RAM_BSS_ATTR static float	daq_fft[DAQ_ADC_CHANNELS][DAQ_FFT_N * 2];

/* local definitions */
#define	daq_wind_hann	dsps_wind_hann_f32
#define	daq_fft_init	dsps_fft2r_init_fc32
#define	daq_fft2r	dsps_fft2r_fc32_aes3
#define	daq_bit_rev2r	dsps_bit_rev2r_fc32
#define	daq_cplx2real	dsps_cplx2real_fc32

/* local variables */
static const char *TAG = "pqa_dqa";

/* local functions */

/*
 * Zero crossing detector.
 *
 * Zero-crossing point is defined by two subsequent samples
 * with different sign.
 * Start searching in provided data set from index specified.
 * Return index of detected zero crossing point
 * or -1 if zero crossing point was not found.
 * Looks for positive, negative or both slopes.
 */
static int
daq_zcd_search(int32_t *s, int start, int dir)
{
	/* Some sanity checks */
	if (start < 0 || start >= DAQ_N - 1) {
		return (-1);
	}

	for (int i = start; i < DAQ_N - 1; i++) {
		int32_t s1 = s[i];
		int32_t s2 = s[i + 1];
		bool raising_zcd = ((s1 < 0) && (s2 >= 0));
		bool falling_zcd = ((s1 > 0) && (s2 <= 0));

		/* Evaluate if requested zero-crossing detected */
		if ((raising_zcd &&
		    (dir == DAQ_ZC_DIR_RISING || dir == DAQ_ZC_DIR_BOTH)) ||
		    (falling_zcd &&
		    (dir == DAQ_ZC_DIR_FALLING || dir == DAQ_ZC_DIR_BOTH))) {
			return (i + 1);
		}
	}

	/* zero-crossing not detected */
	return (-1);
}

/*
 * calculate frequency
 */
static float
daq_calc_freq(int ch)
{
	/* get activate data set for specified channel */
	int32_t	*s = DAQ_ACTIVE_SET[ch];

	/* Clean and initialize resulting values */
	daq.zcd[ch].first = -1;
	daq.zcd[ch].num = 0;
	/*
	 * Find zero-crossing points.
	 */
	for (int i = daq_zcd_search(s, 0, DAQ_ZC_DIR_BOTH); i != -1;
	    i = daq_zcd_search(s, i, DAQ_ZC_DIR_BOTH)) {
		daq.zcd[ch].num++;

		/* save the index of the first zc point found */
		if (daq.zcd[ch].first == -1) {
			daq.zcd[ch].first = i;
		}
		/*
		 * Save the index of the last zc point found.
		 * But only if it marks the complete period.
		 */
		if (daq.zcd[ch].num % 2 != 0) {
			daq.zcd[ch].last = i;
		}
	}

	/*
	 * Less than three zero-crossing points are not sufficient,
	 * since it means no complete period has been identified.
	 * Thus, return with error.
	 */
	if (daq.zcd[ch].num < 3) {
		return (-1);
	}

	/* If the last point belongs to incomplete period, drop it. */
	if (daq.zcd[ch].num % 2 == 0) {
		daq.zcd[ch].num--;
	}

	/* Calculate fractions for the first and last points. */
	float zcff = abs(s[daq.zcd[ch].first]) /
	    (abs(s[daq.zcd[ch].first]) + abs(s[daq.zcd[ch].first - 1]));
	float zclf = abs(s[daq.zcd[ch].last - 1]) /
	    (abs(s[daq.zcd[ch].last - 1]) + abs(s[daq.zcd[ch].last]));

	/* Now calculate frequency with 0.01 Hz resolution */
//	return ((int)((DAQ_FS * (daq_zcn - 1.0)) /
//	    (2.0 * (daq_zcl - daq_zcf))));
	return (DAQ_FS * (daq.zcd[ch].num - 1.0) /
	    (2.0 * (daq.zcd[ch].last - 1 - daq.zcd[ch].first + zclf + zcff)));
}

/*
 * Calculate phase shift between two channels.
 * Return value in <-180; 180) degrees or -360.0 in case of error.
 * Requires that frequency of channel 0 was previously calculated.
 */
static float
daq_calc_phase_shift(int ch1, int ch2)
{
	/* get activate data sets for specified channels */
	int32_t	*s1 = DAQ_ACTIVE_SET[ch1];
	int32_t	*s2 = DAQ_ACTIVE_SET[ch2];

	int ch1_zc, ch2_zc;

	/* Find first rising zero-crossing point for channel 1 */
	ch1_zc = daq_zcd_search(s1, 0, DAQ_ZC_DIR_RISING);

	/* Zero-crossing point not found */
	if (ch1_zc == -1) {
		return (-360.0);
	}

	/*
	 * Now find zero-crossing point for channel 2,
	 * starting searching from ch1_zc.
	 */
	ch2_zc = daq_zcd_search(s2, ch1_zc, DAQ_ZC_DIR_RISING);

	/* Zero-crossing point not found */
	if (ch2_zc == -1) {
		return (-360.0);
	}

	/* Calculate fractions for the first and the second points. */
	float zcff = 1.0 * s1[ch1_zc] / (s1[ch1_zc] - s1[ch1_zc - 1]);
	float zclf = -1.0 * s2[ch2_zc - 1] / (s2[ch2_zc] - s2[ch2_zc - 1]);

	/* Now calculate phase shift */
	float ph = 360.0 * daq.freq[DAQ_CHANNEL_U3]
	    * (ch2_zc - 1 - ch1_zc + zcff + zclf) / DAQ_FS;

	while (ph >= 180.0) {
		ph -= 360.0;
	}

	return (ph);
}

/*
 * Calculate DC offset for all channels and the whole data set.
 */
static void
daq_calc_dc(void)
{
	for (int i = 0; i < DAQ_ADC_CHANNELS; i++) {
		daq_dc[i] = 0;
		for (int j = 0; j < DAQ_N; j++) {
			daq_dc[i] += (int64_t)DAQ_SAMPLE(i, j);
		}
		daq_dc[i] /= DAQ_N;
	}
}

/*
 * Remove DC offset from all channels.
 * daq_calc_dc() must be called first to calculate DC offset.
 */
static void
daq_remove_dc(void)
{
//	int n = daq_zcl - daq_zcf + 1;

	/* calculate DC offset */
//	daq_mean = 0;
//	for (int i = daq_zcf; i <= daq_zcl; i++) {

//	for (int i = 0; i < DAQ_N; i++) {
//		daq_mean += (int64_t)s[i];
//	}
//	daq_mean /= n;
//	daq_mean = daq_mean / DAQ_N;

//	printf("daq_mean = %lld\n", daq_mean);

	/* remove DC offset */
	for (int i = 0; i < DAQ_ADC_CHANNELS; i++) {
		for (int j = 0; j < DAQ_N; j++) {
			DAQ_SAMPLE(i, j) -= daq_dc[i];
		}
	}
}

/*
 * Calculate RMS in millivolts for all channels and the whole data set.
 * daq_calc_dc() & daq_remove_dc() must be run first to remove DC offset.
 */
static void
daq_calc_rms(void)
{
//	int64_t sum = 0;
//	int n = daq_zcl - daq_zcf + 1;

	/* root-mean-square */
//	for (int i = daq_zcf; i <= daq_zcl; i++) {
	for (int i = 0; i < DAQ_ADC_CHANNELS; i++) {
		int64_t sum = 0;

		for (int j = 0; j < DAQ_N; j++) {
			int64_t s = (int64_t)DAQ_SAMPLE(i, j);
			sum += s * s;
		}
		daq_rms[i] = DAQ_ADC2MILLIVOLTS(sqrtf((float)(sum / DAQ_N)));
	}
}

/*
 * Calculate amplitude in millivolts for all channels and the whole data set.
 */
static void
daq_calc_amp(void)
{
	for (int i = 0; i < DAQ_ADC_CHANNELS; i++) {
		int64_t min = 1LL << 23;
		int64_t max = -min;

		for (int j = 0; j < DAQ_N; j++) {
			int64_t s = (int64_t)DAQ_SAMPLE(i, j);

			if (s < min) {
				min = s;
			}

			if (s > max) {
				max = s;
			}
		}
		daq_amp[i] = DAQ_ADC2MILLIVOLTS((max - min) / 2.0);
	}
}

/*
 * Calculate phase shifts
 */
static void
daq_calc_phase_shifts(void)
{
	daq.phase_shift[DAQ_PHASE_SHIFT_U1I1] =
	    daq_calc_phase_shift(DAQ_CHANNEL_U1, DAQ_CHANNEL_I1) -
	    DAQ_CT_PHASE_ERROR_1;
	daq.phase_shift[DAQ_PHASE_SHIFT_U2I2] =
	    daq_calc_phase_shift(DAQ_CHANNEL_U2, DAQ_CHANNEL_I2) -
	    DAQ_CT_PHASE_ERROR_2;
	daq.phase_shift[DAQ_PHASE_SHIFT_U3I3] =
	    daq_calc_phase_shift(DAQ_CHANNEL_U3, DAQ_CHANNEL_I3) -
	    DAQ_CT_PHASE_ERROR_3;
	daq.phase_shift[DAQ_PHASE_SHIFT_U1U2] =
	    daq_calc_phase_shift(DAQ_CHANNEL_U1, DAQ_CHANNEL_U2);
	daq.phase_shift[DAQ_PHASE_SHIFT_U2U3] =
	    daq_calc_phase_shift(DAQ_CHANNEL_U2, DAQ_CHANNEL_U3);
}

/*
 * Calculate power factor
 */
static void
daq_calc_pf(void)
{
	daq.L[DAQ_L1].pf =
	    cosf(DEG2RAD(daq.phase_shift[DAQ_PHASE_SHIFT_U1I1]));
	daq.L[DAQ_L2].pf =
	    cosf(DEG2RAD(daq.phase_shift[DAQ_PHASE_SHIFT_U2I2]));
	daq.L[DAQ_L3].pf =
	    cosf(DEG2RAD(daq.phase_shift[DAQ_PHASE_SHIFT_U3I3]));
}

/*
 * Calculate active and reactive power
 */
static void
daq_calc_power(void)
{
	for (int i = 0; i < DAQ_LNUM; i++) {
		//float pf_q = sqrtf(1.0 - daq.pf[i] * daq.pf[i]);

		/*
		 * Calculate pf_q as sinf(phase_shift) instead of using formula
		 * pf_q = sqrtf(1.0 - pf^2) to obtain type of calculated
		 * reactive power, inductive(positive) or capacitive(negative).
		 */
		float pf_q = sinf(
		    DEG2RAD(daq.phase_shift[DAQ_PHASE_SHIFT_U1I1 + i]));

		daq.L[i].P = daq.L[i].U * daq.L[i].I * daq.L[i].pf;
		daq.L[i].Q = daq.L[i].U * daq.L[i].I * pf_q;
	}
}

/*
 * Calculate L1, L2, L3 voltages and currents
 */
static void
daq_calc_ui(void)
{
	/* Voltages */
	for (int i = 0; i < DAQ_LNUM; i++) {
		daq.L[i].U = DAQ_CALC_U(i);
	}

	/* Currents */
	for (int i = 0; i < DAQ_LNUM; i++) {
		daq.L[i].I = DAQ_CALC_I(i);
	}
}

/*
 * Calculate FFT.
 */
static void
daq_calc_fft(void)
{
	for (int i = 0; i < DAQ_ADC_CHANNELS; i++) {
		/*
		 * Apply window to the input signal and convert it to float.
		 */
		for (int j = 0; j < DAQ_FFT_N ; j++) {
			daq_fft[i][2 * j] = DAQ_SAMPLE(i, j);
			daq_fft[i][2 * j + 1] = 0;
		}

		/* FFT */
		daq_fft2r(daq_fft[i], DAQ_FFT_N);

		/* Bit reverse */
		daq_bit_rev2r(daq_fft[i], DAQ_FFT_N);

		/*
		 * Convert one complex vector with length N
		 * to one real spectrum vector with length N.
		 */
		daq_cplx2real(daq_fft[i], DAQ_FFT_N);

		/* Calculate logarithmic absolute values */
		for (int j = 0 ; j < DAQ_FFT_N; j++) {
			float x1 = daq_fft[i][j * 2];
			float x2 = daq_fft[i][j * 2 + 1];

			daq_fft[i][j] = 10.0 * log10f((x1 * x1 + x2 * x2)
			    / DAQ_FFT_N);
		}
	}
}

/* global functions */

/*
 * Data acquisition task
 */
void
daq_task(void *args)
{
	static int iot_send_cycle;

	bool data_sent = false;

	int64_t	t1;

	/* Initialize data structures */

	/* FFT */
	int ret = daq_fft_init(NULL, DAQ_FFT_N);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to initialize FFT. Error %i", ret);
	}

	/* Initialize Hanning window for FFT */
//	daq_wind_hann(daq_fft_window, DAQ_FFT_N);

	/* initialize MCP3913 */
	mcp3913_init();

	/* Create mutex protecting raw ADC samples served by REST server */
	daq_rest_samples_mutex = xSemaphoreCreateMutex();

	while (1) {
		static uint32_t int_cnt_prev;

		int64_t ct;
		uint32_t int_cnt_diff;

		/* wait until data set is ready */
		xSemaphoreTake(mcp3913_semaphore, portMAX_DELAY);

		/* Measure how long the whole round takes */
		t1 = esp_timer_get_time();

		/* save raw samples for REST server */
		/* lock data set */
		xSemaphoreTake(daq_rest_samples_mutex, portMAX_DELAY);

		ct = esp_timer_get_time();
		/* Save current samples to be served by REST server */
		bcopy(DAQ_ACTIVE_SET, daq_rest_samples, sizeof (daq_rest_samples));
		ct = esp_timer_get_time() - ct;
		printf("bcopy(daq_rest_samples): %lld us\n", ct);

		/* unlock data set */
		xSemaphoreGive(daq_rest_samples_mutex);

		/* Send data to the server - only once */
		if (!data_sent) {
			data_sent = true;
			memcpy(daq_data, DAQ_ACTIVE_SET,
			    sizeof (daq_data));

			xSemaphoreGive(send_data_semaphore);
		}

		int_cnt_diff = int_cnt - int_cnt_prev;
		int_cnt_prev = int_cnt;
		printf("daq_cset: %u\n", daq_cset);
		printf("int_cnt_diff: %lu\n", int_cnt_diff);
		printf("int_min_time: %lld\n", int_min_time);
		printf("int_max_time: %lld\n", int_max_time);
//		printf("skipped_samples: %lu\n", skipped_samples);

		/* Process collected samples */
		for (int i = 0; i < DAQ_ADC_CHANNELS; i++) {
			int64_t mean;
			int32_t min, max;

			mean = 0;
			min = 1 << 24;
			max = -(1 << 24);

			for (int j = 0; j < DAQ_N; j++) {
				int32_t sample = DAQ_SAMPLE(i, j);

				/* Calculate min, max, mean values */
				mean += (int64_t) sample;
				if (sample < min) {
					min = sample;
				}
				if (sample > max) {
					max = sample;
				}
			}
			mean /= DAQ_N;

			printf("Channel %d: %lld <%ld, %ld>\n", i,
			    mean, min, max);
		}

		ct = esp_timer_get_time();

		/* Calculate DC offset for all channels */
		daq_calc_dc();

		/* Remove DC offset from all channels */
		daq_remove_dc();

		/*
		 * Calculate zero-crossing points
		 * and frequency for channel 0
		 */
		daq.freq[DAQ_CHANNEL_I1] = daq_calc_freq(DAQ_CHANNEL_I1);
		daq.freq[DAQ_CHANNEL_U1] = daq_calc_freq(DAQ_CHANNEL_U1);
		daq.freq[DAQ_CHANNEL_U3] = daq_calc_freq(DAQ_CHANNEL_U3);
		ct = esp_timer_get_time() - ct;

		printf("Zero-crossing: zcf = %d, zcl = %d, zcn = %d\n",
		    daq.zcd[0].first, daq.zcd[0].last, daq.zcd[0].num);
//		printf("U1 freq: %.3f Hz, %lld us\n", daq.freq[3], ct);
//		printf("I1 freq: %.3f Hz, %lld us\n", daq.freq[0], ct);
		printf("U3 freq: %.3f Hz, %lld us\n", daq.freq[DAQ_CHANNEL_U3],
		    ct);

		ct = esp_timer_get_time();
		/* Calculate RMS for all channels */
		daq_calc_rms();

		/* Calculate L1, L2, L3 voltages and currents */
		daq_calc_ui();

		/* Calculate phase shifts */
		daq_calc_phase_shifts();

		/* Calculate power factors */
		daq_calc_pf();

		/* Calculate power */
		daq_calc_power();

		/* Calculate amplitude for all channels */
		daq_calc_amp();
		ct = esp_timer_get_time() - ct;

		/* Mark calculated values with timestamp */
		if (gettimeofday(&daq.ts, NULL) != 0) {
			ESP_LOGE(TAG, "gettimeofday() failed with errno %d",
			    errno);
		}

		printf("---------------------------------------------------");
		printf("-----------\n");
		printf("         ch0\tch1\tch2\tch3\tch4\tch5\ttime\n");

		/* Print DC offsets */
		printf("dc     : ");
		for (int i = 0; i < DAQ_ADC_CHANNELS; i++) {
			printf("%lld\t", daq_dc[i]);
		}
		printf("\n");

		/* Print RMS */
		printf("rms[mV]: ");
		for (int i = 0; i < DAQ_ADC_CHANNELS; i++) {
			printf("%0.2f\t", daq_rms[i]);
		}

		/* Print amplitude */
		printf("\namp[mV]: ");
		for (int i = 0; i < DAQ_ADC_CHANNELS; i++) {
			printf("%0.2f\t", daq_amp[i]);
		}

		/* Print RMS */
		printf("\nrms[UI]: ");
		for (int i = 0; i < DAQ_ADC_CHANNELS; i++) {
			if (i <= DAQ_CHANNEL_I3) {
				printf("%0.3fA\t", daq_rms[i] * 2.0e-2);
			} else {
				printf("%0.2fV\t", daq_rms[i] *
				    (300.470 / 470.0));
			}
		}

		/* Print amplitude */
		printf("\namp[UI]: ");
		for (int i = 0; i < DAQ_ADC_CHANNELS; i++) {
			if (i <= DAQ_CHANNEL_I3) {
				printf("%0.3fA\t", daq_amp[i] * 2.0e-2);
			} else {
				printf("%0.2fV\t", daq_amp[i] *
				    (300.470 / 470.0));
			}
		}
		printf("%lld us\n", ct);

		/* Print phase shift */
		printf("ps[deg]: ");
		printf("%0.1f\t", daq_calc_phase_shift(0, 1));
		printf("%0.1f\t", daq_calc_phase_shift(0, 2));
		printf("%0.1f\t", daq_calc_phase_shift(3, 4));
		printf("%0.1f\t", daq_calc_phase_shift(3, 5));
		printf("%0.1f\t", daq_calc_phase_shift(3, 0) -
		    DAQ_CT_PHASE_ERROR_1);
		printf("%0.1f\t", daq_calc_phase_shift(4, 1) -
		    DAQ_CT_PHASE_ERROR_2);
		printf("%0.1f\t", daq_calc_phase_shift(5, 2) -
		    DAQ_CT_PHASE_ERROR_3);
		printf("\n");

		/* Print power factor */
		printf("cos(fi): ");
		for (int i = 0; i < DAQ_LNUM; i++) {
			printf("%0.2f\t", daq.L[i].pf);
		}
		printf("\n");
		
		/* Print power */
		printf("P[W] Q[VA]: ");
		for (int i = 0; i < DAQ_LNUM; i++) {
			printf("%0.2f\t%0.2f\t",
			    daq.L[i].P, daq.L[i].Q);
		}
		printf("\n");

		/* Calculate FFT for ch0 */
		ct = esp_timer_get_time();
		daq_calc_fft();
		ct = esp_timer_get_time() - ct;

		printf("FFT: %lld us", ct);
		for (int i = 0; i < DAQ_ADC_CHANNELS; i++) {
			printf("\nCh%d: ", i);

			for (int j = 0; j < DAQ_FFT_N / 2; j++) {
/*
				if (i % 100 == 0) {
					printf("\n");
				}
*/

				/* Always display DC */
				if (daq_fft[i][j] < 110.0 && j != 0) {
					continue;
				}

				printf("%s%0.2f=%0.2f", j > 0 ? ", " : "",
				    DAQ_FFT_RESF * j, daq_fft[i][j]);
			}
		}

		t1 = esp_timer_get_time() - t1;
		printf("\nt1: %lld us\n", t1);

		printf("---------------------------------------------------");
		printf("-----------\n");

		/*
		 * Once a while send voltages and currents
		 * to ThingSpeak IoT server.
		 */
		if (++iot_send_cycle % 8 == 0) {
			xSemaphoreGive(send_data_semaphore);
		}

//        	vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

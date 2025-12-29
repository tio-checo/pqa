/*
 * Data acquisition module
 */

#include <time.h>

/* number of ADC channels */
#define DAQ_ADC_CHANNELS	6
/* number of collected samples per one measurement */
#define DAQ_N			4096
/* number of data sets */
#define DAQ_NUM_SETS		2

/* analog channels */
#define	DAQ_CHANNEL_I1	0
#define	DAQ_CHANNEL_I2	1
#define	DAQ_CHANNEL_I3	2
#define	DAQ_CHANNEL_U1	3
#define	DAQ_CHANNEL_U2	4
#define	DAQ_CHANNEL_U3	5

/* Number of voltage and current channels */
#define DAQ_U_OFFSET	3
#define DAQ_I_OFFSET	0

/* number of calculate power factors and powers */
#define	DAQ_PF_NUM	3
#define	DAQ_POWER_NUM	3

/* types */
/* phase shift between two measured signals */
typedef enum {
	DAQ_PHASE_SHIFT_U1I1,
	DAQ_PHASE_SHIFT_U2I2,
	DAQ_PHASE_SHIFT_U3I3,
	DAQ_PHASE_SHIFT_U1U2,
	DAQ_PHASE_SHIFT_U2U3,
	DAQ_PHASE_SHIFT_NUM
} daq_phase_shift_t;

/* AC lines */
typedef enum {
	DAQ_L1,
	DAQ_L2,
	DAQ_L3,
	DAQ_LNUM
} daq_l_t;

/* DAQ calculated values */
typedef struct {
	/* timestamp */
	struct timeval ts;

	/* zero-crossing points */
	struct {
		int	first;	/* first point */
		int	last;	/* last point */
		int	num;	/* total number of points */
	} zcd[DAQ_ADC_CHANNELS];

	float	freq[DAQ_ADC_CHANNELS];	/* frequency */
	float	phase_shift[DAQ_PHASE_SHIFT_NUM];	/* phase shift */

	/* AC line characteristics */
	struct {
		float	U;	/* voltage */
		float	I;	/* current */

		float	pf;	/* power factor */
		float	P;	/* active power */
		float	Q;	/* reactive power */
	} L[DAQ_LNUM];
} daq_t;

/* global variables */
extern int32_t	daq_channels[DAQ_NUM_SETS][DAQ_ADC_CHANNELS][DAQ_N];
extern int	daq_cset;
extern daq_t	daq;
/* mutex protecting raw ADC samples served by REST server */
extern SemaphoreHandle_t	daq_rest_samples_mutex;

/* raw samples served by REST server */
extern int32_t	daq_rest_samples[DAQ_ADC_CHANNELS][DAQ_N];

/*
 * Data acquisition task
 */
void daq_task(void *);

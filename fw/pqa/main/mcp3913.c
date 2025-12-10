/*********************************************************************
 *
 * Support for MCP3913 six-channel Analog Front End (AFE).
 *
 *********************************************************************/

#include <ctype.h>
#include <errno.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "mcp3913.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"

#include "esp_timer.h"

#include "freertos/semphr.h"

#include "board.h"
#include "daq.h"

/* constants */
/* SPI address */
#define MCP3913_SPI_ADDR	(0x01 << 6)
/* SPI read */
#define MCP3913_SPI_READ	0x01
/* SPI write */
#define MCP3913_SPI_WRITE	0x00
/* number of SPI bytes per one read value */
#define MCP3913_SPI_BYTES_PER_READ	3

/* addresses of MCP3913 registers */
#define MCP3913_REG_CH0		0x00	/* Channel 0 ADC */
#define MCP3913_REG_CH1		0x01	/* Channel 1 ADC */
#define MCP3913_REG_CH2		0x02	/* Channel 2 ADC */
#define MCP3913_REG_CH3		0x03	/* Channel 3 ADC */
#define MCP3913_REG_CH4		0x04	/* Channel 4 ADC */
#define MCP3913_REG_CH5		0x05	/* Channel 5 ADC */
#define MCP3913_REG_MOD		0x08	/* Delta-Sigma Modulators Output Val */
#define MCP3913_REG_PHASE0	0x09	/* Phase Delay Config - Ch 4/5 */
#define MCP3913_REG_PHASE1	0x0A	/* Phase Delay Config - Ch 0/1, 2/3 */
#define MCP3913_REG_GAIN	0x0B	/* Gain Configuration */
#define MCP3913_REG_STATUSCOM	0x0C	/* Status and Communication Register */
#define MCP3913_REG_CONFIG0	0x0D	/* Configuration Register */
#define MCP3913_REG_CONFIG1	0x0E	/* Configuration Register */
#define MCP3913_REG_OFFCAL_CH0	0x0F	/* Offset Correction - Channel 0 */
#define MCP3913_REG_GAINCAL_CH0	0x10	/* Gain Correction - Channel 0 */
#define MCP3913_REG_OFFCAL_CH1	0x11	/* Offset Correction - Channel 1 */
#define MCP3913_REG_GAINCAL_CH1	0x12	/* Gain Correction - Channel 1 */
#define MCP3913_REG_OFFCAL_CH2	0x13	/* Offset Correction - Channel 2 */
#define MCP3913_REG_GAINCAL_CH2	0x14	/* Gain Correction - Channel 2 */
#define MCP3913_REG_OFFCAL_CH3	0x15	/* Offset Correction - Channel 3 */
#define MCP3913_REG_GAINCAL_CH3	0x16	/* Gain Correction - Channel 3 */
#define MCP3913_REG_OFFCAL_CH4	0x17	/* Offset Correction - Channel 4 */
#define MCP3913_REG_GAINCAL_CH4	0x18	/* Gain Correction - Channel 4 */
#define MCP3913_REG_OFFCAL_CH5	0x19	/* Offset Correction - Channel 5 */
#define MCP3913_REG_GAINCAL_CH5	0x1A	/* Gain Correction - Channel 5 */
#define MCP3913_REG_LOCK_CRC	0x1F	/* Security Register */

/* Gain 1, 2, 4, 8, 16, 32 */
#define MCP3913_GAIN_1		0x00
#define MCP3913_GAIN_2		0x01
#define MCP3913_GAIN_4		0x02
#define MCP3913_GAIN_8		0x03
#define MCP3913_GAIN_16		0x04
#define MCP3913_GAIN_32		0x05

/* SPI port and pins */
#define MCP3913_SPI_HOST	SPI2_HOST

#define MCP3913_SPI_MISO	12
#define MCP3913_SPI_MOSI	13
#define MCP3913_SPI_CLK		11
#define MCP3913_SPI_CS		10

/* GPIO connected to interrupt pin */
#define MCP3913_INT_PIN		7

/* debug pin */
#define	MCP3913_DBG_PIN		GPIO_NUM_17

/* external variables */
extern uint32_t			int_cnt;
extern int64_t			int_min_time, int_max_time;
extern uint32_t			skipped_samples;

/* global variables */
SemaphoreHandle_t	mcp3913_semaphore;

/* local types */

/* local variables */
static spi_device_handle_t spi;

static int32_t 	adc[MCP3913_NUM_CHANNELS];

/*
 * ADC calibration offsets
 * This value is simply added to the output code of the channel, bit-by-bit.
 * It is a 24-bit two’s complement MSB first coding.
 * CHn Output Code = OFFCAL_CHn + ADC CHn Output Code.
 */
#if MCP3913_BOARD_SN == 1
static const int32_t	adc_offcal[MCP3913_NUM_CHANNELS] = {
    -3533, -5547, -3683, -6400, -6145, -3653};
#elif  MCP3913_BOARD_SN == 2
static const int32_t	adc_offcal[MCP3913_NUM_CHANNELS] = {
    -5244, -6656, -6297, -6405, -6200, -5978};
#else
#error "Board S/N not specified"
#endif /* MCP3913_BOARD */

/*
 * ADC calibration gains
 * These are 24-bit signed MSB first coding with a range of -1x to +0.9999999x
 * (from 0x800000 to 0x7FFFFF). The gain calibration adds 1x to this value
 * and multiplies it to the output code of the channel, bit-by-bit, after offset
 * calibration. The range of the gain calibration is thus from 0x to 1.9999999x
 * (from 0x800000 to 0x7FFFFF). The LSB corresponds to a 2^-23 increment
 * in the multiplier.
 * CHn Output Code = (GAINCAL_CHn + 1) * ADC CHn Output Code.
 */
#if MCP3913_BOARD_SN == 1
#define ADC_GAINCAL		((1 << 23) * 100 / 103 - (1 << 23))
static const int32_t	adc_gaincal[MCP3913_NUM_CHANNELS] = {
    ADC_GAINCAL, ADC_GAINCAL, ADC_GAINCAL,
    ADC_GAINCAL, ADC_GAINCAL, ADC_GAINCAL};
#elif  MCP3913_BOARD_SN == 2
/* I1 */
#define ADC_GAINCAL0		((1LL << 23) * 697 / 706 - (1 << 23))
/* I2 */
#define ADC_GAINCAL1		((1LL << 23) * 697 / 711 - (1 << 23))
/* I3 */
#define ADC_GAINCAL2		((1LL << 23) * 697 / 717 - (1 << 23))
/* U1 */
//#define ADC_GAINCAL3		((1LL << 23) * 2400 / 2400 - (1 << 23))
#define ADC_GAINCAL3		((1LL << 23) * 2400 / 2444 - (1 << 23))
/* U2 */
//#define ADC_GAINCAL4		((1LL << 23) * 2400 / 2400 - (1 << 23))
#define ADC_GAINCAL4		((1LL << 23) * 2400 / 2438 - (1 << 23))
/* U3 */
//#define ADC_GAINCAL5		((1LL << 23) * 2400 / 2400 - (1 << 23))
#define ADC_GAINCAL5		((1LL << 23) * 2400 / 2433 - (1 << 23))
static const int32_t	adc_gaincal[MCP3913_NUM_CHANNELS] = {
    ADC_GAINCAL0, ADC_GAINCAL1, ADC_GAINCAL2,
    ADC_GAINCAL3, ADC_GAINCAL4, ADC_GAINCAL5};
#else
#error "Board S/N not specified"
#endif /* MCP3913_BOARD_SN */

/* local functions */
static void IRAM_ATTR
mcp3913_isr_handler(void *arg)
{
	static int	curr;
	static int	cnt;
	static int64_t	prev_time = -1;
	static int64_t	curr_time, diff_time;

	/* sample data set */
	int	daq_sset = (daq_cset + 1) % DAQ_NUM_SETS;

	/* activate debug pin */
	gpio_set_level(MCP3913_DBG_PIN, 0);

	/* Measure min and max time between interrupts */
	if (cnt % 10 == 0) {
		int_min_time = 10000000LL;
		int_max_time = 0;
	}

	/* de-activate debug pin */
	gpio_set_level(MCP3913_DBG_PIN, 1);

	curr_time = esp_timer_get_time();

	/* activate debug pin */
	gpio_set_level(MCP3913_DBG_PIN, 0);
	/* First measurement, initialize */
	if (prev_time == -1) {
		prev_time = curr_time;
	} else {
		diff_time = curr_time - prev_time;
		prev_time = curr_time;

		if (diff_time < int_min_time) {
			int_min_time = diff_time;
		}

		if (diff_time > int_max_time) {
			int_max_time = diff_time;
		}
	}

	int_cnt++;

	/* de-activate debug pin */
	gpio_set_level(MCP3913_DBG_PIN, 1);

	/* Read all analog channels at once */
	mcp3913_read_all_channels(adc);

	/* activate debug pin */
	gpio_set_level(MCP3913_DBG_PIN, 0);

	/*
	 * Store measured values in the sample data set.
	 */
	for (int i = 0; i < MCP3913_NUM_CHANNELS; i++) {
		daq_channels[daq_sset][i][curr] = adc[i];
	}

	/* Enough samples collected */
	if (++curr >= DAQ_N) {
		curr = 0;
		cnt++;

		/* let upper layer process just collected data set */
		daq_cset = daq_sset;

		xSemaphoreGiveFromISR(mcp3913_semaphore, NULL);
	}

	/* de-activate debug pin */
	gpio_set_level(MCP3913_DBG_PIN, 1);
}

/* global functions */
/*
 * Initialization routine.
 */
int
mcp3913_init(void)
{
	esp_err_t ret;

	/*
	 * semaphore signaling upper layer
	 * that new sampled data set is available
	 */
	mcp3913_semaphore = xSemaphoreCreateBinary();

	/* SPI configuration */
	spi_bus_config_t buscfg = {
		.miso_io_num = MCP3913_SPI_MISO,
		.mosi_io_num = MCP3913_SPI_MOSI,
		.sclk_io_num = MCP3913_SPI_CLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 32
	};
	spi_device_interface_config_t devcfg = {
		.clock_speed_hz = 10 * 1000 * 1000,	//Clock out at 10 MHz
		.mode = 0,				//SPI mode 0
		.spics_io_num = MCP3913_SPI_CS,		//CS pin
		.queue_size = 1,	//Able to queue 1 transaction at a time
	};

	/* Initialize the SPI bus */
	ret = spi_bus_initialize(MCP3913_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
	ESP_ERROR_CHECK(ret);

	/* Attach MCP3913 to the SPI bus */
	ret = spi_bus_add_device(MCP3913_SPI_HOST, &devcfg, &spi);
	ESP_ERROR_CHECK(ret);

	/*
	 * StatusCom register
	 *  - READ[1:0] = 10
	 *  - WRITE = 1
	 *  - DR_HIZ = 0
	 *  - DR_LINK = 1
	 *  - WIDTH_CRC = 0
	 *  - WIDTH_DATA[1:0] = 01
	 *  - EN_CRCCOM = 0
	 *  - EN_INT = 0
	 */
	ret = mcp3913_write(MCP3913_REG_STATUSCOM, 0xa90000);

	/*
	 * Config0 register
	 *  - EN_OFFCAL = 1		- Offset Error Calibration enabled
	 *  - EN_GAINCAL = 1		- Gain Error Calibration enabled
	 *  - DITHER[1:0] = 11		- Maximum Dithering Strength
	 *  - BOOST[1:0] = 10		- Boost x1
	 *  - PRE[1:0] = 00		- AMCLK = MCLK
	 *  - OSR[2:0] = 100		- 512 oversampling ratio, 2048 samples/s
	 *  - VREFCAL[7:0] = 0x50	- Vref Temperature Coefficient
	 */
	ret = mcp3913_write(MCP3913_REG_CONFIG0, 0xf88050);

	/*
	 * Config1 register
	 *  - RESET[5:0] = 0		- Channels not in Soft Reset Mode
	 *  - SHUTDOWN[5:0] = 0		- Channels not in Shutdown Mode
	 *  - VREFEXT = 0		- Internal voltage reference
	 *  - CLKEXT = 0		- MCLK generated from crystal oscillator
	 */
	ret = mcp3913_write(MCP3913_REG_CONFIG1, 0x00);

	/* Digital Offset Calibration registers */
	for (int i = 0; i < MCP3913_NUM_CHANNELS; i++) {
		ret = mcp3913_write(MCP3913_REG_OFFCAL_CH0 + 2 * i,
		    (uint32_t)adc_offcal[i]);
	}

	/* Digital Gain Error Calibration registers */
	for (int i = 0; i < MCP3913_NUM_CHANNELS; i++) {
		ret = mcp3913_write(MCP3913_REG_GAINCAL_CH0 + 2 * i,
		    (uint32_t)adc_gaincal[i]);
	}

	/*
	 * Gain register
	 *  - Gain 1 for all channels
	 */
	ret = mcp3913_write(MCP3913_REG_GAIN,
	    MCP3913_GAIN_1 << 0 | MCP3913_GAIN_1 << 3 | MCP3913_GAIN_1 << 6 |
	    MCP3913_GAIN_1 << 9 | MCP3913_GAIN_1 << 12 | MCP3913_GAIN_1 << 15);
#if 0
	/*
	 * Configure LEDC timer to generate MCLK for ADC
	 */
	ledc_timer_config_t ledc_timer = {
		.timer_num = LEDC_TIMER_0,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.duty_resolution = LEDC_TIMER_2_BIT,
		.freq_hz = 10000000,		/* 10 MHz */
		.clk_cfg = LEDC_AUTO_CLK
	};
	ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

	ledc_channel_config_t ledc_channel = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel = LEDC_CHANNEL_0,
		.timer_sel = LEDC_TIMER_0,
		.intr_type = LEDC_INTR_DISABLE,
		.gpio_num  = 5,
		.duty = 2,
	};
	ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
#endif
	/* debug pin */
	gpio_set_direction(MCP3913_DBG_PIN, GPIO_MODE_OUTPUT);

	/* Initialize interrupt handler */
	/* Interrupt pin */
	gpio_config_t int_conf = {
		.pin_bit_mask = 1ULL << MCP3913_INT_PIN,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = 1,
		.intr_type = GPIO_INTR_NEGEDGE,
	};
	gpio_config(&int_conf);
	gpio_install_isr_service(0);
	gpio_isr_handler_add(MCP3913_INT_PIN, mcp3913_isr_handler, NULL);

	return (0);
}

/*
 * Read register.
 */
int
mcp3913_read(uint8_t reg, uint32_t *data)
{
	esp_err_t ret;
	spi_transaction_t t;
	uint8_t b0 = MCP3913_SPI_ADDR | (reg << 1) | MCP3913_SPI_READ;

	// When using SPI_TRANS_CS_KEEP_ACTIVE, bus must be locked/acquired
	spi_device_acquire_bus(spi, portMAX_DELAY);

	memset(&t, 0, sizeof (t));
	t.length = 8 * 4;
	t.rxlength = 8 * 3;
	t.tx_buffer = &b0;
	t.flags = SPI_TRANS_USE_RXDATA;

	ret = spi_device_polling_transmit(spi, &t);
	assert(ret == ESP_OK);

	*data = 0;
	for (int i = 1; i <= 3; i++) {
		*data <<= 8;
		*data |= (uint32_t)t.rx_data[i];
	}

	// Release bus
	spi_device_release_bus(spi);

	return (ret);
}

/*
 * Read all six ADC channels at once.
 * Expects data space allocated by caller.
 */
int
mcp3913_read_all_channels(int32_t *data)
{
	esp_err_t ret;
	spi_transaction_t t;
	uint8_t b0 = MCP3913_SPI_ADDR | (MCP3913_REG_CH0 << 1) |
	    MCP3913_SPI_READ;
	uint8_t rx_buf[1 + MCP3913_NUM_CHANNELS * MCP3913_SPI_BYTES_PER_READ];
	uint32_t *d = (uint32_t *)data;


	// When using SPI_TRANS_CS_KEEP_ACTIVE, bus must be locked/acquired
	spi_device_acquire_bus(spi, portMAX_DELAY);

	memset(&t, 0, sizeof (t));
	t.length = 8 * (1 + MCP3913_NUM_CHANNELS * MCP3913_SPI_BYTES_PER_READ);
	t.rxlength = t.length;
	t.tx_buffer = &b0;
	t.rx_buffer = rx_buf;

	ret = spi_device_polling_transmit(spi, &t);
	assert(ret == ESP_OK);

	// Release bus
	spi_device_release_bus(spi);

	/* Convert received 24-bit values to output data */
	for (int i = 0; i < MCP3913_NUM_CHANNELS; i++) {
		d[i] = 0;
		for (int j = 0; j < MCP3913_SPI_BYTES_PER_READ; j++) {
			d[i] <<= 8;
			d[i] |= (uint32_t)rx_buf[1 +
			    i * MCP3913_SPI_BYTES_PER_READ + j];
		}
		if (d[i] & 0x800000UL) {
			d[i] |= 0xFF000000UL;
		}
	}

	return (ret);
}
/*
 * Write register.
 */
int
mcp3913_write(uint8_t reg, uint32_t data)
{
	esp_err_t ret;
	spi_transaction_t t;
	uint8_t b[4];

	b[0] = MCP3913_SPI_ADDR | (reg << 1) | MCP3913_SPI_WRITE;

	for (int i = 0; i <= 2; i++) {
		/* SPI sends LSB last */
		b[3 - i] = data & 0xff;
		data >>= 8;
	}

	// When using SPI_TRANS_CS_KEEP_ACTIVE, bus must be locked/acquired
	spi_device_acquire_bus(spi, portMAX_DELAY);

	memset(&t, 0, sizeof (t));

	/* One byte of address plus three bytes of data */
	t.length = 8 * 4;
	t.tx_buffer = b;
	// Keep CS active after data transfer
	// t.flags = SPI_TRANS_CS_KEEP_ACTIVE;

	ret = spi_device_polling_transmit(spi, &t);
	assert(ret == ESP_OK);

	// Release bus
	spi_device_release_bus(spi);

	return (ret);
}

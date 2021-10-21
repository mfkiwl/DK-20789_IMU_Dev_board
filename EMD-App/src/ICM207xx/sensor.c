/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2017 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */
#include <asf.h>

/* InvenSense drivers and utils */
#include "Invn/Devices/Drivers/Icm207xx/Icm207xx.h"
#include "Invpres.h"
#include "Invn/Devices/SensorTypes.h"
#include "Invn/Devices/SensorConfig.h"
#include "Invn/EmbUtils/InvScheduler.h"
#include "Invn/EmbUtils/RingByteBuffer.h"
#include "Invn/EmbUtils/Message.h"
#include "Invn/EmbUtils/ErrorHelper.h"
#include "Invn/EmbUtils/DataConverter.h"
#include "Invn/EmbUtils/RingBuffer.h"
#include "Invn/DynamicProtocol/DynProtocol.h"
#include "Invn/DynamicProtocol/DynProtocolTransportUart.h"

/* InvenSense LibExport */
#include "algo_eapi.h"

#include "ASF/sam/drivers/pio/pio.h"
#include "ASF/sam/drivers/pio/pio_handler.h"
#include "ASF/sam/drivers/twi/twi.h"
#include "ASF/sam/drivers/tc/tc.h"

#include "main.h"
#include "system.h"
#include "sensor.h"
#include "fw_version.h"

/*
 * Just a handy variable to handle the icm207xx object
 */
static const uint8_t EXPECTED_WHOAMI[] = { 0x95, 0x02, 0x03};  /* WHOAMI value for ICM207xx or derivative */

/* FSR configurations */
int32_t cfg_acc_fsr = 4000; // Default = +/- 4g. Valid ranges: 2000, 4000, 8000, 16000
int32_t cfg_gyr_fsr = 2000; // Default = +/- 2000dps. Valid ranges: 250, 500, 1000, 2000

static sensor_odr_t sensor_odr_range[SENSOR_MAX];

inv_icm207xx_t icm_device;
inv_invpres_t invpres_device;

/*
 * Mask to keep track of enabled sensors
 */
uint32_t enabled_sensor_mask = 0;

/* Forward declaration */
static void apply_mouting_matrix(const int32_t mounting_matrix[9], const int16_t raw[3], int32_t out[3]);
static int icm207xx_run_selftest(void);
static void convert_sensor_event_to_dyn_prot_data(const inv_sensor_event_t * event, VSensorDataAny * vsensor_data);
static void check_rc(int rc, const char * context_str);
static void store_offsets(const int32_t acc_bias_q16[3], const int32_t gyr_bias_q16[3], const int32_t mag_bias_q16[3]);
static int handle_command(enum DynProtocolEid eid, const DynProtocolEdata_t * edata, DynProtocolEdata_t * respdata);
static void notify_event(uint64_t timestamp);

void sensor_event(const inv_sensor_event_t * event, void * arg);

/*
 * Variable to keep track of the expected period common for all sensors
 */
uint32_t period_us = DEFAULT_ODR_US /* 50Hz by default */;

static uint32_t pressure_ms = DEFAULT_PRESSURE_ODR /* 40Hz by default */;

/*
 * Variable to drop the first timestamp(s) after a sensor start cached by the interrupt line.
 * This is needed to be inline with the driver. The first data polled from the FIFO is always discard.
 */
static uint8_t timestamp_to_drop = 0;

/*
 * Variable keeping track of chip information
 */
uint8_t chip_info[3];

/*
 * Variable keeping track of gyro-assisted calibration support
 */
static uint8_t hmd_features_supported = 0;

/* Hold all algorithm inputs, given to the algo encapsulation API */
static algo_input inputs;

/* Hold all algorithm outputs, given to the algo encapsulation API */
algo_output outputs;

/* Keep a copy of the estimated biases loaded from FLASH in RAM */
static int32_t last_loaded_acc_bias[3] = {0};
static int32_t last_loaded_gyr_bias[3] = {0};
static int32_t last_loaded_mag_bias[3] = {0};

/* 
 * Mounting matrix configuration applied for both Accel and Gyro 
 * The coefficient values are coded in integer q30
 */
static int32_t cfg_mounting_matrix[9]= { 1.f*(1<<30), 0,           0,
                                         0,           1.f*(1<<30), 0,
                                         0,           0,           1.f*(1<<30) };

/*
 * Dynamic protocol and transport handles
 */
DynProtocol_t protocol;
DynProTransportUart_t transport;

static void init_sensor_odr(void)
{
	uint8_t	ii;
	
	for (ii = 0; ii < SENSOR_MAX; ii++) {
		switch (ii) {
			case SENSOR_GRV:
			case SENSOR_PREDQUAT_0:
			case SENSOR_PREDQUAT_1:
				sensor_odr_range[ii].min = 20000;
				sensor_odr_range[ii].max = 5000;
			break;
			case SENSOR_CUSTOM_PRESSURE:
				sensor_odr_range[ii].min = 1000000;
				sensor_odr_range[ii].max = 25000;
			break;
			case SENSOR_RAW_ACC:
			case SENSOR_RAW_GYR:
			case SENSOR_ACC:
			case SENSOR_GYR:
			case SENSOR_UGYR:
			case SENSOR_GRA:
			case SENSOR_LINACC:
			default:
				sensor_odr_range[ii].min = 250000;
				sensor_odr_range[ii].max = 5000;
			break;
		}
	}
}

static void get_sensor_odr_min_max(uint32_t *min_odr, uint32_t *max_odr, uint32_t sensor_mask)
{
	uint8_t	ii;
	
	*min_odr = 0x0;
	*max_odr = 0xFFFFFFFF;

	for (ii = 0; ii < SENSOR_MAX; ii++) {
		if (sensor_mask & (1 << ii)) {
			if (sensor_odr_range[ii].min > *min_odr)
			*min_odr = sensor_odr_range[ii].min;
			if (sensor_odr_range[ii].max < *max_odr)
			*max_odr = sensor_odr_range[ii].max;
		}
	}
	if (*min_odr == 0x0)
		*min_odr = DEFAULT_ODR_US;
	if (*max_odr == 0xFFFFFFFF)
		*max_odr = DEFAULT_ODR_US;
}

void InvEMDFrontEnd_busyWaitUsHook(uint32_t us)
{
	delay_us(us);
}

int InvEMDFrontEnd_isHwFlowCtrlSupportedHook(void)
{
	return 0;
}

int InvEMDFrontEnd_putcharHook(int c)
{
	if(usart_serial_putchar(CONF_UART, (uint8_t)c))
		return c;
	else
		return -1;
}

/*
 * Sleep implementation for ICM207xx
 */
void inv_icm207xx_sleep(int ms)
{
	delay_ms(ms);
}

void inv_icm207xx_sleep_us(int us)
{
	delay_us(us);
}

int icm207xx_sensor_setup(void)
{
	int rc;
	uint8_t i, whoami = 0xff;

	init_sensor_odr();
	/*
	 * Just get the whoami
	 */
	rc = inv_icm207xx_get_whoami(&icm_device, &whoami);
#if !USE_SPI_NOT_I2C						// If we're using I2C
	if (whoami == 0xff) {					// if whoami fails try the other I2C Address
		I2C_Address = ICM_I2C_ADDR_REVA;
		rc = inv_icm207xx_get_whoami(&icm_device, &whoami);
	}
#endif
	INV_MSG(INV_MSG_LEVEL_INFO, "ICM207xx WHOAMI=0x%02x", whoami);
	check_rc(rc, "Error reading WHOAMI");

	/*
	 * Check if WHOAMI value corresponds to any value from EXPECTED_WHOAMI array
	 */
	for(i = 0; i < sizeof(EXPECTED_WHOAMI)/sizeof(EXPECTED_WHOAMI[0]); ++i) {
		if(whoami == EXPECTED_WHOAMI[i])
			break;
	}

	if(i == sizeof(EXPECTED_WHOAMI)/sizeof(EXPECTED_WHOAMI[0])) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Bad WHOAMI value. Got 0x%02x. Expected 0x17.", whoami);
		check_rc(-1, "");
	}
	rc = inv_icm207xx_get_chip_info(&icm_device, chip_info);
	check_rc(rc, "Could not obtain chip info");

	/*
	 * Configure and initialize the ICM207xx for normal use
	 */
	INV_MSG(INV_MSG_LEVEL_INFO, "Booting up icm207xx...");

	/* set default power mode */
	if (!inv_icm207xx_is_sensor_enabled(&icm_device, INV_ICM207XX_SENSOR_GYRO) &&
		!inv_icm207xx_is_sensor_enabled(&icm_device, INV_ICM207XX_SENSOR_ACCEL)) {
		INV_MSG(INV_MSG_LEVEL_VERBOSE, "Putting icm207xx in sleep mode...");
		rc = inv_icm207xx_initialize(&icm_device);
		check_rc(rc, "Error %d while setting-up icm207xx device");
	}

	/* set default ODR = 50Hz */
	rc = inv_icm207xx_set_sensor_period(&icm_device, INV_ICM207XX_SENSOR_ACCEL, DEFAULT_ODR_US/1000 /*ms*/);
	check_rc(rc, "Error %d while setting-up icm207xx device");

	rc = inv_icm207xx_set_sensor_period(&icm_device, INV_ICM207XX_SENSOR_GYRO, DEFAULT_ODR_US/1000 /*ms*/);
	check_rc(rc, "Error %d while setting-up icm207xx device");

	period_us = DEFAULT_ODR_US;

	/* we should be good to go ! */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "We're good to go !");

	return 0;
}

int icm207xx_sensor_configuration(void)
{
	int rc;

	INV_MSG(INV_MSG_LEVEL_INFO, "Configuring accelerometer FSR");
	rc = inv_icm207xx_set_accel_fullscale(&icm_device, inv_icm207xx_accel_fsr_2_reg(cfg_acc_fsr));
	check_rc(rc, "Error configuring ACC sensor");

	INV_MSG(INV_MSG_LEVEL_INFO, "Configuring gyroscope FSR");
	rc = inv_icm207xx_set_gyro_fullscale(&icm_device, inv_icm207xx_gyro_fsr_2_reg(cfg_gyr_fsr));
	check_rc(rc, "Error configuring GYR sensor");

	return rc;
}

/*
 * Helper function to check RC value and block program execution
 */
static void check_rc(int rc, const char * msg_context)
{
	if(rc < 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "%s: error %d (%s)", msg_context, rc, inv_error_str(rc));
		while(1);
	}
}

/*
 * Send response event to host to signal setup is finished
 */
int InvEMDFrontEnd_acknowledgeReset(void)
{
	static DynProtocolEdata_t respEdata; /* static to take on .bss */
	static uint8_t respBuffer[64]; /* static to take on .bss */
	uint16_t respLen;

	respEdata.d.response.rc = 0;
	INV_MSG(INV_MSG_LEVEL_DEBUG, "InvEMDFrontEnd: Acknowledge reset now");

	DynProtocol_encodeResponse(&protocol, DYN_PROTOCOL_EID_RESET, &respEdata,
			respBuffer, sizeof(respBuffer), &respLen);
			
	DynProTransportUart_tx(&transport, respBuffer, respLen);
	
	return INV_ERROR_SUCCESS;
}

/*
 * IddWrapper protocol handler function
 *
 * Will dispatch command and send response back
 */
void iddwrapper_protocol_event_cb(
	enum DynProtocolEtype etype,
	enum DynProtocolEid eid,
	const DynProtocolEdata_t * edata,
	void * cookie
)
{
	(void)cookie;

	static DynProtocolEdata_t resp_edata; /* static to take on .bss */
	static uint8_t respBuffer[256]; /* static to take on .bss */
	uint16_t respLen;
	
	switch(etype) {
		case DYN_PROTOCOL_ETYPE_CMD:
			resp_edata.d.response.rc = handle_command(eid, edata, &resp_edata);

			/* send back response */		
			if(DynProtocol_encodeResponse(&protocol, eid, &resp_edata,
				respBuffer, sizeof(respBuffer), &respLen) != 0) {
				goto error_dma_buffer;
			}
					
			DynProTransportUart_tx(&transport, respBuffer, respLen);
			break;

		default:
			INV_MSG(INV_MSG_LEVEL_WARNING, "DeviceEmdWrapper: unexpected packet received. Ignored.");
			break; /* no suppose to happen */
	}
	return;
	
error_dma_buffer:
	INV_MSG(INV_MSG_LEVEL_WARNING, "iddwrapper_protocol_event_cb: encode error, response dropped");

	return;
}

/*
 * IddWrapper transport handler function
 *
 * This function will:
 *  - feed the Dynamic protocol layer to analyze for incoming CMD packet
 *  - forward byte coming from transport layer to be send over uart to the host
 */
void iddwrapper_transport_event_cb(enum DynProTransportEvent e,
	union DynProTransportEventData data, void * cookie)
{
	(void)cookie;

	int rc;
	int timeout = 5000; /* us */

	switch(e) {
		case DYN_PRO_TRANSPORT_EVENT_ERROR:
			INV_MSG(INV_MSG_LEVEL_ERROR, "ERROR event with value %d received from IddWrapper transport", data.error);
			break;
		
		case DYN_PRO_TRANSPORT_EVENT_PKT_SIZE:
			break;
		
		case DYN_PRO_TRANSPORT_EVENT_PKT_BYTE:
			/* Feed IddWrapperProtocol to look for packet */
			rc = DynProtocol_processPktByte(&protocol, data.pkt_byte);
			if(rc < 0) {
				INV_MSG(INV_MSG_LEVEL_DEBUG, "DynProtocol_processPktByte(%02x) returned %d", data.pkt_byte, rc);
			}
			break;
		
		case DYN_PRO_TRANSPORT_EVENT_PKT_END:
			break;

		/* forward buffer from EMD Transport, to the SERIAL */
		case DYN_PRO_TRANSPORT_EVENT_TX_START:
			break;
		
		case DYN_PRO_TRANSPORT_EVENT_TX_BYTE:
			while ((InvEMDFrontEnd_putcharHook(data.tx_byte) == EOF) && (timeout > 0)) {
				InvEMDFrontEnd_busyWaitUsHook(10);
				timeout -= 10;
			}
			break;
		
		case DYN_PRO_TRANSPORT_EVENT_TX_END:
			break;
			
		case DYN_PRO_TRANSPORT_EVENT_TX_START_DMA:
			break;	
	}
}

int sensor_configure_odr(uint32_t odr_us, uint32_t sensor_mask)
{
	int rc = 0;

	uint32_t min_odr, max_odr;

	/* All sensors running at the same rate */
	
	get_sensor_odr_min_max(&min_odr, &max_odr, sensor_mask);
	/*
	 * Maximum supported rate is 200 Hz
	 */
	if(odr_us > min_odr)
		odr_us = min_odr;

	if(odr_us < max_odr)
		odr_us = max_odr;

	/*
	 *  Call driver APIs to start/stop sensors
	 */
	rc = inv_icm207xx_set_sensor_period(&icm_device, INV_ICM207XX_SENSOR_ACCEL, odr_us / 1000);
	rc += inv_icm207xx_set_sensor_period(&icm_device, INV_ICM207XX_SENSOR_GYRO, odr_us / 1000);
	
	/* FIFO has been reset by ODR change */
	if (rc == 0) {
		pio_clear(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_MASK);
	
		/* Clear any remaining interrupts */
		__disable_irq();
		irq_from_device = 0;
		__enable_irq();
	}
	
	/* Keep track in static variable of the odr value for further algorithm use */
	period_us = odr_us;

	return rc;
}

int sensor_control(int enable)
{
	int rc = 0;
	static uint8_t sensors_on = 0;

	/* Keep track of the sensors state */
	if(enable && sensors_on)
		return rc;

	if(enable)
		sensors_on = 1;
	else 
		sensors_on = 0;

	/*
	 *  Call driver APIs to start/stop sensors
	 */
	if (enable) {
		/* Clock is more accurate when gyro is enabled, so let's enable it first to prevent side effect at startup */
		if (!inv_icm207xx_is_sensor_enabled(&icm_device, INV_ICM207XX_SENSOR_GYRO))
			rc += inv_icm207xx_enable_sensor(&icm_device, INV_ICM207XX_SENSOR_GYRO, 1);
		if (!inv_icm207xx_is_sensor_enabled(&icm_device, INV_ICM207XX_SENSOR_ACCEL))
			rc += inv_icm207xx_enable_sensor(&icm_device, INV_ICM207XX_SENSOR_ACCEL, 1);
		if (!inv_icm207xx_is_sensor_enabled(&icm_device, INV_ICM207XX_SENSOR_TEMPERATURE))
			rc += inv_icm207xx_enable_sensor(&icm_device, INV_ICM207XX_SENSOR_TEMPERATURE, 1);
		/*
		 * There is a situation where two samples need to be dropped: if
		 * accelerometer is enable before gyroscope first interrupt triggers,
		 * both interrupts are raised causing the odr to be wrong if only one
		 * sample is dropped.
		 * We are in this exact situation since both sensors are enabled one after
		 * the other.
		 */
		timestamp_to_drop = 2;
	} else {
		rc += inv_icm207xx_enable_sensor(&icm_device, INV_ICM207XX_SENSOR_GYRO, 0);
		rc += inv_icm207xx_enable_sensor(&icm_device, INV_ICM207XX_SENSOR_ACCEL, 0);
		rc += inv_icm207xx_enable_sensor(&icm_device, INV_ICM207XX_SENSOR_TEMPERATURE, 0);
	}

	/* Clear the remaining items in the IRQ timestamp buffer when stopping all sensors */
	if(inv_icm207xx_all_sensors_off(&icm_device)) 
		pio_clear(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_MASK);
	

	return rc;
}

/*
* Poll devices for data
*/
int Icm207xx_data_poll(void) {
	int rc = 0;
	uint8_t ddry = 0;
	uint8_t int_status;
	int16_t raw_acc[3], raw_gyro[3];
	int8_t unused_accuracy_flag;

	/*
	 *  Ensure data ready status
	 */
	if((rc = inv_icm207xx_get_int_status(&icm_device, &int_status)) == 0)
		ddry = inv_icm207xx_check_drdy(&icm_device, int_status);

	if(ddry) {
		struct inv_icm207xx_fifo_states fifo_states;

		rc = inv_icm207xx_poll_fifo_data_setup(&icm_device, &fifo_states, int_status);
		check_rc(rc, "Error %d while polling the icm207xx device");
		if(rc == 1) {
			/* 
			 * Overflow detected
			 */
			INV_MSG(INV_MSG_LEVEL_WARNING, "FIFO overflow detected!");
			inv_icm207xx_reset_fifo(&icm_device);
			pio_clear(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_MASK);		
		}
		else if(fifo_states.packet_count > 0 && fifo_states.packet_size > 0) {
			
			/*
			 * Read FIFO only when data is expected in FIFO 
			 */
			while((rc = inv_icm207xx_poll_fifo_data(&icm_device, &fifo_states, raw_acc, &inputs.sRtemp_data, raw_gyro, NULL)) > 0) {

				uint64_t timestamp = InvEMDFrontEnd_getTimestampUs();
				/* 
				 * Drop the first timestamp(s) caught by the interrupt 
				 * because the first data in FIFO is always dropped by
				 * the icm207xx driver. 6-axis fusion needs two 
				 * samples to be dropped.
				 */
				while (timestamp_to_drop > 0) {
					timestamp = InvEMDFrontEnd_getTimestampUs();
					timestamp_to_drop--;
				}

				/*
				 * Apply the mounting matrix configuration to the data polled
				 */
				apply_mouting_matrix(cfg_mounting_matrix, raw_acc, inputs.sRacc_data);
				apply_mouting_matrix(cfg_mounting_matrix, raw_gyro, inputs.sRgyro_data);

				/*
				 * Compute calibration and orientation algorithms
				 */
				unused_accuracy_flag = outputs.mag_accuracy_flag;
				outputs.acc_accuracy_flag = 0;
				outputs.gyr_accuracy_flag = 0;
				outputs.mag_accuracy_flag = -1;
				algorithms_process(&inputs, &outputs);
				outputs.mag_accuracy_flag = unused_accuracy_flag;

				/* 
				 * Notify upon new sensor data event
				 */
				notify_event(timestamp);
			}
		}
	}

	__disable_irq();
	irq_from_device = 0;
	__enable_irq();
	
	return rc;
}

static void apply_mouting_matrix(const int32_t mounting_matrix[9], const int16_t raw[3], int32_t out[3])
{
	unsigned i;

	for(i = 0; i < 3; i++) {
		out[i]  = (int32_t)((int64_t)mounting_matrix[3*i+0]*raw[0] >> 30);
		out[i] += (int32_t)((int64_t)mounting_matrix[3*i+1]*raw[1] >> 30);
		out[i] += (int32_t)((int64_t)mounting_matrix[3*i+2]*raw[2] >> 30);
	}
}

void notify_event(uint64_t timestamp)
{
	inv_sensor_event_t event;
	memset(&event, 0, sizeof(event));

	/* 
	 * New raw accel data 
	 */
	if(enabled_sensor_mask & (1 << SENSOR_RAW_ACC)) {
		event.sensor	= INV_SENSOR_TYPE_RAW_ACCELEROMETER;
		event.timestamp = timestamp;
		event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
		event.data.raw3d.vect[0] = inputs.sRacc_data[0];
		event.data.raw3d.vect[1] = inputs.sRacc_data[1];
		event.data.raw3d.vect[2] = inputs.sRacc_data[2];

		sensor_event(&event, NULL);
	}

	/* 
	 * New calibrated accel event
	 */
	if(enabled_sensor_mask & (1 << SENSOR_ACC)) {
		event.sensor	= INV_SENSOR_TYPE_ACCELEROMETER;
		event.timestamp = timestamp;
		event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
		event.data.acc.bias[0] = (float) outputs.acc_bias_q16[0] / (1 << 16);
		event.data.acc.bias[1] = (float) outputs.acc_bias_q16[1] / (1 << 16);
		event.data.acc.bias[2] = (float) outputs.acc_bias_q16[2] / (1 << 16);
		event.data.acc.vect[0] = (float) outputs.acc_cal_q16[0] / (1 << 16);
		event.data.acc.vect[1] = (float) outputs.acc_cal_q16[1] / (1 << 16);
		event.data.acc.vect[2] = (float) outputs.acc_cal_q16[2] / (1 << 16);
		event.data.acc.accuracy_flag = outputs.acc_accuracy_flag & DATA_ACCURACY_MASK;

		sensor_event(&event, NULL);
	}

	/* 
	 * New raw gyro data 
	 */
	if(enabled_sensor_mask & (1 << SENSOR_RAW_GYR)) {
		event.sensor	= INV_SENSOR_TYPE_RAW_GYROSCOPE;
		event.timestamp = timestamp;
		event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
		event.data.raw3d.vect[0] = inputs.sRgyro_data[0];
		event.data.raw3d.vect[1] = inputs.sRgyro_data[1];
		event.data.raw3d.vect[2] = inputs.sRgyro_data[2];

		sensor_event(&event, NULL);
	}

	/* 
	 * New uncalibrated gyro event
	 */
	if(enabled_sensor_mask & (1 << SENSOR_UGYR)) {
		event.sensor	= INV_SENSOR_TYPE_UNCAL_GYROSCOPE;
		event.timestamp = timestamp;
		event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
		event.data.gyr.bias[0] = (float) outputs.gyr_bias_q16[0] / (1 << 16);
		event.data.gyr.bias[1] = (float) outputs.gyr_bias_q16[1] / (1 << 16);
		event.data.gyr.bias[2] = (float) outputs.gyr_bias_q16[2] / (1 << 16);
		event.data.gyr.vect[0] = (float) outputs.gyr_uncal_q16[0] / (1 << 16);
		event.data.gyr.vect[1] = (float) outputs.gyr_uncal_q16[1] / (1 << 16);
		event.data.gyr.vect[2] = (float) outputs.gyr_uncal_q16[2] / (1 << 16);
		event.data.gyr.accuracy_flag = outputs.gyr_accuracy_flag & DATA_ACCURACY_MASK;

		sensor_event(&event, NULL);
	}

	/* 
	 * New calibrated gyro event
	 */
	if(enabled_sensor_mask & (1 << SENSOR_GYR)) {
		event.sensor	= INV_SENSOR_TYPE_GYROSCOPE;
		event.timestamp = timestamp;
		event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
		event.data.gyr.bias[0] = (float) outputs.gyr_bias_q16[0] / (1 << 16);
		event.data.gyr.bias[1] = (float) outputs.gyr_bias_q16[1] / (1 << 16);
		event.data.gyr.bias[2] = (float) outputs.gyr_bias_q16[2] / (1 << 16);
		event.data.gyr.vect[0] = (float) outputs.gyr_cal_q16[0] / (1 << 16);
		event.data.gyr.vect[1] = (float) outputs.gyr_cal_q16[1] / (1 << 16);
		event.data.gyr.vect[2] = (float) outputs.gyr_cal_q16[2] / (1 << 16);
		event.data.gyr.accuracy_flag = outputs.gyr_accuracy_flag & DATA_ACCURACY_MASK;

		sensor_event(&event, NULL);
	}

	/* 
	 * New GRV event 
	 * scheduled on gyroscope data update
	 */
	if(enabled_sensor_mask & (1 << SENSOR_GRV)) {
		event.sensor	= INV_SENSOR_TYPE_GAME_ROTATION_VECTOR;
		event.timestamp = timestamp;
		event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
		event.data.quaternion.quat[0] = (float)outputs.grv_quat_q30[0] / (1 << 30);
		event.data.quaternion.quat[1] = (float)outputs.grv_quat_q30[1] / (1 << 30);
		event.data.quaternion.quat[2] = (float)outputs.grv_quat_q30[2] / (1 << 30);
		event.data.quaternion.quat[3] = (float)outputs.grv_quat_q30[3] / (1 << 30);
		/* Report additional accuracy flag, being currently a copy of GYR accuracy flag (ACC flag could also be considered) */
		event.data.quaternion.accuracy_flag = outputs.gyr_accuracy_flag & DATA_ACCURACY_MASK;

		sensor_event(&event, NULL);
	}
	
	/* 
	 * New Predictive quaternion event from instance 0
	 * scheduled on gyroscope data update
	 */
	if((enabled_sensor_mask & (1 << SENSOR_PREDQUAT_0)) && hmd_features_supported) {
		event.sensor	= INV_SENSOR_TYPE_PRED_QUAT_0;
		event.timestamp = timestamp;
		event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
		event.data.quaternion.quat[0] = (float)outputs.predgrv0_quat_q30[0] / (1 << 30);
		event.data.quaternion.quat[1] = (float)outputs.predgrv0_quat_q30[1] / (1 << 30);
		event.data.quaternion.quat[2] = (float)outputs.predgrv0_quat_q30[2] / (1 << 30);
		event.data.quaternion.quat[3] = (float)outputs.predgrv0_quat_q30[3] / (1 << 30);
		/* Report additional accuracy flag, being currently a copy of GYR accuracy flag (ACC flag could also be considered) */
		event.data.quaternion.accuracy_flag = outputs.gyr_accuracy_flag & DATA_ACCURACY_MASK;

		sensor_event(&event, NULL);
	}

	/* 
	 * New Predictive quaternion event from instance 1
	 * scheduled on gyroscope data update
	 */
	if((enabled_sensor_mask & (1 << SENSOR_PREDQUAT_1)) && hmd_features_supported) {
		event.sensor	= INV_SENSOR_TYPE_PRED_QUAT_1;
		event.timestamp = timestamp;
		event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
		event.data.quaternion.quat[0] = (float)outputs.predgrv1_quat_q30[0] / (1 << 30);
		event.data.quaternion.quat[1] = (float)outputs.predgrv1_quat_q30[1] / (1 << 30);
		event.data.quaternion.quat[2] = (float)outputs.predgrv1_quat_q30[2] / (1 << 30);
		event.data.quaternion.quat[3] = (float)outputs.predgrv1_quat_q30[3] / (1 << 30);
		/* Report additional accuracy flag, being currently a copy of GYR accuracy flag (ACC flag could also be considered) */
		event.data.quaternion.accuracy_flag = outputs.gyr_accuracy_flag & DATA_ACCURACY_MASK;

		sensor_event(&event, NULL);
	}
	
	/* 
	 * New gravity event 
	 */
	if(enabled_sensor_mask & (1 << SENSOR_GRA)) {
		event.sensor	= INV_SENSOR_TYPE_GRAVITY;
		event.timestamp = timestamp;
		event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
		event.data.acc.vect[0] = (float)outputs.gravity_q16[0] / (1 << 16);
		event.data.acc.vect[1] = (float)outputs.gravity_q16[1] / (1 << 16);
		event.data.acc.vect[2] = (float)outputs.gravity_q16[2] / (1 << 16);
		/* Report additional accuracy flag, being currently the copy of GRV accuracy flag (copied from GYR accuracy flag) */
		event.data.acc.accuracy_flag = outputs.gyr_accuracy_flag & DATA_ACCURACY_MASK;

		sensor_event(&event, NULL);
	}

	/* 
	 * New linear acceleration event 
	 */
	if(enabled_sensor_mask & (1 << SENSOR_LINACC)) {
		event.sensor	= INV_SENSOR_TYPE_LINEAR_ACCELERATION;
		event.timestamp = timestamp;
		event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
		event.data.acc.vect[0] = (float)outputs.linearacc_q16[0] / (1 << 16);
		event.data.acc.vect[1] = (float)outputs.linearacc_q16[1] / (1 << 16);
		event.data.acc.vect[2] = (float)outputs.linearacc_q16[2] / (1 << 16);
		/* Report additional accuracy flag, being currently the copy of GRV accuracy flag (copied from GYR accuracy flag) */
		event.data.acc.accuracy_flag = outputs.gyr_accuracy_flag & DATA_ACCURACY_MASK;

		sensor_event(&event, NULL);
	}
}

void sensor_event(const inv_sensor_event_t * event, void * arg)
{
	/* arg will contained the value provided at init time */
	(void)arg;

	/*
	 * Encode sensor event and sent to host over UART through IddWrapper protocol
	 */
	static DynProtocolEdata_t async_edata; /* static to take on .bss */
	static uint8_t async_buffer[256]; /* static to take on .bss */
	uint16_t async_bufferLen;
	
	async_edata.sensor_id = event->sensor;
	async_edata.d.async.sensorEvent.status = DYN_PRO_SENSOR_STATUS_DATA_UPDATED;
	convert_sensor_event_to_dyn_prot_data(event, &async_edata.d.async.sensorEvent.vdata);

	if(DynProtocol_encodeAsync(&protocol,
			DYN_PROTOCOL_EID_NEW_SENSOR_DATA, &async_edata,
			async_buffer, sizeof(async_buffer), &async_bufferLen) != 0) {
		goto error_dma_buf;
	}
	
	DynProTransportUart_tx(&transport, async_buffer, async_bufferLen);
	return;

error_dma_buf:
	INV_MSG(INV_MSG_LEVEL_WARNING, "sensor_event_cb: encode error, frame dropped");
	
	return;
}

/*
 * Convert sensor_event to VSensorData because dynamic protocol transports VSensorData
 */
static void convert_sensor_event_to_dyn_prot_data(const inv_sensor_event_t * event, VSensorDataAny * vsensor_data)
{
	vsensor_data->base.timestamp = event->timestamp;

	switch(event->sensor) {
	case DYN_PRO_SENSOR_TYPE_RESERVED:
		break;
	case DYN_PRO_SENSOR_TYPE_GRAVITY:
	case DYN_PRO_SENSOR_TYPE_LINEAR_ACCELERATION:
	case DYN_PRO_SENSOR_TYPE_ACCELEROMETER:
		inv_dc_float_to_sfix32(&event->data.acc.vect[0], 3, 16, (int32_t *)&vsensor_data->data.u32[0]);
		vsensor_data->base.meta_data = event->data.acc.accuracy_flag;
		break;
	case DYN_PRO_SENSOR_TYPE_GYROSCOPE:
		inv_dc_float_to_sfix32(&event->data.gyr.vect[0], 3, 16, (int32_t *)&vsensor_data->data.u32[0]);
		vsensor_data->base.meta_data = event->data.gyr.accuracy_flag;
		break;
	case DYN_PRO_SENSOR_TYPE_UNCAL_GYROSCOPE:
		inv_dc_float_to_sfix32(&event->data.gyr.vect[0], 3, 16, (int32_t *)&vsensor_data->data.u32[0]);
		inv_dc_float_to_sfix32(&event->data.gyr.bias[0], 3, 16, (int32_t *)&vsensor_data->data.u32[3]);
		vsensor_data->base.meta_data = event->data.gyr.accuracy_flag;
		break;
	case DYN_PRO_SENSOR_TYPE_PRED_QUAT_0:
	case DYN_PRO_SENSOR_TYPE_PRED_QUAT_1:
	case DYN_PRO_SENSOR_TYPE_GAME_ROTATION_VECTOR:
		inv_dc_float_to_sfix32(&event->data.quaternion.quat[0], 4, 30, (int32_t *)&vsensor_data->data.u32[0]);
		vsensor_data->base.meta_data = event->data.quaternion.accuracy_flag;
		break;
	case DYN_PRO_SENSOR_TYPE_RAW_ACCELEROMETER:
	case DYN_PRO_SENSOR_TYPE_RAW_GYROSCOPE:
		vsensor_data->data.u32[0] = event->data.raw3d.vect[0];
		vsensor_data->data.u32[1] = event->data.raw3d.vect[1];
		vsensor_data->data.u32[2] = event->data.raw3d.vect[2];
		break;
	case DYN_PRO_SENSOR_TYPE_CUSTOM_PRESSURE:
		// raw pressure
		vsensor_data->data.u32[0] = event->data.custom_pressure.raw_pressure;
		// pressure
		inv_dc_float_to_sfix32(&event->data.custom_pressure.pressure, 1, 8, (int32_t *)&vsensor_data->data.u32[1]);
		// raw temperature
		vsensor_data->data.u32[2] = event->data.custom_pressure.raw_temperature;
		// temperature
		inv_dc_float_to_sfix32(&event->data.custom_pressure.temperature, 1, 8, (int32_t *)&vsensor_data->data.u32[3]);
		break;
	default:
		break;
	}
}

enum sensor idd_sensortype_conversion(int sensor)
{
	switch(sensor) {
		case INV_SENSOR_TYPE_RAW_ACCELEROMETER:      return SENSOR_RAW_ACC;
		case INV_SENSOR_TYPE_RAW_GYROSCOPE:          return SENSOR_RAW_GYR;
		case INV_SENSOR_TYPE_ACCELEROMETER:          return SENSOR_ACC;
		case INV_SENSOR_TYPE_GYROSCOPE:              return SENSOR_GYR;
		case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:        return SENSOR_UGYR;
		case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:   return SENSOR_GRV;
		case INV_SENSOR_TYPE_PRED_QUAT_0:            return hmd_features_supported ? SENSOR_PREDQUAT_0: SENSOR_MAX;
		case INV_SENSOR_TYPE_PRED_QUAT_1:            return hmd_features_supported ? SENSOR_PREDQUAT_1: SENSOR_MAX;
		case INV_SENSOR_TYPE_GRAVITY:                return SENSOR_GRA;
		case INV_SENSOR_TYPE_LINEAR_ACCELERATION:    return SENSOR_LINACC;
		case INV_SENSOR_TYPE_CUSTOM_PRESSURE:		 return SENSOR_CUSTOM_PRESSURE;
		default:                                     return SENSOR_MAX;
	}
}

static int handle_command(enum DynProtocolEid eid, const DynProtocolEdata_t * edata, DynProtocolEdata_t * respdata)
{
	int rc = 0;
	int i;
	uint8_t whoami;
	const int sensor = edata->sensor_id;

	switch(eid) {

		case DYN_PROTOCOL_EID_GET_SW_REG:
		if(edata->d.command.regAddr == DYN_PROTOCOL_EREG_HANDSHAKE_SUPPORT)
		return InvEMDFrontEnd_isHwFlowCtrlSupportedHook();
		return 0;

		case DYN_PROTOCOL_EID_SETUP:
		INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command setup");
		/* Re-init the device */
		rc = icm207xx_sensor_setup();
		rc += icm207xx_sensor_configuration();

		/* Re-init algorithms */
		algorithms_init(outputs.acc_bias_q16, outputs.gyr_bias_fnm_q16, outputs.mag_bias_q16, chip_info);
		sensor_configure_odr(period_us, 0);

		algorithms_configure_odr(period_us, 0);

		/* Enable all sensors but.. */
		algorithms_sensor_control(1);
		rc += sensor_control(1);
		/* .. no sensors are reporting on setup */
		enabled_sensor_mask = 0;
		return rc;

		case DYN_PROTOCOL_EID_WHO_AM_I:
		rc = inv_icm207xx_get_whoami(&icm_device, &whoami);
		return (rc == 0) ? whoami : rc;

		case DYN_PROTOCOL_EID_RESET:
		INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command reset");
		/* --- Cleanup --- */
		algorithms_sensor_control(0);
		rc = sensor_control(0);
		/* Store the most reliable biases (either previous with accuracy of 3, or new offsets with accuracy 3) */
		store_offsets(
		(outputs.acc_accuracy_flag == 3) ? outputs.acc_bias_q16 : last_loaded_acc_bias,
		(outputs.gyr_accuracy_flag == 3) ? outputs.gyr_bias_fnm_q16 : last_loaded_gyr_bias,
		(outputs.mag_accuracy_flag == 3) ? outputs.mag_bias_q16 : last_loaded_mag_bias);
		/* Soft reset */
		rc += inv_icm207xx_soft_reset(&icm_device);
		/* --- Setup --- */
		/* Re-init the device */
		rc += icm207xx_sensor_setup();
		rc += icm207xx_sensor_configuration();
		/* Re-init algorithms */
		algorithms_init(outputs.acc_bias_q16, outputs.gyr_bias_fnm_q16, outputs.mag_bias_q16, chip_info);
		sensor_configure_odr(period_us, 0);
		
		algorithms_configure_odr(period_us, 0);

		/* Enable all sensor but.. */
		algorithms_sensor_control(1);
		rc += sensor_control(1);
		/* All sensors stop reporting on reset */
		enabled_sensor_mask = 0;
		return rc;

		case DYN_PROTOCOL_EID_PING_SENSOR:
		INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command ping(%s)", inv_sensor_2str(sensor));
		if((sensor == INV_SENSOR_TYPE_RAW_ACCELEROMETER)
		|| (sensor == INV_SENSOR_TYPE_RAW_GYROSCOPE)
		|| (sensor == INV_SENSOR_TYPE_ACCELEROMETER)
		|| (sensor == INV_SENSOR_TYPE_GYROSCOPE)
		|| (sensor == INV_SENSOR_TYPE_UNCAL_GYROSCOPE)
		|| (sensor == INV_SENSOR_TYPE_GAME_ROTATION_VECTOR)
		|| (sensor == INV_SENSOR_TYPE_GRAVITY)
		|| (sensor == INV_SENSOR_TYPE_LINEAR_ACCELERATION)
		|| (sensor == INV_SENSOR_TYPE_CUSTOM_PRESSURE)
		) {
			return 0;
		} else if((sensor == INV_SENSOR_TYPE_PRED_QUAT_0)
		|| (sensor == INV_SENSOR_TYPE_PRED_QUAT_1)
		) {
			if (hmd_features_supported)
			return 0;
			else
			return INV_ERROR_BAD_ARG;
		} else
		return INV_ERROR_BAD_ARG;

		case DYN_PROTOCOL_EID_SELF_TEST:
		INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command selft_test(%s)", inv_sensor_2str(sensor));
		if((sensor == INV_SENSOR_TYPE_RAW_ACCELEROMETER) ||
		   (sensor == INV_SENSOR_TYPE_ACCELEROMETER) ||
		   (sensor == INV_SENSOR_TYPE_RAW_GYROSCOPE) ||
		   (sensor == INV_SENSOR_TYPE_GYROSCOPE))
			return icm207xx_run_selftest();
		else
			return INV_ERROR_NIMPL;

		case DYN_PROTOCOL_EID_START_SENSOR:
		INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command start(%s)", inv_sensor_2str(sensor));
		if (sensor > 0 && idd_sensortype_conversion(sensor) < SENSOR_MAX) {
			/* Sensor data will be notified */
			//if (sensor == INV_SENSOR_TYPE_CUSTOM_PRESSURE) { 
				//set_pressure_timer(pressure_ms);
				//inv_invpres_enable_sensor(&invpres_device, 1);
			//}
			enabled_sensor_mask |= (1 << idd_sensortype_conversion(sensor));
			return 0;
		} else
		return INV_ERROR_NIMPL; /*this sensor is not supported*/

		case DYN_PROTOCOL_EID_STOP_SENSOR:
		INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command stop(%s)", inv_sensor_2str(sensor));
		if (sensor > 0 && idd_sensortype_conversion(sensor) < SENSOR_MAX) {
			/* Sensor data will not be notified anymore */
			if (sensor == INV_SENSOR_TYPE_CUSTOM_PRESSURE && (enabled_sensor_mask & (1 << SENSOR_CUSTOM_PRESSURE))) {
				inv_invpres_enable_sensor(&invpres_device, 0);
			}
			enabled_sensor_mask &= ~(1 << idd_sensortype_conversion(sensor));
			return 0;
		} else
		return INV_ERROR_NIMPL; /*this sensor is not supported*/

		case DYN_PROTOCOL_EID_SET_SENSOR_PERIOD:
		{	
			if (sensor == INV_SENSOR_TYPE_CUSTOM_PRESSURE) {
				//pressure_ms = edata->d.command.period / 1000;
				//if ((pressure_ms) > 1000)	// Min 1000 ms (or 1 Hz)
					//pressure_ms = 1000;
				//else if ((pressure_ms) < 25) // Max 25ms (or 40 Hz)
					//pressure_ms = 25;
				//set_pressure_timer(pressure_ms);
			}
			else {
				INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command set_period(%d us)",edata->d.command.period);
				rc = sensor_configure_odr(edata->d.command.period, enabled_sensor_mask | (1 << idd_sensortype_conversion(sensor)));

				algorithms_configure_odr(period_us, 0);
			}
			return rc;
		}

		case DYN_PROTOCOL_EID_SET_SENSOR_CFG:
		INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command set_sensor_config(%s)", inv_sensor_2str(sensor));
		switch(edata->d.command.cfg.base.type) {
			
			case VSENSOR_CONFIG_TYPE_REFERENCE_FRAME:
			{
				float mmatrix[9];
				
				/* Ensure any float manipulation is word-aligned because of unsupported unaligned float access depending on CPU target */
				memcpy(mmatrix, edata->d.command.cfg.buffer, edata->d.command.cfg.base.size);
				
				if((sensor == INV_SENSOR_TYPE_RAW_ACCELEROMETER)
				|| (sensor == INV_SENSOR_TYPE_ACCELEROMETER)
				|| (sensor == INV_SENSOR_TYPE_RAW_GYROSCOPE)
				|| (sensor == INV_SENSOR_TYPE_GYROSCOPE)
				|| (sensor == INV_SENSOR_TYPE_UNCAL_GYROSCOPE)) {
					for(i = 0; i < 9; ++i)
					cfg_mounting_matrix[i] = (int32_t)(mmatrix[i] * (1L << 30));
					return 0;
				}

				else return INV_ERROR_BAD_ARG;
			}
			
			case INV_SENSOR_CONFIG_PRED_GRV:
			{
				int param = ALGO_PARAM_MAX;
				
				/* check sensor type */
				if(sensor == INV_SENSOR_TYPE_PRED_QUAT_0)
				param = ALGO_PARAM_CONFIG_PRED_GRV0;
				else if(sensor == INV_SENSOR_TYPE_PRED_QUAT_1)
				param = ALGO_PARAM_CONFIG_PRED_GRV1;
				
				/* if sensor type ko return error */
				if (param == ALGO_PARAM_MAX)
				rc = INV_ERROR_BAD_ARG;
				/* else parse config parameters and feed algo */
				else
				rc = algorithms_set_param(param, edata->d.command.cfg.buffer);
				return rc;
			}
			
			case INV_SENSOR_CONFIG_BT_STATE:
			{
				return algorithms_set_param(ALGO_PARAM_CONFIG_BT_STATE, edata->d.command.cfg.buffer);
			}
			
			default:
			return INV_ERROR_NIMPL;
		}
		
		case DYN_PROTOCOL_EID_CLEANUP:
		INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command cleanup");
		algorithms_sensor_control(0);
		rc = sensor_control(0);
		store_offsets(
		(outputs.acc_accuracy_flag == 3) ? outputs.acc_bias_q16 : last_loaded_acc_bias,
		(outputs.gyr_accuracy_flag == 3) ? outputs.gyr_bias_fnm_q16 : last_loaded_gyr_bias,
		(outputs.mag_accuracy_flag == 3) ? outputs.mag_bias_q16 : last_loaded_mag_bias);
		/* Soft reset */
		rc += inv_icm207xx_soft_reset(&icm_device);

		/* All sensors stop reporting on cleanup */
		enabled_sensor_mask = 0;
		return rc;

		default:
		return INV_ERROR_NIMPL;
	}
}

void store_offsets(const int32_t acc_bias_q16[3], const int32_t gyr_bias_q16[3], const int32_t mag_bias_q16[3])
{
	uint8_t i, idx = 0;
	int raw_bias[12] = {0};
	uint8_t sensor_bias[84] = {0};
	
	/* Strore offsets in NV memory */
	inv_icm207xx_get_st_bias(&icm_device, raw_bias);
	/* Store ST biases: 3(axis) * 4(AccLP, AccLN, GyrLP, GyrLN) * 4(uint32_t) = 48 B [total=48 B] */
	for(i = 0; i < 12; i++)
		inv_dc_int32_to_little8(raw_bias[i], &sensor_bias[i * sizeof(uint32_t)]);
	idx += sizeof(raw_bias);
	
	/* Store Calib Accel biases: 3(axis) * 4(uint32_t) = 12 B [total=60 B] */
	for(i = 0; i < 3; i++)
		inv_dc_int32_to_little8(acc_bias_q16[i], &sensor_bias[idx + i * sizeof(uint32_t)]);
	idx += (sizeof(acc_bias_q16[0]) * 3);
	
	/* Store Calib Gyro biases: 3(axis) * 4(uint32_t) = 12 B [total=72 B] */
	for(i = 0; i < 3; i++)
		inv_dc_int32_to_little8(gyr_bias_q16[i], &sensor_bias[idx + i * sizeof(uint32_t)]);
	idx += (sizeof(gyr_bias_q16[0]) * 3);
}

int icm207xx_run_selftest(void)
{
	int raw_bias[12];
	int rc = 0;

	if (icm_device.selftest_done == 1) {
		INV_MSG(INV_MSG_LEVEL_INFO, "Self-test has already run. Skipping.");
	}
	else {
		/* 
		 * Perform self-test
		 * For ICM207xx self-test is performed for both RAW_ACC/RAW_GYR
		 */
		INV_MSG(INV_MSG_LEVEL_INFO, "Running self-test...");

		/* Run the self-test */
		rc = inv_icm207xx_run_selftest(&icm_device);
		/* Check transport errors */
		check_rc(rc, "Self-test failure");
		if (rc != 0x3) {
			/*
			 * Check for GYR success (1 << 0) and ACC success (1 << 1),
			 * but don't block as these are 'usage' failures.
			 */
			INV_MSG(INV_MSG_LEVEL_ERROR, "Self-test failure");
			/* 0 would be considered OK, we want KO */
			return INV_ERROR;
		} else
			/* On success, offset will be kept until reset */
			icm_device.selftest_done = 1;

		/* It's advised to re-init the icm207xx device after self-test for normal use */
		rc = icm207xx_sensor_setup();
	}

	/* 
	 * Get Low Noise / Low Power bias computed by self-tests scaled by 2^16
	 */
	INV_MSG(INV_MSG_LEVEL_INFO, "Getting LP/LN bias");
	inv_icm207xx_get_st_bias(&icm_device, raw_bias);
	INV_MSG(INV_MSG_LEVEL_INFO, "GYR LN bias (FS=250dps) (dps): x=%f, y=%f, z=%f", 
			(float)(raw_bias[0] / (float)(1 << 16)), (float)(raw_bias[1] / (float)(1 << 16)), (float)(raw_bias[2] / (float)(1 << 16)));
	INV_MSG(INV_MSG_LEVEL_INFO, "GYR LP bias (FS=250dps) (dps): x=%f, y=%f, z=%f", 
			(float)(raw_bias[3] / (float)(1 << 16)), (float)(raw_bias[4] / (float)(1 << 16)), (float)(raw_bias[5] / (float)(1 << 16)));
	INV_MSG(INV_MSG_LEVEL_INFO, "ACC LN bias (FS=2g) (g): x=%f, y=%f, z=%f", 
			(float)(raw_bias[0 + 6] / (float)(1 << 16)), (float)(raw_bias[1 + 6] / (float)(1 << 16)), (float)(raw_bias[2 + 6] / (float)(1 << 16)));
	INV_MSG(INV_MSG_LEVEL_INFO, "ACC LP bias (FS=2g) (g): x=%f, y=%f, z=%f", 
			(float)(raw_bias[3 + 6] / (float)(1 << 16)), (float)(raw_bias[4 + 6] / (float)(1 << 16)), (float)(raw_bias[5 + 6] / (float)(1 << 16)));

	return rc;
}

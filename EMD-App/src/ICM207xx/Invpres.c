/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2017-18 InvenSense Inc. All rights reserved.
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
#include "Invpres.h"
#include "Invn/EmbUtils/Message.h"
#include "Invn/Devices/SensorTypes.h"
#include "math.h"

#define INVPRES_ODR_MIN_DELAY 25000 /* us => 40 Hz */

#define INVPRES_CRC8_INIT 0xFF
#define INVPRES_RESP_DWORD_LEN 2
#define INVPRES_RESP_CRC_LEN 1
#define INVPRES_RESP_FRAME_LEN (INVPRES_RESP_DWORD_LEN + INVPRES_RESP_CRC_LEN)
#define INVPRES_CRC8_POLYNOM 0x31

extern void inv_icm207xx_sleep_us(int us);
extern void sensor_event(const inv_sensor_event_t * event, void * arg);

#ifdef _DEBUG
static unsigned char invpres_check_crc(uint8_t *frame);
#endif
static int send_measurement_command(struct inv_invpres * s);

#ifdef _DEBUG
static unsigned char invpres_check_crc(uint8_t *frame)
{
	uint8_t crc = INVPRES_CRC8_INIT;
	uint8_t current_byte;
	uint8_t bit;

	/* Calculates 8-bit checksum with given polynomial. */
	for (current_byte = 0; current_byte < INVPRES_RESP_DWORD_LEN; ++current_byte)
	{
		crc ^= (frame[current_byte]);
		for (bit = 8; bit > 0; --bit)
		{
			if (crc & 0x80)
				crc = (crc << 1) ^ INVPRES_CRC8_POLYNOM;
			else
				crc = (crc << 1);
		}
	}
	return crc;
}
#endif

void init_base(struct inv_invpres * s, short *otp)
{
	int i;

	for(i = 0; i < 4; i++)
		s->sensor_constants[i] = (float)otp[i];

	s->p_Pa_calib[0] = 45000.0;
	s->p_Pa_calib[1] = 80000.0;
	s->p_Pa_calib[2] = 105000.0;
	s->LUT_lower = 3.5 * (1<<20);
	s->LUT_upper = 11.5 * (1<<20);
	s->quadr_factor = 1 / 16777216.0;
	s->offst_factor = 2048.0;
}

// p_Pa -- List of 3 values corresponding to applied pressure in Pa
// p_LUT -- List of 3 values corresponding to the measured p_LUT values at the applied pressures.
void calculate_conversion_constants(struct inv_invpres * s, float *p_Pa, float *p_LUT, float *out)
{
	float A,B,C;

	C = (p_LUT[0] * p_LUT[1] * (p_Pa[0] - p_Pa[1]) +
		p_LUT[1] * p_LUT[2] * (p_Pa[1] - p_Pa[2]) +
		p_LUT[2] * p_LUT[0] * (p_Pa[2] - p_Pa[0])) /
		(p_LUT[2] * (p_Pa[0] - p_Pa[1]) +
		p_LUT[0] * (p_Pa[1] - p_Pa[2]) +
		p_LUT[1] * (p_Pa[2] - p_Pa[0]));
	A = (p_Pa[0] * p_LUT[0] - p_Pa[1] * p_LUT[1] - (p_Pa[1] - p_Pa[0]) * C) / (p_LUT[0] - p_LUT[1]);
	B = (p_Pa[0] - A) * (p_LUT[0] + C);

	out[0] = A;
	out[1] = B;
	out[2] = C;
}

// p_LSB -- Raw pressure data from sensor
// T_LSB -- Raw temperature data from sensor
int inv_invpres_process_data(struct inv_invpres * s, int p_LSB, int T_LSB, float * pressure, float * temperature)
{
	float t;
	float s1,s2,s3;
	float in[3];
	float out[3];
	float A,B,C;

	t = (float)(T_LSB - 32768);
	s1 = s->LUT_lower + (float)(s->sensor_constants[0] * t * t) * s->quadr_factor;
	s2 = s->offst_factor * s->sensor_constants[3] + (float)(s->sensor_constants[1] * t * t) * s->quadr_factor;
	s3 = s->LUT_upper + (float)(s->sensor_constants[2] * t * t) * s->quadr_factor;
	in[0] = s1;
	in[1] = s2;
	in[2] = s3;

	calculate_conversion_constants(s, s->p_Pa_calib, in, out);
	A = out[0];
	B = out[1];
	C = out[2];

	*pressure = A + B / (C + p_LSB);

	*temperature = -45.f + 175.f/65536.f * T_LSB;

	return 0;
}

int read_id_from_i2c(struct inv_invpres * s, uint8_t * whoami)
{
	unsigned char data_write[10];
	unsigned char data_read[10] = {0};
	int status;
	int out;

	// Read ID of pressure sensor
	data_write[0] = 0xEF;
	data_write[1] = 0xC8;
	status = inv_invpres_serif_write_reg(&s->serif, ICC_ADDR_PRS, data_write, 2);
	if (status)
		return status;

	status = inv_invpres_serif_read_reg(&s->serif, ICC_ADDR_PRS, data_read, 3);
	if (status)
		return status;

	out = data_read[0]<<8 | data_read[1];
	out &= 0x3f; // take bit5 to 0

	*whoami = (uint8_t)out;
	return 0;
}

int read_otp_from_i2c(struct inv_invpres * s, short *out)
{
	unsigned char data_write[10];
	unsigned char data_read[10] = {0};
	int status;
	int i;

	// OTP Read mode
	data_write[0] = 0xC5;
	data_write[1] = 0x95;
	data_write[2] = 0x00;
	data_write[3] = 0x66;
	data_write[4] = 0x9C;
	status = inv_invpres_serif_write_reg(&s->serif, ICC_ADDR_PRS, data_write, 5);
	if (status)
		return status;

	// Read OTP values
	for (i = 0; i < 4; i++) {
		data_write[0] = 0xC7;
		data_write[1] = 0xF7;
		status = inv_invpres_serif_write_reg(&s->serif, ICC_ADDR_PRS, data_write, 2);
		if (status)
			return status;

		status = inv_invpres_serif_read_reg(&s->serif, ICC_ADDR_PRS, data_read, 3);
		if (status)
			return status;
		out[i] = data_read[0]<<8 | data_read[1];
	}

	return 0;
}

static int send_measurement_command(struct inv_invpres * s)
{
	int status = 0;
	unsigned char data_write[10];

	// Send Measurement Command
	data_write[0] = 0x50; // Read P First (Mode3)
	data_write[1] = 0x59;

	status = inv_invpres_serif_write_reg(&s->serif, ICC_ADDR_PRS, data_write, 2);
	return status;
}

int read_raw_pressure_temp_from_i2c(struct inv_invpres * s, int *pressure, int *temp)
{
	unsigned char data_read[10] = {0};
	int status;

	status = inv_invpres_serif_read_reg(&s->serif, ICC_ADDR_PRS, data_read, 9);
	if (status)
		return status;

#ifdef _DEBUG
	unsigned crc;
	crc = invpres_check_crc(&data_read[0]);
	INV_MSG(0, "ICC41250 Data1 : 0x%02X 0x%02X CRC:0x%02X cal_crc:0x%02X\r\n", data_read[0]&0xff, data_read[1]&0xff, data_read[2]&0xff, crc);
	crc = invpres_check_crc(&data_read[3]);
	INV_MSG(0, "ICC41250 Data2 : 0x%02X 0x%02X CRC:0x%02X cal_crc:0x%02X\r\n", data_read[3]&0xff, data_read[4]&0xff, data_read[5]&0xff, crc);
	crc = invpres_check_crc(&data_read[6]);
	INV_MSG(0, "ICC41250 Data3 : 0x%02X 0x%02X CRC:0x%02X cal_crc:0x%02X\r\n", data_read[6]&0xff, data_read[7]&0xff, data_read[8]&0xff, crc);
#endif

	// Temperature
	*temp = data_read[6] << 8 | data_read[7];

	// Pressure
	*pressure = data_read[0]<<(8*2) | data_read[1]<<(8*1) | data_read[3]<<(8*0);

#ifdef _DEBUG
	float T = -45.f + 175.f/65536.f * (float)*temp;
	INV_MSG(INV_MSG_LEVEL_INFO, "raw temp = %d\r\n", *temp);
	INV_MSG(INV_MSG_LEVEL_INFO, "Temperature = %f\r\n", T);
#endif

#ifdef _DEBUG
	INV_MSG(INV_MSG_LEVEL_INFO, "raw pressure = %d\r\n", *pressure);
#endif

	status = send_measurement_command(s);

	return status;
}

int inv_invpres_init(struct inv_invpres * s)
{
	uint8_t whoami = 0;
	short otp[4];

	inv_invpres_get_whoami(s, &whoami);
	if(whoami != INVPRES_ID_REG_EXPECTED_VALUE)
		return -1;

	s->min_delay_us = INVPRES_ODR_MIN_DELAY;

	read_otp_from_i2c(s, otp);

	init_base(s, otp);

	return 0;
}

int inv_invpres_get_data(struct inv_invpres * s, int * raw_pressure, int * raw_temperature, float * pressure, float * temperature)
{
	int rc = 0;

	int rawP, rawT;
	float pressure_Pa, temperature_C;

	if ( (s->pressure_en) || (s->temperature_en) ) {
		rc = read_raw_pressure_temp_from_i2c(s, &rawP, &rawT);
		if(rc != 0)
			return rc;
		rc = inv_invpres_process_data(s, rawP, rawT, &pressure_Pa, &temperature_C);
		if (s->pressure_en) {
			if (pressure) {
				*raw_pressure = rawP;
				*pressure = pressure_Pa;
			}
		}
		if (s->temperature_en) {
			if (temperature) {
				*raw_temperature = rawT;
				*temperature = temperature_C;
			}
		}
	}

	return rc;
}

int inv_invpres_enable_sensor(struct inv_invpres * s, inv_bool_t en)
{
	int rc = 0;
	if (en) {
		s->pressure_en = 1;
		s->temperature_en = 1;

		rc = send_measurement_command(s);
	}
	else {
		s->pressure_en = 0;
		s->temperature_en = 0;
	}

	return rc;
}

int inv_invpres_pressure_enable_sensor(struct inv_invpres * s, inv_bool_t en)
{
	int rc = 0;

	if (en)
		s->pressure_en = 1;
	else
		s->pressure_en = 0;

	if (!s->temperature_en) {
		rc = inv_invpres_enable_sensor(s, en);
	}

	return rc;
}

int inv_invpres_temperature_enable_sensor(struct inv_invpres * s, inv_bool_t en)
{
	int rc = 0;

	if (en)
		s->temperature_en = 1;
	else
		s->temperature_en = 0;

	if (!s->pressure_en) {
		rc = inv_invpres_enable_sensor(s, en);
	}

	return rc;
}

int inv_invpres_get_whoami(struct inv_invpres * s, uint8_t * whoami)
{
	return read_id_from_i2c(s, whoami);
}

int inv_invpres_soft_reset(struct inv_invpres * s)
{
	int status = 0;
	unsigned char data_write[10];

	// Send Soft Reset Command
	data_write[0] = 0x80;
	data_write[1] = 0x5D;

	status = inv_invpres_serif_write_reg(&s->serif, ICC_ADDR_PRS, data_write, 2);

	inv_icm207xx_sleep_us(170);
	return status;
}

void inv_invpres_poll(struct inv_invpres * s, int64_t timestamp)
{
	float pressure, temperature;
	int raw_pressure, raw_temperature;
	
	int rc;
	inv_sensor_event_t event;
	
	rc = inv_invpres_get_data(s, &raw_pressure, &raw_temperature, &pressure, &temperature);
	if(rc == 0) {
		memset(&event, 0, sizeof(event));
		event.sensor	= INV_SENSOR_TYPE_CUSTOM_PRESSURE;
		event.timestamp = timestamp;
		event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
		event.data.custom_pressure.raw_pressure = raw_pressure;
		event.data.custom_pressure.raw_temperature = raw_temperature;
		event.data.custom_pressure.pressure = pressure;
		event.data.custom_pressure.temperature = temperature;
		sensor_event(&event, NULL);
	}
}

void inv_invpres_setup(struct inv_invpres * s)
{
	int rc;
	
	rc = inv_invpres_soft_reset(s);
	if(rc == 0)
		inv_invpres_init(s);
}

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
#include "algo_eapi.h"
#include "Invn/LibAlgo/LibAlgo.h"
#include "Invn/EmbUtils/ErrorHelper.h"

#include <stdio.h>
#include <assert.h>
#include <string.h>

/* Sensitivity configurations */
#define ACC_SENSITIVITY (int32_t) ( ((cfg_acc_fsr/1000) * (1 << 16)) / INT16_MAX)
#define GYR_SENSITIVITY (int32_t) ( (cfg_gyr_fsr * (1 << 16) / INT16_MAX) )

#define RV_ANOMALIES_REJECTION             1073741824L // = 100% rejection = 2^30L
#define RV_THRESHOLD_YAW_STOP_CONVERGENCE  -1L         // 18740330L = 1 deg * pi/180 * 2^30
#define RV_THRESHOLD_GYR_STOP_CONVERGENCE  1073742L    // 1073742L = 2dps * 2^30 / 2000
#define RV_THRESHOLD_YAW_SMOOTH_CONVERGENCE_DEFAULT 650L // 650L = 1% * 2^16
	
#define DATA_ACCURACY_MASK  ((uint32_t)0x7)
/* factor to convert degree to radian expressed in q30 */
#define FACTOR_DPS_TO_RPS   (int32_t) 18740330 // ((PI / 180) * (1 << 30))


static struct {
	union {
		uint8_t buf[ALGO_INVN_CALIBRATION_ACGO_CONFIG_SIZE];
		float   flt; /* ensure correct memory alignment of the buffer */
	} C_buffer;
} sCalAcc;

static struct {
	union {
		uint8_t buf[ALGO_INVN_CALIBRATION_GYR_CAL_FXP_SIZE];
		float   flt; /* ensure correct memory alignment of the buffer */
	} C_buffer;
	union {
		uint8_t buf[ALGO_INVN_CALIBRATION_GYRO_BT_CONFIG_SIZE];
		float   flt; /* ensure correct memory alignment of the buffer */
	} C_buffer_bt;
} sCalGyr;

static struct {
	union {
		uint8_t buf[ALGO_INVN_ORIENTATION_CONFIG_SIZE];
		float flt; /* ensure proper alignement */
	} C_buffer;
} sGRV;

static struct {
	union {
		uint8_t buf[ALGO_INVN_PREDICTIVEQUATERNION_CONFIG_SIZE];
		float flt; /* ensure proper alignement */
	} C_buffer;
} sPredGRV0;

static struct {
	union {
		uint8_t buf[ALGO_INVN_PREDICTIVEQUATERNION_CONFIG_SIZE];
		float flt; /* ensure proper alignement */
	} C_buffer;
} sPredGRV1;


static struct {
	union {
		uint8_t buf[ALGO_INVN_CALIBRATION_CCGO_CONFIG_SIZE];
		float flt; /* ensure correct memory alignment */
	} C_buffer;
} sCalMag;

static struct {
	union {
		uint8_t buf[ALGO_INVN_ORIENTATION_CONFIG_SIZE];
		float flt; /* ensure correct memory alignment */
	} C_buffer;
} sRV;

static struct {
	union {
		uint8_t buf[ALGO_INVN_ORIENTATION_CONFIG_SIZE];
		float flt; /* ensure correct memory alignement */
	} C_buffer;
} sGeoRV;

/*
 * Variable keeping track of HMD features support
 */
static uint8_t hmd_features_supported = 0;

/*
 * Variable keeping track of chip information
 */
uint8_t chip_info[3];

/*
 * Variable keeping track of whether algorithms can used magnetometer
 */
static uint8_t mag_is_available = 0;

/*
 * Variable keeping track of the period, used by some 'Update' functions
 */
static uint32_t period_us;

uint8_t algorithms_init(int32_t acc_bias_q16[3], int32_t gyr_bias_q16[3], int32_t mag_bias_q16[3], void *params)
{
	int32_t gyro_offset_2000dps_q30[3] = {
		((gyr_bias_q16[0] << 3) / 2000) << (30 - 16 - 3),
		((gyr_bias_q16[1] << 3) / 2000) << (30 - 16 - 3),
		((gyr_bias_q16[2] << 3) / 2000) << (30 - 16 - 3)
	};
	uint8_t gyr_accuracy = 0;
	int32_t acc_offset_q25[3] = {
		(acc_bias_q16[0]) << (25 - 16),
		(acc_bias_q16[1]) << (25 - 16),
		(acc_bias_q16[2]) << (25 - 16)
	};
	uint8_t acc_accuracy = 0;
	uint32_t mag_period_us;
	uint8_t mag_accuracy = 0;

	memcpy(chip_info, params, sizeof(chip_info));
	
	/* 
	 * Get the HMD features support availability for the chip 
	 */
	hmd_features_supported = Algo_InvnCalibration_isHMDDevice(chip_info);

	/*
	 * Init the gyroscope FNM calibration
	 */
	if ((gyro_offset_2000dps_q30[0] != 0 ) || (gyro_offset_2000dps_q30[1] != 0 ) || (gyro_offset_2000dps_q30[2] != 0 ) )
		gyr_accuracy = 2;
	Algo_InvnCalibration_GyroCalibrationFxp_Init(sCalGyr.C_buffer.buf, gyro_offset_2000dps_q30, gyr_accuracy);
	Algo_InvnCalibration_GyroCalibrationFxp_SetUserParam(sCalGyr.C_buffer.buf, 0, HMD_VR_MODE);

	if (hmd_features_supported) {
		/*
	 	 * Init the gyroscope Bias Tracker orientation
	 	 */
		Algo_InvnCalibration_GyroBiasTrackerFxp_Init(sCalGyr.C_buffer_bt.buf);
		Algo_InvnCalibration_GyroBiasTrackerFxp_SetBias(sCalGyr.C_buffer_bt.buf, gyro_offset_2000dps_q30, gyr_accuracy);
	}
	
	/* 
	 * Init the accelerometer calibration 
	 */
	if ((acc_offset_q25[0] != 0 ) || (acc_offset_q25[1] != 0 ) || (acc_offset_q25[2] != 0 ) )
		acc_accuracy = 3;
	Algo_InvnCalibration_AccelCalibrationGyroOptionalFxp_Init(sCalAcc.C_buffer.buf, acc_offset_q25, acc_accuracy, DEFAULT_ODR_US);

	/*
	 * Init the GRV orientation
	 */
	Algo_InvnOrientation_BodyToWorldFrameFxp_Init(sGRV.C_buffer.buf);
	
	/*
	 * Init the Predictive quaternions orientation
	 */
	if(hmd_features_supported) {
		Algo_InvnOrientation_PredictiveQuaternionFxp_Init(sPredGRV0.C_buffer.buf);
		Algo_InvnOrientation_PredictiveQuaternionFxp_SetParam(sPredGRV0.C_buffer.buf,
			ALGO_INVN_PREDICTIVEQUATERNION_DEFAULT_METHOD_ENNUM, ALGO_INVN_PREDICTIVEQUATERNION_DEFAULT_PREDICTIVE_TIME_US
		);

		Algo_InvnOrientation_PredictiveQuaternionFxp_Init(sPredGRV1.C_buffer.buf);
		Algo_InvnOrientation_PredictiveQuaternionFxp_SetParam(sPredGRV1.C_buffer.buf,
			ALGO_INVN_PREDICTIVEQUATERNION_DEFAULT_METHOD_ENNUM, ALGO_INVN_PREDICTIVEQUATERNION_SHORTER_PREDICTIVE_TIME_US
		);
	}

	/* Mag is not present */
	if (!mag_bias_q16) {
		mag_is_available = 0;
		return hmd_features_supported;
	}
	mag_is_available = 1;

	/*
	 * Init the compass calibration algorithm, 
	 * mag_period_us is trusted to be correctly given by caller
	 */
	mag_period_us = DEFAULT_ODR_US;

	if ((mag_bias_q16[0] != 0 ) || (mag_bias_q16[1] != 0 ) || (mag_bias_q16[2] != 0 ) )
		mag_accuracy = 1;
	Algo_InvnCalibration_CompassCalibrationGyroOptionalFxp_Init(sCalMag.C_buffer.buf, mag_bias_q16, mag_accuracy, mag_period_us);

	/*
	 * Init the RV orientation
	 */
	Algo_InvnOrientation_BodyToWorldFrameFxp_Init(sRV.C_buffer.buf);
	Algo_InvnOrientation_BodyToWorldFrameFxp_SetMagCustomParams(sRV.C_buffer.buf,
			RV_ANOMALIES_REJECTION,
			RV_THRESHOLD_YAW_STOP_CONVERGENCE,
			RV_THRESHOLD_GYR_STOP_CONVERGENCE,
			RV_THRESHOLD_YAW_SMOOTH_CONVERGENCE_DEFAULT
	);

	/*
	 * Init the GeoRV orientation
	 */
	Algo_InvnOrientation_BodyToWorldFrameFxp_Init(sGeoRV.C_buffer.buf);
	
	return hmd_features_supported;
}

void algorithms_configure_odr(uint32_t odr_us, uint32_t mag_odr_us)
{
	/* 
	 * Update algorithm parameters for Gyroscope calibration, including bias tracker
	 */
	Algo_InvnCalibration_GyroCalibrationFxp_SetSamplingPeriod(sCalGyr.C_buffer.buf, odr_us);
	if (hmd_features_supported) {
		Algo_InvnCalibration_GyroBiasTrackerFxp_SetGyrSamplingPeriod(sCalGyr.C_buffer_bt.buf, odr_us, chip_info);
		Algo_InvnCalibration_GyroBiasTrackerFxp_SetAccSamplingPeriod(sCalGyr.C_buffer_bt.buf, odr_us, chip_info);
		if (mag_is_available)
			Algo_InvnCalibration_GyroBiasTrackerFxp_SetMagSamplingPeriod(sCalGyr.C_buffer_bt.buf, mag_odr_us);
	}
	
	/* 
	 * Update algorithm parameters for Accelerometer calibration
	 */
	Algo_InvnCalibration_AccelCalibrationGyroOptionalFxp_SetSamplingPeriod(sCalAcc.C_buffer.buf, odr_us);

	/* 
	 * Update algorithm parameters for Magnetometer calibration
	 */
	if (mag_is_available)
		Algo_InvnCalibration_CompassCalibrationGyroOptionalFxp_SetSamplingPeriod(sCalMag.C_buffer.buf, mag_odr_us);

	/* 
	 * Update algorithm parameters for GRV orientation
	 */
	Algo_InvnOrientation_BodyToWorldFrameFxp_SetGyrSamplingPeriod(sGRV.C_buffer.buf, odr_us, chip_info);
	Algo_InvnOrientation_BodyToWorldFrameFxp_SetAccSamplingPeriod(sGRV.C_buffer.buf, odr_us, chip_info);
	
	/* 
	 * Update algorithm parameters for Predictive Quaternion orientation
	 */
	if(hmd_features_supported) {
		Algo_InvnOrientation_PredictiveQuaternionFxp_SetSamplingPeriod(sPredGRV0.C_buffer.buf, odr_us);
		Algo_InvnOrientation_PredictiveQuaternionFxp_SetSamplingPeriod(sPredGRV1.C_buffer.buf, odr_us);
	}
	
	/* 
	 * Update algorithm parameters for RV orientation
	 */
	if (mag_is_available) {
		Algo_InvnOrientation_BodyToWorldFrameFxp_SetGyrSamplingPeriod(sRV.C_buffer.buf, odr_us, chip_info);
		Algo_InvnOrientation_BodyToWorldFrameFxp_SetAccSamplingPeriod(sRV.C_buffer.buf, odr_us, chip_info);
		Algo_InvnOrientation_BodyToWorldFrameFxp_SetMagSamplingPeriod(sRV.C_buffer.buf, mag_odr_us);
	}

	/* Keep copy in compilation unit for 'Update' calls which need it */
	period_us = odr_us;
}

void algorithms_sensor_control(uint32_t enable)
{
	static uint8_t sensors_on = 0;
	static uint32_t active_sensors_cnt = 0;

	if(enable) {
		active_sensors_cnt++;
	} else {
		if(active_sensors_cnt)
			active_sensors_cnt--;
	}
	
	/* Keep track of the sensors state */
	if(enable && sensors_on)	// If the also sensors are already on, don't need to enable them.
		return;

	if(enable==0 && sensors_on && active_sensors_cnt!=0)	 // If multiple sensors are active, don't disable them.
		return;

	if(enable)
		sensors_on = 1;
	else
		sensors_on = 0;
	
	/* Handling of Game Rotation Vector (6-axis AG) */
	if (enable) {
		/* Handles the orientation algoritm state */
		Algo_InvnOrientation_BodyToWorldFrameFxp_ResetStates(sGRV.C_buffer.buf);
		Algo_InvnOrientation_BodyToWorldFrameFxp_AG_Enable(sGRV.C_buffer.buf);
	} else
		Algo_InvnOrientation_BodyToWorldFrameFxp_AG_Disable(sGRV.C_buffer.buf);

	/* Handling of Predictive Quaternion (6-axis AG) */
	if (enable && hmd_features_supported) {
		/* Handles the orientation algoritm state for both pred quat instances */
		Algo_InvnOrientation_PredictiveQuaternionFxp_ResetStates(sPredGRV0.C_buffer.buf);
		Algo_InvnOrientation_PredictiveQuaternionFxp_SetSamplingPeriod(sPredGRV0.C_buffer.buf, period_us);
		
		Algo_InvnOrientation_PredictiveQuaternionFxp_ResetStates(sPredGRV1.C_buffer.buf);
		Algo_InvnOrientation_PredictiveQuaternionFxp_SetSamplingPeriod(sPredGRV1.C_buffer.buf, period_us);
	}

	if (mag_is_available) {
		/* Handling of Rotation Vector (9-axis AGM) */
		if (enable) {
			/* Handles the orientation algoritm state */
			Algo_InvnOrientation_BodyToWorldFrameFxp_ResetStates(sRV.C_buffer.buf);
			Algo_InvnOrientation_BodyToWorldFrameFxp_AGM_Enable(sRV.C_buffer.buf);
		} else
			Algo_InvnOrientation_BodyToWorldFrameFxp_AGM_Disable(sRV.C_buffer.buf);

		/* Handling of Geomagnetic Rotation Vector (6-axis AGM) */
		if (enable) {
			/* Handles the orientation algoritm state */
			Algo_InvnOrientation_BodyToWorldFrameFxp_ResetStates(sGeoRV.C_buffer.buf);
			Algo_InvnOrientation_BodyToWorldFrameFxp_AM_Enable(sGeoRV.C_buffer.buf);
		} else
			Algo_InvnOrientation_BodyToWorldFrameFxp_AM_Disable(sGeoRV.C_buffer.buf);
	}
}

void algorithms_process(algo_input *inputs, algo_output *outputs)
{
	/* Get the temperature value by applying the sensitivity 326.8 LSB/degC and adding 25degC offset */
	const int32_t temp_degC_q16 = (((int32_t)(inputs->sRtemp_data << 16)) / 3268 * 10) + (int32_t)(25 << 16);
	const int32_t temp_100degC = (temp_degC_q16 * 100) >> 16;
	int32_t gyro_cal_2000dps_q30 [3] = {0};

	/*
	 * Compute the calibrated accelerometer data
	 */
	if (outputs->acc_accuracy_flag != -1)
	{
		const int32_t raw_accel_q25[3] = {
			(inputs->sRacc_data[0] * ACC_SENSITIVITY) << (25 - 16),
			(inputs->sRacc_data[1] * ACC_SENSITIVITY) << (25 - 16),
			(inputs->sRacc_data[2] * ACC_SENSITIVITY) << (25 - 16),
		};
		int32_t accel_bias_q25[3];
		int32_t accel_cal_q25[3];

		Algo_InvnCalibration_AccelCalibrationGyroOptionalFxp_UpdateAcc(sCalAcc.C_buffer.buf, raw_accel_q25, temp_100degC, accel_bias_q25);
		accel_cal_q25[0] = raw_accel_q25[0] - accel_bias_q25[0];
		accel_cal_q25[1] = raw_accel_q25[1] - accel_bias_q25[1];
		accel_cal_q25[2] = raw_accel_q25[2] - accel_bias_q25[2];

		outputs->acc_bias_q16[0] = accel_bias_q25[0] >> (25 - 16);
		outputs->acc_bias_q16[1] = accel_bias_q25[1] >> (25 - 16);
		outputs->acc_bias_q16[2] = accel_bias_q25[2] >> (25 - 16);
		outputs->acc_cal_q16[0] = accel_cal_q25[0] >> (25 -16);
		outputs->acc_cal_q16[1] = accel_cal_q25[1] >> (25 -16);
		outputs->acc_cal_q16[2] = accel_cal_q25[2] >> (25 -16);
		outputs->acc_accuracy_flag = Algo_InvnCalibration_AccelCalibrationGyroOptionalFxp_GetAccuracy(sCalAcc.C_buffer.buf);

		/* Update GRV algorithm with acc value and acc accuracy */
		Algo_InvnOrientation_BodyToWorldFrameFxp_UpdateAcc(sGRV.C_buffer.buf, 
				accel_cal_q25, outputs->acc_accuracy_flag);

		if (mag_is_available) {
			/* Update RV algorithm with acc value and acc accuracy */
			Algo_InvnOrientation_BodyToWorldFrameFxp_UpdateAcc(sRV.C_buffer.buf, 
					accel_cal_q25, outputs->acc_accuracy_flag);
			/* Update GeoRV with acc values and acc accuracy */
			Algo_InvnOrientation_BodyToWorldFrameFxp_UpdateAcc(sGeoRV.C_buffer.buf,
					accel_cal_q25, outputs->acc_accuracy_flag);
		}

		/*
		 * Compute the calibrated gyroscope bias tracker data based on calibrated accelerometer data
		 */
		if (hmd_features_supported) {
			Algo_InvnCalibration_GyroBiasTrackerFxp_UpdateAcc(sCalGyr.C_buffer_bt.buf, accel_cal_q25, outputs->acc_accuracy_flag);
		}
	}

	/*
	 * Compute the calibrated gyroscope data
	 */
	if (outputs->gyr_accuracy_flag != -1)
	{
		const int32_t raw_gyr_2000dps_q15[3] = {
			(inputs->sRgyro_data[0] * GYR_SENSITIVITY) / (2*2000),
			(inputs->sRgyro_data[1] * GYR_SENSITIVITY) / (2*2000),
			(inputs->sRgyro_data[2] * GYR_SENSITIVITY) / (2*2000)
		};
		int32_t gyro_uncal_2000dps_q30[3];
		int32_t gyro_offset_2000dps_q30[3];
		int32_t gyro_offset_from_bt_2000dps_q30[3];

		Algo_InvnCalibration_GyroCalibrationFxp_UpdateGyr(sCalGyr.C_buffer.buf, raw_gyr_2000dps_q15, temp_100degC);
		
		Algo_InvnCalibration_GyroCalibrationFxp_GetUncalibrated(sCalGyr.C_buffer.buf, gyro_uncal_2000dps_q30);
		outputs->gyr_uncal_q16[0] = ((gyro_uncal_2000dps_q30[0] >> 11) * 2000) >> (30 - 11 - 16);
		outputs->gyr_uncal_q16[1] = ((gyro_uncal_2000dps_q30[1] >> 11) * 2000) >> (30 - 11 - 16);
		outputs->gyr_uncal_q16[2] = ((gyro_uncal_2000dps_q30[2] >> 11) * 2000) >> (30 - 11 - 16);

		Algo_InvnCalibration_GyroCalibrationFxp_GetBias(sCalGyr.C_buffer.buf, gyro_offset_2000dps_q30);
		if (hmd_features_supported) {
			Algo_InvnCalibration_GyroBiasTrackerFxp_UpdateGyr(sCalGyr.C_buffer_bt.buf,
				gyro_uncal_2000dps_q30,
				gyro_offset_2000dps_q30,
				Algo_InvnCalibration_GyroCalibrationFxp_GetAccuracy(sCalGyr.C_buffer.buf),
				temp_100degC,
				gyro_offset_from_bt_2000dps_q30
				);

			outputs->gyr_bias_fnm_q16[0] = ((gyro_offset_2000dps_q30[0] >> 11) * 2000) >> (30 - 11 - 16);
			outputs->gyr_bias_fnm_q16[1] = ((gyro_offset_2000dps_q30[1] >> 11) * 2000) >> (30 - 11 - 16);
			outputs->gyr_bias_fnm_q16[2] = ((gyro_offset_2000dps_q30[2] >> 11) * 2000) >> (30 - 11 - 16);
			gyro_cal_2000dps_q30[0] = gyro_uncal_2000dps_q30[0]-gyro_offset_from_bt_2000dps_q30[0];
			gyro_cal_2000dps_q30[1] = gyro_uncal_2000dps_q30[1]-gyro_offset_from_bt_2000dps_q30[1];
			gyro_cal_2000dps_q30[2] = gyro_uncal_2000dps_q30[2]-gyro_offset_from_bt_2000dps_q30[2];
			outputs->gyr_bias_q16[0] = ((gyro_offset_from_bt_2000dps_q30[0] >> 11) * 2000) >> (30 - 11 - 16);
			outputs->gyr_bias_q16[1] = ((gyro_offset_from_bt_2000dps_q30[1] >> 11) * 2000) >> (30 - 11 - 16);
			outputs->gyr_bias_q16[2] = ((gyro_offset_from_bt_2000dps_q30[2] >> 11) * 2000) >> (30 - 11 - 16);
			outputs->gyr_cal_q16[0] = outputs->gyr_uncal_q16[0]-outputs->gyr_bias_q16[0];
			outputs->gyr_cal_q16[1] = outputs->gyr_uncal_q16[1]-outputs->gyr_bias_q16[1];
			outputs->gyr_cal_q16[2] = outputs->gyr_uncal_q16[2]-outputs->gyr_bias_q16[2];
			outputs->gyr_accuracy_flag = Algo_InvnCalibration_GyroBiasTrackerFxp_GetAccuracy(sCalGyr.C_buffer_bt.buf);

			int32_t cal_gyr_q16_rps[3] = {
				(int32_t) (((int64_t) outputs->gyr_cal_q16[0] * FACTOR_DPS_TO_RPS) >> 30),
				(int32_t) (((int64_t) outputs->gyr_cal_q16[1] * FACTOR_DPS_TO_RPS) >> 30),
				(int32_t) (((int64_t) outputs->gyr_cal_q16[2] * FACTOR_DPS_TO_RPS) >> 30)
			};
			/* Feed gyro-assisted calibration for Accelerometer */
			Algo_InvnCalibration_AccelCalibrationGyroOptionalFxp_UpdateGyr(sCalAcc.C_buffer.buf, 
					cal_gyr_q16_rps, period_us, outputs->gyr_accuracy_flag, chip_info);
			/* Feed gyro-assisted calibration for Magnetometer */
			if (mag_is_available)
				Algo_InvnCalibration_CompassCalibrationGyroOptionalFxp_UpdateGyr(sCalMag.C_buffer.buf,
						cal_gyr_q16_rps, period_us, outputs->gyr_accuracy_flag, chip_info);
		}
		else {
			Algo_InvnCalibration_GyroCalibrationFxp_GetCalibrated(sCalGyr.C_buffer.buf, gyro_cal_2000dps_q30);		
			outputs->gyr_bias_q16[0] = ((gyro_offset_2000dps_q30[0] >> 11) * 2000) >> (30 - 11 - 16);
			outputs->gyr_bias_q16[1] = ((gyro_offset_2000dps_q30[1] >> 11) * 2000) >> (30 - 11 - 16);
			outputs->gyr_bias_q16[2] = ((gyro_offset_2000dps_q30[2] >> 11) * 2000) >> (30 - 11 - 16);
			outputs->gyr_bias_fnm_q16[0] = outputs->gyr_bias_q16[0];
			outputs->gyr_bias_fnm_q16[1] = outputs->gyr_bias_q16[1];
			outputs->gyr_bias_fnm_q16[2] = outputs->gyr_bias_q16[2];

			outputs->gyr_cal_q16[0] = outputs->gyr_uncal_q16[0] - outputs->gyr_bias_q16[0];
			outputs->gyr_cal_q16[1] = outputs->gyr_uncal_q16[1] - outputs->gyr_bias_q16[1];
			outputs->gyr_cal_q16[2] = outputs->gyr_uncal_q16[2] - outputs->gyr_bias_q16[2];

			outputs->gyr_accuracy_flag = Algo_InvnCalibration_GyroCalibrationFxp_GetAccuracy(sCalGyr.C_buffer.buf);
		}

		/* Update GRV with gyr value and gyr accuracy */
		Algo_InvnOrientation_BodyToWorldFrameFxp_UpdateGyr(sGRV.C_buffer.buf, 
				gyro_cal_2000dps_q30, outputs->gyr_accuracy_flag);
		/* Update RV with gyr value and gyr accuracy */
		if (mag_is_available)
			Algo_InvnOrientation_BodyToWorldFrameFxp_UpdateGyr(sRV.C_buffer.buf, 
					gyro_cal_2000dps_q30, outputs->gyr_accuracy_flag);
	}

	/*
	 * Compute the game rotation vector data
	 * Note : the orientation may drift until the GRV accuracy flag reaches 3. Once calibrated, the position is kept as initial reference.
	 */
	if (outputs->acc_accuracy_flag != -1 && outputs->gyr_accuracy_flag != -1)
	{
		Algo_InvnOrientation_BodyToWorldFrameFxp_GetGameRotationVector(sGRV.C_buffer.buf, outputs->grv_quat_q30);
	}


	/*
	 * Compute the predictive quaternion data
	 */
	if(hmd_features_supported && outputs->acc_accuracy_flag != -1 && outputs->gyr_accuracy_flag != -1) {
		Algo_InvnOrientation_PredictiveQuaternionFxp_Update(sPredGRV0.C_buffer.buf, 
				gyro_cal_2000dps_q30, outputs->grv_quat_q30, outputs->predgrv0_quat_q30, chip_info);
		
		Algo_InvnOrientation_PredictiveQuaternionFxp_Update(sPredGRV1.C_buffer.buf, 
				gyro_cal_2000dps_q30, outputs->grv_quat_q30, outputs->predgrv1_quat_q30, chip_info);
	}

	/*
	 * Compute the gravity data
	 */
	if (outputs->acc_accuracy_flag != -1 && outputs->gyr_accuracy_flag != -1)
	{
		/* x axis */
		outputs->gravity_q16[0] = (2 * (int32_t)(((int64_t)outputs->grv_quat_q30[1] * outputs->grv_quat_q30[3]) >> 30)
				- 2 * (int32_t)(((int64_t)outputs->grv_quat_q30[0] * outputs->grv_quat_q30[2]) >> 30)) >> (30 - 16);
		/* y axis */
		outputs->gravity_q16[1] = (2 * (int32_t)(((int64_t)outputs->grv_quat_q30[2] * outputs->grv_quat_q30[3]) >> 30)
				+ 2 * (int32_t)(((int64_t)outputs->grv_quat_q30[0] * outputs->grv_quat_q30[1]) >> 30)) >> (30 - 16);
		/* z axis */
		outputs->gravity_q16[2] = ((1 << 30) - 2 * (int32_t)(((int64_t)outputs->grv_quat_q30[1] * outputs->grv_quat_q30[1]) >> 30)
				- 2 * (int32_t)(((int64_t)outputs->grv_quat_q30[2] * outputs->grv_quat_q30[2]) >> 30)) >> (30 - 16);
	}

	/*
	 * Compute the linear acceleration data
	 */
	if (outputs->acc_accuracy_flag != -1 && outputs->gyr_accuracy_flag != -1)
	{
		outputs->linearacc_q16[0] = outputs->acc_cal_q16[0] - outputs->gravity_q16[0];
		outputs->linearacc_q16[1] = outputs->acc_cal_q16[1] - outputs->gravity_q16[1];
		outputs->linearacc_q16[2] = outputs->acc_cal_q16[2] - outputs->gravity_q16[2];
	}

	/*
	 * Compute the rotation vector data
	 */
	if (mag_is_available) {
		int32_t heading_accuracy_q27 = Algo_InvnOrientation_BodyToWorldFrameFxp_GetRotationVector(sRV.C_buffer.buf,
			outputs->rv_quat_q30);
		outputs->rv_accuracy = (int32_t)(((int64_t)heading_accuracy_q27 * (3754936 /* 180/pi * 2^16) */)) >> 27);

		if (outputs->mag_accuracy_flag != -1) {
			/*
			 * Compute the calibrated magnetometer data
			 */
			{
				const int32_t soft_iron[9] = { 1, 0, 0,
											   0, 1, 0,
											   0, 0, 1 };
				int32_t raw_mag_ut_q16[3];
				int32_t local_field_norm_q16;
				unsigned i;

				for(i = 0; i < 3; i++) {
					raw_mag_ut_q16[i]  = (int32_t)((int64_t)soft_iron[3*i+0]*inputs->sRmag_data[0]);
					raw_mag_ut_q16[i] += (int32_t)((int64_t)soft_iron[3*i+1]*inputs->sRmag_data[1]);
					raw_mag_ut_q16[i] += (int32_t)((int64_t)soft_iron[3*i+2]*inputs->sRmag_data[2]);
					raw_mag_ut_q16[i] *= MAG_SENSITIVITY;
				}

				Algo_InvnCalibration_CompassCalibrationGyroOptionalFxp_UpdateMag(sCalMag.C_buffer.buf, raw_mag_ut_q16, outputs->mag_bias_q16);

				outputs->mag_uncal_q16[0] = raw_mag_ut_q16[0];
				outputs->mag_uncal_q16[1] = raw_mag_ut_q16[1];
				outputs->mag_uncal_q16[2] = raw_mag_ut_q16[2];
				outputs->mag_cal_q16[0] = outputs->mag_uncal_q16[0] - outputs->mag_bias_q16[0];
				outputs->mag_cal_q16[1] = outputs->mag_uncal_q16[1] - outputs->mag_bias_q16[1];
				outputs->mag_cal_q16[2] = outputs->mag_uncal_q16[2] - outputs->mag_bias_q16[2];
				outputs->mag_accuracy_flag = Algo_InvnCalibration_CompassCalibrationGyroOptionalFxp_GetAccuracy(sCalMag.C_buffer.buf);
				local_field_norm_q16 = Algo_InvnCalibration_CompassCalibrationGyroOptionalFxp_GetFieldNorm(sCalMag.C_buffer.buf);

				/* Update RV with mag values and mag accuracy */
				Algo_InvnOrientation_BodyToWorldFrameFxp_UpdateMag(sRV.C_buffer.buf,
						outputs->mag_cal_q16, outputs->mag_accuracy_flag, 0 /*the local magnetic field norm is not used for RV*/);
				/* Update GeoRV with mag values and mag accuracy */
				Algo_InvnOrientation_BodyToWorldFrameFxp_UpdateMag(sGeoRV.C_buffer.buf, 
						outputs->mag_cal_q16, outputs->mag_accuracy_flag, local_field_norm_q16);
			}
			
			/*
			 * Compute the calibrated gyroscope bias tracker data based on calibrated magnetometer data
			 */
			if (hmd_features_supported) {
				Algo_InvnCalibration_GyroBiasTrackerFxp_UpdateMag(sCalGyr.C_buffer_bt.buf,
					outputs->mag_cal_q16, outputs->mag_accuracy_flag);
			}
			
			/*
			 * Compute the geomagnetic rotation vector data
			 */
			{
				int32_t heading_accuracy_q27_2 = Algo_InvnOrientation_BodyToWorldFrameFxp_GetGeoMagRotationVector(sGeoRV.C_buffer.buf, outputs->georv_quat_q30);
				outputs->georv_accuracy = (int32_t)(((int64_t)heading_accuracy_q27_2 * (3754936 /* 180/pi * 2^16) */)) >> 27);
			}
		}
	}
}

int algorithms_set_param(int param, const uint8_t *value)
{
	int rc;
	switch (param) {
	case ALGO_PARAM_CONFIG_PRED_GRV0:
	case ALGO_PARAM_CONFIG_PRED_GRV1:
	{
		int8_t method_ennum;
		int32_t predictive_time;

		method_ennum = value[0];
		// use memcpy here to avoid unaligned word access
		memcpy(&predictive_time, &value[1], sizeof(predictive_time));

		if (ALGO_PARAM_CONFIG_PRED_GRV0 == param)
			rc = Algo_InvnOrientation_PredictiveQuaternionFxp_SetParam(sPredGRV0.C_buffer.buf, method_ennum, predictive_time);
		else /*if (ALGO_PARAM_CONFIG_PRED_GRV1 == param)*/
			rc = Algo_InvnOrientation_PredictiveQuaternionFxp_SetParam(sPredGRV1.C_buffer.buf, method_ennum, predictive_time);
		/* overwrite error code in case of error to return something meaningful to caller */
		if (rc != 0)
			rc = INV_ERROR_BAD_ARG;
		return rc;
	}
	case ALGO_PARAM_CONFIG_BT_STATE:
	{
		uint32_t enable = ((uint32_t *)value)[0];
		
		if (enable)
			Algo_InvnCalibration_GyroBiasTrackerFxp_EnableTracking(sCalGyr.C_buffer_bt.buf);
		else
			Algo_InvnCalibration_GyroBiasTrackerFxp_DisableTracking(sCalGyr.C_buffer_bt.buf);
		return 0;
	}
	default:
		return -1;
	}
}

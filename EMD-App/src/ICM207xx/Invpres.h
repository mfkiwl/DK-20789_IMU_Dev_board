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
#ifndef _INV_INVPRES_H_
#define _INV_INVPRES_H_

#include "Invn/EmbUtils/InvExport.h"
#include "Invn/EmbUtils/InvBool.h"
#include "Invn/EmbUtils/InvError.h"

#include "InvpresSerif.h"

#include <stdint.h>
#include <assert.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ICC_ADDR_PRS    (0x63)
#define INVPRES_ID_REG_EXPECTED_VALUE 0x08

typedef struct inv_invpres {
	struct inv_invpres_serif serif;
	uint32_t min_delay_us;
	uint8_t pressure_en;
	uint8_t temperature_en;
    float sensor_constants[4]; // OTP
    float p_Pa_calib[3];
    float LUT_lower;
    float LUT_upper;
    float quadr_factor;
    float offst_factor;
}inv_invpres_t;

/** @brief Reset and initialize driver states
 *  @param[in] s handle to driver states structure
 *  @param[in] serif handle to SERIF object for underlying register access
 */
static inline void inv_invpres_reset_states(struct inv_invpres * s,
		const struct inv_invpres_serif * serif)
{
	memset(s, 0, sizeof(*s));
	s->serif = *serif;
}

void init_base(struct inv_invpres * s, short *otp);
void calculate_conversion_constants(struct inv_invpres * s, float *p_Pa, float *p_LUT, float *out);
int read_id_from_i2c(struct inv_invpres * s, uint8_t * whoami);
int read_otp_from_i2c(struct inv_invpres * s, short *out);
int read_raw_pressure_temp_from_i2c(struct inv_invpres * s, int *pressure, int *temp);
int inv_invpres_process_data(struct inv_invpres * s, int p_LSB, int T_LSB, float * pressure, float * temperature);

/** @brief Initialize INVPRES : check whoami through serial interface and load compensation parameters
 */
int INV_EXPORT inv_invpres_init(struct inv_invpres * s);

/** @brief Initialize INVPRES : does complete setup.
 */
void INV_EXPORT inv_invpres_setup(struct inv_invpres * s);

/** @brief Poll INVPRES : poll the InvPrese for data
 */
void INV_EXPORT inv_invpres_poll(struct inv_invpres * s, int64_t timestamp);

/** @brief Read raw data directly from INVPRES register without any post processing
 *  @param[out] raw_data Raw data values as read from device registers
 */
//int INV_EXPORT inv_invpres_get_raw_data(struct inv_invpres * s, uint8_t raw_data[6]);

/** @brief Compute pressure and temperature values based on raw data previously read in registers
 *  @param[in] raw_data Raw data values as previously read from device registers
 *  @param[out] pressure pressure data in Pascal
 *  @param[out] temperature temperature data in Degree Celsius
 */
//int INV_EXPORT inv_invpres_process_data(struct inv_invpres * s, uint8_t raw_data[6], uint32_t * pressure, int32_t * temperature);

/** @brief Check and retrieve for new data
 *  @param[out] pressure pressure data in Pascal
 *  @param[out] temperature temperature data in Degree Celsius
 *  @return     0 on success, negative value on error
 */
int INV_EXPORT inv_invpres_get_data(struct inv_invpres * s, int * raw_pressure, int * raw_temperature, float * pressure, float * temperature);

/** @brief Enables / disables the invpres sensor for both pressure and temperature
 * @param[in] enable			0=off, 1=on
 * @return 0 in case of success, negative value on error
 */
int INV_EXPORT inv_invpres_enable_sensor(struct inv_invpres * s, inv_bool_t en);

/** @brief Enables / disables the invpres sensor for pressure
 * @param[in] enable			0=off, 1=on
 * @return 0 in case of success, negative value on error
 */
int INV_EXPORT inv_invpres_pressure_enable_sensor(struct inv_invpres * s, inv_bool_t en);

/** @brief Enables / disables the invpres sensor for temperature
 * @param[in] enable			0=off, 1=on
 * @return 0 in case of success, negative value on error
 */
int INV_EXPORT inv_invpres_temperature_enable_sensor(struct inv_invpres * s, inv_bool_t en);

/** @brief return WHOAMI value
 *  @param[out] whoami WHOAMI for device
 *  @return     0 on success, negative value on error
 */
int INV_EXPORT inv_invpres_get_whoami(struct inv_invpres * s, uint8_t * whoami);

/** @brief Send soft reset
 *  @return     0 on success, negative value on error
 */
int INV_EXPORT inv_invpres_soft_reset(struct inv_invpres * s);

/** @brief Hook for low-level system sleep() function to be implemented by upper layer
 *  @param[in] ms number of millisecond the calling thread should sleep
 */
extern void inv_invpres_sleep(int ms);

/** @brief Hook for low-level high res system sleep() function to be implemented by upper layer
 *  ~100us resolution is sufficient
 *  @param[in] us number of us the calling thread should sleep
 */
extern void inv_invpres_sleep_us(int us);

/** @brief Hook for low-level system time() function to be implemented by upper layer
 *  @return monotonic timestamp in us
 */
extern uint64_t inv_invpres_get_time_us(void);

/** @brief Hook called when magnetometer is started, implemented by upper layer which knows about timers
 *  @param[in] period sensor period in micro-second
 *  @return 0 on success, -1 on failure
 */
extern int inv_invpres_start_timer_us(uint32_t period);

/** @brief Hook called when magnetometer is stopped, implemented by upper layer which knows about timers
 *  @return 0 on success, -1 on failure
 */
extern int inv_invpres_stop_timer(void);

/** @brief Hook called when magnetometer odr is updated, implemented by upper layer which knows about timers
 *  @param[in] period sensor period in micro-second
 *  @return 0 on success, -1 on failure
 */
extern int inv_invpres_reconfigure_timer_us(uint32_t period);

#ifdef __cplusplus
}
#endif

#endif /* _INV_INVPRES_H_ */


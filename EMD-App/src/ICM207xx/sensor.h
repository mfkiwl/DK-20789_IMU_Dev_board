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
/*
 * Example configuration
 */
/* On Atmel platform, Slave Address should be set to 0x68 as pin SA0 is logic low */
#define PRES_I2C_ADDR			0x63 /* I2C slave address for InvPres */
#define ICM_I2C_ADDR_REVA		0x68 /* I2C slave address for INV device on Rev A board */
#define ICM_I2C_ADDR_REVB		0x69 /* I2C slave address for INV device on Rev B board */
#define DATA_ACCURACY_MASK		((uint32_t)0x7)

#define MIN_ODR_US				5000		// 5 ms sample spacing = 200 HZ
#define MAX_ODR_US				200000		// 200 ms sample spacing = 5 HZ

#define TIMER_200MHz			200

/* UART buffer size configurations */
#define UART_LOG_FIFO_SIZE		4096
#define UART_RX_FIFO_SIZE		256
#define UART_TX_FIFO_SIZE		4096

#define DEFAULT_PRESSURE_ODR	25

/* 
 * Sensor identifier for control function
 */
enum sensor {
	SENSOR_RAW_ACC,
	SENSOR_RAW_GYR,
	SENSOR_ACC,
	SENSOR_GYR,
	SENSOR_UGYR,
	SENSOR_GRV,
	SENSOR_PREDQUAT_0,
	SENSOR_GRA,
	SENSOR_LINACC,
	SENSOR_PREDQUAT_1,
	SENSOR_CUSTOM_PRESSURE,
	SENSOR_MAX
};

typedef struct sensor_odr {
	uint32_t min;
	uint32_t max;
} sensor_odr_t;

extern int sensor_control(int enable);
extern int sensor_configure_odr(uint32_t odr_us, uint32_t sensor_mask);
int Icm207xx_data_poll(void);
extern int icm207xx_sensor_setup(void);
extern int icm207xx_sensor_configuration(void);
extern void iddwrapper_protocol_event_cb(enum DynProtocolEtype etype, enum DynProtocolEid eid, const DynProtocolEdata_t * edata, void * cookie);
extern void iddwrapper_transport_event_cb(enum DynProTransportEvent e, union DynProTransportEventData data, void * cookie);
extern int InvEMDFrontEnd_acknowledgeReset(void);
void InvEMDFrontEnd_busyWaitUsHook(uint32_t us);
int InvEMDFrontEnd_isHwFlowCtrlSupportedHook(void);
int InvEMDFrontEnd_putcharHook(int c);
extern enum sensor idd_sensortype_conversion(int sensor);
extern DynProtocol_t protocol;
extern DynProTransportUart_t transport;
extern uint32_t period_us;
extern inv_icm207xx_t icm_device;
//extern inv_invpres_t invpres_device;
extern uint32_t enabled_sensor_mask;






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

static InvSchedulerTask blinkerLedTask;
static InvSchedulerTask commandHandlerTask;
extern algo_output outputs;
extern uint8_t chip_info[3];

static void msg_printer(int level, const char * str, va_list ap);

/*
 * BlinkerLedTaskMain - Task that blinks the LED.
 */
static void BlinkerLedTaskMain(void * arg)
{
	(void)arg;

	ioport_toggle_pin_level(LED_0_PIN);
}

/*
 * CommandHandlerTaskMain - Task that monitors the UART
 */
static void CommandHandlerTaskMain(void * arg)
{
	(void)arg;

	int byte;
	do {
		byte = EOF;
		
		__disable_irq();
		if(!RingByteBuffer_isEmpty(&uart_rx_rb)) {
			byte = RingByteBuffer_popByte(&uart_rx_rb);
		}
		__enable_irq();

		if(byte != EOF) {
			DynProTransportUart_rxProcessByte(&transport, (uint8_t)byte);
		}
	}
	while(byte != EOF);
}

int main (void)
{	
	int useSpiNotI2C = USE_SPI_NOT_I2C;
	
	/* Hardware initialization */
	sysclk_init();
	board_init(); 
	sysclk_enable_peripheral_clock(ID_TC0);	
	
	/* Configure Device - Host Interface */
	configure_console();
	
	/* Setup message logging */
	INV_MSG_SETUP(INV_MSG_ENABLE, msg_printer);
	
	INV_MSG(INV_MSG_LEVEL_INFO, "###################################");
	INV_MSG(INV_MSG_LEVEL_INFO, "#   ICM207xx LITE example   #");
	INV_MSG(INV_MSG_LEVEL_INFO, "###################################");
	
	/* Initialize External Sensor Interrupt */
	ext_int_initialize(&ext_interrupt_handler);

	interface_initialize();

	/* Configure sysTick Timer */
	SysTick_Config(sysclk_get_cpu_hz() / MILLISECONDS_PER_SECOND);	 
	
	/* 
	 * Initialize icm207xx serif structure 
	 */
	struct inv_icm207xx_serif icm207xx_serif;
	icm207xx_serif.context   = 0; /* no need */
	icm207xx_serif.read_reg  = idd_io_hal_read_reg;
	icm207xx_serif.write_reg = idd_io_hal_write_reg;
	icm207xx_serif.max_read  = 1024*16; /* maximum number of bytes allowed per serial read */
	icm207xx_serif.max_write = 1024*16; /* maximum number of bytes allowed per serial write */
	/*
	 * Init SPI/I2C communication
	 */
	if(useSpiNotI2C) {
		icm207xx_serif.is_spi    = 1;
		INV_MSG(INV_MSG_LEVEL_INFO, "Opening serial interface through SPI");
	} else {
		icm207xx_serif.is_spi    = 0;
		INV_MSG(INV_MSG_LEVEL_INFO, "Opening serial interface through I2C");
	}

	/* 
	 * Reset icm207xx driver states
	 */
	inv_icm207xx_reset_states(&icm_device, &icm207xx_serif);
	
	/*
	 * Setup the icm207xx device
	 */
	icm207xx_sensor_setup();
	icm207xx_sensor_configuration();

	/* 
	* Initialize invpres serif structure 
	*/
	struct inv_invpres_serif invpres_serif;

	invpres_serif.context   = 0; /* no need */
	invpres_serif.read_reg  = invpres_hal_read_reg;
	invpres_serif.write_reg = invpres_hal_write_reg;
	invpres_serif.max_read  = 64; /* maximum number of bytes allowed per serial read */
	invpres_serif.max_write = 64; /* maximum number of bytes allowed per serial write */
	invpres_serif.is_spi    = 0;

	/* 
	 * Reset invpres driver states 
	 */
	inv_invpres_reset_states(&invpres_device, &invpres_serif);

	inv_invpres_setup(&invpres_device);
	/*
	 * Initialize Dynamic protocol stuff
	 */
	DynProTransportUart_init(&transport, iddwrapper_transport_event_cb, 0);
	DynProtocol_init(&protocol, iddwrapper_protocol_event_cb, 0);

	/* Initialize algorithms */
	algorithms_init(outputs.acc_bias_q16, outputs.gyr_bias_fnm_q16, outputs.mag_bias_q16, chip_info);

	/*
	 * Initializes the default sensor ODR in order to properly init the algorithms
	 */
	sensor_configure_odr(period_us, 0);
	
	algorithms_configure_odr(period_us, 0);
	
	/*
	 * At boot time, all sensors are turned on.
	 */
	algorithms_sensor_control(1);
	sensor_control(1);

	InvScheduler_init(&scheduler);
	InvScheduler_initTask(&scheduler, &commandHandlerTask, "commandHandlerTask", CommandHandlerTaskMain, 0, INVSCHEDULER_TASK_PRIO_MIN, 1);
	InvScheduler_initTask(&scheduler, &blinkerLedTask, "blinkerLedTask", BlinkerLedTaskMain, 0, INVSCHEDULER_TASK_PRIO_MIN+1, 1000000/SCHEDULER_PERIOD);

	InvScheduler_startTask(&blinkerLedTask, 0);
	InvScheduler_startTask(&commandHandlerTask, 0);

	InvEMDFrontEnd_acknowledgeReset();
	
	hw_timer_start(20);		// Start the timestamp timer at 20 Hz.
	while (1) {
		InvScheduler_dispatchTasks(&scheduler);
		
		if (irq_from_device == 1) {
			Icm207xx_data_poll(); 

			__disable_irq();
			irq_from_device = 0;
			__enable_irq();
		}

		if (irq_start_pressure_capture == 1) {
			uint64_t timestamp = InvEMDFrontEnd_getTimestampUs();
			
			if (enabled_sensor_mask & (1 << idd_sensortype_conversion(INV_SENSOR_TYPE_CUSTOM_PRESSURE))) 
			    inv_invpres_poll(&invpres_device, timestamp);
			
			__disable_irq();
			irq_start_pressure_capture = 0;
			__enable_irq();
		}

	}	
	return 0;
}

/*
 * Printer function for message facility
 */
static void msg_printer(int level, const char * str, va_list ap)
{
#ifdef INV_MSG_ENABLE
	static char out_str[256]; /* static to limit stack usage */
	unsigned idx = 0;
	const char * ptr = out_str;
	const char * s[INV_MSG_LEVEL_MAX] = {
		"",    // INV_MSG_LEVEL_OFF
		"[E] ", // INV_MSG_LEVEL_ERROR
		"[W] ", // INV_MSG_LEVEL_WARNING
		"[I] ", // INV_MSG_LEVEL_INFO
		"[V] ", // INV_MSG_LEVEL_VERBOSE
		"[D] ", // INV_MSG_LEVEL_DEBUG
	};
	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "%s", s[level]);
	if(idx >= (sizeof(out_str)))
		return;
	idx += vsnprintf(&out_str[idx], sizeof(out_str) - idx, str, ap);
	if(idx >= (sizeof(out_str)))
		return;
	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "\r\n");
	if(idx >= (sizeof(out_str)))
		return;

	while(*ptr != '\0') {
		usart_serial_putchar(DEBUG_UART, *ptr);
		++ptr;
	}
#else

	(void)level, (void)str, (void)ap;

#endif
}

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
/** external interrupt pin definition */
#define PIN_EXT_INTERRUPT       {PIO_PB3, PIOB, ID_PIOB, PIO_INPUT, PIO_DEFAULT | PIO_IT_RISE_EDGE}
#define PIN_EXT_INTERRUPT_MASK  PIO_PB3
#define PIN_EXT_INTERRUPT_PIO   PIOB
#define PIN_EXT_INTERRUPT_ID    ID_PIOB
#define PIN_EXT_INTERRUPT_TYPE  PIO_INPUT
#define PIN_EXT_INTERRUPT_ATTR  (PIO_DEFAULT | PIO_IT_RISE_EDGE)
#define PIN_EXT_INTERRUPT_IRQn  PIOB_IRQn

/* 
 * Select communication between Atmel and INV device
 */
#define USE_SPI_NOT_I2C			0

/* 
 * Select communication between Atmel and INV device by setting 0/1 to one of the following defines
 */
#define SERIF_TYPE_SPI			(USE_SPI_NOT_I2C)
#define SERIF_TYPE_I2C			!(USE_SPI_NOT_I2C)

/* The system timer period in microseconds */
#define SYSTIMER_PERIOD_US		10
#define MILLISECONDS_PER_SECOND	1000
#define MICROSECONDS_PER_50ms	50000	

/* On Atmel platform, Slave Address should be set to 0x68 as pin SA0 is logic low */
#define PRES_I2C_ADDR			0x63 /* I2C slave address for InvPres */
#define ICM_I2C_ADDR_REVA		0x68 /* I2C slave address for INV device on Rev A board */
#define ICM_I2C_ADDR_REVB		0x69 /* I2C slave address for INV device on Rev B board */
#define DATA_ACCURACY_MASK		((uint32_t)0x7)

#define MIN_ODR_US				5000		// 5 ms sample spacing = 200 HZ
#define MAX_ODR_US				200000		// 200 ms sample spacing = 5 HZ

/* UART buffer size configurations */
#define UART_LOG_FIFO_SIZE		4096
#define UART_RX_FIFO_SIZE		256
#define UART_TX_FIFO_SIZE		4096

extern void configure_console(void);
unsigned long i2c_master_read_register(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue);
unsigned long i2c_master_write_register(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue);
unsigned long i2c_pres_read_register(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue);
unsigned long i2c_pres_write_register(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue);
int idd_io_hal_read_reg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
int idd_io_hal_write_reg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
int invpres_hal_read_reg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
int invpres_hal_write_reg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
extern void hw_timer_start(uint32_t timer_freq);
extern void hw_timer_stop(void);
extern void ext_int_initialize(void (*handler_function)(void));
extern void interface_initialize(void);
extern void ext_interrupt_handler(void);
extern void set_pressure_timer(uint32_t new_timer_val);
extern uint64_t InvEMDFrontEnd_getTimestampUs(void);

extern RingByteBuffer uart_rx_rb;
extern volatile int irq_from_device;
extern volatile int irq_start_pressure_capture;
extern InvScheduler 	scheduler;
extern volatile uint32_t ul_ticks;
extern uint8_t I2C_Address;

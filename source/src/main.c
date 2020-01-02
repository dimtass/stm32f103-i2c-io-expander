/**
 * This is a demo for the ILI9341 spi display.
 * 
 * st-flash --reset write build-stm32/stm32f103-ili9341-dma.bin 0x8000000
 * 
 *  Created on: Jul 5, 2018
 *      Author: Dimitris Tassopoulos
*/

#include <stdio.h>
#include "stm32f10x.h"
#include "various_defs.h"
#include "debug_trace.h"
#include "mod_led.h"
#include "timer_sched.h"
#include "dev_i2c.h"
#include "states.h"
#include "dev_uart.h"
#include "dev_gpio.h"
#ifdef USE_STTERM
#include "stlinky.h"
#endif

#define LED_TIMER_MS 500

#define LED_PORT GPIOC
#define LED_PIN GPIO_Pin_13

#define INT_PIN_PORT	GPIOB
#define INT_PIN	GPIO_Pin_5

volatile uint32_t glb_tmr_1ms;
volatile uint32_t tmr_1sec;
struct i2c_client i2c;

uint8_t i2c_dummy_register = 0;

/* I2C machine states */
enum en_i2c_states {
	SM_IDLE,
	SM_I2C_READ_REGISTER,
	SM_I2C_WRITE_REGISTER,
};
struct tp_state i2c_states_list[] = {
	[SM_IDLE] = DECLARE_STATE(SM_IDLE, NULL, NULL, NULL),
    [SM_I2C_READ_REGISTER] = DECLARE_STATE(SM_I2C_READ_REGISTER, NULL, NULL, NULL),
    [SM_I2C_WRITE_REGISTER] = DECLARE_STATE(SM_I2C_WRITE_REGISTER, NULL, NULL, NULL),
};
DECLARE_STATE_OBJ(i2c_sm, NULL, i2c_states_list, ARRAY_SIZE(i2c_states_list),
		&i2c_states_list[SM_IDLE], &i2c_states_list[SM_IDLE]);
uint8_t i2c_cmd;	// last i2c command

uint32_t trace_levels;
#define TRACE_LEVEL_I2C 	(1 << 1)
#define TRACE_LEVEL_SM		(1 << 2)

/* Create the list head for the timer */
static LIST_HEAD(obj_timer_list);

/* dummy i2c registers */
#define I2C_REG_PREAMBLE 		0xBEEF
#define I2C_CONFIG_MASK_IO 		0x01
#define I2C_CONFIG_MASK_INIT 	0x02
#define I2C_CONFIG_MASK_INVERT 	0x04
#define I2C_NUM_OF_GPIOS		8
enum i2c_reg_index {
	I2C_REG_INDEX_PREAMBLE_1 = 0,	// Read-only
	I2C_REG_INDEX_PREAMBLE_2,		// Read-only
	I2C_REG_INDEX_ACTIVATE,			// Write-only (always 0 when read)
	I2C_REG_INDEX_IO_PORT = I2C_REG_INDEX_ACTIVATE + 1,				// Nx 1 byte per port
	I2C_REG_INDEX_IO_PIN = (I2C_REG_INDEX_IO_PORT + (I2C_NUM_OF_GPIOS * 1)),		// Nx 1 byte per pin
	I2C_REG_INDEX_IO_CONFIG = (I2C_REG_INDEX_IO_PIN + (I2C_NUM_OF_GPIOS * 1)),		// Nx 1 byte per value (b0: I/O, b1: init_value, b2: invert)
	I2C_REG_INDEX_IO_VALUE = (I2C_REG_INDEX_IO_CONFIG + (I2C_NUM_OF_GPIOS * 1)),	// Nx 1 byte per value
	I2C_REG_LEN = I2C_REG_INDEX_IO_VALUE + (I2C_NUM_OF_GPIOS * 1)
};
uint8_t i2c_regs[I2C_REG_LEN] = {0xFF};

/* array of gpio pointers */
struct tp_gpio * gpios[I2C_NUM_OF_GPIOS] = {0};

#ifdef USE_DBGUART
// Declare uart
DECLARE_UART_DEV(dbg_uart, USART1, 115200, 256, 10, 1);
#endif

/* Callbacks */
void dbg_uart_parser(uint8_t *buffer, size_t bufferlen, uint8_t sender);
uint8_t i2c_interrupt(struct i2c_client * i2c, enum i2c_event event, uint8_t * buffer, uint16_t index);

#ifdef USE_SEMIHOSTING
extern void initialise_monitor_handles(void);
#endif

void led_on(void *data)
{
	LED_PORT->ODR |= LED_PIN;
}

void led_off(void *data)
{
	LED_PORT->ODR &= ~LED_PIN;
}

void led_init(void *data)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = LED_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(LED_PORT, &GPIO_InitStructure);

	LED_PORT->ODR |= LED_PIN;
}

static inline void main_loop(void)
{
	/* 1 ms timer */
	if (glb_tmr_1ms) {
		glb_tmr_1ms = 0;
		mod_timer_polling(&obj_timer_list);
	}
	state_handler(&i2c_sm);
}

int main(void)
{
	if (SysTick_Config(SystemCoreClock / 1000)) {
		/* Capture error */
		while (1);
	}

	/* enable/disable traces */
	trace_levels_set(
			0
			| TRACE_LEVEL_DEFAULT
			// | TRACE_LEVEL_I2C
			,1);
			
#ifdef USE_SEMIHOSTING
	initialise_monitor_handles();
#elif USE_STTERM
	stlinky_init();
#elif USE_DBGUART
	// setup uart port
	dev_uart_add(&dbg_uart);
	/* set callback for uart rx */
	dbg_uart.fp_dev_uart_cb = dbg_uart_parser;
 	mod_timer_add((void*) &dbg_uart, 5, (void*) &dev_uart_update, &obj_timer_list);
#endif

	/* Declare LED module and initialize it */
	DECLARE_MODULE_LED(led_module, 8, 250);
	mod_led_init(&led_module);
	mod_timer_add((void*) &led_module, led_module.tick_ms,
				(void*) &mod_led_update, &obj_timer_list);

	/* Declare LED */
	DECLARE_DEV_LED(def_led, &led_module, 1, NULL, &led_init, &led_on, &led_off);
	dev_led_add(&def_led);
	dev_led_set_pattern(&def_led, 0b11001100);

	/* Initialize I2C */
	i2c_init(DEV_I2C1, 0x08, 100000, &i2c_interrupt, &i2c);
	i2c_enable(&i2c);

	/* Initialize I2C registers */
	i2c_regs[0] = I2C_REG_PREAMBLE >> 8;
	i2c_regs[1] = I2C_REG_PREAMBLE & 0xFF;
	i2c_regs[2] = 0;

	/* Declare default gpios */
	DECLARE_GPIO(gpio_0, 0xFF, 0xFF, GPIO_DIR_OUTPUT, 0, 0);
	DECLARE_GPIO(gpio_1, 0xFF, GPIO_Pin_0, GPIO_DIR_OUTPUT, 0, 0);
	DECLARE_GPIO(gpio_2, 0xFF, GPIO_Pin_0, GPIO_DIR_OUTPUT, 0, 0);
	DECLARE_GPIO(gpio_3, 0xFF, GPIO_Pin_0, GPIO_DIR_OUTPUT, 0, 0);
	DECLARE_GPIO(gpio_4, 0xFF, GPIO_Pin_0, GPIO_DIR_OUTPUT, 0, 0);
	DECLARE_GPIO(gpio_5, 0xFF, GPIO_Pin_0, GPIO_DIR_OUTPUT, 0, 0);
	DECLARE_GPIO(gpio_6, 0xFF, GPIO_Pin_0, GPIO_DIR_OUTPUT, 0, 0);
	DECLARE_GPIO(gpio_7, 0xFF, GPIO_Pin_0, GPIO_DIR_OUTPUT, 0, 0);
	gpios[0] = &gpio_0;
	gpios[1] = &gpio_1;
	gpios[2] = &gpio_2;
	gpios[3] = &gpio_3;
	gpios[4] = &gpio_4;
	gpios[5] = &gpio_5;
	gpios[6] = &gpio_6;
	gpios[7] = &gpio_7;

	state_change(&i2c_sm, SM_IDLE);
	TRACE(("Program started. SystemCoreClock: %lu\n", SystemCoreClock));

	while(1) {
		main_loop();
	}
}

static inline void parse_i2c_data(const uint8_t reg, uint8_t value)
{
	if (reg < I2C_REG_LEN) {
		// TRACE(("parse_i2c_data: %d,%d\n", reg, value));

		/* Need to change value? */
		if (reg >= I2C_REG_INDEX_IO_VALUE) {
			uint8_t index = reg - I2C_REG_INDEX_IO_VALUE;
			// TRACE(("index:%d,reg:%02X,val:%02X\n", index, reg, i2c_regs[reg]));
			if (value) {
				gpio_set(gpios[index]);
				i2c_regs[reg] = 0x01;
				// TRACE(("Setting pin %d\n", index));
			}
			else {
				gpio_reset(gpios[index]);
				i2c_regs[reg] = 0;
				// TRACE(("Reset pin %d\n", index));
			}
		}
		/* Set configuration */
		else if (reg >= I2C_REG_INDEX_IO_CONFIG) {
			uint8_t index = reg - I2C_REG_INDEX_IO_CONFIG;
			gpios[index]->dir = (value & I2C_CONFIG_MASK_IO) ? GPIO_DIR_OUTPUT : GPIO_DIR_INPUT;
			gpios[index]->init_value = (value & I2C_CONFIG_MASK_INIT) ? 1 : 0;
			gpios[index]->invert = (value & I2C_CONFIG_MASK_INVERT) ? 1 : 0;
			i2c_regs[reg] = value;
		}
		/* Change pin num */
		else if (reg >= I2C_REG_INDEX_IO_PIN) {
			uint8_t index = reg - I2C_REG_INDEX_IO_PIN;
			gpios[index]->pin = (1 << value);
			i2c_regs[reg] = value;
		}
		/* Change port */
		else if (reg >= I2C_REG_INDEX_IO_PORT) {
			uint8_t index = reg - I2C_REG_INDEX_IO_PORT;
			gpios[index]->port = value;
			i2c_regs[reg] = value;
		}
		/* Activate changes */
		else if (reg == I2C_REG_INDEX_ACTIVATE) {
			i2c_regs[reg] = 0;
			/* init only the given gpio */
			if (value < I2C_NUM_OF_GPIOS)
				gpio_init(gpios[value]);
			/* init all gpios */
			if (value == 0xFF) {
				for (int i=0; i<I2C_NUM_OF_GPIOS; i++) {
					gpio_init(gpios[i]);
				}
			}
		}
	}
}

uint8_t i2c_interrupt(struct i2c_client * i2c, enum i2c_event event, uint8_t *buff, uint16_t buff_len)
{
	uint8_t resp = 0;
	
	TRACEL(TRACE_LEVEL_I2C, ("evt:%d,b:0x%02X,l:%d\n", event, buff[0], buff_len));
	switch (event) {
	case I2C_SLAVE_ADDRESSED_REQ: {
		uint8_t reg = buff[0];
		if (reg < I2C_REG_LEN) {
			buff[0] = i2c_regs[reg];
			TRACE(("Reading I2C register [0x%02X]=0x%02X\n", reg, i2c_regs[reg]));
			resp = 1;
		}
		break;
	}
	case I2C_SLAVE_WRITE_REQUESTED: {
		TRACE(("Setting I2C register [0x%02X]=0x%02X\n", buff[0], buff[1]));
		parse_i2c_data(buff[0], buff[1]);
		break;
	}
	default:
		break;
	};
	return resp;
}

void dbg_uart_parser(uint8_t *buffer, size_t bufferlen, uint8_t sender)
{
	buffer[bufferlen] = 0;
	TRACE(("dbg_uart_parser: %s\n", buffer));
}
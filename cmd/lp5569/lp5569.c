/*
 * Control LP5569 over i2c
 *
 * Copyright (c) 2023 Workaround GmbH
 *
 */

#include <common.h>
#include <command.h>
#include <errno.h>
#include <dm.h>
#include <log.h>
#include <malloc.h>
#include <asm/gpio.h>
#include <linux/err.h>
#include <linux/delay.h>

#include "i2c_bitbang.h"

#define LP5569_MAX_PROGRAM_LEN 32

#define LP5569_MEM_PROGRAM_LOCATION 0x00
#define LP5569_MEM_LEDMUX_LOCATION 0x10

#define LP5569_I2C_ADDR 0x64

#define LP5569_REG_CONFIG 0x00
#define LP5569_REG_MISC 0x2F
#define LP5569_REG_RESET 0x3F
#define LP5569_REG_LED_PWM_BASE 0x16
#define LP5569_REG_ENGINE1_PROG_START 0x4B
#define LP5569_REG_LED_ENGINE_CONTROL1 0x01
#define LP5569_REG_LED_ENGINE_CONTROL2 0x02
#define LP5569_REG_PROG_MEM_PAGE_SELECT 0x4F
#define LP5569_REG_PROGRAM_MEM_00 0x50

#define LP5569_REG_LED0_CURRENT 0x22
#define LP5569_REG_LED1_CURRENT 0x23
#define LP5569_REG_LED2_CURRENT 0x24
#define LP5569_REG_LED3_CURRENT 0x25
#define LP5569_REG_LED4_CURRENT 0x26
#define LP5569_REG_LED5_CURRENT 0x27
#define LP5569_REG_LED6_CURRENT 0x28
#define LP5569_REG_LED7_CURRENT 0x29
#define LP5569_REG_LED8_CURRENT 0x2A

#define RED_CURRENT 10
#define GREEN_CURRENT 3
#define BLUE_CURRENT 8

// The LED mux table contains rows of 16 bytes. Each row indicates which
// connected LEDs are mapped to the execution engine and can be selected
// sequentially by the actual program.
//
// LED No.:         8 7654 3210
// RGB LED:         3 3322 2111
// COLOR:           B GRBG RBGR
// Bitmask: 0000 0000 0000 0000
static const uint8_t led_mux_tapes[2][LP5569_MAX_PROGRAM_LEN] = {
	{
		// 0000 0000 0000 0010 -> LED 1 selected, green of first RGB
		0x00, 0x02,
		// 0000 0000 0010 0000 -> LED 5 selected, blue of second RGB
		0x00, 0x20,
		// 0000 0000 1000 0000 -> LED 7 selected, green of third RGB
		0x00, 0x80,
		// 0000 0000 0000 0100 -> LED 2 selected, blue of first RGB
		0x00, 0x04,
		// 0000 0000 0001 0000 -> LED 4 selected, green of second RGB
		0x00, 0x10,
		// 0000 0001 0000 0000 -> LED 8 selected, blue of third RGB
		0x01, 0x00,
	},
	{
		// 0000 0000 0100 1001 -> LED 0/3/6 selected 3 red
		0x00, 0x49,
		0x00, 0x49,
		0x00, 0x49,
		0x00, 0x49,
		0x00, 0x49,
		0x00, 0x49,
	}
};

// The actual program instructions can be found in the data sheet. A
// short explanaion is given for all instructions. The fixed instruction
// bits are marked with x, the variable ones are explained.
//
// First the led mux table is loaded, then the selected LEDs are turned
// on, after a wait of ~1 second, they are turned off again, the next
// row in the LED mux table is selected and a jump to the third
// instruction is performed. Since the LED mux table is loaded by both
// start and end address, it just wraps around and can run forever.
static const uint8_t led_prog[LP5569_MAX_PROGRAM_LEN] = {
	// xxxx xxxx x
	// 1001 1100 0001 0000 -> map_start from address 0x10
	0x9c, LP5569_MEM_LEDMUX_LOCATION,
	// xxxx xxxx x
	// 1001 1100 1001 0101 -> load_end at address 0x15
	0x9c, 0x95,
	// xxxx xxxx
	// 0100 0000 1111 1111 -> set_pwm to 0xFF
	0x40, 0xff,
	// x       x xxxx xxxx
	// 0111 1110 0000 0000 -> wait with prescale on and 31 cycles
	//			  wait time. -> 484ms
	0x7E, 0x00,
	// xxxx xxxx
	// 0100 0000 0000 0000 -> set_pwm to 0x00
	0x40, 0x00,
	// xxxx xxxx xxxx xxxx
	// 1001 1101 1000 0000 -> map_next: next row in led mux loaded
	0x9D, 0x80,
	// xxx
	// 1010 0000 0000 0010 -> branch infinitely to instruction 0x02
	0xA0, 0x02,
	// Fill with 0s.
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};


static int lp5569_set_register(struct gpio_i2c *bus, uint8_t reg, uint8_t val)
{
	return i2c_set_register(bus, LP5569_I2C_ADDR, reg, val);
}

/*
 * @brief Write a number of bytes to adjacent registers.
 *
 * @param reg	   The start register used as base.
 * @param buf 	   A pointer to the values to write.
 * @param buf_size The number of bytes to write.
 */
static int lp5569_write_bulk(struct gpio_i2c *bus,
							 uint8_t reg, const uint8_t *buf, uint8_t buf_size)
{
	int ret = 0;

	for (int i = 0; i < buf_size; i++) {
		ret = lp5569_set_register(bus, reg + i, buf[i]);
		if (ret) {
			printf("failed to write register in bulk operation: %d\n", ret);
			return ret;
		}

	}

	return 0;
}

static int lp5569_engine_load_program(struct gpio_i2c *bus) {
	return lp5569_set_register(bus, LP5569_REG_LED_ENGINE_CONTROL2, 0x54);
}

static int lp5569_engine_halt(struct gpio_i2c *bus) {
	return lp5569_set_register(bus, LP5569_REG_LED_ENGINE_CONTROL2, 0x00);
}

static int lp5569_engine_run(struct gpio_i2c *bus) {
	int ret;

	ret = lp5569_set_register(bus, LP5569_REG_LED_ENGINE_CONTROL2, 0x80);
	if (ret) {
		printf("failed to set Engine 1 to run mode: %d\n", ret);
		return ret;
	}

	ret = lp5569_set_register(bus, LP5569_REG_LED_ENGINE_CONTROL1, 0x80);
	if (ret) {
		printf("failed to set Engine 1 to free run mode: %d\n", ret);
		return ret;
	}

	return 0;
}

static int lp5569_mem_page_select(struct gpio_i2c *bus, bool is_mux) {
	uint8_t page;

	if (is_mux) {
		page = 0x01;
	} else {
		page = 0x00;
	}

	return lp5569_set_register(bus, LP5569_REG_PROG_MEM_PAGE_SELECT, page);
}

static int lp5569_init(struct gpio_i2c *bus) {
	int ret;

	ret = lp5569_set_register(bus, LP5569_REG_RESET, 0xFF);
	if (ret) {
		printf("failed to reset LED chip: %d\n", ret);
		return ret;
	}

	ret = lp5569_set_register(bus, LP5569_REG_CONFIG, 1 << 6);
	if (ret) {
		printf("failed to enable chip: %d\n", ret);
		return ret;
	}

	// 1: Internal 32-kHz oscillator
	// 2-3: auto mode
	// 6: auto increment i2c address
	ret = lp5569_set_register(bus, LP5569_REG_MISC, 1 << 0 | 3 << 3 | 1 << 6);
	if (ret) {
		printf("failed to configure chip: %d\n", ret);
		return ret;
	}

	ret = lp5569_set_register(bus, LP5569_REG_ENGINE1_PROG_START, 0x00);
	if (ret) {
		printf("failed to set engine program start address: %d\n", ret);
		return ret;
	}

	// Red current
	ret = lp5569_set_register(bus, LP5569_REG_LED0_CURRENT, RED_CURRENT);
	if (ret) {
		printf("failed to set current: %d\n", ret);
		return ret;
	}
	ret = lp5569_set_register(bus, LP5569_REG_LED3_CURRENT, RED_CURRENT);
	if (ret) {
		printf("failed to set current: %d\n", ret);
		return ret;
	}
	ret = lp5569_set_register(bus, LP5569_REG_LED6_CURRENT, RED_CURRENT);
	if (ret) {
		printf("failed to set current: %d\n", ret);
		return ret;
	}

	// Green current
	ret = lp5569_set_register(bus, LP5569_REG_LED1_CURRENT, GREEN_CURRENT);
	if (ret) {
		printf("failed to set current: %d\n", ret);
		return ret;
	}
	ret = lp5569_set_register(bus, LP5569_REG_LED4_CURRENT, GREEN_CURRENT);
	if (ret) {
		printf("failed to set current: %d\n", ret);
		return ret;
	}
	ret = lp5569_set_register(bus, LP5569_REG_LED7_CURRENT, GREEN_CURRENT);
	if (ret) {
		printf("failed to set current: %d\n", ret);
		return ret;
	}


	// Blue current
	ret = lp5569_set_register(bus, LP5569_REG_LED2_CURRENT, BLUE_CURRENT);
	if (ret) {
		printf("failed to set current: %d\n", ret);
		return ret;
	}
	ret = lp5569_set_register(bus, LP5569_REG_LED5_CURRENT, BLUE_CURRENT);
	if (ret) {
		printf("failed to set current: %d\n", ret);
		return ret;
	}
	ret = lp5569_set_register(bus, LP5569_REG_LED8_CURRENT, BLUE_CURRENT);
	if (ret) {
		printf("failed to set current: %d\n", ret);
		return ret;
	}

	return 0;
}

static int lp5569_run_program(struct gpio_i2c *bus,
							  const uint8_t *led_mux_buf,
							  uint8_t led_mux_buf_len,
							  const uint8_t *prog_buf,
							  uint8_t prog_buf_len) {
	int ret;

	if ((led_mux_buf_len > LP5569_MAX_PROGRAM_LEN) ||
		(prog_buf_len > LP5569_MAX_PROGRAM_LEN)) {
		printf("only 32 bytes for program size and LED mux info\n");
		return -1;
	}

	ret = lp5569_engine_load_program(bus);
	if (ret) {
		printf("failed to set engines to load program mode: %d\n", ret);
		return ret;
	}

	ret = lp5569_mem_page_select(bus, true);
	if (ret) {
		printf("failed to select LED mux memory page: %d\n", ret);
		return ret;
	}

	ret = lp5569_write_bulk(bus, LP5569_REG_PROGRAM_MEM_00,
							led_mux_buf, led_mux_buf_len);
	if (ret) {
		printf("failed to write LED mux table to memory: %d\n", ret);
		return ret;
	}

	ret = lp5569_mem_page_select(bus, false);
	if (ret) {
		printf("failed to select program memory page: %d\n", ret);
		return ret;
	}

	ret = lp5569_write_bulk(bus, LP5569_REG_PROGRAM_MEM_00,
							prog_buf, prog_buf_len);
	if (ret) {
		printf("failed to write program to memory: %d\n", ret);
		return ret;
	}

	ret = lp5569_engine_halt(bus);
	if (ret) {
		printf("failed to set engine to HALT mode: %d\n", ret);
		return ret;
	}

	ret = lp5569_engine_run(bus);
	if (ret) {
		printf("Failed to start engine program execution: %d\n", ret);
		return ret;
	}

	return 0;
}

static int lp5569_play_tape(struct gpio_i2c *bus, uint32_t tape_index) {
	int ret;

	if (tape_index >= sizeof (led_mux_tapes) / LP5569_MAX_PROGRAM_LEN) {
		printf("out or range LED tape index\n");
		return -1;
	}

	if ((ret = lp5569_run_program(bus,
								  led_mux_tapes[tape_index],
								  LP5569_MAX_PROGRAM_LEN,
								  led_prog, sizeof (led_prog))) != 0) {
		printf("failed to play tape %u: %d\n", tape_index, ret);
		return ret;
	}

	return 0;
}

static int lp5569_cmd(uint32_t tape_index,
					  struct cmd_tbl *cmdtp, int flag, int argc,
					  char *const argv[]) {
	struct gpio_i2c gpio_i2c;
	int ret, ret2;
	int i;

	if ((ret = i2c_init(&gpio_i2c)) != 0) {
		printf("failed to init i2c: %d\n", ret);
		return cmd_process_error(cmdtp, ret);
	}

	for (i = 0; i < 15; i++) {
		if ((ret = lp5569_init(&gpio_i2c)) != 0) {
			printf("failed to init lp5569: %d\n", ret);
			udelay(100 * 1000);
		} else {
			break;
		}
	}

	if (ret != 0) {
		goto deinit;
	}

	if ((ret = lp5569_play_tape(&gpio_i2c, tape_index)) != 0) {
		printf("failed to play tape: %d\n", ret);
		goto deinit;
	}

 deinit:
	if ((ret2 = i2c_deinit(&gpio_i2c) != 0)) {
		printf("failed to deinit i2c: %d\n", ret2);
		return cmd_process_error(cmdtp, ret ? ret : ret2);
	}

	return 0;
}

static int do_lp5569(struct cmd_tbl *cmdtp, int flag, int argc,
					 char *const argv[])
{
	return lp5569_cmd(0, cmdtp, flag, argc, argv);
}

static int do_lp5569_error(struct cmd_tbl *cmdtp, int flag, int argc,
					 char *const argv[])
{
	int ret = lp5569_cmd(1, cmdtp, flag, argc, argv);

	while (1) {
		udelay(100 * 1000);
	}

	return ret;
}

U_BOOT_CMD(lp5569, 1, 0,  do_lp5569,
		   "LP5569 boot LEDs over i2c\n", NULL);
U_BOOT_CMD(lp5569_error, 1, 0,  do_lp5569_error,
		   "LP5569 error LEDs over i2c\n", NULL);

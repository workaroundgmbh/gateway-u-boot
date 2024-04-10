/*
 * i2c bitbang
 *
 * Copyright (c) 2023 Workaround GmbH
 *
 */

#include <common.h>
#include <asm/gpio.h>
#include <linux/delay.h>

#include "i2c_bitbang.h"

#define SDA_PIN		2
#define SCL_PIN		3


static int gpio_write(int gpio, int bit) {
	int ret;

	if ((ret = gpio_direction_output(gpio, bit)) != 0) {
		printf("gpio: failed to set gpio %d: %d\n", gpio, ret);
		return ret;
	}

	return 0;
}

static int gpio_read(int gpio) {
	int ret;

	if ((ret = gpio_direction_input(gpio)) != 0) {
		printf("gpio: failed to get %d value: %d\n", gpio, ret);
		return ret;
	}

	ret = gpio_get_value(gpio);
	if (ret != 0 && ret != 1) {
        printf("gpio: invalid value for %d: %d\n", gpio, ret);
		return -EINVAL;
	}

	return ret;
}

static int get_sda(struct gpio_i2c *bus) {
	return gpio_read(bus->sda);
}

static void set_sda(struct gpio_i2c *bus, int bit) {
	gpio_write(bus->sda, bit);
}

static void set_scl(struct gpio_i2c *bus, int bit) {
	gpio_write(bus->scl, bit);
}

static void i2c_start(struct gpio_i2c *bus)
{
	int delay = bus->delay;

	udelay(delay);
    bus->set_sda(bus, 1);
    udelay(delay);
    bus->set_scl(bus, 1);
    udelay(delay);
    bus->set_sda(bus, 0);
    udelay(delay);
}

static void i2c_stop(struct gpio_i2c *bus)
{
	int delay = bus->delay;

	bus->set_scl(bus, 0);
    udelay(delay);
    bus->set_sda(bus, 0);
    udelay(delay);
    bus->set_scl(bus, 1);
    udelay(delay);
    bus->set_sda(bus, 1);
    udelay(delay);
}

static void i2c_write_bit(struct gpio_i2c *bus, uint8_t bit)
{
	int delay = bus->delay;

    bus->set_scl(bus, 0);
    udelay(delay);
    bus->set_sda(bus, bit);
    udelay(delay);
    bus->set_scl(bus, 1);
    udelay(2 * delay);
}

static int i2c_read_bit(struct gpio_i2c *bus)
{
	int value;
	int delay = bus->delay;

	bus->set_scl(bus, 1);
	udelay(delay);
	value = bus->get_sda(bus);
	udelay(delay);
	bus->set_scl(bus, 0);
	udelay(2 * delay);

	return value;
}

static void i2c_sda_high(struct gpio_i2c *bus)
{
	int delay = bus->delay;

    bus->set_scl(bus, 0);
    udelay(delay);
    bus->set_sda(bus, 1);
    udelay(delay);

	bus->get_sda(bus);
}

static int i2c_write_byte(struct gpio_i2c* bus, uint8_t data)
{
	int j;
    int nack;
	int delay = bus->delay;

    for (j = 0; j < 8; j++) {
        i2c_write_bit(bus, data & 0x80);
        data <<= 1;
    }

    udelay(delay);

    /* Look for an <ACK>(negative logic) and return it */
    i2c_sda_high(bus);
    nack = i2c_read_bit(bus);

    return nack;    /* not a nack is an ack */
}

int i2c_init(struct gpio_i2c *bus) {
	int ret;

	bus->scl = SCL_PIN;
	bus->sda = SDA_PIN;
	bus->set_sda = set_sda;
	bus->set_scl = set_scl;
	bus->get_sda = get_sda;
	bus->delay = 5; // 100KHz

	ret = gpio_request(bus->scl, "cmd_gpio");
	if (ret) {
		printf("gpio: requesting SCL failed: %d\n", ret);
		return ret;
	}

	ret = gpio_request(bus->sda, "cmd_gpio");
	if (ret) {
		gpio_free(bus->scl);
		printf("gpio: requesting SDA failed: %d\n", ret);
		return ret;
	}

	return 0;
}

int i2c_deinit(struct gpio_i2c *bus) {
	int ret;

	if ((ret = gpio_free(bus->scl)) != 0) {
		printf("gpio: failed to free SCL\n");
		return ret;
	}

	if ((ret = gpio_free(bus->sda)) != 0) {
		printf("gpio: failed to free SDA\n");
		return ret;
	}

	return 0;
}

int i2c_set_register(struct gpio_i2c *bus, uint8_t addr, uint8_t reg, uint8_t val)
{
	int ret;

	i2c_start(bus);

	ret = i2c_write_byte(bus, addr);
	if (ret) {
		printf("i2c: writing address %02X failed: %d\n", addr, ret);
		i2c_stop(bus);
		return -1;
	}

	ret = i2c_write_byte(bus, reg);
	if (ret) {
		printf("i2c: Writing register %u failed: %d\n", reg, ret);
		i2c_stop(bus);
		return -1;
	}

	ret = i2c_write_byte(bus, val);
	if (ret) {
		printf("Writing I2C value failed\n");
		i2c_stop(bus);
		return -1;
	}

	i2c_stop(bus);

	return 0;
}

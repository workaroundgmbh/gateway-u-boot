/*
 * i2c bitbang
 *
 * Copyright (c) 2023 Workaround GmbH
 *
 */

#ifndef I2C_BITBANG_H
#define I2C_BITBANG_H


struct gpio_i2c {
	int scl;
	int sda;
	int delay;

	int (*get_sda)(struct gpio_i2c *bus);
    void (*set_sda)(struct gpio_i2c *bus, int bit);
    void (*set_scl)(struct gpio_i2c *bus, int bit);
};


int i2c_init(struct gpio_i2c *gpio_i2c);
int i2c_deinit(struct gpio_i2c *gpio_i2c);
int i2c_set_register(struct gpio_i2c *gpio_i2c, uint8_t addr,
					 uint8_t reg, uint8_t vall);

#endif // I2C_BITBANG_H

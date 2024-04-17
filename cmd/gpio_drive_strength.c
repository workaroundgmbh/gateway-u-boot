/*
 * GPIO driver strength
 *
 * Copyright (c) 2024 Workaround GmbH
 *
 */

#include <common.h>
#include <command.h>
#include <errno.h>
#include <dm.h>
#include <log.h>
#include <asm/io.h>

/* https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#gpio-pads-control */
/* https://paulwratt.github.io/rpi-internal-registers-online/Region_PM.html#pm_pads0 */
#define BCM283X_PADCTRL_ADDRESS (rpi_pm_base + 0x2c)

struct _bcm2835_gpio_padctrl_bits {
	u32 drive: 3;
	u32 hyst: 1;
	u32 slew: 1;
	u32 reserved: 19;
	u32 password: 8;
};

struct bcm2835_gpio_padctrl {
	union {
		struct _bcm2835_gpio_padctrl_bits bits;
		u32 val;
	};
};

static void print_pad_value(uint8_t num, struct bcm2835_gpio_padctrl *padctrl) {
	uint32_t *padctrl_addr = (uint32_t*)BCM283X_PADCTRL_ADDRESS + num;
	padctrl->val = readl(padctrl_addr);

	printf("get pad control %u (%p), val: %08X, slew: %u, hyst: %u, drive; %u\n",
		   num, padctrl_addr,
		   padctrl->val,
		   padctrl->bits.slew, padctrl->bits.hyst, padctrl->bits.drive);
}

static void set_pad_value(uint8_t num, struct bcm2835_gpio_padctrl *padctrl) {
	uint32_t *padctrl_addr = (uint32_t*)BCM283X_PADCTRL_ADDRESS + num;

	padctrl->bits.password = 0x5a;

	printf("set pad control %u (%p), val: %08X, slew: %u, hyst: %u, drive; %u\n",
		   num, padctrl_addr,
		   padctrl->val,
		   padctrl->bits.slew, padctrl->bits.hyst, padctrl->bits.drive);

	writel(padctrl->val, padctrl_addr);
}

static int do_gpio_drive_strength(struct cmd_tbl *cmdtp, int flag, int argc,
					 char *const argv[])
{
	int padnum;
	int slew;
	int hyst;
	int drive;

	if (argc != 5)
		return CMD_RET_USAGE;

	padnum = dectoul(argv[1], NULL);
	slew = dectoul(argv[2], NULL);
	hyst = dectoul(argv[3], NULL);
	drive = dectoul(argv[4], NULL);

	if (padnum < 0 || padnum > 3) {
		printf("invali pad number\n");
		return -1;
	}

	if (slew < 0 || slew > 1) {
		printf("invalid slew\n");
		return -1;
	}

	if (hyst < 0 || hyst > 1) {
		printf("invalid hysteresis\n");
		return -1;
	}

	if (drive < 0 || drive > 7) {
		printf("invalid drive\n");
		return -1;
	}

	struct bcm2835_gpio_padctrl padctrl;

	print_pad_value(padnum, &padctrl);

	padctrl.bits.slew = slew;
	padctrl.bits.hyst = hyst;
	padctrl.bits.drive = drive;

	set_pad_value(padnum, &padctrl);

	return 0;
}


U_BOOT_CMD(gpio_drive_strength, 5, 0,  do_gpio_drive_strength,
		   "GPIO driver strength",
		   "<pad> <slew> <hyst> <drive>\n"
		   "pad        pad to control (0-3)\n"
		   "slew       slew rate (0|1)\n"
		   "hyst       hysteresis (0|1)\n"
		   "drive      drive strength (0-7)");

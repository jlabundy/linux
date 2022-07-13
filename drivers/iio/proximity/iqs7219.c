// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Azoteq IQS7219A SAR Proximity Sensor
 *
 * Copyright (C) 2022 Azoteq (Pty) Ltd
 * Author: Jeff LaBundy <jeff@labundy.com>
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <asm/unaligned.h>

#define IQS7219_PROD_NUM			0x00
#define IQS7219_PROD_NUM_A			685

#define IQS7219_SYS_STATUS			0x10
#define IQS7219_SYS_STATUS_RESET		BIT(11)
#define IQS7219_SYS_STATUS_ATI_ERROR		BIT(9)
#define IQS7219_SYS_STATUS_ATI_ACTIVE		BIT(8)

#define IQS7219_SYS_SETUP			0x80
#define IQS7219_SYS_SETUP_INTF_MODE_MASK	GENMASK(7, 6)
#define IQS7219_SYS_SETUP_INTF_MODE_STAND	BIT(7)
#define IQS7219_SYS_SETUP_INTF_MODE_EVENT	BIT(6)
#define IQS7219_SYS_SETUP_PWR_MODE_MASK		GENMASK(5, 4)
#define IQS7219_SYS_SETUP_REDO_ATI		BIT(2)
#define IQS7219_SYS_SETUP_ACK_RESET		BIT(0)

#define IQS7219_CAP_SETUP_VREF_HALF		BIT(6)
#define IQS7219_CAP_SETUP_SAMP_DOUBLE		BIT(4)
#define IQS7219_CAP_SETUP_COUNTS_MASK		GENMASK(1, 0)
#define IQS7219_CAP_SETUP_COUNTS_MAX		IQS7219_CAP_SETUP_COUNTS_MASK

#define IQS7219_EVENT_MASK_ALL			GENMASK(4, 0)
#define IQS7219_EVENT_MASK_ATI			BIT(3)

#define IQS7219_COMMS_SETUP			0x8D
#define IQS7219_COMMS_SETUP_MIN_FW		((1 << 16) | 25)
#define IQS7219_COMMS_HOLD			BIT(8)
#define IQS7219_COMMS_ERROR			0xEEEE
#define IQS7219_COMMS_RETRY_MS			50
#define IQS7219_COMMS_TIMEOUT_MS		100
#define IQS7219_RESET_TIMEOUT_MS		250
#define IQS7219_ATI_TIMEOUT_MS			2000

#define IQS7219_GPIO_OPEN_DRAIN			BIT(3)
#define IQS7219_GPIO_ACTIVE_HIGH		BIT(0)

#define IQS7219_NUM_COLS_STAT			12
#define IQS7219_NUM_COLS_ATI			5
#define IQS7219_NUM_COLS_SYS			10
#define IQS7219_NUM_COLS_PXS			1
#define IQS7219_NUM_COLS_EVENT			5
#define IQS7219_NUM_COLS_CHAN			13
#define IQS7219_NUM_CHAN			2
#define IQS7219_NUM_CYCLES			IQS7219_NUM_CHAN
#define IQS7219_NUM_RETRIES			5

#define IQS7219_UHZ_PER_MS			1000000000
#define IQS7219_MAX_RATE_MS			3000
#define IQS7219_MAX_RATE_HZ			1000

#define IQS7219_NAME_DELTA			"delta"
#define IQS7219_NAME_FILT			"counts_filt"
#define IQS7219_NAME_RAW			"counts_raw"
#define IQS7219_NAME_LTA			"lta"
#define IQS7219_NAME_VAR			"variance"
#define IQS7219_NAME_PXS			"pxs_flags"

#define IQS7219_IIO_CHAN(chan_index)					\
	{								\
		.type = IIO_PROXIMITY,					\
		.channel = chan_index,					\
		.scan_index = chan_index,				\
		.scan_type = {						\
			.sign = 'u',					\
			.realbits = 32,					\
			.storagebits = 32,				\
		},							\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
		.event_spec = iqs7219_iio_events,			\
		.num_event_specs = ARRAY_SIZE(iqs7219_iio_events),	\
		.ext_info = iqs7219_ext_info,				\
		.indexed = 1,						\
	}

static const char * const iqs7219_pxs_events[] = {
	"event-halt",
	"event-prox",
	"event-touch",
};

static const char * const iqs7219_sense_modes[] = {
	"sense-mode-proj",
	"sense-mode-self",
};

static const unsigned int iqs7219_gpios[] = { 1, 2, 5, };

enum iqs7219_scan_id {
	IQS7219_SCAN_DELTA,
	IQS7219_SCAN_FILT,
	IQS7219_SCAN_RAW,
	IQS7219_SCAN_LTA,
	IQS7219_SCAN_VAR,
	IQS7219_SCAN_PXS,
	IQS7219_NUM_SCAN
};

static const char * const iqs7219_scan_names[] = {
	[IQS7219_SCAN_DELTA] = IQS7219_NAME_DELTA,
	[IQS7219_SCAN_FILT] = IQS7219_NAME_FILT,
	[IQS7219_SCAN_RAW] = IQS7219_NAME_RAW,
	[IQS7219_SCAN_LTA] = IQS7219_NAME_LTA,
	[IQS7219_SCAN_VAR] = IQS7219_NAME_VAR,
	[IQS7219_SCAN_PXS] = IQS7219_NAME_PXS,
};

enum iqs7219_reg_key_id {
	IQS7219_REG_KEY_ATI,
	IQS7219_REG_KEY_SYS,
	IQS7219_REG_KEY_PXS,
	IQS7219_REG_KEY_CAP,
	IQS7219_REG_KEY_EVENT,
	IQS7219_REG_KEY_CHAN,
};

enum iqs7219_reg_grp_id {
	IQS7219_REG_GRP_ATI,
	IQS7219_REG_GRP_SYS,
	IQS7219_REG_GRP_PXS,
	IQS7219_REG_GRP_CAP,
	IQS7219_REG_GRP_EVENT_0,
	IQS7219_REG_GRP_EVENT_1,
	IQS7219_REG_GRP_CHAN_0,
	IQS7219_REG_GRP_CHAN_1,
	IQS7219_NUM_REG_GRPS
};

static const char * const iqs7219_reg_grp_names[] = {
	[IQS7219_REG_GRP_ATI] = "ati",
	[IQS7219_REG_GRP_PXS] = "cycle",
	[IQS7219_REG_GRP_CHAN_0] = "channel",
	[IQS7219_REG_GRP_CHAN_1] = "channel",
};

struct iqs7219_reg_grp_desc {
	u16 base;
	enum iqs7219_reg_key_id reg_key;
	int num_row;
	int num_col;
};

static const struct iqs7219_reg_grp_desc iqs7219_reg_grps[] = {
	[IQS7219_REG_GRP_ATI] = {
		.base = 0x20,
		.reg_key = IQS7219_REG_KEY_ATI,
		.num_row = IQS7219_NUM_CHAN,
		.num_col = IQS7219_NUM_COLS_ATI,
	},
	[IQS7219_REG_GRP_SYS] = {
		.base = IQS7219_SYS_SETUP,
		.reg_key = IQS7219_REG_KEY_SYS,
		.num_row = 1,
		.num_col = IQS7219_NUM_COLS_SYS,
	},
	[IQS7219_REG_GRP_PXS] = {
		.base = 0x8A,
		.reg_key = IQS7219_REG_KEY_PXS,
		.num_row = IQS7219_NUM_CYCLES,
		.num_col = IQS7219_NUM_COLS_PXS,
	},
	[IQS7219_REG_GRP_CAP] = {
		.base = 0x8C,
		.reg_key = IQS7219_REG_KEY_CAP,
		.num_row = 1,
		.num_col = 1,
	},
	[IQS7219_REG_GRP_EVENT_0] = {
		.base = 0x90,
		.reg_key = IQS7219_REG_KEY_EVENT,
		.num_row = ARRAY_SIZE(iqs7219_pxs_events),
		.num_col = IQS7219_NUM_COLS_EVENT,
	},
	[IQS7219_REG_GRP_CHAN_0] = {
		.base = 0xA0,
		.reg_key = IQS7219_REG_KEY_CHAN,
		.num_row = 1,
		.num_col = IQS7219_NUM_COLS_CHAN,
	},
	[IQS7219_REG_GRP_EVENT_1] = {
		.base = 0xB0,
		.reg_key = IQS7219_REG_KEY_EVENT,
		.num_row = ARRAY_SIZE(iqs7219_pxs_events),
		.num_col = IQS7219_NUM_COLS_EVENT,
	},
	[IQS7219_REG_GRP_CHAN_1] = {
		.base = 0xC0,
		.reg_key = IQS7219_REG_KEY_CHAN,
		.num_row = 1,
		.num_col = IQS7219_NUM_COLS_CHAN,
	},
};

struct iqs7219_prop_desc {
	const char *name;
	enum iqs7219_reg_key_id reg_key;
	int reg_offset;
	int reg_shift;
	int reg_width;
	int val_pitch;
	int val_min;
	int val_max;
	bool invert;
	const char *label;
};

static const struct iqs7219_prop_desc iqs7219_props[] = {
	{
		.name = "azoteq,ati-frac-mult-coarse",
		.reg_key = IQS7219_REG_KEY_ATI,
		.reg_offset = 0,
		.reg_shift = 0,
		.reg_width = 4,
		.label = "ATI coarse fractional multiplier",
	},
	{
		.name = "azoteq,ati-frac-div-coarse",
		.reg_key = IQS7219_REG_KEY_ATI,
		.reg_offset = 1,
		.reg_shift = 0,
		.reg_width = 5,
		.label = "ATI coarse fractional divider",
	},
	{
		.name = "azoteq,ati-frac-div-fine",
		.reg_key = IQS7219_REG_KEY_ATI,
		.reg_offset = 2,
		.reg_shift = 0,
		.reg_width = 5,
		.label = "ATI fine fractional divider",
	},
	{
		.name = "azoteq,ati-comp-div",
		.reg_key = IQS7219_REG_KEY_ATI,
		.reg_offset = 3,
		.reg_shift = 0,
		.reg_width = 5,
		.label = "ATI compensation divider",
	},
	{
		.name = "azoteq,ati-comp-select",
		.reg_key = IQS7219_REG_KEY_ATI,
		.reg_offset = 4,
		.reg_shift = 0,
		.reg_width = 10,
		.label = "ATI compensation selection",
	},
	{
		.name = "azoteq,rate-np-segment",
		.reg_key = IQS7219_REG_KEY_SYS,
		.reg_offset = 0,
		.reg_shift = 8,
		.reg_width = 2,
		.label = "normal-power mode segment rate",
	},
	{
		.name = "azoteq,power-mode",
		.reg_key = IQS7219_REG_KEY_SYS,
		.reg_offset = 0,
		.reg_shift = 4,
		.reg_width = 2,
		.label = "power mode",
	},
	{
		.name = "azoteq,timeout-comms-ms",
		.reg_key = IQS7219_REG_KEY_SYS,
		.reg_offset = 1,
		.reg_shift = 0,
		.reg_width = 8,
		.label = "communication timeout",
	},
	{
		.name = "azoteq,timeout-ati-ms",
		.reg_key = IQS7219_REG_KEY_SYS,
		.reg_offset = 2,
		.reg_shift = 0,
		.reg_width = 16,
		.label = "ATI error timeout",
	},
	{
		.name = "azoteq,rate-ati-ms",
		.reg_key = IQS7219_REG_KEY_SYS,
		.reg_offset = 3,
		.reg_shift = 0,
		.reg_width = 16,
		.label = "ATI report rate",
	},
	{
		.name = "azoteq,timeout-np-ms",
		.reg_key = IQS7219_REG_KEY_SYS,
		.reg_offset = 4,
		.reg_shift = 0,
		.reg_width = 16,
		.label = "normal-power mode timeout",
	},
	{
		.name = "azoteq,rate-np-ms",
		.reg_key = IQS7219_REG_KEY_SYS,
		.reg_offset = 5,
		.reg_shift = 0,
		.reg_width = 16,
		.val_max = IQS7219_MAX_RATE_MS,
		.label = "normal-power mode report rate",
	},
	{
		.name = "azoteq,timeout-lp-ms",
		.reg_key = IQS7219_REG_KEY_SYS,
		.reg_offset = 6,
		.reg_shift = 0,
		.reg_width = 16,
		.label = "low-power mode timeout",
	},
	{
		.name = "azoteq,rate-lp-ms",
		.reg_key = IQS7219_REG_KEY_SYS,
		.reg_offset = 7,
		.reg_shift = 0,
		.reg_width = 16,
		.val_max = IQS7219_MAX_RATE_MS,
		.label = "low-power mode report rate",
	},
	{
		.name = "azoteq,timeout-ulp-ms",
		.reg_key = IQS7219_REG_KEY_SYS,
		.reg_offset = 8,
		.reg_shift = 0,
		.reg_width = 16,
		.label = "ultra-low-power mode timeout",
	},
	{
		.name = "azoteq,rate-ulp-ms",
		.reg_key = IQS7219_REG_KEY_SYS,
		.reg_offset = 9,
		.reg_shift = 0,
		.reg_width = 16,
		.val_max = IQS7219_MAX_RATE_MS,
		.label = "ultra-low-power mode report rate",
	},
	{
		.name = "azoteq,channel-select",
		.reg_key = IQS7219_REG_KEY_PXS,
		.reg_offset = 0,
		.reg_shift = 8,
		.reg_width = 8,
		.label = "channel selection",
	},
	{
		.name = "azoteq,sense-mode",
		.reg_key = IQS7219_REG_KEY_PXS,
		.reg_offset = 0,
		.reg_shift = 0,
		.reg_width = 2,
		.label = "sensing mode",
	},
	{
		.name = "azoteq,proj-bias",
		.reg_key = IQS7219_REG_KEY_CAP,
		.reg_offset = 0,
		.reg_shift = 2,
		.reg_width = 2,
		.label = "projected bias current",
	},
	{
		.name = "azoteq,timeout-active-ms",
		.reg_key = IQS7219_REG_KEY_EVENT,
		.reg_offset = 0,
		.reg_shift = 0,
		.reg_width = 16,
		.label = "active state timeout",
	},
	{
		.name = "azoteq,hyst",
		.reg_key = IQS7219_REG_KEY_EVENT,
		.reg_offset = 1,
		.reg_shift = 0,
		.reg_width = 16,
		.label = "hysteresis",
	},
	{
		.name = "azoteq,thresh",
		.reg_key = IQS7219_REG_KEY_EVENT,
		.reg_offset = 2,
		.reg_shift = 0,
		.reg_width = 16,
		.label = "threshold",
	},
	{
		.name = "azoteq,debounce-exit",
		.reg_key = IQS7219_REG_KEY_EVENT,
		.reg_offset = 3,
		.reg_shift = 8,
		.reg_width = 8,
		.label = "debounce exit factor",
	},
	{
		.name = "azoteq,debounce-enter",
		.reg_key = IQS7219_REG_KEY_EVENT,
		.reg_offset = 3,
		.reg_shift = 0,
		.reg_width = 8,
		.label = "debounce entrance factor",
	},
	{
		.name = "azoteq,counts-beta-lp",
		.reg_key = IQS7219_REG_KEY_CHAN,
		.reg_offset = 0,
		.reg_shift = 12,
		.reg_width = 4,
		.label = "low-power mode counts beta",
	},
	{
		.name = "azoteq,counts-beta-np",
		.reg_key = IQS7219_REG_KEY_CHAN,
		.reg_offset = 0,
		.reg_shift = 8,
		.reg_width = 4,
		.label = "normal-power mode counts beta",
	},
	{
		.name = "azoteq,direction-enable",
		.reg_key = IQS7219_REG_KEY_CHAN,
		.reg_offset = 0,
		.reg_shift = 6,
		.reg_width = 1,
	},
	{
		.name = "azoteq,invert-enable",
		.reg_key = IQS7219_REG_KEY_CHAN,
		.reg_offset = 0,
		.reg_shift = 1,
		.reg_width = 1,
	},
	{
		.name = "azoteq,dual-direction",
		.reg_key = IQS7219_REG_KEY_CHAN,
		.reg_offset = 0,
		.reg_shift = 0,
		.reg_width = 1,
	},
	{
		.name = "azoteq,lta-fast-beta-lp",
		.reg_key = IQS7219_REG_KEY_CHAN,
		.reg_offset = 1,
		.reg_shift = 12,
		.reg_width = 4,
		.label = "low-power mode long-term average fast beta",
	},
	{
		.name = "azoteq,lta-fast-beta-np",
		.reg_key = IQS7219_REG_KEY_CHAN,
		.reg_offset = 1,
		.reg_shift = 8,
		.reg_width = 4,
		.label = "normal-power mode long-term average fast beta",
	},
	{
		.name = "azoteq,lta-beta-lp",
		.reg_key = IQS7219_REG_KEY_CHAN,
		.reg_offset = 1,
		.reg_shift = 4,
		.reg_width = 4,
		.label = "low-power mode long-term average beta",
	},
	{
		.name = "azoteq,lta-beta-np",
		.reg_key = IQS7219_REG_KEY_CHAN,
		.reg_offset = 1,
		.reg_shift = 0,
		.reg_width = 4,
		.label = "normal-power mode long-term average beta",
	},
	{
		.name = "azoteq,conv-period",
		.reg_key = IQS7219_REG_KEY_CHAN,
		.reg_offset = 2,
		.reg_shift = 8,
		.reg_width = 8,
		.label = "conversion period",
	},
	{
		.name = "azoteq,conv-frac",
		.reg_key = IQS7219_REG_KEY_CHAN,
		.reg_offset = 2,
		.reg_shift = 0,
		.reg_width = 8,
		.label = "conversion frequency fractional divider",
	},
	{
		.name = "azoteq,conv-scale",
		.reg_key = IQS7219_REG_KEY_CHAN,
		.reg_offset = 3,
		.reg_shift = 0,
		.reg_width = 8,
		.val_max = 3,
		.label = "conversion frequency scaling factor",
	},
	{
		.name = "azoteq,ati-base",
		.reg_key = IQS7219_REG_KEY_CHAN,
		.reg_offset = 5,
		.reg_shift = 0,
		.reg_width = 16,
		.val_max = 500,
		.label = "ATI base",
	},
	{
		.name = "azoteq,ati-target",
		.reg_key = IQS7219_REG_KEY_CHAN,
		.reg_offset = 6,
		.reg_shift = 0,
		.reg_width = 16,
		.val_max = 4000,
		.label = "ATI target",
	},
	{
		.name = "azoteq,ati-band",
		.reg_key = IQS7219_REG_KEY_CHAN,
		.reg_offset = 7,
		.reg_shift = 0,
		.reg_width = 16,
		.val_max = 1500,
		.label = "ATI band",
	},
	{
		.name = "azoteq,ati-mode",
		.reg_key = IQS7219_REG_KEY_CHAN,
		.reg_offset = 8,
		.reg_shift = 0,
		.reg_width = 3,
		.val_max = 5,
		.label = "ATI mode",
	},
	{
		.name = "azoteq,ati-frac-div-coarse",
		.reg_key = IQS7219_REG_KEY_CHAN,
		.reg_offset = 9,
		.reg_shift = 8,
		.reg_width = 5,
		.label = "ATI coarse fractional divider",
	},
	{
		.name = "azoteq,ati-frac-div-fine",
		.reg_key = IQS7219_REG_KEY_CHAN,
		.reg_offset = 9,
		.reg_shift = 0,
		.reg_width = 5,
		.label = "ATI fine fractional divider",
	},
	{
		.name = "azoteq,ati-comp-select",
		.reg_key = IQS7219_REG_KEY_CHAN,
		.reg_offset = 10,
		.reg_shift = 0,
		.reg_width = 10,
		.label = "ATI compensation selection",
	},
	{
		.name = "azoteq,thresh",
		.reg_key = IQS7219_REG_KEY_CHAN,
		.reg_offset = 11,
		.reg_shift = 0,
		.reg_width = 16,
		.label = "threshold",
	},
	{
		.name = "azoteq,fast-filt-band",
		.reg_key = IQS7219_REG_KEY_CHAN,
		.reg_offset = 12,
		.reg_shift = 0,
		.reg_width = 8,
		.label = "fast filter band",
	},
};

struct iqs7219_private {
	struct gpio_desc *reset_gpio;
	struct gpio_desc *irq_gpio;
	struct i2c_client *client;
	struct iio_trigger *trig;
	struct mutex lock;
	u16 pxs_flags;
	u16 intf_mode;
	u16 ati_setup[IQS7219_NUM_CHAN]
		     [IQS7219_NUM_COLS_ATI];
	u16 sys_setup[IQS7219_NUM_COLS_SYS];
	u16 pxs_setup[IQS7219_NUM_CYCLES]
		     [IQS7219_NUM_COLS_PXS];
	u16 cap_setup;
	u16 event_setup[IQS7219_NUM_CHAN]
		       [ARRAY_SIZE(iqs7219_pxs_events)]
		       [IQS7219_NUM_COLS_EVENT];
	u16 chan_setup[IQS7219_NUM_CHAN]
		      [IQS7219_NUM_COLS_CHAN];
	u16 event_mask[IQS7219_NUM_CHAN];
	u16 comms_setup;
	int scan_data[IQS7219_NUM_CHAN];
	bool event_enable[IQS7219_NUM_CHAN];
	bool trig_valid[IQS7219_NUM_CHAN];
	bool trig_enable;
	s64 timestamp;
	enum iqs7219_scan_id scan_mux[IQS7219_NUM_CHAN];
};

static u16 *iqs7219_setup(struct iqs7219_private *iqs7219,
			  enum iqs7219_reg_grp_id reg_grp, int row)
{
	switch (reg_grp) {
	case IQS7219_REG_GRP_ATI:
		return iqs7219->ati_setup[row];

	case IQS7219_REG_GRP_SYS:
		return iqs7219->sys_setup;

	case IQS7219_REG_GRP_PXS:
		return iqs7219->pxs_setup[row];

	case IQS7219_REG_GRP_CAP:
		return &iqs7219->cap_setup;

	case IQS7219_REG_GRP_EVENT_0:
	case IQS7219_REG_GRP_EVENT_1:
		return iqs7219->event_setup[reg_grp - IQS7219_REG_GRP_EVENT_0][row];

	case IQS7219_REG_GRP_CHAN_0:
	case IQS7219_REG_GRP_CHAN_1:
		return iqs7219->chan_setup[reg_grp - IQS7219_REG_GRP_CHAN_0];

	default:
		return NULL;
	}
}

static int iqs7219_irq_poll(struct iqs7219_private *iqs7219, u16 timeout_ms)
{
	ktime_t irq_timeout = ktime_add_ms(ktime_get(), timeout_ms);
	int ret;

	do {
		usleep_range(1000, 1100);

		ret = gpiod_get_value_cansleep(iqs7219->irq_gpio);
		if (ret < 0)
			return ret;
		else if (ret > 0)
			return 0;
	} while (ktime_compare(ktime_get(), irq_timeout) < 0);

	return -EBUSY;
}

static int iqs7219_hard_reset(struct iqs7219_private *iqs7219)
{
	struct i2c_client *client = iqs7219->client;
	int error;

	if (!iqs7219->reset_gpio)
		return 0;

	gpiod_set_value_cansleep(iqs7219->reset_gpio, 1);
	usleep_range(1000, 1100);

	gpiod_set_value_cansleep(iqs7219->reset_gpio, 0);

	error = iqs7219_irq_poll(iqs7219, IQS7219_RESET_TIMEOUT_MS);
	if (error)
		dev_err(&client->dev, "Failed to reset device: %d\n", error);

	return error;
}

static int iqs7219_force_comms(struct iqs7219_private *iqs7219)
{
	u8 msg_buf[] = { 0xFF, 0x00, };
	int ret;

	/*
	 * The device cannot communicate until it asserts its interrupt (RDY)
	 * pin. Attempts to do so while RDY is deasserted return an ACK; how-
	 * ever all write data is ignored, and all read data returns 0xEE.
	 *
	 * Unsolicited communication must be preceded by a special force com-
	 * munication command, after which the device eventually asserts its
	 * RDY pin and agrees to communicate.
	 *
	 * Regardless of whether communication is forced or the result of an
	 * interrupt, the device automatically deasserts its RDY pin once it
	 * detects an I2C stop condition, or a timeout expires.
	 */
	ret = gpiod_get_value_cansleep(iqs7219->irq_gpio);
	if (ret < 0)
		return ret;
	else if (ret > 0)
		return 0;

	ret = i2c_master_send(iqs7219->client, msg_buf, sizeof(msg_buf));
	if (ret < (int)sizeof(msg_buf)) {
		if (ret >= 0)
			ret = -EIO;

		msleep(IQS7219_COMMS_RETRY_MS);
		return ret;
	}

	return iqs7219_irq_poll(iqs7219, IQS7219_COMMS_TIMEOUT_MS);
}

static int iqs7219_read_burst(struct iqs7219_private *iqs7219,
			      u8 reg, void *val, u16 num_val)
{
	int ret, i;
	struct i2c_client *client = iqs7219->client;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(reg),
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = num_val * sizeof(__le16),
			.buf = (u8 *)val,
		},
	};

	/*
	 * The following loop protects against an edge case in which the RDY
	 * pin is automatically deasserted just as the read is initiated. In
	 * that case, the read must be retried using forced communication.
	 */
	for (i = 0; i < IQS7219_NUM_RETRIES; i++) {
		ret = iqs7219_force_comms(iqs7219);
		if (ret < 0)
			continue;

		ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		if (ret < (int)ARRAY_SIZE(msg)) {
			if (ret >= 0)
				ret = -EIO;

			msleep(IQS7219_COMMS_RETRY_MS);
			continue;
		}

		if (get_unaligned_le16(msg[1].buf) == IQS7219_COMMS_ERROR) {
			ret = -ENODATA;
			continue;
		}

		ret = 0;
		break;
	}

	/*
	 * The following delay ensures the device has deasserted the RDY pin
	 * following the I2C stop condition.
	 */
	usleep_range(50, 100);

	if (ret < 0)
		dev_err(&client->dev,
			"Failed to read from address 0x%02X: %d\n", reg, ret);

	return ret;
}

static int iqs7219_read_word(struct iqs7219_private *iqs7219, u8 reg, u16 *val)
{
	__le16 val_buf;
	int error;

	error = iqs7219_read_burst(iqs7219, reg, &val_buf, 1);
	if (error)
		return error;

	*val = le16_to_cpu(val_buf);

	return 0;
}

static int iqs7219_write_burst(struct iqs7219_private *iqs7219,
			       u8 reg, const void *val, u16 num_val)
{
	int val_len = num_val * sizeof(__le16);
	int msg_len = sizeof(reg) + val_len;
	int ret, i;
	struct i2c_client *client = iqs7219->client;
	u8 *msg_buf;

	msg_buf = kzalloc(msg_len, GFP_KERNEL);
	if (!msg_buf)
		return -ENOMEM;

	*msg_buf = reg;
	memcpy(msg_buf + sizeof(reg), val, val_len);

	/*
	 * The following loop protects against an edge case in which the RDY
	 * pin is automatically asserted just before the force communication
	 * command is sent.
	 *
	 * In that case, the subsequent I2C stop condition tricks the device
	 * into preemptively deasserting the RDY pin and the command must be
	 * sent again.
	 */
	for (i = 0; i < IQS7219_NUM_RETRIES; i++) {
		ret = iqs7219_force_comms(iqs7219);
		if (ret < 0)
			continue;

		ret = i2c_master_send(client, msg_buf, msg_len);
		if (ret < msg_len) {
			if (ret >= 0)
				ret = -EIO;

			msleep(IQS7219_COMMS_RETRY_MS);
			continue;
		}

		ret = 0;
		break;
	}

	kfree(msg_buf);

	usleep_range(50, 100);

	if (ret < 0)
		dev_err(&client->dev,
			"Failed to write to address 0x%02X: %d\n", reg, ret);

	return ret;
}

static int iqs7219_write_word(struct iqs7219_private *iqs7219, u8 reg, u16 val)
{
	__le16 val_buf = cpu_to_le16(val);

	return iqs7219_write_burst(iqs7219, reg, &val_buf, 1);
}

static int iqs7219_ati_trigger(struct iqs7219_private *iqs7219)
{
	struct i2c_client *client = iqs7219->client;
	ktime_t ati_timeout;
	u16 sys_status = 0;
	u16 sys_setup;
	int error, i;

	error = iqs7219_read_word(iqs7219, IQS7219_SYS_SETUP, &sys_setup);
	if (error)
		return error;

	sys_setup &= ~IQS7219_SYS_SETUP_INTF_MODE_MASK;

	for (i = 0; i < IQS7219_NUM_RETRIES; i++) {
		/*
		 * Trigger ATI from streaming and normal-power modes so that
		 * the RDY pin continues to be asserted during ATI.
		 */
		error = iqs7219_write_word(iqs7219, IQS7219_SYS_SETUP,
					   (sys_setup &
					  ~IQS7219_SYS_SETUP_PWR_MODE_MASK) |
					   IQS7219_SYS_SETUP_REDO_ATI);
		if (error)
			return error;

		ati_timeout = ktime_add_ms(ktime_get(), IQS7219_ATI_TIMEOUT_MS);

		do {
			error = iqs7219_irq_poll(iqs7219,
						 IQS7219_COMMS_TIMEOUT_MS);
			if (error)
				continue;

			error = iqs7219_read_word(iqs7219, IQS7219_SYS_STATUS,
						  &sys_status);
			if (error)
				return error;

			if (sys_status & IQS7219_SYS_STATUS_ATI_ACTIVE)
				continue;

			if (sys_status & IQS7219_SYS_STATUS_ATI_ERROR)
				break;

			sys_setup |= iqs7219->intf_mode;

			return iqs7219_write_word(iqs7219, IQS7219_SYS_SETUP,
						  sys_setup);
		} while (ktime_compare(ktime_get(), ati_timeout) < 0);

		dev_err(&client->dev,
			"ATI attempt %d of %d failed with status 0x%02X, %s\n",
			i + 1, IQS7219_NUM_RETRIES, (u8)sys_status,
			i < IQS7219_NUM_RETRIES ? "retrying..." : "stopping");
	}

	return -ETIMEDOUT;
}

static int iqs7219_write_comms(struct iqs7219_private *iqs7219, u16 event_mask)
{
	int error;
	u16 val;

	if (!iqs7219->comms_setup)
		return 0;

	error = iqs7219_read_word(iqs7219, iqs7219->comms_setup, &val);
	if (error)
		return error;

	val &= ~(IQS7219_COMMS_HOLD | IQS7219_EVENT_MASK_ALL);
	val |= event_mask;

	return iqs7219_write_word(iqs7219, iqs7219->comms_setup, val);
}

static int iqs7219_dev_init(struct iqs7219_private *iqs7219, int dir)
{
	u16 event_mask = IQS7219_EVENT_MASK_ATI;
	int error, i, j;

	/*
	 * Acknowledge reset before writing any registers in case the device
	 * suffers a spurious reset during initialization.
	 */
	if (dir == WRITE) {
		error = iqs7219_write_word(iqs7219, IQS7219_SYS_SETUP,
					   iqs7219->sys_setup[0] |
					   IQS7219_SYS_SETUP_ACK_RESET);
		if (error)
			return error;
	}

	/*
	 * Take advantage of the stop-bit disable function, if available, to
	 * save the trouble of having to reopen a communication window after
	 * each burst read or write.
	 */
	error = iqs7219_write_comms(iqs7219, IQS7219_COMMS_HOLD);
	if (error)
		return error;

	for (i = 0; i < IQS7219_NUM_REG_GRPS; i++) {
		int num_val = iqs7219_reg_grps[i].num_row *
			      iqs7219_reg_grps[i].num_col;
		__le16 *val_buf;
		u16 *val;

		val = iqs7219_setup(iqs7219, i, 0);
		if (!val)
			continue;

		val_buf = kcalloc(num_val, sizeof(__le16), GFP_KERNEL);
		if (!val_buf)
			return -ENOMEM;

		switch (dir) {
		case READ:
			error = iqs7219_read_burst(iqs7219,
						   iqs7219_reg_grps[i].base,
						   val_buf, num_val);
			for (j = 0; j < num_val; j++)
				val[j] = le16_to_cpu(val_buf[j]);
			break;

		case WRITE:
			for (j = 0; j < num_val; j++)
				val_buf[j] = cpu_to_le16(val[j]);
			error = iqs7219_write_burst(iqs7219,
						    iqs7219_reg_grps[i].base,
						    val_buf, num_val);
			break;

		default:
			error = -EINVAL;
		}

		kfree(val_buf);

		if (error)
			return error;
	}

	for (i = 0; i < IQS7219_NUM_CHAN; i++)
		if (iqs7219->trig_enable || iqs7219->event_enable[i])
			event_mask |= iqs7219->event_mask[i];

	error = iqs7219_write_comms(iqs7219, event_mask);
	if (error)
		return error;

	if (dir == READ)
		return 0;

	return iqs7219_ati_trigger(iqs7219);
}

static int iqs7219_parse_props(struct iqs7219_private *iqs7219,
			       struct fwnode_handle **child_node,
			       int child_index,
			       enum iqs7219_reg_grp_id reg_grp)
{
	u16 *setup = iqs7219_setup(iqs7219, reg_grp, child_index);
	struct i2c_client *client = iqs7219->client;
	struct fwnode_handle *reg_grp_node;
	char reg_grp_name[16];
	int i;

	switch (reg_grp) {
	case IQS7219_REG_GRP_ATI:
	case IQS7219_REG_GRP_PXS:
	case IQS7219_REG_GRP_EVENT_0:
	case IQS7219_REG_GRP_EVENT_1:
	case IQS7219_REG_GRP_CHAN_0:
	case IQS7219_REG_GRP_CHAN_1:
		/*
		 * These groups derive a child node and return it to the caller
		 * for additional group-specific processing. In some cases, the
		 * child node may have already been derived.
		 */
		reg_grp_node = *child_node;
		if (reg_grp_node)
			break;

		snprintf(reg_grp_name, sizeof(reg_grp_name), "%s-%d",
			 iqs7219_reg_grp_names[reg_grp], child_index);

		reg_grp_node = device_get_named_child_node(&client->dev,
							   reg_grp_name);
		if (!reg_grp_node)
			return 0;

		*child_node = reg_grp_node;
		break;

	case IQS7219_REG_GRP_SYS:
	case IQS7219_REG_GRP_CAP:
		/*
		 * These groups are not organized beneath a child node, nor are
		 * they subject to any additional processing by the caller.
		 */
		reg_grp_node = dev_fwnode(&client->dev);
		break;

	default:
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(iqs7219_props); i++) {
		const char *name = iqs7219_props[i].name;
		enum iqs7219_reg_key_id reg_key = iqs7219_props[i].reg_key;
		int reg_offset = iqs7219_props[i].reg_offset;
		int reg_shift = iqs7219_props[i].reg_shift;
		int reg_width = iqs7219_props[i].reg_width;
		int val_pitch = iqs7219_props[i].val_pitch ? : 1;
		int val_min = iqs7219_props[i].val_min;
		int val_max = iqs7219_props[i].val_max;
		bool invert = iqs7219_props[i].invert;
		const char *label = iqs7219_props[i].label ? : name;
		unsigned int val;
		int error;

		if (reg_key != iqs7219_reg_grps[reg_grp].reg_key)
			continue;

		/*
		 * Boolean register fields are one bit wide; they are forcibly
		 * reset to provide a means to undo changes by a bootloader if
		 * necessary.
		 *
		 * Scalar fields, on the other hand, are left untouched unless
		 * their corresponding properties are present.
		 */
		if (reg_width == 1) {
			if (invert)
				setup[reg_offset] |= BIT(reg_shift);
			else
				setup[reg_offset] &= ~BIT(reg_shift);
		}

		if (!fwnode_property_present(reg_grp_node, name))
			continue;

		if (reg_width == 1) {
			if (invert)
				setup[reg_offset] &= ~BIT(reg_shift);
			else
				setup[reg_offset] |= BIT(reg_shift);

			continue;
		}

		error = fwnode_property_read_u32(reg_grp_node, name, &val);
		if (error) {
			dev_err(&client->dev, "Failed to read %s %s: %d\n",
				fwnode_get_name(reg_grp_node), label, error);
			return error;
		}

		if (!val_max)
			val_max = GENMASK(reg_width - 1, 0) * val_pitch;

		if (val < val_min || val > val_max) {
			dev_err(&client->dev, "Invalid %s %s: %u\n",
				fwnode_get_name(reg_grp_node), label, val);
			return -EINVAL;
		}

		setup[reg_offset] &= ~GENMASK(reg_shift + reg_width - 1,
					      reg_shift);
		setup[reg_offset] |= (val / val_pitch << reg_shift);
	}

	return 0;
}

static int iqs7219_parse_chan(struct iqs7219_private *iqs7219, int chan_index)
{
	struct i2c_client *client = iqs7219->client;
	struct fwnode_handle *chan_node = NULL;
	unsigned int val;
	int error, i, j;
	u16 *chan_setup = iqs7219->chan_setup[chan_index];

	error = iqs7219_parse_props(iqs7219, &chan_node, chan_index,
				    IQS7219_REG_GRP_CHAN_0 + chan_index);
	if (error)
		return error;

	if (!chan_node)
		return 0;

	if (fwnode_property_present(chan_node, "azoteq,rx-enable")) {
		unsigned int pins[4];
		int count;

		count = fwnode_property_count_u32(chan_node,
						  "azoteq,rx-enable");
		if (count < 0) {
			dev_err(&client->dev,
				"Failed to count %s CRx pins: %d\n",
				fwnode_get_name(chan_node), count);
			return count;
		} else if (count > ARRAY_SIZE(pins)) {
			dev_err(&client->dev,
				"Invalid number of %s CRx pins\n",
				fwnode_get_name(chan_node));
			return -EINVAL;
		}

		error = fwnode_property_read_u32_array(chan_node,
						       "azoteq,rx-enable",
						       pins, count);
		if (error) {
			dev_err(&client->dev,
				"Failed to read %s CRx pins: %d\n",
				fwnode_get_name(chan_node), error);
			return error;
		}

		chan_setup[3] &= ~GENMASK(8 + ARRAY_SIZE(pins) - 1, 8);

		for (i = 0; i < count; i++) {
			if (pins[i] >= ARRAY_SIZE(pins)) {
				dev_err(&client->dev,
					"Invalid %s CRx pin: %u\n",
					fwnode_get_name(chan_node), pins[i]);
				return -EINVAL;
			}

			chan_setup[3] |= BIT(pins[i] + 8);
		}
	}

	if (fwnode_property_present(chan_node, "azoteq,tx-enable")) {
		unsigned int pins[9];
		int count;

		count = fwnode_property_count_u32(chan_node,
						  "azoteq,tx-enable");
		if (count < 0) {
			dev_err(&client->dev,
				"Failed to count %s CTx pins: %d\n",
				fwnode_get_name(chan_node), count);
			return count;
		} else if (count > ARRAY_SIZE(pins)) {
			dev_err(&client->dev,
				"Invalid number of %s CTx pins\n",
				fwnode_get_name(chan_node));
			return -EINVAL;
		}

		error = fwnode_property_read_u32_array(chan_node,
						       "azoteq,tx-enable",
						       pins, count);
		if (error) {
			dev_err(&client->dev,
				"Failed to read %s CTx pins: %d\n",
				fwnode_get_name(chan_node), error);
			return error;
		}

		chan_setup[4] &= ~GENMASK(ARRAY_SIZE(pins) - 1, 0);

		for (i = 0; i < count; i++) {
			if (pins[i] >= ARRAY_SIZE(pins)) {
				dev_err(&client->dev,
					"Invalid %s CTx pin: %u\n",
					fwnode_get_name(chan_node), pins[i]);
				return -EINVAL;
			}

			chan_setup[4] |= BIT(pins[i]);
		}
	}

	for (i = 0; i < ARRAY_SIZE(iqs7219_pxs_events); i++) {
		u16 *event_setup = iqs7219->event_setup[chan_index][i];
		struct fwnode_handle *event_node;

		event_setup[4] &= ~GENMASK(7, 0);

		event_node = fwnode_get_named_child_node(chan_node,
							 iqs7219_pxs_events[i]);
		if (!event_node)
			continue;

		error = iqs7219_parse_props(iqs7219, &event_node, i,
					    IQS7219_REG_GRP_EVENT_0 + chan_index);
		if (error)
			return error;

		if (!fwnode_property_present(event_node,
					     "azoteq,trigger-disable"))
			iqs7219->event_mask[chan_index] |= BIT(i);

		chan_setup[0] &= ~BIT(i + 3);
		if (fwnode_property_present(event_node, "azoteq,lta-track"))
			chan_setup[0] |= BIT(i + 3);

		if (!fwnode_property_present(event_node, "azoteq,gpio-select"))
			continue;

		error = fwnode_property_read_u32(event_node,
						 "azoteq,gpio-select", &val);
		if (error) {
			dev_err(&client->dev,
				"Failed to read %s GPIO selection: %d\n",
				fwnode_get_name(event_node), error);
			return error;
		}

		for (j = 0; j < ARRAY_SIZE(iqs7219_gpios); j++)
			if (val == iqs7219_gpios[j])
				break;

		if (j == ARRAY_SIZE(iqs7219_gpios)) {
			dev_err(&client->dev,
				"Invalid %s GPIO selection: %u\n",
				fwnode_get_name(event_node), val);
			return -EINVAL;
		}

		event_setup[4] |= BIT(val);

		if (fwnode_property_present(event_node, "drive-open-drain"))
			event_setup[4] |= IQS7219_GPIO_OPEN_DRAIN;
		else if (fwnode_property_present(event_node,
						 "azoteq,invert-enable"))
			event_setup[4] |= IQS7219_GPIO_ACTIVE_HIGH;

		/*
		 * GPIOs 1, 2 and 5 are shared with the SDA, SCL and RDY pins,
		 * respectively. Selecting any GPIO requires the device to be
		 * placed in stand-alone mode, during which it cannot communi-
		 * cate over I2C.
		 */
		iqs7219->intf_mode = IQS7219_SYS_SETUP_INTF_MODE_STAND;
	}

	if (!fwnode_property_present(chan_node, "azoteq,scan-mux"))
		return 0;

	error = fwnode_property_read_u32(chan_node, "azoteq,scan-mux", &val);
	if (error) {
		dev_err(&client->dev,
			"Failed to read %s scan multiplexer: %d\n",
			fwnode_get_name(chan_node), error);
		return error;
	}

	if (val > IQS7219_NUM_SCAN) {
		dev_err(&client->dev, "Invalid %s scan multiplexer: %u\n",
			fwnode_get_name(chan_node), val);
		return -EINVAL;
	}

	iqs7219->scan_mux[chan_index] = val;

	return 0;
}

static int iqs7219_parse_all(struct iqs7219_private *iqs7219)
{
	struct i2c_client *client = iqs7219->client;
	int error, i;

	if (!device_property_present(&client->dev, "azoteq,streaming-comms"))
		iqs7219->intf_mode = IQS7219_SYS_SETUP_INTF_MODE_EVENT;

	for (i = 0; i < IQS7219_NUM_CYCLES; i++) {
		struct fwnode_handle *cycle_node = NULL;

		error = iqs7219_parse_props(iqs7219, &cycle_node, i,
					    IQS7219_REG_GRP_PXS);
		if (error)
			return error;
	}

	for (i = 0; i < ARRAY_SIZE(iqs7219_sense_modes); i++) {
		u16 *cap_setup = &iqs7219->cap_setup;
		struct fwnode_handle *cap_node;
		unsigned int val;

		cap_node = device_get_named_child_node(&client->dev,
						       iqs7219_sense_modes[i]);
		if (!cap_node)
			continue;

		*cap_setup &= ~(IQS7219_CAP_SETUP_VREF_HALF << i * 8);
		if (fwnode_property_present(cap_node, "azoteq,vref-half"))
			*cap_setup |= (IQS7219_CAP_SETUP_VREF_HALF << i * 8);

		*cap_setup &= ~(IQS7219_CAP_SETUP_SAMP_DOUBLE << i * 8);
		if (fwnode_property_present(cap_node, "azoteq,samp-cap-double"))
			*cap_setup |= (IQS7219_CAP_SETUP_SAMP_DOUBLE << i * 8);

		if (!fwnode_property_present(cap_node, "azoteq,max-counts"))
			continue;

		error = fwnode_property_read_u32(cap_node, "azoteq,max-counts",
						 &val);
		if (error) {
			dev_err(&client->dev,
				"Failed to read %s maximum counts: %d\n",
				fwnode_get_name(cap_node), error);
			return error;
		}

		if (val > IQS7219_CAP_SETUP_COUNTS_MAX) {
			dev_err(&client->dev,
				"Invalid %s maximum counts: %u\n",
				fwnode_get_name(cap_node), val);
			return -EINVAL;
		}

		*cap_setup &= ~(IQS7219_CAP_SETUP_COUNTS_MASK << i * 8);
		*cap_setup |= (val << i * 8);
	}

	error = iqs7219_parse_props(iqs7219, NULL, 0, IQS7219_REG_GRP_CAP);
	if (error)
		return error;

	for (i = 0; i < IQS7219_NUM_CHAN; i++) {
		struct fwnode_handle *ati_node = NULL;

		error = iqs7219_parse_props(iqs7219, &ati_node, i,
					    IQS7219_REG_GRP_ATI);
		if (error)
			return error;

		error = iqs7219_parse_chan(iqs7219, i);
		if (error)
			return error;
	}

	return iqs7219_parse_props(iqs7219, NULL, 0, IQS7219_REG_GRP_SYS);
}

static int iqs7219_report_sync(struct iio_dev *indio_dev,
			       enum iqs7219_scan_id scan_id, int chan_index,
			       int *val)
{
	struct iqs7219_private *iqs7219 = iio_priv(indio_dev);
	struct i2c_client *client = iqs7219->client;
	s64 timestamp = iio_get_time_ns(indio_dev);
	__le16 val_buf[IQS7219_NUM_COLS_STAT];
	__le16 sys_flags, pxs_flags;
	bool trig_pending = false;
	int error, i, j;

	mutex_lock(&iqs7219->lock);

	error = iqs7219_read_burst(iqs7219, IQS7219_SYS_STATUS, val_buf,
				   ARRAY_SIZE(val_buf));
	if (error)
		goto out_mutex;

	sys_flags = le16_to_cpu(val_buf[0]);
	pxs_flags = le16_to_cpu(val_buf[1]);

	if (sys_flags & IQS7219_SYS_STATUS_RESET) {
		dev_err(&client->dev, "Unexpected device reset\n");

		error = iqs7219_dev_init(iqs7219, WRITE);
		if (val && !error)
			error = -EAGAIN;

		goto out_mutex;
	}

	if (sys_flags & IQS7219_SYS_STATUS_ATI_ERROR) {
		dev_err(&client->dev, "Unexpected ATI error\n");

		error = iqs7219_ati_trigger(iqs7219);
		if (val && !error)
			error = -EAGAIN;

		goto out_mutex;
	}

	if (sys_flags & IQS7219_SYS_STATUS_ATI_ACTIVE) {
		if (val)
			error = -EAGAIN;
		goto out_mutex;
	}

	for (i = 0; i < IQS7219_NUM_CHAN; i++) {
		int scan[IQS7219_NUM_SCAN];

		scan[IQS7219_SCAN_FILT] = le16_to_cpu(val_buf[2 + i * 2]);
		scan[IQS7219_SCAN_RAW] = le16_to_cpu(val_buf[10 + i]);
		scan[IQS7219_SCAN_LTA] = le16_to_cpu(val_buf[3 + i * 2]);

		scan[IQS7219_SCAN_VAR] = (le16_to_cpu(val_buf[7 + i * 2])) << 16 |
					  le16_to_cpu(val_buf[6 + i * 2]);

		scan[IQS7219_SCAN_DELTA] = max(scan[IQS7219_SCAN_LTA] -
					       scan[IQS7219_SCAN_FILT], 0);

		scan[IQS7219_SCAN_PXS] = pxs_flags >> i * 4 & GENMASK(3, 0);

		iqs7219->scan_data[i] = scan[iqs7219->scan_mux[i]];

		if (i == chan_index)
			*val = scan[scan_id];

		iqs7219->trig_valid[i] = !iqs7219->intf_mode &&
					  iqs7219->trig_enable;

		for (j = 0; j < ARRAY_SIZE(iqs7219_pxs_events); j++) {
			u16 event_mask = (iqs7219->event_mask[i] & BIT(j)) << i * 4;
			u16 event_state = pxs_flags & event_mask;
			u16 event_cache = iqs7219->pxs_flags & event_mask;
			enum iio_event_direction dir = IIO_EV_DIR_FALLING;

			if (!(event_state ^ event_cache))
				continue;

			iqs7219->trig_valid[i] |= iqs7219->trig_enable;

			if (!iqs7219->event_enable[i])
				continue;

			if (event_state)
				dir = IIO_EV_DIR_RISING;

			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_PROXIMITY, i,
							    IIO_EV_TYPE_THRESH,
							    dir),
				       timestamp);
		}

		trig_pending |= iqs7219->trig_valid[i];
	}

	iqs7219->timestamp = timestamp;
	iqs7219->pxs_flags = pxs_flags;

out_mutex:
	mutex_unlock(&iqs7219->lock);

	if (!error && trig_pending)
		iio_trigger_poll_chained(iqs7219->trig);

	return error;
}

static int iqs7219_report_async(struct iio_dev *indio_dev,
				enum iqs7219_scan_id scan_id, int chan_index,
				int *val)
{
	struct iqs7219_private *iqs7219 = iio_priv(indio_dev);
	int error;

	/*
	 * I2C communication prompts the device to assert its RDY pin if it is
	 * not already asserted. As such, the interrupt must be disabled so as
	 * to prevent reentrant interrupts.
	 */
	disable_irq(gpiod_to_irq(iqs7219->irq_gpio));

	error = iqs7219_report_sync(indio_dev, scan_id, chan_index, val);

	enable_irq(gpiod_to_irq(iqs7219->irq_gpio));

	return error;
}

static irqreturn_t iqs7219_irq(int irq, void *context)
{
	struct iio_dev *indio_dev = context;

	return iqs7219_report_sync(indio_dev, -1, -1, NULL) ? IRQ_NONE
							    : IRQ_HANDLED;
}

static irqreturn_t iqs7219_trigger_consumer(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct iqs7219_private *iqs7219 = iio_priv(indio_dev);
	struct {
#ifdef CONFIG_IQS7219_SINGLE_CHAN
		u32 data[1];
#else
		u32 data[IQS7219_NUM_CHAN];
#endif
		s64 timestamp __aligned(8);
	} scan;
	bool buffer_push = false;
	int error, i, offset = 0;

	/*
	 * If the device is not coupled to its own trigger, its data buffer is
	 * stale at this point and must be refreshed.
	 */
	if (!iio_trigger_using_own(indio_dev)) {
		error = iqs7219_report_async(indio_dev, -1, -1, NULL);
		if (error)
			goto out_notify;

		buffer_push = true;
	}

	memset(&scan, 0, sizeof(scan));

	mutex_lock(&iqs7219->lock);

	for (i = 0; i < ARRAY_SIZE(scan.data); i++) {
		if (!test_bit(i, indio_dev->active_scan_mask))
			continue;

		if (iqs7219->trig_valid[i])
			buffer_push = true;

		scan.data[offset++] = iqs7219->scan_data[i];
	}

	mutex_unlock(&iqs7219->lock);

	if (!buffer_push)
		goto out_notify;

	iio_push_to_buffers_with_timestamp(indio_dev, &scan,
					   iqs7219->timestamp);

out_notify:
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static void iqs7219_read_rate(struct iqs7219_private *iqs7219,
			      int *val, int *val2)
{
	struct i2c_client *client = iqs7219->client;
	int rate_uhz;

	mutex_lock(&iqs7219->lock);

	if (!(iqs7219->sys_setup[0] & ~IQS7219_SYS_SETUP_PWR_MODE_MASK))
		dev_warn(&client->dev,
			 "Device is not locked in normal-power mode\n");

	rate_uhz = IQS7219_UHZ_PER_MS / (iqs7219->sys_setup[5] ? : 1);

	mutex_unlock(&iqs7219->lock);

	*val = rate_uhz / 1000000;
	*val2 = rate_uhz - *val * 1000000;
}

static int iqs7219_write_rate(struct iqs7219_private *iqs7219,
			      int val, int val2)
{
	struct i2c_client *client = iqs7219->client;
	int rate_uhz, rate_ms, error;

	if (val > IQS7219_MAX_RATE_HZ)
		return -EINVAL;

	rate_uhz = val * 1000000 + val2;
	if (!rate_uhz)
		return -EINVAL;

	rate_ms = IQS7219_UHZ_PER_MS / rate_uhz;
	if (rate_ms > IQS7219_MAX_RATE_MS)
		return -EINVAL;

	disable_irq(gpiod_to_irq(iqs7219->irq_gpio));
	mutex_lock(&iqs7219->lock);

	if (!(iqs7219->sys_setup[0] & ~IQS7219_SYS_SETUP_PWR_MODE_MASK))
		dev_warn(&client->dev,
			 "Device is not locked in normal-power mode\n");

	error = iqs7219_write_word(iqs7219, IQS7219_SYS_SETUP + 5, rate_ms);
	if (!error)
		iqs7219->sys_setup[5] = rate_ms;

	mutex_unlock(&iqs7219->lock);
	enable_irq(gpiod_to_irq(iqs7219->irq_gpio));

	return error;
}

static int iqs7219_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct iqs7219_private *iqs7219 = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		return iqs7219_report_async(indio_dev, IQS7219_SCAN_DELTA,
					    chan->channel, val) ? : IIO_VAL_INT;

	case IIO_CHAN_INFO_SAMP_FREQ:
		iqs7219_read_rate(iqs7219, val, val2);
		return IIO_VAL_INT_PLUS_MICRO;

	default:
		return -EINVAL;
	}
}

static int iqs7219_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct iqs7219_private *iqs7219 = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return iqs7219_write_rate(iqs7219, val, val2);

	default:
		return -EINVAL;
	}
}

static int iqs7219_read_event_config(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir)
{
	struct iqs7219_private *iqs7219 = iio_priv(indio_dev);

	return iqs7219->event_enable[chan->channel];
}

static int iqs7219_write_event_config(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      enum iio_event_type type,
				      enum iio_event_direction dir, int state)
{
	struct iqs7219_private *iqs7219 = iio_priv(indio_dev);
	u16 event_mask = IQS7219_EVENT_MASK_ATI;
	int error, i;

	disable_irq(gpiod_to_irq(iqs7219->irq_gpio));
	mutex_lock(&iqs7219->lock);

	for (i = 0; i < IQS7219_NUM_CHAN; i++) {
		if (iqs7219->trig_enable)
			event_mask |= iqs7219->event_mask[i];

		if (i != chan->channel && iqs7219->event_enable[i])
			event_mask |= iqs7219->event_mask[i];
	}

	if (state)
		event_mask |= iqs7219->event_mask[chan->channel];

	error = iqs7219_write_comms(iqs7219, event_mask);
	if (!error)
		iqs7219->event_enable[chan->channel] = state;

	mutex_unlock(&iqs7219->lock);
	enable_irq(gpiod_to_irq(iqs7219->irq_gpio));

	return error;
}

static const struct iio_info iqs7219_info = {
	.read_raw = iqs7219_read_raw,
	.write_raw = iqs7219_write_raw,
	.read_event_config = iqs7219_read_event_config,
	.write_event_config = iqs7219_write_event_config,
};

static int iqs7219_set_trigger_state(struct iio_trigger *trig, bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct iqs7219_private *iqs7219 = iio_priv(indio_dev);
	u16 event_mask = IQS7219_EVENT_MASK_ATI;
	int error, i;

	disable_irq(gpiod_to_irq(iqs7219->irq_gpio));
	mutex_lock(&iqs7219->lock);

	for (i = 0; i < IQS7219_NUM_CHAN; i++)
		if (state || iqs7219->event_enable[i])
			event_mask |= iqs7219->event_mask[i];

	error = iqs7219_write_comms(iqs7219, event_mask);
	if (!error)
		iqs7219->trig_enable = state;

	mutex_unlock(&iqs7219->lock);
	enable_irq(gpiod_to_irq(iqs7219->irq_gpio));

	return error;
}

static const struct iio_trigger_ops iqs7219_trigger_ops = {
	.set_trigger_state = iqs7219_set_trigger_state,
};

static const struct iio_event_spec iqs7219_iio_events[] = {
#ifndef CONFIG_IQS7219_HIDE_EVENTS
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	},
#endif
};

static ssize_t iqs7219_ext_read(struct iio_dev *indio_dev, uintptr_t private,
				const struct iio_chan_spec *chan, char *buf)
{
	int error, val;

	error = iqs7219_report_async(indio_dev, private, chan->channel, &val);

	return error ? : scnprintf(buf, PAGE_SIZE, "%d\n", val);
}

static int iqs7219_scan_mux_set(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan,
				unsigned int scan_mux)
{
	struct iqs7219_private *iqs7219 = iio_priv(indio_dev);

	if (scan_mux > IQS7219_NUM_SCAN)
		return -EINVAL;

	mutex_lock(&iqs7219->lock);

	iqs7219->scan_mux[chan->channel] = scan_mux;

	mutex_unlock(&iqs7219->lock);

	return 0;
}

static int iqs7219_scan_mux_get(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan)
{
	struct iqs7219_private *iqs7219 = iio_priv(indio_dev);

	return iqs7219->scan_mux[chan->channel];
}

static const struct iio_enum iqs7219_scan_mux_enum = {
	.items = iqs7219_scan_names,
	.num_items = ARRAY_SIZE(iqs7219_scan_names),
	.set = iqs7219_scan_mux_set,
	.get = iqs7219_scan_mux_get,
};

static const struct iio_chan_spec_ext_info iqs7219_ext_info[] = {
	{
		.name = IQS7219_NAME_FILT,
		.shared = IIO_SEPARATE,
		.read = iqs7219_ext_read,
		.private = IQS7219_SCAN_FILT,
	},
	{
		.name = IQS7219_NAME_RAW,
		.shared = IIO_SEPARATE,
		.read = iqs7219_ext_read,
		.private = IQS7219_SCAN_RAW,
	},
	{
		.name = IQS7219_NAME_LTA,
		.shared = IIO_SEPARATE,
		.read = iqs7219_ext_read,
		.private = IQS7219_SCAN_LTA,
	},
	{
		.name = IQS7219_NAME_VAR,
		.shared = IIO_SEPARATE,
		.read = iqs7219_ext_read,
		.private = IQS7219_SCAN_VAR,
	},
	{
		.name = IQS7219_NAME_PXS,
		.shared = IIO_SEPARATE,
		.read = iqs7219_ext_read,
		.private = IQS7219_SCAN_PXS,
	},
	IIO_ENUM("scan_mux", IIO_SEPARATE, &iqs7219_scan_mux_enum),
	IIO_ENUM_AVAILABLE("scan_mux", &iqs7219_scan_mux_enum),
	{ }
};

static const struct iio_chan_spec iqs7219_channels_all[] = {
	IQS7219_IIO_CHAN(0),
	IQS7219_IIO_CHAN(1),
	IIO_CHAN_SOFT_TIMESTAMP(2),
};

static const struct iio_chan_spec iqs7219_channels_single[] = {
	{
		.type = IIO_PROXIMITY,
		.scan_index = 0,
		.scan_type = {
			.sign = 'u',
			.realbits = 32,
			.storagebits = 32,
		},
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.event_spec = iqs7219_iio_events,
		.num_event_specs = ARRAY_SIZE(iqs7219_iio_events),
		.ext_info = iqs7219_ext_info,
	},
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

static int iqs7219_probe(struct i2c_client *client)
{
	struct iqs7219_private *iqs7219;
	struct iio_dev *indio_dev;
	unsigned long irq_flags;
	__le16 val_buf[3];
	int error, irq;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*iqs7219));
	if (!indio_dev)
		return -ENOMEM;

	iqs7219 = iio_priv(indio_dev);
	iqs7219->client = client;

	mutex_init(&iqs7219->lock);

	/*
	 * The RDY pin behaves as an interrupt, but must also be polled ahead
	 * of unsolicited I2C communication. As such, it is first opened as a
	 * GPIO and then passed to gpiod_to_irq() to register the interrupt.
	 */
	iqs7219->irq_gpio = devm_gpiod_get(&client->dev, "irq", GPIOD_IN);
	if (IS_ERR(iqs7219->irq_gpio)) {
		error = PTR_ERR(iqs7219->irq_gpio);
		dev_err(&client->dev, "Failed to request IRQ GPIO: %d\n",
			error);
		return error;
	}

	iqs7219->reset_gpio = devm_gpiod_get_optional(&client->dev, "reset",
						      GPIOD_OUT_HIGH);
	if (IS_ERR(iqs7219->reset_gpio)) {
		error = PTR_ERR(iqs7219->reset_gpio);
		dev_err(&client->dev, "Failed to request reset GPIO: %d\n",
			error);
		return error;
	}

	error = iqs7219_hard_reset(iqs7219);
	if (error)
		return error;

	error = iqs7219_read_burst(iqs7219, IQS7219_PROD_NUM, val_buf,
				   ARRAY_SIZE(val_buf));
	if (error)
		return error;

	if (le16_to_cpu(val_buf[0]) != IQS7219_PROD_NUM_A) {
		dev_err(&client->dev, "Invalid product number: %u\n",
			le16_to_cpu(val_buf[0]));
		return -EINVAL;
	}

	if (((le16_to_cpu(val_buf[1]) << 16) |
	      le16_to_cpu(val_buf[2])) >= IQS7219_COMMS_SETUP_MIN_FW)
		iqs7219->comms_setup = IQS7219_COMMS_SETUP;

	error = iqs7219_dev_init(iqs7219, READ);
	if (error)
		return error;

	error = iqs7219_parse_all(iqs7219);
	if (error)
		return error;

	error = iqs7219_dev_init(iqs7219, WRITE);
	if (error)
		return error;

	/*
	 * The device cannot communicate over I2C during stand-alone mode, so
	 * there is no point in proceeding any further.
	 */
	if (iqs7219->intf_mode == IQS7219_SYS_SETUP_INTF_MODE_STAND)
		return 0;

	if (IS_ENABLED(CONFIG_IQS7219_SINGLE_CHAN)) {
		indio_dev->channels = iqs7219_channels_single;
		indio_dev->num_channels = ARRAY_SIZE(iqs7219_channels_single);
	} else {
		indio_dev->channels = iqs7219_channels_all;
		indio_dev->num_channels = ARRAY_SIZE(iqs7219_channels_all);
	}

	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->name = client->name;
	indio_dev->info = &iqs7219_info;

	iqs7219->trig = devm_iio_trigger_alloc(&client->dev, "%s-dev%d",
					       indio_dev->name,
					       iio_device_id(indio_dev));
	if (!iqs7219->trig)
		return -ENOMEM;

	iio_trigger_set_drvdata(iqs7219->trig, indio_dev);
	iqs7219->trig->ops = &iqs7219_trigger_ops;

	error = devm_iio_trigger_register(&client->dev, iqs7219->trig);
	if (error)
		return error;

	error = devm_iio_triggered_buffer_setup(&client->dev, indio_dev, NULL,
						iqs7219_trigger_consumer, NULL);
	if (error)
		return error;

	irq = gpiod_to_irq(iqs7219->irq_gpio);
	if (irq < 0)
		return irq;

	irq_flags = gpiod_is_active_low(iqs7219->irq_gpio) ? IRQF_TRIGGER_LOW
							   : IRQF_TRIGGER_HIGH;
	irq_flags |= IRQF_ONESHOT;

	error = devm_request_threaded_irq(&client->dev, irq, NULL, iqs7219_irq,
					  irq_flags, client->name, indio_dev);
	if (error) {
		dev_err(&client->dev, "Failed to request IRQ: %d\n", error);
		return error;
	}

	return devm_iio_device_register(&client->dev, indio_dev);
}

static const struct of_device_id iqs7219_of_match[] = {
	{ .compatible = "azoteq,iqs7219a" },
	{ }
};
MODULE_DEVICE_TABLE(of, iqs7219_of_match);

static struct i2c_driver iqs7219_i2c_driver = {
	.driver = {
		.name = "iqs7219",
		.of_match_table = iqs7219_of_match,
	},
	.probe_new = iqs7219_probe,
};
module_i2c_driver(iqs7219_i2c_driver);

MODULE_AUTHOR("Jeff LaBundy <jeff@labundy.com>");
MODULE_DESCRIPTION("Azoteq IQS7219A SAR Proximity Sensor");
MODULE_LICENSE("GPL");

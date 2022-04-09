// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Azoteq IQS323 Capacitive/Inductive Sensing Controller
 *
 * Copyright (C) 2023 Azoteq (Pty) Ltd
 * Author: Jeff LaBundy <jeff@labundy.com>
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <asm/unaligned.h>

#define IQS323_PROD_NUM				0x00
#define IQS323_PROD_NUM_REL			1106
#define IQS323_PROD_NUM_MOV			1462

#define IQS323_SYS_STATUS			0x10
#define IQS323_SYS_STATUS_RESET			BIT(7)
#define IQS323_SYS_STATUS_ATI_ERROR		BIT(6)
#define IQS323_SYS_STATUS_ATI_ACTIVE		BIT(5)

#define IQS323_SLDR_STATUS_BUSY			BIT(7)
#define IQS323_SLDR_STATUS_EVENT		BIT(6)

#define IQS323_MOV_STATUS			0x23

#define IQS323_SNSR_SETUP_0_MOV_EN		BIT(6)
#define IQS323_SNSR_SETUP_0_CHAN_EN		BIT(0)
#define IQS323_SNSR_SETUP_2_SENSE_MODE_MASK	GENMASK(5, 0)
#define IQS323_SNSR_SETUP_4_WAVE_PAT_1_MASK	GENMASK(15, 12)
#define IQS323_SNSR_SETUP_4_WAVE_PAT_0_MASK	GENMASK(11, 8)
#define IQS323_SNSR_SETUP_5_WAVE_SEL_MASK	GENMASK(7, 0)

#define IQS323_CHAN_SETUP_0_REF_SEL_MASK	GENMASK(7, 4)
#define IQS323_CHAN_SETUP_0_REF_SEL_SHIFT	4
#define IQS323_CHAN_SETUP_0_REF_MODE_MASK	GENMASK(3, 0)
#define IQS323_CHAN_SETUP_0_REF_MODE_FOLLOW	BIT(0)
#define IQS323_CHAN_SETUP_0_REF_MODE_REF	BIT(1)

#define IQS323_SLDR_SETUP_0_CHAN_CNT_MASK	GENMASK(2, 0)

#define IQS323_SYS_SETUP			0xC0
#define IQS323_SYS_SETUP_EVENT_MODE		BIT(7)
#define IQS323_SYS_SETUP_POWER_MODE_MASK	GENMASK(6, 4)
#define IQS323_SYS_SETUP_POWER_MODE_SHIFT	4
#define IQS323_SYS_SETUP_REDO_ATI		BIT(2)
#define IQS323_SYS_SETUP_ACK_RESET		BIT(0)

#define IQS323_GEN_SETUP_0_GPIO_INV		BIT(15)
#define IQS323_GEN_SETUP_0_GPIO_SET		GENMASK(14, 0)

#define IQS323_EVENT_MASK_ALL			GENMASK(6, 0)
#define IQS323_EVENT_MASK_ATI			(BIT(6) | BIT(4))
#define IQS323_EVENT_MASK_SLDR			BIT(2)
#define IQS323_EVENT_MASK_TOUCH			BIT(1)
#define IQS323_EVENT_MASK_PROX			BIT(0)

#define IQS323_COMMS_ERROR			0xEEEE
#define IQS323_COMMS_RETRY_MS			50
#define IQS323_COMMS_SLEEP_US			100
#define IQS323_COMMS_TIMEOUT_US			(100 * USEC_PER_MSEC)
#define IQS323_RESET_TIMEOUT_MS			300
#define IQS323_ATI_TIMEOUT_US			(2 * USEC_PER_SEC)

#define IQS323_MAX_COLS_SNSR			10
#define IQS323_MAX_COLS_CHAN			5
#define IQS323_MAX_COLS_SLDR			9
#define IQS323_MAX_COLS_GEST			7
#define IQS323_MAX_COLS_FILT			5
#define IQS323_MAX_COLS_SYS			6
#define IQS323_MAX_COLS_GEN			5

#define IQS323_MIN_REPORT_LEN			10
#define IQS323_MAX_REPORT_LEN			22

#define IQS323_NUM_CHAN				3
#define IQS323_NUM_RETRIES			5
#define IQS323_REG_OFFSET			0x10

/*
 * The following delay is used during instances that must wait for the open-
 * drain RDY pin to settle. Its value is calculated as 5*R*C, where R and C
 * represent typical datasheet values of 4.7k and 100 nF, respectively.
 */
#define iqs323_irq_wait()			usleep_range(2500, 2600)

enum iqs323_intf_mode_id {
	IQS323_INTF_MODE_ERROR,
	IQS323_INTF_MODE_EVENT,
	IQS323_INTF_MODE_STREAM,
};

enum iqs323_reg_key_id {
	IQS323_REG_KEY_NONE,
	IQS323_REG_KEY_PROX,
	IQS323_REG_KEY_TOUCH,
	IQS323_REG_KEY_MOV,
	IQS323_REG_KEY_TAP,
	IQS323_REG_KEY_HOLD,
	IQS323_REG_KEY_AXIAL,
	IQS323_REG_KEY_RESERVED
};

enum iqs323_reg_grp_id {
	IQS323_REG_GRP_SNSR,
	IQS323_REG_GRP_CHAN,
	IQS323_REG_GRP_SLDR,
	IQS323_REG_GRP_GEST,
	IQS323_REG_GRP_FILT,
	IQS323_REG_GRP_SYS,
	IQS323_REG_GRP_GEN,
	IQS323_REG_GRP_GPIO,
	IQS323_REG_GRP_REL,
	IQS323_NUM_REG_GRPS
};

static const char * const iqs323_reg_grp_names[IQS323_NUM_REG_GRPS] = {
	[IQS323_REG_GRP_SNSR] = "channel-%d",
	[IQS323_REG_GRP_CHAN] = "channel-%d",
	[IQS323_REG_GRP_SLDR] = "slider",
	[IQS323_REG_GRP_GPIO] = "gpio",
	[IQS323_REG_GRP_REL] = "release",
};

static const unsigned int iqs323_max_cols[IQS323_NUM_REG_GRPS] = {
	[IQS323_REG_GRP_SNSR] = IQS323_MAX_COLS_SNSR,
	[IQS323_REG_GRP_CHAN] = IQS323_MAX_COLS_CHAN,
	[IQS323_REG_GRP_SLDR] = IQS323_MAX_COLS_SLDR,
	[IQS323_REG_GRP_GEST] = IQS323_MAX_COLS_GEST,
	[IQS323_REG_GRP_FILT] = IQS323_MAX_COLS_FILT,
	[IQS323_REG_GRP_SYS] = IQS323_MAX_COLS_SYS,
	[IQS323_REG_GRP_GEN] = IQS323_MAX_COLS_GEN,
};

enum iqs323_power_mode_id {
	IQS323_POWER_MODE_NP,
	IQS323_POWER_MODE_LP,
	IQS323_POWER_MODE_ULP,
	IQS323_POWER_MODE_HALT,
	IQS323_POWER_MODE_AUTO,
	IQS323_POWER_MODE_AUTO_NO_ULP,
	IQS323_POWER_MODE_USER
};

static const u16 iqs323_sense_modes[] = { 0x10, 0x13, 0x1D, 0x3D, };

static const u16 iqs323_wave_patterns[] = { 0x0300, 0x0E00, 0, 0x0B00, };

static const u16 iqs323_rx_inactives[] = { 0x00, 0x05, 0x0A, 0x0F, };

struct iqs323_event_desc {
	const char *name;
	u16 mask;
	u16 enable;
	enum iqs323_reg_key_id reg_key;
};

static const struct iqs323_event_desc iqs323_kp_events[] = {
	{
		.name = "event-prox",
		.mask = IQS323_EVENT_MASK_PROX,
		.enable = IQS323_EVENT_MASK_PROX,
		.reg_key = IQS323_REG_KEY_PROX,
	},
	{
		.name = "event-touch",
		.mask = IQS323_EVENT_MASK_TOUCH,
		.enable = IQS323_EVENT_MASK_TOUCH,
		.reg_key = IQS323_REG_KEY_TOUCH,
	},
	{
		.name = "movement",
		.reg_key = IQS323_REG_KEY_MOV,
	},
};

static const struct iqs323_event_desc iqs323_sl_events[] = {
	{ .name = "event-press", },
	{
		.name = "event-tap",
		.mask = BIT(0),
		.enable = BIT(0),
		.reg_key = IQS323_REG_KEY_TAP,
	},
	{
		.name = "event-swipe-pos",
		.mask = BIT(1) | IQS323_SLDR_STATUS_BUSY,
		.enable = BIT(1),
		.reg_key = IQS323_REG_KEY_AXIAL,
	},
	{
		.name = "event-swipe-neg",
		.mask = BIT(2) | IQS323_SLDR_STATUS_BUSY,
		.enable = BIT(1),
		.reg_key = IQS323_REG_KEY_AXIAL,
	},
	{
		.name = "event-flick-pos",
		.mask = BIT(3),
		.enable = BIT(2),
		.reg_key = IQS323_REG_KEY_AXIAL,
	},
	{
		.name = "event-flick-neg",
		.mask = BIT(4),
		.enable = BIT(2),
		.reg_key = IQS323_REG_KEY_AXIAL,
	},
	{
		.name = "event-hold",
		.mask = BIT(5) | IQS323_SLDR_STATUS_BUSY,
		.enable = BIT(3),
		.reg_key = IQS323_REG_KEY_HOLD,
	},
};

struct iqs323_reg_grp_desc {
	u16 base;
	int num_row;
	int num_col;
};

struct iqs323_dev_desc {
	u16 prod_num;
	u16 touch_link;
	u16 delta_links[IQS323_NUM_CHAN];
	struct iqs323_reg_grp_desc reg_grps[IQS323_NUM_REG_GRPS];
};

static const struct iqs323_dev_desc iqs323_devs[] = {
	{
		.prod_num = IQS323_PROD_NUM_REL,
		.touch_link = 0x552,
		.delta_links = { 0x430, 0x472, 0x4B4, },
		.reg_grps = {
			[IQS323_REG_GRP_SNSR] = {
				.base = 0x30,
				.num_row = IQS323_NUM_CHAN,
				.num_col = IQS323_MAX_COLS_SNSR,
			},
			[IQS323_REG_GRP_CHAN] = {
				.base = 0x60,
				.num_row = IQS323_NUM_CHAN,
				.num_col = IQS323_MAX_COLS_CHAN - 1,
			},
			[IQS323_REG_GRP_SLDR] = {
				.base = 0x90,
				.num_row = 1,
				.num_col = IQS323_MAX_COLS_SLDR,
			},
			[IQS323_REG_GRP_GEST] = {
				.base = 0xA0,
				.num_row = 1,
				.num_col = IQS323_MAX_COLS_GEST,
			},
			[IQS323_REG_GRP_FILT] = {
				.base = 0xB0,
				.num_row = 1,
				.num_col = IQS323_MAX_COLS_FILT,
			},
			[IQS323_REG_GRP_SYS] = {
				.base = IQS323_SYS_SETUP,
				.num_row = 1,
				.num_col = IQS323_MAX_COLS_SYS,
			},
			[IQS323_REG_GRP_GEN] = {
				.base = 0xD0,
				.num_row = 1,
				.num_col = IQS323_MAX_COLS_GEN,
			},
			[IQS323_REG_GRP_GPIO] = { .num_row = 1, },
			[IQS323_REG_GRP_REL] = { .num_row = 1, },
		},
	},
	{
		.prod_num = IQS323_PROD_NUM_MOV,
		.touch_link = 0x558,
		.delta_links = { 0x430, 0x474, 0x4B8, },
		.reg_grps = {
			[IQS323_REG_GRP_SNSR] = {
				.base = 0x30,
				.num_row = IQS323_NUM_CHAN,
				.num_col = IQS323_MAX_COLS_SNSR,
			},
			[IQS323_REG_GRP_CHAN] = {
				.base = 0x60,
				.num_row = IQS323_NUM_CHAN,
				.num_col = IQS323_MAX_COLS_CHAN,
			},
			[IQS323_REG_GRP_SLDR] = {
				.base = 0x90,
				.num_row = 1,
				.num_col = IQS323_MAX_COLS_SLDR,
			},
			[IQS323_REG_GRP_GEST] = {
				.base = 0xA0,
				.num_row = 1,
				.num_col = IQS323_MAX_COLS_GEST,
			},
			[IQS323_REG_GRP_FILT] = {
				.base = 0xB0,
				.num_row = 1,
				.num_col = IQS323_MAX_COLS_FILT,
			},
			[IQS323_REG_GRP_SYS] = {
				.base = IQS323_SYS_SETUP,
				.num_row = 1,
				.num_col = IQS323_MAX_COLS_SYS,
			},
			[IQS323_REG_GRP_GEN] = {
				.base = 0xD0,
				.num_row = 1,
				.num_col = IQS323_MAX_COLS_GEN,
			},
			[IQS323_REG_GRP_GPIO] = { .num_row = 1, },
		},
	},
};

struct iqs323_prop_desc {
	const char *name;
	enum iqs323_reg_grp_id reg_grp;
	enum iqs323_reg_key_id reg_key;
	int reg_offset;
	int reg_shift;
	int reg_width;
	const u16 *val_subs;
	int val_pitch;
	int val_min;
	int val_max;
	bool invert;
	const char *label;
};

static const struct iqs323_prop_desc iqs323_props[] = {
	{
		.name = "azoteq,tx-freq-fosc",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 0,
		.reg_shift = 5,
		.reg_width = 1,
	},
	{
		.name = "azoteq,vbias-enable",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 0,
		.reg_shift = 4,
		.reg_width = 1,
	},
	{
		.name = "azoteq,invert-enable",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 0,
		.reg_shift = 3,
		.reg_width = 1,
	},
	{
		.name = "azoteq,dual-direction",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 0,
		.reg_shift = 2,
		.reg_width = 1,
	},
	{
		.name = "azoteq,linearize",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 0,
		.reg_shift = 1,
		.reg_width = 1,
	},
	{
		.name = "azoteq,conv-period",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 1,
		.reg_shift = 8,
		.reg_width = 8,
		.label = "conversion period",
	},
	{
		.name = "azoteq,conv-frac",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 1,
		.reg_shift = 0,
		.reg_width = 8,
		.label = "conversion frequency fractional divider",
	},
	{
		.name = "azoteq,vref-half",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 2,
		.reg_shift = 14,
		.reg_width = 1,
	},
	{
		.name = "azoteq,samp-cap-double",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 2,
		.reg_shift = 12,
		.reg_width = 1,
	},
	{
		.name = "azoteq,proj-bias",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 2,
		.reg_shift = 8,
		.reg_width = 2,
		.label = "projected bias current",
	},
	{
		.name = "azoteq,max-counts",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 2,
		.reg_shift = 6,
		.reg_width = 2,
		.label = "maximum counts",
	},
	{
		.name = "azoteq,sense-mode",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 2,
		.reg_shift = 0,
		.reg_width = 6,
		.val_subs = iqs323_sense_modes,
		.val_max = ARRAY_SIZE(iqs323_sense_modes) - 1,
		.label = "sensing mode",
	},
	{
		.name = "azoteq,tref-enable",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 3,
		.reg_shift = 13,
		.reg_width = 1,
	},
	{
		.name = "azoteq,iref-enable",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 3,
		.reg_shift = 12,
		.reg_width = 1,
	},
	{
		.name = "azoteq,dead-time-enable",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 3,
		.reg_shift = 6,
		.reg_width = 1,
	},
	{
		.name = "azoteq,auto-mode",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 3,
		.reg_shift = 2,
		.reg_width = 2,
		.label = "number of conversions",
	},
	{
		.name = "azoteq,wave-pattern-1",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 4,
		.reg_shift = 12,
		.reg_width = 4,
		.label = "waveform pattern 1",
	},
	{
		.name = "azoteq,wave-pattern-0",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 4,
		.reg_shift = 8,
		.reg_width = 4,
		.label = "waveform pattern 0",
	},
	{
		.name = "azoteq,rx-inactive",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 4,
		.reg_shift = 0,
		.reg_width = 4,
		.val_subs = iqs323_rx_inactives,
		.val_max = ARRAY_SIZE(iqs323_rx_inactives) - 1,
		.label = "inactive CRx pin termination"
	},
	{
		.name = "azoteq,iref-level",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 5,
		.reg_shift = 12,
		.reg_width = 4,
		.label = "current reference level",
	},
	{
		.name = "azoteq,iref-trim",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 5,
		.reg_shift = 8,
		.reg_width = 4,
		.label = "current reference trim",
	},
	{
		.name = "azoteq,ati-factor",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 6,
		.reg_shift = 4,
		.reg_width = 12,
		.label = "ATI resolution factor",
	},
	{
		.name = "azoteq,ati-band-tighten",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 6,
		.reg_shift = 3,
		.reg_width = 1,
		.invert = true,
	},
	{
		.name = "azoteq,ati-mode",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 6,
		.reg_shift = 0,
		.reg_width = 3,
		.val_max = 4,
		.label = "ATI mode",
	},
	{
		.name = "azoteq,ati-base",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 7,
		.reg_shift = 0,
		.reg_width = 16,
		.val_max = 16384,
		.label = "ATI base",
	},
	{
		.name = "azoteq,ati-frac-mult-fine",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 8,
		.reg_shift = 14,
		.reg_width = 2,
		.label = "ATI fine fractional multiplier",
	},
	{
		.name = "azoteq,ati-frac-div-fine",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 8,
		.reg_shift = 9,
		.reg_width = 5,
		.label = "ATI fine fractional divider",
	},
	{
		.name = "azoteq,ati-frac-mult-coarse",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 8,
		.reg_shift = 5,
		.reg_width = 4,
		.label = "ATI coarse fractional multiplier",
	},
	{
		.name = "azoteq,ati-frac-div-coarse",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 8,
		.reg_shift = 0,
		.reg_width = 5,
		.label = "ATI coarse fractional divider",
	},
	{
		.name = "azoteq,ati-comp-div",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 9,
		.reg_shift = 11,
		.reg_width = 5,
		.label = "ATI compensation divider",
	},
	{
		.name = "azoteq,ati-comp-select",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 9,
		.reg_shift = 0,
		.reg_width = 10,
		.label = "ATI compensation selection",
	},
	{
		.name = "azoteq,debounce-exit",
		.reg_grp = IQS323_REG_GRP_CHAN,
		.reg_key = IQS323_REG_KEY_PROX,
		.reg_offset = 1,
		.reg_shift = 12,
		.reg_width = 4,
		.label = "debounce exit factor",
	},
	{
		.name = "azoteq,debounce-enter",
		.reg_grp = IQS323_REG_GRP_CHAN,
		.reg_key = IQS323_REG_KEY_PROX,
		.reg_offset = 1,
		.reg_shift = 8,
		.reg_width = 4,
		.label = "debounce entrance factor",
	},
	{
		.name = "azoteq,thresh",
		.reg_grp = IQS323_REG_GRP_CHAN,
		.reg_key = IQS323_REG_KEY_PROX,
		.reg_offset = 1,
		.reg_shift = 0,
		.reg_width = 8,
		.label = "threshold",
	},
	{
		.name = "azoteq,thresh",
		.reg_grp = IQS323_REG_GRP_CHAN,
		.reg_key = IQS323_REG_KEY_TOUCH,
		.reg_offset = 2,
		.reg_shift = 0,
		.reg_width = 8,
		.label = "threshold",
	},
	{
		.name = "azoteq,hyst",
		.reg_grp = IQS323_REG_GRP_CHAN,
		.reg_key = IQS323_REG_KEY_TOUCH,
		.reg_offset = 2,
		.reg_shift = 8,
		.reg_width = 8,
		.label = "hysteresis",
	},
	{
		.name = "azoteq,debounce-exit",
		.reg_grp = IQS323_REG_GRP_CHAN,
		.reg_key = IQS323_REG_KEY_MOV,
		.reg_offset = 4,
		.reg_shift = 12,
		.reg_width = 4,
		.label = "debounce exit factor",
	},
	{
		.name = "azoteq,debounce-enter",
		.reg_grp = IQS323_REG_GRP_CHAN,
		.reg_key = IQS323_REG_KEY_MOV,
		.reg_offset = 4,
		.reg_shift = 8,
		.reg_width = 4,
		.label = "debounce entrance factor",
	},
	{
		.name = "azoteq,thresh",
		.reg_grp = IQS323_REG_GRP_CHAN,
		.reg_key = IQS323_REG_KEY_MOV,
		.reg_offset = 4,
		.reg_shift = 0,
		.reg_width = 8,
		.label = "threshold",
	},
	{
		.name = "azoteq,lower-cal",
		.reg_grp = IQS323_REG_GRP_SLDR,
		.reg_offset = 0,
		.reg_shift = 8,
		.reg_width = 8,
		.label = "lower calibration",
	},
	{
		.name = "azoteq,static-beta",
		.reg_grp = IQS323_REG_GRP_SLDR,
		.reg_offset = 0,
		.reg_shift = 6,
		.reg_width = 1,
	},
	{
		.name = "azoteq,bottom-beta",
		.reg_grp = IQS323_REG_GRP_SLDR,
		.reg_offset = 0,
		.reg_shift = 3,
		.reg_width = 3,
		.label = "bottom beta",
	},
	{
		.name = "azoteq,bottom-speed",
		.reg_grp = IQS323_REG_GRP_SLDR,
		.reg_offset = 1,
		.reg_shift = 8,
		.reg_width = 8,
		.label = "bottom speed",
	},
	{
		.name = "azoteq,upper-cal",
		.reg_grp = IQS323_REG_GRP_SLDR,
		.reg_offset = 1,
		.reg_shift = 0,
		.reg_width = 8,
		.label = "upper calibration",
	},
	{
		.name = "azoteq,top-speed",
		.reg_grp = IQS323_REG_GRP_SLDR,
		.reg_offset = 2,
		.reg_shift = 0,
		.reg_width = 16,
		.label = "top speed",
	},
	{
		.name = "azoteq,slider-size",
		.reg_grp = IQS323_REG_GRP_SLDR,
		.reg_offset = 3,
		.reg_shift = 0,
		.reg_width = 16,
		.label = "size",
	},
	{
		.name = "azoteq,gesture-min-ms",
		.reg_grp = IQS323_REG_GRP_GEST,
		.reg_key = IQS323_REG_KEY_TAP,
		.reg_offset = 1,
		.reg_shift = 0,
		.reg_width = 16,
		.label = "minimum gesture time",
	},
	{
		.name = "azoteq,gesture-max-ms",
		.reg_grp = IQS323_REG_GRP_GEST,
		.reg_key = IQS323_REG_KEY_TAP,
		.reg_offset = 2,
		.reg_shift = 0,
		.reg_width = 16,
		.label = "maximum gesture time",
	},
	{
		.name = "azoteq,gesture-max-ms",
		.reg_grp = IQS323_REG_GRP_GEST,
		.reg_key = IQS323_REG_KEY_AXIAL,
		.reg_offset = 3,
		.reg_shift = 0,
		.reg_width = 16,
		.label = "maximum gesture time",
	},
	{
		.name = "azoteq,gesture-min-ms",
		.reg_grp = IQS323_REG_GRP_GEST,
		.reg_key = IQS323_REG_KEY_HOLD,
		.reg_offset = 4,
		.reg_shift = 0,
		.reg_width = 16,
		.label = "maximum gesture time",
	},
	{
		.name = "azoteq,gesture-dist",
		.reg_grp = IQS323_REG_GRP_GEST,
		.reg_key = IQS323_REG_KEY_TAP,
		.reg_offset = 5,
		.reg_shift = 0,
		.reg_width = 16,
		.label = "gesture distance",
	},
	{
		.name = "azoteq,gesture-dist",
		.reg_grp = IQS323_REG_GRP_GEST,
		.reg_key = IQS323_REG_KEY_AXIAL,
		.reg_offset = 6,
		.reg_shift = 0,
		.reg_width = 16,
		.label = "gesture distance",
	},
	{
		.name = "azoteq,counts-beta-lp",
		.reg_grp = IQS323_REG_GRP_FILT,
		.reg_offset = 0,
		.reg_shift = 8,
		.reg_width = 8,
		.label = "low-power mode counts beta",
	},
	{
		.name = "azoteq,counts-beta-np",
		.reg_grp = IQS323_REG_GRP_FILT,
		.reg_offset = 0,
		.reg_shift = 0,
		.reg_width = 8,
		.label = "normal-power mode counts beta",
	},
	{
		.name = "azoteq,lta-beta-lp",
		.reg_grp = IQS323_REG_GRP_FILT,
		.reg_offset = 1,
		.reg_shift = 8,
		.reg_width = 8,
		.label = "low-power mode long-term average beta",
	},
	{
		.name = "azoteq,lta-beta-np",
		.reg_grp = IQS323_REG_GRP_FILT,
		.reg_offset = 1,
		.reg_shift = 0,
		.reg_width = 8,
		.label = "normal-power mode long-term average beta",
	},
	{
		.name = "azoteq,lta-fast-beta-lp",
		.reg_grp = IQS323_REG_GRP_FILT,
		.reg_offset = 2,
		.reg_shift = 8,
		.reg_width = 8,
		.label = "low-power mode long-term average fast beta",
	},
	{
		.name = "azoteq,lta-fast-beta-np",
		.reg_grp = IQS323_REG_GRP_FILT,
		.reg_offset = 2,
		.reg_shift = 0,
		.reg_width = 8,
		.label = "normal-power mode long-term average fast beta",
	},
	{
		.name = "azoteq,press-lta-beta-lp",
		.reg_grp = IQS323_REG_GRP_FILT,
		.reg_offset = 3,
		.reg_shift = 8,
		.reg_width = 8,
		.label = "low-power mode press long-term average beta",
	},
	{
		.name = "azoteq,press-lta-beta-np",
		.reg_grp = IQS323_REG_GRP_FILT,
		.reg_offset = 3,
		.reg_shift = 0,
		.reg_width = 8,
		.label = "normal-power mode press long-term average beta",
	},
	{
		.name = "azoteq,fast-filt-band",
		.reg_grp = IQS323_REG_GRP_FILT,
		.reg_offset = 4,
		.reg_shift = 0,
		.reg_width = 16,
		.label = "fast filter band",
	},
	{
		.name = "azoteq,power-mode",
		.reg_grp = IQS323_REG_GRP_SYS,
		.reg_offset = 0,
		.reg_shift = IQS323_SYS_SETUP_POWER_MODE_SHIFT,
		.reg_width = 3,
		.val_max = IQS323_POWER_MODE_AUTO_NO_ULP,
		.label = "power mode",
	},
	{
		.name = "azoteq,rate-np-ms",
		.reg_grp = IQS323_REG_GRP_SYS,
		.reg_offset = 1,
		.reg_shift = 0,
		.reg_width = 16,
		.val_max = 3000,
		.label = "normal-power mode report rate",
	},
	{
		.name = "azoteq,rate-lp-ms",
		.reg_grp = IQS323_REG_GRP_SYS,
		.reg_offset = 2,
		.reg_shift = 0,
		.reg_width = 16,
		.val_max = 3000,
		.label = "low-power mode report rate",
	},
	{
		.name = "azoteq,rate-ulp-ms",
		.reg_grp = IQS323_REG_GRP_SYS,
		.reg_offset = 3,
		.reg_shift = 0,
		.reg_width = 16,
		.val_max = 3000,
		.label = "ultra-low-power mode report rate",
	},
	{
		.name = "azoteq,rate-halt-ms",
		.reg_grp = IQS323_REG_GRP_SYS,
		.reg_offset = 4,
		.reg_shift = 0,
		.reg_width = 16,
		.val_max = 3000,
		.label = "halt mode report rate",
	},
	{
		.name = "azoteq,timeout-xp-ms",
		.reg_grp = IQS323_REG_GRP_SYS,
		.reg_offset = 5,
		.reg_shift = 0,
		.reg_width = 16,
		.val_max = 65000,
		.label = "power mode timeout",
	},
	{
		.name = "azoteq,timeout-comms-ms",
		.reg_grp = IQS323_REG_GRP_GEN,
		.reg_offset = 1,
		.reg_shift = 0,
		.reg_width = 16,
		.val_min = 2,
		.val_max = 230,
		.label = "communication timeout",
	},
	{
		.name = "azoteq,timeout-press-ms",
		.reg_grp = IQS323_REG_GRP_GEN,
		.reg_key = IQS323_REG_KEY_TOUCH,
		.reg_offset = 2,
		.reg_shift = 8,
		.reg_width = 8,
		.val_pitch = 512,
		.label = "press timeout",
	},
	{
		.name = "azoteq,timeout-press-ms",
		.reg_grp = IQS323_REG_GRP_GEN,
		.reg_key = IQS323_REG_KEY_PROX,
		.reg_offset = 2,
		.reg_shift = 0,
		.reg_width = 8,
		.val_pitch = 512,
		.label = "press timeout",
	},
	{
		.name = "azoteq,thresh",
		.reg_grp = IQS323_REG_GRP_REL,
		.reg_offset = 3,
		.reg_shift = 8,
		.reg_width = 8,
		.label = "threshold",
	},
	{
		.name = "azoteq,delta-delay",
		.reg_grp = IQS323_REG_GRP_REL,
		.reg_offset = 4,
		.reg_shift = 8,
		.reg_width = 8,
		.label = "delta snapshot sample delay",
	},
	{
		.name = "azoteq,delta-weight",
		.reg_grp = IQS323_REG_GRP_REL,
		.reg_offset = 4,
		.reg_shift = 0,
		.reg_width = 8,
		.val_max = 128,
		.label = "delta weight",
	},
	{
		.name = "azoteq,timeout-press-ms",
		.reg_grp = IQS323_REG_GRP_GEN,
		.reg_key = IQS323_REG_KEY_MOV,
		.reg_offset = 4,
		.reg_shift = 0,
		.reg_width = 16,
		.val_pitch = 512,
		.label = "press timeout",
	},
};

struct iqs323_prop_desc iqs323_pins[] = {
	{
		.name = "azoteq,tx-select",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 5,
		.reg_shift = 0,
		.reg_width = 8,
		.val_max = 4,
		.label = "CTx",
	},
	{
		.name = "azoteq,rx-enable",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 3,
		.reg_shift = 8,
		.reg_width = 3,
		.val_max = 3,
		.label = "CRx",
	},
	{
		.name = "azoteq,tx-enable",
		.reg_grp = IQS323_REG_GRP_SNSR,
		.reg_offset = 0,
		.reg_shift = 8,
		.reg_width = 4,
		.val_max = 4,
		.label = "CTx",
	},
};

struct iqs323_ver_info {
	__le16 prod_num;
	__le16 major;
	__le16 minor;
	__le32 patch;
} __packed;

struct iqs323_private {
	const struct iqs323_dev_desc *dev_desc;
	struct gpio_desc *irq_gpio;
	struct i2c_client *client;
	struct input_dev *kp_idev;
	struct iqs323_ver_info ver_info;
	enum iqs323_intf_mode_id intf_mode;
	unsigned int kp_type[IQS323_NUM_CHAN][ARRAY_SIZE(iqs323_kp_events)];
	unsigned int kp_code[IQS323_NUM_CHAN][ARRAY_SIZE(iqs323_kp_events)];
	unsigned int sl_code[ARRAY_SIZE(iqs323_sl_events)];
	unsigned int sl_axis;
	u16 snsr_setup[IQS323_NUM_CHAN][IQS323_MAX_COLS_SNSR];
	u16 chan_setup[IQS323_NUM_CHAN][IQS323_MAX_COLS_CHAN];
	u16 sldr_setup[IQS323_MAX_COLS_SLDR];
	u16 gest_setup[IQS323_MAX_COLS_GEST];
	u16 filt_setup[IQS323_MAX_COLS_FILT];
	u16 sys_setup[IQS323_MAX_COLS_SYS];
	u16 gen_setup[IQS323_MAX_COLS_GEN];
	u16 wear_mask;
};

static u16 *iqs323_setup(struct iqs323_private *iqs323,
			 enum iqs323_reg_grp_id reg_grp, int row)
{
	switch (reg_grp) {
	case IQS323_REG_GRP_SNSR:
		return iqs323->snsr_setup[row];

	case IQS323_REG_GRP_CHAN:
		return iqs323->chan_setup[row];

	case IQS323_REG_GRP_SLDR:
		return iqs323->sldr_setup;

	case IQS323_REG_GRP_GEST:
		return iqs323->gest_setup;

	case IQS323_REG_GRP_FILT:
		return iqs323->filt_setup;

	case IQS323_REG_GRP_SYS:
		return iqs323->sys_setup;

	case IQS323_REG_GRP_GEN:
	case IQS323_REG_GRP_REL:
		return iqs323->gen_setup;

	default:
		return NULL;
	}
}

static int iqs323_irq_poll(struct iqs323_private *iqs323, u64 timeout_us)
{
	int error, val;

	error = readx_poll_timeout(gpiod_get_value_cansleep, iqs323->irq_gpio,
				   val, val, IQS323_COMMS_SLEEP_US, timeout_us);

	return val < 0 ? val : error;
}

static int iqs323_hard_reset(struct iqs323_private *iqs323)
{
	gpiod_set_value_cansleep(iqs323->irq_gpio, 1);

	/*
	 * The following delay ensures the shared RDY/MCLR pin is sampled in
	 * between periodic assertions by the device.
	 */
	msleep(IQS323_RESET_TIMEOUT_MS);

	gpiod_set_value_cansleep(iqs323->irq_gpio, 0);
	iqs323_irq_wait();

	return iqs323_irq_poll(iqs323, IQS323_COMMS_TIMEOUT_US);
}

static int iqs323_force_comms(struct iqs323_private *iqs323)
{
	u8 msg_buf[] = { 0xFF, };
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
	ret = gpiod_get_value_cansleep(iqs323->irq_gpio);
	if (ret < 0)
		return ret;
	else if (ret > 0)
		return 0;

	ret = i2c_master_send(iqs323->client, msg_buf, sizeof(msg_buf));
	if (ret < (int)sizeof(msg_buf)) {
		if (ret >= 0)
			ret = -EIO;

		msleep(IQS323_COMMS_RETRY_MS);
		return ret;
	}

	iqs323_irq_wait();

	return iqs323_irq_poll(iqs323, IQS323_COMMS_TIMEOUT_US);
}

static int iqs323_read_burst(struct iqs323_private *iqs323,
			     u8 reg, void *val, u16 num_val)
{
	int ret, i;
	struct i2c_client *client = iqs323->client;
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
	for (i = 0; i < IQS323_NUM_RETRIES; i++) {
		ret = iqs323_force_comms(iqs323);
		if (ret < 0)
			continue;

		ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		if (ret < (int)ARRAY_SIZE(msg)) {
			if (ret >= 0)
				ret = -EIO;

			msleep(IQS323_COMMS_RETRY_MS);
			continue;
		}

		if (get_unaligned_le16(msg[1].buf) == IQS323_COMMS_ERROR) {
			ret = -ENODATA;
			continue;
		}

		ret = 0;
		break;
	}

	iqs323_irq_wait();

	if (ret < 0)
		dev_err(&client->dev,
			"Failed to read from address 0x%02X: %d\n", reg, ret);

	return ret;
}

static int iqs323_read_word(struct iqs323_private *iqs323, u8 reg, u16 *val)
{
	__le16 val_buf;
	int error;

	error = iqs323_read_burst(iqs323, reg, &val_buf, 1);
	if (error)
		return error;

	*val = le16_to_cpu(val_buf);

	return 0;
}

static int iqs323_write_burst(struct iqs323_private *iqs323,
			      u8 reg, const void *val, u16 num_val)
{
	int val_len = num_val * sizeof(__le16);
	int msg_len = sizeof(reg) + val_len;
	int ret, i;
	struct i2c_client *client = iqs323->client;
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
	for (i = 0; i < IQS323_NUM_RETRIES; i++) {
		ret = iqs323_force_comms(iqs323);
		if (ret < 0)
			continue;

		ret = i2c_master_send(client, msg_buf, msg_len);
		if (ret < msg_len) {
			if (ret >= 0)
				ret = -EIO;

			msleep(IQS323_COMMS_RETRY_MS);
			continue;
		}

		ret = 0;
		break;
	}

	kfree(msg_buf);

	iqs323_irq_wait();

	if (ret < 0)
		dev_err(&client->dev,
			"Failed to write to address 0x%02X: %d\n", reg, ret);

	return ret;
}

static int iqs323_write_word(struct iqs323_private *iqs323, u8 reg, u16 val)
{
	__le16 val_buf = cpu_to_le16(val);

	return iqs323_write_burst(iqs323, reg, &val_buf, 1);
}

static int iqs323_write_mode(struct iqs323_private *iqs323,
			     enum iqs323_intf_mode_id intf_mode,
			     enum iqs323_power_mode_id power_mode, u16 cmd)
{
	u16 sys_setup = iqs323->sys_setup[0] | cmd;
	int error;

	if (intf_mode == IQS323_INTF_MODE_EVENT)
		sys_setup |= IQS323_SYS_SETUP_EVENT_MODE;
	else
		sys_setup &= ~IQS323_SYS_SETUP_EVENT_MODE;

	if (power_mode != IQS323_POWER_MODE_USER) {
		sys_setup &= ~IQS323_SYS_SETUP_POWER_MODE_MASK;
		sys_setup |= (power_mode << IQS323_SYS_SETUP_POWER_MODE_SHIFT);
	}

	error = iqs323_write_word(iqs323, IQS323_SYS_SETUP, sys_setup);
	if (error)
		iqs323->intf_mode = IQS323_INTF_MODE_ERROR;
	else
		iqs323->intf_mode = intf_mode;

	return error;
}

static int iqs323_ati_trigger(struct iqs323_private *iqs323)
{
	struct i2c_client *client = iqs323->client;
	u16 sys_status;
	int error, i;

	for (i = 0; i < IQS323_NUM_RETRIES; i++) {
		error = iqs323_write_mode(iqs323, IQS323_INTF_MODE_STREAM,
					  IQS323_POWER_MODE_USER,
					  IQS323_SYS_SETUP_REDO_ATI);
		if (error)
			return error;

		error = iqs323_irq_poll(iqs323, IQS323_ATI_TIMEOUT_US);
		if (error)
			continue;

		error = iqs323_read_word(iqs323, IQS323_SYS_STATUS, &sys_status);
		if (error)
			return error;

		/*
		 * If the device suffers a spurious reset during ATI, return
		 * successfully so that the interrupt handler may intervene.
		 *
		 * Otherwise, continue waiting for ATI to complete and retry
		 * if the device signals an error.
		 */
		if (sys_status & IQS323_SYS_STATUS_RESET)
			break;

		if (sys_status & IQS323_SYS_STATUS_ATI_ERROR) {
			error = -EIO;
			continue;
		}

		if (sys_status & IQS323_SYS_STATUS_ATI_ACTIVE) {
			error = -EBUSY;
			continue;
		}

		break;
	}

	if (error)
		dev_err(&client->dev, "Failed to complete ATI: %d\n", error);

	return error;
}

static int iqs323_dev_init(struct iqs323_private *iqs323, int dir)
{
	const struct iqs323_dev_desc *dev_desc = iqs323->dev_desc;
	int error, i, j, k;

	/*
	 * Acknowledge reset before writing any registers in case the device
	 * suffers a spurious reset during initialization.
	 */
	if (dir == WRITE) {
		error = iqs323_write_mode(iqs323, IQS323_INTF_MODE_STREAM,
					  IQS323_POWER_MODE_USER,
					  IQS323_SYS_SETUP_ACK_RESET);
		if (error)
			return error;
	}

	for (i = 0; i < IQS323_NUM_REG_GRPS; i++) {
		int num_row = dev_desc->reg_grps[i].num_row;
		int num_col = dev_desc->reg_grps[i].num_col;
		u16 reg = dev_desc->reg_grps[i].base;
		__le16 *val_buf;
		u16 *val;

		if (!num_col)
			continue;

		val = iqs323_setup(iqs323, i, 0);
		if (!val)
			continue;

		val_buf = kcalloc(num_col, sizeof(*val_buf), GFP_KERNEL);
		if (!val_buf)
			return -ENOMEM;

		for (j = 0; j < num_row; j++) {
			switch (dir) {
			case READ:
				error = iqs323_read_burst(iqs323, reg,
							  val_buf, num_col);
				for (k = 0; k < num_col; k++)
					val[k] = le16_to_cpu(val_buf[k]);
				break;

			case WRITE:
				for (k = 0; k < num_col; k++)
					val_buf[k] = cpu_to_le16(val[k]);
				error = iqs323_write_burst(iqs323, reg,
							   val_buf, num_col);
				break;

			default:
				error = -EINVAL;
			}

			if (error)
				break;

			reg += IQS323_REG_OFFSET;
			val += iqs323_max_cols[i];
		}

		kfree(val_buf);

		if (error)
			return error;
	}

	if (dir == READ)
		return 0;

	return iqs323_ati_trigger(iqs323);
}

static int iqs323_parse_props(struct iqs323_private *iqs323,
			      struct fwnode_handle *reg_grp_node,
			      int reg_grp_index,
			      enum iqs323_reg_grp_id reg_grp,
			      enum iqs323_reg_key_id reg_key)
{
	u16 *setup = iqs323_setup(iqs323, reg_grp, reg_grp_index);
	struct i2c_client *client = iqs323->client;
	int i;

	if (!setup)
		return 0;

	for (i = 0; i < ARRAY_SIZE(iqs323_props); i++) {
		const char *name = iqs323_props[i].name;
		int reg_offset = iqs323_props[i].reg_offset;
		int reg_shift = iqs323_props[i].reg_shift;
		int reg_width = iqs323_props[i].reg_width;
		int val_pitch = iqs323_props[i].val_pitch ? : 1;
		int val_min = iqs323_props[i].val_min;
		int val_max = iqs323_props[i].val_max;
		bool invert = iqs323_props[i].invert;
		const char *label = iqs323_props[i].label ? : name;
		unsigned int val;
		int error;

		if (iqs323_props[i].reg_grp != reg_grp ||
		    iqs323_props[i].reg_key != reg_key)
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

			if (fwnode_property_present(reg_grp_node, name)) {
				if (invert)
					setup[reg_offset] &= ~BIT(reg_shift);
				else
					setup[reg_offset] |= BIT(reg_shift);
			}

			continue;
		}

		error = fwnode_property_read_u32(reg_grp_node, name, &val);
		if (error == -EINVAL) {
			continue;
		} else if (error) {
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
		if (iqs323_props[i].val_subs)
			setup[reg_offset] |= iqs323_props[i].val_subs[val];
		else
			setup[reg_offset] |= (val / val_pitch << reg_shift);
	}

	return 0;
}

static int iqs323_parse_event(struct iqs323_private *iqs323,
			      struct fwnode_handle *event_node,
			      int reg_grp_index,
			      enum iqs323_reg_grp_id reg_grp,
			      enum iqs323_reg_key_id reg_key,
			      u16 event_mask,
			      unsigned int *event_type,
			      unsigned int *event_code)
{
	struct i2c_client *client = iqs323->client;
	int error;

	error = iqs323_parse_props(iqs323, event_node, reg_grp_index,
				   reg_grp, reg_key);
	if (error)
		return error;

	error = iqs323_parse_props(iqs323, event_node, reg_grp_index,
				   IQS323_REG_GRP_GEN, reg_key);
	if (error)
		return error;

	if (fwnode_property_present(event_node, "azoteq,gpio-select"))
		iqs323->gen_setup[0] |= event_mask;

	if (!event_code)
		return 0;

	error = fwnode_property_read_u32(event_node, "linux,code", event_code);
	if (error == -EINVAL) {
		return 0;
	} else if (error) {
		dev_err(&client->dev, "Failed to read %s code: %d\n",
			fwnode_get_name(event_node), error);
		return error;
	}

	if (!event_type) {
		input_set_capability(iqs323->kp_idev, EV_KEY, *event_code);
		return 0;
	}

	error = fwnode_property_read_u32(event_node, "linux,input-type",
					 event_type);
	if (error == -EINVAL) {
		*event_type = EV_KEY;
	} else if (error) {
		dev_err(&client->dev, "Failed to read %s input type: %d\n",
			fwnode_get_name(event_node), error);
		return error;
	} else if (*event_type != EV_KEY && *event_type != EV_SW) {
		dev_err(&client->dev, "Invalid %s input type: %d\n",
			fwnode_get_name(event_node), *event_type);
		return -EINVAL;
	}

	input_set_capability(iqs323->kp_idev, *event_type, *event_code);

	return 0;
}

static int iqs323_parse_snsr(struct iqs323_private *iqs323,
			     struct fwnode_handle *snsr_node, int snsr_index)
{
	struct i2c_client *client = iqs323->client;
	u16 *snsr_setup = iqs323->snsr_setup[snsr_index];
	int error, i, j;

	snsr_setup[0] |= IQS323_SNSR_SETUP_0_CHAN_EN;
	snsr_setup[5] &= ~IQS323_SNSR_SETUP_5_WAVE_SEL_MASK;

	for (i = 0; i < ARRAY_SIZE(iqs323_pins); i++) {
		const char *name = iqs323_pins[i].name;
		int reg_offset = iqs323_pins[i].reg_offset;
		int reg_shift = iqs323_pins[i].reg_shift;
		int reg_width = iqs323_pins[i].reg_width;
		int num_pins = iqs323_pins[i].val_max;
		const char *label = iqs323_pins[i].label ? : name;
		unsigned int pins[4];
		int count;

		if (iqs323_pins[i].reg_grp != IQS323_REG_GRP_SNSR)
			continue;

		count = fwnode_property_count_u32(snsr_node, name);
		if (count == -EINVAL) {
			continue;
		} else if (count < 0) {
			dev_err(&client->dev,
				"Failed to count %s %s pins: %d\n",
				fwnode_get_name(snsr_node), label, count);
			return count;
		} else if (count > num_pins) {
			dev_err(&client->dev,
				"Invalid number of %s %s pins\n",
				fwnode_get_name(snsr_node), label);
			return -EINVAL;
		}

		error = fwnode_property_read_u32_array(snsr_node, name, pins,
						       count);
		if (error) {
			dev_err(&client->dev,
				"Failed to read %s %s pins: %d\n",
				fwnode_get_name(snsr_node), label, error);
			return error;
		}

		snsr_setup[reg_offset] &= ~GENMASK(reg_shift + reg_width - 1,
						   reg_shift);

		for (j = 0; j < count; j++) {
			if (pins[j] >= num_pins) {
				dev_err(&client->dev,
					"Invalid %s %s pin: %u\n",
					fwnode_get_name(snsr_node),
					label, pins[j]);
				return -EINVAL;
			}

			snsr_setup[reg_offset] |= BIT(pins[j] + reg_shift);
		}
	}

	/*
	 * In the absence of any explicit waveform patterns, both patterns are
	 * defined as a function of sensing mode.
	 */
	if (!fwnode_property_present(snsr_node, "azoteq,wave-pattern-0")) {
		u16 mode = snsr_setup[2] & IQS323_SNSR_SETUP_2_SENSE_MODE_MASK;

		for (i = 0; i < ARRAY_SIZE(iqs323_sense_modes); i++)
			if (iqs323_sense_modes[i] == mode)
				break;

		if (i == ARRAY_SIZE(iqs323_sense_modes)) {
			dev_err(&client->dev, "Invalid %s sensing mode: %u\n",
				fwnode_get_name(snsr_node), mode);
			return -EINVAL;
		}

		snsr_setup[4] &= ~IQS323_SNSR_SETUP_4_WAVE_PAT_0_MASK;
		snsr_setup[4] |= iqs323_wave_patterns[i];
	}

	if (!fwnode_property_present(snsr_node, "azoteq,wave-pattern-1"))
		snsr_setup[4] &= ~IQS323_SNSR_SETUP_4_WAVE_PAT_1_MASK;

	return 0;
}

static int iqs323_parse_chan(struct iqs323_private *iqs323,
			     struct fwnode_handle *chan_node, int chan_index)
{
	const struct iqs323_dev_desc *dev_desc = iqs323->dev_desc;
	struct i2c_client *client = iqs323->client;
	u16 *snsr_setup = iqs323->snsr_setup[chan_index];
	u16 *chan_setup = iqs323->chan_setup[chan_index];
	unsigned int val;
	int error, i;
	int chan_shift = chan_index * 2 + 8;

	error = fwnode_property_read_u32(chan_node, "azoteq,ref-select", &val);
	if (!error) {
		u16 *ref_setup;

		if (val >= IQS323_NUM_CHAN) {
			dev_err(&client->dev,
				"Invalid %s reference channel: %u\n",
				fwnode_get_name(chan_node), val);
			return -EINVAL;
		}

		ref_setup = iqs323->chan_setup[val];

		/*
		 * Configure the current channel as a follower of the selected
		 * reference channel.
		 */
		chan_setup[0] &= ~IQS323_CHAN_SETUP_0_REF_MODE_MASK;
		chan_setup[0] |= IQS323_CHAN_SETUP_0_REF_MODE_FOLLOW;

		chan_setup[0] &= ~IQS323_CHAN_SETUP_0_REF_SEL_MASK;
		chan_setup[0] |= (val << IQS323_CHAN_SETUP_0_REF_SEL_SHIFT);

		error = fwnode_property_read_u32(chan_node, "azoteq,ref-weight",
						 &val);
		if (!error) {
			if (val > U16_MAX) {
				dev_err(&client->dev,
					"Invalid %s reference weight: %u\n",
					fwnode_get_name(chan_node), val);
				return -EINVAL;
			}

			chan_setup[3] = val;
		} else if (error != -EINVAL) {
			dev_err(&client->dev,
				"Failed to read %s reference weight: %d\n",
				fwnode_get_name(chan_node), error);
			return error;
		}

		/*
		 * Configure the selected channel as a reference channel which
		 * serves the current channel.
		 */
		ref_setup[0] &= ~IQS323_CHAN_SETUP_0_REF_MODE_MASK;
		ref_setup[0] |= IQS323_CHAN_SETUP_0_REF_MODE_REF;

		ref_setup[0] |= (IQS323_EVENT_MASK_TOUCH |
				 IQS323_EVENT_MASK_PROX) << chan_shift;
	} else if (error != -EINVAL) {
		dev_err(&client->dev,
			"Failed to read %s reference channel: %d\n",
			fwnode_get_name(chan_node), error);
		return error;
	}

	if (dev_desc->prod_num == IQS323_PROD_NUM_REL &&
	    fwnode_property_present(chan_node, "azoteq,release-enable"))
		snsr_setup[0] |= IQS323_SNSR_SETUP_0_MOV_EN;
	else
		snsr_setup[0] &= ~IQS323_SNSR_SETUP_0_MOV_EN;

	for (i = 0; i < ARRAY_SIZE(iqs323_kp_events); i++) {
		unsigned int *event_type = &iqs323->kp_type[chan_index][i];
		unsigned int *event_code = &iqs323->kp_code[chan_index][i];
		u16 event_mask = iqs323_kp_events[i].mask << chan_shift;
		const char *event_name = iqs323_kp_events[i].name;
		struct fwnode_handle *event_node;

		if (iqs323_kp_events[i].reg_key == IQS323_REG_KEY_MOV &&
		    dev_desc->prod_num != IQS323_PROD_NUM_MOV)
			continue;

		event_node = fwnode_get_named_child_node(chan_node, event_name);
		if (!event_node)
			continue;

		if (iqs323_kp_events[i].reg_key == IQS323_REG_KEY_MOV)
			snsr_setup[0] |= IQS323_SNSR_SETUP_0_MOV_EN;

		if (fwnode_property_present(event_node, "azoteq,wear-select"))
			iqs323->wear_mask |= event_mask;

		error = iqs323_parse_event(iqs323, event_node, chan_index,
					   IQS323_REG_GRP_CHAN,
					   iqs323_kp_events[i].reg_key,
					   event_mask,
					   event_mask ? event_type : NULL,
					   event_mask ? event_code : NULL);
		fwnode_handle_put(event_node);
		if (error)
			return error;

		iqs323->gen_setup[3] |= iqs323_kp_events[i].enable;
	}

	if (fwnode_property_present(chan_node, "azoteq,timeout-press-disable"))
		iqs323->sys_setup[0] |= BIT(chan_index + 8);
	else
		iqs323->sys_setup[0] &= ~BIT(chan_index + 8);

	return 0;
}

static int iqs323_parse_sldr(struct iqs323_private *iqs323,
			     struct fwnode_handle *sldr_node, int sldr_index)
{
	const struct iqs323_dev_desc *dev_desc = iqs323->dev_desc;
	struct i2c_client *client = iqs323->client;
	unsigned int chan_sel[IQS323_NUM_CHAN], val;
	u16 *sldr_setup = iqs323->sldr_setup;
	u16 *gest_setup = iqs323->gest_setup;
	int count, error, i;

	count = fwnode_property_count_u32(sldr_node, "azoteq,channel-select");
	if (count < 0) {
		dev_err(&client->dev, "Failed to count %s channels: %d\n",
			fwnode_get_name(sldr_node), count);
		return count;
	} else if (count > ARRAY_SIZE(chan_sel)) {
		dev_err(&client->dev, "Invalid number of %s channels\n",
			fwnode_get_name(sldr_node));
		return -EINVAL;
	}

	error = fwnode_property_read_u32_array(sldr_node,
					       "azoteq,channel-select",
					       chan_sel, count);
	if (error) {
		dev_err(&client->dev, "Failed to read %s channels: %d\n",
			fwnode_get_name(sldr_node), error);
		return error;
	}

	sldr_setup[0] |= count;
	sldr_setup[4] &= ~GENMASK(IQS323_NUM_CHAN - 1, 0);

	for (i = 0; i < ARRAY_SIZE(chan_sel); i++) {
		sldr_setup[6 + i] = 0;
		if (i >= count || chan_sel[i] == U8_MAX)
			continue;

		if (chan_sel[i] >= IQS323_NUM_CHAN) {
			dev_err(&client->dev, "Invalid %s channel: %u\n",
				fwnode_get_name(sldr_node), chan_sel[i]);
			return -EINVAL;
		}

		/*
		 * The following fields indicate which channels participate in
		 * the slider, as well as each channel's relative placement.
		 */
		sldr_setup[4] |= BIT(chan_sel[i]);
		sldr_setup[6 + i] = dev_desc->delta_links[chan_sel[i]];
	}

	sldr_setup[5] = dev_desc->touch_link;

	if (!sldr_setup[3]) {
		dev_err(&client->dev, "Undefined %s size\n",
			fwnode_get_name(sldr_node));
		return -EINVAL;
	}

	error = fwnode_property_read_u32(sldr_node, "linux,axis", &val);
	if (!error) {
		input_set_abs_params(iqs323->kp_idev, val,
				     0, sldr_setup[3] - 1, 0, 0);
		iqs323->sl_axis = val;

		/*
		 * In order to report granular coordinates, the device must be
		 * placed in streaming mode after the slider enters a state of
		 * touch. In this case, touch interrupts are unmasked.
		 */
		iqs323->gen_setup[3] |= IQS323_EVENT_MASK_TOUCH;
	} else if (error != -EINVAL) {
		dev_err(&client->dev, "Failed to read %s axis: %d\n",
			fwnode_get_name(sldr_node), error);
		return error;
	}

	for (i = 0; i < ARRAY_SIZE(iqs323_sl_events); i++)
		gest_setup[0] &= ~iqs323_sl_events[i].enable;

	for (i = 0; i < ARRAY_SIZE(iqs323_sl_events); i++) {
		const char *event_name = iqs323_sl_events[i].name;
		struct fwnode_handle *event_node;

		event_node = fwnode_get_named_child_node(sldr_node, event_name);
		if (!event_node)
			continue;

		error = iqs323_parse_event(iqs323, event_node, sldr_index,
					   IQS323_REG_GRP_GEST,
					   iqs323_sl_events[i].reg_key,
					   iqs323_sl_events[i].mask &
					   ~IQS323_SLDR_STATUS_BUSY,
					   NULL, &iqs323->sl_code[i]);
		fwnode_handle_put(event_node);
		if (error)
			return error;

		gest_setup[0] |= iqs323_sl_events[i].enable;

		/*
		 * The press/release event is determined based on whether the
		 * coordinate field reports 0xFFFF and solely relies on touch
		 * interrupts to be unmasked.
		 *
		 * Likewise, swipe and hold gestures rely on touch interrupts
		 * to signal that the contact has left the slider.
		 */
		if (iqs323_sl_events[i].enable) {
			iqs323->gen_setup[3] |= IQS323_EVENT_MASK_SLDR;
			if (iqs323_sl_events[i].mask & IQS323_SLDR_STATUS_BUSY)
				iqs323->gen_setup[3] |= IQS323_EVENT_MASK_TOUCH;
		} else {
			iqs323->gen_setup[3] |= IQS323_EVENT_MASK_TOUCH;
		}
	}

	return 0;
}

static int iqs323_parse_gpio(struct iqs323_private *iqs323,
			     struct fwnode_handle *gpio_node, int gpio_index)
{
	if (fwnode_property_present(gpio_node, "output-high"))
		iqs323->gen_setup[0] = IQS323_GEN_SETUP_0_GPIO_SET;
	else if (fwnode_property_present(gpio_node, "azoteq,invert-enable"))
		iqs323->gen_setup[0] |= IQS323_GEN_SETUP_0_GPIO_INV;

	return 0;
}

static int (*iqs323_parse_extra[IQS323_NUM_REG_GRPS])
			       (struct iqs323_private *iqs323,
				struct fwnode_handle *reg_grp_node,
				int reg_grp_index) = {
	[IQS323_REG_GRP_SNSR] = iqs323_parse_snsr,
	[IQS323_REG_GRP_CHAN] = iqs323_parse_chan,
	[IQS323_REG_GRP_SLDR] = iqs323_parse_sldr,
	[IQS323_REG_GRP_GPIO] = iqs323_parse_gpio,
};

static int iqs323_parse_reg_grp(struct iqs323_private *iqs323,
				enum iqs323_reg_grp_id reg_grp,
				int reg_grp_index)
{
	struct i2c_client *client = iqs323->client;
	struct fwnode_handle *reg_grp_node;
	int error;

	if (iqs323_reg_grp_names[reg_grp]) {
		char reg_grp_name[16];

		snprintf(reg_grp_name, sizeof(reg_grp_name),
			 iqs323_reg_grp_names[reg_grp], reg_grp_index);

		reg_grp_node = device_get_named_child_node(&client->dev,
							   reg_grp_name);
	} else {
		reg_grp_node = fwnode_handle_get(dev_fwnode(&client->dev));
	}

	if (!reg_grp_node)
		return 0;

	error = iqs323_parse_props(iqs323, reg_grp_node, reg_grp_index,
				   reg_grp, IQS323_REG_KEY_NONE);

	if (!error && iqs323_parse_extra[reg_grp])
		error = iqs323_parse_extra[reg_grp](iqs323, reg_grp_node,
						    reg_grp_index);

	fwnode_handle_put(reg_grp_node);

	return error;
}

static int iqs323_parse_all(struct iqs323_private *iqs323)
{
	const struct iqs323_dev_desc *dev_desc = iqs323->dev_desc;
	struct i2c_client *client = iqs323->client;
	enum iqs323_power_mode_id power_mode;
	int error, i, j;

	iqs323->gen_setup[0] = 0;

	iqs323->gen_setup[3] &= ~IQS323_EVENT_MASK_ALL;
	iqs323->gen_setup[3] |= IQS323_EVENT_MASK_ATI;

	for (i = 0; i < IQS323_NUM_CHAN; i++)
		iqs323->snsr_setup[i][0] &= ~IQS323_SNSR_SETUP_0_CHAN_EN;

	iqs323->sldr_setup[0] &= ~IQS323_SLDR_SETUP_0_CHAN_CNT_MASK;

	for (i = 0; i < IQS323_NUM_REG_GRPS; i++) {
		for (j = 0; j < dev_desc->reg_grps[i].num_row; j++) {
			error = iqs323_parse_reg_grp(iqs323, i, j);
			if (error)
				return error;
		}
	}

	/*
	 * Neither automatic nor manual ULP modes may be used if the movement
	 * UI or channel timeouts are in use. In either case, LP mode must be
	 * used instead.
	 */
	power_mode = iqs323->sys_setup[0] & IQS323_SYS_SETUP_POWER_MODE_MASK;
	power_mode >>= IQS323_SYS_SETUP_POWER_MODE_SHIFT;
	if (power_mode == IQS323_POWER_MODE_ULP ||
	    power_mode == IQS323_POWER_MODE_AUTO) {
		for (i = 0; i < IQS323_NUM_CHAN; i++)
			if (dev_desc->prod_num == IQS323_PROD_NUM_MOV &&
			    iqs323->snsr_setup[i][0] & IQS323_SNSR_SETUP_0_MOV_EN)
				break;

		if (i < IQS323_NUM_CHAN || iqs323->gen_setup[2]) {
			dev_warn(&client->dev, "ULP mode is restricted\n");

			if (power_mode == IQS323_POWER_MODE_ULP)
				power_mode = IQS323_POWER_MODE_LP;
			else
				power_mode = IQS323_POWER_MODE_AUTO_NO_ULP;

			iqs323->sys_setup[0] &= ~IQS323_SYS_SETUP_POWER_MODE_MASK;
			iqs323->sys_setup[0] |= (power_mode <<
						 IQS323_SYS_SETUP_POWER_MODE_SHIFT);
		}
	}

	return 0;
}

static int iqs323_process_events(struct iqs323_private *iqs323,
				 __le16 *val_buf, u16 num_val)
{
	enum iqs323_intf_mode_id intf_mode = IQS323_INTF_MODE_EVENT;
	u16 sys_status, sldr_status, sldr_pos, check_comms;
	struct i2c_client *client = iqs323->client;
	int error, i, j;

	error = iqs323_read_burst(iqs323, IQS323_SYS_STATUS, val_buf, num_val);
	if (error)
		return error;

	check_comms = le16_to_cpu(val_buf[IQS323_MIN_REPORT_LEN - 1]);

	/*
	 * The following check ensures that an unimplemented register returns
	 * a known value. Anything else indicates the device is in an invalid
	 * state and must be recovered.
	 */
	if (check_comms != IQS323_COMMS_ERROR) {
		dev_err(&client->dev, "Unexpected device status\n");

		error = iqs323_hard_reset(iqs323);
		if (error)
			return error;

		return iqs323_dev_init(iqs323, WRITE) ? : -EAGAIN;
	}

	sys_status = le16_to_cpu(val_buf[0]);

	if (sys_status & IQS323_SYS_STATUS_RESET) {
		dev_err(&client->dev, "Unexpected device reset\n");
		return iqs323_dev_init(iqs323, WRITE) ? : -EAGAIN;
	}

	if (sys_status & IQS323_SYS_STATUS_ATI_ERROR) {
		dev_err(&client->dev, "Unexpected ATI error\n");
		return iqs323_ati_trigger(iqs323) ? : -EAGAIN;
	}

	if (sys_status & IQS323_SYS_STATUS_ATI_ACTIVE)
		return -EAGAIN;

	for (i = 0; i < IQS323_NUM_CHAN; i++) {
		for (j = 0; j < ARRAY_SIZE(iqs323_kp_events); j++) {
			u16 mask = iqs323_kp_events[j].mask << (i * 2 + 8);

			if (!iqs323->kp_type[i][j])
				continue;

			input_event(iqs323->kp_idev,
				    iqs323->kp_type[i][j],
				    iqs323->kp_code[i][j],
				    !!(sys_status & mask));
		}
	}

	sldr_pos = le16_to_cpu(val_buf[2]);

	if (sldr_pos < U16_MAX && test_bit(EV_ABS, iqs323->kp_idev->evbit)) {
		input_report_abs(iqs323->kp_idev, iqs323->sl_axis, sldr_pos);
		intf_mode = IQS323_INTF_MODE_STREAM;
	}

	sldr_status = le16_to_cpu(val_buf[1]) & U8_MAX;
	sldr_status &= ~IQS323_SLDR_STATUS_EVENT;

	for (i = 0; i < ARRAY_SIZE(iqs323_sl_events); i++) {
		u16 mask = iqs323_sl_events[i].mask;

		input_report_key(iqs323->kp_idev,
				 iqs323->sl_code[i],
				 mask ? (sldr_status & mask) == mask
				      : sldr_pos < U16_MAX);

		if (mask & IQS323_SLDR_STATUS_BUSY)
			sldr_status &= ~(mask & ~IQS323_SLDR_STATUS_BUSY);
	}

	if (sldr_status & ~IQS323_SLDR_STATUS_BUSY) {
		input_sync(iqs323->kp_idev);

		for (i = 0; i < ARRAY_SIZE(iqs323_sl_events); i++) {
			u16 mask = iqs323_sl_events[i].mask;

			if (!mask)
				continue;

			if (!(mask & IQS323_SLDR_STATUS_BUSY))
				input_report_key(iqs323->kp_idev,
						 iqs323->sl_code[i], 0);
		}
	}

	input_sync(iqs323->kp_idev);

	if (intf_mode == iqs323->intf_mode)
		return 0;

	return iqs323_write_mode(iqs323, intf_mode, IQS323_POWER_MODE_USER, 0);
}

static int iqs323_report_async(struct iqs323_private *iqs323,
			       __le16 *val_buf, u16 num_val)
{
	int error;

	/*
	 * I2C communication prompts the device to assert its RDY pin if it is
	 * not already asserted. As such, the interrupt must be disabled so as
	 * to prevent reentrant interrupts.
	 */
	disable_irq(gpiod_to_irq(iqs323->irq_gpio));

	error = iqs323_process_events(iqs323, val_buf, num_val);

	enable_irq(gpiod_to_irq(iqs323->irq_gpio));

	return error;
}

static int iqs323_report_sync(struct iqs323_private *iqs323)
{
	__le16 val_buf[IQS323_MIN_REPORT_LEN];
	int error;

	error = iqs323_process_events(iqs323, val_buf, ARRAY_SIZE(val_buf));
	if (error == -EAGAIN)
		return 0;

	return error;
}

static irqreturn_t iqs323_irq(int irq, void *context)
{
	struct iqs323_private *iqs323 = context;

	return iqs323_report_sync(iqs323) ? IRQ_NONE : IRQ_HANDLED;
}

static int iqs323_runtime_pm(struct device *dev,
			     enum iqs323_power_mode_id power_mode)
{
	struct iqs323_private *iqs323 = dev_get_drvdata(dev);
	const struct iqs323_dev_desc *dev_desc = iqs323->dev_desc;
	int error, i;
	u16 val;

	if (device_may_wakeup(dev))
		return 0;

	disable_irq(gpiod_to_irq(iqs323->irq_gpio));

	/*
	 * The following loop protects against an edge case in which precisely
	 * timed forced communication may place the device in an invalid state,
	 * identifiable by unexpected register contents.
	 */
	for (i = 0; i < IQS323_NUM_RETRIES; i++) {
		error = iqs323_write_mode(iqs323, iqs323->intf_mode,
					  power_mode, 0);
		if (error || power_mode == IQS323_POWER_MODE_USER)
			break;

		msleep(IQS323_COMMS_RETRY_MS);

		error = iqs323_read_word(iqs323, IQS323_PROD_NUM, &val);
		if (error || val == dev_desc->prod_num)
			break;

		error = iqs323_hard_reset(iqs323);
		if (error)
			break;

		error = iqs323_dev_init(iqs323, WRITE);
		if (error)
			break;
	}

	enable_irq(gpiod_to_irq(iqs323->irq_gpio));

	return i < IQS323_NUM_RETRIES ? error : -EIO;
}

static int iqs323_suspend(struct device *dev)
{
	return iqs323_runtime_pm(dev, IQS323_POWER_MODE_HALT);
}

static int iqs323_resume(struct device *dev)
{
	return iqs323_runtime_pm(dev, IQS323_POWER_MODE_USER);
}

static DEFINE_SIMPLE_DEV_PM_OPS(iqs323_pm, iqs323_suspend, iqs323_resume);

static ssize_t move_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iqs323_private *iqs323 = dev_get_drvdata(dev);
	__le16 val_buf[2];
	int error;

	disable_irq(gpiod_to_irq(iqs323->irq_gpio));

	error = iqs323_read_burst(iqs323, IQS323_MOV_STATUS, val_buf,
				  ARRAY_SIZE(val_buf));
	if (error)
		goto err_irq;

	if (le16_to_cpu(val_buf[1]) != IQS323_COMMS_ERROR) {
		dev_err(dev, "Unexpected movement status\n");

		error = iqs323_hard_reset(iqs323);
		if (error)
			goto err_irq;

		error = iqs323_dev_init(iqs323, WRITE) ? : -EAGAIN;
	}

err_irq:
	enable_irq(gpiod_to_irq(iqs323->irq_gpio));

	if (error)
		return error;

	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 !!(le16_to_cpu(val_buf[0]) &
			    GENMASK(IQS323_NUM_CHAN - 1, 0)));
}

static ssize_t wear_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iqs323_private *iqs323 = dev_get_drvdata(dev);
	__le16 val_buf[IQS323_MIN_REPORT_LEN];
	int error;

	error = iqs323_report_async(iqs323, val_buf, ARRAY_SIZE(val_buf));
	if (error)
		return error;

	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 !!(le16_to_cpu(val_buf[0]) & iqs323->wear_mask));
}

static ssize_t ch_info_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct iqs323_private *iqs323 = dev_get_drvdata(dev);
	const struct iqs323_dev_desc *dev_desc = iqs323->dev_desc;
	__le16 *val_buf;
	ssize_t len = 0;
	int error, i;

	val_buf = kcalloc(IQS323_MAX_REPORT_LEN, sizeof(*val_buf), GFP_KERNEL);
	if (!val_buf)
		return -ENOMEM;

	error = iqs323_report_async(iqs323, val_buf, IQS323_MAX_REPORT_LEN);
	if (error)
		goto err_kfree;

	for (i = 0; i < IQS323_NUM_CHAN; i++) {
		u16 *snsr_setup = iqs323->snsr_setup[i];
		u16 sys_status = le16_to_cpu(val_buf[0]);
		u16 counts = le16_to_cpu(val_buf[3 + i * 2]);
		u16 lta = le16_to_cpu(val_buf[4 + i * 2]);
		int delta = lta - counts;
		u16 lta_active = le16_to_cpu(val_buf[16 + i]);
		int delta_snap = le16_to_cpu(val_buf[19 + i]);
		int mov_status = !!(le16_to_cpu(val_buf[19]) & BIT(i));
		int j = i * 2 + 8;
		const char *fmt;

		if (!(snsr_setup[0] & IQS323_SNSR_SETUP_0_CHAN_EN))
			continue;

		if (snsr_setup[0] & IQS323_SNSR_SETUP_0_MOV_EN)
			fmt = "%d: %u, %u, %u, %u, %d; %u, %d, %d\n";
		else
			fmt = "%d: %u, %u, %u, %u, %d\n";

		if (dev_desc->prod_num == IQS323_PROD_NUM_MOV)
			delta_snap = -1;
		else
			mov_status = -1;

		len += scnprintf(buf + len, PAGE_SIZE, fmt, i,
				 !!(sys_status & (IQS323_EVENT_MASK_PROX << j)),
				 !!(sys_status & (IQS323_EVENT_MASK_TOUCH << j)),
				 counts, lta, delta,
				 lta_active, delta_snap, mov_status);
	}

	if (!len)
		error = -ENODATA;

err_kfree:
	kfree(val_buf);

	return error ? : len;
}

static ssize_t fw_info_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct iqs323_private *iqs323 = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u.%u.%u.%u\n",
			 le16_to_cpu(iqs323->ver_info.prod_num),
			 le32_to_cpu(iqs323->ver_info.patch),
			 le16_to_cpu(iqs323->ver_info.major),
			 le16_to_cpu(iqs323->ver_info.minor));
}

static DEVICE_ATTR_RO(move_status);
static DEVICE_ATTR_RO(wear_status);
static DEVICE_ATTR_RO(ch_info);
static DEVICE_ATTR_RO(fw_info);

static struct attribute *iqs323_attrs[] = {
	&dev_attr_move_status.attr,
	&dev_attr_wear_status.attr,
	&dev_attr_ch_info.attr,
	&dev_attr_fw_info.attr,
	NULL,
};

static umode_t iqs323_is_visible(struct kobject *kobj,
				 struct attribute *attr, int i)
{
	struct device *dev = kobj_to_dev(kobj);
	struct iqs323_private *iqs323 = dev_get_drvdata(dev);
	const struct iqs323_dev_desc *dev_desc = iqs323->dev_desc;

	if (attr == &dev_attr_move_status.attr &&
	    dev_desc->prod_num != IQS323_PROD_NUM_MOV)
		return 0;

	if (attr == &dev_attr_wear_status.attr && !iqs323->wear_mask)
		return 0;

	return attr->mode;
}

static const struct attribute_group iqs323_group = {
	.is_visible = iqs323_is_visible,
	.attrs = iqs323_attrs,
};

static const struct attribute_group *iqs323_groups[] = {
	&iqs323_group,
	NULL,
};

static int iqs323_probe(struct i2c_client *client)
{
	struct iqs323_private *iqs323;
	unsigned long irq_flags;
	int error, irq, i;
	u16 prod_num;

	iqs323 = devm_kzalloc(&client->dev, sizeof(*iqs323), GFP_KERNEL);
	if (!iqs323)
		return -ENOMEM;

	i2c_set_clientdata(client, iqs323);
	iqs323->client = client;

	iqs323->kp_idev = devm_input_allocate_device(&client->dev);
	if (!iqs323->kp_idev)
		return -ENOMEM;

	iqs323->kp_idev->name = client->name;
	iqs323->kp_idev->id.bustype = BUS_I2C;

	/*
	 * The RDY pin behaves as an interrupt, but must also be polled ahead
	 * of unsolicited I2C communication. As such, it is first opened as a
	 * GPIO and then passed to gpiod_to_irq() to register the interrupt.
	 *
	 * Note that because the RDY and MCLR pins are combined into a single
	 * bidirectional pin, the platform's GPIO must be configured to be an
	 * open-drain output.
	 */
	iqs323->irq_gpio = devm_gpiod_get(&client->dev, "irq", GPIOD_OUT_LOW);
	if (IS_ERR(iqs323->irq_gpio)) {
		error = PTR_ERR(iqs323->irq_gpio);
		dev_err(&client->dev, "Failed to request IRQ GPIO: %d\n",
			error);
		return error;
	}

	error = iqs323_hard_reset(iqs323);
	if (error) {
		dev_err(&client->dev, "Failed to reset device: %d\n", error);
		return error;
	}

	error = iqs323_read_burst(iqs323, IQS323_PROD_NUM, &iqs323->ver_info,
				  sizeof(iqs323->ver_info) / sizeof(__le16));
	if (error)
		return error;

	prod_num = le16_to_cpu(iqs323->ver_info.prod_num);

	for (i = 0; i < ARRAY_SIZE(iqs323_devs); i++)
		if (prod_num == iqs323_devs[i].prod_num)
			break;

	if (i == ARRAY_SIZE(iqs323_devs)) {
		dev_err(&client->dev, "Invalid product number: %u\n", prod_num);
		return -EINVAL;
	}

	iqs323->dev_desc = &iqs323_devs[i];

	error = iqs323_dev_init(iqs323, READ);
	if (error)
		return error;

	error = iqs323_parse_all(iqs323);
	if (error)
		return error;

	error = iqs323_dev_init(iqs323, WRITE);
	if (error)
		return error;

	error = iqs323_report_sync(iqs323);
	if (error)
		return error;

	error = input_register_device(iqs323->kp_idev);
	if (error) {
		dev_err(&client->dev, "Failed to register device: %d\n", error);
		return error;
	}

	irq = gpiod_to_irq(iqs323->irq_gpio);
	if (irq < 0)
		return irq;

	irq_flags = gpiod_is_active_low(iqs323->irq_gpio) ? IRQF_TRIGGER_LOW
							  : IRQF_TRIGGER_HIGH;
	irq_flags |= IRQF_ONESHOT;

	error = devm_request_threaded_irq(&client->dev, irq, NULL, iqs323_irq,
					  irq_flags, client->name, iqs323);
	if (error)
		dev_err(&client->dev, "Failed to request IRQ: %d\n", error);

	return error;
}

static const struct of_device_id iqs323_of_match[] = {
	{ .compatible = "azoteq,iqs323" },
	{ }
};
MODULE_DEVICE_TABLE(of, iqs323_of_match);

static struct i2c_driver iqs323_i2c_driver = {
	.driver = {
		.name = "iqs323",
		.of_match_table = iqs323_of_match,
		.dev_groups = iqs323_groups,
		.pm = pm_sleep_ptr(&iqs323_pm),
	},
	.probe_new = iqs323_probe,
};
module_i2c_driver(iqs323_i2c_driver);

MODULE_AUTHOR("Jeff LaBundy <jeff@labundy.com>");
MODULE_DESCRIPTION("Azoteq IQS323 Capacitive/Inductive Sensing Controller");
MODULE_LICENSE("GPL");

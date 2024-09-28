// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Azoteq IQS9150/9151 Trackpad Controller
 *
 * Copyright (C) 2024 Azoteq (Pty) Ltd
 * Author: Jeff LaBundy <jeff@labundy.com>
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <asm/unaligned.h>

#define IQS9150_PROD_NUM			0x1000
#define IQS9150_STATUS				0x1018

#define IQS9150_INFO_SHOW_RESET			BIT(7)
#define IQS9150_INFO_ALP_ATI_AGAIN		BIT(6)
#define IQS9150_INFO_ALP_ATI_ERROR		BIT(5)
#define IQS9150_INFO_TP_ATI_AGAIN		BIT(4)
#define IQS9150_INFO_TP_ATI_ERROR		BIT(3)
#define IQS9150_INFO_CHARGE_MODE		GENMASK(2, 0)
#define IQS9150_INFO_CHARGE_MODE_LP1		3

#define IQS9150_REG_BUF_START			(0x115C)
#define IQS9150_REG_BUF_LEN			(0x14F0 - IQS9150_REG_BUF_START)

#define IQS9150_SETTINGS_MINOR			0x1178
#define IQS9150_SETTINGS_MAJOR			0x1179

#define IQS9150_TIMEOUT_COMMS			0x11B8

#define IQS9150_CONTROL				0x11BC
#define IQS9150_CONTROL_SUSPEND			BIT(11)
#define IQS9150_CONTROL_ACK_RESET		BIT(7)
#define IQS9150_CONTROL_ATI_ALP			BIT(6)
#define IQS9150_CONTROL_ATI_TP			BIT(5)

#define IQS9150_CONFIG				0x11BE
#define IQS9150_CONFIG_EVENT_MASK		GENMASK(15, 9)
#define IQS9150_CONFIG_EVENT_ATI		BIT(11)
#define IQS9150_CONFIG_EVENT_MODE		BIT(8)
#define IQS9150_CONFIG_FORCED_COMMS		BIT(4)

#define IQS9150_OTHER				0x11C0
#define IQS9150_OTHER_SW_ENABLE			BIT(15)

#define IQS9150_ALP_SETUP			0x11C5
#define IQS9150_ALP_SETUP_ENABLE		BIT(7)

#define IQS9150_ALP_RX_ENABLE			0x11C2
#define IQS9150_ALP_TX_ENABLE			0x11C6

#define IQS9150_TOTAL_RX			0x11E3
#define IQS9150_TOTAL_TX			0x11E4
#define IQS9150_NUM_CONTACTS			0x11E5
#define IQS9150_X_RES				0x11E6
#define IQS9150_Y_RES				0x11E8
#define IQS9150_ANGLE_AXIAL			0x120C
#define IQS9150_ANGLE_SCROLL			0x120D
#define IQS9150_RX_TX_MAP			0x1218

#define IQS9150_ENG_BUF_START			0x2000
#define IQS9150_ENG_BUF_LEN			6

#define IQS9150_COMMS_ERROR			0xEEEE
#define IQS9150_COMMS_RETRY_MS			50
#define IQS9150_COMMS_SLEEP_US			100
#define IQS9150_COMMS_TIMEOUT_US		(100 * USEC_PER_MSEC)
#define IQS9150_START_TIMEOUT_US		(1 * USEC_PER_SEC)

#define IQS9150_NUM_RETRIES			5
#define IQS9150_MAX_LEN				PAGE_SIZE

#define IQS9150_NUM_RX				26
#define IQS9150_NUM_TX				22
#define IQS9150_MAX_TX				45
#define IQS9150_RDY_TX				44
#define IQS9150_NUM_CHANNELS			506
#define IQS9150_MAX_CONTACTS			7

#define iqs9150_irq_wait()			usleep_range(50, 100)

#define iqs9150_get_word(reg)			get_unaligned_le16(&iqs9150->reg_buf[reg - 0x115C])
#define iqs9150_put_word(reg, val)		put_unaligned_le16(val,	      \
								   &iqs9150->reg_buf[reg - 0x115C])
#define iqs9150_reg(reg)			(iqs9150->reg_buf[reg - 0x115C])

enum iqs9150_dev_id {
	IQS9150,
	IQS9151,
};

enum iqs9150_comms_mode {
	IQS9150_COMMS_MODE_WAIT,
	IQS9150_COMMS_MODE_FREE,
	IQS9150_COMMS_MODE_FORCE,
};

enum iqs9150_reg_key_id {
	IQS9150_REG_KEY_NONE,
	IQS9150_REG_KEY_SPAN,
	IQS9150_REG_KEY_MASK,
	IQS9150_REG_KEY_TAP,
	IQS9150_REG_KEY_HOLD,
	IQS9150_REG_KEY_PALM,
	IQS9150_REG_KEY_AXIAL_X,
	IQS9150_REG_KEY_AXIAL_Y,
	IQS9150_REG_KEY_ZOOM,
	IQS9150_REG_KEY_SCROLL_X,
	IQS9150_REG_KEY_SCROLL_Y,
	IQS9150_REG_KEY_RESERVED
};

enum iqs9150_reg_grp_id {
	IQS9150_REG_GRP_TP,
	IQS9150_REG_GRP_1F,
	IQS9150_REG_GRP_2F,
	IQS9150_REG_GRP_SW,
	IQS9150_REG_GRP_ALP,
	IQS9150_REG_GRP_SYS,
	IQS9150_NUM_REG_GRPS
};

struct iqs9150_reg_grp_desc {
	const char *name;
	int status_offs;
	u16 enable_addr;
	u16 event_mask;
	u16 ati_mask;
};

static const struct iqs9150_reg_grp_desc
		    iqs9150_reg_grps[IQS9150_NUM_REG_GRPS] = {
	[IQS9150_REG_GRP_TP] = {
		.name = "trackpad",
		.event_mask = BIT(10),
		.ati_mask = IQS9150_INFO_TP_ATI_ERROR |
			    IQS9150_INFO_TP_ATI_AGAIN,
	},
	[IQS9150_REG_GRP_1F] = {
		.name = "gesture-single",
		.status_offs = 0,
		.enable_addr = 0x11F6,
		.event_mask = BIT(9),
		.ati_mask = IQS9150_INFO_TP_ATI_ERROR |
			    IQS9150_INFO_TP_ATI_AGAIN,
	},
	[IQS9150_REG_GRP_2F] = {
		.name = "gesture-double",
		.status_offs = 1,
		.enable_addr = 0x11F8,
		.event_mask = BIT(9),
		.ati_mask = IQS9150_INFO_TP_ATI_ERROR |
			    IQS9150_INFO_TP_ATI_AGAIN,
	},
	[IQS9150_REG_GRP_SW] = {
		.name = "switch",
		.status_offs = 2,
		.enable_addr = IQS9150_OTHER,
		.event_mask = BIT(14),
	},
	[IQS9150_REG_GRP_ALP] = {
		.name = "alp",
		.status_offs = 2,
		.event_mask = BIT(12),
		.ati_mask = IQS9150_INFO_ALP_ATI_ERROR |
			    IQS9150_INFO_ALP_ATI_AGAIN,
	},
};

struct iqs9150_event_desc {
	const char *name;
	u16 status_mask;
	u16 enable_mask;
	u16 travel_mask;
	enum iqs9150_reg_grp_id reg_grp;
	enum iqs9150_reg_key_id reg_key;
};

static const struct iqs9150_event_desc iqs9150_kp_events[] = {
	{
		.name = "event-tap",
		.status_mask = BIT(0),
		.enable_mask = BIT(0),
		.reg_grp = IQS9150_REG_GRP_1F,
		.reg_key = IQS9150_REG_KEY_TAP,
	},
	{
		.name = "event-tap-double",
		.status_mask = BIT(1),
		.enable_mask = BIT(1),
		.reg_grp = IQS9150_REG_GRP_1F,
		.reg_key = IQS9150_REG_KEY_TAP,
	},
	{
		.name = "event-tap-triple",
		.status_mask = BIT(2),
		.enable_mask = BIT(2),
		.reg_grp = IQS9150_REG_GRP_1F,
		.reg_key = IQS9150_REG_KEY_TAP,
	},
	{
		.name = "event-hold",
		.status_mask = BIT(3),
		.enable_mask = BIT(3),
		.reg_grp = IQS9150_REG_GRP_1F,
		.reg_key = IQS9150_REG_KEY_HOLD,
	},
	{
		.name = "event-palm",
		.status_mask = BIT(4),
		.enable_mask = BIT(4),
		.reg_grp = IQS9150_REG_GRP_1F,
		.reg_key = IQS9150_REG_KEY_PALM,
	},
	{
		.name = "event-swipe-x-pos",
		.status_mask = BIT(8),
		.enable_mask = BIT(8),
		.reg_grp = IQS9150_REG_GRP_1F,
		.reg_key = IQS9150_REG_KEY_AXIAL_X,
	},
	{
		.name = "event-swipe-x-neg",
		.status_mask = BIT(9),
		.enable_mask = BIT(9),
		.reg_grp = IQS9150_REG_GRP_1F,
		.reg_key = IQS9150_REG_KEY_AXIAL_X,
	},
	{
		.name = "event-swipe-y-pos",
		.status_mask = BIT(10),
		.enable_mask = BIT(10),
		.reg_grp = IQS9150_REG_GRP_1F,
		.reg_key = IQS9150_REG_KEY_AXIAL_Y,
	},
	{
		.name = "event-swipe-y-neg",
		.status_mask = BIT(11),
		.enable_mask = BIT(11),
		.reg_grp = IQS9150_REG_GRP_1F,
		.reg_key = IQS9150_REG_KEY_AXIAL_Y,
	},
	{
		.name = "event-swipe-x-pos-hold",
		.status_mask = BIT(12),
		.enable_mask = BIT(12),
		.reg_grp = IQS9150_REG_GRP_1F,
		.reg_key = IQS9150_REG_KEY_HOLD,
	},
	{
		.name = "event-swipe-x-neg-hold",
		.status_mask = BIT(13),
		.enable_mask = BIT(13),
		.reg_grp = IQS9150_REG_GRP_1F,
		.reg_key = IQS9150_REG_KEY_HOLD,
	},
	{
		.name = "event-swipe-y-pos-hold",
		.status_mask = BIT(14),
		.enable_mask = BIT(14),
		.reg_grp = IQS9150_REG_GRP_1F,
		.reg_key = IQS9150_REG_KEY_HOLD,
	},
	{
		.name = "event-swipe-y-neg-hold",
		.status_mask = BIT(15),
		.enable_mask = BIT(15),
		.reg_grp = IQS9150_REG_GRP_1F,
		.reg_key = IQS9150_REG_KEY_HOLD,
	},
	{
		.name = "event-tap",
		.status_mask = BIT(0),
		.enable_mask = BIT(0),
		.reg_grp = IQS9150_REG_GRP_2F,
		.reg_key = IQS9150_REG_KEY_TAP,
	},
	{
		.name = "event-tap-double",
		.status_mask = BIT(1),
		.enable_mask = BIT(1),
		.reg_grp = IQS9150_REG_GRP_2F,
		.reg_key = IQS9150_REG_KEY_TAP,
	},
	{
		.name = "event-tap-triple",
		.status_mask = BIT(2),
		.enable_mask = BIT(2),
		.reg_grp = IQS9150_REG_GRP_2F,
		.reg_key = IQS9150_REG_KEY_TAP,
	},
	{
		.name = "event-hold",
		.status_mask = BIT(3),
		.enable_mask = BIT(3),
		.reg_grp = IQS9150_REG_GRP_2F,
		.reg_key = IQS9150_REG_KEY_HOLD,
	},
	{
		.name = "event-zoom-pos",
		.status_mask = BIT(4),
		.enable_mask = BIT(4),
		.reg_grp = IQS9150_REG_GRP_2F,
		.reg_key = IQS9150_REG_KEY_ZOOM,
	},
	{
		.name = "event-zoom-neg",
		.status_mask = BIT(5),
		.enable_mask = BIT(5),
		.reg_grp = IQS9150_REG_GRP_2F,
		.reg_key = IQS9150_REG_KEY_ZOOM,
	},
	{
		.name = "event-scroll-y-pos",
		.status_mask = BIT(6),
		.enable_mask = BIT(6),
		.reg_grp = IQS9150_REG_GRP_2F,
		.reg_key = IQS9150_REG_KEY_SCROLL_Y,
	},
	{
		.name = "event-scroll-y-neg",
		.status_mask = BIT(6),
		.enable_mask = BIT(6),
		.travel_mask = BIT(15),
		.reg_grp = IQS9150_REG_GRP_2F,
		.reg_key = IQS9150_REG_KEY_SCROLL_Y,
	},
	{
		.name = "event-scroll-x-pos",
		.status_mask = BIT(7),
		.enable_mask = BIT(7),
		.reg_grp = IQS9150_REG_GRP_2F,
		.reg_key = IQS9150_REG_KEY_SCROLL_X,
	},
	{
		.name = "event-scroll-x-neg",
		.status_mask = BIT(7),
		.enable_mask = BIT(7),
		.travel_mask = BIT(15),
		.reg_grp = IQS9150_REG_GRP_2F,
		.reg_key = IQS9150_REG_KEY_SCROLL_X,
	},
	{
		.status_mask = BIT(10),
		.enable_mask = IQS9150_OTHER_SW_ENABLE,
		.reg_grp = IQS9150_REG_GRP_SW,
		.reg_key = IQS9150_REG_KEY_HOLD,
	},
	{
		.status_mask = BIT(8),
		.reg_grp = IQS9150_REG_GRP_ALP,
		.reg_key = IQS9150_REG_KEY_TAP,
	},
};

struct iqs9150_dev_desc {
	const char *tp_name;
	const char *kp_name;
	u16 prod_num;
	int num_rx;
	int num_tx;
	int min_tx;
};

static const struct iqs9150_dev_desc iqs9150_devs[] = {
	[IQS9150] = {
		.tp_name = "iqs9150_trackpad",
		.kp_name = "iqs9150_keys",
		.prod_num = 0x076A,
		.num_rx = IQS9150_NUM_RX,
		.num_tx = IQS9150_NUM_TX,
	},
	[IQS9151] = {
		.tp_name = "iqs9151_trackpad",
		.kp_name = "iqs9151_keys",
		.prod_num = 0x09BC,
		.num_rx = 13,
		.num_tx = 12,
		.min_tx = 33,
	},
};

struct iqs9150_prop_desc {
	const char *name;
	enum iqs9150_reg_key_id reg_key;
	u16 reg_addr[IQS9150_NUM_REG_GRPS];
	u16 reg_span[IQS9150_NUM_REG_GRPS];
	int reg_size;
	int reg_shift;
	int reg_width;
	int val_pitch;
	int val_min;
	int val_max;
	const char *label;
};

static const struct iqs9150_prop_desc iqs9150_props[] = {
	{
		.name = "azoteq,ati-comp-div",
		.reg_key = IQS9150_REG_KEY_SPAN,
		.reg_addr[IQS9150_REG_GRP_ALP] = 0x115C,
		.reg_span[IQS9150_REG_GRP_ALP] = IQS9150_NUM_RX / 2 - 1,
		.reg_size = sizeof(u16),
		.reg_shift = 10,
		.reg_width = 5,
		.label = "ATI compensation divider",
	},
	{
		.name = "azoteq,ati-comp-select",
		.reg_key = IQS9150_REG_KEY_SPAN,
		.reg_addr[IQS9150_REG_GRP_ALP] = 0x115C,
		.reg_span[IQS9150_REG_GRP_ALP] = IQS9150_NUM_RX / 2 - 1,
		.reg_size = sizeof(u16),
		.reg_shift = 0,
		.reg_width = 10,
		.label = "ATI compensation selection",
	},
	{
		.name = "azoteq,exp-settings-minor",
		.reg_addr[IQS9150_REG_GRP_SYS] = IQS9150_SETTINGS_MINOR,
		.label = "exported settings minor version",
	},
	{
		.name = "azoteq,exp-settings-major",
		.reg_addr[IQS9150_REG_GRP_SYS] = IQS9150_SETTINGS_MAJOR,
		.label = "exported settings major version",
	},
	{
		.name = "azoteq,ati-frac-mult-fine",
		.reg_key = IQS9150_REG_KEY_SPAN,
		.reg_addr = {
			[IQS9150_REG_GRP_TP] = 0x117A,
			[IQS9150_REG_GRP_ALP] = 0x117C,
		},
		.reg_span = {
			[IQS9150_REG_GRP_TP] = 0,
			[IQS9150_REG_GRP_ALP] = IQS9150_NUM_RX / 2 - 1,
		},
		.reg_size = sizeof(u16),
		.reg_shift = 14,
		.reg_width = 2,
		.label = "ATI fine fractional multiplier",
	},
	{
		.name = "azoteq,ati-frac-div-fine",
		.reg_key = IQS9150_REG_KEY_SPAN,
		.reg_addr = {
			[IQS9150_REG_GRP_TP] = 0x117A,
			[IQS9150_REG_GRP_ALP] = 0x117C,
		},
		.reg_span = {
			[IQS9150_REG_GRP_TP] = 0,
			[IQS9150_REG_GRP_ALP] = IQS9150_NUM_RX / 2 - 1,
		},
		.reg_size = sizeof(u16),
		.reg_shift = 9,
		.reg_width = 5,
		.label = "ATI fine fractional divider",
	},
	{
		.name = "azoteq,ati-frac-mult-coarse",
		.reg_key = IQS9150_REG_KEY_SPAN,
		.reg_addr = {
			[IQS9150_REG_GRP_TP] = 0x117A,
			[IQS9150_REG_GRP_ALP] = 0x117C,
		},
		.reg_span = {
			[IQS9150_REG_GRP_TP] = 0,
			[IQS9150_REG_GRP_ALP] = IQS9150_NUM_RX / 2 - 1,
		},
		.reg_size = sizeof(u16),
		.reg_shift = 5,
		.reg_width = 4,
		.label = "ATI coarse fractional multiplier",
	},
	{
		.name = "azoteq,ati-frac-div-coarse",
		.reg_key = IQS9150_REG_KEY_SPAN,
		.reg_addr = {
			[IQS9150_REG_GRP_TP] = 0x117A,
			[IQS9150_REG_GRP_ALP] = 0x117C,
		},
		.reg_span = {
			[IQS9150_REG_GRP_TP] = 0,
			[IQS9150_REG_GRP_ALP] = IQS9150_NUM_RX / 2 - 1,
		},
		.reg_size = sizeof(u16),
		.reg_shift = 0,
		.reg_width = 5,
		.label = "ATI coarse fractional divider",
	},
	{
		.name = "azoteq,ati-target",
		.reg_addr = {
			[IQS9150_REG_GRP_TP] = 0x1196,
			[IQS9150_REG_GRP_ALP] = 0x1198,
		},
		.reg_size = sizeof(u16),
		.label = "ATI target",
	},
	{
		.name = "azoteq,ati-base",
		.reg_addr[IQS9150_REG_GRP_ALP] = 0x119A,
		.reg_size = sizeof(u16),
		.label = "ATI base",
	},
	{
		.name = "azoteq,ati-delta-neg",
		.reg_addr[IQS9150_REG_GRP_TP] = 0x119C,
		.reg_size = sizeof(u16),
		.label = "ATI negative delta threshold",
	},
	{
		.name = "azoteq,ati-delta-pos",
		.reg_addr[IQS9150_REG_GRP_TP] = 0x119E,
		.reg_size = sizeof(u16),
		.label = "ATI positive delta threshold",
	},
	{
		.name = "azoteq,ati-drift",
		.reg_addr = {
			[IQS9150_REG_GRP_TP] = 0x11A0,
			[IQS9150_REG_GRP_ALP] = 0x11A1,
		},
		.label = "ATI drift limit",
	},
	{
		.name = "azoteq,rate-active-ms",
		.reg_addr[IQS9150_REG_GRP_SYS] = 0x11A2,
		.reg_size = sizeof(u16),
		.label = "active mode report rate",
	},
	{
		.name = "azoteq,rate-touch-ms",
		.reg_addr[IQS9150_REG_GRP_SYS] = 0x11A4,
		.reg_size = sizeof(u16),
		.label = "idle-touch mode report rate",
	},
	{
		.name = "azoteq,rate-idle-ms",
		.reg_addr[IQS9150_REG_GRP_SYS] = 0x11A6,
		.reg_size = sizeof(u16),
		.label = "idle mode report rate",
	},
	{
		.name = "azoteq,rate-lp1-ms",
		.reg_addr[IQS9150_REG_GRP_SYS] = 0x11A8,
		.reg_size = sizeof(u16),
		.label = "low-power mode 1 report rate",
	},
	{
		.name = "azoteq,rate-lp2-ms",
		.reg_addr[IQS9150_REG_GRP_SYS] = 0x11AA,
		.reg_size = sizeof(u16),
		.label = "low-power mode 2 report rate",
	},
	{
		.name = "azoteq,timeout-press-ms",
		.reg_addr[IQS9150_REG_GRP_SYS] = 0x11AC,
		.reg_size = sizeof(u16),
		.val_pitch = 1000,
		.label = "active mode (press) timeout",
	},
	{
		.name = "azoteq,timeout-touch-ms",
		.reg_addr[IQS9150_REG_GRP_SYS] = 0x11AE,
		.reg_size = sizeof(u16),
		.val_pitch = 1000,
		.label = "idle-touch mode timeout",
	},
	{
		.name = "azoteq,timeout-idle-ms",
		.reg_addr[IQS9150_REG_GRP_SYS] = 0x11B0,
		.reg_size = sizeof(u16),
		.val_pitch = 1000,
		.label = "idle mode timeout",
	},
	{
		.name = "azoteq,timeout-lp1-ms",
		.reg_addr[IQS9150_REG_GRP_SYS] = 0x11B2,
		.reg_size = sizeof(u16),
		.val_pitch = 1000,
		.label = "low-power mode 1 timeout",
	},
	{
		.name = "azoteq,timeout-release-ms",
		.reg_addr[IQS9150_REG_GRP_SYS] = 0x11B4,
		.reg_size = sizeof(u16),
		.label = "active mode (release) timeout",
	},
	{
		.name = "azoteq,timeout-ati-ms",
		.reg_addr[IQS9150_REG_GRP_SYS] = 0x11B6,
		.val_pitch = 1000,
		.val_max = 60000,
		.label = "ATI error timeout",
	},
	{
		.name = "azoteq,rate-ref-ms",
		.reg_addr[IQS9150_REG_GRP_SYS] = 0x11B7,
		.val_pitch = 1000,
		.val_max = 60000,
		.label = "trackpad reference value update rate",
	},
	{
		.name = "azoteq,timeout-snap-ms",
		.reg_addr[IQS9150_REG_GRP_SYS] = 0x11BA,
		.val_pitch = 1000,
		.label = "snap timeout",
	},
	{
		.name = "azoteq,sleep-conv",
		.reg_addr[IQS9150_REG_GRP_SYS] = IQS9150_CONFIG,
		.reg_size = sizeof(u16),
		.reg_shift = 5,
		.reg_width = 1,
		.label = "processing during conversions disable state",
	},
	{
		.name = "azoteq,ati-mode",
		.reg_addr[IQS9150_REG_GRP_ALP] = IQS9150_CONFIG,
		.reg_size = sizeof(u16),
		.reg_shift = 1,
		.reg_width = 1,
		.label = "ATI mode",
	},
	{
		.name = "azoteq,sleep-mode",
		.reg_addr[IQS9150_REG_GRP_SYS] = IQS9150_CONFIG,
		.reg_size = sizeof(u16),
		.reg_shift = 0,
		.reg_width = 1,
		.label = "sleep mode",
	},
	{
		.name = "azoteq,pin-polarity",
		.reg_addr[IQS9150_REG_GRP_SW] = IQS9150_OTHER,
		.reg_size = sizeof(u16),
		.reg_shift = 14,
		.reg_width = 1,
		.label = "pin polarity",
	},
	{
		.name = "azoteq,fosc-trim",
		.reg_addr[IQS9150_REG_GRP_SYS] = IQS9150_OTHER,
		.reg_size = sizeof(u16),
		.reg_shift = 12,
		.reg_width = 2,
		.label = "sensing engine oscillator frequency trim",
	},
	{
		.name = "azoteq,fosc-freq",
		.reg_addr[IQS9150_REG_GRP_SYS] = IQS9150_OTHER,
		.reg_size = sizeof(u16),
		.reg_shift = 6,
		.reg_width = 2,
		.val_max = 3,
		.label = "main oscillator frequency",
	},
	{
		.name = "azoteq,auto-prox-lp2",
		.reg_addr[IQS9150_REG_GRP_ALP] = IQS9150_OTHER,
		.reg_size = sizeof(u16),
		.reg_shift = 5,
		.reg_width = 1,
		.label = "low-power mode 2 auto prox enable state",
	},
	{
		.name = "azoteq,auto-prox-lp1",
		.reg_addr[IQS9150_REG_GRP_ALP] = IQS9150_OTHER,
		.reg_size = sizeof(u16),
		.reg_shift = 4,
		.reg_width = 1,
		.label = "low-power mode 1 auto prox enable state",
	},
	{
		.name = "azoteq,auto-prox-cycles-lp2",
		.reg_addr[IQS9150_REG_GRP_ALP] = IQS9150_OTHER,
		.reg_size = sizeof(u16),
		.reg_shift = 2,
		.reg_width = 2,
		.label = "low-power mode 2 auto prox number of cycles",
	},
	{
		.name = "azoteq,auto-prox-cycles-lp1",
		.reg_addr[IQS9150_REG_GRP_ALP] = IQS9150_OTHER,
		.reg_size = sizeof(u16),
		.reg_shift = 0,
		.reg_width = 2,
		.label = "low-power mode 1 auto prox number of cycles",
	},
	{
		.name = "azoteq,count-filter",
		.reg_addr[IQS9150_REG_GRP_ALP] = IQS9150_ALP_SETUP,
		.reg_shift = 6,
		.reg_width = 1,
		.label = "count filter enable state",
	},
	{
		.name = "azoteq,sense-mode",
		.reg_addr[IQS9150_REG_GRP_ALP] = IQS9150_ALP_SETUP,
		.reg_shift = 5,
		.reg_width = 1,
		.label = "sensing mode",
	},
	{
		.name = "azoteq,tx-shield",
		.reg_addr[IQS9150_REG_GRP_ALP] = IQS9150_ALP_SETUP,
		.reg_shift = 4,
		.reg_width = 1,
		.label = "TX pin shield state",
	},
	{
		.name = "azoteq,touch-enter",
		.reg_addr[IQS9150_REG_GRP_TP] = 0x11CC,
		.label = "touch entrance factor",
	},
	{
		.name = "azoteq,touch-exit",
		.reg_addr[IQS9150_REG_GRP_TP] = 0x11CD,
		.label = "touch exit factor",
	},
	{
		.name = "azoteq,thresh",
		.reg_addr[IQS9150_REG_GRP_ALP] = 0x11CE,
		.label = "threshold",
	},
	{
		.name = "azoteq,auto-prox-delta",
		.reg_addr[IQS9150_REG_GRP_ALP] = 0x11CF,
		.label = "auto prox delta threshold",
	},
	{
		.name = "azoteq,debounce-enter",
		.reg_addr[IQS9150_REG_GRP_ALP] = 0x11D0,
		.label = "debounce entrance factor",
	},
	{
		.name = "azoteq,debounce-exit",
		.reg_addr[IQS9150_REG_GRP_ALP] = 0x11D1,
		.label = "debounce exit factor",
	},
	{
		.name = "azoteq,snap-enter",
		.reg_addr[IQS9150_REG_GRP_TP] = 0x11D2,
		.label = "snap entrance factor",
	},
	{
		.name = "azoteq,snap-exit",
		.reg_addr[IQS9150_REG_GRP_TP] = 0x11D3,
		.label = "snap exit factor",
	},
	{
		.name = "azoteq,counts-beta-lp1",
		.reg_addr[IQS9150_REG_GRP_ALP] = 0x11D4,
		.label = "low-power mode 1 counts beta",
	},
	{
		.name = "azoteq,lta-beta-lp1",
		.reg_addr[IQS9150_REG_GRP_ALP] = 0x11D5,
		.label = "low-power mode 1 long-term average beta",
	},
	{
		.name = "azoteq,counts-beta-lp2",
		.reg_addr[IQS9150_REG_GRP_ALP] = 0x11D6,
		.label = "low-power mode 2 counts beta",
	},
	{
		.name = "azoteq,lta-beta-lp2",
		.reg_addr[IQS9150_REG_GRP_ALP] = 0x11D7,
		.label = "low-power mode 2 long-term average beta",
	},
	{
		.name = "azoteq,conv-frac",
		.reg_addr = {
			[IQS9150_REG_GRP_TP] = 0x11D8,
			[IQS9150_REG_GRP_ALP] = 0x11DB,
		},
		.label = "conversion frequency fractional divider",
	},
	{
		.name = "azoteq,conv-period-1",
		.reg_addr = {
			[IQS9150_REG_GRP_TP] = 0x11D9,
			[IQS9150_REG_GRP_ALP] = 0x11DC,
		},
		.label = "conversion period 1",
	},
	{
		.name = "azoteq,conv-period-2",
		.reg_addr = {
			[IQS9150_REG_GRP_TP] = 0x11DA,
			[IQS9150_REG_GRP_ALP] = 0x11DD,
		},
		.label = "conversion period 2",
	},
	{
		.name = "azoteq,delay-cycles",
		.reg_addr = {
			[IQS9150_REG_GRP_TP] = 0x11DE,
			[IQS9150_REG_GRP_ALP] = 0x11E0,
		},
		.reg_size = sizeof(u16),
		.reg_shift = 14,
		.reg_width = 2,
		.label = "initial cycle delay",
	},
	{
		.name = "azoteq,proj-bias",
		.reg_addr = {
			[IQS9150_REG_GRP_TP] = 0x11DE,
			[IQS9150_REG_GRP_ALP] = 0x11E0,
		},
		.reg_size = sizeof(u16),
		.reg_shift = 11,
		.reg_width = 3,
		.label = "projected bias current",
	},
	{
		.name = "azoteq,max-counts",
		.reg_addr = {
			[IQS9150_REG_GRP_TP] = 0x11DE,
			[IQS9150_REG_GRP_ALP] = 0x11E0,
		},
		.reg_size = sizeof(u16),
		.reg_shift = 8,
		.reg_width = 3,
		.val_max = 4,
		.label = "maximum counts",
	},
	{
		.name = "azoteq,samp-cap-discharge",
		.reg_addr = {
			[IQS9150_REG_GRP_TP] = 0x11DE,
			[IQS9150_REG_GRP_ALP] = 0x11E0,
		},
		.reg_size = sizeof(u16),
		.reg_shift = 7,
		.reg_width = 1,
		.label = "sample capacitor discharge voltage",
	},
	{
		.name = "azoteq,rf-filter",
		.reg_addr = {
			[IQS9150_REG_GRP_TP] = 0x11DE,
			[IQS9150_REG_GRP_ALP] = 0x11E0,
		},
		.reg_size = sizeof(u16),
		.reg_shift = 6,
		.reg_width = 1,
		.label = "RF filter enable state",
	},
	{
		.name = "azoteq,nm-static-out",
		.reg_addr = {
			[IQS9150_REG_GRP_TP] = 0x11DE,
			[IQS9150_REG_GRP_ALP] = 0x11E0,
		},
		.reg_size = sizeof(u16),
		.reg_shift = 5,
		.reg_width = 1,
		.label = "NM static output enable state",
	},
	{
		.name = "azoteq,nm-static-in",
		.reg_addr = {
			[IQS9150_REG_GRP_TP] = 0x11DE,
			[IQS9150_REG_GRP_ALP] = 0x11E0,
		},
		.reg_size = sizeof(u16),
		.reg_shift = 4,
		.reg_width = 1,
		.label = "NM static input enable state",
	},
	{
		.name = "azoteq,proj-offset",
		.reg_addr = {
			[IQS9150_REG_GRP_TP] = 0x11DE,
			[IQS9150_REG_GRP_ALP] = 0x11E0,
		},
		.reg_size = sizeof(u16),
		.reg_shift = 0,
		.reg_width = 4,
		.label = "projected offset voltage",
	},
	{
		.name = "azoteq,area-filter",
		.reg_addr[IQS9150_REG_GRP_TP] = 0x11E2,
		.reg_shift = 6,
		.reg_width = 1,
		.label = "area filter disable state",
	},
	{
		.name = "azoteq,jitter-filter",
		.reg_addr[IQS9150_REG_GRP_TP] = 0x11E2,
		.reg_shift = 5,
		.reg_width = 1,
		.label = "jitter filter enable state",
	},
	{
		.name = "azoteq,iir-static",
		.reg_addr[IQS9150_REG_GRP_TP] = 0x11E2,
		.reg_shift = 4,
		.reg_width = 1,
		.label = "IIR filtering method",
	},
	{
		.name = "azoteq,iir-filter",
		.reg_addr[IQS9150_REG_GRP_TP] = 0x11E2,
		.reg_shift = 3,
		.reg_width = 1,
		.label = "IIR filter enable state",
	},
	{
		.name = "azoteq,num-contacts",
		.reg_addr[IQS9150_REG_GRP_TP] = IQS9150_NUM_CONTACTS,
		.val_min = 1,
		.val_max = IQS9150_MAX_CONTACTS,
		.label = "number of contacts",
	},
	{
		.name = "azoteq,bottom-speed",
		.reg_addr[IQS9150_REG_GRP_TP] = 0x11EA,
		.reg_size = sizeof(u16),
		.label = "bottom speed",
	},
	{
		.name = "azoteq,top-speed",
		.reg_addr[IQS9150_REG_GRP_TP] = 0x11EC,
		.reg_size = sizeof(u16),
		.label = "top speed",
	},
	{
		.name = "azoteq,bottom-beta",
		.reg_addr[IQS9150_REG_GRP_TP] = 0x11EE,
		.label = "bottom beta",
	},
	{
		.name = "azoteq,static-beta",
		.reg_addr[IQS9150_REG_GRP_TP] = 0x11EF,
		.label = "static beta",
	},
	{
		.name = "azoteq,thresh",
		.reg_addr[IQS9150_REG_GRP_TP] = 0x11F0,
		.label = "threshold",
	},
	{
		.name = "azoteq,contact-split",
		.reg_addr[IQS9150_REG_GRP_TP] = 0x11F1,
		.label = "contact split factor",
	},
	{
		.name = "azoteq,trim-x",
		.reg_addr[IQS9150_REG_GRP_TP] = 0x11F2,
		.label = "horizontal trim width",
	},
	{
		.name = "azoteq,trim-y",
		.reg_addr[IQS9150_REG_GRP_TP] = 0x11F3,
		.label = "vertical trim height",
	},
	{
		.name = "azoteq,jitter-delta",
		.reg_addr[IQS9150_REG_GRP_TP] = 0x11F4,
		.label = "jitter filter delta threshold",
	},
	{
		.name = "azoteq,contact-confidence",
		.reg_addr[IQS9150_REG_GRP_TP] = 0x11F5,
		.label = "contact confidence threshold",
	},
	{
		.name = "azoteq,gesture-max-ms",
		.reg_key = IQS9150_REG_KEY_TAP,
		.reg_addr = {
			[IQS9150_REG_GRP_1F] = 0x11FA,
			[IQS9150_REG_GRP_2F] = 0x11FA,
		},
		.reg_size = sizeof(u16),
		.label = "maximum gesture time",
	},
	{
		.name = "azoteq,gesture-mid-ms",
		.reg_key = IQS9150_REG_KEY_TAP,
		.reg_addr = {
			[IQS9150_REG_GRP_1F] = 0x11FC,
			[IQS9150_REG_GRP_2F] = 0x11FC,
		},
		.reg_size = sizeof(u16),
		.label = "repeated gesture time",
	},
	{
		.name = "azoteq,gesture-dist",
		.reg_key = IQS9150_REG_KEY_TAP,
		.reg_addr = {
			[IQS9150_REG_GRP_1F] = 0x11FE,
			[IQS9150_REG_GRP_2F] = 0x11FE,
		},
		.reg_size = sizeof(u16),
		.label = "gesture distance",
	},
	{
		.name = "azoteq,gesture-dist",
		.reg_key = IQS9150_REG_KEY_HOLD,
		.reg_addr = {
			[IQS9150_REG_GRP_1F] = 0x11FE,
			[IQS9150_REG_GRP_2F] = 0x11FE,
		},
		.reg_size = sizeof(u16),
		.label = "gesture distance",
	},
	{
		.name = "azoteq,gesture-min-ms",
		.reg_key = IQS9150_REG_KEY_HOLD,
		.reg_addr = {
			[IQS9150_REG_GRP_1F] = 0x1200,
			[IQS9150_REG_GRP_2F] = 0x1200,
		},
		.reg_size = sizeof(u16),
		.label = "minimum gesture time",
	},
	{
		.name = "azoteq,gesture-max-ms",
		.reg_key = IQS9150_REG_KEY_AXIAL_X,
		.reg_addr[IQS9150_REG_GRP_1F] = 0x1202,
		.reg_size = sizeof(u16),
		.label = "maximum gesture time",
	},
	{
		.name = "azoteq,gesture-max-ms",
		.reg_key = IQS9150_REG_KEY_AXIAL_Y,
		.reg_addr[IQS9150_REG_GRP_1F] = 0x1202,
		.reg_size = sizeof(u16),
		.label = "maximum gesture time",
	},
	{
		.name = "azoteq,gesture-dist",
		.reg_key = IQS9150_REG_KEY_AXIAL_X,
		.reg_addr[IQS9150_REG_GRP_1F] = 0x1204,
		.reg_size = sizeof(u16),
		.label = "gesture distance",
	},
	{
		.name = "azoteq,gesture-dist",
		.reg_key = IQS9150_REG_KEY_AXIAL_Y,
		.reg_addr[IQS9150_REG_GRP_1F] = 0x1206,
		.reg_size = sizeof(u16),
		.label = "gesture distance",
	},
	{
		.name = "azoteq,gesture-dist-rep",
		.reg_key = IQS9150_REG_KEY_AXIAL_X,
		.reg_addr[IQS9150_REG_GRP_1F] = 0x1208,
		.reg_size = sizeof(u16),
		.label = "repeated gesture distance",
	},
	{
		.name = "azoteq,gesture-dist-rep",
		.reg_key = IQS9150_REG_KEY_AXIAL_Y,
		.reg_addr[IQS9150_REG_GRP_1F] = 0x120A,
		.reg_size = sizeof(u16),
		.label = "repeated gesture distance",
	},
	{
		.name = "azoteq,gesture-dist",
		.reg_key = IQS9150_REG_KEY_ZOOM,
		.reg_addr[IQS9150_REG_GRP_2F] = 0x120E,
		.reg_size = sizeof(u16),
		.label = "gesture distance",
	},
	{
		.name = "azoteq,gesture-dist-rep",
		.reg_key = IQS9150_REG_KEY_ZOOM,
		.reg_addr[IQS9150_REG_GRP_2F] = 0x1210,
		.reg_size = sizeof(u16),
		.label = "repeated gesture distance",
	},
	{
		.name = "azoteq,gesture-dist",
		.reg_key = IQS9150_REG_KEY_SCROLL_X,
		.reg_addr[IQS9150_REG_GRP_2F] = 0x1212,
		.reg_size = sizeof(u16),
		.label = "gesture distance",
	},
	{
		.name = "azoteq,gesture-dist",
		.reg_key = IQS9150_REG_KEY_SCROLL_Y,
		.reg_addr[IQS9150_REG_GRP_2F] = 0x1212,
		.reg_size = sizeof(u16),
		.label = "gesture distance",
	},
	{
		.name = "azoteq,gesture-dist-rep",
		.reg_key = IQS9150_REG_KEY_SCROLL_X,
		.reg_addr[IQS9150_REG_GRP_2F] = 0x1214,
		.reg_size = sizeof(u16),
		.label = "repeated gesture distance",
	},
	{
		.name = "azoteq,gesture-dist-rep",
		.reg_key = IQS9150_REG_KEY_SCROLL_Y,
		.reg_addr[IQS9150_REG_GRP_2F] = 0x1214,
		.reg_size = sizeof(u16),
		.label = "repeated gesture distance",
	},
	{
		.name = "azoteq,thresh",
		.reg_key = IQS9150_REG_KEY_PALM,
		.reg_addr[IQS9150_REG_GRP_1F] = 0x1216,
		.reg_size = sizeof(u16),
		.val_max = IQS9150_NUM_CHANNELS,
		.label = "threshold",
	},
	{
		.name = "azoteq,channel-ignore",
		.reg_key = IQS9150_REG_KEY_MASK,
		.reg_addr[IQS9150_REG_GRP_TP] = 0x1246,
		.reg_size = 88,
		.val_max = IQS9150_NUM_CHANNELS - 1,
		.label = "ignored channel(s)",
	},
	{
		.name = "azoteq,snap-select",
		.reg_key = IQS9150_REG_KEY_MASK,
		.reg_addr[IQS9150_REG_GRP_TP] = 0x129E,
		.reg_size = 88,
		.val_max = IQS9150_NUM_CHANNELS - 1,
		.label = "snap channel(s)",
	},
	{
		.name = "azoteq,trim-touch",
		.reg_addr[IQS9150_REG_GRP_TP] = 0x12F6,
		.reg_size = IQS9150_NUM_CHANNELS,
		.val_max = U8_MAX,
		.label = "touch trim adjustment(s)",
	},
};

static const u8 iqs9150_gesture_angle[] = {
	0x00, 0x01, 0x02, 0x03,
	0x04, 0x06, 0x07, 0x08,
	0x09, 0x0A, 0x0B, 0x0C,
	0x0E, 0x0F, 0x10, 0x11,
	0x12, 0x14, 0x15, 0x16,
	0x17, 0x19, 0x1A, 0x1B,
	0x1C, 0x1E, 0x1F, 0x21,
	0x22, 0x23, 0x25, 0x26,
	0x28, 0x2A, 0x2B, 0x2D,
	0x2E, 0x30, 0x32, 0x34,
	0x36, 0x38, 0x3A, 0x3C,
	0x3E, 0x40, 0x42, 0x45,
	0x47, 0x4A, 0x4C, 0x4F,
	0x52, 0x55, 0x58, 0x5B,
	0x5F, 0x63, 0x66, 0x6B,
	0x6F, 0x73, 0x78, 0x7E,
	0x83, 0x89, 0x90, 0x97,
	0x9E, 0xA7, 0xB0, 0xBA,
	0xC5, 0xD1, 0xDF, 0xEF,
};

struct iqs9150_ver_info {
	__le16 prod_num;
	__le16 major;
	__le16 minor;
	__le32 patch;
} __packed;

struct iqs9150_touch_data {
	__le16 abs_x;
	__le16 abs_y;
	__le16 pressure;
	__le16 area;
} __packed;

struct iqs9150_status {
	__le16 gesture_x;
	__le16 gesture_y;
	__le16 flags[4];
	struct iqs9150_touch_data touch_data[IQS9150_MAX_CONTACTS];
} __packed;

struct iqs9150_private {
	const struct iqs9150_dev_desc *dev_desc;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *irq_gpio;
	struct i2c_client *client;
	struct input_dev *tp_idev;
	struct input_dev *kp_idev;
	struct iqs9150_ver_info ver_info;
	struct iqs9150_status status;
	struct touchscreen_properties prop;
	enum iqs9150_comms_mode comms_mode;
	unsigned int kp_type[ARRAY_SIZE(iqs9150_kp_events)];
	unsigned int kp_code[ARRAY_SIZE(iqs9150_kp_events)];
	u8 reg_buf[IQS9150_REG_BUF_LEN];
	u8 eng_buf[IQS9150_ENG_BUF_LEN];
};

static int iqs9150_irq_poll(struct iqs9150_private *iqs9150, u64 timeout_us)
{
	int error, val;

	error = readx_poll_timeout(gpiod_get_value_cansleep, iqs9150->irq_gpio,
				   val, val, IQS9150_COMMS_SLEEP_US, timeout_us);

	return val < 0 ? val : error;
}

static int iqs9150_hard_reset(struct iqs9150_private *iqs9150)
{
	if (!iqs9150->reset_gpio)
		return 0;

	gpiod_set_value_cansleep(iqs9150->reset_gpio, 1);
	usleep_range(1000, 1100);

	gpiod_set_value_cansleep(iqs9150->reset_gpio, 0);

	return iqs9150_irq_poll(iqs9150, IQS9150_START_TIMEOUT_US);
}

static void iqs9150_hold_reset(void *data)
{
	struct iqs9150_private *iqs9150 = data;

	gpiod_set_value_cansleep(iqs9150->reset_gpio, 1);
}

static int iqs9150_force_comms(struct iqs9150_private *iqs9150)
{
	u8 msg_buf[] = { 0xFF, };
	int ret;

	switch (iqs9150->comms_mode) {
	case IQS9150_COMMS_MODE_WAIT:
		return iqs9150_irq_poll(iqs9150, IQS9150_START_TIMEOUT_US);

	case IQS9150_COMMS_MODE_FREE:
		return 0;

	case IQS9150_COMMS_MODE_FORCE:
		break;

	default:
		return -EINVAL;
	}

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
	ret = gpiod_get_value_cansleep(iqs9150->irq_gpio);
	if (ret < 0)
		return ret;
	else if (ret > 0)
		return 0;

	ret = i2c_master_send(iqs9150->client, msg_buf, sizeof(msg_buf));
	if (ret < (int)sizeof(msg_buf)) {
		if (ret >= 0)
			ret = -EIO;

		msleep(IQS9150_COMMS_RETRY_MS);
		return ret;
	}

	iqs9150_irq_wait();

	return iqs9150_irq_poll(iqs9150, IQS9150_COMMS_TIMEOUT_US);
}

static int __iqs9150_read_burst(struct iqs9150_private *iqs9150,
				u16 reg, void *val, u16 val_len)
{
	__le16 reg_buf = cpu_to_le16(reg);
	int ret, i;
	struct i2c_client *client = iqs9150->client;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(reg_buf),
			.buf = (u8 *)&reg_buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = val_len,
			.buf = (u8 *)val,
		},
	};

	/*
	 * The following loop protects against an edge case in which the RDY
	 * pin is automatically deasserted just as the read is initiated. In
	 * that case, the read must be retried using forced communication.
	 */
	for (i = 0; i < IQS9150_NUM_RETRIES; i++) {
		ret = iqs9150_force_comms(iqs9150);
		if (ret < 0)
			continue;

		ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		if (ret < (int)ARRAY_SIZE(msg)) {
			if (ret >= 0)
				ret = -EIO;

			msleep(IQS9150_COMMS_RETRY_MS);
			continue;
		}

		if (get_unaligned_le16(msg[1].buf) == IQS9150_COMMS_ERROR) {
			ret = -ENODATA;
			continue;
		}

		ret = 0;
		break;
	}

	iqs9150_irq_wait();

	if (ret < 0)
		dev_err(&client->dev,
			"Failed to read from address 0x%02X: %d\n", reg, ret);

	return ret;
}

static int iqs9150_read_burst(struct iqs9150_private *iqs9150,
			      u16 reg, void *val, u16 val_len)
{
	u16 rem_len = val_len;
	int error = 0;

	while (rem_len) {
		u16 burst_len = min(rem_len, IQS9150_MAX_LEN);
		u16 reg_offs = val_len - rem_len;

		error = __iqs9150_read_burst(iqs9150, reg + reg_offs,
					     val + reg_offs, burst_len);
		if (error)
			break;

		rem_len -= burst_len;
	}

	return error;
}

static int iqs9150_read_word(struct iqs9150_private *iqs9150, u16 reg, u16 *val)
{
	__le16 val_buf;
	int error;

	error = iqs9150_read_burst(iqs9150, reg, &val_buf, sizeof(val_buf));
	if (error)
		return error;

	*val = le16_to_cpu(val_buf);

	return 0;
}

static int __iqs9150_write_burst(struct iqs9150_private *iqs9150,
				 u16 reg, const void *val, u16 val_len)
{
	int msg_len = sizeof(reg) + val_len;
	int ret, i;
	struct i2c_client *client = iqs9150->client;
	u8 *msg_buf;

	msg_buf = kzalloc(msg_len, GFP_KERNEL);
	if (!msg_buf)
		return -ENOMEM;

	put_unaligned_le16(reg, msg_buf);
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
	for (i = 0; i < IQS9150_NUM_RETRIES; i++) {
		ret = iqs9150_force_comms(iqs9150);
		if (ret < 0)
			continue;

		ret = i2c_master_send(client, msg_buf, msg_len);
		if (ret < msg_len) {
			if (ret >= 0)
				ret = -EIO;

			msleep(IQS9150_COMMS_RETRY_MS);
			continue;
		}

		ret = 0;
		break;
	}

	kfree(msg_buf);

	iqs9150_irq_wait();

	if (ret < 0)
		dev_err(&client->dev,
			"Failed to write to address 0x%02X: %d\n", reg, ret);

	return ret;
}

static int iqs9150_write_burst(struct iqs9150_private *iqs9150,
			       u16 reg, const void *val, u16 val_len)
{
	u16 rem_len = val_len;
	int error = 0;

	while (rem_len) {
		u16 burst_len = min(rem_len, IQS9150_MAX_LEN);
		u16 reg_offs = val_len - rem_len;

		error = __iqs9150_write_burst(iqs9150, reg + reg_offs,
					      val + reg_offs, burst_len);
		if (error)
			break;

		rem_len -= burst_len;
	}

	return error;
}

static int iqs9150_write_word(struct iqs9150_private *iqs9150, u16 reg, u16 val)
{
	__le16 val_buf = cpu_to_le16(val);

	return iqs9150_write_burst(iqs9150, reg, &val_buf, sizeof(val_buf));
}

static int iqs9150_start_comms(struct iqs9150_private *iqs9150)
{
	const struct iqs9150_dev_desc *dev_desc = iqs9150->dev_desc;
	struct i2c_client *client = iqs9150->client;
	unsigned int timeout_comms = 0;
	bool forced_comms;
	u16 config;
	int error;

	/*
	 * Until forced communication can be enabled, the host must wait for a
	 * communication window each time it intends to elicit a response from
	 * the device.
	 *
	 * Forced communication is not necessary, however, if the host adapter
	 * can support clock stretching. In that case, the device freely clock
	 * stretches until all pending conversions are complete.
	 */
	forced_comms = device_property_present(&client->dev,
					       "azoteq,forced-comms");

	error = device_property_read_u32(&client->dev,
					 "azoteq,timeout-comms-ms",
					 &timeout_comms);
	if (error && error != -EINVAL) {
		dev_err(&client->dev,
			"Failed to read communication timeout: %d\n", error);
		return error;
	} else if (timeout_comms > U16_MAX) {
		dev_err(&client->dev,
			"Invalid communication timeout: %u\n", timeout_comms);
		return -EINVAL;
	}

	error = iqs9150_hard_reset(iqs9150);
	if (error) {
		dev_err(&client->dev, "Failed to reset device: %d\n", error);
		return error;
	}

	error = iqs9150_read_burst(iqs9150, IQS9150_PROD_NUM,
				   &iqs9150->ver_info,
				   sizeof(iqs9150->ver_info));
	if (error)
		return error;

	if (le16_to_cpu(iqs9150->ver_info.prod_num) != dev_desc->prod_num) {
		dev_err(&client->dev, "Invalid product number: %u\n",
			le16_to_cpu(iqs9150->ver_info.prod_num));
		return -EINVAL;
	}

	error = iqs9150_read_word(iqs9150, IQS9150_CONFIG, &config);
	if (error)
		return error;

	if (forced_comms)
		config |= IQS9150_CONFIG_FORCED_COMMS;
	else
		config &= ~IQS9150_CONFIG_FORCED_COMMS;

	config &= ~IQS9150_CONFIG_EVENT_MASK;
	config |= (IQS9150_CONFIG_EVENT_MODE |
		   IQS9150_CONFIG_EVENT_ATI |
		   iqs9150_reg_grps[IQS9150_REG_GRP_TP].event_mask);

	error = iqs9150_write_word(iqs9150, IQS9150_CONFIG, config);
	if (error)
		return error;

	if (forced_comms)
		iqs9150->comms_mode = IQS9150_COMMS_MODE_FORCE;
	else
		iqs9150->comms_mode = IQS9150_COMMS_MODE_FREE;

	if (timeout_comms) {
		error = iqs9150_write_word(iqs9150, IQS9150_TIMEOUT_COMMS,
					   timeout_comms);
		if (error)
			return error;
	}

	error = iqs9150_read_burst(iqs9150, IQS9150_REG_BUF_START,
				   &iqs9150_reg(IQS9150_REG_BUF_START),
				   IQS9150_REG_BUF_LEN);
	if (error)
		return error;

	iqs9150_put_word(IQS9150_CONTROL, 0);

	iqs9150_put_word(IQS9150_OTHER,
			 iqs9150_get_word(IQS9150_OTHER) &
			 ~IQS9150_OTHER_SW_ENABLE);

	iqs9150_reg(IQS9150_ALP_SETUP) &= ~IQS9150_ALP_SETUP_ENABLE;

	return 0;
}

static int iqs9150_init_device(struct iqs9150_private *iqs9150)
{
	int error;

	/*
	 * Acknowledge reset before writing any registers in case the device
	 * suffers a spurious reset during initialization.
	 */
	error = iqs9150_write_word(iqs9150, IQS9150_CONTROL,
				   IQS9150_CONTROL_ACK_RESET);
	if (error)
		return error;

	error = iqs9150_write_word(iqs9150, IQS9150_TIMEOUT_COMMS,
				   iqs9150_get_word(IQS9150_TIMEOUT_COMMS));
	if (error)
		return error;

	error = iqs9150_write_burst(iqs9150, IQS9150_REG_BUF_START,
				    &iqs9150_reg(IQS9150_REG_BUF_START),
				    IQS9150_REG_BUF_LEN);
	if (error)
		return error;

	if (get_unaligned_le16(iqs9150->eng_buf) != IQS9150_COMMS_ERROR) {
		error = iqs9150_write_burst(iqs9150, IQS9150_ENG_BUF_START,
					    iqs9150->eng_buf,
					    IQS9150_ENG_BUF_LEN);
		if (error)
			return error;
	}

	error = iqs9150_write_word(iqs9150, IQS9150_CONTROL,
				   IQS9150_CONTROL_ATI_ALP |
				   IQS9150_CONTROL_ATI_TP);
	if (error)
		return error;

	return 0;
}

static int iqs9150_parse_props(struct iqs9150_private *iqs9150,
			       struct fwnode_handle *reg_grp_node,
			       enum iqs9150_reg_grp_id reg_grp,
			       enum iqs9150_reg_key_id reg_key,
			       unsigned int index)
{
	struct i2c_client *client = iqs9150->client;
	int i;

	for (i = 0; i < ARRAY_SIZE(iqs9150_props); i++) {
		const char *name = iqs9150_props[i].name;
		u16 reg_addr = iqs9150_props[i].reg_addr[reg_grp];
		u16 reg_offs;
		int reg_size = iqs9150_props[i].reg_size ? : sizeof(u8);
		int reg_shift = iqs9150_props[i].reg_shift;
		int reg_width = iqs9150_props[i].reg_width ? : reg_size *
							       BITS_PER_BYTE;
		int val_pitch = iqs9150_props[i].val_pitch ? : 1;
		int val_min = iqs9150_props[i].val_min;
		int val_max = iqs9150_props[i].val_max;
		const char *label = iqs9150_props[i].label ? : name;
		unsigned int val, val_buf;
		int error;

		if (iqs9150_props[i].reg_key != reg_key)
			continue;

		if (!reg_addr || reg_size > sizeof(u16))
			continue;

		if (index > iqs9150_props[i].reg_span[reg_grp]) {
			dev_err(&client->dev, "Invalid %s index: %u\n",
				fwnode_get_name(reg_grp_node), index);
			return -EINVAL;
		}

		reg_offs = reg_addr + index * reg_size;

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
			dev_err(&client->dev, "Invalid %s: %u\n", label, val);
			return -EINVAL;
		}

		if (reg_size > sizeof(u8))
			val_buf = iqs9150_get_word(reg_offs);
		else
			val_buf = iqs9150_reg(reg_offs);

		val_buf &= ~GENMASK(reg_shift + reg_width - 1, reg_shift);
		val_buf |= (val / val_pitch << reg_shift);

		if (reg_size > sizeof(u8))
			iqs9150_put_word(reg_offs, val_buf);
		else
			iqs9150_reg(reg_offs) = val_buf;
	}

	return 0;
}

static int iqs9150_parse_event(struct iqs9150_private *iqs9150,
			       struct fwnode_handle *event_node,
			       enum iqs9150_reg_grp_id reg_grp,
			       enum iqs9150_reg_key_id reg_key,
			       unsigned int index)
{
	struct i2c_client *client = iqs9150->client;
	unsigned int val;
	int error;

	error = iqs9150_parse_props(iqs9150, event_node, reg_grp, reg_key, 0);
	if (error)
		return error;

	if (reg_key == IQS9150_REG_KEY_AXIAL_X ||
	    reg_key == IQS9150_REG_KEY_AXIAL_Y ||
	    reg_key == IQS9150_REG_KEY_SCROLL_X ||
	    reg_key == IQS9150_REG_KEY_SCROLL_Y) {
		error = fwnode_property_read_u32(event_node,
						 "azoteq,gesture-angle", &val);
		if (!error) {
			u16 reg_addr;

			if (val >= ARRAY_SIZE(iqs9150_gesture_angle)) {
				dev_err(&client->dev,
					"Invalid %s gesture angle: %u\n",
					fwnode_get_name(event_node), val);
				return -EINVAL;
			}

			if (reg_key == IQS9150_REG_KEY_AXIAL_X ||
			    reg_key == IQS9150_REG_KEY_AXIAL_Y)
				reg_addr = IQS9150_ANGLE_AXIAL;
			else
				reg_addr = IQS9150_ANGLE_SCROLL;

			iqs9150_reg(reg_addr) = iqs9150_gesture_angle[val];
		} else if (error != -EINVAL) {
			dev_err(&client->dev,
				"Failed to read %s gesture angle: %d\n",
				fwnode_get_name(event_node), error);
			return error;
		}
	}

	error = fwnode_property_read_u32(event_node, "linux,code", &val);
	if (error == -EINVAL && reg_grp != IQS9150_REG_GRP_SW) {
		return 0;
	} else if (error) {
		dev_err(&client->dev, "Failed to read %s code: %d\n",
			fwnode_get_name(event_node), error);
		return error;
	}

	iqs9150->kp_code[index] = val;

	if (reg_grp == IQS9150_REG_GRP_SW) {
		error = fwnode_property_read_u32(event_node,
						 "linux,input-type", &val);
		if (error == -EINVAL) {
			val = EV_KEY;
		} else if (error) {
			dev_err(&client->dev,
				"Failed to read %s input type: %d\n",
				fwnode_get_name(event_node), error);
			return error;
		} else if (val != EV_KEY && val != EV_SW) {
			dev_err(&client->dev, "Invalid %s input type: %u\n",
				fwnode_get_name(event_node), val);
			return -EINVAL;
		}
	} else {
		val = EV_KEY;
	}

	iqs9150->kp_type[index] = val;

	return 0;
}

static int iqs9150_parse_tp(struct iqs9150_private *iqs9150,
			    struct fwnode_handle *tp_node)
{
	const struct iqs9150_dev_desc *dev_desc = iqs9150->dev_desc;
	struct i2c_client *client = iqs9150->client;
	int error, total_rx, total_tx, i, j;
	unsigned int pins[IQS9150_NUM_RX];

	total_rx = fwnode_property_count_u32(tp_node, "azoteq,rx-enable");
	if (total_rx == -EINVAL) {
		return 0;
	} else if (total_rx < 0) {
		dev_err(&client->dev, "Failed to count %s RX pins: %d\n",
			fwnode_get_name(tp_node), total_rx);
		return total_rx;
	} else if (total_rx > dev_desc->num_rx) {
		dev_err(&client->dev, "Invalid number of %s RX pins\n",
			fwnode_get_name(tp_node));
		return -EINVAL;
	}

	error = fwnode_property_read_u32_array(tp_node, "azoteq,rx-enable",
					       pins, total_rx);
	if (error) {
		dev_err(&client->dev, "Failed to read %s RX pins: %d\n",
			fwnode_get_name(tp_node), error);
		return error;
	}

	for (i = 0; i < total_rx; i++) {
		if (pins[i] > dev_desc->num_rx - 1) {
			dev_err(&client->dev, "Invalid %s RX pin: %u\n",
				fwnode_get_name(tp_node), pins[i]);
			return -EINVAL;
		}

		iqs9150_reg(IQS9150_RX_TX_MAP + i) = pins[i];
	}

	total_tx = fwnode_property_count_u32(tp_node, "azoteq,tx-enable");
	if (total_tx < 0) {
		dev_err(&client->dev, "Failed to count %s TX pins: %d\n",
			fwnode_get_name(tp_node), total_tx);
		return total_tx;
	} else if (total_tx > dev_desc->num_tx) {
		dev_err(&client->dev, "Invalid number of %s TX pins\n",
			fwnode_get_name(tp_node));
		return -EINVAL;
	}

	error = fwnode_property_read_u32_array(tp_node, "azoteq,tx-enable",
					       pins, total_tx);
	if (error) {
		dev_err(&client->dev, "Failed to read %s TX pins: %d\n",
			fwnode_get_name(tp_node), error);
		return error;
	}

	for (i = 0; i < total_tx; i++) {
		if (pins[i] > IQS9150_MAX_TX || pins[i] == IQS9150_RDY_TX ||
		    pins[i] < dev_desc->min_tx) {
			dev_err(&client->dev, "Invalid %s TX pin: %u\n",
				fwnode_get_name(tp_node), pins[i]);
			return -EINVAL;
		}

		for (j = 0; j < total_rx; j++) {
			if (iqs9150_reg(IQS9150_RX_TX_MAP + j) != pins[i])
				continue;

			dev_err(&client->dev, "Conflicting %s TX pin: %u\n",
				fwnode_get_name(tp_node), pins[i]);
			return -EINVAL;
		}

		iqs9150_reg(IQS9150_RX_TX_MAP + total_rx + i) = pins[i];
	}

	iqs9150_reg(IQS9150_TOTAL_RX) = total_rx;
	iqs9150_reg(IQS9150_TOTAL_TX) = total_tx;

	error = iqs9150_parse_props(iqs9150, tp_node, IQS9150_REG_GRP_TP,
				    IQS9150_REG_KEY_SPAN, 0);
	if (error)
		return error;

	for (i = 0; i < ARRAY_SIZE(iqs9150_props); i++) {
		const char *name = iqs9150_props[i].name;
		enum iqs9150_reg_key_id reg_key = iqs9150_props[i].reg_key;
		u16 reg_addr = iqs9150_props[i].reg_addr[IQS9150_REG_GRP_TP];
		int reg_size = iqs9150_props[i].reg_size;
		int val_max = iqs9150_props[i].val_max;
		const char *label = iqs9150_props[i].label ? : name;
		unsigned int *val;
		int count;

		if (!reg_addr || reg_size < sizeof(unsigned int))
			continue;

		count = fwnode_property_count_u32(tp_node, name);
		if (count == -EINVAL) {
			continue;
		} else if (count < 0) {
			dev_err(&client->dev, "Failed to count %s %s: %d\n",
				fwnode_get_name(tp_node), label, error);
			return count;
		} else if (count > IQS9150_NUM_CHANNELS) {
			dev_err(&client->dev, "Invalid number of %s %s\n",
				fwnode_get_name(tp_node), label);
			return -EINVAL;
		}

		val = kcalloc(count, sizeof(*val), GFP_KERNEL);
		if (!val)
			return -ENOMEM;

		error = fwnode_property_read_u32_array(tp_node, name, val,
						       count);
		if (error) {
			dev_err(&client->dev, "Failed to read %s %s: %d\n",
				fwnode_get_name(tp_node), label, error);
			kfree(val);
			return error;
		}

		memset(&iqs9150_reg(reg_addr), 0, reg_size);

		for (j = 0; j < count; j++) {
			if (val[j] > val_max) {
				dev_err(&client->dev, "Invalid %s %s: %u\n",
					fwnode_get_name(tp_node), label, val[j]);
				kfree(val);
				return -EINVAL;
			}

			if (reg_key == IQS9150_REG_KEY_MASK) {
				int row = val[j] / total_rx;
				int col = val[j] % total_rx;
				u16 reg_offs = reg_addr +
					       row * sizeof(unsigned int) +
					       col / BITS_PER_BYTE;
				u8 bit_offs = col % BITS_PER_BYTE;

				iqs9150_reg(reg_offs) |= BIT(bit_offs);
			} else {
				u16 reg_offs = reg_addr + j;

				iqs9150_reg(reg_offs) = val[j];
			}
		}

		kfree(val);
	}

	return 0;
}

static int iqs9150_parse_alp(struct iqs9150_private *iqs9150,
			     struct fwnode_handle *alp_node)
{
	const struct iqs9150_dev_desc *dev_desc = iqs9150->dev_desc;
	struct i2c_client *client = iqs9150->client;
	struct fwnode_handle *sense_node;
	int error, count, i;

	count = fwnode_property_count_u32(alp_node, "azoteq,rx-enable");
	if (count < 0 && count != -EINVAL) {
		dev_err(&client->dev, "Failed to count %s RX pins: %d\n",
			fwnode_get_name(alp_node), count);
		return count;
	} else if (count > dev_desc->num_rx) {
		dev_err(&client->dev, "Invalid number of %s RX pins\n",
			fwnode_get_name(alp_node));
		return -EINVAL;
	} else if (count >= 0) {
		unsigned int pins[IQS9150_NUM_RX];

		error = fwnode_property_read_u32_array(alp_node,
						       "azoteq,rx-enable",
						       pins, count);
		if (error) {
			dev_err(&client->dev, "Failed to read %s RX pins: %d\n",
				fwnode_get_name(alp_node), error);
			return error;
		}

		iqs9150_reg(IQS9150_ALP_SETUP) &= ~GENMASK((IQS9150_NUM_RX - 1) %
							   BITS_PER_BYTE, 0);
		memset(&iqs9150_reg(IQS9150_ALP_RX_ENABLE), 0,
		       (IQS9150_NUM_RX - 1) / BITS_PER_BYTE);

		for (i = 0; i < count; i++) {
			u16 reg_offs = IQS9150_ALP_RX_ENABLE +
				       pins[i] / BITS_PER_BYTE;
			u8 bit_offs = pins[i] % BITS_PER_BYTE;

			if (pins[i] > dev_desc->num_rx - 1) {
				dev_err(&client->dev, "Invalid %s RX pin: %u\n",
					fwnode_get_name(alp_node), pins[i]);
				return -EINVAL;
			}

			iqs9150_reg(reg_offs) |= BIT(bit_offs);
		}
	}

	count = fwnode_property_count_u32(alp_node, "azoteq,tx-enable");
	if (count < 0 && count != -EINVAL) {
		dev_err(&client->dev, "Failed to count %s TX pins: %d\n",
			fwnode_get_name(alp_node), count);
		return count;
	} else if (count > dev_desc->num_tx) {
		dev_err(&client->dev, "Invalid number of %s TX pins\n",
			fwnode_get_name(alp_node));
		return -EINVAL;
	} else if (count >= 0) {
		unsigned int pins[IQS9150_NUM_TX];

		error = fwnode_property_read_u32_array(alp_node,
						       "azoteq,tx-enable",
						       pins, count);
		if (error) {
			dev_err(&client->dev, "Failed to read %s TX pins: %d\n",
				fwnode_get_name(alp_node), error);
			return error;
		}

		memset(&iqs9150_reg(IQS9150_ALP_TX_ENABLE), 0,
		       IQS9150_MAX_TX / BITS_PER_BYTE + 1);

		for (i = 0; i < count; i++) {
			u16 reg_offs = IQS9150_ALP_TX_ENABLE +
				       pins[i] / BITS_PER_BYTE;
			u8 bit_offs = pins[i] % BITS_PER_BYTE;

			if (pins[i] > IQS9150_MAX_TX ||
			    pins[i] == IQS9150_RDY_TX ||
			    pins[i] < dev_desc->min_tx) {
				dev_err(&client->dev, "Invalid %s TX pin: %u\n",
					fwnode_get_name(alp_node), pins[i]);
				return -EINVAL;
			}

			iqs9150_reg(reg_offs) |= BIT(bit_offs);
		}
	}

	fwnode_for_each_child_node(alp_node, sense_node) {
		unsigned int val;

		error = fwnode_property_read_u32(sense_node, "reg", &val);
		if (error) {
			dev_err(&client->dev, "Failed to read %s offset: %d\n",
				fwnode_get_name(sense_node), error);
			fwnode_handle_put(sense_node);
			return error;
		}

		error = iqs9150_parse_props(iqs9150, sense_node,
					    IQS9150_REG_GRP_ALP,
					    IQS9150_REG_KEY_SPAN, val);
		if (error) {
			fwnode_handle_put(sense_node);
			return error;
		}
	}

	iqs9150_reg(IQS9150_ALP_SETUP) |= IQS9150_ALP_SETUP_ENABLE;

	return 0;
}

static int iqs9150_parse_sys(struct iqs9150_private *iqs9150,
			     struct fwnode_handle *sys_node)
{
	struct i2c_client *client = iqs9150->client;
	int error, count, i;

	put_unaligned_le16(IQS9150_COMMS_ERROR, iqs9150->eng_buf);

	count = fwnode_property_count_u32(sys_node, "azoteq,eng-settings");
	if (count < 0 && count != -EINVAL) {
		dev_err(&client->dev,
			"Failed to count %s engineering settings: %d\n",
			fwnode_get_name(sys_node), count);
		return count;
	} else if (count != IQS9150_ENG_BUF_LEN && count != -EINVAL) {
		dev_err(&client->dev,
			"Invalid number of %s engineering settings\n",
			fwnode_get_name(sys_node));
		return -EINVAL;
	} else if (count == IQS9150_ENG_BUF_LEN) {
		unsigned int val[IQS9150_ENG_BUF_LEN];

		error = fwnode_property_read_u32_array(sys_node,
						       "azoteq,eng-settings",
						       val, count);
		if (error) {
			dev_err(&client->dev,
				"Failed to read %s engineering settings: %d\n",
				fwnode_get_name(sys_node), error);
			return error;
		}

		for (i = 0; i < count; i++) {
			if (val[i] > U8_MAX) {
				dev_err(&client->dev,
					"Invalid %s engineering setting: %u\n",
					fwnode_get_name(sys_node), val[i]);
				return -EINVAL;
			}

			iqs9150->eng_buf[i] = val[i];
		}

		if (get_unaligned_le16(iqs9150->eng_buf) == IQS9150_COMMS_ERROR) {
			dev_err(&client->dev,
				"Invalid %s engineering settings\n",
				fwnode_get_name(sys_node));
			return -EINVAL;
		}
	}

	return 0;
}

static int (*iqs9150_parse_extra[IQS9150_NUM_REG_GRPS])
				(struct iqs9150_private *iqs9150,
				 struct fwnode_handle *reg_grp_node) = {
	[IQS9150_REG_GRP_TP] = iqs9150_parse_tp,
	[IQS9150_REG_GRP_ALP] = iqs9150_parse_alp,
	[IQS9150_REG_GRP_SYS] = iqs9150_parse_sys,
};

static int iqs9150_parse_reg_grp(struct iqs9150_private *iqs9150,
				 struct fwnode_handle *reg_grp_node,
				 enum iqs9150_reg_grp_id reg_grp)
{
	u16 enable_addr = iqs9150_reg_grps[reg_grp].enable_addr, config;
	int error, i;

	error = iqs9150_parse_props(iqs9150, reg_grp_node, reg_grp,
				    IQS9150_REG_KEY_NONE, 0);
	if (error)
		return error;

	if (iqs9150_parse_extra[reg_grp]) {
		error = iqs9150_parse_extra[reg_grp](iqs9150, reg_grp_node);
		if (error)
			return error;
	}

	config = iqs9150_get_word(IQS9150_CONFIG);

	for (i = 0; i < ARRAY_SIZE(iqs9150_kp_events); i++) {
		const char *event_name = iqs9150_kp_events[i].name;
		u16 enable_mask = iqs9150_kp_events[i].enable_mask;
		struct fwnode_handle *event_node;

		if (iqs9150_kp_events[i].reg_grp != reg_grp)
			continue;

		if (event_name)
			event_node = fwnode_get_named_child_node(reg_grp_node,
								 event_name);
		else
			event_node = fwnode_handle_get(reg_grp_node);

		if (!event_node)
			continue;

		error = iqs9150_parse_event(iqs9150, event_node,
					    iqs9150_kp_events[i].reg_grp,
					    iqs9150_kp_events[i].reg_key, i);
		fwnode_handle_put(event_node);
		if (error)
			return error;

		if (!iqs9150->kp_type[i])
			continue;

		if (enable_addr)
			iqs9150_put_word(enable_addr,
					 iqs9150_get_word(enable_addr) |
					 enable_mask);

		config |= iqs9150_reg_grps[reg_grp].event_mask;
	}

	iqs9150_put_word(IQS9150_CONFIG, config);

	return 0;
}

static int iqs9150_register_kp(struct iqs9150_private *iqs9150)
{
	const struct iqs9150_dev_desc *dev_desc = iqs9150->dev_desc;
	struct input_dev *kp_idev = iqs9150->kp_idev;
	struct i2c_client *client = iqs9150->client;
	int error, i;

	for (i = 0; i < ARRAY_SIZE(iqs9150_kp_events); i++)
		if (iqs9150->kp_type[i])
			break;

	if (i == ARRAY_SIZE(iqs9150_kp_events))
		return 0;

	kp_idev = devm_input_allocate_device(&client->dev);
	if (!kp_idev)
		return -ENOMEM;

	iqs9150->kp_idev = kp_idev;

	kp_idev->name = dev_desc->kp_name;
	kp_idev->id.bustype = BUS_I2C;

	for (i = 0; i < ARRAY_SIZE(iqs9150_kp_events); i++)
		if (iqs9150->kp_type[i])
			input_set_capability(iqs9150->kp_idev,
					     iqs9150->kp_type[i],
					     iqs9150->kp_code[i]);

	error = input_register_device(kp_idev);
	if (error)
		dev_err(&client->dev, "Failed to register %s: %d\n",
			kp_idev->name, error);

	return error;
}

static int iqs9150_register_tp(struct iqs9150_private *iqs9150)
{
	const struct iqs9150_dev_desc *dev_desc = iqs9150->dev_desc;
	struct touchscreen_properties *prop = &iqs9150->prop;
	struct input_dev *tp_idev = iqs9150->tp_idev;
	struct i2c_client *client = iqs9150->client;
	int error;

	tp_idev = devm_input_allocate_device(&client->dev);
	if (!tp_idev)
		return -ENOMEM;

	iqs9150->tp_idev = tp_idev;

	tp_idev->name = dev_desc->tp_name;
	tp_idev->id.bustype = BUS_I2C;

	input_set_abs_params(tp_idev, ABS_MT_POSITION_X,
			     0, iqs9150_get_word(IQS9150_X_RES), 0, 0);

	input_set_abs_params(tp_idev, ABS_MT_POSITION_Y,
			     0, iqs9150_get_word(IQS9150_Y_RES), 0, 0);

	input_set_abs_params(tp_idev, ABS_MT_PRESSURE, 0, U16_MAX, 0, 0);

	touchscreen_parse_properties(tp_idev, true, prop);

	/*
	 * The device reserves 0xFFFF for coordinates that correspond to slots
	 * which are not in a state of touch.
	 */
	if (prop->max_x >= U16_MAX || prop->max_y >= U16_MAX) {
		dev_err(&client->dev, "Invalid trackpad size: %u*%u\n",
			prop->max_x, prop->max_y);
		return -EINVAL;
	}

	iqs9150_put_word(IQS9150_X_RES, prop->max_x);
	iqs9150_put_word(IQS9150_Y_RES, prop->max_y);

	error = input_mt_init_slots(tp_idev, iqs9150_reg(IQS9150_NUM_CONTACTS),
				    INPUT_MT_DIRECT);
	if (error) {
		dev_err(&client->dev, "Failed to initialize slots: %d\n",
			error);
		return error;
	}

	error = input_register_device(tp_idev);
	if (error)
		dev_err(&client->dev, "Failed to register %s: %d\n",
			tp_idev->name, error);

	return error;
}

static int iqs9150_report(struct iqs9150_private *iqs9150)
{
	struct iqs9150_status *status = &iqs9150->status;
	struct i2c_client *client = iqs9150->client;
	u16 gesture_x, gesture_y, info;
	int error, i;

	error = iqs9150_read_burst(iqs9150, IQS9150_STATUS, status,
				   sizeof(*status));
	if (error)
		return error;

	gesture_x = le16_to_cpu(status->gesture_x);
	gesture_y = le16_to_cpu(status->gesture_y);

	info = le16_to_cpu(status->flags[2]);

	if (info & IQS9150_INFO_SHOW_RESET) {
		enum iqs9150_comms_mode comms_mode = iqs9150->comms_mode;

		dev_err(&client->dev, "Unexpected device reset\n");

		/*
		 * The device may or may not expect forced communication after
		 * it exits hardware reset, so the corresponding state machine
		 * must be reset as well.
		 */
		iqs9150->comms_mode = IQS9150_COMMS_MODE_WAIT;

		error = iqs9150_write_word(iqs9150, IQS9150_CONFIG,
					   iqs9150_get_word(IQS9150_CONFIG));
		if (error)
			return error;

		iqs9150->comms_mode = comms_mode;

		return iqs9150_init_device(iqs9150);
	}

	if (info & IQS9150_INFO_TP_ATI_ERROR) {
		dev_err(&client->dev, "Unexpected %s ATI error\n",
			iqs9150_reg_grps[IQS9150_REG_GRP_TP].name);
	} else if (info & IQS9150_INFO_TP_ATI_AGAIN) {
		dev_dbg(&client->dev, "New %s ATI occurrence\n",
			iqs9150_reg_grps[IQS9150_REG_GRP_TP].name);
	} else {
		for (i = 0; i < iqs9150_reg(IQS9150_NUM_CONTACTS); i++) {
			u16 abs_x = le16_to_cpu(status->touch_data[i].abs_x);
			u16 abs_y = le16_to_cpu(status->touch_data[i].abs_y);
			u16 pressure = le16_to_cpu(status->touch_data[i].pressure);

			input_mt_slot(iqs9150->tp_idev, i);
			if (input_mt_report_slot_state(iqs9150->tp_idev,
						       MT_TOOL_FINGER,
						       pressure)) {
				touchscreen_report_pos(iqs9150->tp_idev,
						       &iqs9150->prop,
						       abs_x, abs_y, true);
				input_report_abs(iqs9150->tp_idev,
						 ABS_MT_PRESSURE, pressure);
			}
		}

		input_mt_sync_frame(iqs9150->tp_idev);
		input_sync(iqs9150->tp_idev);
	}

	if (info & IQS9150_INFO_ALP_ATI_ERROR)
		dev_err(&client->dev, "Unexpected %s ATI error\n",
			iqs9150_reg_grps[IQS9150_REG_GRP_ALP].name);
	else if (info & IQS9150_INFO_ALP_ATI_AGAIN)
		dev_dbg(&client->dev, "New %s ATI occurrence\n",
			iqs9150_reg_grps[IQS9150_REG_GRP_ALP].name);

	if (iqs9150->kp_idev) {
		bool flush = false;

		for (i = 0; i < ARRAY_SIZE(iqs9150_kp_events); i++) {
			enum iqs9150_reg_grp_id reg_grp =
			     iqs9150_kp_events[i].reg_grp;
			enum iqs9150_reg_key_id reg_key =
			     iqs9150_kp_events[i].reg_key;
			int status_offs = iqs9150_reg_grps[reg_grp].status_offs;
			u16 status_mask = iqs9150_kp_events[i].status_mask;
			u16 travel_mask = iqs9150_kp_events[i].travel_mask;
			u16 flags = le16_to_cpu(status->flags[status_offs]);

			if (!iqs9150->kp_type[i])
				continue;

			if (info & iqs9150_reg_grps[reg_grp].ati_mask)
				continue;

			if (reg_grp == IQS9150_REG_GRP_ALP &&
			    (info & IQS9150_INFO_CHARGE_MODE) <
				    IQS9150_INFO_CHARGE_MODE_LP1)
				continue;

			if ((reg_key == IQS9150_REG_KEY_SCROLL_X &&
			    (gesture_x & BIT(15)) ^ travel_mask) ||
			    (reg_key == IQS9150_REG_KEY_SCROLL_Y &&
			    (gesture_y & BIT(15)) ^ travel_mask))
				continue;

			input_event(iqs9150->kp_idev, iqs9150->kp_type[i],
				    iqs9150->kp_code[i], !!(flags & status_mask));

			if (reg_key != IQS9150_REG_KEY_HOLD &&
			    reg_key != IQS9150_REG_KEY_PALM)
				flush |= flags & status_mask;
		}

		/*
		 * Hold and palm gestures persist while the contact remains in
		 * place; all others are momentary and hence are followed by a
		 * complementary release event.
		 */
		if (flush) {
			input_sync(iqs9150->kp_idev);

			for (i = 0; i < ARRAY_SIZE(iqs9150_kp_events); i++) {
				enum iqs9150_reg_key_id reg_key =
				     iqs9150_kp_events[i].reg_key;

				if (reg_key != IQS9150_REG_KEY_HOLD &&
				    reg_key != IQS9150_REG_KEY_PALM)
					input_event(iqs9150->kp_idev,
						    iqs9150->kp_type[i],
						    iqs9150->kp_code[i], 0);
			}
		}

		input_sync(iqs9150->kp_idev);
	}

	return 0;
}

static irqreturn_t iqs9150_irq(int irq, void *context)
{
	struct iqs9150_private *iqs9150 = context;

	return iqs9150_report(iqs9150) ? IRQ_NONE : IRQ_HANDLED;
}

static int iqs9150_suspend(struct device *dev)
{
	struct iqs9150_private *iqs9150 = dev_get_drvdata(dev);
	int error;

	if (device_may_wakeup(dev))
		return 0;

	/*
	 * I2C communication prompts the device to assert its RDY pin if it is
	 * not already asserted. As such, the interrupt must be disabled so as
	 * to prevent reentrant interrupts.
	 */
	disable_irq(gpiod_to_irq(iqs9150->irq_gpio));

	error = iqs9150_write_word(iqs9150, IQS9150_CONTROL,
				   IQS9150_CONTROL_SUSPEND);

	enable_irq(gpiod_to_irq(iqs9150->irq_gpio));

	return error;
}

static int iqs9150_resume(struct device *dev)
{
	struct iqs9150_private *iqs9150 = dev_get_drvdata(dev);
	int error;

	if (device_may_wakeup(dev))
		return 0;

	disable_irq(gpiod_to_irq(iqs9150->irq_gpio));

	error = iqs9150_write_word(iqs9150, IQS9150_CONTROL, 0);

	enable_irq(gpiod_to_irq(iqs9150->irq_gpio));

	return error;
}

static DEFINE_SIMPLE_DEV_PM_OPS(iqs9150_pm, iqs9150_suspend, iqs9150_resume);

static ssize_t fw_info_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct iqs9150_private *iqs9150 = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u.%u.%u.%u:%u.%u\n",
			 le16_to_cpu(iqs9150->ver_info.prod_num),
			 le32_to_cpu(iqs9150->ver_info.patch),
			 le16_to_cpu(iqs9150->ver_info.major),
			 le16_to_cpu(iqs9150->ver_info.minor),
			 iqs9150_reg(IQS9150_SETTINGS_MAJOR),
			 iqs9150_reg(IQS9150_SETTINGS_MINOR));
}

static DEVICE_ATTR_RO(fw_info);

static struct attribute *iqs9150_attrs[] = {
	&dev_attr_fw_info.attr,
	NULL
};
ATTRIBUTE_GROUPS(iqs9150);

static const struct of_device_id iqs9150_of_match[] = {
	{
		.compatible = "azoteq,iqs9150",
		.data = &iqs9150_devs[IQS9150],
	},
	{
		.compatible = "azoteq,iqs9151",
		.data = &iqs9150_devs[IQS9151],
	},
	{ }
};
MODULE_DEVICE_TABLE(of, iqs9150_of_match);

static int iqs9150_probe(struct i2c_client *client)
{
	struct iqs9150_private *iqs9150;
	enum iqs9150_reg_grp_id reg_grp;
	unsigned long irq_flags;
	int error, irq;

	iqs9150 = devm_kzalloc(&client->dev, sizeof(*iqs9150), GFP_KERNEL);
	if (!iqs9150)
		return -ENOMEM;

	i2c_set_clientdata(client, iqs9150);
	iqs9150->client = client;

	iqs9150->dev_desc = device_get_match_data(&client->dev);
	if (!iqs9150->dev_desc)
		return -ENODEV;

	/*
	 * The RDY pin behaves as an interrupt, but must also be polled ahead
	 * of unsolicited I2C communication. As such, it is first opened as a
	 * GPIO and then passed to gpiod_to_irq() to register the interrupt.
	 */
	iqs9150->irq_gpio = devm_gpiod_get(&client->dev, "irq", GPIOD_IN);
	if (IS_ERR(iqs9150->irq_gpio)) {
		error = PTR_ERR(iqs9150->irq_gpio);
		dev_err(&client->dev, "Failed to request IRQ GPIO: %d\n",
			error);
		return error;
	}

	iqs9150->reset_gpio = devm_gpiod_get_optional(&client->dev, "reset",
						      GPIOD_OUT_HIGH);
	if (IS_ERR(iqs9150->reset_gpio)) {
		error = PTR_ERR(iqs9150->reset_gpio);
		dev_err(&client->dev, "Failed to request reset GPIO: %d\n",
			error);
		return error;
	}

	error = devm_regulator_get_enable(&client->dev, "vdd");
	if (error) {
		dev_err(&client->dev, "Failed to request VDD regulator: %d\n",
			error);
		return error;
	}

	error = devm_add_action_or_reset(&client->dev, iqs9150_hold_reset,
					 iqs9150);
	if (error)
		return error;

	error = iqs9150_start_comms(iqs9150);
	if (error)
		return error;

	for (reg_grp = 0; reg_grp < IQS9150_NUM_REG_GRPS; reg_grp++) {
		const char *reg_grp_name = iqs9150_reg_grps[reg_grp].name;
		struct fwnode_handle *reg_grp_node;

		if (reg_grp_name)
			reg_grp_node = device_get_named_child_node(&client->dev,
								   reg_grp_name);
		else
			reg_grp_node = fwnode_handle_get(dev_fwnode(&client->dev));

		if (!reg_grp_node)
			continue;

		error = iqs9150_parse_reg_grp(iqs9150, reg_grp_node, reg_grp);
		fwnode_handle_put(reg_grp_node);
		if (error)
			return error;
	}

	error = iqs9150_register_tp(iqs9150);
	if (error)
		return error;

	error = iqs9150_register_kp(iqs9150);
	if (error)
		return error;

	error = iqs9150_init_device(iqs9150);
	if (error)
		return error;

	irq = gpiod_to_irq(iqs9150->irq_gpio);
	if (irq < 0)
		return irq;

	irq_flags = gpiod_is_active_low(iqs9150->irq_gpio) ? IRQF_TRIGGER_LOW
							   : IRQF_TRIGGER_HIGH;
	irq_flags |= IRQF_ONESHOT;

	error = devm_request_threaded_irq(&client->dev, irq, NULL, iqs9150_irq,
					  irq_flags, client->name, iqs9150);
	if (error)
		dev_err(&client->dev, "Failed to request IRQ: %d\n", error);

	return error;
}

static struct i2c_driver iqs9150_i2c_driver = {
	.probe = iqs9150_probe,
	.driver = {
		.name = "iqs9150",
		.of_match_table = iqs9150_of_match,
		.dev_groups = iqs9150_groups,
		.pm = pm_sleep_ptr(&iqs9150_pm),
	},
};
module_i2c_driver(iqs9150_i2c_driver);

MODULE_AUTHOR("Jeff LaBundy <jeff@labundy.com>");
MODULE_DESCRIPTION("Azoteq IQS9150/9151 Trackpad Controller");
MODULE_LICENSE("GPL");

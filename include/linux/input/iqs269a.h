/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Azoteq IQS269A Capacitive Touch Controller
 *
 * Copyright (C) 2020 Jeff LaBundy <jeff@labundy.com>
 */

#ifndef __LINUX_INPUT_IQS269A_H
#define __LINUX_INPUT_IQS269A_H

enum iqs269_sync_action {
	IQS269_SYNC_STOP,
	IQS269_SYNC_RESTART,
	IQS269_SYNC_TRIGGER,
};

extern int iqs269_sync(enum iqs269_sync_action action);

#endif /* __LINUX_INPUT_IQS269A_H */

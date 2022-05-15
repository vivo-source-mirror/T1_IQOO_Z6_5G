/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Generic GPIO card-detect helper header
 *
 * Copyright (C) 2011, Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 */

#ifndef MMC_SLOT_GPIO_H
#define MMC_SLOT_GPIO_H

#include <linux/types.h>
#include <linux/irqreturn.h>

struct mmc_host;

int mmc_gpio_get_ro(struct mmc_host *host);
int mmc_gpio_get_cd(struct mmc_host *host);
int mmc_gpiod_request_cd(struct mmc_host *host, const char *con_id,
			 unsigned int idx, bool override_active_level,
			 unsigned int debounce, bool *gpio_invert);
int mmc_gpiod_request_ro(struct mmc_host *host, const char *con_id,
			 unsigned int idx,
			 unsigned int debounce, bool *gpio_invert);
void mmc_gpio_set_cd_isr(struct mmc_host *host,
			 irqreturn_t (*isr)(int irq, void *dev_id));
int mmc_gpio_set_cd_wake(struct mmc_host *host, bool on);
void mmc_gpiod_request_cd_irq(struct mmc_host *host);
bool mmc_can_gpio_cd(struct mmc_host *host);
bool mmc_can_gpio_ro(struct mmc_host *host);

#if IS_ENABLED(CONFIG_MMC_NON_GKI)
int mmc_gpio_get_int_cnt_enable(struct mmc_host *host);
void mmc_gpio_set_int_cnt_enable(struct mmc_host *host, int int_cnt_enable);
u64 mmc_gpio_get_int_cnt(struct mmc_host *host);
void mmc_gpio_set_int_cnt(struct mmc_host *host, u64 int_cnt);
void mmc_gpio_update_status(struct mmc_host *host);
#endif

#endif

/* Copyright (c) 2017-2019, The Linux Foundation. All rights reserved.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 and
* only version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

/*
 * File: miniisp_customer_define.c
 * Description: Mini ISP sample codes
 *
 *
 *  2017/03/14;LouisWang; Initial version
 */

/******Include File******/
/* Linux headers*/
#include <linux/delay.h>
#include  <linux/of_gpio.h>

#include "include/miniisp_customer_define.h"
#include "include/miniisp.h"
#include "include/miniisp_ctrl.h"
#include "include/altek_statefsm.h"

#define MINI_ISP_LOG_TAG "[miniisp_customer_define]"


extern void mini_isp_poweron(void)
{
	errcode ret = 0;
	void *devdata;
	struct misp_global_variable *dev_global_variable;
	struct altek_statefsm *fsm;

	fsm = get_mini_isp_fsm();
	dev_global_variable = get_mini_isp_global_variable();
	/*reset mini-isp keep low for at least 200us, release to high for 20ms*/
	misp_err("[miniISP]mini_isp_poweron");
	dev_global_variable->before_booting = 1;
	dev_global_variable->be_set_to_bypass = 0;

	devdata = get_mini_isp_intf(MINIISP_I2C_SLAVE);

	/*state check*/
	ret = altek_statefsmispdrv_open(fsm, ((void *)devdata));
	if (ret != 0){
		misp_err("%s err, %x", __func__, ret);
		return;
	}

	dev_global_variable->now_state = MINI_ISP_POWER_ON;
}
EXPORT_SYMBOL(mini_isp_poweron);

extern void mini_isp_poweroff(void)
{
	void *devdata;
	struct misp_global_variable *dev_global_variable;
	int ret = 0;
	struct altek_statefsm *fsm;

	fsm = get_mini_isp_fsm();
	misp_err("[miniISP]mini_isp_poweroff");
	dev_global_variable = get_mini_isp_global_variable();

	devdata = get_mini_isp_intf(MINIISP_I2C_SLAVE);

	/*state check*/
	ret = altek_statefsmispdrv_close(fsm, devdata);
	if (ret != 0) {
		misp_err("%s err, %x", __func__, ret);
		return;
	}

	dev_global_variable->now_state = MINI_ISP_POWER_OFF;
	misp_info("%s - X", __func__);
}
EXPORT_SYMBOL(mini_isp_poweroff);

extern int mini_isp_gpio_init(struct device *dev,
			struct misp_data *drv_data,
			struct misp_global_variable *drv_global_variable)
{
	int ret = 0;

	if (VCC1_GPIO != NULL) {
		drv_global_variable->vcc1_gpio =
			of_get_named_gpio(dev->of_node, VCC1_GPIO, 0);
		misp_info("%s - probe vcc1-gpios = %d", __func__,
			drv_global_variable->vcc1_gpio);

		ret = devm_gpio_request(dev,
			drv_global_variable->vcc1_gpio, VCC1_GPIO);
		if (ret) {
			misp_err("%s -step 4. request vcc1-gpio error",
				__func__);
			goto err_gpio1_config;
		}

		gpio_direction_output(drv_global_variable->vcc1_gpio, 1);
		msleep(20);
		gpio_set_value(drv_global_variable->vcc1_gpio, 1);
		msleep(20);
	}

	if (VCC2_GPIO != NULL) {
		drv_global_variable->vcc2_gpio = of_get_named_gpio(
			dev->of_node, VCC2_GPIO, 0);
		misp_info("%s - probe vcc2-gpios = %d", __func__,
			drv_global_variable->vcc2_gpio);

		ret = devm_gpio_request(dev,
			drv_global_variable->vcc2_gpio, VCC2_GPIO);
		if (ret) {
			misp_err("%s -step 4. request vcc2-gpios error",
				__func__);
			goto err_gpio2_config;
		}

		gpio_direction_output(drv_global_variable->vcc2_gpio, 1);
		msleep(20);
		gpio_set_value(drv_global_variable->vcc2_gpio, 1);
		msleep(20);
	}

	if (VCC3_GPIO != NULL) {
		drv_global_variable->vcc3_gpio = of_get_named_gpio(
			dev->of_node, VCC3_GPIO, 0);
		misp_err("%s - probe vcc3-gpios = %d", __func__,
					drv_global_variable->vcc3_gpio);

		ret = devm_gpio_request(dev,
			drv_global_variable->vcc3_gpio, VCC3_GPIO);
		if (ret) {
			misp_err("%s -step 4. request vcc3-gpio error",
				__func__);
			goto err_gpio_config;
		}

		gpio_direction_output(drv_global_variable->vcc3_gpio, 1);
		gpio_set_value(drv_global_variable->vcc3_gpio, 1);
		msleep(20);

	}
	if (ISP_CLK != NULL) {
		drv_global_variable->isp_clk = devm_clk_get(dev,
						ISP_CLK);
		misp_err("clk_ptr = %p", drv_global_variable->isp_clk);
		ret = clk_set_rate(drv_global_variable->isp_clk, 19200000L);
		if (ret < 0)
			misp_err("clk_set_rate failed, not fatal\n");

		misp_err("clk_get_rate %ld\n", clk_get_rate(
					drv_global_variable->isp_clk));
		ret = clk_prepare_enable(drv_global_variable->isp_clk);
		if (ret < 0) {
			misp_err("clk_prepare_enable failed\n");
			goto err_clk_config;
		}
		msleep(20);
	}

	if (RESET_GPIO != NULL) {
		drv_global_variable->reset_gpio =
			of_get_named_gpio(dev->of_node, RESET_GPIO, 0);
		misp_info("%s - probe reset_gpio = %d", __func__,
			drv_global_variable->reset_gpio);

		ret = devm_gpio_request(dev,
			drv_global_variable->reset_gpio, RESET_GPIO);
		if (ret) {
			misp_err("%s -step 4. request reset gpio error",
				__func__);
			goto err_reset_config;
		}

		gpio_direction_output(drv_global_variable->reset_gpio, 0);
		gpio_set_value(drv_global_variable->reset_gpio, 0);
		msleep(20);

	}

	if (IRQ_GPIO != NULL) {

		drv_global_variable->irq_gpio = of_get_named_gpio(dev->of_node, IRQ_GPIO, 0);

		misp_info("%s - probe irq_gpio = %d", __func__, drv_global_variable->irq_gpio);

		ret = gpio_request(drv_global_variable->irq_gpio, IRQ_GPIO);
		if (ret) {
			misp_err("%s -step 4. request irq gpio error",
				__func__);
			goto err_irq_config;
		}
		gpio_direction_input(drv_global_variable->irq_gpio);

		drv_global_variable->irq_num = gpio_to_irq(drv_global_variable->irq_gpio);

		misp_err("%s - probe spi->irq = %d %d ",
			__func__, drv_global_variable->irq_num,
			gpio_to_irq(drv_global_variable->irq_gpio));

		ret = request_threaded_irq(drv_global_variable->irq_num, NULL, mini_isp_irq,
		IRQF_TRIGGER_HIGH | IRQF_ONESHOT, "mini_isp", drv_data);

		if (ret) {
			misp_err("%s - step4. probe - request irq error",
				__func__);
			goto err_dev_attr;
		}
		misp_info("%s - step4 done. irq number:%d", __func__,
			drv_global_variable->irq_num);

		free_irq(drv_global_variable->irq_num, drv_data);
	}

	/*step 5:other additional config*/

	misp_info("%s - step5 done", __func__);

	if (RESET_GPIO != NULL) {
		gpio_direction_output(drv_global_variable->reset_gpio, 1);
		gpio_set_value(drv_global_variable->reset_gpio, 1);
		msleep(20);
	}

	return ret;

err_dev_attr:
	free_irq(drv_global_variable->irq_num, drv_data);
err_irq_config:
	if (IRQ_GPIO != NULL)
		gpio_free(drv_global_variable->irq_gpio);

err_reset_config:
	if (RESET_GPIO != NULL)
		gpio_free(drv_global_variable->reset_gpio);

err_clk_config:
	if (ISP_CLK != NULL)
		clk_disable_unprepare(drv_global_variable->isp_clk);

err_gpio_config:
	if (VCC2_GPIO != NULL)
		gpio_free(drv_global_variable->vcc2_gpio);
err_gpio2_config:
	if (VCC1_GPIO != NULL)
		gpio_free(drv_global_variable->vcc1_gpio);

err_gpio1_config:

	return ret;
}


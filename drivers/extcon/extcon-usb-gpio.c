/**
 * drivers/extcon/extcon-usb-gpio.c - USB GPIO extcon driver
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com
 * Author: Roger Quadros <rogerq@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/extcon.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/acpi.h>

#include <linux/hrtimer.h>

#define USB_GPIO_DEBOUNCE_MS	20	/* ms */

struct usb_extcon_info {
	struct device *dev;
	struct extcon_dev *edev;

	struct gpio_desc *id_gpiod;
	struct gpio_desc *vbus_gpiod;
	int id_irq;
	int vbus_irq;

	unsigned long debounce_jiffies;
	struct delayed_work wq_detcable;

	/* Button Support */
	struct delayed_work       debounce_timer;
	bool            enable_button_mode;
	struct pinctrl      *pinctrl;
	struct pinctrl_state    *button_active;
};

static const unsigned int usb_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

static void hrtimer_event(struct work_struct *work)
{
	struct usb_extcon_info *info = container_of(to_delayed_work(work),
						    struct usb_extcon_info,
						    debounce_timer);
	int button;
	int mode;

	button = gpiod_get_value_cansleep(info->vbus_gpiod);

	if(!button){
	        /* service button press */
		mode = extcon_get_state(info->edev, EXTCON_USB_HOST);
		if(!mode){
			/* we are in device mode, switch to host */
			extcon_set_state_sync(info->edev, EXTCON_USB, false);
			extcon_set_state_sync(info->edev, EXTCON_USB_HOST, true);
		} else {
			/* we are in host mode, switch to device */
			pr_err("switching to device mode \n");
			extcon_set_state_sync(info->edev, EXTCON_USB_HOST, false);
			extcon_set_state_sync(info->edev, EXTCON_USB, true);
		}
	} else {
		/* button was not pressed long enough */
		pr_info("mode switch button wasn't pressed long enough \n");
	}
}

static irqreturn_t gpio_usbdetect_button_irq(int irq, void *data)
{
	struct usb_extcon_info *info = data;

	/* Kick off debounce timer for a valid button press */
	queue_delayed_work(system_power_efficient_wq, &info->debounce_timer,
			   msecs_to_jiffies(250));

	return IRQ_HANDLED;
}

/*
 * "USB" = VBUS and "USB-HOST" = !ID, so we have:
 * Both "USB" and "USB-HOST" can't be set as active at the
 * same time so if "USB-HOST" is active (i.e. ID is 0)  we keep "USB" inactive
 * even if VBUS is on.
 *
 *  State              |    ID   |   VBUS
 * ----------------------------------------
 *  [1] USB            |    H    |    H
 *  [2] none           |    H    |    L
 *  [3] USB-HOST       |    L    |    H
 *  [4] USB-HOST       |    L    |    L
 *
 * In case we have only one of these signals:
 * - VBUS only - we want to distinguish between [1] and [2], so ID is always 1.
 * - ID only - we want to distinguish between [1] and [4], so VBUS = ID.
 */
static void usb_extcon_detect_cable(struct work_struct *work)
{
	int id, vbus;
	struct usb_extcon_info *info = container_of(to_delayed_work(work),
						    struct usb_extcon_info,
						    wq_detcable);

	/* check ID and VBUS and update cable state */
	id = info->id_gpiod ?
		gpiod_get_value_cansleep(info->id_gpiod) : 1;
	vbus = info->vbus_gpiod ?
		gpiod_get_value_cansleep(info->vbus_gpiod) : id;

	/* at first we clean states which are no longer active */
	if (id)
		extcon_set_state_sync(info->edev, EXTCON_USB_HOST, false);
	if (!vbus)
		extcon_set_state_sync(info->edev, EXTCON_USB, false);

	if (!id) {
		extcon_set_state_sync(info->edev, EXTCON_USB_HOST, true);
	} else {
		if (vbus)
			extcon_set_state_sync(info->edev, EXTCON_USB, true);
	}
}

static irqreturn_t usb_irq_handler(int irq, void *dev_id)
{
	struct usb_extcon_info *info = dev_id;

	queue_delayed_work(system_power_efficient_wq, &info->wq_detcable,
			   info->debounce_jiffies);

	return IRQ_HANDLED;
}

static int gpio_usbdetect_pinctrl_dt(struct usb_extcon_info *usb)
{
	int ret = 0;

	usb->pinctrl = devm_pinctrl_get(usb->dev);
	if (IS_ERR(usb->pinctrl)) {
		ret = PTR_ERR(usb->pinctrl);
		if (ret == -EPROBE_DEFER)
			return ret;
	dev_err(usb->dev, "pinctrl not available\n");
	ret = -ENODEV;
	goto exit;
	}

	usb->button_active = pinctrl_lookup_state(usb->pinctrl,
				"button_active");
	if (IS_ERR(usb->button_active)){
		dev_err(usb->dev, "pinctrl lookup button_active failed\n");
		ret = -ENODEV;
	}

	ret = pinctrl_select_state(usb->pinctrl,
			usb->button_active);
	if (ret < 0){
		dev_err(usb->dev, "unable to set active pinctrl state\n");
	}

exit:
	return ret;
}

static int usb_extcon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct usb_extcon_info *info;
	int ret;

	if (!np && !ACPI_HANDLE(dev))
		return -EINVAL;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->enable_button_mode = of_property_read_bool(pdev->dev.of_node,
			"qcom,enable-button-mode");
	info->dev = dev;
	if(info->enable_button_mode){
		INIT_DELAYED_WORK(&info->debounce_timer, hrtimer_event);
		ret = gpio_usbdetect_pinctrl_dt(info);
		if(ret< 0){
			if(ret == -EPROBE_DEFER)
				return ret;
			dev_err(&pdev->dev, "Pinctrl settings may not be complete, button might not work\n");
		}
	}

	info->id_gpiod = devm_gpiod_get_optional(&pdev->dev, "id", GPIOD_IN);
	info->vbus_gpiod = devm_gpiod_get_optional(&pdev->dev, "vbus",
						   GPIOD_IN);

	if (!info->id_gpiod && !info->vbus_gpiod) {
		dev_err(dev, "failed to get gpios\n");
		return -ENODEV;
	}

	if (IS_ERR(info->id_gpiod))
		return PTR_ERR(info->id_gpiod);

	if (IS_ERR(info->vbus_gpiod))
		return PTR_ERR(info->vbus_gpiod);

	info->edev = devm_extcon_dev_allocate(dev, usb_extcon_cable);
	if (IS_ERR(info->edev)) {
		dev_err(dev, "failed to allocate extcon device\n");
		return -ENOMEM;
	}

	ret = devm_extcon_dev_register(dev, info->edev);
	if (ret < 0) {
		dev_err(dev, "failed to register extcon device\n");
		return ret;
	}

	if (info->id_gpiod)
		ret = gpiod_set_debounce(info->id_gpiod,
					 USB_GPIO_DEBOUNCE_MS * 1000);
	if (!ret && info->vbus_gpiod)
		ret = gpiod_set_debounce(info->vbus_gpiod,
					 USB_GPIO_DEBOUNCE_MS * 1000);

	if (ret < 0)
		info->debounce_jiffies = msecs_to_jiffies(USB_GPIO_DEBOUNCE_MS);

	INIT_DELAYED_WORK(&info->wq_detcable, usb_extcon_detect_cable);

	if (info->id_gpiod) {
		info->id_irq = gpiod_to_irq(info->id_gpiod);
		if (info->id_irq < 0) {
			dev_err(dev, "failed to get ID IRQ\n");
			return info->id_irq;
		}

		ret = devm_request_threaded_irq(dev, info->id_irq, NULL,
						usb_irq_handler,
						IRQF_TRIGGER_RISING |
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT,
						pdev->name, info);
		if (ret < 0) {
			dev_err(dev, "failed to request handler for ID IRQ\n");
			return ret;
		}
	}

	if (info->vbus_gpiod) {
		info->vbus_irq = gpiod_to_irq(info->vbus_gpiod);
		if (info->vbus_irq < 0) {
			dev_err(dev, "failed to get VBUS IRQ\n");
			return info->vbus_irq;
		}

		ret = devm_request_threaded_irq(dev, info->vbus_irq, NULL,
						info->enable_button_mode ? gpio_usbdetect_button_irq : usb_irq_handler,
						IRQF_TRIGGER_RISING |
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT,
						pdev->name, info);
		if (ret < 0) {
			dev_err(dev, "failed to request handler for VBUS IRQ\n");
			return ret;
		}
	}

	platform_set_drvdata(pdev, info);
	device_init_wakeup(dev, true);

	/* Perform initial detection */
	usb_extcon_detect_cable(&info->wq_detcable.work);

	return 0;
}

static int usb_extcon_remove(struct platform_device *pdev)
{
	struct usb_extcon_info *info = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&info->wq_detcable);
	cancel_delayed_work_sync(&info->debounce_timer);
	device_init_wakeup(&pdev->dev, false);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int usb_extcon_suspend(struct device *dev)
{
	struct usb_extcon_info *info = dev_get_drvdata(dev);
	int ret = 0;

	if (device_may_wakeup(dev)) {
		if (info->id_gpiod) {
			ret = enable_irq_wake(info->id_irq);
			if (ret)
				return ret;
		}
		if (info->vbus_gpiod) {
			ret = enable_irq_wake(info->vbus_irq);
			if (ret) {
				if (info->id_gpiod)
					disable_irq_wake(info->id_irq);

				return ret;
			}
		}
	}

	/*
	 * We don't want to process any IRQs after this point
	 * as GPIOs used behind I2C subsystem might not be
	 * accessible until resume completes. So disable IRQ.
	 */
	if (info->id_gpiod)
		disable_irq(info->id_irq);
	if (info->vbus_gpiod)
		disable_irq(info->vbus_irq);

	return ret;
}

static int usb_extcon_resume(struct device *dev)
{
	struct usb_extcon_info *info = dev_get_drvdata(dev);
	int ret = 0;

	if (device_may_wakeup(dev)) {
		if (info->id_gpiod) {
			ret = disable_irq_wake(info->id_irq);
			if (ret)
				return ret;
		}
		if (info->vbus_gpiod) {
			ret = disable_irq_wake(info->vbus_irq);
			if (ret) {
				if (info->id_gpiod)
					enable_irq_wake(info->id_irq);

				return ret;
			}
		}
	}

	if (info->id_gpiod)
		enable_irq(info->id_irq);
	if (info->vbus_gpiod)
		enable_irq(info->vbus_irq);

	queue_delayed_work(system_power_efficient_wq,
			   &info->wq_detcable, 0);

	return ret;
}
#endif

static SIMPLE_DEV_PM_OPS(usb_extcon_pm_ops,
			 usb_extcon_suspend, usb_extcon_resume);

static const struct of_device_id usb_extcon_dt_match[] = {
	{ .compatible = "linux,extcon-usb-gpio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, usb_extcon_dt_match);

static const struct platform_device_id usb_extcon_platform_ids[] = {
	{ .name = "extcon-usb-gpio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, usb_extcon_platform_ids);

static struct platform_driver usb_extcon_driver = {
	.probe		= usb_extcon_probe,
	.remove		= usb_extcon_remove,
	.driver		= {
		.name	= "extcon-usb-gpio",
		.pm	= &usb_extcon_pm_ops,
		.of_match_table = usb_extcon_dt_match,
	},
	.id_table = usb_extcon_platform_ids,
};

module_platform_driver(usb_extcon_driver);

MODULE_AUTHOR("Roger Quadros <rogerq@ti.com>");
MODULE_DESCRIPTION("USB GPIO extcon driver");
MODULE_LICENSE("GPL v2");

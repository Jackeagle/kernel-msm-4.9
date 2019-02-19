/* Copyright (c) 2016, The Linux Foundation. All rights reserved.
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

#ifndef __TI_DRV8830__

#define DRV8830_CONTROL_REG 0x0

#define DRV8830_VSET_OFFSET 2
#define DRV8830_IN0     BIT(0)
#define DRV8830_IN1     BIT(1)

#define DRV8830_FAULT_REG 0x1

#define DRV8830_CLEAR   BIT(7)
#define DRV8830_ILIMIT  BIT(4)
#define DRV8830_OTS     BIT(3)
#define DRV8830_UVLO    BIT(2)
#define DRV8830_OCP     BIT(1)
#define DRV8830_FAULT   BIT(0)

#define DRV8830_BRAKE (DRV8830_IN0 | DRV8830_IN1)

struct drv8830_pdata {
	const char *name;
	s32 fault_gpio;
	u8 vset;
};

#endif

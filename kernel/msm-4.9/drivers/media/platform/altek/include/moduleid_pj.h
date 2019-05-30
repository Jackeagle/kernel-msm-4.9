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
 * File: Moduleid_PJ.h
 * Description: Define module id  in project
 */
#ifndef _MODULEID_PJ_H_
#define _MODULEID_PJ_H_

#include "moduleid.h"

/* Project-dependent module starts from MODULEID_PROJECT (0x400)*/
#define MODULEID_PJ_MINIISP_STATE	 (MODULEID_PROJECT + 0x2C)
#define MODULEID_PJ_ISPCTRLIF_SLAVE	 (MODULEID_PROJECT + 0x2D)
#define MODULEID_PJ_ISPCTRLIF_MASTER	(MODULEID_PROJECT + 0x2E)
#define MODULEID_PJ_MINIISP			 (MODULEID_PROJECT + 0x2F)
#endif





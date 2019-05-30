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
 * File: error_pj.h
 * Description: Error code base of modules
 */
#ifndef _ERROR_PJ_H_
#define _ERROR_PJ_H_

#include "error.h"
#include "moduleid_pj.h"

/**
 *@ingroup ErrorCode
 *@{
 */

/* The following constants define the module ID and the error code base*/
#define ERR_BASE_PJ_MINIISP_STATE	   ERR_BASE(MODULEID_PJ_MINIISP_STATE)
#define ERR_BASE_PJ_ISPCTRLIF_SLAVE	 ERR_BASE(MODULEID_PJ_ISPCTRLIF_SLAVE)
#define ERR_BASE_PJ_ISPCTRLIF_MASTER	ERR_BASE(MODULEID_PJ_ISPCTRLIF_MASTER)
#define ERR_BASE_PJ_MINIISP			 ERR_BASE(MODULEID_PJ_MINIISP)



#endif

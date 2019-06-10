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

/**
 * File: ispctrl_if_master_err.h
 * Description: ISP Ctrl IF Master error definition
 *
 *
 *  2013/10/14; Aaron Chuang; Initial version
 *  2013/12/05; Bruce Chung; 2nd version
 */

#ifndef	_ISPCTRLIFMASTER_ERR_H_
#define	_ISPCTRLIFMASTER_ERR_H_

/******Include File******/


#include "../error_pj.h"


/******Public Define******/
/* command Size mismatch*/
#define ERR_MASTERCMDSIZE_MISMATCH (ERR_BASE_PJ_ISPCTRLIF_MASTER + 0x000)
/* null buffer*/
#define ERR_MASTERCMDBUF_NULL (ERR_BASE_PJ_ISPCTRLIF_MASTER + 0x001)
/* command options unsupport*/
#define ERR_MASTERCMDOPTION_UNSUPPORT (ERR_BASE_PJ_ISPCTRLIF_MASTER + 0x002)
/* open file error*/
#define ERR_MASTERCMDCKSM_INVALID (ERR_BASE_PJ_ISPCTRLIF_MASTER + 0x003)
/* open file error*/
#define ERR_MASTEROPENFILE_FAILED (ERR_BASE_PJ_ISPCTRLIF_MASTER + 0x004)
/* open file error*/
#define ERR_MASTEROPENFILE_FILENAME_INVALID (ERR_BASE_PJ_ISPCTRLIF_MASTER+\
						0x005)
/* event timeout*/
#define ERR_MASTER_EVENT_TIMEOUT (ERR_BASE_PJ_ISPCTRLIF_MASTER + 0x006)


#endif

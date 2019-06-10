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
 * File: operation_cmd.c
 * Description: operation command
 *
 *
 *  2013/10/14; Aaron Chuang; Initial version
 *  2013/12/05; Bruce Chung; 2nd version
 */

/******Include File******/


#include <linux/string.h>
#include <uapi/asm-generic/errno-base.h>

#include "include/error.h"
#include "include/mtype.h"
#include "include/error.h"
#include "include/isp_camera_cmd.h"
#include "include/miniisp.h"
#include "include/ispctrl_if_master.h"


/******Private Constant Definition******/


#define MINI_ISP_LOG_TAG "[operation_cmd]"

extern struct file *g_filp[FIRMWARE_MAX];

/******Public Function******/

/**
 *\brief Mini ISP open
 *\param devdata [In], CMD param
 *\param opcode [In], CMD param
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_operation_cmd_miniisp_open(void *devdata,
						u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u64 *boot_pointer,
		*basic_pointer,
		*advanced_pointer,
		*scenario_pointer,
		*hdr_pointer,
		*irp0_pointer,
		*irp1_pointer,
		*ppmap_pointer,
		*depth_pointer;

	char *bootpath,  /*boot code*/
		 *basicpath,  /*basic code*/
		 *advancedpath, /*advanced code*/
		 *scenariopath,  /*scenario data*/
		 *hdrpath,  /*hdr qmerge data*/
		 *irp0path, /*irp0 qmerge data*/
		 *irp1path, /*irp1 qmerge data*/
		 *ppmappath, /*pp map*/
		 *depthpath; /*depth qmerge data*/

	boot_pointer	= (u64 *)&param[0];
	basic_pointer = (u64 *)&param[8];
	advanced_pointer	= (u64 *)&param[16];
	scenario_pointer	= (u64 *)&param[24];
	hdr_pointer = (u64 *)&param[32];
	irp0_pointer	= (u64 *)&param[40];
	irp1_pointer	= (u64 *)&param[48];
	ppmap_pointer	= (u64 *)&param[56];
	depth_pointer	= (u64 *)&param[64];

	bootpath	 = (char *)*boot_pointer;
	basicpath	 = (char *)*basic_pointer;
	advancedpath = (char *)*advanced_pointer;
	scenariopath	 = (char *)*scenario_pointer;
	hdrpath	 = (char *)*hdr_pointer;
	irp0path = (char *)*irp0_pointer;
	irp1path	 = (char *)*irp1_pointer;
	ppmappath	 = (char *)*ppmap_pointer;
	depthpath	 = (char *)*depth_pointer;

	misp_info("%s - start", __func__);

	/*open boot code*/
	if (bootpath) {
		/*
		 *misp_info("%s - boot code, path: %s",
		 *		__func__, bootpath);
		 */
		err = ispctrl_if_mast_request_firmware(bootpath,
						BOOT_CODE);
		if (err != ERR_SUCCESS) {
			misp_err("%s - open boot code failed", __func__);
			goto mast_operation_cmd_miniisp_open_end;
		}
	}

	/*open basic code*/
	if (basicpath) {
		/*
		 *misp_info("%s - basic code, path: %s",
		 *		__func__, basicpath);
		 */
		err = ispctrl_if_mast_request_firmware(basicpath,
						BASIC_CODE);
		if (err != ERR_SUCCESS) {
			misp_err("%s - open basic code failed", __func__);
			goto mast_operation_cmd_miniisp_open_end;
		}
	}

	/*open advanced code*/
	if (advancedpath) {
		/*
		 *misp_info("%s - advanced code, path: %s",
		 *		__func__, advancedpath);
		 */
		err = ispctrl_if_mast_request_firmware(advancedpath,
						ADVANCED_CODE);
		if (err != ERR_SUCCESS) {
			misp_err("%s - open advanced code failed", __func__);
			goto mast_operation_cmd_miniisp_open_end;
		}
	}


	/*open scenario data*/
	if (scenariopath) {
		/*
		 *misp_info("%s - scenario data, path: %s",
		 *		__func__, scenariopath);
		 */
		err = ispctrl_if_mast_request_firmware(scenariopath,
						SCENARIO_CODE);
		if (err != ERR_SUCCESS) {
			misp_err("%s - open scenario data failed", __func__);
			goto mast_operation_cmd_miniisp_open_end;
		}
	}

	/*open hdr qmerge data*/
	if (hdrpath) {

		err = ispctrl_if_mast_request_firmware(hdrpath,
						HDR_CODE);
		if (err != ERR_SUCCESS) {
			misp_err("%s - open hdr qmerge data failed", __func__);
			goto mast_operation_cmd_miniisp_open_end;
		}
	}
	/*open irp0 qmerge data*/
	if (irp0path) {

		err = ispctrl_if_mast_request_firmware(irp0path,
						IRP0_CODE);
		if (err != ERR_SUCCESS) {
			misp_err("%s - open irp0 qmerge data failed", __func__);
			goto mast_operation_cmd_miniisp_open_end;
		}
	}
	/*open irp1 qmerge data*/
	if (irp1path) {

		err = ispctrl_if_mast_request_firmware(irp1path,
						IRP1_CODE);
		if (err != ERR_SUCCESS) {
			misp_err("%s - open irp1 qmerge data failed", __func__);
			goto mast_operation_cmd_miniisp_open_end;
		}
	}

	/*open pp map*/
	if (ppmappath) {

		err = ispctrl_if_mast_request_firmware(ppmappath,
						PPMAP_CODE);
		if (err != ERR_SUCCESS) {
			misp_err("%s - open PP map failed", __func__);
			goto mast_operation_cmd_miniisp_open_end;
		}
	}

	/*open depth qmerge data*/
	if (depthpath) {

		err = ispctrl_if_mast_request_firmware(depthpath,
						DEPTH_CODE);
		if (err != ERR_SUCCESS) {
			/* Ignored error for 'No such file or directory' */
			if ((ENOENT + err) == 0) {
			    g_filp[DEPTH_CODE] = NULL;
				err = ERR_SUCCESS;
			}
			misp_info("%s - open depth qmerge data failed", __func__);
			/* goto mast_operation_cmd_miniisp_open_end; */
		}
	}

	misp_info("%s end", __func__);

mast_operation_cmd_miniisp_open_end:

	return err;
}

/******Private Function******/





/****** End Of File******/

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
 * File: miniisp_ctrl_intf.c
 * Description: mini ISP control cmd interface. use for handling the control cmds instead of debug cmds
 *
 *
 *  2018/08/28; PhenixChen; Initial version
 */

/******Include File******/
#include <miniISP/miniISP_ioctl.h>
#include <linux/uaccess.h> // copy_*_user()
#include "include/miniisp.h"
#include "include/miniisp_ctrl.h" // mini_isp_drv_setting()
#include "include/isp_camera_cmd.h" // MINI_ISP_MODE_E2A, MINI_ISP_MODE_NORMAL
#include "include/ispctrl_if_master.h" // ispctrl_if_mast_execute_cmd()
#include "include/miniisp_customer_define.h"
#include "include/miniisp_ctrl_intf.h"

/******Private Constant Definition******/
#define MINI_ISP_LOG_TAG	"[miniisp_ctrl_intf]"

/******Private Function Prototype******/

/******Public Function Prototype******/

/******Private Global Variable******/

//AL6100 Kernel Base Solution >>>
struct file *internal_file[ECHO_OTHER_MAX];
//AL6100 Kernel Base Solution <<<

long handle_ControlFlowCmd(unsigned int cmd, unsigned long arg)
{
	long retval = 0;
	struct miniISP_cmd_config *config = NULL;
	u8 *param = NULL;

	misp_info("%s - enter", __func__);

	/* step1: allocate & receive cmd struct from user space */
	config = kzalloc(sizeof(struct miniISP_cmd_config), GFP_KERNEL);
	if (NULL == config) {
		retval = -ENOMEM;
		goto done;
	}
	if (copy_from_user(config, (void __user *)arg, sizeof(struct miniISP_cmd_config))) {
		retval = -EFAULT;
		goto done;
	}

	/* step2: allocate & receive cmd parameter from user space if needed*/
	if (config->size > 0) {
		param = kzalloc(config->size, GFP_KERNEL);
		if (NULL == param){
			retval = -ENOMEM;
			goto done;
		}
		if (copy_from_user((void *)param, (void __user *)(config->param), config->size)) {
			retval = -EFAULT;
			goto done;
		}
	}

	switch (cmd) {
	case IOCTL_ISP_LOAD_FW:
		misp_info("%s - IOCTL_ISP_LOAD_FW", __func__);
		//open boot and FW file then write boot code and FW code
		mini_isp_poweron();
		mini_isp_drv_setting(MINI_ISP_MODE_GET_CHIP_ID);
#ifndef AL6100_SPI_NOR
		mini_isp_drv_setting(MINI_ISP_MODE_CHIP_INIT); //if boot form SPI NOR, do not call this
#endif
		mini_isp_drv_setting(MINI_ISP_MODE_E2A);
		mini_isp_drv_setting(MINI_ISP_MODE_NORMAL);
		break;
	case IOCTL_ISP_PURE_BYPASS:
		misp_info("%s - IOCTL_ISP_PURE_BYPASS", __func__);
		mini_isp_poweron();
		mini_isp_drv_setting(MINI_ISP_MODE_GET_CHIP_ID);
		mini_isp_drv_set_bypass_mode(1);
		break;
	case IOCTL_ISP_CTRL_CMD:
		if (param == NULL) {
			misp_info("%s - cmd parameter is NULL", __func__);
			break;
		}
		retval = ispctrl_if_mast_execute_cmd(config->opcode, param);
		break;
	case IOCTL_ISP_DEINIT:
		misp_info("%s - IOCTL_ISP_DEINIT", __func__);
		mini_isp_poweroff();
		break;
	default:
		misp_info("%s - UNKNOWN CMD[0x%x]", __func__, cmd);
		retval = -ENOTTY;
		break;
	}

done:
	if (param != NULL) {
		kfree(param);
	}
	if (config != NULL) {
		kfree(config);
	}
	misp_info("%s - leave", __func__);
	return retval;
}


//AL6100 Kernel Base Solution >>>
void mini_isp_other_drv_open(char *file_name, u8 type) {

	/* Error Code*/
	errcode err = ERR_SUCCESS;
	mm_segment_t oldfs;

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	misp_info("%s filepath : %s", __func__, file_name);

	internal_file[type] = filp_open(file_name, O_RDONLY, 0644);
	set_fs(oldfs);

	if (IS_ERR(internal_file[type])) {
		err = PTR_ERR(internal_file[type]);
		misp_err("%s open file failed. err: %x", __func__, err);
	} else {
		misp_info("%s open file success!", __func__);
	}
}

void mini_isp_other_drv_read(struct file *filp, u8 type) {
	static u8 *calibration_data_buf_addr;
	errcode err = ERR_SUCCESS;
	u32 filesize;
	off_t currpos;
	mm_segment_t oldfs;
	loff_t offset;

	if (filp == NULL) {
		misp_err("%s - file didn't exist.", __func__);
		err = ~ERR_SUCCESS;
		goto read_calibration_data_end;
	}

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	/*get the file size*/
	currpos = vfs_llseek(filp, 0L, SEEK_END);
	if (currpos == -1) {
		set_fs(oldfs);
		misp_err("%s  llseek end failed", __func__);
		err = ~ERR_SUCCESS;
		goto read_calibration_data_end;
	}

	filesize = (u32)currpos;
	/*misp_info("%s  filesize : %u", __func__, filesize);*/

	currpos = vfs_llseek(filp, 0L, SEEK_SET);
	if (currpos == -1) {
		set_fs(oldfs);
		misp_err("%s  llseek set failed", __func__);
		err = ~ERR_SUCCESS;
		goto read_calibration_data_end;
	}

	/*Request memory*/
	calibration_data_buf_addr = kzalloc(filesize, GFP_KERNEL);
	if (!calibration_data_buf_addr) {
		err = ~ERR_SUCCESS;
		kfree(calibration_data_buf_addr);
		goto read_calibration_data_end;
	}

	/*read the header info (first 16 bytes in the data)*/
	offset = filp->f_pos;
	err = vfs_read(filp, calibration_data_buf_addr, filesize,
		&offset);
	set_fs(oldfs);
	if (err == -1) {
		misp_err("%s Read file failed.", __func__);
		/*close the file*/
		filp_close(filp, NULL);
		kfree(calibration_data_buf_addr);
		goto read_calibration_data_end;
	}
	filp->f_pos = offset;
	vfs_llseek(filp, 0L, SEEK_SET);

	// write_calibration_data
	err = mini_isp_drv_write_calibration_data(type, calibration_data_buf_addr, filesize);
	if(err != 0)
		misp_err("%s write_calibration_data_fail", __func__);
	filp_close(filp, NULL);
	kfree(calibration_data_buf_addr);
	goto done;

read_calibration_data_end:
	misp_err("%s read_calibration_data_fail", __func__);
done:
	misp_info("%s end", __func__);
}


long handle_ControlFlowCmd_Kernel(unsigned int cmd, unsigned long arg) {
	long retval = 0;
	struct miniISP_cmd_config *config = NULL;
	struct miniISP_chi_param *param = NULL;

	static uint8_t irflood_mode = 0;
	struct isp_cmd_active_ae active_ae;
	struct isp_cmd_led_power_control control_param;
	struct isp_cmd_set_output_format output_fmt;
	struct isp_cmd_cycle_trigger_depth_process cycle_trigger_depth_param;
	struct isp_cmd_depth_auto_interleave_param depth_auto_interleave_param;
	struct isp_cmd_lighting_ctrl lighting_ctrl;

	memset(&active_ae, 0, sizeof(struct isp_cmd_active_ae));
	memset(&control_param, 0, sizeof(struct isp_cmd_led_power_control));
	memset(&output_fmt, 0, sizeof(struct isp_cmd_set_output_format));
	memset(&cycle_trigger_depth_param, 0, sizeof(struct isp_cmd_cycle_trigger_depth_process));
	memset(&depth_auto_interleave_param, 0, sizeof(struct isp_cmd_depth_auto_interleave_param));
	memset(&lighting_ctrl, 0, sizeof(struct isp_cmd_lighting_ctrl));

	misp_info("%s - enter", __func__);

	switch (cmd) {
	case IOCTL_ISP_RUN_TASK_START:
		misp_info("%s - IOCTL_ISP_RUN_TASK_START", __func__);

		/* step1: allocate & receive cmd struct from user space */
		config = kzalloc(sizeof(struct miniISP_cmd_config), GFP_KERNEL);
		if (NULL == config) {
			retval = -ENOMEM;
			break;
		}
		if (copy_from_user(config, (void __user *)arg, sizeof(struct miniISP_cmd_config))) {
			retval = -EFAULT;
			break;
		}

		/* step2: allocate & receive cmd parameter from user space if needed*/
		if (config->size > 0) {
			param = kzalloc(sizeof(struct miniISP_chi_param), GFP_KERNEL);
			if (NULL == param){
				retval = -ENOMEM;
				break;
			}
			if (copy_from_user((void *)param, (void __user *)(config->param), sizeof(struct miniISP_chi_param))) {
				retval = -EFAULT;
				break;
			}
		}

		/* step3: request IRQ and load FW*/
		mini_isp_poweron();
		mini_isp_drv_setting(MINI_ISP_MODE_GET_CHIP_ID);
		mini_isp_drv_setting(MINI_ISP_MODE_CHIP_INIT); //if boot form SPI NOR, do not call this
		mini_isp_drv_setting(MINI_ISP_MODE_E2A);
		mini_isp_drv_setting(MINI_ISP_MODE_NORMAL);

		/* step4: load bulk data */
		mini_isp_other_drv_open(IQCALIBRATIONDATA_FILE_LOCATION, ECHO_IQ_CODE);
		mini_isp_other_drv_read(internal_file[ECHO_IQ_CODE], ECHO_IQ_CODE);
		mini_isp_other_drv_open(DEPTHPACKDATA_FILE_LOCATION, ECHO_DEPTH_CODE);
		mini_isp_other_drv_read(internal_file[ECHO_DEPTH_CODE], ECHO_DEPTH_CODE); // Depth calibration data
		mini_isp_drv_write_calibration_data(2, NULL, 0); // Scenario table
		mini_isp_drv_write_calibration_data(3, NULL, 0); // HDR Qmerge
		mini_isp_drv_write_calibration_data(4, NULL, 0); // IRP0 Qmerge
		mini_isp_drv_write_calibration_data(5, NULL, 0); // IRP1 Qmerge
		// mini_isp_drv_write_calibration_data(7, NULL, 0); // Blending table for ground depth
		// mini_isp_drv_write_calibration_data(8, NULL, 0); // Depth Qmerge

		/* step5: config AL6100 to depth mode */
		/* AL6100 AE initial */
		active_ae.active_ae = 1;
		active_ae.f_number_x1000 = 2000;
		mini_isp_drv_active_ae(&active_ae);

		/* AL6100 AE on */
		mini_isp_drv_isp_ae_control_mode_on_off(1);

		/* set depth output resolution */
		output_fmt.depth_size = 19;
		output_fmt.reserve[0] = 7;
		mini_isp_drv_set_output_format(&output_fmt);

		/* depth cycle trigger. Must configure it before set sensor mode */
		if (param->irflood_mode == 1) {//P_F_INTERLEAVE
			cycle_trigger_depth_param.cycleLen = 2;
			cycle_trigger_depth_param.depth_triggerBitField = 1;			// 1st frame depth trigger
			cycle_trigger_depth_param.depthoutput_triggerBitField = 3;
			mini_isp_drv_cycle_trigger_depth_process(&cycle_trigger_depth_param);
		}

		/* set sensor mode */
		misp_info("%s,sensor_mode, scid:%d, merge_mode: %d", __func__, param->scid, param->merge_mode);
		// default meage mode : 0
		// ss module meage mode :0x10
		mini_isp_drv_set_sensor_mode(1, param->scid, 0, 0, param->merge_mode);

		if (param->irflood_mode == 1) {//P_F_INTERLEAVE
			/* set interleave mode */
			irflood_mode = 1;
			depth_auto_interleave_param.depth_interleave_mode_on_off = 1;
			depth_auto_interleave_param.skip_frame_num_after_illuminator_pulse = 0;
			depth_auto_interleave_param.projector_power_level = 255;
			depth_auto_interleave_param.illuminator_power_level = 255;
			mini_isp_drv_set_depth_auto_interleave_mode(&depth_auto_interleave_param);

			/* lighting control P_F interleave */
			lighting_ctrl.cycle_len = 2;
			lighting_ctrl.cycle[0].source = 1;				// 1st frame projector on
			lighting_ctrl.cycle[0].TxDrop = 3;				// 1st frame TX0 & TX1 drop
			lighting_ctrl.cycle[0].co_frame_rate = 0;		// 1st use sensor default frame rate
			lighting_ctrl.cycle[1].source = 2;				// 2st frame flood on
			lighting_ctrl.cycle[1].TxDrop = 0;				// 2st frame no drop
			lighting_ctrl.cycle[1].co_frame_rate = 0;		// 2st use sensor default frame rate
			mini_isp_drv_lighting_ctrl(&lighting_ctrl);
		} else if (param->irflood_mode == 0) {//continue mode
			/* projector control */
			irflood_mode = 0;
			control_param.led_on_off = 3;
			control_param.control_mode = 3;
			control_param.led_power_level = 255;
			control_param.control_projector_id = 0;
			mini_isp_drv_led_power_control(&control_param);
		}

		/* open preview */
		mini_isp_drv_preview_stream_on_off(1, 1);
		break;
	case IOCTL_ISP_RUN_TASK_STOP:
		misp_info("%s - IOCTL_ISP_RUN_TASK_STOP", __func__);

		mini_isp_drv_preview_stream_on_off(0, 0);
		if (irflood_mode == 1) {//P_F_INTERLEAVE
			mini_isp_drv_set_depth_auto_interleave_mode(&depth_auto_interleave_param);
		}

		if (irflood_mode == 0) {
			mini_isp_drv_led_power_control(&control_param);
		}
		mini_isp_drv_active_ae(&active_ae);
		mini_isp_drv_isp_ae_control_mode_on_off(0);
		mini_isp_drv_set_sensor_mode(0, 0, 0, 0, 0);
		mini_isp_poweroff();
		break;
	default:
		misp_info("%s - UNKNOWN CMD[%x]", __func__, cmd);
		retval = -ENOTTY;
		break;
	}

	if (param != NULL) {
		kfree(param);
	}
	if (config != NULL) {
		kfree(config);
	}

	misp_info("%s - leave", __func__);
	return retval;
}

//AL6100 Kernel Base Solution <<<


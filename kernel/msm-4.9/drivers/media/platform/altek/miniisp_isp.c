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
 * File: miniisp_spi.c
 * Description: Mini ISP sample codes
 *
 *
 *  2017/04/11; LouisWang; Initial version
 */

/************************************************************
*			Include File									*
*************************************************************/
/* Linux headers*/
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/buffer_head.h>
#include <linux/of_gpio.h>


#include "include/miniisp.h"
#include "include/miniisp_ctrl.h"
#include "include/miniisp_customer_define.h"
#include "include/miniisp_chip_base_define.h"
#include "include/altek_statefsm.h"
#include "include/ispctrl_if_master.h"
#include "include/isp_camera_cmd.h"
#include "include/error/miniisp_err.h"
// ALTEK_AL6100_ECHO >>>
#include "include/miniisp_ctrl_intf.h"
// ALTEK_AL6100_ECHO <<<

#ifdef ALTEK_TEST
#include "include/altek_test.h"
#endif

/****************************************************************************
*						 Private Constant Definition						*
****************************************************************************/
#define DEBUG_NODE 0
/*#define DEBUG_ALERT*/
#define MINI_ISP_LOG_TAG "[miniisp_isp]"
/*drv debug defination*/
#define _SPI_DEBUG

/****************************************************************************
*						Private Global Variable								*
****************************************************************************/
static struct misp_global_variable *misp_drv_global_variable;
static struct class *mini_isp_class;
static struct device *mini_isp_dev;
struct altek_statefsm *altek_state;
static int dump_reg_range;
// ALTEK_AL6100_ECHO >>>
struct file *l_internal_file[ECHO_OTHER_MAX];
// ALTEK_AL6100_ECHO <<<
extern u16 fw_version_before_point;
extern u16 fw_version_after_point;
extern char fw_build_by[];
extern u32 sc_build_date;
/************************************************************
*		  Public Global Variable						   	*
*************************************************************/

/************************************************************
*		  Private Macro Definition							*
*************************************************************/

/************************************************************
*		  Public Function Prototype							*
*************************************************************/
/// AL6100 debug tool >>>
extern struct device *miniisp_chdev_create(struct class *mini_isp_class);
/// AL6100 debug tool <<<
extern struct misp_data *get_mini_isp_intf_spi(void);
extern struct misp_data *get_mini_isp_intf_i2c(int i2c_type);
//extern struct misp_data *get_mini_isp_intf_cci(int i2c_type);

/************************************************************
*					Private Function					   	*
*************************************************************/
void mini_isp_other_drv_open_l(char *file_name, u8 type) {

	/* Error Code*/
	errcode err = ERR_SUCCESS;
	mm_segment_t oldfs;

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	misp_info("%s filepath : %s", __func__, file_name);

	l_internal_file[type] = filp_open(file_name, O_RDONLY, 0644);
	set_fs(oldfs);

	if (IS_ERR(l_internal_file[type])) {
		err = PTR_ERR(l_internal_file[type]);
		misp_err("%s open file failed. err: %x", __func__, err);
	} else {
		misp_info("%s open file success!", __func__);
	}
}
static ssize_t mini_isp_mode_config_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/*misp_info("%s - mini_isp_spi_send return %d", __func__, ret);*/
	return snprintf(buf, 32, "load fw:0 e_to_a:1 a_to_e:2\n");
}

static ssize_t mini_isp_mode_config_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	u8 buf_chip_id_use[4];

	if ('0' == buf[0]) {
		mini_isp_chip_init();
		mini_isp_e_to_a();
		mini_isp_drv_load_fw();
	} else if ('1' == buf[0]) {
		mini_isp_chip_init();
		mini_isp_e_to_a();
	} else if ('2' == buf[0]) {
		mini_isp_a_to_e();
	} else if ('4' == buf[0]) {
		mini_isp_get_chip_id(CHIP_ID_ADDR, buf_chip_id_use);
	} else if ('7' == buf[0]) {
		buf_chip_id_use[0] = 0;
		mini_isp_debug_dump_img();
		mini_isp_a_to_e();
		mini_isp_chip_base_dump_irp_and_depth_based_register();
		mini_isp_memory_write(0x10, buf_chip_id_use, 1);
		mini_isp_e_to_a();
// ALTEK_AL6100_ECHO >>>
	}  else if ('5' == buf[0]) {
		struct isp_cmd_led_power_control control_param;
		struct isp_cmd_active_ae active_ae;
		struct isp_cmd_set_output_format output_fmt;
		struct isp_cmd_cycle_trigger_depth_process cycle_trigger_depth_param;
		struct isp_cmd_depth_auto_interleave_param depth_auto_interleave_param;
		struct isp_cmd_lighting_ctrl lighting_ctrl;

		memset(&control_param, 0, sizeof(struct isp_cmd_led_power_control));
		memset(&active_ae, 0, sizeof(struct isp_cmd_active_ae));
		memset(&output_fmt, 0, sizeof(struct isp_cmd_set_output_format));
		memset(&cycle_trigger_depth_param, 0, sizeof(struct isp_cmd_cycle_trigger_depth_process));
		memset(&depth_auto_interleave_param, 0, sizeof(struct isp_cmd_depth_auto_interleave_param));
		memset(&lighting_ctrl, 0, sizeof(struct isp_cmd_lighting_ctrl));
		// open auto depth mode
		mini_isp_poweron();

		if(0 != mini_isp_drv_setting(MINI_ISP_MODE_GET_CHIP_ID)){
			misp_err("get chip id failed \n");
		}
		if(0 != mini_isp_drv_setting(MINI_ISP_MODE_CHIP_INIT)){ //if boot form SPI NOR, do not call this
			misp_err("chip init failed \n");
		}
		if(0 != mini_isp_drv_setting(MINI_ISP_MODE_E2A)){
			misp_err("change MINI_ISP_MODE_E2A failed failed \n");
		}
		if(0 != mini_isp_drv_setting(MINI_ISP_MODE_NORMAL)){
			misp_err("misp_load_fw failed \n");
		}

		mini_isp_other_drv_open_l(IQCALIBRATIONDATA_FILE_LOCATION, ECHO_IQ_CODE);
		mini_isp_other_drv_read(l_internal_file[ECHO_IQ_CODE], ECHO_IQ_CODE); // IQ calibration data

		mini_isp_other_drv_open_l(DEPTHPACKDATA_FILE_LOCATION, ECHO_DEPTH_CODE);
		mini_isp_other_drv_read(l_internal_file[ECHO_DEPTH_CODE], ECHO_DEPTH_CODE); // Depth calibration data

		mini_isp_drv_write_calibration_data(2, NULL, 0); // Scenario table
		mini_isp_drv_write_calibration_data(3, NULL, 0); // HDR Qmerge
		mini_isp_drv_write_calibration_data(4, NULL, 0); // IRP0 Qmerge
		mini_isp_drv_write_calibration_data(5, NULL, 0); // IRP1 Qmerge
		// mini_isp_drv_write_calibration_data(7, NULL, 0); // Blending table for ground depth
		// mini_isp_drv_write_calibration_data(8, NULL, 0); // Depth Qmerge

		// AL6100 AE initial
		active_ae.active_ae = 1;			// active ae initial flow
		active_ae.f_number_x1000 = 2000; // set focus number
		mini_isp_drv_active_ae(&active_ae);

		// AL6100 AE on
		mini_isp_drv_isp_ae_control_mode_on_off(1);


		// set depth output resolution
		output_fmt.depth_size = 19;
		output_fmt.reserve[0] = 7;
		mini_isp_drv_set_output_format(&output_fmt);// set depth output resolution

#ifdef P_F_INTERLEAVE
		// depth cycle trigger. Must configure it before set sensor
		cycle_trigger_depth_param.cycleLen = 2;
		cycle_trigger_depth_param.depth_triggerBitField = 1;			// 1st frame depth trigger
		cycle_trigger_depth_param.depthoutput_triggerBitField = 3;
		mini_isp_drv_cycle_trigger_depth_process(&cycle_trigger_depth_param);
#endif

		mini_isp_drv_set_sensor_mode(1,2,0,0,0);  // Set sensor mode

#ifdef P_F_INTERLEAVE
		// set interleave mode
		depth_auto_interleave_param.depth_interleave_mode_on_off = 1;
		depth_auto_interleave_param.skip_frame_num_after_illuminator_pulse = 0;
		depth_auto_interleave_param.projector_power_level = 255;
		depth_auto_interleave_param.illuminator_power_level = 255;
		mini_isp_drv_set_depth_auto_interleave_mode(&depth_auto_interleave_param);

		// lighting control P_F interleave
		lighting_ctrl.cycle_len = 2;
		lighting_ctrl.cycle[0].source = 1;				// 1st frame projector on
		lighting_ctrl.cycle[0].TxDrop = 3;				// 1st frame TX0 & TX1 drop
		lighting_ctrl.cycle[0].co_frame_rate = 0;		// 1st use sensor default frame rate
		lighting_ctrl.cycle[1].source = 2;				// 2st frame flood on
		lighting_ctrl.cycle[1].TxDrop = 0;				// 2st frame no drop
		lighting_ctrl.cycle[1].co_frame_rate = 0;		// 2st use sensor default frame rate
		mini_isp_drv_lighting_ctrl(&lighting_ctrl);
#else
		//projector control
		control_param.led_on_off = 1;
		control_param.control_mode = 3;
		control_param.led_power_level = 255;
		control_param.control_projector_id = 0;
		mini_isp_drv_led_power_control(&control_param);
#endif
		mini_isp_drv_preview_stream_on_off(1,1); // open preview

		}else if ('6' == buf[0]) {
		// close auto depth mode
		struct isp_cmd_led_power_control control_param;
		struct isp_cmd_active_ae active_ae;
		struct isp_cmd_set_output_format output_fmt;
		memset(&control_param, 0, sizeof(struct isp_cmd_led_power_control));
		memset(&active_ae, 0, sizeof(struct isp_cmd_active_ae));
		memset(&output_fmt, 0, sizeof(struct isp_cmd_set_output_format));

		mini_isp_drv_preview_stream_on_off(0,0); // close preview
		mini_isp_drv_isp_ae_control_mode_on_off(0);
		mini_isp_drv_led_power_control(&control_param);
		mini_isp_drv_set_sensor_mode(0,0,0,0,0);  // Set sensor mode
		mini_isp_drv_active_ae(&active_ae);
		output_fmt.depth_size = 0;
		output_fmt.reserve[0] = 0;
		mini_isp_drv_set_output_format(&output_fmt); // set depth output resolution:
		// ALTEK_AL6100_ECHO >>>                 // 0: Disable depth function (Depth engine is disable)
	}  else if ('8' == buf[0]) {
		mini_isp_debug_dump_img();                  /* Dump IRP image. Save to MINIISP_INFO_DUMPLOCATION path */
		mini_isp_debug_depth_rect_combo_dump(0);    /* Dump normal depth image and register. Save to MINIISP_INFO_DUMPLOCATION path */
		mini_isp_debug_packdata_dump();             /* Dump packdata. Save to MINIISP_INFO_DUMPLOCATION path */
		mini_isp_debug_IQCalib_dump();              /* Dump IQ calibration data. Save to MINIISP_INFO_DUMPLOCATION path */
		mini_isp_debug_metadata_dump();             /* Dump meta data. Save to MINIISP_INFO_DUMPLOCATION path */

		/* Below debug API will print AL6100 system status to dmesg log */
		mini_isp_debug_depth_info();                /* Print depth module information to dmesg */
		mini_isp_debug_metadata_info();             /* Print metadata information to dmesg */
		mini_isp_debug_sensor_info();               /* Print sensor information to dmesg */
		mini_isp_debug_led_info();                  /* Print led information to dmesg */
	}
	else {
		//mini_isp_poweron();
		mini_isp_get_chip_id(CHIP_ID_ADDR, buf_chip_id_use);
		mini_isp_pure_bypass(1);
	}
	return size;
}

static DEVICE_ATTR(mini_isp_mode_config, 0660, mini_isp_mode_config_show,
		mini_isp_mode_config_store);

static ssize_t mini_isp_reset_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret = -EINVAL;

	ret = gpio_get_value(misp_drv_global_variable->reset_gpio);
	misp_info("%s - reset_gpio is %d", __func__, ret);

	return snprintf(buf, 32, "%d", ret);
}

static ssize_t mini_isp_reset_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	if ('0' == buf[0])
		gpio_set_value(misp_drv_global_variable->reset_gpio, 0);
	else
		gpio_set_value(misp_drv_global_variable->reset_gpio, 1);

	misp_info("%s - ", __func__);

	return size;
}

static DEVICE_ATTR(mini_isp_reset, 0660,
					mini_isp_reset_show,
					mini_isp_reset_store);

static ssize_t mini_isp_poweron_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret = -EINVAL;

	ret = gpio_get_value(misp_drv_global_variable->vcc1_gpio);
	misp_info("%s - vcc1_gpio is %d", __func__, ret);
	return snprintf(buf, 32, "%d", ret);
}

static ssize_t mini_isp_poweron_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	/*int ret = -EINVAL;*/
	if ('1' == buf[0])
		mini_isp_poweron();
	else
		mini_isp_poweroff();

	misp_info("%s - ", __func__);

	return size;
}

static DEVICE_ATTR(mini_isp_poweron, S_IRUSR | S_IWUSR, mini_isp_poweron_show,
		mini_isp_poweron_store);

static ssize_t mini_isp_dump_reg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret = -EINVAL;

	misp_info("%s - enter", __func__);
	mini_isp_a_to_e();
	if (dump_reg_range == 0)
		ret = mini_isp_utility_read_reg_e_mode();
	else if (dump_reg_range == 1)
		ret = mini_isp_utility_read_reg_e_mode_for_bypass_use();
	else
		ret = mini_isp_chip_base_dump_irp_and_depth_based_register();

	if (!ret)
		return snprintf(buf, 32, "dump reg success!!\n");
	else
		return snprintf(buf, 32, "dump reg fail!!\n");
}

static ssize_t mini_isp_dump_reg_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	/*int ret = -EINVAL;*/
	/*0 means dump all reg value, 1 means dump bypass mode reg value */
	if ('0' == buf[0])
		dump_reg_range = 0;
	else if ('1' == buf[0])
		dump_reg_range = 1;
	else
		dump_reg_range = 2;
	misp_info("%s - ", __func__);

	return size;
}

static DEVICE_ATTR(mini_isp_dump_reg, 0660, mini_isp_dump_reg_show,
		mini_isp_dump_reg_store);


static ssize_t mini_isp_frame_sync_ctrl_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 32, "frame sync ctrl!!\n");
}

static ssize_t mini_isp_frame_sync_ctrl_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int ret = -EINVAL;
	int temp[8];
	int sscanf_param_num;
	int request_param_num;
	struct isp_cmd_frame_sync_control frame_sync_control_param;

	misp_info("Set frame sync control start!!");

	sscanf_param_num = sscanf(buf, "%d %d %d %d %d %d %d %d",
								&temp[0],
								&temp[1],
								&temp[2],
								&temp[3],
								&temp[4],
								&temp[5],
								&temp[6],
								&temp[7]);

	request_param_num =
		sizeof(struct isp_cmd_frame_sync_control)/sizeof(u8);

	if (sscanf_param_num < request_param_num) {
		misp_info("Input parameter number %d is mismatch command definition",
					sscanf_param_num);
		return size;
	}

	misp_info("Param_Number = %d ", sscanf_param_num);
	misp_info("Param_0 = %d ", temp[0]);
	misp_info("Param_1 = %d ", temp[1]);
	misp_info("Param_2 = %d ", temp[2]);
	misp_info("Param_3 = %d ", temp[3]);
	misp_info("Param_4 = %d ", temp[4]);
	misp_info("Param_5 = %d ", temp[5]);
	misp_info("Param_6 = %d ", temp[6]);
	misp_info("Param_7 = %d ", temp[7]);

	frame_sync_control_param.control_deviceID = temp[0];
	frame_sync_control_param.delay_framephase = temp[1];
	frame_sync_control_param.active_framephase = temp[2];
	frame_sync_control_param.deactive_framephase = temp[3];
	frame_sync_control_param.active_timelevel = temp[4];
	frame_sync_control_param.reserve[0] = temp[5];
	frame_sync_control_param.reserve[1] = temp[6];
	frame_sync_control_param.reserve[2] = temp[7];

	ret = mini_isp_drv_frame_sync_control(&frame_sync_control_param);
	if (!ret)
		misp_info("Set frame sync control success!!");
	else
		misp_info("Set frame sync control fail, error = %d ", ret);

	return size;
}

static DEVICE_ATTR(mini_isp_frame_sync_control, 0660,
					mini_isp_frame_sync_ctrl_show,
					mini_isp_frame_sync_ctrl_store);

static ssize_t mini_isp_lighting_ctrl_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return 0; // TODO: read it back from device?
}

static ssize_t mini_isp_lighting_ctrl_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	return size; // TODO: for test
}

static DEVICE_ATTR(mini_isp_lighting_ctrl, 0660,
					mini_isp_lighting_ctrl_show,
					mini_isp_lighting_ctrl_store);

static ssize_t mini_isp_exposure_param_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 32, "Set Exposure Param!!\n");
}

static ssize_t mini_isp_exposure_param_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int ret = -EINVAL;
	int temp[3];
	int sscanf_param_num;
	int request_param_num;
	struct isp_cmd_exposure_param exposure_param;

	misp_info("Set Exposure Param start!!");

	sscanf_param_num = sscanf(buf, "%d %d %d",
								&temp[0],
								&temp[1],
								&temp[2]);

	request_param_num = 3;

	if (sscanf_param_num < request_param_num) {
		misp_info("Input parameter number %d is mismatch command definition",
					sscanf_param_num);
		return size;
	}

	misp_info("Param_Number = %d ", sscanf_param_num);
	misp_info("Param_0 = %d ", temp[0]);
	misp_info("Param_1 = %d ", temp[1]);
	misp_info("Param_2 = %d ", temp[2]);

	exposure_param.udExpTime = temp[0];
	exposure_param.uwISO = temp[1];
	exposure_param.ucActiveDevice = temp[2];

	ret = mini_isp_drv_set_exposure_param(&exposure_param);
	if (!ret)
		misp_info("Set Exposure Param success!!");
	else
		misp_info("Set Exposure Param fail, error = %d ", ret);

	return size;
}

static DEVICE_ATTR(mini_isp_exposure_param, 0660,
					mini_isp_exposure_param_show,
					mini_isp_exposure_param_store);


static ssize_t mini_isp_max_exposure_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 32, "Set Max Exposure Param!!\n");
}

static ssize_t mini_isp_max_exposure_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int ret = -EINVAL;
	int temp[3];
	int sscanf_param_num;
	int request_param_num;
	struct isp_cmd_exposure_param max_exposure_param;

	misp_info("Set Max Exposure Param start!!");

	sscanf_param_num = sscanf(buf, "%d %d %d",
								&temp[0],
								&temp[1],
								&temp[2]);

	request_param_num = 3;

	if (sscanf_param_num < request_param_num) {
		misp_info("Input parameter number %d is mismatch command definition",
					sscanf_param_num);
		return size;
	}

	misp_info("Param_Number = %d ", sscanf_param_num);
	misp_info("Param_0 = %d ", temp[0]);
	misp_info("Param_1 = %d ", temp[1]);
	misp_info("Param_2 = %d ", temp[2]);

	max_exposure_param.udExpTime= temp[0];
	max_exposure_param.uwISO= temp[1];
	max_exposure_param.ucActiveDevice = temp[2];

	ret = mini_isp_drv_set_max_exposure(&max_exposure_param);
	if (!ret)
		misp_info("Set Max Exposure Param success!!");
	else
		misp_info("Set Max Exposure Param fail, error = %d ", ret);

	return size;
}

static DEVICE_ATTR(mini_isp_max_exposure, 0660,
					mini_isp_max_exposure_show,
					mini_isp_max_exposure_store);

static ssize_t mini_isp_target_mean_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 32, "Set Target Mean Param!!\n");
}

static ssize_t mini_isp_target_mean_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int ret = -EINVAL;
	int temp[2];
	int sscanf_param_num;
	int request_param_num;
	struct isp_cmd_target_mean target_mean_param;

	misp_info("Set Target Mean Param start!!");

	sscanf_param_num = sscanf(buf, "%d %d",
								&temp[0],
								&temp[1]);

	request_param_num = 2;

	if (sscanf_param_num < request_param_num) {
		misp_info("Input parameter number %d is mismatch command definition",
					sscanf_param_num);
		return size;
	}

	misp_info("Param_Number = %d ", sscanf_param_num);
	misp_info("Param_0 = %d ", temp[0]);
	misp_info("Param_1 = %d ", temp[1]);

	target_mean_param.target_mean = temp[0];
	target_mean_param.ucActiveDevice = temp[1];

	ret = mini_isp_drv_set_target_mean(&target_mean_param);
	if (!ret)
		misp_info("Set Target Mean Param success!!");
	else
		misp_info("Set Target Mean Param fail, error = %d ", ret);

	return size;
}

static DEVICE_ATTR(mini_isp_target_mean, 0660,
					mini_isp_target_mean_show,
					mini_isp_target_mean_store);


static ssize_t mini_isp_rectab_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 32, "Set rectab Param!!\n");
}

static ssize_t mini_isp_rectab_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	u8 trans_mode;
	u32 block_size;
	struct depth_rectab_invrect_param rect_param[3];
	misp_info("Set rectab start!!");

	// fill test pattern
	memset((u8* )&rect_param[0], 0xa, 3*sizeof(struct depth_rectab_invrect_param));

	trans_mode = 0;
	block_size = 64;

	mini_isp_drv_write_depth_rectab_invrect(
		&rect_param[0], trans_mode, block_size);

	misp_info("Set rectab end!!");
	return size;
}

static DEVICE_ATTR(mini_isp_rectab, 0660,
					mini_isp_rectab_show,
					mini_isp_rectab_store);

static ssize_t mini_isp_depth_compensation_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 32, "Set depth_shift!!\n");
}

static ssize_t mini_isp_depth_compensation_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	u8 en_update = 0;
	struct isp_cmd_depth_compensation_param depth_compensation_param;
	u32 temp[4];
	misp_info("Set depth_compensation start!!");

	sscanf(buf, "%d %d %d %d", &temp[0], &temp[1], &temp[2], &temp[3]);

	misp_info("%d %d %d %d", temp[0], temp[1], temp[2], temp[3]);

	en_update = (temp[0] << 4) | temp[1];
	depth_compensation_param.en_updated = (u8)en_update;
	depth_compensation_param.short_distance_value = (u16)temp[2];
	depth_compensation_param.compensation = (s8)temp[3];

	mini_isp_drv_depth_compensation(&depth_compensation_param);

	misp_info("Set depth_compensation end!!");
	return size;
}

static DEVICE_ATTR(mini_isp_depth_compensation, 0660,
					mini_isp_depth_compensation_show,
					mini_isp_depth_compensation_store);

static ssize_t mini_isp_cycle_trigger_depth_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 32, "Set cycle trigger depth!!\n");
}

static ssize_t mini_isp_cycle_trigger_depth_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct isp_cmd_cycle_trigger_depth_process depth_param;
	u32 temp[3];
	misp_info("Set cycle trigger depth start!!");

	sscanf(buf, "%d %d %d", &temp[0], &temp[1], &temp[2]);

	misp_info("%d %d %d", temp[0], temp[1], temp[2]);

	depth_param.cycleLen = (u8)temp[0];
	depth_param.depth_triggerBitField = (u16)temp[1];
	depth_param.depthoutput_triggerBitField = (u16)temp[2];
	mini_isp_drv_cycle_trigger_depth_process(&depth_param);

	misp_info("Set cycle trigger depth end!!");
	return size;
}

static DEVICE_ATTR(mini_isp_cycle_trigger_depth, 0660,
					mini_isp_cycle_trigger_depth_show,
					mini_isp_cycle_trigger_depth_store);

static ssize_t mini_isp_min_exposure_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 32, "Set Min Exposure Param!!\n");
}

static ssize_t mini_isp_min_exposure_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int ret = -EINVAL;
	int temp[3];
	int sscanf_param_num;
	int request_param_num;
	struct isp_cmd_exposure_param min_exposure_param;

	misp_info("Set Min Exposure Param start!!");

	sscanf_param_num = sscanf(buf, "%d %d %d",
								&temp[0],
								&temp[1],
								&temp[2]);

	request_param_num = 3;

	if (sscanf_param_num < request_param_num) {
		misp_info("Input parameter number %d is mismatch command definition",
					sscanf_param_num);
		return size;
	}

	misp_info("Param_Number = %d ", sscanf_param_num);
	misp_info("Param_0 = %d ", temp[0]);
	misp_info("Param_1 = %d ", temp[1]);
	misp_info("Param_2 = %d ", temp[2]);

	min_exposure_param.udExpTime = temp[0];
	min_exposure_param.uwISO = temp[1];
	min_exposure_param.ucActiveDevice = temp[2];

	ret = mini_isp_drv_set_min_exposure(&min_exposure_param);
	if (!ret)
		misp_info("Set Min Exposure Param success!!");
	else
		misp_info("Set Min Exposure Param fail, error = %d ", ret);

	return size;
}

static DEVICE_ATTR(mini_isp_min_exposure, 0660,
					mini_isp_min_exposure_show,
					mini_isp_min_exposure_store);

static ssize_t mini_isp_max_exposure_slope_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 32, "Set Max Exposure Slope Param!!\n");
}


static ssize_t mini_isp_max_exposure_slope_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int ret = -EINVAL;
	int temp[3];
	int sscanf_param_num;
	int request_param_num;
	struct isp_cmd_max_exposure_slope max_exposure_slope_param;

	misp_info("Set Min Exposure Param start!!");

	sscanf_param_num = sscanf(buf, "%d",&temp[0]);

	request_param_num = 1;

	if (sscanf_param_num < request_param_num) {
		misp_info("Input parameter number %d is mismatch command definition",
					sscanf_param_num);
		return size;
	}

	misp_info("Param_Number = %d ", sscanf_param_num);
	misp_info("Param_0 = %d ", temp[0]);

	max_exposure_slope_param.max_exposure_slope = temp[0];

	ret = mini_isp_drv_set_max_exposure_slope(&max_exposure_slope_param);
	if (!ret)
		misp_info("Set Max Exposure Slope Param success!!");
	else
		misp_info("Set Max Exposure Slope Param fail, error = %d ", ret);

	return size;
}

static DEVICE_ATTR(mini_isp_max_exposure_slope, 0660,
					mini_isp_max_exposure_slope_show,
					mini_isp_max_exposure_slope_store);

static ssize_t mini_isp_comlog_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 32, "comlog!!\n");
}

static ssize_t mini_isp_comlog_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct common_log_hdr_info info;
	misp_info("mini_isp_comlog_store Start!!");

	info.block_size = SPI_TX_BULK_SIZE;
	info.total_size = LEVEL_LOG_BUFFER_SIZE;
	ispctrl_if_mast_execute_cmd(ISPCMD_BULK_READ_COMLOG, (u8 *)&info);
	misp_info("mini_isp_comlog_store end!!");
	return size;
}

static DEVICE_ATTR(mini_isp_comlog, 0660,
					mini_isp_comlog_show,
					mini_isp_comlog_store);

static ssize_t mini_isp_mem_dump_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 32, "mini_isp_mem_dump_show\n");
}

static ssize_t mini_isp_mem_dump_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	u32 start_addr = 0;
	u32 dump_size = 0;
	char str[10];
	int err = 0;
	void *read_buf = NULL;
	struct file *filp = NULL;
	mm_segment_t fs;
	char filename[80];
	struct misp_global_variable *dev_global_variable;

	misp_info("mini_isp_mem_dump_store S\n");

	sscanf(buf, "0x%x %s", &start_addr, str);
	if(str[0] == '0' && str[1] == 'x')
		sscanf(str, "0x%x", &dump_size);
	else
		sscanf(str, "%d", &dump_size);

	misp_info("addr: 0x%x, size: %d", start_addr, dump_size);

	dev_global_variable = get_mini_isp_global_variable();

	// switch to E mode
	if ((dev_global_variable->intf_status & INTF_SPI_READY) &&
		(dev_global_variable->altek_spi_mode == ALTEK_SPI_MODE_A)) {
		misp_info("%s a_to_e", __func__);
		mini_isp_a_to_e();
	}

	// request read buffer
	read_buf = vmalloc(dump_size);
	if(read_buf == NULL) {
		misp_info("allocate buffer fail\n");
		goto BufferFail;
	}
	// Read AL6100 memory
	mini_isp_memory_read(start_addr, read_buf, dump_size); // E mode

	snprintf(filename, 80, "%s/miniISP_memory_0x%x.log",
				MINIISP_INFO_DUMPLOCATION, start_addr);
	filp = filp_open(filename, O_APPEND|O_CREAT|O_RDWR, 0777);
	/*Get current segment descriptor*/
	fs = get_fs();
	/*Set segment descriptor associated*/
	set_fs(get_ds());

	if (IS_ERR(filp)) {
		err = PTR_ERR(filp);
		misp_err("%s open file failed. err: %d", __func__, err);
		set_fs(fs);
		goto FileErr;
	}

	err = vfs_write(filp, read_buf, dump_size,
					&filp->f_pos);
	if (err == -1) {
		misp_info("%s write file failed.", __func__);
		goto FileErr;
	}

	/*Restore segment descriptor*/
	set_fs(fs);
	filp_close(filp, NULL);

FileErr:
BufferFail:
	vfree(read_buf);

	// switch to A mode
	if ((dev_global_variable->intf_status & INTF_SPI_READY) &&
		(dev_global_variable->altek_spi_mode == ALTEK_SPI_MODE_E)) {
		misp_info("%s e_to_a", __func__);
		mini_isp_e_to_a();
	}
	misp_info("mini_isp_mem_dump_store E\n");
	return size;
}

static DEVICE_ATTR(mini_isp_mem_dump, 0660, mini_isp_mem_dump_show,
		mini_isp_mem_dump_store);

u32 led_on_off = 3;
u32 led_power_level = 255;
u32 control_projector_id = 0;
static ssize_t mini_isp_Led_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	misp_info("led_on_off: %d\n", led_on_off);
	misp_info("led_power_level: %d\n", led_power_level);
	misp_info("control_projector_id: %d\n", control_projector_id);
	return 0;
}

static ssize_t mini_isp_Led_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct isp_cmd_led_power_control projector_control_param = {0};
	/* Error Code*/
	ssize_t err = ERR_SUCCESS;

	sscanf(buf, "%d %d %d", &led_on_off, &led_power_level, &control_projector_id);

	projector_control_param.led_on_off	  = (u8) led_on_off;
	projector_control_param.led_power_level = (u8) led_power_level;
	projector_control_param.control_projector_id = (u8) control_projector_id;
	projector_control_param.delay_after_sof = 0;
	projector_control_param.pulse_time	  = 0;
	projector_control_param.control_mode	= 0;
	projector_control_param.pulse_mode_skip_frame = 0;
	projector_control_param.rolling_shutter = 0;

	misp_info("led_on_off: %d\n", projector_control_param.led_on_off);
	misp_info("led_power_level: %d\n", projector_control_param.led_power_level);
	misp_info("control_projector_id: %d\n", projector_control_param.control_projector_id);
	err = mini_isp_drv_led_power_control(&projector_control_param);

	return size;
}

static DEVICE_ATTR(mini_isp_Led, 0660, mini_isp_Led_show,
		mini_isp_Led_store);

static ssize_t mini_isp_output_format_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 32, "mini_isp_output_format_show\n");
}

static ssize_t mini_isp_output_format_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct isp_cmd_set_output_format output_format_param = {0};
	/* Error Code*/
	ssize_t err = ERR_SUCCESS;
	u32 temp[3];
	misp_info("Set output format start!!");

	sscanf(buf, "%d %d %d", &temp[0], &temp[1], &temp[2]);
	misp_info("0x%x, 0x%x, 0x%x", temp[0], temp[1], temp[2]);
	output_format_param.depth_size = (u8)temp[0];
	output_format_param.reserve[0] = (u8)temp[1];
	output_format_param.reserve[1] = (u8)temp[2];

	err = ispctrl_if_mast_execute_cmd(ISPCMD_CAMERA_SET_OUTPUTFORMAT,
										(u8 *)&output_format_param);
	misp_info("Set output format end!!");
	return size;
}

static DEVICE_ATTR(mini_isp_output_format, 0660, mini_isp_output_format_show,
		mini_isp_output_format_store);

static ssize_t mini_isp_reg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 70,
	"Get register: g <reg addr>\nSet register: s <reg addr> <reg val>\n");
}

static ssize_t mini_isp_reg_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{

	u32 tempbuf[2];
	char ch;
	u32 reg_val = 0;
	struct misp_global_variable *dev_global_variable;

	dev_global_variable = get_mini_isp_global_variable();

	// switch to E mode
	if ((dev_global_variable->intf_status & INTF_SPI_READY) &&
		(dev_global_variable->altek_spi_mode == ALTEK_SPI_MODE_A)) {
		misp_info("%s a_to_e", __func__);
		mini_isp_a_to_e();
	}
#if 1
	sscanf(buf, "%c 0x%x 0x%x", &ch, &tempbuf[0], &tempbuf[1]);
	if(ch == 'g' || ch == 'G'){
		mini_isp_register_read(tempbuf[0], &reg_val);
		misp_info("get register 0x%x, 0x%x", tempbuf[0], reg_val);
	}else if(ch == 's' || ch == 'S'){
		mini_isp_register_write(tempbuf[0], tempbuf[1]);
		misp_info("set register 0x%x, 0x%x", tempbuf[0], tempbuf[1]);
	}else
		misp_info("input err");
#else
	if (buf[0] == '0' && buf[1] == 'x'){
		reg_str = &buf[2];
	} else {
		reg_str = &buf[0];
	}

	/* 32bit hex string to dec (little-endian)*/
	for(i = 0; i < 4; i++){
		err = hex2bin(tempbuf+3-i, reg_str+(2*i), 1);
		if(err)
			misp_info("hex2bin err: 0x%zx", err);
	}

	memcpy(&reg_addr, tempbuf, 4);
	mini_isp_register_read(reg_addr, &reg_val);
	misp_info("0x%X 0x%X", reg_addr, reg_val);
#endif
	// switch to A mode
	if ((dev_global_variable->intf_status & INTF_SPI_READY) &&
		(dev_global_variable->altek_spi_mode == ALTEK_SPI_MODE_E)) {
		misp_info("%s e_to_a", __func__);
		mini_isp_e_to_a();
	}
	return size;
}

static DEVICE_ATTR(mini_isp_reg, 0660, mini_isp_reg_show,
		mini_isp_reg_store);

ssize_t echo_mini_isp_drv_set_depth_3a_info(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	char cmd_name[20];
	struct isp_cmd_depth_3a_info depth_3a_info;
	memset(&depth_3a_info, 0, sizeof(struct isp_cmd_depth_3a_info));
	misp_info("%s S!!", __func__);

	sscanf(cmd_buf, "%s %hu %u %hu %hu %hu %hu %hu %hu %hd %hhu\
							%u %hu %hu %hu %hu %hu %hu %hd %hhu\
							%hu %hu %hd %hd %hu %hu %hhu",
	cmd_name,
	&depth_3a_info.hdr_ratio,
	&depth_3a_info.main_cam_exp_time,
	&depth_3a_info.main_cam_exp_gain,
	&depth_3a_info.main_cam_amb_r_gain,
	&depth_3a_info.main_cam_amb_g_gain,
	&depth_3a_info.main_cam_amb_b_gain,
	&depth_3a_info.main_cam_iso,
	&depth_3a_info.main_cam_bv,
	&depth_3a_info.main_cam_vcm_position,
	&depth_3a_info.main_cam_vcm_status,
	&depth_3a_info.sub_cam_exp_time,
	&depth_3a_info.sub_cam_exp_gain,
	&depth_3a_info.sub_cam_amb_r_gain,
	&depth_3a_info.sub_cam_amb_g_gain,
	&depth_3a_info.sub_cam_amb_b_gain,
	&depth_3a_info.sub_cam_iso,
	&depth_3a_info.sub_cam_bv,
	&depth_3a_info.sub_cam_vcm_position,
	&depth_3a_info.sub_cam_vcm_status,
	&depth_3a_info.main_cam_isp_d_gain,
	&depth_3a_info.sub_cam_isp_d_gain,
	&depth_3a_info.hdr_long_exp_ev_x1000,
	&depth_3a_info.hdr_short_exp_ev_x1000,
	&depth_3a_info.ghost_prevent_low,
	&depth_3a_info.ghost_prevent_high,
	&depth_3a_info.depth_proc_mode);

	misp_info("%hu %u %hu %hu %hu %hu %hu %hu %hd %hhu\
				   %u %hu %hu %hu %hu %hu %hu %hd %hhu\
				   %hu %hu %hd %hd %hu %hu %hhu",
		depth_3a_info.hdr_ratio,
		depth_3a_info.main_cam_exp_time,
		depth_3a_info.main_cam_exp_gain,
		depth_3a_info.main_cam_amb_r_gain,
		depth_3a_info.main_cam_amb_g_gain,
		depth_3a_info.main_cam_amb_b_gain,
		depth_3a_info.main_cam_iso,
		depth_3a_info.main_cam_bv,
		depth_3a_info.main_cam_vcm_position,
		depth_3a_info.main_cam_vcm_status,
		depth_3a_info.sub_cam_exp_time,
		depth_3a_info.sub_cam_exp_gain,
		depth_3a_info.sub_cam_amb_r_gain,
		depth_3a_info.sub_cam_amb_g_gain,
		depth_3a_info.sub_cam_amb_b_gain,
		depth_3a_info.sub_cam_iso,
		depth_3a_info.sub_cam_bv,
		depth_3a_info.sub_cam_vcm_position,
		depth_3a_info.sub_cam_vcm_status,
		depth_3a_info.main_cam_isp_d_gain,
		depth_3a_info.sub_cam_isp_d_gain,
		depth_3a_info.hdr_long_exp_ev_x1000,
		depth_3a_info.hdr_short_exp_ev_x1000,
		depth_3a_info.ghost_prevent_low,
		depth_3a_info.ghost_prevent_high,
		depth_3a_info.depth_proc_mode);

	mini_isp_drv_set_depth_3a_info(&depth_3a_info);
	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_set_depth_auto_interleave_mode(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	char cmd_name[20];
	u32 param[4];
	struct isp_cmd_depth_auto_interleave_param depth_auto_interleave_param;
	memset(&depth_auto_interleave_param, 0, sizeof(struct isp_cmd_depth_auto_interleave_param));
	misp_info("%s S!!", __func__);

	sscanf(cmd_buf, "%s %d %d %d %d", cmd_name, &param[0], &param[1], &param[2], &param[3]);
	depth_auto_interleave_param.depth_interleave_mode_on_off = (u8) param[0];
	depth_auto_interleave_param.skip_frame_num_after_illuminator_pulse = (u8) param[1];
	depth_auto_interleave_param.projector_power_level = (u8) param[2];
	depth_auto_interleave_param.illuminator_power_level = (u8) param[3];

	misp_info("%d, %d, %d, %d",
		depth_auto_interleave_param.depth_interleave_mode_on_off,
		depth_auto_interleave_param.skip_frame_num_after_illuminator_pulse,
		depth_auto_interleave_param.projector_power_level,
		depth_auto_interleave_param.illuminator_power_level);

	mini_isp_drv_set_depth_auto_interleave_mode(&depth_auto_interleave_param);
	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_set_exposure_param(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	char cmd_name[20];
	u32 param[3];

	struct isp_cmd_exposure_param set_exposure_param;
	memset(&set_exposure_param, 0, sizeof(struct isp_cmd_exposure_param));
	misp_info("%s S!!", __func__);

	sscanf(cmd_buf, "%s %d %d %d", cmd_name, &param[0], &param[1], &param[2]);

	set_exposure_param.udExpTime = (u32)param[0];
	set_exposure_param.uwISO = (u16)param[1];
	set_exposure_param.ucActiveDevice = (u8)param[2];

	misp_info("menu exposure param: %d %d %d",
		set_exposure_param.udExpTime,
		set_exposure_param.uwISO,
		set_exposure_param.ucActiveDevice);

	mini_isp_drv_set_exposure_param(&set_exposure_param);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_set_sensor_mode(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	char cmd_name[20];
	u8 sensor_on_off = 0;
	u8 scenario_id = 0;
	u8 mipi_tx_skew_enable = 0;
	u8 ae_weighting_table_index = 0;
	u8 merge_mode_enable = 0;
	misp_info("%s S!!", __func__);

	sscanf(cmd_buf, "%s %hhu %hhu %hhu %hhu %hhu", cmd_name, &sensor_on_off, &scenario_id, &mipi_tx_skew_enable,
						&ae_weighting_table_index, &merge_mode_enable);
	misp_info("0x%x, 0x%x, 0x%x, 0x%x, 0x%x", sensor_on_off, scenario_id, mipi_tx_skew_enable,
						ae_weighting_table_index, merge_mode_enable);

	mini_isp_drv_set_sensor_mode(sensor_on_off, scenario_id, mipi_tx_skew_enable,
									ae_weighting_table_index, merge_mode_enable);
	misp_info("%s E!!", __func__);
	return errcode;
}


ssize_t echo_mini_isp_drv_set_output_format(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	char cmd_name[20];
	u32 param[3];
	struct isp_cmd_set_output_format output_format_param;
	memset(&output_format_param, 0, sizeof(struct isp_cmd_set_output_format));
	misp_info("%s S!!", __func__);

	sscanf(cmd_buf, "%s %d %d %d", cmd_name, &param[0], &param[1], &param[2]);
	misp_info("0x%x, 0x%x, 0x%x", param[0], param[1], param[2]);
	output_format_param.depth_size = (u8)param[0];
	output_format_param.reserve[0] = (u8)param[1];
	output_format_param.reserve[1] = (u8)param[2];

	errcode = mini_isp_drv_set_output_format(&output_format_param);
	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_preview_stream_on_off(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	char cmd_name[20];
	u8 tx0_stream_on_off;
	u8 tx1_stream_on_off;
	misp_info("%s S!!", __func__);

	sscanf(cmd_buf, "%s %hhu %hhu", cmd_name, &tx0_stream_on_off, &tx1_stream_on_off);
	misp_info("0x%x, 0x%x", tx0_stream_on_off, tx1_stream_on_off);
	errcode = mini_isp_drv_preview_stream_on_off(tx0_stream_on_off, tx1_stream_on_off);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_led_power_control(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	char cmd_name[20];
	u32 param[8];
	struct isp_cmd_led_power_control projector_control_param;
	memset(&projector_control_param, 0, sizeof(struct isp_cmd_led_power_control));
	misp_info("%s S!!", __func__);

	sscanf(cmd_buf, "%s %d %d %d %d %d %d %d %d",
		cmd_name, &param[0], &param[1], &param[2], &param[3], &param[4], &param[5],
		&param[6], &param[7]);

	projector_control_param.led_on_off =		  (u8)param[0];
	projector_control_param.led_power_level =	 (u8)param[1];
	projector_control_param.control_projector_id =(u8)param[2];
	projector_control_param.delay_after_sof =		 param[3];
	projector_control_param.pulse_time =			  param[4];
	projector_control_param.control_mode =		(u8)param[5];
	projector_control_param.pulse_mode_skip_frame=(u8)param[6];
	projector_control_param.rolling_shutter =	 (u8)param[7];

	misp_info("%d, %d, %d, %d, %d, %d, %d, %d",
		projector_control_param.led_on_off,
		projector_control_param.led_power_level,
		projector_control_param.control_projector_id,
		projector_control_param.delay_after_sof,
		projector_control_param.pulse_time,
		projector_control_param.control_mode,
		projector_control_param.pulse_mode_skip_frame,
		projector_control_param.rolling_shutter);

	errcode = mini_isp_drv_led_power_control(&projector_control_param);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_active_ae(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	char cmd_name[20];
	u32 param[2];
	struct isp_cmd_active_ae active_ae_param;
	memset(&active_ae_param, 0, sizeof(struct isp_cmd_active_ae));
	misp_info("%s S!!", __func__);

	sscanf(cmd_buf, "%s %d %d", cmd_name, &param[0], &param[1]);

	active_ae_param.active_ae = (u8)param[0];
	active_ae_param.f_number_x1000 = param[1];

	misp_info("%d, %d",
		active_ae_param.active_ae,
		active_ae_param.f_number_x1000);

	errcode = mini_isp_drv_active_ae(&active_ae_param);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_isp_ae_control_mode_on_off(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	char cmd_name[20];
	u32 param;
	misp_info("%s S!!", __func__);

	sscanf(cmd_buf, "%s %d", cmd_name, &param);

	misp_info("%d", param);

	errcode = mini_isp_drv_isp_ae_control_mode_on_off(param);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_set_max_exposure(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	char cmd_name[20];
	u32 param[3];

	struct isp_cmd_exposure_param set_max_exposure;
	memset(&set_max_exposure, 0, sizeof(struct isp_cmd_exposure_param));
	misp_info("%s S!!", __func__);

	sscanf(cmd_buf, "%s %d %d %d", cmd_name, &param[0], &param[1], &param[2]);

	set_max_exposure.udExpTime = (u32)param[0];
	set_max_exposure.uwISO = (u16)param[1];
	set_max_exposure.ucActiveDevice = (u8)param[2];

	misp_info("max exposure param: %d %d %d",
		set_max_exposure.udExpTime,
		set_max_exposure.uwISO,
		set_max_exposure.ucActiveDevice);

	mini_isp_drv_set_max_exposure(&set_max_exposure);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_frame_sync_control(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	char cmd_name[20];
	u32 param[5];
	struct isp_cmd_frame_sync_control frame_sync_control_param;
	memset(&frame_sync_control_param, 0, sizeof(struct isp_cmd_frame_sync_control));
	misp_info("%s S!!", __func__);

	sscanf(cmd_buf, "%s %d %d %d %d %d", cmd_name,
		&param[0], &param[1], &param[2], &param[3], &param[4]);

	frame_sync_control_param.control_deviceID = (u8) param[0];
	frame_sync_control_param.delay_framephase = (u8) param[1];
	frame_sync_control_param.active_framephase= (u8) param[2];
	frame_sync_control_param.deactive_framephase=(u8)param[3];
	frame_sync_control_param.active_timelevel = (u8) param[4];

	misp_info("%d %d %d %d %d",
		frame_sync_control_param.control_deviceID,
		frame_sync_control_param.delay_framephase,
		frame_sync_control_param.active_framephase,
		frame_sync_control_param.deactive_framephase,
		frame_sync_control_param.active_timelevel);

	errcode = mini_isp_drv_frame_sync_control(&frame_sync_control_param);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_set_shot_mode(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	char cmd_name[20];
	u32 param[2];
	struct isp_cmd_set_shot_mode set_shot_mode_param;
	memset(&set_shot_mode_param, 0, sizeof(struct isp_cmd_set_shot_mode));
	misp_info("%s S!!", __func__);

	sscanf(cmd_buf, "%s %d %d", cmd_name, &param[0], &param[1]);

	set_shot_mode_param.shot_mode = (u8) param[0];
	set_shot_mode_param.frame_rate = (u16) param[1];

	misp_info("%d %d",
		set_shot_mode_param.shot_mode,
		set_shot_mode_param.frame_rate);

	errcode = mini_isp_drv_set_shot_mode(&set_shot_mode_param);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_lighting_ctrl(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	char cmd_name[20];
	u8 i = 0;
	u32 param[25];
	struct isp_cmd_lighting_ctrl lighting_ctrl;
	memset(&lighting_ctrl, 0, sizeof(struct isp_cmd_lighting_ctrl));
	misp_info("%s S!!", __func__);

	sscanf(cmd_buf, "%s %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\
		%d %d %d %d %d",
		cmd_name, &param[0], &param[1], &param[2], &param[3], &param[4], &param[5], &param[6],
		&param[7], &param[8], &param[9], &param[10], &param[11], &param[12], &param[13], &param[14],
		&param[15], &param[16], &param[17], &param[18], &param[19], &param[20], &param[21], &param[22],
		&param[23], &param[24]);
	lighting_ctrl.cycle_len = param[0];
	for(i = 0; i < lighting_ctrl.cycle_len; i++) {
		lighting_ctrl.cycle[i].source = (u8)param[i*3+1];
		lighting_ctrl.cycle[i].TxDrop = (u8)param[i*3+2];
		lighting_ctrl.cycle[i].co_frame_rate = (u16)param[i*3+3];
	}

	misp_info("cycle_len: %d", lighting_ctrl.cycle_len);
	for(i = 0; i < lighting_ctrl.cycle_len; i++) {
		misp_info("cycle[%d]: %d, %d, %d", i, lighting_ctrl.cycle[i].source,
			   lighting_ctrl.cycle[i].TxDrop, lighting_ctrl.cycle[i].co_frame_rate);
	}

	errcode = mini_isp_drv_lighting_ctrl(&lighting_ctrl);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_depth_compensation(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	char cmd_name[20];
	u32 param[4];
	u8 en_update = 0;
	struct isp_cmd_depth_compensation_param depth_compensation_param;
	memset(&depth_compensation_param, 0, sizeof(struct isp_cmd_depth_compensation_param));
	misp_info("%s S!!", __func__);

	sscanf(cmd_buf, "%s %d %d %d %d", cmd_name, &param[0], &param[1], &param[2], &param[3]);

	en_update = (param[0] << 4) | param[1];
	depth_compensation_param.en_updated = (u8)en_update;
	depth_compensation_param.short_distance_value = (u16)param[2];
	depth_compensation_param.compensation = (s8)param[3];

	misp_info("0x%x %d %d",
		depth_compensation_param.en_updated,
		depth_compensation_param.short_distance_value,
		depth_compensation_param.compensation);

	mini_isp_drv_depth_compensation(&depth_compensation_param);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_cycle_trigger_depth_process(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	char cmd_name[20];
	u32 param[3];

	struct isp_cmd_cycle_trigger_depth_process depth_cycle_param;
	memset(&depth_cycle_param, 0, sizeof(struct isp_cmd_cycle_trigger_depth_process));
	misp_info("%s S!!", __func__);

	sscanf(cmd_buf, "%s %d %d %d", cmd_name, &param[0], &param[1], &param[2]);

	depth_cycle_param.cycleLen= (u8)param[0];
	depth_cycle_param.depth_triggerBitField= (u16)param[1];
	depth_cycle_param.depthoutput_triggerBitField= (u16)param[2];

	misp_info("depth cycle len: 0%d %d %d",
		depth_cycle_param.cycleLen,
		depth_cycle_param.depth_triggerBitField,
		depth_cycle_param.depthoutput_triggerBitField);

	mini_isp_drv_cycle_trigger_depth_process(&depth_cycle_param);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_set_min_exposure(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	char cmd_name[20];
	u32 param[3];

	struct isp_cmd_exposure_param set_min_exposure;
	memset(&set_min_exposure, 0, sizeof(struct isp_cmd_exposure_param));
	misp_info("%s S!!", __func__);

	sscanf(cmd_buf, "%s %d %d %d", cmd_name, &param[0], &param[1], &param[2]);

	set_min_exposure.udExpTime = (u32)param[0];
	set_min_exposure.uwISO = (u16)param[1];
	set_min_exposure.ucActiveDevice = (u8)param[2];

	misp_info("min exposure param: %d %d %d",
		set_min_exposure.udExpTime,
		set_min_exposure.uwISO,
		set_min_exposure.ucActiveDevice);

	mini_isp_drv_set_min_exposure(&set_min_exposure);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_set_max_exposure_slope(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	char cmd_name[20];
	u32 param[2];

	struct isp_cmd_max_exposure_slope max_exposure_slope;
	memset(&max_exposure_slope, 0, sizeof(struct isp_cmd_max_exposure_slope));
	misp_info("%s S!!", __func__);

	sscanf(cmd_buf, "%s %d %d", cmd_name, &param[0], &param[1]);

	max_exposure_slope.max_exposure_slope= (u32)param[0];
	max_exposure_slope.ucActiveDevice= (u8)param[1];

	misp_info("max exposure slope: %d %d",
		max_exposure_slope.max_exposure_slope,
		max_exposure_slope.ucActiveDevice);

	mini_isp_drv_set_max_exposure_slope(&max_exposure_slope);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_led_active_delay(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	char cmd_name[20];
	u32 delay_ms;

	misp_info("%s S!!", __func__);

	sscanf(cmd_buf, "%s %d", cmd_name, &delay_ms);

	misp_info("[delay]: %d", delay_ms);

	mini_isp_drv_led_active_delay(delay_ms);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_get_comlog(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	struct common_log_hdr_info info;
	memset(&info, 0, sizeof(struct common_log_hdr_info));
	misp_info("%s S!!", __func__);

	info.block_size = SPI_TX_BULK_SIZE;
	info.total_size = LEVEL_LOG_BUFFER_SIZE;
	ispctrl_if_mast_execute_cmd(ISPCMD_BULK_READ_COMLOG, (u8 *)&info);
	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_set_register(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	char cmd_name[20];
	u32 param[2];
	struct misp_global_variable *dev_global_variable;
	dev_global_variable = get_mini_isp_global_variable();

	misp_info("%s S", __func__);
	// switch to E mode
	if ((dev_global_variable->intf_status & INTF_SPI_READY) &&
		(dev_global_variable->altek_spi_mode == ALTEK_SPI_MODE_A)) {
		misp_info("%s a_to_e", __func__);
		mini_isp_a_to_e();
	}

	sscanf(cmd_buf, "%s 0x%x 0x%x", cmd_name, &param[0], &param[1]);
	mini_isp_register_write(param[0], param[1]);
	misp_info("set register 0x%x, 0x%x", param[0], param[1]);

	// switch to A mode
	if ((dev_global_variable->intf_status & INTF_SPI_READY) &&
		(dev_global_variable->altek_spi_mode == ALTEK_SPI_MODE_E)) {
		misp_info("%s e_to_a", __func__);
		mini_isp_e_to_a();
	}
	misp_info("%s E", __func__);
	return errcode;
}

ssize_t echo_get_register(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	char cmd_name[20];
	u32 param;
	u32 reg_val;
	struct misp_global_variable *dev_global_variable;
	dev_global_variable = get_mini_isp_global_variable();

	misp_info("%s S", __func__);
	// switch to E mode
	if ((dev_global_variable->intf_status & INTF_SPI_READY) &&
		(dev_global_variable->altek_spi_mode == ALTEK_SPI_MODE_A)) {
		misp_info("%s a_to_e", __func__);
		mini_isp_a_to_e();
	}

	sscanf(cmd_buf, "%s 0x%x", cmd_name, &param);
	mini_isp_register_read(param, &reg_val);
	misp_info("get register 0x%x, 0x%x", param, reg_val);

	// switch to A mode
	if ((dev_global_variable->intf_status & INTF_SPI_READY) &&
		(dev_global_variable->altek_spi_mode == ALTEK_SPI_MODE_E)) {
		misp_info("%s e_to_a", __func__);
		mini_isp_e_to_a();
	}
	misp_info("%s E", __func__);
	return errcode;
}

ssize_t echo_memdump(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	char cmd_name[2];
	char filename[80];
	u32 param[2];
	struct misp_global_variable *dev_global_variable;
	dev_global_variable = get_mini_isp_global_variable();

	misp_info("%s S", __func__);
	// switch to E mode
	if ((dev_global_variable->intf_status & INTF_SPI_READY) &&
		(dev_global_variable->altek_spi_mode == ALTEK_SPI_MODE_A)) {
		misp_info("%s a_to_e", __func__);
		mini_isp_a_to_e();
	}

	sscanf(cmd_buf, "%s 0x%x %d", cmd_name, &param[0], &param[1]);
	misp_info("Get mem 0x%x, %d\n", param[0], param[1]);

	snprintf(filename, 80, "memdump_0x%x_%d", param[0], param[1]);
	mini_isp_memory_read_then_write_file(param[0], param[1],
	    MINIISP_INFO_DUMPLOCATION, filename);

	// switch to A mode
	if ((dev_global_variable->intf_status & INTF_SPI_READY) &&
		(dev_global_variable->altek_spi_mode == ALTEK_SPI_MODE_E)) {
		misp_info("%s e_to_a", __func__);
		mini_isp_e_to_a();
	}
	misp_info("%s E", __func__);
	return errcode;
}

ssize_t echo_show_version(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	misp_info("MINIISP_DRIVER_VERSION: %s", MINIISP_DRIVER_VERSION);
	misp_info("AL6100 fw ver: %05d.%05d, build by %s",
	    fw_version_before_point, fw_version_after_point, fw_build_by);
	misp_info("SC table build data: %d", sc_build_date);
	return errcode;
}

ssize_t echo_set_fsm_status(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	u32 param;
	char cmd_name[20];
	struct misp_global_variable *dev_global_variable;
	dev_global_variable = get_mini_isp_global_variable();

	sscanf(cmd_buf, "%s %d", cmd_name, &param);
	dev_global_variable->now_state = param;
	misp_info("set fsm status: %d", dev_global_variable->now_state);
	return errcode;
}

ssize_t echo_cfg_cmd_send(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	u32 param;
	char cmd_name[20];
	struct misp_global_variable *dev_global_variable;
	dev_global_variable = get_mini_isp_global_variable();

	sscanf(cmd_buf, "%s %d", cmd_name, &param);
	dev_global_variable->en_cmd_send= param;
	misp_info("set en_cmd_send: %d", dev_global_variable->en_cmd_send);
	return errcode;
}

ssize_t echo_mini_isp_a_to_e(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	misp_info("%s, S", __func__);
	mini_isp_a_to_e();
	misp_info("%s, E", __func__);
	return errcode;
}

ssize_t echo_mini_isp_e_to_a(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	misp_info("%s, S", __func__);
	mini_isp_e_to_a();
	misp_info("%s, E", __func__);
	return errcode;
}

ssize_t echo_mini_isp_chip_init(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	misp_info("%s, S", __func__);
	mini_isp_chip_init();
	misp_info("%s, E", __func__);
	return errcode;
}

ssize_t echo_mini_isp_debug_dump_img(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	misp_info("%s, S", __func__);
	errcode = mini_isp_debug_dump_img();
	misp_info("%s, E", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_set_bypass_mode(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	u16 param;
	char cmd_name[20];
	misp_info("%s S", __func__);
	sscanf(cmd_buf, "%s %hu", cmd_name, &param);
	if(param == 0)
		param = 1;
	errcode = mini_isp_drv_set_bypass_mode(param);
	misp_info("%s E", __func__);
	return errcode;
}

ssize_t echo_mini_isp_utility_read_reg_e_mode_for_bypass_use(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	misp_info("%s S", __func__);
	errcode = mini_isp_utility_read_reg_e_mode_for_bypass_use();
	misp_info("%s E", __func__);
	return errcode;
}

ssize_t echo_mini_isp_utility_read_reg_e_mode(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	misp_info("%s S", __func__);
	errcode = mini_isp_utility_read_reg_e_mode();
	misp_info("%s E", __func__);
	return errcode;
}

ssize_t echo_mini_isp_debug_packdata_dump(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	misp_info("%s S", __func__);
	errcode = mini_isp_debug_packdata_dump();
	misp_info("%s E", __func__);
	return errcode;
}

ssize_t echo_mini_isp_debug_IQCalib_dump(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	misp_info("%s S", __func__);
	errcode = mini_isp_debug_IQCalib_dump();
	misp_info("%s E", __func__);
	return errcode;
}

ssize_t echo_mini_isp_debug_metadata_dump(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	misp_info("%s S", __func__);
	errcode = mini_isp_debug_metadata_dump();
	misp_info("%s E", __func__);
	return errcode;
}



ssize_t echo_mini_isp_debug_rect_combo_dump(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	u8 param;
	char cmd_name[20];
	misp_info("%s S", __func__);
	sscanf(cmd_buf, "%s %hhu", cmd_name, &param);
	misp_info("dump mode %hhu", param);
	errcode = mini_isp_debug_depth_rect_combo_dump(param);
	misp_info("%s E", __func__);
	return errcode;
}

ssize_t echo_mini_isp_debug_depth_info(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	misp_info("%s S", __func__);
	errcode = mini_isp_debug_depth_info();
	misp_info("%s E", __func__);
	return errcode;
}


ssize_t echo_mini_isp_debug_metadata_info(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	misp_info("%s S", __func__);
	errcode = mini_isp_debug_metadata_info();
	misp_info("%s E", __func__);
	return errcode;
}

ssize_t echo_mini_isp_debug_sensor_info(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	misp_info("%s S", __func__);
	errcode = mini_isp_debug_sensor_info();
	misp_info("%s E", __func__);
	return errcode;
}

ssize_t echo_mini_isp_debug_led_info(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	misp_info("%s S", __func__);
	errcode = mini_isp_debug_led_info();
	misp_info("%s E", __func__);
	return errcode;
}


#include <linux/namei.h>
#include <linux/fcntl.h>
ssize_t echo_test(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
#if 1
	struct dentry *dentry;
	struct path path;
	int err;
	misp_info("%s S", __func__);

	dentry = kern_path_create(AT_FDCWD, "/data/local/tmp/test/",
	        &path, LOOKUP_DIRECTORY);
	if (IS_ERR(dentry))
		return PTR_ERR(dentry);

	err = vfs_mkdir(path.dentry->d_inode, dentry, 0777);

	done_path_create(&path, dentry);

	misp_info("%s, E", __func__);
#endif
	return errcode;
}

struct echo_cmd_format{
	u32 opcode;
	char cmd_name[30];
	ssize_t (*pfunc)(const char *cmd_buf);
};

struct echo_cmd_format echo_cmd_list[] = {
	{.opcode = 0x10B9,  .cmd_name = "depth_3a_info",.pfunc = echo_mini_isp_drv_set_depth_3a_info},
	{.opcode = 0x10BC,  .cmd_name = "interleave",   .pfunc = echo_mini_isp_drv_set_depth_auto_interleave_mode},
	{.opcode = 0x10BF,  .cmd_name = "set_menu_exposure",   .pfunc = echo_mini_isp_drv_set_exposure_param},
	{.opcode = 0x300A,  .cmd_name = "set_sensor",   .pfunc = echo_mini_isp_drv_set_sensor_mode},
	{.opcode = 0x300D,  .cmd_name = "output_format",.pfunc = echo_mini_isp_drv_set_output_format},
	{.opcode = 0x3010,  .cmd_name = "streamon",	 .pfunc = echo_mini_isp_drv_preview_stream_on_off},
	{.opcode = 0x3012,  .cmd_name = "led_power",	.pfunc = echo_mini_isp_drv_led_power_control},
	{.opcode = 0x3013,  .cmd_name = "active_ae",	.pfunc = echo_mini_isp_drv_active_ae},
	{.opcode = 0x3014,  .cmd_name = "ae_onoff",	 .pfunc = echo_mini_isp_drv_isp_ae_control_mode_on_off},
	{.opcode = 0x3017,  .cmd_name = "set_max_exposure",	 .pfunc = echo_mini_isp_drv_set_max_exposure},
	{.opcode = 0x3019,  .cmd_name = "frame_sync",   .pfunc = echo_mini_isp_drv_frame_sync_control},
	{.opcode = 0x301A,  .cmd_name = "shot_mode",	.pfunc = echo_mini_isp_drv_set_shot_mode},
	{.opcode = 0x301B,  .cmd_name = "lighting_ctrl",.pfunc = echo_mini_isp_drv_lighting_ctrl},
	{.opcode = 0x301C,  .cmd_name = "depth_compensation",.pfunc = echo_mini_isp_drv_depth_compensation},
	{.opcode = 0x301D,  .cmd_name = "cycle_trigger_depth",.pfunc = echo_mini_isp_drv_cycle_trigger_depth_process},
	{.opcode = 0x301E,  .cmd_name = "set_min_exposure",.pfunc = echo_mini_isp_drv_set_min_exposure},
	{.opcode = 0x301F,  .cmd_name = "set_max_exposure_slop",.pfunc = echo_mini_isp_drv_set_max_exposure_slope},
	{.opcode = 0x3020,  .cmd_name = "led_active_delay",.pfunc = echo_mini_isp_drv_led_active_delay},
	{.opcode = 0xFFFF,  .cmd_name = "comlog",	   .pfunc = echo_get_comlog},
	{.opcode = 0xFFFF,  .cmd_name = "setreg",	   .pfunc = echo_set_register},
	{.opcode = 0xFFFF,  .cmd_name = "getreg",	   .pfunc = echo_get_register},
	{.opcode = 0xFFFF,  .cmd_name = "memdump",	  .pfunc = echo_memdump},
	{.opcode = 0xFFFF,  .cmd_name = "version",	  .pfunc = echo_show_version},
	{.opcode = 0xFFFF,  .cmd_name = "set_fsm_status",.pfunc = echo_set_fsm_status},
	{.opcode = 0xFFFF,  .cmd_name = "cfg_cmd_send", .pfunc = echo_cfg_cmd_send},
	{.opcode = 0xFFFF,  .cmd_name = "a2e",		  .pfunc = echo_mini_isp_a_to_e},
	{.opcode = 0xFFFF,  .cmd_name = "e2a",		  .pfunc = echo_mini_isp_e_to_a},
	{.opcode = 0xFFFF,  .cmd_name = "chip_init",	.pfunc = echo_mini_isp_chip_init},
	{.opcode = 0xFFFF,  .cmd_name = "pure_bypass",  .pfunc = echo_mini_isp_drv_set_bypass_mode},
	{.opcode = 0xFFFF,  .cmd_name = "dump_bypass_reg",  .pfunc = echo_mini_isp_utility_read_reg_e_mode_for_bypass_use},
	{.opcode = 0xFFFF,  .cmd_name = "dump_normal_reg",  .pfunc = echo_mini_isp_utility_read_reg_e_mode},
	{.opcode = 0xFFFF,  .cmd_name = "dump_packdata",  .pfunc = echo_mini_isp_debug_packdata_dump},
	{.opcode = 0xFFFF,  .cmd_name = "dump_IQCalib",  .pfunc = echo_mini_isp_debug_IQCalib_dump},
	{.opcode = 0xFFFF,  .cmd_name = "dump_Meta",  .pfunc = echo_mini_isp_debug_metadata_dump},
	{.opcode = 0xFFFF,  .cmd_name = "dump_irp_img", .pfunc = echo_mini_isp_debug_dump_img},
	{.opcode = 0xFFFF,  .cmd_name = "dump_depth_reg",  .pfunc = echo_mini_isp_debug_rect_combo_dump},
	{.opcode = 0xFFFF,  .cmd_name = "debug_depth_info",  .pfunc = echo_mini_isp_debug_depth_info},
	{.opcode = 0xFFFF,  .cmd_name = "debug_metadata_info",  .pfunc = echo_mini_isp_debug_metadata_info},
	{.opcode = 0xFFFF,  .cmd_name = "debug_sensor_info",  .pfunc = echo_mini_isp_debug_sensor_info},
	{.opcode = 0xFFFF,  .cmd_name = "debug_led_info",  .pfunc = echo_mini_isp_debug_led_info},
	{.opcode = 0xFFFF,  .cmd_name = "test",         .pfunc = echo_test},
};

#define echo_cmd_list_len (sizeof(echo_cmd_list)/ sizeof(struct echo_cmd_format))

static ssize_t mini_isp_cmd_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/* print cmd list */
	u32 total_len = 0, count = 0;
	u32 i = 0;
	for (i=0; i < echo_cmd_list_len; i++) {
		count = snprintf(buf, 70, "%s\n", echo_cmd_list[i].cmd_name);
		buf += count; /* move buffer pointer */
		total_len += count;
	}

	return total_len;
}

static ssize_t mini_isp_cmd_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	char cmd_name[20];
	u32 opcode = 0x0;
	u32 err = ERR_SUCCESS;
	u32 loopidx = 0;
	sscanf(buf, "%s", cmd_name);
	// check first input is opcode or cmd_name
	if(cmd_name[0] == '0' && cmd_name[1] == 'x')
		sscanf(cmd_name, "0x%x", &opcode);

	for(loopidx = 0; loopidx < echo_cmd_list_len; loopidx++){
		if(echo_cmd_list[loopidx].opcode == opcode ||
			strcmp(echo_cmd_list[loopidx].cmd_name, cmd_name) == 0)
		{
			err = echo_cmd_list[loopidx].pfunc(buf);
			if(err)
				misp_info("%s, err: 0x%x", __func__, err);

			return size;
		}
	}

	misp_info("command not find!");
	return size;
}

static DEVICE_ATTR(mini_isp_cmd, 0660, mini_isp_cmd_show,
		mini_isp_cmd_store);

/************************************************************
*					Public Function						*
*************************************************************/

struct misp_data *get_mini_isp_intf(int i2c_type)
{
	if (misp_drv_global_variable->intf_status & INTF_SPI_READY) {
		return get_mini_isp_intf_spi();
	} else if (misp_drv_global_variable->intf_status & INTF_I2C_READY) {
		return get_mini_isp_intf_i2c(i2c_type);
	/*} else if (misp_drv_global_variable->intf_status & INTF_CCI_READY) {
		return get_mini_isp_intf_cci(i2c_type);*/
	} else {
		misp_err("%s - error i2c type %d", __func__, i2c_type);
		return NULL;
	}
}
void set_mini_isp_data(struct misp_data *data, int intf_type)
{
	if (!misp_drv_global_variable)
		misp_err("%s - set global_variable error", __func__);
	else
		misp_drv_global_variable->intf_status |= intf_type;
}

struct misp_global_variable *get_mini_isp_global_variable(void)
{
	if (!misp_drv_global_variable) {
		misp_err("%s - get global_variable error", __func__);
		return NULL;
	} else {
		return misp_drv_global_variable;
	}
}

struct altek_statefsm *get_mini_isp_fsm(void)
{
	if (!altek_state) {
		misp_err("%s - get fsm error", __func__);
		return NULL;
	} else {
		return altek_state;
	}
}


int mini_isp_setup_resource(struct device *dev, struct misp_data *drv_data)
{
	int status = 0;

	misp_info("%s - start", __func__);
	if (misp_drv_global_variable != NULL) {
		misp_err("%s - resource already been setupped", __func__);
		goto setup_done;
	}

	/*step 1: alloc misp_drv_global_variable*/
	misp_drv_global_variable =
		kzalloc(sizeof(*misp_drv_global_variable), GFP_KERNEL);

	if (!misp_drv_global_variable) {
		misp_info("%s - Out of memory", __func__);
		status = -ENOMEM;
		goto alloc_fail;
	}
	misp_info("%s - step1 done.", __func__);

	/*step 2: init mutex and gpio resource*/
	mutex_init(&misp_drv_global_variable->busy_lock);
	status = mini_isp_gpio_init(dev, drv_data, misp_drv_global_variable);
	if (status < 0) {
		misp_info("%s - gpio init fail", __func__);
		goto setup_fail;
	}
	misp_info("%s - step2 done.", __func__);

	misp_drv_global_variable->before_booting = 1;
	misp_drv_global_variable->en_cmd_send = 1;

	/*step 3: register to VFS as character device*/
	mini_isp_class = class_create(THIS_MODULE, "mini_isp");
	if (IS_ERR(mini_isp_class))
		misp_err("Failed to create class(mini_isp_class)!");
	mini_isp_dev = miniisp_chdev_create(mini_isp_class);

	if (IS_ERR(mini_isp_dev))
		misp_err("Failed to create device(mini_isp_dev)!");

	status = device_create_file(mini_isp_dev,
				&dev_attr_mini_isp_mode_config);

	if (status < 0)
		misp_err("Failed to create device file(%s)!",
			dev_attr_mini_isp_mode_config.attr.name);

	if (RESET_GPIO != NULL) {
		status = device_create_file(mini_isp_dev,
						&dev_attr_mini_isp_reset);

		if (status < 0)
			misp_err("Failed to create device file(%s)!",
				dev_attr_mini_isp_reset.attr.name);
	}

	status = device_create_file(mini_isp_dev, &dev_attr_mini_isp_poweron);
	if (status < 0)
		misp_err("Failed to create device file(%s)!",
			dev_attr_mini_isp_poweron.attr.name);

	status = device_create_file(mini_isp_dev, &dev_attr_mini_isp_dump_reg);
	if (status < 0)
		misp_err("Failed to create device file(%s)!",
			dev_attr_mini_isp_dump_reg.attr.name);

	status = device_create_file(mini_isp_dev,
			&dev_attr_mini_isp_frame_sync_control);
	if (status < 0)
		misp_err("Failed to create device file(%s)!",
			dev_attr_mini_isp_frame_sync_control.attr.name);

	status = device_create_file(mini_isp_dev, &dev_attr_mini_isp_lighting_ctrl);
	if (status < 0)
		misp_err("Failed to create device file(%s)!",
			dev_attr_mini_isp_lighting_ctrl.attr.name);

	status = device_create_file(mini_isp_dev,
			&dev_attr_mini_isp_exposure_param);
	if (status < 0)
		misp_err("Failed to create device file(%s)!",
			dev_attr_mini_isp_exposure_param.attr.name);

	status = device_create_file(mini_isp_dev,
			&dev_attr_mini_isp_max_exposure);
	if (status < 0)
		misp_err("Failed to create device file(%s)!",
			dev_attr_mini_isp_max_exposure.attr.name);

	status = device_create_file(mini_isp_dev,
			&dev_attr_mini_isp_min_exposure);
	if (status < 0)
		misp_err("Failed to create device file(%s)!",
			dev_attr_mini_isp_min_exposure.attr.name);

	status = device_create_file(mini_isp_dev,
			&dev_attr_mini_isp_max_exposure_slope);
	if (status < 0)
		misp_err("Failed to create device file(%s)!",
			dev_attr_mini_isp_max_exposure_slope.attr.name);

	status = device_create_file(mini_isp_dev,
			&dev_attr_mini_isp_target_mean);
	if (status < 0)
		misp_err("Failed to create device file(%s)!",
			dev_attr_mini_isp_target_mean.attr.name);

	status = device_create_file(mini_isp_dev,
			&dev_attr_mini_isp_rectab);
	if (status < 0)
		misp_err("Failed to create device file(%s)!",
			dev_attr_mini_isp_rectab.attr.name);

	status = device_create_file(mini_isp_dev,
			&dev_attr_mini_isp_depth_compensation);
	if (status < 0)
		misp_err("Failed to create device file(%s)!",
			dev_attr_mini_isp_depth_compensation.attr.name);

	status = device_create_file(mini_isp_dev,
			&dev_attr_mini_isp_cycle_trigger_depth);
	if (status < 0)
		misp_err("Failed to create device file(%s)!",
			dev_attr_mini_isp_cycle_trigger_depth.attr.name);

	status = device_create_file(mini_isp_dev,
			&dev_attr_mini_isp_comlog);
	if (status < 0)
		misp_err("Failed to create device file(%s)!",
			dev_attr_mini_isp_comlog.attr.name);

	status = device_create_file(mini_isp_dev,
			&dev_attr_mini_isp_mem_dump);
	if (status < 0)
		misp_err("Failed to create device file(%s)!",
			dev_attr_mini_isp_mem_dump.attr.name);

	status = device_create_file(mini_isp_dev,
			&dev_attr_mini_isp_Led);
	if (status < 0)
		misp_err("Failed to create device file(%s)!",
			dev_attr_mini_isp_Led.attr.name);

	status = device_create_file(mini_isp_dev,
		&dev_attr_mini_isp_output_format);
	if (status < 0)
		misp_err("Failed to create device file(%s)!",
			dev_attr_mini_isp_output_format.attr.name);

	status = device_create_file(mini_isp_dev,
		&dev_attr_mini_isp_reg);
	if (status < 0)
		misp_err("Failed to create device file(%s)!",
			dev_attr_mini_isp_reg.attr.name);

	status = device_create_file(mini_isp_dev,
		&dev_attr_mini_isp_cmd);
	if (status < 0)
		misp_err("Failed to create device file(%s)!",
			dev_attr_mini_isp_cmd.attr.name);

	misp_info("%s - step3 done.", __func__);

	misp_info("%s - success.", __func__);
	goto setup_done;

setup_fail:
	mutex_destroy(&misp_drv_global_variable->busy_lock);
	kfree(misp_drv_global_variable);

alloc_fail:
	misp_drv_global_variable = NULL;

setup_done:
	return status;
}

static int __init mini_isp_init(void)
{
	int ret = 0;
	struct altek_statefsm *fsm = NULL;

	misp_info("%s - start", __func__);

	fsm = altek_statefsmcreate();
	altek_state = fsm;
	misp_info("MINIISP_DRIVER_VERSION: %s", MINIISP_DRIVER_VERSION);
	misp_info("%s - success", __func__);

	return ret;
}

static void __exit mini_isp_exit(void)
{
	misp_info("%s", __func__);

	if (misp_drv_global_variable->irq_gpio)
		gpio_free(misp_drv_global_variable->irq_gpio);

	/*if (misp_drv_global_variable)*/
		kfree(misp_drv_global_variable);

	altek_statefsmdelete(altek_state);
	altek_state = NULL;
}

module_init(mini_isp_init);
module_exit(mini_isp_exit);
MODULE_LICENSE("Dual BSD/GPL");

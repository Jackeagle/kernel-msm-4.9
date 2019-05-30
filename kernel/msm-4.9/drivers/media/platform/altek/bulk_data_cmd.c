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
 * File:  bulk_data_cmd.c
 * Description: Mini ISP sample codes
 *
 *
 *  2013/10/14; Bruce Chung; Initial version
 *  2013/12/05; Bruce Chung; 2nd version
 *  2016/05/05; Louis Wang; Linux Coding Style
 */

/******Include File******/
#include <linux/buffer_head.h>
#include <linux/delay.h>

#include "include/isp_camera_cmd.h"
#include "include/ispctrl_if_master.h"
#include "include/error/ispctrl_if_master_err.h"
#include "include/miniisp.h"
#include "include/ispctrl_if_master_local.h"
#include "include/miniisp_customer_define.h"
/******Private Constant Definition******/
#define LOGSIZE  (4*1024)
//#define BLOCKSIZE SPI_TX_BULK_SIZE
#define RAWBLOCKSIZE SPI_TX_BULK_SIZE_BOOT
#define MINI_ISP_LOG_TAG	"[[miniisp]bulk_data_cmd]"
#define MID_PJ_EXEBIN_BUF (1024*1024)

/*Private Type Declaration*/
/*Basic code buffer address*/
static u8 *basic_code_buf_addr;
/*Calibration data buffer address*/
static u8 *calibration_data_buf_addr;

u16 fw_version_before_point;
u16 fw_version_after_point;
char fw_build_by[9];
u32 sc_build_date;

/******Private Function Prototype******/
static u16 calculate_check_sum(u8 *input_buffer_addr, u16 input_buffer_size);

/******Private Global Variable******/


/******Public Global Variable*******/

/******Public Function******/


/**
 *\brief Write Boot Code
 *\param devdata [In], misp_data
 *\param param [In], CMD param
 *\param filp [In], boot code file pointer
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_boot_code(void *devdata,
						u8 *param, struct file *filp)
{
	errcode err = ERR_SUCCESS;
	u32 total_size;
	off_t currpos;

	if (filp == NULL) {
		misp_err("%s - file didn't exist.", __func__);
		err = ~ERR_SUCCESS;
		goto mast_bulk_data_cmd_write_boot_code_end;
	}

	/*get the file size*/
	currpos = vfs_llseek(filp, 0L, SEEK_END);
	if (currpos == -1) {
		misp_err("%s  llseek failed", __func__);
		err = ~ERR_SUCCESS;
		goto mast_bulk_data_cmd_write_boot_code_end;
	}
	total_size = (u32)currpos;
	/*misp_info("%s  filesize : %d", __func__, total_size);*/
	vfs_llseek(filp, 0L, SEEK_SET);

	/*misp_info("block_size %d", RAWBLOCKSIZE);*/

	/*Transfer boot code*/
	/* boot code & main code can only be sent by SPI */
	err = ispctrl_if_mast_send_bulk(devdata,
		basic_code_buf_addr, filp, total_size, RAWBLOCKSIZE, true);

	if (err != ERR_SUCCESS)
		goto mast_bulk_data_cmd_write_boot_code_end;

	misp_info("%s send boot code success", __func__);

mast_bulk_data_cmd_write_boot_code_end:

	return err;
}


/**
 *\brief Write Boot Code (Short SPI Len)
 *\param devdata [In], misp_data
 *\param param [In], CMD param
 *\param filp [In], boot code file pointer
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_boot_code_shortlen(void *devdata,
						u8 *param, struct file *filp)
{
	errcode err = ERR_SUCCESS;
	u32 total_size;
	u32 block_size;
	off_t currpos;

	block_size = SPI_BLOCK_LEN;

	if (filp == NULL) {
		misp_err("%s - file didn't exist.", __func__);
		err = ~ERR_SUCCESS;
		goto mast_bulk_data_cmd_write_boot_code_end;
	}

	/*get the file size*/
	currpos = vfs_llseek(filp, 0L, SEEK_END);
	if (currpos == -1) {
		misp_err("%s  llseek failed", __func__);
		err = ~ERR_SUCCESS;
		goto mast_bulk_data_cmd_write_boot_code_end;
	}
	total_size = (u32)currpos;
	/*misp_info("%s  filesize : %d", __func__, total_size);*/
	vfs_llseek(filp, 0L, SEEK_SET);

	/*misp_info("block_size %d", RAWBLOCKSIZE);*/

	/*Transfer boot code*/
	/* boot code & main code can only be sent by SPI */
	err = ispctrl_if_mast_send_bulk(devdata,
		basic_code_buf_addr, filp, total_size, block_size, true);

	if (err != ERR_SUCCESS)
		goto mast_bulk_data_cmd_write_boot_code_end;

	misp_info("%s send boot code success", __func__);

mast_bulk_data_cmd_write_boot_code_end:

	return err;
}



/**
 *\brief Write Basic Code
 *\param devdata [In], misp_data
 *\param param [In], CMD param
 *\param filp  [In], basic code file pointer
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_basic_code(void *devdata,
						u8 *param, struct file *filp)
{
	errcode err = ERR_SUCCESS;
	u8 fw_version[4];
	u32 para_size = ISPCMD_EXEBIN_INFOBYTES;
	u32 *total_size = (u32 *)&param[ISPCMD_EXEBIN_ADDRBYTES];
	u32 file_total_size;
	u32 block_size;
	off_t currpos;
	loff_t offset;
	mm_segment_t oldfs;
	struct misp_global_variable *devdata_global_variable;

	devdata_global_variable = get_mini_isp_global_variable();

	if (filp == NULL) {
		misp_err("%s - file didn't exist.", __func__);
		err = ~ERR_SUCCESS;
		goto mast_bulk_data_cmd_write_basic_code_end;
	}

	block_size = ((struct misp_data *)devdata)->bulk_cmd_blocksize;

	oldfs = get_fs();
	set_fs(KERNEL_DS);


	/*get the file size*/
	currpos = vfs_llseek(filp, 0L, SEEK_END);
	if (currpos == -1) {
		set_fs(oldfs);
		misp_err("%s - llseek end failed", __func__);
		err = ~ERR_SUCCESS;
		goto mast_bulk_data_cmd_write_basic_code_end;
	}

	file_total_size = (u32)currpos;
	/*misp_info("%s  filesize : %u", __func__, file_total_size);*/

	currpos = vfs_llseek(filp, 0L, SEEK_SET);
	if (currpos == -1) {
		set_fs(oldfs);
		misp_err("%s - llseek set failed", __func__);
		err = ~ERR_SUCCESS;
		goto mast_bulk_data_cmd_write_basic_code_end;
	}


	/*read the header info (first 16 bytes in the basic code)*/
	offset = filp->f_pos;
	err = vfs_read(filp, param, ISPCMD_EXEBIN_INFOBYTES, &offset);
	set_fs(oldfs);
	if (err == -1) {
		misp_err("%s - Read file failed.", __func__);
		/*close the file*/
		filp_close(filp, NULL);
		goto mast_bulk_data_cmd_write_basic_code_end;
	}
	filp->f_pos = offset;

	/*To copy checksum value to correct header point*/
	memcpy((u8 *)(param + ISPCMD_EXEBIN_ADDRBYTES +
		  ISPCMD_EXEBIN_TOTALSIZEBYTES + ISPCMD_EXEBIN_BLOCKSIZEBYTES),
		(u8 *)(param + ISPCMD_EXEBIN_ADDRBYTES +
		ISPCMD_EXEBIN_TOTALSIZEBYTES), sizeof(u32));
	/*Assign block size to correct header point*/
	memcpy((u8 *)(param + ISPCMD_EXEBIN_ADDRBYTES +
		ISPCMD_EXEBIN_TOTALSIZEBYTES), &block_size, sizeof(u32));

	/*misp_info("%s param[0][1][2][3]: %02x %02x %02x %02x",
	 *__func__, param[0], param[1], param[2], param[3]);
	 *misp_info("%s param[4][5][6][7]: %02x %02x %02x %02x",
	 *__func__, param[4], param[5], param[6], param[7]);
	 *misp_info("%s param[8][9][10][11]: %02x %02x %02x %02x",
	 *__func__, param[8], param[9], param[10], param[11]);
	 *misp_info("%s param[12][13][14][15]: %02x %02x %02x %02x",
	 *__func__, param[12], param[13], param[14], param[15]);
	 */

	/*Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata,
		ISPCMD_BULK_WRITE_BASICCODE, param, para_size);
	if (err != ERR_SUCCESS)
		goto mast_bulk_data_cmd_write_basic_code_end;

	/*misp_info("%s send leaking packet success", __func__);*/

	/*misp_info("block_size %d", BLOCKSIZE);*/
	misp_info("%s send basic code start", __func__);
	/*Transfer basic code*/
	err = ispctrl_if_mast_send_bulk(devdata,
		basic_code_buf_addr, filp, *total_size, block_size, false);
	if (err != ERR_SUCCESS)
		goto mast_bulk_data_cmd_write_basic_code_end;

	misp_info("%s - send basic code success", __func__);

	/*wait for the interrupt*/
	err = mini_isp_wait_for_event(MINI_ISP_RCV_BULKDATA);
	devdata_global_variable->AL6100_irq_flag = 0;
	if (err != ERR_SUCCESS)
		goto mast_bulk_data_cmd_write_basic_code_end;

	set_fs(KERNEL_DS);
	offset = filp->f_pos;
	currpos = vfs_llseek(filp, file_total_size - 12, SEEK_SET);
	err = vfs_read(filp, fw_version, 4, &filp->f_pos);
	err = vfs_read(filp, fw_build_by, 8, &filp->f_pos);
	set_fs(oldfs);

	if (err == -1) {
		misp_info("%s - Read file failed.", __func__);
	} else {
		 err = 0;
		 fw_version_before_point = fw_version[1]*256 + fw_version[0];
		 fw_version_after_point = fw_version[3]*256 + fw_version[2];
		 fw_build_by[8] = '\0';
		 misp_info("%s - fw version: %05d.%05d, build by %s",
			__func__, fw_version_before_point,
			fw_version_after_point, fw_build_by);
}
mast_bulk_data_cmd_write_basic_code_end:

	return err;
}



/**
 *\brief Write Basic Code (Short SPI Len)
 *\param devdata [In], misp_data
 *\param param [In], CMD param
 *\param filp  [In], basic code file pointer
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_basic_code_shortlen(void *devdata,
						u8 *param, struct file *filp)
{
	errcode err = ERR_SUCCESS;
	u8 fw_version[4];
	// u32 para_size = ISPCMD_EXEBIN_INFOBYTES;
	u32 *total_size = (u32 *)&param[ISPCMD_EXEBIN_ADDRBYTES];
	u32 file_total_size;
	u32 block_size;
	off_t currpos;
	loff_t offset;
	mm_segment_t oldfs;

	if (filp == NULL) {
		misp_err("%s - file didn't exist.", __func__);
		err = ~ERR_SUCCESS;
		goto mast_bulk_data_cmd_write_basic_code_shortlen_end;
	}

	block_size = SPI_BLOCK_LEN;

	oldfs = get_fs();
	set_fs(KERNEL_DS);


	/*get the file size*/
	currpos = vfs_llseek(filp, 0L, SEEK_END);
	if (currpos == -1) {
		set_fs(oldfs);
		misp_err("%s - llseek end failed", __func__);
		err = ~ERR_SUCCESS;
		goto mast_bulk_data_cmd_write_basic_code_shortlen_end;
	}

	file_total_size = (u32)currpos;
	/*misp_info("%s  filesize : %u", __func__, file_total_size);*/

	currpos = vfs_llseek(filp, 0L, SEEK_SET);
	if (currpos == -1) {
		set_fs(oldfs);
		misp_err("%s - llseek set failed", __func__);
		err = ~ERR_SUCCESS;
		goto mast_bulk_data_cmd_write_basic_code_shortlen_end;
	}


	/*read the header info (first 16 bytes in the basic code)*/
	offset = filp->f_pos;
	err = vfs_read(filp, param, ISPCMD_EXEBIN_INFOBYTES, &offset);
	set_fs(oldfs);
	if (err == -1) {
		misp_err("%s - Read file failed.", __func__);
		/*close the file*/
		filp_close(filp, NULL);
		goto mast_bulk_data_cmd_write_basic_code_shortlen_end;
	}
	filp->f_pos = offset;

	/*To copy checksum value to correct header point*/
	memcpy((u8 *)(param + ISPCMD_EXEBIN_ADDRBYTES +
		  ISPCMD_EXEBIN_TOTALSIZEBYTES + ISPCMD_EXEBIN_BLOCKSIZEBYTES),
		(u8 *)(param + ISPCMD_EXEBIN_ADDRBYTES +
		ISPCMD_EXEBIN_TOTALSIZEBYTES), sizeof(u32));
	/*Assign block size to correct header point*/
	memcpy((u8 *)(param + ISPCMD_EXEBIN_ADDRBYTES +
		ISPCMD_EXEBIN_TOTALSIZEBYTES), &block_size, sizeof(u32));


	/*misp_info("%s param[0][1][2][3]: %02x %02x %02x %02x",
	 *__func__, param[0], param[1], param[2], param[3]);
	 *misp_info("%s param[4][5][6][7]: %02x %02x %02x %02x",
	 *__func__, param[4], param[5], param[6], param[7]);
	 *misp_info("%s param[8][9][10][11]: %02x %02x %02x %02x",
	 *__func__, param[8], param[9], param[10], param[11]);
	 *misp_info("%s param[12][13][14][15]: %02x %02x %02x %02x",
	 *__func__, param[12], param[13], param[14], param[15]);
	 */

	/*
	 *misp_info("%s Main Code Address  >>>> param[0]: %04x",
	 *	 __func__, *(u32 *)&param[0]);
	 *misp_info("%s Main Code Size     >>>> param[4]: %04x",
	 *	 __func__, *(u32 *)&param[4]);
	 *misp_info("%s Main Code Checksum >>>> param[12]: %04x",
	 *	 __func__, *(u32 *)&param[12]);
	 */

	/*Send command to slave*/
	// Step 1: Main Code Address
	err = ispctrl_mast_send_cmd_to_slave(devdata,
		ISPCMD_BULK_WRITE_BASICCODE_CODEADDR,
		 &param[0], ISPCMD_EXEBIN_ADDRBYTES);

	if (err != ERR_SUCCESS)
		goto mast_bulk_data_cmd_write_basic_code_shortlen_end;

	msleep(1);

	// Step 2: Main Code Size
	err = ispctrl_mast_send_cmd_to_slave(devdata,
		ISPCMD_BULK_WRITE_BASICCODE_CODESIZE, &param[4],
		 ISPCMD_EXEBIN_TOTALSIZEBYTES);
	if (err != ERR_SUCCESS)
		goto mast_bulk_data_cmd_write_basic_code_shortlen_end;

	msleep(1);

	// Step 3: Main Code Checksum
	err = ispctrl_mast_send_cmd_to_slave(devdata,
		ISPCMD_BULK_WRITE_BASICCODE_CODESUM, &param[12],
		ISPCMD_EXEBIN_CKSUMBYTES);
	if (err != ERR_SUCCESS)
		goto mast_bulk_data_cmd_write_basic_code_shortlen_end;



	/*misp_info("%s send leaking packet success", __func__);*/

	/*misp_info("block_size %d", BLOCKSIZE);*/
	misp_info("%s send basic code start", __func__);
	/*Transfer basic code*/
	err = ispctrl_if_mast_send_bulk(devdata,
		basic_code_buf_addr, filp, *total_size, block_size, false);
	if (err != ERR_SUCCESS)
		goto mast_bulk_data_cmd_write_basic_code_shortlen_end;

	misp_info("%s - send basic code success", __func__);


	// mdelay(1);

	/*wait for the interrupt*/
	err = mini_isp_wait_for_event(MINI_ISP_RCV_BULKDATA);

	if (err != ERR_SUCCESS)
		goto mast_bulk_data_cmd_write_basic_code_shortlen_end;

	set_fs(KERNEL_DS);
	offset = filp->f_pos;
	currpos = vfs_llseek(filp, file_total_size - 12, SEEK_SET);
	err = vfs_read(filp, fw_version, 4, &filp->f_pos);
	err = vfs_read(filp, fw_build_by, 8, &filp->f_pos);
	set_fs(oldfs);

	if (err == -1) {
		misp_info("%s - Read file failed.", __func__);
	} else {
		 err = 0;
		 fw_version_before_point = fw_version[1]*256 + fw_version[0];
		 fw_version_after_point = fw_version[3]*256 + fw_version[2];
		 fw_build_by[8] = '\0';
		 misp_info("%s - fw version: %05d.%05d, build by %s",
			__func__, fw_version_before_point,
			fw_version_after_point, fw_build_by);
}
mast_bulk_data_cmd_write_basic_code_shortlen_end:

	return err;
}




/**
 *\brief Write Calibration Data
 *\param devdata [In], misp_data
 *\param param [In], CMD param
 *\param filp  [In], calibration data file pointer
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_calibration_data(void *devdata,
						u8 *param, struct file *filp)
{
	errcode err = ERR_SUCCESS;
	u8 infomode;
	u16 opcode;
	u16 ckecksum;
	u32 para_size = 11;
	u32 filesize;
	u32 block_size;
	off_t currpos;
	loff_t offset;
	mm_segment_t oldfs;
	struct misp_global_variable *devdata_global_variable;

	devdata_global_variable = get_mini_isp_global_variable();
	infomode = param[8];
	block_size = ((struct misp_data *)devdata)->bulk_cmd_blocksize;

	/* Trasfered files have been opened in kernel
	 * when mini_isp_drv_load_fw() */
	if (((infomode >= 2) && (infomode < 7)) || (infomode == 8)) {
		if (filp == NULL) {
			misp_err("%s - file didn't exist.", __func__);
			err = ~ERR_SUCCESS;
			goto mast_bulk_data_cmd_write_calibration_data_end;
		}

		oldfs = get_fs();
		set_fs(KERNEL_DS);

		/*get the file size*/
		currpos = vfs_llseek(filp, 0L, SEEK_END);
		if (currpos == -1) {
			set_fs(oldfs);
			misp_err("%s  llseek end failed", __func__);
			err = ~ERR_SUCCESS;
			goto mast_bulk_data_cmd_write_calibration_data_end;
		}

		filesize = (u32)currpos;
		/*misp_info("%s  filesize : %u", __func__, filesize);*/

		currpos = vfs_llseek(filp, 0L, SEEK_SET);
		if (currpos == -1) {
			set_fs(oldfs);
			misp_err("%s  llseek set failed", __func__);
			err = ~ERR_SUCCESS;
			goto mast_bulk_data_cmd_write_calibration_data_end;
		}

		/*Request memory*/
		calibration_data_buf_addr = kzalloc(filesize, GFP_KERNEL);
		if (!calibration_data_buf_addr) {
			err = ~ERR_SUCCESS;
			kfree(calibration_data_buf_addr);
			goto mast_bulk_data_cmd_write_calibration_data_fail;
		}

		/* if the file is SC table, print the SC table built day */
		if (infomode == 2) {
			vfs_llseek(filp, 16, SEEK_SET);
			vfs_read(filp, calibration_data_buf_addr, 4,
				&filp->f_pos);
			sc_build_date =
				calibration_data_buf_addr[3]*256*256*256 +
				calibration_data_buf_addr[2]*256*256 +
				calibration_data_buf_addr[1]*256 +
				calibration_data_buf_addr[0];
			misp_info("%s - SC table build date %d.", __func__,
				sc_build_date);
			vfs_llseek(filp, 0L, SEEK_SET);
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
			goto mast_bulk_data_cmd_write_calibration_data_end;
		}
		filp->f_pos = offset;
		vfs_llseek(filp, 0L, SEEK_SET);

		ckecksum = calculate_check_sum((u8 *) calibration_data_buf_addr,
						filesize);

		/*Assign Info ID to correct header point*/
		memcpy((u8 *)(param + 8), &infomode, sizeof(u8));
		/*To copy Total Size to correct header point*/
		memcpy((u8 *)param, &filesize, sizeof(u32));
		/*Assign block size to correct header point*/
		memcpy((u8 *)(param + 4), &block_size, sizeof(u32));
		/*Assign check sum to correct header point*/
		memcpy((u8 *)(param + 9), &ckecksum, sizeof(u16));

		misp_info("%s [0][1][2][3]:%02x %02x %02x %02x",
			__func__, param[0], param[1], param[2], param[3]);
		misp_info("%s [4][5][6][7]:%02x %02x %02x %02x",
			__func__, param[4], param[5], param[6], param[7]);
		misp_info("%s [8][9][10]:%02x %02x %02x",
			__func__, param[8], param[9], param[10]);

	} else {
	/* Trasfered buffer has opened in user space and passed to kernel.*/
		memcpy(&filesize, param, sizeof(u32));
		calibration_data_buf_addr = kzalloc(filesize, GFP_KERNEL);

		if (!calibration_data_buf_addr) {
			err = ~ERR_SUCCESS;
			kfree(calibration_data_buf_addr);
			goto mast_bulk_data_cmd_write_calibration_data_fail;
		}
		memcpy(calibration_data_buf_addr,
			param + T_SPI_CMD_LENGTH, filesize);
	}

	if (SPI_SHORT_LEN_MODE)	{
		opcode = ISPCMD_BULK_WRITE_CALIBRATION_NO_BLOCK;
		// Total size(4byte) + data id(1byte) + checksum(2byte)
		para_size = 7;
		// left shift 4byte
		memcpy((u8 *)(param + 4), (u8 *)(param + 8),
				sizeof(u32) + sizeof(u16));

		block_size = SPI_BLOCK_LEN;
	} else {
		opcode = ISPCMD_BULK_WRITE_CALIBRATION_DATA;
	}

	/*Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata,
		opcode, param, para_size);
	if (err != ERR_SUCCESS) {
		kfree(calibration_data_buf_addr);
		goto mast_bulk_data_cmd_write_calibration_data_end;
	}

	/*misp_info("%s send leaking packet success", __func__);*/

	/*misp_info("block_size %d", BLOCKSIZE);*/

	err = mini_isp_wait_for_event(MINI_ISP_RCV_CMD_READY);
	if (err)
		goto mast_bulk_data_cmd_write_calibration_data_fail;

	err = ispctrl_if_mast_send_bulk(devdata,
		calibration_data_buf_addr, filp, filesize, block_size,
		false);

	kfree(calibration_data_buf_addr);

	if (err != ERR_SUCCESS)
		goto mast_bulk_data_cmd_write_calibration_data_end;

	err = mini_isp_wait_for_event(MINI_ISP_RCV_BULKDATA);
	devdata_global_variable->AL6100_irq_flag = 0;
	if (err)
		goto mast_bulk_data_cmd_write_calibration_data_fail;

	if (infomode == 0)
		misp_info("%s write IQ calibration data success", __func__);
	else if (infomode == 1)
		misp_info("%s write depth packet data success", __func__);
	else if (infomode == 2)
		misp_info("%s write scenario table success", __func__);
	else if (infomode == 3)
		misp_info("%s write HDR Qmerge data success", __func__);
	else if (infomode == 4)
		misp_info("%s write IRP0 Qmerge data success", __func__);
	else if (infomode == 5)
		misp_info("%s write IRP1 Qmerge data success", __func__);
	else if (infomode == 6)
		misp_info("%s write PP map success", __func__);
	else if (infomode == 7)
		misp_info("%s write blending table success", __func__);
	else if (infomode == 8)
		misp_info("%s write Depth Qmerge data success", __func__);

	goto mast_bulk_data_cmd_write_calibration_data_end;

mast_bulk_data_cmd_write_calibration_data_fail:
	misp_err("%s mast_bulk_data_cmd_write_calibration_data_fail", __func__);

mast_bulk_data_cmd_write_calibration_data_end:
	return err;

}

/*
 *\brief Read Memory Data
 *\param devdata [In], misp_data
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_bulk_data_cmd_read_memory_data(void *devdata,
							u8 *param)
{
	/*Error Code*/
	errcode err = ERR_SUCCESS;
	u8 *allocated_memmory = 0;
	u32 read_size = 0;
	u8 filename[80];
	/*
	 *Parameter size
	 *4bytes for start addr, 4 bytes for total size, 4 bytes for block size,
	 *4 bytes for memory dump mode
	 */
	u32 para_size = sizeof(struct memmory_dump_hdr_info);
	/*Total size*/
	u32 total_size;
	struct memmory_dump_hdr_info *memory_dump_hdr_config;
	struct file *f;
	mm_segment_t fs;

	read_size = MID_PJ_EXEBIN_BUF;

	/*Request memory*/
	allocated_memmory = kzalloc(read_size, GFP_KERNEL);
	if (!allocated_memmory) {
		err = ~ERR_SUCCESS;
		misp_err("%s Allocate memory failed.", __func__);
		goto allocate_memory_fail;
	}


	memory_dump_hdr_config = (struct memmory_dump_hdr_info *)param;

	/*Assign total size*/
	total_size = memory_dump_hdr_config->total_size;
	if (total_size > read_size) {
		err = ERR_MASTERCMDSIZE_MISMATCH;
		misp_err("%s total_size error.", __func__);
		goto mast_bulk_data_cmd_read_memory_data_end;
	}

	/*Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata,
		ISPCMD_BULK_READ_MEMORY, param, para_size);
	if (err != ERR_SUCCESS) {
		kfree(allocated_memmory);
		misp_err("%s send command fail, 0x%x.", __func__, err);
		goto mast_bulk_data_cmd_read_memory_data_end;
	}
	/*Get memory data from slave*/
	err = ispctrl_if_mast_recv_memory_data_from_slave(devdata,
			allocated_memmory,
			&total_size,
			memory_dump_hdr_config->block_size,
			true);
	if (err  != ERR_SUCCESS) {
		kfree(allocated_memmory);
		misp_err("%s get bulk fail, 0x%x.", __func__, err);
		goto mast_bulk_data_cmd_read_memory_data_end;
	}

	misp_info("%s - Read memory finished.", __func__);
	/*write out allocated_memmory to file here*/
	/*** add your codes here ***/
	snprintf(filename, 80, "%s/miniISP_memory.log",
		MINIISP_INFO_DUMPLOCATION);
	f = filp_open(filename, O_APPEND|O_CREAT, 0777);
	/*Get current segment descriptor*/
	fs = get_fs();
	/*Set segment descriptor associated*/
	set_fs(get_ds());

	if (IS_ERR(f)) {
		err = PTR_ERR(f);
		misp_err("%s open file failed. err: %d", __func__, err);
		set_fs(fs);
		goto file_open_fail;
	}

	/*write the file*/
	vfs_write(f, (char *)allocated_memmory, total_size,
		&f->f_pos);

	/*Restore segment descriptor*/
	set_fs(fs);
	filp_close(f, NULL);
	/*** end of the codes ***/
file_open_fail:
allocate_memory_fail:
	kfree(allocated_memmory);
mast_bulk_data_cmd_read_memory_data_end:

	return err;


}

/*
 *\brief Read common log data
 *\param devdata [In], misp_data
 *\param param [In], CMD param
 *\return Error code
 */
errcode bulk_data_cmd_read_common_log(void *devdata, u8 *param)
{
	/*Error Code*/
	errcode err = ERR_SUCCESS;
	u8 *allocated_memmory = 0;
	u32 read_size = LOGSIZE;
	struct file *f;
	mm_segment_t fs;
	u8 filename[80];

	/*Parameter size : 4 bytes for total size, 4 bytes for block size*/
	u32 para_size = sizeof(struct common_log_hdr_info);
	/*Total size*/
	u32 total_size;
	struct common_log_hdr_info *common_log_hdr_cfg;


	/*Request memory*/
	allocated_memmory = kzalloc(read_size, GFP_KERNEL);
	if (!allocated_memmory) {
		err = ~ERR_SUCCESS;
		goto allocate_memory_fail;
	}


	common_log_hdr_cfg = (struct common_log_hdr_info *)param;

	/*Assign total size*/
	total_size = common_log_hdr_cfg->total_size;
	if (total_size > read_size) {
		err = ERR_MASTERCMDSIZE_MISMATCH;
		kfree(allocated_memmory);
		goto bulk_data_cmd_read_common_log_end;
	}

	/*Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata,
		ISPCMD_BULK_READ_COMLOG, param, para_size);
	if (err != ERR_SUCCESS) {
		kfree(allocated_memmory);
		misp_info("%s err - 0x%x.", __func__, err);
		goto bulk_data_cmd_read_common_log_end;
	}
	misp_info("%s - Start to read log.", __func__);

	/*
	 *Get memory data from slave,don't wait INT
	 *and use polling interval to wait
	 */
	err = ispctrl_if_mast_recv_memory_data_from_slave(devdata,
			allocated_memmory,
			&total_size,
			common_log_hdr_cfg->block_size,
			true);
	if (err  != ERR_SUCCESS) {
		kfree(allocated_memmory);
		goto bulk_data_cmd_read_common_log_end;
	}
	misp_info("%s - Read log finished.", __func__);
	snprintf(filename, 80, "%s/miniISP_Common_Log.log",
		MINIISP_INFO_DUMPLOCATION);
	f = filp_open(filename, O_APPEND|O_CREAT|O_RDWR, 0777);
	/*Get current segment descriptor*/
	fs = get_fs();

	/*Set segment descriptor associated*/
	set_fs(get_ds());

	if (IS_ERR(f)) {
		err = PTR_ERR(f);
		misp_err("%s open file failed. err: %d", __func__, err);
		goto file_open_fail;
	}
	/*write the file*/
	vfs_write(f, (char *)allocated_memmory, strlen(allocated_memmory),
		&f->f_pos);

file_open_fail:
	/*Restore segment descriptor*/
	set_fs(fs);
	filp_close(f, NULL);

	goto bulk_data_cmd_read_common_log_end;
allocate_memory_fail:

	misp_err("%s Allocate memory failed.", __func__);
bulk_data_cmd_read_common_log_end:
	kfree(allocated_memmory);
	return err;
}


/*
 *\brief Get depth Rect A, B, Invrect
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_depth_rectab_invrect(
	void *devdata, u16 opcode, u8 *param)
{
	/*Error Code */
	errcode err = ERR_SUCCESS;
	u8 *allocated_memmory = 0;
	u32 para_size = 0;
	u32 rect_set_num = 0;
	u32 send_total_bulk_size = 0;
	u8 send_cmd[T_SPI_CMD_LENGTH];
	struct isp_cmd_depth_rectab_invrect_info *depth_rect_info;

	depth_rect_info = (struct isp_cmd_depth_rectab_invrect_info *)param;
	if (depth_rect_info->trans_mode == 0)
		rect_set_num = 3;
	else
		rect_set_num = 1;

	send_total_bulk_size =
		rect_set_num*sizeof(struct depth_rectab_invrect_param);

	/* Fill command buffer for send */
	memcpy(&send_cmd[0], &(depth_rect_info->trans_mode),
		sizeof(depth_rect_info->trans_mode));
	para_size = sizeof(depth_rect_info->trans_mode);

	memcpy(&send_cmd[para_size], &(depth_rect_info->block_size),
		sizeof(depth_rect_info->block_size));
	para_size += sizeof(depth_rect_info->block_size);

	/*Request memory*/
	allocated_memmory = kzalloc(send_total_bulk_size, GFP_KERNEL);
	if (!allocated_memmory) {
		err = ~ERR_SUCCESS;
		goto allocate_memory_fail;
	}

	/*Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata,
		ISPCMD_BULK_WRITE_DEPTH_RECTAB_INVRECT,
		(u8 *)&send_cmd[0], para_size);

	if (err  != ERR_SUCCESS)
		goto mast_bulk_data_cmd_write_depth_rectab_invrect_end;

	err = mini_isp_wait_for_event(MINI_ISP_RCV_CMD_READY);
	if (err)
		goto mast_bulk_data_cmd_write_depth_rectab_invrect_end;

	/* Send bulk data to slave*/
	err = ispctrl_if_mast_send_bulk(
			devdata,
			(u8 *)&depth_rect_info->rect_param[0],
			NULL, send_total_bulk_size,
			depth_rect_info->block_size, false);

	if (err != ERR_SUCCESS)
		goto mast_bulk_data_cmd_write_depth_rectab_invrect_end;

	err = mini_isp_wait_for_event(MINI_ISP_RCV_BULKDATA);
	if (err)
		goto mast_bulk_data_cmd_write_depth_rectab_invrect_end;

mast_bulk_data_cmd_write_depth_rectab_invrect_end:
	kfree(allocated_memmory);
	allocated_memmory = NULL;

allocate_memory_fail:
	misp_err("%s Allocate memory failed.", __func__);

	return err;
}

/******Private Function******/
static u16 calculate_check_sum(u8 *input_buffer_addr, u16 input_buffer_size)
{
	u16 i;
	u32 sum = 0;
	u16 sumvalue;

	/*calculating unit is 2 bytes */
	for (i = 0; i < input_buffer_size; i++) {
		if (0 == (i%2))
			sum += input_buffer_addr[i];
		else
			sum += (input_buffer_addr[i] << 8);
	}

	/*Do 2's complement */
	sumvalue = (u16)(65536 - (sum & 0x0000FFFF));

	return sumvalue;
}

/******End Of File******/

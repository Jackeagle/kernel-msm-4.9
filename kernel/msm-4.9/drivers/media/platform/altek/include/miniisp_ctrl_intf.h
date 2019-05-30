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

long handle_ControlFlowCmd(unsigned int cmd, unsigned long arg);
//AL6100 Kernel Base Solution >>>
enum miniisp_firmware {
	ECHO_IQ_CODE,
	ECHO_DEPTH_CODE,
	ECHO_OTHER_MAX
};

long handle_ControlFlowCmd_Kernel(unsigned int cmd, unsigned long arg);
void mini_isp_other_drv_open(char *file_name, u8 type);
void mini_isp_other_drv_read(struct file *filp, u8 type);
//AL6100 Kernel Base Solution <<<

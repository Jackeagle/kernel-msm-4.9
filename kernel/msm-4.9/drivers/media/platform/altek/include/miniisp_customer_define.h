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
 * File: miniisp_customer_define.h
 * Description: miniISP customer define
 */
#include <linux/spi/spi.h>
/******Public Define******/

/*SPI MODE*/
#define SPI_MODE (SPI_MODE_3) /*SPI_MODE_3 | SPI_CS_HIGH*/
/*boot file location*/
#define BOOT_FILE_LOCATION "/data/firmware/miniBoot.bin"//"/system/vendor/firmware/miniBoot.bin"
/*basic code location*/
#define BASIC_FILE_LOCATION "/data/firmware/TBM_SK1.bin"//"/system/vendor/firmware/TBM_SK1.bin"
/*advanced code location*/
#define ADVANCED_FILE_LOCATION NULL
/*scenario table location*/
#define SCENARIO_TABLE_FILE_LOCATION "/data/firmware/SCTable.asb"//  "/system/vendor/firmware/SCTable.asb"

/*hdr qmerge data location*/
#define HDR_QMERGE_DATA_FILE_LOCATION "/data/firmware/HDR.bin"  //  "/system/vendor/firmware/HDR.bin"
/*irp0 qmerge data location*/
#define IRP0_QMERGE_DATA_FILE_LOCATION "/data/firmware/IRP0.bin" //  "/system/vendor/firmware/IRP0.bin"
/*irp1 qmerge data location*/
#define IRP1_QMERGE_DATA_FILE_LOCATION "/data/firmware/IRP1.bin" //  "/system/vendor/firmware/IRP1.bin"
/*pp map location*/
#define PP_MAP_FILE_LOCATION NULL/*"/system/etc/firmware/PPmap.bin"*/
/*depth qmerge data location*/
#define DPETH_QMERGE_DATA_FILE_LOCATION "/data/firmware/Depth.bin"

// ALTEK_AL6100_CHI >>>
/*iq calibaration data location*/
#define IQCALIBRATIONDATA_FILE_LOCATION "/data/misc/camera_otp/IQCalibrationData_Decrypt.bin" /*"/system/etc/firmware/PPmap.bin"*/
/*depth pack data location*/
#define DEPTHPACKDATA_FILE_LOCATION "/data/misc/camera_otp/DepthPackData_Decrypt.bin" /*"/system/etc/firmware/PPmap.bin"*/
// ALTEK_AL6100_CHI <<<
/*miniISP dump info save location*/
/*Add location folder where you let Altek debug info saving in your device*/
#define MINIISP_INFO_DUMPLOCATION "/data/miniISP_dump_reg"

/*miniISP bypass setting file location*/
/*Add location folder where you let Altek debug info saving in your device*/
#define MINIISP_BYPASS_SETTING_FILE_PATH "/data/firmware/"//"/system/vendor/firmware/"

/*define for gpio*/
/*vcc1 : if no use, set NULL*/
#define VCC1_GPIO NULL /*"vcc1-gpios"*/

/*vcc2 : if no use, set NULL*/
#define VCC2_GPIO NULL/*"vcc2-gpios"*/

/*vcc3 : if no use, set NULL*/
#define VCC3_GPIO NULL/*"vcc3-gpios"*/

/*reset*/
#define RESET_GPIO NULL  //"reset-gpios"

/*irq*/
#define IRQ_GPIO "irq-gpios"

/*isp_clk : if no use, set NULL*/
#define ISP_CLK NULL /*"al6100_clk"*/

/*Enable SPI short length mode*/
#define SPI_SHORT_LEN_MODE false
#define SPI_BLOCK_LEN 14


/******Public Function Prototype******/


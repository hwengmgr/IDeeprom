/*
 * Copyright (c) 2020, Teradyne Inc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TMP107_H
#define TMP107_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tmp107_arch_config.h"

// #defines
#define TMP107_Success			0
#define TMP107_EchoError		1

// required timeouts
#define TMP107_EEPROM_prog_time	16  //16 msec programming time for each EEPROM location

// Speed independent timeouts
#define TMP107_InitTimeout      72  // global addr initia time required
#define TMP107_AddrInitTimeout	1250  // for writing EEPROM during Init



/* Command Byte */
#define TMP107_Command_bit		0x4
#define TMP107_Read_bit			0x2
#define TMP107_Global_bit		0x1

#define TMP107_AddressInitialize 0x12
#define TMP107_LastDevice		0xA
#define TMP107_SoftwareReset	0xB
#define	TMP107_Alert1Clear		0x16
#define TMP107_Alert2Clear		0xE

 /* Registers */
#define TMP107_Temp_reg			0xA0
#define TMP107_Conf_reg			0xA1
#define TMP107_HiLimit1_reg		0xA2
#define TMP107_LoLimit1_reg		0xA3
#define TMP107_HiLimit2_reg		0xA4
#define TMP107_LoLimit2_reg		0xA5
#define TMP107_EEPROM1_reg		0xA6
#define TMP107_EEPROM2_reg		0xA7
#define TMP107_EEPROM3_reg		0xA8
#define TMP107_EEPROM4_reg		0xA9
#define TMP107_EEPROM5_reg		0xAA
#define TMP107_EEPROM6_reg		0xAB
#define TMP107_EEPROM7_reg		0xAC
#define TMP107_EEPROM8_reg		0xAD
#define TMP107_DieID_reg		0xAF //Read Only

//data structures
//unsigned char initializeTmp107(); un inzilizzed
char TMP107_start();
char TMP107_close();
void testpattern();
char TMP107_GlobalAddressInit(); //unsigned char device_count);
char TMP107_ReadRegister(char DeviceAddr, char RegAddr, int16_t *RegData);
char TMP107_WriteRegister(char DeviceAddr, char RegAddr, int16_t WriteData);
void TMP107_ReadTemperature(char last_addr, unsigned char device_count, char *rx);
char TMP107_LastDevicePoll();
void TMP107_GlobalSoftwareReset();
void TMP107_GlobalAlertClear1();
void TMP107_GlobalAlertClear2();

float TMP107_DecodeTemperatureResult(int HByte, int LByte);
unsigned char TMP107_Encode5bitAddress(unsigned char addr);
unsigned char TMP107_Decode5bitAddress(unsigned char addr);



void TMP107_SetAlertsasGPIO();
void TMP107_AlertPinHiLimitReg();
void TMP107_AlertClear2(char lowhigh, char addr);

//*************************

struct sps30_measurement {
    float32_t mc_1p0;
    float32_t mc_2p5;
    float32_t mc_4p0;
    float32_t mc_10p0;
    float32_t nc_0p5;
    float32_t nc_1p0;
    float32_t nc_2p5;
    float32_t nc_4p0;
    float32_t nc_10p0;
    float32_t typical_particle_size;
};

//****************************************

#ifdef __cplusplus
}
#endif

#endif /* TMP107_H */

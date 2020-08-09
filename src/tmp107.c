/* --COPYRIGHT--,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/*
 * tmp107.c
 *
 *  Created on: Dec 17, 2015
 *      Author: a0271474
 */

#include "tmp107.h"
#include "tmp107_uart.h"
#include <stdio.h>  // printf
#include <unistd.h>  //usleep

int16_t retcount;

char TMP107_start(){
	// open port
	int16_t status;
	printf("call open\n");
	status = tmp107_uart_open();
	printf("open status: %d\n", status);
	return status;
}

char TMP107_close(){
	int16_t status;
	printf("closing port\n");
	status = tmp107_uart_close();
	return status;
}

void testpattern() {
	char data[10];
	char tx_size, rx_size;
	int16_t rx_data[32];
	int16_t status;

	status = tmp107_uart_open();
	printf("%d\n", status);
	usleep(10000);
	data[0] = 0x1;
	data[1] = 0x50;
	data[2] = 0x80;
	status = TMP107_Transmit(data,3);
	printf("%d\n", status);
	tx_size = 4;
	rx_size = tx_size;
		
	status = TMP107_RetrieveReadback(tx_size, rx_data, rx_size);

	status = TMP107_close();
	printf("%d\n", status);
	
}

char TMP107_GlobalAddressInit(){
	// initialize the chain and return the last device address
	// returns address of last device on chain
	char data[2];
	int16_t rx[32];
    
	data[0] = TMP107_Global_bit | 0x94; // AddressInit command code
	data[1] = 0x5 | TMP107_Encode5bitAddress(0x1); // give the first device logical address 1
	printf("data array: %d, %d\n", data[1], data[0]);
	
	retcount = TMP107_Transmit(data,2);
	printf("retcount from transmit %d\n", retcount);
	
    /* must wait 1250ms after receiving last device response
     * during address init operation. this is not baud rate
     * dependent. this is because address init writes to the
     * internal eeprom, which takes additional time.
     */
	//last_response = TMP107_WaitForEcho(2,33,TMP107_AddrInitTimeout); //force timeout with unreachable count
 // responses take 8msec per module. max 8 mods plus 1 for controller is 9 * 8 msec = 72 msec. independent of bus speed
	usleep(TMP107_InitTimeout * 1000);  //72msec

// 12 max responses with 8 mods, 1 controller and 3 bytes transmit. pad to 15 for margin.
	retcount = TMP107_RetrieveReadback(2, rx, 15); 
	printf("\nRead bytes retrieved %d\n", retcount);

// sleep 1.25sec for EEPROM to write. independent of actual bus speed.
	usleep(TMP107_AddrInitTimeout*1000); //1250msec
	
	/* the last address received is actually the address of
	 * the next device (which does not exist.) in order to get
	 * the address of the last device, we have to look at
	 * the address that was received 2nd to last.
	 */
	return (rx[retcount - 1] & 0xF8)/8;
}

char TMP107_LastDevicePoll(){
	// query the device chain to find the last device in the chain
	// returns address of last device on chain
	char tx[1];
	int16_t rx[1];
	unsigned char retval;
	tx[0] = 0x57; // LastDevicePoll command code
	retcount = TMP107_Transmit(tx,1);
//	usleep((TMP107_Timeout + 20) * 1000);  //40msec
	retcount = TMP107_RetrieveReadback(1,rx,10);
	printf("Read Bytes retrieved: %x\n", retcount);
	retval = (rx[retcount - 1] & 0xF8) / 8; // mask the unused address bits that are always 0b11
	return retval;
}

char TMP107_ReadRegister(char DeviceAddr, char RegAddr, int16_t *RegData){
	// read a register from the device address passed in.
	// all registers are 16 bits
	int16_t ReadAddr;
	char tx[2];
	int16_t rx[10];
	
	// shift device addr up 3 bits and OR in lower control bits.
	// control bits: not global, Not command, Read
	ReadAddr = TMP107_Read_bit | (DeviceAddr << 3);
	printf("Read Address: %x\n", ReadAddr);

	//RegAddr is already correct as is from Defines
	// Do transfer. Send device address, the reg address, then wait 2.5msec and retrieve data
	tx[0] = ReadAddr;
	tx[1] = RegAddr;
	retcount = TMP107_Transmit (tx, 2);
	printf("Bytes transmitted: %x\n", retcount);
	
	// Now wait 2.5msec and retrieve data
//	usleep (2.5 * 1000); // .5msec
	retcount = TMP107_RetrieveReadback(1,rx,6);
	printf("Read Bytes retrieved: %x\n", retcount);
	RegData[0] = rx[3];
	RegData[1] = rx[4];
	return(retcount - 3);  // bytes of read data
	
}

char TMP107_WriteRegister(char DeviceAddr, char RegAddr, int16_t WriteData){
	// write data to the device address passed in.
	// all registers are 16 bits
	int16_t WriteAddr;
	char tx[2];
	int16_t rx[32];  // used for getting echo bytes
	
	// shift device addr up 3 bits and OR in lower control bits.
	// control bits: not global, Not command, Read
	WriteAddr = (DeviceAddr << 3); // lower 3 LSB's not set for write transfer
	printf("Write Address: %x\n", WriteAddr);

	//RegAddr is already correct as is from Defines
	// Do transfer. Send device address, the reg address, then wait 2.5msec and retrieve data
	tx[0] = WriteAddr;
	tx[1] = RegAddr;
	tx[2] = WriteData & 0xff; // lower 8 bits are sent first
	tx[3] = (WriteData & 0xff00) >> 8; // upper 8 bits are sent last
	printf("data to be sent: upper byte: %x, lower byte: %x\n", tx[3], tx[2]);
		
	retcount = TMP107_Transmit (tx, 4);
	printf("Bytes transmitted: %x\n", retcount);
	
	// Now wait 5msec and retreive echo data. should be 5 bytes
//	usleep (5 * 1000); // 5msec
	retcount = TMP107_RetrieveReadback(1,rx,6);
	printf("Echo Bytes retrieved: %x\n", retcount);
	

return(retcount);
	
}

void TMP107_GlobalSoftwareReset(){
	// reset all devices in chain
	char tx[1];
	int16_t rx[10]; //for echo data
	tx[0] = 0x5D; // GlobalSoftwareReset command code
	TMP107_Transmit(tx,1);

	// Now wait 1msec and retreive echo data. should be 2 bytes
//	usleep (1 * 1000); // 5msec
	retcount = TMP107_RetrieveReadback(1,rx,3);
	printf("Echo Bytes retrieved: %x\n", retcount);
	
}

void TMP107_SetAlertsasGPIO(){
    // global write on all alert pins in the sensor modules to
    // set the alert pins as IO's
    // high alert register to 7FFC
    // low alert register to 1000
    char tx[4];
	int16_t rx[32];
	
    // HIGH LIMIT REGISTER
    tx[0] = TMP107_Global_bit | (0x4 << 3); //(CMD and ADD) GLOBAL WRITE
    tx[1] = TMP107_HiLimit2_reg; //0xA4 REG POINTER
    tx[2] = 0xFC; // 0x7FFC
    tx[3] = 0x7F;

    // transmit
    TMP107_Transmit(tx,4);

    // LOW LIMIT REGISTER
    tx[0] = TMP107_Global_bit | (0x4 << 3); //(CMD and ADD)
    tx[1] = TMP107_LoLimit2_reg; //0xA5 REG POINTER
    tx[2] = 0x00; // 0x8000
    tx[3] = 0x80;

    // transmit
    TMP107_Transmit(tx,4);

   	// Now retreive echo data. should be 10 bytes
//	usleep (5 * 1000); // 5msec
	retcount = TMP107_RetrieveReadback(1,rx,15);
	printf("Echo Bytes retrieved: %x\n", retcount);

    // HIGH LIMIT REGISTER
    tx[0] = TMP107_Global_bit | (0x4 << 3); //(CMD and ADD) GLOBAL WRITE
    tx[1] = TMP107_HiLimit1_reg; //0xA2 REG POINTER
    tx[2] = 0xFC; // 0x7FFC
    tx[3] = 0x7F;

    // transmit
    TMP107_Transmit(tx,4);

    // LOW LIMIT REGISTER
    tx[0] = TMP107_Global_bit | (0x4 << 3); //(CMD and ADD)
    tx[1] = TMP107_LoLimit1_reg; //0xA3 REG POINTER
    tx[2] = 0x00; // 0x8000
    tx[3] = 0x80;

    // transmit
    TMP107_Transmit(tx,4);

   	// Now retreive echo data. should be 10 bytes
//	usleep (5 * 1000); // 5msec
	retcount = TMP107_RetrieveReadback(1,rx,15);
	printf("Echo Bytes retrieved: %x\n", retcount);

}

void TMP107_GlobalAlertClear1(){
	// clear all Alert1
	char tx[1];
	int16_t rx[5];
	tx[0] = 0xB5; // GlobalAlertClear1 command code
	TMP107_Transmit(tx,1);
	TMP107_RetrieveReadback(1,rx,3);
}
void TMP107_GlobalAlertClear2(){
	// clear all Alert2
	char tx[1];
	int16_t rx[5];
	tx[0] = 0x75; // GlobalAlertClear2 command code
	TMP107_Transmit(tx,1);
	TMP107_RetrieveReadback(1,rx,3);

}

float TMP107_DecodeTemperatureResult(int HByte, int LByte){
	// convert raw byte response to floating point temperature
	int Bytes;
	float temperature;
	Bytes = HByte << 8 | LByte;
	Bytes &= 0xFFFC; //Mask NVM bits not used in Temperature Result
	temperature = (float) Bytes / 256;
	return temperature;
}

unsigned char TMP107_Encode5bitAddress(unsigned char addr){
	// bit-reverse logical address to get TMP107-encoded address
	char i;
	unsigned char out = 0;
	for (i = 0; i < 5; i++){
		if (addr & (1 << i)){
			out |= 1 << (3+i);
		}
	}
	return out;
}

unsigned char TMP107_Decode5bitAddress(unsigned char addr){
	// bit-reverse TMP107-encoded address to get logical address
	char i;
	unsigned char out = 0;
	for (i = 3; i < 8; i++){
		if (addr & (1 << i)){
			out |= 1 << (i-3);
		}
	}
	return out;
}

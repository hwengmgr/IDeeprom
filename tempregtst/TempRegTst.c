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

#include <stdio.h>  // printf
#include <stdlib.h> // system
#include <unistd.h> // usleep

// #include "tmp107_uart.h "
#include "tmp107.h"

/**
* TO USE CONSOLE OUTPUT (PRINTF) AND WAIT (SLEEP) PLEASE ADAPT THEM TO YOUR
* PLATFORM
*/
//#define printf(...)
#define DEVICE 2
#define DEVICEALT 3
#define true 1
#define false 0

void DumpEEPROM(int16_t device);

int main()
{
	//char msg[20];
	//int16_t tx[8];
	int16_t rx[64];
	int16_t retCount;


// Define Bools to enable each section of the code
  	int runGlobInit = true;
  	int runRegTest = false;
  	int runTempRead = true;
  	int runInteractAlert = false; //true;

/*
 	printf("enter the device to use:");
 	int newdev, rcvd;
 	rcvd = scanf("%d", &newdev);

  	device = newdev;
*/

// turn power on to modules
	system("../pwron.sh");

	retCount = TMP107_start();  // open port
  
// check if device is real. get last addr and check that requested device is <= that addr
	printf("***Start last device poll\n");
	int16_t LastAddr = TMP107_LastDevicePoll();
	printf("This program assumes Global Addr Init has already been run\n");
	printf ("Testing temp reads from %d devices. \n", LastAddr); 

//read devices from last to first
	for (int i = LastAddr; i > 0; i--) {
	 	retCount = TMP107_ReadRegister (i, TMP107_Temp_reg, rx);
	 	printf("retcount of temp reg read: %d\n", retCount);
		printf("Data read: %x, 	%x\n", rx[1], rx[0]);
		printf("							Temp of device %d is %f\n", i, TMP107_DecodeTemperatureResult(rx[1], rx[0]));
	}

// power off and close port
	system("../pwroff.sh");

	// close port on way out
	retCount = TMP107_close();
	printf("Close status %d\n", retCount);
}

// dump present EEPROM contents
void DumpEEPROM(int16_t device) {
	int16_t rx[64];
	int16_t retCount;

	for (int i = 0; i < 8; i++) {
		retCount = TMP107_ReadRegister(device, TMP107_EEPROM1_reg + i, rx);
	 	printf("							EEPROM Location %d Data read: %x%x\n", i, rx[1], rx[0]);

	}
}

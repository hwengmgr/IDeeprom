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

int main()
{
	//char msg[20];
	//int16_t tx[8];
	int16_t rx[64];
	int16_t retCount;


// turn power on to modules
	system("../pwron.sh");

	retCount = TMP107_start();  // open port
  
// get last addr
	printf("***Start last device poll\n");
	int16_t LastAddr = TMP107_LastDevicePoll();
	printf("This program assumes Global Addr Init has already been run and Alert threshold voltages have been set.\n");
	printf ("Blinking red LED on %d devices for 20 seconds. \n", LastAddr); 

// calculate time between LED changes. Assume 1 second to go through all LEDs
	int16_t WaitTime = 1000 / LastAddr; // in msecs
	printf("Time between LED changes: %d\n", WaitTime);

	int16_t WriteData;

	for (int s = 0; s < 20; s++) {  // do 20 loops
//turn on red led devices from last to first
		for (int i = LastAddr; i > 0; i--) {
	 		retCount = TMP107_ReadRegister (i, TMP107_Conf_reg, rx);
	 		printf("retcount of conf reg read: %d\n", retCount);
			printf("Data read: %x, 	%x\n", rx[1], rx[0]);
			WriteData = (rx[1] << 8) + (rx[0] | 0x8);  // Set "8" bit to 1
			printf("					rx1 shifted %x	rx0  %x		WriteData: %x\n", rx[1] << 8, rx[0], WriteData);
			retCount = TMP107_WriteRegister (i, TMP107_Conf_reg, WriteData);
		}

		usleep(1000000/2); // 1 second

// turn off red LED
		for (int i = LastAddr; i > 0; i--) {
	 		retCount = TMP107_ReadRegister (i, TMP107_Conf_reg, rx);
	 		printf("retcount of conf reg read: %d\n", retCount);
			printf("Data read: %x, 	%x\n", rx[1], rx[0]);
			WriteData = (rx[1] << 8) + (rx[0] & 0xf7); // Set "8" bit to 0
			printf("					rx1 shifted %x	rx0  %x		WriteData: %x\n", rx[1] << 8, rx[0], WriteData);
			retCount = TMP107_WriteRegister (i, TMP107_Conf_reg, WriteData);
		}
		usleep(1000000/2); // 1 second

	}
// power off and close port
	system("../pwroff.sh");

	// close port on way out
	retCount = TMP107_close();
	printf("Close status %d\n", retCount);
}

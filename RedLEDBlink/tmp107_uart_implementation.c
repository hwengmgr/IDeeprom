/*
 * Copyright (c) 2018, Teradyne Inc
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

/*
* This File contains the lowest level code for communicating with the TMP107 chain. It has the open, close, write (transmit), read (retrieve) and waitForEcho routines that are
* called by the higher level routines in tmp107.c.
*/

#include "tmp107_arch_config.h"
#include "tmp107_uart.h"
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

// Adapted from
// http://www.raspberry-projects.com/pi/programming-in-c/uart-serial-port/using-the-uart

//#define TTYDEV "/dev/ttyUSB0"
//#define TTYDEV "/dev/serial0"
#define TTYDEV "/dev/ttyAMA0"  //tx3/rx3
static int uart_fd = -1;

/// application globals
char tmp107_rx[128]; // receive buffer
char tmp107_rxcnt; // receive count tracks current location in rx buffer

/**
 * tmp107_uart_select_port() - select the UART port index to use
 *                                THE IMPLEMENTATION IS OPTIONAL ON SINGLE-PORT
 *                                SETUPS (only one SPS30)
 *
 * Return:      0 on success, an error code otherwise
 */
int16_t tmp107_uart_select_port(uint8_t port) {
    return 0;
}



int16_t tmp107_uart_open() {
    // The flags (defined in fcntl.h):
    //    Access modes (use 1 of these):
    //        O_RDONLY - Open for reading only.
    //        O_RDWR - Open for reading and writing.
    //        O_WRONLY - Open for writing only.
    //    O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode.
    //      When set read requests on the file can return immediately with a
    //      failure status if there is no input immediately available (instead
    //      of blocking). Likewise, write requests can also return immediately
    //      with a failure status if the output can't be written immediately.
    //    O_NOCTTY - When set and path identifies a terminal device, open()
    //      shall not cause the terminal device to become the controlling
    //      terminal for the process.
    uart_fd = open(TTYDEV, O_RDWR | O_NOCTTY);  // add O_NDELAY for non blocking
    if (uart_fd == -1) {
        fprintf(stderr, "Error opening UART. Ensure it's not otherwise used\n");
        return -1;
    }

    // see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html:
    //    CSIZE:- CS5, CS6, CS7, CS8
    //    CLOCAL - Ignore modem status lines
    //    CREAD - Enable receiver
    //    IGNPAR = Ignore characters with parity errors
    //    ICRNL - Map CR to NL on input (Use for ASCII comms where you want to
    //                                   auto correct end of line characters,
    //                                   don't use for bianry comms)
    //    PARENB - Parity enable
    //    PARODD - Odd parity (else even)
    struct termios options;
    tcgetattr(uart_fd, &options);
    options.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;  // set baud rate
//    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;  // set baud rate
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
        
    //set timeout on read
    options.c_cc[VTIME] = 10; //1sec
    options.c_cc[VMIN] = 0;
    
    tcflush(uart_fd, TCIFLUSH);
    tcsetattr(uart_fd, TCSANOW, &options);
    return 0;
}

int16_t tmp107_uart_close() {
    return close(uart_fd);
}

int16_t tmp107_uart_tx(uint16_t data_len, const uint8_t *data) {
    if (uart_fd == -1)
        return -1;

    return write(uart_fd, (void *)data, data_len);
}

int16_t tmp107_uart_rx(uint16_t max_data_len, uint8_t *data) {
    if (uart_fd == -1)
        return -1;

    return read(uart_fd, (void *)data, max_data_len);
}
void tmp107_sleep_usec(uint32_t useconds) {
    usleep(useconds);
}

int16_t TMP107_Transmit(char* tx_data, char tx_size) {

	//char i;
	////clear receive buffer
	//tmp107_rxcnt = 0;
	////send calibration byte
	//USCI_A_UART_transmitData(USCI_A0_BASE,0x55);
	////send rest of packet
	//for (i = 0; i < tx_size; i++)
	//	USCI_A_UART_transmitData(USCI_A0_BASE,tx_data[i]);
	//*****************************************************
	char sync[1];
	sync[0]=0x55;
	
//----- TX BYTES -----

	if (uart_fd != -1)
	{
		// send calibration byte 0x55
		int count = write(uart_fd, &sync, 1);
    usleep (PKTTIME * 1);  // sleep PKTTIME for sync byte to transfer
		if (count < 0)
		{
			printf("UART TX error\n");
		}
		//send rest of packet
		int datcount = write(uart_fd, tx_data, tx_size);		//Filestream, bytes to write, number of bytes to write
    usleep (PKTTIME * tx_size );  // sleep PKTTIME for each byte transferred
		if (datcount < 0)														// Or Use a FOR LOOP if this is not working
		{
			printf("UART TX error\n");
		}
		//Bytes sent
		int i;
		printf("%i bytes sent : \n", datcount); // Will be commented
		for (i=0;i<datcount;i++){
			 printf("%x ", tx_data[i]);  // print bytes out
		 }
		printf("\n");
		return(datcount);
	}
}

// Not needed?
int16_t TMP107_WaitForEcho(char tx_size, char rx_size, int timeout_ms){

	/* used after a call to Transmit, this function will exit once the transmit echo
	 * and any additional rx bytes are received. it will also exit due to time out
	 * if timeout_ms lapses without receiving new bytes. this function returns the
	 * number of bytes received after tx echo.
	 */
	char i = 0;
	int count_ms = 0;
	char expected_rxcnt;
	// echo of cal byte + echo of transmission + additional bytes if read command
	expected_rxcnt = 1 + tx_size + rx_size;
	/* loop synopsis:
	 * wait for expected_rxcnt
	 * check once per millisecond, up to 40ms time out
	 * reset time out counter when a byte is received
	 *
	 * this loop runs while UART RX is being handled by ISR,
	 * and reacts to the number of bytes that are currently
	 * in the RX buffer.
	 *
	 * it is essential that all bytes are received, or that the
	 * apropriate timeout has been endured, before another
	 * transmit can occur. otherwise, corruption can occur.
	 */
	while (count_ms < timeout_ms){
		if (tmp107_rxcnt < expected_rxcnt) {
#
			if (tmp107_rxcnt > i)
				count_ms = 0;
			i = tmp107_rxcnt;
//			__delay_cycles(TMP107_Wait_ms);
			usleep(1000);  // sleep 1 ms
			count_ms++;
		} else {
			count_ms = timeout_ms;
		}
	}

	return (tmp107_rxcnt - 1 - tx_size);
}


int16_t TMP107_CheckEcho(char* tx_data, char tx_size){
// not needed???

}


int16_t TMP107_RetrieveReadback(char tx_size, int16_t* rx_data, char rx_size){

	// copy bytes received from UART buffer to user supplied array
	/*char i;
	if (rx_size > 0){
		for (i = 0; i < rx_size; i++){
			rx_data[i] = tmp107_rx[1 + tx_size + i];
		}
	}*/
	//****************************************************
//----- CHECK FOR ANY RX BYTES -----
    int rx_length =0;
    char rx_bytes[32];  // to get byte data from read
    
	if (uart_fd != -1)
	{
   // wait long enough for rx_size bytes to be read back. 1msec per byte plus PKTTIME's for margin
   usleep (PKTTIME * (rx_size + 2));
   
		// Read up to 255 characters from the port if they are there
		//unsigned char rx_buffer[256];
		if (rx_size > 0)
			rx_length = read(uart_fd, (void*)rx_bytes, rx_size);		//Filestream, buffer to store in, number of bytes to read (max)
		if (rx_length < 0)
		{
			//An error occured (will occur if there are no bytes)
			//return (-1);
		}
		else if (rx_length == 0)
		{
			//No data waiting
			//return (0);
		}
		else
		{
			//Bytes received
			int i;
			printf("%i bytes read : \n", rx_length); // Will be commented
			for (i=0;i<rx_length;i++){
				 rx_data[i] = rx_bytes[i];  // copy bytes into int16 array
				 printf("%x ", rx_data[i]);  // print bytes out
			 }
			rx_data[rx_length] = '\0';
			return (rx_length);
		}
	}
}

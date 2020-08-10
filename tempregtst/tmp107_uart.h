/*
 * Copyright (c) 2020, Teradyne Inc.
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

#ifndef TMP107_UART_H
#define TMP107_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tmp107_arch_config.h"

// to change baud rate change both of these lines. Use only integers for calculating PKTTIME. PKTTIME is in usec.
#define BAUDRATE B38400 //B9600
// 96000 baud is 104usec/bit * 10 bits per PKT = 1040 + 1/2 bit time for stop = 1100usec
#define PKTTIME 275 //1100


/**
 * tmp107_uart_select_port() - select the UART port index to use
 *                                THE IMPLEMENTATION IS OPTIONAL ON SINGLE-PORT
 *                                SETUPS (only one SPS30)
 *
 * Return:      0 on success, an error code otherwise
 */
int16_t tmp107_uart_select_port(uint8_t port);

/**
 * tmp107_uart_open() - initialize UART
 *
 * Return:      0 on success, an error code otherwise
 */
int16_t tmp107_uart_open();

/**
 * tmp107_uart_close() - release UART resources
 *
 * Return:      0 on success, an error code otherwise
 */
int16_t tmp107_uart_close();

int16_t TMP107_Transmit(char* tx_data, char tx_size);
int16_t TMP107_WaitForEcho(char tx_size, char rx_size, int timeout_ms);
int16_t TMP107_CheckEcho(char* tx_data, char tx_size);
int16_t TMP107_RetrieveReadback(char tx_size, int16_t* rx_data, char rx_size);

/**
 * tmp107_uart_tx() - transmit data over UART
 *
 * @data_len:   number of bytes to send
 * @data:       data to send
 * Return:      Number of bytes sent or a negative error code
 */
int16_t tmp107_uart_tx(uint16_t data_len, const uint8_t *data);

/**
 * tmp107_uart_rx() - receive data over UART
 *
 * @data_len:   max number of bytes to receive
 * @data:       Memory where received data is stored
 * Return:      Number of bytes received or a negative error code
 */
int16_t tmp107_uart_rx(uint16_t max_data_len, uint8_t *data);

/**
 * Sleep for a given number of microseconds. The function should delay the
 * execution for at least the given time, but may also sleep longer.
 *
 * Despite the unit, a <10 millisecond precision is sufficient.
 *
 * @param useconds the sleep time in microseconds
 */
void tmp107_sleep_usec(uint32_t useconds);

#ifdef __cplusplus
}
#endif

#endif /* TMP107_UART_H */

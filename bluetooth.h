/*
 * bluetooth.h
 *
 *  Created on: 2015-2-2
 *      Author: liqi
 */

#ifndef UART_H_
#define UART_H_
#include <stdint.h>
#include <stdbool.h>

void bluetooth_init();
void write_bluetooth(const char * chr);
void int2str(int32_t num, char * str);

#endif /* UART_H_ */

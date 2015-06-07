/*
 *  UART0.h
 *  Created on: 2015年5月1日
 *  Author: HuangKan
 *  Description: TODO
 */

#ifndef UART0_H_
#define UART0_H_

#include <stdint.h>

//公有符号定义
#define ENDLINE ("\n")

void UART0_Init(void);

void UART_Send_String(char str[]);
void UART0_Interrupt();
void UART_Int_To_String(uint32_t num, char* pString, \
		uint8_t numLen, uint8_t numBase);
uint8_t UART_Int_Len(uint32_t num, uint8_t numBase);
#endif /* UART0_H_ */

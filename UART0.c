/*
 *  UART0.c
 *  Created on: 2015年5月1日
 *  Author: HuangKan
 *  Description:
 */
#include "UART0.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "inc/hw_nvic.h"
#include "inc/hw_ints.h"

#include "utils/uartstdio.h"

#define UART0_TIMEOUT_TICKS 50
uint8_t UART0_Ticks = 0;

extern uint8_t Motor_RPS_Sampled;
extern float g_L_Sample_RPS;
extern float g_R_Sample_RPS;

void UART0_Printf(void)
{
    UARTprintf("Left 100*rps:  %d\n",(uint32_t)(g_L_Sample_RPS*100));
    UARTprintf("Right 100*rps: %d\n",(uint32_t)(g_R_Sample_RPS*100));
}

void UART0_Check_Timeout(void)
{
    UART0_Ticks ++;
    if (UART_UPDATE_TICKS==UART0_Ticks){
        UART0_Ticks = 0;
        //UART0超时处理
        if (1==Motor_RPS_Sampled)
        {
            UART0_Printf();
        }
    }
}
//    UART_counter ++;
//    if (UART_UPDATE_TICKS==UART_counter){
//        UART_counter = 0;
//        UARTprintf("Left rps:  %d\n",(uint32_t)(QEIVelocityGet(LEFT_QEI)));
//        UARTprintf("Right rps: %d\n",(uint32_t)(QEIVelocityGet(RIGHT_QEI)));
//        //UARTprintf("Left Position:  %d\n",QEIPositionGet(LEFT_QEI));
//        //UARTprintf("Right Position: %d\n",QEIPositionGet(RIGHT_QEI));
//    }

//向串口0发送一字符串
void UART_Send_String(char str[])
{
	char length,i;
	length = strlen(str);

	//一个字符一个字符的打印到电脑上
	for (i=0;i<length;i++)
	{
		if (str[i] == '\n')
			UARTCharPut(UART0_BASE,'\r');
		UARTCharPut(UART0_BASE,str[i]);
	}
}

//串口0中断处理函数，该例中用来处理接收中断
void UART0_Interrupt()
{
	 unsigned long ulStatus;
	 char lInChar;

	//获取串口0中断状态
    ulStatus = UARTIntStatus(UART0_BASE, true);
	//清除对应中断状态
    UARTIntClear(UART0_BASE, ulStatus);

	//接收接收缓冲区中全部数据，该例中FIFO关闭，数据缓冲区深度
	//为1B，所以缓冲区中只有一个字节的数据
	while(UARTCharsAvail(UART0_BASE))
    {
		//接收到数据后发回到电脑
        lInChar = UARTCharGetNonBlocking(UART0_BASE);
		UARTCharPut(UART0_BASE,lInChar);
    }
}

//串口0初始化函数
void UART0_Init(void)
{
	//使能端口A
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	//使能外设串口0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	//配置引脚功能
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

	//设置引脚模式
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	//设置串口0时钟源，片内16M精准晶振
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

	//配置串口0，波特率，数据长度，停止位，校验位
	UARTConfigSetExpClk(UART0_BASE,16000000, 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	//关闭串口FIFO，使得接收缓冲区深度为1字节，这样当接收到一个字节的数据立马就能产生中断
	UARTFIFODisable(UART0_BASE);

	//串口0，中断处理函数注册
	UARTIntRegister(UART0_BASE,UART0_Interrupt);
	//使能串口0接收中断
	UARTIntEnable(UART0_BASE,UART_INT_RX );
	//Console设置
	UARTStdioConfig(0, 115200, 16000000);
}

void UART_Int_To_String(uint32_t num, char* pString, \
		uint8_t numLen, uint8_t numBase)
{
	pString[numLen] = 0x00;
	int i = 0;
	int remainder;
	for (i=1; i<numLen+1; i++)
	{
		remainder = num % numBase;
		num /= numBase;
		pString[numLen-i] = remainder + 0x30;
	}
}

uint8_t UART_Int_Len(uint32_t num, uint8_t numBase)
{
	int len = 0;
	do
	{
		len++;
	}while ((num = num / numBase) != 0);
	return len;
}

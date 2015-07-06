/*
 * uart.cpp
 *
 *  Created on: 2015-2-2
 *      Author: liqi
 */

#include "bluetooth.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"

extern char com_uart[10];

extern "C" void UARTIntHandler(void)
{
    uint32_t ui32Status;
    uint8_t i = 0;
    ui32Status = UARTIntStatus(UART7_BASE, true); //get interrupt status

    UARTIntClear(UART7_BASE, ui32Status); //clear the asserted interrupts

    for(int j = 0; j < 4; j++)
    {
    	 while(!UARTCharsAvail(UART7_BASE));
    	 com_uart[i++] = UARTCharGetNonBlocking(UART7_BASE);
    }
    com_uart[i] = '\0';
}


void bluetooth_init()
{
	//enable the GPIO E
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	//Enable UART7
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);

	//configure GPIO PE0 and PE1 as UARTs
	GPIOPinConfigure(GPIO_PE0_U7RX);
	GPIOPinConfigure(GPIO_PE1_U7TX);
	GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART7_BASE, SysCtlClockGet(), 115200,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTFIFOLevelSet(UART7_BASE, UART_FIFO_TX1_8, UART_FIFO_RX2_8);
    UARTIntRegister(UART7_BASE, UARTIntHandler);
    IntEnable(INT_UART7);
    UARTIntEnable(UART7_BASE, UART_INT_RX);
}

void write_bluetooth(const char * chr)
{
	uint8_t length = strlen(chr);

	for(int i = 0; i < length; i++)
	{
			UARTCharPut(UART7_BASE, chr[i]);
	}
}

void int2str(int32_t num, char * str)
{
	int i = 0, div = 1000000000;
	if(num < 0)
	{
		str[i++] = '-';
		num = -num;
	}
	if(num != 0)
	{
		while(0 == num / div)
		{
			num = num % div;
			div = div / 10;
		}
		while(div != 0)
		{
			str[i] = num / div + 48;
			num %= div;
			div /= 10;
			i++;
		}
		str[i] = '\0';
	}
	else
	{
		str[0] = 48;
		str[1] = '\0';
	}
}

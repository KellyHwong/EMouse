/*
 * Port.h
 *
 *  Created on: 2015年6月6日
 *      Author: HuangKan
 */

#ifndef PORT_H_
#define PORT_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "inc/tm4c123gh6pm.h"

//QEI
#define RIGHT_QEI QEI0_BASE
#define LEFT_QEI QEI1_BASE
//所用计时器
#define NEC_TIMER_PERIPH SYSCTL_PERIPH_TIMER0
#define NEC_TIMER TIMER0_BASE

#define PID_TIMER_PERIPH SYSCTL_PERIPH_WTIMER0
#define PID_TIMER WTIMER0_BASE
#define INT_PID_TIMER INT_WTIMER0A

#define SCH_TIMER_PERIPH SYSCTL_PERIPH_WTIMER1
#define SCH_TIMER WTIMER1_BASE
#define INT_SCH_TIMER INT_WTIMER1A

#endif /* PORT_H_ */

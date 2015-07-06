/*
 * InfSen.h
 *
 *  Created on: 2015年6月6日
 *      Author: HuangKan
 */

#ifndef INFSEN_H_
#define INFSEN_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "inc/tm4c123gh6pm.h"

//TODO 写光传感器感知状态的定义
#define OUT_OF_LINE 0  //全0，不在线上
#define LEFT_ON_LINE 1  //
#define RIGHT_ON_LINE 2  //
#define PERPENDICULAR_WITH_LINE_1 3  //中间与线垂直
#define CASE4 4  //
#define CASE5 5  //
#define CASE6 6  //
#define CASE7 7  //
#define CASE8 8
#define CASE9 9
#define CASE10 10
#define CASE11 11
#define PERPENDICULAR_WITH_LINE_2 12  //下方与线垂直
#define CASE13 13
#define CASE14 14
#define CASE15 15
//走正，但是也有可能是和线垂直，取决于下一步的状态；
//如果下一步是走正则继续走，否则遇到了垂直的情况（case3），就转弯来回到走正的状态
#define GO_STRAIGHT 16
#define CASE17 17
#define CASE18 18
#define CASE19 19
#define CASE20 20
#define CASE21 21
#define CASE22 22
#define CASE23 23
#define CASE24 24
#define CASE25 25
#define CASE26 26
#define CASE27 27
#define CASE28 28
#define CASE29 29
#define CASE30 30
#define CASE31 31

#define RIGHT_AHEAD 16
#define LITTLE_LEFT
#define MIDDLE_LEFT
#define TOO_LEFT

#define IN_SEN_1_PERIPH SYSCTL_PERIPH_GPIOA
#define IN_SEN_1_PORT GPIO_PORTA_BASE
#define IN_SEN_1_PIN GPIO_PIN_5

#define IN_SEN_2_PERIPH SYSCTL_PERIPH_GPIOD
#define IN_SEN_2_PORT GPIO_PORTD_BASE
#define IN_SEN_2_PIN GPIO_PIN_2

#define IN_SEN_3_PERIPH SYSCTL_PERIPH_GPIOE
#define IN_SEN_3_PORT GPIO_PORTE_BASE
#define IN_SEN_3_PIN GPIO_PIN_2

#define IN_SEN_4_PERIPH SYSCTL_PERIPH_GPIOE
#define IN_SEN_4_PORT GPIO_PORTE_BASE
#define IN_SEN_4_PIN GPIO_PIN_3

#define IN_SEN_5_PERIPH SYSCTL_PERIPH_GPIOB
#define IN_SEN_5_PORT GPIO_PORTB_BASE
#define IN_SEN_5_PIN GPIO_PIN_3

#define INFRARED_1_PERIPH SYSCTL_PERIPH_GPIOB
#define INFRARED_1_PORT GPIO_PORTB_BASE
#define INFRARED_1_PIN GPIO_PIN_2

#define INFRARED_2_PERIPH SYSCTL_PERIPH_GPIOA
#define INFRARED_2_PORT GPIO_PORTA_BASE
#define INFRARED_2_PIN GPIO_PIN_4

#define INFRARED_3_PERIPH SYSCTL_PERIPH_GPIOA
#define INFRARED_3_PORT GPIO_PORTA_BASE
#define INFRARED_3_PIN GPIO_PIN_3

#define INFRARED_4_PERIPH SYSCTL_PERIPH_GPIOA
#define INFRARED_4_PORT GPIO_PORTA_BASE
#define INFRARED_4_PIN GPIO_PIN_2

#define INFRARED_5_PERIPH SYSCTL_PERIPH_GPIOC
#define INFRARED_5_PORT GPIO_PORTC_BASE
#define INFRARED_5_PIN GPIO_PIN_7

void InfSen_Init(void);
uint8_t InfSen_Read(void);

#endif /* INFSEN_H_ */

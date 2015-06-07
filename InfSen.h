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

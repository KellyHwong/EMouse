/*
 * InfSen.c
 *
 *  Created on: 2015年6月6日
 *      Author: HuangKan
 */

#include "InfSen.h"

void InfSen_Init(void) {
  //
  SysCtlPeripheralEnable(IN_SEN_1_PERIPH);
  SysCtlPeripheralEnable(IN_SEN_2_PERIPH);
  SysCtlPeripheralEnable(IN_SEN_3_PERIPH);
  SysCtlPeripheralEnable(IN_SEN_4_PERIPH);
  SysCtlPeripheralEnable(IN_SEN_5_PERIPH);
  //
  GPIOPinTypeGPIOInput(IN_SEN_1_PORT, IN_SEN_1_PIN);
  GPIOPinTypeGPIOInput(IN_SEN_2_PORT, IN_SEN_2_PIN);
  GPIOPinTypeGPIOInput(IN_SEN_3_PORT, IN_SEN_3_PIN);
  GPIOPinTypeGPIOInput(IN_SEN_4_PORT, IN_SEN_4_PIN);
  GPIOPinTypeGPIOInput(IN_SEN_5_PORT, IN_SEN_5_PIN);
  //
  SysCtlPeripheralEnable(INFRARED_1_PERIPH);
  SysCtlPeripheralEnable(INFRARED_2_PERIPH);
  SysCtlPeripheralEnable(INFRARED_3_PERIPH);
  SysCtlPeripheralEnable(INFRARED_4_PERIPH);
  SysCtlPeripheralEnable(INFRARED_5_PERIPH);
  //
  GPIOPinTypeGPIOOutput(INFRARED_1_PORT, INFRARED_1_PIN);
  GPIOPinTypeGPIOOutput(INFRARED_2_PORT, INFRARED_2_PIN);
  GPIOPinTypeGPIOOutput(INFRARED_3_PORT, INFRARED_3_PIN);
  GPIOPinTypeGPIOOutput(INFRARED_4_PORT, INFRARED_4_PIN);
  GPIOPinTypeGPIOOutput(INFRARED_5_PORT, INFRARED_5_PIN);
  //
  GPIOPinWrite(INFRARED_1_PORT, INFRARED_1_PIN, 0xFF);
  GPIOPinWrite(INFRARED_2_PORT, INFRARED_2_PIN, 0xFF);
  GPIOPinWrite(INFRARED_3_PORT, INFRARED_3_PIN, 0xFF);
  GPIOPinWrite(INFRARED_4_PORT, INFRARED_4_PIN, 0xFF);
  GPIOPinWrite(INFRARED_5_PORT, INFRARED_5_PIN, 0xFF);
}

uint8_t InfSen_Read(void) {
  uint8_t ui8Value = 0;
  if (GPIOPinRead(IN_SEN_1_PORT, IN_SEN_1_PIN))
    ui8Value += 1;
  if (GPIOPinRead(IN_SEN_2_PORT, IN_SEN_2_PIN))
    ui8Value += 2;
  if (GPIOPinRead(IN_SEN_3_PORT, IN_SEN_3_PIN))
    ui8Value += 4;
  if (GPIOPinRead(IN_SEN_4_PORT, IN_SEN_4_PIN))
    ui8Value += 8;
  if (GPIOPinRead(IN_SEN_5_PORT, IN_SEN_5_PIN))
    ui8Value += 16;
  return ui8Value;
}

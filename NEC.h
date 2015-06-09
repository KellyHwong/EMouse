/*
 *  nec.h
 *  Created on: 2015年5月4日
 *  Author: HuangKan
 *  Description: 红外NEC协议解码
 */

#ifndef NEC_H_
#define NEC_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"

//To Adapt，根据接口的端口做相应的修改
#define INFRA_RED_PORT GPIO_PORTE_BASE
#define INFRA_RED_PIN GPIO_PIN_0
//所用中断
#define INT_NEC_TIMER INT_TIMER0A
#define INT_INFRA_RED_PORT INT_GPIOE_TM4C123
//用户码定义
#define NEC_ADDRESS 0x00

//其他宏定义
#define MAX_RAW_BITS 128
#define NEC_TIMEOUT_TICKS 50//10mS per tick

typedef enum
{
	LeadZeros,
	LeadOnes,
	WaitOne,
	CountOnes,
	TransmitSuccess,//32位数据传送成功
	TransmitError,//32位数据传送失败
	RepeatSignal
}NECMachineStatue;

//所有可能被解析命令
typedef enum
{
	//0~9按顺序对应
	Num0,//数字0
	Num1,//数字1
	Num2,//数字2
	Num3,//数字3
	Num4,//数字4
	Num5,//数字5
	Num6,//数字6
	Num7,//数字7
	Num8,//数字8
	Num9,//数字9
	Repeat,//重复信号，按着按钮不动时接收到
	AddressError,//用户码错误
	ValidateError,//校验错误
	UnknownCommand,//未知
	ChannelMinus,//CH-
	Channel,//CH
	ChannelPlus,//CH+
	FastLeft,//快退
	FastRight,//快进
	Pause,//暂停
	Minus,//减号
	Plus,//加号
	Equal,//EQ
	Num100Plus,//100+
	Num200Plus,//200+
	//为命令状态机增加一个状态
	WaitCommand//等待命令态
}NECCommand;

NECMachineStatue machineState;

uint64_t ui64InfraRawCode[2];
uint32_t ui32NECEncoded;

//控制PE0口中断的变量
uint32_t ui32PE0IntEnable;
//传输控制变量（读PE0端口使能）
uint8_t ui8TransmitEnable;

uint8_t ui8RawBitCounter;
uint8_t ui8EncodedBitCounter;

uint8_t ui8LeadOnesCounter;
uint8_t ui8CountOnesCounter;

void NEC_Init(void);
void InfraPortInit();
void InfraPortIntHandler();
void NECTimerInit();
void NECTimerIntHandler();

void NECReceiveReset(bool dataRestore);
NECMachineStatue StateMachine(NECMachineStatue stateNow, uint8_t bit);
void NECReceiveBit(uint8_t bit);

NECCommand NECCommandExtract(uint32_t ui32NECEncoded);
NECCommand NECCommandMenu(NECCommand necCommandMenuStatus, NECCommand necCommand);
NECCommand NECCommandExecute(NECCommand necCommand);

void NEC_LED_Off(void);
void NEC_LED_On(void);
#endif /* NEC_H_ */

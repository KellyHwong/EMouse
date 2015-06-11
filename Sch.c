/*
 * Sch.c
 *
 *  Created on: 2015年6月11日
 *      Author: HuangKan
 */

#ifndef SCH_C_
#define SCH_C_

#include "Sch.h"
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/qei.h"

#include "Port.h"
#include "Motor.h"
#include "driverlib/qei.h"
#include "Seg.h"
#include "UART0.h"
#include "utils/uartstdio.h"

#include "NEC.h"
#include "PID.h"

#define SCH_DELAY_TIME 10//mS

void Sch_Timer_ISR(void)
{
    //合作式调度器，10mS调度一次
    TimerIntClear(SCH_TIMER, TIMER_TIMA_TIMEOUT);
    //数码管输出设置，见Seg.c
    Seg_Update();
    //NEC的LED超时设置
    NEC_LED_Check_Timeout();
    //
//    Motor_Check_Timeout();
    //更新速度
    Motor_Sample_RPS();
    //
    UART0_Check_Timeout();
    //
    PID_Check_Timeout();
}

void Sch_Init(void)
{
    Sch_Init_Timer();
}

void Sch_Init_Timer(void)
{
    uint32_t ui32SystemClock;
    SysCtlPeripheralEnable(SCH_TIMER_PERIPH);
    //设置为周期性定时器
    TimerConfigure(SCH_TIMER, TIMER_CFG_PERIODIC);
    //设置为系统时钟 50MHz
    TimerClockSourceSet(SCH_TIMER, TIMER_CLOCK_SYSTEM);
    ui32SystemClock = SysCtlClockGet();
    //50,000-1就是1ms, 50-1就是1us
    TimerLoadSet(SCH_TIMER, TIMER_A, ui32SystemClock/1000*SCH_DELAY_TIME - 1);

    TimerIntRegister(SCH_TIMER, TIMER_A, Sch_Timer_ISR);
    TimerIntClear(SCH_TIMER, TIMER_TIMA_TIMEOUT);
    //开始计时
    TimerEnable(SCH_TIMER, TIMER_A);
    //允许处理中断
    TimerIntEnable(SCH_TIMER, TIMER_TIMA_TIMEOUT);
}

#endif /* SCH_C_ */

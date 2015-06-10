// Private includes
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/fpu.h"
#include "driverlib/qei.h"
#include "driverlib/interrupt.h"
#include "utils/uartstdio.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/timer.h"

#include "EMouse.h"
#include "Motor.h"
#include "NEC.h"
#include "InfSen.h"
#include "Seg.h"
#include "UART0.h"
#include "PID.h"
#include "Port.h"

//PID的外部变量
extern float g_L_RPS;
extern float g_L_Dir;
//
extern uint16_t g_L_Cur_PWM;
extern uint16_t g_R_Cur_PWM;
//PID参数调试
extern float g_L_Kp;
extern float g_R_Kp;

int main(void)
{
    //
    EMouse_Init();
    //
    //PID_Move(1,1,2*71/15,2*71/15);
    while (1)
    {
        // Do nothing
    }
    return 1;
}

//电脑鼠初始化，都往这里面写
void EMouse_Init(void)
{
    //系统时钟50MHz
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | \
        SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    //开启浮点运算单元
    FPUEnable();
    //电机驱动初始化
    Motor_Init();
    //红外接收管初始化
    NEC_Init();
    //5个光传感器初始化
    InfSen_Init();
    //UART调试口初始化
    UART0_Init();
    //PID计时器初始化
    PID_Init();
    TimerIntEnable(PID_TIMER, TIMER_TIMA_TIMEOUT);
    //中断优先级设置
    IntPrioritySet(INT_NEC_TIMER, 0x20);//中断优先级为1
    //红外端口中断
    IntPrioritySet(INT_INFRA_RED_PORT, 0x00);//中断优先级为0（最高）
    //
    IntPrioritySet(INT_PID_TIMER, 0x00);
    //开启中断
    IntMasterEnable();
}

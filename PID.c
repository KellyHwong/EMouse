/*
 * PID.c
 *
 *  Created on: 2015年6月6日
 *      Author: HuangKan
 */

#include "PID.h"
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

#include "Port.h"
#include "Motor.h"
#include "driverlib/qei.h"
#include "Seg.h"
//私有定义
#define MAX_RPS 50//限制rps输出幅度
#define MAX_PWM 200//

//私有变量及其初始化
//PID系数
float g_L_Kp = 1;
float g_R_Kp = 0.5;
//
float g_L_Ki = 0;
float g_R_Ki = 0;
//
float g_L_Kd = 0;
float g_R_Kd = 0;

//rps与PWM的线性关系
//float g_L_A = 0.1316;//0.0838;//rps与PWM的线性关系
//float g_L_B = 0;//14.831;//rps = A*PWM + B
//float g_R_A = 0.1352;//0.0767;//rps与PWM的线性关系
//float g_R_B = 0;//18.132;//rps = A*PWM + B
//rps与PWM的分段线性关系
#define BOUNDARY 40.0
float g_L_A = 0.2156;
float g_L_A1 = 0.0355;
float g_L_B1 = 32.583;
float g_R_A = 0.241;
float g_R_A1 = 0.0282;
float g_R_B1 = 35.808;

float g_L_RPS = 1.0;
uint8_t g_L_Dir = 1;
float g_L_Pre_Err = .0;
float g_L_Integral = .0;
float g_R_RPS = 1.0;
uint8_t g_R_Dir = 1;
float g_R_Pre_Err = .0;
float g_R_Integral = .0;

void PID_Move(uint8_t l_Dir, uint8_t r_Dir, float l_RPS,  float r_RPS)
{
    //RPS不要超过50
    g_L_Dir = l_Dir;
    g_L_RPS = l_RPS;
    g_R_Dir = r_Dir;
    g_R_RPS = r_RPS;
    //清空误差
    g_L_Pre_Err = .0;
    g_L_Integral = .0;
    g_R_Pre_Err = .0;
    g_R_Integral = .0;
    PID_Start();
}

//PID控制开始，即响应PID定时器中断
void PID_Start(void)
{
    //先设定RPS
    PID_Set_Left_RPS(g_L_Dir, g_L_RPS);
    PID_Set_Right_RPS(g_R_Dir, g_R_RPS);
    //再开启中断
    TimerIntEnable(PID_TIMER, TIMER_TIMA_TIMEOUT);
}

//rps-PWM分段线性函数
float PID_Left_RPS_To_PWM(float rps)
{
    float PWM;
    if (rps < BOUNDARY) PWM = rps/g_L_A;
    else PWM = 1/g_L_A1 * (rps-g_L_B1);
    return PWM;
}

float PID_Right_RPS_To_PWM(float rps)
{
    float PWM;
    if (rps < BOUNDARY) PWM = rps/g_R_A;
    else PWM = 1/g_R_A1 * (rps-g_R_B1);
    return PWM;
}

//根据rps与PWM的线性关系逆推PWM的占空比
void PID_Set_Left_RPS(uint8_t dir, float rps)
{
    float PWM;
    uint16_t width;
    if (dir > 0) LeftMotor_Forward;
    else LeftMotor_Backward;
    PWM = PID_Left_RPS_To_PWM(rps);
    //限制PWM幅度
    if (PWM < 0) PWM = -PWM;
    if (PWM > MAX_PWM) PWM = MAX_PWM;
    width = (uint16_t)PWM;
    Motor_Left_PWM_Width(width);
}

void PID_Set_Right_RPS(uint8_t dir, float rps)
{
    float PWM;
    uint16_t width;
    if (dir > 0) RightMotor_Forward;
    else RightMotor_Backward;
    PWM = PID_Right_RPS_To_PWM(rps);
    //限制PWM幅度
    if (PWM < 0) PWM = -PWM;
    if (PWM > MAX_PWM) PWM = MAX_PWM;
    width = (uint16_t)PWM;
    Motor_Right_PWM_Width(width);
}

void PID_Init(void)
{
    PID_Init_Timer();
}

void PID_Init_Timer(void)
{
    uint32_t ui32Delta=PID_DELAY_TIME;
    uint32_t ui32SystemClock;
    SysCtlPeripheralEnable(PID_TIMER_PERIPH);
    //设置为周期性定时器
    TimerConfigure(PID_TIMER, TIMER_CFG_PERIODIC);
    //设置为系统时钟 50MHz
    TimerClockSourceSet(PID_TIMER, TIMER_CLOCK_SYSTEM);
    ui32SystemClock = SysCtlClockGet();
    //50,000-1就是1ms, 50-1就是1us
    //560us 间隔 读PE0电平
    TimerLoadSet(PID_TIMER, TIMER_A, ui32SystemClock/1000*ui32Delta - 1);
    IntEnable(INT_TIMER0A);
    TimerIntRegister(PID_TIMER, TIMER_A, PIDTimerIntHandler);
    TimerIntClear(PID_TIMER, TIMER_TIMA_TIMEOUT);
    //开始计时
    TimerEnable(PID_TIMER, TIMER_A);
    //不允许处理中断（开始时开启）
    TimerIntDisable(PID_TIMER, TIMER_TIMA_TIMEOUT);
}

void PIDTimerIntHandler(void)
{
    TimerIntClear(PID_TIMER, TIMER_TIMA_TIMEOUT);
    PID_Cali_Left_RPS();
    PID_Cali_Right_RPS();
    //TODO 数码管输出设置
    Seg_Display_Num((int)(g_L_Kp*1000)+(int)(g_R_Kp*10));
}

//PID调节算法
void PID_Cali_Left_RPS(void)
{
    float error;
    float measured_rps;
    float derivative;
    float output;
    measured_rps = QEIVelocityGet(LEFT_QEI)/512.0;//rps
    error = g_L_RPS - measured_rps;
    g_L_Integral = g_L_Integral + error*PID_DELAY_TIME/1000;
    derivative = (error - g_L_Pre_Err)/PID_DELAY_TIME*1000;
    g_L_Pre_Err = error;
    //预设值加上控制输出
    output = g_L_RPS + g_L_Kp*error + g_L_Ki*g_L_Integral + g_L_Kd*derivative;
    //限幅输出
    if (output > MAX_RPS) output = MAX_RPS;
    PID_Set_Left_RPS(g_L_Dir, output);
}

void PID_Cali_Right_RPS(void)
{
    float error;
    float measured_rps;
    float derivative;
    float output;
    measured_rps = QEIVelocityGet(RIGHT_QEI)/512.0;//rps
    error = g_R_RPS - measured_rps;
    g_R_Integral = g_R_Integral + error*PID_DELAY_TIME/1000;
    derivative = (error - g_R_Pre_Err)/PID_DELAY_TIME*1000;
    g_R_Pre_Err = error;
    //预设值加上控制输出
    output = g_R_RPS + g_R_Kp*error + g_R_Ki*g_R_Integral + g_R_Kd*derivative;
    //限幅输出
    if (output > MAX_RPS) output = MAX_RPS;
    PID_Set_Right_RPS(g_R_Dir, output);
}

//PID控制结束
void PID_Stop(void)
{
    TimerIntDisable(PID_TIMER, TIMER_TIMA_TIMEOUT);
}

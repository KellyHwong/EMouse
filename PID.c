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
#include "driverlib/qei.h"

#include "Port.h"
#include "Motor.h"
#include "driverlib/qei.h"
#include "Seg.h"
#include "UART0.h"
#include "utils/uartstdio.h"

#include "NEC.h"
//私有定义
#define MAX_RPS 10//限制rps输出幅度

//
#define BOUNDARY 45.0
//私有变量及其初始化
//PID系数
float g_L_Kp = 0.8;
float g_R_Kp = 0.8;
//
float g_L_Ki = 0.01;
float g_R_Ki = 0.01;
//
float g_L_Kd = 0.00;
float g_R_Kd = 0.00;

#define MAX_INTEGRAL (100.0)
#define MIN_INTEGRAL (-100.0)

//在地板上的时候的rps与PWM的关系
float g_L_A0 = 0.01370;  //rps = A*PWM + B
float g_L_B0 = -0.41874;
float g_R_A0 = 0.01267;
float g_R_B0 = 0.15181;

float g_L_RPS = .0;  //PID设定的RPS
uint8_t g_L_Dir = 1;
float g_L_Pre_Err = .0;
float g_L_Integral = .0;
float g_R_RPS = .0;
uint8_t g_R_Dir = 1;
float g_R_Pre_Err = .0;
float g_R_Integral = .0;
float g_Sum_RPS = 0;

//当前PWM脉宽
extern uint16_t g_L_Cur_PWM;
extern uint16_t g_R_Cur_PWM;
//
extern float g_L_Sample_RPS;
extern float g_R_Sample_RPS;
//
extern uint8_t UART_counter;
//
extern uint8_t NEC_LED_G;
extern uint8_t NEC_Time_Ticks;
//超时计数
uint8_t PID_G = 0;
extern uint8_t Motor_G;
uint8_t PID_Ticks = 0;
#define PID_TIMEOUT_TICKS 100
#define PID_CLEAR_TIMEOUT 200//mS清空一次误差
//两边转速差
#define PID_BALANCE_START_TIMEOUT 80
uint8_t diff_ticks = 0;
uint8_t diff_counter = 0;
float diff_rps = 0;  //两边转速差
float diff_Kp = 0.3;  //调整系数
float diff_Ki = 0.01;
float diff_Kd = 0.02;
float pre_diff_rps = 0;
float diff_Integral = 0;
float diff_deri = 0;
uint8_t success_flag = 0;
uint16_t success_count = 0;

#define MAX_RPS_DIFF 0.5
#define MIN_RPS_DIFF (-MAX_RPS_DIFF)
#define MAX_ADJ 0.5

//调用这个函数即可
void PID_Move(uint8_t l_Dir, uint8_t r_Dir, float l_RPS, float r_RPS) {
  Motor_G = 1;
  //防止为负数
  if (l_RPS < 0)
    l_RPS = 0;
  if (r_RPS < 0)
    r_RPS = 0;
  //限幅，不要超过MAX_RPS
  if (l_RPS > MAX_RPS)
    l_RPS = MAX_RPS;
  if (r_RPS > MAX_RPS)
    r_RPS = MAX_RPS;
  //
  g_L_Dir = l_Dir;
  g_L_RPS = l_RPS;
  g_R_Dir = r_Dir;
  g_R_RPS = r_RPS;
  //
  g_Sum_RPS = g_L_RPS + g_R_RPS;
}

//PID控制开始，即响应PID定时器中断
void PID_Start(void) {
  PID_G = 1;
  //先设定RPS
  PID_Set_Left_RPS(g_L_Dir, g_L_RPS);
  PID_Set_Right_RPS(g_R_Dir, g_R_RPS);
  //再开启中断
  TimerIntEnable(PID_TIMER, TIMER_TIMA_TIMEOUT);
}

void PIDTimerIntHandler(void) {
  float adj;
  TimerIntClear(PID_TIMER, TIMER_TIMA_TIMEOUT);
  //PID反馈控制算法（只能保证稳定，还不能保证两边速度一致）
  PID_Cali_Left_RPS();
  PID_Cali_Right_RPS();
  diff_counter++;
  diff_ticks++;
  //平衡算法800mS后开始
  if (diff_ticks >= PID_BALANCE_START_TIMEOUT) {
    diff_ticks = 0;
    //10分频
    if (diff_counter >= 1) {
      diff_counter = 0;
      diff_rps = g_L_Sample_RPS - g_R_Sample_RPS;
//        if (diff_rps<MIN_RPS_DIFF) diff_rps=MIN_RPS_DIFF;
//        if (diff_rps>MAX_RPS_DIFF) diff_rps=MAX_RPS_DIFF;
      if (0 == success_flag) {
        if (diff_rps > 1) {
          adj = 0.5 * diff_rps;
          //粗调
          PID_Move(g_L_Dir, g_R_Dir, g_L_RPS - adj, g_R_RPS + adj);
        } else if (diff_rps > 0.5) {
          adj = 0.1 * diff_rps;
          //微调
          PID_Move(g_L_Dir, g_R_Dir, g_L_RPS - adj, g_R_RPS + adj);
        } else if (diff_rps > 0.1) {
          adj = 0.05 * diff_rps;
          PID_Move(g_L_Dir, g_R_Dir, g_L_RPS - adj, g_R_RPS + adj);
        }
        //        else if (diff_rps>0.05){
        //            adj = 0.01;
        //            PID_Move(g_L_Dir,g_R_Dir,g_Sum_RPS/2-adj,g_Sum_RPS/2+adj);
        //        }
        else {
          adj = 0;
          if (g_L_Sample_RPS > 2) {
            success_count++;
            //TODO 记录此时的Setted_RPS和PWM
            if (success_count >= 1000)
              success_flag = 0;
            if (success_flag) {
              UARTprintf("Adjust Successfully!!!\n");
              UARTprintf("Left Setted RPS: %d\n", (uint16_t) (g_L_RPS * 100));
              UARTprintf("Right Setted RPS: %d\n", (uint16_t) (g_R_RPS * 100));
              UARTprintf("Left 100*rps:  %d\n",
                         (uint32_t) (g_L_Sample_RPS * 100));
              UARTprintf("Right 100*rps: %d\n",
                         (uint32_t) (g_R_Sample_RPS * 100));
            }
          }
          PID_Move(g_L_Dir, g_R_Dir, g_L_RPS - adj, g_R_RPS + adj);
        }
      }
//        diff_Integral += diff_rps*PID_DELAY_TIME/1000;
//        diff_deri = (diff_rps - pre_diff_rps)/PID_DELAY_TIME*1000;
//        pre_diff_rps = diff_rps;
//        adj = diff_Kp*diff_rps+diff_Ki*diff_Integral+diff_Kd*diff_deri;
//        if (adj<0) adj=-adj;
//        if (adj>MAX_ADJ) adj=MAX_ADJ;
//        PID_Move(g_L_Dir,g_R_Dir,g_Sum_RPS/2-adj,g_Sum_RPS/2+adj);
    }
  }
}

//rps-PWM线性函数
float PID_Left_RPS_To_PWM(float rps) {
  float PWM;
  PWM = 1 / g_L_A0 * (rps - g_L_B0);
  //防止PWM为负数
  if (PWM < 0)
    PWM = 1;
  return PWM;
}

float PID_Right_RPS_To_PWM(float rps) {
  float PWM;
  PWM = 1 / g_R_A0 * (rps - g_R_B0);
  //防止PWM为负数
  if (PWM < 0)
    PWM = 1;
  return PWM;
}

//根据rps与PWM的线性关系逆推PWM的占空比
void PID_Set_Left_RPS(uint8_t dir, float rps) {
  float PWM;
  uint16_t width;
  if (dir > 0)
    LeftMotor_Forward
    ;
  else
    LeftMotor_Backward
    ;
  PWM = PID_Left_RPS_To_PWM(rps);
  //限制PWM幅度
  if (PWM < 0)
    PWM = -PWM;
  if (PWM > MAX_PWM)
    PWM = MAX_PWM;
  width = (uint16_t) PWM;
  g_L_Cur_PWM = width;
  Motor_Left_PWM_Width(width);
}

void PID_Set_Right_RPS(uint8_t dir, float rps) {
  float PWM;
  uint16_t width;
  if (dir > 0)
    RightMotor_Forward
    ;
  else
    RightMotor_Backward
    ;
  PWM = PID_Right_RPS_To_PWM(rps);
  //限制PWM幅度
  if (PWM < 0)
    PWM = -PWM;
  if (PWM > MAX_PWM)
    PWM = MAX_PWM;
  width = (uint16_t) PWM;
  g_R_Cur_PWM = width;
  Motor_Right_PWM_Width(width);
}

void PID_Init(void) {
  PID_Init_Timer();
}

void PID_Init_Timer(void) {
  uint32_t ui32SystemClock;
  SysCtlPeripheralEnable(PID_TIMER_PERIPH);
  //设置为周期性定时器
  TimerConfigure(PID_TIMER, TIMER_CFG_PERIODIC);
  //设置为系统时钟 50MHz
  TimerClockSourceSet(PID_TIMER, TIMER_CLOCK_SYSTEM);
  ui32SystemClock = SysCtlClockGet();
  //50,000-1就是1ms, 50-1就是1us
  TimerLoadSet(PID_TIMER, TIMER_A, ui32SystemClock / 1000 * PID_DELAY_TIME - 1);

  TimerIntRegister(PID_TIMER, TIMER_A, PIDTimerIntHandler);
  TimerIntClear(PID_TIMER, TIMER_TIMA_TIMEOUT);
  //开始计时
  TimerEnable(PID_TIMER, TIMER_A);
  //不允许处理中断（开始时开启）
  TimerIntDisable(PID_TIMER, TIMER_TIMA_TIMEOUT);
}

//PID调节算法
void PID_Cali_Left_RPS(void) {
  float error;
  float measured_rps;
  float derivative;
  float output;
  measured_rps = g_L_Sample_RPS;    //rps
  error = g_L_RPS - measured_rps;
  g_L_Integral = g_L_Integral + error * PID_DELAY_TIME / 1000;
  derivative = (error - g_L_Pre_Err) / PID_DELAY_TIME * 1000;
  g_L_Pre_Err = error;
  //预设值加上控制输出
  output = g_L_RPS + g_L_Kp * error + g_L_Ki * g_L_Integral
      + g_L_Kd * derivative;
  //限幅输出
  if (output > MAX_RPS)
    output = MAX_RPS;
  PID_Set_Left_RPS(g_L_Dir, output);
}

void PID_Cali_Right_RPS(void) {
  float error;
  float measured_rps;
  float derivative;
  float output;
  measured_rps = g_R_Sample_RPS;    //rps
  error = g_R_RPS - measured_rps;
  g_R_Integral = g_R_Integral + error * PID_DELAY_TIME / 1000;
  derivative = (error - g_R_Pre_Err) / PID_DELAY_TIME * 1000;
  g_R_Pre_Err = error;
  //预设值加上控制输出
  output = g_R_RPS + g_R_Kp * error + g_R_Ki * g_R_Integral
      + g_R_Kd * derivative;
  //限幅输出
  if (output > MAX_RPS)
    output = MAX_RPS;
  PID_Set_Right_RPS(g_R_Dir, output);
}

//PID控制结束
void PID_Stop(void) {
  TimerIntDisable(PID_TIMER, TIMER_TIMA_TIMEOUT);
  PID_G = 0;
  Motor_G = 0;
}

//PID超时
void PID_Check_Timeout(void) {
  if (PID_G) {
    PID_Ticks++;
  } else {
    PID_Ticks = 0;
  }
  //500mS清空一次误差
  if (PID_Ticks >= PID_CLEAR_TIMEOUT) {
    PID_Ticks = 0;
    //清空误差
    g_L_Pre_Err = .0;
    g_L_Integral = .0;
    g_R_Pre_Err = .0;
    g_R_Integral = .0;
    //
    pre_diff_rps = 0;
    diff_Integral = 0;
    diff_deri = 0;
  }
}

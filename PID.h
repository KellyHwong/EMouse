/*
 * PID.h
 *
 *  Created on: 2015年6月6日
 *      Author: HuangKan
 */

#ifndef PID_H_
#define PID_H_

#include <stdint.h>

//公有宏定义
#define DEBUG_RPS 40.0
#define PID_DELAY_TIME 10//单位mS

void PID_Init(void);
void PID_Init_Timer(void);
void PIDTimerIntHandler(void);
void PID_Start(void);
void PID_Stop(void);
float PID_Left_RPS_To_PWM(float rps);
float PID_Right_RPS_To_PWM(float rps);
void PID_Set_Left_RPS(uint8_t dir, float rps);
void PID_Set_Right_RPS(uint8_t dir, float rps);
void PID_Move(uint8_t l_Dir, uint8_t r_Dir, float l_RPS,  float r_RPS);
void PID_Cali_Left_RPS(void);
void PID_Cali_Right_RPS(void);
void PID_Check_Timeout(void);

#endif /* PID_H_ */

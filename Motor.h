#ifndef _MOTOR_H
#define _MOTOR_H

#include <stdint.h>

//公有宏定义
#define DEBUG_MOTOR_PWM_WIDTH 40//
#define QEI_UPDATE_TIME PID_DELAY_TIME//Veciloty update time, in mS

#define LINES 512.0//电机线速度

#define QEI_LEFT_MOTOR_FORWARD (-1)//这些好像没什么用
#define QEI_LEFT_MOTOR_BACKWARD (1)
#define QEI_RIGHT_MOTOR_FORWARD (1)
#define QEI_RIGHT_MOTOR_BACKWARD (-1)

//PWM频率100kHz
#define MOTOR_PWM_FREQ (100000)

#define LeftMotor_Forward do{                                                               \
                                PWM0_ENABLE_R = (PWM0_ENABLE_R & 0xfffffffb) | 0x00000008;  \
                            }while(0)

#define LeftMotor_Backward do{                                                              \
                                PWM0_ENABLE_R = (PWM0_ENABLE_R & 0xfffffff7) | 0x00000004;  \
                            }while(0)

#define RightMotor_Forward do{                                  \
                                PWM1_ENABLE_R &= 0xfffffff7;    \
                                PWM0_ENABLE_R |= 0x00000040;    \
                            }while(0)

#define RightMotor_Backward do{                                 \
                                PWM0_ENABLE_R &= 0xffffffbf;    \
                                PWM1_ENABLE_R |= 0x00000008;    \
                            }while(0)

// Public function prototypes
void Motor_Init_PWM(void);
void Motor_Init(void);
void Motor_Init_QEI(void);
void Motor_Left_PWM_Width(uint32_t);
void Motor_Right_PWM_Width(uint32_t);
void Motor_Move(uint8_t, uint8_t, uint16_t, uint16_t);

void Motor_Start(void);
void Motor_Stop(void);
void Motor_Break(void);

#endif

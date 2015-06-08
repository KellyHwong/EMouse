// Private includes
#include "Motor.h"
#include "EMouse.h"
#include "PID.h"

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/qei.h"

uint16_t g_L_Cur_PWM = 1;//初始化PWM脉宽为1
uint16_t g_R_Cur_PWM = 1;
uint8_t g_L_Cur_Dir = 1;
uint8_t g_R_Cur_Dir = 1;
float g_L_Sample_RPS = 0;//采样到的转速，初始化为0
float g_R_Sample_RPS = 0;

#define BREAK_TIME 10//单位mS

void Motor_Init_PWM(void)
{
    uint32_t period = SYSTEM_CLOCK/MOTOR_PWM_FREQ;
    //设置PWM时钟
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
    //使能PWM外设端口
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    //使能PWM模块
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    //设置PWM输入逻辑
    GPIOPinConfigure(GPIO_PB4_M0PWM2);
    GPIOPinConfigure(GPIO_PB5_M0PWM3);
    GPIOPinConfigure(GPIO_PC4_M0PWM6);
    GPIOPinConfigure(GPIO_PA7_M1PWM3);

    GPIOPinTypePWM(GPIO_PORTB_BASE,GPIO_PIN_4 | GPIO_PIN_5);
    GPIOPinTypePWM(GPIO_PORTC_BASE,GPIO_PIN_4);
    GPIOPinTypePWM(GPIO_PORTA_BASE,GPIO_PIN_7);
        //Configure PWM Options
    //PWMnCTL PWMnGENA PWMnGENB registers
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    //TODO 这个部分存疑
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, period);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, period);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, period);
    //设置PWM脉宽，1是最小值，表示电机不转动
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 1);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, 1);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, 1);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 1);
    //使能PWM发生器输出
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);
    PWMGenEnable(PWM1_BASE, PWM_GEN_1);
}

void Motor_Init_QEI(void)
{
    //端口使能
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    //外设使能
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
    //PD7解锁
    // Unlock the Pin PD7 and Set the Commit Bit
    //
    GPIO_PORTD_LOCK_R = 0x4C4F434B;
    GPIO_PORTD_CR_R = 0x00000080;
    //端口功能配置
    GPIOPinConfigure(GPIO_PD6_PHA0);
    GPIOPinConfigure(GPIO_PD7_PHB0);

    GPIOPinConfigure(GPIO_PC5_PHA1);
    GPIOPinConfigure(GPIO_PC6_PHB1);
    //端口模式配置
    GPIOPinTypeQEI(GPIO_PORTD_BASE,GPIO_PIN_6);
    GPIOPinTypeQEI(GPIO_PORTD_BASE,GPIO_PIN_7);

    GPIOPinTypeQEI(GPIO_PORTC_BASE,GPIO_PIN_5);
    GPIOPinTypeQEI(GPIO_PORTC_BASE,GPIO_PIN_6);
    //QEI配置
    QEIConfigure(QEI0_BASE,QEI_CONFIG_CAPTURE_A_B|QEI_CONFIG_NO_RESET|
                QEI_CONFIG_QUADRATURE|QEI_CONFIG_NO_SWAP, 0x0FFFFFFFF);
    QEIConfigure(QEI1_BASE,QEI_CONFIG_CAPTURE_A_B|QEI_CONFIG_NO_RESET|
                QEI_CONFIG_QUADRATURE|QEI_CONFIG_NO_SWAP, 0x0FFFFFFFF);
    //QEI速度配置
    QEIVelocityConfigure(QEI0_BASE,QEI_VELDIV_1,QEI_UPDATE_TIME* \
            SysCtlClockGet()/1000);
    QEIVelocityConfigure(QEI1_BASE,QEI_VELDIV_1,QEI_UPDATE_TIME* \
            SysCtlClockGet()/1000);
    //QEI模块使能
    QEIEnable(QEI0_BASE);
    QEIEnable(QEI1_BASE);
    //qei速度使能
    QEIVelocityEnable(QEI0_BASE);
    QEIVelocityEnable(QEI1_BASE);
}

void Motor_Init(void)
{
    Motor_Init_PWM();
    Motor_Init_QEI();
}

//width的范围是0到499
void Motor_Left_PWM_Width(uint32_t width)
{
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, width);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, width);
}

void Motor_Right_PWM_Width(uint32_t width)
{
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, width);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, width);
}

//不改变当前PWM脉宽，以先前的脉宽启动电机
//注意，不能改变电机方向！
void Motor_Start(void)
{
    Motor_Left_PWM_Width(g_L_Cur_PWM);
    Motor_Right_PWM_Width(g_R_Cur_PWM);
}

//停止电机，不改变当前PWM脉宽
void Motor_Stop(void)
{
    Motor_Left_PWM_Width(1);
    Motor_Right_PWM_Width(1);
}

//刹车，即反向转动一定的时间
void Motor_Break(void)
{
    Motor_Move((1-g_L_Cur_Dir), (1-g_R_Cur_Dir), g_L_Cur_PWM, g_R_Cur_PWM);
    SysCtlDelay(SysCtlClockGet()/3000*BREAK_TIME);
    Motor_Stop();
}

//改变当前PWM脉宽，启动电机
//可以控制电机方向
void Motor_Move(uint8_t leftCtl, uint8_t rightCtl, \
    uint16_t leftWidth, uint16_t rightWidth)
{
    g_L_Cur_PWM = leftWidth;
    g_R_Cur_PWM = rightWidth;
    g_L_Cur_Dir = leftCtl;
    g_R_Cur_Dir = rightCtl;
    //1向前，0向后
    if (1 == leftCtl)
    {
        LeftMotor_Forward;
        Motor_Left_PWM_Width(leftWidth);
    }
    else if (0 == leftCtl)
    {
        LeftMotor_Backward;
        Motor_Left_PWM_Width(leftWidth);
    }

    if (1 == rightCtl)
    {
        RightMotor_Forward;
        Motor_Right_PWM_Width(rightWidth);
    }
    else if (0 == rightCtl)
    {
        RightMotor_Backward;
        Motor_Right_PWM_Width(rightWidth);
    }
}

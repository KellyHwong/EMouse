// Private includes
#include "Motor.h"
#include "EMouse.h"
#include "PID.h"
#include "Port.h"
#include "NEC.h"

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

uint8_t Motor_G = 0;
uint8_t Motor_Accel_Ticks = 0;
uint8_t Motor_Ticks = 0;
uint8_t Motor_Timeout_Count = 0;
uint8_t Motor_RPS_Sampled = 0;

#define BREAK_TIME 5//单位mS
#define MOTOR_TIMEOUT_TICKS 50//10mS each tick
#define ACCEL_TIMEOUT_TICKS 50//

//注册到了PID的定时器中(见PID.c)，电机超时处理，在运动到一半时采样速度
void Motor_Check_Timeout(void)
{
    float l_PWM_Div, r_PWM_Div;
    uint16_t tmpL, tmpR;
    //电机被打开
    if (Motor_G) {
        Motor_Ticks ++;
        Motor_Accel_Ticks ++;
    }
    //电机关闭
    else {
        Motor_Ticks = 0;
        Motor_Accel_Ticks = 0;
    }
    //匀加速
    if (Motor_G && (Motor_Accel_Ticks<=ACCEL_TIMEOUT_TICKS)) {
        //不匀加速
        l_PWM_Div = g_L_Cur_PWM;//1.0*g_L_Cur_PWM*Motor_Accel_Ticks/ACCEL_TIMEOUT_TICKS;
        r_PWM_Div = g_R_Cur_PWM;//1.0*g_R_Cur_PWM*Motor_Accel_Ticks/ACCEL_TIMEOUT_TICKS;
        //不改变全局变量g_L_Cur_PWM和g_R_Cur_PWM
        tmpL = g_L_Cur_PWM; tmpR = g_R_Cur_PWM;
        Motor_Move(g_L_Cur_Dir, g_R_Cur_Dir, \
                (uint16_t)l_PWM_Div,(uint16_t)r_PWM_Div);
        g_L_Cur_PWM = tmpL;
        g_R_Cur_PWM = tmpR;
    }
    //运动和采样速度
    if (Motor_Ticks >= MOTOR_TIMEOUT_TICKS) {
        Motor_Ticks = 0;
        //超时处理
        Motor_Timeout_Count ++;
        //2s时采样速度（每一次持续MOTOR_TIMEOUT_TICKS*10mS）
        if (2==Motor_Timeout_Count) {
            Motor_Sample_RPS();
        }
        //2.5s时停止电机
        if (2==Motor_Timeout_Count) {
            Motor_Timeout_Count = 0;
            //停止电机
            Motor_Break();
        }
    }
}

//不改变当前PWM脉宽，以先前的脉宽启动电机
//注意，不能改变电机方向！
//在这里写匀加速
inline void Motor_Start(uint16_t leftWidth, uint16_t rightWidth)
{
    Motor_SetPWM_Not_Move(leftWidth, rightWidth);
    Motor_G = 1;
    //向前行驶
    //Motor_Move(g_L_Cur_Dir,g_R_Cur_Dir,g_L_Cur_PWM,g_R_Cur_PWM);
}

inline void Motor_Sample_RPS(void)
{
    Motor_RPS_Sampled = 1;
    g_L_Sample_RPS = QEIVelocityGet(LEFT_QEI)/LINES/LINE_DIV* \
        1000.0/QEI_UPDATE_TIME*SMALL_WHEEL/BIG_WHEEL;
    g_R_Sample_RPS = QEIVelocityGet(RIGHT_QEI)/LINES/LINE_DIV* \
        1000.0/QEI_UPDATE_TIME*SMALL_WHEEL/BIG_WHEEL;
}

//暂停电机，不改变当前PWM脉宽
void Motor_Pause(void)
{
    Motor_Left_PWM_Width(1);
    Motor_Right_PWM_Width(1);
    Motor_G = 0;
}

//
void Motor_SetPWM_Not_Move(uint16_t leftWidth, uint16_t rightWidth)
{
    if (leftWidth >= MAX_PWM) leftWidth = MAX_PWM;
    if (rightWidth >= MAX_PWM) rightWidth = MAX_PWM;
    g_L_Cur_PWM = leftWidth;
    g_R_Cur_PWM = rightWidth;
}
//
void Motor_SetPWM_And_Move(uint16_t leftWidth, uint16_t rightWidth)
{
    if (leftWidth >= MAX_PWM) leftWidth = MAX_PWM;
    if (rightWidth >= MAX_PWM) rightWidth = MAX_PWM;
    g_L_Cur_PWM = leftWidth;
    g_R_Cur_PWM = rightWidth;
    Motor_Left_PWM_Width(leftWidth);
    Motor_Right_PWM_Width(rightWidth);
}

//
void Motor_SetDir(uint8_t leftCtl, uint8_t rightCtl)
{
    if (1==leftCtl)
    {
        g_L_Cur_Dir = 1;
        LeftMotor_Forward;
    }
    else if (0==leftCtl)
    {
        g_L_Cur_Dir = 0;
        LeftMotor_Backward;
    }

    if (1==rightCtl)
    {
        g_R_Cur_Dir = 1;
        RightMotor_Forward;
    }
    else if (0==rightCtl)
    {
        g_R_Cur_Dir = 0;
        RightMotor_Backward;
    }
}

//刹车，即反向转动一定的时间
void Motor_Break(void)
{
    Motor_Move((1-g_L_Cur_Dir), (1-g_R_Cur_Dir), MAX_PWM, MAX_PWM);
    SysCtlDelay(SysCtlClockGet()/3000*BREAK_TIME);
    //再次反向，即不改变方向
    Motor_Move((1-g_L_Cur_Dir), (1-g_R_Cur_Dir), 1,1);
    Motor_G = 0;
}

//电机反向
void Motor_Invert(void)
{
    Motor_SetDir((1-g_L_Cur_Dir), (1-g_R_Cur_Dir));
    Motor_SetPWM_And_Move(g_L_Cur_PWM, g_R_Cur_PWM);
}

//改变当前PWM脉宽，启动电机
//可以控制电机方向
void Motor_Move(uint8_t leftCtl, uint8_t rightCtl, \
    uint16_t leftWidth, uint16_t rightWidth)
{
    Motor_G = 1;
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
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | \
            PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | \
            PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | \
            PWM_GEN_MODE_NO_SYNC);
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
    //初始化QEI位移为0
    QEIPositionSet(QEI0_BASE,0);
    QEIPositionSet(QEI1_BASE,0);
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

/*
 *  nec.c
 *  Created on: 2015年5月4日
 *  Author: HuangKan
 *  Description: 红外NEC协议解码
 *  在这个文件中编写对命令的响应，实现快速的调试
 */
#include "NEC.h"
#include "Port.h"
#include "Seg.h"
#include "Motor.h"
#include "PID.h"
#include "utils/uartstdio.h"
#include "driverlib/qei.h"
#include "UART0.h"
//私有宏定义
#define KP_INC_DEC 0.1
#define KI_INC_DEC 1
#define KD_INC_DEC 0.01
#define PWM_INC_DEC 10
#define DEBUG_RPS 5
//Tech Get，需要初始化的变量要在.c源文件中定义
//NEC命令状态机状态
NECCommand necCommandMenuStatus = WaitCommand;
uint32_t freqToSet = 0;

//To Adapt, 需要控制的外部变量
//当前PWM脉宽，在Motor.c中定义
extern uint16_t g_L_Cur_PWM;
extern uint16_t g_R_Cur_PWM;
//
extern uint8_t g_L_Cur_Dir;
extern uint8_t g_R_Cur_Dir;
//
extern float g_L_Sample_RPS;  //采样到的转速
extern float g_R_Sample_RPS;
//PID算法Kp值
extern float g_L_Kd;
extern float g_R_Kd;

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

uint8_t NEC_LED_G = 0;
uint8_t NEC_Time_Ticks = 0;

void NEC_Init(void) {
  //LED灯初始化
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
  //熄灭LED灯
  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x00);
  NEC_LED_G = 0;
  InfraPortInit();
  NECTimerInit();
  //刷新Seg显示器
  Seg_Update();
}

//考虑重复命令
NECCommand gPreCommand = WaitCommand;
//在这里定义按键的功能，注意返回WaitCommand状态
//NECCommand的类型在头文件中定义
NECCommand NECCommandExecute(NECCommand necCommand) {
  //保存上一个非Repeat命令
  if (Repeat != necCommand)
    gPreCommand = necCommand;
  //首次执行命令
  switch (necCommand) {
    //执行上一个命令
    case (Repeat): {
      NECCommandExecute(gPreCommand);
      return WaitCommand;
    }
    case (Pause): {
//        Motor_Pause();
      PID_Stop();
      return WaitCommand;
    }
    case (Plus): {
//      Motor_SetDir(1, 1);
//      Motor_SetPWM_And_Move(g_L_Cur_PWM, g_R_Cur_PWM);
      //开启电机
      PID_Move(1, 1, DEBUG_RPS, DEBUG_RPS);
      return WaitCommand;
    }
    case (Minus): {
      //反向移动，不只是倒退的意思
//        Motor_Invert();
//      Motor_SetDir(0, 0);
//      Motor_SetPWM_And_Move(g_L_Cur_PWM, g_R_Cur_PWM);
      PID_Move(0, 0, DEBUG_RPS, DEBUG_RPS);
      return WaitCommand;
    }
    case (Num0): {
//      g_L_Cur_PWM += PWM_INC_DEC;
//      g_R_Cur_PWM += PWM_INC_DEC;
      g_L_Kd +=KD_INC_DEC;
      g_R_Kd +=KD_INC_DEC;
      return WaitCommand;
    }
    case (Num100Plus): {
//      g_L_Cur_PWM += PWM_INC_DEC;
      g_L_Kd +=KD_INC_DEC;
      return WaitCommand;
    }
    case (Num200Plus): {
//      g_R_Cur_PWM += PWM_INC_DEC;
      g_R_Kd +=KD_INC_DEC;
      return WaitCommand;
    }
    case (Num1): {
//      g_L_Cur_PWM -= PWM_INC_DEC;
//      g_R_Cur_PWM -= PWM_INC_DEC;
      g_L_Kd -=KD_INC_DEC;
      g_R_Kd -=KD_INC_DEC;
      return WaitCommand;
    }
    case (Num2): {
//      g_L_Cur_PWM -= PWM_INC_DEC;
      g_L_Kd -=KD_INC_DEC;
      return WaitCommand;
    }
    case (Num3): {
//      g_R_Cur_PWM -= PWM_INC_DEC;
      g_R_Kd -=KD_INC_DEC;
      return WaitCommand;
    }
    case (Channel): {
      Motor_Print_Data();
      return WaitCommand;
    }
    case (ChannelPlus): {
      UART0_Printf();
      return WaitCommand;
    }
    default:
      return WaitCommand;
  }
}

//检查超时
void NEC_LED_Check_Timeout(void) {
  if (NEC_LED_G)
    NEC_Time_Ticks++;
  else
    NEC_Time_Ticks = 0;
  //超时关闭LED
  if (NEC_Time_Ticks >= NEC_TIMEOUT_TICKS) {
    NEC_Time_Ticks = 0;
    NEC_LED_Off();
  }
}

void InfraPortInit() {
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  GPIOPinTypeGPIOInput(INFRA_RED_PORT, INFRA_RED_PIN);
  GPIOPadConfigSet(INFRA_RED_PORT, INFRA_RED_PIN, GPIO_STRENGTH_2MA,
  GPIO_PIN_TYPE_STD_WPU);
  //设置中断
  GPIOIntTypeSet(INFRA_RED_PORT, INFRA_RED_PIN, GPIO_FALLING_EDGE);
  GPIOIntRegister(INFRA_RED_PORT, InfraPortIntHandler);
  GPIOIntEnable(INFRA_RED_PORT, GPIO_INT_PIN_0);
}
void NECTimerInit() {
  uint32_t ui32Delta = 560;  //单位us
  uint32_t ui32SystemClock;
  SysCtlPeripheralEnable(NEC_TIMER_PERIPH);
  //设置为周期性定时器
  TimerConfigure(NEC_TIMER, TIMER_CFG_PERIODIC);
  //设置为系统时钟 50MHz
  TimerClockSourceSet(NEC_TIMER, TIMER_CLOCK_SYSTEM);
  ui32SystemClock = SysCtlClockGet();
  //50,000-1就是1ms, 50-1就是1us
  //560us 间隔 读PE0电平
  TimerLoadSet(NEC_TIMER, TIMER_A,
               ui32SystemClock / 1000 / 1000 * ui32Delta - 1);
  IntEnable(INT_TIMER0A);
  TimerIntEnable(NEC_TIMER, TIMER_TIMA_TIMEOUT);
  TimerIntRegister(NEC_TIMER, TIMER_A, NECTimerIntHandler);
  TimerIntClear(NEC_TIMER, TIMER_TIMA_TIMEOUT);
  //开始计时
  TimerEnable(NEC_TIMER, TIMER_A);

  NECReceiveReset(false);
}
void InfraPortIntHandler() {
  //TODO PE0中断被触发了两次，所以LED会亮两次，
  //第一次亮后会灭，第二次不会灭（传输有问题）
  //下降沿触发中断后
  GPIOIntClear(INFRA_RED_PORT, GPIO_INT_PIN_0);
  //点亮三色LED
  NEC_LED_On();
  NECReceiveReset(false);
  //除了允许读使能，其他全部复位，只有PE0中断能打开传输使能
  ui8TransmitEnable = 1;
  //关闭下降沿中断
  GPIOIntDisable(INFRA_RED_PORT, GPIO_INT_PIN_0);
  ui32PE0IntEnable = 0;
}
void NECTimerIntHandler() {
  uint8_t bit;
  NECCommand necCommand;
  TimerIntClear(NEC_TIMER, TIMER_TIMA_TIMEOUT);
  //读使能打开
  if (ui8TransmitEnable) {   //调试，读取560us间隔码
                             //读取PE0口的状态
    ui8RawBitCounter++;
    bit = GPIOPinRead(INFRA_RED_PORT, INFRA_RED_PIN);
    if (bit)
      bit = 1;
    else
      bit = 0;

    if (ui8RawBitCounter <= 64) {
      ui64InfraRawCode[0] <<= 1;
      ui64InfraRawCode[0] += bit;
    } else {
      ui64InfraRawCode[1] <<= 1;
      ui64InfraRawCode[1] += bit;
    }

    if (ui8RawBitCounter == MAX_RAW_BITS) {
      ui8TransmitEnable = 0;
      ui8RawBitCounter = 0;
    }
    //进入状态机，NEC解码
    machineState = StateMachine(machineState, bit);
    if ((TransmitSuccess == machineState) || (RepeatSignal == machineState)
        || (TransmitError == machineState)) {
      //关闭三色LED
      NEC_LED_Off();
      //传输结束，保留数据
      NECReceiveReset(true);
      if (TransmitSuccess == machineState) {
        //响应相应的命令
        necCommand = NECCommandExtract(ui32NECEncoded);
        //命令没有问题
        if ((necCommand != AddressError) && (necCommand != ValidateError)
            && (necCommand != UnknownCommand)) {
          if (WaitCommand == necCommandMenuStatus) {
            necCommandMenuStatus = NECCommandExecute(necCommand);
          } else
            necCommandMenuStatus = NECCommandMenu(necCommandMenuStatus,
                                                  necCommand);
        }
      } else if (RepeatSignal == machineState) {
        //命令菜单状态机，不用可以不管
        necCommandMenuStatus = NECCommandMenu(necCommandMenuStatus, Repeat);
        //考虑Repeat命令
        necCommandMenuStatus = NECCommandExecute(Repeat);
      }
    }
  }

  //100ms开启一次PE0下降沿中断
  ui32PE0IntEnable++;        //0.56*195=109.2;
  if (ui32PE0IntEnable > 190) {
    //再清楚中断标志试试看
    GPIOIntClear(INFRA_RED_PORT, GPIO_INT_PIN_0);
    GPIOIntEnable(INFRA_RED_PORT, GPIO_INT_PIN_0);
    ui32PE0IntEnable = 0;
  }
}

NECMachineStatue StateMachine(NECMachineStatue stateNow, uint8_t bit) {
  switch (stateNow) {
    case (LeadZeros):
      if (0 == bit)
        return LeadZeros;
      else if (1 == bit) {
        ui8LeadOnesCounter++;
        return LeadOnes;
      }
      break;
    case (LeadOnes):
      if (1 == bit) {
        ui8LeadOnesCounter++;
        return LeadOnes;
      } else if (0 == bit)
        if (ui8LeadOnesCounter > 5)
          return WaitOne;
        else if (ui8LeadOnesCounter <= 5)
          return RepeatSignal;
      break;
    case (WaitOne):
      if (0 == bit)
        return WaitOne;
      else if (1 == bit) {
        ui8CountOnesCounter++;
        return CountOnes;
      }
      break;
    case (CountOnes):
      if (1 == bit) {
        ui8CountOnesCounter++;
        //高电平超过3个，则判断接收状态
        if (ui8CountOnesCounter >= 4)
          if (32 == ui8EncodedBitCounter)
            return TransmitSuccess;
          else
            return TransmitError;
        return CountOnes;
      }
      //收到低电平确认信号
      else if (0 == bit) {
        //收到0，1~2个高电平为0
        if ((ui8CountOnesCounter >= 1) && (ui8CountOnesCounter <= 2))
          NECReceiveBit(0);
        //收到1，3~4个高电平为1
        else if ((ui8CountOnesCounter >= 3) && (ui8CountOnesCounter <= 4))
          NECReceiveBit(1);
        //等待下次接收
        return WaitOne;
      }
      break;
    default:
      return LeadZeros;
  }
  return LeadZeros;
}

void NECReceiveBit(uint8_t bit) {
  //高电平计数置零
  ui8CountOnesCounter = 0;
  ui8EncodedBitCounter++;
  ui32NECEncoded >>= 1;        //右移一位，低对低（先收到的），高对高
  ui32NECEncoded += (bit << 31);        //应该先右移，再相加
}

void NECReceiveReset(bool dataRestore) {
  if (dataRestore) {
    //传输结束
    ui8TransmitEnable = 0;
  } else {
    //停止传输
    ui8TransmitEnable = 0;
    //状态归零
    machineState = LeadZeros;
    //收到的原码和解码都归零
    ui64InfraRawCode[0] = 0;
    ui64InfraRawCode[1] = 0;
    ui32NECEncoded = 0;
    //原码计数器和解码计数器归零
    ui8RawBitCounter = 0;
    ui8EncodedBitCounter = 0;
    //引导信号高电平计数器归零
    ui8LeadOnesCounter = 0;
    //高电平计数器归零
    ui8CountOnesCounter = 0;
  }
}

NECCommand NECCommandExtract(uint32_t ui32NECEncoded) {
  uint8_t addr, n_addr, command, n_command;
  addr = ui32NECEncoded & 0x11;
  n_addr = (ui32NECEncoded >> 8) & 0xff;
  command = (ui32NECEncoded >> 16) & 0xff;
  n_command = (ui32NECEncoded >> 24) & 0xff;
  //用户码检验
  if (addr != NEC_ADDRESS)
    return AddressError;
  //反码检验
  if (((addr ^ n_addr) != 0xff) || ((command ^ n_command) != 0xff))
    return ValidateError;
  else
    switch (command) {
      //To Adapt，自己根据命令码修改命令表
      case (0x45):
        return ChannelMinus;
        break;        //CH-
      case (0x46):
        return Channel;
        break;        //CH
      case (0x47):
        return ChannelPlus;
        break;        //CH+
      case (0x44):
        return FastLeft;
        break;        //<<
      case (0x40):
        return FastRight;
        break;        //>>
      case (0x43):
        return Pause;
        break;        //pause
      case (0x07):
        return Minus;
        break;        //-
      case (0x15):
        return Plus;
        break;        //+
      case (0x09):
        return Equal;
        break;        //EQ
      case (0x16):
        return Num0;
        break;        //0
      case (0x19):
        return Num100Plus;
        break;        //100+
      case (0x0D):
        return Num200Plus;
        break;        //200+
      case (0x0C):
        return Num1;
        break;        //1
      case (0x18):
        return Num2;
        break;        //2
      case (0x5E):
        return Num3;
        break;        //3
      case (0x08):
        return Num4;
        break;        //4
      case (0x1C):
        return Num5;
        break;        //5
      case (0x5A):
        return Num6;
        break;        //6
      case (0x42):
        return Num7;
        break;        //7
      case (0x52):
        return Num8;
        break;        //8
      case (0x4A):
        return Num9;
        break;        //9
      default:
        return UnknownCommand;
    }
}

//输入参数，当前状态，命令
//输出，返回状态
NECCommand NECCommandMenu(NECCommand necCommandMenuStatus,
                          NECCommand necCommand) {
  switch (necCommandMenuStatus) {
    case (ChannelMinus):
    case (ChannelPlus):
    default:
      return WaitCommand;
  }
}

void NEC_LED_Off(void) {
  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x00);
  NEC_LED_G = 0;
}

void NEC_LED_On(void) {
  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0xFF);
  NEC_LED_G = 1;
}

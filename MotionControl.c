/**
* @Filename: MotionControl.c
* @Author: Kelly Hwong
* @Update:
* @Description: 动作控制
*/

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
#include "MotionControl.h"
#include "PID.h"
#include "InfSen.h"

//TODO 写红外感应控制状态机
#define BASE_RPS 14 // 基准速度
#define RPS_DIFF 2 // 速度偏差
#define MAX_NODES_NUM 80 // 最大节点数量

extern uint8_t InfSenValue;

typedef enum {
  GoStraight,
  TurnLeft, // 左转90度
  TurnRight, // 右转90度
  TurnBack, // 左转180度
  STOP
} MotionTypeDef; // 动作类型

typedef enum {
  START,
  END,
  LEFT,
  RIGHT,
  LEFT_FRONT,
  RIGHT_FRONT,
  LEFT_RIGHT,
  LEFT_RIGHT_FRONT,
  INVALID
} NodeTypeDef; // 迷宫的节点类型

NodeTypeDef nodes[MAX_NODES_NUM]; // 存历史节点
uint8_t nodes_ptr;
MotionTypeDef motions[MAX_NODES_NUM]; // 存历史动作

uint8_t pre_sen;
uint8_t now_sen;
MotionTypeDef now_motion;
uint8_t end_flag;
// 动作控制, 偏移校准等
// 输入红外感光值
void MotionControl(uint8_t ns){
  now_sen = ns;
  // 首先确定现在的动作
  // 有多种选择的时候,就一直向左,绝对没错
  if(~end_flag){
    if(nodes_ptr==0) { // 刚开始
      now_motion = GoStraight;
    }
    else if((now_sen & 1) == 1){ //向左
      nodes_ptr++;
      nodes[nodes_ptr]=LEFT;
      now_motion = TurnLeft;
    }
    else if((now_sen & 4) == 2){
      nodes_ptr++;
      nodes[nodes_ptr]=RIGHT;
      now_motion = TurnRight;
    }
    else if((now_sen & 17) == 17){
      nodes_ptr++;
      nodes[nodes_ptr]=LEFT_FRONT;
      now_motion = TurnLeft; // 去的时候优先向左
    }
    else if((now_sen & 18) == 18){
      nodes_ptr++;
      nodes[nodes_ptr]=RIGHT_FRONT;
      now_motion = GoStraight; // 去的时候优先向前
    }
    else if((now_sen & 3) == 3){
      nodes_ptr++;
      nodes[nodes_ptr]=LEFT_RIGHT;
      now_motion = TurnLeft; // 去的时候优先向左
    }
    else if((now_sen & 19) == 19){
      nodes_ptr++;
      nodes[nodes_ptr]=LEFT_RIGHT_FRONT;
      now_motion = TurnLeft; // 去的时候优先向左
    }
    else if((now_sen) == 0){ // 此路不通
      nodes_ptr++;
      nodes[nodes_ptr] = INVALID;
      now_motion = TurnBack;
    }
    else if((now_sen & 31)==31){ // 到达终点了
      nodes_ptr++;
      nodes[nodes_ptr] = END;
      now_motion = TurnBack;
      end_flag = 1;
    }
  }
  else {
    if((now_sen & 1) == 1){ //向左
      nodes_ptr++;
      nodes[nodes_ptr]=LEFT;
      now_motion = TurnLeft;
    }
    else if((now_sen & 4) == 2){
      nodes_ptr++;
      nodes[nodes_ptr]=RIGHT;
      now_motion = TurnRight;
    }
    else if((now_sen & 17) == 17){
      nodes_ptr++;
      nodes[nodes_ptr]=LEFT_FRONT;
      now_motion = TurnLeft; // 去的时候优先向左
    }
    else if((now_sen & 18) == 18){
      nodes_ptr++;
      nodes[nodes_ptr]=RIGHT_FRONT;
      now_motion = GoStraight; // 去的时候优先向前
    }
    else if((now_sen & 3) == 3){
      nodes_ptr++;
      nodes[nodes_ptr]=LEFT_RIGHT;
      now_motion = TurnLeft; // 去的时候优先向左
    }
    else if((now_sen & 19) == 19){
      nodes_ptr++;
      nodes[nodes_ptr]=LEFT_RIGHT_FRONT;
      now_motion = TurnLeft; // 去的时候优先向左
    }
    else if((now_sen) == 0){ // 此路不通
      nodes_ptr++;
      nodes[nodes_ptr] = INVALID;
      now_motion = TurnBack;
    }
    else if((now_sen & 31)==31){ // 回到终点了
      nodes_ptr++;
      nodes[nodes_ptr] = END;
      now_motion = STOP;
    }
  }

  switch(now_motion){
      case(STOP):{
        PID_Move(0,0,0,0);
        SysCtlDelay(SysCtlClockGet()/3);
        PID_Stop();
      }
      case(GoStraight):{
        if((now_sen & 4) == 4){
          PID_Move(1,1,BASE_RPS-RPS_DIFF,BASE_RPS+RPS_DIFF); //
        }
        if((now_sen & 8) == 8){
          PID_Move(1,1,BASE_RPS+RPS_DIFF,BASE_RPS-RPS_DIFF); //
        }
      break;
      }

    case(TurnLeft):{
      PID_Move(0,1,BASE_RPS,BASE_RPS);
      // 阻塞
      while((InfSenValue&16)!=16);

      PID_Move(1,1,BASE_RPS,BASE_RPS);
      break;
    }
    case(TurnRight):{
      PID_Move(1,0,BASE_RPS,BASE_RPS);
      // 阻塞
      while( (InfSenValue & 16) != 16) ;

      PID_Move(1,1,BASE_RPS,BASE_RPS);
      break;
    }
    case(TurnBack):{
      // TODO TurnBack 的方法肯定是有问题的
      PID_Move(1,0,BASE_RPS,BASE_RPS);
      // 阻塞
      while((InfSenValue & 16) != 16) ;
      PID_Move(1,1,BASE_RPS,BASE_RPS);
      break;
    }
  }
}

// 迷宫搜索算法,摸墙算法,左手定则(根据地图定的)
/*void WallFollowing(void) {
  ;
}*/

void MotionControl_Init(void){
  //int i;
  now_motion = GoStraight; // 一开始走直线
  nodes_ptr = 0;
  nodes[nodes_ptr] = START;

  end_flag = 0;
}

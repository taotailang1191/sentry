#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "gimbal_task.h"
#include "shoot_task.h"
#include "chassis_task.h"
#include "beep.h"
#include "stm32f4xx_it.h"
#include "tim.h"
#include "main.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "judge_task.h"
#include "bsp_io.h"

#define TIMEOUT 200	// ms
#define TIME_WAIT 60000 // ms
//#define TIME_WAIT 500 // ms   for debug

#define TONE_MAX_SIZE 21   // Must larger than TICK_MAX_SIZE and EX_MAX_SIZE
#define TICK_MAX_SIZE 9    // Size of Tick_Type_t
typedef enum {
	eTX2,
	ePitch,
	eYaw,
	eFeed,
	eFric1,
	eFric2,
	eCM1,
	eCM2,
	eJudge,
} Tick_Type_t;

#define EX_MAX_SIZE 2
typedef enum {
	eFricLocked,
	eEdgeError,
} EX_Type_t;

typedef enum {
	Red_Armor,
	Blue_Armor,
} Aim_Armor_Color_t;

typedef enum {										//云台电机模式选择
	GIMBAL_Depart_Mode,								//离线模式
	GIMBAL_Cruise_Mode,								//巡航模式
	GIMBAL_Auto_Mode,									//自动模式	
	GIMBAL_Test_Mode,									//测试模式
} GIMBAL_Mode_t;	

typedef enum {										//机架电机模式选择
	CHASSIS_Depart_Mode,							//离线模式
	CHASSIS_Auto_Mode,								//自动模式
} CHASSIS_Mode_t; 

typedef enum {										//云台自动模式的两种状态：在瞄准与无目标
	NoTarget,
	Aiming,
} AUTO_Gimbal_t;

typedef enum {										//机架电机自动模式的四种状态
	Cruise,														//巡航
	Twist,														//卷曲？
	Rand,															//无规则运动
	Follow,														//跟随模式
} AUTO_Chassis_t;									

typedef struct{										//哨兵的五个运动状态	
	uint8_t fire;											//开火状态
	GIMBAL_Mode_t gimbal;							//云台模式
	CHASSIS_Mode_t chassis;						//机架电机模式
	AUTO_Gimbal_t gimbal_state;				//云台电机状态：瞄准与无目标
	AUTO_Chassis_t chassis_state;			//机架电机状态：五种方式
} Sentry_Mode_t;


extern uint8_t exception[EX_MAX_SIZE];
extern uint8_t offline[TICK_MAX_SIZE];
extern uint32_t tick[TICK_MAX_SIZE];
extern uint32_t tick_wait;
extern uint32_t tick_FPS;
extern uint32_t tick_notarget;
extern Sentry_Mode_t Sentry_Mode;
extern unsigned char aim_armor_color;

void mode_switch(void);
void mode_init(void);
void Control_Task(void);

#endif

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

typedef enum {										//��̨���ģʽѡ��
	GIMBAL_Depart_Mode,								//����ģʽ
	GIMBAL_Cruise_Mode,								//Ѳ��ģʽ
	GIMBAL_Auto_Mode,									//�Զ�ģʽ	
	GIMBAL_Test_Mode,									//����ģʽ
} GIMBAL_Mode_t;	

typedef enum {										//���ܵ��ģʽѡ��
	CHASSIS_Depart_Mode,							//����ģʽ
	CHASSIS_Auto_Mode,								//�Զ�ģʽ
} CHASSIS_Mode_t; 

typedef enum {										//��̨�Զ�ģʽ������״̬������׼����Ŀ��
	NoTarget,
	Aiming,
} AUTO_Gimbal_t;

typedef enum {										//���ܵ���Զ�ģʽ������״̬
	Cruise,														//Ѳ��
	Twist,														//������
	Rand,															//�޹����˶�
	Follow,														//����ģʽ
} AUTO_Chassis_t;									

typedef struct{										//�ڱ�������˶�״̬	
	uint8_t fire;											//����״̬
	GIMBAL_Mode_t gimbal;							//��̨ģʽ
	CHASSIS_Mode_t chassis;						//���ܵ��ģʽ
	AUTO_Gimbal_t gimbal_state;				//��̨���״̬����׼����Ŀ��
	AUTO_Chassis_t chassis_state;			//���ܵ��״̬�����ַ�ʽ
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

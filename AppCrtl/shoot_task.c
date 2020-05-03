// 2018-07-15 by LSS
#include "shoot_task.h"

SHOOTEncoder_t SHOOTEncoder[3];				//三个摩擦轮的编码器，摩擦轮0为初始设定值
int threshold = 0;										//阈值
int fric_speed = 0;										//摩擦轮速度
int shoot_feed = 0;										//拨弹轮速度
int16_t shoot_fric[2] = {0,0};

void shoot_task(void) {
	int16_t fric_fdb0 = SHOOTEncoder[FRIC1_ID-FEED_ID].velocity;					//用不同摩擦轮的id减去拨弹轮的id，得到对应的结构体的速度作为摩擦轮的速度
	int16_t fric_fdb1 = SHOOTEncoder[FRIC2_ID-FEED_ID].velocity;					//两个炮管
	if((HAL_GetTick()-tick_controller) < TIMEOUT) {												//在系统误差时间内，tick control会在操控的时候获取一遍系统时间，gettick函数是获取系统运行时间
		turn_on_laser();																										//打开激光测距？laser激光																			
	} else if(fric_fdb0 || fric_fdb1) {																		//只要有一个射击速度不为0						
		shoot_stop();																												//射击停止，并关闭测距，直接退出函数															
		turn_off_laser();
		return;
	} else {																															//当两个枪管的射速均为0时
		exception[eFricLocked] = 0;																					//control test 173行，意思为发出特定频段的声音
		return;																															
			//friclocker表示摩擦轮锁死，当所需射速不为0而摩擦轮转速为0时，证明摩擦轮锁死，令其等于1，发出报警声音
			//当射速为0 时，或者射速不为0且摩擦轮转速不为0时，令其为0，表示正常工作，
	}
	
	if(fric_speed) {
		if(fric_fdb0 == 0 && fric_fdb1 == 0) exception[eFricLocked] = 1;		//当摩擦轮转速不为0，且两个枪管的射速均为0时，发出声音2
		else exception[eFricLocked] = 0;																		//
	}
	
	if(Sentry_Mode.fire) {																								//当哨兵射击状态为1时
		if(shootHeat > 240) ShootFeedSPID.ref = 1000;												//由热量来控制拨弹速度的快慢，
					//250应为2018的热量阈值，应根据实际情况进行修改，240为禁戒值，所设置的1000的pid需要我们根据设备来进行调节
		else ShootFeedSPID.ref = -4000;
		threshold = 250;
	} else {
		ShootFeedSPID.ref = 0;
		threshold = 999;
	}
	
	if(shootHeat > threshold) ShootFeedSPID.ref = 0;											//假如热量超过给定阈值，
	ShootFeedSPID.fdb = SHOOTEncoder[0].velocity;													//将摩擦轮0的速度设置为拨弹论速度pid的反馈值
	PID_Calc(&ShootFeedSPID);																							//由拨弹轮速度计算出射速并赋值给shoot_feed																					
	shoot_feed = ShootFeedSPID.output;
	turn_on_friction_wheel(fric_speed);																		//开启摩擦轮以指定速度？
	Set_Shoot_Current(&hcan1, shoot_feed, shoot_fric);										//将各个的速度回传
}

void shoot_stop() {
	uint8_t TxMessage[8];
	TxMessage[0] = 0;
	TxMessage[1] = 0;
	TxMessage[2] = 0;
	TxMessage[3] = 0;
	TxMessage[4] = 0;
	TxMessage[5] = 0;
	TxMessage[6] = 0;
	TxMessage[7] = 0;
	CAN_Send_Msg(&hcan1, TxMessage, 0x200 , 8);
}

void turn_on_friction_wheel(uint16_t spd) {
	ShootFric1PID.ref = -spd;
	ShootFric2PID.ref = spd;
	ShootFric1PID.fdb = SHOOTEncoder[1].velocity;
	ShootFric2PID.fdb = SHOOTEncoder[2].velocity;
	PID_Calc(&ShootFric1PID);
	PID_Calc(&ShootFric2PID);
	shoot_fric[0] = ShootFric1PID.output;
	shoot_fric[1] = ShootFric2PID.output;
}

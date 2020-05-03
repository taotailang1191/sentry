// 2018-07-15 by LSS
#include "shoot_task.h"

SHOOTEncoder_t SHOOTEncoder[3];				//����Ħ���ֵı�������Ħ����0Ϊ��ʼ�趨ֵ
int threshold = 0;										//��ֵ
int fric_speed = 0;										//Ħ�����ٶ�
int shoot_feed = 0;										//�������ٶ�
int16_t shoot_fric[2] = {0,0};

void shoot_task(void) {
	int16_t fric_fdb0 = SHOOTEncoder[FRIC1_ID-FEED_ID].velocity;					//�ò�ͬĦ���ֵ�id��ȥ�����ֵ�id���õ���Ӧ�Ľṹ����ٶ���ΪĦ���ֵ��ٶ�
	int16_t fric_fdb1 = SHOOTEncoder[FRIC2_ID-FEED_ID].velocity;					//�����ڹ�
	if((HAL_GetTick()-tick_controller) < TIMEOUT) {												//��ϵͳ���ʱ���ڣ�tick control���ڲٿص�ʱ���ȡһ��ϵͳʱ�䣬gettick�����ǻ�ȡϵͳ����ʱ��
		turn_on_laser();																										//�򿪼����ࣿlaser����																			
	} else if(fric_fdb0 || fric_fdb1) {																		//ֻҪ��һ������ٶȲ�Ϊ0						
		shoot_stop();																												//���ֹͣ�����رղ�ֱ࣬���˳�����															
		turn_off_laser();
		return;
	} else {																															//������ǹ�ܵ����پ�Ϊ0ʱ
		exception[eFricLocked] = 0;																					//control test 173�У���˼Ϊ�����ض�Ƶ�ε�����
		return;																															
			//friclocker��ʾĦ�������������������ٲ�Ϊ0��Ħ����ת��Ϊ0ʱ��֤��Ħ�����������������1��������������
			//������Ϊ0 ʱ���������ٲ�Ϊ0��Ħ����ת�ٲ�Ϊ0ʱ������Ϊ0����ʾ����������
	}
	
	if(fric_speed) {
		if(fric_fdb0 == 0 && fric_fdb1 == 0) exception[eFricLocked] = 1;		//��Ħ����ת�ٲ�Ϊ0��������ǹ�ܵ����پ�Ϊ0ʱ����������2
		else exception[eFricLocked] = 0;																		//
	}
	
	if(Sentry_Mode.fire) {																								//���ڱ����״̬Ϊ1ʱ
		if(shootHeat > 240) ShootFeedSPID.ref = 1000;												//�����������Ʋ����ٶȵĿ�����
					//250ӦΪ2018��������ֵ��Ӧ����ʵ����������޸ģ�240Ϊ����ֵ�������õ�1000��pid��Ҫ���Ǹ����豸�����е���
		else ShootFeedSPID.ref = -4000;
		threshold = 250;
	} else {
		ShootFeedSPID.ref = 0;
		threshold = 999;
	}
	
	if(shootHeat > threshold) ShootFeedSPID.ref = 0;											//������������������ֵ��
	ShootFeedSPID.fdb = SHOOTEncoder[0].velocity;													//��Ħ����0���ٶ�����Ϊ�������ٶ�pid�ķ���ֵ
	PID_Calc(&ShootFeedSPID);																							//�ɲ������ٶȼ�������ٲ���ֵ��shoot_feed																					
	shoot_feed = ShootFeedSPID.output;
	turn_on_friction_wheel(fric_speed);																		//����Ħ������ָ���ٶȣ�
	Set_Shoot_Current(&hcan1, shoot_feed, shoot_fric);										//���������ٶȻش�
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

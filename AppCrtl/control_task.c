// 2018-07-22 by LSS
#include "control_task.h"

static uint16_t time_tick_1ms = 0;
uint32_t tick_wait = 0;
uint32_t tick_FPS = 0;
uint32_t tick_notarget = 0;
extern uint32_t tick_rand;
uint8_t offline[TICK_MAX_SIZE] = {0};
uint32_t tick[TICK_MAX_SIZE] = {0};													//tick��ʲô��˼��һ��������9��Ϊ0�Ŀռ�
uint8_t exception[EX_MAX_SIZE] = {0};
Sentry_Mode_t Sentry_Mode;																	//�ڱ���ģʽ����
unsigned char aim_armor_color = Red_Armor;									//ʶ��װ��Ϊ��ɫװ��

const Sound_tone_e Tone_Warning[TONE_MAX_SIZE] = {
	Do1L,     ///*261.63Hz*/    3822us
	Re2L,     ///*293.66Hz*/    3405us
	Mi3L,     ///*329.63Hz*/    3034us
	Fa4L,     ///*349.23Hz*/    2863us
	So5L,     ///*392.00Hz*/    2551us
	La6L,     ///*440.00Hz*/    2272us
	Si7L,     ///*493.88Hz*/    2052us
	Do1M,     ///*523.25Hz*/    1911us
	Re2M,     ///*587.33Hz*/    1703us
	Mi3M,     ///*659.26Hz*/    1517us
	Fa4M,     ///*698.46Hz*/    1432us
	So5M,     ///*784.00Hz*/    1276us
	La6M,     ///*880.00Hz*/    1136us
	Si7M,     ///*987.77Hz*/    1012us
	Do1H,     ///*1046.50Hz*/   956us
	Re2H,     ///*1174.66Hz*/   851us
	Mi3H,     ///*1318.51Hz*/   758us
	Fa4H,     ///*1396.91Hz*/   716us
	So5H,     ///*1567.98Hz*/   638us
	La6H,     ///*1760.00Hz*/   568us
	Si7H,     ///*1975.53Hz*/   506us
};																										//���������ڷ��䲿�ֵ�������ʱ����в�����

void offline_warning(int index) {											//�ڱ����߾�������ֿ��ܣ������������ʲô��
	//���������������ʱ�������׼���Ƚ��бȽϣ����ж�״̬
	if(Startup_Success_music_index < Startup_Success_music_len) return;
	if(index < TICK_MAX_SIZE) Sing(Tone_Warning[index]);									
}

void offline_check(void) {														//�����������ǴӺ������Ͽ����������ж��ڱ��Ƿ����ߣ�ͨ��ϵͳʱ��ļ����Լ�������
	uint32_t Period = 400; // ms											//����
	uint32_t TimeOut = 200; // ms												
	uint8_t offline_num = 0;
	static uint8_t offline_count = 0;
	for(int i=0; i<TICK_MAX_SIZE; i++) {								//��ϵͳʱ������ж�
		if((HAL_GetTick()-tick[i]) < TimeOut) offline[i] = 0;
		else {
			switch(i) {
				case eTX2:
					if(rc.sw1 == RC_DN) offline[i] = 0;
					else offline[i] = 1;
					break;
				default: offline[i] = 1;
			}
		}
		offline_num += offline[i];
	}
	uint8_t offline_index = 0;
	for(int i=0; i<TICK_MAX_SIZE; i++) {
		if(offline[i]) {
			offline_index = i;															//ֻҪǰ���жϵ�offline����Ϊ1��ֵ��offline_index��Ϊ1
			break;
		}
	}
	
	if(Startup_Success_music_index < Startup_Success_music_len) return;		//��ʵ���ô���Ϊ�ж��������ַ��Ƿ�Ϊ���ڵ�����
	if((HAL_GetTick()-tick_controller) > TIMEOUT) { Sing(Silent); return; }		//�ӿɿ���ʱ�����жϣ��ڱ��Ƿ��ڿɿط�Χ�ڣ������ڣ��й̶����ַ�
	for(int i=0; i<EX_MAX_SIZE; i++) {
		if(exception[i]) {
			Sing(Tone_Warning[i]);
			return;
		}
	}
	
	if(offline_num == 0) {
		Sing(Silent);
	} else {
		if(time_tick_1ms < 0.5f*Period) {
			if(offline_count <= offline_index) offline_warning(offline_count);
			else Sing(Silent);
		}
		if(time_tick_1ms % Period == 0) {
			offline_count++;
			if(offline_count > offline_index+2) offline_count = 0;
		}
	}
}


void mode_init() {																											//�ڱ���ʼ״̬����
	pr.x = 0;
	pr.y = 200;
	pr.z = 1000;																													//λ����Ϣ
	Sentry_Mode.chassis_state = Cruise;																		//��̨�������Ѳ��״̬
	Sentry_Mode.gimbal_state = NoTarget;																	//��̨���������Ŀ��״̬
	last_HP = remain_HP;
}

void mode_switch() {																										
	switch(rc.sw1) {																										//��ң��������1������ģʽ��ѡ��
		case RC_UP:																													//������һ�ڸߵ�λ���Զ������ֻ���ÿ��ض���ȷ��ʶ��װ�װ���ɫ
			Sentry_Mode.gimbal = GIMBAL_Auto_Mode;
			Sentry_Mode.chassis = CHASSIS_Auto_Mode;														//�ڱ���̨��������������Զ�ģʽ�˶�
			fric_speed = FRIC_HIGH;																							//Ħ���ָ���
			if(rc.sw2 == RC_MI) aim_armor_color = Red_Armor;										//�����ض��⵽�м䵲λ��ʱ���ʾʶ���ɫװ�װ�
			else if(rc.sw2 == RC_DN) aim_armor_color = Blue_Armor;							//�����ض��⵽�͵�λ��ʱ���ʾʶ����ɫװ�װ�
			break;
		
		case RC_MI:																													//������һ���е�λ���е�����λ��Ϊ���Ե�������ʵ��ⶨ 
			Sentry_Mode.gimbal = GIMBAL_Test_Mode;															//��̨����Բ���ģʽ����
			Sentry_Mode.chassis = CHASSIS_Auto_Mode;														//���ܵ�����Զ�ģʽ����
			fric_speed = FRIC_HIGH;																							//Ħ���ָ���
			if(rc.sw2 == RC_UP) Sentry_Mode.fire = 1;													//���ض����ڸߵ�λ
			else Sentry_Mode.fire = 0;																					//�ڱ�����״̬Ϊ����
			tick_wait = 0;																											//��ʱ��Ϊ0
			if(last_rc.sw1 == RC_DN) set_edge();															//���翪��һ��һ״̬Ϊ�͵�λ�������ñ�Ե
			if(edge < 200) exception[eEdgeError] = 1;														//����״��ȷ���Ƿ񱨾�
			else exception[eEdgeError] = 0;
			break;
		
		case RC_DN:																													//������һ�ڵ͵�λ��
			Sentry_Mode.gimbal = GIMBAL_Depart_Mode;														//��̨�������ܵ������������ģʽ
			Sentry_Mode.chassis = CHASSIS_Depart_Mode;
			if(rc.sw2 == RC_UP) {																							//�����ض����ڸߵ�λ
				Sentry_Mode.fire = 1;																							//�ڱ��Ը��ٿ���
				fric_speed = FRIC_HIGH;
			} else {
				Sentry_Mode.fire = 0;																							//��Ȼ�򿪻�״̬Ϊ0��Ħ���ֿ�ת
				fric_speed = FRIC_IDLING;
			}
			if(rc.sw2 == RC_MI) aim_armor_color = Red_Armor;										//�ɿ��ض���ȷ��Ŀ���װ�װ���ɫ
			else if(rc.sw2 == RC_DN) aim_armor_color = Blue_Armor;
			mode_init();
			tick_wait = 0;																						//�ڱ����������ؿ��ƣ�����һ���Ƶ��̵������̨�����ģʽ�����ض�����ʶ���Ŀ����ɫ�Լ��Ƿ񿪻�
			hit = 0;
			break;
		
		default:;
	}
	last_rc = rc;
}

void Control_Task(void) {
	time_tick_1ms++;
	tick_rand++;
	tick_FPS++;
	if(tick_wait < TIME_WAIT) tick_wait++;
	
	offline_check();
	mode_switch();
	imu_task();																										//�������ǻ�ȡ�ڱ�������̬����Ѳ����
	gimbal_task();																								//��̨����Ŀ��Ʋ��֣���ز����ĵ�����Ҫ��������޸�
	if(time_tick_1ms % 10 == 0) chassis_task();										//ÿʮ��ѭ���Ի��ܵ�������״̬����һ������
	if(time_tick_1ms % 10 == 0) shoot_task();
	
	if(time_tick_1ms % 80 == 0) {																	//ÿ80��ѭ�������ڱ���״̬����һ���ж�
    if(Startup_Success_music_index < Startup_Success_music_len)
      Sing_Startup_music(Startup_Success_music_index);
    Startup_Success_music_index++;
  }
	if(time_tick_1ms % 400 == 0) {																//ÿ400��ѭ����ʶ��װ�װ����ɫ����һ���жϣ����ɴ�����ͬ�ĵ�
		can_send_TX2(aim_armor_color);
		if(aim_armor_color == Red_Armor) {
			LED_Red_On();
			LED_Green_Off();
		} else {
			LED_Green_On();
			LED_Red_Off();
		}
		time_tick_1ms = 0;
	}
}

// 2018-07-22 by LSS
#include "control_task.h"

static uint16_t time_tick_1ms = 0;
uint32_t tick_wait = 0;
uint32_t tick_FPS = 0;
uint32_t tick_notarget = 0;
extern uint32_t tick_rand;
uint8_t offline[TICK_MAX_SIZE] = {0};
uint32_t tick[TICK_MAX_SIZE] = {0};													//tick是什么意思，一共声明了9个为0的空间
uint8_t exception[EX_MAX_SIZE] = {0};
Sentry_Mode_t Sentry_Mode;																	//哨兵的模式声明
unsigned char aim_armor_color = Red_Armor;									//识别装甲为红色装甲

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
};																										//报警音，在发射部分当卡弹的时候会有部分音

void offline_warning(int index) {											//哨兵离线警告的两种可能，但是输入的是什么？
	//利用音乐所代表的时间来与标准长度进行比较，来判断状态
	if(Startup_Success_music_index < Startup_Success_music_len) return;
	if(index < TICK_MAX_SIZE) Sing(Tone_Warning[index]);									
}

void offline_check(void) {														//看不懂，但是从函数名上看来是用于判断哨兵是否离线，通过系统时间的计算以及，，，
	uint32_t Period = 400; // ms											//周期
	uint32_t TimeOut = 200; // ms												
	uint8_t offline_num = 0;
	static uint8_t offline_count = 0;
	for(int i=0; i<TICK_MAX_SIZE; i++) {								//从系统时间层面判断
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
			offline_index = i;															//只要前面判断的offline里有为1的值，offline_index即为1
			break;
		}
	}
	
	if(Startup_Success_music_index < Startup_Success_music_len) return;		//无实际用处，为判断索引音乐符是否为库内的音乐
	if((HAL_GetTick()-tick_controller) > TIMEOUT) { Sing(Silent); return; }		//从可控制时间来判断，哨兵是否在可控范围内，若不在，有固定音乐放
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


void mode_init() {																											//哨兵初始状态定义
	pr.x = 0;
	pr.y = 200;
	pr.z = 1000;																													//位置信息
	Sentry_Mode.chassis_state = Cruise;																		//云台电机处于巡航状态
	Sentry_Mode.gimbal_state = NoTarget;																	//云台电机处于无目标状态
	last_HP = remain_HP;
}

void mode_switch() {																										
	switch(rc.sw1) {																										//由遥控器开关1来控制模式的选择
		case RC_UP:																													//若开关一在高档位，自动打击，只需用开关二来确定识别装甲板颜色
			Sentry_Mode.gimbal = GIMBAL_Auto_Mode;
			Sentry_Mode.chassis = CHASSIS_Auto_Mode;														//哨兵云台电机与底座电机以自动模式运动
			fric_speed = FRIC_HIGH;																							//摩擦轮高速
			if(rc.sw2 == RC_MI) aim_armor_color = Red_Armor;										//当开关二扳到中间挡位的时候表示识别红色装甲板
			else if(rc.sw2 == RC_DN) aim_armor_color = Blue_Armor;							//当开关二扳到低档位的时候表示识别蓝色装甲板
			break;
		
		case RC_MI:																													//若开关一在中档位，中低两档位均为测试档，具体实验测定 
			Sentry_Mode.gimbal = GIMBAL_Test_Mode;															//云台电机以测试模式启动
			Sentry_Mode.chassis = CHASSIS_Auto_Mode;														//机架电机以自动模式启动
			fric_speed = FRIC_HIGH;																							//摩擦轮高速
			if(rc.sw2 == RC_UP) Sentry_Mode.fire = 1;													//开关二处于高档位
			else Sentry_Mode.fire = 0;																					//哨兵开火状态为开火
			tick_wait = 0;																											//延时？为0
			if(last_rc.sw1 == RC_DN) set_edge();															//假如开关一上一状态为低档位，则设置边缘
			if(edge < 200) exception[eEdgeError] = 1;														//两种状况确定是否报警
			else exception[eEdgeError] = 0;
			break;
		
		case RC_DN:																													//若开关一在低档位，
			Sentry_Mode.gimbal = GIMBAL_Depart_Mode;														//云台电机与机架电机均处于离线模式
			Sentry_Mode.chassis = CHASSIS_Depart_Mode;
			if(rc.sw2 == RC_UP) {																							//若开关二处于高档位
				Sentry_Mode.fire = 1;																							//哨兵以高速开火
				fric_speed = FRIC_HIGH;
			} else {
				Sentry_Mode.fire = 0;																							//不然则开火状态为0，摩擦轮空转
				fric_speed = FRIC_IDLING;
			}
			if(rc.sw2 == RC_MI) aim_armor_color = Red_Armor;										//由开关二来确定目标的装甲板颜色
			else if(rc.sw2 == RC_DN) aim_armor_color = Blue_Armor;
			mode_init();
			tick_wait = 0;																						//哨兵由两个开关控制，开关一控制底盘电机与云台电机的模式，开关二控制识别的目标颜色以及是否开火
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
	imu_task();																										//由陀螺仪获取哨兵自身姿态角与巡航角
	gimbal_task();																								//云台电机的控制部分，相关参数的调整需要对其进行修改
	if(time_tick_1ms % 10 == 0) chassis_task();										//每十次循环对机架电机与射击状态进行一次修正
	if(time_tick_1ms % 10 == 0) shoot_task();
	
	if(time_tick_1ms % 80 == 0) {																	//每80次循环对于哨兵的状态进行一次判断
    if(Startup_Success_music_index < Startup_Success_music_len)
      Sing_Startup_music(Startup_Success_music_index);
    Startup_Success_music_index++;
  }
	if(time_tick_1ms % 400 == 0) {																//每400次循环对识别装甲板的颜色进行一次判断，并由此亮不同的灯
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

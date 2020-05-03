#include "usart.h"
#include "bsp_uart.h"
#include "string.h"
#include "control_task.h"

#define U8	unsigned char
#define U16	unsigned short int
#define U32	unsigned int

//Part: Student_interface
extern uint8_t USART_STU_BUF[200];     //ѧ���ӿڽ��ջ���,���200���ֽ�.
extern uint16_t USART_STU_STA;       //ѧ���ӿڽ���״̬���		
extern uint8_t USART_FRAME[46];
extern uint8_t USART_LEN;
extern uint8_t recLength;

extern uint8_t   robotlevel;  //�����˵ȼ�
extern uint16_t  remain_HP;
extern uint8_t   hurtType;    //װ���˺����յ�������

extern uint8_t   shoottype, shootFreq;   //1->17, 2->42
extern uint16_t  shootHeat;   //ǹ������
extern float     shootSpeed;

extern float power_v, power, power_buffer;

void HAL_UART6_IRQHandler(void);
void HAL_judge_UART_IRQHandler(void);

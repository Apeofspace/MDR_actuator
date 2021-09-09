#include "main.h"



void CPU_init(){
	MDR_RST_CLK->HS_CONTROL=RST_CLK_HSE_ON; //���. HSE
	while((MDR_RST_CLK->CLOCK_STATUS&RST_CLK_CLOCK_STATUS_HSE_RDY )!=RST_CLK_CLOCK_STATUS_HSE_RDY );		//��� HSE
	MDR_RST_CLK->CPU_CLOCK=0x00000102; //�������������� ������ ������������� c2 �� CPU_C1
	MDR_RST_CLK->PLL_CONTROL = ((1 << 2) | (9 << 8));//���. PLL  | ����. ��������� = 9+1
	while ((MDR_RST_CLK->CLOCK_STATUS & RST_CLK_CLOCK_STATUS_PLL_CPU_RDY) != RST_CLK_CLOCK_STATUS_PLL_CPU_RDY);// ��� PLL_CPU
	MDR_RST_CLK->CPU_CLOCK=0x00000106; // ������ ������������� �2 �� PLLCPU
	
//	MDR_RST_CLK->CPU_CLOCK = ((2 << 0)//�������� ��� CPU_C1
//										 | (1 << 2)//�������� ��� CPU_C2
//										 | (0 << 4)//������������ ��� CPU_C3
//										 | (1 << 8));//�������� ��� HCLK
//											/* �� �������� �� ������� HSE ���������� */
}


void GPIO_init(){
	PORT_InitTypeDef GPIO_user_init;
	
	PORT_DeInit(MDR_PORTA);
	PORT_DeInit(MDR_PORTB);
	PORT_DeInit(MDR_PORTC);
	PORT_DeInit(MDR_PORTD);
	PORT_DeInit(MDR_PORTE);
	PORT_DeInit(MDR_PORTF);		
	
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTC, ENABLE);		
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTA, ENABLE);	
	
	GPIO_user_init.PORT_Pin       = PORT_Pin_0;
	GPIO_user_init.PORT_OE        = PORT_OE_OUT;
	GPIO_user_init.PORT_PULL_UP   = PORT_PULL_UP_OFF;
	GPIO_user_init.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
	GPIO_user_init.PORT_PD_SHM    = PORT_PD_SHM_OFF;
	GPIO_user_init.PORT_PD        = PORT_PD_DRIVER;
	GPIO_user_init.PORT_GFEN      = PORT_GFEN_OFF;
	GPIO_user_init.PORT_FUNC      = PORT_FUNC_PORT;
	GPIO_user_init.PORT_SPEED     = PORT_SPEED_MAXFAST;
	GPIO_user_init.PORT_MODE      = PORT_MODE_DIGITAL;	
	PORT_Init(MDR_PORTC, &GPIO_user_init);
	
	GPIO_user_init.PORT_Pin       = PORT_Pin_1;
	GPIO_user_init.PORT_OE        = PORT_OE_OUT;
	GPIO_user_init.PORT_PULL_UP   = PORT_PULL_UP_OFF;
	GPIO_user_init.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
	GPIO_user_init.PORT_PD_SHM    = PORT_PD_SHM_OFF;
	GPIO_user_init.PORT_PD        = PORT_PD_DRIVER;
	GPIO_user_init.PORT_GFEN      = PORT_GFEN_OFF;
	GPIO_user_init.PORT_FUNC      = PORT_FUNC_ALTER;
	GPIO_user_init.PORT_SPEED     = PORT_SPEED_MAXFAST;
	GPIO_user_init.PORT_MODE      = PORT_MODE_DIGITAL;	
	PORT_Init(MDR_PORTA, &GPIO_user_init);
	
	GPIO_user_init.PORT_Pin       = PORT_Pin_3;
	GPIO_user_init.PORT_OE        = PORT_OE_OUT;
	GPIO_user_init.PORT_PULL_UP   = PORT_PULL_UP_OFF;
	GPIO_user_init.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
	GPIO_user_init.PORT_PD_SHM    = PORT_PD_SHM_OFF;
	GPIO_user_init.PORT_PD        = PORT_PD_DRIVER;
	GPIO_user_init.PORT_GFEN      = PORT_GFEN_OFF;
	GPIO_user_init.PORT_FUNC      = PORT_FUNC_ALTER;
	GPIO_user_init.PORT_SPEED     = PORT_SPEED_MAXFAST;
	GPIO_user_init.PORT_MODE      = PORT_MODE_DIGITAL;	
	PORT_Init(MDR_PORTA, &GPIO_user_init);
}

void TIMER_init(){
	/*���������*/
	MDR_TIMER1->CH1_CNTRL = 0x00000000;
	MDR_TIMER1->CH2_CNTRL = 0x00000000;
	MDR_TIMER1->CH3_CNTRL = 0x00000000;
	MDR_TIMER1->CH4_CNTRL = 0x00000000;
	MDR_TIMER1->CH1_CNTRL1 = 0x00000000;
	MDR_TIMER1->CH2_CNTRL1 = 0x00000000;
	MDR_TIMER1->CH3_CNTRL1 = 0x00000000;
	MDR_TIMER1->CH4_CNTRL1 = 0x00000000;	
	MDR_TIMER1->CH1_CNTRL2 = 0x00000000;
	MDR_TIMER1->CH2_CNTRL2 = 0x00000000;
	MDR_TIMER1->CH3_CNTRL2 = 0x00000000;
	MDR_TIMER1->CH4_CNTRL2 = 0x00000000;	
	MDR_TIMER1->STATUS = 0x00000000;
	MDR_TIMER1->CNTRL = 0x00000000; // ����� ������������� �������
	MDR_TIMER1->CNT = 0x00000000; // ��������� �������� ��������	
	
	/*������������ ���������*/
	MDR_RST_CLK->PER_CLOCK |= (1 << 14 );
	MDR_RST_CLK->TIM_CLOCK = 0x01000000; //(1 << 24);
	
	/*��������� �������*/
	MDR_TIMER1->PSG = 1; // ������������ �������
	MDR_TIMER1->ARR = 99; // ��������� ����� (16 ���)
	
	/*��������� �������*/
	MDR_TIMER1->CH1_CNTRL = (6 << 9);
  MDR_TIMER1->CH1_CNTRL1 = (1 << 0 )|(3 << 2); //����� ������ ���., 2 �� ����� REF, 3 �� ����� DTG
	MDR_TIMER1->CH1_DTG = (1 << 0)|(0 << 4)|(15 << 8); //������������, ������� �� TIM_CLK, �������� ��������
	MDR_TIMER1->CCR1 = 65;
	
	MDR_TIMER1->CH2_CNTRL = (6 << 9);
	MDR_TIMER1->CH2_CNTRL1 = (1 << 0 )|(3 << 2); //����� ������ ���., 2 �� ����� REF, 3 �� ����� DTG
	MDR_TIMER1->CCR2 = 65;
	
	/*���������� ������*/
	MDR_TIMER1->IE = 0x00001102;		//���������� �� cnt=arr
	MDR_TIMER1->CNTRL = 0x00000001; // ���� ����� �� TIM_CLK, � �������� ������.	
	NVIC_EnableIRQ(Timer1_IRQn); //��������� ���������� �������
}

int main(){
	CPU_init();
	GPIO_init();
	TIMER_init();
	
	while (1){}
}

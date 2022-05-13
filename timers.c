#include "main.h"

void deinit_TIMER(MDR_TIMER_TypeDef *Timer){
	/*���������*/
	Timer->CH1_CNTRL = 0x00000000;
	Timer->CH2_CNTRL = 0x00000000;
	Timer->CH3_CNTRL = 0x00000000;
	Timer->CH4_CNTRL = 0x00000000;
	Timer->CH1_CNTRL1 = 0x00000000;
	Timer->CH2_CNTRL1 = 0x00000000;
	Timer->CH3_CNTRL1 = 0x00000000;
	Timer->CH4_CNTRL1 = 0x00000000;	
	Timer->CH1_CNTRL2 = 0x00000000;
	Timer->CH2_CNTRL2 = 0x00000000;
	Timer->CH3_CNTRL2 = 0x00000000;
	Timer->CH4_CNTRL2 = 0x00000000;	
	Timer->STATUS = 0x00000000;
	Timer->CNTRL = 0x00000000; // ����� ������������� �������
	Timer->CNT = 0x00000000; // ��������� �������� ��������	
}
//-----------------------------------------------------------------------
void init_SysTick(){
	SysTick->LOAD = 0x00FFFFFF; 
	SysTick->CTRL = 0;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk |SysTick_CTRL_TICKINT_Msk; //HCLK and Enable and interrupt enable
	NVIC_EnableIRQ(SysTick_IRQn);
}
//-----------------------------------------------------------------------
void init_TIMER1(){
	/*������ ���*/
	/*���������*/
	deinit_TIMER(MDR_TIMER1);
	
	/*������������ ��������*/
	MDR_RST_CLK->PER_CLOCK |= RST_CLK_PCLK_TIMER1;
	MDR_RST_CLK->TIM_CLOCK |= RST_CLK_TIM_CLOCK_TIM1_CLK_EN;
	
	/*��������� �������*/
	MDR_TIMER1->PSG = T1PSG; // ������������ �������
	MDR_TIMER1->ARR = T1ARR; // ��������� ����� (16 ���)
	MDR_TIMER1->CNTRL = 0x00000042; //����������� ���� ����� � ����
	
	/*��������� �������*/
	MDR_TIMER1->CH1_CNTRL = (6 << TIMER_CH_CNTRL_OCCM_Pos); //������ ������� 6: 1 ���� CNT<CCR
  MDR_TIMER1->CH1_CNTRL1 = (1 << TIMER_CH_CNTRL1_SELOE_Pos )|(3 << TIMER_CH_CNTRL1_SELO_Pos)| //����� ������ ���., 2 �� ����� REF, 3 �� ����� DTG
													 (1 << TIMER_CH_CNTRL1_NSELOE_Pos )|(3 << TIMER_CH_CNTRL1_NSELO_Pos);
	MDR_TIMER1->CH1_DTG = (0 << TIMER_CH_DTGX_Pos)|(0 << TIMER_CH_DTG_EDTS_Pos)|(DEADTIMECONST << TIMER_CH_DTG_Pos); //������������, ������� �� TIM_CLK, �������� ��������
	MDR_TIMER1->CCR1 = T1ARR;	
	
	MDR_TIMER1->CH2_CNTRL = (6 << TIMER_CH_CNTRL_OCCM_Pos);
  MDR_TIMER1->CH2_CNTRL1 = (1 << TIMER_CH_CNTRL1_SELOE_Pos )|(3 << TIMER_CH_CNTRL1_SELO_Pos)| //����� ������ ���., 2 �� ����� REF, 3 �� ����� DTG
													 (1 << TIMER_CH_CNTRL1_NSELOE_Pos )|(3 << TIMER_CH_CNTRL1_NSELO_Pos);
	MDR_TIMER1->CH2_DTG = (0 << TIMER_CH_DTGX_Pos)|(0 << TIMER_CH_DTG_EDTS_Pos)|(DEADTIMECONST << TIMER_CH_DTG_Pos); //������������, ������� �� TIM_CLK, �������� ��������
	MDR_TIMER1->CCR2 = T1ARR;	
	
	/*���������� ������*/
	MDR_TIMER1->CNTRL |= TIMER_CNTRL_CNT_EN; //�������� ������.	
}
//-----------------------------------------------------------------------
void init_TIMER2(){
	/*��������� ����. ����������, �������� ���������� */
	/*������������ ��������*/
	MDR_RST_CLK->PER_CLOCK |= RST_CLK_PCLK_TIMER2;
	MDR_RST_CLK->TIM_CLOCK |= RST_CLK_TIM_CLOCK_TIM2_CLK_EN;

		/*���������*/
	deinit_TIMER(MDR_TIMER2);
	
	/*��������� �������*/
	MDR_TIMER2->PSG = T2PSG; // ������������ �������
	MDR_TIMER2->ARR = T2ARR; // ��������� ����� (16 ���)
	MDR_TIMER2->CNTRL = 0x00000002; //�����������
	
	/*���������� ������*/
	MDR_TIMER2->IE = 0x00001102;		//���������� �� cnt=arr
	NVIC_EnableIRQ(Timer2_IRQn); //��������� ���������� �������	
	MDR_TIMER2->CNTRL |= TIMER_CNTRL_CNT_EN; //  �������� ������.	
}

//-----------------------------------------------------------------------
void Timer2_IRQHandler(void){
	MDR_TIMER2->STATUS &= ~(1UL << 1UL);
	control_loop();
}
//-----------------------------------------------------------------------
void SysTick_Handler(void){
	timestamp_overflow_counter ++;
}


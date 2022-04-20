#include "main.h"

void deinit_TIMER(MDR_TIMER_TypeDef *Timer){
	/*Обнуление*/
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
	Timer->CNTRL = 0x00000000; // режим инициализации таймера
	Timer->CNT = 0x00000000; // Начальное значение счетчика	
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
	/*Управление ШИМ*/
	/*Обнуление*/
	deinit_TIMER(MDR_TIMER1);
	
	/*Тактирование таймеров*/
	MDR_RST_CLK->PER_CLOCK |= RST_CLK_PCLK_TIMER1;
	MDR_RST_CLK->TIM_CLOCK |= RST_CLK_TIM_CLOCK_TIM1_CLK_EN;
	
	/*Настройки таймера*/
	MDR_TIMER1->PSG = T1PSG; // Предделитель частоты
	MDR_TIMER1->ARR = T1ARR; // Основание счета (16 бит)
	MDR_TIMER1->CNTRL = 0x00000042; //буферизация счет вверх и вниз
	
	/*Настройка каналов*/
	MDR_TIMER1->CH1_CNTRL = (6 << TIMER_CH_CNTRL_OCCM_Pos); //формат сигнала 6: 1 если CNT<CCR
  MDR_TIMER1->CH1_CNTRL1 = (1 << TIMER_CH_CNTRL1_SELOE_Pos )|(3 << TIMER_CH_CNTRL1_SELO_Pos)| //выход всегда вкл., 2 на выход REF, 3 на выход DTG
													 (1 << TIMER_CH_CNTRL1_NSELOE_Pos )|(3 << TIMER_CH_CNTRL1_NSELO_Pos);
	MDR_TIMER1->CH1_DTG = (0 << TIMER_CH_DTGX_Pos)|(0 << TIMER_CH_DTG_EDTS_Pos)|(DEADTIMECONST << TIMER_CH_DTG_Pos); //предделитель, частота от TIM_CLK, основной делитель
	MDR_TIMER1->CCR1 = T1ARR;	
	
	MDR_TIMER1->CH2_CNTRL = (6 << TIMER_CH_CNTRL_OCCM_Pos);
  MDR_TIMER1->CH2_CNTRL1 = (1 << TIMER_CH_CNTRL1_SELOE_Pos )|(3 << TIMER_CH_CNTRL1_SELO_Pos)| //выход всегда вкл., 2 на выход REF, 3 на выход DTG
													 (1 << TIMER_CH_CNTRL1_NSELOE_Pos )|(3 << TIMER_CH_CNTRL1_NSELO_Pos);
	MDR_TIMER1->CH2_DTG = (0 << TIMER_CH_DTGX_Pos)|(0 << TIMER_CH_DTG_EDTS_Pos)|(DEADTIMECONST << TIMER_CH_DTG_Pos); //предделитель, частота от TIM_CLK, основной делитель
	MDR_TIMER1->CCR2 = T1ARR;	
	
	/*Разрешения работы*/
	MDR_TIMER1->CNTRL |= TIMER_CNTRL_CNT_EN; //включить таймер.	
}
//-----------------------------------------------------------------------
void init_TIMER2(){
	/*Обнуление*/
	deinit_TIMER(MDR_TIMER2);
	
	/*Тактирование таймеров*/
	MDR_RST_CLK->PER_CLOCK |= RST_CLK_PCLK_TIMER2;
	MDR_RST_CLK->TIM_CLOCK |= RST_CLK_TIM_CLOCK_TIM2_CLK_EN;
//	
//	/*Настройки таймера*/
//	MDR_TIMER2->PSG = T2PSG; // Предделитель частоты
//	MDR_TIMER2->ARR = T2ARR; // Основание счета (16 бит)
//	MDR_TIMER1->CNTRL = TIMER_CNTRL_ARRB_EN;//включение буферизации
//	
//	/*Разрешения работы*/
//	MDR_TIMER2->IE = 0x00001102;		//прерывание по cnt=arr  //********************TODO: ТУТ ЯВНО ЛИШНЕЕ? ДОСТАТОЧНО ПРОСТО 0x02 ?? прочекать когда подключу полено
//																														//*******************	MDR_TIMER1->IE = TIMER_IE_CNT_ARR_EVENT_IE; //прерывание по cnt=arr
//	NVIC_EnableIRQ(Timer2_IRQn); //разрешить прерывания таймера	
//	MDR_TIMER2->CNTRL |= TIMER_CNTRL_CNT_EN; //  включить таймер.	
	
		/*Обнуление*/
	deinit_TIMER(MDR_TIMER2);
	
	/*Настройки таймера*/
	MDR_TIMER2->PSG = T2PSG; // Предделитель частоты
	MDR_TIMER2->ARR = T2ARR; // Основание счета (16 бит)
	MDR_TIMER2->CNTRL = 0x00000002; //буферизация
	
	/*Разрешения работы*/
	MDR_TIMER2->IE = 0x00001102;		//прерывание по cnt=arr
	NVIC_EnableIRQ(Timer2_IRQn); //разрешить прерывания таймера	
	MDR_TIMER2->CNTRL |= TIMER_CNTRL_CNT_EN; //  включить таймер.	
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


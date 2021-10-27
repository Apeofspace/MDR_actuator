#include "main.h"
#include "MDR32F9Qx_adc.h"

void Timer2_IRQHandler(void){
	MDR_TIMER2->STATUS &= ~(1UL << 1UL);
	control_loop();
}

void SysTick_Handler(void){
	timestamp_overflow_counter ++;
}

void DMA_IRQHandler(void)
{ 
  //  ������������� ����������� ��������� ���
  BRD_ADC1_RunSample(0);
  BRD_ADC1_RunSingle(0);
  //  ���������� ������, ����� ����� ������ sreq � DMA  
  ADC1_GetResult();
  
  completedIRQ = 1;
  PORT_ResetBits(MDR_PORTC,PORT_Pin_1);
	PORT_SetBits(MDR_PORTC,PORT_Pin_0);
  NVIC_ClearPendingIRQ (DMA_IRQn);
}

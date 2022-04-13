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
	#if defined(USE_DMA_FILTER)
  //  Останавливаем непрерывное измерение АЦП
  BRD_ADC1_RunSample(0);
  BRD_ADC1_RunSingle(0);
  //  Вычитываем данные, чтобы снять запрос sreq к DMA  
  ADC1_GetResult();
	DMA_Cmd(DMA_Channel_ADC1, DISABLE);	//необязательно?
  NVIC_ClearPendingIRQ (DMA_IRQn);
	#elif !defined (USE_DMA_FILTER)
	NVIC_ClearPendingIRQ (DMA_IRQn); //?????
	UART_DMACmd(MDR_UART2, UART_DMA_TXE, DISABLE);
	DMA_Cmd(DMA_Channel_UART2_TX, DISABLE);
	while(MDR_UART2->FR & UART_FR_BUSY);
	
	if (RESET_DE_RO_KOSTIL_FLAG)	//если надо переключиться на режим приема
	{
		Protocol_change_mode(MODE_RECIEVE);
	}
	#endif
}


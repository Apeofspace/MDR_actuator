#include "main.h"
#include "MDR32F9Qx_adc.h"

void Timer1_IRQHandler(void){
	MDR_TIMER1->STATUS &= ~(1UL << 1UL);
//	//MDR_TIMER1->STATUS = 0;
//	switch (PORT_ReadInputDataBit(MDR_PORTC, PORT_Pin_0)){
//		case Bit_RESET: 
//			PORT_SetBits(MDR_PORTC, PORT_Pin_0);
//			//PORT_SetBits(MDR_PORTA, PORT_Pin_1);
//			break;
//		case Bit_SET:  
//			PORT_ResetBits(MDR_PORTC, PORT_Pin_0);
//			//PORT_ResetBits(MDR_PORTA, PORT_Pin_1);
//			break;
//		}
}

void ADC_IRQHandler(void){
	if(ADC1_GetFlagStatus(ADCx_FLAG_END_OF_CONVERSION) == SET){
		
		
		uint32_t r = ADC1_GetResult();
		r = r&0xFFF;
		uint32_t m =  map_ADC_result(r ,0, 0xFFF, 0, T1MAX, MAPINVERT);
		MDR_TIMER1->CCR1 = m;
		MDR_ADC->ADC1_STATUS &=~(1 << 2);
		
		switch (PORT_ReadInputDataBit(MDR_PORTC, PORT_Pin_0)){
		case Bit_RESET: 
			PORT_SetBits(MDR_PORTC, PORT_Pin_0);
			//PORT_SetBits(MDR_PORTA, PORT_Pin_1);
			break;
		case Bit_SET:  
			PORT_ResetBits(MDR_PORTC, PORT_Pin_0);
			//PORT_ResetBits(MDR_PORTA, PORT_Pin_1);
			break;
		}
//		
  	MDR_ADC->ADC1_STATUS = ADCx_IT_END_OF_CONVERSION << 2; //סבנמס פכאדא REG_EOCIF
		//MDR_ADC->ADC1_STATUS = 1<< 2;
	}
}

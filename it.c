#include "main.h"
#include "MDR32F9Qx_adc.h"

void Timer2_IRQHandler(void){
	MDR_TIMER2->STATUS &= ~(1UL << 1UL);
	control_loop();
}

void SysTick_Handler(void){
	timestamp_overflow_counter ++;
}

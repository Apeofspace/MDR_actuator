#include "main.h"

uint32_t T1CCR = 65;
uint16_t com_angle = 2300;
uint8_t telemetry_to_send[TELEMETRY_DATA_BUFFER_SIZE];

uint8_t uart_busy_flag = RESET;
uint8_t recieving_data_flag = RESET;
uint8_t uart_package_recieved_flag = RESET;

//-----------------------------------------------------------------------
void deinit_all_GPIO(void){
	PORT_DeInit(MDR_PORTA);
	PORT_DeInit(MDR_PORTB);
	PORT_DeInit(MDR_PORTC);
	PORT_DeInit(MDR_PORTD);
	PORT_DeInit(MDR_PORTE);
	PORT_DeInit(MDR_PORTF);	
}
//-----------------------------------------------------------------------
void init_CPU(){
	MDR_RST_CLK->HS_CONTROL=RST_CLK_HSE_ON; //Вкл. HSE
	while((MDR_RST_CLK->CLOCK_STATUS&RST_CLK_CLOCK_STATUS_HSE_RDY )!=RST_CLK_CLOCK_STATUS_HSE_RDY );		//Ждём HSE
	MDR_RST_CLK->CPU_CLOCK=0x00000102; //Предварительно меняем мультиплексор c2 на CPU_C1
	MDR_RST_CLK->PLL_CONTROL = ((1 << 2) | (9 << 8));//вкл. PLL  | коэф. умножения = 9+1
	while ((MDR_RST_CLK->CLOCK_STATUS & RST_CLK_CLOCK_STATUS_PLL_CPU_RDY) != RST_CLK_CLOCK_STATUS_PLL_CPU_RDY);// Ждём PLL_CPU
	MDR_RST_CLK->CPU_CLOCK=0x00000106; // меняем мультиплексор с2 на PLLCPU
}
//-----------------------------------------------------------------------
void init_GPIO(){
	PORT_InitTypeDef GPIO_user_init;
	
	deinit_all_GPIO();
	
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTA, ENABLE);	
	
	GPIO_user_init.PORT_PULL_UP   = PORT_PULL_UP_OFF;
	GPIO_user_init.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
	GPIO_user_init.PORT_PD_SHM    = PORT_PD_SHM_OFF;
	GPIO_user_init.PORT_PD        = PORT_PD_DRIVER;
	GPIO_user_init.PORT_GFEN      = PORT_GFEN_OFF;
	
	/*Rudder 1 timer 1 channel 1 & 2*/
	GPIO_user_init.PORT_Pin       = (RUD1_CH1_PIN|RUD1_CH2_PIN|RUD1_CH1N_PIN|RUD1_CH2N_PIN);
	GPIO_user_init.PORT_OE        = PORT_OE_OUT;
	GPIO_user_init.PORT_FUNC      = RUD1_PORT_FUNC;
	GPIO_user_init.PORT_SPEED     = PORT_SPEED_MAXFAST;
	GPIO_user_init.PORT_MODE      = PORT_MODE_DIGITAL;	
	PORT_Init(RUD1_PWM_PORT, &GPIO_user_init);
}
//-----------------------------------------------------------------------
void init_ADC(void){
	ADC_InitTypeDef ADC_InitStruct;
	ADCx_InitTypeDef ADCx_InitStruct;
	
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTD, ENABLE);
	RST_CLK_PCLKcmd(RST_CLK_PCLK_ADC, ENABLE);	
	
	ADC_DeInit();
  ADC_StructInit(&ADC_InitStruct);
  ADCx_StructInit(&ADCx_InitStruct);
	
	ADCx_InitStruct.ADC_SamplingMode = ADC_SAMPLING_MODE_SINGLE_CONV;/* режим многократного преобразования */
	ADCx_InitStruct.ADC_ChannelNumber = ADC_OBJ_CHANNEL;/* выбор номера канала */
	ADCx_InitStruct.ADC_Prescaler = ADC_CLK_div_64; //выбор делителя тактовой частоты
  ADCx_InitStruct.ADC_DelayGo = 0x7;/* Дополнительная задержка перед началом преобразования после выбора канала (sequential mode) */
	
	ADC_Init(&ADC_InitStruct);
	ADC1_Init(&ADCx_InitStruct);
	ADC1_Cmd(ENABLE);				//ВКЛЮЧИТЬ АЦП				
}
//-----------------------------------------------------------------------
uint16_t get_COM_angle(void){
	return com_angle;	
}

//-----------------------------------------------------------------------
uint16_t filter_basic(uint16_t data, SIGNAL_CHANNEL channel){
	static uint16_t filter[BASIC_FILTER_SIZE][FILTER_NUMBER_OF_CHANNELS];
	static uint8_t filter_count = 0;
	uint16_t sum = 0; //должно хватить: 4 бита впереди свободные. значит можно просуммировать до 16 значений
	filter[filter_count][channel] = data;
	filter_count++;
	if (filter_count >= BASIC_FILTER_SIZE) filter_count = 0;
	for (uint8_t i = 0; i < BASIC_FILTER_SIZE; i++) sum += filter[i][channel]; 
	return(sum/BASIC_FILTER_SIZE);	
}

//-----------------------------------------------------------------------
uint16_t get_OBJ_angle(void){
	ADC1_SetChannel(ADC_OBJ_CHANNEL);
#if defined (USE_BASIC_FILTER)&&!defined(USE_DMA_FILTER)
		//run ADC
	ADC1_Start();
	while (!(MDR_ADC->ADC1_STATUS & ADCx_FLAG_END_OF_CONVERSION));	
	return filter_basic(ADC1_GetResult()&ADC_MASK, OBJ);
}
#elif defined (USE_NO_FILTER)
	//run ADC
	ADC1_SetChannel(ADC_OBJ_CHANNEL);
	ADC1_Start();
	while (!(MDR_ADC->ADC1_STATUS & ADCx_FLAG_END_OF_CONVERSION));	
	return ADC1_GetResult();
}
#endif
//-----------------------------------------------------------------------
uint32_t map_PWM(uint32_t data, uint32_t base_min, uint32_t base_max, uint32_t range_min, uint32_t range_max, uint8_t Ku, float dead_zone, MAP_INVERT invert){
	/* data - входное значение
		 base_min, base_max - пределы значений, в которых может колебаться входной сигнал
		 range_min, range_max - пределы значений, в которых может быть выходной сигнал
		 koef_usil - коэффициент усиления
		 dead_zone - мертвая зона (от 0 до 1)
		 invert - нужно ли инвертировать результат
		 result - выходное значение в диапазоне range, соответствующее входному значению в диапазоне base, умноженному на коэффициент усиления*/
	if ((range_min>range_max) || (base_min>base_max) || (data<base_min)) return 0;
	uint32_t delta_range = range_max-range_min;
	uint32_t delta_base = base_max - base_min; 
	uint32_t data_prived = data - base_min;
	float coef_zapoln = ((float)data_prived/(float)(delta_base+1));
	if (coef_zapoln<(float)dead_zone) return (invert == MAPNONINVERT)? range_min : range_max; //зона нечувствительности
	coef_zapoln = coef_zapoln * Ku;
	if (coef_zapoln>1) coef_zapoln = 1;
	return (invert == MAPNONINVERT)? range_min + (uint32_t)(coef_zapoln*(float)delta_range) : range_max - (uint32_t)(coef_zapoln*(float)delta_range);
}
//-----------------------------------------------------------------------
void main_loop(void){
	static uint8_t telemetry_divider = 0;
	uint16_t error_signal;
	uint32_t mapped_timer_ccr_value;
	uint16_t OBJ_angle = get_OBJ_angle();	
	uint16_t COM_angle = get_COM_angle();	

	if (COM_angle<COM_LIMIT_LEFT) COM_angle = COM_LIMIT_LEFT;
	if (COM_angle>COM_LIMIT_RIGHT) COM_angle = COM_LIMIT_RIGHT;

	error_signal = (OBJ_angle>COM_angle)?  OBJ_angle-COM_angle : COM_angle-OBJ_angle;	
	
	PWM_DIRECTION direction = (OBJ_angle>COM_angle)? PWMBACKWARD : PWMFORWARD;
	mapped_timer_ccr_value = map_PWM(error_signal, 0, 0xFFF, 0, T1ARR, PWM_KOEF_USIL, (float)PWM_DEAD_ZONE, MAPNONINVERT);
	if (mapped_timer_ccr_value > T1ARR) mapped_timer_ccr_value = T1ARR;
	change_PWM(!direction, mapped_timer_ccr_value);	// ***********тут можно менять дирекшн и !дирекшн в зависимости от того как потенциометр подключен
		
	if (++telemetry_divider>=4)
	{
		telemetry_divider = 0;
		if ((uart_package_recieved_flag == SET)&&(uart_busy_flag == RESET))
			{				
				uart_package_recieved_flag = RESET;
				#if defined(USE_PROTOCOL)
				telemetry_to_send[0] = OBJ_angle;
				telemetry_to_send[1] = OBJ_angle>>8;
				telemetry_to_send[2] = COM_angle;
				telemetry_to_send[3] = COM_angle>>8;
				
				telemetry_to_send[4] = mapped_timer_ccr_value;
				telemetry_to_send[5] = mapped_timer_ccr_value>>8;
				telemetry_to_send[6] = mapped_timer_ccr_value>>16;
				telemetry_to_send[7] = mapped_timer_ccr_value>>24;
				Protocol_send_message(telemetry_to_send, 8);
				#else
				telemetry_to_send[0] = OBJ_angle;
				telemetry_to_send[1] = OBJ_angle>>8;
				telemetry_to_send[2] = COM_angle;
				telemetry_to_send[3] = COM_angle>>8;
				
				telemetry_to_send[4] = mapped_timer_ccr_value;
				telemetry_to_send[5] = mapped_timer_ccr_value>>8;
				telemetry_to_send[6] = mapped_timer_ccr_value>>16;
				telemetry_to_send[7] = mapped_timer_ccr_value>>24;
				SEND_DATA_UART_DMA(telemetry_to_send, TELEMETRY_DATA_BUFFER_SIZE);
				#endif
			}
		}
}
//-----------------------------------------------------------------------
void change_PWM(PWM_DIRECTION direction, uint32_t mapped_ccr){
	//поочередный
	switch (direction){
		case PWMFORWARD:
			MDR_TIMER1->CCR1 = (T1ARR>>1) + (mapped_ccr>>1);
			MDR_TIMER1->CCR2 = (T1ARR>>1) - (mapped_ccr>>1);
			break;		
		case PWMBACKWARD:
			MDR_TIMER1->CCR1 = (T1ARR>>1) - (mapped_ccr>>1);
			MDR_TIMER1->CCR2 = (T1ARR>>1) + (mapped_ccr>>1);
	}	
}
//-----------------------------------------------------------------------
void UART_message_parsing(){	
	static uint16_t uart_timeout = 0;
	if (UART_GetFlagStatus (MDR_UART2, UART_FLAG_RXFF)== SET)
	{
		recieving_data_flag = SET;
		uart_timeout = 0;
		#if defined(USE_PROTOCOL)
		Protocol_recieve_message();
		#else
		UART_recieved_data_buffer[UART_recieved_data_length++] = (uint8_t) UART_ReceiveData(MDR_UART2);
		if (UART_recieved_data_length >= 2)
		{
			//little endian
			uint16_t res;
			res = UART_recieved_data_buffer[1];
			res = (res<<8)|UART_recieved_data_buffer[0];
			com_angle = res;
			UART_recieved_data_length=0;
			recieving_data_flag = RESET;
			uart_package_recieved_flag = SET;
		}
		#endif
	}
	else
	{
		if (uart_timeout<UART_TIMEOUT) uart_timeout++;
		else 
		{
			UART_recieved_data_length = 0;
			recieving_data_flag = RESET;
		}
	}
}
//-----------------------------------------------------------------------
int main(){
	init_CPU();
	init_GPIO();
	
	NVIC_SetPriority(DMA_IRQn,1);
	NVIC_SetPriority(Timer2_IRQn,2);
	
	init_ADC();
	init_DMA();
	init_UART();
	init_TIMER1();
	init_TIMER2();

	while (1){
		UART_message_parsing();
	}
}

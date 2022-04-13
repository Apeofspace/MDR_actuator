#include "main.h"

uint32_t T1CCR = 65;
uint16_t com_angle = 2250;
uint64_t timestamp_command_recieved = 0;
uint64_t timestamp_obj_recieved = 0;
uint8_t timestamp_overflow_counter = 0;
volatile uint32_t data_to_send[USB_DATA_BUFFER_SIZE];
//uint32_t completedIRQ;
uint32_t dmaCtrlStart;
uint16_t data_dma[DMA_FILTER_SIZE];
uint8_t amount_of_data_bites = USB_DATA_BUFFER_SIZE * 4;

void deinit_all_GPIO(void){
	PORT_DeInit(MDR_PORTA);
	PORT_DeInit(MDR_PORTB);
	PORT_DeInit(MDR_PORTC);
	PORT_DeInit(MDR_PORTD);
	PORT_DeInit(MDR_PORTE);
	PORT_DeInit(MDR_PORTF);	
}

void init_CPU(){
	MDR_RST_CLK->HS_CONTROL=RST_CLK_HSE_ON; //Вкл. HSE
	while((MDR_RST_CLK->CLOCK_STATUS&RST_CLK_CLOCK_STATUS_HSE_RDY )!=RST_CLK_CLOCK_STATUS_HSE_RDY );		//Ждём HSE
	MDR_RST_CLK->CPU_CLOCK=0x00000102; //Предварительно меняем мультиплексор c2 на CPU_C1
	MDR_RST_CLK->PLL_CONTROL = ((1 << 2) | (9 << 8));//вкл. PLL  | коэф. умножения = 9+1
	while ((MDR_RST_CLK->CLOCK_STATUS & RST_CLK_CLOCK_STATUS_PLL_CPU_RDY) != RST_CLK_CLOCK_STATUS_PLL_CPU_RDY);// Ждём PLL_CPU
	MDR_RST_CLK->CPU_CLOCK=0x00000106; // меняем мультиплексор с2 на PLLCPU
}

void init_GPIO(){
	PORT_InitTypeDef GPIO_user_init;
	deinit_all_GPIO();
	
	//timer1 PWM
	GPIO_user_init.PORT_Pin       = (CH1_PIN|CH2_PIN|CH1N_PIN|CH2N_PIN);
	GPIO_user_init.PORT_OE        = PORT_OE_OUT;
	GPIO_user_init.PORT_PULL_UP   = PORT_PULL_UP_OFF;
	GPIO_user_init.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
	GPIO_user_init.PORT_PD_SHM    = PORT_PD_SHM_OFF;
	GPIO_user_init.PORT_PD        = PORT_PD_DRIVER;
	GPIO_user_init.PORT_GFEN      = PORT_GFEN_OFF;
	GPIO_user_init.PORT_FUNC      = PORT_FUNC_ALTER;
	GPIO_user_init.PORT_SPEED     = PORT_SPEED_MAXFAST;
	GPIO_user_init.PORT_MODE      = PORT_MODE_DIGITAL;	
	PORT_Init(PWM_PORT, &GPIO_user_init);
}

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

void init_TIMER1(){
	/*Управление ШИМ*/
	/*Обнуление*/
	deinit_TIMER(MDR_TIMER1);
	
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

void init_TIMER2(){
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

void init_ADC(void){
	ADC_InitTypeDef ADC_InitStruct;
	ADCx_InitTypeDef ADCx_InitStruct;
	
	ADC_DeInit();
  ADC_StructInit(&ADC_InitStruct);
  ADCx_StructInit(&ADCx_InitStruct);
	
//	ADC_InitStruct.ADC_StartDelay = 0x05;
	
	ADCx_InitStruct.ADC_SamplingMode = ADC_SAMPLING_MODE_SINGLE_CONV;/* режим многократного преобразования */
	ADCx_InitStruct.ADC_ChannelNumber = ADC_OBJ_CHANNEL;/* выбор номера канала */
	ADCx_InitStruct.ADC_Prescaler = ADC_CLK_div_64; //выбор делителя тактовой частоты
  ADCx_InitStruct.ADC_DelayGo = 0x7;/* Дополнительная задержка перед началом преобразования после выбора канала (sequential mode) */
	
	ADC_Init(&ADC_InitStruct);
	ADC1_Init(&ADCx_InitStruct);
	ADC1_Cmd(ENABLE);				//ВКЛЮЧИТЬ АЦП				
}

void init_PER(void){		
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTC, ENABLE);		
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTA, ENABLE);	
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTD, ENABLE);	
	RST_CLK_PCLKcmd(RST_CLK_PCLK_ADC, ENABLE);
	
	/*Тактирование таймеров*/
	MDR_RST_CLK->PER_CLOCK |= (1 << 14 )|(1 << 15 );
	MDR_RST_CLK->TIM_CLOCK = (RST_CLK_TIM_CLOCK_TIM1_CLK_EN)|(RST_CLK_TIM_CLOCK_TIM2_CLK_EN);
}

uint16_t get_COM_angle(void){
	return com_angle;
}

uint16_t filter_analog(uint16_t data, SIGNAL_CHANNEL channel){
	static uint16_t filter[BASIC_FILTER_SIZE][3];
	static uint8_t filter_count = 0;
	uint16_t sum = 0; //должно хватить: 4 бита впереди свободные. значит можно просуммировать до 16 значений
	filter[filter_count][channel] = data;
	filter_count++;
	if (filter_count >= BASIC_FILTER_SIZE) filter_count = 0;
	for (uint8_t i = 0; i < BASIC_FILTER_SIZE; i++) sum += filter[i][channel]; 
	return(sum/BASIC_FILTER_SIZE);	
}

uint16_t get_OBJ_angle(void){
	ADC1_SetChannel(ADC_OBJ_CHANNEL);
	#if defined (USE_BASIC_FILTER)&&!defined(USE_DMA_FILTER)
		//run ADC
	BRD_ADC1_RunSample(0); //0 = одиночное 1 = последовательное
	ADC1_Start();
	while (!(MDR_ADC->ADC1_STATUS & ADCx_FLAG_END_OF_CONVERSION));	
	return filter_analog(ADC1_GetResult()&ADC_MASK, OBJ);
}
	#elif defined (USE_DMA_FILTER)
	uint32_t sum = 0;
	BRD_ADC1_RunSample(1);	
	// Restart DMA
	DMA_ControlTable[DMA_Channel_ADC1].DMA_Control = dmaCtrlStart;
	DMA_Cmd(DMA_Channel_ADC1, ENABLE);	
	for (uint32_t i = 0; i < DMA_FILTER_SIZE; i++) sum += data_dma[i]&ADC_MASK; 
	#if defined(USE_BASIC_FILTER)
	return filter_analog(sum/DMA_FILTER_SIZE, OBJ); // не нужн
	#elif !defined(USE_BASIC_FILTER)
	return sum/DMA_FILTER_SIZE;
	#endif
}
	#elif defined (USE_NO_FILTER)
	//run ADC
	ADC1_SetChannel(ADC_OBJ_CHANNEL);
	BRD_ADC1_RunSample(0);
	ADC1_Start();
	while (!(MDR_ADC->ADC1_STATUS & ADCx_FLAG_END_OF_CONVERSION));	
	return ADC1_GetResult();
}
#endif

#if defined (MEASURE_AND_SEND_TOK)
uint16_t get_TOK(void){
	//this doesn't work with DMA yet
	ADC1_SetChannel(ADC_TOK_CHANNEL);
	#if defined(USE_DMA_FILTER)
	uint32_t sum = 0;
	BRD_ADC1_RunSample(1);
	DMA_ControlTable[DMA_Channel_ADC1].DMA_Control = dmaCtrlStart;
	DMA_Cmd(DMA_Channel_ADC1, ENABLE);	
	for (uint32_t i = 0; i < DMA_FILTER_SIZE; i++) sum += data_dma[i]&ADC_MASK; 
	return sum/DMA_FILTER_SIZE;	
}
	#elif !defined (USE_DMA_FILTER)
	//run ADC
	BRD_ADC1_RunSample(0);
	ADC1_Start();
	while (!(MDR_ADC->ADC1_STATUS & ADCx_FLAG_END_OF_CONVERSION));	
	return ADC1_GetResult();
}
#endif
#endif

/* Коэффициент заполнения умножается на koef_usil, но не может быть больше 1 */
uint32_t map_PWM(uint32_t data, uint32_t base_min, uint32_t base_max, uint32_t range_min, uint32_t range_max, uint8_t koef_usil, MAP_INVERT invert){
	if ((range_min>range_max) || (base_min>base_max) || (data<base_min)) return 0;
	uint32_t delta_range = range_max-range_min;
	uint32_t delta_base = base_max - base_min;
	uint32_t data_prived = data - base_min;
	float coef_zapoln = ((float)data_prived/(float)(delta_base+1));
	if (coef_zapoln<(float)PWMDEADZONE) return (invert == MAPNONINVERT)? range_min : range_max; //зона нечувствительности
	coef_zapoln = coef_zapoln * koef_usil;
	if (coef_zapoln>1) coef_zapoln = 1;
	return (invert == MAPNONINVERT)? range_min + (uint32_t)(coef_zapoln*(float)delta_range) : range_max - (uint32_t)(coef_zapoln*(float)delta_range);
}

void control_loop(void){
	uint16_t PWMpower;
	uint32_t mapped_ccr;
	uint16_t OBJ_angle = get_OBJ_angle();
	take_timestamp(&timestamp_obj_recieved);
	timestamp_command_recieved = timestamp_obj_recieved;
	reload_SysTick();
	
	uint16_t COM_angle = get_COM_angle();
	take_timestamp(&timestamp_command_recieved);
	reload_SysTick();
	
	uint16_t TOK = 0;
	#if defined (MEASURE_AND_SEND_TOK)
	TOK = get_TOK();
	#endif

	if (COM_angle<COM_LIMIT_LEFT) COM_angle = COM_LIMIT_LEFT;
	if (COM_angle>COM_LIMIT_RIGHT) COM_angle = COM_LIMIT_RIGHT;

	PWMpower = (OBJ_angle>COM_angle)?  OBJ_angle-COM_angle : COM_angle-OBJ_angle;
	PWM_DIRECTION direction = (OBJ_angle>COM_angle)? PWMBACKWARD : PWMFORWARD;
	mapped_ccr = map_PWM(PWMpower, 0, 0xFFF, 0, T1ARR, PWM_KOEF_USIL, MAPNONINVERT);
//	mapped_ccr = PWMpower *PWM_KOEF_USIL;
	if (mapped_ccr > T1ARR) mapped_ccr = T1ARR;
	changePWM(direction, mapped_ccr);	
	
	data_to_send[0] = (COM_angle<<16)|(OBJ_angle);
	data_to_send[1] = (timestamp_command_recieved&0xFFFFFFFF);
	data_to_send[2] = (timestamp_command_recieved>>32)&0xFFFFFFFF;
	data_to_send[3] = (timestamp_obj_recieved&0xFFFFFFFF);
	data_to_send[4] = (timestamp_obj_recieved>>32)&0xFFFFFFFF;
	data_to_send[5] = mapped_ccr;
	data_to_send[6] = direction;
	data_to_send[7] = TOK;
	
	send_data(amount_of_data_bites);
}

void changePWM(PWM_DIRECTION direction, uint32_t mapped_ccr){
	//поочередный (для него надо включать MAPNONINVERT и менять data_to_send[5])
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

void init_SysTick(){
	SysTick->LOAD = 0x00FFFFFF; 
	SysTick->CTRL = 0;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk |SysTick_CTRL_TICKINT_Msk; //HCLK and Enable and interrupt enable
	NVIC_EnableIRQ(SysTick_IRQn);
}

void reload_SysTick(){
	SysTick->VAL = 1; //запись любого значения очищает регистр в 0, а также очищает COUNTFLAG в SysTick->CTRL
	timestamp_overflow_counter = 0;
}

void take_timestamp(uint64_t* timestamp){
	*timestamp +=0x00FFFFFF-SysTick->VAL;
	if (timestamp_overflow_counter>0) *timestamp += timestamp_overflow_counter * 0x00FFFFFF;
}

void send_data(uint32_t Length){
	//Отправляются данные о принятом сигнале и моменте принятия сигнала, о снятом сигнале с объекта управления и времени снятия сигнала
	USB_CDC_SendData(&data_to_send, Length);
}

int main(){
	init_CPU();
	init_USB();
  init_PER();
	init_GPIO();
	init_debug_LED();
	#ifdef USE_DMA_FILTER
	init_DMA();
	#endif
	init_ADC();
	init_UART();
	init_SysTick();
	init_TIMER1();
	init_TIMER2();
	

	while (1){}
}

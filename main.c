#include "main.h"

uint32_t T1CCR = 65;
uint16_t com_angle = 2000;
uint64_t timestamp_command_recieved = 0;
uint64_t timestamp_obj_recieved = 0;
uint8_t timestamp_overflow_counter = 0;
volatile uint32_t data_to_send[5];
uint32_t completedIRQ;
uint32_t dmaCtrlStart;
uint16_t data_dma[FILTER_SIZE];

void deinit_all_GPIO(void){
	PORT_DeInit(MDR_PORTA);
	PORT_DeInit(MDR_PORTB);
	PORT_DeInit(MDR_PORTC);
	PORT_DeInit(MDR_PORTD);
	PORT_DeInit(MDR_PORTE);
	PORT_DeInit(MDR_PORTF);	
}

void init_CPU(){
	MDR_RST_CLK->HS_CONTROL=RST_CLK_HSE_ON; //���. HSE
	while((MDR_RST_CLK->CLOCK_STATUS&RST_CLK_CLOCK_STATUS_HSE_RDY )!=RST_CLK_CLOCK_STATUS_HSE_RDY );		//��� HSE
	MDR_RST_CLK->CPU_CLOCK=0x00000102; //�������������� ������ ������������� c2 �� CPU_C1
	MDR_RST_CLK->PLL_CONTROL = ((1 << 2) | (9 << 8));//���. PLL  | ����. ��������� = 9+1
	while ((MDR_RST_CLK->CLOCK_STATUS & RST_CLK_CLOCK_STATUS_PLL_CPU_RDY) != RST_CLK_CLOCK_STATUS_PLL_CPU_RDY);// ��� PLL_CPU
	MDR_RST_CLK->CPU_CLOCK=0x00000106; // ������ ������������� �2 �� PLLCPU
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
	
	//ADC1
	GPIO_user_init.PORT_Pin       = (ADC_OBJ_PIN);
	GPIO_user_init.PORT_OE        = PORT_OE_IN;
	GPIO_user_init.PORT_PULL_UP   = PORT_PULL_UP_OFF;
	GPIO_user_init.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
	GPIO_user_init.PORT_PD_SHM    = PORT_PD_SHM_OFF;
	GPIO_user_init.PORT_PD        = PORT_PD_DRIVER;
	GPIO_user_init.PORT_GFEN      = PORT_GFEN_OFF;
	GPIO_user_init.PORT_FUNC      = PORT_FUNC_PORT;
	GPIO_user_init.PORT_SPEED     = PORT_SPEED_MAXFAST;
	GPIO_user_init.PORT_MODE      = PORT_MODE_ANALOG;	
	PORT_Init(ADC_PORT, &GPIO_user_init);
}

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

void init_TIMER1(){
	/*���������*/
	deinit_TIMER(MDR_TIMER1);
	
	/*��������� �������*/
	MDR_TIMER1->PSG = T1PSG; // ������������ �������
	MDR_TIMER1->ARR = T1ARR; // ��������� ����� (16 ���)
	MDR_TIMER1->CNTRL = TIMER_CNTRL_ARRB_EN; //����������� 
	
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
	MDR_TIMER1->CNTRL |= TIMER_CNTRL_CNT_EN; // ���� ����� �� TIM_CLK, � �������� ������.	
}

void init_TIMER2(){
	/*���������*/
	deinit_TIMER(MDR_TIMER2);
	
	/*��������� �������*/
	MDR_TIMER2->PSG = T2PSG; // ������������ �������
	MDR_TIMER2->ARR = T2ARR; // ��������� ����� (16 ���)
	MDR_TIMER2->CNTRL = TIMER_CNTRL_ARRB_EN; //����������� 
	
	/*���������� ������*/
	MDR_TIMER2->IE = 0x00001102;		//���������� �� cnt=arr
	NVIC_EnableIRQ(Timer2_IRQn); //��������� ���������� �������	
	MDR_TIMER2->CNTRL |= TIMER_CNTRL_CNT_EN; // ���� ����� �� TIM_CLK, � �������� ������.	
}

void init_ADC(void){
	ADC_InitTypeDef ADC_InitStruct;
	ADCx_InitTypeDef ADCx_InitStruct;
	
	ADC_DeInit();
  ADC_StructInit(&ADC_InitStruct);
  ADCx_StructInit(&ADCx_InitStruct);
	
	ADC_InitStruct.ADC_StartDelay = 0x05;
	
	ADCx_InitStruct.ADC_SamplingMode = ADC_SAMPLING_MODE_SINGLE_CONV;/* ����� ������������� �������������� */
	ADCx_InitStruct.ADC_ChannelNumber = ADC_OBJ_CHANNEL;/* ����� ������ ������ */
	ADCx_InitStruct.ADC_Prescaler = ADC_CLK_div_128; //����� �������� �������� �������
  ADCx_InitStruct.ADC_DelayGo = 0x7;/* �������������� �������� ����� ������� �������������� ����� ������ ������ (sequential mode) */
	
	ADC_Init(&ADC_InitStruct);
	ADC1_Init(&ADCx_InitStruct);
	ADC1_Cmd(ENABLE);				//�������� ���				
}

void init_PER(void){		
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTC, ENABLE);		
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTA, ENABLE);	
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTD, ENABLE);	
	RST_CLK_PCLKcmd(RST_CLK_PCLK_ADC, ENABLE);
	
	/*������������ ��������*/
	MDR_RST_CLK->PER_CLOCK |= (1 << 14 )|(1 << 15 );
	MDR_RST_CLK->TIM_CLOCK = (RST_CLK_TIM_CLOCK_TIM1_CLK_EN)|(RST_CLK_TIM_CLOCK_TIM2_CLK_EN);
}

uint16_t get_COM_angle(void){
	return com_angle;
}

uint16_t get_OBJ_angle(void){
	#if defined (USE_DMA_FILTER)
	PORT_SetBits(MDR_PORTC,PORT_Pin_1);
	uint32_t sum = 0;
	completedIRQ = 0;
	// Restart DMA
	DMA_ControlTable[DMA_Channel_ADC1].DMA_Control = dmaCtrlStart;
	DMA_Cmd(DMA_Channel_ADC1, ENABLE);	
	BRD_ADC1_RunSample(1);
	while(!completedIRQ){};
	for (uint32_t i = 0; i < FILTER_SIZE; i++) sum += data_dma[i]; 
	uint32_t res = sum/FILTER_SIZE;
	return (sum/FILTER_SIZE);
}
	#elif defined (USE_BASIC_FILTER)
		//run ADC
	ADC1_SetChannel(ADC_OBJ_CHANNEL);
	ADC1_Start();
	while (!(MDR_ADC->ADC1_STATUS & ADCx_FLAG_END_OF_CONVERSION));	
	return filter_analog(ADC1_GetResult()&ADC_MASK, OBJ);
	
}

uint16_t filter_analog(uint16_t data, SIGNAL_CHANNEL channel){
	static uint16_t filter[FILTER_SIZE][2];
	static uint8_t filter_count = 0;
	uint16_t sum = 0; //������ �������: 4 ���� ������� ���������. ������ ����� �������������� �� 16 ��������
	filter[filter_count][channel] = data;
	filter_count++;
	if (filter_count >= FILTER_SIZE) filter_count = 0;
	for (uint8_t i = 0; i < FILTER_SIZE; i++) sum += filter[i][channel]; 
	return(sum/FILTER_SIZE);	
}

	#elif defined (USE_NO_FILTER)
	//run ADC
	ADC1_SetChannel(ADC_OBJ_CHANNEL);
	ADC1_Start();
	while (!(MDR_ADC->ADC1_STATUS & ADCx_FLAG_END_OF_CONVERSION));	
	return ADC1_GetResult();
}
#endif

/* ����������� ���������� ���������� �� saturation_coef, �� �� ����� ���� ������ 1 */
uint32_t map_PWM(uint32_t data, uint32_t base_min, uint32_t base_max, uint32_t range_min, uint32_t range_max, uint8_t saturation_coef, MAP_INVERT invert){
	if ((range_min>range_max) || (base_min>base_max) || (data<base_min)) return 0;
	uint32_t delta_range = range_max-range_min;
	uint32_t delta_base = base_max - base_min;
	uint32_t data_prived = data - base_min;
	float coef_zapoln = ((float)data_prived/(float)delta_base);
	if (coef_zapoln<(float)PWMDEADZONE) return (invert == MAPNONINVERT)? range_min : range_max; //���� ������������������
	coef_zapoln = coef_zapoln * saturation_coef;
	if (coef_zapoln>1) coef_zapoln = 1;
	return (invert == MAPNONINVERT)? range_min + (uint32_t)(coef_zapoln*(float)delta_range) : range_max - (uint32_t)(coef_zapoln*(float)delta_range);
}

void control_loop(void){
	uint16_t OBJ_angle = get_OBJ_angle();
	take_timestamp(&timestamp_obj_recieved);
	timestamp_command_recieved = timestamp_obj_recieved;
	reload_SysTick();
	
	uint16_t COM_angle = get_COM_angle();
	take_timestamp(&timestamp_command_recieved);
	reload_SysTick();
	
	data_to_send[0] = (COM_angle<<16)|(OBJ_angle);
	data_to_send[2] = (timestamp_command_recieved>>32)&0xFFFFFFFF;
	data_to_send[1] = (timestamp_command_recieved&0xFFFFFFFF);
	data_to_send[4] = (timestamp_obj_recieved>>32)&0xFFFFFFFF;
	data_to_send[3] = (timestamp_obj_recieved&0xFFFFFFFF);
	send_data();
	
	if (COM_angle<COM_LIMIT_LEFT) COM_angle = COM_LIMIT_LEFT;
	if (COM_angle>COM_LIMIT_RIGHT) COM_angle = COM_LIMIT_RIGHT;

 	if (COM_angle>=OBJ_angle){		
		changePWM(PWMFORWARD, COM_angle-OBJ_angle);		
	}
	if (OBJ_angle>COM_angle){
		changePWM(PWMBACKWARD, OBJ_angle-COM_angle);		
	}
}

void changePWM(PWM_DIRECTION direction, uint16_t PWMpower){
	uint32_t mapped_ccr = map_PWM(PWMpower, 0, 0xFFF, 0, T1ARR, PWM_SATURATION_COEFFICIENT, MAPINVERT);
//	//��������������	
	switch (direction){
		case PWMFORWARD:
			MDR_TIMER1->CCR1 = mapped_ccr;
			MDR_TIMER1->CCR2 = T1ARR; //��������
			break;		
		case PWMBACKWARD:
			MDR_TIMER1->CCR1 = T1ARR; //��������
			MDR_TIMER1->CCR2 = mapped_ccr;
	}
}

void init_SysTick(){
	SysTick->LOAD = 0x00FFFFFF; 
	SysTick->CTRL = 0;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk |SysTick_CTRL_TICKINT_Msk; //HCLK and Enable and interrupt enable
	NVIC_EnableIRQ(SysTick_IRQn);
}

void reload_SysTick(){
	SysTick->VAL = 1; //������ ������ �������� ������� ������� � 0, � ����� ������� COUNTFLAG � SysTick->CTRL
	timestamp_overflow_counter = 0;
}

void take_timestamp(uint64_t* timestamp){
	*timestamp +=0x00FFFFFF-SysTick->VAL;
	if (timestamp_overflow_counter>0) *timestamp += timestamp_overflow_counter * 0x00FFFFFF;
}

void send_data(){
	//������������ ������ � �������� ������� � ������� �������� �������, � ������ ������� � ������� ���������� � ������� ������ �������
	USB_CDC_SendData(&data_to_send, 20);
}

int main(){
	init_CPU();
	init_USB();
  init_PER();
	init_GPIO();
	#ifdef USE_DMA_FILTER
	init_DMA();
	#endif
	init_ADC();
	init_SysTick();
	init_TIMER1();
	init_TIMER2();

	while (1){}
}

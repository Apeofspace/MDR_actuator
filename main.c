#include "main.h"

uint32_t T1CCR = 65;
uint16_t com_angle = 2250;
uint64_t timestamp_command_recieved = 0;
uint64_t timestamp_obj_recieved = 0;
uint8_t timestamp_overflow_counter = 0;
uint8_t telemetry_to_send[TELEMETRY_DATA_BUFFER_SIZE];
//uint32_t telemetry_to_send[TELEMETRY_DATA_BUFFER_SIZE];
//uint32_t amount_of_telemetry_bytes = TELEMETRY_DATA_BUFFER_SIZE * 4;

uint8_t uart_busy_flag = RESET;
uint8_t sending_telemetry_flag = RESET;
uint8_t recieving_data_flag = RESET;
uint8_t uart_package_recieved_flag = RESET;


#if defined(USE_DMA_FILTER) 
uint32_t dmaCtrlStart;
uint16_t data_dma[DMA_FILTER_SIZE];
#endif
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
	MDR_RST_CLK->HS_CONTROL=RST_CLK_HSE_ON; //���. HSE
	while((MDR_RST_CLK->CLOCK_STATUS&RST_CLK_CLOCK_STATUS_HSE_RDY )!=RST_CLK_CLOCK_STATUS_HSE_RDY );		//��� HSE
	MDR_RST_CLK->CPU_CLOCK=0x00000102; //�������������� ������ ������������� c2 �� CPU_C1
	MDR_RST_CLK->PLL_CONTROL = ((1 << 2) | (9 << 8));//���. PLL  | ����. ��������� = 9+1
	while ((MDR_RST_CLK->CLOCK_STATUS & RST_CLK_CLOCK_STATUS_PLL_CPU_RDY) != RST_CLK_CLOCK_STATUS_PLL_CPU_RDY);// ��� PLL_CPU
	MDR_RST_CLK->CPU_CLOCK=0x00000106; // ������ ������������� �2 �� PLLCPU
}
//-----------------------------------------------------------------------
void init_GPIO(){
	PORT_InitTypeDef GPIO_user_init;
	
	deinit_all_GPIO();
	
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTA, ENABLE);	
//	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTB, ENABLE);	
//	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTD, ENABLE);	
//	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTE, ENABLE);	
	
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
	
//	/*Rudder 2 timer 3 channel 3 & 4 */
//	GPIO_user_init.PORT_Pin       = (RUD2_CH1_PIN|RUD2_CH2_PIN|RUD2_CH1N_PIN|RUD2_CH2N_PIN);
//	GPIO_user_init.PORT_OE        = PORT_OE_OUT;
//	GPIO_user_init.PORT_FUNC      = RUD2_PORT_FUNC;
//	GPIO_user_init.PORT_SPEED     = PORT_SPEED_MAXFAST;
//	GPIO_user_init.PORT_MODE      = PORT_MODE_DIGITAL;	
//	PORT_Init(RUD2_PWM_PORT, &GPIO_user_init);
//	
//	/*Rudder 3 timer 3 channel 1 & 2*/
//	GPIO_user_init.PORT_Pin       = (RUD3_CH1_PIN|RUD3_CH2_PIN|RUD3_CH1N_PIN|RUD3_CH2N_PIN);
//	GPIO_user_init.PORT_OE        = PORT_OE_OUT;
//	GPIO_user_init.PORT_FUNC      = RUD3_PORT_FUNC;
//	GPIO_user_init.PORT_SPEED     = PORT_SPEED_MAXFAST;
//	GPIO_user_init.PORT_MODE      = PORT_MODE_DIGITAL;	
//	PORT_Init(RUD1_PWM_PORT, &GPIO_user_init);
//	
//	/*Rudder 4  timer 2 channel 1 & 3*/
//	GPIO_user_init.PORT_Pin       = (RUD4_CH1_PIN|RUD4_CH2_PIN|RUD4_CH1N_PIN|RUD4_CH2N_PIN);
//	GPIO_user_init.PORT_OE        = PORT_OE_OUT;
//	GPIO_user_init.PORT_FUNC      = RUD4_PORT_FUNC;
//	GPIO_user_init.PORT_SPEED     = PORT_SPEED_MAXFAST;
//	GPIO_user_init.PORT_MODE      = PORT_MODE_DIGITAL;	
//	PORT_Init(RUD1_PWM_PORT, &GPIO_user_init);
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
	
	ADCx_InitStruct.ADC_SamplingMode = ADC_SAMPLING_MODE_SINGLE_CONV;/* ����� ������������� �������������� */
	ADCx_InitStruct.ADC_ChannelNumber = ADC_OBJ_CHANNEL;/* ����� ������ ������ */
	ADCx_InitStruct.ADC_Prescaler = ADC_CLK_div_64; //����� �������� �������� �������
  ADCx_InitStruct.ADC_DelayGo = 0x7;/* �������������� �������� ����� ������� �������������� ����� ������ ������ (sequential mode) */
	
	ADC_Init(&ADC_InitStruct);
	ADC1_Init(&ADCx_InitStruct);
	ADC1_Cmd(ENABLE);				//�������� ���				
}
//-----------------------------------------------------------------------
uint16_t get_COM_angle(void){
	return com_angle;	
}

//-----------------------------------------------------------------------
#if defined(USE_DMA_FILTER)
void BRD_ADC1_RunSingle(uint32_t goEna)
	//same as ADC1_Start();
{
  if (goEna)
    MDR_ADC->ADC1_CFG |= ADC1_CFG_REG_GO;
  else
    MDR_ADC->ADC1_CFG &= ~ADC1_CFG_REG_GO;
}
//-----------------------------------------------------------------------
void BRD_ADC1_RunSample(uint32_t sampleEna)
	//starts continuous conversion
{
  if (sampleEna)
    MDR_ADC->ADC1_CFG |= ADC1_CFG_REG_SAMPLE;
  else
    MDR_ADC->ADC1_CFG &= ~ADC1_CFG_REG_SAMPLE;
}
#endif
//-----------------------------------------------------------------------
uint16_t filter_analog(uint16_t data, SIGNAL_CHANNEL channel){
	static uint16_t filter[BASIC_FILTER_SIZE][3];
	static uint8_t filter_count = 0;
	uint16_t sum = 0; //������ �������: 4 ���� ������� ���������. ������ ����� �������������� �� 16 ��������
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
//	BRD_ADC1_RunSample(0); //0 = ��������� 1 = ����������������
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
	return filter_analog(sum/DMA_FILTER_SIZE, OBJ); // �� ����
	#elif !defined(USE_BASIC_FILTER)
	return sum/DMA_FILTER_SIZE;
	#endif
}
	#elif defined (USE_NO_FILTER)
	//run ADC
	ADC1_SetChannel(ADC_OBJ_CHANNEL);
//	BRD_ADC1_RunSample(0);
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
//-----------------------------------------------------------------------
/* ����������� ���������� ���������� �� koef_usil, �� �� ����� ���� ������ 1 */
uint32_t map_PWM(uint32_t data, uint32_t base_min, uint32_t base_max, uint32_t range_min, uint32_t range_max, uint8_t koef_usil, MAP_INVERT invert){
	if ((range_min>range_max) || (base_min>base_max) || (data<base_min)) return 0;
	uint32_t delta_range = range_max-range_min;
	uint32_t delta_base = base_max - base_min;
	uint32_t data_prived = data - base_min;
	float coef_zapoln = ((float)data_prived/(float)(delta_base+1));
	if (coef_zapoln<(float)PWMDEADZONE) return (invert == MAPNONINVERT)? range_min : range_max; //���� ������������������
	coef_zapoln = coef_zapoln * koef_usil;
	if (coef_zapoln>1) coef_zapoln = 1;
	return (invert == MAPNONINVERT)? range_min + (uint32_t)(coef_zapoln*(float)delta_range) : range_max - (uint32_t)(coef_zapoln*(float)delta_range);
}
//-----------------------------------------------------------------------
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
	if (mapped_ccr > T1ARR) mapped_ccr = T1ARR;
	changePWM(!direction, mapped_ccr);	// ***********��� ����� ������ ������� � !������� � ����������� �� ���� ��� ������������ ���������
	 
	if ((uart_package_recieved_flag == SET)&&(uart_busy_flag == RESET))
		{
			telemetry_to_send[0] = OBJ_angle;
			telemetry_to_send[1] = OBJ_angle>>8;
			telemetry_to_send[2] = COM_angle;
			telemetry_to_send[3] = COM_angle>>8;
			
			telemetry_to_send[4] = mapped_ccr;
			telemetry_to_send[5] = mapped_ccr>>8;
			telemetry_to_send[6] = mapped_ccr>>16;
			telemetry_to_send[7] = mapped_ccr>>24;
			
//			
//			telemetry_to_send[4] = timestamp_command_recieved;
//			telemetry_to_send[5] = timestamp_command_recieved>>8;
//			telemetry_to_send[6] = timestamp_command_recieved>>16;
//			telemetry_to_send[7] = timestamp_command_recieved>>24;
//			
//			telemetry_to_send[8] = timestamp_command_recieved>>32;
//			telemetry_to_send[9] = timestamp_command_recieved>>40;
//			telemetry_to_send[10] = timestamp_command_recieved>>48;
//			telemetry_to_send[11] = timestamp_command_recieved>>56;
//			
//			telemetry_to_send[12] = timestamp_obj_recieved;
//			telemetry_to_send[13] = timestamp_obj_recieved>>8;
//			telemetry_to_send[14] = timestamp_obj_recieved>>16;
//			telemetry_to_send[15] = timestamp_obj_recieved>>24;
//			
//			telemetry_to_send[16] = timestamp_obj_recieved>>32;
//			telemetry_to_send[17] = timestamp_obj_recieved>>40;
//			telemetry_to_send[18] = timestamp_obj_recieved>>48;
//			telemetry_to_send[19] = timestamp_obj_recieved>>56;
//			
//			telemetry_to_send[20] = mapped_ccr;
//			telemetry_to_send[21] = mapped_ccr>>8;
//			telemetry_to_send[22] = mapped_ccr>>16;
//			telemetry_to_send[23] = mapped_ccr>>24;
//			
//			telemetry_to_send[24] = direction;
//			telemetry_to_send[25] = TOK;
//			telemetry_to_send[26] = TOK>>8;
			
			uart_busy_flag = SET;
//			send_telemetry(TELEMETRY_DATA_BUFFER_SIZE); //��� ����� �������� � ����� �� �������
			uart_package_recieved_flag = RESET;
			
//			RESET_DE_RO_KOSTIL_FLAG = 1;
			SEND_DATA_UART_DMA(telemetry_to_send, TELEMETRY_DATA_BUFFER_SIZE);
		}
}
//-----------------------------------------------------------------------
void changePWM(PWM_DIRECTION direction, uint32_t mapped_ccr){
	//����������� (��� ���� ���� �������� MAPNONINVERT � ������ telemetry_to_send[5])
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
void reload_SysTick(){
	SysTick->VAL = 1; //������ ������ �������� ������� ������� � 0, � ����� ������� COUNTFLAG � SysTick->CTRL
	timestamp_overflow_counter = 0;
}
//-----------------------------------------------------------------------
void take_timestamp(uint64_t* timestamp){
	*timestamp +=0x00FFFFFF-SysTick->VAL;
	if (timestamp_overflow_counter>0) *timestamp += timestamp_overflow_counter * 0x00FFFFFF;
}
//-----------------------------------------------------------------------
void send_telemetry(uint32_t Length){

	PORT_SetBits(RS485_DE_RO_PORT, RS485_DE_RO_PIN);
	
	for (uint32_t i =0; i < Length; i++){
		while (UART_GetFlagStatus (UART485, UART_FLAG_BUSY)== SET);
		UART_SendData(UART485, telemetry_to_send[i]);
	}
	
	while (UART_GetFlagStatus (UART485, UART_FLAG_BUSY)== SET);
	PORT_ResetBits(RS485_DE_RO_PORT, RS485_DE_RO_PIN);
}
//-----------------------------------------------------------------------
void UART_message_parsing(){
	uint16_t res;
	static uint16_t uart_timeout = 0;
	if (UART_GetFlagStatus (MDR_UART2, UART_FLAG_RXFF)== SET)
	{
		recieving_data_flag = SET;
		uart_timeout = 0;
		UART_recieved_data_buffer[UART_recieved_data_length++] = (uint8_t) UART_ReceiveData(MDR_UART2);
		if (UART_recieved_data_length >= 2)
		{
			//little endian
			res = UART_recieved_data_buffer[1];
			res = (res<<8)|UART_recieved_data_buffer[0];
			com_angle = res;
			UART_recieved_data_length=0;
			recieving_data_flag = RESET;
			uart_package_recieved_flag = SET;
		}
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
	#ifdef USE_DMA_FILTER
	init_DMA();
	#endif
	
	NVIC_SetPriority(DMA_IRQn,1);
	NVIC_SetPriority(Timer2_IRQn,2);
	
	init_ADC();
	DMA_common_ini();
	init_UART();
	init_SysTick();
	init_TIMER1();
	init_TIMER2();

	while (1){
		UART_message_parsing();
//		control_loop();
	}
}

#include "main.h"

uint32_t T1CCR = 65;
void deinit_all_GPIO(void)
{
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
	//leds
	GPIO_user_init.PORT_Pin       = (PORT_Pin_0|PORT_Pin_11);
	GPIO_user_init.PORT_OE        = PORT_OE_OUT;
	GPIO_user_init.PORT_PULL_UP   = PORT_PULL_UP_OFF;
	GPIO_user_init.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
	GPIO_user_init.PORT_PD_SHM    = PORT_PD_SHM_OFF;
	GPIO_user_init.PORT_PD        = PORT_PD_DRIVER;
	GPIO_user_init.PORT_GFEN      = PORT_GFEN_OFF;
	GPIO_user_init.PORT_FUNC      = PORT_FUNC_PORT;
	GPIO_user_init.PORT_SPEED     = PORT_SPEED_MAXFAST;
	GPIO_user_init.PORT_MODE      = PORT_MODE_DIGITAL;	
	PORT_Init(MDR_PORTC, &GPIO_user_init);
	
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
	GPIO_user_init.PORT_Pin       = (ADC_COM_PIN | ADC_OBJ_PIN);
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

void init_TIMER(){
	/*���������*/
	deinit_TIMER(MDR_TIMER1);
	
//	/*������������ ���������*/
//	MDR_RST_CLK->PER_CLOCK |= (1 << 14 );
//	MDR_RST_CLK->TIM_CLOCK = 0x01000000; //(1 << 24);
	
	/*��������� �������*/
	MDR_TIMER1->PSG = T1PSG; // ������������ �������
	MDR_TIMER1->ARR = T1MAX; // ��������� ����� (16 ���)
	MDR_TIMER1->CNTRL = TIMER_CNTRL_ARRB_EN; //����������� 
	
	/*��������� �������*/
	MDR_TIMER1->CH1_CNTRL = (6 << TIMER_CH_CNTRL_OCCM_Pos); //������ ������� 6: 1 ���� CNT<CCR
  MDR_TIMER1->CH1_CNTRL1 = (1 << TIMER_CH_CNTRL1_SELOE_Pos )|(3 << TIMER_CH_CNTRL1_SELO_Pos)| //����� ������ ���., 2 �� ����� REF, 3 �� ����� DTG
													 (1 << TIMER_CH_CNTRL1_NSELOE_Pos )|(3 << TIMER_CH_CNTRL1_NSELO_Pos);
	MDR_TIMER1->CH1_DTG = (0 << TIMER_CH_DTGX_Pos)|(0 << TIMER_CH_DTG_EDTS_Pos)|(DEADTIMECONST << TIMER_CH_DTG_Pos); //������������, ������� �� TIM_CLK, �������� ��������
	MDR_TIMER1->CCR1 = T1MAX;	
	
	MDR_TIMER1->CH2_CNTRL = (6 << TIMER_CH_CNTRL_OCCM_Pos);
  MDR_TIMER1->CH2_CNTRL1 = (1 << TIMER_CH_CNTRL1_SELOE_Pos )|(3 << TIMER_CH_CNTRL1_SELO_Pos)| //����� ������ ���., 2 �� ����� REF, 3 �� ����� DTG
													 (1 << TIMER_CH_CNTRL1_NSELOE_Pos )|(3 << TIMER_CH_CNTRL1_NSELO_Pos);
	MDR_TIMER1->CH2_DTG = (0 << TIMER_CH_DTGX_Pos)|(0 << TIMER_CH_DTG_EDTS_Pos)|(DEADTIMECONST << TIMER_CH_DTG_Pos); //������������, ������� �� TIM_CLK, �������� ��������
	MDR_TIMER1->CCR2 = T1MAX;	
	
	
	/*���������� ������*/
//	MDR_TIMER1->IE = 0x00001102;		//���������� �� cnt=arr
//	NVIC_EnableIRQ(Timer1_IRQn); //��������� ���������� �������	
	MDR_TIMER1->CNTRL |= TIMER_CNTRL_CNT_EN; // ���� ����� �� TIM_CLK, � �������� ������.	

}

void init_ADC(void)
{
	ADC_InitTypeDef ADC_InitStruct;
	ADCx_InitTypeDef ADCx_InitStruct;
	
	ADC_DeInit();
  ADC_StructInit(&ADC_InitStruct);
  ADCx_StructInit(&ADCx_InitStruct);
	
	ADCx_InitStruct.ADC_SamplingMode = ADC_SAMPLING_MODE_SINGLE_CONV;/* ����� ������������� �������������� */
	ADCx_InitStruct.ADC_ChannelNumber = ADC_COM_CHANNEL;/* ����� ������ ������ */
	ADCx_InitStruct.ADC_Prescaler = ADC_CLK_div_64; //����� �������� �������� �������
//	ADCx_InitStruct.ADC_DelayGo = 0x7;/* �������� �������� ����� ������� ���������� �������������� */
	
	ADC_Init(&ADC_InitStruct);
	ADC1_Init(&ADCx_InitStruct);
//	ADC1_ITConfig(ADC1_IT_END_OF_CONVERSION, ENABLE);  //��������� ����������
//	NVIC_EnableIRQ(ADC_IRQn); //��������� ����������
	ADC1_Cmd(ENABLE);				//�������� ���				
}

void init_PER(void){		
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTC, ENABLE);		
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTA, ENABLE);	
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTD, ENABLE);	
	RST_CLK_PCLKcmd(RST_CLK_PCLK_ADC, ENABLE);
	
	/*������������ ��������*/
	MDR_RST_CLK->PER_CLOCK |= (1 << 14 );
	MDR_RST_CLK->TIM_CLOCK = 0x01000000; //(1 << 24);
}

/* ������ MAP �� �������� */
uint32_t map_ADC_result(uint32_t data, uint32_t base_min, uint32_t base_max, uint32_t range_min, uint32_t range_max, MAP_INVERT invert){
	uint32_t delta_range = range_max-range_min;
	uint32_t delta_base = base_max - base_min;
	if ((range_min>range_max) || (base_min>base_max)) return 0;
	float k = (float)(data-base_min) / (float)delta_base; // ��������� ��������, float �� �������� ��� ������� �����?
	float t =  k * (float)delta_range;
	if ((uint32_t)t > delta_range) t = (float)delta_range; //�� ������ ������
	return (invert == MAPNONINVERT)? range_min + (uint32_t)t : range_max - (uint32_t)t;
}

void control_loop(void){
	uint16_t COM_angle = get_COM_angle();
	uint16_t OBJ_angle = get_OBJ_angle();
	
	if (COM_angle>OBJ_angle){		
		changePWM(PWMFORWARD, COM_angle-OBJ_angle);		
	}
	if (OBJ_angle>COM_angle){
		changePWM(PWMBACKWARD, OBJ_angle-COM_angle);		
	}
	
}

uint16_t get_COM_angle(void){
	ADC1_SetChannel(ADC_COM_CHANNEL);
	ADC1_Start();
	PORT_SetBits(MDR_PORTC, PORT_Pin_0);
	while (!(MDR_ADC->ADC1_STATUS & ADCx_FLAG_END_OF_CONVERSION));
	PORT_ResetBits(MDR_PORTC, PORT_Pin_0);		
	return filter_analog(ADC1_GetResult()&ADC_MAX, COM);
}

uint16_t get_OBJ_angle(void){
	ADC1_SetChannel(ADC_OBJ_CHANNEL);
	ADC1_Start();
	PORT_SetBits(MDR_PORTC, PORT_Pin_11);
	while (!(MDR_ADC->ADC1_STATUS & ADCx_FLAG_END_OF_CONVERSION));
	PORT_ResetBits(MDR_PORTC, PORT_Pin_11);		
	return filter_analog(ADC1_GetResult()&ADC_MAX, OBJ);
}

uint16_t filter_analog(uint16_t data, SIGNAL_CHANNEL channel){
	static uint16_t filter[FILTER_SIZE][2];
	static uint8_t filter_count = 0;
	uint16_t sum = 0; //������ �������: 4 ���� ������� ���������. ������ ����� �������������� �� 16 ��������
	uint8_t i;
	
	filter[filter_count][channel] = data; 
	filter_count++;
	if (filter_count >= FILTER_SIZE) filter_count = 0;
	for (i = 0; i < FILTER_SIZE; i++) sum += filter[i][channel]; 
	return(sum/FILTER_SIZE);	
}

void changePWM(PWM_DIRECTION direction, uint16_t PWMpower){
	uint32_t mapped_ccr = map_ADC_result(PWMpower, 0, ADC_MAX, T1MIN, T1MAX, MAPINVERT); //�������� �1��� ���� ������� 1 ����� ��� ������� ��� ������
	//��������������
	switch (direction){
		case PWMFORWARD:
		{
			MDR_TIMER1->CCR1 = mapped_ccr;
			MDR_TIMER1->CCR2 = T1MAX; //��������
			break;
		}			
		case PWMBACKWARD:
		{
			MDR_TIMER1->CCR1 = T1MAX; //��������
			MDR_TIMER1->CCR2 = mapped_ccr;

		}
	}
}

int main(){
	init_CPU();
	init_PER();
	init_GPIO();
	init_TIMER();
	init_ADC();
	while (1){	
		control_loop();
	}
}

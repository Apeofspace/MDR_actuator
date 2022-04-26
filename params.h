//-----------------------------------------------------------------------
/*��� ���������� ��������� �������*/
#define USE_BASIC_FILTER
//#define USE_NO_FILTER
//#define USE_DMA_FILTER //don't!!! 
//#define MEASURE_AND_SEND_TOK //������ �������� ������������ � DMA ��������
#define ADC_MASK 0xFFC //��������� ��� ��������� ���� � ��������� ���
#define BASIC_FILTER_SIZE 7UL
#define T1PSG 0
#define T1ARR 3999
#define T2PSG 79
#define T2ARR 249
#define DEADTIMECONST 400  //��� 80��� 1 = 0.125 *10^-7 � (��� DTG �� CPU_CLK) 8 ���. 80 = 1���
#define PWMDEADZONE 0.0025  //���� �������������������
#define PWM_KOEF_USIL 20 //����������� ��������
#define COM_LIMIT_LEFT 1000 //����� �� ������ �� ����
#define COM_LIMIT_RIGHT 3500
#define BUFFER_SIZE 50
#define OWN_ADRESS 0x01
#define TARGET_ADRESS 0x01


//-----------------------------------------------------------------------
/*�����*/
#define ADC_PORT MDR_PORTD 
#define ADC_OBJ_CHANNEL ADC_CH_ADC7 //����� ���, ������������� ������ ���������� XS9 10
#define ADC_TOK_CHANNEL ADC_CH_ADC6

/*Rudder 1 timer 1 channel 1 & 2*/
#define RUD1_PWM_PORT MDR_PORTA //XS10
#define RUD1_CH1_PIN PORT_Pin_1 //PA1 XS10  12
#define RUD1_CH2_PIN PORT_Pin_3 //PA3 XS10  10
#define RUD1_CH1N_PIN PORT_Pin_2 //PA2 
#define RUD1_CH2N_PIN PORT_Pin_4 //PA4 
#define RUD1_PORT_FUNC PORT_FUNC_ALTER
/*Rudder 2 timer 3 channel 3 & 4 */
#define RUD2_PWM_PORT MDR_PORTB //XS10
#define RUD2_CH1_PIN PORT_Pin_5 // XS10  12
#define RUD2_CH2_PIN PORT_Pin_7 //PA3 XS10  10
#define RUD2_CH1N_PIN PORT_Pin_6 //PA2 
#define RUD2_CH2N_PIN PORT_Pin_8 //PA4 
#define RUD2_PORT_FUNC PORT_FUNC_OVERRID
/*Rudder 3 timer 3 channel 1 & 2*/
#define RUD3_PWM_PORT MDR_PORTD //XS10
#define RUD3_CH1_PIN PORT_Pin_0 //PA1 XS10  12
#define RUD3_CH2_PIN PORT_Pin_2 //PA3 XS10  10
#define RUD3_CH1N_PIN PORT_Pin_1 //PA2 
#define RUD3_CH2N_PIN PORT_Pin_3 //PA4 
#define RUD3_PORT_FUNC PORT_FUNC_OVERRID
/*Rudder 4  timer 2 channel 1 & 3*/
#define RUD4_PWM_PORT MDR_PORTE //XS10
#define RUD4_CH1_PIN PORT_Pin_0 //PA1 XS10  12
#define RUD4_CH2_PIN PORT_Pin_2 //PA3 XS10  10
#define RUD4_CH1N_PIN PORT_Pin_1 //PA2 
#define RUD4_CH2N_PIN PORT_Pin_3 //PA4 
#define RUD4_PORT_FUNC PORT_FUNC_ALTER

//-----------------------------------------------------------------------
/*��������� UART*/
#define UART485 MDR_UART2
#define UART485_PORT MDR_PORTF
#define UART485_PINS (PORT_Pin_0 | PORT_Pin_1)
#define UART485_PINS_FUNCTION PORT_FUNC_OVERRID
#define UART485_BAUD_RATE 1250000
//#define RS485_DE_RE_PIN (PORT_Pin_2|PORT_Pin_3)
#define RS485_DE_RE_PIN PORT_Pin_2
#define RS485_DE_RE_PORT MDR_PORTF
//-----------------------------------------------------------------------

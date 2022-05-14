//-----------------------------------------------------------------------
/*��� ���������� ��������� �������*/
////#define USE_BASIC_FILTER
#define USE_NO_FILTER
#define ADC_MASK 0xFFC //��������� ��� ��������� ���� � ��������� ���
#define BASIC_FILTER_SIZE 7UL
#define T1PSG 0
#define T1ARR 3999
#define T2PSG 79
#define T2ARR 249
#define DEADTIMECONST 400  //��� 80��� 1 = 0.125 *10^-7 � (��� DTG �� CPU_CLK) 8 ���. 80 = 1���
#define PWMDEADZONE 0.0025  //���� �������������������
#define PWM_KOEF_USIL 20 //����������� ��������
#define COM_LIMIT_LEFT 800 //����� �� ������ �� ����
#define COM_LIMIT_RIGHT 3800
#define OWN_ADRESS 0x01
#define TARGET_ADRESS 0x00
#define TELEMETRY_DATA_BUFFER_SIZE 13

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

//-----------------------------------------------------------------------
/*��������� UART*/
#define UART485 MDR_UART2
#define UART485_PORT MDR_PORTF
#define UART485_PINS (PORT_Pin_0 | PORT_Pin_1)
#define UART485_PINS_FUNCTION PORT_FUNC_OVERRID
#define RS485_DE_RO_PIN PORT_Pin_2
#define RS485_DE_RO_PORT MDR_PORTF
#define UART485_BAUD_RATE 230400
#define UART_TIMEOUT 1200
#define BUFFER_SIZE 50
//-----------------------------------------------------------------------

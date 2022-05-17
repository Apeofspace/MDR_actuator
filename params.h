//-----------------------------------------------------------------------
/*Все изменяемые параметры системы*/
//#define USE_PROTOCOL
//#define USE_BASIC_FILTER
#define USE_NO_FILTER
#define ADC_MASK 0xFFE //Отбросить последние биты с показаний АЦП
#define BASIC_FILTER_SIZE 7UL
#define FILTER_NUMBER_OF_CHANNELS 1
#define T1PSG 0
#define T1ARR 3999
#define T2PSG 79
#define T2ARR 249
#define DEADTIMECONST 400  //При 80мГц 1 = 0.125 *10^-7 с (при DTG от CPU_CLK) 8 бит. 80 = 1мкс
#define PWM_DEAD_ZONE 0.0025  //зона нечувстсвительности
#define PWM_KOEF_USIL 20 //коэффициент усиления
#define COM_LIMIT_LEFT 800 //чтобы не билось об края
#define COM_LIMIT_RIGHT 3800
#if defined(USE_PROTOCOL)
#define OWN_ADRESS 0x01
#define TARGET_ADRESS 0x00
#define TELEMETRY_DATA_BUFFER_SIZE 13
#else
#define TELEMETRY_DATA_BUFFER_SIZE 8
#endif

//-----------------------------------------------------------------------
/*Порты*/
#define ADC_PORT MDR_PORTD 
#define ADC_OBJ_CHANNEL ADC_CH_ADC7 //канал АЦП, отслеживающий объект управления XS9 10
#define ADC_TOK_CHANNEL ADC_CH_ADC6

/*Rudder 1 timer 1 channel 1 & 2*/
#define RUD1_PWM_PORT MDR_PORTA //XS10
#define RUD1_CH1_PIN PORT_Pin_1 //PA1 XS10  12
#define RUD1_CH2_PIN PORT_Pin_3 //PA3 XS10  10
#define RUD1_CH1N_PIN PORT_Pin_2 //PA2 
#define RUD1_CH2N_PIN PORT_Pin_4 //PA4 
#define RUD1_PORT_FUNC PORT_FUNC_ALTER

//-----------------------------------------------------------------------
/*Параметры UART*/
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

#include "MDR32Fx.h"
#include "MDR32F9Qx_rst_clk.h"          // Keil::Drivers:RST_CLK
#include "MDR32F9Qx_port.h"             // Keil::Drivers:PORT
#include "MDR32F9Qx_config.h"           // Keil::Device:Startup
#include "MDR32F9Qx_timer.h"            // Keil::Drivers:TIMER
#include "MDR32F9Qx_adc.h"
#include "MDR32F9Qx_dma.h"
#include "MDR32F9Qx_uart.h"             // Keil::Drivers:UART

#include "params.h"
//-----------------------------------------------------------------------
/*Переменные*/
extern uint16_t com_angle;
extern uint32_t T1CCR;
extern uint8_t telemetry_to_send[TELEMETRY_DATA_BUFFER_SIZE];
typedef enum {MAPINVERT = 1, MAPNONINVERT = 0} MAP_INVERT;
typedef enum {PWMFORWARD = 1, PWMBACKWARD = 0} PWM_DIRECTION;
typedef enum {COM = 1, OBJ = 0} SIGNAL_CHANNEL;

typedef enum{MODE_ADRESS = UART_Parity_1, MODE_DATA = UART_Parity_0} Protocol_parity_mode_type;
typedef enum{MODE_RECIEVE, MODE_SEND} Protocol_mode_type;
extern Protocol_parity_mode_type PROTOCOL_CURRENT_PARITY_MODE; 
extern Protocol_mode_type PROTOCOL_CURRENT_MODE; 

extern uint8_t UART_recieved_data_buffer[BUFFER_SIZE];
extern uint32_t UART_recieved_data_length;

extern DMA_ChannelInitTypeDef DMA_InitStr_TX;
extern DMA_CtrlDataInitTypeDef DMA_PriCtrlStr_TX;

//-----------------------------------------------------------------------
/*Флаги*/
extern uint8_t uart_busy_flag;
extern uint8_t recieving_data_flag; //this flag is pointless, replace with UART_recieved_data_length
extern uint8_t uart_package_recieved_flag; 
//-----------------------------------------------------------------------
/*Инициализация*/
void init_CPU(void);
void init_USB(void);
void init_GPIO(void);
void init_TIMER1(void);
void init_TIMER2(void);
void deinit_all_GPIO(void);
void deinit_TIMER(MDR_TIMER_TypeDef *Timer);
void init_ADC(void);
void init_UART(void);
void init_DMA(void);
void USART_TX_DMA_ini(uint8_t* SourceBuffer, uint8_t Length);
//-----------------------------------------------------------------------
/*Функции*/
uint32_t map_PWM(uint32_t data, uint32_t base_min, uint32_t base_max, uint32_t range_min, uint32_t range_max, uint8_t koef_usil, float dead_zone, MAP_INVERT invert);
void changePWM(PWM_DIRECTION direction, uint32_t mapped_ccr);
void main_loop(void);
uint16_t get_OBJ_angle(void);
uint16_t get_COM_angle(void);
uint16_t filter_basic(uint16_t data, SIGNAL_CHANNEL channel);
void SEND_DATA_UART_DMA(uint8_t* data_buffer, uint8_t length);
void UART_message_parsing(void);
//-----------------------------------------------------------------------
/*Функции, связанные с протоколом*/
unsigned short CRC1(unsigned char * A, unsigned char * N);
unsigned short CRC2(unsigned char * pcBlock, unsigned short len);
void Protocol_change_mode(Protocol_mode_type mode);
void Protocol_change_parity_mode(Protocol_parity_mode_type mode);
int Protocol_check_adress(uint8_t* adress);
int Protocol_check_parity(uint16_t* recieved_byte);
void Protocol_UART_message_recieved_callback(uint8_t* Buffer); 
void Protocol_recieve_message(void);
void Protocol_send_message(uint8_t length);
//-----------------------------------------------------------------------
/*Прерывания*/
void Timer2_IRQHandler(void);
void DMA_IRQHandler(void);

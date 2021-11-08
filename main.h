#include "MDR32Fx.h"
#include "MDR32F9Qx_rst_clk.h"          // Keil::Drivers:RST_CLK
#include "MDR32F9Qx_port.h"             // Keil::Drivers:PORT
#include "MDR32F9Qx_config.h"           // Keil::Device:Startup
#include "MDR32F9Qx_timer.h"            // Keil::Drivers:TIMER
#include "MDR32F9Qx_adc.h"
#include "ports.h"
#include "MDR32F9Qx_usb_handlers.h"
#include "MDR32F9Qx_usb_CDC.h"
#include "MDR32F9Qx_dma.h"

/*��� ���������� ���������*/
#define USE_DMA_FILTER
#define USE_BASIC_FILTER
//#define USE_NO_FILTER
#define ADC_MASK 0xFFC //��������� ��� ��������� ���� � ��������� ���
#define DMA_FILTER_SIZE 30UL
#define BASIC_FILTER_SIZE 7UL
#define T1PSG 0
#define T1ARR 3999
#define T2PSG 79
#define T2ARR 999
#define DEADTIMECONST 80  //��� 80��� 1 = 0.125 *10^-7 � (��� DTG �� CPU_CLK) 8 ���. 80 = 1���
#define PWMDEADZONE 0.0015  //���� �������������������
#define PWM_SATURATION_COEFFICIENT 12 //����������� ��������� ������������ ����������
#define COM_LIMIT_LEFT 0x100 //����� �� �������������� ����� ����
#define COM_LIMIT_RIGHT 0xEFF

/*����������*/
#define USB_DATA_BUFFER_SIZE 7
extern uint32_t dmaCtrlStart;
extern uint32_t completedIRQ;
extern uint16_t com_angle;
extern uint32_t T1CCR;
extern uint64_t timestamp_command_recieved, timestamp_obj_recieved; //�����, ����� ���� �������� �������, � �����, ����� ���� ���������� �������
extern uint8_t timestamp_overflow_counter;
extern volatile uint32_t data_to_send[USB_DATA_BUFFER_SIZE];
typedef enum {MAPINVERT = 1, MAPNONINVERT = 0} MAP_INVERT;
typedef enum {PWMFORWARD = 1, PWMBACKWARD = 0} PWM_DIRECTION;
typedef enum {COM = 1, OBJ = 0} SIGNAL_CHANNEL;
extern DMA_CtrlDataInitTypeDef DMA_DataCtrl_Pri;
extern DMA_ChannelInitTypeDef DMA_ChanCtrl;
extern uint16_t data_dma[DMA_FILTER_SIZE];
extern DMA_CtrlDataTypeDef DMA_ControlTable[DMA_Channels_Number * (1 + DMA_AlternateData)];

/*Inits*/
void init_CPU(void);
void init_USB(void);
void init_PER(void);
void init_GPIO(void);
void init_TIMER1(void);
void init_TIMER2(void);
void deinit_all_GPIO(void);
void deinit_TIMER(MDR_TIMER_TypeDef *Timer);
void init_ADC(void);
void init_SysTick(void);
void init_DMA(void);
void init_LED(void);


/*Functions*/
uint32_t map_PWM(uint32_t data, uint32_t base_min, uint32_t base_max, uint32_t range_min, uint32_t range_max, uint8_t saturation_coef, MAP_INVERT invert);
void changePWM(PWM_DIRECTION direction, uint32_t mapped_ccr);
void control_loop(void);
uint16_t get_OBJ_angle(void);
uint16_t get_COM_angle(void);
uint16_t filter_analog(uint16_t data, SIGNAL_CHANNEL channel);
void take_timestamp(uint64_t* timestamp);
void reload_SysTick(void);
void send_data(uint32_t Length);
void BRD_ADC1_RunSingle(uint32_t goEna);
void BRD_ADC1_RunSample(uint32_t sampleEna);

/*IRQs*/
void Timer2_IRQHandler(void);
void SysTick_Handler(void);
void DMA_IRQHandler(void);
	

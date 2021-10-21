#include "MDR32Fx.h"
#include "MDR32F9Qx_rst_clk.h"          // Keil::Drivers:RST_CLK
#include "MDR32F9Qx_port.h"             // Keil::Drivers:PORT
#include "MDR32F9Qx_config.h"           // Keil::Device:Startup
#include "MDR32F9Qx_timer.h"            // Keil::Drivers:TIMER
#include "MDR32F9Qx_adc.h"
#include "ports.h"
#include "MDR32F9Qx_usb_handlers.h"
#include "MDR32F9Qx_usb_CDC.h"

/*Все изменяемые параметры*/
#define T1PSG 79
#define T1ARR 99
#define T2PSG 79
#define T2ARR 999
#define DEADTIMECONST 80  //При 80мГц 1 = 0.125 *10^-7 с (при DTG от CPU_CLK) 8 бит. 80 = 1мкс
#define ADC_MASK 0xFFC //Отбросить два последних бита с показаний АЦП
#define FILTER_SIZE 5UL
#define PWMDEADZONE 0.0015  //зона нечувстсвительности
#define PWM_SATURATION_COEFFICIENT 12 //коэффициент умножения коэффициента заполнения
#define COM_LIMIT_LEFT 0x100 //чтобы не перекатывалось через ноль
#define COM_LIMIT_RIGHT 0xEFF

///*Макрос*/
//#define SysTick_to_US(SysTick) ((double) SysTick *  1000000U/SystemCoreClock) 	

extern uint16_t com_angle;
extern uint32_t T1CCR;
extern uint64_t timestamp_command_recieved, timestamp_obj_recieved; //время, когда было получена команда, и время, когда была отработана команда
extern uint8_t timestamp_overflow_counter;
extern volatile uint32_t data_to_send[5];
typedef enum {MAPINVERT = 1, MAPNONINVERT = 0} MAP_INVERT;
typedef enum {PWMFORWARD = 1, PWMBACKWARD = 0} PWM_DIRECTION;
typedef enum {COM = 1, OBJ = 0} SIGNAL_CHANNEL;

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


/*Functions*/
uint32_t map_PWM(uint32_t data, uint32_t base_min, uint32_t base_max, uint32_t range_min, uint32_t range_max, uint8_t saturation_coef, MAP_INVERT invert);
void changePWM(PWM_DIRECTION direction, uint16_t PWMpower);
void control_loop(void);
uint16_t get_OBJ_angle(void);
uint16_t get_COM_angle(void);
uint16_t filter_analog(uint16_t data, SIGNAL_CHANNEL channel);
void take_timestamp(uint64_t* timestamp);
void reload_SysTick(void);
void send_data(void);

/*IRQs*/
void Timer2_IRQHandler(void);
void SysTick_Handler(void);

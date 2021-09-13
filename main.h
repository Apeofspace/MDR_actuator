#include "MDR32Fx.h"
#include "MDR32F9Qx_rst_clk.h"          // Keil::Drivers:RST_CLK
#include "MDR32F9Qx_port.h"             // Keil::Drivers:PORT
#include "MDR32F9Qx_config.h"           // Keil::Device:Startup
#include "MDR32F9Qx_timer.h"            // Keil::Drivers:TIMER
#include "MDR32F9Qx_adc.h"
#include "ports.h"

#define T1PSG 79
#define T1MIN 0
#define T1MAX 99
#define DEADTIMECONST 8 //��� 80��� 1 = 0.125 *10^-6 � (��� DTG �� CPU_CLK) 8 ���
#define ADC_MAX 0xfff
#define FILTER_SIZE 5UL

extern uint32_t T1CCR;
typedef enum {MAPINVERT = 1, MAPNONINVERT = 0} MAP_INVERT;
typedef enum {PWMFORWARD = 1, PWMBACKWARD = 0} PWM_DIRECTION;
typedef enum {COM = 1, OBJ = 0} SIGNAL_CHANNEL;

void init_CPU(void);
void init_PER(void);
void init_GPIO(void);
void init_TIMER(void);
void deinit_all_GPIO(void);
void deinit_TIMER(MDR_TIMER_TypeDef *Timer);
void init_ADC(void);

uint32_t map_ADC_result(uint32_t data, uint32_t base_min, uint32_t base_max, uint32_t range_min, uint32_t range_max, MAP_INVERT invert);
void changePWM(PWM_DIRECTION direction, uint16_t PWMpower);
void control_loop(void);
uint16_t get_OBJ_angle(void);
uint16_t get_COM_angle(void);
uint16_t filter_analog(uint16_t data, SIGNAL_CHANNEL channel);

void Timer1_IRQHandler(void);
void ADC_IRQHandler(void);

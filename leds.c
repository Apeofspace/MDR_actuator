#include "main.h"

void flip_LED(MDR_PORT_TypeDef* MDR_PORTx, PORT_Pin_TypeDef PORT_Pin){
	if ((MDR_PORTx->RXTX)&PORT_Pin) MDR_PORTx->RXTX &= ~(PORT_Pin);
	else MDR_PORTx->RXTX |=(PORT_Pin);
}

void init_debug_LED(){
	// нркюднвмши дхнд опнярю
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTE, ENABLE);
	PORT_InitTypeDef GPIOInitStruct;		
	GPIOInitStruct.PORT_OE        = PORT_OE_OUT;
	GPIOInitStruct.PORT_PULL_UP   = PORT_PULL_UP_OFF;
	GPIOInitStruct.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
	GPIOInitStruct.PORT_PD_SHM    = PORT_PD_SHM_OFF;
	GPIOInitStruct.PORT_PD        = PORT_PD_DRIVER;
	GPIOInitStruct.PORT_GFEN      = PORT_GFEN_OFF;
	GPIOInitStruct.PORT_FUNC      = PORT_FUNC_PORT;
	GPIOInitStruct.PORT_SPEED     = PORT_SPEED_MAXFAST;
	GPIOInitStruct.PORT_MODE      = PORT_MODE_DIGITAL;	
  GPIOInitStruct.PORT_Pin   = PORT_Pin_7;
  PORT_Init (MDR_PORTE, &GPIOInitStruct);	
}

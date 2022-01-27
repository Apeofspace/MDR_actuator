#include "main.h"

//DMA_CtrlDataTypeDef DMA_ControlTable[DMA_Channels_Number * (1 + DMA_AlternateData)];

// Настройки управления данными DMA
DMA_CtrlDataInitTypeDef DMA_DataCtrl_Pri = 
{
  0,                            // DMA_SourceBaseAddr - Адрес источника данных
  0,                            // DMA_DestBaseAddr   - Адрес назначения данных
  DMA_SourceIncNo,              // DMA_SourceIncSize  - Автоувеличение адреса источника данных
  DMA_DestIncHalfword,              // DMA_DestIncSize    - Автоувеличение адреса назначения данных
  DMA_MemoryDataSize_HalfWord,      // DMA_MemoryDataSize - Размер пакета данных
  DMA_Mode_Basic,               // DMA_Mode           - Режим работы DMA
  1,                            // DMA_CycleSize      - Кол. данных на передачу (длина цикла DMA)
  DMA_Transfers_1,              // DMA_NumContinuous  - Количество непрерывных передач (до арбитража)
  DMA_SourcePrivileged,         // DMA_SourceProtCtrl - Режим защиты передатчика
  DMA_DestPrivileged            // DMA_DestProtCtrl   - Режим защиты приемника
};
  
//  Настройки канала DMA
DMA_ChannelInitTypeDef DMA_ChanCtrl = 
{
  &DMA_DataCtrl_Pri,        // DMA_PriCtrlData         - Основная структура управления данными
  &DMA_DataCtrl_Pri,        // DMA_AltCtrlStr          - Альтернативная структура управления данными
   DMA_AHB_Privileged,      // DMA_ProtCtrl 
   DMA_Priority_High,       // DMA_Priority            - Приоритет канала
   DMA_BurstClear,          // DMA_UseBurst
   DMA_CTRL_DATA_PRIMARY    // DMA_SelectDataStructure - Используемая структура управления данными
};


void BRD_ADC1_RunSingle(uint32_t goEna)
	//same as ADC1_Start();
{
  if (goEna)
    MDR_ADC->ADC1_CFG |= ADC1_CFG_REG_GO;
  else
    MDR_ADC->ADC1_CFG &= ~ADC1_CFG_REG_GO;
}

void BRD_ADC1_RunSample(uint32_t sampleEna)
	//starts continuous conversion
{
  if (sampleEna)
    MDR_ADC->ADC1_CFG |= ADC1_CFG_REG_SAMPLE;
  else
    MDR_ADC->ADC1_CFG &= ~ADC1_CFG_REG_SAMPLE;
}


void init_DMA(){
	init_LED();
	// Включение тактирования модуля DMA
  RST_CLK_PCLKcmd (RST_CLK_PCLK_SSP1 | RST_CLK_PCLK_SSP2 | RST_CLK_PCLK_DMA, ENABLE);
	// Деинициализация DMA
  DMA_DeInit();	
	// Сброс прерывания от DMA
  NVIC_ClearPendingIRQ (DMA_IRQn);  

	
  DMA_DataCtrl_Pri.DMA_SourceBaseAddr = (uint32_t)&MDR_ADC->ADC1_RESULT;
  DMA_DataCtrl_Pri.DMA_DestBaseAddr   = (uint32_t)&data_dma;
  DMA_DataCtrl_Pri.DMA_CycleSize      = DMA_FILTER_SIZE;
	DMA_Init(DMA_Channel_ADC1, &DMA_ChanCtrl);
  DMA_Cmd(DMA_Channel_ADC1, ENABLE);
	
  NVIC_SetPriority (DMA_IRQn, 1);
  NVIC_ClearPendingIRQ (DMA_IRQn);  
  NVIC_EnableIRQ (DMA_IRQn); 
	
  //  for restart DMA Cycle
  dmaCtrlStart = DMA_ControlTable[DMA_Channel_ADC1].DMA_Control;
}

void init_LED(){
	PORT_InitTypeDef GPIO_user_init;

	GPIO_user_init.PORT_Pin       = (PORT_Pin_1|PORT_Pin_0);
	GPIO_user_init.PORT_OE        = PORT_OE_OUT;
	GPIO_user_init.PORT_PULL_UP   = PORT_PULL_UP_OFF;
	GPIO_user_init.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
	GPIO_user_init.PORT_PD_SHM    = PORT_PD_SHM_OFF;
	GPIO_user_init.PORT_PD        = PORT_PD_DRIVER;
	GPIO_user_init.PORT_GFEN      = PORT_GFEN_OFF;
	GPIO_user_init.PORT_FUNC      = PORT_FUNC_PORT;
	GPIO_user_init.PORT_SPEED     = PORT_SPEED_MAXFAST;
	GPIO_user_init.PORT_MODE      = PORT_MODE_DIGITAL;	
	
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTC, ENABLE);
	PORT_Init(MDR_PORTC, &GPIO_user_init);
}


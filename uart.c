#include "main.h"


Protocol_parity_mode_type PROTOCOL_CURRENT_PARITY_MODE = MODE_ADRESS;
Protocol_mode_type PROTOCOL_CURRENT_MODE = MODE_RECIEVE;
	
uint8_t UART_recieved_data_buffer[BUFFER_SIZE];
uint32_t UART_recieved_data_length = 0;
//-----------------------------------------------------------------------
	DMA_CtrlDataInitTypeDef DMA_PriCtrlStr_TX =
{
	0,
	0,
	DMA_SourceIncByte,
	DMA_DestIncNo,
	DMA_MemoryDataSize_Byte,
	DMA_Mode_Basic,
	1,
	DMA_Transfers_1,
	DMA_SourcePrivileged,
	DMA_DestPrivileged
};

	DMA_ChannelInitTypeDef DMA_InitStr_TX =
{
	&DMA_PriCtrlStr_TX,
	&DMA_PriCtrlStr_TX,
	DMA_AHB_Privileged,
	DMA_Priority_High, 
	DMA_BurstClear, 
	DMA_CTRL_DATA_PRIMARY
};
//-----------------------------------------------------------------------	
void init_UART(){
	UART_InitTypeDef UART_InitStruct;
	PORT_InitTypeDef GPIOInitStruct;
	
	 // Включение тактирования
	RST_CLK_PCLKcmd(RST_CLK_PCLK_UART2, ENABLE);
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTF, ENABLE);
	
	//инициализация структуры
	GPIOInitStruct.PORT_PULL_UP   = PORT_PULL_UP_OFF;
	GPIOInitStruct.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
	GPIOInitStruct.PORT_PD_SHM    = PORT_PD_SHM_OFF;
	GPIOInitStruct.PORT_PD        = PORT_PD_DRIVER;
	GPIOInitStruct.PORT_GFEN      = PORT_GFEN_OFF;
	GPIOInitStruct.PORT_SPEED     = PORT_SPEED_MAXFAST;
	GPIOInitStruct.PORT_MODE      = PORT_MODE_DIGITAL;	
		
	 // Конфигурация и инициализация линии для приема данных 
	GPIOInitStruct.PORT_FUNC  = UART485_PINS_FUNCTION;
  GPIOInitStruct.PORT_Pin   = UART485_PINS;
  PORT_Init (UART485_PORT, &GPIOInitStruct);

  // Конфигурация и инициализация линии для передачи данных 
  GPIOInitStruct.PORT_FUNC  = PORT_FUNC_OVERRID;	
  GPIOInitStruct.PORT_OE    = PORT_OE_OUT;
  GPIOInitStruct.PORT_Pin   = PORT_Pin_1;
  PORT_Init (UART485_PORT, &GPIOInitStruct);
	
	// Конфигурация и инициализация ноги для дрыганья режимом прием/передача
  GPIOInitStruct.PORT_FUNC  = PORT_FUNC_PORT;	
  GPIOInitStruct.PORT_OE    = PORT_OE_OUT;
  GPIOInitStruct.PORT_Pin   = RS485_DE_RO_PIN;
  PORT_Init (RS485_DE_RO_PORT, &GPIOInitStruct);
	
	 // Создание структуры UART_StructInit c дефолтными параметрами
	UART_StructInit(&UART_InitStruct);
	UART_InitStruct.UART_WordLength = UART_WordLength8b;
	#if defined(USE_PROTOCOL)
	UART_InitStruct.UART_Parity = UART_Parity_1;
	#else
	UART_InitStruct.UART_Parity = UART_Parity_No;
	#endif
	UART_InitStruct.UART_BaudRate = UART485_BAUD_RATE;
	
	 // Инициализация модуля UART
	UART_Init(UART485, &UART_InitStruct);
	
	 // Выбор предделителя тактовой частоты модуля UART
  UART_BRGInit (UART485, UART_HCLKdiv1);
	
	// Выбор источников прерываний (прием и передача данных)
//  UART_ITConfig (UART485, UART_IT_RX, ENABLE);
//	NVIC_EnableIRQ(UART2_IRQn);

  // Разрешение работы модуля UART
  UART_Cmd (UART485, ENABLE);
}

//-----------------------------------------------------------------------
void init_DMA(void)
	{
	/* Костыли для обхода ошибок в DMA миландра */
  RST_CLK_PCLKcmd (RST_CLK_PCLK_SSP1 | RST_CLK_PCLK_SSP2 | RST_CLK_PCLK_DMA, ENABLE);// Включение тактирования модуля DMA
  DMA_DeInit();	// Деинициализация DMA
  NVIC_ClearPendingIRQ (DMA_IRQn);  // Сброс прерывания от DMA
}
//-----------------------------------------------------------------------

void USART_TX_DMA_ini(uint8_t* SourceBuffer, uint8_t Length)
{
	DMA_PriCtrlStr_TX.DMA_SourceBaseAddr 	= (uint32_t) SourceBuffer;
	DMA_PriCtrlStr_TX.DMA_DestBaseAddr 		= (uint32_t) (&(MDR_UART2->DR));
	DMA_PriCtrlStr_TX.DMA_CycleSize = Length;
	
	UART_DMACmd(MDR_UART2, UART_DMA_TXE, DISABLE);
	DMA_Cmd(DMA_Channel_UART2_TX, DISABLE);
	
	DMA_Init(DMA_Channel_UART2_TX, &DMA_InitStr_TX);
	
	UART_DMACmd(MDR_UART2, UART_DMA_TXE, ENABLE);
	DMA_Cmd(DMA_Channel_UART2_TX, ENABLE);
	
	NVIC_EnableIRQ(DMA_IRQn);
}
//-----------------------------------------------------------------------
void DMA_IRQHandler(void)
{	
	UART_DMACmd(UART485, UART_DMA_TXE, DISABLE);
	DMA_Cmd(DMA_Channel_UART2_TX, DISABLE);
	while(MDR_UART2->FR & UART_FR_BUSY);
	#if defined(USE_PROTOCOL)
	Protocol_change_mode(MODE_RECIEVE);
	#else
	PORT_ResetBits(RS485_DE_RO_PORT, RS485_DE_RO_PIN);
	#endif
	uart_busy_flag = RESET;
}
//-----------------------------------------------------------------------
void SEND_DATA_UART_DMA(uint8_t* data_buffer, uint8_t length)
{
	#if !defined(USE_PROTOCOL)
	PORT_SetBits(RS485_DE_RO_PORT, RS485_DE_RO_PIN); 
	#endif
	
	USART_TX_DMA_ini(data_buffer, length);
}
//-----------------------------------------------------------------------
#if defined(USE_PROTOCOL)
void Protocol_send_message(uint8_t* data_buffer, uint8_t length)
{	
	uint8_t extended_data_buffer[TELEMETRY_DATA_BUFFER_SIZE];
	Protocol_change_mode(MODE_SEND);
	uart_busy_flag = SET;
	extended_data_buffer[0] = TARGET_ADRESS;
	UART_SendData(MDR_UART2, extended_data_buffer[0]);//отправка адреса без ДМА, т.к. всего 1 байт
	while (UART_GetFlagStatus (MDR_UART2, UART_FLAG_BUSY)== SET)
	
	extended_data_buffer[1] = length + 3;
	
	for (int i = 2; i<length+2; i++)
	{
		extended_data_buffer[i] = *data_buffer;
		data_buffer++;
	}
	
	uint16_t crc_2 = CRC2((uint8_t*)telemetry_to_send, telemetry_to_send[1]-1);
	extended_data_buffer[length+1] = crc_2;
	extended_data_buffer[length+2] = crc_2>>8; 
	SEND_DATA_UART_DMA(extended_data_buffer, extended_data_buffer[1]);
}

//-----------------------------------------------------------------------
void Protocol_recieve_message()
{
	uint16_t recieved_byte;
	recieved_byte = UART_ReceiveData(MDR_UART2);
		UART_recieved_data_buffer[UART_recieved_data_length++] = (uint8_t)recieved_byte;	
		
		/* Проверка четности принятого бита*/
		if (Protocol_check_parity(&recieved_byte)==0){
			Protocol_change_parity_mode(MODE_ADRESS);
			return ;
		}
		
			/* Проверка адреса */
			if (PROTOCOL_CURRENT_PARITY_MODE == MODE_ADRESS){
				if (Protocol_check_adress(&recieved_byte)){
					Protocol_change_parity_mode(MODE_DATA);	//начать приём данных
				}
				else{
					Protocol_change_parity_mode(MODE_ADRESS); // обнулить	
					return ;
				}
			}
							
			/* Приём данных */
			if (PROTOCOL_CURRENT_PARITY_MODE == MODE_DATA){

				if ((UART_recieved_data_length == (UART_recieved_data_buffer[1] + 1))&&UART_recieved_data_length>3){ //Принята контрольная сумма CRC2
					/* Приняты все данные, проверка контрольной суммы */
					uint16_t CRC_recieved = ((uint16_t)(UART_recieved_data_buffer[UART_recieved_data_buffer[1]-1])<<8) | (uint16_t)(UART_recieved_data_buffer[UART_recieved_data_buffer[1]]);
					uint16_t CRC_actual = CRC2((uint16_t*)UART_recieved_data_buffer, UART_recieved_data_buffer[1]-1);
					if (CRC_actual == CRC_recieved){
						/* СООБЩЕНИЕ УСПЕШНО ПРИНЯТО */
						Protocol_change_parity_mode(MODE_ADRESS); // обнулить	
						Protocol_UART_message_recieved_callback(&UART_recieved_data_buffer);
						return;
					}
					else{ // не прошло crc2 проверку
						Protocol_change_parity_mode(MODE_ADRESS); // обнулить	
						return ;
					}
				}			
			}
}
//-----------------------------------------------------------------------
void Protocol_change_parity_mode(Protocol_parity_mode_type mode){ 
	/*Эта функция меняет четность УАРТа. 
	Бит четности должен быть всегда 1 при приеме адреса,
	и всегда 0 при приеме остальной части сообщения, если адресс соответсвует
	адресу данного устройства*/
//	  UART_Parity_No   = ((uint16_t )0x00),
//    UART_Parity_Even = ((uint16_t)0x06),
//    UART_Parity_Odd  = ((uint16_t)0x02),
//    UART_Parity_1    = ((uint16_t)0x82),
//    UART_Parity_0    = ((uint16_t)0x86)
	MDR_UART2->LCR_H &= ~(0x86UL); //обнулить значащие биты
	MDR_UART2->LCR_H |=(mode);
	
	if (PROTOCOL_CURRENT_MODE == MODE_RECIEVE){
		if(mode == MODE_ADRESS){
			// обнуляемся и готовимся к новому сообщению
			UART_recieved_data_length = 0;
			for (uint32_t i = 0; i<(sizeof(UART_recieved_data_buffer)/sizeof(uint8_t)); i++){UART_recieved_data_buffer[i] = 0;}
		}	
	PROTOCOL_CURRENT_PARITY_MODE = mode;
	}
}

//-----------------------------------------------------------------------
void Protocol_change_mode(Protocol_mode_type mode){
	if (mode == MODE_SEND) {
		PORT_SetBits(RS485_DE_RO_PORT, RS485_DE_RO_PIN);
		Protocol_change_parity_mode(MODE_ADRESS);
	}
	else {
		PORT_ResetBits(RS485_DE_RO_PORT, RS485_DE_RO_PIN);
		Protocol_change_parity_mode(MODE_ADRESS);
	}
	PROTOCOL_CURRENT_MODE = mode;
}

//-----------------------------------------------------------------------
int Protocol_check_adress(uint8_t* adress){
	/* 1 если хорошо, 0 если плохо*/
	if (*adress == OWN_ADRESS) return 1;
	else return 0;
}
//-----------------------------------------------------------------------
int Protocol_check_parity(uint16_t* recieved_byte){
	/* 1 если хорошо, 0 если плохо*/
		if ((*recieved_byte >> 9) & 0x01) return 0; // есть ошибка четности
			else return 1;
}

//-----------------------------------------------------------------------
void Protocol_UART_message_recieved_callback(uint8_t* Buffer){
	uint16_t res;
//	little endian
	res = UART_recieved_data_buffer[3];
	res = (res<<8)|UART_recieved_data_buffer[2];
	com_angle = res;
	UART_recieved_data_length=0;
	recieving_data_flag = RESET;
	uart_package_recieved_flag = SET;
}
#endif
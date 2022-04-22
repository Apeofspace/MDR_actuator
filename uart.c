#include "main.h"


uint32_t RESET_DE_RO_KOSTIL_FLAG = RESET;
//uint32_t UART_RECIEVE_IN_PROGRESS_FLAG = RESET;

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
	
	 // ��������� ������������
	RST_CLK_PCLKcmd(RST_CLK_PCLK_UART2, ENABLE);
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTF, ENABLE);
	
	//������������� ���������
	GPIOInitStruct.PORT_PULL_UP   = PORT_PULL_UP_OFF;
	GPIOInitStruct.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
	GPIOInitStruct.PORT_PD_SHM    = PORT_PD_SHM_OFF;
	GPIOInitStruct.PORT_PD        = PORT_PD_DRIVER;
	GPIOInitStruct.PORT_GFEN      = PORT_GFEN_OFF;
	GPIOInitStruct.PORT_SPEED     = PORT_SPEED_MAXFAST;
	GPIOInitStruct.PORT_MODE      = PORT_MODE_DIGITAL;	
		
	 // ������������ � ������������� ����� ��� ������ ������ 
	GPIOInitStruct.PORT_FUNC  = UART485_PINS_FUNCTION;
  GPIOInitStruct.PORT_Pin   = UART485_PINS;
  PORT_Init (UART485_PORT, &GPIOInitStruct);

  // ������������ � ������������� ����� ��� �������� ������ 
  GPIOInitStruct.PORT_FUNC  = PORT_FUNC_OVERRID;	
  GPIOInitStruct.PORT_OE    = PORT_OE_OUT;
  GPIOInitStruct.PORT_Pin   = PORT_Pin_1;
  PORT_Init (UART485_PORT, &GPIOInitStruct);
	
	// ������������ � ������������� ���� ��� �������� ������� �����/��������
  GPIOInitStruct.PORT_FUNC  = PORT_FUNC_PORT;	
  GPIOInitStruct.PORT_OE    = PORT_OE_OUT;
  GPIOInitStruct.PORT_Pin   = RS485_DE_RE_PIN;
  PORT_Init (RS485_DE_RE_PORT, &GPIOInitStruct);
	
	 // �������� ��������� UART_StructInit c ���������� �����������
	UART_StructInit(&UART_InitStruct);
	UART_InitStruct.UART_WordLength = UART_WordLength8b;
	UART_InitStruct.UART_Parity = UART_Parity_No;
	UART_InitStruct.UART_BaudRate = UART485_BAUD_RATE;
	
	 // ������������� ������ UART
	UART_Init(UART485, &UART_InitStruct);
	
	 // ����� ������������ �������� ������� ������ UART
  UART_BRGInit (UART485, UART_HCLKdiv1);
	
	// ����� ���������� ���������� (����� � �������� ������)
  UART_ITConfig (UART485, UART_IT_RX, ENABLE);
	NVIC_EnableIRQ(UART2_IRQn);

  // ���������� ������ ������ UART
  UART_Cmd (UART485, ENABLE);
}

//-----------------------------------------------------------------------
void DMA_common_ini(void)
	{
	/* ������� ��� ������ ������ � DMA �������� */
  RST_CLK_PCLKcmd (RST_CLK_PCLK_SSP1 | RST_CLK_PCLK_SSP2 | RST_CLK_PCLK_DMA, ENABLE);// ��������� ������������ ������ DMA
  DMA_DeInit();	// ��������������� DMA
  NVIC_ClearPendingIRQ (DMA_IRQn);  // ����� ���������� �� DMA
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
	
	if (RESET_DE_RO_KOSTIL_FLAG)	//���� ���� ������������� �� ����� ������
	{
		Protocol_change_mode(MODE_RECIEVE);
	}
}
//-----------------------------------------------------------------------

void SEND_DATA_UART_DMA(uint8_t* data_buffer, uint8_t length)
{
	Protocol_change_mode(MODE_SEND);	
	USART_TX_DMA_ini(data_buffer, length);
}
//-----------------------------------------------------------------------

void UART2_IRQHandler()
{	
	uint16_t recieved_byte;
	if (UART_GetITStatusMasked(MDR_UART2, UART_IT_RX) == SET){

		recieved_byte = UART_ReceiveData(MDR_UART2);

		// check for errors
		if ((recieved_byte>>8)>0)
		{
			//errors found
			UART_recieved_data_length = 0;
			return;
		}

		UART_recieved_data_buffer[UART_recieved_data_length++] = (uint8_t) recieved_byte;
//		
//		//check header
//		if (UART_recieved_data_length == 2)
//		{
//			if ((UART_recieved_data_buffer[0] != 0xAB)||(UART_recieved_data_buffer[1] != 0xCD))
//			{
//				//wrong header
//				UART_recieved_data_length = 0;
//				return;
//			}
//		}
		
		//check ending
		if (UART_recieved_data_length == 6)
		{
			if ((UART_recieved_data_buffer[4] != 0xEF)||(UART_recieved_data_buffer[5] != 0xEF))
			{
				//wrong ending
				UART_recieved_data_length = 0;
				return;
			}
			else
			{
				//correct message
			}
		}
		UART_ClearITPendingBit(MDR_UART2, UART_IT_RX);
	}
}
//-----------------------------------------------------------------------
void Protocol_recieve_message()
{
	uint16_t recieved_byte;
	recieved_byte = UART_ReceiveData(MDR_UART2);
		UART_recieved_data_buffer[UART_recieved_data_length++] = (uint8_t)recieved_byte;	
		
		/* �������� �������� ��������� ����*/
		if (Protocol_check_parity(&recieved_byte)==0){
			Protocol_change_parity_mode(MODE_ADRESS);
			return ;
		}
		
			/* �������� ������ */
			if (PROTOCOL_CURRENT_PARITY_MODE == MODE_ADRESS){
				if (Protocol_check_adress(&recieved_byte)){
					Protocol_change_parity_mode(MODE_DATA);	//������ ���� ������
				}
				else{
					Protocol_change_parity_mode(MODE_ADRESS); // ��������	
					return ;
				}
			}
							
			/* ���� ������ */
			if (PROTOCOL_CURRENT_PARITY_MODE == MODE_DATA){
				if (UART_recieved_data_length == 3){ //������� ����������� ����� CRC1
					/* ������ ���������, �������� ���������� �����*/
					if (CRC1(&UART_recieved_data_buffer[0], &UART_recieved_data_buffer[1])!= UART_recieved_data_buffer[2]){
						Protocol_change_parity_mode(MODE_ADRESS); // ��������	
						return ;
					}
				}
				if ((UART_recieved_data_length == (UART_recieved_data_buffer[1] + 1))&&UART_recieved_data_length>3){ //������� ����������� ����� CRC2
					/* ������� ��� ������, �������� ����������� ����� */
					uint16_t CRC_recieved = ((uint16_t)(UART_recieved_data_buffer[UART_recieved_data_buffer[1]-1])<<8) | (uint16_t)(UART_recieved_data_buffer[UART_recieved_data_buffer[1]]);
					uint16_t CRC_actual = CRC2((uint16_t*)UART_recieved_data_buffer, UART_recieved_data_buffer[1]-1);
					if (CRC_actual == CRC_recieved){
						/* ��������� ������� ������� */
						Protocol_change_parity_mode(MODE_ADRESS); // ��������	
						Protocol_UART_message_recieved_callback(&UART_recieved_data_buffer);
						return;
					}
					else{ // �� ������ crc2 ��������
						Protocol_change_parity_mode(MODE_ADRESS); // ��������	
						return ;
					}
				}			
			}
}
//-----------------------------------------------------------------------
void Protocol_change_parity_mode(Protocol_parity_mode_type mode){ 
	/*��� ������� ������ �������� �����. 
	��� �������� ������ ���� ������ 1 ��� ������ ������,
	� ������ 0 ��� ������ ��������� ����� ���������, ���� ������ ������������
	������ ������� ����������*/
//	  UART_Parity_No   = ((uint16_t )0x00),
//    UART_Parity_Even = ((uint16_t)0x06),
//    UART_Parity_Odd  = ((uint16_t)0x02),
//    UART_Parity_1    = ((uint16_t)0x82),
//    UART_Parity_0    = ((uint16_t)0x86)
	MDR_UART2->LCR_H &= ~(0x86UL); //�������� �������� ����
	MDR_UART2->LCR_H |=(mode);
	
	if (PROTOCOL_CURRENT_MODE == MODE_RECIEVE){
		if(mode == MODE_ADRESS){
			// ���������� � ��������� � ������ ���������
			UART_recieved_data_length = 0;
			for (uint32_t i = 0; i<(sizeof(UART_recieved_data_buffer)/sizeof(uint8_t)); i++){UART_recieved_data_buffer[i] = 0;}
		}	
	PROTOCOL_CURRENT_PARITY_MODE = mode;
	}
}

//-----------------------------------------------------------------------
void Protocol_change_mode(Protocol_mode_type mode){
	if (mode == MODE_SEND) {
		PORT_SetBits(RS485_DE_RE_PORT, RS485_DE_RE_PIN);
	}
	else {
		PORT_ResetBits(RS485_DE_RE_PORT, RS485_DE_RE_PIN);
		Protocol_change_parity_mode(MODE_ADRESS);
	}
	PROTOCOL_CURRENT_MODE = mode;
}

//-----------------------------------------------------------------------
int Protocol_check_adress(uint8_t* adress){
	/* 1 ���� ������, 0 ���� �����*/
	if (*adress == OWN_ADRESS) return 1;
	else return 0;
}
//-----------------------------------------------------------------------
int Protocol_check_parity(uint16_t* recieved_byte){
	/* 1 ���� ������, 0 ���� �����*/
		if ((*recieved_byte >> 9) & 0x01) return 0; // ���� ������ ��������
			else return 1;
}

//-----------------------------------------------------------------------
void Protocol_UART_message_recieved_callback(uint8_t* Buffer){
	//��� ����� ���� ��������� ��������, ��� ��� ������ ��� ����������� ���������� ����������. ��� ����� ���� �������� ��� ������ ������
	//��������� �� ���� ��� � �������� ������. 	
//	flip_LED(MDR_PORTE, PORT_Pin_7);
	
	uint16_t data_ch1;
	data_ch1 = (Buffer[3]<<8UL) | Buffer[4];
	
	com_angle = data_ch1;
////////////////////////////////////	CDC_recieved_data_length = Buffer[2]- 4;
////////////////////////////////////	for (int i = 0; i< CDC_recieved_data_length; i++) CDC_recieved_data_buffer[i] = Buffer[i+3];
}

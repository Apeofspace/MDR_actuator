#include "main.h"

#define USB_BUFFER_LENGTH 100

static USB_Clock_TypeDef USB_Clock_InitStruct;
static USB_DeviceBUSParam_TypeDef USB_DeviceBUSParam;

static uint8_t Buffer[USB_BUFFER_LENGTH];
static USB_CDC_LineCoding_TypeDef LineCoding;

static void VCom_Configuration(void);

void VCom_Configuration(void){
    LineCoding.dwDTERate = 115200;
    LineCoding.bCharFormat = 0;
    LineCoding.bParityType = 0;
    LineCoding.bDataBits = 8;
}

void init_USB(void){
		VCom_Configuration();
		USB_CDC_Init(Buffer, 4, SET);
	
    /* Enables the CPU_CLK clock on USB */
    RST_CLK_PCLKcmd(RST_CLK_PCLK_USB, ENABLE);
    /* Device layer initialization */
    USB_Clock_InitStruct.USB_USBC1_Source = USB_C1HSEdiv2;
    USB_Clock_InitStruct.USB_PLLUSBMUL    = USB_PLLUSBMUL12;
    USB_DeviceBUSParam.MODE  = USB_SC_SCFSP_Full;
    USB_DeviceBUSParam.SPEED = USB_SC_SCFSR_12Mb;
    USB_DeviceBUSParam.PULL  = USB_HSCR_DP_PULLUP_Set;
    USB_DeviceInit(&USB_Clock_InitStruct, &USB_DeviceBUSParam);
    /* Enable all USB interrupts */
    USB_SetSIM(USB_SIS_Msk);
    USB_DevicePowerOn();
    /* Enable interrupt on USB */
    NVIC_EnableIRQ(USB_IRQn);

    USB_DEVICE_HANDLE_RESET;
}

USB_Result USB_CDC_GetLineCoding(uint16_t wINDEX, USB_CDC_LineCoding_TypeDef* DATA){
    assert_param(DATA);
    if (wINDEX != 0){
        /* Invalid interface */
        return USB_ERR_INV_REQ;
    }
    /* Just store received settings */
    *DATA = LineCoding;
    return USB_SUCCESS;
}

USB_Result USB_CDC_SetLineCoding(uint16_t wINDEX, const USB_CDC_LineCoding_TypeDef* DATA){
    assert_param(DATA);
    if (wINDEX != 0){
        /* Invalid interface */
        return USB_ERR_INV_REQ;
    }
    /* Just send back settings stored earlier */
    LineCoding = *DATA;
    return USB_SUCCESS;
}

USB_Result USB_CDC_RecieveData(uint8_t* Buffer, uint32_t Length){   
	uint16_t res = 0;
	uint16_t r = 0;
	for (uint32_t i = 0; i<Length; i++){
		r = *Buffer - '0';
		res = res*10 + r;
		Buffer++;
	}
	com_angle = res;
	return USB_SUCCESS;
}

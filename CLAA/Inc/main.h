
#ifndef __MAIN_H
	#define __MAIN_H
	
	#include "stm32f4xx_hal.h"
    #include "stm32f4xx.h"
	#include "cmsis_os.h"

	#ifdef MCU_PARA_GLOBAL
		#define EXTERN_MCU_PARA_GLOBAL
	#else
		#define EXTERN_MCU_PARA_GLOBAL extern
	#endif
	
	#define 	IRQ_DISABLE()     	(__disable_irq())
	#define  	IRQ_ENABLE()  		(__enable_irq())
	
	EXTERN_MCU_PARA_GLOBAL osThreadId lora_mac_task_handle;
	EXTERN_MCU_PARA_GLOBAL osThreadId lora_task_handle;

	EXTERN_MCU_PARA_GLOBAL UART_HandleTypeDef huart3;///本地串口
	EXTERN_MCU_PARA_GLOBAL TIM_HandleTypeDef  htim6;
	
	void LocalPrint(uint8_t data[],uint16_t len);
	void 	ErrorHandler(void);
	void 	CheckRtcTime(void);
	void 	SpiFlashInit(void);
    void    SystermSleep(void);
#endif 
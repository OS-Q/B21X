#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "cmsis_os.h"

#include "include.h"

void NMI_Handler(void)
{
	while (1);
}
void HardFault_Handler(void)
{
	while (1);
}
///TIMER6 只供CLASS-B使用
void TIM6_DAC_IRQHandler(void)
{
	if(__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) != RESET)
	{
		if(__HAL_TIM_GET_IT_SOURCE(&htim6, TIM_IT_UPDATE) !=RESET)
		{
			__HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
			HAL_TIM_PeriodElapsedCallback(&htim6);
			LoraMacClassBMsecEvent();
		}
	}
}
///LORA 协议栈、OS、UASRT接收
void SysTick_Handler(void)
{
	HAL_IncTick();
	LoraTimerTick();
    HAL_SYSTICK_IRQHandler();
	osSystickHandler();
}
///LORA使用
void EXTI0_IRQHandler( void )
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET)
	{
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
		SX1276ExtiCallback(GPIO_PIN_0);
	}
}
void EXTI1_IRQHandler( void )
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1) != RESET)
	{
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
		SX1276ExtiCallback(GPIO_PIN_1);
	}
} 
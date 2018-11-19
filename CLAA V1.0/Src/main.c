#define MCU_PARA_GLOBAL

#include "include.h"
void ErrorHandler(void)
{
    taskENTER_CRITICAL();
	while(1);
}
void LocalPrint(uint8_t data[],uint16_t len)
{
	uint16_t i;

	for(i=0;i<len;i++)
	{
		while((__HAL_UART_GET_FLAG((&huart3), UART_FLAG_TXE) ? SET : RESET) == RESET);
		(&huart3)->Instance->DR = data[i];
	}
}
void UartInit(void)
{
	GPIO_InitTypeDef 	GPIO_InitStruct;
	///本地串口
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	__HAL_RCC_USART3_CLK_ENABLE();
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&huart3);
	
	HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(USART3_IRQn);
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);
}

void Timer_6_Init(void)
{
	TIM_MasterConfigTypeDef sMasterConfig;

	__HAL_RCC_TIM6_CLK_ENABLE();

	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 40000-1;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 2-1;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
	{
		ErrorHandler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
	{
		ErrorHandler();
	}

	HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
	
	HAL_TIM_Base_Start_IT(&htim6);
}
void GpioInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	__HAL_RCC_GPIOH_CLK_ENABLE( );
	GPIO_InitStructure.Pin = GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStructure );

	LoraTxLed(0);
	LoraRxLed(0);
}
void SystemClockConfig(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 160;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		while(1);
	}
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		while(1);
	}

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		while(1);
	}

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

void StartLoraTask(void const * argument)
{
	LoraTask();
}
void StartLoraMacTask(void const * argument)
{
	LoraMacTask();
}

void StartMainTask(void const * argument)
{
	while(1)
	{
		osDelay(1000);
	}
}

int main(void)
{
	HAL_Init();
	SystemClockConfig();
	GpioInit();
	UartInit();
	LocalPrint("mwt main app start...\r\n",strlen("mwt main app start...\r\n"));

    Timer_6_Init();
	
	osThreadDef(MainTask, StartMainTask, osPriorityNormal, 0, 256);
	osThreadCreate(osThread(MainTask), NULL);

	osThreadDef(LoraTask, StartLoraTask, osPriorityNormal, 0, 1024*7);
	lora_task_handle = osThreadCreate(osThread(LoraTask), NULL);
	
	osThreadDef(LoraMacTask, StartLoraMacTask, osPriorityNormal, 0, 1024*2);
	lora_mac_task_handle = osThreadCreate(osThread(LoraMacTask), NULL);
	osKernelStart();
	while (1)
	{
	}
}

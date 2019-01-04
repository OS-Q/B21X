#define LORA_TASK_GLOBAL
#include "include.h"

LORA_MAC_PRIMITIVES_T LoRaMacPrimitives;
LORA_MAC_CALLBACK_T LoRaMacCallback;

static timer_event_t timer_list_head = {0,NULL,NULL};

void LoraTxLed(uint8_t flag)
{
	if(flag)
	{
		lora_task_struct.tx_led_flag = 1;
		HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_SET);
	}
	else
	{
		lora_task_struct.tx_led_flag = 0;
		HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_RESET);
	}
}
void LoraRxLed(uint8_t flag)
{
	if(flag)
	{
		lora_task_struct.rx_led_flag = 1;
		HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_SET);
	}
	else
	{
		lora_task_struct.rx_led_flag = 0;
		HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_RESET);
	}
}


void LoraLedFun(void)
{
	static uint16_t tx_led_cnt = 0;
	static uint16_t rx_led_cnt = 0;
	
	if(lora_task_struct.tx_led_flag)
	{
		tx_led_cnt++;
		if(tx_led_cnt >= 100)
		{
			tx_led_cnt = 0;
			LoraTxLed(0);
		}	
	}
	if(lora_task_struct.rx_led_flag)
	{
		rx_led_cnt++;
		if(rx_led_cnt >= 500)
		{
			rx_led_cnt = 0;
			LoraRxLed(0);
		}	
	}
}

void LoraTimerTick(void)
{
	timer_event_t* cur = &timer_list_head;

	while(cur->next != NULL)
	{
		cur = cur->next;
		if(cur->de_counter > 0)
		{
			cur->de_counter -= 1;
			if(cur->de_counter == 0)
			{
				cur->callback();
			}
		}
	}
	LoraLedFun();
}
void TimerInit( timer_event_t *obj,void( *callback)(void))
{
	timer_event_t* cur = &timer_list_head;
	
	while(cur->next != NULL)
	{
		cur = cur->next;
	}
	cur->next = obj;
	obj->de_counter = 0;
	obj->callback = callback;
	obj->next = NULL;
}
void TimerStart( timer_event_t *obj,uint32_t timer_val)
{
	obj->de_counter = timer_val;
}
void TimerStop( timer_event_t *obj)
{
	obj->de_counter = 0;
}


uint8_t GetBatteryLevel(void)
{
    ///采集终端电压，根据终端自身实现
    return 0xfe;///full_power,
}
void LoraMacMcpsConfirm(MCPS_CONFIRM_T *mcps_confirm)
{
    char str_char[100];
	uint8_t str_len,i,*p_tmep;
	
	MemCpy((uint8_t*)&lora_task_struct.mcps_confirm,(uint8_t*)mcps_confirm,sizeof(MCPS_CONFIRM_T));
	if((lora_task_struct.mcps_confirm.mcps_ind.rx_ack_flag)||
	   (lora_task_struct.mcps_confirm.mcps_ind.rx_data_flag))
	{
		LoraRxLed(1);
		if(lora_task_struct.mcps_confirm.mcps_ind.rx_data_flag)
		{
			p_tmep = lora_task_struct.mcps_confirm.mcps_ind.buffer;
		
			LocalPrint("\r\nrx payload:",strlen("\r\nrx payload:"));
			for(i=0;i<lora_task_struct.mcps_confirm.mcps_ind.buffer_size;i++)
			{
				str_len = sprintf(str_char,"%02X ",p_tmep[i]);
				LocalPrint((uint8_t*)&str_char,str_len);
			}
			LocalPrint("\r\n",2);
		}
	}
    lora_task_struct.tx_enable_flag = 1;	
}
void LoraMacMlmeConfirm(MLME_CONFIRM_T *mlmeConfirm)
{
	if( mlmeConfirm->status == MLMECONFIRM_STATUS_JOIN_OK )
    {
		lora_task_struct.join_net_flag = 1;
		LoraRxLed(1);
    }
	lora_task_struct.tx_enable_flag = 1;
}

void LoRaMacInit(void)
{
    LoRaMacPrimitives.MacMcpsConfirm = LoraMacMcpsConfirm;
    LoRaMacPrimitives.MacMlmeConfirm = LoraMacMlmeConfirm;
    LoRaMacCallback.GetBatteryLevel  = GetBatteryLevel;
	
    LoRaMacInitialization( &LoRaMacPrimitives,&LoRaMacCallback);
}
void LoraParaInit(void)
{
	///以下三个参数，实际应用中需要更改，在MSP注册时使用
	uint8_t dev_eui[] = { 0XFF, 0XFF,0XFF,0XFF,0XFF, 0XFF,0xFF,0xF1};
	uint8_t app_eui[] = { 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22 };
	uint8_t app_key[] = { 0x11,0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11 };
	
	///CLASS-A
	lora_work_para_struct.class_mode 		= (uint8_t)CLASS_A;
	lora_work_para_struct.claa_mode 		= CLAA_A;
	lora_work_para_struct.tx_auto_flag  	= (uint8_t)VALID_AA;
	lora_work_para_struct.freq_rand_flag   	= false;
	lora_work_para_struct.tx_freq           = 482300000;
	///CLASS-B
/**	lora_work_para_struct.class_mode 		= (uint8_t)CLASS_B;
	lora_work_para_struct.claa_mode 		= CLAA_D;
	lora_work_para_struct.tx_auto_flag  	= false;
	lora_work_para_struct.freq_rand_flag   	= false;
	lora_work_para_struct.tx_freq           = 484300000;
**/	
	lora_work_para_struct.tx_interval 		= (uint8_t)CLASS_A_DEFAULT_MIN_TX_MSEC_INTERVAL;
	lora_work_para_struct.tx_confirm_flag 	= VALID_AA;
	lora_work_para_struct.tx_pwr 			= LORAMAC_MAX_TX_POWER;
	lora_work_para_struct.tx_sf				= LORAMAC_MAX_TX_SF;
	lora_work_para_struct.re_tx_num         = 3;
	
	MemCpy(lora_work_para_struct.dev_eui,dev_eui,8);
	MemCpy(lora_work_para_struct.app_eui,app_eui,8);
	MemCpy(lora_work_para_struct.app_key,app_key,16);
}
void LoRaMacJoinReq(void)
{
	LORA_MAC_STATUS_T status;
    MLME_REQ_T mlme_request;

	mlme_request.type = MLME_JOIN;
	mlme_request.Join.app_eui = lora_work_para_struct.app_eui;
	mlme_request.Join.app_key = lora_work_para_struct.app_key;
	mlme_request.Join.dev_eui = lora_work_para_struct.dev_eui;
	
	status = LoRaMacMlmeRequest( &mlme_request );
	if(status == LORAMAC_STATUS_TX_OK)
	{
		lora_task_struct.tx_enable_flag = 0;
		LoraTxLed(1);
	}
}
void LoraMoteJoinNet(void)
{
	uint32_t delay_ms;
	
	LocalPrint("mote join start...,",sizeof("mote join start...,"));
	lora_task_struct.join_net_flag = 0;
	lora_task_struct.tx_enable_flag = 1;
	while(!lora_task_struct.join_net_flag)
	{
		if(lora_task_struct.tx_enable_flag == 1)
		{
			if(lora_work_para_struct.tx_auto_flag)
			{
				delay_ms = rand() % 5000;
				osDelay(delay_ms);///rand ms,avoid collid
			}
			
			if(lora_task_struct.join_net_flag)
			{
				break;
			}
			
			LoRaMacJoinReq();
			if(lora_work_para_struct.tx_auto_flag)
			{
				osDelay(10000-delay_ms);///rand ms,avoid collid
			}
		}
		else
		{
			osDelay(1000);
		}
	}
	lora_task_struct.tx_enable_flag = 1;
	if(lora_task_struct.join_net_flag == 1)
	{
		LocalPrint("join net ok\r\n",sizeof("join net ok\r\n"));
	}
}
void LoraMoteNetTest( void )
{
	uint8_t i,tx_len,tx_data[256];
	uint32_t delay_ms;
	MCPS_REQ_T mcps_request;
	LORA_MAC_STATUS_T status;

	tx_len = 30;
	for(i=0;i<tx_len;i++)
	{
		tx_data[i] = i;
	}
	
	while(1)
	{
		if(lora_task_struct.tx_enable_flag == 1)
		{
			MemSet((uint8_t*)&mcps_request,0x00,sizeof(MCPS_REQ_T));
			
			if(lora_work_para_struct.tx_auto_flag)
			{
				delay_ms = rand() % (lora_work_para_struct.tx_interval*1000);
				osDelay(delay_ms);
			}
			
			if(lora_work_para_struct.tx_confirm_flag == VALID_AA)
			{
				mcps_request.type = MCPS_CONFIRMED;
				mcps_request.req.confirm.buffer = tx_data;
				mcps_request.req.confirm.buffer_size = tx_len;
				mcps_request.req.confirm.fport = LORA_WAN_PORT;
			}
			else
			{
				mcps_request.type = MCPS_UNCONFIRMED;
				mcps_request.req.unconfirm.buffer = tx_data;
				mcps_request.req.unconfirm.buffer_size = tx_len;
				mcps_request.req.unconfirm.fport = LORA_WAN_PORT;
			}
			
			MemSet((uint8_t*)&lora_task_struct.mcps_confirm,0x00,sizeof(MCPS_CONFIRM_T));
			status = LoRaMacMcpsRequest(&mcps_request);
			if(status == LORAMAC_STATUS_TX_OK)
			{
				lora_task_struct.tx_enable_flag = 0;
				LoraTxLed(1);
			}

			if(lora_work_para_struct.tx_auto_flag)
			{
				osDelay(lora_work_para_struct.tx_interval*1000-delay_ms);
			}
		}
		else
		{
			osDelay(1000);
		}
	}
}

void LoraTask(void)
{
	LoraParaInit();
	LoRaMacInit();
	LoraMoteJoinNet();
	while(1)
	{
		LoraMoteNetTest();
	}
}
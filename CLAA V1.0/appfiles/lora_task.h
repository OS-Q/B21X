
#ifndef __LORA_TASK_H__
#define __LORA_TASK_H__
	#include "stm32f4xx.h"
	#include "stm32f4xx_hal.h"
	
	#ifdef LORA_TASK_GLOBAL
		#define EXTERN_LORA_TASK_STRUCT
	#else
		#define EXTERN_LORA_TASK_STRUCT extern
	#endif
	
	#include "lora_mac.h"

	#define LORA_WAN_PORT       						0x21
	#define VALID_AAAA									0xAAAA
	#define VALID_AA    	                            0xAA
					
	typedef struct timer_event
	{
		uint32_t de_counter;
		void ( *callback )( void );
		struct timer_event *next;
	}timer_event_t;
	
	typedef struct
	{
		uint8_t  join_net_flag;///入网标志
		uint8_t  tx_enable_flag;///允许发送标志
		
		uint8_t  lora_net_test_flag;///LORA网络测试状态机
		
		uint8_t  scan_net_flag;///LORA搜网状态机
		uint8_t  scan_net_mode;///搜网模式，0为D,B,E,C,A,1为CLAA-A,2,为CLAA-B...
		
		uint8_t  tx_led_flag;
		uint8_t  rx_led_flag;
		
		
		uint8_t  tx_mac_cmd_flag;///发送命令标志
		uint8_t  tx_mac_cmd_id;///应用有命令要发标志
		uint8_t  tx_mac_cmd_para_1;
		uint8_t  tx_mac_cmd_para_2;
		uint8_t  rx_cmd_ack_flag;
		uint8_t  rx_cmd_ack;
		
		uint8_t  classb_back_to_classa_flag;
		uint8_t  ping_slot_param;
		
		uint8_t  swt_class_ac_flag;///切换CLASS AC标志
		uint8_t  rx_swt_class_ac_ack_flag;///收到切换CLASS AC ACK标志
		uint8_t  swt_class_ac_ack;///class_a到class_c切换应答
		
		uint8_t tx_mac_time_req_flag;
		uint8_t rx_beacon_flag;
		
		MCPS_CONFIRM_T mcps_confirm;
	}LORA_TASK_STRUCT;

	typedef struct
	{
		uint8_t  dev_eui[8];
		uint8_t  app_eui[8];
		uint8_t  app_key[16];
		uint32_t tx_freq;///发射频率
		uint8_t  freq_rand_flag;///是否随机频点
		uint8_t  tx_pwr;///发射功率
		uint8_t  tx_sf;///扩频因子
		uint8_t  class_mode;///CLASS模式，A,B,C;
		uint8_t  claa_mode;///CLAA模式，CLAA-A,B,C,D,E
		uint8_t  tx_auto_flag;	///自动发送标志
		uint8_t  tx_interval;///自动时，发送频率
		uint8_t  tx_pkt_len;///发包长度
		uint8_t  re_tx_num;///确认帧重发次数
		uint8_t  tx_confirm_flag;///发确认帧标志
		uint8_t  channel_mask[10];///信道屏蔽，为1，则可用
		
		uint8_t  check_val;
	}LORA_WORK_PARA_STRUCT;///工作参数

	
	EXTERN_LORA_TASK_STRUCT LORA_TASK_STRUCT  		lora_task_struct;
	EXTERN_LORA_TASK_STRUCT LORA_WORK_PARA_STRUCT	lora_work_para_struct;
	
	void LoraTxLed(uint8_t flag);
	void LoraRxLed(uint8_t flag);
	
	void LoraTimerTick(void);
	void TimerInit( timer_event_t *obj,void( *callback)(void));
	void TimerStart( timer_event_t *obj,uint32_t timer_val);
	void TimerStop( timer_event_t *obj);
	void TimerIrqCallBack(void);
	
	void LoraWorkParaInit(void);
	void LoraTxPktDelay(uint32_t ms);
	void LoraMcpsConfirm(MCPS_CONFIRM_T *mcps_confirm);
	void LoraMlmeConfirm(MLME_CONFIRM_T *mlmeConfirm);
	void LoRaMacJoinReq(void);
	void LoRaMacInit(void);
	void LoraDataInit(void);
	void LoraTask(void);
#endif

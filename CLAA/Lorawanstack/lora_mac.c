#define LORA_MAC_GLOBAL
#include "include.h"
#include "lora_stack.h"

const uint8_t 	mac_data_rate_array[]  		= { 12, 11, 10,  9,  8,  7};
const uint8_t 	max_payload_array[]   	 	= { 51, 51, 51, 115, 222, 222};
///各SF下，满包发送时间
const uint32_t  mac_max_tx_time_array[]  	= { 2630, 1397, 699,  677,  656,  369};


static uint8_t *p_lora_mac_dev_eui;
static uint8_t *p_lora_mac_app_eui;
static uint8_t *p_lora_mac_app_key;

static uint8_t lora_mac_nwk_skey[] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static uint8_t lora_mac_app_skey[] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
const uint32_t CLAA_A_ALL_FREQ[72] = 
{/**	
	482300000,482500000,482700000,482900000,483100000,483300000,483500000,483700000,
	484300000,484500000,484700000,484900000,485100000,485300000,485500000,485700000,
	486300000,486500000,486700000,486900000,487100000,487300000,487500000,487700000,
	488300000,488500000,488700000,488900000,489100000,489300000,489500000,489700000,
	490300000,490500000,490700000,490900000,491100000,491300000,491500000,491700000,
	492300000,492500000,492700000,492900000,493100000,493300000,493500000,493700000,
	494300000,494500000,494700000,494900000,495100000,495300000,495500000,495700000,
	496300000,496500000,496700000,496900000,497100000,497300000,497500000,497700000,
	498300000,498500000,498700000,498900000,499100000,499300000,499500000,499700000,
**/
	482300000,482500000,482700000,482900000,483100000,483300000,483500000,483700000,
	482300000,484500000,484700000,484900000,485100000,485300000,485500000,485700000,
	482300000,486500000,486700000,486900000,487100000,487300000,487500000,487700000,
	482300000,488500000,488700000,488900000,489100000,489300000,489500000,489700000,
	482300000,490500000,490700000,490900000,491100000,491300000,491500000,491700000,
	482300000,492500000,492700000,492900000,493100000,493300000,493500000,493700000,
	482300000,494500000,494700000,494900000,495100000,495300000,495500000,495700000,
	482300000,496500000,496700000,496900000,497100000,497300000,497500000,497700000,
	482300000,498500000,498700000,498900000,499100000,499300000,499500000,499700000,
};
const uint32_t CLAA_B_ALL_FREQ[80] = 
{
	470300000,470500000,470700000,470900000,471100000,471300000,471500000,471700000,
	472300000,472500000,472700000,472900000,473100000,473300000,473500000,473700000,
	474300000,474500000,474700000,474900000,475100000,475300000,475500000,475700000,
	476300000,476500000,476700000,476900000,477100000,477300000,477500000,477700000,
	478300000,478500000,478700000,478900000,479100000,479300000,479500000,479700000,
	480300000,480500000,480700000,480900000,481100000,481300000,481500000,481700000,
	482300000,482500000,482700000,482900000,483100000,483300000,483500000,483700000,
	484300000,484500000,484700000,484900000,485100000,485300000,485500000,485700000,
	486300000,486500000,486700000,486900000,487100000,487300000,487500000,487700000,
	488300000,488500000,488700000,488900000,489100000,489300000,489500000,489700000,
	
};
const uint32_t CLAA_C_ALL_FREQ[80] = 
{
	490300000,490500000,490700000,490900000,491100000,491300000,491500000,491700000,
	492300000,492500000,492700000,492900000,493100000,493300000,493500000,493700000,
	494300000,494500000,494700000,494900000,495100000,495300000,495500000,495700000,
	496300000,496500000,496700000,496900000,497100000,497300000,497500000,497700000,
	498300000,498500000,498700000,498900000,499100000,499300000,499500000,499700000,
	500300000,500500000,500700000,500900000,501100000,501300000,501500000,501700000,
	502300000,502500000,502700000,502900000,503100000,503300000,503500000,503700000,
	504300000,504500000,504700000,504900000,505100000,505300000,505500000,505700000,
	506300000,506500000,506700000,506900000,507100000,507300000,507500000,507700000,
	508300000,508500000,508700000,508900000,509100000,509300000,509700000,509700000,
};
const uint32_t CLAA_D_UP_FREQ[48] = 
{
	480300000,480500000,480700000,480900000,481100000,481300000,481500000,481700000,481900000,482100000,
	482300000,482500000,482700000,482900000,483100000,483300000,483500000,483700000,483900000,484100000,
	484300000,484500000,484700000,484900000,485100000,485300000,485500000,485700000,485900000,486100000,
	486300000,486500000,486700000,486900000,487100000,487300000,487500000,487700000,487900000,488100000,
	488300000,488500000,488700000,488900000,489100000,489300000,489500000,489700000,
};
const uint32_t CLAA_D_DOWN_FREQ[30] = 
{
	500100000,500300000,500500000,500700000,500900000,501100000,501300000,501500000,501700000,501900000,
	502100000,502300000,502500000,502700000,502900000,503100000,503300000,503500000,503700000,503900000,
	504100000,504300000,504500000,504700000,504900000,505100000,505300000,505500000,505700000,505900000,
};
const uint32_t CLAA_E_UP_FREQ[48] = 
{
	470300000,470500000,470700000,470900000,471100000,471300000,471500000,471700000,471900000,472100000,
	472300000,472500000,472700000,472900000,473100000,473300000,473500000,473700000,473900000,474100000,
	474300000,474500000,474700000,474900000,475100000,475300000,475500000,475700000,475900000,476100000,
	476300000,476500000,476700000,476900000,477100000,477300000,477500000,477700000,477900000,478100000,
	478300000,478500000,478700000,478900000,479100000,479300000,479500000,479700000,
};
const uint32_t CLAA_E_DOWN_FREQ[30] = 
{
	490100000,490300000,490500000,490700000,490900000,491100000,491300000,491500000,491700000,491900000,
	492100000,492300000,492500000,492700000,492900000,493100000,493300000,493500000,493700000,493900000,
	494100000,494300000,494500000,494700000,494900000,495100000,495300000,495500000,495700000,495900000,
};



static uint16_t 				lora_mac_tx_pkt_len = 0;
static uint8_t  				lora_mac_tx_pkt_buf[LORAMAC_PHY_MAXPAYLOAD];
static uint8_t  				lora_mac_rx_pkt_buf[LORAMAC_PHY_MAXPAYLOAD];

static uint8_t 					node_need_ack_flag = false;
static uint8_t 					node_need_ack_tx_counter = 0;

static uint16_t 				lora_mac_dev_nonce;
static uint32_t 				lora_mac_net_id;
static uint32_t 				lora_mac_dev_addr;
static 							CLASS_MODE_T node_device_class_type;
static uint32_t 				stack_tx_pkt_index = 0;
static uint32_t 				stack_rx_pkt_index = 0;
static bool 					server_need_ack_flag = false;
static bool 					node_join_network_flag = false;
static bool 					node_adr_ctrl_flag = false;
uint8_t 						lora_mac_adr_ack_req_flag = 0;
static uint32_t 				node_adr_counter = 0;

static uint8_t  				mac_tx_cmd_buf_index = 0;
static uint8_t  				mac_tx_cmd_buf[LORA_MAC_COMMAND_MAX_LENGTH];

uint8_t 						cad_done_flag = false;
uint8_t 						cad_channel_busy_flag = false;



static LORA_MAC_PRIMITIVES_T 	*p_lora_mac_primitives;
static LORA_MAC_CALLBACK_T   	*p_lora_mac_callback;
static MCPS_CONFIRM_T 			mcps_confirm_struct;
static MLME_CONFIRM_T			mlme_confirm_struct;
LORA_MAC_REQ_FLAG_T				lora_mac_req_flag_struct;
static RadioEvents_t 			radio_event_struct;
LORA_MAC_RX_PARA_T 				lora_mac_rx_para_struct;

static timer_event_t 			rx_window1_timer;
static timer_event_t 			rx_window2_timer;

static uint8_t  				lora_mac_rx_slot = 0;
static uint32_t 				rx_window1_delay_time;
static uint32_t 				rx_window2_delay_time;

LORA_MAC_RX_TX_FLAG_T 			lora_mac_rxtx_flag = LORAMAC_RXTX_FLAG_NONE;
LORA_MAC_TX_STATE_T  			lora_mac_exe_flag = MAC_IDLE;

CLASS_B_T                    	class_b_para;
CLASS_C_T                    	class_c_para;

uint8_t         	channel_rand_mask[10] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};///随机化信道用，本次使用哪个信道，则失效该信道，下次切换下个可用信道
static void 		OnRadioTxDone( void );
static void 		OnRadioTxTimeout( void );
static void 		OnRadioRxError( void );
static void 		OnRadioRxTimeout( void );
static void 		OnRadioRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int16_t snr);

static void 		OnRxWindow1TimerEvent( void );
static void 		OnRxWindow2TimerEvent( void );

void 				LoraMacSetRxChannel( uint32_t freq, uint8_t datarate, uint32_t bandwidth, uint16_t timeout, bool rxContinuous );
void 				LoraMacProcessCmd( uint8_t *payload, uint8_t macIndex, uint8_t commandsSize, int8_t snr,uint8_t pkt_len);
LORA_MAC_STATUS_T 	LoraMacPrepareFrame( LORA_MAC_HEADER_T *macHdr, LORA_MAC_FRAME_CTRL_T *fCtrl, uint8_t fport, void *buffer, uint16_t buffer_size );
///保存通信上下文信息
void LoraMacSaveCommInfo(void)
{
	/**
	lora_mac_nwk_skey
	lora_mac_app_skey
	lora_mac_net_id
	lora_mac_dev_addr
	stack_tx_pkt_index
	stack_rx_pkt_index
	**/
}
///根据频点，获取信道号
uint8_t LoraMacGetFreqChan(uint32_t freq,const uint32_t *freq_array,uint8_t array_size)
{
	uint8_t i;
	for(i=0;i<array_size;i++)
	{
		if(freq_array[i] == freq)
		{
			break;
		}
	}
	return i;
}
///设置默认信道
void LoraMacSetDefaultChanMask(void)
{
	uint8_t i,claa_d_e_mask[6] = {0x80,0x20,0x08,0x02,0x00,0x80};
	
	for(i=0;i<10;i++)
	{
		lora_work_para_struct.channel_mask[i] = 0x00;
	}
	
	if(lora_work_para_struct.claa_mode == CLAA_A)
	{	for(i=0;i<9;i++)
		{
			lora_work_para_struct.channel_mask[i] = 0x80;
		}	
	}
	else if((lora_work_para_struct.claa_mode == CLAA_B)||(lora_work_para_struct.claa_mode == CLAA_C))
	{
		for(i=0;i<10;i++)
		{
			lora_work_para_struct.channel_mask[i] = 0x80;
		}	
	}
	else if((lora_work_para_struct.claa_mode == CLAA_D)||(lora_work_para_struct.claa_mode == CLAA_E))
	{
		for(i=0;i<6;i++)
		{
			lora_work_para_struct.channel_mask[i] = claa_d_e_mask[i];
		}	
	}
}
///随机化信道
uint8_t LoraMacRandChan(void)
{
	static 
	uint8_t i,j,bit_num,i_bit_num,j_bit_num,rand_val;
	
UP_LAB:	
	///计算所有可用信道数量
	bit_num = Logic1Bits(channel_rand_mask,10);
	if(bit_num == 0)
	{
		j_bit_num = Logic1Bits(lora_work_para_struct.channel_mask,10);
		if(j_bit_num == 0)
		{
			LoraMacSetDefaultChanMask();
		}
		MemCpy(channel_rand_mask,lora_work_para_struct.channel_mask,10);
		goto UP_LAB;
	}
	
	rand_val = rand() % bit_num;
    i_bit_num = 0;
	for(i=0;i<10;i++)
	{
		for(j=0;j<8;j++ )
		{
			if( ( channel_rand_mask[i] & ( 1 << (7-j) ) ) == ( 1 << (7-j) ) )
			{
				if(i_bit_num == rand_val)
				{
					channel_rand_mask[i] &= ~(1 << (7-j));
					return i*8+j;
				}
				i_bit_num++;
			}
		}
	}	
	return 0;
}	
///根据CLAA-MODE，获取发射频点
uint32_t LoraMacGetFreq(uint8_t freq_index)
{
	uint32_t tx_freq;
	
	if(lora_work_para_struct.claa_mode == CLAA_A)
	{
		tx_freq = CLAA_A_ALL_FREQ[freq_index];
	}
	else if(lora_work_para_struct.claa_mode == CLAA_B)
	{
		tx_freq = CLAA_B_ALL_FREQ[freq_index];
	}
	else if(lora_work_para_struct.claa_mode == CLAA_C)
	{
		tx_freq = CLAA_C_ALL_FREQ[freq_index];
	}
	else if(lora_work_para_struct.claa_mode == CLAA_D)
	{
		tx_freq = CLAA_D_UP_FREQ[freq_index];
	}
	else
	{
		tx_freq = CLAA_E_UP_FREQ[freq_index];
	}
	
	return tx_freq;
}
///根据随机信道，获得发射频点
void LoraMacRandTxFreq(void)
{
    uint8_t freq_index;
	
	if((lora_work_para_struct.freq_rand_flag == (uint8_t)VALID_AA)||(lora_task_struct.scan_net_flag))
	{

		if(!node_join_network_flag)
		{
			MemSet(channel_rand_mask,0,10);
		}
		
		freq_index = LoraMacRandChan();
		lora_work_para_struct.tx_freq = LoraMacGetFreq(freq_index);
	}
	///如果没有入网，则初始化RX2参数
	if(!node_join_network_flag)
	{
		if((lora_work_para_struct.claa_mode == CLAA_A)||(lora_work_para_struct.claa_mode == CLAA_B)||(lora_work_para_struct.claa_mode == CLAA_C))
		{
			lora_mac_rx_para_struct.rx2_freq = lora_work_para_struct.tx_freq;
			lora_mac_rx_para_struct.rx2_dr = lora_mac_rx_para_struct.rx1_dr;
		}
		else if(lora_work_para_struct.claa_mode == CLAA_D)
		{
			lora_mac_rx_para_struct.rx2_freq = 502500000;
			lora_mac_rx_para_struct.rx2_dr = lora_mac_rx_para_struct.rx1_dr;
		}
		else if(lora_work_para_struct.claa_mode == CLAA_E)
		{
			lora_mac_rx_para_struct.rx2_freq = 492500000;
			lora_mac_rx_para_struct.rx2_dr = lora_mac_rx_para_struct.rx1_dr;
		}
	}
}
///不同的SF下,可发送的最大字节数据
uint8_t LoraMacGetMaxTxSize(uint8_t sf)
{
	return max_payload_array[12-sf];
}
///发送完成中断回调函数
static void OnRadioTxDone( void )
{
	if(( node_device_class_type == CLASS_A )||( node_device_class_type == CLASS_B ))
	{
		Radio.Sleep( );///进入低功耗模式
    }
    
    TimerStart( &rx_window1_timer, rx_window1_delay_time );
	
    if(( node_device_class_type == CLASS_A )||( node_device_class_type == CLASS_B ))
    {
        TimerStart( &rx_window2_timer, rx_window2_delay_time );
    }

}
///发送超时中断回调函数
static void OnRadioTxTimeout( void )
{
    if( node_device_class_type == CLASS_C )
    {
        OnRxWindow2TimerEvent( );
    }
    else
    {
        Radio.Sleep( );
    }
	
	lora_mac_rxtx_flag = LORAMAC_RXTX_FLAG_TXERROR;
}
///CAD检测中断回调函数，CSMA-CD发包冲突避让用
static void OnRadioCadDone(  bool channelActivityDetected  )
{
	cad_done_flag = true;
	cad_channel_busy_flag = channelActivityDetected;
}
///接收错误回调函数，CRC错误
static void OnRadioRxError( void )
{
    if( node_device_class_type == CLASS_C )
    {
        OnRxWindow2TimerEvent( );
    }
    else
    {
        Radio.Sleep( );
    }

    if( lora_mac_rx_slot == 1 )
    {
		lora_mac_rxtx_flag = LORAMAC_RXTX_FLAG_RXERROR;
    }
}
///接收超时回调函数
static void OnRadioRxTimeout( void )
{	
    if( node_device_class_type == CLASS_C )
    {
        OnRxWindow2TimerEvent( );
    }
    else
    {
        Radio.Sleep( );
    }

    if( lora_mac_rx_slot == 1 )
    {
		lora_mac_rxtx_flag = LORAMAC_RXTX_FLAG_RXERROR;
    }
}
///接收完成中断回调函数，主要区分JOIN-ACCEPT和正常数据包及CLASS-B BEACON帧
static void OnRadioRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int16_t snr)
{
    LORA_MAC_HEADER_T macHdr;
    LORA_MAC_FRAME_CTRL_T fCtrl;
    uint8_t pktHeaderLen = 0,appPayloadStartIndex = 0, port = 0xFF,frameLen = 0;
    uint32_t address,micRx,mic = 0;

    static uint16_t last_rx_pkt_index = 0;
    uint16_t current_rx_pkt_index = 0;
    uint16_t seq_diff = 0;
    uint32_t temp_pkt_index = 0;

    uint8_t *nwkSKey = lora_mac_nwk_skey;
    uint8_t *appSKey = lora_mac_app_skey;

    uint8_t multicast_flag = false,isMicOk = false;
	
    mcps_confirm_struct.mcps_ind.rssi = rssi;
    mcps_confirm_struct.mcps_ind.snr = snr;
	snr /= 10;
    mcps_confirm_struct.mcps_ind.rx_slot = lora_mac_rx_slot;
    mcps_confirm_struct.mcps_ind.port = 0;
    mcps_confirm_struct.mcps_ind.multi_cast_flag = 0;
    mcps_confirm_struct.mcps_ind.fpending = 0;
    mcps_confirm_struct.mcps_ind.buffer = NULL;
    mcps_confirm_struct.mcps_ind.buffer_size = 0;
    mcps_confirm_struct.mcps_ind.rx_ack_flag = false;
    mcps_confirm_struct.mcps_ind.Ind_type = MCPS_INDICATION_NONE;
	mcps_confirm_struct.mcps_ind.rx_data_flag = false;
	mcps_confirm_struct.mcps_ind.rx_cmd_flag = false;
	
    if( node_device_class_type != CLASS_C )
    {
        Radio.Sleep( );
    }
	
    TimerStop( &rx_window2_timer );
	
	mcps_confirm_struct.status = MCPSCONFIRM_STATUS_IDLE;
	mlme_confirm_struct.status = MLMECONFIRM_STATUS_IDLE;
	
    macHdr.value = payload[pktHeaderLen++];
	
	if(class_b_para.wait_beacon_frame_flag)
	{
		///CLASS-B BEACON 帧
		if(size == 17)
		{
			lora_task_struct.rx_beacon_flag = true;
			class_b_para.rx_abs_nwk_time_flag = true;
			class_b_para.wait_beacon_frame_flag = false;
			node_device_class_type = CLASS_B;
			class_b_para.nwk_sec = U8ToUint32(payload+3);
			class_b_para.ms_counter = 289;
			LoraMacCountClassBPingSlot(class_b_para.nwk_sec);
			goto RETURN_LAB;
		}
	}
	else if( lora_mac_req_flag_struct.bits.mlme_req == 1 )
	{
		///JOIN-ACCEPT 帧
		if(macHdr.bits.m_type == FRAME_TYPE_JOIN_ACCEPT)
		{
			LoRaMacJoinDecrypt( payload + 1, size - 1, p_lora_mac_app_key, lora_mac_rx_pkt_buf + 1 );
			lora_mac_rx_pkt_buf[0] = macHdr.value;
			LoRaMacJoinComputeMic( lora_mac_rx_pkt_buf, size - LORAMAC_MFR_LEN, p_lora_mac_app_key, &mic );

			micRx  = ( uint32_t )lora_mac_rx_pkt_buf[size - LORAMAC_MFR_LEN];
			micRx |= ( ( uint32_t )lora_mac_rx_pkt_buf[size - LORAMAC_MFR_LEN + 1] << 8 );
			micRx |= ( ( uint32_t )lora_mac_rx_pkt_buf[size - LORAMAC_MFR_LEN + 2] << 16 );
			micRx |= ( ( uint32_t )lora_mac_rx_pkt_buf[size - LORAMAC_MFR_LEN + 3] << 24 );

			if( micRx == mic )
			{
				LoRaMacJoinComputeSKeys( p_lora_mac_app_key, lora_mac_rx_pkt_buf + 1, lora_mac_dev_nonce, lora_mac_nwk_skey, lora_mac_app_skey );

				lora_mac_net_id = ( uint32_t )lora_mac_rx_pkt_buf[4];
				lora_mac_net_id |= ( ( uint32_t )lora_mac_rx_pkt_buf[5] << 8 );
				lora_mac_net_id |= ( ( uint32_t )lora_mac_rx_pkt_buf[6] << 16 );

				lora_mac_dev_addr = ( uint32_t )lora_mac_rx_pkt_buf[7];
				lora_mac_dev_addr |= ( ( uint32_t )lora_mac_rx_pkt_buf[8] << 8 );
				lora_mac_dev_addr |= ( ( uint32_t )lora_mac_rx_pkt_buf[9] << 16 );
				lora_mac_dev_addr |= ( ( uint32_t )lora_mac_rx_pkt_buf[10] << 24 );

				/// DLSettings
				lora_mac_rx_para_struct.rx1_dr_offset = ( lora_mac_rx_pkt_buf[11] >> 4 ) & 0x07;
				lora_mac_rx_para_struct.rx2_dr = lora_mac_rx_pkt_buf[11] & 0x0F;

				/// RxDelay
				lora_mac_rx_para_struct.ReceiveDelay1 = ( lora_mac_rx_pkt_buf[12] & 0x0F );
				if( lora_mac_rx_para_struct.ReceiveDelay1 == 0 )
				{
					lora_mac_rx_para_struct.ReceiveDelay1 = 1;
				}
				lora_mac_rx_para_struct.ReceiveDelay1 *= 1e3;
				lora_mac_rx_para_struct.ReceiveDelay2 = lora_mac_rx_para_struct.ReceiveDelay1 + (uint32_t)(1e3);
				
				MemSet(lora_work_para_struct.channel_mask,0,10);
				if((lora_work_para_struct.claa_mode == CLAA_A)||(lora_work_para_struct.claa_mode == CLAA_B)||(lora_work_para_struct.claa_mode == CLAA_B))
				{
					if(lora_work_para_struct.claa_mode != (CLAA_MODE_T)(lora_mac_rx_pkt_buf[24]%5))
					{
						goto RETURN_LAB;
					}
					
					if(lora_work_para_struct.claa_mode == CLAA_A)
					{
						MemCpy(lora_work_para_struct.channel_mask,lora_mac_rx_pkt_buf+14,9);
					}
					else
					{
						MemCpy(lora_work_para_struct.channel_mask,lora_mac_rx_pkt_buf+14,10);
					}
					///与CLASS相关
					if(CLASS_B == (CLASS_MODE_T)lora_work_para_struct.class_mode)
					{
						class_b_para.rxb_ch = lora_mac_rx_pkt_buf[25];
						class_b_para.rxb_dr = lora_mac_rx_pkt_buf[26];
						///rx beacon ch 为default ch
						if(lora_work_para_struct.claa_mode == CLAA_A)
						{
							class_b_para.rx_beacon_ch = LoraMacGetFreqChan(lora_mac_rx_para_struct.rx1_freq,CLAA_A_ALL_FREQ,72);
						}
						else if(lora_work_para_struct.claa_mode == CLAA_B)
						{
							class_b_para.rx_beacon_ch = LoraMacGetFreqChan(lora_mac_rx_para_struct.rx1_freq,CLAA_B_ALL_FREQ,80);
						}
						else if(lora_work_para_struct.claa_mode == CLAA_C)
						{
							class_b_para.rx_beacon_ch = LoraMacGetFreqChan(lora_mac_rx_para_struct.rx1_freq,CLAA_C_ALL_FREQ,80);
						}
						class_b_para.rx_abs_nwk_time_flag = false;
					}
					else if(CLASS_C == (CLASS_MODE_T)lora_work_para_struct.class_mode)
					{
						class_c_para.rxc_ch = lora_mac_rx_pkt_buf[25];
						class_c_para.rxc_dr = lora_mac_rx_pkt_buf[26];
						node_device_class_type = CLASS_C;
					}
					lora_mac_rx_para_struct.rx2_freq = LoraMacGetFreq(lora_mac_rx_pkt_buf[13]);		
				}
				else
				{
					if(lora_work_para_struct.claa_mode != (CLAA_MODE_T)(lora_mac_rx_pkt_buf[13]%5))
					{
						goto RETURN_LAB;
					}
					MemCpy(lora_work_para_struct.channel_mask,lora_mac_rx_pkt_buf+14,6);
					///与CLASS相关
					if(CLASS_B == (CLASS_MODE_T)lora_work_para_struct.class_mode)
					{
						class_b_para.rxb_ch = lora_mac_rx_pkt_buf[20];
						class_b_para.rxb_dr = lora_mac_rx_pkt_buf[21];
						class_b_para.ping_nb = (class_b_para.rxb_dr & 0x38)>>3;
						class_b_para.rx_abs_nwk_time_flag = false;
					}
					else if(CLASS_C == (CLASS_MODE_T)lora_work_para_struct.class_mode)
					{
						class_c_para.rxc_ch = lora_mac_rx_pkt_buf[20];
						class_c_para.rxc_dr = lora_mac_rx_pkt_buf[21];
						node_device_class_type = CLASS_C;
					}
					
					if(lora_work_para_struct.claa_mode == CLAA_D)
					{
						lora_mac_rx_para_struct.rx2_freq = 502500000;
					}
					else
					{
						lora_mac_rx_para_struct.rx2_freq = 492500000;
					}
				}
				mlme_confirm_struct.status = MLMECONFIRM_STATUS_JOIN_OK;
				node_join_network_flag = true;
				///不同CLAA-MODE下，使能信道列表
				stack_rx_pkt_index = 0;
				last_rx_pkt_index = 0;
				if(!lora_task_struct.scan_net_flag)
				{
					MemCpy(channel_rand_mask,lora_work_para_struct.channel_mask,10);
				}
			}
		}
	}
	else
	{
		if((macHdr.bits.m_type != FRAME_TYPE_DATA_CONFIRMED_DOWN)&&
		   (macHdr.bits.m_type != FRAME_TYPE_DATA_UNCONFIRMED_DOWN)&&
		   (macHdr.bits.m_type != FRAME_TYPE_PROPRIETARY))
		{
			goto RETURN_LAB;
		}
		///数据帧
		if(macHdr.bits.m_type == FRAME_TYPE_PROPRIETARY)
		{
			MemCpy( lora_mac_rx_pkt_buf, &payload[pktHeaderLen], size );
			mcps_confirm_struct.mcps_ind.Ind_type = MCPS_INDICATION_PROPRIETARY;
			mcps_confirm_struct.mcps_ind.buffer = lora_mac_rx_pkt_buf;
			mcps_confirm_struct.mcps_ind.buffer_size = size - pktHeaderLen;
			mcps_confirm_struct.status = MCPSCONFIRM_STATUS_RX_DATA_OK;
			goto RETURN_LAB;
		}
		
		address = payload[pktHeaderLen++];
		address |= ( (uint32_t)payload[pktHeaderLen++] << 8 );
		address |= ( (uint32_t)payload[pktHeaderLen++] << 16 );
		address |= ( (uint32_t)payload[pktHeaderLen++] << 24 );

		if( address == lora_mac_dev_addr )
		{///local addr
			multicast_flag = 0;
			nwkSKey = lora_mac_nwk_skey;
			appSKey = lora_mac_app_skey;
			temp_pkt_index = stack_rx_pkt_index;
		}
		else
		{
			goto RETURN_LAB;
		}

		fCtrl.value = payload[pktHeaderLen++];

		current_rx_pkt_index = ( uint16_t )payload[pktHeaderLen++];///下行fcnt
		current_rx_pkt_index |= ( uint16_t )payload[pktHeaderLen++] << 8;

		appPayloadStartIndex = 8 + fCtrl.bits.fopts_len;

		micRx  =   ( uint32_t )payload[size - LORAMAC_MFR_LEN];
		micRx |= ( ( uint32_t )payload[size - LORAMAC_MFR_LEN + 1] << 8 );
		micRx |= ( ( uint32_t )payload[size - LORAMAC_MFR_LEN + 2] << 16 );
		micRx |= ( ( uint32_t )payload[size - LORAMAC_MFR_LEN + 3] << 24 );	
				
		if(current_rx_pkt_index < last_rx_pkt_index)
		{
			seq_diff = (0x10000 - last_rx_pkt_index) + current_rx_pkt_index;
		}
		else
		{
			seq_diff = current_rx_pkt_index - last_rx_pkt_index;
		}
		
		temp_pkt_index += seq_diff;
		LoRaMacComputeMic( payload, size - LORAMAC_MFR_LEN, nwkSKey, address, DOWN_LINK, temp_pkt_index, &mic );
		if( micRx == mic )
		{
			isMicOk = true;
		}
		else
		{
			mcps_confirm_struct.status = MCPSCONFIRM_STATUS_RX_ERROR;
		}

		if( seq_diff >= MAX_FCNT_GAP )///bug
		{
			mcps_confirm_struct.status = MCPSCONFIRM_STATUS_MANY_FRAMES_LOSS;			
			goto RETURN_LAB;
		}

		if( isMicOk == true )
		{
			mcps_confirm_struct.status = MCPSCONFIRM_STATUS_RX_MIC_OK;
			
			mcps_confirm_struct.mcps_ind.multi_cast_flag = multicast_flag;
			mcps_confirm_struct.mcps_ind.fpending = fCtrl.bits.fpending;
			mcps_confirm_struct.mcps_ind.buffer = NULL;
			mcps_confirm_struct.mcps_ind.buffer_size = 0;
			/// Update 32 bits downlink counter
			if( multicast_flag == 1 )
			{
				mcps_confirm_struct.mcps_ind.Ind_type = MCPS_INDICATION_MULTICAST;
				goto RETURN_LAB;
			}
			else
			{
				if( macHdr.bits.m_type == FRAME_TYPE_DATA_CONFIRMED_DOWN )
				{
					server_need_ack_flag = true;
					mcps_confirm_struct.mcps_ind.Ind_type = MCPS_INDICATION_CONFIRMED;

					if( ( stack_rx_pkt_index == temp_pkt_index ) &&
						( stack_rx_pkt_index != 0 ) )
					{
						mcps_confirm_struct.status = MCPSCONFIRM_STATUS_RX_REPEATED;
						goto RETURN_LAB;;
					}
				}
				else
				{
					server_need_ack_flag = false;
					mcps_confirm_struct.mcps_ind.Ind_type = MCPS_INDICATION_UNCONFIRMED;

					if( ( stack_rx_pkt_index == temp_pkt_index ) &&
						( stack_rx_pkt_index != 0 ) )
					{
						mcps_confirm_struct.status = MCPSCONFIRM_STATUS_RX_REPEATED;
						goto RETURN_LAB;
					}
				}
				stack_rx_pkt_index = temp_pkt_index;
				last_rx_pkt_index = current_rx_pkt_index;
			}

			if( fCtrl.bits.ack == 1 )
			{
				mcps_confirm_struct.mcps_ind.rx_ack_flag = true;
			}
			else
			{
				mcps_confirm_struct.mcps_ind.rx_ack_flag = false;
			}

			if( ( ( size - 4 ) - appPayloadStartIndex ) > 0 )
			{
				///有FRM-PAYLOAD
				port = payload[appPayloadStartIndex++];
				frameLen = ( size - 4 ) - appPayloadStartIndex;

				mcps_confirm_struct.mcps_ind.port = port;

				if( port == 0 )
				{
					if( fCtrl.bits.fopts_len == 0 )
					{
						///接收到命令
						LoRaMacPayloadDecrypt( payload + appPayloadStartIndex,
											   frameLen,
											   nwkSKey,
											   address,
											   DOWN_LINK,
											   temp_pkt_index,
											   lora_mac_rx_pkt_buf );

						/// Decode frame payload MAC commands
						LoraMacProcessCmd( lora_mac_rx_pkt_buf, 0, frameLen, snr,size);
						mcps_confirm_struct.mcps_ind.rx_cmd_flag = true;
						mcps_confirm_struct.status = MCPSCONFIRM_STATUS_RX_DATA_OK;

						goto RETURN_LAB;
					}
				}
				else
				{
					if( fCtrl.bits.fopts_len > 0 )
					{
						///接收到命令
						LoraMacProcessCmd( payload, 8, appPayloadStartIndex - 1, snr,size);
						mcps_confirm_struct.mcps_ind.rx_cmd_flag = true;
					}

					LoRaMacPayloadDecrypt( payload + appPayloadStartIndex,
										   frameLen,
										   appSKey,
										   address,
										   DOWN_LINK,
										   temp_pkt_index,
										   lora_mac_rx_pkt_buf );

					mcps_confirm_struct.mcps_ind.buffer = lora_mac_rx_pkt_buf;
					mcps_confirm_struct.mcps_ind.buffer_size = frameLen;
					mcps_confirm_struct.mcps_ind.rx_data_flag = true;///接收到数据FRMpayload
					
					mcps_confirm_struct.status = MCPSCONFIRM_STATUS_RX_DATA_OK;
				}
			}
			else
			{///无FRM-PAYLOAD
				if( fCtrl.bits.fopts_len > 0 )
				{///接收到命令
					LoraMacProcessCmd( payload, 8, appPayloadStartIndex, snr,size);
					mcps_confirm_struct.mcps_ind.rx_cmd_flag = true;
					mcps_confirm_struct.status = MCPSCONFIRM_STATUS_RX_DATA_OK;
				}
			}
		}
	}
RETURN_LAB:

	if((node_device_class_type == CLASS_C))
    {
		///CLASS-C模式，转入接收模式
        OnRxWindow2TimerEvent( );
    }
	
	lora_mac_rxtx_flag = LORAMAC_RXTX_FLAG_RXDONE;
	
}

static void LoraMacStateCheck( void )
{
	if((lora_mac_rxtx_flag == LORAMAC_RXTX_FLAG_TXERROR)||
	   (lora_mac_rxtx_flag == LORAMAC_RXTX_FLAG_RXERROR)||
	   (lora_mac_rxtx_flag == LORAMAC_RXTX_FLAG_RXDONE))
    {
		node_adr_counter++;	
		
		if(node_need_ack_flag == true)
		{
			///确认包重传
			if((lora_mac_rxtx_flag == LORAMAC_RXTX_FLAG_RXERROR)||(lora_mac_rxtx_flag == LORAMAC_RXTX_FLAG_TXERROR))
			{
				if(node_need_ack_tx_counter < lora_work_para_struct.re_tx_num)
				{
					node_need_ack_tx_counter++;
					lora_mac_rxtx_flag = LORAMAC_RXTX_FLAG_NONE;
					LoraMacSendOnChannel();
					LoraTxLed(1);
					goto RET_LAB;
				}
			}
		}
		
		if((lora_mac_req_flag_struct.bits.mlme_req == 1)&&( mlme_confirm_struct.status == MLMECONFIRM_STATUS_JOIN_OK ))
		{
			stack_tx_pkt_index = 0;
		}
		else
		{
			if(lora_mac_req_flag_struct.bits.mcps_req == 1)
			{
				stack_tx_pkt_index++;
			}
		}
		
		if( lora_mac_req_flag_struct.bits.mlme_req == 1 )///mlme req
		{
			if(lora_mac_rxtx_flag == LORAMAC_RXTX_FLAG_TXERROR)
			{
				mlme_confirm_struct.status = MLMECONFIRM_STATUS_TX_TIMEOUT;
			}
			else if(lora_mac_rxtx_flag == LORAMAC_RXTX_FLAG_RXERROR)
			{
				mlme_confirm_struct.status = MLMECONFIRM_STATUS_RX_ERROR;
			}
			p_lora_mac_primitives->MacMlmeConfirm( &mlme_confirm_struct );
			lora_mac_req_flag_struct.bits.mlme_req = 0;
		}
		else
		{///mcps or class-b or class-c
			if(lora_mac_rxtx_flag == LORAMAC_RXTX_FLAG_TXERROR)
			{
				mcps_confirm_struct.status = MCPSCONFIRM_STATUS_TX_TIMEOUT;
			}
			else if(lora_mac_rxtx_flag == LORAMAC_RXTX_FLAG_RXERROR)
			{
				mcps_confirm_struct.status = MCPSCONFIRM_STATUS_RX_ERROR;
			}
			p_lora_mac_primitives->MacMcpsConfirm( &mcps_confirm_struct );
		}
		node_need_ack_flag = false;
		lora_mac_rxtx_flag = LORAMAC_RXTX_FLAG_NONE;
		lora_mac_exe_flag &= ~MAC_TX_RUNNING;
    }
RET_LAB:
	return;
}
///CLASS-B代码，计算PIN-SLOT,下行接收时间点
void LoraMacCountClassBPingSlot(uint32_t unix_sec)
{
	uint16_t i,tmp_rand;
	
	tmp_rand = LoRaMacClassBSlotRand(unix_sec,lora_mac_dev_addr);
	
	class_b_para.pingOffset = tmp_rand %(1 << (12-class_b_para.ping_nb));

	for(i=0;i<(1 << class_b_para.ping_nb);i++)
	{
		class_b_para.ping_slot[i] = 2120+(class_b_para.pingOffset+i*(1 << (12-class_b_para.ping_nb)))*30;
	}
}
///CLASS-B，毫秒定时器事件，主要是接收BEACON帧与PING-SLOT帧
void LoraMacClassBMsecEvent(void)
{
	uint32_t i,tmp_sec,tmp_msec;
	
	if(CLASS_B == (CLASS_MODE_T)lora_work_para_struct.class_mode)
	{
		class_b_para.no_rx_beacon_sec_counter++;
		if(class_b_para.no_rx_beacon_sec_counter >= 7200000)
		{
			class_b_para.no_rx_beacon_sec_counter = 0;
			lora_task_struct.rx_beacon_flag = false;
			if(node_device_class_type == CLASS_B)
			{
				node_device_class_type = CLASS_A;
			}
		}
		class_b_para.ms_counter++;
		if(class_b_para.ms_counter >= 128000)
		{
			class_b_para.ms_counter = 0;
			class_b_para.nwk_sec += 128;
		}
		
		if((node_join_network_flag)&&(class_b_para.rx_abs_nwk_time_flag))
		{
			tmp_sec = class_b_para.nwk_sec % 128;
			tmp_msec = tmp_sec*1000 + class_b_para.ms_counter + RADIO_WAKEUP_TIME;
			if(tmp_msec == 128000)
			{
				LoraMacCountClassBPingSlot(class_b_para.nwk_sec+128);
				if(lora_mac_exe_flag != MAC_TX_RUNNING)
				{
					///打开beacon接收窗口
					if(!lora_task_struct.classb_back_to_classa_flag)
					{
						LoraMacOpenClassBRxBeaconFrameWindow(false);
					}
				}
			}
			
			for(i=0;i<(1 << class_b_para.ping_nb);i++)
			{
				if(tmp_msec == class_b_para.ping_slot[i])
				{
					if(lora_mac_exe_flag != MAC_TX_RUNNING)
					{
						///打开slot接收窗口
						if(!lora_task_struct.classb_back_to_classa_flag)
						{
							LoraMacOpenClassBRxSlotWindow();
						}
					}
				}
			}
		}
	}
}
///CLASS-B 设置接收PING-SLOT帧参数，开启接收
void LoraMacOpenClassBRxSlotWindow(void)
{
	uint8_t  datarate = 0;
    uint16_t symb_timeout = 5; ///DR_2, DR_1, DR_0
	uint32_t rx_freq = 0;
	
    lora_mac_rx_slot = 2;
	rx_freq = LoraMacGetFreq(class_b_para.rxb_ch);
	
	if((class_b_para.rxb_dr & 0x40) == 0x40)
	{
		///使用rx1_dr
		if(lora_mac_rx_para_struct.rx1_dr >= lora_mac_rx_para_struct.rx1_dr_offset)
		{
			datarate = lora_mac_rx_para_struct.rx1_dr - lora_mac_rx_para_struct.rx1_dr_offset;
		}
		else
		{
			datarate = DR_0;
		}
	}
	else
	{
		///使用rxb_dr
		datarate = class_b_para.rxb_dr & 0x07;
	}
	
	if( ( datarate == DR_3 ) || ( datarate == DR_4 ) )
	{
		symb_timeout = 8;
	}
	else if( datarate == DR_5 )
	{
		symb_timeout = 10;
	}
	
	LoraMacSetRxChannel(rx_freq, datarate, 0, symb_timeout, false);
}
///CLASS-B 设置接收BEACON帧参数，开启接收
void LoraMacOpenClassBRxBeaconFrameWindow( uint8_t continue_flag )
{
    uint32_t rx_freq;
	
	lora_mac_rx_slot = 2;
	rx_freq = LoraMacGetFreq(class_b_para.rx_beacon_ch);
	Radio.SetChannel( rx_freq );
	///SF10,8symb
	Radio.SetRxConfig( MODEM_LORA, 0, 10, 1, 0, 8, 5, true, 17, false, 0, 0, false, continue_flag);
	
	Radio.SetMaxPayloadLength( MODEM_LORA, max_payload_array[DR_3] + LORA_MAC_FRMPAYLOAD_OVERHEAD );

    Radio.Rx(lora_mac_rx_para_struct.max_rx_window);
		
	class_b_para.wait_beacon_frame_flag = true;
}
///RX1接收
static void OnRxWindow1TimerEvent( void )
{
	uint8_t  datarate = 0,freq_index;
    uint16_t symb_timeout = 5; /// DR_2, DR_1, DR_0
    uint32_t rx_freq = 0;
	
    TimerStop( &rx_window1_timer );
    lora_mac_rx_slot = 0;
	
	if(lora_mac_rx_para_struct.rx1_dr >= lora_mac_rx_para_struct.rx1_dr_offset)
    {
		datarate = lora_mac_rx_para_struct.rx1_dr - lora_mac_rx_para_struct.rx1_dr_offset;
	}
	else
	{
		datarate = DR_0;
	}

    if( ( datarate == DR_3 ) || ( datarate == DR_4 ) )
    {
        symb_timeout = 8;
    }
    else if( datarate == DR_5 )
    {
        symb_timeout = 10;
    }

	if(lora_work_para_struct.claa_mode == CLAA_D)
	{
		freq_index = LoraMacGetFreqChan(lora_mac_rx_para_struct.rx1_freq,CLAA_D_UP_FREQ,48);
		freq_index = freq_index % 24;
		rx_freq = CLAA_D_DOWN_FREQ[freq_index];
	}
	else if(lora_work_para_struct.claa_mode == CLAA_E)
	{
		freq_index = LoraMacGetFreqChan(lora_mac_rx_para_struct.rx1_freq,CLAA_E_UP_FREQ,48);
		freq_index = freq_index % 24;
		rx_freq = CLAA_E_DOWN_FREQ[freq_index];
	}
	else
	{
		rx_freq = lora_mac_rx_para_struct.rx1_freq;
	}
    LoraMacSetRxChannel(rx_freq, datarate, 0, symb_timeout, false );
}
///RX2接收
static void OnRxWindow2TimerEvent( void )
{
	uint8_t  rx_continue_flag = false,datarate = 0;
    uint16_t symb_timeout = 5; ///DR_2, DR_1, DR_0
	uint32_t rx_freq = 0;
	
    TimerStop( &rx_window2_timer );
	
    lora_mac_rx_slot = 1;

	if( node_device_class_type == CLASS_A )
	{
		datarate = lora_mac_rx_para_struct.rx2_dr;
		rx_freq = lora_mac_rx_para_struct.rx2_freq;
	}
	else if( node_device_class_type == CLASS_C )
	{
		///持续接收
		rx_continue_flag = true;
		///信道
		if((class_c_para.rxc_ch & 0x80)==0x80)
		{
			///使用RX2_CH
			rx_freq = lora_mac_rx_para_struct.rx2_freq;
		}
		else
		{
			///使用RXC_CH
			rx_freq = LoraMacGetFreq(class_c_para.rxc_ch & 0x7F);
		}
		
		if((class_c_para.rxc_dr & 0x08) == 0x08)
		{
			///使用rx1_dr
			if(lora_mac_rx_para_struct.rx1_dr >= lora_mac_rx_para_struct.rx1_dr_offset)
			{
				datarate = lora_mac_rx_para_struct.rx1_dr - lora_mac_rx_para_struct.rx1_dr_offset;
			}
			else
			{
				datarate = DR_0;
			}
		}
		else
		{
			///使用rxc_dr
			datarate = class_c_para.rxc_dr & 0x07;
		}
	}
	
	if( ( datarate == DR_3 ) || ( datarate == DR_4 ) )
	{
		symb_timeout = 8;
	}
	else if( datarate == DR_5 )
	{
		symb_timeout = 10;
	}
	
	LoraMacSetRxChannel(rx_freq, datarate, 0, symb_timeout, rx_continue_flag);
}
///接收信道设置
static void LoraMacSetRxChannel( uint32_t freq, uint8_t datarate, uint32_t bandwidth, uint16_t timeout, bool rxContinuous )
{
    uint8_t downlinkDatarate = mac_data_rate_array[datarate];
    RadioModems_t modem;

    if( Radio.GetStatus( ) == RF_IDLE )
    {
        Radio.SetChannel( freq );
        mcps_confirm_struct.mcps_ind.rx_dr = datarate;

		modem = MODEM_LORA;

		Radio.SetRxConfig( modem, bandwidth, downlinkDatarate, 1, 0, 8, timeout, false, 0, false, 0, 0, true, rxContinuous );///倒数第2参数，iq反转

        Radio.SetMaxPayloadLength( modem, max_payload_array[datarate] + LORA_MAC_FRMPAYLOAD_OVERHEAD );

        if( rxContinuous == false )
        {
            Radio.Rx(lora_mac_rx_para_struct.max_rx_window);///接收3秒，超时，则产生接收错误
        }
        else
        {
            Radio.Rx( 0 ); ///持续接收
        }
    }
}
///数据包发送函数
LORA_MAC_STATUS_T LoraMacSend( LORA_MAC_HEADER_T *macHdr, uint8_t fport, void *buffer, uint16_t buffer_size )
{
    LORA_MAC_FRAME_CTRL_T fCtrl;
    LORA_MAC_STATUS_T status;

    fCtrl.value = 0;
    fCtrl.bits.fopts_len      = 0;
    fCtrl.bits.fpending      = 0;
    fCtrl.bits.ack           = false;
    fCtrl.bits.adr_ack_req     = lora_mac_adr_ack_req_flag;
    fCtrl.bits.adr           = node_adr_ctrl_flag;

	if(node_device_class_type == CLASS_B)
	{
		fCtrl.value |= 0X10;
	}
    status = LoraMacPrepareFrame( macHdr, &fCtrl, fport, buffer, buffer_size );
	if(status == LORAMAC_STATUS_OK)
    {
		mcps_confirm_struct.mcps_ind.rx_ack_flag = false;
		status = LoraMacSendOnChannel( );
	}

    return status;
}
///发送数据打包函数
LORA_MAC_STATUS_T LoraMacPrepareFrame( LORA_MAC_HEADER_T *macHdr, LORA_MAC_FRAME_CTRL_T *fCtrl, uint8_t fport, void *buffer, uint16_t buffer_size)
{
    uint8_t i,j,pktHeaderLen = 0;
    uint32_t mic = 0;
    const void* payload = buffer;
    uint8_t payloadSize = buffer_size;
    uint8_t framePort = fport;
	char str_char[20];
	uint8_t str_len;
	uint8_t tmp_buf[LORAMAC_PHY_MAXPAYLOAD];
	LORA_MAC_STATUS_T status = LORAMAC_STATUS_ERROR;
	
    lora_mac_tx_pkt_len = 0;

    if( buffer == NULL )
    {
        buffer_size = 0;
    }

    lora_mac_tx_pkt_buf[pktHeaderLen++] = macHdr->value;

    switch( macHdr->bits.m_type )
    {
        case FRAME_TYPE_JOIN_REQ:
            rx_window1_delay_time = lora_mac_rx_para_struct.JoinAcceptDelay1 - RADIO_WAKEUP_TIME;
            rx_window2_delay_time = lora_mac_rx_para_struct.JoinAcceptDelay2 - RADIO_WAKEUP_TIME;

            lora_mac_tx_pkt_len = pktHeaderLen;

            MemCpyRev( lora_mac_tx_pkt_buf + lora_mac_tx_pkt_len, p_lora_mac_app_eui, 8 );
            lora_mac_tx_pkt_len += 8;
            MemCpyRev( lora_mac_tx_pkt_buf + lora_mac_tx_pkt_len, p_lora_mac_dev_eui, 8 );
            lora_mac_tx_pkt_len += 8;

            lora_mac_dev_nonce = Radio.Random( );

            lora_mac_tx_pkt_buf[lora_mac_tx_pkt_len++] = lora_mac_dev_nonce & 0xFF;
            lora_mac_tx_pkt_buf[lora_mac_tx_pkt_len++] = ( lora_mac_dev_nonce >> 8 ) & 0xFF;

            LoRaMacJoinComputeMic( lora_mac_tx_pkt_buf, lora_mac_tx_pkt_len & 0xFF, p_lora_mac_app_key, &mic );

            lora_mac_tx_pkt_buf[lora_mac_tx_pkt_len++] = mic & 0xFF;
            lora_mac_tx_pkt_buf[lora_mac_tx_pkt_len++] = ( mic >> 8 ) & 0xFF;
            lora_mac_tx_pkt_buf[lora_mac_tx_pkt_len++] = ( mic >> 16 ) & 0xFF;
            lora_mac_tx_pkt_buf[lora_mac_tx_pkt_len++] = ( mic >> 24 ) & 0xFF;
			status = LORAMAC_STATUS_OK;
            break;
        case FRAME_TYPE_DATA_CONFIRMED_UP:
        case FRAME_TYPE_DATA_UNCONFIRMED_UP:

            rx_window1_delay_time = lora_mac_rx_para_struct.ReceiveDelay1 - RADIO_WAKEUP_TIME;
            rx_window2_delay_time = lora_mac_rx_para_struct.ReceiveDelay2 - RADIO_WAKEUP_TIME;

            if( server_need_ack_flag == true )
            {
                server_need_ack_flag = false;
                fCtrl->bits.ack = 1;
            }

            lora_mac_tx_pkt_buf[pktHeaderLen++] = ( lora_mac_dev_addr ) & 0xFF;
            lora_mac_tx_pkt_buf[pktHeaderLen++] = ( lora_mac_dev_addr >> 8 ) & 0xFF;
            lora_mac_tx_pkt_buf[pktHeaderLen++] = ( lora_mac_dev_addr >> 16 ) & 0xFF;
            lora_mac_tx_pkt_buf[pktHeaderLen++] = ( lora_mac_dev_addr >> 24 ) & 0xFF;

            lora_mac_tx_pkt_buf[pktHeaderLen++] = fCtrl->value;

            lora_mac_tx_pkt_buf[pktHeaderLen++] = stack_tx_pkt_index & 0xFF;
            lora_mac_tx_pkt_buf[pktHeaderLen++] = ( stack_tx_pkt_index >> 8 ) & 0xFF;
			
			if( ( payload != NULL ) && ( payloadSize > 0 ) )
			{
				if(mac_tx_cmd_buf_index != 0)
				{
					fCtrl->bits.fopts_len = mac_tx_cmd_buf_index;
					lora_mac_tx_pkt_buf[5] = fCtrl->value;
					for(i=0;i<mac_tx_cmd_buf_index;i++)
					{
						lora_mac_tx_pkt_buf[pktHeaderLen++] = mac_tx_cmd_buf[i];
					}
					mac_tx_cmd_buf_index = 0;
				}
			}
			else
			{
				if(mac_tx_cmd_buf_index!=0)
				{
					payloadSize = mac_tx_cmd_buf_index;
					payload = mac_tx_cmd_buf;
					mac_tx_cmd_buf_index = 0;
					framePort = 0;
				}
			}

            if( ( payload != NULL ) && ( payloadSize > 0 ) )
            {
                lora_mac_tx_pkt_buf[pktHeaderLen++] = framePort;

                if( framePort == 0 )
                {
                    LoRaMacPayloadEncrypt( (uint8_t* ) payload, payloadSize, lora_mac_nwk_skey, lora_mac_dev_addr, UP_LINK, stack_tx_pkt_index, tmp_buf );
                }
                else
                {
                    LoRaMacPayloadEncrypt( (uint8_t* ) payload, payloadSize, lora_mac_app_skey, lora_mac_dev_addr, UP_LINK, stack_tx_pkt_index, tmp_buf );
                }

                MemCpy( lora_mac_tx_pkt_buf + pktHeaderLen, tmp_buf, payloadSize );
            }
            lora_mac_tx_pkt_len = pktHeaderLen + payloadSize;

            LoRaMacComputeMic( lora_mac_tx_pkt_buf, lora_mac_tx_pkt_len, lora_mac_nwk_skey, lora_mac_dev_addr, UP_LINK, stack_tx_pkt_index, &mic );

            lora_mac_tx_pkt_buf[lora_mac_tx_pkt_len + 0] = mic & 0xFF;
            lora_mac_tx_pkt_buf[lora_mac_tx_pkt_len + 1] = ( mic >> 8 ) & 0xFF;
            lora_mac_tx_pkt_buf[lora_mac_tx_pkt_len + 2] = ( mic >> 16 ) & 0xFF;
            lora_mac_tx_pkt_buf[lora_mac_tx_pkt_len + 3] = ( mic >> 24 ) & 0xFF;

            lora_mac_tx_pkt_len += LORAMAC_MFR_LEN;
			status = LORAMAC_STATUS_OK;
            break;
        case FRAME_TYPE_PROPRIETARY:
            if( ( buffer != NULL ) && ( buffer_size > 0 ) )
            {
                MemCpy( lora_mac_tx_pkt_buf + pktHeaderLen, ( uint8_t* ) buffer, buffer_size );
                lora_mac_tx_pkt_len = pktHeaderLen + buffer_size;
				status = LORAMAC_STATUS_OK;
            }
            break;
        default:
            break;
    }

	if(status == LORAMAC_STATUS_OK)
	{
		LocalPrint("\r\nmote tx lora pkt:",strlen("\r\nmote tx lora pkt:"));
		for(j=0;j<lora_mac_tx_pkt_len;j++)
		{
			str_len = sprintf(str_char,"%02X ",lora_mac_tx_pkt_buf[j]);
			LocalPrint((uint8_t*)&str_char,str_len);
		}
		LocalPrint("\r\n",2);
	}
	return status;
}
///在选择的频点上发送数据包
LORA_MAC_STATUS_T LoraMacSendOnChannel(void)
{
    uint8_t datarate;
	uint8_t i,j,max_tx_count;
	uint32_t ms_delay,tx_time_on_air;
	
	if(lora_mac_rx_para_struct.rx1_dr > DR_5)
	{
		lora_mac_rx_para_struct.rx1_dr = DR_5;
	}
	
	datarate = mac_data_rate_array[lora_mac_rx_para_struct.rx1_dr];
	
	if(lora_work_para_struct.tx_pwr > LORAMAC_MAX_TX_POWER)
	{
		lora_work_para_struct.tx_pwr = LORAMAC_MAX_TX_POWER;
	}
	if(lora_work_para_struct.tx_pwr < LORAMAC_MIN_TX_POWER)
	{
		lora_work_para_struct.tx_pwr = LORAMAC_MIN_TX_POWER;
	}
	
    mlme_confirm_struct.status = MLMECONFIRM_STATUS_IDLE;
    mcps_confirm_struct.status = MCPSCONFIRM_STATUS_IDLE;

	Radio.SetMaxPayloadLength( MODEM_LORA, lora_mac_tx_pkt_len );
	Radio.SetTxConfig( MODEM_LORA,lora_work_para_struct.tx_pwr, 0, 0, datarate, 1, 8, false, true, 0, 0, false, 3e3 );

	tx_time_on_air = Radio.TimeOnAir( MODEM_LORA, lora_mac_tx_pkt_len );

	for(i=0;i<4;i++)
	{
		max_tx_count = Logic1Bits(lora_work_para_struct.channel_mask,10);
		for(j=0;j<max_tx_count;j++)
		{
			LoraMacRandTxFreq();
			Radio.SetChannel(lora_work_para_struct.tx_freq);
			SX1276StartCad();
			cad_done_flag = false;
			while(!cad_done_flag)
			{
				osDelay(2);
			}
			if(cad_channel_busy_flag)
			{
				LocalPrint("\r\n ch busy\r\n",strlen("\r\n ch busy\r\n"));
			}
			else
			{
				break;
			}
		}

		if(j == max_tx_count)
		{
			///csma-ca
			ms_delay = (2*mac_max_tx_time_array[lora_mac_rx_para_struct.rx1_dr]+8*tx_time_on_air)/10*(2^i)+
					  ((rand()%11)*lora_mac_rx_para_struct.rx1_dr*1024)/1000;
					  
			osDelay(ms_delay);
		}
		else
		{
			break;
		}
		
	}

	if(i == 4)
	{
		return LORAMAC_STATUS_ERROR;
	}
	
	lora_mac_rx_para_struct.rx1_freq = lora_work_para_struct.tx_freq;
			
    Radio.Send( lora_mac_tx_pkt_buf, lora_mac_tx_pkt_len );

    lora_mac_exe_flag |= MAC_TX_RUNNING;
	
	class_b_para.wait_beacon_frame_flag = false;

    return LORAMAC_STATUS_TX_OK;
}
///添加MAC命令的应答
void LoraMacAddMacCommand( uint8_t cmd, uint8_t p1, uint8_t p2 )
{
	switch( cmd )
    {
        case MOTE_MAC_LINK_CHECK_REQ:
            if( mac_tx_cmd_buf_index < LORA_MAC_COMMAND_MAX_LENGTH )
            {
                mac_tx_cmd_buf[mac_tx_cmd_buf_index++] = cmd;
            }
			break;
        case MOTE_MAC_LINK_ADR_ANS: /// Margin
            if( mac_tx_cmd_buf_index < ( LORA_MAC_COMMAND_MAX_LENGTH - 1 ) )
            {
                mac_tx_cmd_buf[mac_tx_cmd_buf_index++] = cmd;
                mac_tx_cmd_buf[mac_tx_cmd_buf_index++] = p1;
            }
            break;
        case MOTE_MAC_DUTY_CYCLE_ANS:
            if( mac_tx_cmd_buf_index < LORA_MAC_COMMAND_MAX_LENGTH )
            {
                mac_tx_cmd_buf[mac_tx_cmd_buf_index++] = cmd;
            }
            break;
        case MOTE_MAC_RX_PARAM_SETUP_ANS:
            if( mac_tx_cmd_buf_index < ( LORA_MAC_COMMAND_MAX_LENGTH - 1 ) )
            {
                mac_tx_cmd_buf[mac_tx_cmd_buf_index++] = cmd;
                mac_tx_cmd_buf[mac_tx_cmd_buf_index++] = p1;
            }
            break;
        case MOTE_MAC_DEV_STATUS_ANS: /// 1st byte Battery，2nd byte Margin
            if( mac_tx_cmd_buf_index < ( LORA_MAC_COMMAND_MAX_LENGTH - 2 ) )
            {
                mac_tx_cmd_buf[mac_tx_cmd_buf_index++] = cmd;
                mac_tx_cmd_buf[mac_tx_cmd_buf_index++] = p1;
                mac_tx_cmd_buf[mac_tx_cmd_buf_index++] = p2;
            }
            break;
        case MOTE_MAC_RX_TIMING_SETUP_ANS:
            if( mac_tx_cmd_buf_index < LORA_MAC_COMMAND_MAX_LENGTH )
            {
                mac_tx_cmd_buf[mac_tx_cmd_buf_index++] = cmd;
            }
			break;
		
		case MOTE_MAC_TIME_DEVICE_TIME_REQ:
			{
				if( mac_tx_cmd_buf_index < LORA_MAC_COMMAND_MAX_LENGTH )
				{
					mac_tx_cmd_buf[mac_tx_cmd_buf_index++] = cmd;
				}
				break;
			}
		case MOTE_MAC_CHECK_CHAN_INFO_REQ:
            if( mac_tx_cmd_buf_index < LORA_MAC_COMMAND_MAX_LENGTH )
            {
                mac_tx_cmd_buf[mac_tx_cmd_buf_index++] = cmd;
            }
			break;
		case MOTE_MAC_CHECK_CHAN_INFO_ANS:
            if( mac_tx_cmd_buf_index < LORA_MAC_COMMAND_MAX_LENGTH )
            {
                mac_tx_cmd_buf[mac_tx_cmd_buf_index++] = cmd;
                mac_tx_cmd_buf[mac_tx_cmd_buf_index++] = p1;
            }
			break;
		case MOTE_MAC_SW_AC_CLASS_REQ:
            if( mac_tx_cmd_buf_index < LORA_MAC_COMMAND_MAX_LENGTH )
            {
                mac_tx_cmd_buf[mac_tx_cmd_buf_index++] = cmd;
                mac_tx_cmd_buf[mac_tx_cmd_buf_index++] = p1;
            }
			break;
		case MOTE_MAC_SW_AC_CLASS_ANS:
			if( mac_tx_cmd_buf_index < LORA_MAC_COMMAND_MAX_LENGTH )
			{
				mac_tx_cmd_buf[mac_tx_cmd_buf_index++] = cmd;
				mac_tx_cmd_buf[mac_tx_cmd_buf_index++] = p1;
			}
		case MOTE_MAC_TIME_INFO_REQ:
		case MOTE_MAC_TIME_INFO_ANS:
		case MOTE_MAC_JUMBO_FRAME_REQ:
			if( mac_tx_cmd_buf_index < LORA_MAC_COMMAND_MAX_LENGTH )
			{
				mac_tx_cmd_buf[mac_tx_cmd_buf_index++] = cmd;
			}
		case MOTE_MAC_BEACON_FREQ_ANS:
			{
				if( mac_tx_cmd_buf_index < LORA_MAC_COMMAND_MAX_LENGTH )
				{
					mac_tx_cmd_buf[mac_tx_cmd_buf_index++] = cmd;
					mac_tx_cmd_buf[mac_tx_cmd_buf_index++] = p1;
				}
				break;
			}
		case MOTE_MAC_PING_SLOT_INFO_REQ:
			{
				if( mac_tx_cmd_buf_index < LORA_MAC_COMMAND_MAX_LENGTH )
				{
					mac_tx_cmd_buf[mac_tx_cmd_buf_index++] = cmd;
					mac_tx_cmd_buf[mac_tx_cmd_buf_index++] = p1;
				}
				break;
			}
        default:
            break;
    }
}
///处理MAC命令
void LoraMacProcessCmd( uint8_t *payload, uint8_t macIndex, uint8_t commandsSize, int8_t snr ,uint8_t pkt_len)
{
	uint8_t mac_cmd,data_rate,tx_pwr,ack_status,rx2_dr,rx1_dr_offset,array_tx_pwr[] = {20,14,11,8,5,2};
	uint8_t ch_mask_2_cntl,ch_mask_3_cntl,mask_1_L,mask_1_H;
	uint8_t mask_23_cntl,ch_mask_cntl,redundancy,mask_1_res,mask_23_res;
    uint8_t d_t[6],d_t_h,d_t_l,i,j;
	uint32_t beacon_freq;
	char str_char[20];
	uint8_t str_len;
	
	LocalPrint("\r\nmote rx mac cmd or cmd-ack:",strlen("\r\nmote rx mac cmd or cmd-ack:"));
	for(j=0;j<commandsSize;j++)
	{
		str_len = sprintf(str_char,"%02X ",payload[macIndex+j]);
		LocalPrint((uint8_t*)&str_char,str_len);
	}
	LocalPrint("\r\n",2);
	
	mac_cmd = payload[macIndex++];
	switch( mac_cmd )
	{
		case SRV_MAC_LINK_CHECK_ANS:
			mcps_confirm_struct.mcps_ind.demod_margin = payload[macIndex++];
			mcps_confirm_struct.mcps_ind.num_of_gateway = payload[macIndex++];
			lora_task_struct.rx_cmd_ack_flag = true;
			break;
		case SRV_MAC_LINK_ADR_REQ:
			{
				ack_status = 0x00;
				data_rate  = payload[macIndex] >> 4;
				tx_pwr     = payload[macIndex] & 0x0f;
				macIndex++;
				if(data_rate < 6)
				{
					lora_mac_rx_para_struct.rx1_dr = data_rate;;
					lora_work_para_struct.tx_sf = mac_data_rate_array[data_rate];
					ack_status |= 0x02;
				}	
				if(tx_pwr < 6)
				{
					lora_work_para_struct.tx_pwr = array_tx_pwr[tx_pwr];
					ack_status |= 0x04;
				}
				macIndex++;
				mask_1_L = payload[macIndex++];
				mask_1_H = payload[macIndex++];
				redundancy = payload[macIndex++];
				ch_mask_cntl = (redundancy >> 4)&0x07;
				if(ch_mask_cntl < 5)
				{
					lora_work_para_struct.channel_mask[ch_mask_cntl*2]  = mask_1_L;
					lora_work_para_struct.channel_mask[ch_mask_cntl*2+1] = mask_1_H;
					mask_1_res = true;
				}
				else
				{
					mask_1_res = false;
				}
				if((redundancy & 0x80) == 0x80)
				{
					///有扩展项
					mask_23_cntl = payload[macIndex++];
					ch_mask_2_cntl = mask_23_cntl & 0x07;
					ch_mask_3_cntl = (mask_23_cntl >> 3) & 0x07;
					if((ch_mask_2_cntl < 5)&&(ch_mask_3_cntl < 5))
					{
						lora_work_para_struct.channel_mask[ch_mask_2_cntl*2]  = payload[macIndex++];
						lora_work_para_struct.channel_mask[ch_mask_2_cntl*2+1] = payload[macIndex++];
						lora_work_para_struct.channel_mask[ch_mask_3_cntl*2]  = payload[macIndex++];
						lora_work_para_struct.channel_mask[ch_mask_3_cntl*2+1] = payload[macIndex++];
						mask_23_res = true;
					}
					else
					{
						mask_23_res = false;
					}
				}
				else
				{
					mask_23_res = true;
				}
				
				if(mask_1_res && mask_23_res)
				{
					ack_status |= 0x01;
				}
				LoraMacAddMacCommand( MOTE_MAC_LINK_ADR_ANS, ack_status, 0 );
			}
			break;
		case SRV_MAC_DUTY_CYCLE_REQ:
			LoraMacAddMacCommand( MOTE_MAC_DUTY_CYCLE_ANS, 0, 0 );
			break;
		case SRV_MAC_RX_PARAM_SETUP_REQ:
			{	   
				///	部分支持
				ack_status = 0x00;
				rx1_dr_offset = ( payload[macIndex] >> 4 ) & 0x07;
				rx2_dr = payload[macIndex] & 0x0F;
				macIndex += 4;
				if(rx1_dr_offset < 6)
				{
					lora_mac_rx_para_struct.rx1_dr_offset = rx1_dr_offset;
					ack_status |= 0x04;
				}
				if(rx2_dr < 6)
				{
					lora_mac_rx_para_struct.rx2_dr = rx2_dr;
					ack_status |= 0x02;
				}

				LoraMacAddMacCommand( MOTE_MAC_RX_PARAM_SETUP_ANS, ack_status, 0 );
			}
			break;
		case SRV_MAC_DEV_STATUS_REQ:
			{
				uint8_t batteryLevel = BAT_LEVEL_NO_MEASURE;
				if( ( p_lora_mac_callback != NULL ) && ( p_lora_mac_callback->GetBatteryLevel != NULL ) )
				{
					batteryLevel = p_lora_mac_callback->GetBatteryLevel();
				}
				LoraMacAddMacCommand( MOTE_MAC_DEV_STATUS_ANS, batteryLevel, snr );
				break;
			}
		case SRV_MAC_NEW_CHANNEL_REQ:
			{	
				///	不支持
				LoraMacAddMacCommand( MOTE_MAC_CHECK_CHAN_INFO_ANS, 0, 0 );
			}
			break;
		case SRV_MAC_RX_TIMING_SETUP_REQ:
			{
				uint8_t delay = payload[macIndex++];
				delay &= 0x0F;
				if( delay == 0 )
				{
					delay++;
				}
				lora_mac_rx_para_struct.ReceiveDelay1 = (uint32_t)(delay * 1e3);
				lora_mac_rx_para_struct.ReceiveDelay2 = lora_mac_rx_para_struct.ReceiveDelay1 + (uint32_t)(1e3);
				LoraMacAddMacCommand( MOTE_MAC_RX_TIMING_SETUP_ANS, 0, 0 );	
			}
			break;
		case SRV_MAC_TIME_DEVICE_TIME_ANS:
			{	
				lora_task_struct.rx_cmd_ack_flag = true;
				class_b_para.nwk_sec = U8ToUint32(payload+macIndex);
				macIndex += 4;
				class_b_para.ms_counter = (payload[macIndex])*39/10;
				class_b_para.ms_counter += Radio.TimeOnAir( MODEM_LORA, pkt_len);
				macIndex += 1;
				class_b_para.rx_abs_nwk_time_flag = true;
			}
			break;
		case SRV_MAC_CHECK_CHAN_INFO_REQ:
			{ 
				LoraMacAddMacCommand( MOTE_MAC_CHECK_CHAN_INFO_ANS, 0x07, 0 );
			}
		case SRV_MAC_CHECK_CHAN_INFO_ACK:
			{	
				///更新可用信道
				if(mac_cmd == SRV_MAC_CHECK_CHAN_INFO_ACK)
				{
					lora_task_struct.rx_cmd_ack_flag = true;
				}
				lora_work_para_struct.claa_mode = (CLAA_MODE_T)payload[macIndex];
				if(payload[macIndex] == CLAA_A)
				{
					MemCpy(lora_work_para_struct.channel_mask,payload+macIndex+1,9);
					lora_mac_rx_para_struct.rx2_freq = LoraMacGetFreq(payload[11]);
					macIndex += 12;
				}
				else if((payload[macIndex] == CLAA_B)||(payload[macIndex] == CLAA_C))
				{
					MemCpy(lora_work_para_struct.channel_mask,payload+macIndex+1,10);
					lora_mac_rx_para_struct.rx2_freq = LoraMacGetFreq(payload[11]);
					macIndex += 12;
				}
				else if((payload[macIndex] == CLAA_D)||(payload[macIndex] == CLAA_E))
				{
					MemCpy(lora_work_para_struct.channel_mask,payload+macIndex+1,6);
					
					if(CLASS_B == (CLASS_MODE_T)lora_work_para_struct.class_mode)
					{
						class_b_para.rxb_ch = LoraMacGetFreq(payload[7]);
					}
					else if(CLASS_C == (CLASS_MODE_T)lora_work_para_struct.class_mode)
					{
						class_c_para.rxc_ch = LoraMacGetFreq(payload[7]);
					}
					macIndex += 8;
				}
			}
			break;
		case SRV_MAC_SW_AC_CLASS_ANS:
			{	
				lora_task_struct.rx_cmd_ack_flag = true;
				lora_task_struct.rx_cmd_ack = payload[macIndex++];
			}
			break;
		case SRV_MAC_SW_AC_CLASS_REQ:
			{	
				if(CLASS_C == (CLASS_MODE_T)lora_work_para_struct.class_mode)
				{
					if(((payload[macIndex] & 0x03) == MOTE_SWT_CLASS_CA)||
					   ((payload[macIndex] & 0x03) == MOTE_SWT_CLASS_AC))
					{
						LoraMacAddMacCommand( MOTE_MAC_SW_AC_CLASS_ANS, 1, 0 );
						node_device_class_type = (CLASS_MODE_T)(payload[macIndex] & 0x03);
					}
					else
					{
						LoraMacAddMacCommand( MOTE_MAC_SW_AC_CLASS_ANS, 0, 0 );
					}
				}
				else
				{
					LoraMacAddMacCommand( MOTE_MAC_SW_AC_CLASS_ANS, 0, 0 );
				}
				macIndex++;
			}
			break;
		case SRV_MAC_TIME_INFO_REQ:
			{	
				LoraMacAddMacCommand( MOTE_MAC_TIME_INFO_ANS, 0, 0 );
			}
		case SRV_MAC_TIME_INFO_ANS:
			{
				lora_task_struct.rx_cmd_ack_flag = true;
				MemCpy(d_t,payload+macIndex,6);
				for(i=0;i<6;i++)
				{
					d_t_h = (d_t[i] >> 4)*10;
					d_t_l = d_t[i] & 0x0f;
					d_t[i] = d_t_h +d_t_l;
				}
				macIndex += 6;
			}
			break;
		case SRV_MAC_JUMBO_FRAME_ANS:
			{	
				lora_task_struct.rx_cmd_ack_flag = true;
				macIndex += 2;
			}
			break;
		
		case SRV_MAC_PING_SLOT_INFO_ANS:
			{
				class_b_para.ping_nb = lora_task_struct.ping_slot_param & 0x07;
				break;
			}
		case SRV_MAC_PING_SLOT_CHANNEL_REQ:
			{
				beacon_freq = payload[macIndex++];
				beacon_freq |= payload[macIndex++] << 16;
				beacon_freq |= payload[macIndex++] << 24;
				beacon_freq *= 100;
				if(lora_work_para_struct.claa_mode == CLAA_A)
				{
					class_b_para.rxb_ch = LoraMacGetFreqChan(beacon_freq,CLAA_A_ALL_FREQ,72);
				}
				else if(lora_work_para_struct.claa_mode == CLAA_B)
				{
					class_b_para.rxb_ch = LoraMacGetFreqChan(beacon_freq,CLAA_B_ALL_FREQ,80);
				}
				else if(lora_work_para_struct.claa_mode == CLAA_C)
				{
					class_b_para.rxb_ch = LoraMacGetFreqChan(beacon_freq,CLAA_C_ALL_FREQ,80);
				}
				else if(lora_work_para_struct.claa_mode == CLAA_D)
				{
					class_b_para.rxb_ch = LoraMacGetFreqChan(beacon_freq,CLAA_D_DOWN_FREQ,30);
				}
				else if(lora_work_para_struct.claa_mode == CLAA_E)
				{
					class_b_para.rxb_ch = LoraMacGetFreqChan(beacon_freq,CLAA_E_DOWN_FREQ,30);
				}
				class_b_para.rxb_dr = payload[macIndex++];
				LoraMacAddMacCommand( MOTE_MAC_BEACON_FREQ_ANS, 0X03, 0 );
				break;
			}
		case SRV_MAC_BEACON_FREQ_REQ:
			{
				beacon_freq = payload[macIndex++];
				beacon_freq |= payload[macIndex++] << 16;
				beacon_freq |= payload[macIndex++] << 24;
				beacon_freq *= 100;
				if(lora_work_para_struct.claa_mode == CLAA_A)
				{
					class_b_para.rx_beacon_ch = LoraMacGetFreqChan(beacon_freq,CLAA_A_ALL_FREQ,72);
				}
				else if(lora_work_para_struct.claa_mode == CLAA_B)
				{
					class_b_para.rx_beacon_ch = LoraMacGetFreqChan(beacon_freq,CLAA_B_ALL_FREQ,80);
				}
				else if(lora_work_para_struct.claa_mode == CLAA_C)
				{
					class_b_para.rx_beacon_ch = LoraMacGetFreqChan(beacon_freq,CLAA_C_ALL_FREQ,80);
				}
				else if(lora_work_para_struct.claa_mode == CLAA_D)
				{
					class_b_para.rx_beacon_ch = LoraMacGetFreqChan(beacon_freq,CLAA_D_DOWN_FREQ,30);
				}
				else if(lora_work_para_struct.claa_mode == CLAA_E)
				{
					class_b_para.rx_beacon_ch = LoraMacGetFreqChan(beacon_freq,CLAA_E_DOWN_FREQ,30);
				}
				
				LoraMacAddMacCommand( MOTE_MAC_BEACON_FREQ_ANS, 0X01, 0 );
				break;
			}
		default:
			break;
	}
}
///获取协议栈参数
LORA_MAC_STATUS_T LoRaMacMibGetRequestConfirm( MIB_REQUEST_CONFIRM_T *mibGet )
{
    LORA_MAC_STATUS_T status = LORAMAC_STATUS_OK;

    if( mibGet == NULL )
    {
        return LORAMAC_STATUS_PARA_NULL;
    }

    switch( mibGet->type )
    {
		case MIB_DEVICE_CLASS:
		{
			mibGet->param.class = node_device_class_type;
			break;
		}
        case MIB_NET_ID:
        {
            mibGet->param.NetID = lora_mac_net_id;
            break;
        }
        case MIB_DEV_ADDR:
        {
            mibGet->param.DevAddr = lora_mac_dev_addr;
            break;
        }

        default:
            status = LORAMAC_STATUS_MIB_CMD_ERROR;
            break;
    }

    return status;
}
///设置协议栈参数
LORA_MAC_STATUS_T LoRaMacMibSetRequestConfirm( MIB_REQUEST_CONFIRM_T *mibSet )
{
    LORA_MAC_STATUS_T status = LORAMAC_STATUS_OK;

    if( mibSet == NULL )
    {
        return LORAMAC_STATUS_PARA_NULL;
    }
	
    if( ( lora_mac_exe_flag & MAC_TX_RUNNING ) == MAC_TX_RUNNING )
    {
        return LORAMAC_STATUS_BUSY;
    }

    switch( mibSet->type )
    {
        case MIB_DEVICE_CLASS:
        {
            node_device_class_type = mibSet->param.class;
            switch( node_device_class_type )
            {
                case CLASS_A:
                {
                    Radio.Sleep( );
                    break;
                }
                case CLASS_B:
                {
                    break;
                }
                case CLASS_C:
                {
                    OnRxWindow2TimerEvent( );
                    break;
                }
            }
            break;
        }
        case MIB_ADR:
        {
            node_adr_ctrl_flag = mibSet->param.AdrEnable;
            break;
        }
        default:
            status = LORAMAC_STATUS_MIB_CMD_ERROR;
            break;
    }

    return status;
}
///初始化部分
void ClassbParaInit(void)
{
	if((lora_work_para_struct.claa_mode == CLAA_D)||(lora_work_para_struct.claa_mode == CLAA_E))
	{
		class_b_para.rx_beacon_ch = 24;
	}
}
void LoraMacParaInit(void)
{
	node_join_network_flag = false;///入网标志
    stack_tx_pkt_index =0;
    stack_rx_pkt_index = 0;
    node_adr_counter = 0;///adr发包计数
	lora_mac_adr_ack_req_flag = 0;
///    duty_cycle_2_power_val = 0;///占空比参数
///    lora_mac_duty_cycle_val = 1;
    mac_tx_cmd_buf_index = 0;///命令缓存下标
    server_need_ack_flag = false;///服务器需要应答标志
    lora_mac_rx_para_struct.max_rx_window = 3000;///最大接收窗口3秒
    lora_mac_rx_para_struct.ReceiveDelay1 = RECEIVE_DELAY1;
    lora_mac_rx_para_struct.ReceiveDelay2 = RECEIVE_DELAY2;
    lora_mac_rx_para_struct.JoinAcceptDelay1 = JOIN_ACCEPT_DELAY1;
    lora_mac_rx_para_struct.JoinAcceptDelay2 = JOIN_ACCEPT_DELAY2;
    lora_mac_rx_para_struct.rx1_dr_offset = 0;///接收窗口1，DR偏移
	
	if(lora_work_para_struct.tx_sf > LORAMAC_MAX_TX_SF)
	{
		lora_work_para_struct.tx_sf = LORAMAC_MAX_TX_SF;
	}
	if(lora_work_para_struct.tx_sf < LORAMAC_MIN_TX_SF)
	{
		lora_work_para_struct.tx_sf = LORAMAC_MIN_TX_SF;
	}
    lora_mac_rx_para_struct.rx1_dr = LORAMAC_MAX_TX_SF-lora_work_para_struct.tx_sf;
}

LORA_MAC_STATUS_T LoRaMacInitialization( LORA_MAC_PRIMITIVES_T *primitives, LORA_MAC_CALLBACK_T *callbacks )
{
    if( primitives == NULL )
    {
        return LORAMAC_STATUS_PARA_NULL;
    }

    if( ( primitives->MacMcpsConfirm == NULL ) || ( primitives->MacMlmeConfirm == NULL ))
    {
        return LORAMAC_STATUS_PARA_NULL;
    }

    p_lora_mac_primitives = primitives;
    p_lora_mac_callback = callbacks;
    lora_mac_req_flag_struct.value = 0;///LORA-MAC请求类型请空
    node_device_class_type = CLASS_A;///节点CLASS类型
    lora_mac_exe_flag = MAC_IDLE;
///	lora_mac_duty_cycle_on_flag = false; ///禁能占空比控制
    LoraMacParaInit();

    TimerInit( &rx_window1_timer, OnRxWindow1TimerEvent );
    TimerInit( &rx_window2_timer, OnRxWindow2TimerEvent );

    radio_event_struct.TxDone = OnRadioTxDone;///初始化SX1278相关部分
    radio_event_struct.RxDone = OnRadioRxDone;
	radio_event_struct.CadDone = OnRadioCadDone;
    radio_event_struct.RxError = OnRadioRxError;
    radio_event_struct.TxTimeout = OnRadioTxTimeout;
    radio_event_struct.RxTimeout = OnRadioRxTimeout;
    Radio.Init( &radio_event_struct );
	
	Radio.SetModem( MODEM_LORA );///设置公有网络
    Radio.Write( REG_LR_SYNCWORD, LORA_MAC_PUBLIC_SYNCWORD );
    Radio.Sleep( );///休眠

    return LORAMAC_STATUS_OK;
}
///数据包发送外部接口
LORA_MAC_STATUS_T LoRaMacMcpsRequest( MCPS_REQ_T *mcps_request )
{
    LORA_MAC_STATUS_T status;
    LORA_MAC_HEADER_T macHdr;
    uint8_t fport = 0;
    void *buffer;
    uint16_t buffer_size;

    if(mcps_request == NULL)
    {
        return LORAMAC_STATUS_PARA_NULL;
    }
    if((lora_mac_exe_flag & MAC_TX_RUNNING) == MAC_TX_RUNNING)
    {
        return LORAMAC_STATUS_BUSY;
    }
	if( node_join_network_flag == false )
	{
		return LORAMAC_STATUS_NOT_JOIN_NET; /// No network has been joined yet
	}
			
    macHdr.value = 0;
    MemSet ( ( uint8_t* ) &mcps_confirm_struct, 0, sizeof( mcps_confirm_struct ) );
    
	lora_mac_req_flag_struct.value = 0;
    switch( mcps_request->type )
    {
        case MCPS_UNCONFIRMED:
        {
            macHdr.bits.m_type = FRAME_TYPE_DATA_UNCONFIRMED_UP;
            fport = mcps_request->req.unconfirm.fport;

			buffer = mcps_request->req.unconfirm.buffer;
			buffer_size = mcps_request->req.unconfirm.buffer_size;
            break;
        }
        case MCPS_CONFIRMED:
        {
			node_need_ack_flag = true;
			node_need_ack_tx_counter = 0;
            macHdr.bits.m_type = FRAME_TYPE_DATA_CONFIRMED_UP;
            fport = mcps_request->req.confirm.fport;

			buffer = mcps_request->req.confirm.buffer;
			buffer_size = mcps_request->req.confirm.buffer_size;
            break;
        }
        case MCPS_PROPRIETARY:
        {
            macHdr.bits.m_type = FRAME_TYPE_PROPRIETARY;
            buffer = mcps_request->req.Proprietary.buffer;
            buffer_size = mcps_request->req.Proprietary.buffer_size;
            break;
        }
        default:
		{
			return LORAMAC_STATUS_PRO_CMD_ERROR;
		}
    }

	status = LoraMacSend( &macHdr, fport, buffer, buffer_size);
	if( status == LORAMAC_STATUS_TX_OK )
    {
        lora_mac_req_flag_struct.bits.mcps_req = 1;
    }
    return status;
}
///JOIN包发送，外部接口函数
LORA_MAC_STATUS_T LoRaMacMlmeRequest( MLME_REQ_T *mlme_request )
{
    LORA_MAC_STATUS_T status;
    LORA_MAC_HEADER_T macHdr;

    if(mlme_request == NULL)
    {
        return LORAMAC_STATUS_PARA_NULL;
    }
	
	if( ( mlme_request->Join.dev_eui == NULL ) ||
		( mlme_request->Join.app_eui == NULL ) ||
		( mlme_request->Join.app_key == NULL ) )
	{
		return LORAMAC_STATUS_PARA_NULL;
	}
	
	if( mlme_request->type != MLME_JOIN)
    {
		return LORAMAC_STATUS_PRO_CMD_ERROR;
    }
	
	lora_mac_exe_flag = MAC_IDLE;
	
    MemSet( ( uint8_t* ) &mlme_confirm_struct, 0, sizeof( mlme_confirm_struct ) );

	lora_mac_req_flag_struct.value = 0;;

	macHdr.value = 0;
	macHdr.bits.m_type  = FRAME_TYPE_JOIN_REQ;
	
	p_lora_mac_dev_eui = mlme_request->Join.dev_eui;
	p_lora_mac_app_eui = mlme_request->Join.app_eui;
	p_lora_mac_app_key = mlme_request->Join.app_key;

	LoraMacParaInit();
	LoraMacSetDefaultChanMask();
    ClassbParaInit();
	status = LoraMacSend( &macHdr, 0, NULL, 0 );
    if( status == LORAMAC_STATUS_TX_OK )
    {
        lora_mac_req_flag_struct.bits.mlme_req = 1;
    }
    return status;
}
///协议栈任务
void LoraMacTask(void)
{
	class_b_para.no_rx_beacon_sec_counter = 0;
	
	while(1)
	{	
		osDelay(100);
		LoraMacStateCheck();
	}
}
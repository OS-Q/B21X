
#ifndef __LORAMAC_H__
#define __LORAMAC_H__

	#include <stdbool.h>
    #include <stdint.h>
	#ifndef TIMERTIME_T
		#define TIMERTIME_T
		typedef uint32_t TimerTime_t;
	#endif
	
	#ifdef LORA_MAC_GLOBAL
		#define EXTERN_LORA_MAC_STRUCT
	#else
		#define EXTERN_LORA_MAC_STRUCT extern
	#endif
	
	#define LORA_MAC_COMMAND_MAX_LENGTH                 15
	#define LORA_MAC_FRMPAYLOAD_OVERHEAD                13 /// MHDR(1) + FHDR(7) + port(1) + MIC(4)
	#define LORAMAC_PHY_MAXPAYLOAD                  	255

	#define MAX_RE_TX_NUM                               8		///CONFIRM帧重发次数
	
	#define BEACON_INTERVAL                             128000	///beacon帧间隔，128秒
	#define RECEIVE_DELAY1                              1000	
	#define RECEIVE_DELAY2                              2000
	#define JOIN_ACCEPT_DELAY1                          5000
	#define JOIN_ACCEPT_DELAY2                          6000

	#define MAX_FCNT_GAP                                16384	///最大丢包数
	#define MAC_STATE_CHECK_TIMEOUT                     1000	///LORA MAC状态机轮询最大时间，1秒
	#define UP_LINK                                     0
	#define DOWN_LINK                                   1
	#define LORAMAC_MFR_LEN                             4
	#define LORA_MAC_PRIVATE_SYNCWORD                   0x12
	#define LORA_MAC_PUBLIC_SYNCWORD                    0x34

	#define DR_0                                        0  /// SF12 - BW125
	#define DR_1                                        1  /// SF11 - BW125
	#define DR_2                                        2  /// SF10 - BW125
	#define DR_3                                        3  /// SF9  - BW125
	#define DR_4                                        4  /// SF8  - BW125
	#define DR_5                                        5  /// SF7  - BW125
	
	#define LORAMAC_TX_MIN_DATARATE                     DR_0
	#define LORAMAC_TX_MAX_DATARATE                     DR_5
	
	#define LORAMAC_RX_MIN_DATARATE                     DR_0
	#define LORAMAC_RX_MAX_DATARATE                     DR_5

	#define LORAMAC_MIN_RX1_DR_OFFSET                   DR_0
	#define LORAMAC_MAX_RX1_DR_OFFSET                   DR_5
	
	#define LORAMAC_MIN_TX_SF                           7
	#define LORAMAC_MAX_TX_SF                           12
	
	#define LORAMAC_MIN_TX_POWER                        1
	#define LORAMAC_MAX_TX_POWER                        20

	#define CLASS_A_DEFAULT_MIN_TX_MSEC_INTERVAL		5
	#define CLASS_BC_DEFAULT_MIN_TX_MSEC_INTERVAL       15
	
	#define FREE_CHANNEL_RSSI_MAX_LIMIT  				(-95)
	#define CLAA_D_DOWN_DEFAULT_RX2_FREQ 				(502500000)
	#define CLAA_E_DOWN_DEFAULT_RX2_FREQ 				(492500000)
	#define MOTE_SWT_CLASS_CA 							0x01///CLASS-AC模式转换
	#define MOTE_SWT_CLASS_AC 							0x02
	
	typedef enum
	{
		CLASS_A,
		CLASS_B,
		CLASS_C,
	}CLASS_MODE_T;
	
	typedef enum
	{
		CLAA_A,
		CLAA_B,
		CLAA_C,
		CLAA_D,
		CLAA_E,
	}CLAA_MODE_T;

	typedef struct
	{
		uint8_t  rx_abs_nwk_time_flag;   ///获取绝对时间标志
		uint8_t  wait_beacon_frame_flag; ///等待BEACON帧标志
		
		uint8_t  rxb_ch;          		///rx_slot用信道
		uint8_t  rxb_dr;		  		///rx_slot用速率
		
		uint8_t  rx_beacon_ch;    		///rx_beacon用信道
		uint8_t  rx_beacon_dr;    		///rx_beacon用速率
		
		uint8_t  ping_nb;
		uint16_t pingOffset;
		
		uint32_t nwk_sec; 		  		///网络时间秒，class-b用
		uint32_t ms_counter;      		///ms计数,class-b用
		uint32_t no_rx_beacon_sec_counter;///没收到beacon毫秒计数
		uint32_t ping_slot[128];
	}CLASS_B_T;
	
	typedef struct
	{
		uint8_t  rxc_ch;
		uint8_t  rxc_dr;
	}CLASS_C_T;

	typedef struct
	{
		uint32_t rx1_freq;		///rx1信道参数
		uint8_t  rx1_dr;
		uint8_t  rx1_dr_offset;
		
		uint32_t rx2_freq;		///rx2信道参数
		uint8_t  rx2_dr;
		
		uint32_t max_rx_window; ///rx接收窗口参数
		uint32_t ReceiveDelay1;
		uint32_t ReceiveDelay2;
		uint32_t JoinAcceptDelay1;
		uint32_t JoinAcceptDelay2;
	}LORA_MAC_RX_PARA_T;

	typedef enum
	{
		FRAME_TYPE_JOIN_REQ              = 0x00,
		FRAME_TYPE_JOIN_ACCEPT           = 0x01,
		FRAME_TYPE_DATA_UNCONFIRMED_UP   = 0x02,
		FRAME_TYPE_DATA_UNCONFIRMED_DOWN = 0x03,
		FRAME_TYPE_DATA_CONFIRMED_UP     = 0x04,
		FRAME_TYPE_DATA_CONFIRMED_DOWN   = 0x05,
		FRAME_TYPE_RFU                   = 0x06,
		FRAME_TYPE_PROPRIETARY           = 0x07,
	}LORA_MAC_FRAME_TYPE_T;
	///LORA MAC 命令
	typedef enum
	{
		MOTE_MAC_LINK_CHECK_REQ          = 0x02,
		MOTE_MAC_LINK_ADR_ANS            = 0x03,
		MOTE_MAC_DUTY_CYCLE_ANS          = 0x04,
		MOTE_MAC_RX_PARAM_SETUP_ANS      = 0x05,
		MOTE_MAC_DEV_STATUS_ANS          = 0x06,
		MOTE_MAC_NEW_CHANNEL_ANS         = 0x07,
		MOTE_MAC_RX_TIMING_SETUP_ANS     = 0x08,
		/**未实现
		MOTE_MAC_TX_PARAM_SETUP_ANS      = 0x09,
		MOTE_MAC_DL_CHANNEL_ANS          = 0x0A,
		MOTE_MAC_RESET_IND               = 0x0B,
		MOTE_MAC_ADR_PARAM_SETUP_ANS     = 0x0c,
		
		MOTE_MAC_FORCE_REJOIN_ANS        = 0x0e,
		MOTE_MAC_REJOIN_PARAM_SETUP_ANS  = 0x0f,
		**/
		MOTE_MAC_TIME_DEVICE_TIME_REQ    = 0x0d,
		
		MOTE_MAC_PING_SLOT_INFO_REQ      = 0x10,///CLASS-B
		MOTE_MAC_PING_SLOT_CHANNEL_ANS   = 0x11,
		MOTE_MAC_BEACON_TIME_REQ         = 0x12,
		MOTE_MAC_BEACON_FREQ_ANS         = 0x13,
		
		MOTE_MAC_DEVICE_MODE_IND         = 0x20,///CLASS-C
		MOTE_MAC_SW_AC_CLASS_ANS         = 0x90,
		MOTE_MAC_SW_AC_CLASS_REQ         = 0x91,
		
		MOTE_MAC_CHECK_CHAN_INFO_REQ     = 0x8a,
		MOTE_MAC_CHECK_CHAN_INFO_ANS     = 0x8b,
		
		MOTE_MAC_TIME_INFO_ANS    		 = 0x8C,
		MOTE_MAC_TIME_INFO_REQ    		 = 0x8d,
		MOTE_MAC_JUMBO_FRAME_REQ     	 = 0x8e,
	}LORA_MAC_MOTE_CMD_T;
	
	typedef enum
	{
		SRV_MAC_LINK_CHECK_ANS           = 0x02,
		SRV_MAC_LINK_ADR_REQ             = 0x03,
		SRV_MAC_DUTY_CYCLE_REQ           = 0x04,
		SRV_MAC_RX_PARAM_SETUP_REQ       = 0x05,
		SRV_MAC_DEV_STATUS_REQ           = 0x06,
		SRV_MAC_NEW_CHANNEL_REQ          = 0x07,
		SRV_MAC_RX_TIMING_SETUP_REQ      = 0x08,
		/**
		SRV_MAC_TX_PARAM_SETUP_REQ       = 0x09,
		SRV_MAC_DL_CHANNEL_REQ           = 0x0A,
		SRV_MAC_RESET_CONF               = 0x0B,
		SRV_MAC_ADR_PARAM_SETUP_REQ      = 0x0c,
		
		SRV_MAC_FORCE_REJOIN_REQ         = 0x0e,
		SRV_MAC_REJOIN_PARAM_SETUP_REQ   = 0x0f,
		**/
		SRV_MAC_TIME_DEVICE_TIME_ANS     = 0x0d,
		
		SRV_MAC_PING_SLOT_INFO_ANS       = 0x10,///CLASS-B
		SRV_MAC_PING_SLOT_CHANNEL_REQ    = 0x11,
		SRV_MAC_BEACON_TIME_ANS          = 0x12,
		SRV_MAC_BEACON_FREQ_REQ          = 0x13,
		
		SRV_MAC_DEVICE_MODE_CONF         = 0x20,///CLASS-C
		SRV_MAC_SW_AC_CLASS_REQ          = 0x90,
		SRV_MAC_SW_AC_CLASS_ANS          = 0x91,
		
		SRV_MAC_CHECK_CHAN_INFO_ACK      = 0x8a,
		SRV_MAC_CHECK_CHAN_INFO_REQ      = 0x8b,

		SRV_MAC_TIME_INFO_REQ    		 = 0x8c,
		SRV_MAC_TIME_INFO_ANS    		 = 0x8d,
		SRV_MAC_JUMBO_FRAME_ANS     	 = 0x8e,
	}LORA_MAC_SRV_CMD_T;
	
	typedef enum
	{
		BAT_LEVEL_EXT_SRC                = 0x00,
		BAT_LEVEL_EMPTY                  = 0x01,
		BAT_LEVEL_FULL                   = 0xFE,
		BAT_LEVEL_NO_MEASURE             = 0xFF,
	}LORA_MAC_BAT_LEVEL_T;
	
	typedef union
	{
		uint8_t value;
		struct hdr_bits
		{
			uint8_t major           : 2;
			uint8_t rfu             : 3;
			uint8_t m_type          : 3;
		}bits;
	}LORA_MAC_HEADER_T;
	///LORA帧控制域
	typedef union
	{
		uint8_t value;
		struct ctrl_bits
		{
			uint8_t fopts_len        	: 4;
			uint8_t fpending        	: 1;
			uint8_t ack             	: 1;
			uint8_t adr_ack_req       	: 1;
			uint8_t adr             	: 1;
		}bits;
	}LORA_MAC_FRAME_CTRL_T;
	
	///MCPS，数据收发状态机
	typedef enum
	{
		MCPSCONFIRM_STATUS_IDLE,				
		MCPSCONFIRM_STATUS_TX_TIMEOUT,
		MCPSCONFIRM_STATUS_RX_DATA_OK,
		MCPSCONFIRM_STATUS_RX_DONE_NONE,
		MCPSCONFIRM_STATUS_RX_MIC_OK,
		MCPSCONFIRM_STATUS_RX_NO_REQ,
		MCPSCONFIRM_STATUS_RX_MULTICAST_REPEATED,
		MCPSCONFIRM_STATUS_RX_REPEATED,
		MCPSCONFIRM_STATUS_MANY_FRAMES_LOSS,
		MCPSCONFIRM_STATUS_RX_ERROR,
	}MCPS_CONFIRM_STATUS_T;
	///MLME，链路管理状态机
	typedef enum
	{
		MLMECONFIRM_STATUS_IDLE,
		MLMECONFIRM_STATUS_TX_TIMEOUT,
		MLMECONFIRM_STATUS_RX_DONE_NONE,
		MLMECONFIRM_STATUS_RX_ERROR,
		MLMECONFIRM_STATUS_JOIN_OK,
	}MLMECONFIRM_STATUS_T;

	typedef union
	{
		uint8_t value;
		struct mac_flag_bit
		{
			uint8_t mcps_req         : 1;
			uint8_t mlme_req         : 1;
		}bits;
	}LORA_MAC_REQ_FLAG_T;

	typedef enum
	{
		MCPS_UNCONFIRMED,
		MCPS_CONFIRMED,
		MCPS_MULTICAST,
		MCPS_PROPRIETARY,
	}MCPS_T;
	
	typedef struct
	{
		uint8_t 	fport;
		uint8_t 	data_rate;
		uint16_t 	buffer_size;
		void 		*buffer;
	}MCPS_REQ_UNCONFIRM_T;
	
	typedef struct
	{
		uint8_t fport;
		uint8_t data_rate;
		uint16_t buffer_size;
		void 	*buffer;
	}MCPS_REQ_CONFIRM_T;
	
	typedef struct
	{
		uint8_t data_rate;
		uint16_t buffer_size;
		void *buffer;
	}MCPS_REQ_PROPRIETARY_T;
	
	typedef struct
	{
		MCPS_T type;
		union req_t
		{
			MCPS_REQ_UNCONFIRM_T 	unconfirm;
			MCPS_REQ_CONFIRM_T   	confirm;
			MCPS_REQ_PROPRIETARY_T 	Proprietary;
		}req;
	}MCPS_REQ_T;
	
	typedef enum
	{
		MCPS_INDICATION_NONE,
		MCPS_INDICATION_PROPRIETARY,
		MCPS_INDICATION_CONFIRMED,
		MCPS_INDICATION_UNCONFIRMED,
		MCPS_INDICATION_MULTICAST,
	}MCPS_IND_TYPE_T;
	
	typedef struct
	{
		MCPS_IND_TYPE_T Ind_type;
		uint8_t multi_cast_flag;
		uint8_t port;
		uint8_t rx_dr;
		uint8_t fpending;
		uint8_t *buffer;
		uint8_t buffer_size;
		uint8_t	rx_data_flag;
		uint8_t	rx_cmd_flag;
		uint8_t	rx_ack_flag;
		int16_t rssi;
		int16_t snr;
		uint8_t rx_slot;
		uint8_t demod_margin;
		uint8_t num_of_gateway;
	}MCPS_INDICATION_T;
	
	typedef struct
	{
		MCPS_CONFIRM_STATUS_T 	status;
		MCPS_INDICATION_T 		mcps_ind;
	}MCPS_CONFIRM_T;
	
	typedef enum
	{
		MLME_JOIN,
	}MLME_T;
	
	typedef struct
	{
		uint8_t *dev_eui;
		uint8_t *app_eui;
		uint8_t *app_key;
	}MLME_JOIN_T;
	
	typedef struct
	{
		MLME_T type;
		MLME_JOIN_T Join;
	}MLME_REQ_T;
	
	typedef struct
	{
		MLMECONFIRM_STATUS_T status;
	}MLME_CONFIRM_T;
	
	typedef enum
	{
		MIB_DEVICE_CLASS,
		MIB_ADR,
		MIB_NET_ID,
		MIB_DEV_ADDR,
	}MIB_T;

	typedef struct
	{
		CLASS_MODE_T class;
		uint8_t 	AdrEnable;
		uint32_t 	NetID;
		uint32_t 	DevAddr;
	}MIB_PARAM_T;
	
	typedef struct
	{
		MIB_T type;
		MIB_PARAM_T param;
	}MIB_REQUEST_CONFIRM_T;
	
	typedef enum eLoRaMacStatus
	{
		LORAMAC_STATUS_OK,
		LORAMAC_STATUS_ERROR,
		LORAMAC_STATUS_BUSY,
		LORAMAC_STATUS_TX_OK,
		LORAMAC_STATUS_TX_DELAYED,
		LORAMAC_STATUS_TX_CMD_FULL,
		LORAMAC_STATUS_NOT_JOIN_NET,
		LORAMAC_STATUS_PARA_NULL,
		LORAMAC_STATUS_PRO_CMD_ERROR,
		LORAMAC_STATUS_MIB_CMD_ERROR,
		LORAMAC_STATUS_MIB_PARA_ERROR,
		LORAMAC_STATUS_DEVICE_SHUT_OFF,
	}LORA_MAC_STATUS_T;
	
	typedef enum
	{
		MAC_IDLE,
		MAC_TX_RUNNING,
		MAC_TX_CONFIG,
	}LORA_MAC_TX_STATE_T;

	typedef enum
	{
		LORAMAC_RXTX_FLAG_NONE,
		LORAMAC_RXTX_FLAG_TXERROR,
		LORAMAC_RXTX_FLAG_RXDONE,
		LORAMAC_RXTX_FLAG_RXERROR,
	}LORA_MAC_RX_TX_FLAG_T;

	typedef struct
	{
		void ( *MacMcpsConfirm )( MCPS_CONFIRM_T *McpsConfirm );
		void ( *MacMlmeConfirm )( MLME_CONFIRM_T *MlmeConfirm );
	}LORA_MAC_PRIMITIVES_T;

	typedef struct
	{
		uint8_t ( *GetBatteryLevel )( void );
	}LORA_MAC_CALLBACK_T;
	
	extern const uint32_t CLAA_A_ALL_FREQ[];
	extern const uint32_t CLAA_B_ALL_FREQ[];
	extern const uint32_t CLAA_C_ALL_FREQ[];
	extern const uint32_t CLAA_D_UP_FREQ[];
	extern const uint32_t CLAA_E_UP_FREQ[];
	
	LORA_MAC_STATUS_T 	LoraMacSend( LORA_MAC_HEADER_T *macHdr, uint8_t fport, void *buffer, uint16_t buffer_size );
	LORA_MAC_STATUS_T 	LoraMacScheduleSend(void);
	LORA_MAC_STATUS_T 	LoraMacSendOnChannel(void);
	LORA_MAC_STATUS_T 	LoRaMacMibGetRequestConfirm( MIB_REQUEST_CONFIRM_T *mibGet );
	LORA_MAC_STATUS_T 	LoRaMacMibSetRequestConfirm( MIB_REQUEST_CONFIRM_T *mibSet );
	LORA_MAC_STATUS_T 	LoRaMacMlmeRequest( MLME_REQ_T *mlme_request );
	LORA_MAC_STATUS_T 	LoRaMacMcpsRequest( MCPS_REQ_T *mcps_request );
	LORA_MAC_STATUS_T 	LoRaMacInitialization( LORA_MAC_PRIMITIVES_T *primitives, LORA_MAC_CALLBACK_T *callbacks );


	uint8_t 			LoraMacFreqInRange( uint32_t freq );	
	uint8_t 			LoraMacGetMaxTxSize(uint8_t sf);
	uint8_t 			LoraMacGetFreqChan(uint32_t freq,const uint32_t *freq_array,uint8_t array_size);
	void            	LoraMacAddMacCommand( uint8_t cmd, uint8_t p1, uint8_t p2 );

	void 				LoraMacSetScanNetChanMask(void);///搜网用
	
	void 				LoraMacOpenClassBRxBeaconFrameWindow( uint8_t continue_flag );///class-b
	void 				LoraMacOpenClassBRxSlotWindow(void);
	void 				LoraMacCountClassBPingSlot(uint32_t unix_sec);
	void 				LoraMacClassBMsecEvent(void);

	void 				LoraMacTask(void);
#endif

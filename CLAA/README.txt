中兴克拉 LORA-MOTE STACK V1.0.0

本协议栈基于STM32F407开发

(一)协议栈移植驱动方面：
1.实现SX1278，DIO0和DIO1的引脚中断，主要用于发送完成中断、接收完成中断、CAD检测中断；
2.实现1ms定时器中断，CLAA协议栈使用，该中断优先级高于其它中断；
  本协议栈中用了2个定时器，其中systick用于rx1、rx2、超时中断等，另一个TIM6，供CLASS-B使用。
  
(二)协议栈执行流程说明

1.主程序
LoraParaInit()->LoRaMacInit()->LoraMoteJoinNet()->LoraMoteNetTest();
2.收发过程
(1)JOIN & JOIN ACCEPT
发送：LoraMoteJoinNet()->LoRaMacJoinReq()->LoRaMacMlmeRequest()->LoraMacSend()->LoraMacPrepareFrame()->
      LoraMacSendOnChannel()->SX1276SetTxConfig()->SX1276Send()->SX1276SetTx()
接收：SX1276OnDio0Irq()->OnRadioTxDone()->OnRxWindow1TimerEvent()->SX1276OnDio0Irq()->OnRadioRxDone()->
      LoraMacStateCheck()->LoraMlmeConfirm()
(2)数据包发送与接收
发送：LoRaMacMcpsRequest()->LoraMacSend()->LoraMacPrepareFrame()->LoraMacSendOnChannel()->SX1276SetTxConfig()->
      SX1276Send()->SX1276SetTx()
接收：SX1276OnDio0Irq()->OnRadioTxDone()->OnRxWindow1TimerEvent()->SX1276OnDio0Irq()->OnRadioRxDone()->
      LoraMacStateCheck()->LoraMcpsConfirm()
(三)实应用更改 LoraParaInit()函数中的dev_eui[],app_eui[],app_key[]；
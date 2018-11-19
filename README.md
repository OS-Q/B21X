# W18:[LoRa通信节点](https://github.com/OS-Q/W18) 

基于STM32的LoRa入网通信系统方案

[![sites](OS-Q/OS-Q.png)](http://www.OS-Q.com)

#### 更多关于：[M5长距无线通信](https://github.com/OS-Q/M5) 可访问 www.OS-Q.com

---

## 简介

LoRaWAN 将终端设备划分成A/B/C三类：

- Class A：双向通信终端设备。这一类的终端设备允许双向通信，每一个终端设备上行传输会伴随着两个下行接收窗口。终端设备的传输时隙是基于其自身通信需求，其微调基于ALOHA协议。
- Class B：具有预设接收时隙的双向通信终端设备。终端设备会在预设时间中开放多余的接收窗口，为了达到这一目的，终端设备会同步从网关接收一个Beacon，通过Beacon将基站与模块的时间进行同步。
- Class C：具有最大接收窗口的双向通信终端设备。这一类的终端设备持续开放接收窗口，只在传输时关闭。


---

## 组成

#### LoRaMac

不同节点类型的keil工程

---

为锻造最美之器

##  www.OS-Q.com   |   qitas@qitas.cn


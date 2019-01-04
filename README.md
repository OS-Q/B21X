# D144：[LoRa MESH](https://github.com/OS-Q/D144) 

[![sites](OS-Q/OS-Q.png)](http://www.OS-Q.com)

#### 归属智能组网：[W21](https://github.com/OS-Q/W21)

#### 关于系统架构：[OS-Q](https://github.com/OS-Q/OS-Q)

## [节点描述](https://github.com/OS-Q/D144/wiki) 

D144 LoRa MESH 组网设备，通过LoRaWAN定义协议完成自组网通信

LoRaWAN 将终端设备划分成A/B/C三类：

- Class A：双向通信终端设备。这一类的终端设备允许双向通信，每一个终端设备上行传输会伴随着两个下行接收窗口。终端设备的传输时隙是基于其自身通信需求，其微调基于ALOHA协议。
- Class B：具有预设接收时隙的双向通信终端设备。终端设备会在预设时间中开放多余的接收窗口，为了达到这一目的，终端设备会同步从网关接收一个Beacon，通过Beacon将基站与模块的时间进行同步。
- Class C：具有最大接收窗口的双向通信终端设备。这一类的终端设备持续开放接收窗口，只在传输时关闭。


### [资源](OS-Q/)

#### LoRaMac

不同节点类型的keil工程

#### CLAA

ZTE CLAA LORA-MOTE STACK V1.0.0


---

- 边缘设备命名规则：体系 Q:[1,4] -> 节点 M:[1,12] -> 平台 W:[1,52] -> 设备 D:[1,365]

- naming patterns：system Q[1,4] -> node M[1,12] -> platform W[1,52] -> device D[1,365]

## [同级设备](https://github.com/OS-Q/W21/wiki) 

#### D141：[BLE MESH](https://github.com/OS-Q/D141)

基于网络泛洪管理的协议，包括中继功能、低功耗功能、友邻功能和代理功能

#### D142：[ZigBee](https://github.com/OS-Q/D142)

基于IEEE802.15.4标准的低功耗个域网协议，支持3个频段

#### D143：[Thread](https://github.com/OS-Q/D143)

基于简化版IPv6的网状网络协议,向下兼容ZigBee协议

#### -> D144：[LoRa MESH](https://github.com/OS-Q/D144)

LoRaWAN  将终端设备划分成A/B/C，可以使用star（星型）网络拓扑

#### D145：[NULL](https://github.com/OS-Q/D145)


#### D146：[NULL](https://github.com/OS-Q/D146)


#### D147：[NULL](https://github.com/OS-Q/D147)

---

####  © qitas@qitas.cn
###  [OS-Q redefined Operation System](http://www.OS-Q.com)
####   @  2019-1-4

# PLC DEMO README

## 文件说明

用于PLC设备相关测试命令演示，目前支持OPCUA协议对PLC进行远程控制，该命令基于LWIP和OPCUA，需要开启相关开关。

多个PLC设备可以组成一个channel，用于一条相关业务线控制。

### 命令行

ShowChannel

显示注册到channel上的PLC设备，范例如下：

 ch_type        ch_name        drv_name       dev_name       cnt                 
-----------------------------------------------------------------
 PLC_Channel    PLC            OPCUA          PLC Demo 4      1   
                                              PLC Demo 3      2   
                                              PLC Demo 2      3   
                                              PLC Demo 1      4   
                                              PLC Demo 0      5  

ShowPLC

用于显示PLC，范例如下：

 device         vendor         model          product        id                  
-----------------------------------------------------------------
 PLC Demo 4     B&R            X20            X20 CP1381     5                   
 PLC Demo 3     B&R            X20            X20 CP1586     4                   
 PLC Demo 2     SIEMSNS        S7-200         CPU SR60       3                   
 PLC Demo 1     SIEMENS        S7-1200        CPU 1215C      2                   
 PLC Demo 0     SIEMENS        S7-1500        CPU 1512SP-1PN 1      
 
 PlcRead [NodeID] 
 
 用于读取PLC节点信息

 - [NodeID]: 如n4,1, 其中4代表namespace，1代表节点号
 
 
 PlcWrite
 
 用于写入PLC节点数值
 
 - [NodeID]: 如n4,1, 其中4代表namespace，1代表节点号

 - [value]: 为写入数值，目前支持bool类型，和int类型。bool型应为0b(代表false), 1b(代表true)

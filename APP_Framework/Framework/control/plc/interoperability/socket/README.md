# PLC SOCKET README

## 文件说明

用于测试PLC socket通信. 通过建立与制定IP的PLC设备的socket连接, 发送命令给PLC设备, 实现相关功能. 实现该功能需要开启LWIP, 同时需要扩大shell的栈大小和内存空间。

### 命令行

PLCSocket ip=[PLC IP] port=[PLC port] tcp=[1: TCP; 0: UDP] cmd=[相关命令] file=[制定配置文件]

配置文件支持json格式, 默认文件名为socket_param.json, 放置于plc目录下, 文件内容如下：

{
   "ip": "192.168.250.6", 
   "port": 102,
   "tcp": 1,
   "cmd": [x, x, x]
}
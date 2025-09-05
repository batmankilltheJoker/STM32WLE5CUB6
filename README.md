# STM32WLE5CUB6 # ZL401 # ZL800
记录 LoRaWAN 的学习

开发板：STM32WLE5CUB6
网关：ZL800

现有功能包括：
  1. 使用CUB6采集温湿度（DS18B20）数据，LoRa传输至网关，并上传云平台
  2. 使用CUB6控制脉冲阀门 开/关 和 定时开/关，并上传至网关状态，云平台可控
  3. lora透传
  4. PingPong
  5. AT (AT将在后续上传)


!!!
  第一次拉取代码 或者 使用CubMx重新编译后，需要手动添加BSP
  对于终端节点，注意看文件中的readme


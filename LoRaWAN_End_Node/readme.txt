！！！
	最初编译可能会报错，需要手动添加BSP
！！！

！！！
	若使用中心频点，lorawan_conf.h 中 将 HYBRID_ENABLED 的宏定义修改为1
！！！

！！！
	重新生代码后，必须将stm32wlxx_hal_pwr_ex.c中 函数HAL_PWREx_EnterSTOP2Mode 中关于休眠代码的注释!!!否则会造成睡死现象
	/* Request Wait For Interrupt */
    //__WFI();
！！！

1. 实现终端采集 DS18B20的数据，并上传至网关
2. 重定义的printf 是串口1
3. se-identity.h文件中可修改：DEVICE_EUI、JOIN_EUI、APP_KEY、NWK_KEY等参数
4. RegionCN470.h文件中可修改：SF、 TX_POWER、BW等参数
5. channel_select.h文件中可修改关于中心频段与其偏移的参数，
6. lora_app.h中，修改LORAWAN_DEFAULT_CONFIRMED_MSG_STATE，决定是否启用 confirmed message


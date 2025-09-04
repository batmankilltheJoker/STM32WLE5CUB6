！！！
	最初编译可能会报错，需要手动添加BSP
！！！

！！！
	重新使用CubMx生成代码后，需要修改RadioSend，添加延时，避免发送入网请求时，发送过快导致mic校验错误(暂时可以不需要添加延时，具体是否需要添加，等待后续更新)
	代码中的注释与开发板上不同，代码中显示的red是green，green是yellow
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
6. 当使用CubMX重新生成代码后，需要手动在RegionCN470.c 函数：RegionCN470TxConfig 中将

	// Setup the radio frequency
    Radio.SetChannel( RegionNvmGroup2->Channels[txConfig->Channel].Frequency );
替换为：
	uint32_t freq;
	const char *name;
	// Setup the radio frequency
	if (plc_get_channel(&freq, &name) == 0)
	{
		RegionNvmGroup2->Channels[txConfig->Channel].Frequency = freq;
		Radio.SetChannel( RegionNvmGroup2->Channels[txConfig->Channel].Frequency );
	}

7. lora_app.h中，修改LORAWAN_DEFAULT_CONFIRMED_MSG_STATE，决定是否启用 confirmed message

现存bug
	1. 与网关的发送信道无法对接
	2. 当收到入网请求同意后，代码运行将会停止（至少看起来是，闪灯任务停止，日志打印停止，也没有消息上传）


/**
 * @file lora_transparent.c
 * @brief LoRa transparent transmission implementation for STM32WL series.
 *
 * This file implements a simple LoRa transparent transmission demo, which periodically sends data
 * (e.g., temperature) and receives LoRa packets, forwarding them as needed. The code is designed
 * for demonstration and can be extended for more complex applications.
 */
/* Includes ------------------------------------------------------------------*/
#include "lora_transparent.h"
#include "radio.h"
#include "radio_driver.h"
#include "gpio.h"
#include "mw_log_conf.h"
#include <string.h>
#include "stm32_systime.h"
#include "ds18b20.h"

// ==================== LoRa 参数（必须与 Arduino 发送端一致）====================
// LoRa parameters (must match Arduino sender)
#define RF_FREQUENCY        471700000 // Hz
#define TX_OUTPUT_POWER     14         // dBm
#define LORA_BANDWIDTH      0         // 125 kHz
#define LORA_SPREADING_SF   7         // SF7
#define LORA_CODINGRATE     1         // 4/5
#define LORA_PREAMBLE_LEN   8         // 前导码长度
#define LORA_FIX_LENGTH     false     // 可变长度
#define LORA_IQ_INVERT      false     // IQ 不反转

// ==================== 缓冲区大小 ====================
// Buffer size
#define BUFFER_SIZE         64

// ==================== 全局变量 ====================
// Global variables
// Counter for sent packets
static uint16_t txCounter = 0;  // 记录发送计数器

// Data buffers
static uint8_t txBuffer[BUFFER_SIZE];
static char txBuffer1[BUFFER_SIZE];
static uint8_t rxBuffer[BUFFER_SIZE];

// Flags for TX/RX completion
static volatile bool txComplete = false;
static volatile bool rxComplete = false;

static RadioEvents_t RadioEvents;

// ==================== 函数声明 ====================
// Function prototypes
static void OnTxDone(void);
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
static void OnRxTimeout(void);
static void OnRxError(void);
static void LED_Init(void);
static void LED_Toggle(void);

// ==================== 回调函数 ====================
// Callback functions

/**
 * @brief Callback when TX is done.
 * Puts radio in standby and sets txComplete flag.
 */
static void OnTxDone(void) {
    Radio.Standby();  // 进入 STANDBY 模式 (Enter STANDBY mode)
    txComplete = true;
    SUBGRF_ClearIrqStatus(IRQ_TX_DONE);  // 清除 TX_DONE 中断 (Clear TX_DONE IRQ)
}

/**
 * @brief Callback when TX times out.
 * Puts radio in standby and sets txComplete flag.
 */
static void OnTxTimeout(void)
{
    Radio.Standby();        // 进入待机模式 (Enter standby mode)
    txComplete = true;     // 标志发送完成 (Mark TX as complete)
}

/**
 * @brief Callback when RX is done.
 * Prints received payload and RSSI info.
 * @param payload Pointer to received data
 * @param size    Size of received data
 * @param rssi    Received signal strength
 * @param snr     Signal-to-noise ratio
 */
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    MW_LOG(TS_OFF, VLEVEL_M,"\r\nreceived packet \"%s\" with Rssi %d , length %d\r\n",payload,rssi,size);
}

/**
 * @brief Callback when RX times out.
 * Optional: handle RX timeout event.
 */
static void OnRxTimeout(void)
{
    // 可选：超时处理（比如什么都不做）
    // 不翻转 LED
	MW_LOG(TS_OFF, VLEVEL_M, "onrxtimeout...");
}

/**
 * @brief Callback when RX error occurs.
 * Optional: handle RX error event.
 */
static void OnRxError(void)
{
    // 可选：错误处理
}

/**
 * @brief Toggle LED (user implementation required)
 */
static void LED_Toggle(void)
{
   // LED_PORT->odt ^= LED_PIN;  // 翻转 PB5
}

// ==================== LoRa 初始化 ====================
// LoRa initialization

static void LoRa_Init(void)
{
    // 设置回调函数 (Set callback functions)
    RadioEvents.TxDone    = OnTxDone;
		RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone    = OnRxDone;

    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError   = OnRxError;



    // 初始化 Radio (Initialize radio)
    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);

	 // 强制写寄存器：确保 Sync Word 是 0x12 (Force write register: ensure Sync Word is 0x12)
    SUBGRF_WriteRegister(0x0740, 0x12);

    // 验证写入（可选）(Optional: verify write)
    uint8_t check = SUBGRF_ReadRegister(0x0740);
//		// 读取芯片类型寄存器（只读）
//		uint8_t chipId = SX126xReadRegister(0x00C0);  // 型号
//		uint8_t hwId   = SX126xReadRegister(0x00C1);  // 硬件 ID
//		uint8_t fwId   = SX126xReadRegister(0x00C2);  // 固件 ID

    // 配置发送参数 (Configure TX parameters)
    Radio.SetTxConfig(
        MODEM_LORA,
        TX_OUTPUT_POWER,
        0,
        LORA_BANDWIDTH,
        LORA_SPREADING_SF,
        LORA_CODINGRATE,
        LORA_PREAMBLE_LEN,
        LORA_FIX_LENGTH,
        true,           // CRC ON
        0, 0, false, 5000  // 最后 txTimeout=0
    );

    HAL_Delay(500);

    // 配置接收参数 (Configure RX parameters)
    Radio.SetRxConfig(
        MODEM_LORA,
        LORA_BANDWIDTH,
        LORA_SPREADING_SF,
        LORA_CODINGRATE,
        0,
        LORA_PREAMBLE_LEN,
        0,
        LORA_FIX_LENGTH,
        0,
        true,           // CRC ON
        0, 0,
        LORA_IQ_INVERT,
        true            // 超时后自动重启接收 (Restart RX after timeout)
    );
		HAL_Delay(500);

}


// ==================== 发送函数 ====================
// LoRa send function

/**
 * @brief Send data over LoRa.
 * Copies data to TX buffer and triggers radio send.
 * @param data Pointer to data to send
 * @param size Size of data
 */
static void LoRa_Send(const char* data, uint8_t size) {
    if (size > BUFFER_SIZE) size = BUFFER_SIZE;

    memcpy(txBuffer, data, size);
    txBuffer[size] = 0;

    txComplete = false;
    Radio.Standby();     // 确保进入 STANDBY 模式 (Ensure standby mode)
    Radio.Send(txBuffer, size); // Send data
		HAL_Delay(200);
}

// ==================== 主函数 ====================
// Main function

/**
 * @brief Main entry for LoRa transparent transmission demo.
 * Initializes LoRa and enters main loop: send and receive data periodically.
 */
void app_lora_init(void) {
    // 初始化 LoRa
    LoRa_Init();

    while (1) {
        // ======== 1. 生成递增的发送数据 ========
        snprintf(txBuffer1, BUFFER_SIZE, "temperature: %.2f", ReadTemperature());
		MW_LOG(TS_OFF, VLEVEL_M, "DS18B20 temperature is %.2f\r\n",ReadTemperature());
        uint8_t size = strlen(txBuffer1);

        txComplete = false;
        rxComplete = false;

        // Send data
        LoRa_Send(txBuffer1, size);

        // 等待发送完成或超时 (Wait for TX to complete or timeout)
        uint32_t timeout = 5000;  // ms
				SysTime_t curtime = SysTimeGet();
        uint32_t start = curtime.SubSeconds;
        while (!txComplete && (curtime.SubSeconds - start) < (timeout/50) )
				{
						curtime = SysTimeGet();
            Radio.IrqProcess();
            HAL_Delay(10);
        }

        if (!txComplete) {
            MW_LOG(TS_OFF, VLEVEL_M, "TX Timeout!\n");
            Radio.Standby();  // 强制退出发送状态 (Force exit TX state)
        } else {
            MW_LOG(TS_OFF, VLEVEL_M, "TX Done: %s\n", txBuffer);
        }

        // ======== 2. 进入接收模式（5秒） ========
        Radio.Rx(5000);  // 接收 5 秒

        // 等待接收完成或超时
				curtime = SysTimeGet();
        start =  curtime.SubSeconds;	// ms级
        while ((curtime.SubSeconds - start) < (5000/50)) {
						curtime = SysTimeGet();
            Radio.IrqProcess();   // 处理中断（DIO1 触发 OnRxDone）
            HAL_Delay(10);
        }

        // 无论是否收到，继续下一轮 (Regardless of RX result, continue next round)
        //txCounter++; // 递增计数器 (Increment counter)
        HAL_Delay(2000);  // 延时 2 秒进入下一轮 (Delay 2 seconds before next round)
    }
}

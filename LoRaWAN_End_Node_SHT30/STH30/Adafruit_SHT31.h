/*
  该头文件将原 Adafruit SHT31 C++ 接口改为纯 C 风格，
  在 STM32 HAL 环境下通过 I2C 访问 SHT31，保持原有功能。
*/

#ifndef ADAFRUIT_SHT31_C_H
#define ADAFRUIT_SHT31_C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "i2c.h" /* 提供 I2C_HandleTypeDef 与 HAL_Delay 原型（经由 main.h/HAL） */

#define SHT31_DEFAULT_ADDR           0x44u
#define SHT31_MEAS_HIGHREP_STRETCH   0x2C06u
#define SHT31_MEAS_MEDREP_STRETCH    0x2C0Du
#define SHT31_MEAS_LOWREP_STRETCH    0x2C10u
#define SHT31_MEAS_HIGHREP           0x2400u
#define SHT31_MEAS_MEDREP            0x240Bu
#define SHT31_MEAS_LOWREP            0x2416u
#define SHT31_READSTATUS             0xF32Du
#define SHT31_CLEARSTATUS            0x3041u
#define SHT31_SOFTRESET              0x30A2u
#define SHT31_HEATEREN               0x306Du
#define SHT31_HEATERDIS              0x3066u

typedef struct {
  uint8_t i2caddr;              /* 7-bit 地址（0x44/0x45） */
  I2C_HandleTypeDef *hi2c;      /* HAL I2C 句柄 */
  float humidity;
  float temp;
} Adafruit_SHT31;

/* 对应原 begin()：初始化句柄与地址，执行一次 reset，始终返回 true（与原库一致） */
bool Adafruit_SHT31_begin(Adafruit_SHT31 *dev, I2C_HandleTypeDef *hi2c, uint8_t i2caddr);

/* 对应原 readTemperature()/readHumidity()，内部按原逻辑触发单次测量并取缓存 */
float Adafruit_SHT31_readTemperature(Adafruit_SHT31 *dev);
float Adafruit_SHT31_readHumidity(Adafruit_SHT31 *dev);
static bool readTempHum(Adafruit_SHT31 *dev);

/* 对应原 readStatus()/reset()/heater()/crc8() */
uint16_t Adafruit_SHT31_readStatus(Adafruit_SHT31 *dev);
void Adafruit_SHT31_reset(Adafruit_SHT31 *dev);
void Adafruit_SHT31_heater(Adafruit_SHT31 *dev, bool on);
uint8_t Adafruit_SHT31_crc8(const uint8_t *data, int len);

#ifdef __cplusplus
}
#endif

#endif /* ADAFRUIT_SHT31_C_H */


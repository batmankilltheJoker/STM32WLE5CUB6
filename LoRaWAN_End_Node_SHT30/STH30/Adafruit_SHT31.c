/*
  将原 Adafruit SHT31 C++ 实现改为纯 C，使用 STM32 HAL I2C。
  公开的 API 见同目录 Adafruit_SHT31.h
*/

#include "Adafruit_SHT31.h"

/* 左移 1 位以符合 HAL 的 8-bit 地址格式 */
static inline uint16_t addr8(Adafruit_SHT31 *dev) { return ((uint16_t)dev->i2caddr) << 1; }

static void writeCommand(Adafruit_SHT31 *dev, uint16_t cmd)
{
  uint8_t buf[2] = { (uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF) };
  (void)HAL_I2C_Master_Transmit(dev->hi2c, addr8(dev), buf, 2, 300);
}

bool Adafruit_SHT31_begin(Adafruit_SHT31 *dev, I2C_HandleTypeDef *hi2c, uint8_t i2caddr)
{
  if (!dev || !hi2c) return false;
  dev->hi2c = hi2c;
  dev->i2caddr = i2caddr;
  Adafruit_SHT31_reset(dev);
  return true; /* 与原库保持一致 */
}

uint16_t Adafruit_SHT31_readStatus(Adafruit_SHT31 *dev)
{
  uint8_t rx[3] = {0};
  writeCommand(dev, SHT31_READSTATUS);
  if (HAL_I2C_Master_Receive(dev->hi2c, addr8(dev), rx, 3, 300) != HAL_OK)
    return 0; /* 原库未严格处理错误，这里返回 0 */
  uint16_t stat = ((uint16_t)rx[0] << 8) | rx[1];
  return stat;
}

void Adafruit_SHT31_reset(Adafruit_SHT31 *dev)
{
  writeCommand(dev, SHT31_SOFTRESET);
  HAL_Delay(10);
}

void Adafruit_SHT31_heater(Adafruit_SHT31 *dev, bool on)
{
  writeCommand(dev, on ? SHT31_HEATEREN : SHT31_HEATERDIS);
}

static bool readTempHum(Adafruit_SHT31 *dev)
{
  uint8_t readbuffer[6];
  writeCommand(dev, SHT31_MEAS_HIGHREP);
  HAL_Delay(500); /* 按原实现等待 500ms */

  if (HAL_I2C_Master_Receive(dev->hi2c, addr8(dev), readbuffer, 6, 500) != HAL_OK)
    return false;

  uint16_t ST = ((uint16_t)readbuffer[0] << 8) | readbuffer[1];
  if (readbuffer[2] != Adafruit_SHT31_crc8(readbuffer, 2)) return false;

  uint16_t SRH = ((uint16_t)readbuffer[3] << 8) | readbuffer[4];
  if (readbuffer[5] != Adafruit_SHT31_crc8(readbuffer + 3, 2)) return false;

  double stemp = (double)ST;
  stemp *= 175.0;
  stemp /= 65535.0; /* 0xFFFF */
  stemp = -45.0 + stemp;
  dev->temp = (float)stemp;

  double shum = (double)SRH;
  shum *= 100.0;
  shum /= 65535.0;
  dev->humidity = (float)shum;
  return true;
}

float Adafruit_SHT31_readTemperature(Adafruit_SHT31 *dev)
{
  if (!readTempHum(dev)) return (float)0.0f/0.0f; /* NaN */
  return dev->temp;
}

float Adafruit_SHT31_readHumidity(Adafruit_SHT31 *dev)
{
  if (!readTempHum(dev)) return (float)0.0f/0.0f; /* NaN */
  return dev->humidity;
}

uint8_t Adafruit_SHT31_crc8(const uint8_t *data, int len)
{
  const uint8_t POLYNOMIAL = 0x31;
  uint8_t crc = 0xFF;
  while (len--) {
    crc ^= *data++;
    for (int i = 0; i < 8; i++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ POLYNOMIAL) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

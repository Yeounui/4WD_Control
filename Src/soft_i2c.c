#include "soft_i2c.h"

#include "stm32f1xx_hal.h"

#define SOFT_I2C_SCL_PORT GPIOA
#define SOFT_I2C_SDA_PORT GPIOB
#define SOFT_I2C_SCL  GPIO_PIN_6
#define SOFT_I2C_SDA  GPIO_PIN_11
#define SOFT_I2C_SCL_SHIFT 24U
#define SOFT_I2C_SDA_SHIFT 12U
#define SOFT_I2C_CRL_MASK  0x0FU
#define SOFT_I2C_OUTPUT_PP 0x03U
#define SOFT_I2C_INPUT_FL  0x04U

static void delay_half_period(void)
{
  volatile uint32_t i;

  for (i = 0U; i < (SystemCoreClock / 800000U); i++)
  {
  }
}

static void scl_low(void)
{
  SOFT_I2C_SCL_PORT->BRR = SOFT_I2C_SCL;
  MODIFY_REG(SOFT_I2C_SCL_PORT->CRL,
             SOFT_I2C_CRL_MASK << SOFT_I2C_SCL_SHIFT,
             SOFT_I2C_OUTPUT_PP << SOFT_I2C_SCL_SHIFT);
}

static void scl_release(void)
{
  SOFT_I2C_SCL_PORT->BRR = SOFT_I2C_SCL;
  MODIFY_REG(SOFT_I2C_SCL_PORT->CRL,
             SOFT_I2C_CRL_MASK << SOFT_I2C_SCL_SHIFT,
             SOFT_I2C_INPUT_FL << SOFT_I2C_SCL_SHIFT);
}

static void sda_low(void)
{
  SOFT_I2C_SDA_PORT->BRR = SOFT_I2C_SDA;
  MODIFY_REG(SOFT_I2C_SDA_PORT->CRH,
             SOFT_I2C_CRL_MASK << SOFT_I2C_SDA_SHIFT,
             SOFT_I2C_OUTPUT_PP << SOFT_I2C_SDA_SHIFT);
}

static void sda_release(void)
{
  SOFT_I2C_SDA_PORT->BRR = SOFT_I2C_SDA;
  MODIFY_REG(SOFT_I2C_SDA_PORT->CRH,
             SOFT_I2C_CRL_MASK << SOFT_I2C_SDA_SHIFT,
             SOFT_I2C_INPUT_FL << SOFT_I2C_SDA_SHIFT);
}

static uint8_t sda_read(void)
{
  return (HAL_GPIO_ReadPin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA) == GPIO_PIN_SET) ? 1U : 0U;
}

static void start(void)
{
  sda_release();
  scl_release();
  delay_half_period();
  sda_low();
  delay_half_period();
  scl_low();
  delay_half_period();
}

static void stop(void)
{
  sda_low();
  delay_half_period();
  scl_release();
  delay_half_period();
  sda_release();
  delay_half_period();
}

static uint8_t write_byte(uint8_t b)
{
  uint8_t bit;
  uint8_t ack;

  for (bit = 0U; bit < 8U; bit++)
  {
    if ((b & 0x80U) != 0U)
    {
      sda_release();
    }
    else
    {
      sda_low();
    }
    delay_half_period();
    scl_release();
    delay_half_period();
    scl_low();
    delay_half_period();
    b <<= 1;
  }

  sda_release();
  delay_half_period();
  scl_release();
  delay_half_period();
  ack = sda_read();
  scl_low();
  delay_half_period();

  return ack;
}

static uint8_t read_byte(uint8_t ack)
{
  uint8_t bit;
  uint8_t value;

  value = 0U;
  for (bit = 0U; bit < 8U; bit++)
  {
    sda_release();
    delay_half_period();
    scl_release();
    delay_half_period();
    value = (uint8_t)((value << 1) | sda_read());
    scl_low();
    delay_half_period();
  }

  if (ack != 0U)
  {
    sda_low();
  }
  else
  {
    sda_release();
  }
  delay_half_period();
  scl_release();
  delay_half_period();
  scl_low();
  delay_half_period();
  sda_release();

  return value;
}

void SoftI2C_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = SOFT_I2C_SCL;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SOFT_I2C_SCL_PORT, &GPIO_InitStruct);

  HAL_GPIO_WritePin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = SOFT_I2C_SDA;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SOFT_I2C_SDA_PORT, &GPIO_InitStruct);

  sda_release();
  scl_release();
}

uint8_t SoftI2C_MemRead(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
  uint16_t i;

  if ((buf == NULL) && (len != 0U))
  {
    return 1U;
  }

  start();
  if (write_byte((uint8_t)(dev_addr & 0xFEU)) != 0U)
  {
    stop();
    return 1U;
  }
  if (write_byte(reg) != 0U)
  {
    stop();
    return 1U;
  }

  start();
  if (write_byte((uint8_t)(dev_addr | 0x01U)) != 0U)
  {
    stop();
    return 1U;
  }

  for (i = 0U; i < len; i++)
  {
    buf[i] = read_byte((i + 1U) < len);
  }
  stop();

  return 0U;
}

uint8_t SoftI2C_MemWrite(uint8_t dev_addr, uint8_t reg, const uint8_t *buf, uint16_t len)
{
  uint16_t i;

  if ((buf == NULL) && (len != 0U))
  {
    return 1U;
  }

  start();
  if (write_byte((uint8_t)(dev_addr & 0xFEU)) != 0U)
  {
    stop();
    return 1U;
  }
  if (write_byte(reg) != 0U)
  {
    stop();
    return 1U;
  }

  for (i = 0U; i < len; i++)
  {
    if (write_byte(buf[i]) != 0U)
    {
      stop();
      return 1U;
    }
  }
  stop();

  return 0U;
}

uint8_t SoftI2C_MemRead16(uint8_t dev_addr, uint16_t reg, uint8_t *buf, uint16_t len)
{
  uint16_t i;

  if ((buf == NULL) && (len != 0U))
  {
    return 1U;
  }

  start();
  if (write_byte((uint8_t)(dev_addr & 0xFEU)) != 0U)
  {
    stop();
    return 1U;
  }
  if (write_byte((uint8_t)(reg >> 8)) != 0U)
  {
    stop();
    return 1U;
  }
  if (write_byte((uint8_t)(reg & 0xFFU)) != 0U)
  {
    stop();
    return 1U;
  }

  start();
  if (write_byte((uint8_t)(dev_addr | 0x01U)) != 0U)
  {
    stop();
    return 1U;
  }

  for (i = 0U; i < len; i++)
  {
    buf[i] = read_byte((i + 1U) < len);
  }
  stop();

  return 0U;
}

uint8_t SoftI2C_MemWrite16(uint8_t dev_addr, uint16_t reg, const uint8_t *buf, uint16_t len)
{
  uint16_t i;

  if ((buf == NULL) && (len != 0U))
  {
    return 1U;
  }

  start();
  if (write_byte((uint8_t)(dev_addr & 0xFEU)) != 0U)
  {
    stop();
    return 1U;
  }
  if (write_byte((uint8_t)(reg >> 8)) != 0U)
  {
    stop();
    return 1U;
  }
  if (write_byte((uint8_t)(reg & 0xFFU)) != 0U)
  {
    stop();
    return 1U;
  }

  for (i = 0U; i < len; i++)
  {
    if (write_byte(buf[i]) != 0U)
    {
      stop();
      return 1U;
    }
  }
  stop();

  return 0U;
}

uint8_t SoftI2C_MasterTransmit(uint8_t dev_addr, const uint8_t *buf, uint16_t len)
{
  uint16_t i;

  if ((buf == NULL) && (len != 0U))
  {
    return 1U;
  }

  start();
  if (write_byte((uint8_t)(dev_addr & 0xFEU)) != 0U)
  {
    stop();
    return 1U;
  }

  for (i = 0U; i < len; i++)
  {
    if (write_byte(buf[i]) != 0U)
    {
      stop();
      return 1U;
    }
  }
  stop();

  return 0U;
}

uint8_t SoftI2C_IsDeviceReady(uint8_t dev_addr)
{
  uint8_t status;

  start();
  status = write_byte((uint8_t)(dev_addr & 0xFEU));
  stop();

  return (status == 0U) ? 0U : 1U;
}

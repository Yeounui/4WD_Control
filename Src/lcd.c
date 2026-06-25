#include "lcd.h"

#include "main.h"

#include <stdio.h>
#include <string.h>

#define LCD_I2C_TIMEOUT_MS 5U
#define LCD_COLUMNS        16U
#define LCD_RS             0x01U
#define LCD_ENABLE         0x04U
#define LCD_BACKLIGHT      0x08U
#define LCD_LINE_1         0x80U
#define LCD_LINE_2         0xC0U

extern I2C_HandleTypeDef hi2c1;

static uint8_t lcd_available;
static uint8_t tx_buffer[(LCD_COLUMNS + 1U) * 6U];

static uint16_t LCD_AppendNibble(uint16_t index, uint8_t nibble, uint8_t flags)
{
  uint8_t value;

  value = (uint8_t)((nibble << 4) | LCD_BACKLIGHT | flags);
  tx_buffer[index++] = value;
  tx_buffer[index++] = (uint8_t)(value | LCD_ENABLE);
  tx_buffer[index++] = value;
  return index;
}

static uint16_t LCD_AppendByte(uint16_t index, uint8_t value, uint8_t flags)
{
  index = LCD_AppendNibble(index, (uint8_t)(value >> 4), flags);
  index = LCD_AppendNibble(index, (uint8_t)(value & 0x0FU), flags);
  return index;
}

static uint8_t LCD_Transmit(uint16_t length)
{
  HAL_StatusTypeDef status;

  status = HAL_I2C_Master_Transmit(&hi2c1,
                                   (uint16_t)(LCD_PCF8574_ADDRESS_7BIT << 1),
                                   tx_buffer,
                                   length,
                                   LCD_I2C_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    lcd_available = 0U;
    return 0U;
  }

  return 1U;
}

static uint8_t LCD_Command(uint8_t command)
{
  uint16_t length;

  length = LCD_AppendByte(0U, command, 0U);
  return LCD_Transmit(length);
}

static void LCD_WriteLine(uint8_t address, const char *text)
{
  uint16_t length;
  uint8_t column;
  uint8_t text_end;

  if ((lcd_available == 0U) || (LCD_Command(address) == 0U))
  {
    return;
  }

  length = 0U;
  text_end = 0U;
  for (column = 0U; column < LCD_COLUMNS; column++)
  {
    uint8_t value;

    if ((text_end == 0U) && (text[column] != '\0'))
    {
      value = (uint8_t)text[column];
    }
    else
    {
      text_end = 1U;
      value = (uint8_t)' ';
    }
    length = LCD_AppendByte(length, value, LCD_RS);
  }
  (void)LCD_Transmit(length);
}

void LCD_Init(void)
{
  uint16_t address;

  lcd_available = 0U;
  address = (uint16_t)(LCD_PCF8574_ADDRESS_7BIT << 1);
  if (HAL_I2C_IsDeviceReady(&hi2c1, address, 2U, LCD_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return;
  }

  lcd_available = 1U;
  HAL_Delay(40U);

  tx_buffer[0] = (uint8_t)(0x30U | LCD_BACKLIGHT);
  tx_buffer[1] = (uint8_t)(0x30U | LCD_BACKLIGHT | LCD_ENABLE);
  tx_buffer[2] = (uint8_t)(0x30U | LCD_BACKLIGHT);
  if (LCD_Transmit(3U) == 0U)
  {
    return;
  }
  HAL_Delay(5U);

  tx_buffer[0] = (uint8_t)(0x30U | LCD_BACKLIGHT);
  tx_buffer[1] = (uint8_t)(0x30U | LCD_BACKLIGHT | LCD_ENABLE);
  tx_buffer[2] = (uint8_t)(0x30U | LCD_BACKLIGHT);
  if (LCD_Transmit(3U) == 0U)
  {
    return;
  }
  HAL_Delay(1U);

  tx_buffer[0] = (uint8_t)(0x30U | LCD_BACKLIGHT);
  tx_buffer[1] = (uint8_t)(0x30U | LCD_BACKLIGHT | LCD_ENABLE);
  tx_buffer[2] = (uint8_t)(0x30U | LCD_BACKLIGHT);
  if (LCD_Transmit(3U) == 0U)
  {
    return;
  }
  HAL_Delay(1U);

  tx_buffer[0] = (uint8_t)(0x20U | LCD_BACKLIGHT);
  tx_buffer[1] = (uint8_t)(0x20U | LCD_BACKLIGHT | LCD_ENABLE);
  tx_buffer[2] = (uint8_t)(0x20U | LCD_BACKLIGHT);
  if (LCD_Transmit(3U) == 0U)
  {
    return;
  }

  if ((LCD_Command(0x28U) == 0U) || (LCD_Command(0x08U) == 0U)
      || (LCD_Command(0x01U) == 0U))
  {
    return;
  }
  HAL_Delay(2U);
  (void)LCD_Command(0x06U);
  (void)LCD_Command(0x0CU);
}

void LCD_Update(const char *state_name, float yaw, uint8_t sensor_ok)
{
  char state_line[LCD_COLUMNS + 1U];
  char sensor_line[LCD_COLUMNS + 1U];
  int32_t yaw_tenths;
  uint32_t yaw_magnitude;

  if ((lcd_available == 0U) || (state_name == NULL))
  {
    return;
  }

  (void)snprintf(state_line, sizeof(state_line), "S:%-14.14s", state_name);
  if (sensor_ok != 0U)
  {
    yaw_tenths = (int32_t)(yaw * 10.0f);
    if (yaw_tenths > 99999)
    {
      yaw_tenths = 99999;
    }
    else if (yaw_tenths < -99999)
    {
      yaw_tenths = -99999;
    }
    yaw_magnitude = (uint32_t)((yaw_tenths < 0) ? -yaw_tenths : yaw_tenths);
    sensor_line[0] = 'Y';
    sensor_line[1] = ':';
    sensor_line[2] = (yaw_tenths < 0) ? '-' : '+';
    sensor_line[3] = (char)('0' + ((yaw_magnitude / 10000U) % 10U));
    sensor_line[4] = (char)('0' + ((yaw_magnitude / 1000U) % 10U));
    sensor_line[5] = (char)('0' + ((yaw_magnitude / 100U) % 10U));
    sensor_line[6] = (char)('0' + ((yaw_magnitude / 10U) % 10U));
    sensor_line[7] = '.';
    sensor_line[8] = (char)('0' + (yaw_magnitude % 10U));
    sensor_line[9] = ' ';
    sensor_line[10] = 'd';
    sensor_line[11] = 'e';
    sensor_line[12] = 'g';
    sensor_line[13] = '\0';
  }
  else
  {
    (void)strncpy(sensor_line, "MPU:NOT READY", sizeof(sensor_line));
    sensor_line[sizeof(sensor_line) - 1U] = '\0';
  }

  LCD_WriteLine(LCD_LINE_1, state_line);
  LCD_WriteLine(LCD_LINE_2, sensor_line);
}

uint8_t LCD_IsAvailable(void)
{
  return lcd_available;
}

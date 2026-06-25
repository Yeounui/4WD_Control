#ifndef LCD_H
#define LCD_H

#include <stdint.h>

#ifndef LCD_PCF8574_ADDRESS_7BIT
#define LCD_PCF8574_ADDRESS_7BIT 0x27U
#endif

void LCD_Init(void);
void LCD_Update(const char *state_name, float yaw, uint8_t sensor_ok);
uint8_t LCD_IsAvailable(void);

#endif /* LCD_H */

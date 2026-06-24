#include "hc06.h"

#include "main.h"

#include <string.h>

extern UART_HandleTypeDef huart2;

static uint8_t rx_byte;
static char rx_line[64];
static uint8_t rx_len;

static void HC06_Parse(const char *cmd)
{
  const char *name;
  const char *state;

  name = cmd;
  if (strncmp(cmd, "AT+", 3) == 0)
  {
    name = cmd + 3;
  }

  state = NULL;

  switch (name[0])
  {
    case 'F':
      if (strcmp(name, "FWD") == 0)
      {
        state = "STRAIGHT";
      }
      break;

    case 'L':
      if (strcmp(name, "LEFT") == 0)
      {
        state = "TURN";
      }
      break;

    case 'R':
      if (strcmp(name, "RIGHT") == 0)
      {
        state = "TURN";
      }
      else if (strcmp(name, "RST") == 0)
      {
        state = "EMERGENCY";
      }
      break;

    case 'S':
      if (strcmp(name, "STOP") == 0)
      {
        state = "IDLE";
      }
      break;

    case 'M':
      if (strcmp(name, "MAN") == 0)
      {
        state = "MANUAL";
      }
      break;

    default:
      break;
  }

  if (state != NULL)
  {
    /* Phase 6: replace echo with FSM_SetState(state) */
    HAL_UART_Transmit(&huart2, (uint8_t *)state, strlen(state), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2U, HAL_MAX_DELAY);
  }
  else
  {
    HAL_UART_Transmit(&huart2, (uint8_t *)"ERROR\r\n", 7U, HAL_MAX_DELAY);
  }
}

static void HC06_OnReceive(uint8_t byte)
{
  if ((byte == '\r') || (byte == '\n'))
  {
    if (rx_len > 0U)
    {
      rx_line[rx_len] = '\0';
      HC06_Parse(rx_line);
      rx_len = 0U;
    }

    return;
  }

  if (rx_len < (sizeof(rx_line) - 1U))
  {
    rx_line[rx_len] = (char)byte;
    rx_len++;
  }
}

void HC06_Init(void)
{
  HAL_UART_Receive_DMA(&huart2, &rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    HC06_OnReceive(rx_byte);
    HAL_UART_Receive_DMA(&huart2, &rx_byte, 1);
  }
}

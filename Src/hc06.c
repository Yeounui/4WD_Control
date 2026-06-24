#include "hc06.h"

#include "main.h"
#include "kalman.h"

#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart2;
extern volatile uint8_t g_param_save_request;

static uint8_t rx_byte;
static char rx_line[64];
static uint8_t rx_len;

typedef enum {
  AT_FWD = 0,
  AT_LEFT,
  AT_RIGHT,
  AT_STOP,
  AT_MAN,
  AT_RST,
  AT_GET,
  AT_SAVE,
  AT_UNKNOWN
} AT_Command;

typedef struct {
    const char* cmd;
    AT_Command command;
} AT_CommandEntry;

static const AT_CommandEntry AT_table[] = {
    {"AT+FWD",   AT_FWD},
    {"AT+LEFT",  AT_LEFT},
    {"AT+RIGHT", AT_RIGHT},
    {"AT+STOP",  AT_STOP},
    {"AT+MAN",   AT_MAN},
    {"AT+RST",   AT_RST},
    {"AT+GET",   AT_GET},
    {"AT+SAVE",  AT_SAVE}
};

static AT_Command HC06_FindCommand(const char *cmd) {
  for (size_t i = 0; i < sizeof(AT_table) / sizeof(AT_table[0]); i++) {
    if (strcmp(cmd, AT_table[i].cmd) == 0) {
      return AT_table[i].command;
    }
  }
  return AT_UNKNOWN;
}

static void HC06_SendParameter(const char *name, float value, float scale) {
  char buffer[32];
  int length;

  length = snprintf(buffer, sizeof(buffer), "%s=%ld\r\n", name, (long)(value * scale));

  if ((length > 0) && ((size_t)length < sizeof(buffer))) {
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, (uint16_t)length, HAL_MAX_DELAY);
  }
}

static void HC06_Parse(const char *cmd) {
  const char *state = NULL;

  switch (HC06_FindCommand(cmd)) {
    case AT_FWD:
      state = "STRAIGHT";
      break;

    case AT_LEFT:
    case AT_RIGHT:
      state = "TURN";
      break;

    case AT_STOP:
      state = "IDLE";
      break;

    case AT_MAN:
      state = "MANUAL";
      break;

    case AT_RST:
      state = "EMERGENCY";
      break;

    case AT_GET:
    {
      /* R is in micro-units, bias in milli-deg/s, and yaw in centi-deg. */
      HC06_SendParameter("R", Kalman_GetR(), 1000000.0f);
      HC06_SendParameter("BIAS", Kalman_GetBias(), 1000.0f);
      HC06_SendParameter("YAW", Kalman_GetAngle(), 100.0f);
      return;
    }

    case AT_SAVE:
      g_param_save_request = 1U;
      return;
    
    case AT_UNKNOWN:
    default:
      break;
  }

  if (state != NULL) {
    /* Phase 6: replace echo with FSM_SetState(state) */
    HAL_UART_Transmit(&huart2, (uint8_t *)state, strlen(state), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2U, HAL_MAX_DELAY);
  }
  else {
    HAL_UART_Transmit(&huart2, (uint8_t *)"ERROR\r\n", 7U, HAL_MAX_DELAY);
  }
}

static void HC06_OnReceive(uint8_t byte) {
  if ((byte == '\r') || (byte == '\n')) {
    if (rx_len > 0U) {
      rx_line[rx_len] = '\0';
      HC06_Parse(rx_line);
      rx_len = 0U;
    }

    return;
  }

  if (rx_len < (sizeof(rx_line) - 1U)) {
    rx_line[rx_len] = (char)byte;
    rx_len++;
  }
}

void HC06_Init(void) {
  HAL_UART_Receive_DMA(&huart2, &rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    HC06_OnReceive(rx_byte);
    HAL_UART_Receive_DMA(&huart2, &rx_byte, 1);
  }
}

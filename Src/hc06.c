#include "hc06.h"

#include "main.h"
#include "fsm.h"
#include "kalman.h"

#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart2;
extern volatile uint8_t g_param_save_request;

static uint8_t rx_byte;
static char rx_line[64];
static uint8_t rx_len;
static uint8_t rx_overflow;
static char pending_command[64];
static volatile uint8_t command_ready;
static char process_command[64];

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
  const char *response = NULL;

  switch (HC06_FindCommand(cmd)) {
    case AT_FWD:
      FSM_SetDirection(FSM_MOTION_FORWARD);
      response = FSM_GetStateName();
      break;

    case AT_LEFT:
      FSM_SetDirection(FSM_MOTION_LEFT);
      response = FSM_GetStateName();
      break;

    case AT_RIGHT:
      FSM_SetDirection(FSM_MOTION_RIGHT);
      response = FSM_GetStateName();
      break;

    case AT_STOP:
      if (FSM_SetState(FSM_STATE_IDLE) != 0U) {
        response = FSM_GetStateName();
      }
      break;

    case AT_MAN:
      if (FSM_SetState(FSM_STATE_MANUAL) != 0U) {
        response = FSM_GetStateName();
      }
      break;

    case AT_RST:
      if (FSM_ResetEmergency() != 0U) {
        response = FSM_GetStateName();
      }
      else {
        response = "EMERGENCY";
      }
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

  if (response != NULL) {
    HAL_UART_Transmit(&huart2, (uint8_t *)response, strlen(response), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2U, HAL_MAX_DELAY);
  }
  else {
    HAL_UART_Transmit(&huart2, (uint8_t *)"ERROR\r\n", 7U, HAL_MAX_DELAY);
  }
}

static void HC06_OnReceive(uint8_t byte) {
  if ((byte == '\r') || (byte == '\n')) {
    if ((rx_len > 0U) && (rx_overflow == 0U) && (command_ready == 0U)) {
      uint8_t index;

      for (index = 0U; index < rx_len; index++) {
        pending_command[index] = rx_line[index];
      }
      pending_command[rx_len] = '\0';
      __DMB();
      command_ready = 1U;
    }

    rx_len = 0U;
    rx_overflow = 0U;
    return;
  }

  if (rx_len < (sizeof(rx_line) - 1U)) {
    rx_line[rx_len] = (char)byte;
    rx_len++;
  }
  else {
    rx_overflow = 1U;
  }
}

void HC06_Init(void) {
  rx_len = 0U;
  rx_overflow = 0U;
  command_ready = 0U;
  HAL_UART_Receive_DMA(&huart2, &rx_byte, 1);
}

void HC06_Process(void) {
  uint32_t primask;
  uint8_t index;

  primask = __get_PRIMASK();
  __disable_irq();
  if (command_ready == 0U) {
    if (primask == 0U) {
      __enable_irq();
    }
    return;
  }

  for (index = 0U; index < sizeof(process_command); index++) {
    process_command[index] = pending_command[index];
    if (process_command[index] == '\0') {
      break;
    }
  }
  process_command[sizeof(process_command) - 1U] = '\0';
  command_ready = 0U;
  if (primask == 0U) {
    __enable_irq();
  }

  HC06_Parse(process_command);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    HC06_OnReceive(rx_byte);
    HAL_UART_Receive_DMA(&huart2, &rx_byte, 1);
  }
}

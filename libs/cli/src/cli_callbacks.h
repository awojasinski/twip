#ifndef CLI_CALLBACKS_H
#define CLI_CALLBACKS_H

#include "cli.h"

enum cmd_enum_t
{
  CLI_CALLBACK_PWM,
  CLI_CALLBACK_LED,
  CLI_CALLBACK_PAUSE,
  CLI_CALLBACK_CONTINUE,
  //Put more callbacks above comment

  CALLBACKS_CNT,
};

#define CLI_CMD_CALLBACKS_CNT CALLBACKS_CNT

typedef struct
{
  uint8_t enum_type;
  char *command;
  cmd_error_t (*callback)(char *);
} cmd_t;

extern const cmd_t cmd_list[CLI_CMD_CALLBACKS_CNT];

#endif
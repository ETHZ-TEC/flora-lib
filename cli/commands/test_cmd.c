/*
 * test_cmd.c
 *
 *  Created on: Jun 1, 2018
 *      Author: marku
 */

#include "flora_lib.h"

#if CLI_ENABLE

static command_return_t test_sync_command_handler(command_execution_t execution);

static parameter_t test_sync_parameter_initiate = {
    .name = "initiate_sync",
    .description = "Will toggle COM_GPIO1 on node to sync to other node",

    .type = CMD_PARAMETER_NONE,

    .options = NULL,
    .option_count = 0,

    .flag = 'i',

    .optional = true,
    .has_flag = true,
    .has_value = false,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t test_sync_parameter_schedule = {
    .name = "schedule",
    .description = "Will schedule sync on initiator to given offset (so both nodes have synced capture and compare value)",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = 's',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t test_sync_parameter_dbg = {
    .name = "dbg",
    .description = "Toggle NSS low and high after a given HS timer offset. Allows to measure precision of input to output propagation path.",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = 'd',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t* test_sync_parameters[] = {
    &test_sync_parameter_initiate,
    &test_sync_parameter_schedule,
    &test_sync_parameter_dbg,
};

static command_t test_sync = {
  .execution_ptr = &test_sync_command_handler,
  .name = "sync",
  .description = "Runs a tx2tx state translation",
  .prompt = "",
  .parameters = (parameter_t**) test_sync_parameters,
  .parameter_count = sizeof(test_sync_parameters) / sizeof(parameter_t*),
  .children = NULL,
  .children_count = 0,
  .executable = true,
  .built_in = false,
  .has_prompt = false,
  .hidden = false,
  .children_only_in_prompt_invokable = false
};

static command_t* test_subcommands[] = {&test_sync};

static command_t test_command = {
  .execution_ptr = NULL,
  .name = "test",
  .description = "Test procedures for high level functionality",
  .prompt = "",
  .parameters = NULL,
  .parameter_count = 0,
  .children = NULL,
  .children_count = 0,
  .executable = false,
  .built_in = false,
  .has_prompt = false,
  .hidden = false,
  .children_only_in_prompt_invokable = false
};

static command_t* test_commands[] = {&test_command};

void test_register_commands() {
  command_register(&test_command, test_subcommands, COMMAND_COUNT(test_subcommands));
  command_register(NULL, test_commands, 1);
}

static void handle_dbg_sync() {
  RADIO_CLR_NSS_PIN();
  delay_us(200);
  RADIO_SET_NSS_PIN();
}

volatile int64_t dbg_sync_offset = -1;

static void handle_irq() {
  if (dbg_sync_offset != -1) {
    hs_timer_schedule(hs_timer_get_capture_timestamp() + dbg_sync_offset, &handle_dbg_sync);
  }

  //lora_set_irq_callback(NULL);
}

static void handle_sync_initiation() {
  HAL_GPIO_WritePin(COM_GPIO1_GPIO_Port, COM_GPIO1_Pin, GPIO_PIN_SET);
  rtc_delay(1);
  HAL_GPIO_WritePin(COM_GPIO1_GPIO_Port, COM_GPIO1_Pin, GPIO_PIN_RESET);

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = COM_GPIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(COM_GPIO1_GPIO_Port, &GPIO_InitStruct);


}

static command_return_t test_sync_command_handler(command_execution_t execution) {
  if (command_get_parameter(&execution, 'i')) {
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = COM_GPIO1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(COM_GPIO1_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(COM_GPIO1_GPIO_Port, COM_GPIO1_Pin, GPIO_PIN_RESET);

    value_t* schedule_param = command_get_parameter(&execution, 's');
    if (schedule_param) {
      uint32_t offset = strtol(schedule_param->value, NULL, 10);

      hs_timer_schedule(hs_timer_get_current_timestamp() + offset, &handle_sync_initiation);
    }
    else {
      handle_sync_initiation();
    }
  }
  else {
    value_t* dbg_sync_offset_param = command_get_parameter(&execution, 'd');
    if (dbg_sync_offset_param) {
      dbg_sync_offset = (uint32_t) strtol(dbg_sync_offset_param->value, NULL, 10);
    }

    radio_set_irq_callback(&handle_irq);
  }

  return CMD_RET_SUCCESS;
}

#endif /* CLI_ENABLE */

/*
 * config.c
 *
 *  Created on: May 9, 2018
 *      Author: marku
 */

#include "arch/arch.h"


extern bool cli_interactive_mode;


config_t config_current = {
    .magic_word = CONFIG_MAGIC_WORD,
    .uid = 0,
    .role = 0,
#if CLI_ENABLE
    .cli_mode = CLI_INTERACTIVE_ENABLE,
#else
    .cli_mode = 1, /* hard-coded if CLI not enabled to prevent compile errors */
#endif /* CLI_ENABLE */
};


#define CONFIG_COUNT_OF_ENTRIES 3
static const char* config_entries[] = {
    "uid",
    "role",
    "cli_mode",
};


static volatile bool config_initialized = false;



void config_init() {
  config_load();
  if (NODE_ID != 0xbeef) {
    config_current.uid = NODE_ID;
  }
  config_initialized = true;
}

config_t* config_get()
{
  return &config_current;
}

void config_set(config_t config)
{
  config_current = config;

  return;
}

void config_store(config_t* config)
{
  if (config == NULL)
  {
    config = &config_current;
  }

  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );

  FLASH_EraseInitTypeDef flash_erase = {
      .TypeErase = FLASH_TYPEERASE_PAGES,
      .Banks = CONFIG_MEMORY_BANK,
      .Page = CONFIG_LAST_PAGE_INDEX,
      .NbPages = CONFIG_NUMBER_OF_PAGES,
  };

  uint32_t flash_erase_error = 0;
  if (HAL_FLASHEx_Erase(&flash_erase, &flash_erase_error) != HAL_OK) {
    HAL_FLASH_Lock();
    return;
  }

  int i;
  for (i = 0; i < ((sizeof(config_t) + 7) / sizeof(uint64_t)); i++)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, CONFIG_FLASH_ADDRESS + i * sizeof(uint64_t), ((uint64_t*) config)[i]) != HAL_OK) {
      HAL_FLASH_Lock();
      return;
    }
  }

  HAL_FLASH_Lock();

  return;
}

config_t* config_load()
{
  config_t* config = (config_t*) CONFIG_FLASH_ADDRESS;

  if (config->magic_word == CONFIG_MAGIC_WORD)
  {
    config_current = *config;
  }

  return &config_current;
}



void config_print(char* search) {
  if (cli_interactive_mode) {
    char buf[128];

    cli_log("", "config", CLI_LOG_LEVEL_DEFAULT);
    cli_log_inline("[Key]:\t[Value]", CLI_LOG_LEVEL_DEFAULT, true, false, true);

    if (strcasecmp(search, "") == 0)
    {
      int i;
      for (i = 0; i < CONFIG_COUNT_OF_ENTRIES; i++)
      {
        snprintf(buf, sizeof(buf), "%s:\t0x%08x", config_entries[i], (unsigned int)((uint32_t*) &config_current)[i+1]);
        if (i == CONFIG_COUNT_OF_ENTRIES - 1) {
          cli_log_inline(buf, CLI_LOG_LEVEL_DEFAULT, true, true, true);
        }
        else {
          cli_log_inline(buf, CLI_LOG_LEVEL_DEFAULT, true, false, true);
        }
      }
    }
    else
    {
      int i;
      for (i = 0; i < CONFIG_COUNT_OF_ENTRIES; i++)
      {
        if (strcasecmp(search, config_entries[i]) == 0)
        {
          snprintf(buf, sizeof(buf), "%s:\t0x%08x", config_entries[i], (unsigned int)((uint32_t*) &config_current)[i+1]);
          cli_log_inline(buf, CLI_LOG_LEVEL_DEFAULT, true, false, true);
          return;
        }
      }

      cli_println("Could not find entry!");
    }
  }
  else {
    cJSON* config_json = cJSON_CreateObject();
    if (config_json == NULL) {
      goto end;
    }

    if (cJSON_AddStringToObject(config_json, "type", "config") == NULL) {
      goto end;
    }

    cJSON* fields = cJSON_CreateArray();
    if (fields == NULL) {
      goto end;
    }

    cJSON_AddItemToObject(config_json, "fields", fields);

    if (strcasecmp(search, "") == 0)
    {
      int i;
      for (i = 0; i < CONFIG_COUNT_OF_ENTRIES; i++)
      {
        cJSON* field = cJSON_CreateObject();
        if (field == NULL) {
          goto end;
        }

        if (cJSON_AddStringToObject(field, "name", config_entries[i]) == NULL) {
          goto end;
        }
        if (cJSON_AddNumberToObject(field, "value", (unsigned int)((uint32_t*) &config_current)[i+1]) == NULL) {
          goto end;
        }

        cJSON_AddItemToArray(fields, field);
      }
    }
    else
    {
      int i;
      for (i = 0; i < CONFIG_COUNT_OF_ENTRIES; i++)
      {
        if (strcasecmp(search, config_entries[i]) == 0)
        {
          cJSON* field = cJSON_CreateObject();
          if (field == NULL) {
            goto end;
          }

          if (cJSON_AddStringToObject(field, "name", config_entries[i]) == NULL) {
            goto end;
          }
          if (cJSON_AddNumberToObject(field, "value", (unsigned int)((uint32_t*) &config_current)[i+1]) == NULL) {
            goto end;
          }

          cJSON_AddItemToArray(fields, field);
        }
      }
    }

    cli_log_json(config_json, "config", CLI_LOG_LEVEL_DEFAULT);
    return;

end:
    cJSON_Delete(config_json);
  }

  return;


}

void config_set_entry(char* entry, uint32_t value) {
  int i;
  for (i = 0; i < CONFIG_COUNT_OF_ENTRIES; i++)
  {
    if (strcasecmp(entry, config_entries[i]) == 0)
    {
      ((uint32_t*) &config_current)[i+1] = value;
    }
  }

  config_print(entry);
}

/*
 * log.c
 *
 *  Created on: Apr 7, 2020
 *      Author: rdaforno
 */

#include "flora_lib.h"

#if LOG_ENABLE

/* --- private VARIABLES --- */

// note: the print buffer is an unprotected shared resource
static char log_print_buffer[LOG_LINE_LENGTH];

#if !LOG_PRINT_IMMEDIATELY
  static char log_buffer[LOG_BUFFER_SIZE];
  static int32_t read_idx  = 0;
  static int32_t write_idx = 0;
#endif /* LOG_PRINT_IMMEDIATELY */

static int_fast8_t log_semaphore   = 0;
static bool        log_lock_failed = false;


/* --- FUNCTIONS --- */

static void log_unlock(void)
{
  log_semaphore--;
  if (log_semaphore < 0) {              /* just for debugging */
    while(1);                           /* not supposed to happen */
  }
}

static bool log_lock(void)
{
  if (log_semaphore == 0) {
    log_semaphore++;
    if (log_semaphore == 1) {
      return true;
    }
    log_lock_failed = true;
    log_unlock();
  }
  return false;
}

#if !LOG_PRINT_IMMEDIATELY

bool log_buffer_empty(void)
{
  return (read_idx == write_idx);
}

bool log_buffer_full(void)
{
  uint32_t w = ((write_idx + 1) == LOG_BUFFER_SIZE) ? 0 : (write_idx + 1);
  return (w == read_idx);
}

uint32_t log_buffer_size(void)
{
  if(write_idx < read_idx) {
    return (LOG_BUFFER_SIZE - read_idx + write_idx);
  }
  return (write_idx - read_idx);
}

uint32_t log_buffer_space(void)
{
  return (LOG_BUFFER_SIZE - log_buffer_size());
}

bool log_flush(void)
{
  static uint_fast8_t flush_lock = 0;
  flush_lock++;
  if (flush_lock != 1) {
    flush_lock--;
    return false;
  }
  bool buffer_full = log_buffer_full();
  if (!log_buffer_empty()) {
    uint32_t n = log_buffer_size();
    uint32_t m = (LOG_BUFFER_SIZE - read_idx);
    if (!LOG_PRINT_FUNC(&log_buffer[read_idx], MIN(n, m))) {
      flush_lock--;
      return false;
    }
    if (m < n) {
      if (!LOG_PRINT_FUNC(&log_buffer[0], n - m)) {
        flush_lock--;
        return false;
      }
    }
    read_idx += n;
    if(read_idx >= LOG_BUFFER_SIZE) {
      read_idx -= LOG_BUFFER_SIZE;
    }
  }
  if (buffer_full) {
    LOG_PRINT_FUNC("~", 1);         /* to indicate that the queue was full! */
  }
  if (log_lock_failed) {
    LOG_PRINT_FUNC(LOG_ERR_MSG_LOCK_FAILED, sizeof(LOG_ERR_MSG_LOCK_FAILED));
    log_lock_failed = false;
  }
  flush_lock--;
  return true;
}

#endif /* LOG_PRINT_IMMEDIATELY */

/* adds a string to the log buffer (private function!) */
static void log_buffer_add(const char* str, uint32_t len)
{
  if (!str) {
    return;
  }
  if (!len) {
    len = strlen(str);
  }
#if LOG_PRINT_IMMEDIATELY
  LOG_PRINT_FUNC((char*)str, strlen(str));

#else /* LOG_PRINT_IMMEDIATELY */
  if (!log_buffer_full()) {
    uint32_t n = MIN(log_buffer_space(), len);
    uint32_t m = (LOG_BUFFER_SIZE - write_idx);
    memcpy(&log_buffer[write_idx], str, MIN(n, m));
    if(m < n) {
      memcpy(log_buffer, str + m, n - m);
    }
    write_idx += n;
    if(write_idx >= LOG_BUFFER_SIZE) {
      write_idx -= LOG_BUFFER_SIZE;
    }
  }
  LOG_TASK_NOTIFY();
#endif /* LOG_PRINT_IMMEDIATELY */
}

void log_print(const char* str)
{
  if (!log_lock()) {
    return;
  }
  log_buffer_add(str, 0);
  log_unlock();
}

/* puts a zero-terminated string into the TX queue and appends a newline */
void log_println(log_level_t level, const char* module, const char* msg)
{
  if (!log_lock()) {
    return;
  }
  /* first, print the timestamp */
#if LOG_ADD_TIMESTAMP
  static char tmp[32];
  uint32_t relative_time = LPTIMER_NOW_MS();
  snprintf(tmp, sizeof(tmp), "[%7lu] ", relative_time);
  log_buffer_add(tmp, 0);
#endif /* LOG_ADD_TIMESTAMP */

  /* print the log level */
#if LOG_ADD_LEVEL
  if (level == LOG_LEVEL_ERROR) {
 #if LOG_USE_COLORS
    log_buffer_add(LOG_COLOR_RED, 0);
 #endif /* LOG_USE_COLORS */
    log_buffer_add(LOG_LEVEL_ERROR_STR, 0);
  } else if (level == LOG_LEVEL_WARNING) {
 #if LOG_USE_COLORS
    log_buffer_add(LOG_COLOR_YELLOW, 0);
 #endif /* LOG_USE_COLORS */
    log_buffer_add(LOG_LEVEL_WARNING_STR, 0);
  } else if (level == LOG_LEVEL_INFO) {
 #if LOG_USE_COLORS
    log_buffer_add(LOG_COLOR_WHITE, 0);
 #endif /* LOG_USE_COLORS */
    log_buffer_add(LOG_LEVEL_INFO_STR, 0);
  } else if (level == LOG_LEVEL_VERBOSE) {
    log_buffer_add(LOG_LEVEL_VERBOSE_STR, 0);
  }
#endif /* LOG_ADD_LEVEL */

#if LOG_ADD_MODULE
  if (module) {
    uint32_t len = 0;
    const char* ext = strrchr(module, '.');
    if (ext) {
      len = (uint32_t)ext - (uint32_t)module;
    }
    /* print the module */
    log_buffer_add(module, len);
    log_buffer_add(": ", 2);
  }
#endif /* LOG_ADD_MODULE */

  /* print the message */
  log_buffer_add(msg, 0);

#if LOG_USE_COLORS
  // reset color
  log_buffer_add(LOG_COLOR_RESET, 0);
#endif /* LOG_USE_COLORS */

  /* print the newline */
  log_buffer_add(LOG_NEWLINE, 0);

  /* release lock */
  log_unlock();
}

/* composes a zero-terminated string with arguments and writes it into the TX queue, newline is appended */
void log_printfln(log_level_t level, const char* module, const char* msg, ...)
{
  va_list lst;
  /* since a common buffer is used, it must be protected with a semaphore */
  va_start(lst, msg);
  vsnprintf(log_print_buffer, sizeof(log_print_buffer), msg, lst);
  va_end(lst);
  log_println(level, module, log_print_buffer);
}

void log_println_ext(log_level_t level, const char* module, const char* msg, const char* src_file, uint32_t src_line)
{
  /* print the file and line number */
  if (src_file) {
    snprintf(log_print_buffer, sizeof(log_print_buffer), "%s (%s, line %lu)", msg, src_file, src_line);
    log_println(level, module, log_print_buffer);
  } else {
    log_println(level, module, msg);
  }
}

void log_marker(log_level_t level, const char* module, uint32_t marker)
{
  char tmp[12];   /* 32-bit integer takes less than 12 characters to print */
  snprintf(tmp, 12, "%lu", marker);
  log_println(level, module, tmp);
}

#endif /* LOG_ENABLE */

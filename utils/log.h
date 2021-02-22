/*
 * log.h
 *
 *  Created on: Apr 7, 2020
 *      Author: rdaforno
 */

#ifndef UTILS_LOG_H_
#define UTILS_LOG_H_


/* --- default CONFIG --- */

#ifndef LOG_ENABLE
#define LOG_ENABLE              1
#endif /* LOG_ENABLE */

/* log level */
#ifndef LOG_LEVEL
#define LOG_LEVEL               LOG_LEVEL_INFO
#endif /* LOG_LEVEL */

/* whether to print immediately or postpone it (write to intermediate buffer)
 * note: also works if DMA is enabled (DMA transfer will then only be initiated when calling log_flush) */
#ifndef LOG_PRINT_IMMEDIATELY
#define LOG_PRINT_IMMEDIATELY   0
#endif /* LOG_PRINT_IMMEDIATELY */

/* prepend a timestamp to all printed lines (applies to debug_println only) */
#ifndef LOG_ADD_TIMESTAMP
#define LOG_ADD_TIMESTAMP       1
#endif /* LOG_ADD_TIMESTAMP */

/* whether or not to print the log level on each line */
#ifndef LOG_ADD_LEVEL
#define LOG_ADD_LEVEL           1
#endif /* LOG_PRINT_LEVEL */

#ifndef LOG_USE_COLORS
#define LOG_USE_COLORS          1
#endif /* LOG_USE_COLORS */

/* whether or not to print the log level on each line */
#ifndef LOG_ADD_MODULE
#define LOG_ADD_MODULE          1
#endif /* LOG_ADD_MODULE */

/* total buffer size */
#ifndef LOG_BUFFER_SIZE
#define LOG_BUFFER_SIZE         4096
#endif /* LOG_BUFFER_SIZE */

/* max. number of chars per line, defines the buffer size used for sprintf() */
#ifndef LOG_LINE_LENGTH
#define LOG_LINE_LENGTH         256
#endif /* LOG_LINE_LENGTH */

#ifndef LOG_USE_DMA
#define LOG_USE_DMA             0
#endif /* LOG_USE_DMA */

/* the actual print function to use, must take 2 arguments (str and length) and return a boolean value (true = success, false = failed) */
#ifndef LOG_PRINT_FUNC
 #if LOG_USE_DMA
  #define LOG_PRINT_FUNC          uart_tx
 #else /* LOG_USE_DMA */
  #define LOG_PRINT_FUNC          uart_tx_direct
 #endif /* LOG_USE_DMA */
#endif /* LOG_PRINT_FUNC */

#ifndef LOG_NEWLINE
#define LOG_NEWLINE             "\r\n"
#endif /* LOG_NEWLINE */

#ifndef LOG_QUEUE_FULL_MARK
#define LOG_QUEUE_FULL_MARK     "\\~"
#endif /* LOG_QUEUE_FULL_MARK */

#if LOG_ADD_LEVEL
  #ifndef LOG_LEVEL_ERROR_STR
    #define LOG_LEVEL_ERROR_STR   "ERROR  "
    #define LOG_LEVEL_WARNING_STR "WARN   "
    #define LOG_LEVEL_INFO_STR    "INFO   "
    #define LOG_LEVEL_VERBOSE_STR "DEBUG  "
  #endif /* LOG_LEVEL_ERROR_STR */
#else /* LOG_ADD_LEVEL */
  #define LOG_LEVEL_ERROR_STR
  #define LOG_LEVEL_WARNING_STR
  #define LOG_LEVEL_INFO_STR
  #define LOG_LEVEL_VERBOSE_STR
#endif /* LOG_ADD_LEVEL */

#define LOG_ERR_MSG_LOCK_FAILED (LOG_NEWLINE "--- ERROR failed to acquire log lock ---" LOG_NEWLINE)

#if LOG_ADD_MODULE
  #define LOG_MODULE_NAME       LOG_SRCFILENAME
#else /* LOG_ADD_MODULE */
  #define LOG_MODULE_NAME       0
#endif /* LOG_ADD_MODULE */

#define LOG_SRCFILENAME         (strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1) : (__FILE__))
#define LOG_SRCLINENO           (__LINE__)

/* in case a FreeRTOS task is supposed to be polled every time a message is written to the buffer, define LOG_TASK_HANDLE */
#ifdef LOG_TASK_HANDLE
  #define LOG_TASK_NOTIFY()     if (LOG_TASK_HANDLE && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)) { \
                                  if (IS_INTERRUPT()) {\
                                    xTaskNotifyFromISR(LOG_TASK_HANDLE, 0, eNoAction, 0);\
                                  } else {\
                                    xTaskNotify(LOG_TASK_HANDLE, 0, eNoAction);\
                                  }\
                                }
#elif !defined(LOG_TASK_NOTIFY)
  #define LOG_TASK_NOTIFY()
#endif /* LOG_TASK_HANDLE != 0 */

/* color codes */
#define LOG_COLOR_BLACK         "\e[30m"
#define LOG_COLOR_RED           "\e[31m"
#define LOG_COLOR_GREEN         "\e[32m"
#define LOG_COLOR_YELLOW        "\e[33m"
#define LOG_COLOR_BLUE          "\e[34m"
#define LOG_COLOR_WHITE         "\e[37m"
#define LOG_COLOR_GRAY          "\e[90m"
#define LOG_COLOR_RESET         "\e[0m"


/* --- TYPEDEFS --- */

#define LOG_LEVEL_QUIET         0
#define LOG_LEVEL_ERROR         1
#define LOG_LEVEL_WARNING       2
#define LOG_LEVEL_INFO          3
#define LOG_LEVEL_VERBOSE       4

typedef enum {
  /* this mapping is required since the preprocessor can't handle typedef enum in #if statements */
  LOG_LVL_QUIET   = LOG_LEVEL_QUIET,
  LOG_LVL_ERROR   = LOG_LEVEL_ERROR,
  LOG_LVL_WARNING = LOG_LEVEL_WARNING,
  LOG_LVL_INFO    = LOG_LEVEL_INFO,
  LOG_LVL_VERBOSE = LOG_LEVEL_VERBOSE
} log_level_t;


/* --- MACROS --- */

#if LOG_ENABLE && LOG_LEVEL > LOG_LEVEL_QUIET
  #define LOG_ERROR(...)          log_printfln(LOG_LEVEL_ERROR, LOG_MODULE_NAME, __VA_ARGS__)
  #define LOG_ERROR_CONST(str)    log_println(LOG_LEVEL_ERROR, LOG_MODULE_NAME, str)
  #define LOG_ERROR_MARKER()      log_marker(LOG_LEVEL_ERROR, LOG_MODULE_NAME, __LINE__)
#else
  #define LOG_ERROR(...)
  #define LOG_ERROR_CONST(str)
  #define LOG_ERROR_MARKER()
#endif

#if LOG_ENABLE && LOG_LEVEL > LOG_LEVEL_ERROR
  #define LOG_WARNING(...)        log_printfln(LOG_LEVEL_WARNING, LOG_MODULE_NAME, __VA_ARGS__)
  #define LOG_WARNING_CONST(str)  log_println(LOG_LEVEL_WARNING, LOG_MODULE_NAME, str)
  #define LOG_WARNING_MARKER()    log_marker(LOG_LEVEL_WARNING, LOG_MODULE_NAME, __LINE__)
#else
  #define LOG_WARNING(...)
  #define LOG_WARNING_CONST(str)
  #define LOG_WARNING_MARKER()
#endif

#if LOG_ENABLE && LOG_LEVEL > LOG_LEVEL_WARNING
  #define LOG_INFO(...)           log_printfln(LOG_LEVEL_INFO, LOG_MODULE_NAME, __VA_ARGS__)
  #define LOG_INFO_CONST(str)     log_println(LOG_LEVEL_INFO, LOG_MODULE_NAME, str)
  #define LOG_INFO_MARKER()       log_marker(LOG_LEVEL_INFO, LOG_MODULE_NAME, __LINE__)
#else
  #define LOG_INFO(...)
  #define LOG_INFO_CONST(str)
  #define LOG_INFO_MARKER()
#endif

#if LOG_ENABLE && LOG_LEVEL > LOG_LEVEL_INFO
  #define LOG_VERBOSE(...)        log_printfln(LOG_LEVEL_VERBOSE, LOG_MODULE_NAME, __VA_ARGS__)
  #define LOG_VERBOSE_CONST(str)  log_println(LOG_LEVEL_VERBOSE, LOG_MODULE_NAME, str)
  #define LOG_VERBOSE_MARKER()    log_marker(LOG_LEVEL_VERBOSE, LOG_MODULE_NAME, __LINE__)
#else
  #define LOG_VERBOSE(...)
  #define LOG_VERBOSE_CONST(str)
  #define LOG_VERBOSE_MARKER()
#endif

/* print a string without any additional formatting by the log module (independent of log level) */
#if LOG_ENABLE
  #define LOG_RAW(str)            log_print(str)
#else
  #define LOG_RAW(str)
#endif


/* --- FUNCTIONS --- */

/* add a string to the log buffer (unformatted, i.e. without timestamp or log level) */
void log_print(const char* str);

/* add a constant string to the log buffer, a newline is appended */
void log_println(log_level_t level, const char* module, const char* msg);

/* compose a string with additional arguments and add it to the log buffer, a newline is appended */
void log_printfln(log_level_t level, const char* module, const char* msg, ...);

/* extended println with additional parameters */
void log_println_ext(log_level_t level, const char* module, const char* msg, const char* src_file, uint32_t src_line);

/* log a marker (e.g. error code or line number) */
void log_marker(log_level_t level, const char* module, uint32_t marker);

#if !LOG_PRINT_IMMEDIATELY || LOG_USE_DMA

/* prints out all queued data in the log buffer; returns true on success, false otherwise (e.g. when UART device is unavailable) */
bool log_flush(void);

/* get infos about the log buffer */
bool log_buffer_empty(void);
bool log_buffer_full(void);
uint32_t log_buffer_size(void);
uint32_t log_buffer_space(void);

#endif /* LOG_PRINT_IMMEDIATELY */


#endif /* UTILS_LOG_H_ */

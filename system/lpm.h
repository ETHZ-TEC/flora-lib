/*
 * lpm.h
 *
 * low-power mode manager
 */

#ifndef SYSTEM_LPM_H_
#define SYSTEM_LPM_H_


#ifndef LPM_ON_IND
#define LPM_ON_IND()
#define LPM_OFF_IND()
#endif /* LPM_ON_IND */

#ifndef LOW_POWER_MODE
#define LOW_POWER_MODE              LP_MODE_SLEEP
#endif /* LOW_POWER_MODE */

#ifndef LPM_DISABLE_GPIO_CLOCKS
#define LPM_DISABLE_GPIO_CLOCKS     1
#endif /* LPM_DISABLE_GPIO_CLOCKS */

#ifndef LPM_RADIO_COLD_SLEEP
#define LPM_RADIO_COLD_SLEEP        1
#endif /* LPM_RADIO_COLD_SLEEP */

#ifndef LPM_DISABLE_SWO_PIN
#define LPM_DISABLE_SWO_PIN         0
#endif /* LPM_DISABLE_SWO_PIN */

#ifndef LPM_DISABLE_DEBUG
#define LPM_DISABLE_DEBUG           0
#endif /* LPM_DISABLE_DEBUG */


typedef enum
{
  OP_MODE_RESET,            /* initial state after a reset */
  OP_MODE_ACTIVE,           /* default mode, running */
  OP_MODE_IDLE,             /* nothing to do, ready for LPM entry */
  OP_MODE_LPM,              /* in LPM */
  OP_MODE_WOKEN,            /* woken up, waiting for restore of MCU config */
  NUM_OP_MODES,
} op_mode_t;

typedef enum
{
  OP_MODE_EVT_INIT,         /* initialization done */
  OP_MODE_EVT_WAKEUP,       /* wakeup trigger occurred */
  OP_MODE_EVT_DONE,         /* communication round finished */
  OP_MODE_EVT_STOPPED,      /* everything turned off, prepared for LPM */
  OP_MODE_EVT_RESTORED,     /* config restored */
  NUM_OP_MODE_EVENTS,
} op_mode_event_t;

typedef enum
{
  LP_MODE_SLEEP,            /* Sleep Mode */
  LP_MODE_STOP2,            /* Stop2 */
  LP_MODE_STANDBY,          /* Standby Mode */
  LP_MODE_SHUTDOWN,         /* Shutdown Mode */
  NUM_LP_MODES,
} lp_mode_t;


void      lpm_init(bool (* deepsleep_entry_cb)(void),
                   void (* wakeup_cb)(void));           /* init the low-power mode manager (callback functions are optional) */
op_mode_t lpm_get_opmode(void);                         /* get current operating mode */
void      lpm_update_opmode(op_mode_event_t evt);       /* update the operating mode based on the given event */
void      lpm_set_prepare_cb(bool (* callback)(void));  /* callback function to execute in lpm_prepare() before entering deepsleep mode, the return value determine whether deepsleep mode is entered */
void      lpm_set_resume_cb(void (* callback)(void));   /* callback function to execute in lpm_resume() after wake-up from deepsleep mode */
bool      lpm_prepare(void);                            /* prepare low-power mode entry */
void      lpm_resume(void);                             /* resume from low-power mode */


#endif /* SYSTEM_LPM_H_ */

/*
 * flocklab.h
 *
 *  Created on: Feb 12, 2019
 *      Author: kelmicha
 */

#ifndef FLOCKLAB_FLOCKLAB_H_
#define FLOCKLAB_FLOCKLAB_H_

#ifndef FLOCKLAB
#define FLOCKLAB      0
#endif /* FLOCKLAB */

extern volatile uint16_t FLOCKLAB_NODE_ID;

#if FLOCKLAB

/* use the flocklab node ID */
#ifndef NODE_ID
#define NODE_ID       FLOCKLAB_NODE_ID
#endif /* NODE_ID */

#ifndef FLOCKLAB_SWD
#define FLOCKLAB_SWD  0
#endif /* FLOCKLAB_SWD */

#ifndef FLOCKLAB_SIG_INT
#define FLOCKLAB_SIG_INT  0           /* enable SIG1 pin interrupt? */
#endif /* FLOCKLAB_SIG_INT */

/* don't print color codes */
#ifndef LOG_USE_COLORS
#define LOG_USE_COLORS  0
#endif /* LOG_USE_COLORS */

/* error check */
#if BOLT_ENABLE
/* SWO cannot be used at the moment since the pin is connected to the DIO1 of the SX1262 radio */
#error "can't use BOLT or SWO on FlockLab"
#endif


/* pin mapping for rev1.1 adapter (small black PCB) */
#define FLOCKLAB_SIG1_Pin           GPIO_PIN_0      /* = BOLT_IND_Pin */
#define FLOCKLAB_SIG1_GPIO_Port     GPIOA
#define FLOCKLAB_SIG2_Pin           GPIO_PIN_4      /* = APP_IND_Pin */
#define FLOCKLAB_SIG2_GPIO_Port     GPIOA
#define FLOCKLAB_INT1_Pin           GPIO_PIN_3      /* = COM_TREQ_Pin */
#define FLOCKLAB_INT1_GPIO_Port     GPIOA
#define FLOCKLAB_INT2_Pin           GPIO_PIN_14     /* = COM_PROG_Pin (SWDCLK) */
#define FLOCKLAB_INT2_GPIO_Port     GPIOA
#define FLOCKLAB_LED3_Pin           GPIO_PIN_13     /* = COM_PROG2_Pin (SWDIO) */
#define FLOCKLAB_LED3_GPIO_Port     GPIOA
#define FLOCKLAB_LED2_Pin           GPIO_PIN_3      /* = COM_GPIO2_Pin (SWO) */
#define FLOCKLAB_LED2_GPIO_Port     GPIOB
#define FLOCKLAB_LED1_Pin           GPIO_PIN_3      /* = COM_GPIO1_Pin (RFDIO1) */
#define FLOCKLAB_LED1_GPIO_Port     GPIOH


#define FLOCKLAB_PIN_SET(p)         PIN_SET(p)
#define FLOCKLAB_PIN_CLR(p)         PIN_CLR(p)
#define FLOCKLAB_PIN_GET(p)         PIN_GET(p)
#define FLOCKLAB_PIN_TOGGLE(p)      PIN_TOGGLE(p)


typedef enum {
  FLOCKLAB_INT1 = FLOCKLAB_INT1_Pin,
#if !FLOCKLAB_SWD
  FLOCKLAB_INT2 = FLOCKLAB_INT2_Pin,
  FLOCKLAB_LED3 = FLOCKLAB_LED3_Pin,
#endif /* !FLOCKLAB_SWD */
} flocklab_trace_pin_t;

typedef enum {
  FLOCKLAB_SIG1 = FLOCKLAB_SIG1_Pin,
  FLOCKLAB_SIG2 = FLOCKLAB_SIG2_Pin,
} flocklab_actuation_pin_t;


/*
 * init GPIOs for Flocklab
 */
void flocklab_init(void);

/*
 * sets all output pins back to zero
 */
void flocklab_reset_pins(void);

/*
 * set a flocklab pin to 1 and right back to 0 for count times
 * pin: one of the pins defined in flocklab_trace_pin_t
 * count: number of flashes
 */
void flocklab_blink(flocklab_trace_pin_t pin, uint8_t count);

/*
 * returns the node ID (= FlockLab observer ID)
 */
uint16_t flocklab_node_id(void);


#else /* FLOCKLAB */

#define FLOCKLAB_PIN_SET(p)
#define FLOCKLAB_PIN_CLR(p)
#define FLOCKLAB_PIN_TOGGLE(p)

#define flocklab_reset_pins()
#define flocklab_blink(p, c)

#endif /* FLOCKLAB */

#endif /* FLOCKLAB_FLOCKLAB_H_ */

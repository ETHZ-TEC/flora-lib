/*
 * misc.h
 *
 *  Created on: Aug 22, 2019
 *      Author: rdaforno
 */

#ifndef UTILS_MISC_H_
#define UTILS_MISC_H_


#ifndef MIN
#define MIN(a, b)     ((a) < (b) ? (a) : (b))
#endif /* MIN */

#ifndef MAX
#define MAX(a, b)     ((a) > (b) ? (a) : (b))
#endif /* MAX */

#ifndef ABS
#define ABS(a)        ((a) >= 0 ? (a) : -(a))
#endif /* ABS */

#ifndef RAND
#define RAND()        random_rand16()
#endif /* RAND */


#define IS_INTERRUPT()        ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0)
#define INTERRUPTS_ENABLED()  ((__get_PRIMASK() & 0x1) == 0)

#define SUSPEND_SYSTICK()     CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk)
#define RESUME_SYSTICK()      SET_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk)

#define SUSPEND_HALTICK()     HAL_SuspendTick()
#define RESUME_HALTICK()      HAL_ResumeTick(); NVIC_EnableIRQ(HALTICK_IRQ)

#define REGVAL16(addr)        (*(volatile uint16_t*)(addr))
#define REGVAL32(addr)        (*(volatile uint32_t*)(addr))
#define REGVAL64(addr)        (*(volatile uint64_t*)(addr))    /* note: this can lead to undesired results if the address is not aligned to 8 bytes */
#define REGVAL                REGVAL32
#define ADDR_VAL16            REGVAL16
#define ADDR_VAL32            REGVAL32
#define ADDR_VAL64            REGVAL64
#define ADDR_VAL              REGVAL32

#define ADDR_OFS(addr, ofs)  ((void*)((uint32_t)addr + ofs))

/* can be used to execute a piece of code atomically (without interruption) */
#define ENTER_CRITICAL_SECTION()    uint32_t mask; mask = __get_PRIMASK(); __disable_irq()
#define LEAVE_CRITICAL_SECTION()    __set_PRIMASK(mask);

#define MASK_INTERRUPTS(priority)   __set_BASEPRI(priority)  /* prevents interrupts with given or lower priority (higher number = lower priority) */
#define UNMASK_INTERRUPTS()         __set_BASEPRI(0)         /* allow all interrupts */


typedef volatile uint8_t semaphore_t;


void delay(volatile uint32_t loop_passes);
void delay_us(volatile uint32_t us);

bool swo_println(const char* str);
bool swo_print(const char* str, uint32_t len);

void     random_init(void);
uint32_t random_rand16(void);        /* returns a 16-bit random number (c-lib) */
#ifdef HAL_RNG_MODULE_ENABLED
uint32_t random_rand32(void);        /* returns a 32-bit random number (only available if the hardware RNG is enabled) */
#endif /* HAL_RNG_MODULE_ENABLED */

bool check_hal_tick(void);           /* returns true if the HAL tick is enabled */

bool semaphore_acquire(semaphore_t* sem);
void semaphore_release(semaphore_t* sem);


#endif /* UTILS_MISC_H_ */

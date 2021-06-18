/*
 * Copyright (c) 2018 - 2021, ETH Zurich, Computer Engineering Group (TEC)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
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
#define RAND()        random_rand32()
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
uint32_t random_rand32(void);        /* returns a 32-bit random number */

bool check_hal_tick(void);           /* returns true if the HAL tick is enabled */

bool semaphore_acquire(semaphore_t* sem);
void semaphore_release(semaphore_t* sem);


#endif /* UTILS_MISC_H_ */

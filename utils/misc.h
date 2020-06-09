/*
 * misc.h
 *
 *  Created on: Aug 22, 2019
 *      Author: rdaforno
 */

#ifndef UTILS_MISC_H_
#define UTILS_MISC_H_


#define IS_INTERRUPT()        ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0)

#define SUSPEND_SYSTICK()     CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk)
#define RESUME_SYSTICK()      SET_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk)

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


void delay(volatile uint32_t loop_passes);
void delay_us(volatile uint32_t us);

bool swo_println(const char* str);
bool swo_print(const char* str, uint32_t len);

#endif /* UTILS_MISC_H_ */

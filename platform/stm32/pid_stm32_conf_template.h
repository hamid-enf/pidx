/**
 * @file    pid_stm32_conf_template.h
 * @brief   Per-project configuration for the PIDX STM32 platform layer.
 *
 * HOW TO USE
 *   1. Copy this file into your own project as "pid_stm32_conf.h".
 *   2. Put its directory on the include path.
 *   3. Edit the defines below to match your device, clock tree and timer.
 *
 * Do NOT edit this template in place: it is part of the library and will be
 * overwritten when you update PIDX. Everything the platform layer needs to
 * know about your board lives in your copy, so the library itself stays
 * device-independent.
 *
 * Nothing in here is visible to the PIDX core. The core (src/, include/pidx/)
 * contains no reference to HAL, CMSIS, SysTick or DWT; if you do not need a
 * platform layer at all you can delete platform/ entirely.
 */
#ifndef PIDX_PID_STM32_CONF_H
#define PIDX_PID_STM32_CONF_H

/* ======================================================================== */
/* 1. Device header                                                          */
/* ======================================================================== */

/**
 * The CMSIS device header for your part. This is the *CMSIS* header, not the
 * HAL one: the platform layer only needs the peripheral struct definitions
 * (TIM_TypeDef, DWT, CoreDebug) and the intrinsics. It never calls HAL_*.
 *
 * Examples: "stm32f103xb.h", "stm32f407xx.h", "stm32g474xx.h",
 *           "stm32h743xx.h", "stm32l432xx.h"
 * Including "stm32f4xx.h" (the family header) also works.
 */
#define PIDX_STM32_DEVICE_HEADER   "stm32f4xx.h"

/* ======================================================================== */
/* 2. Timebase selection                                                     */
/* ======================================================================== */

/*  PIDX_STM32_TB_TIM      A general-purpose timer free-running at 1 MHz.
 *                         Best default. 1 us resolution, costs one timer,
 *                         works on every STM32 including M0/M0+.
 *
 *  PIDX_STM32_TB_DWT      The Cortex-M DWT cycle counter.
 *                         Sub-microsecond, costs no peripheral, but ARMv7-M
 *                         only: NOT available on Cortex-M0/M0+ (STM32F0, G0,
 *                         L0, C0) and can be disabled by some debug/security
 *                         configurations.
 *
 *  PIDX_STM32_TB_CALLBACK You supply the microsecond counter yourself, e.g.
 *                         from an RTOS or an existing timer. Set the function
 *                         with PIDs_TimebaseInitCallback().
 */
#define PIDX_STM32_TIMEBASE        PIDX_STM32_TB_TIM

/* ---- settings for PIDX_STM32_TB_TIM ---- */

/**
 * Timer instance used as the timebase. Pick a 32-bit timer if you have one
 * (TIM2/TIM5 on F4/F7/H7, TIM2 on G4): its 1 MHz counter then wraps every
 * 71.6 minutes instead of every 65.5 ms. The layer detects the width at run
 * time, so a 16-bit timer also works - see the wrap notes in pid_stm32.h.
 *
 * The timer must not be used for anything else. Its clock must be enabled by
 * your code (__HAL_RCC_TIMx_CLK_ENABLE() or the equivalent LL/register write)
 * BEFORE calling PIDs_TimebaseInitTim().
 */
#define PIDX_STM32_TIM             TIM2

/**
 * Input clock of that timer in Hz, i.e. the APBx timer clock, NOT SystemCoreClock.
 * On most STM32s, when the APB prescaler is not 1 the timer clock is twice the
 * APB clock. Get it right or every dt in the system is wrong by that factor.
 *
 * If you use HAL you can pass HAL_RCC_GetPCLK1Freq() * 2 at run time to
 * PIDs_TimebaseInitTim() instead of relying on this constant.
 */
#define PIDX_STM32_TIMER_CLK_HZ    84000000UL

/* ---- settings for PIDX_STM32_TB_DWT and for the cycle profiler ---- */

/** CPU clock in Hz (SystemCoreClock). Used to convert cycles to microseconds. */
#define PIDX_STM32_CORE_CLK_HZ     168000000UL

/**
 * Set to 1 on ARMv7-M cores (M3/M4/M7 -> STM32F1/F2/F3/F4/F7/H7/G4/L4/L5/U5)
 * to make the DWT cycle counter and the cycle profiler available.
 * Set to 0 on Cortex-M0/M0+ (STM32F0/G0/L0/C0): there is no DWT there, and the
 * profiler then falls back to the microsecond timebase.
 */
#define PIDX_STM32_HAS_DWT         1

/**
 * Cortex-M7 (and some M33 parts) gate DWT register writes behind a lock.
 * Set to 1 on H7/F7 if PIDs_CycleInit() reports PID_ERR_UNSUPPORTED.
 */
#define PIDX_STM32_DWT_HAS_LAR     0

/* ======================================================================== */
/* 3. Critical sections                                                      */
/* ======================================================================== */

/**
 * 0 -> PRIMASK: masks every interrupt including the highest priority ones.
 *      Simple, works on M0/M0+, but adds latency to unrelated ISRs.
 * n>0 -> BASEPRI: masks only interrupts with a priority number >= n, so your
 *      fastest ISRs keep running. ARMv7-M only. The value is a raw BASEPRI
 *      value, already shifted for the implemented priority bits: with 4 bits
 *      of priority (the usual STM32 setting) use (level << 4).
 *
 * PIDX only needs a critical section when a 64-bit timestamp is read from more
 * than one context; the control path itself takes no locks.
 */
#define PIDX_STM32_CRITICAL_BASEPRI  0

#endif /* PIDX_PID_STM32_CONF_H */

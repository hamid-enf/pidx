/**
 * @file    pid_stm32.h
 * @brief   STM32 / Cortex-M integration layer for PIDX: timebase, cycle
 *          profiler, non-blocking rate driver and ISR load monitor.
 *
 * WHAT THIS IS
 *   An OPTIONAL adapter. The PIDX core (src/, include/pidx/) contains zero
 *   references to HAL, LL, CMSIS, SysTick or DWT - verified by grep as part of
 *   the build gate. Delete platform/ and the library still builds and passes
 *   its tests. This file exists only to answer the questions the core refuses
 *   to answer on its own: "what time is it" and "how long did that take".
 *
 * WHAT IT DOES NOT DO
 *   It does not configure clocks, GPIO, ADC, PWM or DMA, and it never calls
 *   HAL_Init/HAL_GetTick. Peripheral bring-up stays in your application, where
 *   your CubeMX-generated code already lives. The layer touches exactly one
 *   timer (the one you name in the config) plus, optionally, the DWT.
 *
 * SETUP
 *   1. Copy pid_stm32_conf_template.h to your project as "pid_stm32_conf.h"
 *      and edit it.
 *   2. Add platform/stm32/pid_stm32.c to the build and this directory plus
 *      your conf directory to the include path.
 *   3. Enable the timer's clock, then call PIDs_TimebaseInitTim() once.
 *
 * HONEST STATUS
 *   This layer is compiled and syntax/logic-checked on the host against a
 *   CMSIS stub (tests/test_stm32_host.c) - the wrap-extension arithmetic, the
 *   rate driver and the load monitor are exercised with a simulated counter.
 *   It has NOT been run on silicon in this workspace, because no ARM toolchain
 *   or hardware is available here. Treat the register-level bring-up as
 *   reviewed-but-unflashed code, and verify it on your board before trusting
 *   it in a machine that can hurt something.
 */
#ifndef PIDX_PID_STM32_H
#define PIDX_PID_STM32_H

#include <stdint.h>
#include <stdbool.h>

#include "pidx/pid.h"

/* Timebase source identifiers. Defined before the user config is included,
 * because the config selects one of them by name. */
#define PIDX_STM32_TB_TIM        1
#define PIDX_STM32_TB_DWT        2
#define PIDX_STM32_TB_CALLBACK   3

#include "pid_stm32_conf.h"

#ifndef PIDX_STM32_TIMEBASE
#error "pid_stm32_conf.h must define PIDX_STM32_TIMEBASE"
#endif

/* The CMSIS device header. Replaced by a stub in the host syntax test. */
#include PIDX_STM32_DEVICE_HEADER

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* 1. Timebase                                                               */
/* ======================================================================== */

/**
 * Start the microsecond timebase on a general-purpose timer.
 *
 * The timer is put in free-running up-counting mode with the prescaler set so
 * that the counter increments at exactly 1 MHz, and ARR at its maximum. No
 * interrupt is enabled and no NVIC line is used: the counter is only ever
 * read.
 *
 * @param tim           Timer instance, e.g. TIM2. Must not be shared.
 * @param timer_clk_hz  Clock feeding that timer, in Hz. This is the APB timer
 *                      clock, which on most STM32s is 2x PCLKx when the APB
 *                      prescaler is not 1 - it is NOT SystemCoreClock. A wrong
 *                      value scales every dt in the system by the same error.
 *                      Must be a multiple of 1 MHz, at least 1 MHz.
 *
 * The caller must have enabled the timer's peripheral clock first.
 *
 * @return PID_OK, PID_ERR_NULL, or PID_ERR_INVALID_PARAM if the clock cannot
 *         produce exactly 1 MHz with an integer prescaler.
 *
 * 16-bit vs 32-bit timers: the width is detected at run time. A 32-bit timer
 * (TIM2/TIM5 on F4/F7/H7, TIM2 on G4) wraps after 71.6 minutes. A 16-bit timer
 * wraps every 65.536 ms, so PIDs_NowUs() must be called at least twice per
 * wrap to keep the 64-bit extension correct - trivially true for a control
 * loop, but not for code that idles for a second and then asks the time.
 */
PID_StatusCode PIDs_TimebaseInitTim(TIM_TypeDef *tim, uint32_t timer_clk_hz);

#if (PIDX_STM32_HAS_DWT)
/**
 * Start the microsecond timebase on the DWT cycle counter instead of a timer.
 *
 * Costs no peripheral and has cycle resolution, but is ARMv7-M only (no
 * Cortex-M0/M0+) and can be unavailable when the debug unit is locked down.
 * The counter is 32-bit in CPU cycles, so at 168 MHz it wraps every 25.6 s.
 *
 * @param core_clk_hz  CPU clock in Hz (SystemCoreClock).
 * @return PID_OK, PID_ERR_INVALID_PARAM, or PID_ERR_UNSUPPORTED if the cycle
 *         counter refuses to run (locked debug unit, no trace hardware).
 */
PID_StatusCode PIDs_TimebaseInitDwt(uint32_t core_clk_hz);
#endif

/**
 * Use a microsecond counter you already have (RTOS tick hook, another driver,
 * an LPTIM in low-power mode...).
 *
 * @param fn            Returns a free-running microsecond count. It may wrap;
 *                      the wrap-extension in PIDs_NowUs() handles that as long
 *                      as it is called often enough. Must be safe to call from
 *                      any context.
 * @param counter_mask  Width of that counter as an all-ones mask: 0xFFFF for a
 *                      16-bit source such as an LPTIM, 0xFFFFFFFF for a 32-bit
 *                      one. Must be 2^n - 1 and at least 0xFFFF. Getting this
 *                      wrong breaks PIDs_DeltaUs() across a wrap, which shows
 *                      up as one absurd dt every counter period.
 */
PID_StatusCode PIDs_TimebaseInitCallback(uint32_t (*fn)(void),
                                         uint32_t counter_mask);

/**
 * Width of the active timebase counter, as an all-ones mask. 0 when no
 * timebase is installed. Exposed so an application can check that its loop
 * period leaves enough margin below the wrap period.
 */
uint32_t PIDs_CounterMask(void);

/** True once a timebase has been installed. */
bool PIDs_TimebaseReady(void);

/**
 * Raw free-running microsecond counter, masked to the hardware width.
 *
 * Lock-free and safe from any context including an ISR. Wraps; use
 * PIDs_DeltaUs() to subtract two samples rather than plain `b - a`, which is
 * wrong on a 16-bit timer.
 */
uint32_t PIDs_NowUs32(void);

/**
 * Wrap-safe difference between two PIDs_NowUs32() samples, in microseconds.
 * Correct as long as less than one full counter period elapsed between them.
 */
uint32_t PIDs_DeltaUs(uint32_t earlier, uint32_t later);

/**
 * Monotonic 64-bit microsecond time.
 *
 * Extends the hardware counter by tracking wraps, so it never rolls over in
 * any practical lifetime. Because it updates shared state it takes a short
 * critical section (see PIDX_STM32_CRITICAL_BASEPRI); the cost is a handful of
 * cycles. Must be called at least twice per hardware wrap period, otherwise a
 * wrap is missed and time jumps backwards by one period. If your application
 * can go quiet for longer than that, use a 32-bit timer.
 */
uint64_t PIDs_NowUs(void);

/** Monotonic time in seconds, derived from PIDs_NowUs(). */
PID_Float PIDs_Now(void);

/**
 * Busy-wait for the given number of microseconds.
 *
 * For peripheral bring-up (sensor reset pulses and the like) only. Never call
 * it from a control loop: it burns the CPU and, unlike PIDs_RateElapsed(), it
 * drifts by the loop body time on every iteration.
 */
void PIDs_DelayUs(uint32_t us);

/* ======================================================================== */
/* 2. Non-blocking fixed-rate driver (super-loop)                            */
/* ======================================================================== */

/**
 * Absolute-deadline rate driver for a polling main loop.
 *
 * The naive "if (now - last >= period) { last = now; ... }" pattern drifts,
 * because `last` is re-based on the moment the check happened rather than on
 * the intended release time - the loop body's execution time is added to every
 * period. This driver advances an absolute deadline by exactly one period, so
 * the error does not accumulate.
 *
 * On an ISR-driven design you do not need this: the timer interrupt is already
 * the schedule. Use PIDs_IsrMonitor there instead.
 */
typedef struct {
    uint32_t period_us;      /**< Nominal period.                            */
    uint32_t next_deadline;  /**< Absolute deadline, in counter units.       */
    uint32_t last_release;   /**< Counter value at the previous release.     */
    uint32_t last_dt_us;     /**< Measured interval of the last release.     */
    uint32_t iterations;     /**< Releases so far.                           */
    uint32_t overruns;       /**< Releases that were already late.           */
    uint32_t worst_late_us;  /**< Largest observed lateness.                 */
    bool     primed;
} PIDs_Rate;

/** Initialise a rate driver. period_us must be > 0 and below half the
 *  hardware wrap period (32 ms on a 16-bit timer). */
PID_StatusCode PIDs_RateInit(PIDs_Rate *r, uint32_t period_us);

/**
 * @return true exactly when the next period has elapsed; false otherwise.
 *         Poll it as fast as you like. When it returns true, `last_dt_us`
 *         holds the real interval since the previous release - feed
 *         `last_dt_us * 1e-6f` to PID_UpdateDt() if you want the controller to
 *         use the measured interval instead of the nominal one.
 */
bool PIDs_RateElapsed(PIDs_Rate *r);

/* ======================================================================== */
/* 3. Cycle profiler (ARMv7-M only)                                          */
/* ======================================================================== */

#if (PIDX_STM32_HAS_DWT)

/**
 * Enable the DWT cycle counter for profiling.
 *
 * Independent of the timebase: you can profile with DWT while running the
 * timebase off a TIM. Returns PID_ERR_UNSUPPORTED if the counter will not
 * start, which happens when the debug unit is powered down or locked (some
 * H7/M33 configurations - see PIDX_STM32_DWT_HAS_LAR).
 */
PID_StatusCode PIDs_CycleInit(void);

/** Raw CPU cycle count. Wraps every 2^32 cycles (25.6 s at 168 MHz). */
uint32_t PIDs_Cycles(void);

/** Cycle accumulator: min / max / mean over repeated measurements. */
typedef struct {
    uint32_t t0;
    uint64_t total;
    uint32_t min;
    uint32_t max;
    uint32_t count;
} PIDs_CycleStat;

void      PIDs_CycleReset(PIDs_CycleStat *s);
void      PIDs_CycleStart(PIDs_CycleStat *s);
void      PIDs_CycleStop(PIDs_CycleStat *s);
PID_Float PIDs_CycleMean(const PIDs_CycleStat *s);
/** Convert cycles to microseconds using PIDX_STM32_CORE_CLK_HZ. */
PID_Float PIDs_CyclesToUs(uint32_t cycles);

#endif /* PIDX_STM32_HAS_DWT */

/* ======================================================================== */
/* 4. ISR load / jitter monitor                                              */
/* ======================================================================== */

/**
 * Measures what a periodic control ISR actually costs and how regularly it is
 * entered. Call PIDs_IsrEnter() as the first statement of the ISR and
 * PIDs_IsrExit() as the last.
 *
 * Two different faults are separated here, and confusing them is the classic
 * way to misdiagnose a control loop:
 *   - period jitter: the ISR is entered late (a higher-priority handler ran
 *     first). The controller sees a wrong dt.
 *   - execution time: the ISR body itself is too slow. If it approaches the
 *     period, the loop is saturated and everything else in the system starves.
 *
 * Overhead is two counter reads plus a few compares - measured in the host
 * test as fewer than 20 arithmetic operations, but the true cost depends on
 * whether the timebase read is a memory-mapped peripheral access.
 */
typedef struct {
    uint32_t nominal_us;     /**< Expected period, for the jitter reference. */
    uint32_t t_enter;        /**< Counter at the last ISR entry.             */
    uint32_t last_period_us; /**< Measured interval between the last entries.*/
    uint32_t last_exec_us;   /**< Duration of the last ISR body.             */
    uint32_t max_exec_us;    /**< Worst body duration seen.                  */
    uint32_t max_jitter_us;  /**< Worst |measured period - nominal|.         */
    uint64_t exec_total_us;  /**< Sum of body durations, for the load figure.*/
    uint32_t entries;        /**< ISR entries counted.                       */
    bool     primed;
} PIDs_IsrMonitor;

PID_StatusCode PIDs_IsrMonitorInit(PIDs_IsrMonitor *m, uint32_t nominal_us);
void           PIDs_IsrEnter(PIDs_IsrMonitor *m);
void           PIDs_IsrExit(PIDs_IsrMonitor *m);

/**
 * Fraction of CPU time spent inside the monitored ISR, in percent, computed as
 * mean execution time over nominal period. A value near 100 means the loop has
 * no headroom left; above 100 the ISR cannot keep up at all.
 */
PID_Float PIDs_IsrLoadPercent(const PIDs_IsrMonitor *m);

/* ======================================================================== */
/* 5. Critical section                                                       */
/* ======================================================================== */

/**
 * Returns the previous mask state, to be handed back to PIDs_ExitCritical().
 * Save/restore rather than unconditional re-enable, so nesting is safe and a
 * section taken inside an already-masked region does not silently open the
 * interrupt window.
 */
uint32_t PIDs_EnterCritical(void);
void     PIDs_ExitCritical(uint32_t state);

#ifdef __cplusplus
}
#endif

#endif /* PIDX_PID_STM32_H */

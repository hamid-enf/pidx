/**
 * @file    pid_esp32.h
 * @brief   ESP32 / ESP-IDF integration layer for PIDX: timebase, cycle
 *          profiler, non-blocking rate driver and FreeRTOS task helpers.
 *
 * WHAT THIS IS
 *   An OPTIONAL adapter, structurally identical to platform/stm32. The PIDX
 *   core (src/, include/pidx/) contains zero references to ESP-IDF, FreeRTOS
 *   or Xtensa intrinsics - verified by grep in the build gate. Delete
 *   platform/ and the library still builds and passes its tests. This file
 *   exists only to answer the questions the core refuses to answer on its
 *   own: "what time is it" and "how long did that take".
 *
 * WHAT IT DOES NOT DO
 *   It does not configure clocks, GPIO, ADC, LEDC/MCPWM or Wi-Fi, and it
 *   never calls app_main() on your behalf. Peripheral bring-up stays in your
 *   application. The layer touches exactly one thing: the timebase source you
 *   select in pid_esp32_conf.h.
 *
 * WHY ESP32 IS EASIER THAN STM32 HERE
 *   ESP-IDF already provides a 64-bit microsecond clock (esp_timer_get_time)
 *   that is monotonic and wrap-free for 292,000 years. So unlike the STM32
 *   layer, this one needs no 16/32-bit wrap extension and no critical section
 *   around the timebase read. The wrap-safe arithmetic is kept only for the
 *   optional CCOUNT source, which is a genuine 32-bit counter.
 *
 * TIMEBASE OPTIONS (select with PIDX_ESP32_TIMEBASE)
 *   PIDX_ESP32_TB_ESP_TIMER  esp_timer_get_time(). 64-bit us, monotonic,
 *                            survives light sleep, safe from any context.
 *                            The default and the right answer almost always.
 *   PIDX_ESP32_TB_CCOUNT     Xtensa CCOUNT cycle register. Cycle resolution
 *                            and the cheapest possible read, but it is 32-bit
 *                            (wraps every ~17.9 s at 240 MHz), it is PER-CORE
 *                            on the dual-core parts, and it does not advance
 *                            in sleep. Use it for profiling, not for wall
 *                            clock, and only when pinned to one core.
 *   PIDX_ESP32_TB_CALLBACK   Any microsecond counter you already have.
 *
 * DUAL-CORE WARNING
 *   ESP32 and ESP32-S3 have two cores with INDEPENDENT CCOUNT registers that
 *   are not synchronised. A control task that migrates between cores while
 *   using the CCOUNT timebase will observe dt jumps of arbitrary sign. Pin the
 *   task with xTaskCreatePinnedToCore() - PIDe_TaskCreate() below does that
 *   for you - or use the esp_timer source, which is core-agnostic.
 *
 * HONEST STATUS
 *   This layer is compiled and logic-checked on the host against a stub
 *   (tests/test_esp32_host.c) - the rate driver, the load monitor and the
 *   wrap-extension arithmetic are exercised with a simulated counter. It has
 *   NOT been run on real silicon in this workspace, because no Xtensa
 *   toolchain or hardware is available here. Treat it as reviewed-but-unflashed
 *   code and verify it on your board before trusting it with a machine that
 *   can hurt something. The same caveat applies to platform/stm32.
 */
#ifndef PIDX_PID_ESP32_H
#define PIDX_PID_ESP32_H

#include <stdint.h>
#include <stdbool.h>

#include "pidx/pid.h"

/* Timebase source identifiers. Defined before the user config is included,
 * because the config selects one of them by name. */
#define PIDX_ESP32_TB_ESP_TIMER   1
#define PIDX_ESP32_TB_CCOUNT      2
#define PIDX_ESP32_TB_CALLBACK    3

#include "pid_esp32_conf.h"

#ifndef PIDX_ESP32_TIMEBASE
#error "pid_esp32_conf.h must define PIDX_ESP32_TIMEBASE"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* 1. Timebase                                                               */
/* ======================================================================== */

/**
 * Start the microsecond timebase.
 *
 * @param cpu_freq_hz  CPU frequency in Hz, needed only by the CCOUNT source
 *                     to convert cycles to microseconds (e.g. 240000000).
 *                     Ignored by the esp_timer source; pass 0 there.
 *
 * With PIDX_ESP32_TB_ESP_TIMER nothing has to be started - esp_timer is
 * already running before app_main() - so this only records the source and
 * returns PID_OK. It is still worth calling: it is what makes
 * PIDe_TimebaseReady() meaningful, and it keeps application code identical
 * across the two platforms.
 *
 * @return PID_OK, or PID_ERR_INVALID_PARAM if the CCOUNT source is selected
 *         with an implausible CPU frequency.
 */
PID_StatusCode PIDe_TimebaseInit(uint32_t cpu_freq_hz);

/**
 * Use a microsecond counter you already have (an RTOS tick hook, a GPTimer,
 * a synchronised network clock...).
 *
 * @param fn            Returns a free-running microsecond count. It may wrap;
 *                      the wrap extension in PIDe_NowUs() handles that as long
 *                      as it is called often enough. Must be safe to call from
 *                      any context, including an ISR.
 * @param counter_mask  Width of that counter as an all-ones mask: 0xFFFFFFFF
 *                      for a 32-bit source, 0xFFFF for 16-bit. Must be 2^n - 1
 *                      and at least 0xFFFF. Getting this wrong breaks
 *                      PIDe_DeltaUs() across a wrap, which shows up as one
 *                      absurd dt every wrap period and a derivative spike to
 *                      match.
 */
PID_StatusCode PIDe_TimebaseInitCallback(uint32_t (*fn)(void),
                                         uint32_t counter_mask);

/** @return true once a timebase has been selected. */
bool PIDe_TimebaseReady(void);

/** @return The counter width mask in force, or 0 if no timebase is set. */
uint32_t PIDe_CounterMask(void);

/**
 * @return Low 32 bits of the microsecond timebase.
 *
 * The cheap read: no critical section, no 64-bit arithmetic. Use it with
 * PIDe_DeltaUs() for interval measurement inside a control loop.
 */
uint32_t PIDe_NowUs32(void);

/**
 * Wrap-safe interval between two PIDe_NowUs32() samples.
 *
 * Plain (later - earlier) is already correct for a full-width 32-bit counter,
 * but not for a narrower callback source, where the borrow would be left in
 * the upper half and produce a delta of ~4.29e9 us instead of a few hundred.
 * Masking to the configured width fixes both cases with one expression.
 */
uint32_t PIDe_DeltaUs(uint32_t earlier, uint32_t later);

/**
 * @return Monotonic microseconds since boot, 64-bit.
 *
 * With the esp_timer source this is a direct read and is exact. With CCOUNT or
 * a narrow callback it is extended to 64 bits by counting wraps, which
 * requires that this function be called at least twice per wrap period
 * (~17.9 s at 240 MHz) - trivially true for a control loop, but not for code
 * that idles for a minute and then asks the time.
 */
uint64_t PIDe_NowUs(void);

/** @return Monotonic time in seconds, as a PID_Float. */
PID_Float PIDe_Now(void);

/**
 * Busy-wait for @p us microseconds.
 *
 * Blocking, and therefore wrong in a FreeRTOS task longer than a tick: it
 * starves lower-priority work and, on the ESP32, can trip the task watchdog.
 * Provided for short hardware settling delays (a few microseconds) only. For
 * anything longer use vTaskDelay() so the scheduler can run.
 */
void PIDe_DelayUs(uint32_t us);

/* ======================================================================== */
/* 2. Cycle profiler                                                         */
/* ======================================================================== */

/**
 * @return The Xtensa CCOUNT cycle counter.
 *
 * The finest measurement available: one CPU cycle, 4.17 ns at 240 MHz. Read
 * directly from the core's special register with no function-call overhead
 * once inlined.
 *
 * PER-CORE. Two samples taken on different cores are not comparable, so a
 * profiled section must not yield or migrate between the two reads. Pin the
 * task, or profile only inside a critical section.
 */
uint32_t PIDe_Cycles(void);

/** Convert a CCOUNT difference to microseconds using the configured clock. */
PID_Float PIDe_CyclesToUs(uint32_t cycles);

/* ======================================================================== */
/* 3. Non-blocking rate driver                                               */
/* ======================================================================== */

/**
 * A fixed-rate scheduler for a control loop that must not block.
 *
 * Poll PIDe_RateElapsed() from your task; it returns true exactly once per
 * period and tells you the real elapsed interval, so the controller can be fed
 * a measured dt instead of a nominal one.
 */
typedef struct {
    uint32_t period_us;      /**< Requested period.                          */
    uint32_t next_deadline;  /**< Absolute schedule point, not now+period.   */
    uint32_t last_release;   /**< When the previous iteration was released.  */
    uint32_t last_dt_us;     /**< Measured interval of the last release.     */
    uint32_t iterations;     /**< Total releases.                            */
    uint32_t overruns;       /**< Releases that were a full period late.     */
    uint32_t worst_late_us;  /**< Worst lateness seen.                       */
    bool     primed;         /**< First call seeds the schedule.             */
} PIDe_Rate;

/**
 * @param period_us  Loop period. Must be under half the counter range so the
 *                   wrap-safe deadline comparison stays unambiguous: ~35
 *                   minutes on a 32-bit microsecond counter.
 */
PID_StatusCode PIDe_RateInit(PIDe_Rate *r, uint32_t period_us);

/**
 * @return true when the period has elapsed, at most once per period.
 *
 * The deadline advances on an absolute schedule rather than being re-based on
 * the moment the poll happened to notice. Re-basing would fold the loop body
 * time and the polling granularity into every period, and the loop would
 * slowly run slower than configured while the controller kept using the
 * nominal dt.
 *
 * A release that is a full period late counts as an overrun and re-bases
 * instead of firing the backlog back-to-back: a catch-up burst feeds the
 * controller a run of tiny dt values and spikes the derivative term.
 */
bool PIDe_RateElapsed(PIDe_Rate *r);

/** @return Measured interval of the last release, in seconds. */
PID_Float PIDe_RateDt(const PIDe_Rate *r);

/** Reset the statistics without disturbing the schedule. */
void PIDe_RateResetStats(PIDe_Rate *r);

/* ======================================================================== */
/* 4. Loop load monitor                                                      */
/* ======================================================================== */

/**
 * Measures how much of each period the control work actually consumes.
 *
 * Wrap PIDe_LoadEnter()/PIDe_LoadExit() around the body of your loop. A loop
 * that is quietly using 80% of its period is one plant change away from
 * missing deadlines, and this is the cheapest way to know before it happens.
 */
typedef struct {
    uint32_t t_enter;        /**< CCOUNT at the last enter.                  */
    uint32_t busy_cycles;    /**< Cycles in the last body.                   */
    uint32_t worst_cycles;   /**< Worst body seen.                           */
    uint64_t total_cycles;   /**< Sum, for the average.                      */
    uint32_t samples;
} PIDe_Load;

void PIDe_LoadInit(PIDe_Load *l);
void PIDe_LoadEnter(PIDe_Load *l);
void PIDe_LoadExit(PIDe_Load *l);

/** @return Duty of the last body against @p period_us, in [0,1]. */
PID_Float PIDe_LoadFraction(const PIDe_Load *l, uint32_t period_us);

/** @return Worst-case duty against @p period_us, in [0,1]. */
PID_Float PIDe_LoadWorstFraction(const PIDe_Load *l, uint32_t period_us);

/* ======================================================================== */
/* 5. FreeRTOS helpers                                                       */
/* ======================================================================== */

#if PIDX_ESP32_USE_FREERTOS

/**
 * Create a control task pinned to one core.
 *
 * Pinning is not a nicety on this chip. With the CCOUNT timebase a migrating
 * task reads two unsynchronised counters and sees nonsense dt; even with
 * esp_timer, letting a hard-real-time loop share a core with the Wi-Fi/BT
 * stack is how deadline jitter gets in. The usual arrangement is control on
 * core 1 and connectivity on core 0.
 *
 * @param fn         Task entry point.
 * @param name       Task name for the FreeRTOS registry.
 * @param stack      Stack depth in BYTES on ESP-IDF (not words as in vanilla
 *                   FreeRTOS). 4096 is a reasonable floor for a loop that
 *                   uses floating point and logs.
 * @param arg        Passed through to @p fn.
 * @param priority   Higher runs first. Keep it above the IDLE priority and
 *                   below the Wi-Fi driver's unless you know why not.
 * @param core_id    0 or 1, or PIDX_ESP32_CORE_ANY to leave it unpinned.
 * @param out_handle Optional; receives the task handle.
 *
 * @return PID_OK or PID_ERR_BUSY when the task could not be created (which on
 *         ESP-IDF means the heap could not supply the stack).
 */
PID_StatusCode PIDe_TaskCreate(void (*fn)(void *), const char *name,
                               uint32_t stack, void *arg,
                               uint32_t priority, int core_id,
                               void **out_handle);

#define PIDX_ESP32_CORE_ANY  (-1)

/**
 * Sleep until the next period boundary, for a task-based control loop.
 *
 * Uses vTaskDelayUntil() semantics: the wake time advances by exactly one
 * period regardless of how long the body took, so the loop does not drift.
 * @p last_wake must persist across iterations and be seeded with
 * xTaskGetTickCount() before the first call.
 *
 * RESOLUTION WARNING. FreeRTOS delays are quantised to the tick, which is
 * 10 ms by default on ESP-IDF (configTICK_RATE_HZ = 100). A 1 kHz control loop
 * cannot be built on vTaskDelayUntil() at that tick rate - you would get 100 Hz
 * with a straight face. Either raise CONFIG_FREERTOS_HZ to 1000 (and accept
 * 1 ms granularity, still not enough for 10 kHz), or drive fast loops from a
 * hardware timer ISR / esp_timer callback and use PIDe_Rate to measure the
 * real dt. This function returns PID_ERR_INVALID_PARAM if the requested period
 * is shorter than one tick rather than silently running at the wrong rate.
 */
PID_StatusCode PIDe_TaskDelayPeriod(uint32_t *last_wake, uint32_t period_us);

#endif /* PIDX_ESP32_USE_FREERTOS */

#ifdef __cplusplus
}
#endif

#endif /* PIDX_PID_ESP32_H */

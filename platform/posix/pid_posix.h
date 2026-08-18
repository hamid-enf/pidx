/**
 * @file    pid_posix.h
 * @brief   POSIX timebase and fixed-rate loop driver for PIDX.
 *
 * This is an OPTIONAL integration layer. The PIDX core never calls it and has
 * no knowledge of it; you can delete this directory and the library still
 * builds. Its job is to answer the one question the core deliberately refuses
 * to answer: "what time is it?"
 *
 * Intended for host simulation, hardware-in-the-loop rigs and unit tests on
 * Linux/macOS. For an embedded target use platform/stm32 instead.
 *
 * Requires _POSIX_C_SOURCE >= 199309L for clock_gettime(). Link with -lrt on
 * glibc older than 2.17.
 */
#ifndef PIDX_PID_POSIX_H
#define PIDX_PID_POSIX_H

#include <stdint.h>
#include <stdbool.h>

#include "pidx/pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* Monotonic timebase                                                        */
/* ======================================================================== */

/**
 * Monotonic time in seconds since an arbitrary origin.
 *
 * Uses CLOCK_MONOTONIC, so it does not jump when the wall clock is stepped by
 * NTP or by the user. Never use gettimeofday() for a control loop: a backward
 * step produces a negative dt, and a forward step produces a huge one that
 * makes the derivative term explode.
 */
double PIDp_Now(void);

/** Monotonic time in microseconds. Integer form, no rounding drift. */
uint64_t PIDp_NowUs(void);

/** Sleep for the given number of microseconds (best effort). */
void PIDp_SleepUs(uint64_t us);

/* ======================================================================== */
/* Fixed-rate loop driver                                                    */
/* ======================================================================== */

/**
 * Absolute-deadline scheduler state.
 *
 * The naive "sleep(period)" loop drifts: every iteration accumulates the
 * execution time of the body plus the scheduler's wake-up latency, so a 1 kHz
 * loop ends up running at 970 Hz and the controller's dt no longer matches
 * reality. This driver instead tracks an ABSOLUTE next-deadline and sleeps
 * until it, which removes the accumulation.
 */
typedef struct {
    uint64_t period_us;      /**< Nominal loop period.                       */
    uint64_t next_deadline;  /**< Absolute time of the next release.         */
    uint64_t last_release;   /**< Actual time of the previous release.       */
    uint64_t started_us;     /**< Time of the first release.                 */
    uint32_t iterations;     /**< Completed iterations.                      */
    uint32_t overruns;       /**< Deadlines already in the past on arrival.  */
    uint64_t worst_lateness; /**< Largest observed release lateness [us].    */
    bool     primed;
} PIDp_Loop;

/**
 * Initialise a fixed-rate loop.
 *
 * @param period_us  Loop period in microseconds, > 0. Must match the sample
 *                   time configured in the controller.
 */
PID_StatusCode PIDp_LoopInit(PIDp_Loop *lp, uint64_t period_us);

/**
 * Block until the next release point.
 *
 * @return the actual elapsed time since the previous release, in seconds.
 *         Feed this to PID_UpdateDt() when you want the controller to use the
 *         measured interval rather than the nominal one.
 *
 * If a deadline has already passed (the loop body took too long), it is
 * counted as an overrun and the schedule is re-based on the current time
 * rather than trying to catch up with a burst of back-to-back iterations.
 */
double PIDp_LoopWait(PIDp_Loop *lp);

/** Mean achieved rate in Hz since PIDp_LoopInit. 0 before the first wait. */
double PIDp_LoopMeanRate(const PIDp_Loop *lp);

/* ======================================================================== */
/* Host timing measurement                                                   */
/* ======================================================================== */

/**
 * Simple elapsed-time accumulator for benchmarking a section of code.
 *
 * This measures wall time on a general-purpose OS, so it includes scheduler
 * noise. It is useful for relative comparisons ("does the fast path cost less
 * than the full path?") and useless as an absolute cycle count. For real
 * per-sample cycle figures on Cortex-M, use bench/bench_dwt.c on hardware.
 */
typedef struct {
    uint64_t t0_us;
    uint64_t total_us;
    uint64_t min_us;
    uint64_t max_us;
    uint32_t count;
} PIDp_Timer;

void   PIDp_TimerReset(PIDp_Timer *t);
void   PIDp_TimerStart(PIDp_Timer *t);
void   PIDp_TimerStop(PIDp_Timer *t);
double PIDp_TimerMeanUs(const PIDp_Timer *t);

#ifdef __cplusplus
}
#endif

#endif /* PIDX_PID_POSIX_H */

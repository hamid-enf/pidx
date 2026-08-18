/**
 * @file    pid_conf.h
 * @brief   Compile-time configuration for the PIDX control library.
 *
 * Every optional subsystem is gated by a PIDX_ENABLE_* macro that defaults to
 * a sensible value. When a feature is disabled:
 *   - its code is not compiled into the image (no Flash cost),
 *   - its state is not present in PID_Handle (no RAM cost),
 *   - no runtime branch tests for it (no CPU cost).
 *
 * To override without editing this file, define PIDX_USER_CONF to the name of
 * your own header, e.g.  -DPIDX_USER_CONF="\"my_pid_conf.h\""
 * or pick a profile, e.g. -DPIDX_PROFILE_MINIMAL
 */
#ifndef PIDX_PID_CONF_H
#define PIDX_PID_CONF_H

#ifdef PIDX_USER_CONF
#include PIDX_USER_CONF
#endif

/* ------------------------------------------------------------------------ */
/* Profiles                                                                   */
/* ------------------------------------------------------------------------ */
/*
 * PIDX_PROFILE_MINIMAL  - smallest possible core. Fast inner loops
 *                         (current control, 10-50 kHz). ~132 B handle.
 * PIDX_PROFILE_MOTION   - motion/robotics: shapers, feedforward, gain
 *                         scheduling, cascade, fixed point.
 * PIDX_PROFILE_PROCESS  - slow process loops: auto-tune, shapers, safety,
 *                         diagnostics.
 * PIDX_PROFILE_FULL     - everything (default). Best for evaluation and for
 *                         hosts where a few KB of Flash is irrelevant.
 */
#if !defined(PIDX_PROFILE_MINIMAL) && !defined(PIDX_PROFILE_MOTION) && \
    !defined(PIDX_PROFILE_PROCESS) && !defined(PIDX_PROFILE_FULL)
#define PIDX_PROFILE_FULL 1
#endif

#if defined(PIDX_PROFILE_MINIMAL)
#  define PIDX_DEF_FEEDFORWARD   0
#  define PIDX_DEF_SHAPER        0
#  define PIDX_DEF_INPUT_FILTER  0
#  define PIDX_DEF_SAFETY        0
#  define PIDX_DEF_GAIN_SCHED    0
#  define PIDX_DEF_CASCADE       0
#  define PIDX_DEF_AUTOTUNE      0
#  define PIDX_DEF_DIAGNOSTICS   0
#  define PIDX_DEF_TELEMETRY     0
#  define PIDX_DEF_FIXED_POINT   0
#elif defined(PIDX_PROFILE_MOTION)
#  define PIDX_DEF_FEEDFORWARD   1
#  define PIDX_DEF_SHAPER        1
#  define PIDX_DEF_INPUT_FILTER  1
#  define PIDX_DEF_SAFETY        1
#  define PIDX_DEF_GAIN_SCHED    1
#  define PIDX_DEF_CASCADE       1
#  define PIDX_DEF_AUTOTUNE      0
#  define PIDX_DEF_DIAGNOSTICS   1
#  define PIDX_DEF_TELEMETRY     0
#  define PIDX_DEF_FIXED_POINT   1
#elif defined(PIDX_PROFILE_PROCESS)
#  define PIDX_DEF_FEEDFORWARD   1
#  define PIDX_DEF_SHAPER        1
#  define PIDX_DEF_INPUT_FILTER  1
#  define PIDX_DEF_SAFETY        1
#  define PIDX_DEF_GAIN_SCHED    1
#  define PIDX_DEF_CASCADE       0
#  define PIDX_DEF_AUTOTUNE      1
#  define PIDX_DEF_DIAGNOSTICS   1
#  define PIDX_DEF_TELEMETRY     1
#  define PIDX_DEF_FIXED_POINT   0
#else /* PIDX_PROFILE_FULL */
#  define PIDX_DEF_FEEDFORWARD   1
#  define PIDX_DEF_SHAPER        1
#  define PIDX_DEF_INPUT_FILTER  1
#  define PIDX_DEF_SAFETY        1
#  define PIDX_DEF_GAIN_SCHED    1
#  define PIDX_DEF_CASCADE       1
#  define PIDX_DEF_AUTOTUNE      1
#  define PIDX_DEF_DIAGNOSTICS   1
#  define PIDX_DEF_TELEMETRY     1
#  define PIDX_DEF_FIXED_POINT   1
#endif

/* ------------------------------------------------------------------------ */
/* Optional modules                                                           */
/* ------------------------------------------------------------------------ */

/** Feedforward term (constant value or user callback). */
#ifndef PIDX_ENABLE_FEEDFORWARD
#define PIDX_ENABLE_FEEDFORWARD   PIDX_DEF_FEEDFORWARD
#endif

/** Setpoint trajectory shaping and output slew-rate limiting. */
#ifndef PIDX_ENABLE_SHAPER
#define PIDX_ENABLE_SHAPER        PIDX_DEF_SHAPER
#endif

/** First-order low-pass filter on the measurement before the controller. */
#ifndef PIDX_ENABLE_INPUT_FILTER
#define PIDX_ENABLE_INPUT_FILTER  PIDX_DEF_INPUT_FILTER
#endif

/** Sensor validation, fault latching and fail-safe output. */
#ifndef PIDX_ENABLE_SAFETY
#define PIDX_ENABLE_SAFETY        PIDX_DEF_SAFETY
#endif

/** Interpolated gain scheduling table. */
#ifndef PIDX_ENABLE_GAIN_SCHED
#define PIDX_ENABLE_GAIN_SCHED    PIDX_DEF_GAIN_SCHED
#endif

/** Cascade controller helper (separate translation unit). */
#ifndef PIDX_ENABLE_CASCADE
#define PIDX_ENABLE_CASCADE       PIDX_DEF_CASCADE
#endif

/** Relay / step auto-tuning state machine (separate translation unit). */
#ifndef PIDX_ENABLE_AUTOTUNE
#define PIDX_ENABLE_AUTOTUNE      PIDX_DEF_AUTOTUNE
#endif

/** PID_Status snapshot of every internal term. */
#ifndef PIDX_ENABLE_DIAGNOSTICS
#define PIDX_ENABLE_DIAGNOSTICS   PIDX_DEF_DIAGNOSTICS
#endif

/** Lock-free SPSC telemetry ring buffer. Requires diagnostics. */
#ifndef PIDX_ENABLE_TELEMETRY
#define PIDX_ENABLE_TELEMETRY     PIDX_DEF_TELEMETRY
#endif

#if PIDX_ENABLE_TELEMETRY && !PIDX_ENABLE_DIAGNOSTICS
#error "PIDX_ENABLE_TELEMETRY requires PIDX_ENABLE_DIAGNOSTICS"
#endif

/** Standalone Q15/Q31 fixed-point controller (separate translation unit). */
#ifndef PIDX_ENABLE_FIXED_POINT
#define PIDX_ENABLE_FIXED_POINT   PIDX_DEF_FIXED_POINT
#endif

/* ------------------------------------------------------------------------ */
/* Core behaviour switches                                                    */
/* ------------------------------------------------------------------------ */

/**
 * Argument checking in the hot path (NULL handle, magic, non-finite input).
 * Keep enabled unless you have profiled and proven it matters; it costs a
 * handful of cycles and catches the failure modes that destroy hardware.
 */
#ifndef PIDX_ENABLE_ARG_CHECKS
#define PIDX_ENABLE_ARG_CHECKS    1
#endif

/**
 * Allow <math.h>. Used ONLY outside the hot path:
 *   - sqrtf() when deriving the automatic back-calculation gain Kt,
 *   - sqrtf()/fabsf() inside the auto-tuner.
 * With 0, Kt falls back to 1/Ti (slightly more conservative) and the
 * auto-tuner uses an internal Newton square root.
 */
#ifndef PIDX_USE_LIBM
#define PIDX_USE_LIBM             1
#endif

/** Use double instead of float for PID_Float. Rarely worth it on Cortex-M. */
#ifndef PIDX_USE_DOUBLE
#define PIDX_USE_DOUBLE           0
#endif

/**
 * Memory barrier used by the telemetry SPSC ring buffer.
 * Override with __DMB() on ARM if the producer and consumer run on different
 * exception levels and your compiler reorders aggressively.
 */
#ifndef PIDX_MEMORY_BARRIER
#define PIDX_MEMORY_BARRIER()     do { } while (0)
#endif

/** Inline hint for small internal helpers. */
#ifndef PIDX_INLINE
#  if defined(__GNUC__)
#    define PIDX_INLINE static inline __attribute__((always_inline))
#  else
#    define PIDX_INLINE static inline
#  endif
#endif

/** Marks intentionally unused parameters without triggering warnings. */
#define PIDX_UNUSED(x)            ((void)(x))

/* ------------------------------------------------------------------------ */
/* Numeric defaults used by PID_ConfigDefault()                               */
/* ------------------------------------------------------------------------ */

/** Default sample time [s] (100 Hz) - safe for a first bring-up. */
#ifndef PIDX_DEFAULT_SAMPLE_TIME
#define PIDX_DEFAULT_SAMPLE_TIME  0.01f
#endif

/** Default derivative filter ratio N in Tf = Kd / (N * Kp). 5..20 is usual. */
#ifndef PIDX_DEFAULT_N_FILTER
#define PIDX_DEFAULT_N_FILTER     10.0f
#endif

/** Sentinel meaning "no limit". Finite on purpose: keeps arithmetic sane. */
#ifndef PIDX_HUGE
#define PIDX_HUGE                 1.0e30f
#endif

#endif /* PIDX_PID_CONF_H */

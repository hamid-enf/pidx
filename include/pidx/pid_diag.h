/**
 * @file    pid_diag.h
 * @brief   Diagnostics, lock-free telemetry and loop-quality metrics.
 *
 * Three separable things live here:
 *
 *  1. PID_Status - a per-cycle snapshot the core fills when diagnostics are
 *     compiled in. Read it with PID_GetStatus().
 *  2. PID_Telemetry - a single-producer/single-consumer ring buffer so an ISR
 *     can log every cycle and a background task can drain it without a mutex
 *     and without ever blocking the control loop.
 *  3. PID_LoopMetrics - running loop-quality statistics (IAE, ISE, saturation
 *     duty, oscillation rate). Cheap enough to leave on in production, and the
 *     honest way to answer "is this loop still tuned properly?".
 *
 * @section spsc Concurrency contract
 *
 * The ring buffer is safe for exactly one producer and one consumer, with
 * these guarantees and no others:
 *   - the producer only ever writes @c head (and the slot it owns);
 *   - the consumer only ever writes @c tail;
 *   - both indices are 16-bit and updated with a single store, which is atomic
 *     on every architecture this library targets (8/16/32-bit MCUs and hosts);
 *   - capacity is a power of two so index wrapping is a mask, never a modulo.
 *
 * If your producer and consumer run at different exception levels on a core
 * that reorders stores, define PIDX_MEMORY_BARRIER() (e.g. to __DMB()).
 *
 * When the ring is full the producer drops the NEWEST record - it increments
 * `dropped`, bumps `seq` and returns - and never blocks the control loop.
 *
 * Dropping the newest rather than the oldest is forced by the lock-free
 * contract, not chosen for its logging semantics. Overwriting the oldest would
 * require the producer to advance `tail`, giving that index two writers; a
 * producer pre-empted between reading and writing `tail` would resume with a
 * stale value and drive it backwards, re-delivering a record the consumer had
 * already taken. Keeping `tail` consumer-owned is what makes the buffer safe
 * without a critical section.
 *
 * Consequence for the consumer: after a burst of drops you hold a contiguous
 * run of OLD records, and the loss shows up as a jump in `seq`. That is the
 * intended way to detect it - see PID_Telemetry_Dropped(), which is a
 * read-and-clear counter.
 */
#ifndef PIDX_PID_DIAG_H
#define PIDX_PID_DIAG_H

#include "pid_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#if PIDX_ENABLE_TELEMETRY

/**
 * Bind storage to a telemetry buffer.
 * @param storage   User-owned array. Never freed by the library.
 * @param capacity  Number of records. MUST be a power of two and >= 2.
 * @return PID_ERR_INVALID_PARAM if capacity is not a power of two.
 */
PID_StatusCode PID_Telemetry_Init(PID_Telemetry *t,
                                  PID_TelemetryRecord *storage,
                                  uint16_t capacity);

/** Attach a buffer to a controller and enable PID_FEAT_TELEMETRY. */
PID_StatusCode PID_Telemetry_Attach(PID_Handle *h, PID_Telemetry *t);

/**
 * Consumer side: pop one record.
 * @return PID_OK, or PID_ERR_BUSY when the buffer is empty.
 */
PID_StatusCode PID_Telemetry_Read(PID_Telemetry *t, PID_TelemetryRecord *out);

/** @return Number of records currently readable. */
uint16_t PID_Telemetry_Count(const PID_Telemetry *t);

/** @return Records dropped because the consumer fell behind. Clears on read. */
uint16_t PID_Telemetry_Dropped(PID_Telemetry *t);

/** Discard everything buffered (consumer context only). */
PID_StatusCode PID_Telemetry_Flush(PID_Telemetry *t);

/**
 * Producer side. Called by the core; exposed so a user-written update loop or
 * a cascade wrapper can log too. Never blocks, never allocates.
 */
void pidd_telemetry_push(PID_Telemetry *t, const PID_Status *s);

#endif /* PIDX_ENABLE_TELEMETRY */

#if PIDX_ENABLE_DIAGNOSTICS

/**
 * Running loop-quality metrics.
 *
 * All integrals are approximated with the rectangle rule over the actual dt of
 * each sample, so a jittery loop still produces meaningful numbers.
 */
typedef struct {
    PID_Float iae;              /**< Integral of |e| dt.                      */
    PID_Float ise;              /**< Integral of e^2 dt - punishes big errors.*/
    PID_Float itae;             /**< Integral of t*|e| dt - punishes slow tails*/
    PID_Float total_time;       /**< Seconds accumulated.                     */
    PID_Float sat_time;         /**< Seconds spent at an output limit.        */
    PID_Float abs_error_max;    /**< Worst |e| seen.                          */
    PID_Float output_travel;    /**< Integral of |du| - actuator wear proxy.  */
    PID_Float last_output;
    PID_Float last_error;
    uint32_t  samples;
    uint32_t  sign_changes;     /**< Error zero-crossings: oscillation proxy. */
    bool      primed;
} PID_LoopMetrics;

/** Zero all accumulators. */
PID_StatusCode PID_Metrics_Reset(PID_LoopMetrics *m);

/**
 * Fold one control cycle into the metrics. Call right after PID_Update().
 * Costs ~10 flops; safe to leave enabled in production.
 */
PID_StatusCode PID_Metrics_Update(PID_LoopMetrics *m, const PID_Handle *h);

/** @return Mean |error| over the accumulated window, or 0 before any sample. */
PID_Float PID_Metrics_MeanAbsError(const PID_LoopMetrics *m);

/** @return Fraction of time the output was saturated, in [0,1]. */
PID_Float PID_Metrics_SaturationDuty(const PID_LoopMetrics *m);

/**
 * @return Error zero-crossings per second. A well-damped loop settles to
 *         roughly zero; a persistently high value means the loop is hunting,
 *         usually too much gain or too little derivative filtering.
 */
PID_Float PID_Metrics_OscillationRate(const PID_LoopMetrics *m);

#endif /* PIDX_ENABLE_DIAGNOSTICS */

#ifdef __cplusplus
}
#endif

#endif /* PIDX_PID_DIAG_H */

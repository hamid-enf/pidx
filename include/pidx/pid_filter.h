/**
 * @file    pid_filter.h
 * @brief   Standalone signal-conditioning filters.
 *
 * These are ordinary reusable filters, not part of the control law. The core
 * uses PID_LPF1 for its optional input filter; everything else here is for you
 * to apply to sensor signals before they reach the controller.
 *
 * @section choosing Choosing a filter
 *
 * | Filter      | Kills            | Phase lag        | Cost/sample |
 * |-------------|------------------|------------------|-------------|
 * | LPF1        | broadband noise  | atan(w*tau)      | 2 flop      |
 * | MovingAvg   | broadband noise  | (N-1)/2 samples  | 2 flop + RAM|
 * | Median3     | isolated spikes  | 1 sample         | compares    |
 * | RateLimiter | impossible jumps | none in-band     | 2 compares  |
 * | Deadband    | quantisation dither | none          | 2 compares  |
 *
 * @warning Every filter in the measurement path adds phase lag, and phase lag
 * is what destabilises a loop. Filter the least you can get away with. If you
 * only need to tame the derivative, use the derivative filter (Tf / N) instead
 * - it attenuates noise where it hurts without lagging the P and I terms.
 *
 * Median3 is the exception worth reaching for first: it removes single-sample
 * spikes (ADC glitches, EMI hits, a dropped CAN frame) almost for free, and a
 * spike is exactly what a low-pass handles worst - it smears it over many
 * samples instead of removing it.
 */
#ifndef PIDX_PID_FILTER_H
#define PIDX_PID_FILTER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "pid_math.h"
#include "pid_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* First-order low-pass                                                      */
/* ======================================================================== */

/**
 * Exponential (first-order IIR) low-pass.
 *
 *   y[k] = a*y[k-1] + (1-a)*x[k],   a = tau/(tau+dt)
 *
 * The coefficient is the exact backward-Euler discretisation of
 * 1/(1+s*tau), so `a` stays in [0,1) for every tau >= 0 and dt > 0 - the
 * filter can never be made unstable by a bad parameter. Cutoff frequency is
 * fc = 1/(2*pi*tau).
 */
typedef struct {
    PID_Float a;        /**< Pole, tau/(tau+dt). Precomputed.                */
    PID_Float state;    /**< y[k-1].                                         */
    PID_Float tau;      /**< Time constant [s]. 0 = pass-through.            */
    bool      primed;   /**< First sample seeds the state directly, so the
                             filter does not have to ramp up from zero.      */
} PID_LPF1;

/** Recompute the pole for a new tau/dt pair. Both must be finite, dt > 0. */
PIDX_INLINE void pidf_lpf1_coeff(PID_LPF1 *f, PID_Float tau, PID_Float dt)
{
    f->tau = tau;
    f->a = (tau > PID_ZERO) ? (tau / (tau + dt)) : PID_ZERO;
}

/** Advance one sample. Hot-path safe: no division, no branchy math. */
PIDX_INLINE PID_Float pidf_lpf1_step(PID_LPF1 *f, PID_Float x)
{
    if (!f->primed) {
        f->state = x;
        f->primed = true;
    } else {
        f->state = (f->a * f->state) + ((PID_ONE - f->a) * x);
    }
    return f->state;
}

/** Initialise (tau in seconds, dt the sample time). */
PID_StatusCode PID_LPF1_Init(PID_LPF1 *f, PID_Float tau, PID_Float dt);

/** Change the time constant, keeping the current state. */
PID_StatusCode PID_LPF1_SetTau(PID_LPF1 *f, PID_Float tau, PID_Float dt);

/** Configure by cutoff frequency instead: tau = 1/(2*pi*fc). */
PID_StatusCode PID_LPF1_SetCutoff(PID_LPF1 *f, PID_Float fc_hz, PID_Float dt);

/** Filter one sample (validated wrapper around pidf_lpf1_step). */
PID_Float PID_LPF1_Update(PID_LPF1 *f, PID_Float x);

/** Forget history; the next sample re-seeds the state. */
PID_StatusCode PID_LPF1_Reset(PID_LPF1 *f);

/* ======================================================================== */
/* Moving average                                                            */
/* ======================================================================== */

/**
 * Boxcar moving average over a user-owned buffer.
 *
 * Better than an LPF at killing periodic noise whose period divides the window
 * (mains hum: window = one mains period), and it has exactly linear phase.
 * Worse at everything else, and it costs N floats of RAM.
 *
 * @note The running sum is fully recomputed once per completed window. A naive
 * running sum accumulates rounding error without bound over hours of
 * operation; this bounds the error to one window's worth at the cost of one
 * O(N) pass every N samples.
 */
typedef struct {
    PID_Float *buffer;      /**< User-owned, length @c size.                 */
    PID_Float  sum;
    uint16_t   size;
    uint16_t   index;
    uint16_t   count;       /**< Samples seen, saturating at @c size.        */
} PID_MovingAvg;

/** @param buffer User storage of @p size elements. Not copied, never freed. */
PID_StatusCode PID_MovingAvg_Init(PID_MovingAvg *f, PID_Float *buffer, uint16_t size);

/** @return Current average. Before the window fills, averages what it has. */
PID_Float PID_MovingAvg_Update(PID_MovingAvg *f, PID_Float x);

PID_StatusCode PID_MovingAvg_Reset(PID_MovingAvg *f);

/* ======================================================================== */
/* Median-of-3 despiker                                                      */
/* ======================================================================== */

/**
 * Three-sample median. Removes any isolated single-sample outlier completely
 * while passing ramps and steps with only one sample of delay.
 *
 * This is the right first line of defence against ADC glitches; a low-pass
 * would instead spread the glitch across several samples.
 */
typedef struct {
    PID_Float x1;
    PID_Float x2;
    uint8_t   count;
} PID_Median3;

PID_StatusCode PID_Median3_Init(PID_Median3 *f);
PID_Float      PID_Median3_Update(PID_Median3 *f, PID_Float x);

/* ======================================================================== */
/* Rate limiter and deadband                                                 */
/* ======================================================================== */

/** Slew-rate limiter: |dy/dt| <= rate_max. */
typedef struct {
    PID_Float value;
    PID_Float rate_max;     /**< [unit/s]. 0 = pass-through.                 */
    bool      primed;
} PID_RateLimiter;

PID_StatusCode PID_RateLimiter_Init(PID_RateLimiter *f, PID_Float rate_max);
PID_Float      PID_RateLimiter_Update(PID_RateLimiter *f, PID_Float x, PID_Float dt);
PID_StatusCode PID_RateLimiter_Reset(PID_RateLimiter *f, PID_Float value);

/**
 * Symmetric deadband around zero.
 *
 * @param subtract  true  -> continuous output: sign(x)*(|x|-width), no jump at
 *                           the band edge. Use this in a control path.
 *                  false -> hard zeroing: output steps by @p width at the edge.
 *                           Only for display or thresholding.
 *
 * Use it to stop a controller hunting against backlash or a quantised sensor.
 * It does not remove steady-state error - it declares a band where you accept
 * the error instead.
 */
PIDX_INLINE PID_Float pidf_deadband(PID_Float x, PID_Float width, bool subtract)
{
    PID_Float r = x;
    if (width > PID_ZERO) {
        if (pidm_abs(x) <= width) {
            r = PID_ZERO;
        } else if (subtract) {
            r = (x > PID_ZERO) ? (x - width) : (x + width);
        } else {
            /* keep x unchanged outside the band */
        }
    }
    return r;
}

#ifdef __cplusplus
}
#endif

#endif /* PIDX_PID_FILTER_H */

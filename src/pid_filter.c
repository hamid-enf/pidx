/**
 * @file    pid_filter.c
 * @brief   Standalone signal-conditioning filters.
 */

#include "pidx/pid_filter.h"

/* ======================================================================== */
/* First-order low-pass                                                      */
/* ======================================================================== */

PID_StatusCode PID_LPF1_Init(PID_LPF1 *f, PID_Float tau, PID_Float dt)
{
    if (f == NULL) {
        return PID_ERR_NULL;
    }
    if (!pidm_isfinite(tau) || (tau < PID_ZERO)) {
        return PID_ERR_INVALID_PARAM;
    }
    if (!pidm_isfinite(dt) || (dt <= PID_ZERO)) {
        return PID_ERR_INVALID_DT;
    }
    f->state = PID_ZERO;
    f->primed = false;
    pidf_lpf1_coeff(f, tau, dt);
    return PID_OK;
}

PID_StatusCode PID_LPF1_SetTau(PID_LPF1 *f, PID_Float tau, PID_Float dt)
{
    if (f == NULL) {
        return PID_ERR_NULL;
    }
    if (!pidm_isfinite(tau) || (tau < PID_ZERO)) {
        return PID_ERR_INVALID_PARAM;
    }
    if (!pidm_isfinite(dt) || (dt <= PID_ZERO)) {
        return PID_ERR_INVALID_DT;
    }
    pidf_lpf1_coeff(f, tau, dt);
    return PID_OK;
}

PID_StatusCode PID_LPF1_SetCutoff(PID_LPF1 *f, PID_Float fc_hz, PID_Float dt)
{
    /* tau = 1/(2*pi*fc). Below Nyquist the discrete pole tracks the analogue
     * cutoff closely; above it the filter still behaves, it just no longer
     * means what the number says. */
    const PID_Float two_pi = (PID_Float)6.283185307179586f;

    if (f == NULL) {
        return PID_ERR_NULL;
    }
    if (!pidm_isfinite(fc_hz) || (fc_hz <= PID_ZERO)) {
        return PID_ERR_INVALID_PARAM;
    }
    return PID_LPF1_SetTau(f, PID_ONE / (two_pi * fc_hz), dt);
}

PID_Float PID_LPF1_Update(PID_LPF1 *f, PID_Float x)
{
    if (f == NULL) {
        return PID_ZERO;
    }
    if (!pidm_isfinite(x)) {
        return f->state;    /* hold: a bad sample must not poison the state */
    }
    return pidf_lpf1_step(f, x);
}

PID_StatusCode PID_LPF1_Reset(PID_LPF1 *f)
{
    if (f == NULL) {
        return PID_ERR_NULL;
    }
    f->state = PID_ZERO;
    f->primed = false;
    return PID_OK;
}

/* ======================================================================== */
/* Moving average                                                            */
/* ======================================================================== */

PID_StatusCode PID_MovingAvg_Init(PID_MovingAvg *f, PID_Float *buffer, uint16_t size)
{
    uint16_t i;

    if ((f == NULL) || (buffer == NULL)) {
        return PID_ERR_NULL;
    }
    if (size == 0U) {
        return PID_ERR_INVALID_PARAM;
    }
    f->buffer = buffer;
    f->size = size;
    f->index = 0U;
    f->count = 0U;
    f->sum = PID_ZERO;
    for (i = 0U; i < size; ++i) {
        buffer[i] = PID_ZERO;
    }
    return PID_OK;
}

PID_Float PID_MovingAvg_Update(PID_MovingAvg *f, PID_Float x)
{
    PID_Float n;

    if ((f == NULL) || (f->buffer == NULL)) {
        return PID_ZERO;
    }
    if (!pidm_isfinite(x)) {
        x = PID_ZERO;
    }

    f->sum -= f->buffer[f->index];
    f->buffer[f->index] = x;
    f->sum += x;

    f->index++;
    if (f->index >= f->size) {
        PID_Float acc = PID_ZERO;
        uint16_t i;
        f->index = 0U;
        /* Re-derive the sum once per window. Incremental add/subtract drifts
         * without bound in float; this caps the error at one window. */
        for (i = 0U; i < f->size; ++i) {
            acc += f->buffer[i];
        }
        f->sum = acc;
    }

    if (f->count < f->size) {
        f->count++;
    }

    n = (PID_Float)f->count;
    return f->sum / n;
}

PID_StatusCode PID_MovingAvg_Reset(PID_MovingAvg *f)
{
    if ((f == NULL) || (f->buffer == NULL)) {
        return PID_ERR_NULL;
    }
    return PID_MovingAvg_Init(f, f->buffer, f->size);
}

/* ======================================================================== */
/* Median of 3                                                               */
/* ======================================================================== */

PID_StatusCode PID_Median3_Init(PID_Median3 *f)
{
    if (f == NULL) {
        return PID_ERR_NULL;
    }
    f->x1 = PID_ZERO;
    f->x2 = PID_ZERO;
    f->count = 0U;
    return PID_OK;
}

PID_Float PID_Median3_Update(PID_Median3 *f, PID_Float x)
{
    PID_Float a;
    PID_Float b;
    PID_Float c;
    PID_Float med;

    if (f == NULL) {
        return PID_ZERO;
    }
    if (!pidm_isfinite(x)) {
        return f->x1;
    }

    a = f->x2;
    b = f->x1;
    c = x;

    f->x2 = f->x1;
    f->x1 = x;

    if (f->count < 2U) {
        f->count++;
        return x;           /* not enough history yet */
    }

    /* Median of three by pairwise comparison: 3 compares, no sorting, no
     * branch on the data beyond the compares themselves. */
    if (a > b) { const PID_Float t = a; a = b; b = t; }
    if (b > c) { b = c; }
    med = (a > b) ? a : b;
    return med;
}

/* ======================================================================== */
/* Rate limiter                                                              */
/* ======================================================================== */

PID_StatusCode PID_RateLimiter_Init(PID_RateLimiter *f, PID_Float rate_max)
{
    if (f == NULL) {
        return PID_ERR_NULL;
    }
    if (!pidm_isfinite(rate_max) || (rate_max < PID_ZERO)) {
        return PID_ERR_INVALID_PARAM;
    }
    f->rate_max = rate_max;
    f->value = PID_ZERO;
    f->primed = false;
    return PID_OK;
}

PID_Float PID_RateLimiter_Update(PID_RateLimiter *f, PID_Float x, PID_Float dt)
{
    PID_Float max_step;
    PID_Float delta;

    if (f == NULL) {
        return PID_ZERO;
    }
    if (!pidm_isfinite(x) || !pidm_isfinite(dt) || (dt <= PID_ZERO)) {
        return f->value;
    }
    if (!f->primed) {
        f->value = x;
        f->primed = true;
        return x;
    }
    if (f->rate_max <= PID_ZERO) {
        f->value = x;
        return x;
    }

    max_step = f->rate_max * dt;
    delta = x - f->value;
    if (delta > max_step) {
        f->value += max_step;
    } else if (delta < -max_step) {
        f->value -= max_step;
    } else {
        f->value = x;
    }
    return f->value;
}

PID_StatusCode PID_RateLimiter_Reset(PID_RateLimiter *f, PID_Float value)
{
    if (f == NULL) {
        return PID_ERR_NULL;
    }
    if (!pidm_isfinite(value)) {
        return PID_ERR_INVALID_PARAM;
    }
    f->value = value;
    f->primed = true;
    return PID_OK;
}

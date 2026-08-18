/**
 * @file    pid_gainsched.c
 * @brief   Interpolated gain scheduling.
 *
 * The table is a piecewise-linear (or smoothstep) function of one scheduling
 * variable. Lookup is a linear scan from a cached index: schedules are small
 * (typically 3-8 points) and the variable moves slowly relative to the sample
 * rate, so the scan almost always terminates on the first or second test. A
 * binary search would be more code and, at this size, slower.
 */

#include "pidx/pid_gainsched.h"

#if PIDX_ENABLE_GAIN_SCHED

#include "pidx/pid.h"

PID_StatusCode PID_GainSched_Init(PID_GainSchedule *s,
                                  const PID_GainPoint *points,
                                  uint8_t count,
                                  PID_SchedSource source,
                                  PID_SchedInterp interp)
{
    uint8_t i;

    if ((s == NULL) || (points == NULL)) {
        return PID_ERR_NULL;
    }
    if ((count < 2U) || (count > (uint8_t)PIDX_GAINSCHED_MAX_POINTS)) {
        return PID_ERR_INVALID_PARAM;
    }
    if ((source > PID_SCHED_SRC_EXTERNAL) || (interp > PID_SCHED_INTERP_HOLD)) {
        return PID_ERR_INVALID_PARAM;
    }

    for (i = 0U; i < count; ++i) {
        if (!pidm_isfinite(points[i].x)) {
            return PID_ERR_INVALID_PARAM;
        }
        /* Strictly ascending: equal breakpoints would divide by zero, and a
         * descending table is always a user mistake rather than an intent. */
        if ((i > 0U) && (points[i].x <= points[i - 1U].x)) {
            return PID_ERR_INVALID_PARAM;
        }
        if (!pidm_isfinite(points[i].kp) || (points[i].kp < PID_ZERO) ||
            !pidm_isfinite(points[i].ki) || (points[i].ki < PID_ZERO) ||
            !pidm_isfinite(points[i].kd) || (points[i].kd < PID_ZERO)) {
            return PID_ERR_INVALID_GAIN;
        }
    }

    s->points = points;
    s->count = count;
    s->source = (uint8_t)source;
    s->interp = (uint8_t)interp;
    s->reserved = 0U;
    s->hysteresis = PID_ZERO;
    s->last_x = points[0].x;
    s->primed = false;
    return PID_OK;
}

PID_StatusCode PID_GainSched_SetHysteresis(PID_GainSchedule *s, PID_Float band)
{
    if (s == NULL) {
        return PID_ERR_NULL;
    }
    if (!pidm_isfinite(band) || (band < PID_ZERO)) {
        return PID_ERR_INVALID_PARAM;
    }
    s->hysteresis = band;
    return PID_OK;
}

PID_StatusCode PID_GainSched_Attach(PID_Handle *h, PID_GainSchedule *s)
{
    if (h == NULL) {
        return PID_ERR_NULL;
    }
    if ((s != NULL) && ((s->points == NULL) || (s->count < 2U))) {
        return PID_ERR_INVALID_PARAM;   /* never passed through Init() */
    }

    h->sched = s;
    if (s != NULL) {
        h->features |= PID_FEAT_GAIN_SCHED;
    } else {
        h->features &= ~(uint32_t)PID_FEAT_GAIN_SCHED;
    }
    h->sched_index_cache = 0U;
    return PID_OK;
}

PID_StatusCode PID_GainSched_SetVar(PID_Handle *h, PID_Float value)
{
    if (h == NULL) {
        return PID_ERR_NULL;
    }
    if (!pidm_isfinite(value)) {
        return PID_ERR_INVALID_PARAM;
    }
    h->sched_var_ext = value;
    return PID_OK;
}

/**
 * Smoothstep 3t^2 - 2t^3.
 * Its derivative is zero at both ends, so the interpolated gain curve is C1
 * continuous across breakpoints. Linear interpolation has a slope jump there,
 * which a noisy scheduling variable turns into audible gain chatter.
 */
static PID_Float pidp_smoothstep(PID_Float t)
{
    return (t * t) * ((PID_Float)3.0f - (PID_TWO * t));
}

PID_StatusCode PID_GainSched_Evaluate(PID_GainSchedule *s, PID_Float x,
                                      PID_Float *kp, PID_Float *ki, PID_Float *kd)
{
    const PID_GainPoint *p;
    uint8_t i;
    PID_Float t;

    if ((s == NULL) || (s->points == NULL)) {
        return PID_ERR_NULL;
    }
    if (!pidm_isfinite(x)) {
        return PID_ERR_INVALID_PARAM;
    }

    /* Hysteresis: ignore movement smaller than the band so that sensor noise
     * around a breakpoint does not dither the gains. */
    if (s->primed && (s->hysteresis > PID_ZERO)) {
        if (pidm_abs(x - s->last_x) < s->hysteresis) {
            x = s->last_x;
        }
    }
    s->last_x = x;
    s->primed = true;

    p = s->points;

    /* Outside the table the gains saturate at the end points. Extrapolating
     * would be worse than useless: it can produce negative gains. */
    if (x <= p[0].x) {
        if (kp != NULL) { *kp = p[0].kp; }
        if (ki != NULL) { *ki = p[0].ki; }
        if (kd != NULL) { *kd = p[0].kd; }
        return PID_OK;
    }
    if (x >= p[s->count - 1U].x) {
        const uint8_t last = (uint8_t)(s->count - 1U);
        if (kp != NULL) { *kp = p[last].kp; }
        if (ki != NULL) { *ki = p[last].ki; }
        if (kd != NULL) { *kd = p[last].kd; }
        return PID_OK;
    }

    for (i = 0U; i < (uint8_t)(s->count - 1U); ++i) {
        if ((x >= p[i].x) && (x < p[i + 1U].x)) {
            break;
        }
    }

    t = (x - p[i].x) / (p[i + 1U].x - p[i].x);   /* strictly ascending: safe */

    switch ((PID_SchedInterp)s->interp) {
    case PID_SCHED_INTERP_HOLD:
        t = PID_ZERO;
        break;
    case PID_SCHED_INTERP_SMOOTH:
        t = pidp_smoothstep(t);
        break;
    case PID_SCHED_INTERP_LINEAR:
    default:
        break;
    }

    if (kp != NULL) { *kp = pidm_lerp(p[i].kp, p[i + 1U].kp, t); }
    if (ki != NULL) { *ki = pidm_lerp(p[i].ki, p[i + 1U].ki, t); }
    if (kd != NULL) { *kd = pidm_lerp(p[i].kd, p[i + 1U].kd, t); }
    return PID_OK;
}

#endif /* PIDX_ENABLE_GAIN_SCHED */

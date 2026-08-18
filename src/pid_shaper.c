/**
 * @file    pid_shaper.c
 * @brief   Standalone trajectory shaper.
 */

#include "pidx/pid_shaper.h"

PID_StatusCode PID_Shaper_Init(PID_Shaper *s, PID_Float rate_max,
                               PID_Float accel, PID_Float decel)
{
    if (s == NULL) {
        return PID_ERR_NULL;
    }
    if (!pidm_isfinite(rate_max) || (rate_max < PID_ZERO) ||
        !pidm_isfinite(accel) || (accel < PID_ZERO) ||
        !pidm_isfinite(decel) || (decel < PID_ZERO)) {
        return PID_ERR_INVALID_PARAM;
    }
    s->position = PID_ZERO;
    s->velocity = PID_ZERO;
    s->target = PID_ZERO;
    s->rate_max = rate_max;
    s->accel = accel;
    s->decel = decel;
    s->moving = false;
    return PID_OK;
}

PID_StatusCode PID_Shaper_SetTarget(PID_Shaper *s, PID_Float target)
{
    if (s == NULL) {
        return PID_ERR_NULL;
    }
    if (!pidm_isfinite(target)) {
        return PID_ERR_INVALID_PARAM;
    }
    s->target = target;
    s->moving = (target != s->position);
    return PID_OK;
}

PID_StatusCode PID_Shaper_Reset(PID_Shaper *s, PID_Float position)
{
    if (s == NULL) {
        return PID_ERR_NULL;
    }
    if (!pidm_isfinite(position)) {
        return PID_ERR_INVALID_PARAM;
    }
    s->position = position;
    s->target = position;
    s->velocity = PID_ZERO;
    s->moving = false;
    return PID_OK;
}

PID_Float PID_Shaper_Update(PID_Shaper *s, PID_Float dt)
{
    if (s == NULL) {
        return PID_ZERO;
    }
    if (!pidm_isfinite(dt) || (dt <= PID_ZERO)) {
        return s->position;
    }
    if (s->rate_max <= PID_ZERO) {
        s->position = s->target;    /* shaping disabled: pass through */
        s->velocity = PID_ZERO;
        s->moving = false;
        return s->position;
    }

    s->moving = pids_profile_step(&s->position, &s->velocity, s->target,
                                  s->rate_max, s->accel, s->decel, dt);
    return s->position;
}

bool PID_Shaper_IsMoving(const PID_Shaper *s)
{
    return (s != NULL) && s->moving;
}

PID_Float PID_Shaper_EstimateTime(const PID_Shaper *s)
{
    PID_Float d;
    PID_Float a;
    PID_Float b;
    PID_Float v;
    PID_Float d_ramp;

    if (s == NULL) {
        return PID_ZERO;
    }

    d = pidm_abs(s->target - s->position);
    if ((d == PID_ZERO) || (s->rate_max <= PID_ZERO)) {
        return PID_ZERO;
    }

    v = s->rate_max;
    a = s->accel;
    if (a <= PID_ZERO) {
        return d / v;               /* rate-only profile */
    }
    b = (s->decel > PID_ZERO) ? s->decel : a;

    /* Distance consumed by accelerating to v and back down to rest. */
    d_ramp = ((v * v) / (PID_TWO * a)) + ((v * v) / (PID_TWO * b));

    if (d >= d_ramp) {
        /* Trapezoid: ramp up + cruise + ramp down. */
        return ((d - d_ramp) / v) + (v / a) + (v / b);
    }

    /* Triangle: never reaches rate_max. Solve
     *   d = vp^2/(2a) + vp^2/(2b)  ->  vp = sqrt(2*d*a*b/(a+b)). */
    {
        const PID_Float vp = pidm_sqrt((PID_TWO * d * a * b) / (a + b));
        return (vp / a) + (vp / b);
    }
}

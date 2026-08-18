/**
 * @file    pid_shaper.h
 * @brief   Setpoint trajectory shaping and output slew limiting.
 *
 * @section why Why shape the setpoint
 *
 * A step change in the setpoint asks the plant for infinite acceleration. The
 * controller responds with whatever the actuator can deliver, saturates, winds
 * up, and overshoots. Shaping the setpoint into something the plant can
 * actually follow fixes the problem at its source instead of detuning the
 * controller to survive a command that was never physical.
 *
 * Two profiles are provided:
 *
 *  - Rate-limited (accel = 0): the setpoint moves at a constant speed. One
 *    parameter, no overshoot, but velocity steps at each end. Fine for a
 *    temperature ramp, harsh for a servo.
 *  - Trapezoidal (accel > 0): accelerate, cruise at rate_max, then decelerate
 *    so as to arrive with zero velocity. This is the standard point-to-point
 *    motion profile.
 *
 * @section braking Braking distance
 *
 * Deceleration must begin when the remaining distance equals
 *
 *     d_brake = v^2 / (2 * a_decel)
 *
 * from integrating v*dv = a*dx. Starting later guarantees overshoot; starting
 * earlier just wastes time. Because the check happens every sample against the
 * current velocity, the profile is self-correcting: a mid-flight target change
 * is handled without replanning.
 *
 * @section scope What this is not
 *
 * There is no jerk limit (the S-curve third derivative) and no multi-axis
 * coordination. Those belong in a motion planner, not a PID library. If you
 * need them, generate the trajectory upstream and feed it in with
 * PID_SetSetpointImmediate().
 */
#ifndef PIDX_PID_SHAPER_H
#define PIDX_PID_SHAPER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "pid_math.h"
#include "pid_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Shared trapezoidal/rate profile integrator.
 *
 * Advances @p pos one step of @p dt towards @p target, updating @p vel.
 * Used by the core's built-in setpoint shaper and by the standalone
 * PID_Shaper below, so there is exactly one implementation of the profile.
 *
 * @param pos      In/out: current position.
 * @param vel      In/out: current velocity.
 * @param target   Destination.
 * @param rate_max Speed limit, > 0.
 * @param accel    Acceleration limit; <= 0 selects the rate-only profile.
 * @param decel    Deceleration limit; <= 0 mirrors @p accel.
 * @param dt       Timestep, > 0.
 * @return true while still moving, false once parked on the target.
 */
PIDX_INLINE bool pids_profile_step(PID_Float *pos, PID_Float *vel,
                                   PID_Float target, PID_Float rate_max,
                                   PID_Float accel, PID_Float decel,
                                   PID_Float dt)
{
    const PID_Float dist = target - *pos;
    PID_Float v = *vel;
    PID_Float step;
    bool moving = true;

    if (rate_max <= PID_ZERO) {
        *pos = target;
        *vel = PID_ZERO;
        return false;
    }
    if (dist == PID_ZERO) {
        *vel = PID_ZERO;
        return false;
    }

    if (accel <= PID_ZERO) {
        v = (dist > PID_ZERO) ? rate_max : -rate_max;
    } else {
        const PID_Float d = (decel > PID_ZERO) ? decel : accel;
        const PID_Float brake = (v * v) / (PID_TWO * d);

        if (dist > PID_ZERO) {
            if (v < PID_ZERO)          { v += d * dt; }      /* wrong way   */
            else if (brake >= dist)    { v -= d * dt; }      /* start brake */
            else                       { v += accel * dt; }  /* speed up    */
        } else {
            if (v > PID_ZERO)          { v -= d * dt; }
            else if (brake >= -dist)   { v += d * dt; }
            else                       { v -= accel * dt; }
        }
        v = pidm_clamp(v, -rate_max, rate_max);
    }

    step = v * dt;
    if (pidm_abs(step) >= pidm_abs(dist)) {
        *pos = target;          /* landing sample: snap, do not overshoot */
        *vel = PID_ZERO;
        moving = false;
    } else {
        *pos += step;
        *vel = v;
    }
    return moving;
}

/* ======================================================================== */
/* Standalone shaper object                                                  */
/* ======================================================================== */

/**
 * A reusable trajectory generator. The controller has this built in
 * (PID_SetSetpointRamp); use this type when you want to shape a signal that
 * is not a PID setpoint, or to run a profile ahead of time.
 */
typedef struct {
    PID_Float position;
    PID_Float velocity;
    PID_Float target;
    PID_Float rate_max;
    PID_Float accel;
    PID_Float decel;
    bool      moving;
} PID_Shaper;

/**
 * @param rate_max Speed limit [unit/s]; 0 disables shaping (pass-through).
 * @param accel    [unit/s^2]; 0 gives the rate-only profile.
 * @param decel    [unit/s^2]; 0 mirrors @p accel.
 */
PID_StatusCode PID_Shaper_Init(PID_Shaper *s, PID_Float rate_max,
                               PID_Float accel, PID_Float decel);

/** Command a new destination. Does not reset velocity - the profile blends. */
PID_StatusCode PID_Shaper_SetTarget(PID_Shaper *s, PID_Float target);

/** Teleport to @p position with zero velocity (e.g. after homing). */
PID_StatusCode PID_Shaper_Reset(PID_Shaper *s, PID_Float position);

/** Advance one timestep. @return the new shaped position. */
PID_Float PID_Shaper_Update(PID_Shaper *s, PID_Float dt);

/** @return true while the profile is still in motion. */
bool PID_Shaper_IsMoving(const PID_Shaper *s);

/**
 * Estimate the time to reach the current target from rest, in seconds.
 *
 * Trapezoidal: if the distance is long enough to reach cruise speed,
 *   t = d/v + v/(2a) + v/(2b);
 * otherwise the profile is triangular and peaks at
 *   v_peak = sqrt(2*d*a*b/(a+b)).
 * Returns 0 when already at the target. Useful for sequencing and for a UI
 * progress estimate; it ignores the current velocity by design.
 */
PID_Float PID_Shaper_EstimateTime(const PID_Shaper *s);

#ifdef __cplusplus
}
#endif

#endif /* PIDX_PID_SHAPER_H */

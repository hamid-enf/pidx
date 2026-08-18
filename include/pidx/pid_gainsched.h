/**
 * @file    pid_gainsched.h
 * @brief   Interpolated gain scheduling.
 *
 * @section why Why gain scheduling
 *
 * A single set of gains is only optimal near one operating point. A valve is
 * more effective when it is half open than when it is nearly shut; a motor's
 * effective inertia changes with arm extension; a heater's loss coefficient
 * grows with temperature. Gain scheduling stores gains at several operating
 * points and interpolates between them.
 *
 * @section how How this implementation avoids discontinuities
 *
 * Two properties together guarantee a bumpless traversal of the table:
 *
 *  1. Gains are interpolated continuously (linear or smoothstep), not selected
 *     from discrete regions. A region-select scheme steps the P term the
 *     instant the boundary is crossed.
 *  2. The core stores the integral term in output units, so changing Ki does
 *     not rescale accumulated history.
 *
 * With PID_SCHED_INTERP_SMOOTH the gain curve is also C1-continuous, which
 * matters when the scheduling variable is itself noisy: linear interpolation
 * has a slope discontinuity at every breakpoint that noise will rattle.
 *
 * @section hysteresis Hysteresis
 *
 * If the scheduling variable is noisy, set a non-zero hysteresis band. The
 * evaluated variable must move by more than the band before the interpolation
 * position is updated, which stops gains dithering at a breakpoint.
 */
#ifndef PIDX_PID_GAINSCHED_H
#define PIDX_PID_GAINSCHED_H

#include "pid_types.h"

#if PIDX_ENABLE_GAIN_SCHED

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum breakpoints in one schedule. Raise if you genuinely need more. */
#ifndef PIDX_GAINSCHED_MAX_POINTS
#define PIDX_GAINSCHED_MAX_POINTS  16U
#endif

/** Which signal drives the schedule. */
typedef enum {
    PID_SCHED_SRC_SETPOINT = 0,   /**< Effective (post-shaper) setpoint.      */
    PID_SCHED_SRC_MEASUREMENT,    /**< Filtered measurement.                  */
    PID_SCHED_SRC_ERROR,          /**< Signed error.                          */
    PID_SCHED_SRC_ABS_ERROR,      /**< |error| - classic "gain by distance".  */
    PID_SCHED_SRC_OUTPUT,         /**< Previous output - actuator authority.  */
    PID_SCHED_SRC_EXTERNAL        /**< Whatever you pass in; see SetVar().    */
} PID_SchedSource;

/** Interpolation between breakpoints. */
typedef enum {
    PID_SCHED_INTERP_LINEAR = 0,  /**< C0 continuous. Cheapest.               */
    PID_SCHED_INTERP_SMOOTH,      /**< Smoothstep, C1 continuous.             */
    PID_SCHED_INTERP_HOLD         /**< Piecewise constant. Steps the gains -
                                       only for plants where that is fine.    */
} PID_SchedInterp;

/** One breakpoint. The table must be sorted by ascending @c x. */
typedef struct {
    PID_Float x;                  /**< Scheduling variable value.             */
    PID_Float kp;
    PID_Float ki;
    PID_Float kd;
} PID_GainPoint;

/**
 * A schedule. The point array is referenced, not copied, so it can live in
 * Flash as `static const`.
 */
typedef struct PID_GainSchedule {
    const PID_GainPoint *points;
    uint8_t          count;
    uint8_t          source;      /**< PID_SchedSource.                       */
    uint8_t          interp;      /**< PID_SchedInterp.                       */
    uint8_t          reserved;
    PID_Float        hysteresis;  /**< Deadband on the scheduling variable.   */
    PID_Float        last_x;      /**< Internal: last accepted variable.      */
    bool             primed;      /**< Internal: last_x is meaningful.        */
} PID_GainSchedule;

/**
 * Validate and initialise a schedule.
 * @param points  Sorted by ascending x, at least 2 entries, all gains finite
 *                and non-negative. Not copied - must outlive the schedule.
 * @return PID_OK, PID_ERR_NULL, PID_ERR_INVALID_PARAM (bad count/order) or
 *         PID_ERR_INVALID_GAIN.
 */
PID_StatusCode PID_GainSched_Init(PID_GainSchedule *s,
                                  const PID_GainPoint *points,
                                  uint8_t count,
                                  PID_SchedSource source,
                                  PID_SchedInterp interp);

/** Set the hysteresis band applied to the scheduling variable. */
PID_StatusCode PID_GainSched_SetHysteresis(PID_GainSchedule *s, PID_Float band);

/**
 * Attach a schedule to a controller and enable PID_FEAT_GAIN_SCHED.
 * Pass NULL to detach; the gains then stay at their last interpolated values.
 */
PID_StatusCode PID_GainSched_Attach(PID_Handle *h, PID_GainSchedule *s);

/** Supply the scheduling variable when the source is PID_SCHED_SRC_EXTERNAL. */
PID_StatusCode PID_GainSched_SetVar(PID_Handle *h, PID_Float value);

/**
 * Evaluate the schedule at @p x. Exposed for testing and for users who want to
 * drive their own gain logic.
 */
PID_StatusCode PID_GainSched_Evaluate(PID_GainSchedule *s, PID_Float x,
                                      PID_Float *kp, PID_Float *ki, PID_Float *kd);

#ifdef __cplusplus
}
#endif

#endif /* PIDX_ENABLE_GAIN_SCHED */
#endif /* PIDX_PID_GAINSCHED_H */

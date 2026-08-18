/**
 * @file    pid_cascade.h
 * @brief   Cascade control: 2..N nested loops driven as one unit.
 *
 * @section idea The idea
 *
 * A single loop must be tuned for the slowest dynamics in the plant. Cascade
 * splits the problem: a fast inner loop closes around the fast, well-behaved
 * part (motor current, valve flow, jacket temperature) while a slow outer loop
 * commands the inner loop's setpoint.
 *
 *     sp --->[ outer PID ]---> sp_inner --->[ inner PID ]---> actuator
 *                 ^                              ^
 *                 |                              |
 *              y_outer                        y_inner
 *
 * What you gain:
 *  - Disturbances hitting the inner variable are rejected by the fast loop
 *    before the outer loop ever notices them.
 *  - The outer loop sees the inner loop as its "actuator", and a closed inner
 *    loop is far more linear than the raw plant. Motor torque ripple, valve
 *    stiction and supply-voltage sag stop being the outer loop's problem.
 *
 * @section rule The one rule that makes it work
 *
 * The inner loop must be substantially faster than the outer one - a factor of
 * 3 to 10 in bandwidth (equivalently, in sample rate) is the standard
 * engineering guidance. If the two loops have similar speeds they interact and
 * the cascade is worse than a single well-tuned loop. PID_Cascade_Validate()
 * checks this ratio for you and reports it; it does not silently "fix" it,
 * because the fix is a design decision, not a computation.
 *
 * Tune inner-first, always: close the inner loop, verify it, then tune the
 * outer loop against the closed inner loop.
 *
 * @section windup The hard part: outer-loop windup
 *
 * This is what a naive cascade implementation gets wrong. When the inner loop
 * saturates its actuator, the inner measurement can no longer follow the
 * setpoint the outer loop is asking for. The outer loop sees persistent error
 * and integrates - forever - even though its output is doing nothing. On
 * recovery the outer integrator dumps a huge command and the system overshoots
 * badly.
 *
 * Output limits on the outer loop are NOT sufficient. The outer loop's output
 * can be perfectly inside its own limits while the inner loop is pinned at the
 * actuator rail.
 *
 * PIDX solves it the correct way: back-propagation of saturation. Each cycle,
 * from the innermost loop outwards, a loop that cannot deliver reports the
 * value it actually achieved, and its parent's integrator is corrected to that
 * achievable value via the parent's existing anti-windup path. See
 * PID_CASCADE_AW_* for the two available strategies.
 *
 * @section rates Multi-rate operation
 *
 * The whole point of cascade is that the loops run at different rates. Set
 * @c decimation on each level: the inner loop runs every call, an outer level
 * with decimation = 5 runs every 5th call and holds its output in between.
 * That is exactly how you would wire it in an ISR, and it keeps the phase
 * relationship between levels fixed and predictable.
 *
 * @section scope Scope
 *
 * This is a coordination layer. It contains no control law of its own - every
 * loop is an ordinary PID_Handle you configure, tune and inspect normally. It
 * owns no memory: you supply the array of handles.
 */
#ifndef PIDX_PID_CASCADE_H
#define PIDX_PID_CASCADE_H

#include "pid.h"

#if PIDX_ENABLE_CASCADE

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Maximum number of levels in one cascade.
 *
 * Three (position -> velocity -> current) is the deepest arrangement in
 * routine industrial use. The limit exists so the coordinator can hold its
 * per-level state inline, with no allocation.
 */
#ifndef PIDX_CASCADE_MAX_LOOPS
#define PIDX_CASCADE_MAX_LOOPS    4U
#endif

/**
 * How a saturated inner loop protects its parent's integrator.
 */
typedef enum {
    /**
     * Do nothing. Only correct when no loop can ever saturate, which in
     * practice means never. Present so the effect of the other modes can be
     * demonstrated and tested against a known-bad baseline.
     */
    PID_CASCADE_AW_NONE = 0,

    /**
     * Back-calculation (default, recommended).
     *
     * The child reports what it could actually achieve. The parent's
     * integrator is nudged towards the command that would have produced that
     * achievable value:
     *
     *     I_parent += Kt_c * (u_achievable - u_parent) * dt_parent
     *
     * The correction is proportional to the shortfall and fades out smoothly
     * as the child recovers, so there is no discontinuity in the parent's
     * output. Kt_c is @c aw_gain, in 1/s.
     */
    PID_CASCADE_AW_BACK_CALC,

    /**
     * Freeze the parent's integrator for as long as the child is saturated in
     * the direction the parent is pushing.
     *
     * Simpler and completely prevents further windup, but it does not unwind
     * what has already accumulated, so recovery is slower than back-calc.
     * Directional: the parent may still integrate the other way, which is what
     * lets it escape saturation.
     */
    PID_CASCADE_AW_FREEZE
} PID_CascadeAntiWindup;

/**
 * Per-level configuration.
 *
 * Index 0 is the OUTERMOST loop (the one that receives the user's setpoint).
 * The last configured level is the innermost, and its output drives the
 * physical actuator. This ordering matches how the loops are named in every
 * textbook diagram - primary/outer first.
 */
typedef struct {
    PID_Handle *pid;        /**< Initialised handle. Not owned, never freed.  */

    /**
     * Run this level once every @c decimation calls to PID_Cascade_Update().
     * 1 = every call. Level 0 (innermost, index n-1) is normally 1.
     * A value of 0 is treated as 1.
     */
    uint16_t    decimation;

    /**
     * Clamp applied to this level's output before it becomes the child's
     * setpoint. Set these to the child's physically meaningful range - an
     * outer position loop should not be allowed to command 10000 rad/s.
     * If @c sp_min >= @c sp_max the clamp is skipped and the child's own
     * setpoint validation applies instead.
     *
     * @note This is a range limit on the command, not anti-windup. Both are
     * needed: the clamp keeps the request sane, the anti-windup keeps the
     * integrator honest when even a sane request cannot be met.
     */
    PID_Float   sp_min;
    PID_Float   sp_max;
} PID_CascadeLevel;

/**
 * Cascade coordinator. Allocate statically; it holds pointers plus a small
 * amount of per-level bookkeeping.
 */
typedef struct {
    PID_CascadeLevel level[PIDX_CASCADE_MAX_LOOPS];

    /** Sub-sample counters, one per level. */
    uint16_t  tick[PIDX_CASCADE_MAX_LOOPS];

    /** Last command each level issued, held between its own updates. */
    PID_Float command[PIDX_CASCADE_MAX_LOOPS];

    uint8_t   count;        /**< Number of active levels, 2..MAX.             */
    uint8_t   aw_mode;      /**< PID_CascadeAntiWindup.                       */
    uint8_t   mode;         /**< PID_Mode applied to the whole chain.         */
    bool      initialised;

    /**
     * Back-calculation gain Kt_c [1/s] for PID_CASCADE_AW_BACK_CALC.
     *
     * Rule of thumb: 1/Ti of the OUTER loop, i.e. unwind roughly as fast as
     * that loop winds up. PID_Cascade_Init() derives it automatically from the
     * outer loop's gains; override with PID_Cascade_SetAntiWindup() if you
     * want a specific value. Larger = more aggressive unwinding, but too large
     * fights the loop's normal action.
     */
    PID_Float aw_gain;

    PID_Float output;       /**< Last actuator command (innermost output).    */
    PID_StatusCode last_error;
} PID_Cascade;

/* ======================================================================== */
/* Lifecycle                                                                 */
/* ======================================================================== */

/**
 * Bind an array of already-initialised handles into a cascade.
 *
 * @param c      Coordinator to initialise.
 * @param loops  Array of handle pointers, OUTERMOST first. Copied into the
 *               coordinator by value (the pointers, not the handles).
 * @param n      Number of levels, 2..PIDX_CASCADE_MAX_LOOPS.
 *
 * Every handle must already have been through PID_Init() / PID_InitDefault().
 * All levels get decimation 1 and no setpoint clamp; use
 * PID_Cascade_ConfigLevel() to change that. Anti-windup defaults to
 * BACK_CALC with @c aw_gain derived from the outer loop.
 *
 * @retval PID_ERR_NULL          @p c or @p loops NULL, or a handle is NULL.
 * @retval PID_ERR_INVALID_PARAM @p n out of range.
 * @retval PID_ERR_NOT_INIT      A handle was not initialised.
 */
PID_StatusCode PID_Cascade_Init(PID_Cascade *c, PID_Handle *const *loops, uint8_t n);

/**
 * Configure one level.
 *
 * @param index      0 = outermost.
 * @param decimation Run every Nth call; 0 or 1 means every call.
 * @param sp_min,sp_max Clamp on the command this level sends downstream.
 *                   Pass min >= max to disable the clamp.
 */
PID_StatusCode PID_Cascade_ConfigLevel(PID_Cascade *c, uint8_t index,
                                       uint16_t decimation,
                                       PID_Float sp_min, PID_Float sp_max);

/**
 * Select the anti-windup strategy and its gain.
 *
 * @param aw_gain Only used by BACK_CALC. Pass <= 0 to keep the value derived
 *                at Init time.
 */
PID_StatusCode PID_Cascade_SetAntiWindup(PID_Cascade *c,
                                         PID_CascadeAntiWindup mode,
                                         PID_Float aw_gain);

/* ======================================================================== */
/* Execution                                                                 */
/* ======================================================================== */

/**
 * Run one cascade cycle.
 *
 * @param c            Coordinator.
 * @param measurements One measurement per level, in the SAME order as the
 *                     handles were passed to Init: index 0 is the outermost
 *                     loop's process variable, index n-1 the innermost.
 * @param setpoint     Target for the outermost loop.
 * @param dt           Elapsed time since the previous call [s]. Each level is
 *                     updated with its own accumulated dt (dt * decimation),
 *                     so the integral and derivative terms stay correct at
 *                     every rate.
 *
 * Execution order is outermost to innermost: each level computes its output,
 * that output (clamped) becomes the next level's setpoint, and the innermost
 * level's output is returned. Saturation is then propagated back inwards-to-
 * outwards in the same call, so no correction is ever a cycle late.
 *
 * @return The actuator command from the innermost loop. On error, the previous
 *         output is held and the reason is recorded in @c last_error.
 */
PID_Float PID_Cascade_Update(PID_Cascade *c, const PID_Float *measurements,
                             PID_Float setpoint, PID_Float dt);

/**
 * Set the mode of every loop in the chain, coordinated and bumpless.
 *
 * Going to MANUAL: the innermost loop takes the manual value and each outer
 * loop is back-solved so that its output equals its child's current setpoint.
 * The whole chain therefore holds a consistent state, and the return to
 * AUTOMATIC is bumpless at every level - not just the innermost one.
 *
 * Switching modes level by level with PID_SetMode() does NOT achieve this and
 * will produce a bump when the chain re-engages.
 */
PID_StatusCode PID_Cascade_SetMode(PID_Cascade *c, PID_Mode mode);

/**
 * Manual actuator command. Only meaningful in MANUAL mode; it is applied to
 * the innermost loop, and the outer loops track it.
 */
PID_StatusCode PID_Cascade_SetManualOutput(PID_Cascade *c, PID_Float output);

/** Reset every loop and all coordinator state. Gains are untouched. */
PID_StatusCode PID_Cascade_Reset(PID_Cascade *c);

/* ======================================================================== */
/* Inspection                                                                */
/* ======================================================================== */

/** Last actuator command. */
PID_Float PID_Cascade_GetOutput(const PID_Cascade *c);

/** The setpoint level @p index is currently being asked to follow. */
PID_Float PID_Cascade_GetLevelSetpoint(const PID_Cascade *c, uint8_t index);

/** Handle at @p index, or NULL. For tuning and diagnostics. */
PID_Handle *PID_Cascade_GetLoop(const PID_Cascade *c, uint8_t index);

/** True while any level is saturated - the usual "cascade is struggling" cue.*/
bool PID_Cascade_IsSaturated(const PID_Cascade *c);

/** Sticky error, cleared on read. */
PID_StatusCode PID_Cascade_GetLastError(PID_Cascade *c);

/**
 * Check the timescale separation between adjacent levels.
 *
 * For each pair, computes the effective rate ratio
 * (parent sample interval) / (child sample interval), where each level's
 * interval is its own sample time multiplied by its decimation.
 *
 * @param min_ratio   Out, optional: the worst (smallest) ratio found.
 * @param worst_index Out, optional: index of the parent in that worst pair.
 *
 * @retval PID_OK               Every adjacent pair is separated by >= 3x.
 * @retval PID_ERR_INVALID_PARAM Some pair is too close together. The cascade
 *                              will still run - this is advice, not a veto -
 *                              but expect the loops to fight each other.
 *
 * Call it once after tuning, not in the control loop.
 */
PID_StatusCode PID_Cascade_Validate(const PID_Cascade *c,
                                    PID_Float *min_ratio,
                                    uint8_t *worst_index);

#ifdef __cplusplus
}
#endif

#endif /* PIDX_ENABLE_CASCADE */
#endif /* PIDX_PID_CASCADE_H */

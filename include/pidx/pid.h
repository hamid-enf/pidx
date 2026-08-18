/**
 * @file    pid.h
 * @brief   PIDX - portable, modular, high-performance PID control framework.
 *
 * @section levels API levels
 *
 * Level 1 (Basic)        - 6 functions, no concepts beyond "gains and setpoint".
 * Level 2 (Intermediate) - limits, anti-windup, filtering, modes, shaping.
 * Level 3 (Advanced)     - 2DOF, feedforward, external reset, safety, status.
 * Level 4 (Expert)       - separate headers: gain scheduling, cascade,
 *                          auto-tuning, telemetry, fixed point.
 *
 * @section threading Threading and ISR rules
 *
 * The library never allocates, never uses global mutable state, and never
 * calls into an OS. Consequences:
 *
 *  - Different PID_Handle objects are fully independent. Running one handle in
 *    a timer ISR and another in a task needs no locking.
 *  - A single handle is NOT internally synchronised. Calling PID_Update() from
 *    an ISR while a task calls PID_SetGains() on the same handle is a data
 *    race: gains are multi-word and are not written atomically.
 *    Recommended patterns are in docs/19_rtos_isr.md; the short version is
 *    "mutate the handle from the same context that updates it, or guard the
 *    setter with a critical section".
 *  - PID_Update() is reentrant with respect to distinct handles and is safe to
 *    call from an ISR. It performs no I/O and takes no locks.
 *
 * @section errors Error reporting
 *
 * Configuration functions return PID_StatusCode. The hot-path update functions
 * return the control output and record problems in a sticky per-handle error
 * plus status flags, so a rare event between two samples is never lost. Use
 * PID_UpdateEx() when you want both the output and an immediate code.
 */
#ifndef PIDX_PID_H
#define PIDX_PID_H

#include "pid_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* LEVEL 1 - BASIC                                                           */
/* ======================================================================== */

/**
 * Initialise a handle with safe defaults: Kp=Ki=Kd=0, dt=10 ms, direct action,
 * automatic mode, derivative on measurement, clamp anti-windup, no output
 * limits, beta=1, gamma=0.
 *
 * Because all gains start at zero the controller outputs zero until you call
 * PID_SetGains() - a deliberately inert starting point.
 *
 * @param h  Handle to initialise. Must not be NULL.
 * @return PID_OK, or PID_ERR_NULL.
 *
 * Thread safety: not safe to call concurrently with any other call on @p h.
 *
 * @code
 * PID_Handle pid;
 * PID_InitDefault(&pid);
 * PID_SetGains(&pid, 2.0f, 0.5f, 0.1f);
 * PID_SetOutputLimits(&pid, 0.0f, 100.0f);
 * PID_SetSetpoint(&pid, 50.0f);
 * for (;;) { actuate(PID_Update(&pid, read_sensor())); }
 * @endcode
 */
PID_StatusCode PID_InitDefault(PID_Handle *h);

/**
 * Set all three gains at once, bumplessly.
 *
 * Because the handle stores the integral term in output units rather than the
 * raw integral of the error, changing Ki does not rescale accumulated history
 * and therefore does not step the output. Use
 * PID_SetGainsRescaleIntegral() if you want the classic rescaling behaviour.
 *
 * @param kp  Proportional gain. Must be finite and >= 0.
 * @param ki  Integral gain [1/s]. Must be finite and >= 0.
 * @param kd  Derivative gain [s]. Must be finite and >= 0.
 * @return PID_OK, PID_ERR_NULL, PID_ERR_NOT_INIT or PID_ERR_INVALID_GAIN.
 *
 * Side effects: recomputes internal coefficients; may recompute Tf when the
 * derivative filter is specified through N.
 * Thread safety: not atomic. See the threading section.
 */
PID_StatusCode PID_SetGains(PID_Handle *h, PID_Float kp, PID_Float ki, PID_Float kd);

/**
 * Set the target value. When the setpoint shaper is enabled the controller
 * ramps towards this target instead of jumping to it.
 * @return PID_OK, PID_ERR_NULL, PID_ERR_NOT_INIT or PID_ERR_INVALID_PARAM (NaN/Inf).
 */
PID_StatusCode PID_SetSetpoint(PID_Handle *h, PID_Float setpoint);

/**
 * Run one control cycle using the nominal sample time.
 *
 * Call this at a genuinely periodic rate - from a timer interrupt or an RTOS
 * task with absolute-time delays. Jitter directly perturbs the I and D terms.
 *
 * @param h            Handle. NULL returns 0.
 * @param measurement  Process variable.
 * @return Control output, already saturated and slew-limited.
 *
 * Errors are recorded in the sticky error and in the status flags.
 * Thread safety: reentrant across distinct handles; ISR-safe.
 */
PID_Float PID_Update(PID_Handle *h, PID_Float measurement);

/**
 * Clear all controller state: integrator, derivative memory, filters, shaper
 * velocity, fault latch and flags. Gains, limits and configuration survive.
 *
 * Call this after the loop has been open for a while (actuator disabled,
 * plant changed) so that stale history does not kick the output.
 */
PID_StatusCode PID_Reset(PID_Handle *h);

/** @return The most recent output, or 0 if @p h is NULL. */
PID_Float PID_GetOutput(const PID_Handle *h);

/* ======================================================================== */
/* LEVEL 2 - INTERMEDIATE                                                    */
/* ======================================================================== */

/**
 * Fill @p cfg with the same safe defaults PID_InitDefault() uses and stamp the
 * ABI version. Always call this before customising a PID_Config: it guarantees
 * that fields added in future versions get meaningful values.
 */
PID_StatusCode PID_ConfigDefault(PID_Config *cfg);

/**
 * Initialise a handle from a fully specified configuration.
 * The configuration is validated as a whole; on failure the handle is left
 * uninitialised and a specific code is returned.
 * @return PID_OK, PID_ERR_NULL, PID_ERR_INVALID_CONFIG, PID_ERR_INVALID_GAIN,
 *         PID_ERR_INVALID_LIMIT or PID_ERR_INVALID_DT.
 */
PID_StatusCode PID_Init(PID_Handle *h, const PID_Config *cfg);

/**
 * Invalidate a handle. No memory is released (none was allocated); the state
 * is zeroed, telemetry is detached and the magic is cleared so that any later
 * PID_Update() returns 0 and reports PID_ERR_NOT_INIT instead of running with
 * stale coefficients.
 */
PID_StatusCode PID_Deinit(PID_Handle *h);

/**
 * Change the nominal sample time [s] and rebuild all dt-dependent coefficients.
 * @return PID_ERR_INVALID_DT if dt <= 0 or is not finite.
 */
PID_StatusCode PID_SetSampleTime(PID_Handle *h, PID_Float dt);

/**
 * @return The nominal sample time, or 0 if @p h is NULL or was never
 *         initialised. The "never initialised" case matters: it is how
 *         PID_Cascade_Init() detects a garbage loop in its array.
 */
PID_Float PID_GetSampleTime(const PID_Handle *h);

/**
 * Run one control cycle with an explicitly measured sample time.
 *
 * Use this when your loop is driven by an event whose period genuinely varies
 * (an encoder capture, a CAN frame). When @p dt equals the value used on the
 * previous call the coefficient cache hits and the cost is one comparison;
 * otherwise three divisions are performed to rebuild the coefficients.
 *
 * @param dt  Elapsed time [s]. Must be > 0 and, if configured, within
 *            [dt_min, dt_max]. Invalid values raise PID_ERR_INVALID_DT, set
 *            PID_FLAG_DT_VIOLATION and fall back to the nominal sample time.
 */
PID_Float PID_UpdateDt(PID_Handle *h, PID_Float measurement, PID_Float dt);

/**
 * Constrain the controller output to [min, max] and enable output limiting.
 * When integral limits were not set explicitly they follow these bounds, which
 * is almost always what you want: it keeps the integrator inside the range the
 * actuator can actually deliver.
 * @return PID_ERR_INVALID_LIMIT if min >= max or either bound is not finite.
 */
PID_StatusCode PID_SetOutputLimits(PID_Handle *h, PID_Float min, PID_Float max);

/** Disable output clamping. The integrator limits, if any, remain. */
PID_StatusCode PID_ClearOutputLimits(PID_Handle *h);

/**
 * Bound the integral term explicitly, in output units. Overrides the bounds
 * inherited from the output limits.
 */
PID_StatusCode PID_SetIntegralLimits(PID_Handle *h, PID_Float min, PID_Float max);

/**
 * Select the anti-windup strategy.
 *
 * @param mode  See PID_AntiWindup.
 * @param kt    Back-calculation gain [1/s], used by BACK_CALCULATION and
 *              TRACKING. Pass 0 to derive it automatically as
 *              Kt = Ki/Kp (i.e. 1/Ti) when both gains are non-zero, which is
 *              the standard robust choice; with Kd present the library uses
 *              the Astrom recommendation Kt = 1/sqrt(Ti*Td).
 * @return PID_ERR_INVALID_PARAM for an unknown mode or a negative kt;
 *         PID_ERR_INVALID_LIMIT if BACK_CALCULATION is selected without finite
 *         output limits (the correction term would be meaningless).
 */
PID_StatusCode PID_SetAntiWindup(PID_Handle *h, PID_AntiWindup mode, PID_Float kt);

/** Choose what the derivative differentiates. Default: on measurement. */
PID_StatusCode PID_SetDerivativeMode(PID_Handle *h, PID_DerivativeMode mode);

/**
 * Set the derivative filter time constant directly [s].
 *
 * The implemented form is unconditionally stable for any tf >= 0 and any
 * dt > 0. tf = 0 selects the raw backward difference, which is only advisable
 * on a genuinely quiet signal.
 *
 * Rule of thumb: tf = Td/N with N in 5..20, i.e. filter roughly one decade
 * above the derivative's corner. Larger N = sharper derivative + more noise.
 */
PID_StatusCode PID_SetDerivativeFilter(PID_Handle *h, PID_Float tf);

/**
 * Set the derivative filter through the ratio N, giving Tf = Kd/(N*Kp).
 * Tf is then recomputed automatically whenever Kp or Kd changes.
 * @return PID_ERR_INVALID_PARAM if n <= 0.
 */
PID_StatusCode PID_SetDerivativeFilterN(PID_Handle *h, PID_Float n);

/** Select direct or reverse action. Takes effect on the next update. */
PID_StatusCode PID_SetDirection(PID_Handle *h, PID_Direction dir);

/**
 * Switch operating mode. Every transition is bumpless:
 *  - AUTOMATIC -> MANUAL: the manual output is seeded with the current output.
 *  - MANUAL -> AUTOMATIC: the integrator was already being back-solved each
 *    sample, so the first automatic output equals the last manual one.
 */
PID_StatusCode PID_SetMode(PID_Handle *h, PID_Mode mode);

/** @return Current mode, or PID_MODE_MANUAL if @p h is NULL. */
PID_Mode PID_GetMode(const PID_Handle *h);

/** Set the value driven while in PID_MODE_MANUAL (clamped to output limits). */
PID_StatusCode PID_SetManualOutput(PID_Handle *h, PID_Float output);

/**
 * @brief Read back the manual output value.
 *
 * This is the value most recently passed to PID_SetManualOutput(), which is
 * not the same thing as PID_GetOutput(): the latter reports the output of the
 * last completed PID_Update(), so it still holds the previous cycle's value
 * until the controller is stepped again.
 *
 * Use this when you need the commanded manual level before the next update -
 * for example to seed an auto-tune bias from the operator's current setting.
 *
 * @return The manual output, or 0 if @p h is NULL.
 */
PID_Float PID_GetManualOutput(const PID_Handle *h);

/**
 * Configure setpoint trajectory shaping.
 * @param rate_max  Maximum |d(sp)/dt| [unit/s]. 0 disables shaping.
 * @param accel     Acceleration [unit/s^2]. 0 gives a rate-only ramp.
 * @param decel     Deceleration [unit/s^2]. 0 mirrors @p accel.
 * With a non-zero accel the shaper produces a trapezoidal velocity profile and
 * begins braking at v^2/(2*decel) from the target, so it lands without
 * overshooting the setpoint.
 * @return PID_ERR_UNSUPPORTED when the shaper was not compiled in.
 */
PID_StatusCode PID_SetSetpointRamp(PID_Handle *h, PID_Float rate_max,
                                   PID_Float accel, PID_Float decel);

/** Limit |du/dt| [unit/s] on the final output. 0 disables. */
PID_StatusCode PID_SetOutputSlewRate(PID_Handle *h, PID_Float slew_max);

/**
 * Enable a first-order low-pass on the measurement.
 * @param tau  Time constant [s]. 0 disables.
 * Prefer this over widening the derivative filter when the noise also disturbs
 * the P term; remember it adds phase lag to the whole loop.
 */
PID_StatusCode PID_SetInputFilter(PID_Handle *h, PID_Float tau);

/* ======================================================================== */
/* LEVEL 3 - ADVANCED                                                        */
/* ======================================================================== */

/** Initialise a PID_Input so every optional field means "unchanged". */
void PID_InputInit(PID_Input *in);

/**
 * Full-control update.
 * @param in   Input bundle; NaN fields fall back to handle state.
 * @param err  Optional; receives the status for this cycle.
 * @return Control output.
 */
PID_Float PID_UpdateEx(PID_Handle *h, const PID_Input *in, PID_StatusCode *err);

/**
 * Minimal-overhead update.
 *
 * Executes exactly: input scaling, P (with beta), backward-Euler I, filtered D
 * on measurement, sum, clamp, integrator clamp. It deliberately IGNORES the
 * shaper, safety checks, gain scheduling, feedforward, input filter,
 * diagnostics, telemetry and mode handling - it does not test for them.
 *
 * Use it for tight inner loops (current control at tens of kHz) where you have
 * decided that a plain PID is all you need. If any ignored feature is enabled
 * the result will silently differ from PID_Update(); PID_UpdateFast_IsSafe()
 * lets you assert against that in development.
 *
 * NULL is tolerated (returns 0), but an UNINITIALISED handle is not checked -
 * that is the safety this entry point trades for speed. Call it only on a
 * handle PID_Init() has accepted.
 */
PID_Float PID_UpdateFast(PID_Handle *h, PID_Float measurement);

/**
 * @return true when the handle's runtime features are all understood by
 *         PID_UpdateFast(), i.e. using it changes nothing.
 */
bool PID_UpdateFast_IsSafe(const PID_Handle *h);

/** Individual gain setters. All bumpless, all validated. */
PID_StatusCode PID_SetKp(PID_Handle *h, PID_Float kp);
PID_StatusCode PID_SetKi(PID_Handle *h, PID_Float ki);
PID_StatusCode PID_SetKd(PID_Handle *h, PID_Float kd);

/** Read back the active gains (post gain-scheduling). Pointers may be NULL. */
PID_StatusCode PID_GetGains(const PID_Handle *h, PID_Float *kp, PID_Float *ki, PID_Float *kd);

/**
 * Change gains while preserving the raw integral of the error, i.e. rescale
 * the stored term by ki_new/ki_old.
 *
 * This reproduces the behaviour of textbook implementations that store the
 * unscaled integral, and it WILL step the output when Ki changes. Use it only
 * when you specifically want the integral's physical meaning preserved across
 * a retune. If ki_old is zero the term cannot be rescaled and is left as is.
 */
PID_StatusCode PID_SetGainsRescaleIntegral(PID_Handle *h, PID_Float kp,
                                           PID_Float ki, PID_Float kd);

/**
 * Set the 2DOF setpoint weights.
 * @param beta   Weight of r in the P term. 1 = classic. Lowering it towards 0
 *               reduces setpoint overshoot without touching disturbance
 *               rejection - the whole point of 2DOF. Typical 0.5..1.
 * @param gamma  Weight of r in the D term. 0 (default) avoids derivative kick.
 *               Only raise it if you deliberately want setpoint-derivative
 *               action and your setpoint is smooth.
 * @return PID_ERR_INVALID_PARAM unless both are finite and within [0, 2].
 */
PID_StatusCode PID_SetWeights(PID_Handle *h, PID_Float beta, PID_Float gamma);

/** Set a constant feedforward contribution (added before saturation). */
PID_StatusCode PID_SetFeedforward(PID_Handle *h, PID_Float ff);

/**
 * Install a feedforward callback evaluated every cycle.
 * @param fn    Called as fn(setpoint, measurement, ctx). Must be fast and must
 *              not block: it runs in the update context, often an ISR.
 *              NULL reverts to the static value.
 * @param gain  Scale factor applied to the callback result.
 */
PID_StatusCode PID_SetFeedforwardFn(PID_Handle *h, PID_FeedforwardFn fn,
                                    void *ctx, PID_Float gain);

/**
 * Integral separation: suspend integration while |error| exceeds @p threshold.
 * During a large transient the integrator would otherwise charge up and cause
 * overshoot; the P and D terms handle the transient and integration resumes
 * near the target to remove steady-state error. 0 disables.
 */
PID_StatusCode PID_SetIntegralSeparation(PID_Handle *h, PID_Float threshold);

/**
 * Integral deadband: suspend integration while |error| is below @p db.
 * Stops the integrator hunting against actuator quantisation or sensor noise.
 * Costs a permanent steady-state error of at most @p db. 0 disables.
 */
PID_StatusCode PID_SetIntegralDeadband(PID_Handle *h, PID_Float db);

/** Master enable/disable for the integral term. Disabling freezes, not clears. */
PID_StatusCode PID_EnableIntegral(PID_Handle *h, bool enable);

/**
 * Force the integral term (in output units). This is external reset: use it to
 * preload a known operating point, or to hand control between controllers.
 */
PID_StatusCode PID_SetIntegrator(PID_Handle *h, PID_Float value);

/** @return The current integral term in output units. */
PID_Float PID_GetIntegrator(const PID_Handle *h);

/**
 * Provide the external tracking signal used by PID_AW_TRACKING. Typically the
 * actual actuator position read back from the drive, which makes the
 * integrator follow reality whenever something downstream overrides it.
 */
PID_StatusCode PID_SetTrackingInput(PID_Handle *h, PID_Float u_track);

/** Choose backward-Euler or trapezoidal integration. */
PID_StatusCode PID_SetIntegrationMethod(PID_Handle *h, PID_IntegrationMethod m);

/* ---- Safety ---- */

/** Apply a complete safety configuration. */
PID_StatusCode PID_SetSafety(PID_Handle *h, const PID_SafetyConfig *sc);

/** Set the value driven while a fault is latched. */
PID_StatusCode PID_SetFaultOutput(PID_Handle *h, PID_Float output);

/** Clear a latched fault and resume control bumplessly. */
PID_StatusCode PID_ClearFault(PID_Handle *h);

/** @return true while a fault is latched. */
bool PID_IsFaulted(const PID_Handle *h);

/* ---- Feature control, status, errors ---- */

/**
 * Enable or disable runtime features.
 * @param mask  One or more PID_FEAT_* bits.
 * @return PID_ERR_UNSUPPORTED if any bit names a module that is not compiled
 *         in - the request is then rejected as a whole.
 */
PID_StatusCode PID_EnableFeature(PID_Handle *h, uint32_t mask, bool enable);

/** @return true if every bit in @p mask is currently enabled. */
bool PID_IsFeatureEnabled(const PID_Handle *h, uint32_t mask);

/** @return The raw status flag word (PID_FLAG_*). */
uint16_t PID_GetFlags(const PID_Handle *h);

/** @return true if the output was saturated on the last cycle. */
bool PID_IsSaturated(const PID_Handle *h);

/** @return Direction-corrected error from the last cycle. */
PID_Float PID_GetError(const PID_Handle *h);

/** @return The effective (post-shaper) setpoint. */
PID_Float PID_GetSetpoint(const PID_Handle *h);

/**
 * Read and clear the sticky error.
 * @param code  Receives the stored code. May be NULL to just clear.
 */
PID_StatusCode PID_GetLastError(PID_Handle *h, PID_StatusCode *code);

/** Read the sticky error without clearing it. */
PID_StatusCode PID_PeekLastError(const PID_Handle *h);

/** Clear the sticky error and the transient status flags. */
PID_StatusCode PID_ClearError(PID_Handle *h);

#if PIDX_ENABLE_DIAGNOSTICS
/** Copy the full diagnostic snapshot of the last cycle. */
PID_StatusCode PID_GetStatus(const PID_Handle *h, PID_Status *out);
#endif

/**
 * Bypass the setpoint shaper and validation and write the setpoint directly.
 *
 * Provided for hard real-time paths that update the setpoint every sample from
 * a trusted source (an interpolated trajectory, an outer loop). It is the only
 * sanctioned way to touch handle state without an accessor.
 */
PIDX_INLINE void PID_SetSetpointImmediate(PID_Handle *h, PID_Float sp)
{
    h->setpoint_target = sp;
    h->setpoint = sp;
}

/** @return Human-readable name of a status code (static string, never NULL). */
const char *PID_StatusToString(PID_StatusCode code);

/** @return Library version string, e.g. "1.0.0". */
const char *PID_GetVersion(void);

#ifdef __cplusplus
}
#endif

#endif /* PIDX_PID_H */

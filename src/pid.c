/**
 * @file    pid.c
 * @brief   PIDX core controller implementation.
 *
 * @section math Control law
 *
 * Continuous-time 2DOF form with a filtered derivative:
 *
 *   u(t) = Kp*(beta*r - y)                          (P, setpoint-weighted)
 *        + Ki*integral(r - y) dt                    (I, always on full error)
 *        + (Kd*s / (1 + s*Tf)) * (gamma*r - y)      (D, filtered)
 *        + u_ff                                     (feedforward)
 *
 * The integral acts on the unweighted error because any other choice leaves a
 * steady-state offset: at equilibrium the integrator stops only when its input
 * is zero, so its input must be the true error.
 *
 * @section disc Discretisation
 *
 * Integral, backward Euler (default):
 *     I[k] = I[k-1] + Ki*dt*e[k]
 * Integral, trapezoidal (optional):
 *     I[k] = I[k-1] + (Ki*dt/2)*(e[k] + e[k-1])
 *
 * Derivative, backward Euler applied to the *filtered* transfer function:
 *     D[k] = (Tf/(Tf+dt))*D[k-1] - (Kd/(Tf+dt))*(x[k] - x[k-1])
 * where x is the direction-corrected derivative source. Deriving the whole
 * filtered block at once - instead of differentiating and then low-passing -
 * yields a pole at Tf/(Tf+dt), which lies in [0,1) for every Tf >= 0 and every
 * dt > 0. The naive "raw difference then separate LPF" implementation has a
 * pole at 1 - dt/Tf and rings, then diverges, once Tf < dt/2. That single
 * detail is the most common source of "my D term explodes" reports.
 *
 * @section units Integrator units
 *
 * h->integrator holds Ki*integral(e), i.e. the I term already expressed in
 * output units, not the raw integral. Consequences:
 *   - changing Ki at runtime does not rescale history, so no output jump;
 *   - gain scheduling can sweep Ki continuously without discontinuity;
 *   - integrator limits are in the same units as the output limits, so
 *     "clamp the integrator to the actuator range" is expressible directly.
 * PID_SetGainsRescaleIntegral() is provided for users who need the classic
 * rescaling semantics instead.
 */

#include "pidx/pid.h"

#if PIDX_ENABLE_SHAPER
#include "pidx/pid_shaper.h"
#endif
#if PIDX_ENABLE_INPUT_FILTER
#include "pidx/pid_filter.h"
#endif
#if PIDX_ENABLE_GAIN_SCHED
#include "pidx/pid_gainsched.h"
#endif
#if PIDX_ENABLE_TELEMETRY
#include "pidx/pid_diag.h"
#endif

/* ======================================================================== */
/* Internal helpers                                                          */
/* ======================================================================== */

/** Record an error without ever overwriting it with success. */
static void pidp_set_error(PID_Handle *h, PID_StatusCode code)
{
    if (code != PID_OK) {
        h->last_error = code;
    }
}

/** @return true when the handle is usable. */
static bool pidp_valid(const PID_Handle *h)
{
    return ((h != NULL) && (h->init_magic == PID_INIT_MAGIC));
}

/**
 * Derive the effective derivative filter time constant.
 *
 * An explicit tf always wins. Otherwise, if the user expressed the filter as
 * the ratio N, compute Tf = Td/N = Kd/(N*Kp). That needs a non-zero Kp; when
 * Kp is zero (a pure ID controller, unusual but legal) the ratio is undefined
 * and we fall back to an unfiltered derivative rather than inventing a value.
 */
static PID_Float pidp_effective_tf(const PID_Handle *h)
{
    PID_Float tf = PID_ZERO;

    if ((h->features & PID_FEAT_D_FILTER) != 0UL) {
        if (h->tf > PID_ZERO) {
            tf = h->tf;
        } else if ((h->n_filter > PID_ZERO) &&
                   (h->kp > PID_ZERO) &&
                   (h->kd > PID_ZERO)) {
            tf = h->kd / (h->n_filter * h->kp);
        } else {
            tf = PID_ZERO;
        }
    }
    return tf;
}

/**
 * Derive the back-calculation gain Kt when the user passed 0.
 *
 * Standard choices (Astrom & Hagglund, "Advanced PID Control"):
 *   Tt = sqrt(Ti*Td)  when derivative action is present,
 *   Tt = Ti           otherwise.
 * Expressed in parallel-form gains, Ti = Kp/Ki and Td = Kd/Kp, so
 *   Ti*Td = Kd/Ki   ->  Kt = 1/sqrt(Kd/Ki) = sqrt(Ki/Kd),
 *   Kt   = Ki/Kp    when Kd == 0.
 */
static PID_Float pidp_effective_kt(const PID_Handle *h)
{
    PID_Float kt = h->kt;

    if (kt <= PID_ZERO) {
        if ((h->ki > PID_ZERO) && (h->kd > PID_ZERO)) {
            kt = pidm_sqrt(h->ki / h->kd);
        } else if ((h->ki > PID_ZERO) && (h->kp > PID_ZERO)) {
            kt = h->ki / h->kp;
        } else {
            kt = PID_ZERO;
        }
    }
    return kt;
}

/**
 * Rebuild every dt-dependent coefficient. This is the only place in the
 * library that performs a division on behalf of the control law, which is what
 * keeps the update path division-free.
 */
static void pidp_recompute(PID_Handle *h, PID_Float dt)
{
    const PID_Float tf = pidp_effective_tf(h);
    const PID_Float den = tf + dt;

    if (h->integ_method == (uint8_t)PID_INTEGRATION_TRAPEZOIDAL) {
        h->c_i = h->ki * dt * PID_HALF;
    } else {
        h->c_i = h->ki * dt;
    }

    /* den >= dt > 0, so this division is always safe. */
    h->c_da = tf / den;
    h->c_db = h->kd / den;

    h->c_aw = pidp_effective_kt(h) * dt;

#if PIDX_ENABLE_INPUT_FILTER
    pidf_lpf1_coeff(&h->in_lpf, h->in_lpf.tau, dt);
#endif

    h->dt_last = dt;
}

/**
 * Integrator bounds in force this cycle.
 *
 * h->i_min/i_max are maintained as the already-resolved effective bounds by
 * PID_Init, PID_SetIntegralLimits, PID_SetOutputLimits and
 * PID_ClearOutputLimits, so there is nothing to decide here. Keeping the
 * accessor makes the intent explicit at the call sites and gives one place to
 * change if the policy ever grows a case.
 */
static void pidp_integral_bounds(const PID_Handle *h, PID_Float *lo, PID_Float *hi)
{
    *lo = h->i_min;
    *hi = h->i_max;
}

/**
 * Force the integrator so that P + I + D + FF reproduces a desired output.
 * This single operation implements bumpless manual->auto transfer, bumpless
 * fault recovery and integrator preloading.
 *
 * The required value is I = desired - P - D - FF, and it is clamped to the
 * integrator bounds because an unbounded integrator is not an option.
 *
 * When the clamp bites, the transfer CANNOT be bumpless: the requested output
 * is simply not reachable from the current P/D/FF with a legal integrator.
 * The usual cause is switching to automatic while the measurement is far from
 * the setpoint, so the P term alone already exceeds the actuator range - for
 * example P = Kp*e = 0.045*50 = 2.25 on an actuator limited to 1.0, which
 * needs I = -1.85 against a lower bound of 0.
 *
 * That case is flagged with PID_FLAG_INTEGRAL_LIMITED rather than being
 * silently accepted, because "bumpless transfer" that quietly steps the
 * actuator is worse than one that tells you it could not.
 *
 * @return true if the exact value was representable.
 */
static bool pidp_back_solve(PID_Handle *h, PID_Float desired,
                            PID_Float p, PID_Float d, PID_Float ff)
{
    PID_Float lo;
    PID_Float hi;
    PID_Float want;
    PID_Float got;

    pidp_integral_bounds(h, &lo, &hi);
    want = desired - p - d - ff;
    got  = pidm_clamp(want, lo, hi);
    h->integrator = got;

    if (got != want) {
        h->flags |= PID_FLAG_INTEGRAL_LIMITED;
        /*
         * Also record it as a sticky error. PID_FLAG_INTEGRAL_LIMITED is a
         * transient flag, rebuilt at the top of every update, so a caller who
         * switches mode and then reads the flags after the next PID_Update
         * would never see it - the very moment it mattered has already been
         * overwritten. The sticky last-error channel survives until the
         * application clears it, which is what makes "did my transfer bump?"
         * an answerable question.
         */
        pidp_set_error(h, PID_ERR_INVALID_LIMIT);
        return false;
    }
    return true;
}

/* ======================================================================== */
/* Setpoint shaping                                                          */
/* ======================================================================== */

#if PIDX_ENABLE_SHAPER
/**
 * Advance the effective setpoint towards the commanded target.
 *
 * The profile itself lives in pid_shaper.h so that the built-in shaper and the
 * standalone PID_Shaper are provably the same algorithm; this wrapper only
 * maintains the PID_FLAG_SP_RAMPING status bit.
 */
static void pidp_shape_setpoint(PID_Handle *h, PID_Float dt)
{
    const bool moving = pids_profile_step(&h->setpoint, &h->sp_velocity,
                                          h->setpoint_target, h->sp_rate_max,
                                          h->sp_accel, h->sp_decel, dt);
    if (moving) {
        h->flags |= PID_FLAG_SP_RAMPING;
    } else {
        h->flags &= (uint16_t)(~PID_FLAG_SP_RAMPING);
    }
}
#endif /* PIDX_ENABLE_SHAPER */

/* ======================================================================== */
/* Safety                                                                    */
/* ======================================================================== */

#if PIDX_ENABLE_SAFETY
/**
 * Validate a measurement against range and slew plausibility.
 * @return PID_OK when the sample may be trusted.
 */
static PID_StatusCode pidp_check_sensor(PID_Handle *h, PID_Float y, PID_Float dt)
{
    PID_StatusCode rc = PID_OK;

    if ((h->meas_max > h->meas_min) && ((y < h->meas_min) || (y > h->meas_max))) {
        rc = PID_ERR_SENSOR_RANGE;
    } else if ((h->meas_rate_max > PID_ZERO) && h->meas_prev_valid) {
        const PID_Float delta = pidm_abs(y - h->meas_prev);
        if (delta > (h->meas_rate_max * dt)) {
            rc = PID_ERR_SENSOR_RATE;
        }
    } else {
        /* nothing further to check */
    }
    return rc;
}
#endif /* PIDX_ENABLE_SAFETY */

/* ======================================================================== */
/* The update path                                                           */
/* ======================================================================== */

/**
 * One control cycle. Shared by PID_Update, PID_UpdateDt and PID_UpdateEx.
 *
 * @param in  Optional extended input; NULL for the simple paths.
 */
static PID_Float pidp_run(PID_Handle *h, PID_Float meas, PID_Float dt,
                          const PID_Input *in, PID_StatusCode *err)
{
    PID_Float y = meas;
    PID_Float sp;
    PID_Float e;
    PID_Float p_term;
    PID_Float d_src;
    PID_Float ff = PID_ZERO;
    PID_Float u_raw;
    PID_Float u;
    PID_Float i_pre = PID_ZERO;   /* Integrator before this cycle's update,
                                   * kept so conditional integration can roll
                                   * the update back once saturation is known. */
    bool      i_stepped = false;
#if PIDX_ENABLE_SAFETY
    PID_Float recover_to = PID_ZERO;  /* Deferred fault-recovery back-solve:  */
    bool      recovering = false;     /* needs P/D/FF, computed further down. */
#endif
    PID_Float i_lo;
    PID_Float i_hi;
    PID_Float gamma_eff;
    PID_StatusCode rc = PID_OK;
    bool integrate;

    /* -------- Stage 0: guards -------------------------------------- */
#if PIDX_ENABLE_ARG_CHECKS
    if (!pidp_valid(h)) {
        if (err != NULL) { *err = PID_ERR_NOT_INIT; }
        return PID_ZERO;
    }
    if (!pidm_isfinite(y)) {
        rc = pidm_isnan(y) ? PID_ERR_NAN_INPUT : PID_ERR_INF_INPUT;
        pidp_set_error(h, rc);
        h->flags |= PID_FLAG_SENSOR_INVALID;
#if PIDX_ENABLE_SAFETY
        if ((h->features & PID_FEAT_SAFETY) != 0UL) {
            h->fault_count++;
            if (h->fault_count >= h->fault_persist_n) {
                h->flags |= PID_FLAG_FAULT;
                h->output = h->failsafe_output;
            }
        }
#endif
        if (err != NULL) { *err = rc; }
        /* Hold the previous output: one bad sample must not command a jump. */
        return h->output;
    }
#else
    if (h == NULL) {
        if (err != NULL) { *err = PID_ERR_NULL; }
        return PID_ZERO;
    }
#endif

    /* Transient flags are rebuilt every cycle; FAULT is latched separately. */
    h->flags &= (uint16_t)(PID_FLAG_FAULT | PID_FLAG_TUNING | PID_FLAG_SP_RAMPING);

    /* -------- Stage 1: timing -------------------------------------- */
    if (dt <= PID_ZERO) {
        rc = PID_ERR_INVALID_DT;
        pidp_set_error(h, rc);
        h->flags |= PID_FLAG_DT_VIOLATION;
        dt = h->dt_nominal;
    } else if (((h->dt_min > PID_ZERO) && (dt < h->dt_min)) ||
               ((h->dt_max > PID_ZERO) && (dt > h->dt_max))) {
        rc = PID_ERR_INVALID_DT;
        pidp_set_error(h, rc);
        h->flags |= PID_FLAG_DT_VIOLATION;
        dt = pidm_clamp(dt,
                        (h->dt_min > PID_ZERO) ? h->dt_min : dt,
                        (h->dt_max > PID_ZERO) ? h->dt_max : dt);
    } else {
        /* dt accepted as given */
    }

    if (dt != h->dt_last) {
        pidp_recompute(h, dt);
    }

    /* -------- Stage 2: sensor validation ---------------------------- */
#if PIDX_ENABLE_SAFETY
    if ((h->features & PID_FEAT_SAFETY) != 0UL) {
        const PID_StatusCode sc = pidp_check_sensor(h, y, dt);

        if (sc != PID_OK) {
            pidp_set_error(h, sc);
            h->flags |= PID_FLAG_SENSOR_INVALID;
            h->fault_count++;
            if (h->fault_count >= h->fault_persist_n) {
                h->flags |= PID_FLAG_FAULT;
            }
        } else if (h->fault_count > 0U) {
            if (h->auto_recover) {
                h->fault_count = 0U;
                if ((h->flags & PID_FLAG_FAULT) != 0U) {
                    /*
                     * Bumpless re-entry: the controller must reproduce the
                     * fail-safe output it has been holding, then move away
                     * from there under normal control.
                     *
                     * The back-solve is DEFERRED to stage 10 rather than done
                     * here, because P, D and FF for this sample do not exist
                     * yet. Solving now with zeros sets I = u_failsafe, and
                     * then the real P term is added on top - which produced a
                     * measured 0.20 -> 0.29 step on recovery, exactly the bump
                     * this code exists to prevent.
                     */
                    h->flags &= (uint16_t)(~PID_FLAG_FAULT);
                    h->d_prev_in = y;
                    recover_to = h->output;
                    recovering = true;
                }
            } else {
                h->fault_count = 0U;   /* sample was fine; latch stays put   */
            }
        } else {
            /* healthy, nothing to do */
        }

        h->meas_prev = y;
        h->meas_prev_valid = true;

        if ((h->flags & PID_FLAG_FAULT) != 0U) {
            h->output = h->failsafe_output;
            if (err != NULL) { *err = (rc != PID_OK) ? rc : PID_ERR_SENSOR_RANGE; }
            return h->output;
        }
    }
#endif

    /* -------- Stage 3: input filter --------------------------------- */
#if PIDX_ENABLE_INPUT_FILTER
    if ((h->features & PID_FEAT_INPUT_FILTER) != 0UL) {
        y = pidf_lpf1_step(&h->in_lpf, y);
    }
#endif

    /* -------- Stage 4: setpoint ------------------------------------- */
    if ((in != NULL) && pidm_isfinite(in->setpoint)) {
        h->setpoint_target = in->setpoint;
    }

#if PIDX_ENABLE_SHAPER
    if ((h->features & PID_FEAT_SP_SHAPER) != 0UL) {
        pidp_shape_setpoint(h, dt);
    } else {
        h->setpoint = h->setpoint_target;
    }
#else
    h->setpoint = h->setpoint_target;
#endif
    sp = h->setpoint;

    /* -------- Stage 5: gain scheduling ------------------------------ */
#if PIDX_ENABLE_GAIN_SCHED
    if (((h->features & PID_FEAT_GAIN_SCHED) != 0UL) && (h->sched != NULL)) {
        PID_Float var;
        PID_Float nkp;
        PID_Float nki;
        PID_Float nkd;

        if ((in != NULL) && pidm_isfinite(in->schedule_var)) {
            var = in->schedule_var;
        } else {
            switch ((PID_SchedSource)h->sched->source) {
            case PID_SCHED_SRC_SETPOINT:    var = sp;                     break;
            case PID_SCHED_SRC_MEASUREMENT: var = y;                      break;
            case PID_SCHED_SRC_ERROR:       var = sp - y;                 break;
            case PID_SCHED_SRC_ABS_ERROR:   var = pidm_abs(sp - y);       break;
            case PID_SCHED_SRC_OUTPUT:      var = h->output;              break;
            case PID_SCHED_SRC_EXTERNAL:
            default:                        var = h->sched_var_ext;       break;
            }
        }

        if (PID_GainSched_Evaluate(h->sched, var,
                                   &nkp, &nki, &nkd) == PID_OK) {
            if ((nkp != h->kp) || (nki != h->ki) || (nkd != h->kd)) {
                h->kp = nkp;
                h->ki = nki;
                h->kd = nkd;
                pidp_recompute(h, dt);
            }
        }
    }
#endif

    /* -------- Stage 6: error and P ---------------------------------- */
    {
        const PID_Float dsign = (PID_Float)h->dir_sign;
        e = dsign * (sp - y);
        p_term = h->kp * dsign * ((h->beta * sp) - y);
    }

    /* -------- Stage 7: derivative ----------------------------------- */
    /*
     * All three derivative modes are the same expression with a different
     * setpoint weight, so there is one code path instead of three:
     *   x = dir * (y - gamma_eff * r)   and   D = -Kd/(Tf+dt) * dx  filtered.
     *   gamma_eff = 0 -> on measurement, 1 -> on error, gamma -> 2DOF.
     */
    switch ((PID_DerivativeMode)h->d_mode) {
    case PID_DERIV_ON_ERROR:            gamma_eff = PID_ONE;   break;
    case PID_DERIV_ON_WEIGHTED_ERROR:   gamma_eff = h->gamma;  break;
    case PID_DERIV_ON_MEASUREMENT:
    default:                            gamma_eff = PID_ZERO;  break;
    }
    d_src = (PID_Float)h->dir_sign * (y - (gamma_eff * sp));

    if ((h->features & PID_FEAT_DERIVATIVE) != 0UL) {
        h->d_state = (h->c_da * h->d_state) - (h->c_db * (d_src - h->d_prev_in));
    } else {
        h->d_state = PID_ZERO;
    }
    h->d_prev_in = d_src;

    /* -------- Stage 8: feedforward ---------------------------------- */
#if PIDX_ENABLE_FEEDFORWARD
    if ((h->features & PID_FEAT_FEEDFORWARD) != 0UL) {
        if ((in != NULL) && pidm_isfinite(in->feedforward)) {
            ff = in->feedforward * h->ff_gain;
        } else if (h->ff_fn != NULL) {
            ff = h->ff_fn(sp, y, h->ff_ctx) * h->ff_gain;
        } else {
            ff = h->ff_value * h->ff_gain;
        }
        if (!pidm_isfinite(ff)) {
            /* A misbehaving user callback must not poison the controller. */
            ff = PID_ZERO;
            pidp_set_error(h, PID_ERR_NAN_INPUT);
        }
    }
#else
    PIDX_UNUSED(in);
#endif

    pidp_integral_bounds(h, &i_lo, &i_hi);

#if PIDX_ENABLE_SAFETY
    /* Deferred from stage 2: now that p_term, d_state and ff are known for
     * this sample, the integrator can be solved so that the sum reproduces
     * the fail-safe output exactly. */
    if (recovering) {
        (void)pidp_back_solve(h, recover_to, p_term, h->d_state, ff);
    }
#endif

    /* -------- Stage 9: manual / hold -------------------------------- */
    if (h->mode == (uint8_t)PID_MODE_MANUAL) {
        u = h->manual_output;
        if ((h->features & PID_FEAT_OUTPUT_LIMIT) != 0UL) {
            u = pidm_clamp(u, h->out_min, h->out_max);
        }
        /* Track continuously so that a switch to AUTOMATIC at any instant is
         * bumpless without needing a special case in PID_SetMode(). */
        (void)pidp_back_solve(h, u, p_term, h->d_state, ff);
        h->output = u;
        h->flags |= PID_FLAG_MANUAL;
        h->e_prev = e;
#if PIDX_ENABLE_DIAGNOSTICS
        goto diag;
#else
        if (err != NULL) { *err = rc; }
        return u;
#endif
    }

    /* -------- Stage 10: integral ------------------------------------ */
    integrate = ((h->features & PID_FEAT_INTEGRAL) != 0UL) &&
                (h->mode != (uint8_t)PID_MODE_HOLD);

    if (integrate) {
        const PID_Float ae = pidm_abs(e);

        /* Integral separation: during a large excursion the integrator would
         * charge far beyond what the steady state needs, guaranteeing
         * overshoot. P and D handle the transient; I re-engages near target. */
        if ((h->i_separation > PID_ZERO) && (ae > h->i_separation)) {
            integrate = false;
        } else if ((h->i_deadband > PID_ZERO) && (ae < h->i_deadband)) {
            /* Deadband: stop hunting against a quantised actuator. */
            integrate = false;
        } else {
            /* Conditional integration is NOT decided here. Whether this
             * sample's accumulation is admissible depends on whether the
             * output it produces saturates, which is only known after the
             * sum in stage 11. Testing the saturation flags at this point
             * would test the PREVIOUS cycle's state - and since the flags are
             * rebuilt at the top of every update, they are always clear here.
             * The decision is made, and undone if necessary, in stage 13. */
        }
    }

    if (integrate) {
        i_pre = h->integrator;
        if (h->integ_method == (uint8_t)PID_INTEGRATION_TRAPEZOIDAL) {
            h->integrator += h->c_i * (e + h->e_prev);
        } else {
            h->integrator += h->c_i * e;
        }
        i_stepped = true;
        h->flags |= PID_FLAG_INTEGRAL_ACTIVE;
    }

    if (h->aw_mode == (uint8_t)PID_AW_CLAMP) {
        const PID_Float clamped = pidm_clamp(h->integrator, i_lo, i_hi);
        if (clamped != h->integrator) {
            h->integrator = clamped;
            h->flags |= PID_FLAG_INTEGRAL_LIMITED;
        }
    }

    h->e_prev = e;

    /* -------- Stage 11: sum ----------------------------------------- */
    u_raw = p_term + h->integrator + h->d_state + ff;

    /* -------- Stage 12: output saturation --------------------------- */
    u = u_raw;
    if ((h->features & PID_FEAT_OUTPUT_LIMIT) != 0UL) {
        if (u > h->out_max) {
            u = h->out_max;
            h->flags |= PID_FLAG_SATURATED_HIGH;
        } else if (u < h->out_min) {
            u = h->out_min;
            h->flags |= PID_FLAG_SATURATED_LOW;
        } else {
            /* inside range */
        }
    }

    /* -------- Stage 13: back-calculation / tracking ------------------ */
    /*
     * Applied in the SAME sample as the saturation it corrects. Deferring it
     * to the next cycle inserts a one-sample delay into the anti-windup loop,
     * which shows up as extra overshoot on recovery.
     */
    if (h->mode != (uint8_t)PID_MODE_HOLD) {
        if (h->aw_mode == (uint8_t)PID_AW_BACK_CALCULATION) {
            if (u != u_raw) {
                h->integrator += h->c_aw * (u - u_raw);
                h->integrator = pidm_clamp(h->integrator, i_lo, i_hi);
            }
        } else if (h->aw_mode == (uint8_t)PID_AW_CONDITIONAL) {
            /*
             * Conditional integration (integrator "clamping" in the Astrom
             * sense): an increment is admissible unless the output saturates
             * AND the error would drive it further past the same limit.
             *
             * The test uses u_raw - the unsaturated sum - because that is what
             * says how far past the limit the controller is asking to go. The
             * increment is then undone rather than merely skipped, so the
             * decision is made with this sample's saturation state instead of
             * the previous one; a one-cycle-late test is the classic way this
             * strategy quietly degrades into no protection at all.
             */
            if (i_stepped &&
                (((u_raw > h->out_max) && (e > PID_ZERO)) ||
                 ((u_raw < h->out_min) && (e < PID_ZERO)))) {
                h->integrator = i_pre;
                h->flags &= (uint16_t)(~PID_FLAG_INTEGRAL_ACTIVE);
                h->flags |= PID_FLAG_INTEGRAL_LIMITED;

                /* Recompute: removing the increment may pull the output back
                 * inside the limits, and holding it at the limit anyway would
                 * throw away authority the controller actually has. */
                u_raw = p_term + h->integrator + h->d_state + ff;
                u = u_raw;
                h->flags &= (uint16_t)(~PID_FLAG_SATURATED);
                if ((h->features & PID_FEAT_OUTPUT_LIMIT) != 0UL) {
                    if (u > h->out_max) {
                        u = h->out_max;
                        h->flags |= PID_FLAG_SATURATED_HIGH;
                    } else if (u < h->out_min) {
                        u = h->out_min;
                        h->flags |= PID_FLAG_SATURATED_LOW;
                    } else {
                        /* inside range */
                    }
                }
            }
        } else if (h->aw_mode == (uint8_t)PID_AW_TRACKING) {
            PID_Float track = h->tracking_input;
            if ((in != NULL) && pidm_isfinite(in->tracking)) {
                track = in->tracking;
            }
            if (pidm_isfinite(track)) {
                h->integrator += h->c_aw * (track - u_raw);
                h->integrator = pidm_clamp(h->integrator, i_lo, i_hi);
            }
        } else {
            /* NONE / CLAMP need nothing here */
        }
    }

    /* -------- Stage 14: output slew --------------------------------- */
#if PIDX_ENABLE_SHAPER
    if (((h->features & PID_FEAT_OUT_SHAPER) != 0UL) && (h->out_slew_max > PID_ZERO)) {
        const PID_Float max_step = h->out_slew_max * dt;
        const PID_Float delta = u - h->output;
        if (delta > max_step) {
            u = h->output + max_step;
            h->flags |= PID_FLAG_OUTPUT_SLEWING;
        } else if (delta < -max_step) {
            u = h->output - max_step;
            h->flags |= PID_FLAG_OUTPUT_SLEWING;
        } else {
            /* within slew budget */
        }
    }
#endif

    /* Final numeric guard. If anything went non-finite despite the checks
     * above - a pathological gain, a denormal cascade - fall back to the last
     * good output and reset the states that could have produced it, rather
     * than propagating NaN into an actuator. */
    if (!pidm_isfinite(u)) {
        pidp_set_error(h, PID_ERR_NAN_INPUT);
        h->integrator = PID_ZERO;
        h->d_state = PID_ZERO;
        u = pidm_isfinite(h->output) ? h->output : PID_ZERO;
    }

    h->output = u;

#if PIDX_ENABLE_DIAGNOSTICS
diag:
    if ((h->features & PID_FEAT_DIAGNOSTICS) != 0UL) {
        PID_Status *s = &h->status;
        s->setpoint_raw = h->setpoint_target;
        s->setpoint_shaped = sp;
        s->measurement_raw = meas;
        s->measurement_filtered = y;
        s->error = e;
        s->p_term = p_term;
        s->i_term = h->integrator;
        s->d_term = h->d_state;
        s->ff_term = ff;
        s->output_unsat = (h->mode == (uint8_t)PID_MODE_MANUAL) ? h->output : u_raw;
        s->output = h->output;
        s->dt_used = dt;
        s->kp_active = h->kp;
        s->ki_active = h->ki;
        s->kd_active = h->kd;
        s->update_count++;
        if ((h->flags & PID_FLAG_SATURATED) != 0U) {
            s->saturation_count++;
        }
        s->flags = h->flags;
        s->last_error = h->last_error;

#if PIDX_ENABLE_TELEMETRY
        if (((h->features & PID_FEAT_TELEMETRY) != 0UL) && (h->telemetry != NULL)) {
            pidd_telemetry_push(h->telemetry, s);
        }
#endif
    }
#endif /* PIDX_ENABLE_DIAGNOSTICS */

    if (err != NULL) { *err = rc; }
    return h->output;
}

/* ======================================================================== */
/* LEVEL 1 - BASIC                                                           */
/* ======================================================================== */

PID_StatusCode PID_ConfigDefault(PID_Config *cfg)
{
    if (cfg == NULL) {
        return PID_ERR_NULL;
    }

    /* Zeroing first guarantees that fields added in a future minor version
     * default to "off" rather than to whatever was on the stack. */
    {
        uint8_t *p = (uint8_t *)cfg;
        size_t i;
        for (i = 0U; i < sizeof(PID_Config); ++i) {
            p[i] = 0U;
        }
    }

    cfg->abi_version = (uint16_t)PIDX_CONFIG_ABI_VERSION;

    cfg->core.kp = PID_ZERO;
    cfg->core.ki = PID_ZERO;
    cfg->core.kd = PID_ZERO;
    cfg->core.sample_time = (PID_Float)PIDX_DEFAULT_SAMPLE_TIME;
    cfg->core.direction = PID_DIRECT;
    cfg->core.mode = PID_MODE_AUTOMATIC;
    cfg->core.integration = PID_INTEGRATION_BACKWARD_EULER;

    cfg->limits.use_output_limits = false;
    cfg->limits.output_min = -PID_HUGE_F;
    cfg->limits.output_max = PID_HUGE_F;
    cfg->limits.use_integral_limits = false;
    cfg->limits.integral_min = -PID_HUGE_F;
    cfg->limits.integral_max = PID_HUGE_F;
    cfg->limits.dt_min = PID_ZERO;
    cfg->limits.dt_max = PID_ZERO;

    cfg->filter.derivative_mode = PID_DERIV_ON_MEASUREMENT;
    cfg->filter.tf = PID_ZERO;
    cfg->filter.n_filter = (PID_Float)PIDX_DEFAULT_N_FILTER;
    cfg->filter.input_lpf_tau = PID_ZERO;

    cfg->integral.mode = PID_AW_CLAMP;
    cfg->integral.kt = PID_ZERO;
    cfg->integral.separation_threshold = PID_ZERO;
    cfg->integral.deadband = PID_ZERO;
    cfg->integral.enabled = true;

    cfg->weight.beta = PID_ONE;
    cfg->weight.gamma = PID_ZERO;

    cfg->feedforward.enabled = false;
    cfg->feedforward.fn = NULL;
    cfg->feedforward.ctx = NULL;
    cfg->feedforward.value = PID_ZERO;
    cfg->feedforward.gain = PID_ONE;

    cfg->shaper.sp_rate_max = PID_ZERO;
    cfg->shaper.sp_accel = PID_ZERO;
    cfg->shaper.sp_decel = PID_ZERO;
    cfg->shaper.out_slew_max = PID_ZERO;

    cfg->safety.enabled = false;
    cfg->safety.meas_min = PID_ZERO;
    cfg->safety.meas_max = PID_ZERO;
    cfg->safety.meas_rate_max = PID_ZERO;
    cfg->safety.failsafe_output = PID_ZERO;
    cfg->safety.fault_persist_n = 3U;
    cfg->safety.auto_recover = false;

    return PID_OK;
}

/** Validate a gain: finite and non-negative. */
static bool pidp_gain_ok(PID_Float g)
{
    return (pidm_isfinite(g) && (g >= PID_ZERO));
}

PID_StatusCode PID_Init(PID_Handle *h, const PID_Config *cfg)
{
    uint32_t feat;

    if ((h == NULL) || (cfg == NULL)) {
        return PID_ERR_NULL;
    }
    if (cfg->abi_version != (uint16_t)PIDX_CONFIG_ABI_VERSION) {
        return PID_ERR_INVALID_CONFIG;
    }
    if (!pidp_gain_ok(cfg->core.kp) || !pidp_gain_ok(cfg->core.ki) ||
        !pidp_gain_ok(cfg->core.kd)) {
        return PID_ERR_INVALID_GAIN;
    }
    if (!pidm_isfinite(cfg->core.sample_time) || (cfg->core.sample_time <= PID_ZERO)) {
        return PID_ERR_INVALID_DT;
    }
    if (cfg->limits.use_output_limits &&
        (!pidm_isfinite(cfg->limits.output_min) ||
         !pidm_isfinite(cfg->limits.output_max) ||
         (cfg->limits.output_min >= cfg->limits.output_max))) {
        return PID_ERR_INVALID_LIMIT;
    }
    if (cfg->limits.use_integral_limits &&
        (cfg->limits.integral_min >= cfg->limits.integral_max)) {
        return PID_ERR_INVALID_LIMIT;
    }
    if ((cfg->integral.mode == PID_AW_BACK_CALCULATION) &&
        !cfg->limits.use_output_limits && !cfg->limits.use_integral_limits) {
        /* u_sat would always equal u_raw, so the correction term is identically
         * zero: the user almost certainly forgot to set limits. */
        return PID_ERR_INVALID_LIMIT;
    }
    if (!pidm_isfinite(cfg->weight.beta) || !pidm_isfinite(cfg->weight.gamma) ||
        (cfg->weight.beta < PID_ZERO) || (cfg->weight.beta > PID_TWO) ||
        (cfg->weight.gamma < PID_ZERO) || (cfg->weight.gamma > PID_TWO)) {
        return PID_ERR_INVALID_CONFIG;
    }

#if !PIDX_ENABLE_SHAPER
    if ((cfg->shaper.sp_rate_max > PID_ZERO) || (cfg->shaper.out_slew_max > PID_ZERO)) {
        return PID_ERR_UNSUPPORTED;
    }
#endif
#if !PIDX_ENABLE_FEEDFORWARD
    if (cfg->feedforward.enabled) {
        return PID_ERR_UNSUPPORTED;
    }
#endif
#if !PIDX_ENABLE_SAFETY
    if (cfg->safety.enabled) {
        return PID_ERR_UNSUPPORTED;
    }
#endif
#if !PIDX_ENABLE_INPUT_FILTER
    if (cfg->filter.input_lpf_tau > PID_ZERO) {
        return PID_ERR_UNSUPPORTED;
    }
#endif

    /* Zero the whole handle so that every conditional field starts defined. */
    {
        uint8_t *p = (uint8_t *)h;
        size_t i;
        for (i = 0U; i < sizeof(PID_Handle); ++i) {
            p[i] = 0U;
        }
    }

    h->kp = cfg->core.kp;
    h->ki = cfg->core.ki;
    h->kd = cfg->core.kd;
    h->dt_nominal = cfg->core.sample_time;
    h->dt_min = cfg->limits.dt_min;
    h->dt_max = cfg->limits.dt_max;
    h->dir_sign = (cfg->core.direction == PID_REVERSE) ? (int8_t)-1 : (int8_t)1;
    h->mode = (uint8_t)cfg->core.mode;
    h->integ_method = (uint8_t)cfg->core.integration;

    h->out_min = cfg->limits.output_min;
    h->out_max = cfg->limits.output_max;
    /*
     * i_min/i_max always hold the EFFECTIVE bounds, already resolved. When the
     * user does not set explicit integral limits they inherit the output
     * limits, which is the sane default: an integrator that can demand more
     * than the actuator can deliver is just windup waiting to happen.
     *
     * Resolving once here rather than per-cycle matters for more than speed.
     * PID_UpdateFast() clamps against these fields directly and cannot afford
     * the branch, so if they held the raw config the two update paths would
     * silently disagree about the integrator bound whenever the user relied
     * on inheritance - and PID_UpdateFast_IsSafe() would have to refuse an
     * otherwise perfectly ordinary PI configuration.
     */
    if (cfg->limits.use_integral_limits) {
        h->i_min = cfg->limits.integral_min;
        h->i_max = cfg->limits.integral_max;
    } else if (cfg->limits.use_output_limits) {
        h->i_min = cfg->limits.output_min;
        h->i_max = cfg->limits.output_max;
    } else {
        h->i_min = -PID_HUGE_F;
        h->i_max = PID_HUGE_F;
    }

    h->d_mode = (uint8_t)cfg->filter.derivative_mode;
    h->tf = cfg->filter.tf;
    h->n_filter = cfg->filter.n_filter;

    h->aw_mode = (uint8_t)cfg->integral.mode;
    h->kt = cfg->integral.kt;
    h->i_separation = cfg->integral.separation_threshold;
    h->i_deadband = cfg->integral.deadband;

    h->beta = cfg->weight.beta;
    h->gamma = cfg->weight.gamma;

    /* Build the runtime feature mask from the configuration. */
    feat = PID_FEAT_DERIVATIVE | PID_FEAT_D_FILTER;
    if (cfg->integral.enabled)          { feat |= PID_FEAT_INTEGRAL; }
    if (cfg->limits.use_output_limits)  { feat |= PID_FEAT_OUTPUT_LIMIT; }
    if (cfg->limits.use_integral_limits){ feat |= PID_FEAT_INTEGRAL_LIMIT; }

#if PIDX_ENABLE_FEEDFORWARD
    if (cfg->feedforward.enabled) {
        feat |= PID_FEAT_FEEDFORWARD;
    }
    h->ff_fn = cfg->feedforward.fn;
    h->ff_ctx = cfg->feedforward.ctx;
    h->ff_value = cfg->feedforward.value;
    h->ff_gain = (cfg->feedforward.gain != PID_ZERO) ? cfg->feedforward.gain : PID_ONE;
#endif

#if PIDX_ENABLE_SHAPER
    h->sp_rate_max = cfg->shaper.sp_rate_max;
    h->sp_accel = cfg->shaper.sp_accel;
    h->sp_decel = cfg->shaper.sp_decel;
    h->out_slew_max = cfg->shaper.out_slew_max;
    if (h->sp_rate_max > PID_ZERO)   { feat |= PID_FEAT_SP_SHAPER; }
    if (h->out_slew_max > PID_ZERO)  { feat |= PID_FEAT_OUT_SHAPER; }
#endif

#if PIDX_ENABLE_INPUT_FILTER
    h->in_lpf.tau = cfg->filter.input_lpf_tau;
    if (h->in_lpf.tau > PID_ZERO) { feat |= PID_FEAT_INPUT_FILTER; }
#endif

#if PIDX_ENABLE_SAFETY
    h->meas_min = cfg->safety.meas_min;
    h->meas_max = cfg->safety.meas_max;
    h->meas_rate_max = cfg->safety.meas_rate_max;
    h->failsafe_output = cfg->safety.failsafe_output;
    h->fault_persist_n = (cfg->safety.fault_persist_n == 0U) ?
                         1U : cfg->safety.fault_persist_n;
    h->auto_recover = cfg->safety.auto_recover;
    if (cfg->safety.enabled) { feat |= PID_FEAT_SAFETY; }
#endif

#if PIDX_ENABLE_DIAGNOSTICS
    feat |= PID_FEAT_DIAGNOSTICS;
#endif

    h->features = feat;
    h->tracking_input = PID_ZERO;
    h->last_error = PID_OK;
    h->init_magic = PID_INIT_MAGIC;

    pidp_recompute(h, h->dt_nominal);
    return PID_OK;
}

PID_StatusCode PID_InitDefault(PID_Handle *h)
{
    PID_Config cfg;
    PID_StatusCode rc;

    if (h == NULL) {
        return PID_ERR_NULL;
    }
    rc = PID_ConfigDefault(&cfg);
    if (rc != PID_OK) {
        return rc;
    }
    return PID_Init(h, &cfg);
}

PID_StatusCode PID_Deinit(PID_Handle *h)
{
    uint8_t *p;
    size_t i;

    if (h == NULL) {
        return PID_ERR_NULL;
    }
    p = (uint8_t *)h;
    for (i = 0U; i < sizeof(PID_Handle); ++i) {
        p[i] = 0U;
    }
    /* init_magic is now 0, so any later update reports PID_ERR_NOT_INIT. */
    return PID_OK;
}

PID_StatusCode PID_SetGains(PID_Handle *h, PID_Float kp, PID_Float ki, PID_Float kd)
{
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (!pidp_gain_ok(kp) || !pidp_gain_ok(ki) || !pidp_gain_ok(kd)) {
        return PID_ERR_INVALID_GAIN;
    }
    h->kp = kp;
    h->ki = ki;
    h->kd = kd;
    pidp_recompute(h, h->dt_last);
    return PID_OK;
}

PID_StatusCode PID_SetGainsRescaleIntegral(PID_Handle *h, PID_Float kp,
                                           PID_Float ki, PID_Float kd)
{
    PID_Float old_ki;

    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (!pidp_gain_ok(kp) || !pidp_gain_ok(ki) || !pidp_gain_ok(kd)) {
        return PID_ERR_INVALID_GAIN;
    }

    old_ki = h->ki;
    if (old_ki > PID_ZERO) {
        /* Preserve integral(e): term_new = Ki_new * integral(e)
         *                              = term_old * (Ki_new / Ki_old). */
        h->integrator = h->integrator * (ki / old_ki);
    }
    h->kp = kp;
    h->ki = ki;
    h->kd = kd;
    pidp_recompute(h, h->dt_last);
    return PID_OK;
}

PID_StatusCode PID_SetKp(PID_Handle *h, PID_Float kp)
{
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (!pidp_gain_ok(kp)) { return PID_ERR_INVALID_GAIN; }
    h->kp = kp;
    pidp_recompute(h, h->dt_last);
    return PID_OK;
}

PID_StatusCode PID_SetKi(PID_Handle *h, PID_Float ki)
{
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (!pidp_gain_ok(ki)) { return PID_ERR_INVALID_GAIN; }
    h->ki = ki;
    pidp_recompute(h, h->dt_last);
    return PID_OK;
}

PID_StatusCode PID_SetKd(PID_Handle *h, PID_Float kd)
{
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (!pidp_gain_ok(kd)) { return PID_ERR_INVALID_GAIN; }
    h->kd = kd;
    pidp_recompute(h, h->dt_last);
    return PID_OK;
}

PID_StatusCode PID_GetGains(const PID_Handle *h, PID_Float *kp, PID_Float *ki, PID_Float *kd)
{
    if (h == NULL) { return PID_ERR_NULL; }
    /* Validated, like PID_GetStatus(). Without this the function reports
     * PID_OK while copying whatever the caller's stack held into the output
     * pointers, which is indistinguishable from a real tuning. */
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (kp != NULL) { *kp = h->kp; }
    if (ki != NULL) { *ki = h->ki; }
    if (kd != NULL) { *kd = h->kd; }
    return PID_OK;
}

PID_StatusCode PID_SetSetpoint(PID_Handle *h, PID_Float setpoint)
{
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (!pidm_isfinite(setpoint)) { return PID_ERR_INVALID_PARAM; }
    h->setpoint_target = setpoint;
#if PIDX_ENABLE_SHAPER
    if ((h->features & PID_FEAT_SP_SHAPER) == 0UL) {
        h->setpoint = setpoint;
    }
#else
    h->setpoint = setpoint;
#endif
    return PID_OK;
}

PID_Float PID_GetSetpoint(const PID_Handle *h)
{
    return (h != NULL) ? h->setpoint : PID_ZERO;
}

PID_Float PID_Update(PID_Handle *h, PID_Float measurement)
{
    if (h == NULL) { return PID_ZERO; }
    return pidp_run(h, measurement, h->dt_nominal, NULL, NULL);
}

PID_Float PID_UpdateDt(PID_Handle *h, PID_Float measurement, PID_Float dt)
{
    if (h == NULL) { return PID_ZERO; }
    return pidp_run(h, measurement, dt, NULL, NULL);
}

PID_Float PID_UpdateEx(PID_Handle *h, const PID_Input *in, PID_StatusCode *err)
{
    PID_Float dt;

    if (h == NULL) {
        if (err != NULL) { *err = PID_ERR_NULL; }
        return PID_ZERO;
    }
    if (in == NULL) {
        if (err != NULL) { *err = PID_ERR_NULL; }
        return h->output;
    }
    dt = (pidm_isfinite(in->dt) && (in->dt > PID_ZERO)) ? in->dt : h->dt_nominal;
    return pidp_run(h, in->measurement, dt, in, err);
}

void PID_InputInit(PID_Input *in)
{
    if (in != NULL) {
        /* A NaN in every optional field is the "leave unchanged" sentinel. */
        const PID_Float nan_v = PID_ZERO / PID_ZERO;
        in->measurement = PID_ZERO;
        in->setpoint = nan_v;
        in->feedforward = nan_v;
        in->dt = nan_v;
        in->tracking = nan_v;
        in->schedule_var = nan_v;
    }
}

PID_Float PID_UpdateFast(PID_Handle *h, PID_Float measurement)
{
    PID_Float e;
    PID_Float p;
    PID_Float u;

    /*
     * The one check that is kept. Everything else the full path does is
     * skipped here by contract, but a NULL dereference is not a "cost" the
     * caller can trade away - it is a crash in an ISR. Measured price on the
     * host: 13 bytes of .text at -Os and no detectable time (6.11 vs 6.10
     * ns/call over 20M iterations, inside run-to-run noise), because the
     * branch is perfectly predicted. Note this is NOT a validity check:
     * an initialised handle is still the caller's responsibility, which is
     * what PID_UpdateFast_IsSafe() is for.
     */
    if (h == NULL) {
        return PID_ZERO;
    }

    e = (PID_Float)h->dir_sign * (h->setpoint - measurement);
    p = h->kp * (PID_Float)h->dir_sign * ((h->beta * h->setpoint) - measurement);

    {
        const PID_Float x = (PID_Float)h->dir_sign * measurement;
        h->d_state = (h->c_da * h->d_state) - (h->c_db * (x - h->d_prev_in));
        h->d_prev_in = x;
    }

    h->integrator += h->c_i * e;
    h->integrator = pidm_clamp(h->integrator, h->i_min, h->i_max);

    u = pidm_clamp(p + h->integrator + h->d_state, h->out_min, h->out_max);
    h->output = u;
    return u;
}

bool PID_UpdateFast_IsSafe(const PID_Handle *h)
{
    if (!pidp_valid(h)) {
        return false;
    }
    /*
     * The fast path clamps unconditionally against out_min/out_max and
     * i_min/i_max, so output limits must be in force. Explicit INTEGRAL_LIMIT
     * is NOT required: i_min/i_max always hold the effective bounds, inherited
     * from the output limits when the user did not set their own.
     *
     * Everything else in this list is a feature the fast path does not
     * implement. Diagnostics is excluded from the advanced mask because the
     * fast path simply does not populate the snapshot - it produces the same
     * OUTPUT, which is what "safe" means here.
     */
    return (((h->features & PID_FEAT_ADVANCED_MASK & ~PID_FEAT_DIAGNOSTICS) == 0UL) &&
            ((h->features & PID_FEAT_OUTPUT_LIMIT) != 0UL) &&
            ((h->features & PID_FEAT_INTEGRAL) != 0UL) &&
            (h->aw_mode == (uint8_t)PID_AW_CLAMP) &&
            (h->integ_method == (uint8_t)PID_INTEGRATION_BACKWARD_EULER) &&
            (h->d_mode == (uint8_t)PID_DERIV_ON_MEASUREMENT) &&
            (h->mode == (uint8_t)PID_MODE_AUTOMATIC) &&
            (h->i_separation <= PID_ZERO) &&
            (h->i_deadband <= PID_ZERO));
}

PID_StatusCode PID_Reset(PID_Handle *h)
{
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }

    h->integrator = PID_ZERO;
    h->d_state = PID_ZERO;
    h->d_prev_in = PID_ZERO;
    h->e_prev = PID_ZERO;
    h->output = PID_ZERO;
    h->flags = 0U;
    h->last_error = PID_OK;

#if PIDX_ENABLE_SHAPER
    h->sp_velocity = PID_ZERO;
    h->setpoint = h->setpoint_target;
#endif
#if PIDX_ENABLE_INPUT_FILTER
    (void)PID_LPF1_Reset(&h->in_lpf);
#endif
#if PIDX_ENABLE_SAFETY
    h->fault_count = 0U;
    h->meas_prev = PID_ZERO;
    h->meas_prev_valid = false;
#endif
#if PIDX_ENABLE_DIAGNOSTICS
    {
        uint8_t *p = (uint8_t *)&h->status;
        size_t i;
        for (i = 0U; i < sizeof(PID_Status); ++i) {
            p[i] = 0U;
        }
    }
#endif
    return PID_OK;
}

PID_Float PID_GetOutput(const PID_Handle *h)
{
    return (h != NULL) ? h->output : PID_ZERO;
}

PID_Float PID_GetManualOutput(const PID_Handle *h)
{
    return (h != NULL) ? h->manual_output : PID_ZERO;
}

/* ======================================================================== */
/* LEVEL 2 - INTERMEDIATE                                                    */
/* ======================================================================== */

PID_StatusCode PID_SetSampleTime(PID_Handle *h, PID_Float dt)
{
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (!pidm_isfinite(dt) || (dt <= PID_ZERO)) { return PID_ERR_INVALID_DT; }
    h->dt_nominal = dt;
    pidp_recompute(h, dt);
    return PID_OK;
}

PID_Float PID_GetSampleTime(const PID_Handle *h)
{
    /*
     * Validated, not just NULL-checked. This getter is the only evidence
     * PID_Cascade_Init() has that a member loop was ever initialised, and
     * reading dt_nominal out of an uninitialised handle returns whatever the
     * caller's stack happened to hold - which is frequently non-zero, so the
     * cascade's PID_ERR_NOT_INIT guard would never fire and the chain would be
     * built around a garbage loop. Returning 0 for an invalid handle keeps the
     * documented contract ("0 if unusable") honest.
     */
    return pidp_valid(h) ? h->dt_nominal : PID_ZERO;
}

PID_StatusCode PID_SetOutputLimits(PID_Handle *h, PID_Float min, PID_Float max)
{
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (!pidm_isfinite(min) || !pidm_isfinite(max) || (min >= max)) {
        return PID_ERR_INVALID_LIMIT;
    }
    h->out_min = min;
    h->out_max = max;
    h->features |= PID_FEAT_OUTPUT_LIMIT;

    /* Keep existing state consistent with the new envelope. Without explicit
     * integral limits the integrator inherits these, so the effective bounds
     * must be re-resolved here too. */
    h->output = pidm_clamp(h->output, min, max);
    if ((h->features & PID_FEAT_INTEGRAL_LIMIT) == 0UL) {
        h->i_min = min;
        h->i_max = max;
        h->integrator = pidm_clamp(h->integrator, min, max);
    }
    return PID_OK;
}

PID_StatusCode PID_ClearOutputLimits(PID_Handle *h)
{
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    h->features &= ~(uint32_t)PID_FEAT_OUTPUT_LIMIT;
    h->out_min = -PID_HUGE_F;
    h->out_max = PID_HUGE_F;
    /* An inherited integral bound has nothing left to inherit from. */
    if ((h->features & PID_FEAT_INTEGRAL_LIMIT) == 0UL) {
        h->i_min = -PID_HUGE_F;
        h->i_max = PID_HUGE_F;
    }
    return PID_OK;
}

PID_StatusCode PID_SetIntegralLimits(PID_Handle *h, PID_Float min, PID_Float max)
{
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (!pidm_isfinite(min) || !pidm_isfinite(max) || (min >= max)) {
        return PID_ERR_INVALID_LIMIT;
    }
    h->i_min = min;
    h->i_max = max;
    h->features |= PID_FEAT_INTEGRAL_LIMIT;
    h->integrator = pidm_clamp(h->integrator, min, max);
    return PID_OK;
}

PID_StatusCode PID_SetAntiWindup(PID_Handle *h, PID_AntiWindup mode, PID_Float kt)
{
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (mode > PID_AW_TRACKING) { return PID_ERR_INVALID_PARAM; }
    if (!pidm_isfinite(kt) || (kt < PID_ZERO)) { return PID_ERR_INVALID_PARAM; }
    if ((mode == PID_AW_BACK_CALCULATION) &&
        ((h->features & (PID_FEAT_OUTPUT_LIMIT | PID_FEAT_INTEGRAL_LIMIT)) == 0UL)) {
        return PID_ERR_INVALID_LIMIT;
    }
    h->aw_mode = (uint8_t)mode;
    h->kt = kt;
    pidp_recompute(h, h->dt_last);
    return PID_OK;
}

PID_StatusCode PID_SetDerivativeMode(PID_Handle *h, PID_DerivativeMode mode)
{
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (mode > PID_DERIV_ON_WEIGHTED_ERROR) { return PID_ERR_INVALID_PARAM; }
    h->d_mode = (uint8_t)mode;
    /* The derivative source changes meaning; re-prime on the next sample to
     * avoid differentiating across the discontinuity. */
    h->d_prev_in = PID_ZERO;
    h->d_state = PID_ZERO;
    return PID_OK;
}

PID_StatusCode PID_SetDerivativeFilter(PID_Handle *h, PID_Float tf)
{
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (!pidm_isfinite(tf) || (tf < PID_ZERO)) { return PID_ERR_INVALID_PARAM; }
    h->tf = tf;
    if (tf > PID_ZERO) {
        h->features |= PID_FEAT_D_FILTER;
    }
    pidp_recompute(h, h->dt_last);
    return PID_OK;
}

PID_StatusCode PID_SetDerivativeFilterN(PID_Handle *h, PID_Float n)
{
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (!pidm_isfinite(n) || (n <= PID_ZERO)) { return PID_ERR_INVALID_PARAM; }
    h->n_filter = n;
    h->tf = PID_ZERO;              /* explicit tf no longer overrides N */
    h->features |= PID_FEAT_D_FILTER;
    pidp_recompute(h, h->dt_last);
    return PID_OK;
}

PID_StatusCode PID_SetDirection(PID_Handle *h, PID_Direction dir)
{
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    h->dir_sign = (dir == PID_REVERSE) ? (int8_t)-1 : (int8_t)1;
    h->d_prev_in = -h->d_prev_in;  /* keep the stored source consistent */
    return PID_OK;
}

PID_StatusCode PID_SetMode(PID_Handle *h, PID_Mode mode)
{
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (mode > PID_MODE_HOLD) { return PID_ERR_INVALID_MODE; }

    if ((h->mode != (uint8_t)PID_MODE_MANUAL) && (mode == PID_MODE_MANUAL)) {
        /* Entering manual: start from where the controller already is. */
        h->manual_output = h->output;
    }
    /* Leaving manual needs no work: pidp_run() back-solves the integrator on
     * every manual sample, so the automatic law already reproduces h->output. */
    h->mode = (uint8_t)mode;
    return PID_OK;
}

PID_Mode PID_GetMode(const PID_Handle *h)
{
    return (h != NULL) ? (PID_Mode)h->mode : PID_MODE_MANUAL;
}

PID_StatusCode PID_SetManualOutput(PID_Handle *h, PID_Float output)
{
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (!pidm_isfinite(output)) { return PID_ERR_INVALID_PARAM; }
    h->manual_output = output;
    return PID_OK;
}

PID_StatusCode PID_SetSetpointRamp(PID_Handle *h, PID_Float rate_max,
                                   PID_Float accel, PID_Float decel)
{
#if PIDX_ENABLE_SHAPER
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (!pidm_isfinite(rate_max) || (rate_max < PID_ZERO) ||
        !pidm_isfinite(accel) || (accel < PID_ZERO) ||
        !pidm_isfinite(decel) || (decel < PID_ZERO)) {
        return PID_ERR_INVALID_PARAM;
    }
    h->sp_rate_max = rate_max;
    h->sp_accel = accel;
    h->sp_decel = decel;
    if (rate_max > PID_ZERO) {
        h->features |= PID_FEAT_SP_SHAPER;
    } else {
        h->features &= ~(uint32_t)PID_FEAT_SP_SHAPER;
        h->sp_velocity = PID_ZERO;
    }
    return PID_OK;
#else
    PIDX_UNUSED(h); PIDX_UNUSED(rate_max); PIDX_UNUSED(accel); PIDX_UNUSED(decel);
    return PID_ERR_UNSUPPORTED;
#endif
}

PID_StatusCode PID_SetOutputSlewRate(PID_Handle *h, PID_Float slew_max)
{
#if PIDX_ENABLE_SHAPER
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (!pidm_isfinite(slew_max) || (slew_max < PID_ZERO)) {
        return PID_ERR_INVALID_PARAM;
    }
    h->out_slew_max = slew_max;
    if (slew_max > PID_ZERO) {
        h->features |= PID_FEAT_OUT_SHAPER;
    } else {
        h->features &= ~(uint32_t)PID_FEAT_OUT_SHAPER;
    }
    return PID_OK;
#else
    PIDX_UNUSED(h); PIDX_UNUSED(slew_max);
    return PID_ERR_UNSUPPORTED;
#endif
}

PID_StatusCode PID_SetInputFilter(PID_Handle *h, PID_Float tau)
{
#if PIDX_ENABLE_INPUT_FILTER
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (!pidm_isfinite(tau) || (tau < PID_ZERO)) { return PID_ERR_INVALID_PARAM; }
    h->in_lpf.tau = tau;
    if (tau > PID_ZERO) {
        h->features |= PID_FEAT_INPUT_FILTER;
    } else {
        h->features &= ~(uint32_t)PID_FEAT_INPUT_FILTER;
        h->in_lpf.primed = false;
    }
    pidp_recompute(h, h->dt_last);
    return PID_OK;
#else
    PIDX_UNUSED(h); PIDX_UNUSED(tau);
    return PID_ERR_UNSUPPORTED;
#endif
}

/* ======================================================================== */
/* LEVEL 3 - ADVANCED                                                        */
/* ======================================================================== */

PID_StatusCode PID_SetWeights(PID_Handle *h, PID_Float beta, PID_Float gamma)
{
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (!pidm_isfinite(beta) || !pidm_isfinite(gamma) ||
        (beta < PID_ZERO) || (beta > PID_TWO) ||
        (gamma < PID_ZERO) || (gamma > PID_TWO)) {
        return PID_ERR_INVALID_PARAM;
    }
    h->beta = beta;
    h->gamma = gamma;
    return PID_OK;
}

PID_StatusCode PID_SetFeedforward(PID_Handle *h, PID_Float ff)
{
#if PIDX_ENABLE_FEEDFORWARD
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (!pidm_isfinite(ff)) { return PID_ERR_INVALID_PARAM; }
    h->ff_value = ff;
    h->features |= PID_FEAT_FEEDFORWARD;
    return PID_OK;
#else
    PIDX_UNUSED(h); PIDX_UNUSED(ff);
    return PID_ERR_UNSUPPORTED;
#endif
}

PID_StatusCode PID_SetFeedforwardFn(PID_Handle *h, PID_FeedforwardFn fn,
                                    void *ctx, PID_Float gain)
{
#if PIDX_ENABLE_FEEDFORWARD
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (!pidm_isfinite(gain)) { return PID_ERR_INVALID_PARAM; }
    h->ff_fn = fn;
    h->ff_ctx = ctx;
    h->ff_gain = (gain != PID_ZERO) ? gain : PID_ONE;
    if (fn != NULL) {
        h->features |= PID_FEAT_FEEDFORWARD;
    }
    return PID_OK;
#else
    PIDX_UNUSED(h); PIDX_UNUSED(fn); PIDX_UNUSED(ctx); PIDX_UNUSED(gain);
    return PID_ERR_UNSUPPORTED;
#endif
}

PID_StatusCode PID_SetIntegralSeparation(PID_Handle *h, PID_Float threshold)
{
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (!pidm_isfinite(threshold) || (threshold < PID_ZERO)) {
        return PID_ERR_INVALID_PARAM;
    }
    h->i_separation = threshold;
    return PID_OK;
}

PID_StatusCode PID_SetIntegralDeadband(PID_Handle *h, PID_Float db)
{
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (!pidm_isfinite(db) || (db < PID_ZERO)) { return PID_ERR_INVALID_PARAM; }
    h->i_deadband = db;
    return PID_OK;
}

PID_StatusCode PID_EnableIntegral(PID_Handle *h, bool enable)
{
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (enable) {
        h->features |= PID_FEAT_INTEGRAL;
    } else {
        h->features &= ~(uint32_t)PID_FEAT_INTEGRAL;
    }
    return PID_OK;
}

PID_StatusCode PID_SetIntegrator(PID_Handle *h, PID_Float value)
{
    PID_Float lo;
    PID_Float hi;

    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (!pidm_isfinite(value)) { return PID_ERR_INVALID_PARAM; }
    pidp_integral_bounds(h, &lo, &hi);
    h->integrator = pidm_clamp(value, lo, hi);
    return PID_OK;
}

PID_Float PID_GetIntegrator(const PID_Handle *h)
{
    return (h != NULL) ? h->integrator : PID_ZERO;
}

PID_StatusCode PID_SetTrackingInput(PID_Handle *h, PID_Float u_track)
{
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (!pidm_isfinite(u_track)) { return PID_ERR_INVALID_PARAM; }
    h->tracking_input = u_track;
    return PID_OK;
}

PID_StatusCode PID_SetIntegrationMethod(PID_Handle *h, PID_IntegrationMethod m)
{
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (m > PID_INTEGRATION_TRAPEZOIDAL) { return PID_ERR_INVALID_PARAM; }
    h->integ_method = (uint8_t)m;
    pidp_recompute(h, h->dt_last);
    return PID_OK;
}

PID_StatusCode PID_SetSafety(PID_Handle *h, const PID_SafetyConfig *sc)
{
#if PIDX_ENABLE_SAFETY
    if ((h == NULL) || (sc == NULL)) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (!pidm_isfinite(sc->failsafe_output)) { return PID_ERR_INVALID_PARAM; }
    if ((sc->meas_max != sc->meas_min) && (sc->meas_max <= sc->meas_min)) {
        return PID_ERR_INVALID_LIMIT;
    }
    h->meas_min = sc->meas_min;
    h->meas_max = sc->meas_max;
    h->meas_rate_max = sc->meas_rate_max;
    h->failsafe_output = sc->failsafe_output;
    h->fault_persist_n = (sc->fault_persist_n == 0U) ? 1U : sc->fault_persist_n;
    h->auto_recover = sc->auto_recover;
    if (sc->enabled) {
        h->features |= PID_FEAT_SAFETY;
    } else {
        h->features &= ~(uint32_t)PID_FEAT_SAFETY;
    }
    return PID_OK;
#else
    PIDX_UNUSED(h); PIDX_UNUSED(sc);
    return PID_ERR_UNSUPPORTED;
#endif
}

PID_StatusCode PID_SetFaultOutput(PID_Handle *h, PID_Float output)
{
#if PIDX_ENABLE_SAFETY
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if (!pidm_isfinite(output)) { return PID_ERR_INVALID_PARAM; }
    h->failsafe_output = output;
    return PID_OK;
#else
    PIDX_UNUSED(h); PIDX_UNUSED(output);
    return PID_ERR_UNSUPPORTED;
#endif
}

PID_StatusCode PID_ClearFault(PID_Handle *h)
{
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    h->flags &= (uint16_t)(~(PID_FLAG_FAULT | PID_FLAG_SENSOR_INVALID));
#if PIDX_ENABLE_SAFETY
    h->fault_count = 0U;
    h->meas_prev_valid = false;
    /* Re-seed the integrator so control resumes from the fail-safe output
     * rather than from whatever the integrator held before the fault. */
    (void)pidp_back_solve(h, h->output, PID_ZERO, PID_ZERO, PID_ZERO);
#endif
    return PID_OK;
}

bool PID_IsFaulted(const PID_Handle *h)
{
    return (h != NULL) && ((h->flags & PID_FLAG_FAULT) != 0U);
}

/**
 * Bits whose module is present in this build. A request touching any other bit
 * is rejected outright so that a disabled feature can never fail silently.
 */
static uint32_t pidp_supported_features(void)
{
    uint32_t m = PID_FEAT_INTEGRAL | PID_FEAT_DERIVATIVE | PID_FEAT_D_FILTER |
                 PID_FEAT_OUTPUT_LIMIT | PID_FEAT_INTEGRAL_LIMIT;
#if PIDX_ENABLE_FEEDFORWARD
    m |= PID_FEAT_FEEDFORWARD;
#endif
#if PIDX_ENABLE_SHAPER
    m |= PID_FEAT_SP_SHAPER | PID_FEAT_OUT_SHAPER;
#endif
#if PIDX_ENABLE_INPUT_FILTER
    m |= PID_FEAT_INPUT_FILTER;
#endif
#if PIDX_ENABLE_SAFETY
    m |= PID_FEAT_SAFETY;
#endif
#if PIDX_ENABLE_GAIN_SCHED
    m |= PID_FEAT_GAIN_SCHED;
#endif
#if PIDX_ENABLE_DIAGNOSTICS
    m |= PID_FEAT_DIAGNOSTICS;
#endif
#if PIDX_ENABLE_TELEMETRY
    m |= PID_FEAT_TELEMETRY;
#endif
    return m;
}

PID_StatusCode PID_EnableFeature(PID_Handle *h, uint32_t mask, bool enable)
{
    if (h == NULL) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    if ((mask & ~pidp_supported_features()) != 0UL) {
        return PID_ERR_UNSUPPORTED;
    }
    if (enable) {
        h->features |= mask;
    } else {
        h->features &= ~mask;
    }
    return PID_OK;
}

bool PID_IsFeatureEnabled(const PID_Handle *h, uint32_t mask)
{
    return (h != NULL) && ((h->features & mask) == mask);
}

uint16_t PID_GetFlags(const PID_Handle *h)
{
    return (h != NULL) ? h->flags : 0U;
}

bool PID_IsSaturated(const PID_Handle *h)
{
    return (h != NULL) && ((h->flags & PID_FLAG_SATURATED) != 0U);
}

PID_Float PID_GetError(const PID_Handle *h)
{
    if (h == NULL) { return PID_ZERO; }
#if PIDX_ENABLE_DIAGNOSTICS
    return h->status.error;
#else
    return h->e_prev;
#endif
}

PID_StatusCode PID_GetLastError(PID_Handle *h, PID_StatusCode *code)
{
    if (h == NULL) { return PID_ERR_NULL; }
    if (code != NULL) { *code = h->last_error; }
    h->last_error = PID_OK;
    return PID_OK;
}

PID_StatusCode PID_PeekLastError(const PID_Handle *h)
{
    return (h != NULL) ? h->last_error : PID_ERR_NULL;
}

PID_StatusCode PID_ClearError(PID_Handle *h)
{
    if (h == NULL) { return PID_ERR_NULL; }
    h->last_error = PID_OK;
    h->flags &= (uint16_t)(~(PID_FLAG_DT_VIOLATION | PID_FLAG_SENSOR_INVALID));
    return PID_OK;
}

#if PIDX_ENABLE_DIAGNOSTICS
PID_StatusCode PID_GetStatus(const PID_Handle *h, PID_Status *out)
{
    if ((h == NULL) || (out == NULL)) { return PID_ERR_NULL; }
    if (!pidp_valid(h)) { return PID_ERR_NOT_INIT; }
    *out = h->status;
    return PID_OK;
}
#endif

const char *PID_StatusToString(PID_StatusCode code)
{
    const char *s;

    switch (code) {
    case PID_OK:                      s = "OK";                          break;
    case PID_ERR_NULL:                s = "NULL pointer";                break;
    case PID_ERR_NOT_INIT:            s = "handle not initialised";      break;
    case PID_ERR_INVALID_CONFIG:      s = "invalid configuration";       break;
    case PID_ERR_INVALID_GAIN:        s = "invalid gain";                break;
    case PID_ERR_INVALID_LIMIT:       s = "invalid limit";               break;
    case PID_ERR_INVALID_DT:          s = "invalid sample time";         break;
    case PID_ERR_INVALID_MODE:        s = "invalid mode";                break;
    case PID_ERR_INVALID_PARAM:       s = "invalid parameter";           break;
    case PID_ERR_NAN_INPUT:           s = "NaN input";                   break;
    case PID_ERR_INF_INPUT:           s = "Inf input";                   break;
    case PID_ERR_SENSOR_RANGE:        s = "sensor out of range";         break;
    case PID_ERR_SENSOR_RATE:         s = "sensor rate implausible";     break;
    case PID_ERR_UNSUPPORTED:         s = "feature not compiled in";     break;
    case PID_ERR_BUSY:                s = "busy";                        break;
    case PID_ERR_TUNE_TIMEOUT:        s = "auto-tune timeout";           break;
    case PID_ERR_TUNE_UNSTABLE:       s = "auto-tune unstable";          break;
    case PID_ERR_TUNE_NO_OSCILLATION: s = "auto-tune no oscillation";    break;
    case PID_ERR_TUNE_MODEL_MISMATCH: s = "tuning rule needs other model";break;
    case PID_ERR_TUNE_ABORTED:        s = "auto-tune aborted";           break;
    case PID_ERR_TUNE_VALIDATION:     s = "auto-tune validation failed";  break;
    default:                          s = "unknown";                     break;
    }
    return s;
}

const char *PID_GetVersion(void)
{
    return PIDX_VERSION_STRING;
}

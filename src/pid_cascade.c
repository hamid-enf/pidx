/**
 * @file    pid_cascade.c
 * @brief   Cascade control coordinator.
 *
 * The whole file is one idea: run the loops outermost-to-innermost, then push
 * the truth about what the actuator actually did back the other way so that no
 * outer integrator can wind up against a saturated inner loop.
 */

#include "pidx/pid_cascade.h"

#if PIDX_ENABLE_CASCADE

/* ======================================================================== */
/* Internal helpers                                                          */
/* ======================================================================== */

static bool pidc_valid(const PID_Cascade *c)
{
    return (c != NULL) && c->initialised && (c->count >= 2U);
}

static void pidc_set_error(PID_Cascade *c, PID_StatusCode e)
{
    /* Sticky, first-wins: an error must survive until somebody reads it, and
     * the first failure in a cycle is the informative one. */
    if ((c != NULL) && (e != PID_OK) && (c->last_error == PID_OK)) {
        c->last_error = e;
    }
}

/** Effective sample interval of a level: its own dt times its decimation. */
static PID_Float pidc_level_period(const PID_Cascade *c, uint8_t i)
{
    const uint16_t dec = (c->level[i].decimation == 0U) ? 1U : c->level[i].decimation;
    return PID_GetSampleTime(c->level[i].pid) * (PID_Float)dec;
}

/* ======================================================================== */
/* Lifecycle                                                                 */
/* ======================================================================== */

PID_StatusCode PID_Cascade_Init(PID_Cascade *c, PID_Handle *const *loops, uint8_t n)
{
    uint8_t i;
    PID_Float kp;
    PID_Float ki;
    PID_Float kd;

    if ((c == NULL) || (loops == NULL)) {
        return PID_ERR_NULL;
    }
    if ((n < 2U) || (n > (uint8_t)PIDX_CASCADE_MAX_LOOPS)) {
        return PID_ERR_INVALID_PARAM;
    }
    for (i = 0U; i < n; ++i) {
        if (loops[i] == NULL) {
            return PID_ERR_NULL;
        }
        /* A handle that never went through PID_Init has no usable dt, so the
         * cascade could not compute per-level periods. Catch it here rather
         * than producing silent nonsense at the first update. */
        if (PID_GetSampleTime(loops[i]) <= PID_ZERO) {
            return PID_ERR_NOT_INIT;
        }
    }

    for (i = 0U; i < (uint8_t)PIDX_CASCADE_MAX_LOOPS; ++i) {
        c->level[i].pid = NULL;
        c->level[i].decimation = 1U;
        c->level[i].sp_min = PID_ZERO;
        c->level[i].sp_max = PID_ZERO;   /* min >= max => clamp disabled */
        c->tick[i] = 0U;
        c->command[i] = PID_ZERO;
    }
    for (i = 0U; i < n; ++i) {
        c->level[i].pid = loops[i];
    }

    c->count = n;
    c->aw_mode = (uint8_t)PID_CASCADE_AW_BACK_CALC;
    c->mode = (uint8_t)PID_GetMode(loops[0]);
    c->output = PID_ZERO;
    c->last_error = PID_OK;
    c->initialised = true;

    /* Derive Kt_c from the outer loop: unwind at roughly the rate that loop
     * winds up, i.e. 1/Ti = Ki/Kp. Falls back to 1 1/s when the outer loop has
     * no integral action yet (gains often get set after wiring the cascade). */
    c->aw_gain = PID_ONE;
    if (PID_GetGains(loops[0], &kp, &ki, &kd) == PID_OK) {
        if ((ki > PID_ZERO) && (kp > PID_ZERO)) {
            c->aw_gain = ki / kp;
        }
    }

    return PID_OK;
}

PID_StatusCode PID_Cascade_ConfigLevel(PID_Cascade *c, uint8_t index,
                                       uint16_t decimation,
                                       PID_Float sp_min, PID_Float sp_max)
{
    if (c == NULL) {
        return PID_ERR_NULL;
    }
    if (!c->initialised || (index >= c->count)) {
        return PID_ERR_INVALID_PARAM;
    }
    if (!pidm_isfinite(sp_min) || !pidm_isfinite(sp_max)) {
        return PID_ERR_INVALID_LIMIT;
    }

    c->level[index].decimation = (decimation == 0U) ? 1U : decimation;
    c->level[index].sp_min = sp_min;
    c->level[index].sp_max = sp_max;
    c->tick[index] = 0U;
    return PID_OK;
}

PID_StatusCode PID_Cascade_SetAntiWindup(PID_Cascade *c,
                                         PID_CascadeAntiWindup mode,
                                         PID_Float aw_gain)
{
    if (c == NULL) {
        return PID_ERR_NULL;
    }
    if (mode > PID_CASCADE_AW_FREEZE) {
        return PID_ERR_INVALID_PARAM;
    }
    if (!pidm_isfinite(aw_gain)) {
        return PID_ERR_INVALID_PARAM;
    }

    c->aw_mode = (uint8_t)mode;
    if (aw_gain > PID_ZERO) {
        c->aw_gain = aw_gain;
    }
    return PID_OK;
}

/* ======================================================================== */
/* Execution                                                                 */
/* ======================================================================== */

PID_Float PID_Cascade_Update(PID_Cascade *c, const PID_Float *measurements,
                             PID_Float setpoint, PID_Float dt)
{
    uint8_t i;
    bool ran[PIDX_CASCADE_MAX_LOOPS];
    PID_Float sp;

    if (!pidc_valid(c)) {
        pidc_set_error(c, (c == NULL) ? PID_ERR_NULL : PID_ERR_NOT_INIT);
        return (c != NULL) ? c->output : PID_ZERO;
    }
    if (measurements == NULL) {
        pidc_set_error(c, PID_ERR_NULL);
        return c->output;
    }
    if (!pidm_isfinite(dt) || (dt <= PID_ZERO)) {
        pidc_set_error(c, PID_ERR_INVALID_DT);
        return c->output;
    }
    if (!pidm_isfinite(setpoint)) {
        pidc_set_error(c, PID_ERR_NAN_INPUT);
        return c->output;
    }

    /* ---------------- Forward pass: outer -> inner ---------------------- */
    sp = setpoint;

    for (i = 0U; i < c->count; ++i) {
        PID_CascadeLevel *lv = &c->level[i];
        const uint16_t dec = (lv->decimation == 0U) ? 1U : lv->decimation;

        ran[i] = false;
        c->tick[i]++;

        if (c->tick[i] >= dec) {
            /* This level is due. It integrates over the whole interval since
             * it last ran, not over the caller's dt - otherwise a decimated
             * loop would under-integrate by exactly its decimation factor. */
            const PID_Float level_dt = dt * (PID_Float)dec;

            c->tick[i] = 0U;
            ran[i] = true;

            PID_SetSetpointImmediate(lv->pid, sp);
            c->command[i] = PID_UpdateDt(lv->pid, measurements[i], level_dt);
        }
        /* else: hold c->command[i] from the previous run - a zero-order hold,
         * which is what the child physically experiences anyway. */

        /* The command becomes the child's setpoint, clamped to a range that
         * makes physical sense for the child. */
        sp = c->command[i];
        if (lv->sp_min < lv->sp_max) {
            sp = pidm_clamp(sp, lv->sp_min, lv->sp_max);
        }
    }

    /* Innermost output is the actuator command. */
    c->output = c->command[c->count - 1U];

    /* ---------------- Backward pass: inner -> outer --------------------- */
    /*
     * Walk from the innermost parent outwards. For each parent, ask whether
     * its child could actually deliver what was requested; if not, correct the
     * parent so its integrator stops accumulating against a wall.
     *
     * Same-cycle, like the core's own back-calculation (stage 13). Deferring
     * the correction one sample re-introduces exactly the lag that anti-windup
     * exists to remove.
     */
    /*
     * HOLD means "the integrator does not move", and MANUAL means "the
     * integrator is owned by the tracking back-solve". Both write the parent's
     * integrator directly via PID_SetIntegrator(), which bypasses the core's
     * own stage-10 mode guard, so the mode has to be honoured here as well -
     * otherwise a cascade in HOLD would still creep whenever a child
     * saturated, and a chain in MANUAL would fight its own tracking solution.
     */
    if ((c->aw_mode != (uint8_t)PID_CASCADE_AW_NONE) &&
        (c->mode == (uint8_t)PID_MODE_AUTOMATIC)) {
        for (i = (uint8_t)(c->count - 1U); i > 0U; --i) {
            const uint8_t p = (uint8_t)(i - 1U);   /* parent index */
            PID_Handle *child = c->level[i].pid;
            PID_Handle *parent = c->level[p].pid;
            const PID_Float requested = c->command[p];   /* parent's raw output */
            PID_Float achievable = requested;
            PID_Float parent_dt;
            bool child_high;
            bool child_low;
            bool clipped_high;
            bool clipped_low;

            if (!ran[p]) {
                continue;   /* parent did not integrate this cycle */
            }
            if (!pidm_isfinite(requested)) {
                continue;
            }

            child_high = ((child->flags & PID_FLAG_SATURATED_HIGH) != 0U);
            child_low  = ((child->flags & PID_FLAG_SATURATED_LOW) != 0U);

            /* Was the parent's request clipped by the level's range clamp? */
            clipped_high = (c->level[p].sp_min < c->level[p].sp_max) &&
                           (requested > c->level[p].sp_max);
            clipped_low  = (c->level[p].sp_min < c->level[p].sp_max) &&
                           (requested < c->level[p].sp_min);

            /*
             * Establish what was actually achievable downstream, and only act
             * when the parent is pushing FURTHER into the obstruction.
             *
             * Direction matters: a child pinned at its upper rail must still
             * let its parent integrate downwards - that is exactly how the
             * pair escapes saturation. Correcting both directions would turn
             * anti-windup into a lock-up.
             *
             * A saturated child parks its measurement wherever the actuator
             * leaves it, so that measurement is the best available evidence of
             * what this branch of the plant can currently deliver.
             */
            if (clipped_high) {
                achievable = c->level[p].sp_max;
            } else if (clipped_low) {
                achievable = c->level[p].sp_min;
            } else if (child_high && (requested > measurements[i])) {
                achievable = measurements[i];
            } else if (child_low && (requested < measurements[i])) {
                achievable = measurements[i];
            } else {
                continue;   /* child is keeping up: nothing to correct */
            }

            if (!pidm_isfinite(achievable)) {
                continue;
            }

            parent_dt = dt * (PID_Float)
                ((c->level[p].decimation == 0U) ? 1U : c->level[p].decimation);

            if (c->aw_mode == (uint8_t)PID_CASCADE_AW_BACK_CALC) {
                /*
                 *   I_parent += Kt_c * (u_achievable - u_requested) * dt_parent
                 *
                 * Identical in form to the core's own back-calculation (stage
                 * 13), with the child standing in for the actuator. Signed and
                 * proportional to the shortfall, so it fades smoothly to zero
                 * as the child recovers and carries no state of its own.
                 */
                const PID_Float corr =
                    c->aw_gain * (achievable - requested) * parent_dt;

                if (pidm_isfinite(corr)) {
                    (void)PID_SetIntegrator(parent,
                                            PID_GetIntegrator(parent) + corr);
                }
            } else {
                /*
                 * FREEZE: undo just this cycle's accumulation, and only when
                 * the parent's error would drive it deeper into the blocked
                 * direction. Whatever it had already banked stays, so it can
                 * still act - it simply cannot dig further.
                 */
                const PID_Float e = PID_GetError(parent);
                const bool digging = ((achievable < requested) && (e > PID_ZERO)) ||
                                     ((achievable > requested) && (e < PID_ZERO));
                if (digging) {
                    PID_Float kp;
                    PID_Float ki;
                    PID_Float kd;
                    if (PID_GetGains(parent, &kp, &ki, &kd) == PID_OK) {
                        const PID_Float step = ki * e * parent_dt;
                        if (pidm_isfinite(step)) {
                            (void)PID_SetIntegrator(
                                parent, PID_GetIntegrator(parent) - step);
                        }
                    }
                }
            }
        }
    }

    /* Surface the first per-loop failure so a cascade user does not have to
     * poll every handle to notice a dead sensor. */
    for (i = 0U; i < c->count; ++i) {
        const PID_StatusCode e = PID_PeekLastError(c->level[i].pid);
        if (e != PID_OK) {
            pidc_set_error(c, e);
            break;
        }
    }

    return c->output;
}

PID_StatusCode PID_Cascade_SetMode(PID_Cascade *c, PID_Mode mode)
{
    uint8_t i;
    PID_StatusCode rc = PID_OK;

    if (c == NULL) {
        return PID_ERR_NULL;
    }
    if (!c->initialised) {
        return PID_ERR_NOT_INIT;
    }
    if (mode > PID_MODE_HOLD) {
        return PID_ERR_INVALID_MODE;
    }

    if (mode == PID_MODE_MANUAL) {
        /*
         * Make the chain self-consistent before freezing it. Each outer loop
         * is told to hold the setpoint its child is currently following, so
         * every level's tracking back-solve lands on a value that is actually
         * true. Skip this and only the innermost loop is bumpless; the outer
         * ones jump the moment the chain re-engages.
         *
         * Innermost first, so each parent reads a child that has already been
         * placed in manual with a settled setpoint.
         */
        for (i = c->count; i > 0U; --i) {
            const uint8_t k = (uint8_t)(i - 1U);
            PID_StatusCode r = PID_SetMode(c->level[k].pid, PID_MODE_MANUAL);
            if (r != PID_OK) { rc = r; }

            if (k > 0U) {
                /* Parent should hold exactly what this child is following. */
                const PID_Float held = PID_GetSetpoint(c->level[k].pid);
                r = PID_SetManualOutput(c->level[k - 1U].pid, held);
                if (r != PID_OK) { rc = r; }
                c->command[k - 1U] = held;
            }
        }
    } else {
        /* Outermost first on the way back: by the time a child switches to
         * AUTOMATIC its parent is already producing a live setpoint. */
        for (i = 0U; i < c->count; ++i) {
            const PID_StatusCode r = PID_SetMode(c->level[i].pid, mode);
            if (r != PID_OK) { rc = r; }
        }
    }

    c->mode = (uint8_t)mode;
    pidc_set_error(c, rc);
    return rc;
}

PID_StatusCode PID_Cascade_SetManualOutput(PID_Cascade *c, PID_Float output)
{
    if (c == NULL) {
        return PID_ERR_NULL;
    }
    if (!c->initialised) {
        return PID_ERR_NOT_INIT;
    }
    if (!pidm_isfinite(output)) {
        return PID_ERR_INVALID_PARAM;
    }

    /* The manual value is an actuator command, so it belongs to the innermost
     * loop. Outer loops keep tracking their children (done in SetMode). */
    c->command[c->count - 1U] = output;
    c->output = output;
    return PID_SetManualOutput(c->level[c->count - 1U].pid, output);
}

PID_StatusCode PID_Cascade_Reset(PID_Cascade *c)
{
    uint8_t i;
    PID_StatusCode rc = PID_OK;

    if (c == NULL) {
        return PID_ERR_NULL;
    }
    if (!c->initialised) {
        return PID_ERR_NOT_INIT;
    }

    for (i = 0U; i < c->count; ++i) {
        const PID_StatusCode r = PID_Reset(c->level[i].pid);
        if (r != PID_OK) { rc = r; }
        c->tick[i] = 0U;
        c->command[i] = PID_ZERO;
    }
    c->output = PID_ZERO;
    c->last_error = PID_OK;
    return rc;
}

/* ======================================================================== */
/* Inspection                                                                */
/* ======================================================================== */

PID_Float PID_Cascade_GetOutput(const PID_Cascade *c)
{
    return (c != NULL) ? c->output : PID_ZERO;
}

PID_Float PID_Cascade_GetLevelSetpoint(const PID_Cascade *c, uint8_t index)
{
    if (!pidc_valid(c) || (index >= c->count)) {
        return PID_ZERO;
    }
    return PID_GetSetpoint(c->level[index].pid);
}

PID_Handle *PID_Cascade_GetLoop(const PID_Cascade *c, uint8_t index)
{
    if (!pidc_valid(c) || (index >= c->count)) {
        return NULL;
    }
    return c->level[index].pid;
}

bool PID_Cascade_IsSaturated(const PID_Cascade *c)
{
    uint8_t i;
    bool sat = false;

    if (pidc_valid(c)) {
        for (i = 0U; i < c->count; ++i) {
            if (PID_IsSaturated(c->level[i].pid)) {
                sat = true;
                break;
            }
        }
    }
    return sat;
}

PID_StatusCode PID_Cascade_GetLastError(PID_Cascade *c)
{
    PID_StatusCode e = PID_ERR_NULL;

    if (c != NULL) {
        e = c->last_error;
        c->last_error = PID_OK;
    }
    return e;
}

PID_StatusCode PID_Cascade_Validate(const PID_Cascade *c,
                                    PID_Float *min_ratio,
                                    uint8_t *worst_index)
{
    /* 3x is the low end of the accepted design range for timescale separation
     * in cascade control; below it the loops interact appreciably. */
    const PID_Float required = (PID_Float)3.0f;
    PID_Float worst = PIDX_HUGE;
    uint8_t worst_i = 0U;
    uint8_t i;

    if (!pidc_valid(c)) {
        return PID_ERR_NOT_INIT;
    }

    for (i = 0U; i < (c->count - 1U); ++i) {
        const PID_Float t_parent = pidc_level_period(c, i);
        const PID_Float t_child = pidc_level_period(c, i + 1U);
        PID_Float ratio;

        if (t_child <= PID_ZERO) {
            continue;
        }
        ratio = t_parent / t_child;
        if (ratio < worst) {
            worst = ratio;
            worst_i = i;
        }
    }

    if (min_ratio != NULL)   { *min_ratio = worst; }
    if (worst_index != NULL) { *worst_index = worst_i; }

    return (worst >= required) ? PID_OK : PID_ERR_INVALID_PARAM;
}

#endif /* PIDX_ENABLE_CASCADE */

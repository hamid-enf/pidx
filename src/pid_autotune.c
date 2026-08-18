/**
 * @file    pid_autotune.c
 * @brief   Non-blocking auto-tune state machine: relay and step identification.
 *
 * The tuner owns the actuator while it runs. PID_AutoTune_Update() returns the
 * value the caller must drive, and every call performs a bounded amount of
 * work: a handful of comparisons in the sampling states, and one burst of
 * arithmetic in the three analysis states, which each execute exactly once.
 * There is no loop that waits, no delay, and no allocation.
 *
 * Relay identification (Astrom-Hagglund describing-function method)
 * -----------------------------------------------------------------
 * A relay with hysteresis eps is placed in the loop:
 *
 *     u = u0 + h            while e >  +eps   (drive up)
 *     u = u0 - h            while e <  -eps   (drive down)
 *
 * The loop settles into a limit cycle at the frequency where the plant phase
 * is -180 degrees. If the cycle has half-amplitude a (half of peak-to-peak)
 * and period Pu, the describing function of the relay gives the ultimate gain
 *
 *     Ku = 4h / (pi * sqrt(a^2 - eps^2)),      a > eps
 *
 * The sqrt term is the hysteresis correction: with eps = 0 it degenerates to
 * the familiar Ku = 4h/(pi*a). Hysteresis is what makes the method usable on
 * a noisy signal - without it, noise crossing zero switches the relay at the
 * noise frequency instead of the process frequency.
 *
 * Step identification (area / moment FOPDT fit)
 * ----------------------------------------------
 * From a step of size du applied in open loop, K = dy_inf / du and the
 * dynamics come from the first two moments of the residual e(t) = y_inf - y(t)
 * of the normalised response. For G(s) = K exp(-Ls)/(1+Ts):
 *
 *     A1 = integral of e(t) dt        = L + T
 *     M1 = integral of t * e(t) dt    = L^2/2 + L*T + T^2
 *
 * which inverts in closed form (verified symbolically):
 *
 *     T = sqrt(2*M1 - A1^2)
 *     L = A1 - T
 *
 * The area method is used in preference to the classical two-point
 * (28.3%/63.2%) fit because both integrals average over the whole transient:
 * a single noisy sample cannot move the answer, whereas a two-point fit reads
 * the model off exactly two samples and needs a final-value estimate that is
 * already accurate while the response is still moving. The two crossing times
 * are still recorded, but only as a cross-check on the area result.
 */

#include "pidx/pid_autotune.h"

#if PIDX_ENABLE_AUTOTUNE

#include "pidx/pid_math.h"

#define PIDT_MAGIC   0x54554E45U   /* 'TUNE' */

#ifndef PIDT_PI
#define PIDT_PI      ((PID_Float)3.14159265358979323846f)
#endif

/* ======================================================================== */
/* Small helpers                                                             */
/* ======================================================================== */

static bool pidt_valid(const PID_AutoTune *t)
{
    return (t != NULL) && (t->magic == PIDT_MAGIC);
}

static void pidt_progress(PID_AutoTune *t, uint8_t pct)
{
    if (t->cfg.on_progress != NULL) {
        t->cfg.on_progress(pct, t->state, t->cfg.cb_ctx);
    }
}

/**
 * Put the controller back exactly as it was found. Called on success, on
 * failure and on abort - a tuner that leaves the plant in MANUAL after an
 * aborted experiment is a safety hazard.
 */
static void pidt_restore(PID_AutoTune *t)
{
    if ((t->h != NULL) && !t->restored) {
        (void)PID_SetManualOutput(t->h, t->saved_manual);
        (void)PID_SetSetpoint(t->h, t->saved_sp);
        (void)PID_SetMode(t->h, t->saved_mode);
        t->restored = true;
    }
}

/** Terminate the run: record the code, restore the plant, fire the callback. */
static void pidt_fail(PID_AutoTune *t, PID_StatusCode code)
{
    t->state       = PID_TUNE_FAILED;
    t->err         = code;
    t->result.code = code;
    t->output      = t->saved_manual;
    pidt_restore(t);
    if (t->cfg.on_done != NULL) {
        t->cfg.on_done(code, t->cfg.cb_ctx);
    }
}

static void pidt_succeed(PID_AutoTune *t)
{
    t->state       = PID_TUNE_COMPLETE;
    t->err         = PID_OK;
    t->result.code = PID_OK;
    t->output      = t->saved_manual;
    pidt_restore(t);
    pidt_progress(t, 100U);
    if (t->cfg.on_done != NULL) {
        t->cfg.on_done(PID_OK, t->cfg.cb_ctx);
    }
}

/** Clamp an injected output to the configured actuator range. */
static PID_Float pidt_clamp_out(const PID_AutoTune *t, PID_Float u)
{
    if (t->cfg.output_max > t->cfg.output_min) {
        return pidm_clamp(u, t->cfg.output_min, t->cfg.output_max);
    }
    return u;
}

/**
 * Safety checks applied on every sample regardless of state.
 * @return true if the run was terminated.
 */
static bool pidt_safety(PID_AutoTune *t, PID_Float y, PID_Float dt)
{
    if (!pidm_isfinite(y)) {
        pidt_fail(t, PID_ERR_NAN_INPUT);
        return true;
    }
    if (t->cfg.abort_fn != NULL) {
        if (t->cfg.abort_fn(t->cfg.cb_ctx)) {
            pidt_fail(t, PID_ERR_TUNE_ABORTED);
            return true;
        }
    }
    if (t->cfg.meas_max > t->cfg.meas_min) {
        if ((y < t->cfg.meas_min) || (y > t->cfg.meas_max)) {
            pidt_fail(t, PID_ERR_SENSOR_RANGE);
            return true;
        }
    }
    if (t->cfg.rate_max > (PID_Float)0.0f) {
        const PID_Float rate = pidm_abs(y - t->y_prev) / dt;
        /* The first sample has no previous value to compare against. */
        if ((t->noise_n > 0U) && (rate > t->cfg.rate_max)) {
            pidt_fail(t, PID_ERR_SENSOR_RATE);
            return true;
        }
    }
    if ((t->cfg.timeout_s > (PID_Float)0.0f)
        && (t->elapsed > t->cfg.timeout_s)) {
        pidt_fail(t, PID_ERR_TUNE_TIMEOUT);
        return true;
    }
    return false;
}

/* ======================================================================== */
/* Configuration                                                             */
/* ======================================================================== */

PID_StatusCode PID_AutoTune_ConfigDefault(PID_AutoTuneConfig *cfg,
                                          PID_IdentMethod ident)
{
    if (cfg == NULL) {
        return PID_ERR_NULL;
    }

    cfg->ident        = ident;
    cfg->rule         = (ident == PID_IDENT_RELAY) ? PID_RULE_TYREUS_LUYBEN
                                                   : PID_RULE_AMIGO_STEP;
    cfg->structure    = PID_STRUCT_PID;
    cfg->lambda       = (PID_Float)0.0f;      /* derived from the model */

    cfg->output_step  = (PID_Float)0.0f;      /* caller must set this   */
    cfg->hysteresis   = (PID_Float)0.0f;
    cfg->bias         = (PID_Float)0.0f;
    cfg->auto_bias    = true;

    cfg->output_min   = (PID_Float)0.0f;
    cfg->output_max   = (PID_Float)0.0f;      /* max<=min disables clamp */
    cfg->meas_min     = (PID_Float)0.0f;
    cfg->meas_max     = (PID_Float)0.0f;      /* max<=min disables check */
    cfg->osc_max      = (PID_Float)0.0f;
    cfg->osc_min      = (PID_Float)0.0f;
    cfg->rate_max     = (PID_Float)0.0f;
    cfg->timeout_s    = (PID_Float)120.0f;

    cfg->warmup_cycles = 2U;
    cfg->eval_cycles   = 4U;
    cfg->stab_time     = (PID_Float)1.0f;
    cfg->stab_rate     = (PID_Float)0.0f;     /* derived from hysteresis */
    cfg->skip_stabilize = false;

    cfg->on_progress  = NULL;
    cfg->on_done      = NULL;
    cfg->abort_fn     = NULL;
    cfg->cb_ctx       = NULL;

    return PID_OK;
}

static PID_StatusCode pidt_check_cfg(const PID_AutoTuneConfig *c)
{
    /* The ident/rule pairing is checked first, ahead of the numeric limits.
     * It is a structural mistake - the chosen rule mathematically cannot be
     * evaluated from the model the chosen experiment produces - and the
     * caller is best served by that specific diagnosis, rather than having it
     * masked by a generic INVALID_PARAM from some amplitude they had not got
     * around to filling in yet. */
    if ((c->ident != PID_IDENT_RELAY) && (c->ident != PID_IDENT_STEP)) {
        return PID_ERR_INVALID_PARAM;
    }
    if ((int)c->rule < 0 || c->rule >= PID_RULE_COUNT_) {
        return PID_ERR_INVALID_PARAM;
    }
    if (c->rule != PID_RULE_CUSTOM) {
        const PID_ModelKind need = PID_TuneRule_RequiredModel(c->rule);
        const PID_ModelKind have = (c->ident == PID_IDENT_RELAY)
                                   ? PID_MODEL_FREQ : PID_MODEL_FOPDT;
        if (need != have) {
            return PID_ERR_TUNE_MODEL_MISMATCH;
        }
    }

    if (!(c->output_step > (PID_Float)0.0f) || !pidm_isfinite(c->output_step)) {
        return PID_ERR_INVALID_PARAM;
    }
    if (c->hysteresis < (PID_Float)0.0f || !pidm_isfinite(c->hysteresis)) {
        return PID_ERR_INVALID_PARAM;
    }
    if ((int)c->structure < 0 || c->structure > PID_STRUCT_PID) {
        return PID_ERR_INVALID_PARAM;
    }
    if (c->eval_cycles == 0U) {
        return PID_ERR_INVALID_PARAM;
    }
    if ((uint16_t)c->eval_cycles > (uint16_t)PIDX_TUNE_MAX_CYCLES) {
        return PID_ERR_INVALID_PARAM;
    }
    return PID_OK;
}

PID_StatusCode PID_AutoTune_Init(PID_AutoTune *t, const PID_AutoTuneConfig *cfg)
{
    PID_StatusCode rc;
    uint8_t *p;
    size_t   i;

    if ((t == NULL) || (cfg == NULL)) {
        return PID_ERR_NULL;
    }
    rc = pidt_check_cfg(cfg);
    if (rc != PID_OK) {
        return rc;
    }

    /* Deliberate byte-wise clear: no memset dependency, and it keeps the
     * struct free of indeterminate padding for the magic check below. */
    p = (uint8_t *)t;
    for (i = 0U; i < sizeof(*t); ++i) {
        p[i] = 0U;
    }

    t->cfg   = *cfg;
    t->magic = PIDT_MAGIC;
    t->state = PID_TUNE_IDLE;
    t->err   = PID_OK;
    t->result.model.kind = PID_MODEL_NONE;
    t->result.suggested_ident = (PID_TuneRule_RequiredModel(cfg->rule)
                                 == PID_MODEL_FOPDT)
                                ? PID_IDENT_STEP : PID_IDENT_RELAY;
    return PID_OK;
}

PID_StatusCode PID_AutoTune_RegisterRule(PID_AutoTune *t, PID_TuneRuleFn fn,
                                         void *ctx)
{
    if (!pidt_valid(t)) {
        return PID_ERR_NULL;
    }
    t->rule_fn  = fn;
    t->rule_ctx = ctx;
    return PID_OK;
}

/* ======================================================================== */
/* Start / abort                                                             */
/* ======================================================================== */

PID_StatusCode PID_AutoTune_Start(PID_AutoTune *t, PID_Handle *h, PID_Float sp)
{
    if (!pidt_valid(t) || (h == NULL)) {
        return PID_ERR_NULL;
    }
    if ((t->state != PID_TUNE_IDLE) && (t->state != PID_TUNE_COMPLETE)
        && (t->state != PID_TUNE_FAILED)) {
        return PID_ERR_BUSY;
    }
    if (!pidm_isfinite(sp)) {
        return PID_ERR_NAN_INPUT;
    }
    if (t->cfg.rule == PID_RULE_CUSTOM && t->rule_fn == NULL) {
        return PID_ERR_INVALID_PARAM;
    }

    t->h           = h;
    t->setpoint    = sp;
    t->saved_mode  = PID_GetMode(h);
    /* Seed the bias from the commanded manual level, not from PID_GetOutput():
     * the latter reports the last completed update, so a caller that has just
     * done SetMode(MANUAL) + SetManualOutput() without stepping the loop would
     * hand the tuner a stale zero and the relay would swing about the wrong
     * operating point. */
    t->saved_manual = PID_GetManualOutput(h);
    t->saved_sp    = PID_GetSetpoint(h);
    t->restored    = false;

    /* The tuner drives the actuator directly; the controller must not fight
     * it. MANUAL also keeps the core back-solving its integrator every sample,
     * so whatever the tune leaves behind is a bumpless starting point. */
    (void)PID_SetMode(h, PID_MODE_MANUAL);

    t->elapsed    = (PID_Float)0.0f;
    t->state_time = (PID_Float)0.0f;
    t->stab_timer = (PID_Float)0.0f;
    t->noise_acc  = (PID_Float)0.0f;
    t->noise_n    = 0U;
    t->cycle_count = 0U;
    t->cycles_kept = 0U;
    t->per_sum = (PID_Float)0.0f;
    t->per_sq  = (PID_Float)0.0f;
    t->amp_sum = (PID_Float)0.0f;
    t->amp_pos_sum = (PID_Float)0.0f;
    t->amp_neg_sum = (PID_Float)0.0f;
    t->per_min = (PID_Float)0.0f;
    t->per_max = (PID_Float)0.0f;
    t->amp_min = (PID_Float)0.0f;
    t->amp_max = (PID_Float)0.0f;
    t->got_283 = false;
    t->got_632 = false;
    t->area1   = (PID_Float)0.0f;
    t->moment1 = (PID_Float)0.0f;
    t->t_end   = (PID_Float)0.0f;
    t->y_acc   = (PID_Float)0.0f;
    t->y_acc_n = 0U;
    t->y_slow      = (PID_Float)0.0f;
    t->y_slow_prev = (PID_Float)0.0f;
    t->settle_timer = (PID_Float)0.0f;
    t->err = PID_OK;

    t->u0 = t->cfg.auto_bias ? t->saved_manual : t->cfg.bias;

    /* Start centred on the bias so the plant is not disturbed before the
     * experiment proper begins. */
    t->output    = pidt_clamp_out(t, t->u0);
    t->relay_high = true;

    t->state = t->cfg.skip_stabilize
               ? ((t->cfg.ident == PID_IDENT_RELAY) ? PID_TUNE_RELAY_WARMUP
                                                    : PID_TUNE_STEP_APPLY)
               : PID_TUNE_STABILIZING;

    t->result.code = PID_ERR_BUSY;
    t->result.model.kind = PID_MODEL_NONE;
    pidt_progress(t, 0U);
    return PID_OK;
}

PID_StatusCode PID_AutoTune_Abort(PID_AutoTune *t)
{
    if (!pidt_valid(t)) {
        return PID_ERR_NULL;
    }
    if ((t->state == PID_TUNE_IDLE) || (t->state == PID_TUNE_COMPLETE)) {
        return PID_ERR_BUSY;
    }
    pidt_fail(t, PID_ERR_TUNE_ABORTED);
    return PID_OK;
}

/* ======================================================================== */
/* State: STABILIZING                                                        */
/* ======================================================================== */

/**
 * Wait until the process is quiet enough that the experiment starts from a
 * defined operating point. The threshold defaults to a value derived from the
 * hysteresis band: a process moving slower than eps per stab_time is, by the
 * relay's own resolution, standing still.
 */
static void pidt_do_stabilize(PID_AutoTune *t, PID_Float y, PID_Float dt)
{
    PID_Float thr = t->cfg.stab_rate;
    const PID_Float rate = pidm_abs(y - t->y_prev) / dt;

    if (!(thr > (PID_Float)0.0f)) {
        const PID_Float dwell = (t->cfg.stab_time > (PID_Float)0.0f)
                                ? t->cfg.stab_time : (PID_Float)1.0f;
        thr = (t->cfg.hysteresis > (PID_Float)0.0f)
              ? (t->cfg.hysteresis / dwell)
              : (pidm_abs(y) * (PID_Float)0.01f / dwell);
        if (!(thr > (PID_Float)0.0f)) {
            thr = (PID_Float)1e-4f;
        }
    }

    /* The default threshold above is derived from the relay hysteresis, which
     * says nothing about how noisy the sensor is - and on a step test there is
     * no hysteresis to derive it from at all. On a 1% noisy signal the raw
     * sample-to-sample rate is around two orders of magnitude above such a
     * threshold even when the plant is completely still, so stabilisation
     * would never be declared and every tune would end in a timeout.
     *
     * Hold the threshold at three times the measured noise rate. The estimate
     * needs a few samples to mean anything, hence the sample count guard. */
    if (t->noise_n > 8U) {
        const PID_Float noise_rate =
            (t->noise_acc / (PID_Float)t->noise_n) / dt;
        const PID_Float floor_rate = (PID_Float)3.0f * noise_rate;
        if (thr < floor_rate) {
            thr = floor_rate;
        }
    }

    t->output = pidt_clamp_out(t, t->u0);

    if (rate <= thr) {
        t->stab_timer += dt;
    } else {
        t->stab_timer = (PID_Float)0.0f;
    }

    if (t->stab_timer >= t->cfg.stab_time) {
        t->y0 = y;
        t->y_min = y;
        t->y_max = y;
        t->y_peak = y;
        t->y_settled = y;
        t->t_last_cross = t->elapsed;
        t->state_time = (PID_Float)0.0f;
        t->state = (t->cfg.ident == PID_IDENT_RELAY) ? PID_TUNE_RELAY_WARMUP
                                                     : PID_TUNE_STEP_APPLY;
        pidt_progress(t, 10U);
    }
}

/* ======================================================================== */
/* State: RELAY                                                              */
/* ======================================================================== */

/**
 * One relay sample.
 *
 * The relay switches on the hysteresis band around the setpoint, and a full
 * period is measured between two successive switches of the SAME direction.
 * Amplitude is taken as half the peak-to-peak excursion accumulated between
 * those two switches, with the positive and negative halves tracked
 * separately so that asymmetry - the signature of a nonlinear plant or a
 * badly chosen bias - can be reported.
 */
static void pidt_do_relay(PID_AutoTune *t, PID_Float y, PID_Float dt)
{
    const PID_Float e   = t->setpoint - y;
    const PID_Float eps = t->cfg.hysteresis;
    bool switched_up = false;

    (void)dt;

    /* Track the extremes of the current half cycle. */
    if (y > t->y_max) { t->y_max = y; }
    if (y < t->y_min) { t->y_min = y; }

    /* Relay with hysteresis: switch only outside the eps band, so noise
     * inside the band cannot chatter the actuator. */
    if (t->relay_high) {
        if (e < -eps) {
            t->relay_high = false;
        }
    } else {
        if (e > eps) {
            t->relay_high = true;
            switched_up = true;
        }
    }

    t->output = pidt_clamp_out(t, t->relay_high ? (t->u0 + t->cfg.output_step)
                                                : (t->u0 - t->cfg.output_step));

    if (!switched_up) {
        return;
    }

    /* A rising switch closes one full period. */
    {
        const PID_Float period = t->elapsed - t->t_last_cross;
        const PID_Float a_pos  = t->y_max - t->setpoint;
        const PID_Float a_neg  = t->setpoint - t->y_min;
        const PID_Float amp    = (PID_Float)0.5f * (t->y_max - t->y_min);

        t->t_last_cross = t->elapsed;
        t->y_max = y;
        t->y_min = y;
        t->cycle_count++;

        if (t->state == PID_TUNE_RELAY_WARMUP) {
            /* Early cycles are transient: the limit cycle has not formed yet
             * and including them biases both Pu and a. */
            if (t->cycle_count >= (uint16_t)t->cfg.warmup_cycles) {
                t->state = PID_TUNE_RELAY_OSC;
                t->cycle_count = 0U;
                pidt_progress(t, 25U);
            }
            return;
        }

        /* Oscillation sanity, checked per cycle. */
        if ((t->cfg.osc_max > (PID_Float)0.0f)
            && ((PID_Float)2.0f * amp > t->cfg.osc_max)) {
            pidt_fail(t, PID_ERR_TUNE_UNSTABLE);
            return;
        }

        /* Accumulate statistics for the kept cycles. */
        if (t->cycles_kept == 0U) {
            t->per_min = period;  t->per_max = period;
            t->amp_min = amp;     t->amp_max = amp;
        } else {
            if (period < t->per_min) { t->per_min = period; }
            if (period > t->per_max) { t->per_max = period; }
            if (amp    < t->amp_min) { t->amp_min = amp; }
            if (amp    > t->amp_max) { t->amp_max = amp; }
        }
        t->per_sum += period;
        t->per_sq  += period * period;
        t->amp_sum += amp;
        t->amp_pos_sum += a_pos;
        t->amp_neg_sum += a_neg;
        t->cycles_kept++;

        {
            const uint8_t pct = (uint8_t)(25U
                + (uint32_t)((uint32_t)t->cycles_kept * 50U)
                  / (uint32_t)t->cfg.eval_cycles);
            pidt_progress(t, (pct > 75U) ? 75U : pct);
        }

        if (t->cycles_kept >= (uint16_t)t->cfg.eval_cycles) {
            t->state = PID_TUNE_ANALYZING;
        }
    }
}

/* ======================================================================== */
/* State: STEP                                                               */
/* ======================================================================== */

/**
 * Open-loop step. The output jumps by output_step and the response is watched
 * for the two crossings the FOPDT fit needs. The end of the experiment is
 * detected by the response going flat, not by a fixed timer, so a slow plant
 * is not cut off early and a fast one does not waste time.
 */
static void pidt_do_step(PID_AutoTune *t, PID_Float y, PID_Float dt)
{
    if (t->state == PID_TUNE_STEP_APPLY) {
        t->y0     = t->y_settled;
        t->output = pidt_clamp_out(t, t->u0 + t->cfg.output_step);
        t->state  = PID_TUNE_STEP_RECORD;
        t->state_time = (PID_Float)0.0f;
        t->y_peak = y;
        t->y_slow      = y;
        t->y_slow_prev = y;
        pidt_progress(t, 20U);
        return;
    }

    t->output = pidt_clamp_out(t, t->u0 + t->cfg.output_step);

    /* Low-pass the response to get a stable y_infinity estimate. The time
     * constant is a fifth of the elapsed test time, so it adapts to the plant
     * instead of needing to be configured. */
    {
        const PID_Float tau = (t->state_time > (PID_Float)0.0f)
                              ? (t->state_time * (PID_Float)0.2f)
                              : dt;
        const PID_Float a   = tau / (tau + dt);
        t->y_settled = a * t->y_settled + ((PID_Float)1.0f - a) * y;
    }

    /* Accumulate the moments of the raw response (y - y0). The final value is
     * subtracted analytically at analysis time, so no y_inf estimate is
     * needed while the experiment is still running - which matters, because
     * the running estimate necessarily lags the response.
     *
     * Both integrals use the trapezoidal rule evaluated at the interval
     * MIDPOINT (t - dt/2). This is not cosmetic. The analytic subtraction
     * uses integral of t dt = te^2/2, and only the midpoint sum matches that
     * exactly; sampling the moment arm at the right endpoint instead makes
     * the discrete sum te^2/2 + te*dt/2, leaving the first moment short by
     * te*dt/2 per unit of dy. That deficit GROWS with test length, so a
     * longer and more careful experiment would produce a worse model - the
     * opposite of what the user would expect. With the midpoint arm the fit
     * error is bounded by the integrator's own O(dt^2) truncation and stays
     * flat as the test runs on. */
    {
        const PID_Float y_avg = (PID_Float)0.5f * ((y - t->y0)
                                                   + (t->y_prev - t->y0));
        const PID_Float t_mid = t->state_time - (PID_Float)0.5f * dt;

        t->area1   += y_avg * dt;
        t->moment1 += t_mid * y_avg * dt;
    }
    t->t_end = t->state_time;

    if (pidm_abs(y - t->y0) > pidm_abs(t->y_peak - t->y0)) {
        t->y_peak = y;
    }

    /* Flatness test. Both moments integrate the residual (y_inf - y), so the
     * experiment must not stop while that residual is still significant: a
     * truncated tail biases A1 low and M1 much lower, which shows up directly
     * as an underestimated T and an overestimated L.
     *
     * The response is therefore required to be within 0.5% of its final value
     * AND to have stopped moving, held for a full 25% of the elapsed test
     * time. The proportional dwell means slow plants automatically get a
     * longer confirmation window than fast ones. */
    {
        /* The slope must be measured on a filtered signal. On a raw noisy
         * measurement, (y - y_prev)/dt is dominated by the noise: for 1% noise
         * at dt = 10 ms the apparent slope is order 1 unit/s even when the
         * plant is perfectly still, so a threshold tight enough to mean
         * "settled" would never be met and the tune could only ever time out.
         *
         * A one-pole filter at a twentieth of the elapsed test time is slow
         * enough to reject that noise yet still far faster than the plant
         * tail it has to detect. */
        const PID_Float tau_s = (t->state_time > (PID_Float)0.0f)
                                ? (t->state_time * (PID_Float)0.05f) : dt;
        const PID_Float as = tau_s / (tau_s + dt);
        PID_Float slope;
        const PID_Float total = pidm_abs(t->y_settled - t->y0);
        PID_Float remaining;
        PID_Float slope_thr;
        PID_Float near_thr;

        t->y_slow_prev = t->y_slow;
        t->y_slow = as * t->y_slow + ((PID_Float)1.0f - as) * y;

        slope     = pidm_abs(t->y_slow - t->y_slow_prev) / dt;
        remaining = pidm_abs(t->y_settled - t->y_slow);
        slope_thr = (total > (PID_Float)0.0f)
                    ? (total * (PID_Float)0.001f) : (PID_Float)1e-6f;
        near_thr  = total * (PID_Float)0.005f;

        /* Both thresholds must stay above the noise floor, or the test can
         * never pass and the only possible outcome is a timeout.
         *
         * noise_acc/noise_n is the mean |y[k] - y[k-1]|, which for a still
         * plant is pure measurement noise. The filter above attenuates a
         * step change of that size to (1-a) of it per sample, so the residual
         * slope jitter is (1-a)*noise/dt; the threshold is held at three
         * times that. near_thr is floored at twice the raw noise for the same
         * reason: the response cannot be shown to be closer to its final
         * value than the noise can resolve. */
        if (t->noise_n > 0U) {
            const PID_Float noise = t->noise_acc / (PID_Float)t->noise_n;
            const PID_Float jitter = ((PID_Float)1.0f - as) * noise / dt;
            const PID_Float floor_slope = (PID_Float)3.0f * jitter;
            const PID_Float floor_near  = (PID_Float)2.0f * noise;

            if (slope_thr < floor_slope) { slope_thr = floor_slope; }
            if (near_thr  < floor_near)  { near_thr  = floor_near;  }
        }

        if ((total > (PID_Float)0.0f) && (slope < slope_thr)
            && (remaining < near_thr)) {
            t->settle_timer += dt;
            /* Average the raw measurement across the flat window. This, not
             * the lagging low-pass estimate, is what the fit will use as
             * y_infinity - see the sensitivity note in pidt_analyze_step. */
            t->y_acc += y;
            if (t->y_acc_n < 0xFFFFU) {
                t->y_acc_n++;
            }
        } else {
            t->settle_timer = (PID_Float)0.0f;
            t->y_acc   = (PID_Float)0.0f;
            t->y_acc_n = 0U;
        }

        /* Guard against declaring victory before the plant has even reacted.
         * Right after the step, total is still tiny, so both relative
         * thresholds are tiny too and noise alone can satisfy them - the tune
         * would "settle" during the dead time and fit a model to nothing.
         * The response must have moved clear of the noise, and by a
         * meaningful fraction of the step it is expected to produce, before
         * settling is even considered. */
        {
            const PID_Float moved = pidm_abs(t->y_slow - t->y0);
            PID_Float move_min = (PID_Float)0.0f;

            if (t->noise_n > 8U) {
                move_min = (PID_Float)10.0f
                           * (t->noise_acc / (PID_Float)t->noise_n);
            }
            if (moved < move_min) {
                t->settle_timer = (PID_Float)0.0f;
                t->y_acc   = (PID_Float)0.0f;
                t->y_acc_n = 0U;
            }
        }

        if ((t->settle_timer > (t->state_time * (PID_Float)0.25f))
            && (t->settle_timer > (PID_Float)20.0f * dt)
            && (t->y_acc_n > 0U)) {
            /* Commit the settle-window mean as the final value. */
            t->y_settled = t->y_acc / (PID_Float)t->y_acc_n;
            t->state = PID_TUNE_ANALYZING;
            return;
        }
    }

    /* Record the two crossings against the running final-value estimate. The
     * crossings are only meaningful once the response is well developed, so
     * the 63.2% mark is only accepted after the 28.3% mark. */
    {
        const PID_Float total = t->y_settled - t->y0;
        const PID_Float now   = y - t->y0;

        if (pidm_abs(total) > (PID_Float)0.0f) {
            const PID_Float frac = now / total;
            if (!t->got_283 && (frac >= (PID_Float)0.283f)) {
                t->t_283 = t->state_time;
                t->got_283 = true;
            }
            if (t->got_283 && !t->got_632 && (frac >= (PID_Float)0.632f)) {
                t->t_632 = t->state_time;
                t->got_632 = true;
                pidt_progress(t, 50U);
            }
        }
    }
}

/* ======================================================================== */
/* State: ANALYZING - raw data to plant model                                */
/* ======================================================================== */

static void pidt_analyze_relay(PID_AutoTune *t)
{
    const PID_Float n   = (PID_Float)t->cycles_kept;
    const PID_Float pu  = t->per_sum / n;
    const PID_Float a   = t->amp_sum / n;
    const PID_Float eps = t->cfg.hysteresis;
    const PID_Float a_pos = t->amp_pos_sum / n;
    const PID_Float a_neg = t->amp_neg_sum / n;
    PID_Float amp_min_ok;
    PID_Float radicand;
    PID_Float ku;

    /* Reject a "limit cycle" that is really just noise. */
    amp_min_ok = (t->cfg.osc_min > (PID_Float)0.0f)
                 ? t->cfg.osc_min
                 : ((eps > (PID_Float)0.0f) ? ((PID_Float)2.0f * eps)
                                            : (PID_Float)0.0f);
    if ((a <= (PID_Float)0.0f) || (a < amp_min_ok)) {
        pidt_fail(t, PID_ERR_TUNE_NO_OSCILLATION);
        return;
    }
    if (!(pu > (PID_Float)0.0f)) {
        pidt_fail(t, PID_ERR_TUNE_NO_OSCILLATION);
        return;
    }

    /* Describing-function inversion with the hysteresis correction:
     *   Ku = 4h / (pi * sqrt(a^2 - eps^2))
     * If the amplitude does not clear the hysteresis band, the relay never
     * really exercised the plant and the formula is undefined. */
    radicand = a * a - eps * eps;
    if (radicand <= (PID_Float)0.0f) {
        pidt_fail(t, PID_ERR_TUNE_NO_OSCILLATION);
        return;
    }
    ku = ((PID_Float)4.0f * t->cfg.output_step)
         / (PIDT_PI * pidm_sqrt(radicand));

    t->result.model.kind = PID_MODEL_FREQ;
    t->result.model.ku   = ku;
    t->result.model.pu   = pu;
    t->result.model.k    = (PID_Float)0.0f;
    t->result.model.t    = (PID_Float)0.0f;
    t->result.model.l    = (PID_Float)0.0f;

    /* Spread of the kept cycles is the repeatability measure; it is what
     * "quality" means here, rather than an invented score. */
    t->result.period_spread = (t->per_max - t->per_min) / pu;
    t->result.amp_spread    = (t->amp_max - t->amp_min) / a;
    t->result.amplitude     = a;
    t->result.cycles_used   = t->cycles_kept;

    {
        const PID_Float denom = a_pos + a_neg;
        t->result.asymmetry = (denom > (PID_Float)0.0f)
                              ? (pidm_abs(a_pos - a_neg) / denom)
                              : (PID_Float)0.0f;
        /* Above 0.30 the plant is nonlinear or the bias is wrong at this
         * operating point. The result is still usable, but the user must be
         * told rather than silently trusting it. */
        t->result.asymmetric = (t->result.asymmetry > (PID_Float)0.30f);
    }

    /* Quality: start at 100 and subtract for period spread, amplitude spread
     * and asymmetry, each against its acceptance threshold from the design
     * doc (10%, 15%, 30%). */
    {
        PID_Float q = (PID_Float)100.0f;
        q -= (t->result.period_spread / (PID_Float)0.10f) * (PID_Float)25.0f;
        q -= (t->result.amp_spread    / (PID_Float)0.15f) * (PID_Float)15.0f;
        q -= (t->result.asymmetry     / (PID_Float)0.30f) * (PID_Float)10.0f;
        if (q < (PID_Float)0.0f)   { q = (PID_Float)0.0f; }
        if (q > (PID_Float)100.0f) { q = (PID_Float)100.0f; }
        t->result.model.quality = (uint8_t)q;
    }

    t->result.model.noise_sigma = (t->noise_n > 0U)
        ? (t->noise_acc / (PID_Float)t->noise_n) : (PID_Float)0.0f;

    t->state = PID_TUNE_COMPUTING;
}

static void pidt_analyze_step(PID_AutoTune *t)
{
    const PID_Float dy = t->y_settled - t->y0;
    PID_Float k;
    PID_Float tt;
    PID_Float l;

    if (pidm_abs(dy) <= (PID_Float)0.0f) {
        pidt_fail(t, PID_ERR_TUNE_NO_OSCILLATION);
        return;
    }

    k = dy / t->cfg.output_step;

    /* NOTE ON SENSITIVITY: the fit subtracts the accumulated area from the
     * enclosing rectangle, and those two are of similar size, so the result
     * is far more sensitive to dy than to the integrals themselves. With
     * A1 = te - area1/dy, a relative error d in dy moves A1 by
     * (area1/dy)*d, i.e. by roughly (te/A1)*d in relative terms - an
     * amplification of about 8x for a test run to eight time constants.
     * dy therefore comes from the mean of the raw measurement over the
     * confirmed-flat settle window, never from the low-pass estimate that
     * tracked the transient.
     *
     * Area / moment fit. area1 and moment1 hold the integrals of (y - y0);
     * subtracting them from the enclosing rectangles dy*te and dy*te^2/2
     * gives the integrals of the residual (y_inf - y), and dividing by the
     * total change dy gives the moments of the UNIT step response, which is
     * what the closed form is derived for:
     *
     *   A1 = (1/dy) * integral (y_inf - y) dt   = L + T
     *   M1 = (1/dy) * integral t*(y_inf - y) dt = L^2/2 + L*T + T^2
     *
     * then  T = sqrt(2*M1 - A1^2),  L = A1 - T.
     *
     * Without the 1/dy the radicand carries a factor of dy^2 against a term
     * in dy and goes negative for any dy > 1.
     *
     * The tail beyond t_end is neglected: settling was declared only once the
     * response was within 0.5% of its final value, so the residual there is
     * already negligible against the accumulated area. */
    {
        const PID_Float te = t->t_end;
        const PID_Float a1 = (dy * te - t->area1) / dy;
        const PID_Float m1 = (dy * te * te * (PID_Float)0.5f - t->moment1) / dy;
        const PID_Float rad = (PID_Float)2.0f * m1 - a1 * a1;

        if (!(rad > (PID_Float)0.0f) || !(a1 > (PID_Float)0.0f)) {
            /* The moments are inconsistent with a first-order model - the
             * response was too short, too noisy, or not first order at all. */
            pidt_fail(t, PID_ERR_TUNE_VALIDATION);
            return;
        }
        tt = pidm_sqrt(rad);
        l  = a1 - tt;
    }

    if (!(tt > (PID_Float)0.0f)) {
        pidt_fail(t, PID_ERR_TUNE_VALIDATION);
        return;
    }
    if (l < (PID_Float)0.0f) {
        /* A negative dead time is physically impossible; it means the model
         * has essentially no transport delay. Floor it at zero and let the
         * quality score reflect that the fit is strained. */
        l = (PID_Float)0.0f;
    }

    t->result.model.kind = PID_MODEL_FOPDT;
    t->result.model.k    = k;
    t->result.model.t    = tt;
    t->result.model.l    = l;
    t->result.model.ku   = (PID_Float)0.0f;
    t->result.model.pu   = (PID_Float)0.0f;
    t->result.amplitude  = pidm_abs(dy);
    t->result.cycles_used = 1U;

    /* Quality combines two independent indicators:
     *  - the normalised dead time L/T, since the fit degrades outside
     *    roughly [0.05, 2];
     *  - agreement with the classical 63.2% crossing, which for a true FOPDT
     *    must fall at t = L + T. A large disagreement means the plant is not
     *    first order, and the caller deserves to know that the model it is
     *    about to tune from is an approximation. */
    {
        const PID_Float ratio = l / tt;
        PID_Float q = (PID_Float)100.0f;

        if (ratio < (PID_Float)0.05f) {
            q -= (PID_Float)40.0f;   /* dead time barely resolvable      */
        } else if (ratio > (PID_Float)2.0f) {
            q -= (PID_Float)45.0f;   /* dead-time dominant, FOPDT strained */
        }
        /* Second indicator: how much of the transient the experiment actually
         * captured. The moments weight late samples by t, so a test stopped
         * at only a few time constants has integrated a truncated tail.
         * Below 5*(L+T) the fit is extrapolating more than measuring.
         *
         * The 63.2% crossing is deliberately NOT used here. It is recorded
         * against the running low-pass estimate of the final value, which
         * lags the response, so the crossing always fires early by an amount
         * that depends on the plant - it measures the estimator, not the fit
         * quality. */
        {
            const PID_Float span = l + tt;
            if (span > (PID_Float)0.0f) {
                const PID_Float covered = t->t_end / span;
                if (covered < (PID_Float)5.0f) {
                    PID_Float d = ((PID_Float)5.0f - covered)
                                  * (PID_Float)10.0f;
                    if (d > (PID_Float)45.0f) { d = (PID_Float)45.0f; }
                    q -= d;
                }
            }
        }
        if (q < (PID_Float)0.0f)   { q = (PID_Float)0.0f; }
        if (q > (PID_Float)100.0f) { q = (PID_Float)100.0f; }
        t->result.model.quality = (uint8_t)q;
    }

    t->result.model.noise_sigma = (t->noise_n > 0U)
        ? (t->noise_acc / (PID_Float)t->noise_n) : (PID_Float)0.0f;

    t->state = PID_TUNE_COMPUTING;
}

/* ======================================================================== */
/* State: COMPUTING and VALIDATING                                           */
/* ======================================================================== */

static void pidt_compute(PID_AutoTune *t)
{
    PID_StatusCode rc;

    if (t->cfg.rule == PID_RULE_CUSTOM) {
        if (t->rule_fn == NULL) {
            pidt_fail(t, PID_ERR_INVALID_PARAM);
            return;
        }
        rc = t->rule_fn(&t->result.model, t->cfg.structure, &t->result.gains,
                        t->rule_ctx);
    } else {
        rc = PID_TuneRule_Apply(t->cfg.rule, &t->result.model,
                                t->cfg.structure, t->cfg.lambda,
                                &t->result.gains);
    }

    if (rc != PID_OK) {
        if (rc == PID_ERR_TUNE_MODEL_MISMATCH) {
            /* Tell the caller which experiment would have worked instead of
             * fabricating the missing model parameters. */
            t->result.suggested_ident =
                (PID_TuneRule_RequiredModel(t->cfg.rule) == PID_MODEL_FOPDT)
                ? PID_IDENT_STEP : PID_IDENT_RELAY;
        }
        pidt_fail(t, rc);
        return;
    }

    /* Cohen-Coon is only valid on a band of normalised dead time; outside it
     * the formula still evaluates but the answer is not trustworthy. */
    if ((t->cfg.rule == PID_RULE_COHEN_COON)
        && (t->result.model.kind == PID_MODEL_FOPDT)) {
        const PID_Float ratio = t->result.model.l / t->result.model.t;
        if ((ratio < (PID_Float)0.05f) || (ratio > (PID_Float)1.5f)) {
            pidt_fail(t, PID_ERR_TUNE_MODEL_MISMATCH);
            return;
        }
    }

    t->state = PID_TUNE_VALIDATING;
}

static void pidt_validate(PID_AutoTune *t)
{
    const PID_Gains *g = &t->result.gains;
    const PID_Float dt = PID_GetSampleTime(t->h);

    if (!pidm_isfinite(g->kp) || !pidm_isfinite(g->ki)
        || !pidm_isfinite(g->kd) || !pidm_isfinite(g->tf)) {
        pidt_fail(t, PID_ERR_TUNE_VALIDATION);
        return;
    }
    if (!(g->kp > (PID_Float)0.0f)) {
        pidt_fail(t, PID_ERR_TUNE_VALIDATION);
        return;
    }
    if ((g->ki < (PID_Float)0.0f) || (g->kd < (PID_Float)0.0f)) {
        pidt_fail(t, PID_ERR_TUNE_VALIDATION);
        return;
    }

    /* Discretisation floor. The relay can only switch on a sample boundary,
     * so each half period is quantised to a whole number of samples and the
     * measured Pu carries an error of order +/-2*dt. At the 8 samples per
     * period this check originally allowed, that is +/-25% - far too coarse
     * to tune from, and every derived gain inherits it. Requiring 20 samples
     * per period holds the quantisation contribution near 10%, comparable to
     * the intrinsic error of the describing-function method, so neither term
     * dominates.
     *
     * This floor bounds the SAMPLING error only. It cannot bound the error of
     * the relay method itself: on a lag-dominated plant (L/T ~ 0.05) the limit
     * cycle runs about 19% slower than the true ultimate period no matter how
     * finely it is sampled. A step test is the right experiment there. */
    if ((t->result.model.kind == PID_MODEL_FREQ)
        && (dt > (PID_Float)0.0f)
        && (t->result.model.pu < (PID_Float)20.0f * dt)) {
        pidt_fail(t, PID_ERR_TUNE_VALIDATION);
        return;
    }
    if ((t->result.model.kind == PID_MODEL_FOPDT)
        && (dt > (PID_Float)0.0f)
        && (t->result.model.t < (PID_Float)4.0f * dt)) {
        pidt_fail(t, PID_ERR_TUNE_VALIDATION);
        return;
    }

    /* Repeatability gate from the design doc: below 50 the experiment did not
     * produce a consistent limit cycle. */
    if (t->result.model.quality < 50U) {
        pidt_fail(t, PID_ERR_TUNE_VALIDATION);
        return;
    }

    t->result.elapsed_s = t->elapsed;
    pidt_succeed(t);
}

/* ======================================================================== */
/* Update                                                                    */
/* ======================================================================== */

PID_Float PID_AutoTune_Update(PID_AutoTune *t, PID_Float measurement,
                              PID_Float dt)
{
    if (!pidt_valid(t)) {
        return (PID_Float)0.0f;
    }
    if ((t->state == PID_TUNE_IDLE) || (t->state == PID_TUNE_COMPLETE)
        || (t->state == PID_TUNE_FAILED)) {
        return t->output;
    }
    if (!(dt > (PID_Float)0.0f) || !pidm_isfinite(dt)) {
        /* A bad dt would corrupt every period and crossing measurement. */
        pidt_fail(t, PID_ERR_INVALID_DT);
        return t->output;
    }

    t->elapsed    += dt;
    t->state_time += dt;

    if (pidt_safety(t, measurement, dt)) {
        return t->output;
    }

    /* Noise estimate: mean absolute sample-to-sample change. Used to report
     * sigma and, indirectly, to justify the hysteresis the user chose. */
    if (t->noise_n > 0U) {
        t->noise_acc += pidm_abs(measurement - t->y_prev);
    }
    if (t->noise_n < 0xFFFFFFFFU) {
        t->noise_n++;
    }

    switch (t->state) {
    case PID_TUNE_STABILIZING:
        t->y_settled = measurement;
        pidt_do_stabilize(t, measurement, dt);
        break;

    case PID_TUNE_RELAY_WARMUP:
    case PID_TUNE_RELAY_OSC:
        pidt_do_relay(t, measurement, dt);
        break;

    case PID_TUNE_STEP_APPLY:
    case PID_TUNE_STEP_RECORD:
        pidt_do_step(t, measurement, dt);
        break;

    default:
        break;
    }

    /* The three analysis states are one-shot: entering one runs it to
     * completion within this same call, so the caller never sees the tuner
     * stall in a state that produces no output. Each is straight-line code -
     * no loops - so the worst-case execution time stays bounded. */
    if (t->state == PID_TUNE_ANALYZING) {
        pidt_progress(t, 80U);
        if (t->cfg.ident == PID_IDENT_RELAY) {
            pidt_analyze_relay(t);
        } else {
            pidt_analyze_step(t);
        }
    }
    if (t->state == PID_TUNE_COMPUTING) {
        pidt_progress(t, 90U);
        pidt_compute(t);
    }
    if (t->state == PID_TUNE_VALIDATING) {
        pidt_validate(t);
    }

    t->y_prev = measurement;
    return t->output;
}

/* ======================================================================== */
/* Queries                                                                   */
/* ======================================================================== */

bool PID_AutoTune_IsComplete(const PID_AutoTune *t)
{
    return pidt_valid(t) && (t->state == PID_TUNE_COMPLETE);
}

bool PID_AutoTune_IsRunning(const PID_AutoTune *t)
{
    if (!pidt_valid(t)) {
        return false;
    }
    return (t->state != PID_TUNE_IDLE) && (t->state != PID_TUNE_COMPLETE)
        && (t->state != PID_TUNE_FAILED);
}

PID_TuneState PID_AutoTune_GetState(const PID_AutoTune *t)
{
    return pidt_valid(t) ? t->state : PID_TUNE_IDLE;
}

PID_StatusCode PID_AutoTune_GetError(const PID_AutoTune *t)
{
    return pidt_valid(t) ? t->err : PID_ERR_NULL;
}

uint8_t PID_AutoTune_GetProgress(const PID_AutoTune *t)
{
    if (!pidt_valid(t)) {
        return 0U;
    }
    switch (t->state) {
    case PID_TUNE_IDLE:          return 0U;
    case PID_TUNE_STABILIZING:   return 5U;
    case PID_TUNE_RELAY_WARMUP:  return 20U;
    case PID_TUNE_STEP_APPLY:    return 20U;
    case PID_TUNE_RELAY_OSC: {
        const uint32_t done = (uint32_t)t->cycles_kept * 50U
                              / (uint32_t)t->cfg.eval_cycles;
        return (uint8_t)(25U + ((done > 50U) ? 50U : done));
    }
    case PID_TUNE_STEP_RECORD:   return t->got_632 ? 60U : 40U;
    case PID_TUNE_ANALYZING:     return 80U;
    case PID_TUNE_COMPUTING:     return 90U;
    case PID_TUNE_VALIDATING:    return 95U;
    case PID_TUNE_COMPLETE:      return 100U;
    case PID_TUNE_FAILED:        return 100U;
    default:                     return 0U;
    }
}

PID_StatusCode PID_AutoTune_GetResult(const PID_AutoTune *t,
                                      PID_AutoTuneResult *r)
{
    if (!pidt_valid(t) || (r == NULL)) {
        return PID_ERR_NULL;
    }
    if ((t->state != PID_TUNE_COMPLETE) && (t->state != PID_TUNE_FAILED)) {
        return PID_ERR_BUSY;
    }
    *r = t->result;
    return (t->state == PID_TUNE_COMPLETE) ? PID_OK : t->err;
}

/* ======================================================================== */
/* Apply / retune                                                            */
/* ======================================================================== */

PID_StatusCode PID_AutoTune_Apply(PID_AutoTune *t, PID_Handle *h)
{
    PID_Handle *dst;
    PID_StatusCode rc;

    if (!pidt_valid(t)) {
        return PID_ERR_NULL;
    }
    if (t->state != PID_TUNE_COMPLETE) {
        return PID_ERR_BUSY;
    }
    dst = (h != NULL) ? h : t->h;
    if (dst == NULL) {
        return PID_ERR_NULL;
    }

    /* Rescaling keeps Ki*integral constant across the gain change, so the
     * output does not step when the new tuning lands. */
    rc = PID_SetGainsRescaleIntegral(dst, t->result.gains.kp,
                                     t->result.gains.ki,
                                     t->result.gains.kd);
    if (rc != PID_OK) {
        return rc;
    }
    if (t->result.gains.tf > (PID_Float)0.0f) {
        (void)PID_SetDerivativeFilter(dst, t->result.gains.tf);
    }
    return PID_OK;
}

PID_StatusCode PID_AutoTune_Retune(PID_AutoTune *t, PID_TuneRule rule,
                                   PID_TuneStructure structure)
{
    PID_StatusCode rc;
    PID_Gains g;

    if (!pidt_valid(t)) {
        return PID_ERR_NULL;
    }
    if (t->result.model.kind == PID_MODEL_NONE) {
        return PID_ERR_BUSY;
    }
    if (rule == PID_RULE_CUSTOM) {
        if (t->rule_fn == NULL) {
            return PID_ERR_INVALID_PARAM;
        }
        rc = t->rule_fn(&t->result.model, structure, &g, t->rule_ctx);
    } else {
        rc = PID_TuneRule_Apply(rule, &t->result.model, structure,
                                t->cfg.lambda, &g);
    }
    if (rc != PID_OK) {
        return rc;
    }

    t->result.gains  = g;
    t->cfg.rule      = rule;
    t->cfg.structure = structure;
    t->state         = PID_TUNE_COMPLETE;
    t->err           = PID_OK;
    t->result.code   = PID_OK;
    return PID_OK;
}

/* ======================================================================== */
/* Strings                                                                   */
/* ======================================================================== */

const char *PID_TuneStateToString(PID_TuneState s)
{
    switch (s) {
    case PID_TUNE_IDLE:         return "IDLE";
    case PID_TUNE_STABILIZING:  return "STABILIZING";
    case PID_TUNE_RELAY_WARMUP: return "RELAY_WARMUP";
    case PID_TUNE_RELAY_OSC:    return "RELAY_OSC";
    case PID_TUNE_STEP_APPLY:   return "STEP_APPLY";
    case PID_TUNE_STEP_RECORD:  return "STEP_RECORD";
    case PID_TUNE_ANALYZING:    return "ANALYZING";
    case PID_TUNE_COMPUTING:    return "COMPUTING";
    case PID_TUNE_VALIDATING:   return "VALIDATING";
    case PID_TUNE_COMPLETE:     return "COMPLETE";
    case PID_TUNE_FAILED:       return "FAILED";
    default:                    return "?";
    }
}

#endif /* PIDX_ENABLE_AUTOTUNE */

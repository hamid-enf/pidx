/**
 * @file    pid_fixed.c
 * @brief   Standalone fixed-point PID controller. No float, no libm, no
 *          dependency on the floating-point core.
 *
 * See pid_fixed.h for the number formats and the overflow analysis. The one
 * structural rule of this file: every division lives in pidq_recompute(),
 * which runs only from Init/SetGains. PIDq_Update() contains multiplies,
 * shifts, adds and comparisons only.
 */
#include "pidx/pid_fixed.h"

#if PIDX_ENABLE_FIXED_POINT

/** Handle marker. Chosen to be unlikely in uninitialised RAM. */
#define PIDQ_MAGIC          0x51D7U

/** Fractional bits of the wide internal state. */
#define PIDQ_FRAC           30
/** Fractional bits of the process signals. */
#define PIDQ_SIG_FRAC       15
/** Fractional bits of the gain format. */
#define PIDQ_GAIN_FRAC      16

#define PIDQ_MICRO          1000000

/* ======================================================================== */
/* Small helpers                                                             */
/* ======================================================================== */

/**
 * Saturating conversion from the wide Q30 domain to a Q15 signal, with
 * round-to-nearest.
 *
 * A plain arithmetic shift truncates toward negative infinity, which biases
 * the output down by up to one LSB on every sample. In a loop that closes
 * around this output the bias behaves like a small constant disturbance, so
 * the rounding term is not cosmetic.
 */
static int16_t pidq_q30_to_q15(int32_t v_q30)
{
    int32_t r;

    /* Add half an LSB before shifting, away from zero on both sides so the
     * conversion stays symmetric (no DC offset for a zero-mean signal). */
    if (v_q30 >= 0) {
        /* Guard the rounding add itself against overflow near INT32_MAX. */
        if (v_q30 > (int32_t)2147483647 - (int32_t)(1 << (PIDQ_FRAC - PIDQ_SIG_FRAC - 1))) {
            return PIDQ_MAX;
        }
        r = (v_q30 + (int32_t)(1 << (PIDQ_FRAC - PIDQ_SIG_FRAC - 1)))
            >> (PIDQ_FRAC - PIDQ_SIG_FRAC);
    } else {
        if (v_q30 < (int32_t)(-2147483647 - 1) + (int32_t)(1 << (PIDQ_FRAC - PIDQ_SIG_FRAC - 1))) {
            return PIDQ_MIN;
        }
        r = (v_q30 - (int32_t)(1 << (PIDQ_FRAC - PIDQ_SIG_FRAC - 1)))
            >> (PIDQ_FRAC - PIDQ_SIG_FRAC);
    }

    if (r > (int32_t)PIDQ_MAX) { r = (int32_t)PIDQ_MAX; }
    if (r < (int32_t)PIDQ_MIN) { r = (int32_t)PIDQ_MIN; }
    return (int16_t)r;
}

/** Promote a Q15 signal into the wide Q30 domain. Always exact. */
static int32_t pidq_q15_to_q30(int16_t v_q15)
{
    return ((int32_t)v_q15) << (PIDQ_FRAC - PIDQ_SIG_FRAC);
}

/** Clamp a wide value. */
static int32_t pidq_clamp32(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

/**
 * Multiply a Q_a value by a Q_b value and return the result in Q30.
 *
 * Performed in int64_t and shifted once, so no intermediate rounding is lost
 * and no 32-bit overflow is possible for the ranges documented in the header.
 * The result is saturated rather than wrapped: a wrapped controller output
 * flips sign, which on a real actuator means full reverse.
 */
static int32_t pidq_mul_shift(int32_t a, int32_t b, unsigned shift)
{
    int64_t p = (int64_t)a * (int64_t)b;
    p >>= shift;
    if (p > (int64_t)2147483647) { return (int32_t)2147483647; }
    if (p < (int64_t)(-2147483647 - 1)) { return (int32_t)(-2147483647 - 1); }
    return (int32_t)p;
}

/** Saturating add in the wide domain. */
static int32_t pidq_add_sat(int32_t a, int32_t b)
{
    int64_t s = (int64_t)a + (int64_t)b;
    if (s > (int64_t)2147483647) { return (int32_t)2147483647; }
    if (s < (int64_t)(-2147483647 - 1)) { return (int32_t)(-2147483647 - 1); }
    return (int32_t)s;
}

static bool pidq_valid(const PIDq_Handle *h)
{
    return (h != NULL) && (h->magic == PIDQ_MAGIC);
}

/* ======================================================================== */
/* Coefficient folding - the only divisions in this file                     */
/* ======================================================================== */

/**
 * Fold the sample period and filter constant into the coefficients used by
 * the update path.
 *
 *   ci = Ki * dt                      [Q30]
 *   cb = Kd / (Tf + dt)               [Q16.16]
 *   ca = Tf / (Tf + dt)               [Q30]
 *
 * Times arrive in microseconds. Writing Ki*dt out in the Q30 domain:
 *
 *   ci_q30 = (ki_q16 / 2^16) * (dt_us / 1e6) * 2^30
 *          = ki_q16 * dt_us * 2^14 / 1e6
 *
 * The numerator is built in int64_t, so the 2^14 scaling is applied before the
 * division and no precision is thrown away first.
 *
 * @return PID_OK, or PID_ERR_INVALID_GAIN when a folded coefficient would not
 *         fit its format (Ki*dt >= 2.0 means the loop integrates more than
 *         full scale in one sample - always a configuration error).
 */
static PID_StatusCode pidq_recompute(PIDq_Handle *h, uint32_t dt_us,
                                     uint32_t tf_us, int32_t ki_q16,
                                     int32_t kd_q16)
{
    int64_t num;
    int64_t den;

    /* --- ci = Ki*dt in Q30 ------------------------------------------------ */
    num = (int64_t)ki_q16 * (int64_t)dt_us;
    num <<= (PIDQ_FRAC - PIDQ_GAIN_FRAC);      /* 2^14 */
    num /= (int64_t)PIDQ_MICRO;
    if ((num > (int64_t)2147483647) || (num < -(int64_t)2147483647)) {
        return PID_ERR_INVALID_GAIN;
    }
    h->ci_q30 = (int32_t)num;

    /* --- cb and ca -------------------------------------------------------- */
    den = (int64_t)tf_us + (int64_t)dt_us;     /* > 0, dt_us validated > 0 */

    /* cb = Kd/(Tf+dt) in Q16.16, times arriving in microseconds:
     *   cb_q16 = (kd_q16 / 2^16) * (1e6 / (tf_us+dt_us)) * 2^16
     *          = kd_q16 * 1e6 / (tf_us + dt_us)                              */
    num = (int64_t)kd_q16 * (int64_t)PIDQ_MICRO;
    num /= den;
    if ((num > (int64_t)2147483647) || (num < -(int64_t)2147483647)) {
        /* Kd huge relative to Tf+dt: the derivative would dominate entirely.
         * Refuse rather than silently saturate every sample. */
        return PID_ERR_INVALID_GAIN;
    }
    h->cb_q16 = (int32_t)num;

    /* ca = Tf/(Tf+dt) in Q30. Strictly < 1, so it always fits. */
    num = ((int64_t)tf_us) << PIDQ_FRAC;
    num /= den;
    h->ca_q30 = (int32_t)num;

    return PID_OK;
}

/* ======================================================================== */
/* Configuration                                                             */
/* ======================================================================== */

PID_StatusCode PIDq_ConfigDefault(PIDq_Config *cfg)
{
    if (cfg == NULL) {
        return PID_ERR_NULL;
    }

    cfg->kp_q16         = 65536;            /* 1.0 */
    cfg->ki_q16         = 0;
    cfg->kd_q16         = 0;
    cfg->dt_us          = 1000U;            /* 1 kHz */
    cfg->tf_us          = 0U;
    cfg->out_min_q15    = PIDQ_MIN;
    cfg->out_max_q15    = PIDQ_MAX;
    cfg->i_min_q15      = PIDQ_MIN;
    cfg->i_max_q15      = PIDQ_MAX;
    cfg->deadband_q15   = 0U;
    cfg->separation_q15 = 0U;
    cfg->aw_mode        = (uint8_t)PIDQ_AW_CLAMP;
    cfg->bc_shift       = 4U;
    cfg->lpf_shift      = 0U;
    cfg->direction      = (uint8_t)PIDQ_DIRECT;
    cfg->mode           = (uint8_t)PIDQ_MODE_AUTOMATIC;

    return PID_OK;
}

PID_StatusCode PIDq_Init(PIDq_Handle *h, const PIDq_Config *cfg)
{
    PID_StatusCode rc;

    if ((h == NULL) || (cfg == NULL)) {
        return PID_ERR_NULL;
    }

    /* --- validation ------------------------------------------------------- */
    if (cfg->dt_us == 0U) {
        return PID_ERR_INVALID_DT;
    }
    /* Gains carry no sign: the sense of the loop is PIDq_Direction. A negative
     * gain here would silently fight the direction setting. */
    if ((cfg->kp_q16 < 0) || (cfg->ki_q16 < 0) || (cfg->kd_q16 < 0)) {
        return PID_ERR_INVALID_GAIN;
    }
    if (cfg->out_max_q15 <= cfg->out_min_q15) {
        return PID_ERR_INVALID_LIMIT;
    }
    if (cfg->i_max_q15 <= cfg->i_min_q15) {
        return PID_ERR_INVALID_LIMIT;
    }
    if (cfg->aw_mode > (uint8_t)PIDQ_AW_BACK_CALC) {
        return PID_ERR_INVALID_PARAM;
    }
    /* Back-calculation coefficient is 2^-bc_shift; shift 0 would feed the full
     * saturation error back in one sample and ring. */
    if ((cfg->aw_mode == (uint8_t)PIDQ_AW_BACK_CALC) &&
        ((cfg->bc_shift == 0U) || (cfg->bc_shift > 15U))) {
        return PID_ERR_INVALID_PARAM;
    }
    if (cfg->lpf_shift > 15U) {
        return PID_ERR_INVALID_PARAM;
    }
    if (cfg->direction > (uint8_t)PIDQ_REVERSE) {
        return PID_ERR_INVALID_PARAM;
    }
    if (cfg->mode > (uint8_t)PIDQ_MODE_AUTOMATIC) {
        return PID_ERR_INVALID_MODE;
    }
    /* Separation must sit above the deadband, otherwise the two windows
     * overlap and the integrator can never run at all. */
    if ((cfg->separation_q15 != 0U) &&
        (cfg->separation_q15 <= cfg->deadband_q15)) {
        return PID_ERR_INVALID_PARAM;
    }

    h->magic = 0U;                          /* invalid until fully built */

    rc = pidq_recompute(h, cfg->dt_us, cfg->tf_us, cfg->ki_q16, cfg->kd_q16);
    if (rc != PID_OK) {
        return rc;
    }

    h->kp_q16        = cfg->kp_q16;
    h->dt_us         = cfg->dt_us;
    h->tf_us         = cfg->tf_us;
    h->out_min_q30   = pidq_q15_to_q30(cfg->out_min_q15);
    h->out_max_q30   = pidq_q15_to_q30(cfg->out_max_q15);
    h->i_min_q30     = pidq_q15_to_q30(cfg->i_min_q15);
    h->i_max_q30     = pidq_q15_to_q30(cfg->i_max_q15);
    h->deadband_q15  = cfg->deadband_q15;
    h->separation_q15= cfg->separation_q15;
    h->aw_mode       = cfg->aw_mode;
    h->bc_shift      = cfg->bc_shift;
    h->lpf_shift     = cfg->lpf_shift;
    h->direction     = cfg->direction;
    h->mode          = cfg->mode;

    h->integ_q30     = 0;
    h->d_q30         = 0;
    h->meas_filt_q30 = 0;
    h->meas_prev_q30 = 0;
    h->out_q30       = 0;
    h->setpoint_q15  = 0;
    h->manual_q15    = 0;
    h->out_q15       = 0;
    h->flags         = 0U;

    h->magic = PIDQ_MAGIC;
    return PID_OK;
}

PID_StatusCode PIDq_Deinit(PIDq_Handle *h)
{
    if (h == NULL) {
        return PID_ERR_NULL;
    }
    h->magic = 0U;
    return PID_OK;
}

PID_StatusCode PIDq_Reset(PIDq_Handle *h)
{
    if (!pidq_valid(h)) {
        return (h == NULL) ? PID_ERR_NULL : PID_ERR_NOT_INIT;
    }
    h->integ_q30     = 0;
    h->d_q30         = 0;
    h->meas_filt_q30 = 0;
    h->meas_prev_q30 = 0;
    h->out_q30       = 0;
    h->out_q15       = 0;
    h->flags         = 0U;
    return PID_OK;
}

/* ======================================================================== */
/* Update                                                                    */
/* ======================================================================== */

int16_t PIDq_Update(PIDq_Handle *h, int16_t measurement_q15)
{
    int32_t x_q30;
    int32_t e_q30;
    int32_t e_q15;
    int32_t abs_e_q15;
    int32_t p_q30;
    int32_t u_q30;
    int32_t u_sat_q30;
    int32_t dx_q30;
    bool    integrate;

    if (!pidq_valid(h)) {
        return 0;
    }

    /* --- input filter ----------------------------------------------------- */
    x_q30 = pidq_q15_to_q30(measurement_q15);

    if ((h->flags & PIDQ_FLAG_PRIMED) == 0U) {
        /* First sample: seed the filter and the derivative history with the
         * current measurement. Without this the first derivative sees a step
         * from zero and kicks the output to a limit. */
        h->meas_filt_q30 = x_q30;
        h->meas_prev_q30 = x_q30;
        h->flags |= PIDQ_FLAG_PRIMED;
    } else if (h->lpf_shift != 0U) {
        /* EMA in the wide domain: y += (x - y) >> shift.
         *
         * The state must be Q30, not Q15. At Q15 the increment (x-y)>>shift
         * truncates to zero as soon as |x-y| < 2^shift, so the filter output
         * sticks one dead-band short of the input and never converges. */
        h->meas_filt_q30 = pidq_add_sat(h->meas_filt_q30,
                                        (x_q30 - h->meas_filt_q30) >> h->lpf_shift);
    } else {
        h->meas_filt_q30 = x_q30;
    }

    /* --- error ------------------------------------------------------------ */
    e_q30 = pidq_q15_to_q30(h->setpoint_q15) - h->meas_filt_q30;
    if (h->direction == (uint8_t)PIDQ_REVERSE) {
        e_q30 = -e_q30;
    }
    e_q15     = e_q30 >> (PIDQ_FRAC - PIDQ_SIG_FRAC);
    abs_e_q15 = (e_q15 < 0) ? -e_q15 : e_q15;

    /* --- manual mode ------------------------------------------------------ */
    if (h->mode == (uint8_t)PIDQ_MODE_MANUAL) {
        /* Track the manual output so a later switch to automatic is bumpless,
         * and keep the derivative history current so it does not see a jump. */
        int32_t man_q30 = pidq_clamp32(pidq_q15_to_q30(h->manual_q15),
                                       h->out_min_q30, h->out_max_q30);
        h->out_q30       = man_q30;
        h->out_q15       = pidq_q30_to_q15(man_q30);
        h->meas_prev_q30 = h->meas_filt_q30;
        h->d_q30         = 0;
        return h->out_q15;
    }

    /* --- proportional ------------------------------------------------------ */
    /* P = Kp * e.  Kp is Q16.16 and e is Q30, so the product is Q46; shifting
     * by the gain's 16 fractional bits brings it back to Q30. */
    p_q30 = pidq_mul_shift(h->kp_q16, e_q30, PIDQ_GAIN_FRAC);

    /* --- derivative on measurement ----------------------------------------- */
    /* D_k = ca*D_{k-1} - cb*(x_k - x_{k-1}), with ca = Tf/(Tf+dt) and
     * cb = Kd/(Tf+dt). Differentiating the measurement instead of the error
     * removes the impulse a setpoint step would otherwise produce.
     *
     * When Tf = 0 the pole ca is 0 and this collapses to the unfiltered
     * backward difference Kd*(x_{k-1} - x_k)/dt, which is what cb becomes. */
    if (h->cb_q16 != 0) {
        dx_q30   = h->meas_filt_q30 - h->meas_prev_q30;
        h->d_q30 = pidq_mul_shift(h->ca_q30, h->d_q30, PIDQ_FRAC)
                   - pidq_mul_shift(h->cb_q16, dx_q30, PIDQ_GAIN_FRAC);
        if (h->direction == (uint8_t)PIDQ_REVERSE) {
            /* Reverse action already flipped the error; the measurement
             * difference has to follow, otherwise D fights P. */
            h->d_q30 = -h->d_q30;
        }
    } else {
        h->d_q30 = 0;
    }
    h->meas_prev_q30 = h->meas_filt_q30;

    /* --- decide whether to integrate this sample --------------------------- */
    integrate = (h->ci_q30 != 0);

    if (integrate && (h->deadband_q15 != 0U) &&
        (abs_e_q15 < (int32_t)h->deadband_q15)) {
        /* Inside the deadband: stop integrating so sensor noise around the
         * setpoint does not slowly walk the actuator. */
        integrate = false;
    }
    if (integrate && (h->separation_q15 != 0U) &&
        (abs_e_q15 > (int32_t)h->separation_q15)) {
        /* Integral separation: far from the setpoint the integrator only
         * accumulates windup, so run pure PD until the error comes back. */
        integrate = false;
    }
    if (integrate && (h->aw_mode == (uint8_t)PIDQ_AW_CONDITIONAL) &&
        ((h->flags & PIDQ_FLAG_SATURATED) != 0U)) {
        /* Conditional integration: keep integrating only if the error would
         * pull the output back out of the limit it is stuck against. */
        bool at_high = (h->out_q30 >= h->out_max_q30);
        if ((at_high && (e_q30 > 0)) || ((!at_high) && (e_q30 < 0))) {
            integrate = false;
        }
    }

    if (integrate) {
        /* I += Ki*dt*e, accumulated in OUTPUT units at Q30.
         *
         * Holding the integrator in output units (rather than accumulating
         * raw error and multiplying by Ki later) is what makes a runtime gain
         * change bumpless: the stored contribution does not rescale. */
        h->integ_q30 = pidq_add_sat(h->integ_q30,
                                    pidq_mul_shift(h->ci_q30, e_q30, PIDQ_FRAC));
    }

    if (h->aw_mode == (uint8_t)PIDQ_AW_CLAMP) {
        h->integ_q30 = pidq_clamp32(h->integ_q30, h->i_min_q30, h->i_max_q30);
    }

    /* --- sum and saturate --------------------------------------------------- */
    u_q30     = pidq_add_sat(pidq_add_sat(p_q30, h->integ_q30), h->d_q30);
    u_sat_q30 = pidq_clamp32(u_q30, h->out_min_q30, h->out_max_q30);

    if (u_sat_q30 != u_q30) {
        h->flags |= PIDQ_FLAG_SATURATED;
        if (h->aw_mode == (uint8_t)PIDQ_AW_BACK_CALC) {
            /* Back-calculation, applied in the same sample as the saturation:
             * I += (u_sat - u) * 2^-bc_shift. Restricting the coefficient to a
             * power of two keeps this a shift, so the hot path stays
             * division-free. The correction is negative when the output is
             * clamped high, so it unwinds the integrator immediately instead
             * of waiting for the error to change sign. */
            h->integ_q30 = pidq_add_sat(h->integ_q30,
                                        (u_sat_q30 - u_q30) >> h->bc_shift);
        }
    } else {
        h->flags &= (uint8_t)~PIDQ_FLAG_SATURATED;
    }

    h->out_q30 = u_sat_q30;
    h->out_q15 = pidq_q30_to_q15(u_sat_q30);
    return h->out_q15;
}

/* ======================================================================== */
/* Accessors                                                                 */
/* ======================================================================== */

PID_StatusCode PIDq_SetGains(PIDq_Handle *h, int32_t kp_q16,
                             int32_t ki_q16, int32_t kd_q16)
{
    PID_StatusCode rc;
    int32_t save_ci;
    int32_t save_cb;
    int32_t save_ca;

    if (!pidq_valid(h)) {
        return (h == NULL) ? PID_ERR_NULL : PID_ERR_NOT_INIT;
    }
    if ((kp_q16 < 0) || (ki_q16 < 0) || (kd_q16 < 0)) {
        return PID_ERR_INVALID_GAIN;
    }

    /* Refold from the stored dt/Tf. If the new gains do not fit their format,
     * pidq_recompute() may already have written part of the set, so snapshot
     * and roll back - a half-applied tuning is worse than a rejected one. */
    save_ci = h->ci_q30;
    save_cb = h->cb_q16;
    save_ca = h->ca_q30;

    rc = pidq_recompute(h, h->dt_us, h->tf_us, ki_q16, kd_q16);
    if (rc != PID_OK) {
        h->ci_q30 = save_ci;
        h->cb_q16 = save_cb;
        h->ca_q30 = save_ca;
        return rc;
    }

    /* The integrator holds its contribution in OUTPUT units, so it is not
     * rescaled here: that is exactly what makes the gain change bumpless. */
    h->kp_q16 = kp_q16;
    return PID_OK;
}

PID_StatusCode PIDq_SetSetpoint(PIDq_Handle *h, int16_t sp_q15)
{
    if (!pidq_valid(h)) {
        return (h == NULL) ? PID_ERR_NULL : PID_ERR_NOT_INIT;
    }
    h->setpoint_q15 = sp_q15;
    return PID_OK;
}

int16_t PIDq_GetSetpoint(const PIDq_Handle *h)
{
    return pidq_valid(h) ? h->setpoint_q15 : (int16_t)0;
}

PID_StatusCode PIDq_SetMode(PIDq_Handle *h, PIDq_Mode mode)
{
    if (!pidq_valid(h)) {
        return (h == NULL) ? PID_ERR_NULL : PID_ERR_NOT_INIT;
    }
    if (mode > PIDQ_MODE_AUTOMATIC) {
        return PID_ERR_INVALID_MODE;
    }

    if ((h->mode == (uint8_t)PIDQ_MODE_MANUAL) &&
        (mode == PIDQ_MODE_AUTOMATIC)) {
        /* Bumpless manual -> automatic.
         *
         * The next automatic output will be P + I + D. Choosing
         *   I = u_manual - P - D
         * makes that sum exactly u_manual, so the actuator does not step.
         * P is evaluated from the current error, and D is zero because manual
         * mode kept the derivative history aligned with the measurement. */
        int32_t e_q30 = pidq_q15_to_q30(h->setpoint_q15) - h->meas_filt_q30;
        int32_t p_q30;
        if (h->direction == (uint8_t)PIDQ_REVERSE) {
            e_q30 = -e_q30;
        }
        p_q30 = pidq_mul_shift(h->kp_q16, e_q30, PIDQ_GAIN_FRAC);
        h->integ_q30 = pidq_clamp32(
            pidq_q15_to_q30(h->manual_q15) - p_q30 - h->d_q30,
            h->i_min_q30, h->i_max_q30);
    }

    h->mode = (uint8_t)mode;
    return PID_OK;
}

PIDq_Mode PIDq_GetMode(const PIDq_Handle *h)
{
    return pidq_valid(h) ? (PIDq_Mode)h->mode : PIDQ_MODE_MANUAL;
}

PID_StatusCode PIDq_SetManualOutput(PIDq_Handle *h, int16_t u_q15)
{
    if (!pidq_valid(h)) {
        return (h == NULL) ? PID_ERR_NULL : PID_ERR_NOT_INIT;
    }
    h->manual_q15 = u_q15;
    return PID_OK;
}

int16_t PIDq_GetManualOutput(const PIDq_Handle *h)
{
    return pidq_valid(h) ? h->manual_q15 : (int16_t)0;
}

PID_StatusCode PIDq_SetOutputLimits(PIDq_Handle *h, int16_t min_q15,
                                    int16_t max_q15)
{
    if (!pidq_valid(h)) {
        return (h == NULL) ? PID_ERR_NULL : PID_ERR_NOT_INIT;
    }
    if (max_q15 <= min_q15) {
        return PID_ERR_INVALID_LIMIT;
    }
    h->out_min_q30 = pidq_q15_to_q30(min_q15);
    h->out_max_q30 = pidq_q15_to_q30(max_q15);
    h->integ_q30   = pidq_clamp32(h->integ_q30, h->i_min_q30, h->i_max_q30);
    return PID_OK;
}

int16_t PIDq_GetOutput(const PIDq_Handle *h)
{
    return pidq_valid(h) ? h->out_q15 : (int16_t)0;
}

int16_t PIDq_GetIntegral(const PIDq_Handle *h)
{
    return pidq_valid(h) ? pidq_q30_to_q15(h->integ_q30) : (int16_t)0;
}

bool PIDq_IsSaturated(const PIDq_Handle *h)
{
    return pidq_valid(h) && ((h->flags & PIDQ_FLAG_SATURATED) != 0U);
}

bool PIDq_SelfTest(void)
{
    volatile int32_t neg = -256;
    volatile int32_t big = 65536;

    /* Arithmetic (sign-extending) right shift. C99 leaves this
     * implementation-defined; the whole module assumes it. */
    if ((neg >> 4) != -16) {
        return false;
    }
    /* A 32x32 multiply must widen to 64 bits rather than truncate.
     * 65536*65536 = 2^32, which is exactly what a 32-bit product loses. */
    if (((int64_t)big * (int64_t)big) != ((int64_t)1 << 32)) {
        return false;
    }
    return true;
}

#endif /* PIDX_ENABLE_FIXED_POINT */

/* Fast-path equivalence suite.
 *
 * Written after example 05 exposed two coupled defects:
 *
 *  1. PID_UpdateFast_IsSafe() refused a plain PI + output-limits handle,
 *     because it demanded an explicit PID_FEAT_INTEGRAL_LIMIT. That is the
 *     single most ordinary fast-path configuration there is, so the check was
 *     useless in practice.
 *  2. Underneath that was a real divergence: the full path resolved the
 *     integrator bound at run time (inheriting the output limits when no
 *     explicit integral limit was set) while PID_UpdateFast clamped against
 *     the raw h->i_min/i_max fields, which still held +/-HUGE. The two paths
 *     would have produced different integrator states.
 *
 * The fix makes i_min/i_max always hold the already-resolved effective bounds.
 * These tests pin that contract: whenever IsSafe() says yes, the two paths
 * must agree EXACTLY, including after the limits are changed at run time.
 */
#include <stdio.h>
#include <math.h>

#include "pidx/pid.h"

static int pass = 0, bad = 0;
#define CK(c,m) do{ if(c){pass++;} else {bad++;printf("  FAIL: %s\n",(m));} }while(0)

static void base_config(PID_Config *c)
{
    PID_ConfigDefault(c);
    c->core.kp = 1.8f;
    c->core.ki = 6.0f;
    c->core.kd = 0.04f;
    c->core.sample_time = 0.002f;
    c->limits.use_output_limits = true;
    c->limits.output_min = -5.0f;
    c->limits.output_max =  5.0f;
    c->integral.mode = PID_AW_CLAMP;
    c->filter.derivative_mode = PID_DERIV_ON_MEASUREMENT;
    c->filter.tf = 0.01f;
    c->filter.n_filter = 0.0f;
}

/** Drive both handles with the same signal and return the worst divergence. */
static double compare(PID_Handle *full, PID_Handle *fast, int n)
{
    double worst = 0.0;
    int i;

    for (i = 0; i < n; i++) {
        /* A signal with steps, ramps and sign changes, so the comparison
         * covers saturation in both directions and the derivative path. */
        double t = (double)i * 0.002;
        PID_Float y = (PID_Float)(2.0 * sin(t * 3.0)
                                  + ((i > (n / 2)) ? 4.0 : 0.0));
        PID_Float a = PID_Update(full, y);
        PID_Float b = PID_UpdateFast(fast, y);
        double d = fabs((double)a - (double)b);

        if (d > worst) { worst = d; }
    }
    return worst;
}

int main(void)
{
    /* ---- 1. IsSafe accepts the ordinary PI/PID configurations ----------- */
    {
        PID_Handle h;
        PID_Config c;

        base_config(&c);
        CK(PID_Init(&h, &c) == PID_OK, "init");
        CK(PID_UpdateFast_IsSafe(&h),
           "PID + output limits + clamp AW is fast-path safe");

        /* Explicit integral limits must also be accepted. */
        c.limits.use_integral_limits = true;
        c.limits.integral_min = -2.0f;
        c.limits.integral_max =  2.0f;
        CK(PID_Init(&h, &c) == PID_OK, "init explicit i-limits");
        CK(PID_UpdateFast_IsSafe(&h),
           "explicit integral limits are fast-path safe");
        c.limits.use_integral_limits = false;

        /* Without output limits the fast path would clamp to +/-HUGE, which
         * is not what an unlimited controller means. Must be refused. */
        c.limits.use_output_limits = false;
        CK(PID_Init(&h, &c) == PID_OK, "init no limits");
        CK(!PID_UpdateFast_IsSafe(&h), "no output limits is NOT safe");
        c.limits.use_output_limits = true;
    }

    /* ---- 2. every feature the fast path skips must be refused ----------- */
    {
        struct { const char *name; int kind; } list[] = {
            { "sensor safety",        0 },
            { "setpoint ramp",        1 },
            { "output slew",          2 },
            { "input filter",         3 },
            { "feedforward",          4 },
            { "trapezoidal integral", 5 },
            { "derivative on error",  6 },
            { "back-calculation AW",  7 },
            { "conditional AW",       8 },
            { "integral separation",  9 },
            { "integral deadband",   10 },
            { "manual mode",         11 },
        };
        size_t k;

        for (k = 0; k < (sizeof(list) / sizeof(list[0])); k++) {
            PID_Handle h;
            PID_Config c;
            char msg[96];

            base_config(&c);
            switch (list[k].kind) {
            case 0: c.safety.enabled = true;
                    c.safety.meas_min = -10.0f;
                    c.safety.meas_max =  10.0f;            break;
            case 1: c.shaper.sp_rate_max = 1.0f;           break;
            case 2: c.shaper.out_slew_max = 10.0f;         break;
            case 3: c.filter.input_lpf_tau = 0.01f;        break;
            case 4: c.feedforward.enabled = true;
                    c.feedforward.value = 0.5f;            break;
            case 5: c.core.integration =
                        PID_INTEGRATION_TRAPEZOIDAL;       break;
            case 6: c.filter.derivative_mode =
                        PID_DERIV_ON_ERROR;                break;
            case 7: c.integral.mode = PID_AW_BACK_CALCULATION;
                    c.integral.kt = 5.0f;                  break;
            case 8: c.integral.mode = PID_AW_CONDITIONAL;  break;
            case 9: c.integral.separation_threshold = 1.0f;break;
            case 10:c.integral.deadband = 0.01f;           break;
            default:c.core.mode = PID_MODE_MANUAL;         break;
            }

            CK(PID_Init(&h, &c) == PID_OK, "init feature case");
            snprintf(msg, sizeof(msg), "%s must be refused by IsSafe",
                     list[k].name);
            CK(!PID_UpdateFast_IsSafe(&h), msg);
        }
    }

    /* ---- 3. exact agreement, inherited integral bounds ------------------ */
    /* This is the case that was actually broken: no explicit integral limits,
     * so the full path inherited +/-5 while the fast path used +/-HUGE. */
    {
        PID_Handle full, fast;
        PID_Config c;
        double worst;

        base_config(&c);
        CK(PID_Init(&full, &c) == PID_OK, "init full");
        CK(PID_Init(&fast, &c) == PID_OK, "init fast");
        CK(PID_UpdateFast_IsSafe(&fast), "configuration is safe");
        PID_SetSetpoint(&full, 3.0f);
        PID_SetSetpoint(&fast, 3.0f);

        worst = compare(&full, &fast, 4000);
        printf("  inherited integral bounds: worst |u_full - u_fast| = %.3e\n",
               worst);
        CK(worst == 0.0, "bit-for-bit identical with inherited bounds");

        /* And the integrator states themselves, not just the outputs. */
        CK(PID_GetIntegrator(&full) == PID_GetIntegrator(&fast),
           "integrator states are identical too");
    }

    /* ---- 4. exact agreement, explicit integral bounds ------------------- */
    {
        PID_Handle full, fast;
        PID_Config c;
        double worst;

        base_config(&c);
        c.limits.use_integral_limits = true;
        c.limits.integral_min = -1.5f;
        c.limits.integral_max =  1.5f;
        (void)PID_Init(&full, &c);
        (void)PID_Init(&fast, &c);
        PID_SetSetpoint(&full, 3.0f);
        PID_SetSetpoint(&fast, 3.0f);

        worst = compare(&full, &fast, 4000);
        printf("  explicit integral bounds : worst |u_full - u_fast| = %.3e\n",
               worst);
        CK(worst == 0.0, "bit-for-bit identical with explicit bounds");
        CK(fabs((double)PID_GetIntegrator(&full)) <= 1.5 + 1e-6,
           "explicit bound actually enforced");
    }

    /* ---- 5. agreement survives a run-time limit change ------------------ */
    /* PID_SetOutputLimits must re-resolve the inherited integral bound, or
     * the two paths drift apart the moment an application retunes. */
    {
        PID_Handle full, fast;
        PID_Config c;
        double w1, w2;

        base_config(&c);
        (void)PID_Init(&full, &c);
        (void)PID_Init(&fast, &c);
        PID_SetSetpoint(&full, 3.0f);
        PID_SetSetpoint(&fast, 3.0f);

        w1 = compare(&full, &fast, 1000);
        CK(w1 == 0.0, "identical before the limit change");

        CK(PID_SetOutputLimits(&full, -1.0f, 1.0f) == PID_OK, "shrink full");
        CK(PID_SetOutputLimits(&fast, -1.0f, 1.0f) == PID_OK, "shrink fast");
        CK(PID_UpdateFast_IsSafe(&fast), "still safe after the change");

        w2 = compare(&full, &fast, 3000);
        printf("  after PID_SetOutputLimits: worst |u_full - u_fast| = %.3e\n",
               w2);
        CK(w2 == 0.0, "still bit-for-bit identical after the limit change");
        CK(fabs((double)PID_GetIntegrator(&full)) <= 1.0 + 1e-6,
           "integrator followed the new inherited bound");
    }

    /* ---- 6. the fast path really does skip what it claims to skip ------- */
    /* Not a defect: a documented trade. Pinned so nobody "fixes" it into
     * being slow, and so the docs cannot drift away from the behaviour. */
    {
        PID_Handle a, b;
        PID_Config c;
        int i;

        base_config(&c);
        c.safety.enabled         = true;
        c.safety.meas_min        = -10.0f;
        c.safety.meas_max        =  10.0f;
        c.safety.failsafe_output = 0.0f;
        c.safety.fault_persist_n = 1U;
        (void)PID_Init(&a, &c);
        (void)PID_Init(&b, &c);
        PID_SetSetpoint(&a, 1.0f);
        PID_SetSetpoint(&b, 1.0f);

        CK(!PID_UpdateFast_IsSafe(&b), "safety config is correctly refused");

        for (i = 0; i < 20; i++) {
            (void)PID_Update(&a,     500.0f);   /* far outside the range */
            (void)PID_UpdateFast(&b, 500.0f);
        }
        CK(PID_IsFaulted(&a), "full path latches the sensor fault");
        CK(!PID_IsFaulted(&b), "fast path does not check - documented trade");
    }

    /* ---- 7. NaN reaches the fast path unchecked ------------------------- */
    /* Also a documented trade: the fast path has no argument checks at all.
     * Verified explicitly so the README's warning is backed by a test. */
    {
        PID_Handle a, b;
        PID_Config c;
        PID_Float ua, ub;

        base_config(&c);
        (void)PID_Init(&a, &c);
        (void)PID_Init(&b, &c);
        PID_SetSetpoint(&a, 1.0f);
        PID_SetSetpoint(&b, 1.0f);

        for (int i = 0; i < 10; i++) {
            (void)PID_Update(&a, 0.0f);
            (void)PID_UpdateFast(&b, 0.0f);
        }
        ua = PID_Update(&a,     (PID_Float)NAN);
        ub = PID_UpdateFast(&b, (PID_Float)NAN);

        printf("  NaN measurement: full -> %.4f (finite=%d),"
               " fast -> finite=%d\n",
               (double)ua, (int)isfinite((double)ua), (int)isfinite((double)ub));
        CK(isfinite((double)ua), "full path rejects NaN and holds last output");
        CK(PID_PeekLastError(&a) == PID_ERR_NAN_INPUT,
           "and reports PID_ERR_NAN_INPUT");
        CK(!isfinite((double)ub),
           "fast path propagates NaN - guard your inputs upstream");
    }

    printf("\n  fast path: %d passed, %d failed\n", pass, bad);
    return (bad == 0) ? 0 : 1;
}

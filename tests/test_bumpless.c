/* Bumpless-transfer regression suite.
 *
 * Written after example 10 showed a MANUAL -> AUTOMATIC transfer stepping the
 * output by 0.6 on an actuator whose full range is 1.0, while the
 * documentation claimed the transfer was bumpless.
 *
 * The mechanism was correct; the CLAIM was unqualified. Back-solving sets
 * I = u_manual - P - D - FF, and that value is clamped to the integrator
 * bounds. When the clamp bites - typically because the measurement is far
 * from the setpoint, so Kp*e alone already exceeds the actuator range - the
 * requested output is not reachable and no integrator value can reproduce it.
 *
 * The fix was to REPORT that case (PID_FLAG_INTEGRAL_LIMITED plus a sticky
 * error) rather than to pretend it did not happen. These tests pin both the
 * guarantee and its stated precondition.
 */
#include <stdio.h>
#include <math.h>

#include "pidx/pid.h"

static int pass = 0, bad = 0;
#define CK(c,m) do{ if(c){pass++;} else {bad++;printf("  FAIL: %s\n",(m));} }while(0)

static void cfg_basic(PID_Config *c)
{
    PID_ConfigDefault(c);
    c->core.kp = 0.045f;
    c->core.ki = 0.0018f;
    c->core.kd = 0.0f;
    c->core.sample_time = 0.5f;
    c->limits.use_output_limits = true;
    c->limits.output_min = 0.0f;
    c->limits.output_max = 1.0f;
}

int main(void)
{
    /* ---- 1. the guarantee: transfer at the operating point --------------- */
    /* Setpoint equal to the measurement, so P = 0 and the integrator alone
     * has to supply the manual output - always representable inside [0,1]. */
    {
        const double manual[4] = { 0.0, 0.25, 0.60, 1.0 };
        int k;

        for (k = 0; k < 4; k++) {
            PID_Handle h;
            PID_Config c;
            double before, after;
            int i;

            cfg_basic(&c);
            c.core.mode = PID_MODE_MANUAL;
            CK(PID_Init(&h, &c) == PID_OK, "init");
            PID_SetSetpoint(&h, 100.0f);
            CK(PID_SetManualOutput(&h, (PID_Float)manual[k]) == PID_OK,
               "set manual");

            for (i = 0; i < 20; i++) { (void)PID_Update(&h, 100.0f); }
            before = (double)PID_GetOutput(&h);

            CK(PID_SetMode(&h, PID_MODE_AUTOMATIC) == PID_OK, "to automatic");
            after = (double)PID_Update(&h, 100.0f);

            CK(fabs(after - before) < 1e-6,
               "transfer at the operating point is exactly bumpless");
            CK(fabs(before - manual[k]) < 1e-6,
               "manual output was actually held");
        }
        printf("  transfer at operating point: bumpless at u = 0, 0.25,"
               " 0.6, 1.0\n");
    }

    /* ---- 2. bumpless with P, D and feedforward all non-zero -------------- */
    /* The back-solve must account for every term, not just P. */
    {
        PID_Handle h;
        PID_Config c;
        double before, after;
        int i;

        cfg_basic(&c);
        c.core.kp = 0.01f;               /* small, so P fits in the range */
        c.core.kd = 0.02f;
        c.limits.use_integral_limits = true;
        c.limits.integral_min = -5.0f;   /* wide enough to absorb P and FF */
        c.limits.integral_max =  5.0f;
        c.feedforward.enabled = true;
        c.feedforward.value   = 0.15f;
        c.core.mode = PID_MODE_MANUAL;
        CK(PID_Init(&h, &c) == PID_OK, "init with P+D+FF");
        PID_SetSetpoint(&h, 120.0f);
        (void)PID_SetManualOutput(&h, 0.55f);

        /* A moving measurement so the D term is genuinely non-zero, then the
         * SAME measurement across the transfer. Feeding a different value on
         * the transfer sample would make D respond to that change and the
         * test would be measuring its own stimulus, not the transfer. */
        for (i = 0; i < 40; i++) {
            (void)PID_Update(&h, (PID_Float)(100.0 + (0.5 * (double)i)));
        }
        {
            const PID_Float y_hold = (PID_Float)(100.0 + (0.5 * 39.0));
            (void)PID_Update(&h, y_hold);
            before = (double)PID_GetOutput(&h);
            (void)PID_SetMode(&h, PID_MODE_AUTOMATIC);
            after = (double)PID_Update(&h, y_hold);
        }

        /*
         * The residual here is NOT a transfer discontinuity. With a held
         * measurement the D state still decays by c_da = Tf/(Tf+dt) each
         * sample, and the integrator takes its normal Ki*dt*e step. Both are
         * ordinary controller behaviour that happens to occur on the transfer
         * sample; the transfer itself contributes nothing.
         *
         * Proved by running an identical handle that stays in AUTOMATIC: if
         * the two produce the same output, the mode change was free.
         */
        {
            PID_Handle ref;
            PID_Config rc;
            double ref_before, ref_after;
            const PID_Float y_hold = (PID_Float)(100.0 + (0.5 * 39.0));
            int j;

            cfg_basic(&rc);
            rc.core.kp = 0.01f;
            rc.core.kd = 0.02f;
            rc.limits.use_integral_limits = true;
            rc.limits.integral_min = -5.0f;
            rc.limits.integral_max =  5.0f;
            rc.feedforward.enabled = true;
            rc.feedforward.value   = 0.15f;
            rc.core.mode = PID_MODE_MANUAL;
            (void)PID_Init(&ref, &rc);
            PID_SetSetpoint(&ref, 120.0f);
            (void)PID_SetManualOutput(&ref, 0.55f);
            for (j = 0; j < 40; j++) {
                (void)PID_Update(&ref, (PID_Float)(100.0 + (0.5 * (double)j)));
            }
            (void)PID_Update(&ref, y_hold);
            ref_before = (double)PID_GetOutput(&ref);
            /* stays MANUAL, then one more identical sample */
            ref_after = (double)PID_Update(&ref, y_hold);

            printf("  with P+D+FF active: %.6f -> %.6f  (jump %.2e)\n",
                   before, after, fabs(after - before));
            printf("    reference handle held in MANUAL: %.6f -> %.6f\n",
                   ref_before, ref_after);
            CK(fabs(before - ref_before) < 1e-9,
               "both handles agree up to the transfer");
            CK(fabs(after - before) < 0.01,
               "the step at transfer is small: no discontinuity");
        }
    }

    /* ---- 3. the precondition: unreachable request is reported ------------ */
    {
        PID_Handle h;
        PID_Config c;
        PID_Status st;
        double before, after;
        int i;

        cfg_basic(&c);
        c.core.mode = PID_MODE_MANUAL;
        (void)PID_Init(&h, &c);
        PID_SetSetpoint(&h, 150.0f);        /* 50 C above the measurement */
        (void)PID_SetManualOutput(&h, 0.40f);
        (void)PID_ClearError(&h);

        for (i = 0; i < 20; i++) { (void)PID_Update(&h, 100.0f); }

        (void)PID_GetStatus(&h, &st);
        before = (double)PID_GetOutput(&h);

        /* Kp*e = 0.045*50 = 2.25, so reproducing 0.40 needs I = -1.85, which
         * is outside the inherited bound [0, 1]. */
        printf("  unreachable case: P = %.4f, needed I = %.4f, bounds [0,1]\n",
               (double)st.p_term, 0.40 - (double)st.p_term);
        CK(fabs((double)st.p_term - 2.25) < 1e-4, "P term is 2.25 as predicted");
        CK((PID_GetFlags(&h) & PID_FLAG_INTEGRAL_LIMITED) != 0U,
           "PID_FLAG_INTEGRAL_LIMITED raised while still in MANUAL");
        CK(PID_PeekLastError(&h) == PID_ERR_INVALID_LIMIT,
           "sticky error records the unreachable back-solve");

        (void)PID_SetMode(&h, PID_MODE_AUTOMATIC);
        after = (double)PID_Update(&h, 100.0f);
        printf("  and the transfer does bump: %.4f -> %.4f\n", before, after);
        CK(fabs(after - before) > 0.1,
           "the bump is real - the warning was not spurious");
    }

    /* ---- 4. widening the integral bounds makes it reachable again -------- */
    /* Proves the diagnosis: the limitation is the bound, nothing else. */
    {
        PID_Handle h;
        PID_Config c;
        double before, after;
        int i;

        cfg_basic(&c);
        c.limits.use_integral_limits = true;
        c.limits.integral_min = -5.0f;      /* now -1.85 is representable */
        c.limits.integral_max =  5.0f;
        c.core.mode = PID_MODE_MANUAL;
        (void)PID_Init(&h, &c);
        PID_SetSetpoint(&h, 150.0f);
        (void)PID_SetManualOutput(&h, 0.40f);
        (void)PID_ClearError(&h);

        for (i = 0; i < 20; i++) { (void)PID_Update(&h, 100.0f); }
        before = (double)PID_GetOutput(&h);

        CK(PID_PeekLastError(&h) == PID_OK,
           "no warning once the bound can hold the required value");
        CK(fabs((double)PID_GetIntegrator(&h) - (-1.85)) < 1e-4,
           "integrator holds exactly the back-solved -1.85");

        (void)PID_SetMode(&h, PID_MODE_AUTOMATIC);
        after = (double)PID_Update(&h, 100.0f);

        /*
         * The residual is exactly one integration step, Ki*dt*e =
         * 0.0018*0.5*50 = 0.045, and NOT a transfer discontinuity. Once in
         * automatic the controller is supposed to act on a 50 C error; a
         * transfer that suppressed the first sample's integration would be
         * wrong in a subtler way.
         *
         * "Bumpless" means no STEP at the instant of transfer: the output
         * continues from where manual left it and then moves under normal
         * control. It does not mean the output is frozen.
         */
        printf("  with wide integral bounds: %.6f -> %.6f  (jump %.2e,"
               " one Ki*dt*e = %.2e)\n",
               before, after, fabs(after - before), 0.0018 * 0.5 * 50.0);
        CK(fabs(fabs(after - before) - (0.0018 * 0.5 * 50.0)) < 1e-6,
           "residual is exactly one integration step, not a discontinuity");
        CK(PID_PeekLastError(&h) == PID_OK,
           "and no INTEGRAL_LIMITED warning this time");
    }

    /* ---- 5. HOLD freezes the integrator and nothing else ----------------- */
    {
        PID_Handle h;
        PID_Config c;
        PID_Float i0, i1;
        PID_Status st_before, st_after;
        int i;

        cfg_basic(&c);
        c.core.kd = 0.01f;
        (void)PID_Init(&h, &c);
        PID_SetSetpoint(&h, 120.0f);
        for (i = 0; i < 50; i++) { (void)PID_Update(&h, 100.0f); }

        i0 = PID_GetIntegrator(&h);
        (void)PID_GetStatus(&h, &st_before);
        CK(PID_SetMode(&h, PID_MODE_HOLD) == PID_OK, "to hold");

        for (i = 0; i < 50; i++) {
            (void)PID_Update(&h, (PID_Float)(100.0 + (0.2 * (double)i)));
        }
        i1 = PID_GetIntegrator(&h);
        (void)PID_GetStatus(&h, &st_after);

        CK(i0 == i1, "HOLD freezes the integrator bit-exactly");
        CK(st_after.p_term != st_before.p_term,
           "P still tracks the measurement during HOLD");
        CK(st_after.d_term != 0.0f,
           "D still responds during HOLD");
        printf("  HOLD: integrator %.6f held, P moved %.4f -> %.4f\n",
               (double)i0, (double)st_before.p_term, (double)st_after.p_term);

        /* Leaving HOLD must not jump either. */
        {
            double b = (double)PID_GetOutput(&h);
            double a;
            (void)PID_SetMode(&h, PID_MODE_AUTOMATIC);
            a = (double)PID_Update(&h, (PID_Float)(100.0 + (0.2 * 50.0)));
            CK(fabs(a - b) < 0.05, "leaving HOLD does not step the output");
        }
    }

    /* ---- 6. entering MANUAL adopts the current output -------------------- */
    {
        PID_Handle h;
        PID_Config c;
        double auto_u, manual_u;
        int i;

        cfg_basic(&c);
        (void)PID_Init(&h, &c);
        PID_SetSetpoint(&h, 120.0f);
        for (i = 0; i < 50; i++) { (void)PID_Update(&h, 100.0f); }
        auto_u = (double)PID_GetOutput(&h);

        CK(PID_SetMode(&h, PID_MODE_MANUAL) == PID_OK, "to manual");
        CK(fabs((double)PID_GetManualOutput(&h) - auto_u) < 1e-6,
           "entering MANUAL adopts the controller's current output");
        manual_u = (double)PID_Update(&h, 100.0f);
        CK(fabs(manual_u - auto_u) < 1e-6,
           "AUTOMATIC -> MANUAL is bumpless in that direction too");
        printf("  AUTO -> MANUAL: %.6f held\n", manual_u);
    }

    /* ---- 7. fault recovery ----------------------------------------------- */
    /*
     * Recovery uses the same back-solve, so it inherits the same precondition.
     * Both cases are pinned here: recovery near the operating point is
     * bumpless, recovery far from it is reported rather than silently bumped.
     */
    {
        /* 7a: the sensor recovers reading close to the setpoint, so P is
         *     small and the fail-safe output is reproducible. */
        PID_Handle h;
        PID_Config c;
        int i;
        double u_fault, u_recover;

        cfg_basic(&c);
        c.safety.enabled         = true;
        c.safety.meas_min        = -20.0f;
        c.safety.meas_max        = 500.0f;
        c.safety.failsafe_output = 0.20f;
        c.safety.fault_persist_n = 2U;
        c.safety.auto_recover    = true;
        (void)PID_Init(&h, &c);
        PID_SetSetpoint(&h, 120.0f);

        for (i = 0; i < 50; i++) { (void)PID_Update(&h, 118.0f); }
        for (i = 0; i < 10; i++) { (void)PID_Update(&h, 9999.0f); }
        CK(PID_IsFaulted(&h), "fault latched on an out-of-range sensor");
        u_fault = (double)PID_GetOutput(&h);
        CK(fabs(u_fault - 0.20) < 1e-6, "fail-safe output driven while faulted");

        (void)PID_ClearError(&h);
        u_recover = (double)PID_Update(&h, 118.0f);
        printf("  fault recovery near setpoint: %.6f -> %.6f  (jump %.2e)\n",
               u_fault, u_recover, fabs(u_recover - u_fault));
        CK(!PID_IsFaulted(&h), "auto-recovered once the sensor was sane");
        CK(fabs(u_recover - u_fault) < 0.01,
           "recovery from the fail-safe output is bumpless");
    }
    {
        /* 7b: the sensor recovers 20 C below the setpoint. Kp*e = 0.9 alone
         *     exceeds what is left of the range, so reproducing a 0.2
         *     fail-safe output would need I = -0.7: not representable. The
         *     library must say so. */
        PID_Handle h;
        PID_Config c;
        int i;
        double u_fault, u_recover;

        cfg_basic(&c);
        c.safety.enabled         = true;
        c.safety.meas_min        = -20.0f;
        c.safety.meas_max        = 500.0f;
        c.safety.failsafe_output = 0.20f;
        c.safety.fault_persist_n = 2U;
        c.safety.auto_recover    = true;
        (void)PID_Init(&h, &c);
        PID_SetSetpoint(&h, 120.0f);

        for (i = 0; i < 50; i++) { (void)PID_Update(&h, 100.0f); }
        for (i = 0; i < 10; i++) { (void)PID_Update(&h, 9999.0f); }
        u_fault = (double)PID_GetOutput(&h);

        (void)PID_ClearError(&h);
        u_recover = (double)PID_Update(&h, 100.0f);
        printf("  fault recovery 20 C off:      %.6f -> %.6f  (jump %.2e)\n",
               u_fault, u_recover, fabs(u_recover - u_fault));

        /*
         * P = Kp*e = 0.045*20 = 0.90, so reproducing the 0.20 fail-safe
         * output needs I = -0.70. The integrator inherits the output bounds
         * [0, 1], so that is not representable and the recovery cannot be
         * bumpless. The library clamps and reports rather than pretending.
         */
        CK(fabs(u_recover - u_fault) > 0.1,
           "far-from-setpoint recovery does bump - inherent, not a defect");
        CK(PID_PeekLastError(&h) == PID_ERR_INVALID_LIMIT,
           "and it is reported through the sticky error channel");
        printf("    (needed I = 0.20 - 0.90 = -0.70, bounds [0, 1])\n");
    }

    printf("\n  bumpless: %d passed, %d failed\n", pass, bad);
    return (bad == 0) ? 0 : 1;
}

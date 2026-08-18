/* Anti-windup regression suite.
 *
 * Written after example 02 exposed that PID_AW_CONDITIONAL was dead code: the
 * stage-10 test read saturation flags that stage 12 only sets later, and the
 * flags are cleared at the top of every update, so the condition was never
 * true and CONDITIONAL behaved exactly like NONE.
 *
 * Every assertion here is against a closed-form expectation, and each strategy
 * is additionally checked to DIFFER from NONE where it must - a test that only
 * checks "the number is finite" would have passed against the broken code.
 */
#include <stdio.h>
#include <math.h>

#include "pidx/pid.h"

static int pass = 0, bad = 0;
#define CK(c,m) do{ if(c){pass++;} else {bad++;printf("  FAIL: %s\n",(m));} }while(0)

/* A plant that physically cannot follow: the output is clamped to [0,1] but
 * the setpoint demands far more, so the loop saturates and stays there. */
typedef struct {
    double y;
    double gain;
    double tau;
} Plant;

static double plant_step(Plant *p, double u, double dt)
{
    double y_inf = p->gain * u;
    p->y += (dt / p->tau) * (y_inf - p->y);
    return p->y;
}

static PID_StatusCode setup(PID_Handle *h, PID_AntiWindup aw, PID_Float kt,
                            PID_Float dt)
{
    PID_Config c;
    PID_ConfigDefault(&c);
    c.core.kp = 0.5f;
    c.core.ki = 2.0f;
    c.core.kd = 0.0f;
    c.core.sample_time = dt;
    c.limits.use_output_limits = true;
    c.limits.output_min = 0.0f;
    c.limits.output_max = 1.0f;
    c.integral.mode = aw;
    c.integral.kt   = kt;
    return PID_Init(h, &c);
}

int main(void)
{
    const PID_Float dt = 0.01f;

    /* ---- 1. windup during an unreachable setpoint ----------------------- */
    /* Plant gain 1.0, output capped at 1.0, setpoint 3.0 -> permanently
     * saturated. Over 5 s with Ki=2 and e ~ 2, an unprotected integrator
     * accumulates roughly Ki*e*t = 2*2*5 = 20 duty units. */
    {
        struct { PID_AntiWindup aw; PID_Float kt; const char *name;
                 double i_peak; double undershoot; double recover; } r[4] = {
            /* recover is seeded with the full phase-2 duration, so a loop
             * that never leaves the limit scores as the worst possible
             * case instead of as a negative sentinel that flatters it. */
            { PID_AW_NONE,             0.0f, "none",      0, 0, 10.0 },
            { PID_AW_CLAMP,            0.0f, "clamp",     0, 0, 10.0 },
            { PID_AW_CONDITIONAL,      0.0f, "cond",      0, 0, 10.0 },
            { PID_AW_BACK_CALCULATION, 5.0f, "back-calc", 0, 0, 10.0 },
        };
        int k;

        for (k = 0; k < 4; k++) {
            PID_Handle h;
            Plant p = { 0.0, 1.0, 0.5 };
            int i;
            double y_min_after = 1e30;

            CK(setup(&h, r[k].aw, r[k].kt, dt) == PID_OK, "init");
            PID_SetSetpoint(&h, 3.0f);

            /* Phase 1: 5 s chasing an impossible target. */
            for (i = 0; i < 500; i++) {
                PID_Float u = PID_Update(&h, (PID_Float)p.y);
                (void)plant_step(&p, (double)u, (double)dt);
                {
                    double ig = fabs((double)PID_GetIntegrator(&h));
                    if (ig > r[k].i_peak) { r[k].i_peak = ig; }
                }
            }

            /* Phase 2: setpoint drops to something reachable. A wound-up
             * integrator must first be discharged, during which the output
             * stays pinned high and the plant overshoots past the new target
             * - the practical damage windup does. */
            PID_SetSetpoint(&h, 0.4f);
            for (i = 0; i < 1000; i++) {
                PID_Float u = PID_Update(&h, (PID_Float)p.y);
                (void)plant_step(&p, (double)u, (double)dt);
                if ((r[k].recover >= 10.0) && (u < 0.999f)) {
                    r[k].recover = (double)i * (double)dt;
                }
                if (i > 5) {
                    double over = p.y - 0.4;
                    if (over > r[k].undershoot) { r[k].undershoot = over; }
                }
                if (p.y < y_min_after) { y_min_after = p.y; }
            }
        }

        printf("  windup on an unreachable setpoint (Ki=2, 5 s saturated):\n");
        printf("    %-12s %10s %12s %12s\n",
               "strategy", "peak |I|", "overshoot", "recover[s]");
        for (k = 0; k < 4; k++) {
            printf("    %-12s %10.3f %12.4f %12.3f\n",
                   r[k].name, r[k].i_peak, r[k].undershoot, r[k].recover);
        }

        /* NONE must wind up to roughly Ki * e * t. */
        CK(r[0].i_peak > 15.0, "NONE winds up to >15 duty units as predicted");

        /* Every strategy must bound the integrator near the output range.
         * The limit is 1.0; allow a small margin for the sample at which
         * saturation is first detected. */
        CK(r[1].i_peak < 1.5, "CLAMP bounds |I| to the output range");
        CK(r[2].i_peak < 1.5, "CONDITIONAL bounds |I|");
        CK(r[3].i_peak < 1.5, "BACK_CALCULATION bounds |I|");

        /* The regression that started this file: CONDITIONAL must not behave
         * like NONE. Comparing peaks directly is the sharpest form of it. */
        CK(r[2].i_peak < (r[0].i_peak / 5.0),
           "CONDITIONAL differs from NONE by more than 5x (was identical)");
        CK(fabs(r[2].i_peak - r[0].i_peak) > 1.0,
           "CONDITIONAL is not bit-identical to NONE");

        /* And recovery must be faster than doing nothing. */
        /* NONE stays pinned for the whole 10 s window; the others come off
         * the limit on the first sample after the setpoint drops, because
         * they have no accumulated excess to burn through first. */
        CK(r[0].recover >= 10.0, "NONE never regains authority in 10 s");
        CK(r[1].recover < 1.0, "CLAMP regains authority within 1 s");
        CK(r[2].recover < 1.0, "CONDITIONAL regains authority within 1 s");
        CK(r[3].recover < 1.0, "BACK_CALC regains authority within 1 s");
        CK(r[0].undershoot > r[2].undershoot,
           "NONE overshoots the new target more than CONDITIONAL");
    }

    /* ---- 2. conditional integration is asymmetric ----------------------- */
    /*
     * It must block only the increment that drives FURTHER past the limit.
     * The first attempt at this test was wrong in an instructive way: it
     * flipped the setpoint from +5 to -5 while saturated high and expected
     * the integrator to move. But that flip immediately drives u_raw below
     * out_min, so the increment is blocked again - correctly - and the test
     * was asserting the wrong thing.
     *
     * The real asymmetry needs an integrator that is already charged, with an
     * error whose sign opposes the limit the output is stuck on. That state is
     * set up directly with PID_SetIntegrator rather than trying to reach it
     * through the plant.
     */
    {
        PID_Handle h;
        PID_Float i0, i1;

        {
            /* The integrator inherits the output limits by default, so
             * PID_SetIntegrator(2.0) would be silently clamped to 1.0 and
             * case B could never be set up. Widen the integral limits
             * explicitly - and note that this is exactly the kind of silent
             * clamp that makes a test pass vacuously. */
            PID_Config c;
            PID_ConfigDefault(&c);
            c.core.kp = 0.5f;
            c.core.ki = 2.0f;
            c.core.sample_time = dt;
            c.limits.use_output_limits = true;
            c.limits.output_min = 0.0f;
            c.limits.output_max = 1.0f;
            c.limits.use_integral_limits = true;
            c.limits.integral_min = -10.0f;
            c.limits.integral_max =  10.0f;
            c.integral.mode = PID_AW_CONDITIONAL;
            CK(PID_Init(&h, &c) == PID_OK, "init cond");
        }

        /* Case A: saturated high, error positive -> blocked. */
        PID_SetSetpoint(&h, 5.0f);
        for (int i = 0; i < 200; i++) { (void)PID_Update(&h, 0.0f); }
        i0 = PID_GetIntegrator(&h);
        CK(PID_IsSaturated(&h), "saturated high after a large positive error");
        (void)PID_Update(&h, 0.0f);
        i1 = PID_GetIntegrator(&h);
        CK(fabs((double)(i1 - i0)) < 1e-9,
           "no accumulation while saturated in the same direction");

        /* Case B: charge the integrator to 2.0 so that even with a negative
         * error the sum stays above out_max = 1.0:
         *   u_raw = Kp*e + I = 0.5*(-1) + 2.0 = 1.5 > 1.0
         * The output is still saturated high, but the error now pushes it
         * back towards the linear range, so the increment must be allowed. */
        CK(PID_SetIntegrator(&h, 2.0f) == PID_OK, "charge the integrator");
        CK(fabs((double)(PID_GetIntegrator(&h) - 2.0f)) < 1e-6,
           "integrator really holds 2.0 (not silently clamped)");
        PID_SetSetpoint(&h, -1.0f);
        (void)PID_Update(&h, 0.0f);
        i1 = PID_GetIntegrator(&h);
        CK(PID_IsSaturated(&h), "still saturated high (u_raw = 1.5)");
        CK(i1 < 2.0f,
           "an error opposing the active limit still integrates (unwinds)");
        CK(fabs((double)(i1 - (2.0f + (2.0f * dt * -1.0f)))) < 1e-6,
           "and by exactly Ki*dt*e");
    }

    /* ---- 3. conditional integration must not restrict the linear range -- */
    /* Inside the limits it has to be indistinguishable from NONE, otherwise
     * it would silently change the tuning of every unsaturated loop. */
    {
        PID_Handle a, b;
        double worst = 0.0;
        int i;

        CK(setup(&a, PID_AW_NONE, 0.0f, dt) == PID_OK, "init none");
        CK(setup(&b, PID_AW_CONDITIONAL, 0.0f, dt) == PID_OK, "init cond 2");
        PID_SetSetpoint(&a, 0.30f);
        PID_SetSetpoint(&b, 0.30f);

        {
            Plant pa = { 0.0, 1.0, 0.5 };
            Plant pb = { 0.0, 1.0, 0.5 };
            for (i = 0; i < 2000; i++) {
                PID_Float ua = PID_Update(&a, (PID_Float)pa.y);
                PID_Float ub = PID_Update(&b, (PID_Float)pb.y);
                double d = fabs((double)(ua - ub));
                if (d > worst) { worst = d; }
                (void)plant_step(&pa, (double)ua, (double)dt);
                (void)plant_step(&pb, (double)ub, (double)dt);
            }
        }
        printf("  never-saturating loop: worst |u_none - u_cond| = %.3e\n", worst);
        CK(worst < 1e-9,
           "CONDITIONAL is identical to NONE when nothing saturates");
    }

    /* ---- 4. back-calculation rate scales with Kt ------------------------ */
    /* I += Kt*dt*(u_sat - u_raw): doubling Kt must roughly double the rate at
     * which the excess is bled off. */
    {
        const PID_Float kts[3] = { 1.0f, 4.0f, 16.0f };
        double settled[3];
        int k;

        for (k = 0; k < 3; k++) {
            PID_Handle h;
            Plant p = { 0.0, 1.0, 0.5 };
            int i;

            (void)setup(&h, PID_AW_BACK_CALCULATION, kts[k], dt);
            PID_SetSetpoint(&h, 3.0f);
            for (i = 0; i < 500; i++) {
                PID_Float u = PID_Update(&h, (PID_Float)p.y);
                (void)plant_step(&p, (double)u, (double)dt);
            }
            settled[k] = fabs((double)PID_GetIntegrator(&h));
        }
        printf("  back-calculation steady |I| at Kt = 1 / 4 / 16:"
               " %.4f / %.4f / %.4f\n", settled[0], settled[1], settled[2]);
        CK(settled[1] < settled[0], "larger Kt leaves less excess integral");
        CK(settled[2] < settled[1], "and still less at Kt = 16");
    }

    /* ---- 5. integral separation is not anti-windup ---------------------- */
    /*
     * Separation stops integrating on LARGE errors; anti-windup stops
     * integrating when SATURATED. They are orthogonal, and this test pins the
     * trap that follows from confusing them.
     *
     * With Kp = 0.5 and a unity-gain plant, a P-only loop settles where
     *   y = Kp*(r - y)   ->   y = r*Kp/(1 + Kp) = r/3,
     * so for r = 0.9 the P-only solution is y = 0.30 and the residual error
     * is 0.60. If the separation threshold is set BELOW that residual, the
     * integrator is locked out forever and the offset it exists to remove
     * becomes permanent - the loop parks at the P-only answer and looks, to
     * an operator, exactly like a controller with no integral action at all.
     */
    {
        const PID_Float thresholds[2] = { 0.20f, 0.60f };
        double final_y[2];
        int k;

        for (k = 0; k < 2; k++) {
            PID_Handle h;
            PID_Config c;
            Plant p = { 0.0, 1.0, 0.5 };
            int i;

            PID_ConfigDefault(&c);
            c.core.kp = 0.5f;
            c.core.ki = 2.0f;
            c.core.sample_time = dt;
            c.limits.use_output_limits = true;
            c.limits.output_min = 0.0f;
            c.limits.output_max = 1.0f;
            c.integral.mode = PID_AW_CLAMP;
            c.integral.separation_threshold = thresholds[k];
            CK(PID_Init(&h, &c) == PID_OK, "init separation");

            PID_SetSetpoint(&h, 0.9f);
            for (i = 0; i < 4000; i++) {
                PID_Float u = PID_Update(&h, (PID_Float)p.y);
                (void)plant_step(&p, (double)u, (double)dt);
            }
            final_y[k] = p.y;
        }

        printf("  separation threshold 0.20 -> y = %.4f;"
               "  threshold 0.60 -> y = %.4f  (target 0.9)\n",
               final_y[0], final_y[1]);

        /* Threshold 0.20 < residual 0.60: locked out, parks at r/3 = 0.30. */
        CK(fabs(final_y[0] - 0.30) < 0.01,
           "threshold below the P-only residual: I locked out, offset stays");
        /* Threshold 0.60 is not exceeded by the residual, so I engages. */
        CK(fabs(final_y[1] - 0.9) < 0.01,
           "threshold 0.60 > P-only offset: integrator engages, offset removed");
        CK(final_y[1] > final_y[0],
           "separation set too tight is strictly worse, not merely slower");
    }

    /* ---- 6. tracking mode drives I towards an external signal ----------- */
    {
        PID_Handle h;
        PID_Float track = 0.25f;
        int i;

        (void)setup(&h, PID_AW_TRACKING, 5.0f, dt);
        PID_SetSetpoint(&h, 0.0f);
        CK(PID_SetTrackingInput(&h, track) == PID_OK, "set tracking input");

        for (i = 0; i < 2000; i++) { (void)PID_Update(&h, 0.0f); }
        printf("  tracking: u = %.4f towards external %.4f\n",
               (double)PID_GetOutput(&h), (double)track);
        CK(fabs((double)(PID_GetOutput(&h) - track)) < 0.01,
           "output converges on the external tracking signal");
    }

    printf("\n  anti-windup: %d passed, %d failed\n", pass, bad);
    return (bad == 0) ? 0 : 1;
}

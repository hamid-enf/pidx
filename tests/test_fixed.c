/* PHASE 14 - fixed-point controller.
 *
 * The headline test is float/fixed equivalence: the SAME plant and the SAME
 * tuning are run through the floating-point core and through PIDq_*, and the
 * trajectories are compared. Everything else here checks a property that
 * fixed-point arithmetic specifically threatens (resolution, overflow,
 * rounding bias), not merely that the code runs.
 */
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include "pidx/pid.h"
#include "pidx/pid_fixed.h"

static int pass = 0, bad = 0;
#define CK(c,m) do{ if(c){pass++;} else {bad++;printf("  FAIL: %s\n",(m));} }while(0)

/* First-order plant  y' = (K*u - y)/tau, exact discretisation. */
typedef struct { double y, K, tau; } Plant;
static double plant_step(Plant *p, double u, double dt)
{
    double a = exp(-dt / p->tau);
    p->y = a * p->y + (1.0 - a) * p->K * u;
    return p->y;
}

int main(void)
{
    /* ---- 0. platform assumptions ---------------------------------------- */
    CK(PIDq_SelfTest(), "arithmetic shift + 64-bit multiply available");

    /* ---- 1. init validation --------------------------------------------- */
    {
        PIDq_Handle h; PIDq_Config c;
        PIDq_ConfigDefault(&c);
        CK(PIDq_Init(&h, &c) == PID_OK, "default config is valid");
        CK(PIDq_Init(NULL, &c) == PID_ERR_NULL, "NULL handle rejected");

        PIDq_ConfigDefault(&c); c.dt_us = 0;
        CK(PIDq_Init(&h, &c) == PID_ERR_INVALID_DT, "dt=0 rejected");
        PIDq_ConfigDefault(&c); c.kp_q16 = -1;
        CK(PIDq_Init(&h, &c) == PID_ERR_INVALID_GAIN, "negative gain rejected");
        PIDq_ConfigDefault(&c); c.out_max_q15 = c.out_min_q15;
        CK(PIDq_Init(&h, &c) == PID_ERR_INVALID_LIMIT, "min==max rejected");
        PIDq_ConfigDefault(&c);
        c.aw_mode = (uint8_t)PIDQ_AW_BACK_CALC; c.bc_shift = 0;
        CK(PIDq_Init(&h, &c) == PID_ERR_INVALID_PARAM, "bc_shift=0 rejected");
        PIDq_ConfigDefault(&c); c.deadband_q15 = 500; c.separation_q15 = 200;
        CK(PIDq_Init(&h, &c) == PID_ERR_INVALID_PARAM,
           "separation below deadband rejected");
        /* Ki*dt >= 2.0 cannot be represented in the folded Q30 coefficient. */
        PIDq_ConfigDefault(&c); c.ki_q16 = PIDQ_F_TO_Q16(3000.0); c.dt_us = 1000;
        CK(PIDq_Init(&h, &c) == PID_ERR_INVALID_GAIN, "absurd Ki*dt rejected");

        PIDq_ConfigDefault(&c);
        PIDq_Init(&h, &c);
        PIDq_Deinit(&h);
        CK(PIDq_Update(&h, 1000) == 0, "update on deinit handle returns 0");
        CK(PIDq_SetSetpoint(&h, 1) == PID_ERR_NOT_INIT, "deinit detected");
    }

    /* ---- 2. THE integral-resolution test -------------------------------- */
    {
        /* This is the failure mode a Q15 integrator dies from. With
         * Ki = 0.5 1/s, dt = 1 ms and an error of 30 LSB (~0.001), the
         * per-sample increment is 0.5*0.001*0.001 = 5e-7, which is 0.016 of
         * a Q15 LSB. A Q15 accumulator rounds that to zero forever; the Q30
         * accumulator must integrate it correctly.
         *
         * Expected after N samples: I = Ki*dt*e*N = 0.5*0.001*(30/32768)*N.
         * For N = 20000: I = 0.00916 -> 300 Q15 LSB. */
        PIDq_Handle h; PIDq_Config c;
        PIDq_ConfigDefault(&c);
        c.kp_q16 = 0; c.ki_q16 = PIDQ_F_TO_Q16(0.5); c.dt_us = 1000;
        c.aw_mode = (uint8_t)PIDQ_AW_NONE;
        PIDq_Init(&h, &c);
        PIDq_SetSetpoint(&h, 30);

        for (int i = 0; i < 20000; i++) { (void)PIDq_Update(&h, 0); }

        double want = 0.5 * 0.001 * (30.0 / 32768.0) * 20000.0;
        double got  = (double)PIDq_GetIntegral(&h) / 32768.0;
        printf("  tiny-error integral: got %.6f want %.6f (%.2f%% err)\n",
               got, want, 100.0 * fabs(got - want) / want);
        CK(fabs(got - want) / want < 0.01,
           "sub-LSB increments accumulate (Q30 state works)");
        CK(PIDq_GetIntegral(&h) > 0, "integrator actually moved");
    }

    /* ---- 3. EMA filter must converge, not stall ------------------------- */
    {
        /* y += (x-y)>>shift stalls in Q15 as soon as |x-y| < 2^shift, leaving
         * a permanent offset of up to 2^shift LSB. In Q30 it must converge to
         * the input within one LSB. */
        PIDq_Handle h; PIDq_Config c;
        PIDq_ConfigDefault(&c);
        c.kp_q16 = 65536; c.ki_q16 = 0; c.lpf_shift = 6;
        PIDq_Init(&h, &c);
        PIDq_SetSetpoint(&h, 1000);
        int16_t u = 0;
        for (int i = 0; i < 2000; i++) { u = PIDq_Update(&h, 1000); }
        printf("  EMA settled output = %d LSB (expect ~0)\n", (int)u);
        CK(u >= -1 && u <= 1, "EMA converges to the input, no stall offset");
    }

    /* ---- 4. float vs fixed equivalence ---------------------------------- */
    {
        const double dt = 0.001, Kp = 1.2, Ki = 3.0, Kd = 0.02, Tf = 0.005;
        const int N = 4000;

        PID_Handle hf; PID_Config cf;
        PID_ConfigDefault(&cf);
        cf.core.kp = (PID_Float)Kp; cf.core.ki = (PID_Float)Ki;
        cf.core.kd = (PID_Float)Kd; cf.core.sample_time = (PID_Float)dt;
        cf.limits.use_output_limits = true;
        cf.limits.output_min = -1.0f; cf.limits.output_max = 1.0f;
        cf.filter.tf = (PID_Float)Tf;
        PID_Init(&hf, &cf);
        PID_SetSetpoint(&hf, 0.5f);

        PIDq_Handle hq; PIDq_Config cq;
        PIDq_ConfigDefault(&cq);
        cq.kp_q16 = PIDQ_F_TO_Q16(Kp); cq.ki_q16 = PIDQ_F_TO_Q16(Ki);
        cq.kd_q16 = PIDQ_F_TO_Q16(Kd);
        cq.dt_us = 1000; cq.tf_us = 5000;
        cq.out_min_q15 = PIDQ_F_TO_Q15(-0.999); cq.out_max_q15 = PIDQ_MAX;
        CK(PIDq_Init(&hq, &cq) == PID_OK, "equivalence config valid");
        PIDq_SetSetpoint(&hq, PIDQ_F_TO_Q15(0.5));

        Plant pf = {0.0, 1.0, 0.08};
        Plant pq = {0.0, 1.0, 0.08};
        double se = 0.0, maxe = 0.0;

        for (int i = 0; i < N; i++) {
            double uf = (double)PID_Update(&hf, (PID_Float)pf.y);
            (void)plant_step(&pf, uf, dt);

            long yl = lround(pq.y * 32768.0);
            if (yl > (long)PIDQ_MAX) { yl = (long)PIDQ_MAX; }
            if (yl < (long)PIDQ_MIN) { yl = (long)PIDQ_MIN; }
            int16_t yq = (int16_t)yl;
            double uq = (double)PIDq_Update(&hq, yq) / 32768.0;
            (void)plant_step(&pq, uq, dt);

            double d = pf.y - pq.y;
            se += d * d;
            if (fabs(d) > maxe) maxe = fabs(d);
        }
        double rms = sqrt(se / (double)N);
        printf("  float vs fixed: RMS %.3e  peak %.3e  (final %.6f / %.6f)\n",
               rms, maxe, pf.y, pq.y);
        /* One Q15 LSB is 3.05e-5. Anything within a few LSB is quantisation,
         * not an algorithmic difference. */
        CK(rms < 5e-4, "trajectory RMS error within a few LSB");
        CK(maxe < 3e-3, "peak deviation bounded");
        CK(fabs(pf.y - pq.y) < 2e-4, "same steady state");
        CK(fabs(pq.y - 0.5) < 1e-3, "fixed-point loop reaches its setpoint");
    }

    /* ---- 5. anti-windup actually bounds the integrator ------------------ */
    {
        struct { const char *name; uint8_t mode; double ipeak; double recov; } r[3];
        const uint8_t modes[3] = { (uint8_t)PIDQ_AW_NONE,
                                   (uint8_t)PIDQ_AW_CLAMP,
                                   (uint8_t)PIDQ_AW_BACK_CALC };
        const char *names[3] = { "none", "clamp", "back-calc" };

        for (int m = 0; m < 3; m++) {
            PIDq_Handle h; PIDq_Config c;
            PIDq_ConfigDefault(&c);
            c.kp_q16 = PIDQ_F_TO_Q16(1.0); c.ki_q16 = PIDQ_F_TO_Q16(5.0);
            c.dt_us = 1000;
            c.out_min_q15 = PIDQ_F_TO_Q15(-0.2);
            c.out_max_q15 = PIDQ_F_TO_Q15(0.2);
            c.i_min_q15   = PIDQ_F_TO_Q15(-0.3);
            c.i_max_q15   = PIDQ_F_TO_Q15(0.3);
            c.aw_mode = modes[m]; c.bc_shift = 3;
            PIDq_Init(&h, &c);

            Plant p = {0.0, 1.0, 0.05};
            double ipeak = 0.0;
            /* Drive hard into saturation for 1 s. */
            PIDq_SetSetpoint(&h, PIDQ_F_TO_Q15(0.9));
            for (int i = 0; i < 1000; i++) {
                int16_t y = (int16_t)lround(p.y * 32768.0);
                double u = (double)PIDq_Update(&h, y) / 32768.0;
                plant_step(&p, u, 0.001);
                double iv = fabs((double)PIDq_GetIntegral(&h) / 32768.0);
                if (iv > ipeak) ipeak = iv;
            }
            /* Then ask for something reachable and time the recovery. */
            PIDq_SetSetpoint(&h, PIDQ_F_TO_Q15(0.1));
            int settle = -1;
            for (int i = 0; i < 3000; i++) {
                int16_t y = (int16_t)lround(p.y * 32768.0);
                double u = (double)PIDq_Update(&h, y) / 32768.0;
                plant_step(&p, u, 0.001);
                if (fabs(p.y - 0.1) < 0.005) { settle = i; break; }
            }
            r[m].name = names[m]; r[m].mode = modes[m];
            r[m].ipeak = ipeak;
            r[m].recov = (settle < 0) ? -1.0 : (double)settle * 0.001;
            printf("  AW %-10s I_peak=%.4f  recovery=%s\n", names[m], ipeak,
                   (settle < 0) ? "NEVER" : "yes");
        }
        CK(r[1].ipeak < r[0].ipeak, "clamp bounds the integrator below none");
        CK(r[2].ipeak < r[0].ipeak, "back-calc bounds it below none");
        CK(r[1].recov >= 0.0, "clamp recovers");
        CK(r[2].recov >= 0.0, "back-calc recovers");
        CK(r[2].recov <= r[1].recov, "back-calc recovers no slower than clamp");
    }

    /* ---- 6. bumpless manual -> automatic --------------------------------- */
    {
        PIDq_Handle h; PIDq_Config c;
        PIDq_ConfigDefault(&c);
        c.kp_q16 = PIDQ_F_TO_Q16(2.0); c.ki_q16 = PIDQ_F_TO_Q16(1.0);
        c.dt_us = 1000; c.mode = (uint8_t)PIDQ_MODE_MANUAL;
        PIDq_Init(&h, &c);
        PIDq_SetSetpoint(&h, PIDQ_F_TO_Q15(0.3));
        PIDq_SetManualOutput(&h, PIDQ_F_TO_Q15(0.42));

        int16_t u_man = 0;
        for (int i = 0; i < 50; i++) { u_man = PIDq_Update(&h, PIDQ_F_TO_Q15(0.1)); }
        CK(u_man == PIDQ_F_TO_Q15(0.42), "manual output is held exactly");
        CK(PIDq_GetManualOutput(&h) == PIDQ_F_TO_Q15(0.42),
           "manual output readable while in manual");

        PIDq_SetMode(&h, PIDQ_MODE_AUTOMATIC);
        int16_t u_auto = PIDq_Update(&h, PIDQ_F_TO_Q15(0.1));
        int delta = (int)u_auto - (int)u_man;
        if (delta < 0) delta = -delta;
        /* The first automatic sample also performs one integration step,
         * so the expected delta is Ki*dt*e = 1.0*0.001*0.2 = 2.0e-4 = 6.55 LSB,
         * not zero. This is the same effect already documented for the
         * floating-point core; it is the integrator doing its job, not a bump.
         * What must NOT happen is a jump of the order of the P term. */
        double expect_lsb = 1.0 * 0.001 * 0.2 * 32768.0;
        printf("  bumpless transfer delta = %d LSB (expect ~%.1f = one Ki*dt*e step)\n",
               delta, expect_lsb);
        CK((double)delta <= expect_lsb + 2.0,
           "transfer bump is one integration step, nothing more");
    }

    /* ---- 7. derivative kick and rounding bias ---------------------------- */
    {
        /* A setpoint step must not produce a derivative impulse, because D
         * acts on the measurement. */
        PIDq_Handle h; PIDq_Config c;
        PIDq_ConfigDefault(&c);
        c.kp_q16 = PIDQ_F_TO_Q16(1.0); c.kd_q16 = PIDQ_F_TO_Q16(0.5);
        c.dt_us = 1000; c.tf_us = 2000;
        PIDq_Init(&h, &c);
        PIDq_SetSetpoint(&h, 0);
        for (int i = 0; i < 20; i++) { (void)PIDq_Update(&h, 0); }
        PIDq_SetSetpoint(&h, PIDQ_F_TO_Q15(0.4));
        int16_t u = PIDq_Update(&h, 0);
        /* Pure P response would be 0.4; any large excess is a derivative kick. */
        printf("  setpoint step -> u = %.4f (P alone = 0.4000)\n",
               (double)u / 32768.0);
        CK(fabs((double)u / 32768.0 - 0.4) < 0.01, "no derivative kick");

        /* Rounding must be symmetric: a zero-mean square wave through the
         * controller must not accumulate a DC offset. With truncation the
         * output would drift negative. */
        PIDq_Handle h2; PIDq_Config c2;
        PIDq_ConfigDefault(&c2);
        c2.kp_q16 = PIDQ_F_TO_Q16(0.5); c2.ki_q16 = 0;
        PIDq_Init(&h2, &c2);
        PIDq_SetSetpoint(&h2, 0);
        long sum = 0;
        for (int i = 0; i < 2000; i++) {
            sum += PIDq_Update(&h2, (int16_t)((i & 1) ? 101 : -101));
        }
        printf("  zero-mean input -> output sum = %ld LSB over 2000 samples\n", sum);
        CK(labs(sum) <= 2000, "no systematic rounding bias (< 1 LSB/sample)");
    }

    /* ---- 8. saturating arithmetic, no wraparound ------------------------- */
    {
        /* A wrapped output flips sign, which on a real actuator is full
         * reverse. Push every term to its extreme and confirm clamping. */
        PIDq_Handle h; PIDq_Config c;
        PIDq_ConfigDefault(&c);
        c.kp_q16 = PIDQ_F_TO_Q16(2000.0);
        c.ki_q16 = PIDQ_F_TO_Q16(1000.0);
        c.kd_q16 = PIDQ_F_TO_Q16(10.0);
        c.dt_us = 1000; c.tf_us = 1000;
        c.aw_mode = (uint8_t)PIDQ_AW_NONE;
        /* Assert the init SUCCEEDS. An earlier version of this test used
         * Kd=100, which makes cb = Kd/(Tf+dt) = 5e4 overflow Q16.16; Init
         * correctly refused, the handle stayed invalid, every Update returned
         * 0 and the "no negative output" check passed for the wrong reason. */
        CK(PIDq_Init(&h, &c) == PID_OK, "extreme-gain config initialises");
        PIDq_SetSetpoint(&h, PIDQ_MAX);

        int neg = 0;
        for (int i = 0; i < 5000; i++) {
            int16_t u = PIDq_Update(&h, PIDQ_MIN);
            if (u < 0) neg++;
        }
        printf("  extreme gains: %d negative outputs (expect 0)\n", neg);
        {   /* And the coefficient that genuinely does not fit must be refused
             * at configuration time rather than saturating every sample. */
            PIDq_Handle hb; PIDq_Config cb2;
            PIDq_ConfigDefault(&cb2);
            cb2.kd_q16 = PIDQ_F_TO_Q16(100.0);
            cb2.dt_us = 1000; cb2.tf_us = 1000;
            CK(PIDq_Init(&hb, &cb2) == PID_ERR_INVALID_GAIN,
               "un-representable Kd/(Tf+dt) refused at init");
        }
        CK(neg == 0, "no sign flip from overflow");
        CK(PIDq_GetOutput(&h) == PIDQ_MAX, "pinned at the upper limit");
        CK(PIDq_IsSaturated(&h), "saturation flag set");
    }

    /* ---- 9. runtime gain change: exact refold, bumpless, rollback -------- */
    {
        /* SetGains must refold ci and cb from the STORED dt/Tf. An earlier
         * draft tried to reconstruct dt from the folded coefficients, which is
         * impossible when the old gain was zero (ci = Ki*dt collapses to 0 and
         * carries no scale). dt_us/tf_us are therefore kept in the handle. */
        PIDq_Handle h; PIDq_Config c;
        PIDq_ConfigDefault(&c);
        c.kp_q16 = PIDQ_F_TO_Q16(1.0);
        c.ki_q16 = 0;                       /* start with NO integral action */
        c.kd_q16 = 0;
        c.dt_us  = 1000; c.tf_us = 4000;
        c.aw_mode = (uint8_t)PIDQ_AW_NONE;
        PIDq_Init(&h, &c);
        PIDq_SetSetpoint(&h, PIDQ_F_TO_Q15(0.25));

        for (int i = 0; i < 100; i++) { (void)PIDq_Update(&h, 0); }
        CK(PIDq_GetIntegral(&h) == 0, "no integral action while Ki = 0");

        /* Enable Ki from zero - the case the broken reconstruction could not
         * handle at all. */
        CK(PIDq_SetGains(&h, PIDQ_F_TO_Q16(1.0), PIDQ_F_TO_Q16(2.0), 0) == PID_OK,
           "SetGains from Ki=0 succeeds");
        for (int i = 0; i < 500; i++) { (void)PIDq_Update(&h, 0); }
        /* I should now be Ki*dt*e*N = 2.0*0.001*0.25*500 = 0.25 */
        double got = (double)PIDq_GetIntegral(&h) / 32768.0;
        printf("  after enabling Ki: I = %.5f (expect 0.25000)\n", got);
        CK(fabs(got - 0.25) < 0.002, "refolded ci is numerically exact");

        /* Gain change must be bumpless: the integrator holds output units, so
         * changing Kp must move the output by exactly the change in the P term
         * and nothing else. */
        PIDq_Handle g; PIDq_Config cg;
        PIDq_ConfigDefault(&cg);
        cg.kp_q16 = PIDQ_F_TO_Q16(1.0); cg.ki_q16 = PIDQ_F_TO_Q16(1.0);
        cg.dt_us = 1000;
        PIDq_Init(&g, &cg);
        PIDq_SetSetpoint(&g, PIDQ_F_TO_Q15(0.2));
        for (int i = 0; i < 300; i++) { (void)PIDq_Update(&g, 0); }
        int16_t before = PIDq_GetOutput(&g);
        int16_t i_before = PIDq_GetIntegral(&g);
        PIDq_SetGains(&g, PIDQ_F_TO_Q16(3.0), PIDQ_F_TO_Q16(1.0), 0);
        int16_t after = PIDq_Update(&g, 0);
        CK(PIDq_GetIntegral(&g) != 0, "integrator survives the gain change");
        /* Expected jump = (3-1)*e = 2*0.2 = 0.4, plus one integration step. */
        double jump = ((double)after - (double)before) / 32768.0;
        printf("  Kp 1->3 at e=0.2: output jump %.4f (expect 0.4000), I kept %.4f\n",
               jump, (double)i_before / 32768.0);
        CK(fabs(jump - 0.4) < 0.005, "jump is exactly the P-term change");

        /* Rejected gains must leave the controller untouched, not half-applied. */
        int32_t bad_kd = PIDQ_F_TO_Q16(100.0);
        PIDq_Handle r; PIDq_Config cr;
        PIDq_ConfigDefault(&cr);
        cr.kp_q16 = PIDQ_F_TO_Q16(1.0); cr.kd_q16 = PIDQ_F_TO_Q16(0.01);
        cr.dt_us = 1000; cr.tf_us = 1000;
        PIDq_Init(&r, &cr);
        PIDq_SetSetpoint(&r, PIDQ_F_TO_Q15(0.1));
        int16_t u_ref = 0;
        for (int i = 0; i < 10; i++) { u_ref = PIDq_Update(&r, 0); }
        CK(PIDq_SetGains(&r, PIDQ_F_TO_Q16(1.0), 0, bad_kd) == PID_ERR_INVALID_GAIN,
           "un-representable Kd refused by SetGains");
        int16_t u_after = PIDq_Update(&r, 0);
        CK(u_after == u_ref, "rejected SetGains left the controller unchanged");
        CK(PIDq_SetGains(&r, -1, 0, 0) == PID_ERR_INVALID_GAIN,
           "negative gain refused");
    }

    printf("\nfixed-point: %d passed, %d failed\n", pass, bad);
    return bad != 0;
}

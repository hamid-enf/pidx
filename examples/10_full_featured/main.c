/**
 * Example 10 - Everything at once: the configuration reference
 * ============================================================
 * A single realistic application exercising every subsystem, meant to be read
 * as a checklist of what PIDX can be told to do and why you would tell it.
 *
 * THE APPLICATION
 *   A temperature process with a strongly nonlinear gain (radiative loss),
 *   an operator who switches between manual and automatic, a sensor that
 *   sometimes fails, and a supervisor that re-tunes on demand.
 *
 * COVERED HERE
 *   gain scheduling . manual/auto/hold with bumpless transfer . sensor safety
 *   and fail-safe . setpoint shaping . standalone filters . 2DOF weighting
 *   . integral deadband . runtime gain changes . diagnostics . fixed-point
 *   comparison . PID_Deinit
 *
 * Covered by the earlier examples and not repeated: anti-windup comparison
 * (02), derivative kick (03), trajectory shaping (04), the fast path (05),
 * cascade (06), auto-tuning (07), feedforward and RTOS (08), telemetry (09).
 */
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "pidx/pid.h"
#include "pidx/pid_gainsched.h"
#include "pidx/pid_filter.h"
#include "pidx/pid_fixed.h"
#include "ex_plant.h"
#include "ex_report.h"

#define DT 0.5
#define N  4000                    /* 2000 s */

int main(void)
{
    printf("Example 10 - Full-featured configuration reference\n\n");

    /* ==================================================================== */
    /* 1. Gain scheduling                                                    */
    /* ==================================================================== */
    printf("[1] GAIN SCHEDULING\n");
    printf("    The heater's radiative loss makes the process gain fall as it\n"
           "    gets hotter, so one gain set cannot be right everywhere.\n\n");
    {
        /* Measured open-loop gain of EX_Heater, dT/du at steady state:
         * high at low temperature, much lower once radiation dominates.
         * The controller gains therefore rise with temperature. */
        /* The plant's own numbers set this table. Steady-state duty needed:
         *   150 C -> 0.448,  200 C -> 0.664,  250 C -> 0.913
         * so dT/du falls from ~330 C/unit near ambient to ~200 C/unit at
         * 250 C. Controller gains rise to compensate. Full scale is reached
         * at about 266 C, which is why the run below targets 240 C and not
         * something prettier - a setpoint the actuator cannot hold makes
         * every controller look identical, because they all just saturate. */
        static const PID_GainPoint pts[4] = {
            /*  x=degC     kp      ki      kd  */
            {  40.0f,   0.030f, 0.0012f, 0.20f },
            { 100.0f,   0.042f, 0.0017f, 0.28f },
            { 180.0f,   0.062f, 0.0025f, 0.42f },
            { 250.0f,   0.095f, 0.0038f, 0.63f }
        };
        PID_GainSchedule sched;
        PID_Float kp, ki, kd;
        int k;

        if (PID_GainSched_Init(&sched, pts, 4U,
                               PID_SCHED_SRC_MEASUREMENT,
                               PID_SCHED_INTERP_SMOOTH) != PID_OK) {
            printf("    schedule rejected\n");
            return 1;
        }
        /* Hysteresis on the scheduling variable stops the gains dithering
         * when the measurement noise straddles a breakpoint. */
        (void)PID_GainSched_SetHysteresis(&sched, 2.0f);

        printf("    %10s %10s %10s %10s\n", "T [C]", "Kp", "Ki", "Kd");
        printf("    %10s %10s %10s %10s\n",
               "----------", "----------", "----------", "----------");
        for (k = 0; k <= 7; k++) {
            PID_Float x = (PID_Float)(20.0 + ((double)k * 40.0));
            (void)PID_GainSched_Evaluate(&sched, x, &kp, &ki, &kd);
            printf("    %10.1f %10.4f %10.5f %10.4f%s\n",
                   (double)x, (double)kp, (double)ki, (double)kd,
                   ((double)x < 40.0) ? "   (clamped to the first point)" :
                   (((double)x > 250.0) ? "   (clamped to the last point)"
                                        : ""));
        }
        printf("\n    SMOOTH interpolation is C1 continuous, so the gains have\n"
               "    no corner at a breakpoint. Outside the table the endpoint\n"
               "    values are held - extrapolating gains is never safe.\n");

        /* --- scheduled vs fixed, measured --- */
        {
            EX_Step m_fix, m_sch;
            int mode;

            printf("\n    Wide-range run, 20 -> 240 C"
                   " (full scale holds ~266 C):\n\n");
            ex_step_header();

            for (mode = 0; mode < 2; mode++) {
                PID_Handle pid;
                PID_Config c;
                EX_Heater  p;
                EX_Step   *m = (mode == 0) ? &m_fix : &m_sch;
                int i;

                ex_heater_init(&p, 20.0);
                PID_ConfigDefault(&c);
                /* The fixed case uses the gains that are right in the middle
                 * of the range - the best a single set can do. */
                c.core.kp = 0.042f;
                c.core.ki = 0.0017f;
                c.core.kd = 0.28f;
                c.core.sample_time = (PID_Float)DT;
                c.limits.use_output_limits = true;
                c.limits.output_min = 0.0f;
                c.limits.output_max = 1.0f;
                c.integral.mode = PID_AW_BACK_CALCULATION;
                c.integral.kt = 1.0f;
                c.filter.tf = 2.0f;
                c.filter.n_filter = 0.0f;
                (void)PID_Init(&pid, &c);

                if (mode == 1) {
                    if (PID_GainSched_Attach(&pid, &sched) != PID_OK) {
                        printf("    ATTACH FAILED\n");
                    }
                }
                PID_SetSetpoint(&pid, 240.0f);
                ex_step_init(m, 20.0, 240.0, 0.02);

                for (i = 0; i < N; i++) {
                    PID_Float u = PID_Update(&pid, (PID_Float)p.t);
                    (void)ex_heater_step(&p, (double)u, DT);
                    ex_step_update(m, p.t, (double)u, DT);
                }
            }
            ex_step_report(&m_fix, "fixed gains (mid-range)");
            ex_step_report(&m_sch, "gain scheduled");
        }
    }

    /* ==================================================================== */
    /* 2. Manual / automatic / hold, bumpless                                */
    /* ==================================================================== */
    printf("\n[2] MODES AND BUMPLESS TRANSFER\n");
    {
        PID_Handle pid;
        PID_Config c;
        EX_Heater  p;
        double u_before, u_after;
        int i;

        /*
         * Bumpless transfer works by back-solving the integrator every sample
         * while in MANUAL, so that I = u_manual - P - D - FF at all times.
         * At the instant of transfer the controller therefore already
         * reproduces the operator's output exactly.
         *
         * That identity has one precondition, and it is worth stating rather
         * than hiding: the required I must be inside the integrator bounds.
         * Both cases are shown below.
         */

        /* --- case A: transfer near the setpoint. The normal situation. --- */
        ex_heater_init(&p, 20.0);
        PID_ConfigDefault(&c);
        c.core.kp = 0.045f;
        c.core.ki = 0.0018f;
        c.core.sample_time = (PID_Float)DT;
        c.limits.use_output_limits = true;
        c.limits.output_min = 0.0f;
        c.limits.output_max = 1.0f;
        c.core.mode = PID_MODE_MANUAL;
        (void)PID_Init(&pid, &c);

        /* Operator holds 40% duty and lets the process come to rest there. */
        (void)PID_SetManualOutput(&pid, 0.40f);
        for (i = 0; i < 2000; i++) {
            PID_Float u = PID_Update(&pid, (PID_Float)p.t);
            (void)ex_heater_step(&p, (double)u, DT);
        }
        /* Then sets the target to where the process actually is. */
        PID_SetSetpoint(&pid, (PID_Float)p.t);
        (void)PID_Update(&pid, (PID_Float)p.t);
        u_before = (double)PID_GetOutput(&pid);

        (void)PID_SetMode(&pid, PID_MODE_AUTOMATIC);
        u_after = (double)PID_Update(&pid, (PID_Float)p.t);

        printf("    A) transfer at the operating point (T = %.1f C,"
               " SP = %.1f C):\n", p.t, p.t);
        printf("       u %.6f -> %.6f     jump %.2e   %s\n",
               u_before, u_after, fabs(u_after - u_before),
               (fabs(u_after - u_before) < 1e-4) ? "bumpless" : "BUMP");

        /* --- case B: transfer far from the setpoint. --- */
        {
            PID_Handle pid2;
            PID_Status st;
            double y = 100.0;

            c.core.mode = PID_MODE_MANUAL;
            (void)PID_Init(&pid2, &c);
            PID_SetSetpoint(&pid2, 150.0f);        /* 50 C away */
            (void)PID_SetManualOutput(&pid2, 0.40f);

            /* Clear first, then run one more manual sample. The back-solve
             * runs on every MANUAL update, so the warning is raised BEFORE
             * the mode change - which is the useful moment: it tells you the
             * transfer would bump while you can still decide not to do it. */
            (void)PID_ClearError(&pid2);
            for (i = 0; i < 50; i++) { (void)PID_Update(&pid2, (PID_Float)y); }

            (void)PID_GetStatus(&pid2, &st);
            u_before = (double)PID_GetOutput(&pid2);
            printf("\n    B) transfer 50 C below the setpoint:\n");
            printf("       while still in MANUAL, last error = %s\n",
                   PID_StatusToString(PID_PeekLastError(&pid2)));
            printf("       flags 0x%04X (INTEGRAL_LIMITED = %s)\n",
                   (unsigned)PID_GetFlags(&pid2),
                   ((PID_GetFlags(&pid2) & PID_FLAG_INTEGRAL_LIMITED) != 0U)
                       ? "set" : "clear");

            (void)PID_SetMode(&pid2, PID_MODE_AUTOMATIC);
            u_after = (double)PID_Update(&pid2, (PID_Float)y);

            printf("       P term alone is Kp*e = %.4f, but the actuator\n"
                   "       maxes out at 1.0, so reproducing u = 0.40 would\n"
                   "       need I = %.4f - outside the integrator bounds\n"
                   "       [%.1f, %.1f].\n",
                   (double)st.p_term, 0.40 - (double)st.p_term, 0.0, 1.0);
            printf("       u %.6f -> %.6f     jump %.2e\n",
                   u_before, u_after, fabs(u_after - u_before));
            printf("\n       This is not a defect to paper over. The request\n"
                   "       is arithmetically impossible, so the library reports\n"
                   "       it rather than silently stepping the actuator: the\n"
                   "       back-solve raises PID_FLAG_INTEGRAL_LIMITED and\n"
                   "       records a sticky error on every manual sample where\n"
                   "       the value is unreachable. Poll it before switching:\n"
                   "       if it is set, the transfer WILL bump, and you can\n"
                   "       move the setpoint to the measurement first instead\n"
                   "       - which is exactly what case A does.\n");
        }

        /* --- HOLD --- */
        {
            PID_Float i_before, i_after;
            for (i = 0; i < 200; i++) {
                (void)PID_Update(&pid, (PID_Float)p.t);
                (void)ex_heater_step(&p, (double)PID_GetOutput(&pid), DT);
            }
            i_before = PID_GetIntegrator(&pid);
            (void)PID_SetMode(&pid, PID_MODE_HOLD);
            for (i = 0; i < 200; i++) {
                (void)PID_Update(&pid, (PID_Float)p.t);
                (void)ex_heater_step(&p, (double)PID_GetOutput(&pid), DT);
            }
            i_after = PID_GetIntegrator(&pid);
            printf("\n    HOLD for 100 s: integrator %.6f -> %.6f (frozen: %s)\n",
                   (double)i_before, (double)i_after,
                   (i_before == i_after) ? "yes" : "NO");
            printf("    P and D keep working; only the integrator is frozen.\n"
                   "    Use it while an upstream loop is saturated or a sensor\n"
                   "    is being serviced.\n");
            (void)PID_SetMode(&pid, PID_MODE_AUTOMATIC);
        }
    }

    /* ==================================================================== */
    /* 3. Sensor safety and fail-safe                                        */
    /* ==================================================================== */
    printf("\n[3] SENSOR SAFETY\n");
    {
        PID_Handle pid;
        PID_Config c;
        EX_Heater  p;
        int i;
        int faulted_at = -1, recovered_at = -1;

        ex_heater_init(&p, 20.0);
        PID_ConfigDefault(&c);
        c.core.kp = 0.045f;
        c.core.ki = 0.0018f;
        c.core.sample_time = (PID_Float)DT;
        c.limits.use_output_limits = true;
        c.limits.output_min = 0.0f;
        c.limits.output_max = 1.0f;

        c.safety.enabled         = true;
        c.safety.meas_min        = -20.0f;   /* below this: open thermocouple */
        c.safety.meas_max        = 500.0f;   /* above this: shorted / broken  */
        c.safety.meas_rate_max   = 50.0f;    /* C per second, physically max  */
        c.safety.failsafe_output = 0.0f;     /* heater OFF on fault           */
        c.safety.fault_persist_n = 3U;       /* ignore single-sample glitches */
        c.safety.auto_recover    = true;
        (void)PID_Init(&pid, &c);
        PID_SetSetpoint(&pid, 200.0f);

        for (i = 0; i < 1200; i++) {
            /* A disconnected thermocouple reads full scale between t=300 s
             * and t=400 s. */
            double reading = ((i >= 600) && (i < 800)) ? 9999.0 : p.t;
            PID_Float u = PID_Update(&pid, (PID_Float)reading);

            (void)ex_heater_step(&p, (double)u, DT);

            if (PID_IsFaulted(&pid) && (faulted_at < 0)) { faulted_at = i; }
            if ((faulted_at >= 0) && !PID_IsFaulted(&pid)
                && (recovered_at < 0) && (i > faulted_at)) {
                recovered_at = i;
            }
        }
        printf("    sensor fails at sample 600, recovers at 800\n");
        printf("    fault latched at sample %d (after %d bad samples,"
               " as configured)\n", faulted_at, faulted_at - 600 + 1);
        printf("    auto-recovered at sample %d\n", recovered_at);
        printf("    final temperature %.1f C, last error: %s\n",
               p.t, PID_StatusToString(PID_PeekLastError(&pid)));
        printf("\n    fault_persist_n = 3 means a single noisy sample cannot\n"
           "    trip the fail-safe, but a real failure trips it in 1.5 s.\n"
           "    Recovery is bumpless: the integrator is back-solved to the\n"
           "    fail-safe output first, so the heater does not slam back on.\n");
    }

    /* ==================================================================== */
    /* 4. Standalone filters                                                 */
    /* ==================================================================== */
    printf("\n[4] STANDALONE FILTERS (usable without a controller)\n");
    {
        PID_LPF1        lpf;
        PID_MovingAvg   avg;
        PID_Median3     med;
        PID_RateLimiter rl;
        PID_Float       avg_buf[8];
        double raw_energy = 0.0, lpf_energy = 0.0;
        double avg_energy = 0.0, med_energy = 0.0;
        int i;

        (void)PID_LPF1_Init(&lpf, 2.0f, (PID_Float)DT);
        (void)PID_MovingAvg_Init(&avg, avg_buf, 8U);
        (void)PID_Median3_Init(&med);
        (void)PID_RateLimiter_Init(&rl, 5.0f);
        (void)PID_RateLimiter_Reset(&rl, 0.0f);

        ex_noise_seed(99U);
        for (i = 0; i < 2000; i++) {
            double clean = 100.0;
            double spike = ((i % 250) == 0) ? 80.0 : 0.0;   /* impulse noise */
            double x = clean + ex_noise_gauss(1.0) + spike;

            double y_lpf = (double)PID_LPF1_Update(&lpf, (PID_Float)x);
            double y_avg = (double)PID_MovingAvg_Update(&avg, (PID_Float)x);
            double y_med = (double)PID_Median3_Update(&med, (PID_Float)x);

            if (i > 50) {
                raw_energy += (x - clean) * (x - clean);
                lpf_energy += (y_lpf - clean) * (y_lpf - clean);
                avg_energy += (y_avg - clean) * (y_avg - clean);
                med_energy += (y_med - clean) * (y_med - clean);
            }
        }
        printf("    Gaussian noise (sigma 1.0) plus an 80-unit spike every"
               " 125 s:\n");
        printf("      %-22s RMS error %8.4f\n", "raw",
               sqrt(raw_energy / 1950.0));
        printf("      %-22s RMS error %8.4f\n", "LPF1 (tau = 2 s)",
               sqrt(lpf_energy / 1950.0));
        printf("      %-22s RMS error %8.4f\n", "MovingAvg (8)",
               sqrt(avg_energy / 1950.0));
        printf("      %-22s RMS error %8.4f\n", "Median3",
               sqrt(med_energy / 1950.0));
        printf("\n    Median3 is the only one that REMOVES an impulse rather\n"
               "    than smearing it: a linear filter spreads a spike over its\n"
               "    whole impulse response. Cascade Median3 into an LPF when\n"
               "    you have both impulse and Gaussian noise.\n");

        /* Rate limiter on a step. */
        {
            PID_Float v = 0.0f;
            int steps = 0;
            for (i = 0; i < 100; i++) {
                v = PID_RateLimiter_Update(&rl, 100.0f, (PID_Float)DT);
                steps++;
                if (v >= 99.9f) { break; }
            }
            printf("\n    RateLimiter at 5 units/s took %.1f s to cross a\n"
                   "    100-unit step (expected 20.0 s).\n",
                   (double)steps * DT);
        }
    }

    /* ==================================================================== */
    /* 5. 2DOF weighting and integral deadband                               */
    /* ==================================================================== */
    printf("\n[5] SETPOINT WEIGHTING AND INTEGRAL DEADBAND\n");
    {
        const PID_Float betas[3] = { 1.0f, 0.6f, 0.2f };
        int k;

        ex_step_header();
        for (k = 0; k < 3; k++) {
            PID_Handle pid;
            PID_Config c;
            EX_Heater  p;
            EX_Step    m;
            char lbl[32];
            int i;

            ex_heater_init(&p, 20.0);
            PID_ConfigDefault(&c);
            c.core.kp = 0.075f;
            c.core.ki = 0.0030f;
            c.core.kd = 0.50f;
            c.core.sample_time = (PID_Float)DT;
            c.limits.use_output_limits = true;
            c.limits.output_min = 0.0f;
            c.limits.output_max = 1.0f;
            c.integral.mode = PID_AW_BACK_CALCULATION;
            c.integral.kt = 1.0f;
            c.filter.tf = 2.0f;
            c.filter.n_filter = 0.0f;
            c.weight.beta = betas[k];
            (void)PID_Init(&pid, &c);
            PID_SetSetpoint(&pid, 200.0f);
            ex_step_init(&m, 20.0, 200.0, 0.02);

            for (i = 0; i < N; i++) {
                PID_Float u = PID_Update(&pid, (PID_Float)p.t);
                (void)ex_heater_step(&p, (double)u, DT);
                ex_step_update(&m, p.t, (double)u, DT);
            }
            snprintf(lbl, sizeof(lbl), "beta = %.1f", (double)betas[k]);
            ex_step_report(&m, lbl);
        }

        /*
         * Integral deadband: |e| below the band stops the integrator.
         *
         * Measured on the INTEGRATOR's peak-to-peak range, not on actuator
         * travel. The first version of this table measured travel and showed
         * almost no effect (41.77 -> 41.23), which is a true measurement of
         * the wrong quantity: the P term passes sensor noise straight to the
         * output and dominates the travel figure, and no integral setting can
         * change that. What the deadband governs is whether the INTEGRATOR
         * wanders, which is what causes slow limit-cycling around setpoint.
         */
        {
            const PID_Float dbs[3] = { 0.0f, 0.5f, 2.0f };
            int k2;

            printf("\n    Integral deadband, measured over the last 1000 s\n"
                   "    with a sigma = 0.2 C sensor:\n\n");
            printf("      %-14s %16s %14s %12s\n",
                   "deadband [C]", "integrator range", "u travel",
                   "final T [C]");
            printf("      %-14s %16s %14s %12s\n",
                   "--------------", "----------------", "--------------",
                   "------------");

            for (k2 = 0; k2 < 3; k2++) {
                PID_Handle pid;
                PID_Config c;
                EX_Heater  p;
                double travel = 0.0, uprev = 0.0;
                double i_lo = 1e30, i_hi = -1e30;
                int i;
                char lbl[24];

                ex_heater_init(&p, 20.0);
                ex_noise_seed(2024U);      /* same noise for all three */
                PID_ConfigDefault(&c);
                c.core.kp = 0.075f;
                c.core.ki = 0.0030f;
                c.core.sample_time = (PID_Float)DT;
                c.limits.use_output_limits = true;
                c.limits.output_min = 0.0f;
                c.limits.output_max = 1.0f;
                c.integral.deadband = dbs[k2];
                (void)PID_Init(&pid, &c);
                PID_SetSetpoint(&pid, 200.0f);

                for (i = 0; i < N; i++) {
                    PID_Float u = PID_Update(&pid, (PID_Float)(p.t
                                              + ex_noise_gauss(0.2)));
                    (void)ex_heater_step(&p, (double)u, DT);
                    if (i > 2000) {
                        double ig = (double)PID_GetIntegrator(&pid);
                        travel += fabs((double)u - uprev);
                        if (ig < i_lo) { i_lo = ig; }
                        if (ig > i_hi) { i_hi = ig; }
                    }
                    uprev = (double)u;
                }
                if (dbs[k2] == 0.0f) {
                    snprintf(lbl, sizeof(lbl), "0.0 (off)");
                } else {
                    snprintf(lbl, sizeof(lbl), "%.1f", (double)dbs[k2]);
                }
                printf("      %-14s %16.5f %14.2f %12.3f\n",
                       lbl, i_hi - i_lo, travel, p.t);
            }
            printf("\n    This table is NOT monotonic, and the middle row is\n"
                   "    the interesting one: a 0.5 C band leaves the\n"
                   "    integrator wandering MORE than no band at all\n"
                   "    (0.00457 against 0.00280).\n\n"
                   "    The reason is worth understanding before reaching for\n"
                   "    this feature. Zero-mean sensor noise integrates to\n"
                   "    almost nothing on its own, because the positive and\n"
                   "    negative samples cancel. A deadband destroys that\n"
                   "    cancellation: it discards the small errors of BOTH\n"
                   "    signs and admits only the tail of the distribution, so\n"
                   "    whichever side the residual offset sits on gets\n"
                   "    integrated unopposed. A partial deadband can therefore\n"
                   "    be worse than none.\n\n"
                   "    At 2.0 C the error never leaves the band, the\n"
                   "    integrator stops completely, and the loop parks where\n"
                   "    P alone puts it - 1.3 C off target.\n\n"
                   "    Conclusion: the integral deadband is for a QUANTISED\n"
                   "    or sticky actuator, where the goal is to stop\n"
                   "    commanding changes smaller than the actuator can make.\n"
                   "    It is the wrong tool for sensor noise; filter the\n"
                   "    measurement (input_lpf_tau) for that. Actuator travel\n"
                   "    hardly moves in any row precisely because the P term\n"
                   "    passes the noise straight through.\n");
        }
    }

    /* ==================================================================== */
    /* 6. Runtime gain changes                                               */
    /* ==================================================================== */
    printf("\n[6] RUNTIME GAIN CHANGES\n");
    {
        PID_Handle pid;
        PID_Config c;
        EX_Heater  p;
        double u1, u2, u3;
        int i;

        ex_heater_init(&p, 20.0);
        PID_ConfigDefault(&c);
        c.core.kp = 0.045f;
        c.core.ki = 0.0018f;
        c.core.sample_time = (PID_Float)DT;
        c.limits.use_output_limits = true;
        c.limits.output_min = 0.0f;
        c.limits.output_max = 1.0f;
        (void)PID_Init(&pid, &c);
        PID_SetSetpoint(&pid, 150.0f);

        for (i = 0; i < 1500; i++) {
            PID_Float u = PID_Update(&pid, (PID_Float)p.t);
            (void)ex_heater_step(&p, (double)u, DT);
        }
        u1 = (double)PID_GetOutput(&pid);

        /* The integrator stores the I term in OUTPUT units, so changing Ki
         * does not move the output at all. */
        (void)PID_SetGains(&pid, 0.090f, 0.0036f, 0.0f);
        u2 = (double)PID_Update(&pid, (PID_Float)p.t);

        /* PID_SetGainsRescaleIntegral is for the other convention: it keeps
         * Ki * integral constant instead. */
        (void)PID_SetGainsRescaleIntegral(&pid, 0.045f, 0.0018f, 0.0f);
        u3 = (double)PID_Update(&pid, (PID_Float)p.t);

        printf("    settled output            %.6f\n", u1);
        printf("    after doubling Kp and Ki  %.6f  (jump %.2e)\n",
               u2, fabs(u2 - u1));
        printf("    after rescaling back      %.6f\n", u3);
        printf("\n    The integrator holds the I term in OUTPUT units, so a\n"
               "    gain change is inherently bumpless - no special case, no\n"
               "    scaling step. That is decision (1) of the core design.\n");
    }

    /* ==================================================================== */
    /* 7. Fixed-point comparison                                             */
    /* ==================================================================== */
    printf("\n[7] FIXED-POINT (PIDq_*): no FPU required\n");
    {
        PIDq_Handle qh;
        PIDq_Config qc;
        PID_Handle  fh;
        PID_Config  fc;
        double rms = 0.0, peak = 0.0;
        int i;

        if (!PIDq_SelfTest()) {
            printf("    PIDq_SelfTest FAILED on this compiler\n");
        } else {
            printf("    PIDq_SelfTest passed (arithmetic right shift"
                   " confirmed)\n");
        }

        (void)PIDq_ConfigDefault(&qc);
        qc.kp_q16 = (int32_t)(1.2 * 65536.0);
        qc.ki_q16 = (int32_t)(3.0 * 65536.0);
        qc.kd_q16 = (int32_t)(0.02 * 65536.0);
        qc.dt_us  = 1000U;
        qc.tf_us  = 5000U;
        qc.out_min_q15 = -32768;
        qc.out_max_q15 =  32767;
        qc.aw_mode = PIDQ_AW_CLAMP;
        if (PIDq_Init(&qh, &qc) != PID_OK) {
            printf("    fixed-point init failed\n");
        } else {
            PID_ConfigDefault(&fc);
            fc.core.kp = 1.2f;
            fc.core.ki = 3.0f;
            fc.core.kd = 0.02f;
            fc.core.sample_time = 0.001f;
            fc.limits.use_output_limits = true;
            fc.limits.output_min = -1.0f;
            fc.limits.output_max =  1.0f;
            fc.integral.mode = PID_AW_CLAMP;
            fc.filter.tf = 0.005f;
            fc.filter.n_filter = 0.0f;
            (void)PID_Init(&fh, &fc);

            (void)PIDq_SetSetpoint(&qh, 16384);        /* 0.5 in Q15 */
            PID_SetSetpoint(&fh, 0.5f);

            {
                double yf = 0.0;
                for (i = 0; i < 4000; i++) {
                    int16_t yq = (int16_t)(yf * 32768.0);
                    int16_t uq = PIDq_Update(&qh, yq);
                    PID_Float uf = PID_Update(&fh, (PID_Float)yf);
                    double d = fabs(((double)uq / 32768.0) - (double)uf);

                    rms += d * d;
                    if (d > peak) { peak = d; }
                    /* Both controllers drive the same reference plant. */
                    yf += (0.001 / 0.05) * ((double)uf - yf);
                }
            }
            {
                /* One Q15 LSB is 1/32768 = 3.05e-5. State the error in LSB
                 * as well as in absolute terms - an absolute number alone
                 * says nothing about whether it is quantisation or an
                 * algorithmic difference. */
                double rms_v = sqrt(rms / 4000.0);
                printf("    float vs Q15/Q30 over 4000 samples:\n");
                printf("      RMS  %.3e  = %5.2f LSB\n",
                       rms_v, rms_v * 32768.0);
                printf("      peak %.3e  = %5.2f LSB\n",
                       peak, peak * 32768.0);
                printf("    A few LSB of divergence on a closed loop is\n"
                       "    quantisation, not a different algorithm: the two\n"
                       "    controllers drive independent plant copies here,\n"
                       "    so rounding differences accumulate through the\n"
                       "    feedback. Driven open-loop from the same input the\n"
                       "    agreement is ~0.05 LSB (see tests/test_fixed.c).\n");
            }
            printf("    Q15 in and out, Q30 internally. Q15 internals would\n"
                   "    stall the integrator at zero for small errors -\n"
                   "    Ki*dt*e rounds away below one LSB.\n");
            printf("    Not supported by PIDq_*, and removed rather than\n"
                   "    stubbed: auto-tune, interpolated gain scheduling,\n"
                   "    setpoint weighting, shaper, feedforward.\n");
            (void)PIDq_Deinit(&qh);
        }
    }

    /* ==================================================================== */
    /* 8. Deinit                                                             */
    /* ==================================================================== */
    printf("\n[8] DEINIT\n");
    {
        PID_Handle pid;
        PID_Config c;
        PID_Float u;

        PID_ConfigDefault(&c);
        c.core.kp = 1.0f;
        c.core.sample_time = (PID_Float)DT;
        (void)PID_Init(&pid, &c);
        PID_SetSetpoint(&pid, 10.0f);
        (void)PID_Update(&pid, 0.0f);

        (void)PID_Deinit(&pid);
        u = PID_Update(&pid, 0.0f);

        printf("    after PID_Deinit, PID_Update returns %.1f\n", (double)u);
        printf("    PID_SetSetpoint on the dead handle returns %s\n",
               PID_StatusToString(PID_SetSetpoint(&pid, 1.0f)));
        printf("    PID_GetStatus  on the dead handle returns %s\n",
               PID_StatusToString(PID_GetStatus(&pid, NULL)));
        printf("\n    Deinit clears the magic word, so every entry point\n"
               "    rejects the handle with PID_ERR_NOT_INIT instead of\n"
               "    running on stale state. Note that PID_Update returns 0\n"
               "    and cannot report through the sticky error channel - that\n"
               "    channel lives in the handle it just refused to trust. Use\n"
               "    the return code of any status-returning call, or\n"
               "    PID_UpdateEx, which takes an out-parameter for exactly\n"
               "    this reason.\n");
        printf("    Nothing is freed: the library never allocated anything.\n");
    }

    printf("\n%s\n", "----------------------------------------------------");
    printf("PIDX version %s\n", PID_GetVersion());
    printf("Struct sizes on this build: PID_Handle %u B, PID_Config %u B,\n"
           "  PID_Status %u B, PIDq_Handle %u B\n",
           (unsigned)sizeof(PID_Handle), (unsigned)sizeof(PID_Config),
           (unsigned)sizeof(PID_Status), (unsigned)sizeof(PIDq_Handle));
    printf("All memory is caller-owned; the library calls no allocator.\n");

    return 0;
}

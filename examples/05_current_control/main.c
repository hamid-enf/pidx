/**
 * Example 05 - Fast current loop and the fast path
 * ================================================
 * A 20 kHz inner loop is the case where per-sample cost actually matters.
 * This example shows PID_UpdateFast(), what it skips, how to check that
 * skipping is safe, and how much it costs to be wrong about that.
 *
 * WHAT THIS SHOWS
 *   - PID_UpdateFast() and PID_UpdateFast_IsSafe()
 *   - a numerical proof that the fast path and the full path agree bit for bit
 *     when the preconditions hold, and diverge when they do not
 *   - relative host timing of the two paths (NOT a Cortex-M cycle count)
 *   - tuning an electrical loop: the plant is a first-order L/R lag, so the
 *     right answer is analytic rather than experimental
 *
 * HONEST SCOPE OF THE TIMING NUMBERS
 *   The times below are host wall-clock on a general-purpose OS. They are
 *   valid for the relative comparison "does the fast path cost less" and
 *   worthless as absolute figures. For real per-sample cycle counts on your
 *   target, run bench/bench_dwt.c on the hardware.
 */
#include <stdio.h>
#include <math.h>
#include <time.h>

#include "pidx/pid.h"
#include "ex_plant.h"
#include "ex_report.h"

#define FS     20000.0            /* 20 kHz current loop */
#define DT     (1.0 / FS)
#define N      4000               /* 200 ms */
#define V_MAX  24.0

/* Motor electrical subsystem seen by the current loop: L di/dt = v - R*i - Ke*w
 * At 20 kHz the shaft speed is essentially constant across a sample, so the
 * back-EMF term is a slowly varying disturbance - exactly the thing integral
 * action is for. */
#define L_H    0.5e-3
#define R_OHM  1.0
#define KE     0.05

/**
 * Analytic tuning for a first-order electrical plant.
 *
 * Plant: i(s)/v(s) = 1/(L*s + R). With a PI controller and pole-zero
 * cancellation (Ti = L/R) the closed loop becomes first order with bandwidth
 * wc = Kp/L, so:
 *      Kp = wc * L
 *      Ki = Kp * R / L = wc * R
 * No experiment needed - the plant is known from the datasheet. Choosing
 * wc = 2*pi*2000 gives a 2 kHz current loop, a decade below the 20 kHz
 * sampling rate, which is the usual rule for keeping discretisation effects
 * out of the design.
 */
static void analytic_gains(double bw_hz, double *kp, double *ki)
{
    double wc = 2.0 * 3.14159265358979 * bw_hz;
    *kp = wc * L_H;
    *ki = wc * R_OHM;
}

int main(void)
{
    double kp_d, ki_d;

    analytic_gains(2000.0, &kp_d, &ki_d);

    printf("Example 05 - Current control at %.0f kHz\n", FS / 1000.0);
    printf("  plant: L = %.1f mH, R = %.1f ohm  ->  tau_e = %.3f ms\n",
           L_H * 1000.0, R_OHM, (L_H / R_OHM) * 1000.0);
    printf("  analytic PI for a 2 kHz bandwidth: Kp = wc*L = %.4f,"
           " Ki = wc*R = %.1f\n", kp_d, ki_d);
    printf("  (pole-zero cancellation: Ti = L/R = %.3f ms)\n\n",
           (L_H / R_OHM) * 1000.0);

    /* ------------------------------------------------------------------ */
    /* 1. Is the fast path safe for this configuration?                    */
    /* ------------------------------------------------------------------ */
    printf("  [1] PID_UpdateFast_IsSafe() on four configurations:\n");
    {
        PID_Handle h;
        PID_Config c;

        /* (a) Plain PI with output limits: the fast path's home ground. */
        PID_ConfigDefault(&c);
        c.core.kp = (PID_Float)kp_d;
        c.core.ki = (PID_Float)ki_d;
        c.core.sample_time = (PID_Float)DT;
        c.limits.use_output_limits = true;
        c.limits.output_min = (PID_Float)(-V_MAX);
        c.limits.output_max = (PID_Float)( V_MAX);
        c.integral.mode = PID_AW_CLAMP;
        (void)PID_Init(&h, &c);
        printf("      %-42s %s\n", "PI + output limits + clamp AW",
               PID_UpdateFast_IsSafe(&h) ? "SAFE" : "not safe");

        /* (b) Add sensor safety checks: the fast path skips them, so using it
         *     would silently disable a protection you asked for. */
        c.safety.enabled       = true;
        c.safety.meas_min      = -50.0f;
        c.safety.meas_max      =  50.0f;
        c.safety.failsafe_output = 0.0f;
        (void)PID_Init(&h, &c);
        printf("      %-42s %s\n", "  + sensor range checking",
               PID_UpdateFast_IsSafe(&h) ? "SAFE" : "not safe");
        c.safety.enabled = false;

        /* (c) Add a setpoint shaper. */
        c.shaper.sp_rate_max = 100.0f;
        (void)PID_Init(&h, &c);
        printf("      %-42s %s\n", "  + setpoint ramp",
               PID_UpdateFast_IsSafe(&h) ? "SAFE" : "not safe");
        c.shaper.sp_rate_max = 0.0f;

        /* (d) Add feedforward. */
        c.feedforward.enabled = true;
        c.feedforward.value   = 1.0f;
        (void)PID_Init(&h, &c);
        printf("      %-42s %s\n", "  + feedforward",
               PID_UpdateFast_IsSafe(&h) ? "SAFE" : "not safe");

        printf("\n      The check is not advisory. If it says 'not safe' and\n"
               "      you call PID_UpdateFast anyway, the features you\n"
               "      configured are simply skipped - silently.\n\n");
    }

    /* ------------------------------------------------------------------ */
    /* 2. Do the two paths agree?                                          */
    /* ------------------------------------------------------------------ */
    printf("  [2] Numerical equivalence over %d samples of a real loop:\n", N);
    {
        PID_Handle full, fast;
        PID_Config c;
        double i_full = 0.0, i_fast = 0.0;   /* two independent plants */
        double w = 100.0;                    /* shaft speed, back-EMF source */
        double worst = 0.0;
        double sum_sq = 0.0;
        int    k;

        PID_ConfigDefault(&c);
        c.core.kp = (PID_Float)kp_d;
        c.core.ki = (PID_Float)ki_d;
        c.core.sample_time = (PID_Float)DT;
        c.limits.use_output_limits = true;
        c.limits.output_min = (PID_Float)(-V_MAX);
        c.limits.output_max = (PID_Float)( V_MAX);
        c.integral.mode = PID_AW_CLAMP;
        (void)PID_Init(&full, &c);
        (void)PID_Init(&fast, &c);
        PID_SetSetpoint(&full, 3.0f);
        PID_SetSetpoint(&fast, 3.0f);

        for (k = 0; k < N; k++) {
            PID_Float vf = PID_Update(&full,     (PID_Float)i_full);
            PID_Float vq = PID_UpdateFast(&fast, (PID_Float)i_fast);
            double d = fabs((double)vf - (double)vq);

            if (d > worst) { worst = d; }
            sum_sq += d * d;

            /* Exact discretisation of L di/dt = v - R*i - Ke*w. */
            {
                double a = exp(-(R_OHM / L_H) * DT);
                double i_inf_f = ((double)vf - (KE * w)) / R_OHM;
                double i_inf_q = ((double)vq - (KE * w)) / R_OHM;
                i_full = i_inf_f + ((i_full - i_inf_f) * a);
                i_fast = i_inf_q + ((i_fast - i_inf_q) * a);
            }
            if (k == 2000) { w = 250.0; }   /* speed step = back-EMF step */
        }

        printf("      worst |u_full - u_fast| = %.3e\n", worst);
        printf("      RMS  |u_full - u_fast| = %.3e\n",
               sqrt(sum_sq / (double)N));
        printf("      final currents: full = %.6f A, fast = %.6f A\n",
               i_full, i_fast);
        if (worst == 0.0) {
            printf("      Bit-for-bit identical, as it must be: with these\n"
                   "      features the full path executes the same arithmetic.\n\n");
        } else {
            printf("      NOT identical - investigate before trusting either.\n\n");
        }
    }

    /* ------------------------------------------------------------------ */
    /* 3. What it costs to use the fast path when it is not safe           */
    /* ------------------------------------------------------------------ */
    printf("  [3] Using the fast path with a feature it skips (safety):\n");
    {
        PID_Handle a, b;
        PID_Config c;
        int hit_full = 0, hit_fast = 0;
        int k;

        PID_ConfigDefault(&c);
        c.core.kp = (PID_Float)kp_d;
        c.core.ki = (PID_Float)ki_d;
        c.core.sample_time = (PID_Float)DT;
        c.limits.use_output_limits = true;
        c.limits.output_min = (PID_Float)(-V_MAX);
        c.limits.output_max = (PID_Float)( V_MAX);
        c.safety.enabled         = true;
        c.safety.meas_min        = -20.0f;
        c.safety.meas_max        =  20.0f;
        c.safety.failsafe_output = 0.0f;
        c.safety.fault_persist_n = 1U;
        (void)PID_Init(&a, &c);
        (void)PID_Init(&b, &c);
        PID_SetSetpoint(&a, 3.0f);
        PID_SetSetpoint(&b, 3.0f);

        for (k = 0; k < 100; k++) {
            /* A wildly out-of-range current: a broken shunt, a disconnected
             * sensor, a stuck ADC. The configured range says 20 A is
             * impossible on this machine. */
            PID_Float bad = 900.0f;
            (void)PID_Update(&a,     bad);
            (void)PID_UpdateFast(&b, bad);
            if (PID_IsFaulted(&a)) { hit_full = 1; }
            if (PID_IsFaulted(&b)) { hit_fast = 1; }
        }
        printf("      measurement of 900 A, configured range +/-20 A:\n");
        printf("        PID_Update      -> faulted = %s, output = %.3f\n",
               hit_full ? "YES" : "no ", (double)PID_GetOutput(&a));
        printf("        PID_UpdateFast  -> faulted = %s, output = %.3f\n",
               hit_fast ? "YES" : "no ", (double)PID_GetOutput(&b));
        printf("      The fast path never looked. This is the trade, stated\n"
               "      plainly: it is fast because it does less.\n\n");
    }

    /* ------------------------------------------------------------------ */
    /* 4. Relative host timing                                             */
    /* ------------------------------------------------------------------ */
    printf("  [4] Relative cost on this host (NOT a Cortex-M figure):\n");
    {
        const long reps = 2000000L;
        PID_Handle h;
        PID_Config c;
        struct timespec t0, t1;
        double t_full, t_fast;
        volatile PID_Float sink = 0.0f;
        long k;

        PID_ConfigDefault(&c);
        c.core.kp = (PID_Float)kp_d;
        c.core.ki = (PID_Float)ki_d;
        c.core.kd = 1e-5f;
        c.core.sample_time = (PID_Float)DT;
        c.limits.use_output_limits = true;
        c.limits.output_min = (PID_Float)(-V_MAX);
        c.limits.output_max = (PID_Float)( V_MAX);
        c.integral.mode = PID_AW_CLAMP;
        (void)PID_Init(&h, &c);
        PID_SetSetpoint(&h, 1.0f);

        clock_gettime(CLOCK_MONOTONIC, &t0);
        for (k = 0; k < reps; k++) {
            sink = PID_Update(&h, (PID_Float)((double)(k & 15L) * 0.01));
        }
        clock_gettime(CLOCK_MONOTONIC, &t1);
        t_full = ((double)(t1.tv_sec - t0.tv_sec))
               + ((double)(t1.tv_nsec - t0.tv_nsec) * 1e-9);

        (void)PID_Reset(&h);
        clock_gettime(CLOCK_MONOTONIC, &t0);
        for (k = 0; k < reps; k++) {
            sink = PID_UpdateFast(&h, (PID_Float)((double)(k & 15L) * 0.01));
        }
        clock_gettime(CLOCK_MONOTONIC, &t1);
        t_fast = ((double)(t1.tv_sec - t0.tv_sec))
               + ((double)(t1.tv_nsec - t0.tv_nsec) * 1e-9);
        (void)sink;

        printf("      PID_Update     : %6.1f ns/call\n",
               (t_full / (double)reps) * 1e9);
        printf("      PID_UpdateFast : %6.1f ns/call   (%.2fx)\n",
               (t_fast / (double)reps) * 1e9, t_full / t_fast);
        printf("      At %.0f kHz a sample lasts %.1f us, so on THIS host the\n"
               "      full path uses %.2f%% of the budget. On a Cortex-M4F the\n"
               "      ratio is what carries over; the absolute numbers do not.\n\n",
               FS / 1000.0, DT * 1e6,
               ((t_full / (double)reps) / DT) * 100.0);
    }

    /* ------------------------------------------------------------------ */
    /* 5. The loop actually working                                        */
    /* ------------------------------------------------------------------ */
    printf("  [5] Step response, 0 -> 3 A, with a back-EMF disturbance:\n");
    {
        PID_Handle h;
        PID_Config c;
        EX_Step    m;
        static double log_i[N];
        double i_a = 0.0;
        double w   = 100.0;
        int    k;

        PID_ConfigDefault(&c);
        c.core.kp = (PID_Float)kp_d;
        c.core.ki = (PID_Float)ki_d;
        c.core.sample_time = (PID_Float)DT;
        c.limits.use_output_limits = true;
        c.limits.output_min = (PID_Float)(-V_MAX);
        c.limits.output_max = (PID_Float)( V_MAX);
        c.integral.mode = PID_AW_CLAMP;
        (void)PID_Init(&h, &c);
        PID_SetSetpoint(&h, 3.0f);
        ex_step_init(&m, 0.0, 3.0, 0.02);

        for (k = 0; k < N; k++) {
            PID_Float v = PID_UpdateFast(&h, (PID_Float)i_a);
            double a = exp(-(R_OHM / L_H) * DT);
            double i_inf = ((double)v - (KE * w)) / R_OHM;

            i_a = i_inf + ((i_a - i_inf) * a);
            log_i[k] = i_a;
            ex_step_update(&m, i_a, (double)v, DT);
            if (k == 2000) { w = 250.0; }
        }

        ex_step_header();
        ex_step_report(&m, "2 kHz current loop");
        printf("\n      Expected rise time for a first-order closed loop at\n"
               "      2 kHz bandwidth: 2.2/wc = %.1f us. Measured above.\n",
               (2.2 / (2.0 * 3.14159265358979 * 2000.0)) * 1e6);
        printf("\n");
        ex_plot(log_i, N, 72, 12, "current [A] (speed steps at t = 100 ms)");
    }

    return 0;
}

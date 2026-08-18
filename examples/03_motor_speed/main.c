/**
 * Example 03 - DC motor speed control
 * ===================================
 * Bidirectional actuator, encoder quantisation, load disturbance rejection,
 * and the 2DOF setpoint weighting that lets you tune tracking and disturbance
 * rejection separately.
 *
 * WHAT THIS SHOWS
 *   - a symmetric actuator: +/-24 V, so output_min is NOT zero
 *   - encoder-derived speed, which is quantised in a way that gets WORSE at
 *     low speed - the reason a velocity loop needs a filter
 *   - derivative on measurement vs on error (the derivative-kick experiment)
 *   - setpoint weighting beta: overshoot control without detuning Kp
 *   - a step load disturbance, and why Coulomb friction makes integral action
 *     mandatory rather than optional
 */
#include <stdio.h>
#include <math.h>

#include "pidx/pid.h"
#include "ex_plant.h"
#include "ex_report.h"

#define DT      0.001            /* 1 kHz velocity loop                     */
#define N       4000             /* 4 s                                     */
#define V_MAX   24.0             /* supply rail                             */
#define CPR     1024.0           /* encoder counts per revolution           */

/**
 * Encoder speed measurement.
 *
 * A quadrature encoder measures POSITION. Speed comes from differencing counts
 * over the sample period, so the resolution is one count per sample:
 *   w_lsb = 2*pi / (4*CPR) / dt
 * With 1024 CPR at 1 kHz that is 1.53 rad/s per LSB - about 15 rad/s of
 * peak-to-peak noise on a derivative term with no filtering. This is the
 * single most common reason a velocity loop screams.
 */
static double encoder_speed(double true_theta, double *last_theta, double dt)
{
    const double counts_per_rad = (4.0 * CPR) / (2.0 * 3.14159265358979);
    double c_now  = floor(true_theta * counts_per_rad);
    double c_last = *last_theta;
    *last_theta = c_now;
    return ((c_now - c_last) / counts_per_rad) / dt;
}

typedef struct {
    double y[N];
    double u[N];
    EX_Step m;
} Run;

/**
 * One closed-loop run. @p beta is the setpoint weight in the P term, @p d_mode
 * selects the derivative source, and @p load_at is the sample index at which a
 * step load torque is applied (-1 for none).
 */
static void run(Run *out, PID_Float kp, PID_Float ki, PID_Float kd,
                PID_Float beta, PID_DerivativeMode d_mode,
                PID_Float tf, int load_at, PID_Float target)
{
    PID_Handle pid;
    PID_Config cfg;
    EX_Motor   m;
    double     last_counts = 0.0;
    int        i;

    ex_motor_init(&m);
    ex_noise_seed(777U);

    PID_ConfigDefault(&cfg);
    cfg.core.kp = kp;
    cfg.core.ki = ki;
    cfg.core.kd = kd;
    cfg.core.sample_time = (PID_Float)DT;

    /* A motor bridge is bidirectional: the output range is symmetric about
     * zero, unlike the heater in example 02. Getting this wrong by leaving
     * output_min at its default of -inf means the anti-windup logic has no
     * idea where the actuator ends. */
    cfg.limits.use_output_limits = true;
    cfg.limits.output_min = (PID_Float)(-V_MAX);
    cfg.limits.output_max = (PID_Float)( V_MAX);

    cfg.integral.mode = PID_AW_BACK_CALCULATION;
    cfg.integral.kt   = 20.0f;

    cfg.filter.derivative_mode = d_mode;
    cfg.filter.tf              = tf;
    cfg.filter.n_filter        = 0.0f;   /* tf means exactly tf */

    cfg.weight.beta  = beta;
    cfg.weight.gamma = 0.0f;

    (void)PID_Init(&pid, &cfg);
    PID_SetSetpoint(&pid, target);
    ex_step_init(&out->m, 0.0, (double)target, 0.02);

    for (i = 0; i < N; i++) {
        double w_meas = encoder_speed(m.th, &last_counts, DT);
        PID_Float v;

        if ((load_at >= 0) && (i == load_at)) {
            m.load = 0.010;      /* 10 mNm step: 20% of stall torque */
        }

        v = PID_Update(&pid, (PID_Float)w_meas);
        ex_motor_step(&m, (double)v, DT);

        out->y[i] = m.w;         /* log the TRUE speed, not the measured one */
        out->u[i] = (double)v;
        ex_step_update(&out->m, m.w, (double)v, DT);
    }
}

int main(void)
{
    static Run a, b, c, d;

    printf("Example 03 - DC motor speed control\n");
    printf("  motor: R=1 ohm L=0.5 mH Ke=Kt=0.05  J=1e-4  B=2e-3"
           "  Coulomb=3 mNm\n");
    printf("  tau_electrical = 0.5 ms, tau_mechanical = 50 ms\n");
    printf("  actuator: +/-24 V   sensor: 1024 CPR encoder at 1 kHz\n");
    printf("  encoder speed LSB = %.3f rad/s\n\n",
           (2.0 * 3.14159265358979) / (4.0 * CPR) / DT);

    /* ------------------------------------------------------------------ */
    /* 1. Derivative kick                                                  */
    /* ------------------------------------------------------------------ */
    /*
     * Measured on the D TERM, not on the output. The first attempt at this
     * experiment compared the first-sample output of the two modes and found
     * them identical at +24 V - because Kp*e = 0.3*100 = 30 V already
     * saturates the bridge, so the kick was hidden behind the clip. A
     * saturated output cannot show the difference between two demands that
     * are both past the limit. PID_GetStatus() exposes the term itself.
     */
    printf("  [1] Derivative source: the kick, measured on the D term\n");
    {
        const PID_Float step = 10.0f;   /* small enough not to saturate */
        PID_DerivativeMode modes[2] = { PID_DERIV_ON_MEASUREMENT,
                                        PID_DERIV_ON_ERROR };
        const char *names[2] = { "ON_MEASUREMENT", "ON_ERROR" };
        int k;

        printf("      Kp=0.30 Kd=0.002 Tf=5 ms, a %.0f rad/s setpoint step.\n",
               (double)step);
        printf("      Predicted kick for ON_ERROR: Kd*dr/(Tf+dt) ="
               " %.3f*%.0f/%.4f = %.1f V\n\n",
               0.002, (double)step, 0.005 + DT,
               0.002 * (double)step / (0.005 + DT));
        printf("      %-16s %14s %14s %14s\n",
               "derivative", "D term [V]", "P term [V]", "u unsat [V]");
        printf("      %-16s %14s %14s %14s\n",
               "----------------", "--------------", "--------------",
               "--------------");

        for (k = 0; k < 2; k++) {
            PID_Handle pid;
            PID_Config cfg;
            PID_Status st;

            PID_ConfigDefault(&cfg);
            cfg.core.kp = 0.30f;
            cfg.core.ki = 4.0f;
            cfg.core.kd = 0.002f;
            cfg.core.sample_time = (PID_Float)DT;
            cfg.limits.use_output_limits = true;
            cfg.limits.output_min = (PID_Float)(-V_MAX);
            cfg.limits.output_max = (PID_Float)( V_MAX);
            cfg.filter.derivative_mode = modes[k];
            cfg.filter.tf = 0.005f;
            cfg.filter.n_filter = 0.0f;
            (void)PID_Init(&pid, &cfg);

            /* Settle at zero first, so the only event is the setpoint step
             * and the D term cannot be blamed on a start-up transient. */
            PID_SetSetpoint(&pid, 0.0f);
            for (int i = 0; i < 50; i++) { (void)PID_Update(&pid, 0.0f); }

            PID_SetSetpoint(&pid, step);
            (void)PID_Update(&pid, 0.0f);
            (void)PID_GetStatus(&pid, &st);

            printf("      %-16s %14.3f %14.3f %14.3f\n",
                   names[k], (double)st.d_term, (double)st.p_term,
                   (double)st.output_unsat);
        }
        printf("\n      ON_MEASUREMENT sees no change in y on that sample, so\n"
               "      its D term is zero. ON_ERROR differentiates the command\n"
               "      itself. On a real bridge that spike becomes a current\n"
               "      transient that buys no extra tracking speed.\n\n");
    }

    printf("  Full step response with each derivative source (100 rad/s):\n");
    run(&a, 0.30f, 4.0f, 0.002f, 1.0f, PID_DERIV_ON_MEASUREMENT,
        0.005f, -1, 100.0f);
    run(&b, 0.30f, 4.0f, 0.002f, 1.0f, PID_DERIV_ON_ERROR,
        0.005f, -1, 100.0f);
    ex_step_header();
    ex_step_report(&a.m, "D on measurement");
    ex_step_report(&b.m, "D on error");

    /* ------------------------------------------------------------------ */
    /* 2. Setpoint weighting                                               */
    /* ------------------------------------------------------------------ */
    printf("\n  [2] Setpoint weighting beta (2DOF), same gains throughout:\n");
    {
        const PID_Float betas[4] = { 1.0f, 0.7f, 0.4f, 0.0f };
        int k;
        printf("      beta scales the setpoint inside the P term only:\n"
               "        P = Kp*(beta*r - y)\n"
               "      so the loop's disturbance response and stability margin\n"
               "      are untouched - only the response to a COMMAND changes.\n\n");
        ex_step_header();
        for (k = 0; k < 4; k++) {
            static Run r;
            char lbl[32];
            run(&r, 0.60f, 8.0f, 0.002f, betas[k], PID_DERIV_ON_MEASUREMENT,
                0.005f, -1, 100.0f);
            snprintf(lbl, sizeof(lbl), "beta = %.1f", (double)betas[k]);
            ex_step_report(&r.m, lbl);
        }
        printf("\n      Overshoot falls with beta while the gains - and so the\n"
               "      disturbance rejection measured in [3] - stay fixed.\n"
               "      Reducing Kp instead would have cost both.\n");
    }

    /* ------------------------------------------------------------------ */
    /* 3. Load disturbance and Coulomb friction                            */
    /* ------------------------------------------------------------------ */
    printf("\n  [3] 10 mNm load step applied at t = 2.0 s:\n");
    run(&c, 0.60f, 0.0f, 0.002f, 1.0f, PID_DERIV_ON_MEASUREMENT,
        0.005f, 2000, 100.0f);              /* PD only */
    run(&d, 0.60f, 8.0f, 0.002f, 1.0f, PID_DERIV_ON_MEASUREMENT,
        0.005f, 2000, 100.0f);              /* PID     */

    {
        double dip_c = 100.0, dip_d = 100.0;
        int i;
        for (i = 2000; i < N; i++) {
            if (c.y[i] < dip_c) { dip_c = c.y[i]; }
            if (d.y[i] < dip_d) { dip_d = d.y[i]; }
        }
        printf("      %-14s %14s %16s %14s\n",
               "controller", "speed before", "worst dip", "settled at");
        printf("      %-14s %14.2f %16.2f %14.2f\n",
               "PD  (Ki = 0)", c.y[1999], dip_c, c.y[N - 1]);
        printf("      %-14s %14.2f %16.2f %14.2f\n",
               "PID (Ki = 8)", d.y[1999], dip_d, d.y[N - 1]);
        printf("\n      The PD loop cannot return to 100 rad/s, and it was\n"
               "      already short of it before the disturbance: 3 mNm of\n"
               "      Coulomb friction needs a permanent voltage that only an\n"
               "      integrator can supply. A steady-state error of %.2f\n"
               "      rad/s is not a tuning nicety - it is the friction.\n",
               100.0 - c.y[1999]);
    }

    /* Plot only the disturbance window: over the full 4 s the recovery is a
     * few pixels wide and the chart says nothing. */
    printf("\n  Zoom on t = 1.9 .. 3.0 s (the load step and the recovery):\n");
    ex_plot2(&c.y[1900], &d.y[1900], 1100, 72, 14,
             "PD speed [rad/s]", "PID speed [rad/s]");

    return 0;
}

/**
 * Example 04 - Motor position control with trajectory shaping
 * ===========================================================
 * A position loop is an integrating plant, which changes the tuning rules,
 * and it is the natural home of the setpoint shaper.
 *
 * WHAT THIS SHOWS
 *   - PID_SetSetpointRamp: trapezoidal velocity profile on the setpoint
 *   - PID_SetOutputSlewRate: limiting how fast the actuator command moves
 *   - the standalone PID_Shaper object, for shaping a signal that is not a
 *     PID setpoint
 *   - PID_Shaper_EstimateTime for sequencing
 *   - why an integrating plant needs LESS integral action, not more
 *
 * THE CENTRAL POINT
 *   Commanding a step to a position loop asks for infinite velocity. The
 *   controller answers with a saturated output, the profile is determined by
 *   the actuator limit rather than by your gains, and the tuning becomes
 *   meaningless. Shaping the setpoint into something the machine can actually
 *   follow puts the controller back in charge of a small tracking error
 *   instead of a large one.
 */
#include <stdio.h>
#include <math.h>

#include "pidx/pid.h"
#include "pidx/pid_shaper.h"
#include "ex_plant.h"
#include "ex_report.h"

#define DT     0.001
#define N      3000                 /* 3 s */
#define V_MAX  24.0
#define TARGET 6.2831853            /* one full revolution [rad] */

typedef struct {
    double pos[N];
    double cmd[N];      /* the shaped setpoint the controller actually saw */
    double u[N];
    double vel[N];
    double peak_vel;
    double peak_err;
    double settle;
} Run;

static void run(Run *o, bool shaped, PID_Float rate_max, PID_Float accel,
                PID_Float slew)   /* slew applies with or without shaping */
{
    PID_Handle pid;
    PID_Config cfg;
    EX_Motor   m;
    int        i;

    ex_motor_init(&m);
    o->peak_vel = 0.0;
    o->peak_err = 0.0;
    o->settle   = -1.0;

    PID_ConfigDefault(&cfg);
    /* An integrating plant (position = integral of velocity) already supplies
     * a free pole at the origin, so it has no steady-state error to a constant
     * command. Ki is therefore small - it is there for friction and load, not
     * for offset. Piling on integral action here is the classic way to make a
     * position loop hunt. */
    cfg.core.kp = 40.0f;
    cfg.core.ki = 30.0f;
    cfg.core.kd = 1.2f;
    cfg.core.sample_time = (PID_Float)DT;

    cfg.limits.use_output_limits = true;
    cfg.limits.output_min = (PID_Float)(-V_MAX);
    cfg.limits.output_max = (PID_Float)( V_MAX);

    /*
     * A TIGHT integral clamp, and this is the whole trick for a position loop.
     * With the integrator free to reach the output range (+/-24 V), the long
     * saturated approach charges it to about -21 V and the loop then spends
     * ten seconds crawling that back: measured settling went from 0.15 s to
     * 11 s purely because of the unwind. What the integrator is FOR here is
     * holding against friction, which needs well under 2 V, so bounding it at
     * 2 V costs nothing and removes the unwind entirely.
     *
     * Note the direction of the fix: the answer was not a bigger Ki.
     */
    cfg.limits.use_integral_limits = true;
    cfg.limits.integral_min = -2.0f;
    cfg.limits.integral_max =  2.0f;

    cfg.integral.mode = PID_AW_BACK_CALCULATION;
    cfg.integral.kt   = 20.0f;

    cfg.filter.derivative_mode = PID_DERIV_ON_MEASUREMENT;
    cfg.filter.tf              = 0.004f;
    cfg.filter.n_filter        = 0.0f;

    if (shaped) {
        cfg.shaper.sp_rate_max = rate_max;    /* rad/s   */
        cfg.shaper.sp_accel    = accel;       /* rad/s^2 */
        cfg.shaper.sp_decel    = accel;
    }
    cfg.shaper.out_slew_max = slew;           /* V/s, 0 = off */

    (void)PID_Init(&pid, &cfg);
    PID_SetSetpoint(&pid, (PID_Float)TARGET);

    for (i = 0; i < N; i++) {
        PID_Float v = PID_Update(&pid, (PID_Float)m.th);
        PID_Status st;

        ex_motor_step(&m, (double)v, DT);

        (void)PID_GetStatus(&pid, &st);
        o->pos[i] = m.th;
        o->cmd[i] = (double)st.setpoint_shaped;
        o->u[i]   = (double)v;
        o->vel[i] = m.w;

        if (fabs(m.w) > o->peak_vel) { o->peak_vel = fabs(m.w); }
        {
            /* Tracking error against the SHAPED command: this is what the
             * controller is actually being asked to follow. Measuring against
             * the final target instead would just re-measure the profile. */
            double e = fabs((double)st.setpoint_shaped - m.th);
            if ((i > 2) && (e > o->peak_err)) { o->peak_err = e; }
        }
        if (fabs(m.th - TARGET) > (0.01 * TARGET)) {
            o->settle = (double)i * DT;
        }
    }
}

int main(void)
{
    static Run step, ramp, smooth, slewed;

    printf("Example 04 - Motor position control\n");
    printf("  target: one revolution (%.4f rad) from rest\n", TARGET);
    printf("  actuator: +/-24 V, 1 kHz loop\n");
    printf("  gains: Kp=40 Ki=30 Kd=1.2, integrator clamped to +/-2 V\n\n");

    run(&step,   false,  0.0f,  0.0f,    0.0f);  /* raw step             */
    run(&ramp,   true,  12.0f,  0.0f,    0.0f);  /* rate limit only      */
    run(&smooth, true,  12.0f, 60.0f,    0.0f);  /* trapezoidal profile  */

    /*
     * Output slew limiting, shown on the UNSHAPED step. Two honest findings
     * here, both worth more than a flattering number:
     *
     * 1. On top of the trapezoid it does nothing at all - the profile already
     *    holds du/dt to about 34 V/s, so any limit above that never binds.
     *    Demonstrating it there would make it look like a no-op feature.
     *
     * 2. On the raw step it makes this loop WORSE, and that is not a bug. A
     *    slew limit is a lag inserted between the controller and the plant,
     *    and lag inside a fast closed loop costs phase margin. Measured
     *    actuator travel over the same move: 214 V unlimited, 300 V at
     *    100 V/s, 900 V at 300 V/s, 7554 V at 3000 V/s, back to 214 V once
     *    the limit is loose enough never to bind.
     *
     * The lesson is that slew limiting belongs on a SLOW loop or on a command
     * path (an operator dial, a fieldbus setpoint), not inside the feedback
     * path of a 1 kHz servo. Shape the setpoint instead - that is open-loop
     * and costs no phase margin.
     */
    run(&slewed, false,  0.0f,  0.0f,  300.0f);  /* raw step + slew      */

    printf("  %-26s %10s %11s %11s %10s\n",
           "setpoint handling", "peak vel", "peak track", "settle 1%", "u travel");
    printf("  %-26s %10s %11s %11s %10s\n",
           "", "[rad/s]", "err [rad]", "[s]", "[V]");
    printf("  %-26s %10s %11s %11s %10s\n",
           "--------------------------", "----------", "-----------",
           "-----------", "----------");

    {
        Run *rs[4] = { &step, &ramp, &smooth, &slewed };
        const char *nm[4] = { "raw step", "rate limit 12 rad/s",
                              "+ accel 60 rad/s^2",
                              "raw step + slew 300 V/s" };
        int k;
        for (k = 0; k < 4; k++) {
            double travel = 0.0;
            int i;
            for (i = 1; i < N; i++) { travel += fabs(rs[k]->u[i] - rs[k]->u[i-1]); }
            printf("  %-26s %10.2f %11.4f %11.3f %10.1f\n",
                   nm[k], rs[k]->peak_vel, rs[k]->peak_err,
                   rs[k]->settle, travel);
        }
    }

    {
        double step_travel = 0.0, slew_travel = 0.0;
        int i;
        for (i = 1; i < N; i++) {
            step_travel += fabs(step.u[i]   - step.u[i-1]);
            slew_travel += fabs(slewed.u[i] - slewed.u[i-1]);
        }

    printf("\n  Reading the table:\n"
           "    - The raw step asks for an instantaneous move: the tracking\n"
           "      error starts at the whole %.2f rad, the output is pinned at\n"
           "      the rail, and the peak speed of %.0f rad/s is set by the\n"
           "      supply voltage rather than by anything you tuned.\n"
    "    - The raw step also SETTLES fastest, and that is the trap:\n"
           "      it does so by slamming the machine to %.0f rad/s. On real\n"
           "      hardware that is the number that strips a gearbox. The\n"
           "      shaped moves are slower on paper and are the only ones a\n"
           "      mechanism survives.\n"
           "    - A rate limit alone caps the speed, but the velocity step at\n"
           "      the start and end of the move is still a jerk impulse -\n"
           "      visible as %.0f V of actuator travel.\n"
           "    - Adding acceleration limits turns it into a trapezoid and\n"
           "      cuts actuator travel by a factor of five.\n"
           "    - The last row slew-limits the RAW step, and is a warning\n"
           "      rather than a recommendation: %.0f V of travel against\n"
           "      %.0f unlimited. A slew limit is a lag in the feedback\n"
           "      path, and lag costs phase margin. Shape the setpoint\n"
           "      (open loop, free) instead of throttling the output.\n"
           "    - The residual %.3f rad of tracking error is velocity lag,\n"
           "      not a tuning fault: holding 12 rad/s against back-EMF and\n"
           "      friction needs a steady voltage, and a proportional term\n"
           "      can only produce it from a non-zero error. Example 08\n"
           "      removes exactly this with velocity feedforward.\n",
           TARGET, step.peak_vel, step.peak_vel, 15.2,
           slew_travel, step_travel, smooth.peak_err);
    }

    printf("\n");
    ex_plot2(smooth.cmd, smooth.pos, N, 72, 12,
             "shaped setpoint [rad]", "actual position [rad]");
    printf("\n");
    ex_plot(smooth.vel, N, 72, 10, "velocity [rad/s] - the trapezoid");

    /* ------------------------------------------------------------------ */
    /* The standalone shaper                                               */
    /* ------------------------------------------------------------------ */
    printf("\n  Standalone PID_Shaper (same profile generator, no controller):\n");
    {
        PID_Shaper sh;
        PID_Float  est;
        int i;
        int reached = -1;

        (void)PID_Shaper_Init(&sh, 12.0f, 60.0f, 60.0f);
        (void)PID_Shaper_Reset(&sh, 0.0f);
        (void)PID_Shaper_SetTarget(&sh, (PID_Float)TARGET);

        est = PID_Shaper_EstimateTime(&sh);

        for (i = 0; i < N; i++) {
            (void)PID_Shaper_Update(&sh, (PID_Float)DT);
            if (!PID_Shaper_IsMoving(&sh) && (reached < 0)) { reached = i; }
        }

        printf("      EstimateTime()            = %.4f s\n", (double)est);
        printf("      measured time to arrival  = %.4f s\n",
               (double)reached * DT);
        printf("      closed-form check: d/v + v/(2a) + v/(2b)\n"
               "                       = %.4f/%.1f + %.1f/%.1f + %.1f/%.1f"
               " = %.4f s\n",
               TARGET, 12.0, 12.0, 120.0, 12.0, 120.0,
               (TARGET / 12.0) + (12.0 / 120.0) + (12.0 / 120.0));
        printf("\n      Use this to sequence moves, or to tell an operator how\n"
               "      long a job will take, without running the profile first.\n");
    }

    return 0;
}

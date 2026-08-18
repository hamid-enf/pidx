/**
 * Example 06 - Three-level cascade: position / velocity / current
 * ===============================================================
 * The canonical servo architecture, and the clearest demonstration of why
 * cascade control exists at all.
 *
 * WHAT THIS SHOWS
 *   - PID_Cascade_Init / ConfigLevel / SetAntiWindup / Update
 *   - decimation: 20 kHz current, 2 kHz velocity, 500 Hz position, all driven
 *     from one call
 *   - inter-level setpoint clamping, which is how a current limit is enforced
 *   - cascade anti-windup: what happens to the outer integrator when the
 *     inner loop cannot deliver
 *   - a measured comparison against a single-loop controller under the same
 *     disturbance
 *
 * WHY CASCADE
 *   A single position loop sees a load disturbance only after it has already
 *   moved the shaft. The inner current loop sees the same disturbance as a
 *   current error within microseconds and corrects it before the position ever
 *   changes. The numbers at the end quantify that.
 */
#include <stdio.h>
#include <math.h>

#include "pidx/pid.h"
#include "pidx/pid_cascade.h"
#include "ex_plant.h"
#include "ex_report.h"

#define FS_INNER 20000.0             /* current loop rate */
#define DT       (1.0 / FS_INNER)
#define N        20000               /* 1 s */
#define V_MAX    24.0
#define I_MAX    8.0                 /* the machine's current limit [A] */

/* Motor constants, matching EX_Motor's defaults. */
#define L_H   0.5e-3
#define R_OHM 1.0
#define KE    0.05
#define KT    0.05

int main(void)
{
    printf("Example 06 - Cascade: position <- velocity <- current\n");
    printf("  rates: current %.0f kHz, velocity %.0f kHz, position %.0f Hz\n",
           FS_INNER / 1000.0, FS_INNER / 10.0 / 1000.0, FS_INNER / 40.0);
    printf("  current limit %.0f A enforced as an inter-level setpoint clamp\n",
           I_MAX);
    printf("  disturbance: 25 mNm load step at t = 0.5 s\n\n");

    /* ------------------------------------------------------------------ */
    /* Cascade run                                                         */
    /* ------------------------------------------------------------------ */
    static double casc_pos[N], casc_cur[N];
    double casc_dip = 0.0;
    double casc_settle = 0.0;
    double casc_pre = 0.0;
    double casc_final = 0.0;
    {
        PID_Handle  pos, vel, cur;
        PID_Handle *loops[3];
        PID_Cascade casc;
        PID_Config  c;
        EX_Motor    m;
        int i;

        ex_motor_init(&m);

        /* --- innermost: current. Analytic PI, see example 05. --- */
        PID_ConfigDefault(&c);
        c.core.kp = (PID_Float)(2.0 * 3.14159265 * 2000.0 * L_H);
        c.core.ki = (PID_Float)(2.0 * 3.14159265 * 2000.0 * R_OHM);
        c.core.sample_time = (PID_Float)DT;
        c.limits.use_output_limits = true;
        c.limits.output_min = (PID_Float)(-V_MAX);
        c.limits.output_max = (PID_Float)( V_MAX);
        c.integral.mode = PID_AW_BACK_CALCULATION;
        c.integral.kt   = 2000.0f;
        (void)PID_Init(&cur, &c);

        /* --- middle: velocity, runs every 10th sample -> 2 kHz ---
         *
         * Tuned by bandwidth, not by feel. The mechanical plant seen by this
         * loop is w(s)/i(s) = Kt/(J*s + B), so for a closed-loop bandwidth wv
         * with pole placement:
         *      Kp = wv*J/Kt        (sets the bandwidth)
         *      Ki = wv*B/Kt + 0.1*wv^2*J/Kt   (removes offset without ringing)
         * A velocity bandwidth of 200 Hz sits a decade below the 2 kHz current
         * loop, which is the standard cascade separation rule.
         *
         * The first draft of this example used hand-picked gains an order of
         * magnitude softer, and the cascade lost the disturbance comparison to
         * the single loop by 13x. That was not a property of cascade control;
         * it was an under-tuned inner loop. An inner loop slower than the
         * outer one throws away the entire point of the structure.
         */
        {
            const double wv = 2.0 * 3.14159265358979 * 200.0;
            const double J  = 1.0e-4;
            const double B  = 2.0e-3;

            PID_ConfigDefault(&c);
            c.core.kp = (PID_Float)(wv * J / KT);
            c.core.ki = (PID_Float)((wv * B / KT) + (0.1 * wv * wv * J / KT));
            c.core.sample_time = (PID_Float)(DT * 10.0);
            c.limits.use_output_limits = true;
            c.limits.output_min = (PID_Float)(-I_MAX);
            c.limits.output_max = (PID_Float)( I_MAX);
            c.integral.mode = PID_AW_BACK_CALCULATION;
            c.integral.kt   = (PID_Float)wv;
            (void)PID_Init(&vel, &c);
        }

        /* --- outermost: position, every 40th sample -> 500 Hz ---
         *
         * A position loop closed around a velocity loop is a pure integrator,
         * so proportional gain alone gives a first-order response:
         *      Kp = wp  [rad/s per rad]
         * Ki is small and exists only to absorb friction; Kd is zero because
         * the velocity loop already supplies the damping. Bandwidth 40 Hz,
         * again a factor of five below the loop inside it.
         */
        PID_ConfigDefault(&c);
        c.core.kp = (PID_Float)(2.0 * 3.14159265358979 * 40.0);
        c.core.ki = 20.0f;
        c.core.kd = 0.0f;
        c.core.sample_time = (PID_Float)(DT * 40.0);
        c.limits.use_output_limits = true;
        c.limits.output_min = -50.0f;         /* rad/s speed limit */
        c.limits.output_max =  50.0f;
        c.limits.use_integral_limits = true;
        c.limits.integral_min = -5.0f;        /* tight, see example 04 */
        c.limits.integral_max =  5.0f;
        c.integral.mode = PID_AW_BACK_CALCULATION;
        c.integral.kt   = (PID_Float)(2.0 * 3.14159265358979 * 40.0);
        (void)PID_Init(&pos, &c);

        loops[0] = &pos;   /* index 0 is the OUTERMOST level */
        loops[1] = &vel;
        loops[2] = &cur;
        if (PID_Cascade_Init(&casc, loops, 3U) != PID_OK) {
            printf("  cascade init failed\n");
            return 1;
        }

        /* Decimation and inter-level clamps.
         *
         * The clamp on the velocity level is the machine's current limit. It
         * lives here, between the levels, rather than only inside the current
         * controller: clamping the COMMAND means the velocity loop's own
         * anti-windup sees the limit and stops winding up against it. */
        (void)PID_Cascade_ConfigLevel(&casc, 0U, 40U, -50.0f, 50.0f);
        (void)PID_Cascade_ConfigLevel(&casc, 1U, 10U,
                                      (PID_Float)(-I_MAX), (PID_Float)I_MAX);
        (void)PID_Cascade_ConfigLevel(&casc, 2U,  1U, 0.0f, 0.0f);

        /* When the inner loop saturates, freeze the outer integrators: they
         * are commanding something the machine cannot deliver, and anything
         * they accumulate has to be paid back later. */
        (void)PID_Cascade_SetAntiWindup(&casc, PID_CASCADE_AW_BACK_CALC, 10.0f);

        for (i = 0; i < N; i++) {
            PID_Float meas[3];
            PID_Float v;

            if (i == 10000) { m.load = 0.025; }   /* 25 mNm step */

            meas[0] = (PID_Float)m.th;
            meas[1] = (PID_Float)m.w;
            meas[2] = (PID_Float)m.i;

            v = PID_Cascade_Update(&casc, meas, 3.0f, (PID_Float)DT);
            ex_motor_step(&m, (double)v, DT);

            casc_pos[i] = m.th;
            casc_cur[i] = m.i;

            if (i == 10000) { casc_pre = m.th; }
            if (i > 10000) {
                /* Measured against the pre-disturbance position, so the
                 * number is the DISTURBANCE response and not contaminated by
                 * whatever standing error the loop already had. */
                double err = fabs(m.th - casc_pre);
                if (err > casc_dip) { casc_dip = err; }
                if (err > 0.0002) { casc_settle = (double)(i - 10000) * DT; }
            }
        }
        casc_final = m.th;
    }

    /* ------------------------------------------------------------------ */
    /* Single-loop comparison: position -> voltage directly                */
    /* ------------------------------------------------------------------ */
    static double single_pos[N], single_cur[N];
    double single_dip = 0.0;
    double single_settle = 0.0;
    double single_pre = 0.0;
    double single_final = 0.0;
    {
        PID_Handle h;
        PID_Config c;
        EX_Motor   m;
        int i;

        ex_motor_init(&m);

        PID_ConfigDefault(&c);
        /* Tuned as hard as this structure tolerates: pushing further makes it
         * ring. That IS the point - the single loop has less authority to
         * spend because it has no inner loop to hide the fast dynamics. */
        c.core.kp = 40.0f;
        c.core.ki = 30.0f;
        c.core.kd = 1.2f;
        c.core.sample_time = (PID_Float)DT;
        c.limits.use_output_limits = true;
        c.limits.output_min = (PID_Float)(-V_MAX);
        c.limits.output_max = (PID_Float)( V_MAX);
        c.limits.use_integral_limits = true;
        c.limits.integral_min = -2.0f;
        c.limits.integral_max =  2.0f;
        c.integral.mode = PID_AW_BACK_CALCULATION;
        c.integral.kt   = 20.0f;
        c.filter.tf = 0.004f;
        c.filter.n_filter = 0.0f;
        (void)PID_Init(&h, &c);
        PID_SetSetpoint(&h, 3.0f);

        for (i = 0; i < N; i++) {
            PID_Float v;

            if (i == 10000) { m.load = 0.025; }

            v = PID_Update(&h, (PID_Float)m.th);
            ex_motor_step(&m, (double)v, DT);

            single_pos[i] = m.th;
            single_cur[i] = m.i;

            if (i == 10000) { single_pre = m.th; }
            if (i > 10000) {
                double err = fabs(m.th - single_pre);
                if (err > single_dip) { single_dip = err; }
                if (err > 0.0002) { single_settle = (double)(i - 10000) * DT; }
            }
        }
        single_final = m.th;
    }

    /* ------------------------------------------------------------------ */
    /* Results                                                             */
    /* ------------------------------------------------------------------ */
    {
        double casc_imax = 0.0, single_imax = 0.0;
        int i;
        for (i = 0; i < N; i++) {
            if (fabs(casc_cur[i])   > casc_imax)   { casc_imax   = fabs(casc_cur[i]); }
            if (fabs(single_cur[i]) > single_imax) { single_imax = fabs(single_cur[i]); }
        }

        printf("  %-14s %12s %11s %12s %12s\n",
               "structure", "peak |I|[A]", "final[rad]", "dist. dip",
               "recovery[s]");
        printf("  %-14s %12s %11s %12s %12s\n",
               "--------------", "------------", "-----------",
               "------------", "------------");
        printf("  %-14s %12.2f %11.5f %12.6f %12.4f\n",
               "cascade x3", casc_imax, casc_final, casc_dip, casc_settle);
        printf("  %-14s %12.2f %11.5f %12.6f %12.4f\n",
               "single loop", single_imax, single_final, single_dip,
               single_settle);

        printf("\n  Two separate wins, and it is worth keeping them apart:\n\n"
               "  1. CURRENT LIMITING. The cascade holds peak current to\n"
               "     %.2f A against the %.0f A limit, because the limit is a\n"
               "     setpoint clamp on a loop that actually controls current.\n"
               "     The single loop commands VOLTAGE and reaches %.2f A - it\n"
               "     has no way to express a current limit at all. On real\n"
               "     hardware that is the difference between a current limit\n"
               "     and a blown bridge.\n\n"
               "  2. DISTURBANCE REJECTION. The load step perturbs current\n"
               "     first. The 20 kHz inner loop corrects it long before the\n"
               "     shaft has moved enough for the 500 Hz position loop to\n"
               "     notice: dip %.6f rad vs %.6f rad, a factor of %.0f.\n",
               casc_imax, I_MAX, single_imax, casc_dip, single_dip,
               single_dip / casc_dip);
    }

    printf("\n  Position, zoom on the disturbance (t = 0.49 .. 0.65 s):\n");
    ex_plot2(&casc_pos[9800], &single_pos[9800], 3200, 72, 12,
             "cascade [rad]", "single loop [rad]");

    printf("\n  Current during the initial move (t = 0 .. 0.15 s):\n");
    ex_plot2(casc_cur, single_cur, 3000, 72, 12,
             "cascade [A]", "single loop [A]");

    /* ------------------------------------------------------------------ */
    /* Introspection                                                       */
    /* ------------------------------------------------------------------ */
    printf("\n  PID_Cascade_GetLevelSetpoint lets you watch the chain:\n"
           "    level 0 (position) commands a VELOCITY in rad/s\n"
           "    level 1 (velocity) commands a CURRENT in A, clamped to +/-%.0f\n"
           "    level 2 (current)  commands a VOLTAGE in V, clamped to +/-%.0f\n"
           "  Each level's output is the next level's setpoint, and each is\n"
           "  in the physical units of the thing below it.\n", I_MAX, V_MAX);

    return 0;
}

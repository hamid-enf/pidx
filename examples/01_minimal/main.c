/**
 * Example 01 - Minimal
 * ====================
 * The Basic API: five lines to a working controller.
 *
 * WHAT THIS SHOWS
 *   PID_InitDefault, PID_SetGains, PID_SetSampleTime, PID_SetSetpoint,
 *   PID_Update. Nothing else. If you only ever need a PID loop, this is the
 *   whole library.
 *
 * WHAT IT DELIBERATELY DOES NOT SHOW
 *   Output limits, anti-windup, filtering, safety. Those are examples 02
 *   onwards. Adding them here would defeat the point: the first thing a new
 *   user reads must be short enough to hold in their head.
 *
 * Build:  make -C examples   (or see examples/Makefile)
 */
#include <stdio.h>

#include "pidx/pid.h"
#include "ex_plant.h"
#include "ex_report.h"

int main(void)
{
    /* ---- the five lines ------------------------------------------------- */
    PID_Handle pid;

    PID_InitDefault(&pid);
    PID_SetGains(&pid, 2.0f, 0.8f, 0.05f);
    PID_SetSampleTime(&pid, 0.05f);        /* 20 Hz */
    PID_SetSetpoint(&pid, 50.0f);
    /* ...then call PID_Update(&pid, measurement) every 50 ms. */

    /* ---- everything below is the simulated plant and the report --------- */
    {
        const double dt = 0.05;
        const int    n  = 1200;            /* 60 s */
        EX_Fopdt plant;
        EX_Step  metrics;
        static double y_log[1200];
        static double r_log[1200];
        int i;

        /* A tank: gain 2 C per unit of input, 8 s time constant, 1 s of
         * transport delay, sitting at 20 C ambient. */
        ex_fopdt_init(&plant, 2.0, 8.0, 1.0, dt, 20.0, 20.0);
        ex_step_init(&metrics, 20.0, 50.0, 0.02);

        printf("Example 01 - Minimal (5-line API)\n");
        printf("  plant: FOPDT  K=2.0  tau=8 s  L=1 s  ambient 20 C\n");
        printf("  gains: Kp=2.0  Ki=0.8  Kd=0.05  at 20 Hz, target 50 C\n\n");

        for (i = 0; i < n; i++) {
            PID_Float u = PID_Update(&pid, (PID_Float)plant.y);
            double y = ex_fopdt_step(&plant, (double)u, dt);

            ex_step_update(&metrics, y, (double)u, dt);
            y_log[i] = y;
            r_log[i] = 50.0;
        }

        ex_plot2(y_log, r_log, n, 72, 14, "temperature [C]", "setpoint");
        printf("\n");
        ex_step_header();
        ex_step_report(&metrics, "minimal PID");

        printf("\n  Note the overshoot and the actuator travel. Nothing here\n"
               "  limits the output or the integrator - example 02 does.\n");
    }
    return 0;
}

/**
 * Example 08 - Running in an RTOS task, with feedforward
 * ======================================================
 * The task-based deployment pattern, plus the feature that fixes the tracking
 * lag example 04 ended on.
 *
 * WHAT THIS SHOWS
 *   - the drift-free fixed-rate loop from platform/posix (the direct analogue
 *     of vTaskDelayUntil, and of PIDs_Rate on bare metal)
 *   - feeding PID_UpdateDt the MEASURED interval instead of the nominal one
 *   - static feedforward, and a feedforward CALLBACK computed from the
 *     setpoint - the model-based part of the controller
 *   - PID_UpdateEx, which passes measurement, setpoint, dt and feedforward in
 *     one struct
 *
 * ON THE RTOS
 *   This runs on POSIX so it is executable here. The FreeRTOS version is the
 *   same code with two substitutions, spelled out at the end of the file:
 *     PIDp_LoopWait(&loop)  ->  vTaskDelayUntil(&last, period_ticks)
 *     PIDp_NowUs()          ->  xTaskGetTickCount() (or a TIM, see example 09)
 *   The important part - an ABSOLUTE deadline rather than a relative delay -
 *   is identical in both.
 */
#include <stdio.h>
#include <math.h>

#include "pidx/pid.h"
#include "pid_posix.h"
#include "ex_plant.h"
#include "ex_report.h"

#define PERIOD_US 2000U            /* 500 Hz */
#define DT        0.002
#define N         1500             /* 3 s */

/* ------------------------------------------------------------------------ */
/* Feedforward model                                                         */
/* ------------------------------------------------------------------------ */

typedef struct {
    double ke;          /**< back-EMF constant [V/(rad/s)]  */
    double r;           /**< winding resistance [ohm]       */
    double kt;          /**< torque constant [Nm/A]         */
    double b;           /**< viscous friction [Nm/(rad/s)]  */
    long   calls;
} FfModel;

/**
 * Model-based feedforward for a DC motor velocity loop.
 *
 * Solve the steady state for the voltage that HOLDS the commanded speed:
 *     0 = Kt*i - B*w        ->  i = B*w/Kt
 *     v = R*i + Ke*w        ->  v = (R*B/Kt + Ke) * w
 *
 * This is not a fudge factor. It is the plant's own inverse, evaluated at the
 * setpoint, and it is what lets the feedback term shrink to just the modelling
 * error instead of having to generate the entire operating voltage.
 *
 * The callback must be fast and reentrant: it runs inside PID_Update, which
 * on a real target is inside a timer ISR.
 */
static PID_Float ff_motor(PID_Float setpoint, PID_Float measurement, void *ctx)
{
    FfModel *m = (FfModel *)ctx;
    (void)measurement;              /* pure feedforward: setpoint only */
    m->calls++;
    return (PID_Float)((((m->r * m->b) / m->kt) + m->ke) * (double)setpoint);
}

/* ------------------------------------------------------------------------ */

typedef struct {
    double y[N];
    double u[N];
    double e_peak;
    double u_fb_peak;               /* peak |feedback| contribution */
    EX_Step m;
} Run;

/**
 * @param ff_mode 0 = none, 1 = static value, 2 = model callback
 * @param real_dt true: feed PID_UpdateDt the measured interval
 */
static void run(Run *o, int ff_mode, bool real_dt, FfModel *model)
{
    PID_Handle pid;
    PID_Config c;
    EX_Motor   m;
    PIDp_Loop  loop;
    const double target = 120.0;    /* rad/s */
    int i;

    ex_motor_init(&m);

    PID_ConfigDefault(&c);
    c.core.kp = 0.08f;
    c.core.ki = 3.0f;
    c.core.kd = 0.0f;
    c.core.sample_time = (PID_Float)DT;
    c.limits.use_output_limits = true;
    c.limits.output_min = -24.0f;
    c.limits.output_max =  24.0f;
    c.integral.mode = PID_AW_BACK_CALCULATION;
    c.integral.kt   = 20.0f;

    if (ff_mode == 1) {
        /* Static feedforward: one number, valid at one operating point. */
        c.feedforward.enabled = true;
        c.feedforward.value   = (PID_Float)((((1.0 * 2.0e-3) / 0.05) + 0.05)
                                            * target);
        c.feedforward.gain    = 1.0f;
    } else if (ff_mode == 2) {
        c.feedforward.enabled = true;
        c.feedforward.fn      = ff_motor;
        c.feedforward.ctx     = model;
        c.feedforward.gain    = 1.0f;
    } else {
        /* no feedforward */
    }

    (void)PID_Init(&pid, &c);
    PID_SetSetpoint(&pid, (PID_Float)target);
    ex_step_init(&o->m, 0.0, target, 0.02);

    o->e_peak    = 0.0;
    o->u_fb_peak = 0.0;

    (void)PIDp_LoopInit(&loop, PERIOD_US);

    for (i = 0; i < N; i++) {
        double dt_meas = PIDp_LoopWait(&loop);
        PID_Float u;
        PID_Status st;

        /* The whole point of the absolute-deadline loop: dt_meas is what
         * actually elapsed. Feeding it to PID_UpdateDt keeps Ki*dt and
         * Kd/dt correct even when the OS was late. */
        if (real_dt) {
            u = PID_UpdateDt(&pid, (PID_Float)m.w, (PID_Float)dt_meas);
        } else {
            u = PID_Update(&pid, (PID_Float)m.w);
        }

        ex_motor_step(&m, (double)u, dt_meas);

        (void)PID_GetStatus(&pid, &st);
        {
            double fb = fabs((double)(st.p_term + st.i_term + st.d_term));
            if (fb > o->u_fb_peak) { o->u_fb_peak = fb; }
        }
        if ((i > 400) && (fabs(target - m.w) > o->e_peak)) {
            o->e_peak = fabs(target - m.w);
        }

        o->y[i] = m.w;
        o->u[i] = (double)u;
        ex_step_update(&o->m, m.w, (double)u, dt_meas);
    }

    printf("      loop: %u iterations, %u overruns, mean rate %.1f Hz,"
           " worst lateness %llu us\n",
           (unsigned)loop.iterations, (unsigned)loop.overruns,
           PIDp_LoopMeanRate(&loop),
           (unsigned long long)loop.worst_lateness);
}

int main(void)
{
    static Run none, stat_ff, model_ff;
    FfModel model = { 0.05, 1.0, 0.05, 2.0e-3, 0 };

    printf("Example 08 - RTOS task pattern + feedforward\n");
    printf("  500 Hz velocity loop on a DC motor, target 120 rad/s\n");
    printf("  scheduler: absolute-deadline (platform/posix), dt measured\n\n");

    printf("  [no feedforward]\n");
    run(&none, 0, true, &model);
    printf("  [static feedforward]\n");
    run(&stat_ff, 1, true, &model);
    printf("  [model callback feedforward]\n");
    run(&model_ff, 2, true, &model);

    printf("\n  Feedforward computes the actuator command the plant MODEL\n"
           "  says is needed; feedback then only has to correct the error in\n"
           "  that model. v_ff = (R*B/Kt + Ke) * w = %.4f * w\n\n",
           ((1.0 * 2.0e-3) / 0.05) + 0.05);

    ex_step_header();
    ex_step_report(&none.m,     "no feedforward");
    ex_step_report(&stat_ff.m,  "static FF");
    ex_step_report(&model_ff.m, "model FF (callback)");

    printf("\n  %-24s %16s %16s\n",
           "", "peak |feedback|", "steady error");
    printf("  %-24s %16s %16s\n",
           "------------------------", "----------------", "----------------");
    printf("  %-24s %16.3f %16.4f\n",
           "no feedforward", none.u_fb_peak, none.e_peak);
    printf("  %-24s %16.3f %16.4f\n",
           "static FF", stat_ff.u_fb_peak, stat_ff.e_peak);
    printf("  %-24s %16.3f %16.4f\n",
           "model FF (callback)", model_ff.u_fb_peak, model_ff.e_peak);
    printf("\n  The callback ran %ld times.\n", model.calls);

    printf("\n  Static vs callback: the static value is correct at exactly one\n"
           "  setpoint. The callback is correct at every setpoint, which is\n"
           "  what matters as soon as the command moves - see below.\n");

    /* ------------------------------------------------------------------ */
    /* Why the callback wins: a changing setpoint                          */
    /* ------------------------------------------------------------------ */
    printf("\n  [changing setpoint: 120 -> 40 -> 200 rad/s]\n");
    {
        const int  n_seg = 800;
        int mode;

        /*
         * Measured over the TRANSIENT after each setpoint change, not over the
         * steady state. The first version of this table averaged the steady
         * state and showed feedforward making things very slightly worse - a
         * true measurement of the wrong thing.
         *
         * At steady state the integrator has already produced whatever
         * voltage the plant needs, so feedforward has nothing left to
         * contribute; all it can do is shift work between the I term and the
         * FF term. Its value is entirely in how fast the loop GETS there,
         * because the integrator no longer has to charge up from zero on
         * every setpoint change.
         */
        printf("  %-24s %14s %14s %14s\n",
               "", "IAE (transient)", "peak |error|", "settle [s]");
        printf("  %-24s %14s %14s %14s\n",
               "------------------------", "--------------",
               "--------------", "--------------");

        for (mode = 0; mode < 3; mode++) {
            PID_Handle pid;
            PID_Config c;
            EX_Motor   mm;
            double iae = 0.0, epk = 0.0, settle_sum = 0.0;
            int i;

            ex_motor_init(&mm);
            PID_ConfigDefault(&c);
            c.core.kp = 0.08f;
            c.core.ki = 3.0f;
            c.core.sample_time = (PID_Float)DT;
            c.limits.use_output_limits = true;
            c.limits.output_min = -24.0f;
            c.limits.output_max =  24.0f;
            c.integral.mode = PID_AW_BACK_CALCULATION;
            c.integral.kt   = 20.0f;
            if (mode == 1) {
                c.feedforward.enabled = true;
                c.feedforward.value = (PID_Float)(0.09 * 120.0);
            } else if (mode == 2) {
                c.feedforward.enabled = true;
                c.feedforward.fn  = ff_motor;
                c.feedforward.ctx = &model;
            } else {
                /* none */
            }
            (void)PID_Init(&pid, &c);

            {
                double seg_settle = 0.0;
                for (i = 0; i < (3 * n_seg); i++) {
                    double sp = (i < n_seg) ? 120.0
                              : ((i < (2 * n_seg)) ? 40.0 : 200.0);
                    PID_Float u;
                    double e;

                    PID_SetSetpoint(&pid, (PID_Float)sp);
                    u = PID_Update(&pid, (PID_Float)mm.w);
                    ex_motor_step(&mm, (double)u, DT);

                    e = fabs(sp - mm.w);
                    /* Only the first 0.4 s after each change. */
                    if ((i % n_seg) < 200) {
                        iae += e * DT;
                        if (e > epk) { epk = e; }
                        if (e > (0.02 * sp)) {
                            seg_settle = (double)(i % n_seg) * DT;
                        }
                    }
                    if ((i % n_seg) == 199) {
                        settle_sum += seg_settle;
                        seg_settle = 0.0;
                    }
                }
            }
            printf("  %-24s %14.4f %14.4f %14.4f\n",
                   (mode == 0) ? "no feedforward"
                               : ((mode == 1) ? "static FF (tuned at 120)"
                                              : "model FF (callback)"),
                   iae, epk, settle_sum / 3.0);
        }
        printf("\n  Now the picture is the right way round. Feedforward buys\n"
               "  TRANSIENT performance, not steady-state accuracy - the\n"
               "  integrator already owns the steady state.\n\n"
               "  The static value is tuned for 120 rad/s, so it is a help at\n"
               "  120, a hindrance at 40 (it commands too much) and only\n"
               "  partial at 200. The callback re-evaluates the model at every\n"
               "  setpoint and is right at all three.\n");
    }

    /* ------------------------------------------------------------------ */
    /* PID_UpdateEx                                                        */
    /* ------------------------------------------------------------------ */
    printf("\n  [PID_UpdateEx: everything in one struct]\n");
    {
        PID_Handle pid;
        PID_Config c;
        PID_Input  in;
        PID_StatusCode err = PID_OK;
        EX_Motor   mm;
        int i;

        ex_motor_init(&mm);
        PID_ConfigDefault(&c);
        c.core.kp = 0.08f;
        c.core.ki = 3.0f;
        c.core.sample_time = (PID_Float)DT;
        c.limits.use_output_limits = true;
        c.limits.output_min = -24.0f;
        c.limits.output_max =  24.0f;
        c.feedforward.enabled = true;
        c.feedforward.value   = 0.0f;
        (void)PID_Init(&pid, &c);

        for (i = 0; i < 600; i++) {
            PID_Float u;

            /* InputInit sets every optional field to NaN, which means "keep
             * what the handle already has". Fill only what changes. */
            PID_InputInit(&in);
            in.measurement = (PID_Float)mm.w;
            in.setpoint    = 150.0f;
            in.dt          = (PID_Float)DT;
            in.feedforward = ff_motor(150.0f, (PID_Float)mm.w, &model);

            u = PID_UpdateEx(&pid, &in, &err);
            if (err != PID_OK) {
                printf("      error: %s\n", PID_StatusToString(err));
                break;
            }
            ex_motor_step(&mm, (double)u, DT);
        }
        printf("      settled at %.3f rad/s (target 150), last status: %s\n",
               mm.w, PID_StatusToString(err));
        printf("      One call carries measurement, setpoint, dt, feedforward,\n"
               "      tracking input and the gain-scheduling variable. Any\n"
               "      field left as NaN keeps the handle's current value.\n");
    }

    /* ------------------------------------------------------------------ */
    printf("\n  ----------------------------------------------------------\n");
    printf("  FreeRTOS version of the loop body above:\n\n");
    printf("    void vControlTask(void *arg) {\n");
    printf("        TickType_t last = xTaskGetTickCount();\n");
    printf("        const TickType_t period = pdMS_TO_TICKS(2);\n");
    printf("        for (;;) {\n");
    printf("            vTaskDelayUntil(&last, period);   /* absolute */\n");
    printf("            float y = read_sensor();\n");
    printf("            float u = PID_Update(&pid, y);\n");
    printf("            write_actuator(u);\n");
    printf("        }\n");
    printf("    }\n\n");
    printf("  vTaskDelayUntil, not vTaskDelay: the first is an absolute\n");
    printf("  deadline and does not drift, the second adds the task body's\n");
    printf("  execution time to every period. Same reasoning as the POSIX\n");
    printf("  loop used above, and as PIDs_Rate on bare metal.\n\n");
    printf("  Give the task a priority above anything that can block it, and\n");
    printf("  never call a blocking API between the delay and PID_Update -\n");
    printf("  a mutex wait there turns your fixed sample time into a random\n");
    printf("  one, and every Ki*dt in the controller becomes wrong.\n");

    return 0;
}

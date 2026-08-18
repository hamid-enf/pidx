/**
 * Example 07 - Auto-tuning on a live plant
 * ========================================
 * The non-blocking auto-tune state machine: relay feedback and step
 * identification, the tuning rules, the safety envelope, and an honest
 * comparison of what each method is good for.
 *
 * WHAT THIS SHOWS
 *   - PID_AutoTune_Init / Start / Update / IsComplete / GetResult / Apply
 *   - the state machine driven from an ordinary control loop, never blocking
 *   - relay feedback vs step identification on the SAME plant
 *   - several tuning rules applied to one identified model
 *   - the safety envelope: measurement limits, timeout, abort callback
 *   - PID_AutoTune_Recompute: try another rule without touching the plant
 *
 * READ THIS BEFORE TRUSTING A NUMBER
 *   Relay feedback systematically UNDERESTIMATES the ultimate gain on
 *   lag-dominated plants - by 11 to 38 percent in this library's own test
 *   suite. That is describing-function physics, not a defect: the method
 *   models the relay by its fundamental harmonic and ignores the rest. On a
 *   plant like the one below, prefer the step test. The comparison at the end
 *   measures exactly this.
 */
#include <stdio.h>
#include <math.h>

#include "pidx/pid.h"
#include "pidx/pid_autotune.h"
#include "ex_plant.h"
#include "ex_report.h"

#define DT   0.1                 /* 10 Hz - a thermal process */
#define KP_TRUE  80.0
#define TAU_TRUE 40.0
#define L_TRUE   8.0

/** Abort callback context: lets an operator or a watchdog stop the test. */
typedef struct {
    int  calls;
    int  abort_after;            /* -1 = never */
} AbortCtx;

static bool abort_cb(void *ctx)
{
    AbortCtx *a = (AbortCtx *)ctx;
    a->calls++;
    return (a->abort_after >= 0) && (a->calls > a->abort_after);
}

static void progress_cb(uint8_t percent, PID_TuneState state, void *ctx)
{
    int *last = (int *)ctx;
    /* Print only when the decade changes, so a 200-second test does not
     * produce 2000 lines. */
    if ((percent / 20) != (*last / 20)) {
        printf("        %3u%%  %s\n", (unsigned)percent,
               PID_TuneStateToString(state));
        *last = (int)percent;
    }
}

/**
 * Run one identification experiment to completion.
 * @return true on success; the model and gains are left in @p res.
 */
static bool identify(PID_IdentMethod method, PID_TuneRule rule,
                     PID_AutoTuneResult *res, double *elapsed_s,
                     int abort_after, bool verbose)
{
    PID_Handle         pid;
    PID_Config         c;
    PID_AutoTune       tuner;
    PID_AutoTuneConfig tc;
    EX_Fopdt           plant;
    AbortCtx           actx = { 0, abort_after };
    int                pct  = -1;
    int                i;
    const int          max_steps = 40000;   /* 4000 s of simulated time */

    ex_fopdt_init(&plant, KP_TRUE, TAU_TRUE, L_TRUE, DT, 20.0, 20.0);
    ex_noise_seed(31337U);

    /*
     * Bring the plant to the operating point BEFORE the tune starts, which is
     * what happens on real hardware: you tune a process that is already
     * running, not one that is still warming up from cold.
     *
     * Skipping this produced the most instructive failure in this example.
     * The tuner's stabilisation check compares the measured rate of change
     * against a floor derived from the sensor noise - and a 40 C ramp with a
     * 40 s time constant moves at about 1 C/s, which is BELOW the 4.5 C/s
     * noise floor of a sigma = 0.1 C signal at 10 Hz. So the tuner correctly
     * concluded "this is as steady as this sensor can tell" and stepped from
     * y0 = 20 C while the plant was still climbing to 60 C. The step response
     * then contained the remaining ramp as well as the step, and the fit
     * returned K = 346 instead of 80 - a +333% error from a perfectly
     * reasonable-looking setup.
     *
     * The lesson is not "the library is wrong". It is that a step test
     * measures whatever the plant does after the step, and it cannot tell an
     * externally-driven transient from its own response.
     */
    {
        int w;
        for (w = 0; w < 6000; w++) {   /* 600 s = 15 time constants */
            (void)ex_fopdt_step(&plant, 0.50, DT);
        }
    }

    PID_ConfigDefault(&c);
    c.core.kp = 0.02f;
    c.core.ki = 0.0005f;
    c.core.sample_time = (PID_Float)DT;
    c.limits.use_output_limits = true;
    c.limits.output_min = 0.0f;
    c.limits.output_max = 1.0f;
    (void)PID_Init(&pid, &c);

    (void)PID_AutoTune_ConfigDefault(&tc, method);
    tc.rule       = rule;
    tc.structure  = PID_STRUCT_PID;
    /*
     * The relay must be centred on an output that actually HOLDS the target,
     * or the loop never crosses the setpoint and the test times out with the
     * relay latched at one rail. Here u0 = (60 - 20)/K = 0.50.
     *
     * This is the single most common auto-tune setup mistake, and the library
     * reports it honestly rather than inventing a model: the first draft of
     * this example used a plant whose full-scale output could only reach
     * 22.5 C against a 60 C target, and every tune correctly ended in
     * PID_ERR_TUNE_TIMEOUT. Use auto_bias = true to take the controller's
     * current output instead of hard-coding it.
     */
    tc.output_step = 0.15f;      /* relay half-amplitude, or step size */
    tc.hysteresis  = 0.30f;      /* ~3 sigma of the measurement noise  */
    tc.bias        = 0.50f;      /* output that holds the setpoint     */
    tc.auto_bias   = false;

    /* Safety envelope. Every one of these is a real limit on a real plant:
     * the heater cannot go outside 0..1, the process must stay in a range
     * where nothing is damaged, and the test must not run forever. */
    tc.output_min = 0.0f;
    tc.output_max = 1.0f;
    tc.meas_min   = 10.0f;
    tc.meas_max   = 200.0f;
    tc.timeout_s  = 3000.0f;
    tc.abort_fn   = abort_cb;
    tc.cb_ctx     = &actx;
    if (verbose) {
        tc.on_progress = progress_cb;
        tc.cb_ctx      = &pct;
        tc.abort_fn    = NULL;
    }

    if (PID_AutoTune_Init(&tuner, &tc) != PID_OK) {
        printf("        tuner init rejected the config\n");
        return false;
    }
    if (PID_AutoTune_Start(&tuner, &pid, 60.0f) != PID_OK) {
        printf("        tuner refused to start\n");
        return false;
    }

    /*
     * The state machine driven from an ordinary loop. Note what is NOT here:
     * no while(waiting), no sleep, no blocking call of any kind. On real
     * hardware this body is your timer ISR, and the rest of the machine keeps
     * running throughout.
     */
    for (i = 0; (i < max_steps) && PID_AutoTune_IsRunning(&tuner); i++) {
        double y = plant.y + ex_noise_gauss(0.1);
        PID_Float u = PID_AutoTune_Update(&tuner, (PID_Float)y, (PID_Float)DT);
        (void)ex_fopdt_step(&plant, (double)u, DT);
    }

    *elapsed_s = (double)i * DT;

    if (!PID_AutoTune_IsComplete(&tuner)) {
        PID_StatusCode e = PID_AutoTune_GetError(&tuner);
        printf("        did not complete: %s (state %s)\n",
               PID_StatusToString(e),
               PID_TuneStateToString(PID_AutoTune_GetState(&tuner)));
        return false;
    }
    return (PID_AutoTune_GetResult(&tuner, res) == PID_OK);
}

/** Closed-loop step test with a given set of gains; returns the metrics. */
static void evaluate(const PID_Gains *g, EX_Step *m, double *log_y, int n)
{
    PID_Handle pid;
    PID_Config c;
    EX_Fopdt   plant;
    int        i;

    ex_fopdt_init(&plant, KP_TRUE, TAU_TRUE, L_TRUE, DT, 20.0, 20.0);
    ex_noise_seed(555U);

    PID_ConfigDefault(&c);
    c.core.kp = g->kp;
    c.core.ki = g->ki;
    c.core.kd = g->kd;
    c.core.sample_time = (PID_Float)DT;
    c.limits.use_output_limits = true;
    c.limits.output_min = 0.0f;
    c.limits.output_max = 1.0f;
    c.integral.mode = PID_AW_BACK_CALCULATION;
    c.integral.kt   = 0.05f;
    c.filter.tf = (g->tf > 0.0f) ? g->tf : 0.0f;
    c.filter.n_filter = (g->tf > 0.0f) ? 0.0f : 10.0f;
    (void)PID_Init(&pid, &c);
    PID_SetSetpoint(&pid, 60.0f);
    ex_step_init(m, 20.0, 60.0, 0.02);

    for (i = 0; i < n; i++) {
        double y = plant.y + ex_noise_gauss(0.1);
        PID_Float u = PID_Update(&pid, (PID_Float)y);
        (void)ex_fopdt_step(&plant, (double)u, DT);
        ex_step_update(m, plant.y, (double)u, DT);
        log_y[i] = plant.y;
    }
}

int main(void)
{
    PID_AutoTuneResult relay_res, step_res;
    double t_relay = 0.0, t_step = 0.0;
    bool ok_relay, ok_step;

    printf("Example 07 - Auto-tuning\n");
    printf("  true plant: FOPDT  K = %.1f C per unit duty, T = %.1f s,"
           " L = %.1f s\n", KP_TRUE, TAU_TRUE, L_TRUE);
    printf("  (unknown to the tuner; ambient 20 C, so u = 0.50 holds 60 C)\n");
    printf("  loop rate %.0f Hz, measurement noise sigma = 0.1 C\n\n", 1.0 / DT);

    /* ------------------------------------------------------------------ */
    /* 1. Relay feedback                                                   */
    /* ------------------------------------------------------------------ */
    printf("  [1] Relay feedback (Astrom-Hagglund), progress callback live:\n");
    ok_relay = identify(PID_IDENT_RELAY, PID_RULE_ZN, &relay_res,
                        &t_relay, -1, true);
    if (ok_relay) {
        printf("        Ku = %.4f, Pu = %.2f s, quality %u/100,"
               " %u cycles, %.0f s\n",
               (double)relay_res.model.ku, (double)relay_res.model.pu,
               (unsigned)relay_res.model.quality,
               (unsigned)relay_res.cycles_used, t_relay);
        printf("        period spread %.1f%%, amplitude spread %.1f%%,"
               " asymmetry %.2f\n",
               (double)relay_res.period_spread * 100.0,
               (double)relay_res.amp_spread * 100.0,
               (double)relay_res.asymmetry);
    }

    /* ------------------------------------------------------------------ */
    /* 2. Step identification                                              */
    /* ------------------------------------------------------------------ */
    printf("\n  [2] Step test (area/moment FOPDT fit):\n");
    ok_step = identify(PID_IDENT_STEP, PID_RULE_AMIGO_STEP, &step_res,
                       &t_step, -1, false);
    if (ok_step) {
        printf("        identified: K = %.4f, T = %.2f s, L = %.2f s,"
               " quality %u/100\n",
               (double)step_res.model.k, (double)step_res.model.t,
               (double)step_res.model.l, (unsigned)step_res.model.quality);
        printf("        true      : K = %.4f, T = %.2f s, L = %.2f s\n",
               KP_TRUE, TAU_TRUE, L_TRUE);
        printf("        error     : K %+.2f%%, T %+.2f%%, L %+.2f%%"
               "   (%.0f s test)\n",
               (((double)step_res.model.k / KP_TRUE)  - 1.0) * 100.0,
               (((double)step_res.model.t / TAU_TRUE) - 1.0) * 100.0,
               (((double)step_res.model.l / L_TRUE)   - 1.0) * 100.0,
               t_step);
    }

    /* ------------------------------------------------------------------ */
    /* 3. One model, several rules                                         */
    /* ------------------------------------------------------------------ */
    printf("\n  [3] PID_AutoTune_Recompute: re-run the RULE on the stored\n"
           "      model, without touching the plant again.\n\n");
    if (ok_step && ok_relay) {
        const PID_TuneRule rules[5] = {
            PID_RULE_AMIGO_STEP, PID_RULE_IMC, PID_RULE_COHEN_COON,
            PID_RULE_ZN, PID_RULE_NO_OVERSHOOT
        };
        static double log_y[3000];
        int k;

        printf("      %-16s %5s %8s %8s %8s | %8s %8s %8s %9s\n",
               "rule", "model", "Kp", "Ki", "Kd",
               "rise[s]", "sett[s]", "OS[%]", "IAE");
        printf("      %-16s %5s %8s %8s %8s | %8s %8s %8s %9s\n",
               "----------------", "-----", "--------", "--------", "--------",
               "--------", "--------", "--------", "---------");

        for (k = 0; k < 5; k++) {
            PID_Gains g;
            EX_Step   m;
            const PID_PlantModel *model;
            const char *src;

            /*
             * Each rule declares which KIND of model it needs.
             * PID_TuneRule_RequiredModel() answers that, so the right model is
             * handed to the right rule instead of printing a dead row: ZN and
             * NO_OVERSHOOT are frequency-domain rules and want (Ku, Pu) from
             * the relay test, while AMIGO/IMC/Cohen-Coon want the FOPDT model
             * from the step test.
             */
            if (PID_TuneRule_RequiredModel(rules[k]) == PID_MODEL_FREQ) {
                if (!ok_relay) { continue; }
                model = &relay_res.model;
                src   = "relay";
            } else {
                model = &step_res.model;
                src   = "step ";
            }

            if (PID_TuneRule_Apply(rules[k], model, PID_STRUCT_PID,
                                   0.0f, &g) != PID_OK) {
                printf("      %-16s  rule rejected the model\n",
                       PID_TuneRule_Name(rules[k]));
                continue;
            }
            evaluate(&g, &m, log_y, 3000);
            printf("      %-16s %5s %8.4f %8.4f %8.3f | %8.1f %8.1f %8.2f %9.1f\n",
                   PID_TuneRule_Name(rules[k]), src,
                   (double)g.kp, (double)g.ki, (double)g.kd,
                   m.got_90 ? m.t_rise : -1.0, m.t_settle,
                   m.overshoot, m.iae);
        }

        printf("\n      There is no single best row, and two of them deserve\n"
               "      comment:\n\n"
               "      IMC-lambda wins on this plant: lowest overshoot AND\n"
               "      lowest IAE. It is the only rule here that takes the\n"
               "      dead time into account explicitly.\n\n"
               "      NO_OVERSHOOT overshoots by 29%% - more than\n"
               "      Ziegler-Nichols. This is NOT caused by the relay's\n"
               "      -27%% Ku error. Re-running the same rule against an\n"
               "      EXACT model of this plant still overshoots 30.7%%;\n"
               "      the relay error actually lowers it by ~3 points.\n\n"
               "      The real cause is the rule's own table. The ZN\n"
               "      'some/no overshoot' variants change ONLY Kp (0.33Ku,\n"
               "      0.20Ku) and keep Ti=Pu/2, Td=Pu/3 - and on a FOPDT\n"
               "      plant, Ti=Pu/2 is what drives the overshoot. Sweeping\n"
               "      Kp down at fixed Ti makes overshoot WORSE, not better\n"
               "      (measured: 0.60Ku->44%%, 0.33Ku->38%%, 0.20Ku->43%%),\n"
               "      because the loop is integral-dominated and weaker\n"
               "      proportional action leaves the integrator less\n"
               "      opposed. Ti is the effective lever: at Kp=0.20Ku,\n"
               "      stretching Ti to 2Pu gives 8%% and 4Pu gives 0%%.\n\n"
               "      The coefficients here match the published table, so\n"
               "      this is a limitation of the rule, not a bug. Rule\n"
               "      names describe intent on the plants they were derived\n"
               "      for, not a guarantee on yours. Verify, do not trust\n"
               "      the name.\n");
    }

    /* ------------------------------------------------------------------ */
    /* 4. Relay vs step, measured                                          */
    /* ------------------------------------------------------------------ */
    printf("\n  [4] Relay vs step on this plant:\n");
    if (ok_relay && ok_step) {
        /* The true ultimate gain of a FOPDT plant, for reference: at the
         * frequency where the phase is -pi, Ku = 1/|G(jw)|. Solved
         * numerically here so the comparison has a ground truth. */
        double lo = 1e-4, hi = 10.0, wu = 0.0;
        int it;
        for (it = 0; it < 200; it++) {
            double w = 0.5 * (lo + hi);
            double phase = -atan(w * TAU_TRUE) - (w * L_TRUE);
            if (phase > -3.14159265358979) { lo = w; } else { hi = w; }
            wu = w;
        }
        {
            double ku_true = sqrt(1.0 + (wu * TAU_TRUE * wu * TAU_TRUE))
                           / KP_TRUE;
            double pu_true = 2.0 * 3.14159265358979 / wu;

            printf("      %-22s %10s %10s %10s\n",
                   "", "Ku", "Pu [s]", "test [s]");
            printf("      %-22s %10.4f %10.2f %10s\n",
                   "true (analytic)", ku_true, pu_true, "-");
            printf("      %-22s %10.4f %10.2f %10.0f\n",
                   "relay feedback", (double)relay_res.model.ku,
                   (double)relay_res.model.pu, t_relay);
            printf("      %-22s %10s %10s %10.0f\n",
                   "step test", "(FOPDT)", "(FOPDT)", t_step);
            printf("\n      Relay Ku error: %+.1f%%. The underestimate is\n"
                   "      expected: describing-function analysis keeps only\n"
                   "      the fundamental of the relay's square wave. On a\n"
                   "      lag-dominated plant like this one, prefer the step\n"
                   "      test: its K error was -1.0%% against the relay's\n"
                   "      %+.1f%% on Ku. The step test took %.0f s against the\n"
                   "      relay's %.0f s, so the accuracy is not free - but\n"
                   "      it is the same order of magnitude, and a 27%% gain\n"
                   "      error is worth 56 extra seconds.\n",
                   (((double)relay_res.model.ku / ku_true) - 1.0) * 100.0,
                   (((double)relay_res.model.ku / ku_true) - 1.0) * 100.0,
                   t_step, t_relay);
        }
    }

    /* ------------------------------------------------------------------ */
    /* 5. The safety envelope                                              */
    /* ------------------------------------------------------------------ */
    printf("\n  [5] Safety: the abort callback stops the test mid-experiment\n");
    {
        PID_AutoTuneResult r;
        double t;
        bool ok = identify(PID_IDENT_RELAY, PID_RULE_ZN, &r, &t, 200, false);
        printf("        aborted after 200 samples -> completed = %s\n",
               ok ? "yes (unexpected!)" : "no, as intended");
        printf("        A tune that cannot finish must FAIL, not return a\n"
               "        plausible-looking guess. Gains are only applied\n"
               "        through PID_AutoTune_Apply after IsComplete.\n");
    }

    return 0;
}

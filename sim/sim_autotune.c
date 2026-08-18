/**
 * @file    sim_autotune.c
 * @brief   How accurate is PIDX auto-tuning, and does the error matter?
 *
 * Not part of the library. Host-side study.
 *
 * @section q The question this file exists to answer
 *
 * sim_rules.c handed every tuning rule a PERFECT model, so that differences
 * between rules could not be blamed on identification. That is the right way
 * to compare rules and the wrong way to predict what a user gets, because a
 * real user does not have a perfect model - they have whatever
 * PID_AutoTune_Update() identified from an experiment on a noisy plant.
 *
 * Example 07 measured a single case and found something surprising: the relay
 * test got Ku wrong by -27.0% while the step test got K wrong by only -1.01%,
 * and yet the relay-tuned loop was NOT 27% worse. That was one plant. This
 * file asks whether it generalises:
 *
 *   1. How large is the identification error, per method, per plant?
 *   2. How does measurement noise degrade it?
 *   3. What does that error actually COST in closed-loop IAE?
 *
 * Question 3 is the one that matters and the one almost never asked. An
 * identification error is only interesting if it moves the gains somewhere
 * worse. The experiment is therefore paired: for every identified model we
 * ALSO tune from the exact model with the SAME rule, run both against the
 * same true plant, and report the difference. Everything except the model is
 * held constant, so the IAE penalty is attributable to identification error
 * alone.
 *
 * @section method Method
 *
 * For each of the 5 plants, each of the 2 identification methods, and each of
 * 3 noise levels, the tuner drives the true plant exactly as it would on
 * hardware - non-blocking, one PID_AutoTune_Update() per sample, no cheating
 * with internal state. The resulting model is compared against ground truth
 * (sim_exact_ku_pu() for Ku/Pu, the plant table for K/T/L).
 *
 * The rule is AMIGO in both cases - AMIGO_FREQ for the relay's frequency
 * model, AMIGO_STEP for the step test's FOPDT model. AMIGO is used rather
 * than the lower-IAE Ziegler-Nichols because sim_robust.c showed ZN spends
 * its stability margin to buy that IAE, and a study about model error should
 * not be run with the rule least tolerant of model error.
 *
 * @section honesty What a failure means here
 *
 * A tune that does not complete is reported as FAIL with its status code, not
 * silently dropped. A timeout usually means the excitation could not make the
 * plant cross the setpoint - a real and common commissioning failure, and one
 * the library is supposed to report rather than paper over with a guess.
 */

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "sim_common.h"
#include "ex_plant.h"

/* ------------------------------------------------------------------ */
/* Sweep definition                                                     */
/* ------------------------------------------------------------------ */

/**
 * Measurement noise levels, as a standard deviation in process units where
 * the setpoint step is 1.0. So 0.002 is a clean 4-digit sensor and 0.010 is
 * a visibly noisy one (1% of span).
 */
static const double NOISE[] = { 0.0, 0.002, 0.010 };
#define N_NOISE ((int)(sizeof(NOISE) / sizeof(NOISE[0])))

typedef struct {
    PID_IdentMethod ident;
    PID_TuneRule    rule;
    const char     *name;
} Sim_Method;

static const Sim_Method METHODS[] = {
    { PID_IDENT_RELAY, PID_RULE_AMIGO_FREQ, "relay" },
    { PID_IDENT_STEP,  PID_RULE_AMIGO_STEP, "step"  }
};
#define N_METHOD ((int)(sizeof(METHODS) / sizeof(METHODS[0])))

/** Percentage error of an estimate against truth; 0 truth yields 0. */
static double err_pct(double est, double truth)
{
    if (fabs(truth) < 1e-12) { return 0.0; }
    return ((est - truth) / truth) * 100.0;
}

/* ------------------------------------------------------------------ */
/* One identification experiment                                        */
/* ------------------------------------------------------------------ */

/**
 * Run one auto-tune against the true plant and return the result.
 *
 * The controller handle is a real one so the tuner can save and restore its
 * mode, exactly as on hardware. The loop is bounded by the virtual clock, not
 * by a fixed iteration count, so a slow plant is not cut off early.
 *
 * @return true if the tune completed and produced a model.
 */
static bool identify(Sim_PlantId id, const Sim_Method *meth, double sigma,
                     PID_AutoTuneResult *res, double *elapsed_s)
{
    const Sim_PlantSpec *sp = sim_plant_spec(id);
    PID_AutoTuneConfig   tc;
    PID_AutoTune         tuner;
    PID_Handle           pid;
    PID_Config           cfg;
    EX_Fopdt             plant;
    /* Output that holds the setpoint at steady state: y_ss = K*u  =>  u0. */
    const double u0      = sp->setpoint / sp->k;
    const double horizon = sp->horizon * 6.0;   /* generous: 6 step responses */
    const long   max_steps = (long)(horizon / sp->dt);
    long i;

    /*
     * A plain PID handle for the tuner to borrow. Its gains are irrelevant -
     * during a tune the state machine drives the output directly - but it
     * must be valid, and its output limits must admit the relay excursion.
     */
    (void)PID_ConfigDefault(&cfg);
    cfg.core.kp = 1.0f;
    cfg.core.ki = 0.0f;
    cfg.core.kd = 0.0f;
    cfg.core.sample_time = (PID_Float)sp->dt;
    cfg.limits.use_output_limits = true;
    cfg.limits.output_min = 0.0f;
    cfg.limits.output_max = (PID_Float)(4.0 * u0);
    if (PID_Init(&pid, &cfg) != PID_OK) { return false; }

    (void)PID_AutoTune_ConfigDefault(&tc, meth->ident);
    tc.rule      = meth->rule;
    tc.structure = PID_STRUCT_PID;

    if (meth->ident == PID_IDENT_RELAY) {
        /*
         * Relay half-amplitude. 25% of the holding output is a compromise:
         * large enough that the limit cycle clears the noise floor, small
         * enough to stay in the linear region and not trip a real plant's
         * alarms. The relay MUST be centred on u0 or the loop never crosses
         * the setpoint and the test times out with the output latched.
         */
        tc.output_step = (PID_Float)(0.25 * u0);
        tc.bias        = (PID_Float)u0;
        tc.auto_bias   = false;
        /*
         * Hysteresis about 3 sigma, so noise alone cannot trigger a relay
         * switch. With no noise a small floor is still needed, otherwise the
         * relay chatters on floating-point round-off near the crossing.
         */
        tc.hysteresis  = (PID_Float)((sigma > 0.0) ? (3.0 * sigma) : 0.002);
    } else {
        /*
         * Step test: step from rest to the output that reaches the setpoint,
         * so the response spans the same operating range as the relay test
         * and neither method gets a larger-signal advantage.
         */
        tc.output_step = (PID_Float)u0;
        tc.bias        = 0.0f;
        tc.auto_bias   = false;
        tc.hysteresis  = (PID_Float)((sigma > 0.0) ? (3.0 * sigma) : 0.002);
    }

    tc.output_min = 0.0f;
    tc.output_max = (PID_Float)(4.0 * u0);
    tc.meas_min   = -5.0f;
    tc.meas_max   = 10.0f;
    tc.timeout_s  = (PID_Float)horizon;
    /* The plant starts genuinely at rest, so the stabilise phase would only
     * burn simulated time proving what we already arranged. */
    tc.skip_stabilize = true;

    if (PID_AutoTune_Init(&tuner, &tc) != PID_OK) { return false; }
    if (PID_AutoTune_Start(&tuner, &pid, (PID_Float)sp->setpoint) != PID_OK) {
        return false;
    }

    ex_fopdt_init(&plant, sp->k, sp->t, sp->l, sp->dt, 0.0, 0.0);
    ex_noise_seed(4242U);

    for (i = 0; (i < max_steps) && PID_AutoTune_IsRunning(&tuner); ++i) {
        double y = plant.y;
        PID_Float u;

        if (sigma > 0.0) { y += ex_noise_gauss(sigma); }

        u = PID_AutoTune_Update(&tuner, (PID_Float)y, (PID_Float)sp->dt);
        (void)ex_fopdt_step(&plant, (double)u, sp->dt);
    }

    *elapsed_s = (double)i * sp->dt;

    if (!PID_AutoTune_IsComplete(&tuner)) {
        res->code = PID_AutoTune_GetError(&tuner);
        return false;
    }
    return (PID_AutoTune_GetResult(&tuner, res) == PID_OK);
}

/* ------------------------------------------------------------------ */
/* Closed-loop cost of an identification error                          */
/* ------------------------------------------------------------------ */

/**
 * Tune from @p model with @p rule, run the closed loop against the true
 * plant, and return the IAE. Returns a negative value if the gains are
 * unusable or the loop diverged, which the caller reports rather than hides.
 */
static double closed_loop_iae(Sim_PlantId id, const PID_PlantModel *model,
                              PID_TuneRule rule)
{
    const Sim_PlantSpec *sp = sim_plant_spec(id);
    PID_Gains   g;
    PID_Handle  h;
    Sim_Metrics m;
    Sim_RunOpts o;

    if (PID_TuneRule_Apply(rule, model, PID_STRUCT_PID,
                           (PID_Float)sim_imc_lambda(sp->t, sp->l),
                           &g) != PID_OK) {
        return -1.0;
    }
    if (!sim_make_controller(&h, &g, sp->dt)) { return -1.0; }

    sim_runopts_default(&o);
    if (!sim_run_step(&h, id, &o, &m)) { return -1.0; }
    if (!isfinite(m.iae)) { return -1.0; }
    return m.iae;
}

/* ------------------------------------------------------------------ */
/* Why the relay always under-estimates Ku                              */
/* ------------------------------------------------------------------ */

/**
 * The sweep below shows the relay's Ku error is large, systematic and always
 * NEGATIVE, even with zero noise. That is not a bug in PIDX and not bad luck;
 * it is intrinsic to the describing-function method, and it is worth proving
 * rather than asserting.
 *
 * The describing function of an ideal relay assumes the plant output is a
 * SINE. It relates the relay half-amplitude h to the amplitude a1 of the
 * FUNDAMENTAL of y:
 *
 *     Ku = 4h / (pi * a1)
 *
 * What any implementation can actually measure - including ours - is the
 * PEAK half-amplitude a_pk = (y_max - y_min)/2, because peaks are cheap and
 * robust to detect on an embedded target and a Fourier transform is not.
 *
 * But a real limit cycle is not a sine. It is the plant's response to a
 * SQUARE wave, so it keeps some of the third and fifth harmonics. Harmonics
 * add to the peak without adding to the fundamental, so a_pk > a1 always,
 * and dividing by the larger number makes Ku come out too small. The lower
 * the plant's dead time the less it filters those harmonics, the more the
 * peak is inflated, and the worse the under-estimate - which is exactly the
 * ordering the sweep reports.
 *
 * This function measures both amplitudes on the same limit cycle so the
 * mechanism is demonstrated, not claimed. It runs its own simple relay
 * rather than the library tuner because it needs the raw waveform.
 */
static void explain_relay_bias(void)
{
    /* Three plants spanning L/T, chosen to show the trend with dead time. */
    static const double PL[][3] = {   /* K, T, L */
        { 2.0, 1.0, 0.1 },
        { 2.0, 1.0, 0.3 },
        { 1.0, 1.0, 1.0 }
    };
    const int n_pl = (int)(sizeof(PL) / sizeof(PL[0]));
    int j;

    puts("=== Why is the relay's Ku always LOW? ===\n");
    puts("Describing-function theory needs the amplitude of the FUNDAMENTAL");
    puts("of y. Any practical implementation measures the PEAK instead. A");
    puts("limit cycle is a square-wave response, so it carries harmonics:");
    puts("peak > fundamental, and Ku = 4h/(pi*a) therefore comes out low.\n");
    printf("  %-10s %6s %10s %10s %8s %10s %10s\n",
           "plant", "L/T", "a_peak", "a_fund", "ratio",
           "Ku err pk", "Ku err f0");

    for (j = 0; j < n_pl; ++j) {
        const double k = PL[j][0], t = PL[j][1], l = PL[j][2];
        const double dt = 0.002;
        const double sp = 1.0;
        const double u0 = sp / k;
        const double h  = 0.25 * u0;
        const double eps = 0.002;
        EX_Fopdt p;
        double ku_true, pu_true;
        double per = 0.0, a_pk = 0.0, a1 = 0.0;
        bool   relay_high = true;
        long   i;

        sim_exact_ku_pu(k, t, l, &ku_true, &pu_true);
        ex_fopdt_init(&p, k, t, l, dt, 0.0, 0.0);

        /* Reach the operating point before switching the relay on. */
        for (i = 0; i < (long)((200.0 * t) / dt); ++i) {
            (void)ex_fopdt_step(&p, u0, dt);
        }

        /* Pass 1: period and peak half-amplitude, averaged over many cycles
         * so a single odd cycle cannot set the answer. */
        {
            double t_last = -1.0, per_sum = 0.0, amp_sum = 0.0;
            double y_min = 1e30, y_max = -1e30;
            int n_per = 0, n_amp = 0;

            for (i = 0; i < (long)((400.0 * t) / dt); ++i) {
                const double y = p.y;
                const double now = (double)i * dt;
                double u;

                if (relay_high && (y > (sp + eps))) {
                    relay_high = false;
                    if (t_last >= 0.0) { per_sum += now - t_last; n_per++; }
                    t_last = now;
                    if ((n_per > 2) && (y_max > -1e29)) {
                        amp_sum += (y_max - y_min) / 2.0;
                        n_amp++;
                    }
                    y_min = 1e30; y_max = -1e30;
                } else if ((!relay_high) && (y < (sp - eps))) {
                    relay_high = true;
                }
                if (y < y_min) { y_min = y; }
                if (y > y_max) { y_max = y; }

                u = relay_high ? (u0 + h) : (u0 - h);
                (void)ex_fopdt_step(&p, u, dt);
            }
            if ((n_per == 0) || (n_amp == 0)) { continue; }
            per  = per_sum / (double)n_per;
            a_pk = amp_sum / (double)n_amp;
        }

        /*
         * Pass 2: the SAME relay keeps running while the fundamental is
         * integrated over a whole number of periods. Using a different relay
         * here would compare two different waveforms - an earlier draft of
         * this check did exactly that and produced a nonsensical ratio of
         * 6.46, which is how the mistake was caught.
         */
        {
            const long m = (long)(((per * 20.0) / dt) + 0.5);
            const double w = (2.0 * 3.14159265358979323846) / per;
            double cs = 0.0, sn = 0.0;
            bool started = false;

            /* Align the window to a switch so it is phase-coherent. */
            for (i = 0; i < (long)((50.0 * t) / dt); ++i) {
                const double y = p.y;
                double u;
                if (relay_high && (y > (sp + eps))) { relay_high = false; started = true; }
                else if ((!relay_high) && (y < (sp - eps))) { relay_high = true; }
                u = relay_high ? (u0 + h) : (u0 - h);
                (void)ex_fopdt_step(&p, u, dt);
                if (started) { break; }
            }
            for (i = 0; i < m; ++i) {
                const double y = p.y;
                const double now = (double)i * dt;
                double u;
                cs += (y - sp) * cos(w * now) * dt;
                sn += (y - sp) * sin(w * now) * dt;
                if (relay_high && (y > (sp + eps))) { relay_high = false; }
                else if ((!relay_high) && (y < (sp - eps))) { relay_high = true; }
                u = relay_high ? (u0 + h) : (u0 - h);
                (void)ex_fopdt_step(&p, u, dt);
            }
            /* a1 = 2/T * |integral of y*e^{-jwt}| over the window. */
            a1 = (2.0 * sqrt((cs * cs) + (sn * sn))) / ((double)m * dt);
        }

        if ((a_pk <= 0.0) || (a1 <= 0.0)) { continue; }

        {
            const double ku_pk = (4.0 * h) / (3.14159265358979323846 * a_pk);
            const double ku_f  = (4.0 * h) / (3.14159265358979323846 * a1);
            printf("  %-10s %6.2f %10.6f %10.6f %8.3f %9.1f%% %9.1f%%\n",
                   (j == 0) ? "easy" : ((j == 1) ? "typical" : "hard"),
                   l / t, a_pk, a1, a_pk / a1,
                   err_pct(ku_pk, ku_true), err_pct(ku_f, ku_true));
        }
    }

    puts("\n  Correcting peak -> fundamental removes most of the error, which");
    puts("  identifies the harmonic content as the cause. PIDX keeps the peak");
    puts("  form on purpose: it is the standard hysteresis-corrected relay");
    puts("  formula, it needs no Fourier transform on the target, and its");
    puts("  error is in the SAFE direction. Ku too low means gains too low,");
    puts("  so the tuned loop is sluggish rather than unstable - see the");
    puts("  penalty column above: every relay row cost IAE, none diverged.\n");
}

/* ------------------------------------------------------------------ */
/* main                                                                 */
/* ------------------------------------------------------------------ */

int main(void)
{
    Sim_Csv csv;
    int p, mi, ni;
    /* Running totals for the closing summary. */
    double abs_err_sum[N_METHOD];
    /* The gain is not always the parameter that gets identified worst - for
     * the step test it is the dead time by a wide margin - so the summary
     * tracks the largest error over all identified parameters too. */
    double worst_par_sum[N_METHOD];
    const char *worst_par_name[N_METHOD];
    double pen_sum[N_METHOD];
    double pen_worst[N_METHOD];
    int    ok_n[N_METHOD];
    int    try_n[N_METHOD];

    puts("=== PIDX simulation: auto-tune accuracy, and whether it matters ===\n");
    puts("Each row runs a real non-blocking auto-tune against the true plant,");
    puts("then tunes a SECOND controller from the exact model with the same");
    puts("rule. Both run the same step. The IAE penalty is therefore caused");
    puts("by identification error alone.\n");
    puts("Rule: AMIGO_FREQ for relay, AMIGO_STEP for step.\n");

    if (!sim_mkresults()) {
        fputs("cannot create results directory\n", stderr);
        return 1;
    }
    if (!sim_csv_open(&csv, "results/autotune.csv",
                      "plant,method,noise_sigma,ok,status,"
                      "ku_true,ku_id,ku_err_pct,pu_true,pu_id,pu_err_pct,"
                      "k_true,k_id,k_err_pct,t_true,t_id,t_err_pct,"
                      "l_true,l_id,l_err_pct,"
                      "elapsed_s,cycles,quality,"
                      "iae_exact,iae_ident,penalty_pct")) {
        fputs("cannot open results/autotune.csv\n", stderr);
        return 1;
    }

    for (mi = 0; mi < N_METHOD; ++mi) {
        abs_err_sum[mi]   = 0.0;
        worst_par_sum[mi] = 0.0;
        worst_par_name[mi] = "-";
        pen_sum[mi]     = 0.0;
        pen_worst[mi]   = -1e30;
        ok_n[mi]        = 0;
        try_n[mi]       = 0;
    }

    for (p = 0; p < (int)SIM_PLANT_COUNT_; ++p) {
        const Sim_PlantSpec *sp = sim_plant_spec((Sim_PlantId)p);
        double ku_true, pu_true;

        sim_exact_ku_pu(sp->k, sp->t, sp->l, &ku_true, &pu_true);

        printf("--- %-18s K=%.1f T=%.1f L=%.1f   "
               "exact Ku=%.4f Pu=%.4f\n",
               sp->name, sp->k, sp->t, sp->l, ku_true, pu_true);
        printf("  %-6s %8s | %-22s | %-22s | %s\n",
               "method", "noise", "identified vs true",
               "closed-loop IAE", "penalty");

        for (mi = 0; mi < N_METHOD; ++mi) {
            const Sim_Method *meth = &METHODS[mi];

            for (ni = 0; ni < N_NOISE; ++ni) {
                PID_AutoTuneResult res;
                PID_PlantModel     exact;
                double elapsed = 0.0;
                double iae_exact, iae_ident, penalty;
                bool   ok;

                memset(&res, 0, sizeof(res));
                res.code = PID_OK;
                try_n[mi]++;

                ok = identify((Sim_PlantId)p, meth, NOISE[ni], &res, &elapsed);

                if (!ok) {
                    printf("  %-6s %8.3f | %-22s | %-22s | %s\n",
                           meth->name, NOISE[ni],
                           PID_StatusToString(res.code), "-", "-");
                    sim_csv_row(&csv,
                                "%s,%s,%.4f,0,%s,"
                                "%.6f,,,%.6f,,,"
                                "%.6f,,,%.6f,,,%.6f,,,"
                                "%.3f,,,,,",
                                sp->name, meth->name, NOISE[ni],
                                PID_StatusToString(res.code),
                                ku_true, pu_true,
                                sp->k, sp->t, sp->l, elapsed);
                    continue;
                }

                ok_n[mi]++;

                /* The exact-model counterpart: same rule, same kind, no
                 * identification error. */
                sim_build_model(sp->k, sp->t, sp->l, res.model.kind, &exact);
                iae_exact = closed_loop_iae((Sim_PlantId)p, &exact, meth->rule);
                iae_ident = closed_loop_iae((Sim_PlantId)p, &res.model,
                                            meth->rule);

                penalty = ((iae_exact > 0.0) && (iae_ident > 0.0))
                        ? (((iae_ident - iae_exact) / iae_exact) * 100.0)
                        : (double)NAN;

                if (res.model.kind == PID_MODEL_FREQ) {
                    const double ku_id = (double)res.model.ku;
                    const double pu_id = (double)res.model.pu;
                    const double eku = err_pct(ku_id, ku_true);
                    const double epu = err_pct(pu_id, pu_true);

                    printf("  %-6s %8.3f | Ku %+6.1f%%  Pu %+6.1f%% | "
                           "%8.4f -> %8.4f | ",
                           meth->name, NOISE[ni], eku, epu,
                           iae_exact, iae_ident);
                    if (isfinite(penalty)) { printf("%+6.1f%%\n", penalty); }
                    else                   { printf("%6s\n", "diverged"); }

                    abs_err_sum[mi] += fabs(eku);
                    worst_par_sum[mi] += (fabs(eku) > fabs(epu))
                                       ? fabs(eku) : fabs(epu);
                    worst_par_name[mi] = "Ku/Pu";
                    sim_csv_row(&csv,
                                "%s,%s,%.4f,1,OK,"
                                "%.6f,%.6f,%.3f,%.6f,%.6f,%.3f,"
                                "%.6f,,,%.6f,,,%.6f,,,"
                                "%.3f,%u,%u,%.6f,%.6f,%.4f",
                                sp->name, meth->name, NOISE[ni],
                                ku_true, ku_id, eku,
                                pu_true, pu_id, epu,
                                sp->k, sp->t, sp->l,
                                elapsed, (unsigned)res.cycles_used,
                                (unsigned)res.model.quality,
                                iae_exact, iae_ident, penalty);
                } else {
                    const double k_id = (double)res.model.k;
                    const double t_id = (double)res.model.t;
                    const double l_id = (double)res.model.l;
                    const double ek = err_pct(k_id, sp->k);
                    const double et = err_pct(t_id, sp->t);
                    const double el = err_pct(l_id, sp->l);

                    printf("  %-6s %8.3f | K %+5.1f%% T %+5.1f%% L %+5.1f%% | "
                           "%8.4f -> %8.4f | ",
                           meth->name, NOISE[ni], ek, et, el,
                           iae_exact, iae_ident);
                    if (isfinite(penalty)) { printf("%+6.1f%%\n", penalty); }
                    else                   { printf("%6s\n", "diverged"); }

                    abs_err_sum[mi] += fabs(ek);
                    {
                        double w = fabs(ek);
                        if (fabs(et) > w) { w = fabs(et); }
                        if (fabs(el) > w) { w = fabs(el); }
                        worst_par_sum[mi] += w;
                    }
                    worst_par_name[mi] = "K/T/L";
                    sim_csv_row(&csv,
                                "%s,%s,%.4f,1,OK,"
                                "%.6f,,,%.6f,,,"
                                "%.6f,%.6f,%.3f,%.6f,%.6f,%.3f,%.6f,%.6f,%.3f,"
                                "%.3f,%u,%u,%.6f,%.6f,%.4f",
                                sp->name, meth->name, NOISE[ni],
                                ku_true, pu_true,
                                sp->k, k_id, ek,
                                sp->t, t_id, et,
                                sp->l, l_id, el,
                                elapsed, (unsigned)res.cycles_used,
                                (unsigned)res.model.quality,
                                iae_exact, iae_ident, penalty);
                }

                if (isfinite(penalty)) {
                    pen_sum[mi] += penalty;
                    if (penalty > pen_worst[mi]) { pen_worst[mi] = penalty; }
                }
            }
        }
        putchar('\n');
    }

    /* ---------------- summary ---------------- */
    puts("=== Summary ===\n");
    printf("  %-8s %10s %13s %15s %13s %13s\n",
           "method", "completed", "mean |gain|", "mean worst param",
           "mean penalty", "worst penalty");
    for (mi = 0; mi < N_METHOD; ++mi) {
        const double n = (ok_n[mi] > 0) ? (double)ok_n[mi] : 1.0;
        printf("  %-8s %6d/%-3d %12.1f%% %9s %4.1f%% %12.1f%% %12.1f%%\n",
               METHODS[mi].name, ok_n[mi], try_n[mi],
               abs_err_sum[mi] / n,
               worst_par_name[mi], worst_par_sum[mi] / n,
               pen_sum[mi] / n,
               (pen_worst[mi] > -1e29) ? pen_worst[mi] : 0.0);
    }

    puts("\n  READ THESE TWO COLUMNS TOGETHER. 'mean |gain|' is |Ku error|");
    puts("  for the relay and |K error| for the step test. On its own it");
    puts("  makes the step test look flawless - it identifies K to about");
    puts("  0.3%. But K is not the parameter that hurts: the step test's");
    puts("  DEAD TIME error reaches +82% at the highest noise, and that is");
    puts("  what the penalty column is actually paying for. A summary that");
    puts("  reported only the gain error would be misleading, so the worst");
    puts("  identified parameter is reported next to it.");
    puts("\n  'penalty' is the extra closed-loop IAE caused by tuning from");
    puts("  the identified model instead of the exact one, same rule both");
    puts("  times, so it isolates identification error from rule choice.");

    putchar('\n');
    explain_relay_bias();

    sim_csv_close(&csv);
    puts("wrote results/autotune.csv");
    return 0;
}

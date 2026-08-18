/**
 * @file    sim_rules.c
 * @brief   Which tuning rule should you actually use? Measured, not asserted.
 *
 * Every PID library ships a list of tuning rules and leaves you to guess.
 * This sweep applies each rule to an EXACTLY known plant model - no
 * identification error in the loop - and scores the resulting closed loop on
 * five plants spanning normalised dead time L/T from 0.1 to 1.0.
 *
 * Because the model handed to the rule is exact, everything measured here is
 * the property of the RULE. That separation is the whole point: sim_robust.c
 * then re-runs the winners with a deliberately wrong model to see which of
 * them survive being wrong, which is the situation you are actually in.
 *
 * Reading the output:
 *   IAE   total tracking cost - the headline "how good was the response"
 *   ITAE  weights late error - punishes slow settling, rewards finishing
 *   OS%   overshoot against the COMMANDED step
 *   u_TV  total variation of the control signal - actuator wear. A rule that
 *         wins on IAE while tripling u_TV has not won.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "sim_common.h"
#include "pidx/pid_autotune.h"

/* FREQ rules need Ku/Pu; FOPDT rules need K/T/L. Both are derived from the
 * same true plant so no rule gets an information advantage. */
static const PID_TuneRule FREQ_RULES[] = {
    PID_RULE_ZN, PID_RULE_TYREUS_LUYBEN, PID_RULE_PESSEN,
    PID_RULE_SOME_OVERSHOOT, PID_RULE_NO_OVERSHOOT, PID_RULE_AMIGO_FREQ
};
static const PID_TuneRule FOPDT_RULES[] = {
    PID_RULE_COHEN_COON, PID_RULE_AMIGO_STEP, PID_RULE_IMC
};

#define N_FREQ  ((int)(sizeof(FREQ_RULES) / sizeof(FREQ_RULES[0])))
#define N_FOPDT ((int)(sizeof(FOPDT_RULES) / sizeof(FOPDT_RULES[0])))
#define N_RULES (N_FREQ + N_FOPDT)

int main(void)
{
    Sim_Csv csv;
    Sim_Csv trace;
    int p, r;
    double best_iae[SIM_PLANT_COUNT_];
    int    best_rule[SIM_PLANT_COUNT_];
    double score_iae[N_RULES];
    int    score_n[N_RULES];
    int    diverged[N_RULES];

    puts("=== PIDX simulation: tuning rule comparison ===\n");
    puts("Each rule receives an EXACT model of the plant, so every difference");
    puts("below belongs to the rule, not to identification error.");
    puts("Actuator limited to +/-2 with back-calculation anti-windup.\n");

    if (!sim_mkresults()) {
        fprintf(stderr, "cannot create results/\n");
        return 1;
    }
    if (!sim_csv_open(&csv, "results/rules.csv",
                      "plant,L_over_T,rule,model,kp,ki,kd,"
                      "rise_s,settle_s,overshoot_pct,iae,itae,u_tv,sse")) {
        return 1;
    }
    if (!sim_csv_open(&trace, "results/rules_trace.csv",
                      "series,t,setpoint,y,u")) {
        sim_csv_close(&csv);
        return 1;
    }

    for (r = 0; r < N_RULES; ++r) {
        score_iae[r] = 0.0;
        score_n[r] = 0;
        diverged[r] = 0;
    }

    for (p = 0; p < (int)SIM_PLANT_COUNT_; ++p) {
        const Sim_PlantSpec *sp = sim_plant_spec((Sim_PlantId)p);
        const double lt = sp->l / sp->t;

        best_iae[p] = 1e300;
        best_rule[p] = -1;

        printf("--- plant %-18s K=%.1f T=%.1f L=%.1f  L/T=%.2f  dt=%.3f s\n",
               sp->name, sp->k, sp->t, sp->l, lt, sp->dt);
        sim_metrics_header();

        for (r = 0; r < N_RULES; ++r) {
            const bool is_freq = (r < N_FREQ);
            const PID_TuneRule rule = is_freq ? FREQ_RULES[r]
                                              : FOPDT_RULES[r - N_FREQ];
            const PID_ModelKind kind = is_freq ? PID_MODEL_FREQ : PID_MODEL_FOPDT;
            PID_PlantModel model;
            PID_Gains g;
            PID_Handle h;
            Sim_Metrics m;
            PID_StatusCode rc;
            char label[64];

            sim_build_model(sp->k, sp->t, sp->l, kind, &model);

            /* lambda only matters to IMC; the other rules ignore it. */
            rc = PID_TuneRule_Apply(rule, &model, PID_STRUCT_PID,
                                    (PID_Float)sim_imc_lambda(sp->t, sp->l),
                                    &g);

            (void)snprintf(label, sizeof(label), "%s", PID_TuneRule_Name(rule));

            if (rc != PID_OK) {
                /* COHEN_COON is documented as valid only for L/T in [0.1,1].
                 * Outside it the library refuses instead of extrapolating -
                 * that refusal is a feature and is recorded as such. */
                printf("  %-22s  rejected: %s\n", label, PID_StatusToString(rc));
                sim_csv_row(&csv, "%s,%.3f,%s,%s,,,,,,,,,,",
                            sp->name, lt, label, is_freq ? "FREQ" : "FOPDT");
                continue;
            }

            if (!sim_make_controller(&h, &g, sp->dt)) {
                printf("  %-22s  unusable gains kp=%.4f ki=%.4f kd=%.4f\n",
                       label, (double)g.kp, (double)g.ki, (double)g.kd);
                continue;
            }

            {
                Sim_RunOpts o;
                sim_runopts_default(&o);
                /* Trace only the typical plant, or the CSV becomes unreadable. */
                if (p == (int)SIM_PLANT_TYPICAL) {
                    o.trace = &trace;
                    o.trace_tag = label;
                }
                if (!sim_run_step(&h, (Sim_PlantId)p, &o, &m)) {
                    printf("  %-22s  DIVERGED\n", label);
                    diverged[r]++;
                    sim_csv_row(&csv, "%s,%.3f,%s,%s,%.5f,%.5f,%.5f,"
                                "diverged,,,,,,",
                                sp->name, lt, label, is_freq ? "FREQ" : "FOPDT",
                                (double)g.kp, (double)g.ki, (double)g.kd);
                    continue;
                }
            }

            sim_metrics_report(&m, label);
            sim_csv_row(&csv,
                        "%s,%.3f,%s,%s,%.5f,%.5f,%.5f,"
                        "%.4f,%.4f,%.3f,%.5f,%.4f,%.4f,%.6f",
                        sp->name, lt, label, is_freq ? "FREQ" : "FOPDT",
                        (double)g.kp, (double)g.ki, (double)g.kd,
                        m.t_rise, m.t_settle, m.overshoot,
                        m.iae, m.itae, m.u_tv, sim_metrics_sse(&m));

            score_iae[r] += m.iae / (sp->k * sp->t);   /* normalised */
            score_n[r]++;
            if (m.iae < best_iae[p]) {
                best_iae[p] = m.iae;
                best_rule[p] = r;
            }
        }
        putchar('\n');
    }

    /* ---------------- summary ---------------- */
    puts("=== Ranking by normalised IAE, averaged over the plants a rule survived ===\n");
    printf("  %-22s %6s %10s %10s\n", "rule", "plants", "mean nIAE", "diverged");
    {
        /* Simple selection sort: N_RULES is 9. */
        int order[N_RULES];
        int i, j;
        for (i = 0; i < N_RULES; ++i) { order[i] = i; }
        for (i = 0; i < N_RULES - 1; ++i) {
            for (j = i + 1; j < N_RULES; ++j) {
                const double a = (score_n[order[i]] > 0)
                    ? score_iae[order[i]] / score_n[order[i]] : 1e300;
                const double b = (score_n[order[j]] > 0)
                    ? score_iae[order[j]] / score_n[order[j]] : 1e300;
                if (b < a) { const int tmp = order[i]; order[i] = order[j]; order[j] = tmp; }
            }
        }
        for (i = 0; i < N_RULES; ++i) {
            const int k = order[i];
            const PID_TuneRule rule = (k < N_FREQ) ? FREQ_RULES[k]
                                                   : FOPDT_RULES[k - N_FREQ];
            if (score_n[k] == 0) {
                printf("  %-22s %6d %10s %10d\n",
                       PID_TuneRule_Name(rule), 0, "-", diverged[k]);
            } else {
                printf("  %-22s %6d %10.4f %10d\n",
                       PID_TuneRule_Name(rule), score_n[k],
                       score_iae[k] / score_n[k], diverged[k]);
            }
        }
    }

    puts("\n=== Best rule per plant (lowest IAE) ===");
    for (p = 0; p < (int)SIM_PLANT_COUNT_; ++p) {
        const int k = best_rule[p];
        const PID_TuneRule rule = (k < 0) ? PID_RULE_ZN
                                 : ((k < N_FREQ) ? FREQ_RULES[k]
                                                 : FOPDT_RULES[k - N_FREQ]);
        printf("  %-20s -> %s\n", sim_plant_spec((Sim_PlantId)p)->name,
               (k < 0) ? "none survived" : PID_TuneRule_Name(rule));
    }

    puts("\nLowest IAE is NOT automatically the right choice: check u_TV and");
    puts("overshoot in results/rules.csv, and read sim_robust's verdict before");
    puts("committing to a rule. A rule that wins here on an exact model can");
    puts("still be the first to go unstable when the model is wrong.");
    puts("\nwrote results/rules.csv and results/rules_trace.csv");

    sim_csv_close(&trace);
    sim_csv_close(&csv);
    return 0;
}

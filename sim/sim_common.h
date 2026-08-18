/**
 * @file    sim_common.h
 * @brief   Shared metrics, CSV output and plant bank for the PIDX simulations.
 *
 * Not part of the library.
 *
 * @section why Why sim/ exists when examples/ already runs closed loops
 *
 * An example answers "how do I use this feature?" - one plant, one scenario,
 * printed for a human to read. A simulation answers "which choice is better,
 * and by how much, across conditions I did not hand-pick?" - many plants, many
 * controllers, written to CSV for a machine to plot.
 *
 * The distinction matters because a single flattering plant can make any
 * tuning rule look good. Everything in sim/ therefore sweeps: over plants,
 * over rules, over model error, over sample rate.
 *
 * @section reuse Plants are reused, not re-written
 *
 * The physics lives in examples/common/ex_plant.c and is included from here
 * rather than copied. Two divergent copies of the same motor model would be a
 * guaranteed source of "the example says X but the simulation says Y".
 *
 * @section honesty What these numbers are
 *
 * Host-side, double-precision plant, float controller, fixed-seed noise. They
 * are exact and reproducible for the models given. They are NOT measurements
 * of your plant, and a tuning rule that wins here can still lose on hardware
 * with unmodelled dynamics. Every metric below is defined against the
 * COMMANDED setpoint, never against the achieved final value, so a controller
 * with steady-state error cannot be graded on its own curve.
 */
#ifndef PIDX_SIM_COMMON_H
#define PIDX_SIM_COMMON_H

#include <stdio.h>
#include <stdbool.h>

#include "pidx/pid.h"
#include "pidx/pid_autotune.h"

/*
 * The simulations compare tuning rules, so they need the auto-tune module.
 * That module is compiled out of the MINIMAL and MOTION profiles, where
 * PID_ModelKind and PID_TuneRule_Apply() do not exist. Without this guard
 * the failure surfaces as a confusing "unknown type name 'PID_ModelKind'"
 * from deep inside this header. sim/ is a host-side analysis harness, not
 * firmware, so requiring the full feature set here costs the target nothing.
 */
#if !PIDX_ENABLE_AUTOTUNE
#error "sim/ requires PIDX_ENABLE_AUTOTUNE (use the PROCESS or FULL profile)"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* Metrics                                                                   */
/* ======================================================================== */

/**
 * Step-response figures of merit, accumulated sample by sample.
 *
 * Deliberately a superset of examples/common EX_Step: this one also tracks
 * the control-signal total variation and the peak absolute error, which are
 * what separate a "fast" tuning from an unusable one.
 */
typedef struct {
    double y0;          /**< Process value when the step was applied.        */
    double target;      /**< Commanded setpoint.                             */
    double band;        /**< Settling band as a fraction of the step.        */

    double t;           /**< Elapsed time [s].                               */
    double t_rise;      /**< 10% -> 90% of the commanded change [s].         */
    double t_settle;    /**< Last exit from the settling band [s].           */
    double t_peak;      /**< Time of the largest excursion [s].              */
    double overshoot;   /**< Percent of the commanded change.                */
    double peak;        /**< Extreme process value seen.                     */
    double y_final;
    double e_peak_abs;  /**< Largest |setpoint - y| after the step.          */

    double iae;         /**< Integral |e| dt - overall tracking cost.        */
    double ise;         /**< Integral e^2 dt - penalises big errors.         */
    double itae;        /**< Integral t*|e| dt - penalises SLOW settling.    */
    double u_tv;        /**< Total variation of u - actuator wear proxy.     */
    double u_min, u_max;

    /* internals */
    double t_10, t_90, u_prev;
    bool   got_10, got_90, primed;
    long   n;
} Sim_Metrics;

void   sim_metrics_init(Sim_Metrics *m, double y0, double target, double band);
void   sim_metrics_update(Sim_Metrics *m, double y, double u, double dt);
double sim_metrics_sse(const Sim_Metrics *m);   /**< Signed steady-state error. */

/** One-line labelled summary plus the header that lines up with it. */
void sim_metrics_header(void);
void sim_metrics_report(const Sim_Metrics *m, const char *label);

/* ======================================================================== */
/* CSV                                                                       */
/* ======================================================================== */

/**
 * Minimal CSV writer. Opens the file, writes a header row, then one row per
 * call. Kept dumb on purpose - plotting belongs in plot.py, not in C.
 */
typedef struct {
    FILE *f;
    int   cols;
} Sim_Csv;

bool sim_csv_open(Sim_Csv *c, const char *path, const char *header);
void sim_csv_row(Sim_Csv *c, const char *fmt, ...);
void sim_csv_close(Sim_Csv *c);

/** Create the results directory if it is missing. Returns false on failure. */
bool sim_mkresults(void);

/* ======================================================================== */
/* Plant bank                                                                */
/* ======================================================================== */

/**
 * The plants every sweep runs against.
 *
 * Chosen to span the ratio that actually decides how hard a loop is: the
 * normalised dead time L/T (also called controllability). Below ~0.1 almost
 * any tuning works; above ~1 the loop is dead-time dominated and aggressive
 * rules go unstable. A rule comparison that only uses one L/T is worthless.
 */
typedef enum {
    SIM_PLANT_EASY = 0,     /**< K=2  T=1    L=0.1   L/T = 0.1  lag dominant */
    SIM_PLANT_TYPICAL,      /**< K=2  T=1    L=0.3   L/T = 0.3  the workhorse*/
    SIM_PLANT_HARD,         /**< K=1  T=1    L=1.0   L/T = 1.0  dead-time dom*/
    SIM_PLANT_SLOW,         /**< K=1  T=10   L=1.0   L/T = 0.1  thermal      */
    SIM_PLANT_FURNACE,      /**< K=3  T=20   L=4.0   L/T = 0.2  big thermal  */
    SIM_PLANT_COUNT_
} Sim_PlantId;

typedef struct {
    const char *name;
    double k, t, l;         /**< True FOPDT parameters.                      */
    double dt;              /**< Recommended controller sample time [s].     */
    double horizon;         /**< Simulation length for a step [s].           */
    double setpoint;
} Sim_PlantSpec;

/** @return Static description; index must be < SIM_PLANT_COUNT_. */
const Sim_PlantSpec *sim_plant_spec(Sim_PlantId id);

/* ======================================================================== */
/* Closed-loop runner                                                        */
/* ======================================================================== */

/**
 * Options for one closed-loop run. Zeroed defaults are sane: no noise, no
 * quantisation, no disturbance, no model error.
 */
typedef struct {
    double noise_sigma;     /**< Measurement noise std-dev [process units].  */
    int    adc_bits;        /**< 0 = no quantisation.                        */
    double adc_lo, adc_hi;
    double dist_time;       /**< Time of a load disturbance [s]; 0 = none.   */
    double dist_size;       /**< Additive step on the plant input.           */
    double gain_mismatch;   /**< True plant gain multiplier, 1.0 = none.     */
    double tau_mismatch;    /**< True time-constant multiplier.              */
    double delay_mismatch;  /**< True dead-time multiplier.                  */
    double u_min, u_max;    /**< Actuator limits handed to the controller.   */
    Sim_Csv *trace;         /**< Optional per-sample trace.                  */
    const char *trace_tag;  /**< Series name written into the trace.         */
} Sim_RunOpts;

void sim_runopts_default(Sim_RunOpts *o);

/**
 * Run one step response of @p h against plant @p id and fill @p out.
 *
 * The controller is driven with PID_UpdateDt at the plant's recommended rate.
 * Mismatch factors scale the TRUE plant away from the nominal model the
 * controller was tuned on - that is how robustness is measured here.
 *
 * @return false if the loop diverged (non-finite state), which is itself a
 *         result and is reported rather than hidden.
 */
bool sim_run_step(PID_Handle *h, Sim_PlantId id, const Sim_RunOpts *o,
                  Sim_Metrics *out);

/* ======================================================================== */
/* Exact plant analysis and controller construction                          */
/* ======================================================================== */

/**
 * Exact ultimate gain and period of K*exp(-Ls)/(1+Ts).
 *
 * Solve angle(G(jw)) = -pi  =>  atan(w*T) + w*L = pi  for w = wu, then
 * Ku = 1/|G(j wu)| = sqrt(1 + (wu*T)^2) / K  and  Pu = 2*pi/wu.
 *
 * Bisection rather than a closed form because the equation is transcendental.
 * The bracket is safe: the left side is 0 at w -> 0 and grows monotonically
 * without bound, so exactly one root exists.
 *
 * This is ground truth, not an estimate: it is what lets the sweeps hand a
 * rule a PERFECT model and so attribute every difference to the rule itself.
 */
void sim_exact_ku_pu(double k, double t, double l, double *ku, double *pu);

/**
 * Fill a PID_PlantModel of the requested kind from FOPDT parameters.
 * FREQ models get Ku/Pu from sim_exact_ku_pu(); FOPDT models get K/T/L
 * directly. Both describe the same plant, so no rule gets an information
 * advantage over another.
 */
void sim_build_model(double k, double t, double l, PID_ModelKind kind,
                     PID_PlantModel *m);

/**
 * Standard lambda for IMC: max(T, 8L)/2, a moderately aggressive choice.
 * Stated here rather than buried in each study so every sweep uses one value.
 */
double sim_imc_lambda(double t, double l);

/**
 * Configure @p h from a rule's gains, with a real actuator (+/-2) and
 * back-calculation anti-windup so windup is part of every comparison rather
 * than assumed away. Derivative is filtered at N = 10 and taken on the
 * measurement.
 *
 * @return false if the gains are unusable (non-finite or Kp <= 0) or if
 *         PID_Init rejects the configuration.
 */
bool sim_make_controller(PID_Handle *h, const PID_Gains *g, double dt);

#ifdef __cplusplus
}
#endif

#endif /* PIDX_SIM_COMMON_H */

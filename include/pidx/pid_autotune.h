/**
 * @file    pid_autotune.h
 * @brief   Non-blocking auto-tuning: plant identification + tuning rules.
 *
 * Architecture (see docs/00_architecture.md section 9): identification and the
 * tuning rule are two independent stages joined by an explicit intermediate
 * data type, PID_PlantModel.
 *
 *   IDENTIFICATION            PLANT MODEL              TUNING RULE
 *   relay feedback     ->   FREQ  {Ku, Pu}      ->   ZN, TL, Pessen, ...
 *   open-loop step     ->   FOPDT {K, T, L}     ->   Cohen-Coon, AMIGO, IMC
 *
 * A rule that needs FOPDT parameters cannot be fed from relay data: one point
 * on the Nyquist curve does not determine three model parameters. Asking for
 * such a combination returns PID_ERR_TUNE_MODEL_MISMATCH and the result names
 * the identification method that would work. No invented (Ku,Pu) -> (K,T,L)
 * conversion exists anywhere in this module.
 *
 * The tuner is a state machine. Every call does a bounded amount of work: no
 * blocking loop, no delay, no allocation. It drives the plant itself while
 * running, so the controller handle is placed in MANUAL and fully restored
 * (mode, manual output, gains, integrator) on completion or abort.
 *
 * Layering: this header is downstream of pid.h. The core never includes it.
 */
#ifndef PIDX_PID_AUTOTUNE_H
#define PIDX_PID_AUTOTUNE_H

#include "pidx/pid.h"

#ifdef __cplusplus
extern "C" {
#endif

#if PIDX_ENABLE_AUTOTUNE

/* ======================================================================== */
/* Identification methods                                                    */
/* ======================================================================== */

/**
 * How the plant is excited to obtain a model.
 */
typedef enum {
    /** Closed-loop relay feedback (Astrom-Hagglund). Keeps the process near
     *  the setpoint, needs no steady state to be reached first, and yields the
     *  ultimate gain and period. The safe default for a running process.     */
    PID_IDENT_RELAY = 0,
    /** Open-loop step in MANUAL. Yields a full FOPDT model but drives the
     *  process away from its operating point, so it needs room to move.      */
    PID_IDENT_STEP
} PID_IdentMethod;

/**
 * Which model an identification run produces, and which a rule consumes.
 */
typedef enum {
    PID_MODEL_NONE = 0,
    /** One frequency point: ultimate gain Ku and ultimate period Pu.         */
    PID_MODEL_FREQ,
    /** First-order plus dead time: gain K, time constant T, dead time L.     */
    PID_MODEL_FOPDT
} PID_ModelKind;

/**
 * Controller structure the rule should produce.
 */
typedef enum {
    PID_STRUCT_P = 0,
    PID_STRUCT_PI,
    PID_STRUCT_PID
} PID_TuneStructure;

/* ======================================================================== */
/* Tuning rules                                                              */
/* ======================================================================== */

/**
 * Closed-form tuning rules. The MODEL column states the required model kind;
 * a mismatch is rejected rather than fudged.
 *
 *   rule                model    typical use
 *   ZN                  FREQ     historical reference, aggressive (1/4 decay)
 *   TYREUS_LUYBEN       FREQ     robust, low overshoot - recommended default
 *   PESSEN              FREQ     faster than ZN, more overshoot
 *   SOME_OVERSHOOT      FREQ     servo / motion loops (see note below)
 *   NO_OVERSHOOT        FREQ     NOT overshoot-free - see note below
 *   AMIGO_FREQ          FREQ     modern balanced rule from Ku/Pu
 *   COHEN_COON          FOPDT    valid for L/T in [0.1, 1]
 *   AMIGO_STEP          FOPDT    modern balanced rule - FOPDT default
 *   IMC                 FOPDT    explicit speed/robustness trade-off via lambda
 *   CUSTOM              either   user callback registered at runtime
 *
 * NOTE on SOME_OVERSHOOT / NO_OVERSHOOT: these names describe the intent of
 * the original 1940s table, not a property this library can promise. Both
 * rows only reduce Kp and keep Ti=Pu/2, and on FOPDT plants that Ti is the
 * dominant cause of overshoot. Measured with an EXACT model (no
 * identification error), NO_OVERSHOOT gives 43% overshoot on K=2 T=1 L=0.1
 * and 31% on K=80 T=40 L=8. If overshoot must genuinely be near zero, use
 * IMC (raise lambda) or AMIGO_STEP, or stretch Ti yourself. Full evidence in
 * sim/sim_rules.c and the comment block in src/pid_autotune_rules.c.
 *
 * HOW TO CHOOSE, if you only read one paragraph. The "typical use" column
 * above ranks rules on an exact model. Real models are not exact, and
 * sim/sim_robust.c measured what happens when the plant moves underneath a
 * controller tuned on the nominal model (810 runs: gain, tau and delay each
 * scaled 0.5x..2.0x over 5 plants). Share of cases that stayed stable and
 * settled:
 *
 *   IMC 100%   NO_OVERSHOOT 99%   AMIGO_STEP 98%   AMIGO_FREQ 97%
 *   TYREUS_LUYBEN 86%   SOME_OVERSHOOT 78%   ZN 71%   COHEN_COON 67%
 *   PESSEN 67%
 *
 * That ordering is close to the REVERSE of the exact-model IAE ordering
 * (Spearman rho = -0.59): ZN and COHEN_COON win on a perfect model and are
 * among the first to break on an imperfect one, because their low IAE is
 * bought with stability margin. So: if your model came from a step test and
 * could easily be 30% wrong, prefer AMIGO_STEP - rank 3 on both tables and
 * the lowest worst-case IAE of the whole study. Reach for ZN or COHEN_COON
 * only when you trust the model or can retune on the real plant.
 */
typedef enum {
    PID_RULE_ZN = 0,
    PID_RULE_TYREUS_LUYBEN,
    PID_RULE_PESSEN,
    PID_RULE_SOME_OVERSHOOT,
    PID_RULE_NO_OVERSHOOT,
    PID_RULE_AMIGO_FREQ,
    PID_RULE_COHEN_COON,
    PID_RULE_AMIGO_STEP,
    PID_RULE_IMC,
    PID_RULE_CUSTOM,
    PID_RULE_COUNT_
} PID_TuneRule;

/**
 * Identified plant model. Produced by identification, consumed by a rule.
 */
typedef struct {
    PID_ModelKind kind;

    /* PID_MODEL_FREQ */
    PID_Float ku;            /**< Ultimate gain [output/input].               */
    PID_Float pu;            /**< Ultimate period [s].                        */

    /* PID_MODEL_FOPDT: G(s) = k * exp(-l*s) / (1 + t*s) */
    PID_Float k;             /**< Static gain [output/input].                 */
    PID_Float t;             /**< Time constant [s].                          */
    PID_Float l;             /**< Dead time [s].                              */

    PID_Float noise_sigma;   /**< Noise std-dev estimated during the test.    */
    uint8_t   quality;       /**< 0..100 repeatability / fit quality.         */
} PID_PlantModel;

/** Parallel-form gains plus the derivative filter time constant. */
typedef struct {
    PID_Float kp;
    PID_Float ki;            /**< = kp / ti                                   */
    PID_Float kd;            /**< = kp * td                                   */
    PID_Float ti;            /**< Integral time [s], 0 if no integral action. */
    PID_Float td;            /**< Derivative time [s].                        */
    PID_Float tf;            /**< Derivative filter time constant [s].        */
} PID_Gains;

/**
 * Signature of a user tuning rule.
 * @param m    Identified model (never NULL, kind is never PID_MODEL_NONE).
 * @param s    Requested controller structure.
 * @param out  Receives kp/ki/kd/ti/td; tf may be left 0 to accept the default.
 * @param ctx  Opaque user pointer given to PID_AutoTune_RegisterRule.
 * @return PID_OK, or PID_ERR_TUNE_MODEL_MISMATCH if the model is unusable.
 */
typedef PID_StatusCode (*PID_TuneRuleFn)(const PID_PlantModel *m,
                                         PID_TuneStructure s,
                                         PID_Gains *out,
                                         void *ctx);

/**
 * Apply a tuning rule to a model without running any experiment.
 * Pure function: useful for offline tuning, tests, and re-tuning from a stored
 * model with a different rule.
 *
 * @return PID_OK, PID_ERR_NULL, PID_ERR_INVALID_PARAM,
 *         PID_ERR_TUNE_MODEL_MISMATCH if the rule needs another model kind.
 */
PID_StatusCode PID_TuneRule_Apply(PID_TuneRule rule,
                                  const PID_PlantModel *model,
                                  PID_TuneStructure structure,
                                  PID_Float lambda,
                                  PID_Gains *out);

/** Model kind a built-in rule requires. PID_MODEL_NONE for CUSTOM. */
PID_ModelKind PID_TuneRule_RequiredModel(PID_TuneRule rule);

/** Human-readable rule name, never NULL. */
const char *PID_TuneRule_Name(PID_TuneRule rule);

/* ======================================================================== */
/* State machine                                                             */
/* ======================================================================== */

typedef enum {
    PID_TUNE_IDLE = 0,       /**< Not started, or reset.                      */
    PID_TUNE_STABILIZING,    /**< Waiting for |dy/dt| below threshold.        */
    PID_TUNE_RELAY_WARMUP,   /**< Relay running, early cycles discarded.      */
    PID_TUNE_RELAY_OSC,      /**< Relay running, cycles being recorded.       */
    PID_TUNE_STEP_APPLY,     /**< Step just applied, waiting for departure.   */
    PID_TUNE_STEP_RECORD,    /**< Recording the step response.                */
    PID_TUNE_ANALYZING,      /**< One-shot: raw data -> PID_PlantModel.       */
    PID_TUNE_COMPUTING,      /**< One-shot: model -> gains via the rule.      */
    PID_TUNE_VALIDATING,     /**< One-shot: sanity checks on the gains.       */
    PID_TUNE_COMPLETE,       /**< Success, waiting for PID_AutoTune_Apply.    */
    PID_TUNE_FAILED          /**< Stopped; see PID_AutoTune_GetError.         */
} PID_TuneState;

/** Human-readable state name, never NULL. */
const char *PID_TuneStateToString(PID_TuneState s);

/* ======================================================================== */
/* Configuration                                                             */
/* ======================================================================== */

/** Optional progress callback. Called from PID_AutoTune_Update, so it must be
 *  short and non-blocking if the tuner runs in an ISR. */
typedef void (*PID_TuneProgressFn)(uint8_t percent, PID_TuneState state, void *ctx);
/** Optional completion callback (success or failure). */
typedef void (*PID_TuneDoneFn)(PID_StatusCode code, void *ctx);
/** Optional watchdog, polled every sample. Returning true aborts the tune. */
typedef bool (*PID_TuneAbortFn)(void *ctx);

/** Maximum relay cycles whose statistics are kept. Fixed storage, no malloc. */
#ifndef PIDX_TUNE_MAX_CYCLES
#define PIDX_TUNE_MAX_CYCLES  12U
#endif

typedef struct {
    /* --- experiment selection --- */
    PID_IdentMethod   ident;         /**< Relay or step.                      */
    PID_TuneRule      rule;          /**< Rule applied to the model.          */
    PID_TuneStructure structure;     /**< P / PI / PID.                       */
    PID_Float         lambda;        /**< IMC closed-loop time constant [s].
                                      *   <= 0 selects max(0.5*L, 0.2*T).     */

    /* --- excitation --- */
    PID_Float output_step;           /**< Relay half-amplitude h, or step size
                                      *   for the step test. Must be > 0.     */
    PID_Float hysteresis;            /**< Relay hysteresis eps in measurement
                                      *   units. Rule of thumb: 2..3 sigma of
                                      *   the noise. 0 is allowed but only on
                                      *   a genuinely clean signal.           */
    PID_Float bias;                  /**< Relay centre output u0. Normally the
                                      *   output that holds the setpoint.     */
    bool      auto_bias;             /**< Use the handle's current output as
                                      *   u0 when the tune starts.            */

    /* --- limits and safety --- */
    PID_Float output_min;            /**< Hard clamp on the injected output.  */
    PID_Float output_max;
    PID_Float meas_min;              /**< Abort if measurement leaves this.   */
    PID_Float meas_max;
    PID_Float osc_max;               /**< Abort if peak-to-peak exceeds this
                                      *   (0 disables).                       */
    PID_Float osc_min;               /**< Amplitude below this is "no
                                      *   oscillation" (0 -> 4*hysteresis).   */
    PID_Float rate_max;              /**< Abort if |dy/dt| exceeds this
                                      *   (0 disables).                       */
    PID_Float timeout_s;             /**< Total wall-clock budget [s].        */

    /* --- experiment shape --- */
    uint8_t   warmup_cycles;         /**< Relay cycles discarded (default 2). */
    uint8_t   eval_cycles;           /**< Cycles that must agree (default 4). */
    PID_Float stab_time;             /**< Steady-state dwell required [s].    */
    PID_Float stab_rate;             /**< |dy/dt| threshold for steady state.
                                      *   0 -> derived from hysteresis.       */
    bool      skip_stabilize;        /**< Assume already steady.              */

    /* --- callbacks (all optional) --- */
    PID_TuneProgressFn on_progress;
    PID_TuneDoneFn     on_done;
    PID_TuneAbortFn    abort_fn;
    void              *cb_ctx;
} PID_AutoTuneConfig;

/* ======================================================================== */
/* Result                                                                    */
/* ======================================================================== */

typedef struct {
    PID_PlantModel model;            /**< Identified plant.                   */
    PID_Gains      gains;            /**< Gains produced by the rule.         */
    PID_StatusCode code;             /**< PID_OK on success.                  */
    PID_Float      elapsed_s;        /**< Experiment duration.                */
    PID_Float      amplitude;        /**< Mean half-amplitude of the cycles.  */
    PID_Float      period_spread;    /**< Max relative period deviation.      */
    PID_Float      amp_spread;       /**< Max relative amplitude deviation.   */
    PID_Float      asymmetry;        /**< |a+ - a-| / (a+ + a-).              */
    uint16_t       cycles_used;      /**< Cycles that entered the average.    */
    bool           asymmetric;       /**< Asymmetry above 0.30: result is
                                      *   usable but the plant is nonlinear
                                      *   or biased at this operating point.  */
    /** When code == PID_ERR_TUNE_MODEL_MISMATCH: the identification method
     *  that would produce the model this rule needs. */
    PID_IdentMethod suggested_ident;
} PID_AutoTuneResult;

/* ======================================================================== */
/* Tuner object                                                              */
/* ======================================================================== */

typedef struct {
    uint32_t            magic;
    PID_AutoTuneConfig  cfg;
    PID_Handle         *h;           /**< Controller being tuned (borrowed).  */
    PID_TuneState       state;
    PID_StatusCode      err;

    PID_Float           setpoint;    /**< Operating point of the experiment.  */
    PID_Float           output;      /**< Value currently injected.           */
    PID_Float           elapsed;     /**< Seconds since Start.                */
    PID_Float           state_time;  /**< Seconds in the current state.       */

    /* saved controller state, restored on finish/abort */
    PID_Mode            saved_mode;
    PID_Float           saved_manual;
    PID_Float           saved_sp;
    bool                restored;

    /* relay engine */
    bool                relay_high;  /**< Current relay polarity.             */
    PID_Float           y_prev;
    PID_Float           y_min;       /**< Extremes within the current cycle.  */
    PID_Float           y_max;
    PID_Float           t_last_cross;/**< Time of the last rising switch.     */
    uint16_t            cycle_count;
    uint16_t            cycles_kept;
    PID_Float           per_sum;     /**< Running sums over kept cycles.      */
    PID_Float           per_sq;
    PID_Float           amp_sum;
    PID_Float           amp_pos_sum;
    PID_Float           amp_neg_sum;
    PID_Float           per_min;
    PID_Float           per_max;
    PID_Float           amp_min;
    PID_Float           amp_max;

    /* step engine */
    PID_Float           y0;          /**< Pre-step steady value.              */
    PID_Float           u0;          /**< Pre-step output.                    */
    PID_Float           y_settled;   /**< Running estimate of y_infinity.     */
    PID_Float           t_283;       /**< Time to 28.3% (cross-check only).   */
    PID_Float           t_632;       /**< Time to 63.2% (cross-check only).   */
    bool                got_283;
    bool                got_632;
    PID_Float           area1;       /**< Integral of (y - y0) dt.            */
    PID_Float           moment1;     /**< Integral of t*(y - y0) dt.          */
    PID_Float           t_end;       /**< Time at which the response settled. */
    PID_Float           y_acc;       /**< Sum of y over the settle window.    */
    uint16_t            y_acc_n;     /**< Samples in y_acc.                   */
    PID_Float           y_slow;      /**< Slow LPF of y, for the slope test.  */
    PID_Float           y_slow_prev; /**< Previous slow LPF sample.           */
    PID_Float           settle_timer;/**< Dwell with a flat response.         */
    PID_Float           y_peak;      /**< Largest excursion seen.             */

    /* noise / steady-state detection */
    PID_Float           noise_acc;   /**< Sum of |y - y_prev| for sigma.      */
    uint32_t            noise_n;
    PID_Float           stab_timer;

    /* custom rule */
    PID_TuneRuleFn      rule_fn;
    void               *rule_ctx;

    PID_AutoTuneResult  result;
} PID_AutoTune;

/* ======================================================================== */
/* API                                                                       */
/* ======================================================================== */

/**
 * Fill a config with safe defaults for the given identification method.
 * Relay defaults: h = 0, eps = 0, warmup 2, eval 4, timeout 120 s.
 * The caller must at least set output_step and the output limits.
 */
PID_StatusCode PID_AutoTune_ConfigDefault(PID_AutoTuneConfig *cfg,
                                          PID_IdentMethod ident);

/** Initialise the tuner. Copies the config; validates it. */
PID_StatusCode PID_AutoTune_Init(PID_AutoTune *t, const PID_AutoTuneConfig *cfg);

/**
 * Begin an experiment on @p h around setpoint @p sp.
 * The handle is switched to MANUAL and its mode/output/setpoint are saved.
 * Returns PID_ERR_BUSY if a tune is already running.
 */
PID_StatusCode PID_AutoTune_Start(PID_AutoTune *t, PID_Handle *h, PID_Float sp);

/**
 * Advance the state machine by one sample and return the control output that
 * the caller must write to the actuator. While tuning, this replaces
 * PID_Update: the tuner drives the plant.
 *
 * When the tune has finished the previous controller output is returned and
 * the handle is back in its saved mode, so a caller that keeps calling this
 * function does not shock the plant.
 *
 * @param measurement Current process value.
 * @param dt          Sample interval [s], > 0.
 */
PID_Float PID_AutoTune_Update(PID_AutoTune *t, PID_Float measurement, PID_Float dt);

/** Stop immediately and restore the controller. Result code is TUNE_ABORTED. */
PID_StatusCode PID_AutoTune_Abort(PID_AutoTune *t);

/** True once the tune finished successfully and gains are available. */
bool PID_AutoTune_IsComplete(const PID_AutoTune *t);

/** True while an experiment is driving the plant. */
bool PID_AutoTune_IsRunning(const PID_AutoTune *t);

/** Current state; PID_TUNE_IDLE if @p t is NULL. */
PID_TuneState PID_AutoTune_GetState(const PID_AutoTune *t);

/** Sticky failure code; PID_OK while running or on success. */
PID_StatusCode PID_AutoTune_GetError(const PID_AutoTune *t);

/** Rough completion percentage, 0..100. */
uint8_t PID_AutoTune_GetProgress(const PID_AutoTune *t);

/** Copy out the result. PID_ERR_BUSY if the run has not finished. */
PID_StatusCode PID_AutoTune_GetResult(const PID_AutoTune *t, PID_AutoTuneResult *r);

/**
 * Write the tuned gains into a handle, bumplessly.
 * Uses PID_SetGainsRescaleIntegral so the output does not jump, and also sets
 * the derivative filter time constant from the result.
 *
 * @param h Handle to receive the gains; NULL means the tuned handle.
 */
PID_StatusCode PID_AutoTune_Apply(PID_AutoTune *t, PID_Handle *h);

/** Install a user rule, used when cfg.rule == PID_RULE_CUSTOM. */
PID_StatusCode PID_AutoTune_RegisterRule(PID_AutoTune *t, PID_TuneRuleFn fn, void *ctx);

/**
 * Re-run only the tuning rule on an already identified model, without touching
 * the plant. Lets an operator try a different rule on stored data.
 */
PID_StatusCode PID_AutoTune_Retune(PID_AutoTune *t, PID_TuneRule rule,
                                   PID_TuneStructure structure);

#endif /* PIDX_ENABLE_AUTOTUNE */

#ifdef __cplusplus
}
#endif

#endif /* PIDX_PID_AUTOTUNE_H */

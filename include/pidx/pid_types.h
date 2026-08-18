/**
 * @file    pid_types.h
 * @brief   Enumerations, configuration structures and the PID handle.
 *
 * The handle is deliberately a complete (non-opaque) type so that users can
 * allocate it statically. Its fields are NOT part of the stable API: read and
 * write them through the accessor functions in pid.h. The only exception is
 * documented in PID_SetSetpointImmediate().
 */
#ifndef PIDX_PID_TYPES_H
#define PIDX_PID_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "pid_conf.h"
#include "pid_status.h"
#include "pid_math.h"
#include "pid_version.h"
#if PIDX_ENABLE_INPUT_FILTER
#include "pid_filter.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif


/* ======================================================================== */
/* Behavioural enumerations                                                  */
/* ======================================================================== */

/**
 * Controller action sign.
 * DIRECT  : output must increase to raise the measurement (heater, throttle).
 * REVERSE : output must increase to lower the measurement (cooler, brake).
 */
typedef enum {
    PID_DIRECT = 0,
    PID_REVERSE = 1
} PID_Direction;

/**
 * Operating mode.
 * MANUAL    : output follows PID_SetManualOutput(). The integrator is
 *             back-solved every sample so that switching to AUTOMATIC is
 *             bumpless.
 * AUTOMATIC : normal closed-loop operation.
 * HOLD      : all terms are computed and the output is produced, but the
 *             integrator is frozen. Useful while an upstream loop is
 *             saturated, during a sensor glitch, or for operator "freeze".
 */
typedef enum {
    PID_MODE_MANUAL = 0,
    PID_MODE_AUTOMATIC,
    PID_MODE_HOLD
} PID_Mode;

/**
 * Integrator anti-windup strategy.
 *
 * NONE             : no protection. Only sane when the actuator cannot saturate.
 * CLAMP            : the integrator state is clamped to [i_min, i_max].
 *                    Simplest and very predictable. Default.
 * CONDITIONAL      : integration stops while the output is saturated AND the
 *                    error would push it further into saturation. Costs one
 *                    branch, never overshoots the clamp, but recovers slightly
 *                    slower than back-calculation.
 * BACK_CALCULATION : I += Kt*dt*(u_sat - u_raw). Continuously bleeds the excess
 *                    off the integrator. Best disturbance recovery; requires
 *                    finite output limits and a sensible Kt.
 * TRACKING         : I is driven towards an external signal (external reset
 *                    feedback), the classic way to build bumpless cascade and
 *                    override/selector schemes.
 */
typedef enum {
    PID_AW_NONE = 0,
    PID_AW_CLAMP,
    PID_AW_CONDITIONAL,
    PID_AW_BACK_CALCULATION,
    PID_AW_TRACKING
} PID_AntiWindup;

/**
 * What the derivative acts on.
 *
 * ON_MEASUREMENT     : d/dt(-y). No derivative kick on setpoint steps. Default
 *                      and the right answer for almost every real plant.
 * ON_ERROR           : d/dt(r-y). Faster setpoint tracking, but a step in r
 *                      produces an impulse of magnitude ~Kd*dr/dt.
 * ON_WEIGHTED_ERROR  : d/dt(gamma*r - y). The 2DOF generalisation; gamma=0
 *                      reduces to ON_MEASUREMENT, gamma=1 to ON_ERROR.
 */
typedef enum {
    PID_DERIV_ON_MEASUREMENT = 0,
    PID_DERIV_ON_ERROR,
    PID_DERIV_ON_WEIGHTED_ERROR
} PID_DerivativeMode;

/**
 * Integral discretisation.
 * BACKWARD_EULER : I += Ki*dt*e.  Unconditionally stable, pairs cleanly with
 *                  back-calculation. Default.
 * TRAPEZOIDAL    : I += Ki*dt/2*(e + e_prev).  Half a sample less phase lag;
 *                  worth it when dt is not small compared to the dominant
 *                  time constant. Costs one extra state.
 */
typedef enum {
    PID_INTEGRATION_BACKWARD_EULER = 0,
    PID_INTEGRATION_TRAPEZOIDAL
} PID_IntegrationMethod;

/* ======================================================================== */
/* Runtime feature bitmask                                                   */
/* ======================================================================== */
/*
 * One mask governs runtime enable/disable. Compile-time macros decide which
 * bits are *permitted*; PID_EnableFeature() returns PID_ERR_UNSUPPORTED for a
 * bit whose module was not compiled in, so a feature can never fail silently.
 */
#define PID_FEAT_INTEGRAL          (1UL << 0)  /**< Integral term active.     */
#define PID_FEAT_DERIVATIVE        (1UL << 1)  /**< Derivative term active.   */
#define PID_FEAT_D_FILTER          (1UL << 2)  /**< Derivative low-pass.      */
#define PID_FEAT_OUTPUT_LIMIT      (1UL << 3)  /**< Clamp output.             */
#define PID_FEAT_INTEGRAL_LIMIT    (1UL << 4)  /**< Clamp integrator.         */
#define PID_FEAT_FEEDFORWARD       (1UL << 5)
#define PID_FEAT_SP_SHAPER         (1UL << 6)  /**< Setpoint ramp/accel.      */
#define PID_FEAT_OUT_SHAPER        (1UL << 7)  /**< Output slew limit.        */
#define PID_FEAT_INPUT_FILTER      (1UL << 8)
#define PID_FEAT_SAFETY            (1UL << 9)  /**< Sensor validation.        */
#define PID_FEAT_GAIN_SCHED        (1UL << 10)
#define PID_FEAT_DIAGNOSTICS       (1UL << 11)
#define PID_FEAT_TELEMETRY         (1UL << 12)

/** Bits that, when all clear, allow the update to take a shorter path. */
#define PID_FEAT_ADVANCED_MASK \
    (PID_FEAT_FEEDFORWARD | PID_FEAT_SP_SHAPER | PID_FEAT_OUT_SHAPER | \
     PID_FEAT_INPUT_FILTER | PID_FEAT_SAFETY | PID_FEAT_GAIN_SCHED | \
     PID_FEAT_DIAGNOSTICS | PID_FEAT_TELEMETRY)

/* ======================================================================== */
/* Status flags                                                              */
/* ======================================================================== */

#define PID_FLAG_SATURATED_HIGH    ((uint16_t)(1U << 0))
#define PID_FLAG_SATURATED_LOW     ((uint16_t)(1U << 1))
#define PID_FLAG_INTEGRAL_ACTIVE   ((uint16_t)(1U << 2))
#define PID_FLAG_INTEGRAL_LIMITED  ((uint16_t)(1U << 3))
#define PID_FLAG_FAULT             ((uint16_t)(1U << 4))
#define PID_FLAG_MANUAL            ((uint16_t)(1U << 5))
#define PID_FLAG_TUNING            ((uint16_t)(1U << 6))
#define PID_FLAG_DT_VIOLATION      ((uint16_t)(1U << 7))
#define PID_FLAG_SENSOR_INVALID    ((uint16_t)(1U << 8))
#define PID_FLAG_SP_RAMPING        ((uint16_t)(1U << 9))
#define PID_FLAG_OUTPUT_SLEWING    ((uint16_t)(1U << 10))

#define PID_FLAG_SATURATED (PID_FLAG_SATURATED_HIGH | PID_FLAG_SATURATED_LOW)

/* ======================================================================== */
/* Configuration structures                                                  */
/* ======================================================================== */

/** Gains and timing: the irreducible minimum every controller needs. */
typedef struct {
    PID_Float kp;                  /**< Proportional gain [u/e].              */
    PID_Float ki;                  /**< Integral gain [u/(e*s)]  = Kp/Ti.     */
    PID_Float kd;                  /**< Derivative gain [u*s/e]  = Kp*Td.     */
    PID_Float sample_time;         /**< Nominal dt [s]. Must be > 0.          */
    PID_Direction direction;
    PID_Mode mode;
    PID_IntegrationMethod integration;
} PID_CoreConfig;

/** Saturation limits and timing validation window. */
typedef struct {
    bool      use_output_limits;
    PID_Float output_min;
    PID_Float output_max;
    bool      use_integral_limits; /**< false -> integrator inherits output.  */
    PID_Float integral_min;
    PID_Float integral_max;
    PID_Float dt_min;              /**< 0 -> no lower check (variable dt).    */
    PID_Float dt_max;              /**< 0 -> no upper check.                  */
} PID_LimitConfig;

/** Derivative source, derivative filtering and optional input filtering. */
typedef struct {
    PID_DerivativeMode derivative_mode;
    PID_Float tf;                  /**< Derivative filter time constant [s].
                                        0 -> use n_filter, or raw derivative. */
    PID_Float n_filter;            /**< Tf = Kd/(N*Kp). Typical 5..20.
                                        Ignored when tf > 0.                  */
    PID_Float input_lpf_tau;       /**< Measurement pre-filter [s]. 0 = off.  */
} PID_FilterConfig;

/** Integrator behaviour beyond plain accumulation. */
typedef struct {
    PID_AntiWindup mode;
    PID_Float kt;                  /**< Back-calculation gain [1/s].
                                        0 -> derived automatically.           */
    PID_Float separation_threshold;/**< |e| above this -> stop integrating.
                                        0 = off. Classic "integral separation":
                                        avoids winding up during large step
                                        transients.                           */
    PID_Float deadband;            /**< |e| below this -> stop integrating.
                                        0 = off. Prevents limit cycling around
                                        a quantised actuator.                 */
    bool      enabled;             /**< Master enable for the I term.         */
} PID_IntegralConfig;

/** Setpoint weighting (1DOF tuning knob / full 2DOF). */
typedef struct {
    PID_Float beta;                /**< Setpoint weight in P. Default 1.      */
    PID_Float gamma;               /**< Setpoint weight in D. Default 0.      */
} PID_WeightConfig;

/** Signature of a user feedforward function. Must be fast and reentrant. */
typedef PID_Float (*PID_FeedforwardFn)(PID_Float setpoint,
                                       PID_Float measurement,
                                       void *ctx);

typedef struct {
    bool      enabled;
    PID_FeedforwardFn fn;          /**< NULL -> use the static value.         */
    void     *ctx;
    PID_Float value;               /**< Static feedforward contribution.      */
    PID_Float gain;                /**< Scale applied to fn()/value. Def. 1.  */
} PID_FeedforwardConfig;

/** Setpoint trajectory and output slew shaping. */
typedef struct {
    PID_Float sp_rate_max;         /**< Max |d(sp)/dt| [unit/s]. 0 = off.     */
    PID_Float sp_accel;            /**< [unit/s^2]. 0 = rate-only (step in v).*/
    PID_Float sp_decel;            /**< [unit/s^2]. 0 -> mirrors sp_accel.    */
    PID_Float out_slew_max;        /**< Max |du/dt| [unit/s]. 0 = off.        */
} PID_ShaperConfig;

/** Sensor validation and fail-safe behaviour. */
typedef struct {
    bool      enabled;
    PID_Float meas_min;
    PID_Float meas_max;
    PID_Float meas_rate_max;       /**< Max plausible |dy/dt|. 0 = off.       */
    PID_Float failsafe_output;     /**< Output driven while faulted.          */
    uint16_t  fault_persist_n;     /**< Consecutive bad samples before latch.
                                        0 or 1 -> latch immediately.          */
    bool      auto_recover;        /**< Clear the fault once inputs are sane
                                        again (bumplessly). Otherwise the user
                                        must call PID_ClearFault().           */
} PID_SafetyConfig;

/**
 * Umbrella configuration. A Basic-level user never sees this: PID_InitDefault()
 * fills it internally. Intermediate users call PID_ConfigDefault() then tweak
 * only the sub-structs they care about.
 */
typedef struct {
    uint16_t              abi_version;   /**< Set by PID_ConfigDefault().     */
    uint16_t              reserved;      /**< Padding; keep zero.             */
    PID_CoreConfig        core;
    PID_LimitConfig       limits;
    PID_FilterConfig      filter;
    PID_IntegralConfig    integral;
    PID_WeightConfig      weight;
    PID_FeedforwardConfig feedforward;
    PID_ShaperConfig      shaper;
    PID_SafetyConfig      safety;
} PID_Config;

/* ======================================================================== */
/* Extended input                                                            */
/* ======================================================================== */

/**
 * Rich input for PID_UpdateEx(). Any field set to NaN means "use the value
 * already stored in the handle", which keeps call sites terse: you only fill
 * what changes. PID_InputInit() sets every optional field to NaN for you.
 */
typedef struct {
    PID_Float measurement;         /**< Required.                             */
    PID_Float setpoint;            /**< NaN -> keep current setpoint.         */
    PID_Float feedforward;         /**< NaN -> use configured value/callback. */
    PID_Float dt;                  /**< NaN or <=0 -> use nominal sample time.*/
    PID_Float tracking;            /**< NaN -> no external reset this sample. */
    PID_Float schedule_var;        /**< NaN -> derive from the configured
                                        gain-scheduling source.               */
} PID_Input;

/* ======================================================================== */
/* Diagnostics                                                               */
/* ======================================================================== */

#if PIDX_ENABLE_DIAGNOSTICS
/**
 * A complete snapshot of one control cycle. Everything a plotter, a MATLAB
 * session or a fault post-mortem needs, with no string formatting anywhere.
 */
typedef struct {
    PID_Float setpoint_raw;        /**< Target before shaping.                */
    PID_Float setpoint_shaped;     /**< Target actually used this cycle.      */
    PID_Float measurement_raw;
    PID_Float measurement_filtered;
    PID_Float error;               /**< Direction-corrected (r - y).          */
    PID_Float p_term;
    PID_Float i_term;
    PID_Float d_term;
    PID_Float ff_term;
    PID_Float output_unsat;        /**< Before saturation/slew.               */
    PID_Float output;              /**< Final commanded value.                */
    PID_Float dt_used;
    PID_Float kp_active;           /**< After gain scheduling.                */
    PID_Float ki_active;
    PID_Float kd_active;
    uint32_t  update_count;
    uint32_t  saturation_count;    /**< Cycles spent saturated.               */
    uint16_t  flags;               /**< PID_FLAG_*.                           */
    PID_StatusCode last_error;
} PID_Status;
#endif /* PIDX_ENABLE_DIAGNOSTICS */

#if PIDX_ENABLE_TELEMETRY
/** One compact telemetry sample (32 bytes with float32). */
typedef struct {
    PID_Float setpoint;
    PID_Float measurement;
    PID_Float p_term;
    PID_Float i_term;
    PID_Float d_term;
    PID_Float ff_term;
    PID_Float output;
    uint16_t  flags;
    uint16_t  seq;                 /**< Wraps; lets a consumer spot gaps.     */
} PID_TelemetryRecord;

/**
 * Single-producer / single-consumer lock-free ring buffer.
 *
 * Producer: the context calling PID_Update() (typically a timer ISR).
 * Consumer: exactly one other context calling PID_Telemetry_Read().
 * Any other usage is a data race. Capacity must be a power of two.
 */
typedef struct {
    PID_TelemetryRecord *storage;
    uint16_t capacity_mask;        /**< capacity - 1, capacity = 2^n.         */
    volatile uint16_t head;        /**< Written by producer only.             */
    volatile uint16_t tail;        /**< Written by consumer only.             */
    uint16_t seq;
    uint16_t dropped;              /**< Overwritten-oldest counter.           */
} PID_Telemetry;
#endif /* PIDX_ENABLE_TELEMETRY */

#if PIDX_ENABLE_GAIN_SCHED
/* Forward declaration; the full definition lives in pid_gainsched.h so that
 * the core never needs to know the table layout. */
struct PID_GainSchedule;
#endif

/* ======================================================================== */
/* The handle                                                               */
/* ======================================================================== */

/** Magic stamped by PID_Init to detect use of uninitialised memory. */
#define PID_INIT_MAGIC  ((uint8_t)0xA7U)

/**
 * Controller instance. Allocate it statically or on the stack - the library
 * never allocates. Roughly 132 B (minimal profile) to 344 B (full profile)
 * with 32-bit floats.
 *
 * Field order is chosen so that everything touched by the fast path sits in
 * the first ~64 bytes, which keeps the compiler using short immediate offsets
 * and keeps a Cortex-M7 data cache line useful.
 */
typedef struct PID_Handle {
    /* ---- HOT: touched every update ---- */
    PID_Float integrator;    /**< I term IN OUTPUT UNITS (Ki*integral(e)).
                                  Storing the scaled term - not the raw
                                  integral - is what makes runtime gain
                                  changes and gain scheduling bumpless.       */
    PID_Float d_state;       /**< Filtered derivative term, output units.     */
    PID_Float d_prev_in;     /**< Previous derivative source sample.          */
    PID_Float e_prev;        /**< Previous error (trapezoidal integration).   */
    PID_Float setpoint;      /**< Effective setpoint after shaping.           */
    PID_Float output;        /**< Last commanded output.                      */

    PID_Float kp;            /**< Active gains (post gain-scheduling).        */
    PID_Float ki;
    PID_Float kd;
    PID_Float beta;
    PID_Float gamma;

    PID_Float c_i;           /**< Precomputed ki*dt (or ki*dt/2).             */
    PID_Float c_da;          /**< Precomputed tf/(tf+dt), the filter pole.    */
    PID_Float c_db;          /**< Precomputed kd/(tf+dt), the filter gain.    */
    PID_Float c_aw;          /**< Precomputed kt*dt.                          */

    PID_Float out_min;
    PID_Float out_max;
    PID_Float i_min;
    PID_Float i_max;

    uint32_t  features;      /**< PID_FEAT_* runtime mask.                    */
    uint16_t  flags;         /**< PID_FLAG_*.                                 */
    int8_t    dir_sign;      /**< +1 direct, -1 reverse.                      */
    uint8_t   mode;          /**< PID_Mode.                                   */
    uint8_t   aw_mode;       /**< PID_AntiWindup.                             */
    uint8_t   d_mode;        /**< PID_DerivativeMode.                         */
    uint8_t   integ_method;  /**< PID_IntegrationMethod.                      */
    uint8_t   init_magic;    /**< PID_INIT_MAGIC once initialised.            */

    /* ---- WARM: configuration, read rarely ---- */
    PID_Float dt_nominal;
    PID_Float dt_min;
    PID_Float dt_max;
    PID_Float dt_last;       /**< Last dt for which coefficients were built.  */
    PID_Float tf;
    PID_Float n_filter;
    PID_Float kt;
    PID_Float i_separation;
    PID_Float i_deadband;
    PID_Float setpoint_target;   /**< Commanded target, pre-shaper.           */
    PID_Float manual_output;
    PID_Float tracking_input;

#if PIDX_ENABLE_SHAPER
    PID_Float sp_rate_max;
    PID_Float sp_accel;
    PID_Float sp_decel;
    PID_Float sp_velocity;   /**< Current setpoint slew velocity.             */
    PID_Float out_slew_max;
#endif

#if PIDX_ENABLE_INPUT_FILTER
    PID_LPF1  in_lpf;        /**< Measurement pre-filter (see pid_filter.h).  */
#endif

#if PIDX_ENABLE_FEEDFORWARD
    PID_FeedforwardFn ff_fn;
    void     *ff_ctx;
    PID_Float ff_value;
    PID_Float ff_gain;
#endif

#if PIDX_ENABLE_SAFETY
    PID_Float meas_min;
    PID_Float meas_max;
    PID_Float meas_rate_max;
    PID_Float failsafe_output;
    PID_Float meas_prev;
    bool      meas_prev_valid;
    bool      auto_recover;
    uint16_t  fault_count;
    uint16_t  fault_persist_n;
#endif

#if PIDX_ENABLE_GAIN_SCHED
    struct PID_GainSchedule *sched;  /**< Non-const: evaluation caches state.*/
    PID_Float sched_var_ext;
    uint8_t   sched_index_cache;
#endif

#if PIDX_ENABLE_DIAGNOSTICS
    PID_Status status;
#endif

#if PIDX_ENABLE_TELEMETRY
    PID_Telemetry *telemetry;
#endif

    PID_StatusCode last_error;   /**< Sticky: cleared by PID_ClearError().    */
} PID_Handle;

#ifdef __cplusplus
}
#endif

#endif /* PIDX_PID_TYPES_H */

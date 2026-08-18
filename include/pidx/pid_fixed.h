/**
 * @file    pid_fixed.h
 * @brief   Standalone fixed-point PID controller (Q15 I/O, Q30 internals).
 *
 * This is NOT a layer over the floating-point core. It is an independent
 * controller with a mirrored API, so a single binary can run a fixed-point
 * current loop and a floating-point temperature loop side by side. It has no
 * dependency on pid.h, pid_types.h or pid_math.h, contains no floating-point
 * arithmetic and no libm call, and performs no dynamic allocation.
 *
 * ---------------------------------------------------------------------------
 * NUMBER FORMATS
 * ---------------------------------------------------------------------------
 * All process signals (setpoint, measurement, output, limits) are Q15: a
 * signed 16-bit integer where 32768 would represent 1.0, so the representable
 * range is [-1.0, +0.99997] with a resolution of 1/32768 = 3.05e-5.
 *
 * The caller is expected to normalise: an ADC reading of 0..4095 becomes
 * 0..32767 by a shift, a PWM duty of 0..1 maps back the same way. Working in
 * a normalised domain is what keeps the range analysis below valid.
 *
 * Gains are Q16.16 (int32_t, 65536 == 1.0):
 *   kp_q16    proportional gain,            dimensionless
 *   ki_q16    integral gain,                units of 1/second
 *   kd_q16    derivative gain,              units of second
 *
 * ---------------------------------------------------------------------------
 * WHY THE INTERNAL STATE IS Q30 AND NOT Q15
 * ---------------------------------------------------------------------------
 * The classic failure of a naive fixed-point PID is "integral resolution
 * death". The per-sample integral increment is Ki*dt*e. With Ki = 0.5,
 * dt = 1 ms and an error of one LSB, that increment is 1.5e-8 in real units,
 * which is 0.0005 of a Q15 LSB. Rounded into a Q15 accumulator it is exactly
 * zero, so the integrator never moves and the loop keeps a permanent
 * steady-state error that no amount of tuning removes.
 *
 * The fix is to hold the integrator (and the input filter state, which has the
 * same problem) at Q30: 32768 times finer than the output LSB. Increments far
 * below one output LSB then accumulate correctly and eventually move the
 * output, exactly as they would in floating point.
 *
 * ---------------------------------------------------------------------------
 * OVERFLOW ANALYSIS  (why every product below fits)
 * ---------------------------------------------------------------------------
 * Error is a difference of two Q15 values, so |e| <= 2.0 (2^16 in Q15 units).
 * Gains are validated at init so that the folded coefficients fit their
 * declared format. Every multiply is performed in int64_t and immediately
 * shifted back, so the worst case is:
 *
 *   P : kp_q16 (<=2^31) * e_q15 (<=2^16)          -> 2^47
 *   I : ci_q30 (<=2^31) * e_q15 (<=2^16)          -> 2^47
 *   D : cb_q16 (<=2^31) * dx_q30 (<=2^31)         -> 2^62
 *   D : ca_q30 (<=2^30) * d_q30 (<=2^31)          -> 2^61
 *
 * all within int64_t (2^63). On Cortex-M3 and above a 32x32->64 multiply is
 * the single-cycle SMULL instruction, so this costs no more than 32-bit math.
 * On Cortex-M0/M0+ it becomes a compiler helper call; that is the documented
 * price of correctness on that core.
 *
 * ---------------------------------------------------------------------------
 * SUPPORTED / NOT SUPPORTED  (see docs/16_fixed_point.md)
 * ---------------------------------------------------------------------------
 * Supported : P, I, D with derivative-on-measurement, first-order derivative
 *             filter, shift-based EMA input filter, output clamping, integral
 *             clamping, anti-windup (clamp / conditional / back-calculation
 *             with a power-of-two coefficient), integral deadband, integral
 *             separation, DIRECT/REVERSE action, manual/automatic with
 *             bumpless transfer.
 * NOT        : auto-tuning, interpolated gain scheduling, setpoint weighting,
 *             setpoint shaping, feedforward curves, cascade helpers.
 *             Those need division or wide dynamic range and belong to the
 *             floating-point core. They are absent, not stubbed.
 *
 * ---------------------------------------------------------------------------
 * MISRA-C:2012 DEVIATION
 * ---------------------------------------------------------------------------
 * Rule 10.1 / implementation-defined behaviour: this module right-shifts
 * negative signed integers and relies on it being an arithmetic shift, which
 * C99 6.5.7 leaves implementation-defined. Every compiler targeted by this
 * library (GCC, Clang, IAR, Keil ARMCC) defines it as arithmetic. The
 * assumption is verified at runtime by PIDq_SelfTest() and in tests/.
 */
#ifndef PIDX_PID_FIXED_H
#define PIDX_PID_FIXED_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "pidx/pid_conf.h"
#include "pidx/pid_status.h"

#if PIDX_ENABLE_FIXED_POINT

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* Format helpers                                                            */
/* ======================================================================== */

/** 1.0 in Q15. Note the representable maximum is PIDQ_ONE - 1. */
#define PIDQ_ONE            32768
/** Largest / smallest representable Q15 signal. */
#define PIDQ_MAX            ((int16_t)32767)
#define PIDQ_MIN            ((int16_t)-32768)

/**
 * Convert a literal to Q15 / Q16.16 at compile time.
 *
 * These macros are the only place a float appears in the fixed-point module,
 * and they are intended for constant expressions, so the conversion is done
 * by the compiler and no floating-point code reaches the target. Build the
 * constants on the host if your toolchain has no soft-float at all.
 */
#define PIDQ_F_TO_Q15(x)    ((int16_t)(((x) * 32768.0) < 0.0 ?                \
                              ((x) * 32768.0 - 0.5) : ((x) * 32768.0 + 0.5)))
#define PIDQ_F_TO_Q16(x)    ((int32_t)(((x) * 65536.0) < 0.0 ?                \
                              ((x) * 65536.0 - 0.5) : ((x) * 65536.0 + 0.5)))

/* ======================================================================== */
/* Enumerations                                                              */
/* ======================================================================== */

/** Controller action. REVERSE is used when a rising measurement must raise
 *  the output (e.g. a cooling actuator). */
typedef enum {
    PIDQ_DIRECT  = 0,
    PIDQ_REVERSE = 1
} PIDq_Direction;

/** Operating mode. In manual the integrator tracks the manual output so the
 *  switch back to automatic is bumpless. */
typedef enum {
    PIDQ_MODE_MANUAL    = 0,
    PIDQ_MODE_AUTOMATIC = 1
} PIDq_Mode;

/** Anti-windup strategy. */
typedef enum {
    /** No protection. Only sane when the output is never limited. */
    PIDQ_AW_NONE        = 0,
    /** Clamp the integrator to [i_min, i_max]. */
    PIDQ_AW_CLAMP       = 1,
    /** Skip integration whenever the output is saturated and the error would
     *  push it further into saturation. */
    PIDQ_AW_CONDITIONAL = 2,
    /** Feed (u_saturated - u_unsaturated) back into the integrator with a
     *  coefficient of 2^-bc_shift. Restricted to powers of two so the
     *  correction is a shift and no division enters the hot path. */
    PIDQ_AW_BACK_CALC   = 3
} PIDq_AntiWindup;

/* ======================================================================== */
/* Configuration                                                             */
/* ======================================================================== */

/**
 * Fixed-point controller configuration.
 *
 * Zero-initialising this struct and calling PIDq_ConfigDefault() gives a safe
 * starting point: unity proportional gain, no integral or derivative action,
 * full-scale output limits, clamp anti-windup, DIRECT, AUTOMATIC.
 */
typedef struct {
    int32_t  kp_q16;            /**< Q16.16 proportional gain.               */
    int32_t  ki_q16;            /**< Q16.16 integral gain [1/s].             */
    int32_t  kd_q16;            /**< Q16.16 derivative gain [s].             */

    uint32_t dt_us;             /**< Sample period in microseconds, > 0.     */
    uint32_t tf_us;             /**< Derivative filter time constant [us].
                                     0 disables the filter (raw difference). */

    int16_t  out_min_q15;       /**< Output lower limit.                     */
    int16_t  out_max_q15;       /**< Output upper limit, > out_min.          */
    int16_t  i_min_q15;         /**< Integrator lower limit (AW_CLAMP).      */
    int16_t  i_max_q15;         /**< Integrator upper limit (AW_CLAMP).      */

    uint16_t deadband_q15;      /**< |e| below this does not integrate.      */
    uint16_t separation_q15;    /**< |e| above this does not integrate.
                                     0 disables integral separation.         */

    uint8_t  aw_mode;           /**< PIDq_AntiWindup.                        */
    uint8_t  bc_shift;          /**< Back-calculation coefficient 2^-shift,
                                     1..15. Ignored unless AW_BACK_CALC.     */
    uint8_t  lpf_shift;         /**< Input EMA filter: y += (x-y) >> shift,
                                     1..15. 0 disables the filter.           */
    uint8_t  direction;         /**< PIDq_Direction.                         */
    uint8_t  mode;              /**< PIDq_Mode.                              */
} PIDq_Config;

/* ======================================================================== */
/* Handle                                                                    */
/* ======================================================================== */

/**
 * Controller state. Entirely caller-owned; allocate it statically or on the
 * stack. Treat every field as private - use the accessors.
 */
typedef struct {
    /* Precomputed coefficients. Recomputed only by Init / SetGains, so the
     * update path contains no division at all. */
    int32_t  kp_q16;            /**< Proportional gain.                      */
    int32_t  ci_q30;            /**< Ki*dt, folded.                          */
    int32_t  cb_q16;            /**< Kd/(Tf+dt), folded.                     */
    int32_t  ca_q30;            /**< Tf/(Tf+dt), derivative filter pole.     */

    /* Kept so PIDq_SetGains() can refold the coefficients. They cannot be
     * recovered from ci/cb/ca: ci = Ki*dt collapses to 0 when Ki is 0, and
     * cb = Kd/(Tf+dt) does the same for Kd, so a gain change from zero would
     * have no scale to rescale from. Storing them costs 8 bytes and keeps
     * SetGains exact for every combination. */
    uint32_t dt_us;             /**< Sample period, microseconds.            */
    uint32_t tf_us;             /**< Derivative filter constant, us.         */

    int32_t  out_min_q30;       /**< Output limits, promoted to Q30.         */
    int32_t  out_max_q30;
    int32_t  i_min_q30;         /**< Integrator limits, promoted to Q30.     */
    int32_t  i_max_q30;

    /* Running state. */
    int32_t  integ_q30;         /**< Integral term in OUTPUT units.          */
    int32_t  d_q30;             /**< Filtered derivative term.               */
    int32_t  meas_filt_q30;     /**< EMA state, held wide to avoid decay to 0*/
    int32_t  meas_prev_q30;     /**< Previous filtered measurement.          */
    int32_t  out_q30;           /**< Last output before rounding.            */

    int16_t  setpoint_q15;
    int16_t  manual_q15;        /**< Output held while in manual.            */
    int16_t  out_q15;           /**< Last returned output.                   */

    uint16_t deadband_q15;
    uint16_t separation_q15;

    uint8_t  aw_mode;
    uint8_t  bc_shift;
    uint8_t  lpf_shift;
    uint8_t  direction;
    uint8_t  mode;
    uint8_t  flags;             /**< Bit 0 saturated, bit 1 primed.          */

    uint16_t magic;             /**< Init marker; PIDq_Deinit clears it.     */
} PIDq_Handle;

/** Set when the last update hit an output limit. */
#define PIDQ_FLAG_SATURATED     0x01U
/** Set once the first measurement has been seen (derivative kick guard). */
#define PIDQ_FLAG_PRIMED        0x02U

/* ======================================================================== */
/* API                                                                       */
/* ======================================================================== */

/** Fill cfg with the documented safe defaults. */
PID_StatusCode PIDq_ConfigDefault(PIDq_Config *cfg);

/**
 * Validate cfg, fold dt and Tf into the internal coefficients, and clear all
 * state. Returns PID_ERR_INVALID_* describing the first offending field.
 */
PID_StatusCode PIDq_Init(PIDq_Handle *h, const PIDq_Config *cfg);

/** Invalidate the handle. A later PIDq_Update returns 0 and changes nothing. */
PID_StatusCode PIDq_Deinit(PIDq_Handle *h);

/** Clear integrator, derivative and filter state, keeping the tuning. */
PID_StatusCode PIDq_Reset(PIDq_Handle *h);

/**
 * Run one control step at the configured sample rate.
 *
 * @param  h                controller, must be initialised
 * @param  measurement_q15  process variable
 * @return control output in Q15, already clamped to the configured limits.
 *         Returns 0 if h is NULL or not initialised.
 */
int16_t PIDq_Update(PIDq_Handle *h, int16_t measurement_q15);

/** Change gains at runtime. Bumpless: the integrator holds output units, so
 *  a gain change does not step the output. Recomputes the coefficients. */
PID_StatusCode PIDq_SetGains(PIDq_Handle *h, int32_t kp_q16,
                             int32_t ki_q16, int32_t kd_q16);

PID_StatusCode PIDq_SetSetpoint(PIDq_Handle *h, int16_t sp_q15);
int16_t        PIDq_GetSetpoint(const PIDq_Handle *h);

/** Switch mode. Entering AUTOMATIC back-solves the integrator from the manual
 *  output so the transfer is bumpless. */
PID_StatusCode PIDq_SetMode(PIDq_Handle *h, PIDq_Mode mode);
PIDq_Mode      PIDq_GetMode(const PIDq_Handle *h);

/** Set the output used while in manual mode. */
PID_StatusCode PIDq_SetManualOutput(PIDq_Handle *h, int16_t u_q15);
int16_t        PIDq_GetManualOutput(const PIDq_Handle *h);

/** Change output limits at runtime; the integrator is re-clamped. */
PID_StatusCode PIDq_SetOutputLimits(PIDq_Handle *h, int16_t min_q15,
                                    int16_t max_q15);

int16_t PIDq_GetOutput(const PIDq_Handle *h);
/** Integral term in Q15 output units (the internal state is Q30). */
int16_t PIDq_GetIntegral(const PIDq_Handle *h);
bool    PIDq_IsSaturated(const PIDq_Handle *h);

/**
 * Verify the two platform assumptions this module makes: arithmetic right
 * shift of negative values, and a 64-bit multiply that does not truncate.
 * Returns true when the platform is suitable. Call once at startup.
 */
bool PIDq_SelfTest(void);

#ifdef __cplusplus
}
#endif

#endif /* PIDX_ENABLE_FIXED_POINT */
#endif /* PIDX_PID_FIXED_H */

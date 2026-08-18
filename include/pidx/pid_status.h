/**
 * @file    pid_status.h
 * @brief   Status and error codes, in a leaf header with no dependencies.
 *
 * Kept separate from pid_types.h so that the standalone filter and shaper
 * modules - which the controller handle itself embeds - can report status
 * without a circular include.
 */
#ifndef PIDX_PID_STATUS_H
#define PIDX_PID_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* Status / error codes                                                      */
/* ======================================================================== */

/**
 * Result of an API call, and the type of the sticky per-handle error.
 * Values are stable; new codes are appended at the end.
 */
typedef enum {
    PID_OK = 0,                    /**< Success.                              */
    PID_ERR_NULL,                  /**< A required pointer was NULL.          */
    PID_ERR_NOT_INIT,              /**< Handle never passed through PID_Init. */
    PID_ERR_INVALID_CONFIG,        /**< Config struct failed validation.      */
    PID_ERR_INVALID_GAIN,          /**< Gain is NaN/Inf or illegally signed.  */
    PID_ERR_INVALID_LIMIT,         /**< min >= max, or a limit is NaN.        */
    PID_ERR_INVALID_DT,            /**< dt <= 0 or outside [dt_min, dt_max].  */
    PID_ERR_INVALID_MODE,          /**< Unknown PID_Mode value.               */
    PID_ERR_INVALID_PARAM,         /**< Generic out-of-range argument.        */
    PID_ERR_NAN_INPUT,             /**< Measurement or setpoint was NaN.      */
    PID_ERR_INF_INPUT,             /**< Measurement or setpoint was Inf.      */
    PID_ERR_SENSOR_RANGE,          /**< Measurement outside configured range. */
    PID_ERR_SENSOR_RATE,           /**< Measurement jumped faster than limit. */
    PID_ERR_UNSUPPORTED,           /**< Feature not compiled into this build. */
    PID_ERR_BUSY,                  /**< Operation illegal in current state.   */
    PID_ERR_TUNE_TIMEOUT,          /**< Auto-tune exceeded its time budget.   */
    PID_ERR_TUNE_UNSTABLE,         /**< Oscillation diverged / hit a limit.   */
    PID_ERR_TUNE_NO_OSCILLATION,   /**< Relay never produced usable swings.   */
    PID_ERR_TUNE_MODEL_MISMATCH,   /**< Rule needs a model the test can't give*/
    PID_ERR_TUNE_ABORTED,          /**< User or watchdog aborted the tune.    */
    PID_ERR_TUNE_VALIDATION        /**< Computed gains failed sanity checks.  */
} PID_StatusCode;

#ifdef __cplusplus
}
#endif

#endif /* PIDX_PID_STATUS_H */

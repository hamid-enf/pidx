"""Enumerations, flags, status codes and configuration records.

Mirrors include/pidx/pid_types.h and include/pidx/pid_status.h. The field
paths are deliberately identical to the C ones (cfg.core.kp, cfg.limits.
output_min, cfg.integral.mode, ...) so that code and documentation translate
between the two without a lookup table.
"""

from dataclasses import dataclass, field
from enum import IntEnum

PIDX_VERSION_MAJOR = 1
PIDX_VERSION_MINOR = 0
PIDX_VERSION_PATCH = 0
PIDX_VERSION_NUM = (PIDX_VERSION_MAJOR * 10000
                    + PIDX_VERSION_MINOR * 100
                    + PIDX_VERSION_PATCH)
PIDX_VERSION_STRING = "1.0.0"

PIDX_CONFIG_ABI_VERSION = 1

#: Default sample time when the caller does not set one, seconds.
PIDX_DEFAULT_SAMPLE_TIME = 0.01
#: Default derivative filter ratio N, giving Tf = Kd / (N * Kp).
PIDX_DEFAULT_N_FILTER = 10.0
#: Stand-in for "no limit". Large enough to never bite, small enough that
#: adding it to a real signal does not lose the signal to rounding.
PID_HUGE_F = 1.0e30

PIDX_GAINSCHED_MAX_POINTS = 16
PIDX_CASCADE_MAX_LOOPS = 4


class Status(IntEnum):
    """Result of an API call, and the type of the sticky per-handle error."""

    OK = 0
    ERR_NULL = 1
    ERR_NOT_INIT = 2
    ERR_INVALID_CONFIG = 3
    ERR_INVALID_GAIN = 4
    ERR_INVALID_LIMIT = 5
    ERR_INVALID_DT = 6
    ERR_INVALID_MODE = 7
    ERR_INVALID_PARAM = 8
    ERR_NAN_INPUT = 9
    ERR_INF_INPUT = 10
    ERR_SENSOR_RANGE = 11
    ERR_SENSOR_RATE = 12
    ERR_UNSUPPORTED = 13
    ERR_BUSY = 14
    ERR_TUNE_TIMEOUT = 15
    ERR_TUNE_UNSTABLE = 16
    ERR_TUNE_NO_OSCILLATION = 17
    ERR_TUNE_MODEL_MISMATCH = 18
    ERR_TUNE_ABORTED = 19
    ERR_TUNE_VALIDATION = 20


_STATUS_TEXT = {
    Status.OK: "OK",
    Status.ERR_NULL: "null pointer",
    Status.ERR_NOT_INIT: "not initialised",
    Status.ERR_INVALID_CONFIG: "invalid config",
    Status.ERR_INVALID_GAIN: "invalid gain",
    Status.ERR_INVALID_LIMIT: "invalid limit",
    Status.ERR_INVALID_DT: "invalid dt",
    Status.ERR_INVALID_MODE: "invalid mode",
    Status.ERR_INVALID_PARAM: "invalid parameter",
    Status.ERR_NAN_INPUT: "NaN input",
    Status.ERR_INF_INPUT: "Inf input",
    Status.ERR_SENSOR_RANGE: "sensor out of range",
    Status.ERR_SENSOR_RATE: "sensor rate exceeded",
    Status.ERR_UNSUPPORTED: "unsupported",
    Status.ERR_BUSY: "busy",
    Status.ERR_TUNE_TIMEOUT: "tune timeout",
    Status.ERR_TUNE_UNSTABLE: "tune unstable",
    Status.ERR_TUNE_NO_OSCILLATION: "tune: no oscillation",
    Status.ERR_TUNE_MODEL_MISMATCH: "tune: model mismatch",
    Status.ERR_TUNE_ABORTED: "tune aborted",
    Status.ERR_TUNE_VALIDATION: "tune: validation failed",
}


def status_to_string(code):
    """Human-readable name of a status code. Never raises."""
    try:
        return _STATUS_TEXT[Status(code)]
    except (ValueError, KeyError):
        return "?"


class Direction(IntEnum):
    DIRECT = 0
    REVERSE = 1


class Mode(IntEnum):
    MANUAL = 0
    AUTOMATIC = 1
    HOLD = 2


class AntiWindup(IntEnum):
    NONE = 0
    CLAMP = 1
    CONDITIONAL = 2
    BACK_CALCULATION = 3
    TRACKING = 4


class DerivativeMode(IntEnum):
    ON_MEASUREMENT = 0
    ON_ERROR = 1
    ON_WEIGHTED_ERROR = 2


class IntegrationMethod(IntEnum):
    BACKWARD_EULER = 0
    TRAPEZOIDAL = 1


class SchedSource(IntEnum):
    SETPOINT = 0
    MEASUREMENT = 1
    ERROR = 2
    ABS_ERROR = 3
    OUTPUT = 4
    EXTERNAL = 5


class SchedInterp(IntEnum):
    LINEAR = 0
    SMOOTH = 1
    HOLD = 2


# ---------------------------------------------------------------------------
# Runtime feature mask
# ---------------------------------------------------------------------------

FEAT_INTEGRAL = 1 << 0
FEAT_DERIVATIVE = 1 << 1
FEAT_D_FILTER = 1 << 2
FEAT_OUTPUT_LIMIT = 1 << 3
FEAT_INTEGRAL_LIMIT = 1 << 4
FEAT_FEEDFORWARD = 1 << 5
FEAT_SP_SHAPER = 1 << 6
FEAT_OUT_SHAPER = 1 << 7
FEAT_INPUT_FILTER = 1 << 8
FEAT_SAFETY = 1 << 9
FEAT_GAIN_SCHED = 1 << 10
FEAT_DIAGNOSTICS = 1 << 11
FEAT_TELEMETRY = 1 << 12

#: Everything the fast path does not implement.
FEAT_ADVANCED_MASK = (FEAT_FEEDFORWARD | FEAT_SP_SHAPER | FEAT_OUT_SHAPER
                      | FEAT_INPUT_FILTER | FEAT_SAFETY | FEAT_GAIN_SCHED
                      | FEAT_DIAGNOSTICS | FEAT_TELEMETRY)

# ---------------------------------------------------------------------------
# Per-cycle status flags
# ---------------------------------------------------------------------------

FLAG_SATURATED_HIGH = 1 << 0
FLAG_SATURATED_LOW = 1 << 1
FLAG_INTEGRAL_ACTIVE = 1 << 2
FLAG_INTEGRAL_LIMITED = 1 << 3
FLAG_FAULT = 1 << 4
FLAG_MANUAL = 1 << 5
FLAG_TUNING = 1 << 6
FLAG_DT_VIOLATION = 1 << 7
FLAG_SENSOR_INVALID = 1 << 8
FLAG_SP_RAMPING = 1 << 9
FLAG_OUTPUT_SLEWING = 1 << 10

FLAG_SATURATED = FLAG_SATURATED_HIGH | FLAG_SATURATED_LOW


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

@dataclass
class CoreConfig:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    sample_time: float = PIDX_DEFAULT_SAMPLE_TIME
    direction: int = Direction.DIRECT
    mode: int = Mode.AUTOMATIC
    integration: int = IntegrationMethod.BACKWARD_EULER


@dataclass
class LimitsConfig:
    use_output_limits: bool = False
    output_min: float = -PID_HUGE_F
    output_max: float = PID_HUGE_F
    use_integral_limits: bool = False
    integral_min: float = -PID_HUGE_F
    integral_max: float = PID_HUGE_F
    #: 0 disables the check. Outside the band dt is clamped, not rejected.
    dt_min: float = 0.0
    dt_max: float = 0.0


@dataclass
class FilterConfig:
    derivative_mode: int = DerivativeMode.ON_MEASUREMENT
    #: Explicit derivative filter time constant. Wins over n_filter.
    tf: float = 0.0
    n_filter: float = PIDX_DEFAULT_N_FILTER
    input_lpf_tau: float = 0.0


@dataclass
class IntegralConfig:
    #: Anti-windup strategy. Named `mode` to match the C struct field.
    mode: int = AntiWindup.CLAMP
    kt: float = 0.0
    separation_threshold: float = 0.0
    deadband: float = 0.0
    enabled: bool = True


@dataclass
class WeightConfig:
    """Setpoint weighting. Both in [0, 2]."""

    beta: float = 1.0
    gamma: float = 0.0


@dataclass
class FeedforwardConfig:
    enabled: bool = False
    fn: object = None
    ctx: object = None
    value: float = 0.0
    gain: float = 1.0


@dataclass
class ShaperConfig:
    sp_rate_max: float = 0.0
    sp_accel: float = 0.0
    sp_decel: float = 0.0
    out_slew_max: float = 0.0


@dataclass
class SafetyConfig:
    enabled: bool = False
    meas_min: float = 0.0
    meas_max: float = 0.0
    meas_rate_max: float = 0.0
    failsafe_output: float = 0.0
    fault_persist_n: int = 3
    auto_recover: bool = False


@dataclass
class Config:
    """Full controller configuration; the analogue of PID_Config."""

    core: CoreConfig = field(default_factory=CoreConfig)
    limits: LimitsConfig = field(default_factory=LimitsConfig)
    filter: FilterConfig = field(default_factory=FilterConfig)
    integral: IntegralConfig = field(default_factory=IntegralConfig)
    weight: WeightConfig = field(default_factory=WeightConfig)
    feedforward: FeedforwardConfig = field(default_factory=FeedforwardConfig)
    shaper: ShaperConfig = field(default_factory=ShaperConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    abi_version: int = PIDX_CONFIG_ABI_VERSION


@dataclass
class Input:
    """Extended input bundle. NaN means "keep the current handle state"."""

    measurement: float = float("nan")
    setpoint: float = float("nan")
    dt: float = float("nan")
    feedforward: float = float("nan")
    tracking: float = float("nan")
    schedule_var: float = float("nan")


@dataclass
class StatusSnapshot:
    """Per-cycle diagnostic snapshot; the analogue of PID_Status."""

    setpoint_raw: float = 0.0
    setpoint_shaped: float = 0.0
    measurement_raw: float = 0.0
    measurement_filtered: float = 0.0
    error: float = 0.0
    p_term: float = 0.0
    i_term: float = 0.0
    d_term: float = 0.0
    ff_term: float = 0.0
    output_unsat: float = 0.0
    output: float = 0.0
    dt_used: float = 0.0
    kp_active: float = 0.0
    ki_active: float = 0.0
    kd_active: float = 0.0
    update_count: int = 0
    saturation_count: int = 0
    flags: int = 0
    last_error: int = Status.OK

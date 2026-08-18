"""PIDX - a production-grade, portable PID control framework.

This is the Python port of the C99 library in src/ and include/pidx/. It is a
behavioural mirror, not a wrapper: there is no compiled extension and no
dependency beyond the standard library. Every number it produces is verified
against the C implementation by ports/compare/compare.py.

Five-line API
-------------
    import pidx

    pid = pidx.PID(pidx.quick_config(kp=2.0, ki=0.5, kd=0.1, dt=0.01))
    pid.set_setpoint(100.0)
    u = pid.update(measurement)

Everything else - anti-windup strategies, 2DOF setpoint weighting,
feedforward, gain scheduling, the trajectory shaper, sensor safety, cascade
control and the tuning rules - is opt-in on top of exactly that.
"""

from .types import (
    AntiWindup, Config, CoreConfig, Direction, DerivativeMode,
    FeedforwardConfig, FilterConfig, Input, IntegralConfig, IntegrationMethod,
    LimitsConfig, Mode, SafetyConfig, SchedInterp, SchedSource, ShaperConfig,
    Status, StatusSnapshot, WeightConfig,
    PID_HUGE_F, PIDX_VERSION_STRING, PIDX_VERSION_NUM,
    status_to_string,
    FEAT_DERIVATIVE, FEAT_DIAGNOSTICS, FEAT_D_FILTER, FEAT_FEEDFORWARD,
    FEAT_GAIN_SCHED, FEAT_INPUT_FILTER, FEAT_INTEGRAL, FEAT_INTEGRAL_LIMIT,
    FEAT_OUTPUT_LIMIT, FEAT_OUT_SHAPER, FEAT_SAFETY, FEAT_SP_SHAPER,
    FEAT_TELEMETRY,
    FLAG_DT_VIOLATION, FLAG_FAULT, FLAG_INTEGRAL_ACTIVE, FLAG_INTEGRAL_LIMITED,
    FLAG_MANUAL, FLAG_OUTPUT_SLEWING, FLAG_SATURATED, FLAG_SATURATED_HIGH,
    FLAG_SATURATED_LOW, FLAG_SENSOR_INVALID, FLAG_SP_RAMPING, FLAG_TUNING,
)
from .controller import PID, config_default, get_version
from .filters import LPF1, Median3, MovingAvg, RateLimiter, deadband
from .shaper import Shaper, profile_step
from .gainsched import GainPoint, GainSchedule, attach, set_var
from .autotune_rules import (
    Gains, IdentMethod, ModelKind, PlantModel, TuneRule, TuneStructure,
    rule_apply, rule_name, rule_required_model,
)
from .cascade import Cascade, CascadeAntiWindup

__version__ = PIDX_VERSION_STRING

__all__ = [
    "PID", "Config", "CoreConfig", "LimitsConfig", "FilterConfig",
    "IntegralConfig", "WeightConfig", "FeedforwardConfig", "ShaperConfig",
    "SafetyConfig", "Input", "StatusSnapshot",
    "AntiWindup", "Direction", "DerivativeMode", "IntegrationMethod", "Mode",
    "SchedInterp", "SchedSource", "Status",
    "LPF1", "MovingAvg", "Median3", "RateLimiter", "deadband",
    "Shaper", "profile_step",
    "GainPoint", "GainSchedule", "attach", "set_var",
    "Gains", "PlantModel", "TuneRule", "TuneStructure", "ModelKind",
    "IdentMethod", "rule_apply", "rule_name", "rule_required_model",
    "Cascade", "CascadeAntiWindup",
    "config_default", "quick_config", "get_version", "status_to_string",
    "PID_HUGE_F", "PIDX_VERSION_STRING", "PIDX_VERSION_NUM",
]


def quick_config(kp=0.0, ki=0.0, kd=0.0, dt=0.01,
                 out_min=None, out_max=None):
    """Build a Config for the common case in one call.

    Supplying both output limits also switches anti-windup protection on in
    the only sense that matters: the integrator inherits those limits, so it
    cannot charge past what the actuator can deliver.
    """
    cfg = Config()
    cfg.core.kp = kp
    cfg.core.ki = ki
    cfg.core.kd = kd
    cfg.core.sample_time = dt
    if out_min is not None and out_max is not None:
        cfg.limits.use_output_limits = True
        cfg.limits.output_min = out_min
        cfg.limits.output_max = out_max
    return cfg

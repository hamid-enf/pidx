"""Interpolated gain scheduling. Mirrors pid_gainsched.h / pid_gainsched.c.

A single set of gains is only optimal near one operating point. A valve is more
effective half open than nearly shut; a motor's effective inertia changes with
arm extension; a heater's loss coefficient grows with temperature.

Two properties together guarantee a bumpless traversal of the table:

  1. gains are interpolated continuously, not selected from discrete regions -
     a region-select scheme steps the P term the instant a boundary is crossed;
  2. the core stores the integral term in output units, so changing Ki does not
     rescale accumulated history.

With SMOOTH interpolation the gain curve is also C1-continuous, which matters
when the scheduling variable is itself noisy: linear interpolation has a slope
discontinuity at every breakpoint that noise will rattle.
"""

from . import mathutil as m
from .types import (PIDX_GAINSCHED_MAX_POINTS, FEAT_GAIN_SCHED, SchedInterp,
                    SchedSource, Status)


class GainPoint:
    """One breakpoint. Tables must be sorted by strictly ascending x."""

    __slots__ = ("x", "kp", "ki", "kd")

    def __init__(self, x, kp, ki, kd):
        self.x = x
        self.kp = kp
        self.ki = ki
        self.kd = kd


def _smoothstep(t):
    """3t^2 - 2t^3.

    Its derivative is zero at both ends, so the interpolated gain curve is C1
    continuous across breakpoints. Linear interpolation has a slope jump there,
    which a noisy scheduling variable turns into audible gain chatter.
    """
    return (t * t) * (3.0 - (2.0 * t))


class GainSchedule:
    """A gain table plus its interpolation policy."""

    __slots__ = ("points", "count", "source", "interp", "hysteresis",
                 "last_x", "primed")

    def __init__(self):
        self.points = None
        self.count = 0
        self.source = int(SchedSource.SETPOINT)
        self.interp = int(SchedInterp.LINEAR)
        self.hysteresis = 0.0
        self.last_x = 0.0
        self.primed = False

    def init(self, points, source=SchedSource.SETPOINT,
             interp=SchedInterp.LINEAR):
        """Validate and install a table.

        The points are referenced, not copied, matching the C layer where the
        array can live in Flash as `static const`.
        """
        if points is None:
            return Status.ERR_NULL
        count = len(points)
        if count < 2 or count > PIDX_GAINSCHED_MAX_POINTS:
            return Status.ERR_INVALID_PARAM
        if source > SchedSource.EXTERNAL or interp > SchedInterp.HOLD:
            return Status.ERR_INVALID_PARAM

        for i, p in enumerate(points):
            if not m.isfinite(p.x):
                return Status.ERR_INVALID_PARAM
            # Strictly ascending: equal breakpoints would divide by zero, and
            # a descending table is always a mistake rather than an intent.
            if i > 0 and p.x <= points[i - 1].x:
                return Status.ERR_INVALID_PARAM
            if (not m.isfinite(p.kp) or p.kp < 0.0
                    or not m.isfinite(p.ki) or p.ki < 0.0
                    or not m.isfinite(p.kd) or p.kd < 0.0):
                return Status.ERR_INVALID_GAIN

        self.points = points
        self.count = count
        self.source = int(source)
        self.interp = int(interp)
        self.hysteresis = 0.0
        self.last_x = points[0].x
        self.primed = False
        return Status.OK

    def set_hysteresis(self, band):
        if not m.isfinite(band) or band < 0.0:
            return Status.ERR_INVALID_PARAM
        self.hysteresis = band
        return Status.OK

    def evaluate(self, x):
        """Interpolate at x. Returns (status, kp, ki, kd)."""
        if self.points is None:
            return Status.ERR_NULL, 0.0, 0.0, 0.0
        if not m.isfinite(x):
            return Status.ERR_INVALID_PARAM, 0.0, 0.0, 0.0

        # Hysteresis: ignore movement smaller than the band so sensor noise
        # around a breakpoint does not dither the gains.
        if self.primed and self.hysteresis > 0.0:
            if m.fabs(x - self.last_x) < self.hysteresis:
                x = self.last_x
        self.last_x = x
        self.primed = True

        p = self.points

        # Outside the table the gains saturate at the end points.
        # Extrapolating would be worse than useless: it can go negative.
        if x <= p[0].x:
            return Status.OK, p[0].kp, p[0].ki, p[0].kd
        last = self.count - 1
        if x >= p[last].x:
            return Status.OK, p[last].kp, p[last].ki, p[last].kd

        i = 0
        while i < (self.count - 1):
            if p[i].x <= x < p[i + 1].x:
                break
            i += 1

        t = (x - p[i].x) / (p[i + 1].x - p[i].x)   # ascending: safe

        if self.interp == SchedInterp.HOLD:
            t = 0.0
        elif self.interp == SchedInterp.SMOOTH:
            t = _smoothstep(t)

        return (Status.OK,
                m.lerp(p[i].kp, p[i + 1].kp, t),
                m.lerp(p[i].ki, p[i + 1].ki, t),
                m.lerp(p[i].kd, p[i + 1].kd, t))


def attach(pid, schedule):
    """Attach a schedule to a controller, or pass None to detach.

    On detach the gains stay at their last interpolated values.
    """
    if pid is None:
        return Status.ERR_NULL
    if schedule is not None and (schedule.points is None or schedule.count < 2):
        return Status.ERR_INVALID_PARAM   # never passed through init()

    pid.sched = schedule
    if schedule is not None:
        pid.features |= FEAT_GAIN_SCHED
    else:
        pid.features &= ~FEAT_GAIN_SCHED
    return Status.OK


def set_var(pid, value):
    """Supply the scheduling variable when the source is EXTERNAL."""
    if pid is None:
        return Status.ERR_NULL
    if not m.isfinite(value):
        return Status.ERR_INVALID_PARAM
    pid.sched_var_ext = value
    return Status.OK

"""Standalone signal-conditioning filters. Mirrors pid_filter.h / pid_filter.c.

These are ordinary reusable filters, not part of the control law. The core uses
LPF1 for its optional input filter; the rest are for conditioning a sensor
signal before it reaches the controller.

Every filter here adds phase lag, and phase lag is what destabilises a loop.
Filter the least you can get away with. Median3 is the exception worth reaching
for first: it removes isolated spikes almost for free, and a spike is exactly
what a low-pass handles worst.
"""

from . import mathutil as m
from .types import Status


class LPF1:
    """First-order IIR low-pass: y[k] = a*y[k-1] + (1-a)*x[k], a = tau/(tau+dt).

    The coefficient is the exact backward-Euler discretisation of 1/(1+s*tau),
    so `a` stays in [0,1) for every tau >= 0 and dt > 0 - the filter cannot be
    made unstable by a bad parameter. Cutoff is fc = 1/(2*pi*tau).
    """

    __slots__ = ("a", "state", "tau", "primed")

    def __init__(self, tau=0.0, dt=0.0):
        self.a = 0.0
        self.state = 0.0
        self.tau = tau
        self.primed = False
        if dt > 0.0:
            self.coeff(tau, dt)

    def coeff(self, tau, dt):
        """Recompute the pole for a new tau/dt pair."""
        self.tau = tau
        self.a = (tau / (tau + dt)) if tau > 0.0 else 0.0

    def step(self, x):
        """Advance one sample without validation. The core's hot path."""
        if not self.primed:
            self.state = x
            self.primed = True
        else:
            self.state = (self.a * self.state) + ((1.0 - self.a) * x)
        return self.state

    # -- validated API -----------------------------------------------------

    def init(self, tau, dt):
        if not m.isfinite(tau) or tau < 0.0:
            return Status.ERR_INVALID_PARAM
        if not m.isfinite(dt) or dt <= 0.0:
            return Status.ERR_INVALID_DT
        self.state = 0.0
        self.primed = False
        self.coeff(tau, dt)
        return Status.OK

    def set_tau(self, tau, dt):
        if not m.isfinite(tau) or tau < 0.0:
            return Status.ERR_INVALID_PARAM
        if not m.isfinite(dt) or dt <= 0.0:
            return Status.ERR_INVALID_DT
        self.coeff(tau, dt)
        return Status.OK

    def set_cutoff(self, fc_hz, dt):
        """Configure by cutoff frequency: tau = 1/(2*pi*fc)."""
        if not m.isfinite(fc_hz) or fc_hz <= 0.0:
            return Status.ERR_INVALID_PARAM
        return self.set_tau(1.0 / (6.283185307179586 * fc_hz), dt)

    def update(self, x):
        """Filter one sample. A non-finite sample holds the state instead of
        poisoning it."""
        if not m.isfinite(x):
            return self.state
        return self.step(x)

    def reset(self):
        self.state = 0.0
        self.primed = False
        return Status.OK


class MovingAvg:
    """Boxcar moving average over a fixed-size window.

    Better than an LPF at killing periodic noise whose period divides the
    window (mains hum: window = one mains period), and it has exactly linear
    phase. Worse at everything else.

    The running sum is fully recomputed once per completed window: a naive
    running sum accumulates rounding error without bound over hours of
    operation, and this bounds it to one window at the cost of one O(N) pass
    every N samples.
    """

    __slots__ = ("buffer", "size", "index", "count", "sum")

    def __init__(self, size):
        if size <= 0:
            raise ValueError("size must be positive")
        self.buffer = [0.0] * size
        self.size = size
        self.index = 0
        self.count = 0
        self.sum = 0.0

    def update(self, x):
        if not m.isfinite(x):
            x = 0.0

        self.sum -= self.buffer[self.index]
        self.buffer[self.index] = x
        self.sum += x

        self.index += 1
        if self.index >= self.size:
            self.index = 0
            acc = 0.0
            for v in self.buffer:
                acc += v
            self.sum = acc

        if self.count < self.size:
            self.count += 1

        return self.sum / float(self.count)

    def reset(self):
        self.buffer = [0.0] * self.size
        self.index = 0
        self.count = 0
        self.sum = 0.0
        return Status.OK


class Median3:
    """Three-sample median despiker.

    Removes any isolated single-sample outlier completely while passing ramps
    and steps with one sample of delay. The right first line of defence against
    ADC glitches; a low-pass would smear the glitch across many samples.
    """

    __slots__ = ("x1", "x2", "count")

    def __init__(self):
        self.x1 = 0.0
        self.x2 = 0.0
        self.count = 0

    def update(self, x):
        if not m.isfinite(x):
            return self.x1

        a, b, c = self.x2, self.x1, x
        self.x2 = self.x1
        self.x1 = x

        if self.count < 2:
            self.count += 1
            return x

        # Median of three by pairwise comparison: 3 compares, no sorting.
        if a > b:
            a, b = b, a
        if b > c:
            b = c
        return a if a > b else b


class RateLimiter:
    """Slew-rate limiter: |dy/dt| <= rate_max."""

    __slots__ = ("value", "rate_max", "primed")

    def __init__(self, rate_max=0.0):
        self.value = 0.0
        self.rate_max = rate_max
        self.primed = False

    def update(self, x, dt):
        if not m.isfinite(x) or not m.isfinite(dt) or dt <= 0.0:
            return self.value
        if not self.primed:
            self.value = x
            self.primed = True
            return x
        if self.rate_max <= 0.0:
            self.value = x
            return x

        max_step = self.rate_max * dt
        delta = x - self.value
        if delta > max_step:
            self.value += max_step
        elif delta < -max_step:
            self.value -= max_step
        else:
            self.value = x
        return self.value

    def reset(self, value=0.0):
        if not m.isfinite(value):
            return Status.ERR_INVALID_PARAM
        self.value = value
        self.primed = True
        return Status.OK


def deadband(x, width, subtract=True):
    """Symmetric deadband around zero.

    subtract=True gives a continuous output, sign(x)*(|x|-width), with no jump
    at the band edge - the only form fit for a control path. subtract=False
    zeroes hard and steps by `width` at the edge; use it for display only.

    It does not remove steady-state error: it declares a band in which you
    accept the error, which is how it stops a controller hunting against
    backlash or a quantised sensor.
    """
    if width > 0.0:
        if m.fabs(x) <= width:
            return 0.0
        if subtract:
            return (x - width) if x > 0.0 else (x + width)
    return x

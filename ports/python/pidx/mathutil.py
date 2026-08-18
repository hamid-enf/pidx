"""Scalar helpers matching include/pidx/pid_math.h.

The C versions inspect the IEEE-754 bit pattern so that they keep working under
-ffast-math, which tells the compiler NaNs do not exist and quietly deletes an
`x != x` test. Python has no such optimiser, so the straightforward spellings
below are exactly equivalent - and the bit-twiddling would be slower here.
"""

import math

#: Machine epsilon for IEEE-754 binary64, the format Python floats use.
PID_FLOAT_EPS = 2.220446049250313e-16


def isfinite(x):
    """True when x is neither NaN nor +/-Inf."""
    return not (math.isnan(x) or math.isinf(x))


def isnan(x):
    return math.isnan(x)


def fabs(x):
    return math.fabs(x)


def clamp(x, lo, hi):
    """Constrain x to [lo, hi].

    Ordered lo-then-hi exactly as the C macro is, so an inverted range
    (lo > hi) resolves to hi in both languages rather than to whichever
    bound the implementation happened to test first.
    """
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def sign(x):
    """-1, 0 or +1. Returns 0 for exactly zero, matching the C helper."""
    if x > 0.0:
        return 1.0
    if x < 0.0:
        return -1.0
    return 0.0


def lerp(a, b, t):
    """Linear interpolation.

    Written as a + t*(b - a) rather than (1-t)*a + t*b to match the C helper
    bit for bit; the two forms differ in the last ulp and that difference
    would show up in the gain-scheduling comparison.
    """
    return a + (t * (b - a))


def safe_inv(x):
    """1/x, or 0 when x is too small for the reciprocal to mean anything.

    The 16*EPS threshold is the C library's; keeping it identical matters
    because the two implementations must agree on WHICH inputs are treated
    as unusable, not merely on the arithmetic.
    """
    if fabs(x) <= (16.0 * PID_FLOAT_EPS):
        return 0.0
    return 1.0 / x


def sqrt(x):
    """Square root of a non-negative value; 0 for negative input.

    The C library offers a libm-free Newton iteration for targets without a
    hardware square root. Both paths converge to the correctly rounded
    result for the magnitudes used here (Kt = sqrt(Ki/Kd)), so Python simply
    uses math.sqrt.
    """
    if x <= 0.0:
        return 0.0
    return math.sqrt(x)

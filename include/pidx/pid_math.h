/**
 * @file    pid_math.h
 * @brief   Tiny, dependency-free numeric helpers used on the PID hot path.
 *
 * Design rule: nothing in this header calls into libm, allocates, or branches
 * unpredictably. IEEE-754 classification is done with integer bit tests, which
 * is both faster than a libm call and immune to -ffast-math (which legally
 * lets the compiler delete `x != x` NaN checks).
 *
 * MISRA-C:2012 deviation: Rule 19.2 (union type punning). Justified - it is
 * the only way to classify a float without libm or violating strict aliasing.
 * Type punning through a union is explicitly well-defined in C99 6.5.2.3p3
 * (footnote 82) and supported by GCC/Clang/ARMCC. See docs/22_misra_deviations.md
 */
#ifndef PIDX_PID_MATH_H
#define PIDX_PID_MATH_H

#include <stdint.h>
#include <stdbool.h>

#include "pid_conf.h"

#if PIDX_USE_DOUBLE
typedef double PID_Float;
#define PID_FLOAT_EPS  ((PID_Float)2.220446049250313e-16)
#else
typedef float PID_Float;
#define PID_FLOAT_EPS  ((PID_Float)1.1920929e-7f)
#endif

/* Literal constants, correctly typed regardless of PIDX_USE_DOUBLE. */
#define PID_ZERO   ((PID_Float)0.0f)
#define PID_ONE    ((PID_Float)1.0f)
#define PID_TWO    ((PID_Float)2.0f)
#define PID_HALF   ((PID_Float)0.5f)
#define PID_HUGE_F ((PID_Float)PIDX_HUGE)

/* ------------------------------------------------------------------------ */
/* IEEE-754 classification                                                    */
/* ------------------------------------------------------------------------ */

/** @return true if x is neither NaN nor +/-Inf. */
PIDX_INLINE bool pidm_isfinite(PID_Float x)
{
#if PIDX_USE_DOUBLE
    union { double f; uint64_t u; } v;
    v.f = x;
    return ((v.u & 0x7FF0000000000000ULL) != 0x7FF0000000000000ULL);
#else
    union { float f; uint32_t u; } v;
    v.f = x;
    return ((v.u & 0x7F800000U) != 0x7F800000U);
#endif
}

/** @return true if x is NaN. */
PIDX_INLINE bool pidm_isnan(PID_Float x)
{
#if PIDX_USE_DOUBLE
    union { double f; uint64_t u; } v;
    v.f = x;
    return (((v.u & 0x7FF0000000000000ULL) == 0x7FF0000000000000ULL) &&
            ((v.u & 0x000FFFFFFFFFFFFFULL) != 0ULL));
#else
    union { float f; uint32_t u; } v;
    v.f = x;
    return (((v.u & 0x7F800000U) == 0x7F800000U) &&
            ((v.u & 0x007FFFFFU) != 0U));
#endif
}

/** Branch-free absolute value (clears the sign bit). */
PIDX_INLINE PID_Float pidm_abs(PID_Float x)
{
#if PIDX_USE_DOUBLE
    union { double f; uint64_t u; } v;
    v.f = x;
    v.u &= 0x7FFFFFFFFFFFFFFFULL;
    return v.f;
#else
    union { float f; uint32_t u; } v;
    v.f = x;
    v.u &= 0x7FFFFFFFU;
    return v.f;
#endif
}

/* ------------------------------------------------------------------------ */
/* Range helpers                                                              */
/* ------------------------------------------------------------------------ */

/** Clamp x into [lo, hi]. Assumes lo <= hi (validated by the setters). */
PIDX_INLINE PID_Float pidm_clamp(PID_Float x, PID_Float lo, PID_Float hi)
{
    PID_Float r = x;
    if (r < lo) { r = lo; }
    if (r > hi) { r = hi; }
    return r;
}

/** @return -1, 0 or +1 as a PID_Float, without branching on FP compare chains. */
PIDX_INLINE PID_Float pidm_sign(PID_Float x)
{
    PID_Float s = PID_ZERO;
    if (x > PID_ZERO) { s = PID_ONE; }
    else if (x < PID_ZERO) { s = -PID_ONE; }
    else { /* exactly zero */ }
    return s;
}

/** Linear interpolation: a + t*(b-a), with t expected in [0,1]. */
PIDX_INLINE PID_Float pidm_lerp(PID_Float a, PID_Float b, PID_Float t)
{
    return a + (t * (b - a));
}

/**
 * Guarded reciprocal. Returns 0 when |x| is too small to invert safely, which
 * lets callers write branch-light code that degrades to "feature off" instead
 * of producing Inf.
 */
PIDX_INLINE PID_Float pidm_safe_inv(PID_Float x)
{
    PID_Float r = PID_ZERO;
    if (pidm_abs(x) > (PID_FLOAT_EPS * (PID_Float)16.0f)) {
        r = PID_ONE / x;
    }
    return r;
}

/**
 * Square root without libm (Newton-Raphson, seeded by an exponent halving).
 * Used only on cold paths (Kt derivation, auto-tune analysis).
 * Accurate to ~1 ulp for normal positive inputs; returns 0 for x <= 0.
 */
PIDX_INLINE PID_Float pidm_sqrt(PID_Float x)
{
    if (x <= PID_ZERO) {
        return PID_ZERO;
    }
#if PIDX_USE_LIBM
    {
        /* Declared locally to keep <math.h> out of the public header set. */
        extern double sqrt(double);
        return (PID_Float)sqrt((double)x);
    }
#else
    {
        union { float f; uint32_t u; } v;
        PID_Float g;
        int i;
        v.f = (float)x;
        /* Halve the biased exponent: a classic 1-line seed, ~5% accurate. */
        v.u = 0x1FBD1DF5U + (v.u >> 1);
        g = (PID_Float)v.f;
        for (i = 0; i < 4; ++i) {
            g = PID_HALF * (g + (x / g));
        }
        return g;
    }
#endif
}

#endif /* PIDX_PID_MATH_H */

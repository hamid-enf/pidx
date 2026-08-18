// PidxMath.cs - scalar helpers matching include/pidx/pid_math.h.
//
// The C versions inspect the IEEE-754 bit pattern so they keep working under
// -ffast-math, which tells the compiler NaNs do not exist and quietly deletes
// an `x != x` test. The CLR has no such licence, so the straightforward
// spellings below are exactly equivalent.

using System;

namespace Pidx
{
    internal static class M
    {
        /// <summary>Machine epsilon for IEEE-754 binary64.</summary>
        public const double FloatEps = 2.220446049250313e-16;

        public static bool IsFinite(double x)
        {
            return !(double.IsNaN(x) || double.IsInfinity(x));
        }

        public static bool IsNaN(double x)
        {
            return double.IsNaN(x);
        }

        public static double Abs(double x)
        {
            return Math.Abs(x);
        }

        /// <summary>
        /// Constrain x to [lo, hi]. Ordered lo-then-hi exactly as the C macro
        /// is, so an inverted range (lo > hi) resolves to hi in both
        /// languages rather than to whichever bound was tested first.
        /// </summary>
        public static double Clamp(double x, double lo, double hi)
        {
            if (x < lo) return lo;
            if (x > hi) return hi;
            return x;
        }

        /// <summary>-1, 0 or +1; 0 for exactly zero, matching the C helper.</summary>
        public static double Sign(double x)
        {
            if (x > 0.0) return 1.0;
            if (x < 0.0) return -1.0;
            return 0.0;
        }

        /// <summary>
        /// Linear interpolation, written as a + t*(b - a) to match the C
        /// helper bit for bit; the algebraically equal (1-t)*a + t*b differs
        /// in the last ulp and that would show up in the gain-schedule
        /// comparison.
        /// </summary>
        public static double Lerp(double a, double b, double t)
        {
            return a + (t * (b - a));
        }

        /// <summary>
        /// 1/x, or 0 when x is too small for the reciprocal to mean anything.
        /// The 16*EPS threshold is the C library's; keeping it identical
        /// matters because the implementations must agree on WHICH inputs are
        /// unusable, not merely on the arithmetic.
        /// </summary>
        public static double SafeInv(double x)
        {
            if (Abs(x) <= (16.0 * FloatEps)) return 0.0;
            return 1.0 / x;
        }

        /// <summary>Square root of a non-negative value; 0 for negative input.</summary>
        public static double Sqrt(double x)
        {
            if (x <= 0.0) return 0.0;
            return Math.Sqrt(x);
        }
    }
}

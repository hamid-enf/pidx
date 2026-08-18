// GainSchedule.cs - interpolated gain scheduling. Mirrors pid_gainsched.c.
//
// A single set of gains is only optimal near one operating point. A valve is
// more effective half open than nearly shut; a motor's effective inertia
// changes with arm extension; a heater's loss coefficient grows with
// temperature.
//
// Two properties together guarantee a bumpless traversal of the table:
//
//   1. gains are interpolated continuously, not selected from discrete
//      regions - a region-select scheme steps the P term the instant a
//      boundary is crossed;
//   2. the core stores the integral term in output units, so changing Ki does
//      not rescale accumulated history.
//
// With SchedInterp.Smooth the gain curve is also C1-continuous, which matters
// when the scheduling variable is itself noisy: linear interpolation has a
// slope discontinuity at every breakpoint that noise will rattle.

using System;

namespace Pidx
{
    /// <summary>One breakpoint. Tables must be sorted by ascending X.</summary>
    public struct GainPoint
    {
        public double X;
        public double Kp;
        public double Ki;
        public double Kd;

        public GainPoint(double x, double kp, double ki, double kd)
        {
            X = x; Kp = kp; Ki = ki; Kd = kd;
        }
    }

    /// <summary>A gain table plus its interpolation policy.</summary>
    public sealed class GainSchedule
    {
        internal GainPoint[] Points;
        internal int Count;
        public SchedSource Source = SchedSource.Setpoint;
        public SchedInterp Interp = SchedInterp.Linear;
        internal double Hysteresis;
        internal double LastX;
        internal bool Primed;

        public GainSchedule() { }

        public GainSchedule(GainPoint[] points,
                            SchedSource source = SchedSource.Setpoint,
                            SchedInterp interp = SchedInterp.Linear)
        {
            Status rc = Init(points, source, interp);
            if (rc != Status.Ok)
                throw new ArgumentException("schedule init failed: " + rc);
        }

        /// <summary>
        /// Validate and install a table. The array is referenced, not copied,
        /// matching the C layer where it can live in Flash as static const.
        /// </summary>
        public Status Init(GainPoint[] points,
                           SchedSource source = SchedSource.Setpoint,
                           SchedInterp interp = SchedInterp.Linear)
        {
            if (points == null) return Status.ErrNull;
            int n = points.Length;
            if (n < 2 || n > Consts.GainSchedMaxPoints)
                return Status.ErrInvalidParam;
            if (source > SchedSource.External || interp > SchedInterp.Hold)
                return Status.ErrInvalidParam;

            for (int i = 0; i < n; i++)
            {
                if (!M.IsFinite(points[i].X)) return Status.ErrInvalidParam;
                // Strictly ascending: equal breakpoints would divide by zero,
                // and a descending table is always a mistake, not an intent.
                if (i > 0 && points[i].X <= points[i - 1].X)
                    return Status.ErrInvalidParam;
                if (!M.IsFinite(points[i].Kp) || points[i].Kp < 0.0
                    || !M.IsFinite(points[i].Ki) || points[i].Ki < 0.0
                    || !M.IsFinite(points[i].Kd) || points[i].Kd < 0.0)
                    return Status.ErrInvalidGain;
            }

            Points = points;
            Count = n;
            Source = source;
            Interp = interp;
            Hysteresis = 0.0;
            LastX = points[0].X;
            Primed = false;
            return Status.Ok;
        }

        public Status SetHysteresis(double band)
        {
            if (!M.IsFinite(band) || band < 0.0) return Status.ErrInvalidParam;
            Hysteresis = band;
            return Status.Ok;
        }

        /// <summary>Interpolate at x.</summary>
        public Status Evaluate(double x, out double kp, out double ki,
                               out double kd)
        {
            kp = 0.0; ki = 0.0; kd = 0.0;

            if (Points == null) return Status.ErrNull;
            if (!M.IsFinite(x)) return Status.ErrInvalidParam;

            // Hysteresis: ignore movement smaller than the band so sensor
            // noise around a breakpoint does not dither the gains.
            if (Primed && Hysteresis > 0.0)
            {
                if (M.Abs(x - LastX) < Hysteresis) x = LastX;
            }
            LastX = x;
            Primed = true;

            GainPoint[] p = Points;

            // Outside the table the gains saturate at the end points.
            // Extrapolating would be worse than useless: it can go negative.
            if (x <= p[0].X)
            {
                kp = p[0].Kp; ki = p[0].Ki; kd = p[0].Kd;
                return Status.Ok;
            }
            int last = Count - 1;
            if (x >= p[last].X)
            {
                kp = p[last].Kp; ki = p[last].Ki; kd = p[last].Kd;
                return Status.Ok;
            }

            int i = 0;
            while (i < Count - 1)
            {
                if (x >= p[i].X && x < p[i + 1].X) break;
                i++;
            }

            double t = (x - p[i].X) / (p[i + 1].X - p[i].X);  // ascending: safe

            if (Interp == SchedInterp.Hold)
            {
                t = 0.0;
            }
            else if (Interp == SchedInterp.Smooth)
            {
                // Smoothstep 3t^2 - 2t^3: derivative zero at both ends, so the
                // gain curve is C1 continuous across breakpoints.
                t = (t * t) * (3.0 - (2.0 * t));
            }

            kp = M.Lerp(p[i].Kp, p[i + 1].Kp, t);
            ki = M.Lerp(p[i].Ki, p[i + 1].Ki, t);
            kd = M.Lerp(p[i].Kd, p[i + 1].Kd, t);
            return Status.Ok;
        }

        /// <summary>
        /// Attach a schedule to a controller, or pass null to detach. On
        /// detach the gains stay at their last interpolated values.
        /// </summary>
        public static Status Attach(Pid pid, GainSchedule schedule)
        {
            if (pid == null) return Status.ErrNull;
            if (schedule != null && (schedule.Points == null || schedule.Count < 2))
                return Status.ErrInvalidParam;   // never passed through Init()

            pid.Sched = schedule;
            if (schedule != null) pid.Features |= Consts.FeatGainSched;
            else pid.Features &= ~Consts.FeatGainSched;
            return Status.Ok;
        }

        /// <summary>Supply the scheduling variable when the source is External.</summary>
        public static Status SetVar(Pid pid, double value)
        {
            if (pid == null) return Status.ErrNull;
            if (!M.IsFinite(value)) return Status.ErrInvalidParam;
            pid.SchedVarExt = value;
            return Status.Ok;
        }
    }
}

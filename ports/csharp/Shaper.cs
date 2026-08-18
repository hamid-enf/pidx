// Shaper.cs - setpoint trajectory shaping. Mirrors pid_shaper.h / .c.
//
// A step change in the setpoint asks the plant for infinite acceleration. The
// controller responds with whatever the actuator can deliver, saturates, winds
// up and overshoots. Shaping the setpoint into something the plant can
// actually follow fixes the problem at its source instead of detuning the
// controller to survive a command that was never physical.
//
// Deceleration must begin when the remaining distance equals
//
//     d_brake = v^2 / (2 * a_decel)
//
// from integrating v*dv = a*dx. Starting later guarantees overshoot; starting
// earlier just wastes time. The check runs every sample against the current
// velocity, so the profile is self-correcting and a mid-flight target change
// needs no replanning.
//
// There is no jerk limit and no multi-axis coordination: those belong in a
// motion planner, not a PID library.

using System;

namespace Pidx
{
    /// <summary>A reusable trajectory generator.</summary>
    public sealed class Shaper
    {
        public double Position;
        public double Velocity;
        public double Target;
        public double RateMax;
        public double Accel;
        public double Decel;
        public bool Moving;

        public Shaper(double rateMax = 0.0, double accel = 0.0,
                      double decel = 0.0)
        {
            RateMax = rateMax;
            Accel = accel;
            Decel = decel;
        }

        /// <summary>
        /// Advance a trapezoidal/rate profile one step. The single
        /// implementation shared by the controller's built-in shaper and this
        /// standalone object, exactly as pids_profile_step() is in C.
        /// </summary>
        public static void ProfileStep(ref double pos, ref double vel,
                                       double target, double rateMax,
                                       double accel, double decel, double dt,
                                       out bool moving)
        {
            double dist = target - pos;
            double v = vel;
            moving = true;

            if (rateMax <= 0.0)
            {
                pos = target;
                vel = 0.0;
                moving = false;
                return;
            }
            if (dist == 0.0)
            {
                vel = 0.0;
                moving = false;
                return;
            }

            if (accel <= 0.0)
            {
                v = (dist > 0.0) ? rateMax : -rateMax;
            }
            else
            {
                double d = (decel > 0.0) ? decel : accel;
                double brake = (v * v) / (2.0 * d);

                if (dist > 0.0)
                {
                    if (v < 0.0) v += d * dt;              // wrong way
                    else if (brake >= dist) v -= d * dt;   // start braking
                    else v += accel * dt;                  // speed up
                }
                else
                {
                    if (v > 0.0) v -= d * dt;
                    else if (brake >= -dist) v += d * dt;
                    else v -= accel * dt;
                }
                v = M.Clamp(v, -rateMax, rateMax);
            }

            double step = v * dt;
            if (M.Abs(step) >= M.Abs(dist))
            {
                // Landing sample: snap to the target rather than overshoot it.
                pos = target;
                vel = 0.0;
                moving = false;
            }
            else
            {
                pos += step;
                vel = v;
            }
        }

        public Status Init(double rateMax, double accel, double decel)
        {
            if (!M.IsFinite(rateMax) || rateMax < 0.0
                || !M.IsFinite(accel) || accel < 0.0
                || !M.IsFinite(decel) || decel < 0.0)
                return Status.ErrInvalidParam;
            Position = 0.0;
            Velocity = 0.0;
            Target = 0.0;
            RateMax = rateMax;
            Accel = accel;
            Decel = decel;
            Moving = false;
            return Status.Ok;
        }

        /// <summary>Command a destination. Does not reset velocity - it blends.</summary>
        public Status SetTarget(double target)
        {
            if (!M.IsFinite(target)) return Status.ErrInvalidParam;
            Target = target;
            Moving = (target != Position);
            return Status.Ok;
        }

        /// <summary>Teleport to `position` with zero velocity, e.g. after homing.</summary>
        public Status Reset(double position = 0.0)
        {
            if (!M.IsFinite(position)) return Status.ErrInvalidParam;
            Position = position;
            Target = position;
            Velocity = 0.0;
            Moving = false;
            return Status.Ok;
        }

        /// <summary>Advance one timestep; returns the new shaped position.</summary>
        public double Update(double dt)
        {
            if (!M.IsFinite(dt) || dt <= 0.0) return Position;
            if (RateMax <= 0.0)
            {
                Position = Target;        // shaping disabled: pass through
                Velocity = 0.0;
                Moving = false;
                return Position;
            }

            bool moving;
            ProfileStep(ref Position, ref Velocity, Target, RateMax, Accel,
                        Decel, dt, out moving);
            Moving = moving;
            return Position;
        }

        public bool IsMoving() { return Moving; }

        /// <summary>
        /// Seconds to reach the current target from rest. Trapezoidal when the
        /// distance is long enough to reach cruise speed, otherwise triangular
        /// with v_peak = sqrt(2*d*a*b/(a+b)). Ignores the current velocity by
        /// design - it answers "how long is this move", not "how long is left".
        /// </summary>
        public double EstimateTime()
        {
            double d = M.Abs(Target - Position);
            if (d == 0.0 || RateMax <= 0.0) return 0.0;

            double v = RateMax;
            double a = Accel;
            if (a <= 0.0) return d / v;          // rate-only profile
            double b = (Decel > 0.0) ? Decel : a;

            double dRamp = ((v * v) / (2.0 * a)) + ((v * v) / (2.0 * b));
            if (d >= dRamp)
                return ((d - dRamp) / v) + (v / a) + (v / b);

            double vp = M.Sqrt((2.0 * d * a * b) / (a + b));
            return (vp / a) + (vp / b);
        }
    }
}

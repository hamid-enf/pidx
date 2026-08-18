// Pid.cs - the PIDX control law. A faithful port of src/pid.c.
//
// CONTROL LAW
//   u = Kp*(beta*r - y) + Ki*integral(r - y)
//       + (Kd*s/(1 + s*Tf))*(gamma*r - y) + u_ff
//
// The integral always acts on the UNWEIGHTED error (r - y). Weighting it would
// leave a steady-state offset of (1 - beta)*r, which is not a tuning choice,
// it is a bug.
//
// INTEGRATOR UNITS
//   `integrator` holds Ki*integral(e) - the integral TERM in output units, not
//   the raw integral of the error. So changing Ki at runtime does not step the
//   output (gain scheduling and auto-tune application are bumpless for free),
//   and integral limits share units with the output limits.
//   SetGainsRescaleIntegral() gives the classic semantics.
//
// DERIVATIVE
//   Discretised as one filtered block rather than a difference then a
//   low-pass:
//       D_k  = c_da*D_{k-1} - c_db*(x_k - x_{k-1})
//       c_da = Tf/(Tf + dt),   c_db = Kd/(Tf + dt)
//   The pole c_da lies in [0,1) for every Tf >= 0 and dt > 0, so the term can
//   never diverge. The naive form has pole (1 - dt/Tf), which leaves the unit
//   circle as soon as Tf < dt/2 - exactly when a user picks a light filter.
//
// STAGE ORDER
//   0 guards, 1 timing, 2 sensor validation, 3 input filter, 4 setpoint,
//   5 gain scheduling, 6 error and P, 7 derivative, 8 feedforward,
//   9 manual/hold, 10 integral, 11 sum, 12 output saturation,
//   13 back-calculation/tracking/conditional rollback, 14 output slew.
//
//   The order is not arbitrary. Anti-windup in stage 13 runs in the SAME
//   sample as the saturation it corrects; deferring it one cycle inserts a
//   delay into the anti-windup loop that shows up as extra overshoot.

using System;

namespace Pidx
{
    /// <summary>A PID controller. The analogue of PID_Handle plus its API.</summary>
    public sealed class Pid
    {
        // -- state ------------------------------------------------------
        internal bool Initialised;

        internal double Kp, Ki, Kd;

        // dt-dependent coefficients, rebuilt by Recompute()
        internal double CI, CDa, CDb, CAw;
        internal double DtLast, DtNominal, DtMin, DtMax;

        internal double SetpointEff, SetpointTarget, SpVelocity;

        internal double Integrator, DState, DPrevIn, EPrev;
        internal double OutputValue, ManualOutputValue, TrackingInput;

        internal double OutMin = -Consts.HugeF, OutMax = Consts.HugeF;
        internal double IMin = -Consts.HugeF, IMax = Consts.HugeF;

        internal double Tf, NFilter, Kt;
        internal double ISeparation, IDeadband;
        internal double Beta = 1.0, Gamma;

        internal Func<double, double, double> FfFn;
        internal double FfValue, FfGain = 1.0;

        internal double SpRateMax, SpAccel, SpDecel, OutSlewMax;

        // input low-pass state
        internal double LpfA, LpfState, LpfTau;
        internal bool LpfPrimed;

        internal double MeasMin, MeasMax, MeasRateMax, FailsafeOutput;
        internal byte FaultPersistN = 1;
        internal bool AutoRecover;
        internal uint FaultCount;
        internal double MeasPrev;
        internal bool MeasPrevValid;

        internal GainSchedule Sched;
        internal double SchedVarExt;

        internal int DirSign = 1;
        internal Mode ModeValue = Mode.Automatic;
        internal IntegrationMethod IntegMethod = IntegrationMethod.BackwardEuler;
        internal AntiWindup AwMode = AntiWindup.Clamp;
        internal DerivativeMode DMode = DerivativeMode.OnMeasurement;

        internal uint Features;
        internal ushort Flags;
        internal Status LastError = Status.Ok;
        internal StatusSnapshot Snapshot = new StatusSnapshot();

        // ---------------------------------------------------------------
        // Lifecycle
        // ---------------------------------------------------------------

        public Pid()
        {
            ZeroState();
        }

        public Pid(Config cfg)
        {
            ZeroState();
            Status rc = Init(cfg);
            if (rc != Status.Ok)
            {
                throw new ArgumentException("PID init failed: " + rc);
            }
        }

        /// <summary>Put every field in a defined state, as PID_Init's memset does.</summary>
        private void ZeroState()
        {
            Initialised = false;
            Kp = Ki = Kd = 0.0;
            CI = CDa = CDb = CAw = 0.0;
            DtLast = DtNominal = DtMin = DtMax = 0.0;
            SetpointEff = SetpointTarget = SpVelocity = 0.0;
            Integrator = DState = DPrevIn = EPrev = 0.0;
            OutputValue = ManualOutputValue = TrackingInput = 0.0;
            OutMin = -Consts.HugeF; OutMax = Consts.HugeF;
            IMin = -Consts.HugeF; IMax = Consts.HugeF;
            Tf = NFilter = Kt = 0.0;
            ISeparation = IDeadband = 0.0;
            Beta = 1.0; Gamma = 0.0;
            FfFn = null; FfValue = 0.0; FfGain = 1.0;
            SpRateMax = SpAccel = SpDecel = OutSlewMax = 0.0;
            LpfA = LpfState = LpfTau = 0.0; LpfPrimed = false;
            MeasMin = MeasMax = MeasRateMax = FailsafeOutput = 0.0;
            FaultPersistN = 1; AutoRecover = false;
            FaultCount = 0; MeasPrev = 0.0; MeasPrevValid = false;
            Sched = null; SchedVarExt = 0.0;
            DirSign = 1;
            ModeValue = Mode.Automatic;
            IntegMethod = IntegrationMethod.BackwardEuler;
            AwMode = AntiWindup.Clamp;
            DMode = DerivativeMode.OnMeasurement;
            Features = 0; Flags = 0; LastError = Status.Ok;
            Snapshot = new StatusSnapshot();
        }

        /// <summary>A gain must be finite and non-negative; sign lives in Direction.</summary>
        private static bool GainOk(double g)
        {
            return M.IsFinite(g) && g >= 0.0;
        }

        /// <summary>
        /// Validate cfg and configure this controller. Validation happens
        /// BEFORE any field is written, so a rejected config leaves an
        /// already-working controller untouched.
        /// </summary>
        public Status Init(Config cfg)
        {
            if (cfg == null) return Status.ErrNull;
            if (cfg.AbiVersion != Consts.ConfigAbiVersion)
                return Status.ErrInvalidConfig;
            if (!GainOk(cfg.Core.Kp) || !GainOk(cfg.Core.Ki) || !GainOk(cfg.Core.Kd))
                return Status.ErrInvalidGain;
            if (!M.IsFinite(cfg.Core.SampleTime) || cfg.Core.SampleTime <= 0.0)
                return Status.ErrInvalidDt;
            if (cfg.Limits.UseOutputLimits &&
                (!M.IsFinite(cfg.Limits.OutputMin) ||
                 !M.IsFinite(cfg.Limits.OutputMax) ||
                 cfg.Limits.OutputMin >= cfg.Limits.OutputMax))
                return Status.ErrInvalidLimit;
            if (cfg.Limits.UseIntegralLimits &&
                cfg.Limits.IntegralMin >= cfg.Limits.IntegralMax)
                return Status.ErrInvalidLimit;
            if (cfg.Integral.Mode == AntiWindup.BackCalculation &&
                !cfg.Limits.UseOutputLimits && !cfg.Limits.UseIntegralLimits)
            {
                // u_sat would always equal u_raw, so the correction term is
                // identically zero: the user almost certainly forgot limits.
                return Status.ErrInvalidLimit;
            }
            if (!M.IsFinite(cfg.Weight.Beta) || !M.IsFinite(cfg.Weight.Gamma) ||
                cfg.Weight.Beta < 0.0 || cfg.Weight.Beta > 2.0 ||
                cfg.Weight.Gamma < 0.0 || cfg.Weight.Gamma > 2.0)
                return Status.ErrInvalidConfig;

            ZeroState();

            Kp = cfg.Core.Kp;
            Ki = cfg.Core.Ki;
            Kd = cfg.Core.Kd;
            DtNominal = cfg.Core.SampleTime;
            DtMin = cfg.Limits.DtMin;
            DtMax = cfg.Limits.DtMax;
            DirSign = (cfg.Core.Direction == Direction.Reverse) ? -1 : 1;
            ModeValue = cfg.Core.Mode;
            IntegMethod = cfg.Core.Integration;

            OutMin = cfg.Limits.OutputMin;
            OutMax = cfg.Limits.OutputMax;

            // IMin/IMax always hold the EFFECTIVE bounds, resolved once here.
            // Without explicit integral limits they inherit the output limits:
            // an integrator that can demand more than the actuator can deliver
            // is just windup waiting to happen. Resolving here rather than
            // per-cycle also lets the fast path clamp against them directly.
            if (cfg.Limits.UseIntegralLimits)
            {
                IMin = cfg.Limits.IntegralMin;
                IMax = cfg.Limits.IntegralMax;
            }
            else if (cfg.Limits.UseOutputLimits)
            {
                IMin = cfg.Limits.OutputMin;
                IMax = cfg.Limits.OutputMax;
            }
            else
            {
                IMin = -Consts.HugeF;
                IMax = Consts.HugeF;
            }

            DMode = cfg.Filter.DerivativeMode;
            Tf = cfg.Filter.Tf;
            NFilter = cfg.Filter.NFilter;

            AwMode = cfg.Integral.Mode;
            Kt = cfg.Integral.Kt;
            ISeparation = cfg.Integral.SeparationThreshold;
            IDeadband = cfg.Integral.Deadband;

            Beta = cfg.Weight.Beta;
            Gamma = cfg.Weight.Gamma;

            uint feat = Consts.FeatDerivative | Consts.FeatDFilter;
            if (cfg.Integral.Enabled) feat |= Consts.FeatIntegral;
            if (cfg.Limits.UseOutputLimits) feat |= Consts.FeatOutputLimit;
            if (cfg.Limits.UseIntegralLimits) feat |= Consts.FeatIntegralLimit;

            if (cfg.Feedforward.Enabled) feat |= Consts.FeatFeedforward;
            FfFn = cfg.Feedforward.Fn;
            FfValue = cfg.Feedforward.Value;
            FfGain = (cfg.Feedforward.Gain != 0.0) ? cfg.Feedforward.Gain : 1.0;

            SpRateMax = cfg.Shaper.SpRateMax;
            SpAccel = cfg.Shaper.SpAccel;
            SpDecel = cfg.Shaper.SpDecel;
            OutSlewMax = cfg.Shaper.OutSlewMax;
            if (SpRateMax > 0.0) feat |= Consts.FeatSpShaper;
            if (OutSlewMax > 0.0) feat |= Consts.FeatOutShaper;

            LpfTau = cfg.Filter.InputLpfTau;
            if (LpfTau > 0.0) feat |= Consts.FeatInputFilter;

            MeasMin = cfg.Safety.MeasMin;
            MeasMax = cfg.Safety.MeasMax;
            MeasRateMax = cfg.Safety.MeasRateMax;
            FailsafeOutput = cfg.Safety.FailsafeOutput;
            FaultPersistN = (cfg.Safety.FaultPersistN == 0)
                ? (byte)1 : cfg.Safety.FaultPersistN;
            AutoRecover = cfg.Safety.AutoRecover;
            if (cfg.Safety.Enabled) feat |= Consts.FeatSafety;

            feat |= Consts.FeatDiagnostics;

            Features = feat;
            TrackingInput = 0.0;
            LastError = Status.Ok;
            Initialised = true;

            Recompute(DtNominal);
            return Status.Ok;
        }

        public Status InitDefault()
        {
            return Init(new Config());
        }

        /// <summary>Invalidate. A later update reports ErrNotInit.</summary>
        public Status Deinit()
        {
            ZeroState();
            return Status.Ok;
        }

        // ---------------------------------------------------------------
        // Internal helpers
        // ---------------------------------------------------------------

        /// <summary>Record an error without ever overwriting it with success.</summary>
        private void SetError(Status code)
        {
            if (code != Status.Ok) LastError = code;
        }

        /// <summary>
        /// Resolve the derivative filter time constant. An explicit Tf always
        /// wins; otherwise Tf = Td/N = Kd/(N*Kp). That needs a non-zero Kp;
        /// with Kp == 0 (a pure ID controller - unusual but legal) the ratio
        /// is undefined and we fall back to an unfiltered derivative rather
        /// than inventing a value.
        /// </summary>
        private double EffectiveTf()
        {
            if ((Features & Consts.FeatDFilter) == 0) return 0.0;
            if (Tf > 0.0) return Tf;
            if (NFilter > 0.0 && Kp > 0.0 && Kd > 0.0) return Kd / (NFilter * Kp);
            return 0.0;
        }

        /// <summary>
        /// Back-calculation gain when the user passed 0. Astrom and Hagglund:
        /// Tt = sqrt(Ti*Td) with derivative action, Tt = Ti without. In
        /// parallel gains Ti = Kp/Ki and Td = Kd/Kp, so Ti*Td = Kd/Ki, giving
        /// Kt = sqrt(Ki/Kd) and Kt = Ki/Kp respectively.
        /// </summary>
        private double EffectiveKt()
        {
            double kt = Kt;
            if (kt <= 0.0)
            {
                if (Ki > 0.0 && Kd > 0.0) kt = M.Sqrt(Ki / Kd);
                else if (Ki > 0.0 && Kp > 0.0) kt = Ki / Kp;
                else kt = 0.0;
            }
            return kt;
        }

        /// <summary>
        /// Rebuild every dt-dependent coefficient. The only place that divides
        /// on behalf of the control law, which keeps the update path
        /// division-free.
        /// </summary>
        private void Recompute(double dt)
        {
            double tf = EffectiveTf();
            double den = tf + dt;

            CI = (IntegMethod == IntegrationMethod.Trapezoidal)
                ? Ki * dt * 0.5
                : Ki * dt;

            // den >= dt > 0, so this division is always safe.
            CDa = tf / den;
            CDb = Kd / den;

            CAw = EffectiveKt() * dt;

            // input LPF pole, exact backward-Euler discretisation
            LpfA = (LpfTau > 0.0) ? (LpfTau / (LpfTau + dt)) : 0.0;

            DtLast = dt;
        }

        /// <summary>
        /// Force the integrator so P + I + D + FF reproduces `desired`. One
        /// operation implementing bumpless manual-to-auto transfer, bumpless
        /// fault recovery and integrator preloading.
        ///
        /// When the clamp bites the transfer CANNOT be bumpless: the requested
        /// output is not reachable from the current P/D/FF with a legal
        /// integrator. That is flagged rather than silently accepted, because
        /// a "bumpless transfer" that quietly steps the actuator is worse than
        /// one that tells you it could not.
        /// </summary>
        private bool BackSolve(double desired, double p, double d, double ff)
        {
            double want = desired - p - d - ff;
            double got = M.Clamp(want, IMin, IMax);
            Integrator = got;

            if (got != want)
            {
                Flags |= Consts.FlagIntegralLimited;
                // Also sticky: the flag is rebuilt every cycle, so a caller
                // who switches mode and reads flags after the next update
                // would never see it. The sticky channel survives until the
                // application clears it.
                SetError(Status.ErrInvalidLimit);
                return false;
            }
            return true;
        }

        private void ShapeSetpoint(double dt)
        {
            bool moving;
            Shaper.ProfileStep(ref SetpointEff, ref SpVelocity, SetpointTarget,
                               SpRateMax, SpAccel, SpDecel, dt, out moving);
            if (moving) Flags |= Consts.FlagSpRamping;
            else Flags &= unchecked((ushort)~Consts.FlagSpRamping);
        }

        /// <summary>Range then slew plausibility. Ok means y may be trusted.</summary>
        private Status CheckSensor(double y, double dt)
        {
            if (MeasMax > MeasMin && (y < MeasMin || y > MeasMax))
                return Status.ErrSensorRange;
            if (MeasRateMax > 0.0 && MeasPrevValid)
            {
                if (M.Abs(y - MeasPrev) > (MeasRateMax * dt))
                    return Status.ErrSensorRate;
            }
            return Status.Ok;
        }

        // ---------------------------------------------------------------
        // The update path
        // ---------------------------------------------------------------

        private double Run(double meas, double dt, Input inp, out Status rc)
        {
            double y = meas;
            double ff = 0.0;
            double iPre = 0.0;
            bool iStepped = false;
            double recoverTo = 0.0;
            bool recovering = false;
            rc = Status.Ok;

            // -------- Stage 0: guards ----------------------------------
            if (!Initialised)
            {
                rc = Status.ErrNotInit;
                return 0.0;
            }

            if (!M.IsFinite(y))
            {
                rc = M.IsNaN(y) ? Status.ErrNanInput : Status.ErrInfInput;
                SetError(rc);
                Flags |= Consts.FlagSensorInvalid;
                if ((Features & Consts.FeatSafety) != 0)
                {
                    FaultCount++;
                    if (FaultCount >= FaultPersistN)
                    {
                        Flags |= Consts.FlagFault;
                        OutputValue = FailsafeOutput;
                    }
                }
                // Hold the previous output: one bad sample must not command
                // a jump.
                return OutputValue;
            }

            // Transient flags are rebuilt every cycle; FAULT is latched.
            Flags &= (ushort)(Consts.FlagFault | Consts.FlagTuning
                              | Consts.FlagSpRamping);

            // -------- Stage 1: timing ----------------------------------
            if (dt <= 0.0)
            {
                rc = Status.ErrInvalidDt;
                SetError(rc);
                Flags |= Consts.FlagDtViolation;
                dt = DtNominal;
            }
            else if ((DtMin > 0.0 && dt < DtMin) || (DtMax > 0.0 && dt > DtMax))
            {
                rc = Status.ErrInvalidDt;
                SetError(rc);
                Flags |= Consts.FlagDtViolation;
                dt = M.Clamp(dt,
                             (DtMin > 0.0) ? DtMin : dt,
                             (DtMax > 0.0) ? DtMax : dt);
            }

            if (dt != DtLast) Recompute(dt);

            // -------- Stage 2: sensor validation -----------------------
            if ((Features & Consts.FeatSafety) != 0)
            {
                Status sc = CheckSensor(y, dt);

                if (sc != Status.Ok)
                {
                    SetError(sc);
                    Flags |= Consts.FlagSensorInvalid;
                    FaultCount++;
                    if (FaultCount >= FaultPersistN) Flags |= Consts.FlagFault;
                }
                else if (FaultCount > 0)
                {
                    if (AutoRecover)
                    {
                        FaultCount = 0;
                        if ((Flags & Consts.FlagFault) != 0)
                        {
                            // Bumpless re-entry. The back-solve is DEFERRED to
                            // stage 10 because P, D and FF for this sample do
                            // not exist yet: solving now with zeros sets I to
                            // the failsafe output and the real P term is then
                            // added on top, which is a measurable step exactly
                            // where this code exists to prevent one.
                            Flags &= unchecked((ushort)~Consts.FlagFault);
                            DPrevIn = y;
                            recoverTo = OutputValue;
                            recovering = true;
                        }
                    }
                    else
                    {
                        FaultCount = 0;   // sample fine; latch stays put
                    }
                }

                MeasPrev = y;
                MeasPrevValid = true;

                if ((Flags & Consts.FlagFault) != 0)
                {
                    OutputValue = FailsafeOutput;
                    if (rc == Status.Ok) rc = Status.ErrSensorRange;
                    return OutputValue;
                }
            }

            // -------- Stage 3: input filter ----------------------------
            if ((Features & Consts.FeatInputFilter) != 0)
            {
                if (!LpfPrimed)
                {
                    LpfState = y;
                    LpfPrimed = true;
                }
                else
                {
                    LpfState = (LpfA * LpfState) + ((1.0 - LpfA) * y);
                }
                y = LpfState;
            }

            // -------- Stage 4: setpoint --------------------------------
            if (inp != null && M.IsFinite(inp.Setpoint))
            {
                SetpointTarget = inp.Setpoint;
            }

            if ((Features & Consts.FeatSpShaper) != 0) ShapeSetpoint(dt);
            else SetpointEff = SetpointTarget;
            double sp = SetpointEff;

            // -------- Stage 5: gain scheduling -------------------------
            if ((Features & Consts.FeatGainSched) != 0 && Sched != null)
            {
                double var;
                if (inp != null && M.IsFinite(inp.ScheduleVar))
                {
                    var = inp.ScheduleVar;
                }
                else
                {
                    switch (Sched.Source)
                    {
                        case SchedSource.Setpoint:    var = sp; break;
                        case SchedSource.Measurement: var = y; break;
                        case SchedSource.Error:       var = sp - y; break;
                        case SchedSource.AbsError:    var = M.Abs(sp - y); break;
                        case SchedSource.Output:      var = OutputValue; break;
                        default:                      var = SchedVarExt; break;
                    }
                }

                double nkp, nki, nkd;
                if (Sched.Evaluate(var, out nkp, out nki, out nkd) == Status.Ok)
                {
                    if (nkp != Kp || nki != Ki || nkd != Kd)
                    {
                        Kp = nkp; Ki = nki; Kd = nkd;
                        Recompute(dt);
                    }
                }
            }

            // -------- Stage 6: error and P -----------------------------
            double dsign = DirSign;
            double e = dsign * (sp - y);
            double pTerm = Kp * dsign * ((Beta * sp) - y);

            // -------- Stage 7: derivative ------------------------------
            // All three modes are the same expression with a different
            // setpoint weight, so there is one code path instead of three:
            //   x = dir*(y - gamma_eff*r), D = -Kd/(Tf+dt)*dx, filtered.
            double gammaEff;
            switch (DMode)
            {
                case DerivativeMode.OnError:         gammaEff = 1.0; break;
                case DerivativeMode.OnWeightedError: gammaEff = Gamma; break;
                default:                             gammaEff = 0.0; break;
            }
            double dSrc = dsign * (y - (gammaEff * sp));

            if ((Features & Consts.FeatDerivative) != 0)
            {
                DState = (CDa * DState) - (CDb * (dSrc - DPrevIn));
            }
            else
            {
                DState = 0.0;
            }
            DPrevIn = dSrc;

            // -------- Stage 8: feedforward -----------------------------
            if ((Features & Consts.FeatFeedforward) != 0)
            {
                if (inp != null && M.IsFinite(inp.Feedforward))
                    ff = inp.Feedforward * FfGain;
                else if (FfFn != null)
                    ff = FfFn(sp, y) * FfGain;
                else
                    ff = FfValue * FfGain;

                if (!M.IsFinite(ff))
                {
                    // A misbehaving user callback must not poison the loop.
                    ff = 0.0;
                    SetError(Status.ErrNanInput);
                }
            }

            double iLo = IMin, iHi = IMax;

            // Deferred from stage 2: now that pTerm, DState and ff exist for
            // this sample, the integrator can be solved so the sum reproduces
            // the fail-safe output exactly.
            if (recovering) BackSolve(recoverTo, pTerm, DState, ff);

            // -------- Stage 9: manual / hold ---------------------------
            if (ModeValue == Mode.Manual)
            {
                double um = ManualOutputValue;
                if ((Features & Consts.FeatOutputLimit) != 0)
                    um = M.Clamp(um, OutMin, OutMax);
                // Track continuously so a switch to Automatic at any instant
                // is bumpless without a special case in SetMode().
                BackSolve(um, pTerm, DState, ff);
                OutputValue = um;
                Flags |= Consts.FlagManual;
                EPrev = e;
                FillStatus(meas, y, sp, e, pTerm, DState, ff, OutputValue, dt);
                return um;
            }

            // -------- Stage 10: integral -------------------------------
            bool integrate = ((Features & Consts.FeatIntegral) != 0)
                             && (ModeValue != Mode.Hold);

            if (integrate)
            {
                double ae = M.Abs(e);
                if (ISeparation > 0.0 && ae > ISeparation)
                {
                    // Integral separation: during a large excursion the
                    // integrator would charge far beyond what the steady state
                    // needs, guaranteeing overshoot. P and D handle the
                    // transient; I re-engages near target.
                    integrate = false;
                }
                else if (IDeadband > 0.0 && ae < IDeadband)
                {
                    // Deadband: stop hunting against a quantised actuator.
                    integrate = false;
                }
                // Conditional integration is NOT decided here: admissibility
                // depends on whether the output saturates, known only after
                // the sum in stage 11. The decision is made, and undone if
                // necessary, in stage 13.
            }

            if (integrate)
            {
                iPre = Integrator;
                if (IntegMethod == IntegrationMethod.Trapezoidal)
                    Integrator += CI * (e + EPrev);
                else
                    Integrator += CI * e;
                iStepped = true;
                Flags |= Consts.FlagIntegralActive;
            }

            if (AwMode == AntiWindup.Clamp)
            {
                double clamped = M.Clamp(Integrator, iLo, iHi);
                if (clamped != Integrator)
                {
                    Integrator = clamped;
                    Flags |= Consts.FlagIntegralLimited;
                }
            }

            EPrev = e;

            // -------- Stage 11: sum ------------------------------------
            double uRaw = pTerm + Integrator + DState + ff;

            // -------- Stage 12: output saturation ----------------------
            double u = uRaw;
            if ((Features & Consts.FeatOutputLimit) != 0)
            {
                if (u > OutMax)
                {
                    u = OutMax;
                    Flags |= Consts.FlagSaturatedHigh;
                }
                else if (u < OutMin)
                {
                    u = OutMin;
                    Flags |= Consts.FlagSaturatedLow;
                }
            }

            // -------- Stage 13: back-calculation / tracking ------------
            // Applied in the SAME sample as the saturation it corrects.
            if (ModeValue != Mode.Hold)
            {
                if (AwMode == AntiWindup.BackCalculation)
                {
                    if (u != uRaw)
                    {
                        Integrator += CAw * (u - uRaw);
                        Integrator = M.Clamp(Integrator, iLo, iHi);
                    }
                }
                else if (AwMode == AntiWindup.Conditional)
                {
                    // Conditional integration in the Astrom sense: an
                    // increment is admissible unless the output saturates AND
                    // the error would drive it further past the same limit.
                    // The test uses uRaw - the unsaturated sum - because that
                    // says how far past the limit the controller is asking to
                    // go. The increment is UNDONE rather than merely skipped,
                    // so the decision uses this sample's saturation state; a
                    // one-cycle-late test is the classic way this strategy
                    // quietly degrades into no protection at all.
                    if (iStepped && ((uRaw > OutMax && e > 0.0)
                                     || (uRaw < OutMin && e < 0.0)))
                    {
                        Integrator = iPre;
                        Flags &= unchecked((ushort)~Consts.FlagIntegralActive);
                        Flags |= Consts.FlagIntegralLimited;

                        // Recompute: removing the increment may pull the
                        // output back inside the limits, and holding it at the
                        // limit anyway would throw away authority the
                        // controller actually has.
                        uRaw = pTerm + Integrator + DState + ff;
                        u = uRaw;
                        Flags &= unchecked((ushort)~Consts.FlagSaturated);
                        if ((Features & Consts.FeatOutputLimit) != 0)
                        {
                            if (u > OutMax)
                            {
                                u = OutMax;
                                Flags |= Consts.FlagSaturatedHigh;
                            }
                            else if (u < OutMin)
                            {
                                u = OutMin;
                                Flags |= Consts.FlagSaturatedLow;
                            }
                        }
                    }
                }
                else if (AwMode == AntiWindup.Tracking)
                {
                    double track = TrackingInput;
                    if (inp != null && M.IsFinite(inp.Tracking))
                        track = inp.Tracking;
                    if (M.IsFinite(track))
                    {
                        Integrator += CAw * (track - uRaw);
                        Integrator = M.Clamp(Integrator, iLo, iHi);
                    }
                }
            }

            // -------- Stage 14: output slew ----------------------------
            if ((Features & Consts.FeatOutShaper) != 0 && OutSlewMax > 0.0)
            {
                double maxStep = OutSlewMax * dt;
                double delta = u - OutputValue;
                if (delta > maxStep)
                {
                    u = OutputValue + maxStep;
                    Flags |= Consts.FlagOutputSlewing;
                }
                else if (delta < -maxStep)
                {
                    u = OutputValue - maxStep;
                    Flags |= Consts.FlagOutputSlewing;
                }
            }

            // Final numeric guard: fall back to the last good output rather
            // than propagating NaN into an actuator.
            if (!M.IsFinite(u))
            {
                SetError(Status.ErrNanInput);
                Integrator = 0.0;
                DState = 0.0;
                u = M.IsFinite(OutputValue) ? OutputValue : 0.0;
            }

            OutputValue = u;
            FillStatus(meas, y, sp, e, pTerm, DState, ff, uRaw, dt);
            return OutputValue;
        }

        private void FillStatus(double meas, double y, double sp, double e,
                                double pTerm, double dTerm, double ff,
                                double unsat, double dt)
        {
            StatusSnapshot s = Snapshot;
            s.SetpointRaw = SetpointTarget;
            s.SetpointShaped = sp;
            s.MeasurementRaw = meas;
            s.MeasurementFiltered = y;
            s.Error = e;
            s.PTerm = pTerm;
            s.ITerm = Integrator;
            s.DTerm = dTerm;
            s.FfTerm = ff;
            s.OutputUnsat = (ModeValue == Mode.Manual) ? OutputValue : unsat;
            s.Output = OutputValue;
            s.DtUsed = dt;
            s.KpActive = Kp;
            s.KiActive = Ki;
            s.KdActive = Kd;
            s.UpdateCount++;
            if ((Flags & Consts.FlagSaturated) != 0) s.SaturationCount++;
            s.Flags = Flags;
            s.LastError = LastError;
        }

        // ---------------------------------------------------------------
        // Level 1 - basic API
        // ---------------------------------------------------------------

        /// <summary>One cycle at the nominal sample time. The five-line entry.</summary>
        public double Update(double measurement)
        {
            Status rc;
            return Run(measurement, DtNominal, null, out rc);
        }

        /// <summary>One cycle with a measured dt. Use when the loop jitters.</summary>
        public double UpdateDt(double measurement, double dt)
        {
            Status rc;
            return Run(measurement, dt, null, out rc);
        }

        /// <summary>Full-control update. NaN fields mean "keep current state".</summary>
        public double UpdateEx(Input inp, out Status rc)
        {
            if (inp == null)
            {
                rc = Status.ErrNull;
                return 0.0;
            }
            double dt = (M.IsFinite(inp.Dt) && inp.Dt > 0.0) ? inp.Dt : DtNominal;
            return Run(inp.Measurement, dt, inp, out rc);
        }

        /// <summary>
        /// Minimal-overhead update: P with beta, backward-Euler I, filtered D
        /// on measurement, sum, clamp, integrator clamp. It deliberately
        /// IGNORES the shaper, safety, gain scheduling, feedforward, input
        /// filter, diagnostics and mode handling - and does not test for them.
        /// If any ignored feature is enabled the result silently differs from
        /// Update(); UpdateFastIsSafe() lets you assert against that.
        /// </summary>
        public double UpdateFast(double measurement)
        {
            if (!Initialised) return 0.0;

            double dsign = DirSign;
            double e = dsign * (SetpointEff - measurement);
            double p = Kp * dsign * ((Beta * SetpointEff) - measurement);

            double x = dsign * measurement;
            DState = (CDa * DState) - (CDb * (x - DPrevIn));
            DPrevIn = x;

            Integrator += CI * e;
            Integrator = M.Clamp(Integrator, IMin, IMax);

            double u = M.Clamp(p + Integrator + DState, OutMin, OutMax);
            OutputValue = u;
            return u;
        }

        /// <summary>
        /// True when UpdateFast() would produce the same output as Update().
        /// Output limits must be in force because the fast path clamps
        /// unconditionally; explicit IntegralLimit is not required since
        /// IMin/IMax always hold the effective bounds. Diagnostics is excluded
        /// because the fast path simply does not fill the snapshot - it
        /// produces the same OUTPUT, which is what "safe" means here.
        /// </summary>
        public bool UpdateFastIsSafe()
        {
            if (!Initialised) return false;
            uint adv = Consts.FeatAdvancedMask & ~Consts.FeatDiagnostics;
            return (Features & adv) == 0
                   && (Features & Consts.FeatOutputLimit) != 0
                   && (Features & Consts.FeatIntegral) != 0
                   && AwMode == AntiWindup.Clamp
                   && IntegMethod == IntegrationMethod.BackwardEuler
                   && DMode == DerivativeMode.OnMeasurement
                   && ModeValue == Mode.Automatic
                   && ISeparation <= 0.0
                   && IDeadband <= 0.0;
        }

        /// <summary>Clear all dynamic state, keeping the configuration.</summary>
        public Status Reset()
        {
            if (!Initialised) return Status.ErrNotInit;
            Integrator = 0.0;
            DState = 0.0;
            DPrevIn = 0.0;
            EPrev = 0.0;
            OutputValue = 0.0;
            Flags = 0;
            LastError = Status.Ok;
            SpVelocity = 0.0;
            SetpointEff = SetpointTarget;
            LpfState = 0.0;
            LpfPrimed = false;
            FaultCount = 0;
            MeasPrev = 0.0;
            MeasPrevValid = false;
            Snapshot = new StatusSnapshot();
            return Status.Ok;
        }

        /// <summary>Change gains without touching the integral TERM - bumpless.</summary>
        public Status SetGains(double kp, double ki, double kd)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (!GainOk(kp) || !GainOk(ki) || !GainOk(kd))
                return Status.ErrInvalidGain;
            Kp = kp; Ki = ki; Kd = kd;
            Recompute(DtLast);
            return Status.Ok;
        }

        /// <summary>
        /// Change gains preserving integral(e) rather than the term:
        /// term_new = term_old * (Ki_new / Ki_old). The classic semantics; it
        /// DOES step the output when Ki changes.
        /// </summary>
        public Status SetGainsRescaleIntegral(double kp, double ki, double kd)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (!GainOk(kp) || !GainOk(ki) || !GainOk(kd))
                return Status.ErrInvalidGain;
            double oldKi = Ki;
            if (oldKi > 0.0) Integrator = Integrator * (ki / oldKi);
            Kp = kp; Ki = ki; Kd = kd;
            Recompute(DtLast);
            return Status.Ok;
        }

        public Status SetKp(double kp)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (!GainOk(kp)) return Status.ErrInvalidGain;
            Kp = kp;
            Recompute(DtLast);
            return Status.Ok;
        }

        public Status SetKi(double ki)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (!GainOk(ki)) return Status.ErrInvalidGain;
            Ki = ki;
            Recompute(DtLast);
            return Status.Ok;
        }

        public Status SetKd(double kd)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (!GainOk(kd)) return Status.ErrInvalidGain;
            Kd = kd;
            Recompute(DtLast);
            return Status.Ok;
        }

        /// <summary>Values are 0 on failure, never junk.</summary>
        public Status GetGains(out double kp, out double ki, out double kd)
        {
            kp = 0.0; ki = 0.0; kd = 0.0;
            if (!Initialised) return Status.ErrNotInit;
            kp = Kp; ki = Ki; kd = Kd;
            return Status.Ok;
        }

        /// <summary>Command a setpoint. Goes through the shaper when enabled.</summary>
        public Status SetSetpoint(double setpoint)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (!M.IsFinite(setpoint)) return Status.ErrInvalidParam;
            SetpointTarget = setpoint;
            if ((Features & Consts.FeatSpShaper) == 0) SetpointEff = setpoint;
            return Status.Ok;
        }

        /// <summary>Bypass the shaper: target and effective setpoint both jump.</summary>
        public void SetSetpointImmediate(double sp)
        {
            SetpointTarget = sp;
            SetpointEff = sp;
        }

        public double GetSetpoint() { return SetpointEff; }
        public double GetOutput() { return OutputValue; }
        public double GetManualOutput() { return ManualOutputValue; }

        // ---------------------------------------------------------------
        // Level 2 - intermediate API
        // ---------------------------------------------------------------

        public Status SetSampleTime(double dt)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (!M.IsFinite(dt) || dt <= 0.0) return Status.ErrInvalidDt;
            DtNominal = dt;
            Recompute(dt);
            return Status.Ok;
        }

        /// <summary>
        /// Nominal sample time, or 0 when unusable. Validated rather than
        /// merely present: this getter is the only evidence a cascade has that
        /// a member loop was ever initialised.
        /// </summary>
        public double GetSampleTime()
        {
            return Initialised ? DtNominal : 0.0;
        }

        public Status SetOutputLimits(double lo, double hi)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (!M.IsFinite(lo) || !M.IsFinite(hi) || lo >= hi)
                return Status.ErrInvalidLimit;
            OutMin = lo;
            OutMax = hi;
            Features |= Consts.FeatOutputLimit;
            // Keep existing state consistent with the new envelope.
            OutputValue = M.Clamp(OutputValue, lo, hi);
            if ((Features & Consts.FeatIntegralLimit) == 0)
            {
                IMin = lo;
                IMax = hi;
                Integrator = M.Clamp(Integrator, lo, hi);
            }
            return Status.Ok;
        }

        public Status ClearOutputLimits()
        {
            if (!Initialised) return Status.ErrNotInit;
            Features &= ~Consts.FeatOutputLimit;
            OutMin = -Consts.HugeF;
            OutMax = Consts.HugeF;
            // An inherited integral bound has nothing left to inherit from.
            if ((Features & Consts.FeatIntegralLimit) == 0)
            {
                IMin = -Consts.HugeF;
                IMax = Consts.HugeF;
            }
            return Status.Ok;
        }

        public Status SetIntegralLimits(double lo, double hi)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (!M.IsFinite(lo) || !M.IsFinite(hi) || lo >= hi)
                return Status.ErrInvalidLimit;
            IMin = lo;
            IMax = hi;
            Features |= Consts.FeatIntegralLimit;
            Integrator = M.Clamp(Integrator, lo, hi);
            return Status.Ok;
        }

        public Status SetAntiWindup(AntiWindup mode, double kt = 0.0)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (mode > AntiWindup.Tracking) return Status.ErrInvalidParam;
            if (!M.IsFinite(kt) || kt < 0.0) return Status.ErrInvalidParam;
            if (mode == AntiWindup.BackCalculation &&
                (Features & (Consts.FeatOutputLimit | Consts.FeatIntegralLimit)) == 0)
                return Status.ErrInvalidLimit;
            AwMode = mode;
            Kt = kt;
            Recompute(DtLast);
            return Status.Ok;
        }

        public Status SetDerivativeMode(DerivativeMode mode)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (mode > DerivativeMode.OnWeightedError)
                return Status.ErrInvalidParam;
            DMode = mode;
            // The derivative source changes meaning; re-prime on the next
            // sample to avoid differentiating across the discontinuity.
            DPrevIn = 0.0;
            DState = 0.0;
            return Status.Ok;
        }

        public Status SetDerivativeFilter(double tf)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (!M.IsFinite(tf) || tf < 0.0) return Status.ErrInvalidParam;
            Tf = tf;
            if (tf > 0.0) Features |= Consts.FeatDFilter;
            Recompute(DtLast);
            return Status.Ok;
        }

        public Status SetDerivativeFilterN(double n)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (!M.IsFinite(n) || n <= 0.0) return Status.ErrInvalidParam;
            NFilter = n;
            Tf = 0.0;                 // explicit tf no longer overrides N
            Features |= Consts.FeatDFilter;
            Recompute(DtLast);
            return Status.Ok;
        }

        public Status SetDirection(Direction dir)
        {
            if (!Initialised) return Status.ErrNotInit;
            DirSign = (dir == Direction.Reverse) ? -1 : 1;
            DPrevIn = -DPrevIn;       // keep the stored source consistent
            return Status.Ok;
        }

        public Status SetMode(Mode mode)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (mode > Mode.Hold) return Status.ErrInvalidMode;
            if (ModeValue != Mode.Manual && mode == Mode.Manual)
            {
                // Entering manual: start from where the controller already is.
                ManualOutputValue = OutputValue;
            }
            // Leaving manual needs no work: Run() back-solves the integrator
            // on every manual sample.
            ModeValue = mode;
            return Status.Ok;
        }

        public Mode GetMode() { return ModeValue; }

        public Status SetManualOutput(double output)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (!M.IsFinite(output)) return Status.ErrInvalidParam;
            ManualOutputValue = output;
            return Status.Ok;
        }

        public Status SetSetpointRamp(double rateMax, double accel = 0.0,
                                      double decel = 0.0)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (!M.IsFinite(rateMax) || rateMax < 0.0
                || !M.IsFinite(accel) || accel < 0.0
                || !M.IsFinite(decel) || decel < 0.0)
                return Status.ErrInvalidParam;
            SpRateMax = rateMax;
            SpAccel = accel;
            SpDecel = decel;
            if (rateMax > 0.0)
            {
                Features |= Consts.FeatSpShaper;
            }
            else
            {
                Features &= ~Consts.FeatSpShaper;
                SpVelocity = 0.0;
            }
            return Status.Ok;
        }

        public Status SetOutputSlewRate(double slewMax)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (!M.IsFinite(slewMax) || slewMax < 0.0)
                return Status.ErrInvalidParam;
            OutSlewMax = slewMax;
            if (slewMax > 0.0) Features |= Consts.FeatOutShaper;
            else Features &= ~Consts.FeatOutShaper;
            return Status.Ok;
        }

        public Status SetInputFilter(double tau)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (!M.IsFinite(tau) || tau < 0.0) return Status.ErrInvalidParam;
            LpfTau = tau;
            if (tau > 0.0)
            {
                Features |= Consts.FeatInputFilter;
            }
            else
            {
                Features &= ~Consts.FeatInputFilter;
                LpfPrimed = false;
            }
            Recompute(DtLast);
            return Status.Ok;
        }

        // ---------------------------------------------------------------
        // Level 3 - advanced API
        // ---------------------------------------------------------------

        public Status SetWeights(double beta, double gamma)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (!M.IsFinite(beta) || !M.IsFinite(gamma)
                || beta < 0.0 || beta > 2.0 || gamma < 0.0 || gamma > 2.0)
                return Status.ErrInvalidParam;
            Beta = beta;
            Gamma = gamma;
            return Status.Ok;
        }

        public Status SetFeedforward(double ff)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (!M.IsFinite(ff)) return Status.ErrInvalidParam;
            FfValue = ff;
            Features |= Consts.FeatFeedforward;
            return Status.Ok;
        }

        /// <summary>Install u_ff = gain * fn(setpoint, measurement).</summary>
        public Status SetFeedforwardFn(Func<double, double, double> fn,
                                       double gain = 1.0)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (!M.IsFinite(gain)) return Status.ErrInvalidParam;
            FfFn = fn;
            FfGain = (gain != 0.0) ? gain : 1.0;
            if (fn != null) Features |= Consts.FeatFeedforward;
            return Status.Ok;
        }

        public Status SetIntegralSeparation(double threshold)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (!M.IsFinite(threshold) || threshold < 0.0)
                return Status.ErrInvalidParam;
            ISeparation = threshold;
            return Status.Ok;
        }

        public Status SetIntegralDeadband(double db)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (!M.IsFinite(db) || db < 0.0) return Status.ErrInvalidParam;
            IDeadband = db;
            return Status.Ok;
        }

        public Status EnableIntegral(bool enable)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (enable) Features |= Consts.FeatIntegral;
            else Features &= ~Consts.FeatIntegral;
            return Status.Ok;
        }

        /// <summary>Preload the integral TERM, in output units.</summary>
        public Status SetIntegrator(double value)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (!M.IsFinite(value)) return Status.ErrInvalidParam;
            Integrator = M.Clamp(value, IMin, IMax);
            return Status.Ok;
        }

        public double GetIntegrator() { return Integrator; }

        public Status SetTrackingInput(double uTrack)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (!M.IsFinite(uTrack)) return Status.ErrInvalidParam;
            TrackingInput = uTrack;
            return Status.Ok;
        }

        public Status SetIntegrationMethod(IntegrationMethod method)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (method > IntegrationMethod.Trapezoidal)
                return Status.ErrInvalidParam;
            IntegMethod = method;
            Recompute(DtLast);
            return Status.Ok;
        }

        public Status SetFaultOutput(double output)
        {
            if (!Initialised) return Status.ErrNotInit;
            if (!M.IsFinite(output)) return Status.ErrInvalidParam;
            FailsafeOutput = output;
            return Status.Ok;
        }

        /// <summary>Drop the latched fault; the next healthy sample resumes.</summary>
        public Status ClearFault()
        {
            if (!Initialised) return Status.ErrNotInit;
            Flags &= unchecked((ushort)~(Consts.FlagFault | Consts.FlagSensorInvalid));
            FaultCount = 0;
            MeasPrevValid = false;
            // Re-seed the integrator so control resumes from the fail-safe
            // output rather than from whatever it held before the fault.
            BackSolve(OutputValue, 0.0, 0.0, 0.0);
            return Status.Ok;
        }

        public bool IsFaulted()
        {
            return (Flags & Consts.FlagFault) != 0;
        }

        public double GetError() { return Snapshot.Error; }

        /// <summary>Read AND clear the sticky error.</summary>
        public Status GetLastError()
        {
            Status code = LastError;
            LastError = Status.Ok;
            return code;
        }

        /// <summary>Read the sticky error without clearing it.</summary>
        public Status PeekLastError() { return LastError; }

        public Status ClearError()
        {
            LastError = Status.Ok;
            return Status.Ok;
        }

        public StatusSnapshot GetStatus()
        {
            return Initialised ? Snapshot : null;
        }

        public ushort GetFlags() { return Flags; }
        public uint GetFeatures() { return Features; }

        public static string GetVersion() { return Consts.VersionString; }
    }
}

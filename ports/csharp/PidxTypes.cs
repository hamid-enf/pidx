// PidxTypes.cs - enumerations, flags, status codes and configuration records.
//
// Mirrors include/pidx/pid_status.h and include/pidx/pid_types.h. The nested
// config field paths are deliberately identical to the C ones
// (cfg.Core.Kp, cfg.Limits.OutputMin, cfg.Integral.Mode, ...) so that code and
// documentation translate between the two without a lookup table. Only the
// casing follows .NET convention.

using System;

namespace Pidx
{
    /// <summary>Result of an API call, and the type of the sticky error.</summary>
    public enum Status
    {
        Ok = 0,
        ErrNull = 1,
        ErrNotInit = 2,
        ErrInvalidConfig = 3,
        ErrInvalidGain = 4,
        ErrInvalidLimit = 5,
        ErrInvalidDt = 6,
        ErrInvalidMode = 7,
        ErrInvalidParam = 8,
        ErrNanInput = 9,
        ErrInfInput = 10,
        ErrSensorRange = 11,
        ErrSensorRate = 12,
        ErrUnsupported = 13,
        ErrBusy = 14,
        ErrTuneTimeout = 15,
        ErrTuneUnstable = 16,
        ErrTuneNoOscillation = 17,
        ErrTuneModelMismatch = 18,
        ErrTuneAborted = 19,
        ErrTuneValidation = 20
    }

    public enum Direction { Direct = 0, Reverse = 1 }

    public enum Mode { Manual = 0, Automatic = 1, Hold = 2 }

    public enum AntiWindup
    {
        None = 0,
        Clamp = 1,
        Conditional = 2,
        BackCalculation = 3,
        Tracking = 4
    }

    public enum DerivativeMode
    {
        OnMeasurement = 0,
        OnError = 1,
        OnWeightedError = 2
    }

    public enum IntegrationMethod { BackwardEuler = 0, Trapezoidal = 1 }

    public enum SchedSource
    {
        Setpoint = 0,
        Measurement = 1,
        Error = 2,
        AbsError = 3,
        Output = 4,
        External = 5
    }

    public enum SchedInterp { Linear = 0, Smooth = 1, Hold = 2 }

    /// <summary>Library-wide constants and the runtime feature/flag bits.</summary>
    public static class Consts
    {
        public const string VersionString = "1.0.0";
        public const int VersionNum = 10000;   // MAJOR*10000 + MINOR*100 + PATCH
        public const int ConfigAbiVersion = 1;

        public const double DefaultSampleTime = 0.01;
        public const double DefaultNFilter = 10.0;

        /// <summary>Stand-in for "no limit".</summary>
        public const double HugeF = 1.0e30;

        public const int GainSchedMaxPoints = 16;
        public const int CascadeMaxLoops = 4;

        // -- runtime feature mask ---------------------------------------
        public const uint FeatIntegral = 1u << 0;
        public const uint FeatDerivative = 1u << 1;
        public const uint FeatDFilter = 1u << 2;
        public const uint FeatOutputLimit = 1u << 3;
        public const uint FeatIntegralLimit = 1u << 4;
        public const uint FeatFeedforward = 1u << 5;
        public const uint FeatSpShaper = 1u << 6;
        public const uint FeatOutShaper = 1u << 7;
        public const uint FeatInputFilter = 1u << 8;
        public const uint FeatSafety = 1u << 9;
        public const uint FeatGainSched = 1u << 10;
        public const uint FeatDiagnostics = 1u << 11;
        public const uint FeatTelemetry = 1u << 12;

        /// <summary>Everything the fast path does not implement.</summary>
        public const uint FeatAdvancedMask =
            FeatFeedforward | FeatSpShaper | FeatOutShaper | FeatInputFilter |
            FeatSafety | FeatGainSched | FeatDiagnostics | FeatTelemetry;

        // -- per-cycle status flags -------------------------------------
        public const ushort FlagSaturatedHigh = 1 << 0;
        public const ushort FlagSaturatedLow = 1 << 1;
        public const ushort FlagIntegralActive = 1 << 2;
        public const ushort FlagIntegralLimited = 1 << 3;
        public const ushort FlagFault = 1 << 4;
        public const ushort FlagManual = 1 << 5;
        public const ushort FlagTuning = 1 << 6;
        public const ushort FlagDtViolation = 1 << 7;
        public const ushort FlagSensorInvalid = 1 << 8;
        public const ushort FlagSpRamping = 1 << 9;
        public const ushort FlagOutputSlewing = 1 << 10;

        public const ushort FlagSaturated = FlagSaturatedHigh | FlagSaturatedLow;

        private static readonly string[] StatusText =
        {
            "OK", "null pointer", "not initialised", "invalid config",
            "invalid gain", "invalid limit", "invalid dt", "invalid mode",
            "invalid parameter", "NaN input", "Inf input",
            "sensor out of range", "sensor rate exceeded", "unsupported",
            "busy", "tune timeout", "tune unstable", "tune: no oscillation",
            "tune: model mismatch", "tune aborted", "tune: validation failed"
        };

        /// <summary>Human-readable name of a status code. Never throws.</summary>
        public static string StatusToString(Status code)
        {
            int i = (int)code;
            return (i >= 0 && i < StatusText.Length) ? StatusText[i] : "?";
        }
    }

    // -------------------------------------------------------------------
    // Configuration
    // -------------------------------------------------------------------

    public sealed class CoreConfig
    {
        public double Kp = 0.0;
        public double Ki = 0.0;
        public double Kd = 0.0;
        public double SampleTime = Consts.DefaultSampleTime;
        public Direction Direction = Direction.Direct;
        public Mode Mode = Mode.Automatic;
        public IntegrationMethod Integration = IntegrationMethod.BackwardEuler;
    }

    public sealed class LimitsConfig
    {
        public bool UseOutputLimits = false;
        public double OutputMin = -Consts.HugeF;
        public double OutputMax = Consts.HugeF;
        public bool UseIntegralLimits = false;
        public double IntegralMin = -Consts.HugeF;
        public double IntegralMax = Consts.HugeF;

        /// <summary>0 disables. Outside the band dt is clamped, not rejected.</summary>
        public double DtMin = 0.0;
        public double DtMax = 0.0;
    }

    public sealed class FilterConfig
    {
        public DerivativeMode DerivativeMode = DerivativeMode.OnMeasurement;

        /// <summary>Explicit derivative filter time constant; wins over NFilter.</summary>
        public double Tf = 0.0;
        public double NFilter = Consts.DefaultNFilter;
        public double InputLpfTau = 0.0;
    }

    public sealed class IntegralConfig
    {
        /// <summary>Anti-windup strategy. Named Mode to match the C field.</summary>
        public AntiWindup Mode = AntiWindup.Clamp;
        public double Kt = 0.0;
        public double SeparationThreshold = 0.0;
        public double Deadband = 0.0;
        public bool Enabled = true;
    }

    /// <summary>Setpoint weighting; both in [0, 2].</summary>
    public sealed class WeightConfig
    {
        public double Beta = 1.0;
        public double Gamma = 0.0;
    }

    public sealed class FeedforwardConfig
    {
        public bool Enabled = false;
        public Func<double, double, double> Fn = null;
        public double Value = 0.0;
        public double Gain = 1.0;
    }

    public sealed class ShaperConfig
    {
        public double SpRateMax = 0.0;
        public double SpAccel = 0.0;
        public double SpDecel = 0.0;
        public double OutSlewMax = 0.0;
    }

    public sealed class SafetyConfig
    {
        public bool Enabled = false;
        public double MeasMin = 0.0;
        public double MeasMax = 0.0;
        public double MeasRateMax = 0.0;
        public double FailsafeOutput = 0.0;
        public byte FaultPersistN = 3;
        public bool AutoRecover = false;
    }

    /// <summary>Full controller configuration; the analogue of PID_Config.</summary>
    public sealed class Config
    {
        public CoreConfig Core = new CoreConfig();
        public LimitsConfig Limits = new LimitsConfig();
        public FilterConfig Filter = new FilterConfig();
        public IntegralConfig Integral = new IntegralConfig();
        public WeightConfig Weight = new WeightConfig();
        public FeedforwardConfig Feedforward = new FeedforwardConfig();
        public ShaperConfig Shaper = new ShaperConfig();
        public SafetyConfig Safety = new SafetyConfig();
        public int AbiVersion = Consts.ConfigAbiVersion;

        /// <summary>Build a config for the common case in one call.</summary>
        public static Config Quick(double kp = 0.0, double ki = 0.0,
                                   double kd = 0.0, double dt = 0.01,
                                   double? outMin = null, double? outMax = null)
        {
            var c = new Config();
            c.Core.Kp = kp;
            c.Core.Ki = ki;
            c.Core.Kd = kd;
            c.Core.SampleTime = dt;
            if (outMin.HasValue && outMax.HasValue)
            {
                c.Limits.UseOutputLimits = true;
                c.Limits.OutputMin = outMin.Value;
                c.Limits.OutputMax = outMax.Value;
            }
            return c;
        }
    }

    /// <summary>Extended input bundle. NaN means "keep the current state".</summary>
    public sealed class Input
    {
        public double Measurement = double.NaN;
        public double Setpoint = double.NaN;
        public double Dt = double.NaN;
        public double Feedforward = double.NaN;
        public double Tracking = double.NaN;
        public double ScheduleVar = double.NaN;
    }

    /// <summary>Per-cycle diagnostic snapshot; the analogue of PID_Status.</summary>
    public sealed class StatusSnapshot
    {
        public double SetpointRaw;
        public double SetpointShaped;
        public double MeasurementRaw;
        public double MeasurementFiltered;
        public double Error;
        public double PTerm;
        public double ITerm;
        public double DTerm;
        public double FfTerm;
        public double OutputUnsat;
        public double Output;
        public double DtUsed;
        public double KpActive;
        public double KiActive;
        public double KdActive;
        public uint UpdateCount;
        public uint SaturationCount;
        public ushort Flags;
        public Status LastError;
    }
}

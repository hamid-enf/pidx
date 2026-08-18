// ConformCs.cs - conformance runner for the C# port.
//
// Reads the scenario file described in ports/SPEC_conformance.md and writes
// the CSV compared against the C reference. Mirrors the dispatch in
// ports/c_ref/conform_c.c exactly.
//
// Build:  mcs -out:conform_cs.exe *.cs
// Run:    mono conform_cs.exe ../compare/scenarios.txt

using System;
using System.Globalization;
using System.IO;
using Pidx;

namespace PidxConform
{
    internal static class Program
    {
        private static Config _cfg = new Config();
        private static Pid _pid = new Pid();
        private static GainPoint[] _schedPoints = null;
        private static string _scenario = "none";
        private static int _row;
        private static bool _inited;

        // Invariant culture everywhere: a machine with a comma decimal
        // separator would otherwise emit "2,4" and silently break the CSV.
        private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

        private static double Num(string tok)
        {
            if (tok == "nan") return double.NaN;
            if (tok == "inf") return double.PositiveInfinity;
            if (tok == "-inf") return double.NegativeInfinity;
            return double.Parse(tok, Inv);
        }

        private static double F(string[] t, int i)
        {
            return (i < t.Length) ? Num(t[i]) : 0.0;
        }

        private static int N(string[] t, int i)
        {
            return (i < t.Length) ? (int)Math.Round(double.Parse(t[i], Inv)) : 0;
        }

        /// <summary>
        /// Render with full round-trip precision. "G17" is the .NET spelling
        /// of %.17g; the comparison script parses both back to binary64, so
        /// what matters is only that no precision is lost on the way out.
        /// </summary>
        private static string Fmt(double v)
        {
            if (double.IsNaN(v)) return "nan";
            if (double.IsPositiveInfinity(v)) return "inf";
            if (double.IsNegativeInfinity(v)) return "-inf";
            return v.ToString("G17", Inv);
        }

        private static void Emit(string cmd, int rc, double outv, double sp,
                                 double err, double p, double i, double d,
                                 double ff, double unsat, int flags, int last)
        {
            Console.WriteLine(
                "{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13}",
                _scenario, _row, cmd, rc, Fmt(outv), Fmt(sp), Fmt(err),
                Fmt(p), Fmt(i), Fmt(d), Fmt(ff), Fmt(unsat), flags, last);
            _row++;
        }

        private static void EmitUpdate(string cmd, Status rc, double outv)
        {
            StatusSnapshot s = _pid.GetStatus();
            if (s == null)
            {
                Emit(cmd, (int)rc, outv, double.NaN, double.NaN, double.NaN,
                     double.NaN, double.NaN, double.NaN, double.NaN,
                     0, (int)_pid.PeekLastError());
                return;
            }
            Emit(cmd, (int)rc, outv, s.SetpointShaped, s.Error, s.PTerm,
                 s.ITerm, s.DTerm, s.FfTerm, s.OutputUnsat,
                 s.Flags, (int)_pid.PeekLastError());
        }

        // -- configuration commands --------------------------------------

        private static void DoConfig(string[] t)
        {
            switch (t[0])
            {
                case "gains":
                    _cfg.Core.Kp = F(t, 1);
                    _cfg.Core.Ki = F(t, 2);
                    _cfg.Core.Kd = F(t, 3);
                    break;
                case "dt": _cfg.Core.SampleTime = F(t, 1); break;
                case "direction": _cfg.Core.Direction = (Direction)N(t, 1); break;
                case "mode": _cfg.Core.Mode = (Mode)N(t, 1); break;
                case "integration":
                    _cfg.Core.Integration = (IntegrationMethod)N(t, 1);
                    break;
                case "outlim":
                    _cfg.Limits.UseOutputLimits = true;
                    _cfg.Limits.OutputMin = F(t, 1);
                    _cfg.Limits.OutputMax = F(t, 2);
                    break;
                case "intlim":
                    _cfg.Limits.UseIntegralLimits = true;
                    _cfg.Limits.IntegralMin = F(t, 1);
                    _cfg.Limits.IntegralMax = F(t, 2);
                    break;
                case "dtlim":
                    _cfg.Limits.DtMin = F(t, 1);
                    _cfg.Limits.DtMax = F(t, 2);
                    break;
                case "aw":
                    _cfg.Integral.Mode = (AntiWindup)N(t, 1);
                    _cfg.Integral.Kt = F(t, 2);
                    break;
                case "separation":
                    _cfg.Integral.SeparationThreshold = F(t, 1);
                    break;
                case "deadband": _cfg.Integral.Deadband = F(t, 1); break;
                case "ienable": _cfg.Integral.Enabled = (N(t, 1) != 0); break;
                case "dmode":
                    _cfg.Filter.DerivativeMode = (DerivativeMode)N(t, 1);
                    break;
                case "tf": _cfg.Filter.Tf = F(t, 1); break;
                case "nfilter": _cfg.Filter.NFilter = F(t, 1); break;
                case "inlpf": _cfg.Filter.InputLpfTau = F(t, 1); break;
                case "weights":
                    _cfg.Weight.Beta = F(t, 1);
                    _cfg.Weight.Gamma = F(t, 2);
                    break;
                case "ff":
                    _cfg.Feedforward.Enabled = (N(t, 1) != 0);
                    _cfg.Feedforward.Value = F(t, 2);
                    _cfg.Feedforward.Gain = F(t, 3);
                    break;
                case "shaper":
                    _cfg.Shaper.SpRateMax = F(t, 1);
                    _cfg.Shaper.SpAccel = F(t, 2);
                    _cfg.Shaper.SpDecel = F(t, 3);
                    _cfg.Shaper.OutSlewMax = F(t, 4);
                    break;
                case "safety":
                    _cfg.Safety.Enabled = (N(t, 1) != 0);
                    _cfg.Safety.MeasMin = F(t, 2);
                    _cfg.Safety.MeasMax = F(t, 3);
                    _cfg.Safety.MeasRateMax = F(t, 4);
                    _cfg.Safety.FailsafeOutput = F(t, 5);
                    _cfg.Safety.FaultPersistN = (byte)N(t, 6);
                    _cfg.Safety.AutoRecover = (N(t, 7) != 0);
                    break;
                default:
                    Console.Error.WriteLine("unknown config cmd: " + t[0]);
                    Environment.Exit(2);
                    break;
            }
        }

        // -- runtime commands --------------------------------------------

        private static void DoRun(string[] t)
        {
            Pid p = _pid;
            switch (t[0])
            {
                case "u":
                    {
                        double o = p.UpdateDt(F(t, 1), F(t, 2));
                        EmitUpdate("u", p.PeekLastError(), o);
                        break;
                    }
                case "un":
                    {
                        double o = p.Update(F(t, 1));
                        EmitUpdate("un", p.PeekLastError(), o);
                        break;
                    }
                case "ufast":
                    {
                        double o = p.UpdateFast(F(t, 1));
                        Emit("ufast", (int)p.PeekLastError(), o,
                             p.GetSetpoint(), double.NaN, double.NaN,
                             p.GetIntegrator(), double.NaN, double.NaN,
                             double.NaN, 0, (int)p.PeekLastError());
                        break;
                    }
                case "uex":
                    {
                        Input inp = new Input();
                        inp.Measurement = F(t, 1);
                        inp.Dt = F(t, 2);
                        inp.Setpoint = F(t, 3);
                        inp.Feedforward = F(t, 4);
                        inp.Tracking = F(t, 5);
                        inp.ScheduleVar = F(t, 6);
                        Status rc;
                        double o = p.UpdateEx(inp, out rc);
                        EmitUpdate("uex", rc, o);
                        break;
                    }
                case "sp": p.SetSetpoint(F(t, 1)); break;
                case "spimm": p.SetSetpointImmediate(F(t, 1)); break;
                case "setmode": p.SetMode((Mode)N(t, 1)); break;
                case "manual": p.SetManualOutput(F(t, 1)); break;
                case "setgains": p.SetGains(F(t, 1), F(t, 2), F(t, 3)); break;
                case "rescale":
                    p.SetGainsRescaleIntegral(F(t, 1), F(t, 2), F(t, 3));
                    break;
                case "setaw":
                    p.SetAntiWindup((AntiWindup)N(t, 1), F(t, 2));
                    break;
                case "setoutlim": p.SetOutputLimits(F(t, 1), F(t, 2)); break;
                case "clroutlim": p.ClearOutputLimits(); break;
                case "setintlim": p.SetIntegralLimits(F(t, 1), F(t, 2)); break;
                case "setint": p.SetIntegrator(F(t, 1)); break;
                case "track": p.SetTrackingInput(F(t, 1)); break;
                case "setdmode":
                    p.SetDerivativeMode((DerivativeMode)N(t, 1));
                    break;
                case "settf": p.SetDerivativeFilter(F(t, 1)); break;
                case "setn": p.SetDerivativeFilterN(F(t, 1)); break;
                case "setdir": p.SetDirection((Direction)N(t, 1)); break;
                case "setweights": p.SetWeights(F(t, 1), F(t, 2)); break;
                case "setff": p.SetFeedforward(F(t, 1)); break;
                case "setramp":
                    p.SetSetpointRamp(F(t, 1), F(t, 2), F(t, 3));
                    break;
                case "setslew": p.SetOutputSlewRate(F(t, 1)); break;
                case "setinlpf": p.SetInputFilter(F(t, 1)); break;
                case "setsep": p.SetIntegralSeparation(F(t, 1)); break;
                case "setdb": p.SetIntegralDeadband(F(t, 1)); break;
                case "setienable": p.EnableIntegral(N(t, 1) != 0); break;
                case "setdtnom": p.SetSampleTime(F(t, 1)); break;
                case "reset": p.Reset(); break;
                case "clearfault": p.ClearFault(); break;
                case "schedpoints":
                    {
                        int cnt = N(t, 1);
                        _schedPoints = new GainPoint[cnt];
                        for (int i = 0; i < cnt; i++)
                        {
                            _schedPoints[i] = new GainPoint(
                                F(t, 2 + i * 4), F(t, 3 + i * 4),
                                F(t, 4 + i * 4), F(t, 5 + i * 4));
                        }
                        break;
                    }
                case "schedcfg":
                    if (_schedPoints != null)
                    {
                        GainSchedule sch = new GainSchedule();
                        sch.Init(_schedPoints, (SchedSource)N(t, 1),
                                 (SchedInterp)N(t, 2));
                        sch.SetHysteresis(F(t, 3));
                        GainSchedule.Attach(p, sch);
                    }
                    break;
                case "schedvar": GainSchedule.SetVar(p, F(t, 1)); break;
                case "rule":
                    {
                        ModelKind kind = (ModelKind)N(t, 3);
                        PlantModel mdl = (kind == ModelKind.Freq)
                            ? PlantModel.Freq(F(t, 4), F(t, 5))
                            : PlantModel.Fopdt(F(t, 4), F(t, 5), F(t, 6));
                        Gains g;
                        Status rc = TuningRules.Apply((TuneRule)N(t, 1), mdl,
                                                      (TuneStructure)N(t, 2),
                                                      F(t, 7), out g);
                        Emit("rule", (int)rc, g.Kp, g.Ki, g.Kd, g.Ti, g.Td,
                             g.Tf, double.NaN, double.NaN, 0, 0);
                        break;
                    }
                default:
                    Console.Error.WriteLine("unknown run cmd: " + t[0]);
                    Environment.Exit(2);
                    break;
            }
        }

        private static int Main(string[] args)
        {
            if (args.Length < 1)
            {
                Console.Error.WriteLine("usage: conform_cs <scenario-file>");
                return 1;
            }

            Console.WriteLine("scenario,k,cmd,rc,output,setpoint,error,p,i,d,"
                              + "ff,unsat,flags,last_error");

            foreach (string raw in File.ReadLines(args[0]))
            {
                string[] t = raw.Split(new[] { ' ', '\t', '\r', '\n' },
                                       StringSplitOptions.RemoveEmptyEntries);
                if (t.Length == 0 || t[0].StartsWith("#")) continue;

                if (t[0] == "scenario")
                {
                    _cfg = new Config();
                    _pid = new Pid();
                    _schedPoints = null;
                    _inited = false;
                    _row = 0;
                    _scenario = (t.Length > 1) ? t[1] : "?";
                }
                else if (t[0] == "init")
                {
                    Status rc = _pid.Init(_cfg);
                    _inited = (rc == Status.Ok);
                    Emit("init", (int)rc, 0.0, double.NaN, double.NaN,
                         double.NaN, double.NaN, double.NaN, double.NaN,
                         double.NaN, 0, 0);
                }
                else if (t[0] == "end")
                {
                    _inited = false;
                }
                else if (!_inited)
                {
                    DoConfig(t);
                }
                else
                {
                    DoRun(t);
                }
            }

            return 0;
        }
    }
}

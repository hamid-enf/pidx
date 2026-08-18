// TuningRules.cs - closed-form tuning rules. Mirrors pid_autotune_rules.c.
//
// Each rule is a pure function from an identified plant model to controller
// gains. No rule touches a controller, allocates, or blocks, so every rule is
// testable in isolation against published coefficients.
//
// Two families exist and they are NOT interchangeable:
//
//   FREQ rules consume (Ku, Pu) - one point on the Nyquist curve, where the
//   loop phase is -180 deg. Ziegler-Nichols and its descendants.
//
//   FOPDT rules consume (K, T, L) from G(s) = K*exp(-L*s)/(1+T*s).
//   Cohen-Coon, AMIGO-step and IMC are derived from that three-parameter model
//   and cannot be evaluated from (Ku, Pu): one complex number does not
//   determine three real parameters. Requesting such a pairing returns
//   ErrTuneModelMismatch rather than inventing a conversion.
//
// Output convention: the parallel form the core uses,
//   u = Kp*e + Ki*integral(e) + Kd*de/dt,  Ki = Kp/Ti,  Kd = Kp*Td.
// A rule that yields no integral action reports Ti = 0 and Ki = 0.

using System;

namespace Pidx
{
    public enum IdentMethod { Relay = 0, Step = 1 }

    public enum ModelKind { None = 0, Freq = 1, Fopdt = 2 }

    public enum TuneStructure { P = 0, Pi = 1, Pid = 2 }

    public enum TuneRule
    {
        Zn = 0,
        TyreusLuyben = 1,
        Pessen = 2,
        SomeOvershoot = 3,
        NoOvershoot = 4,
        AmigoFreq = 5,
        CohenCoon = 6,
        AmigoStep = 7,
        Imc = 8,
        Custom = 9
    }

    /// <summary>Identified plant: produced by identification, consumed by a rule.</summary>
    public sealed class PlantModel
    {
        public ModelKind Kind = ModelKind.None;

        public double Ku;    // ultimate gain
        public double Pu;    // ultimate period [s]

        public double K;     // FOPDT static gain
        public double T;     // FOPDT time constant [s]
        public double L;     // FOPDT dead time [s]

        public double NoiseSigma;
        public byte Quality;

        public static PlantModel Freq(double ku, double pu)
        {
            return new PlantModel { Kind = ModelKind.Freq, Ku = ku, Pu = pu };
        }

        public static PlantModel Fopdt(double k, double t, double l)
        {
            return new PlantModel { Kind = ModelKind.Fopdt, K = k, T = t, L = l };
        }
    }

    /// <summary>Parallel-form gains plus the derivative filter time constant.</summary>
    public sealed class Gains
    {
        public double Kp;
        public double Ki;
        public double Kd;
        public double Ti;
        public double Td;
        public double Tf;
    }

    public static class TuningRules
    {
        /// <summary>Which model kind a built-in rule requires.</summary>
        public static ModelKind RequiredModel(TuneRule rule)
        {
            switch (rule)
            {
                case TuneRule.Zn:
                case TuneRule.TyreusLuyben:
                case TuneRule.Pessen:
                case TuneRule.SomeOvershoot:
                case TuneRule.NoOvershoot:
                case TuneRule.AmigoFreq:
                    return ModelKind.Freq;
                case TuneRule.CohenCoon:
                case TuneRule.AmigoStep:
                case TuneRule.Imc:
                    return ModelKind.Fopdt;
                default:
                    return ModelKind.None;
            }
        }

        /// <summary>Human-readable rule name, never null.</summary>
        public static string Name(TuneRule rule)
        {
            switch (rule)
            {
                case TuneRule.Zn: return "Ziegler-Nichols";
                case TuneRule.TyreusLuyben: return "Tyreus-Luyben";
                case TuneRule.Pessen: return "Pessen-Integral";
                case TuneRule.SomeOvershoot: return "Some-Overshoot";
                case TuneRule.NoOvershoot: return "No-Overshoot";
                case TuneRule.AmigoFreq: return "AMIGO-freq";
                case TuneRule.CohenCoon: return "Cohen-Coon";
                case TuneRule.AmigoStep: return "AMIGO-step";
                case TuneRule.Imc: return "IMC-lambda";
                case TuneRule.Custom: return "Custom";
                default: return "?";
            }
        }

        /// <summary>Convert (Kp, Ti, Td) into the parallel form the core uses.</summary>
        private static void Finish(Gains g, double kp, double ti, double td)
        {
            g.Kp = kp;
            g.Ti = ti;
            g.Td = td;
            // Ti == 0 encodes "no integral action" - do not divide by it.
            g.Ki = (ti > 0.0) ? (kp / ti) : 0.0;
            g.Kd = kp * td;
            // Derivative filter from the standard N = 10 rule: Tf = Td/N.
            // Without a filter the derivative differentiates noise without
            // bound.
            g.Tf = td * 0.1;
        }

        /// <summary>
        /// Published coefficient triples (a, b, c): Kp = a*Ku, Ti = b*Pu,
        /// Td = c*Pu.
        ///
        /// WARNING - "no overshoot" is aspirational, not a guarantee. That row
        /// and "some overshoot" differ from ZN only in Kp; Ti stays pinned at
        /// Pu/2, and on FOPDT plants that Ti is what produces the overshoot.
        /// Measured on K=2 T=1 L=0.1 with an EXACT model, NoOvershoot still
        /// overshoots 43%. Stretching Ti is what fixes it (Ti = 4*Pu gives
        /// 0.0%). The coefficients are faithful to the published table; the
        /// limitation is the rule's. Prefer Imc or AmigoStep when overshoot
        /// must genuinely be near zero.
        /// </summary>
        private static double[] FreqCoef(TuneRule rule, TuneStructure s)
        {
            double[][] tab;
            switch (rule)
            {
                case TuneRule.Zn:
                    tab = new[] {
                        new[] { 0.50, 0.0, 0.0 },
                        new[] { 0.45, 1.0 / 1.2, 0.0 },
                        new[] { 0.60, 0.50, 0.125 } };
                    break;
                case TuneRule.TyreusLuyben:
                    tab = new[] {
                        new[] { 0.50, 0.0, 0.0 },
                        new[] { 0.31, 2.20, 0.0 },
                        new[] { 0.45, 2.20, 1.0 / 6.3 } };
                    break;
                case TuneRule.Pessen:
                    tab = new[] {
                        new[] { 0.50, 0.0, 0.0 },
                        new[] { 0.45, 1.0 / 1.2, 0.0 },
                        new[] { 0.70, 0.40, 0.15 } };
                    break;
                case TuneRule.SomeOvershoot:
                    tab = new[] {
                        new[] { 0.33, 0.0, 0.0 },
                        new[] { 0.33, 0.50, 0.0 },
                        new[] { 0.33, 0.50, 1.0 / 3.0 } };
                    break;
                case TuneRule.NoOvershoot:
                    tab = new[] {
                        new[] { 0.20, 0.0, 0.0 },
                        new[] { 0.20, 0.50, 0.0 },
                        new[] { 0.20, 0.50, 1.0 / 3.0 } };
                    break;
                default:
                    tab = new[] {
                        new[] { 0.0, 0.0, 0.0 },
                        new[] { 0.0, 0.0, 0.0 },
                        new[] { 0.0, 0.0, 0.0 } };
                    break;
            }
            return tab[(int)s];
        }

        /// <summary>
        /// AMIGO in frequency-domain form (Astrom and Hagglund 2004). The rule
        /// is expressed through the normalised gain kappa = 1/(Ku*K), but with
        /// only (Ku, Pu) available the robust published approximation is used:
        /// the coefficients AMIGO collapses to at the design point Ms = 1.4
        /// when the normalised dead time is unknown. Deliberately
        /// conservative, which is the whole point of AMIGO versus ZN.
        /// </summary>
        private static void AmigoFreq(Gains g, PlantModel m, TuneStructure s)
        {
            if (s == TuneStructure.P)
                Finish(g, 0.20 * m.Ku, 0.0, 0.0);
            else if (s == TuneStructure.Pi)
                Finish(g, 0.16 * m.Ku, 0.46 * m.Pu, 0.0);
            else
                Finish(g, 0.16 * m.Ku, 0.46 * m.Pu, 0.10 * m.Pu);
        }

        /// <summary>
        /// Cohen-Coon (1953), quarter-amplitude decay on dead-time dominant
        /// processes. Valid for L/T roughly in [0.1, 1]; outside that band the
        /// formulas still evaluate but the caller's quality check reports the
        /// mismatch rather than silently producing a wild gain.
        /// </summary>
        private static void CohenCoon(Gains g, PlantModel m, TuneStructure s)
        {
            double tau = m.L / m.T;                 // normalised dead time
            double inv = 1.0 / (m.K * tau);

            if (s == TuneStructure.P)
            {
                Finish(g, inv * (1.0 + tau / 3.0), 0.0, 0.0);
            }
            else if (s == TuneStructure.Pi)
            {
                double kp = inv * (0.9 + tau / 12.0);
                double ti = m.L * (30.0 + 3.0 * tau) / (9.0 + 20.0 * tau);
                Finish(g, kp, ti, 0.0);
            }
            else
            {
                double kp = inv * (4.0 / 3.0 + tau / 4.0);
                double ti = m.L * (32.0 + 6.0 * tau) / (13.0 + 8.0 * tau);
                double td = m.L * 4.0 / (11.0 + 2.0 * tau);
                Finish(g, kp, ti, td);
            }
        }

        /// <summary>
        /// AMIGO step rule (Astrom and Hagglund 2004), the modern FOPDT
        /// default. Designed for maximum sensitivity Ms = 1.4, i.e. an
        /// explicit robustness target - unlike ZN, which has none.
        /// </summary>
        private static void AmigoStep(Gains g, PlantModel m, TuneStructure s)
        {
            double k = m.K, t = m.T, l = m.L;

            if (s == TuneStructure.P)
            {
                // AMIGO defines no pure-P rule; the PI proportional part with
                // the integral removed is the conservative fallback.
                double sum = l + t;
                double kp = (0.15 + (0.35 - l * t / (sum * sum)) * t / l) / k;
                Finish(g, kp, 0.0, 0.0);
            }
            else if (s == TuneStructure.Pi)
            {
                double sum = l + t;
                double kp = (0.15 + (0.35 - l * t / (sum * sum)) * t / l) / k;
                double ti = 0.35 * l + 13.0 * l * t * t
                            / (t * t + 12.0 * l * t + 7.0 * l * l);
                Finish(g, kp, ti, 0.0);
            }
            else
            {
                double kp = (0.2 + 0.45 * t / l) / k;
                double ti = (0.4 * l + 0.8 * t) / (l + 0.1 * t) * l;
                double td = 0.5 * l * t / (0.3 * l + t);
                Finish(g, kp, ti, td);
            }
        }

        /// <summary>
        /// IMC / lambda tuning (Rivera-Morari-Skogestad), FOPDT with a
        /// first-order Pade approximation of the dead time. lambda is the
        /// desired closed-loop time constant: the single knob for the
        /// speed/robustness trade-off.
        /// </summary>
        private static void Imc(Gains g, PlantModel m, TuneStructure s,
                                double lambda)
        {
            double k = m.K, t = m.T, l = m.L;
            double lam = lambda;

            if (!(lam > 0.0))
            {
                // Default: the larger of the dead-time floor and a fifth of
                // the dominant time constant. Both standard conservative
                // choices.
                double a = 0.5 * l;
                double b = 0.2 * t;
                lam = (a > b) ? a : b;
            }
            // Robustness floor: below 0.2*L the controller depends on a dead
            // time estimate it cannot trust.
            if (lam < 0.2 * l) lam = 0.2 * l;

            if (s == TuneStructure.P)
            {
                Finish(g, t / (k * (lam + l)), 0.0, 0.0);
            }
            else if (s == TuneStructure.Pi)
            {
                Finish(g, t / (k * (lam + l)), t, 0.0);
            }
            else
            {
                double half = 0.5 * l;
                double kp = (t + half) / (k * (lam + half));
                double ti = t + half;
                double td = t * l / (2.0 * t + l);
                Finish(g, kp, ti, td);
            }
        }

        /// <summary>
        /// Apply a tuning rule to a model without running any experiment.
        /// A pure function: useful for offline tuning, tests, and re-tuning a
        /// stored model with a different rule.
        /// </summary>
        public static Status Apply(TuneRule rule, PlantModel model,
                                   TuneStructure structure, double lambda,
                                   out Gains outGains)
        {
            Gains g = new Gains();
            outGains = g;

            if (model == null) return Status.ErrNull;
            if ((int)rule < 0 || rule > TuneRule.Custom)
                return Status.ErrInvalidParam;
            if ((int)structure < 0 || structure > TuneStructure.Pid)
                return Status.ErrInvalidParam;
            if (rule == TuneRule.Custom)
                return Status.ErrInvalidParam;   // dispatched by the tuner

            ModelKind need = RequiredModel(rule);
            if (model.Kind != need)
            {
                // The central honesty check: a frequency point is not a FOPDT
                // model and no correct conversion between them exists.
                return Status.ErrTuneModelMismatch;
            }

            if (need == ModelKind.Freq)
            {
                if (!M.IsFinite(model.Ku) || !M.IsFinite(model.Pu)
                    || model.Ku <= 0.0 || model.Pu <= 0.0)
                    return Status.ErrTuneValidation;
            }
            else
            {
                if (!M.IsFinite(model.K) || !M.IsFinite(model.T)
                    || !M.IsFinite(model.L)
                    || M.Abs(model.K) <= 0.0
                    || model.T <= 0.0 || model.L <= 0.0)
                    return Status.ErrTuneValidation;
            }

            switch (rule)
            {
                case TuneRule.AmigoFreq: AmigoFreq(g, model, structure); break;
                case TuneRule.CohenCoon: CohenCoon(g, model, structure); break;
                case TuneRule.AmigoStep: AmigoStep(g, model, structure); break;
                case TuneRule.Imc: Imc(g, model, structure, lambda); break;
                default:
                    {
                        double[] c = FreqCoef(rule, structure);
                        Finish(g, c[0] * model.Ku, c[1] * model.Pu,
                               c[2] * model.Pu);
                        break;
                    }
            }

            if (!M.IsFinite(g.Kp) || !M.IsFinite(g.Ki) || !M.IsFinite(g.Kd))
                return Status.ErrTuneValidation;
            return Status.Ok;
        }
    }
}

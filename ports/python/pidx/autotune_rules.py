"""Closed-form tuning rules. Mirrors src/pid_autotune_rules.c.

Each rule is a pure function from an identified plant model to controller
gains. No rule touches a controller, allocates, or blocks, so every rule is
testable in isolation against published coefficients.

Two families exist and they are NOT interchangeable:

  FREQ rules consume (Ku, Pu) - one point on the Nyquist curve, where the loop
  phase is -180 deg. Ziegler-Nichols and its descendants are all of this form.

  FOPDT rules consume (K, T, L) from G(s) = K*exp(-L*s)/(1+T*s). Cohen-Coon,
  AMIGO-step and IMC are derived from that three-parameter model and cannot be
  evaluated from (Ku, Pu): one complex number does not determine three real
  parameters. Requesting such a pairing returns ERR_TUNE_MODEL_MISMATCH.

Output convention: the parallel form the core uses,
    u = Kp*e + Ki*integral(e) + Kd*de/dt,   Ki = Kp/Ti,  Kd = Kp*Td.
A rule that yields no integral action reports Ti = 0 and Ki = 0.

NUMERIC NOTE
------------
The C table stores its coefficients as `float` literals (0.45f, 1.0f/1.2f,
...). This module writes the same coefficients as exact decimal values. The two
therefore differ in the 8th significant digit whenever the C library is built
with PIDX_USE_DOUBLE=1, because a float literal promoted to double keeps its
float rounding: 0.45f is 0.44999998807907104, not 0.45.

That is a property of the C build option, not a disagreement about the rule.
See docs/24_port_comparison.md - the conformance harness treats these cells
with a float-grade tolerance and the deviation is documented rather than
papered over.
"""

from enum import IntEnum

from . import mathutil as m
from .types import Status


class IdentMethod(IntEnum):
    RELAY = 0
    STEP = 1


class ModelKind(IntEnum):
    NONE = 0
    FREQ = 1
    FOPDT = 2


class TuneStructure(IntEnum):
    P = 0
    PI = 1
    PID = 2


class TuneRule(IntEnum):
    ZN = 0
    TYREUS_LUYBEN = 1
    PESSEN = 2
    SOME_OVERSHOOT = 3
    NO_OVERSHOOT = 4
    AMIGO_FREQ = 5
    COHEN_COON = 6
    AMIGO_STEP = 7
    IMC = 8
    CUSTOM = 9


class PlantModel:
    """Identified plant. Produced by identification, consumed by a rule."""

    __slots__ = ("kind", "ku", "pu", "k", "t", "l", "noise_sigma", "quality")

    def __init__(self, kind=ModelKind.NONE, ku=0.0, pu=0.0,
                 k=0.0, t=0.0, l=0.0):
        self.kind = kind
        self.ku = ku            # ultimate gain
        self.pu = pu            # ultimate period [s]
        self.k = k              # FOPDT static gain
        self.t = t              # FOPDT time constant [s]
        self.l = l              # FOPDT dead time [s]
        self.noise_sigma = 0.0
        self.quality = 0


class Gains:
    """Parallel-form gains plus the derivative filter time constant."""

    __slots__ = ("kp", "ki", "kd", "ti", "td", "tf")

    def __init__(self):
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.ti = 0.0
        self.td = 0.0
        self.tf = 0.0

    def as_tuple(self):
        return (self.kp, self.ki, self.kd, self.ti, self.td, self.tf)


#: Coefficient triples (a, b, c): Kp = a*Ku, Ti = b*Pu, Td = c*Pu.
#: b == 0 means no integral action, c == 0 means no derivative action.
#:
#: ZN (1942, quarter-amplitude decay), Tyreus-Luyben (1992, robust on
#: lag-dominant processes), Pessen Integral Rule (faster than ZN, more
#: overshoot), and the "some/no overshoot" rows as tabulated by Astrom &
#: Hagglund.
#:
#: WARNING - "no overshoot" is aspirational, not a guarantee. Those two rows
#: differ from ZN only in Kp; Ti stays pinned at Pu/2, and on FOPDT plants that
#: Ti is what produces the overshoot. Measured on K=2 T=1 L=0.1 with an EXACT
#: model, NO_OVERSHOOT still overshoots 43%. Stretching Ti is what fixes it
#: (Ti=4Pu -> 0.0%). The coefficients are faithful to the published table; the
#: limitation is the rule's. Prefer IMC or AMIGO when overshoot must be small.
_FREQ_TAB = {
    TuneRule.ZN: {
        TuneStructure.P:   (0.50, 0.0, 0.0),
        TuneStructure.PI:  (0.45, 1.0 / 1.2, 0.0),
        TuneStructure.PID: (0.60, 0.50, 0.125),
    },
    TuneRule.TYREUS_LUYBEN: {
        TuneStructure.P:   (0.50, 0.0, 0.0),
        TuneStructure.PI:  (0.31, 2.20, 0.0),
        TuneStructure.PID: (0.45, 2.20, 1.0 / 6.3),
    },
    TuneRule.PESSEN: {
        TuneStructure.P:   (0.50, 0.0, 0.0),
        TuneStructure.PI:  (0.45, 1.0 / 1.2, 0.0),
        TuneStructure.PID: (0.70, 0.40, 0.15),
    },
    TuneRule.SOME_OVERSHOOT: {
        TuneStructure.P:   (0.33, 0.0, 0.0),
        TuneStructure.PI:  (0.33, 0.50, 0.0),
        TuneStructure.PID: (0.33, 0.50, 1.0 / 3.0),
    },
    TuneRule.NO_OVERSHOOT: {
        TuneStructure.P:   (0.20, 0.0, 0.0),
        TuneStructure.PI:  (0.20, 0.50, 0.0),
        TuneStructure.PID: (0.20, 0.50, 1.0 / 3.0),
    },
}

_RULE_MODEL = {
    TuneRule.ZN: ModelKind.FREQ,
    TuneRule.TYREUS_LUYBEN: ModelKind.FREQ,
    TuneRule.PESSEN: ModelKind.FREQ,
    TuneRule.SOME_OVERSHOOT: ModelKind.FREQ,
    TuneRule.NO_OVERSHOOT: ModelKind.FREQ,
    TuneRule.AMIGO_FREQ: ModelKind.FREQ,
    TuneRule.COHEN_COON: ModelKind.FOPDT,
    TuneRule.AMIGO_STEP: ModelKind.FOPDT,
    TuneRule.IMC: ModelKind.FOPDT,
    TuneRule.CUSTOM: ModelKind.NONE,
}

_RULE_NAME = {
    TuneRule.ZN: "Ziegler-Nichols",
    TuneRule.TYREUS_LUYBEN: "Tyreus-Luyben",
    TuneRule.PESSEN: "Pessen-Integral",
    TuneRule.SOME_OVERSHOOT: "Some-Overshoot",
    TuneRule.NO_OVERSHOOT: "No-Overshoot",
    TuneRule.AMIGO_FREQ: "AMIGO-freq",
    TuneRule.COHEN_COON: "Cohen-Coon",
    TuneRule.AMIGO_STEP: "AMIGO-step",
    TuneRule.IMC: "IMC-lambda",
    TuneRule.CUSTOM: "Custom",
}


def rule_required_model(rule):
    return _RULE_MODEL.get(rule, ModelKind.NONE)


def rule_name(rule):
    return _RULE_NAME.get(rule, "?")


def _finish(g, kp, ti, td):
    """Convert (Kp, Ti, Td) into the parallel form the core uses."""
    g.kp = kp
    g.ti = ti
    g.td = td
    # Ti == 0 encodes "no integral action" - do not divide by it.
    g.ki = (kp / ti) if ti > 0.0 else 0.0
    g.kd = kp * td
    # Derivative filter from the standard N = 10 rule: Tf = Td/N. Without a
    # filter the derivative term differentiates sensor noise without bound.
    g.tf = td * 0.1


def _amigo_freq(g, mdl, s):
    """AMIGO in frequency-domain form (Astrom & Hagglund 2004).

    The rule is expressed through the normalised gain kappa = 1/(Ku*K), but
    with only (Ku, Pu) available the robust published approximation is used:
    Kp = 0.16*Ku, Ti = 0.46*Pu, Td = 0.10*Pu. These are the coefficients AMIGO
    collapses to at the design point Ms = 1.4 when the normalised dead time is
    unknown - deliberately conservative, which is the point of AMIGO over ZN.
    """
    if s == TuneStructure.P:
        _finish(g, 0.20 * mdl.ku, 0.0, 0.0)
    elif s == TuneStructure.PI:
        _finish(g, 0.16 * mdl.ku, 0.46 * mdl.pu, 0.0)
    else:
        _finish(g, 0.16 * mdl.ku, 0.46 * mdl.pu, 0.10 * mdl.pu)


def _cohen_coon(g, mdl, s):
    """Cohen-Coon (1953), quarter-amplitude decay on dead-time dominant plants.

    With tau = L/T:
      P   : Kp = (1/K)(1/tau)(1 + tau/3)
      PI  : Kp = (1/K)(1/tau)(0.9 + tau/12), Ti = L(30 + 3tau)/(9 + 20tau)
      PID : Kp = (1/K)(1/tau)(4/3 + tau/4),  Ti = L(32 + 6tau)/(13 + 8tau),
            Td = 4L/(11 + 2tau)

    Valid for L/T roughly in [0.1, 1]. Outside that band the formulas still
    evaluate but the caller's quality check reports the mismatch rather than
    silently producing a wild gain.
    """
    tau = mdl.l / mdl.t
    inv = 1.0 / (mdl.k * tau)

    if s == TuneStructure.P:
        _finish(g, inv * (1.0 + tau / 3.0), 0.0, 0.0)
    elif s == TuneStructure.PI:
        kp = inv * (0.9 + tau / 12.0)
        ti = mdl.l * (30.0 + 3.0 * tau) / (9.0 + 20.0 * tau)
        _finish(g, kp, ti, 0.0)
    else:
        kp = inv * (4.0 / 3.0 + tau / 4.0)
        ti = mdl.l * (32.0 + 6.0 * tau) / (13.0 + 8.0 * tau)
        td = mdl.l * 4.0 / (11.0 + 2.0 * tau)
        _finish(g, kp, ti, td)


def _amigo_step(g, mdl, s):
    """AMIGO step rule (Astrom & Hagglund 2004), the modern FOPDT default.

      PI  : Kp = (1/K)(0.15 + (0.35 - L*T/(L+T)^2)*T/L)
            Ti = 0.35L + 13*L*T^2/(T^2 + 12LT + 7L^2)
      PID : Kp = (1/K)(0.2 + 0.45*T/L)
            Ti = (0.4L + 0.8T)/(L + 0.1T) * L
            Td = 0.5*L*T/(0.3L + T)

    Designed for maximum sensitivity Ms = 1.4, i.e. an explicit robustness
    target - unlike ZN, which has none.
    """
    k, t, l = mdl.k, mdl.t, mdl.l

    if s == TuneStructure.P:
        # AMIGO defines no pure-P rule; the PI proportional part with the
        # integral removed is the conservative fallback.
        sm = l + t
        kp = (0.15 + (0.35 - l * t / (sm * sm)) * t / l) / k
        _finish(g, kp, 0.0, 0.0)
    elif s == TuneStructure.PI:
        sm = l + t
        kp = (0.15 + (0.35 - l * t / (sm * sm)) * t / l) / k
        ti = 0.35 * l + 13.0 * l * t * t / (t * t + 12.0 * l * t + 7.0 * l * l)
        _finish(g, kp, ti, 0.0)
    else:
        kp = (0.2 + 0.45 * t / l) / k
        ti = (0.4 * l + 0.8 * t) / (l + 0.1 * t) * l
        td = 0.5 * l * t / (0.3 * l + t)
        _finish(g, kp, ti, td)


def _imc(g, mdl, s, lambda_):
    """IMC / lambda tuning (Rivera-Morari-Skogestad), FOPDT with a first-order
    Pade approximation of the dead time:

      PI  : Kp = T/(K(lambda + L)),           Ti = T
      PID : Kp = (T + L/2)/(K(lambda + L/2)), Ti = T + L/2, Td = TL/(2T + L)

    lambda is the desired closed-loop time constant and is the single knob for
    the speed/robustness trade-off.
    """
    k, t, l = mdl.k, mdl.t, mdl.l
    lam = lambda_

    if not (lam > 0.0):
        # Default: the larger of the dead-time floor and a fifth of the
        # dominant time constant. Both are standard conservative choices.
        a = 0.5 * l
        b = 0.2 * t
        lam = a if a > b else b
    # Robustness floor: below 0.2*L the controller depends on a dead-time
    # estimate it cannot trust.
    if lam < 0.2 * l:
        lam = 0.2 * l

    if s == TuneStructure.P:
        _finish(g, t / (k * (lam + l)), 0.0, 0.0)
    elif s == TuneStructure.PI:
        _finish(g, t / (k * (lam + l)), t, 0.0)
    else:
        half = 0.5 * l
        kp = (t + half) / (k * (lam + half))
        ti = t + half
        td = t * l / (2.0 * t + l)
        _finish(g, kp, ti, td)


def rule_apply(rule, model, structure=TuneStructure.PID, lambda_=0.0):
    """Apply a tuning rule to a model. Returns (status, Gains).

    A pure function: useful for offline tuning, tests, and re-tuning a stored
    model with a different rule.
    """
    out = Gains()

    if model is None:
        return Status.ERR_NULL, out
    if rule not in _RULE_MODEL:
        return Status.ERR_INVALID_PARAM, out
    if structure not in (TuneStructure.P, TuneStructure.PI, TuneStructure.PID):
        return Status.ERR_INVALID_PARAM, out
    if rule == TuneRule.CUSTOM:
        return Status.ERR_INVALID_PARAM, out   # dispatched by the tuner

    need = _RULE_MODEL[rule]
    if model.kind != need:
        # The central honesty check: a frequency point is not a FOPDT model and
        # no correct conversion between them exists.
        return Status.ERR_TUNE_MODEL_MISMATCH, out

    if need == ModelKind.FREQ:
        if (not m.isfinite(model.ku) or not m.isfinite(model.pu)
                or model.ku <= 0.0 or model.pu <= 0.0):
            return Status.ERR_TUNE_VALIDATION, out
    else:
        if (not m.isfinite(model.k) or not m.isfinite(model.t)
                or not m.isfinite(model.l)
                or m.fabs(model.k) <= 0.0
                or model.t <= 0.0 or model.l <= 0.0):
            return Status.ERR_TUNE_VALIDATION, out

    if rule == TuneRule.AMIGO_FREQ:
        _amigo_freq(out, model, structure)
    elif rule == TuneRule.COHEN_COON:
        _cohen_coon(out, model, structure)
    elif rule == TuneRule.AMIGO_STEP:
        _amigo_step(out, model, structure)
    elif rule == TuneRule.IMC:
        _imc(out, model, structure, lambda_)
    else:
        a, b, c = _FREQ_TAB[rule][structure]
        _finish(out, a * model.ku, b * model.pu, c * model.pu)

    if not (m.isfinite(out.kp) and m.isfinite(out.ki) and m.isfinite(out.kd)):
        return Status.ERR_TUNE_VALIDATION, out
    return Status.OK, out

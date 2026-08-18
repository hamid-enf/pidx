/**
 * @file    pid_autotune_rules.c
 * @brief   Closed-form tuning rules, table driven.
 *
 * Each rule is a pure function from an identified plant model to controller
 * gains. No rule touches a handle, allocates, or blocks, so every rule is
 * testable in isolation against published coefficients.
 *
 * Two families exist and they are NOT interchangeable:
 *
 *   FREQ rules consume (Ku, Pu) - one point on the Nyquist curve, the point
 *   where the loop phase is -180 deg. Ziegler-Nichols and its descendants are
 *   all of this form: Kp = c*Ku, Ti = c*Pu, Td = c*Pu.
 *
 *   FOPDT rules consume (K, T, L) from G(s) = K*exp(-L*s)/(1+T*s). Cohen-Coon,
 *   AMIGO-step and IMC are derived from that three-parameter model and cannot
 *   be evaluated from (Ku, Pu): one complex number does not determine three
 *   real parameters. Requesting such a pairing returns
 *   PID_ERR_TUNE_MODEL_MISMATCH.
 *
 * Output convention: the parallel form used by the core,
 *   u = Kp*e + Ki*integral(e) + Kd*de/dt,  Ki = Kp/Ti,  Kd = Kp*Td.
 * A rule that yields no integral action reports Ti = 0 and Ki = 0.
 */

#include "pidx/pid_autotune.h"

#if PIDX_ENABLE_AUTOTUNE

#include "pidx/pid_math.h"

/* ======================================================================== */
/* Table descriptor                                                          */
/* ======================================================================== */

/**
 * Coefficient triple for the frequency-domain family:
 *   Kp = a*Ku,  Ti = b*Pu,  Td = c*Pu
 * A zero b means "no integral action", a zero c means "no derivative action".
 */
typedef struct {
    PID_Float a;
    PID_Float b;
    PID_Float c;
} PID_FreqCoef;

/** One row per rule: the P, PI and PID coefficient sets. */
typedef struct {
    PID_FreqCoef p;
    PID_FreqCoef pi;
    PID_FreqCoef pid;
} PID_FreqRuleRow;

/**
 * Published coefficients.
 *
 * ZN (Ziegler-Nichols 1942, quarter-amplitude decay):
 *   P   Kp=0.5Ku
 *   PI  Kp=0.45Ku  Ti=Pu/1.2
 *   PID Kp=0.6Ku   Ti=Pu/2    Td=Pu/8
 *
 * Tyreus-Luyben (1992), tuned for robustness on lag-dominant processes:
 *   PI  Kp=0.31Ku  Ti=2.2Pu
 *   PID Kp=0.45Ku  Ti=2.2Pu   Td=Pu/6.3
 *
 * Pessen Integral Rule - faster than ZN, larger overshoot:
 *   PID Kp=0.7Ku   Ti=Pu/2.5  Td=3Pu/20
 *
 * "Some overshoot" and "no overshoot" variants (Ziegler-Nichols family as
 * tabulated by Astrom & Hagglund):
 *   some: Kp=0.33Ku Ti=Pu/2   Td=Pu/3
 *   none: Kp=0.20Ku Ti=Pu/2   Td=Pu/3
 *
 *   WARNING - the name is aspirational, not a guarantee. These two rows
 *   differ from ZN only in Kp; Ti stays pinned at Pu/2. On FOPDT plants
 *   that Ti is what produces the overshoot, so lowering Kp alone does not
 *   remove it and can make it worse. Measured on K=2 T=1 L=0.1 with an
 *   EXACT model (zero identification error), Ti=Pu/2, Td=Pu/3:
 *      Kp=0.60Ku -> 44.2%   Kp=0.33Ku -> 38.3%   Kp=0.20Ku -> 43.2%
 *   while stretching Ti at Kp=0.20Ku is what actually works:
 *      Ti=Pu/2 -> 43.2%   Ti=2Pu -> 8.4%   Ti=4Pu -> 0.0%
 *   The coefficients below are faithful to the published table; the
 *   limitation is the rule's, not this implementation's. Prefer IMC or
 *   AMIGO when overshoot genuinely must be near zero. See sim/sim_rules.c.
 *
 * Rows are indexed by PID_TuneRule; FOPDT rules are computed, not tabulated,
 * so their rows are zero and never read.
 */
static const PID_FreqRuleRow pidt_freq_tab[PID_RULE_COUNT_] = {
    /* PID_RULE_ZN */
    { { (PID_Float)0.50, PID_ZERO,         PID_ZERO           },
      { (PID_Float)0.45, (PID_Float)(1.0 / 1.2), PID_ZERO         },
      { (PID_Float)0.60, (PID_Float)0.50,   (PID_Float)0.125   } },
    /* PID_RULE_TYREUS_LUYBEN */
    { { (PID_Float)0.50, PID_ZERO,         PID_ZERO           },
      { (PID_Float)0.31, (PID_Float)2.20,   PID_ZERO           },
      { (PID_Float)0.45, (PID_Float)2.20,   (PID_Float)(1.0 / 6.3)} },
    /* PID_RULE_PESSEN */
    { { (PID_Float)0.50, PID_ZERO,         PID_ZERO           },
      { (PID_Float)0.45, (PID_Float)(1.0 / 1.2), PID_ZERO         },
      { (PID_Float)0.70, (PID_Float)0.40,   (PID_Float)0.15    } },
    /* PID_RULE_SOME_OVERSHOOT */
    { { (PID_Float)0.33, PID_ZERO,         PID_ZERO           },
      { (PID_Float)0.33, (PID_Float)0.50,   PID_ZERO           },
      { (PID_Float)0.33, (PID_Float)0.50,   (PID_Float)(1.0 / 3.0)} },
    /* PID_RULE_NO_OVERSHOOT */
    { { (PID_Float)0.20, PID_ZERO,         PID_ZERO           },
      { (PID_Float)0.20, (PID_Float)0.50,   PID_ZERO           },
      { (PID_Float)0.20, (PID_Float)0.50,   (PID_Float)(1.0 / 3.0)} },
    /* PID_RULE_AMIGO_FREQ  - computed, see pidt_amigo_freq() */
    { { PID_ZERO, PID_ZERO, PID_ZERO }, { PID_ZERO, PID_ZERO, PID_ZERO },
      { PID_ZERO, PID_ZERO, PID_ZERO } },
    /* FOPDT rules - computed */
    { { PID_ZERO, PID_ZERO, PID_ZERO }, { PID_ZERO, PID_ZERO, PID_ZERO },
      { PID_ZERO, PID_ZERO, PID_ZERO } },
    { { PID_ZERO, PID_ZERO, PID_ZERO }, { PID_ZERO, PID_ZERO, PID_ZERO },
      { PID_ZERO, PID_ZERO, PID_ZERO } },
    { { PID_ZERO, PID_ZERO, PID_ZERO }, { PID_ZERO, PID_ZERO, PID_ZERO },
      { PID_ZERO, PID_ZERO, PID_ZERO } },
    /* PID_RULE_CUSTOM */
    { { PID_ZERO, PID_ZERO, PID_ZERO }, { PID_ZERO, PID_ZERO, PID_ZERO },
      { PID_ZERO, PID_ZERO, PID_ZERO } }
};

/** Model kind each built-in rule requires. */
static const PID_ModelKind pidt_rule_model[PID_RULE_COUNT_] = {
    PID_MODEL_FREQ,   /* ZN               */
    PID_MODEL_FREQ,   /* TYREUS_LUYBEN    */
    PID_MODEL_FREQ,   /* PESSEN           */
    PID_MODEL_FREQ,   /* SOME_OVERSHOOT   */
    PID_MODEL_FREQ,   /* NO_OVERSHOOT     */
    PID_MODEL_FREQ,   /* AMIGO_FREQ       */
    PID_MODEL_FOPDT,  /* COHEN_COON       */
    PID_MODEL_FOPDT,  /* AMIGO_STEP       */
    PID_MODEL_FOPDT,  /* IMC              */
    PID_MODEL_NONE    /* CUSTOM           */
};

static const char *const pidt_rule_name[PID_RULE_COUNT_] = {
    "Ziegler-Nichols",
    "Tyreus-Luyben",
    "Pessen-Integral",
    "Some-Overshoot",
    "No-Overshoot",
    "AMIGO-freq",
    "Cohen-Coon",
    "AMIGO-step",
    "IMC-lambda",
    "Custom"
};

PID_ModelKind PID_TuneRule_RequiredModel(PID_TuneRule rule)
{
    if ((int)rule < 0 || rule >= PID_RULE_COUNT_) {
        return PID_MODEL_NONE;
    }
    return pidt_rule_model[rule];
}

const char *PID_TuneRule_Name(PID_TuneRule rule)
{
    if ((int)rule < 0 || rule >= PID_RULE_COUNT_) {
        return "?";
    }
    return pidt_rule_name[rule];
}

/* ======================================================================== */
/* Helpers                                                                   */
/* ======================================================================== */

/** Convert (Kp, Ti, Td) into the parallel form the core uses. */
static void pidt_finish(PID_Gains *g, PID_Float kp, PID_Float ti, PID_Float td)
{
    g->kp = kp;
    g->ti = ti;
    g->td = td;
    /* Ti = 0 encodes "no integral action" - do not divide by it. */
    g->ki = (ti > (PID_Float)0.0) ? (kp / ti) : (PID_Float)0.0;
    g->kd = kp * td;
    /* Derivative filter from the standard N = 10 rule: Tf = Td / N. Without a
     * filter the derivative term differentiates sensor noise without bound. */
    g->tf = td * (PID_Float)0.1;
}

/**
 * AMIGO in its frequency-domain form (Astrom & Hagglund 2004).
 * The rule is expressed through the normalised gain kappa = 1/(Ku*K), but with
 * only (Ku, Pu) available the robust published approximation is used:
 *
 *   Kp = 0.16 * Ku
 *   Ti = 0.46 * Pu
 *   Td = 0.10 * Pu
 *
 * These are the coefficients AMIGO collapses to at the design point Ms = 1.4
 * for a plant whose normalised dead time is unknown - deliberately
 * conservative, which is the whole point of AMIGO versus ZN.
 */
static void pidt_amigo_freq(PID_Gains *g, const PID_PlantModel *m,
                            PID_TuneStructure s)
{
    if (s == PID_STRUCT_P) {
        pidt_finish(g, (PID_Float)0.20 * m->ku, (PID_Float)0.0, (PID_Float)0.0);
    } else if (s == PID_STRUCT_PI) {
        pidt_finish(g, (PID_Float)0.16 * m->ku, (PID_Float)0.46 * m->pu,
                    (PID_Float)0.0);
    } else {
        pidt_finish(g, (PID_Float)0.16 * m->ku, (PID_Float)0.46 * m->pu,
                    (PID_Float)0.10 * m->pu);
    }
}

/**
 * Cohen-Coon (1953). Derived from a FOPDT model, matched for quarter-amplitude
 * decay on dead-time dominant processes. With tau = L/T:
 *
 *   P   : Kp = (1/K)(1/tau)(1 + tau/3)
 *   PI  : Kp = (1/K)(1/tau)(0.9 + tau/12)
 *         Ti = L (30 + 3tau) / (9 + 20tau)
 *   PID : Kp = (1/K)(1/tau)(4/3 + tau/4)
 *         Ti = L (32 + 6tau) / (13 + 8tau)
 *         Td = L * 4 / (11 + 2tau)
 *
 * Valid for L/T in roughly [0.1, 1]. Outside that band the formulas still
 * evaluate but the model is reported as a mismatch by the caller's quality
 * check rather than silently producing a wild gain.
 */
static void pidt_cohen_coon(PID_Gains *g, const PID_PlantModel *m,
                            PID_TuneStructure s)
{
    const PID_Float tau  = m->l / m->t;          /* normalised dead time     */
    const PID_Float inv  = (PID_Float)1.0 / (m->k * tau);

    if (s == PID_STRUCT_P) {
        pidt_finish(g, inv * ((PID_Float)1.0 + tau / (PID_Float)3.0),
                    (PID_Float)0.0, (PID_Float)0.0);
    } else if (s == PID_STRUCT_PI) {
        const PID_Float kp = inv * ((PID_Float)0.9 + tau / (PID_Float)12.0);
        const PID_Float ti = m->l * ((PID_Float)30.0 + (PID_Float)3.0 * tau)
                                  / ((PID_Float)9.0 + (PID_Float)20.0 * tau);
        pidt_finish(g, kp, ti, (PID_Float)0.0);
    } else {
        const PID_Float kp = inv * ((PID_Float)4.0 / (PID_Float)3.0
                                    + tau / (PID_Float)4.0);
        const PID_Float ti = m->l * ((PID_Float)32.0 + (PID_Float)6.0 * tau)
                                  / ((PID_Float)13.0 + (PID_Float)8.0 * tau);
        const PID_Float td = m->l * (PID_Float)4.0
                                  / ((PID_Float)11.0 + (PID_Float)2.0 * tau);
        pidt_finish(g, kp, ti, td);
    }
}

/**
 * AMIGO step rule (Astrom & Hagglund 2004), the modern default for FOPDT.
 *
 *   PI  : Kp = (1/K)(0.15 + (0.35 - L*T/(L+T)^2) * T/L)
 *         Ti = 0.35 L + 13 L T^2 / (T^2 + 12 L T + 7 L^2)
 *   PID : Kp = (1/K)(0.2 + 0.45 T/L)
 *         Ti = (0.4 L + 0.8 T) / (L + 0.1 T) * L
 *         Td = 0.5 L T / (0.3 L + T)
 *
 * Designed for a maximum sensitivity Ms = 1.4, i.e. an explicit robustness
 * target - unlike ZN, which has none.
 */
static void pidt_amigo_step(PID_Gains *g, const PID_PlantModel *m,
                            PID_TuneStructure s)
{
    const PID_Float k = m->k;
    const PID_Float t = m->t;
    const PID_Float l = m->l;

    if (s == PID_STRUCT_P) {
        /* AMIGO defines no pure-P rule; fall back to the PI proportional part
         * with the integral removed, which is the conservative choice. */
        const PID_Float sum = l + t;
        const PID_Float kp  = ((PID_Float)0.15
                               + ((PID_Float)0.35 - l * t / (sum * sum))
                                 * t / l) / k;
        pidt_finish(g, kp, (PID_Float)0.0, (PID_Float)0.0);
    } else if (s == PID_STRUCT_PI) {
        const PID_Float sum = l + t;
        const PID_Float kp  = ((PID_Float)0.15
                               + ((PID_Float)0.35 - l * t / (sum * sum))
                                 * t / l) / k;
        const PID_Float ti  = (PID_Float)0.35 * l
                            + (PID_Float)13.0 * l * t * t
                              / (t * t + (PID_Float)12.0 * l * t
                                 + (PID_Float)7.0 * l * l);
        pidt_finish(g, kp, ti, (PID_Float)0.0);
    } else {
        const PID_Float kp = ((PID_Float)0.2 + (PID_Float)0.45 * t / l) / k;
        const PID_Float ti = ((PID_Float)0.4 * l + (PID_Float)0.8 * t)
                             / (l + (PID_Float)0.1 * t) * l;
        const PID_Float td = (PID_Float)0.5 * l * t
                             / ((PID_Float)0.3 * l + t);
        pidt_finish(g, kp, ti, td);
    }
}

/**
 * IMC / lambda tuning (Rivera-Morari-Skogestad), FOPDT with a first-order
 * Pade approximation of the dead time:
 *
 *   PI  : Kp = T / (K (lambda + L)),          Ti = T
 *   PID : Kp = (T + L/2) / (K (lambda + L/2)), Ti = T + L/2,
 *         Td = T L / (2T + L)
 *
 * lambda is the desired closed-loop time constant and is the single knob for
 * the speed/robustness trade-off: lambda -> 0 is aggressive, large lambda is
 * sluggish but very robust. lambda >= 0.5*L is the usual robustness floor.
 */
static void pidt_imc(PID_Gains *g, const PID_PlantModel *m,
                     PID_TuneStructure s, PID_Float lambda)
{
    const PID_Float k = m->k;
    const PID_Float t = m->t;
    const PID_Float l = m->l;
    PID_Float lam = lambda;

    if (!(lam > (PID_Float)0.0)) {
        /* Default: the larger of the dead-time floor and a fifth of the
         * dominant time constant. Both are standard conservative choices. */
        const PID_Float a = (PID_Float)0.5 * l;
        const PID_Float b = (PID_Float)0.2 * t;
        lam = (a > b) ? a : b;
    }
    /* Robustness floor: a lambda below 0.2*L makes the controller depend on a
     * dead time estimate it cannot trust. */
    if (lam < (PID_Float)0.2 * l) {
        lam = (PID_Float)0.2 * l;
    }

    if (s == PID_STRUCT_P) {
        pidt_finish(g, t / (k * (lam + l)), (PID_Float)0.0, (PID_Float)0.0);
    } else if (s == PID_STRUCT_PI) {
        pidt_finish(g, t / (k * (lam + l)), t, (PID_Float)0.0);
    } else {
        const PID_Float half = (PID_Float)0.5 * l;
        const PID_Float kp   = (t + half) / (k * (lam + half));
        const PID_Float ti   = t + half;
        const PID_Float td   = t * l / ((PID_Float)2.0 * t + l);
        pidt_finish(g, kp, ti, td);
    }
}

/* ======================================================================== */
/* Public entry point                                                        */
/* ======================================================================== */

PID_StatusCode PID_TuneRule_Apply(PID_TuneRule rule,
                                  const PID_PlantModel *model,
                                  PID_TuneStructure structure,
                                  PID_Float lambda,
                                  PID_Gains *out)
{
    PID_ModelKind need;

    if ((model == NULL) || (out == NULL)) {
        return PID_ERR_NULL;
    }
    if ((int)rule < 0 || rule >= PID_RULE_COUNT_) {
        return PID_ERR_INVALID_PARAM;
    }
    if ((int)structure < 0 || structure > PID_STRUCT_PID) {
        return PID_ERR_INVALID_PARAM;
    }
    if (rule == PID_RULE_CUSTOM) {
        /* Dispatched by the tuner, not here. */
        return PID_ERR_INVALID_PARAM;
    }

    need = pidt_rule_model[rule];
    if (model->kind != need) {
        /* The central honesty check: a frequency point is not a FOPDT model
         * and no correct conversion between them exists. */
        return PID_ERR_TUNE_MODEL_MISMATCH;
    }

    /* Reject models that would divide by zero or produce nonsense gains. */
    if (need == PID_MODEL_FREQ) {
        if (!pidm_isfinite(model->ku) || !pidm_isfinite(model->pu)
            || (model->ku <= (PID_Float)0.0) || (model->pu <= (PID_Float)0.0)) {
            return PID_ERR_TUNE_VALIDATION;
        }
    } else {
        if (!pidm_isfinite(model->k) || !pidm_isfinite(model->t)
            || !pidm_isfinite(model->l)
            || (pidm_abs(model->k) <= (PID_Float)0.0)
            || (model->t <= (PID_Float)0.0)
            || (model->l <= (PID_Float)0.0)) {
            return PID_ERR_TUNE_VALIDATION;
        }
    }

    out->kp = (PID_Float)0.0;
    out->ki = (PID_Float)0.0;
    out->kd = (PID_Float)0.0;
    out->ti = (PID_Float)0.0;
    out->td = (PID_Float)0.0;
    out->tf = (PID_Float)0.0;

    switch (rule) {
    case PID_RULE_AMIGO_FREQ:
        pidt_amigo_freq(out, model, structure);
        break;
    case PID_RULE_COHEN_COON:
        pidt_cohen_coon(out, model, structure);
        break;
    case PID_RULE_AMIGO_STEP:
        pidt_amigo_step(out, model, structure);
        break;
    case PID_RULE_IMC:
        pidt_imc(out, model, structure, lambda);
        break;
    default: {
        /* Table-driven frequency rules. */
        const PID_FreqRuleRow *row = &pidt_freq_tab[rule];
        const PID_FreqCoef *c =
            (structure == PID_STRUCT_P)  ? &row->p  :
            (structure == PID_STRUCT_PI) ? &row->pi : &row->pid;
        pidt_finish(out, c->a * model->ku, c->b * model->pu, c->c * model->pu);
        break;
    }
    }

    if (!pidm_isfinite(out->kp) || !pidm_isfinite(out->ki)
        || !pidm_isfinite(out->kd)) {
        return PID_ERR_TUNE_VALIDATION;
    }
    return PID_OK;
}

#endif /* PIDX_ENABLE_AUTOTUNE */

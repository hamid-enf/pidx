function kind = ruleRequiredModel(rule)
%PIDX.RULEREQUIREDMODEL  Which plant model a tuning rule consumes.
%
%   KIND = PIDX.RULEREQUIREDMODEL(RULE) returns MODEL_FREQ for the
%   Ziegler-Nichols family (which needs Ku and Pu), MODEL_FOPDT for the rules
%   derived from G(s) = K*exp(-L*s)/(1+T*s), and MODEL_NONE for CUSTOM.
%
%   Use it before running an experiment: it tells you which identification
%   method - relay (FREQ) or open-loop step (FOPDT) - can actually feed the
%   rule you intend to use.

    K = pidx.Const;
    switch rule
        case {K.RULE_ZN, K.RULE_TYREUS_LUYBEN, K.RULE_PESSEN, ...
              K.RULE_SOME_OVERSHOOT, K.RULE_NO_OVERSHOOT, K.RULE_AMIGO_FREQ}
            kind = K.MODEL_FREQ;
        case {K.RULE_COHEN_COON, K.RULE_AMIGO_STEP, K.RULE_IMC}
            kind = K.MODEL_FOPDT;
        otherwise
            kind = K.MODEL_NONE;
    end
end

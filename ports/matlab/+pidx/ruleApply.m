function [rc, g] = ruleApply(rule, model, structure, lambda)
%PIDX.RULEAPPLY  Apply a closed-form tuning rule. Port of pid_autotune_rules.c.
%
%   [RC, G] = PIDX.RULEAPPLY(RULE, MODEL, STRUCTURE, LAMBDA)
%
%   A pure function from an identified plant model to controller gains: it
%   touches no controller, runs no experiment, and is therefore testable in
%   isolation against the published coefficients.
%
%   MODEL is a struct from PIDX.PLANTMODEL with fields kind, ku, pu, k, t, l.
%   G is a struct with fields kp, ki, kd, ti, td, tf.
%
%   Two families exist and they are NOT interchangeable:
%
%     FREQ rules consume (Ku, Pu) - one point on the Nyquist curve, where the
%     loop phase is -180 deg. Ziegler-Nichols and its descendants.
%
%     FOPDT rules consume (K, T, L) from G(s) = K*exp(-L*s)/(1+T*s).
%     Cohen-Coon, AMIGO-step and IMC. These cannot be evaluated from (Ku, Pu):
%     one complex number does not determine three real parameters. Asking for
%     such a pairing returns ERR_TUNE_MODEL_MISMATCH rather than inventing a
%     conversion.
%
%   Output convention is the parallel form the core uses:
%       u = Kp*e + Ki*integral(e) + Kd*de/dt,  Ki = Kp/Ti,  Kd = Kp*Td.
%   A rule with no integral action reports Ti = 0 and Ki = 0.
%
%   HOW TO CHOOSE. On an EXACT model, ZN and Cohen-Coon give the lowest IAE.
%   On a model that is 30% wrong - which is what a step test actually gives
%   you - that ordering nearly reverses (Spearman rho = -0.59 over 810 runs;
%   see sim/sim_robust.c). Share of cases that stayed stable and settled:
%   IMC 100%, NO_OVERSHOOT 99%, AMIGO_STEP 98%, AMIGO_FREQ 97%,
%   TYREUS_LUYBEN 86%, SOME_OVERSHOOT 78%, ZN 71%, COHEN_COON 67%,
%   PESSEN 67%. Prefer AMIGO_STEP unless you trust the model.

    K = pidx.Const;

    g = struct('kp', 0, 'ki', 0, 'kd', 0, 'ti', 0, 'td', 0, 'tf', 0);
    if nargin < 3, structure = K.STRUCT_PID; end
    if nargin < 4, lambda = 0.0; end

    if isempty(model)
        rc = K.ERR_NULL; return;
    end
    if rule < 0 || rule > K.RULE_CUSTOM
        rc = K.ERR_INVALID_PARAM; return;
    end
    if structure < 0 || structure > K.STRUCT_PID
        rc = K.ERR_INVALID_PARAM; return;
    end
    if rule == K.RULE_CUSTOM
        rc = K.ERR_INVALID_PARAM; return;   % dispatched by the tuner
    end

    need = pidx.ruleRequiredModel(rule);
    if model.kind ~= need
        % The central honesty check: a frequency point is not a FOPDT model
        % and no correct conversion between them exists.
        rc = K.ERR_TUNE_MODEL_MISMATCH; return;
    end

    if need == K.MODEL_FREQ
        if ~isfinite(model.ku) || ~isfinite(model.pu) || ...
           model.ku <= 0 || model.pu <= 0
            rc = K.ERR_TUNE_VALIDATION; return;
        end
    else
        if ~isfinite(model.k) || ~isfinite(model.t) || ~isfinite(model.l) ...
           || abs(model.k) <= 0 || model.t <= 0 || model.l <= 0
            rc = K.ERR_TUNE_VALIDATION; return;
        end
    end

    switch rule
        case K.RULE_AMIGO_FREQ
            % AMIGO in frequency-domain form (Astrom & Hagglund 2004). The
            % rule is expressed through the normalised gain kappa = 1/(Ku*K),
            % but with only (Ku,Pu) available the robust published
            % approximation is used - the coefficients AMIGO collapses to at
            % the design point Ms = 1.4 when the normalised dead time is
            % unknown. Deliberately conservative, which is the point of AMIGO
            % over ZN.
            if structure == K.STRUCT_P
                g = finish(0.20 * model.ku, 0, 0);
            elseif structure == K.STRUCT_PI
                g = finish(0.16 * model.ku, 0.46 * model.pu, 0);
            else
                g = finish(0.16 * model.ku, 0.46 * model.pu, 0.10 * model.pu);
            end

        case K.RULE_COHEN_COON
            % Cohen-Coon (1953), quarter-amplitude decay on dead-time
            % dominant plants. Valid for L/T roughly in [0.1, 1].
            tau = model.l / model.t;
            inv = 1.0 / (model.k * tau);
            if structure == K.STRUCT_P
                g = finish(inv * (1.0 + tau / 3.0), 0, 0);
            elseif structure == K.STRUCT_PI
                kp = inv * (0.9 + tau / 12.0);
                ti = model.l * (30.0 + 3.0 * tau) / (9.0 + 20.0 * tau);
                g = finish(kp, ti, 0);
            else
                kp = inv * (4.0 / 3.0 + tau / 4.0);
                ti = model.l * (32.0 + 6.0 * tau) / (13.0 + 8.0 * tau);
                td = model.l * 4.0 / (11.0 + 2.0 * tau);
                g = finish(kp, ti, td);
            end

        case K.RULE_AMIGO_STEP
            % AMIGO step rule (Astrom & Hagglund 2004), designed for maximum
            % sensitivity Ms = 1.4 - an explicit robustness target, unlike ZN
            % which has none.
            k = model.k; t = model.t; l = model.l;
            if structure == K.STRUCT_P
                % AMIGO defines no pure-P rule; the PI proportional part with
                % the integral removed is the conservative fallback.
                sm = l + t;
                kp = (0.15 + (0.35 - l * t / (sm * sm)) * t / l) / k;
                g = finish(kp, 0, 0);
            elseif structure == K.STRUCT_PI
                sm = l + t;
                kp = (0.15 + (0.35 - l * t / (sm * sm)) * t / l) / k;
                ti = 0.35 * l + 13.0 * l * t * t / ...
                     (t * t + 12.0 * l * t + 7.0 * l * l);
                g = finish(kp, ti, 0);
            else
                kp = (0.2 + 0.45 * t / l) / k;
                ti = (0.4 * l + 0.8 * t) / (l + 0.1 * t) * l;
                td = 0.5 * l * t / (0.3 * l + t);
                g = finish(kp, ti, td);
            end

        case K.RULE_IMC
            % IMC / lambda tuning (Rivera-Morari-Skogestad), FOPDT with a
            % first-order Pade approximation of the dead time. lambda is the
            % desired closed-loop time constant: the single knob for the
            % speed/robustness trade-off.
            k = model.k; t = model.t; l = model.l;
            lam = lambda;
            if ~(lam > 0)
                % Default: the larger of the dead-time floor and a fifth of
                % the dominant time constant. Both standard conservative
                % choices.
                a = 0.5 * l;
                b = 0.2 * t;
                if a > b, lam = a; else, lam = b; end
            end
            % Robustness floor: below 0.2*L the controller depends on a dead
            % time estimate it cannot trust.
            if lam < 0.2 * l
                lam = 0.2 * l;
            end
            if structure == K.STRUCT_P
                g = finish(t / (k * (lam + l)), 0, 0);
            elseif structure == K.STRUCT_PI
                g = finish(t / (k * (lam + l)), t, 0);
            else
                half = 0.5 * l;
                kp = (t + half) / (k * (lam + half));
                ti = t + half;
                td = t * l / (2.0 * t + l);
                g = finish(kp, ti, td);
            end

        otherwise
            % Table-driven frequency rules: Kp = a*Ku, Ti = b*Pu, Td = c*Pu.
            c = freqCoef(rule, structure);
            g = finish(c(1) * model.ku, c(2) * model.pu, c(3) * model.pu);
    end

    if ~isfinite(g.kp) || ~isfinite(g.ki) || ~isfinite(g.kd)
        rc = K.ERR_TUNE_VALIDATION; return;
    end
    rc = K.OK;
end

% ---------------------------------------------------------------------------

function g = finish(kp, ti, td)
    % Convert (Kp, Ti, Td) into the parallel form the core uses.
    g = struct('kp', kp, 'ki', 0, 'kd', 0, 'ti', ti, 'td', td, 'tf', 0);
    % Ti == 0 encodes "no integral action" - do not divide by it.
    if ti > 0
        g.ki = kp / ti;
    end
    g.kd = kp * td;
    % Derivative filter from the standard N = 10 rule: Tf = Td/N. Without a
    % filter the derivative term differentiates sensor noise without bound.
    g.tf = td * 0.1;
end

function c = freqCoef(rule, structure)
    % Published coefficient triples (a, b, c).
    %
    % ZN (1942, quarter-amplitude decay); Tyreus-Luyben (1992, robust on
    % lag-dominant processes); Pessen Integral Rule (faster than ZN, more
    % overshoot); "some/no overshoot" rows as tabulated by Astrom & Hagglund.
    %
    % WARNING - "no overshoot" is aspirational, not a guarantee. Those two
    % rows differ from ZN only in Kp; Ti stays pinned at Pu/2, and on FOPDT
    % plants that Ti is what produces the overshoot. Measured on K=2 T=1 L=0.1
    % with an EXACT model, NO_OVERSHOOT still overshoots 43%. Stretching Ti is
    % what fixes it (Ti = 4*Pu gives 0.0%). The coefficients are faithful to
    % the published table; the limitation is the rule's.
    K = pidx.Const;
    switch rule
        case K.RULE_ZN
            tab = [0.50 0.0      0.0
                   0.45 1.0/1.2  0.0
                   0.60 0.50     0.125];
        case K.RULE_TYREUS_LUYBEN
            tab = [0.50 0.0      0.0
                   0.31 2.20     0.0
                   0.45 2.20     1.0/6.3];
        case K.RULE_PESSEN
            tab = [0.50 0.0      0.0
                   0.45 1.0/1.2  0.0
                   0.70 0.40     0.15];
        case K.RULE_SOME_OVERSHOOT
            tab = [0.33 0.0      0.0
                   0.33 0.50     0.0
                   0.33 0.50     1.0/3.0];
        case K.RULE_NO_OVERSHOOT
            tab = [0.20 0.0      0.0
                   0.20 0.50     0.0
                   0.20 0.50     1.0/3.0];
        otherwise
            tab = zeros(3, 3);
    end
    c = tab(structure + 1, :);
end

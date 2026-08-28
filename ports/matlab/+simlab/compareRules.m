function r = compareRules(plant, opt)
%SIMLAB.COMPARERULES  Rank all nine tuning rules on YOUR plant, honestly.
%
%   R = SIMLAB.COMPARERULES(PLANT)
%   R = SIMLAB.COMPARERULES(PLANT, 'mode', 'robust', 'nRuns', 40)
%
% Every textbook ranks the tuning rules by how fast they respond. That ranking
% is measured against a perfect model, and you do not have a perfect model.
% This function gives you both rankings, side by side, on your own plant.
%
% THREE MODES
%   'exact'    the identified model IS the plant. Answers "how fast could
%              this be". This is the ranking every datasheet shows.
%   'robust'   the plant is perturbed around the model - K, tau and L each
%              over 0.5x..2x. Answers "how often will this work". Default.
%   'noisy'    exact model, but the sensor noise and ADC quantisation of the
%              plant are turned up. Answers "how much does D cost me".
%
% WHAT CAME OUT OF THE C STUDY (sim/sim_robust.c, 810 runs)
%   Ranking the nine rules by IAE against a perfect model correlates
%   rho = -0.586 with ranking them by survival when the plant is wrong.
%   Ziegler-Nichols is 1st on a perfect model and 7th when it is not; IMC is
%   9th and 1st. Low IAE is bought with stability margin.
%
%   Survival shares: IMC 100%, NO_OVERSHOOT 99%, AMIGO_STEP 98%,
%   AMIGO_FREQ 97%, TYREUS_LUYBEN 86%, SOME_OVERSHOOT 78%, ZN 71%,
%   COHEN_COON 67%, PESSEN 67%.
%
%   Run this and check whether YOUR plant agrees before you pick. With nine
%   rules a rank correlation cannot be significant, but the survival shares
%   rest on real runs and the sign is the finding.
%
% OPTIONS
%   'model'      a pidx.plantModel to tune from. Default: the true FOPDT
%                parameters of the plant, i.e. a perfect identification. Pass
%                the output of an actual auto-tune to see what YOUR data
%                buys you - that is usually the more interesting run.
%   'mode'       'exact' | 'robust' | 'noisy'. Default 'robust'.
%   'nRuns'      plants per rule in 'robust' mode. Default 30.
%   'structure'  STRUCT_P | STRUCT_PI | STRUCT_PID. Default PID.
%   'lambda'     IMC closed-loop time constant. Default: the rule's own.
%   'scenario'   a simlab.Scenario. Default presets('stepResponse').
%   'spread'     perturbation factor. Default 2.
%   'seed'       RNG seed. Default 1.
%   'verbose'    print the table as it fills. Default true.
%
% RESULT
%   R.table      one row per rule: name, model needed, ok, Kp, Ki, Kd, Ti,
%                Td, Tf, IAE median, overshoot median, settling median,
%                survival share, rank by IAE, rank by survival
%   R.spearman   rank correlation between the two rankings
%   R.best       the struct for the recommended rule
%   R.recommend  a sentence saying which to use and why

    if nargin < 2, opt = struct(); end
    K = pidx.Const;

    o = fillOpt(opt, 'model', []);
    o = fillOpt(o, 'mode', 'robust');
    o = fillOpt(o, 'nRuns', 30);
    o = fillOpt(o, 'structure', K.STRUCT_PID);
    o = fillOpt(o, 'lambda', 0);
    o = fillOpt(o, 'scenario', []);
    o = fillOpt(o, 'spread', 2.0);
    o = fillOpt(o, 'seed', 1);
    o = fillOpt(o, 'verbose', true);
    o = fillOpt(o, 'dt', []);

    rng(o.seed, 'twister');

    if isempty(o.model)
        % A perfect identification. Say so in the result, because "tuned from
        % the true parameters" is a different claim from "tuned from a step
        % test" and the difference is the whole point of the study.
            o.model = pidx.plantModel(K.MODEL_FOPDT, ...
            plant.steadyStateGain(), plant.tau(), plant.transportDelay());
        o.modelWasPerfect = true;
    else
        o.modelWasPerfect = false;
    end

    sc = o.scenario;
    if isempty(sc), sc = simlab.Scenario.presets('stepResponse'); end

    rules = [K.RULE_ZN, K.RULE_TYREUS_LUYBEN, K.RULE_PESSEN, ...
             K.RULE_SOME_OVERSHOOT, K.RULE_NO_OVERSHOOT, K.RULE_AMIGO_FREQ, ...
             K.RULE_COHEN_COON, K.RULE_AMIGO_STEP, K.RULE_IMC];
    nr = numel(rules);

    dt = o.dt;
    if isempty(dt), dt = plant.dt; end
    if isempty(dt) || ~(dt > 0), dt = 0.01; end

    tab = struct();
    tab.name = cell(nr, 1);
    tab.model = cell(nr, 1);
    tab.ok = false(nr, 1);
    tab.kp = nan(nr, 1); tab.ki = nan(nr, 1); tab.kd = nan(nr, 1);
    tab.ti = nan(nr, 1); tab.td = nan(nr, 1); tab.tf = nan(nr, 1);
    tab.iae = nan(nr, 1); tab.overshoot = nan(nr, 1);
    tab.settling = nan(nr, 1); tab.survival = nan(nr, 1);
    tab.note = cell(nr, 1);

    for i = 1:nr
        rule = rules(i);
        tab.name{i} = ruleName(rule);
        need = pidx.ruleRequiredModel(rule);
        if need == K.MODEL_FREQ
            tab.model{i} = 'FREQ';
        else
            tab.model{i} = 'FOPDT';
        end

        % A FREQ rule cannot be fed from a FOPDT identification. Rather than
        % invent a (K,T,L) -> (Ku,Pu) conversion, identify the plant with a
        % relay the way the target would, and use THAT. It costs a few seconds
        % and it is the honest number.
        model = o.model; %#ok<NASGU>
        if need == K.MODEL_FREQ && o.model.kind ~= K.MODEL_FREQ
            model = relayIdentify(plant, dt, o);
            if isempty(model)
                tab.note{i} = 'relay identification failed on this plant';
                continue;
            end
        elseif need == K.MODEL_FOPDT && o.model.kind ~= K.MODEL_FOPDT
            tab.note{i} = 'needs a FOPDT model (run a step test)';
            continue;
        end

        [rc, g] = pidx.ruleApply(rule, model, o.structure, o.lambda);
        if rc ~= K.OK
            tab.note{i} = K.statusToString(rc);
            continue;
        end
        tab.ok(i) = true;
        tab.kp(i) = g.kp; tab.ki(i) = g.ki; tab.kd(i) = g.kd;
        tab.ti(i) = g.ti; tab.td(i) = g.td; tab.tf(i) = g.tf;

        [iae, os, ts, surv] = evaluate(plant, g, sc, o, dt);
        tab.iae(i) = iae;
        tab.overshoot(i) = os;
        tab.settling(i) = ts;
        tab.survival(i) = surv;

        if o.verbose
            fprintf('  %-16s Kp=%-10.4g Ki=%-10.4g Kd=%-10.4g  IAE=%-10.4g OS=%-7.3g%% surv=%3.0f%%\n', ...
                    tab.name{i}, g.kp, g.ki, g.kd, iae, os, 100 * surv);
        end
    end

    % ---- the two rankings, and how much they disagree -------------------
    okIdx = find(tab.ok & ~isnan(tab.iae));
    [~, ordIae] = sort(tab.iae(okIdx));
    rankIae = nan(nr, 1);
    rankIae(okIdx(ordIae)) = 1:numel(okIdx);
    tab.rankIae = rankIae;

    okS = find(tab.ok & ~isnan(tab.survival));
    [~, ordS] = sort(-tab.survival(okS));
    rankS = nan(nr, 1);
    rankS(okS(ordS)) = 1:numel(okS);
    tab.rankSurvival = rankS;

    both = okIdx(~isnan(rankS(okIdx)));
    if numel(both) >= 3
        % Spearman = Pearson on the ranks. Computed here rather than through
        % corr() so the study needs no Statistics Toolbox.
        r.spearman = pearson(rankIae(both), rankS(both));
    else
        r.spearman = NaN;
    end
    r.rankCorrelationNote = ...
        ['Spearman rho between "fastest on a perfect model" and ' ...
         '"survives a wrong model". The C study over 810 runs got ' ...
         'rho = -0.586: the two orderings are close to opposite.'];

    % ---- recommendation --------------------------------------------------
    % Not "the lowest IAE". Lowest IAE on a perfect model is the trap the C
    % study documented. Require the rule to have survived, then take the
    % fastest survivor.
    cand = find(tab.ok & tab.survival >= 0.9 & ~isnan(tab.iae));
    if isempty(cand)
        cand = find(tab.ok & ~isnan(tab.iae));
    end
    if isempty(cand)
        r.best = [];
        r.recommend = 'no rule produced usable gains on this plant';
    else
        [~, j] = min(tab.iae(cand));
        bi = cand(j);
        r.best = struct('name', tab.name{bi}, 'kp', tab.kp(bi), ...
            'ki', tab.ki(bi), 'kd', tab.kd(bi), 'ti', tab.ti(bi), ...
            'td', tab.td(bi), 'tf', tab.tf(bi), 'iae', tab.iae(bi), ...
            'overshoot', tab.overshoot(bi), 'settling', tab.settling(bi), ...
            'survival', tab.survival(bi));
        r.recommend = sprintf( ...
            ['%s: fastest of the rules that survived >=90%% of the ' ...
             'perturbed plants (%.0f%%), IAE %.4g, overshoot %.1f%%. ' ...
             'A rule with a lower IAE exists in the table and is not ' ...
             'recommended - it did not survive.'], ...
            tab.name{bi}, 100 * tab.survival(bi), tab.iae(bi), ...
            tab.overshoot(bi));
    end

    r.table = tab;
    r.mode = o.mode;
    r.nRuns = o.nRuns;
    r.model = o.model;
    r.modelWasPerfect = o.modelWasPerfect;
    r.scenarioName = sc.name;
end

% ---------------------------------------------------------------------------

function [iae, os, ts, surv] = evaluate(plant, g, sc, o, dt)
% Run the rule's gains on this plant. In 'robust' mode over a cloud of
% perturbed plants, so the answer is a survival share and a median, not a
% single flattering number.
    if strcmp(o.mode, 'robust')
        % The gains carry the plant's actuator limits into every run: a rule
        % that only works when the output is unbounded is not a rule you can
        % ship, and windup behaviour is exactly what separates these rules.
        [lo, hi] = plant.actuatorLimits();
        gg = gainsToStruct(g, dt, lo, hi);
        mc = simlab.monteCarlo(plant, gg, ...
            struct('nRuns', o.nRuns, 'spread', o.spread, ...
                   'scenario', sc, 'seed', o.seed, 'verbose', false, ...
                   'dt', dt));
        surv = mc.share;
        t = mc.table(mc.stable, :);
        if isempty(t)
            iae = Inf; os = Inf; ts = Inf;
        else
            iae = median(t(:, 5));
            os = median(t(:, 6));
            ts = median(t(:, 7));
        end
    else
        pl = plant;
        if strcmp(o.mode, 'noisy')
            % Ten times the sensor noise and four bits off the converter.
            % Arbitrary factors, stated openly: the point is to see the ORDER
            % change, and a rule that only wins on a noiseless signal is not
            % a rule you can use.
            pl.degradeSensor(10, 4);
        end
        cfg = pidx.config('kp', g.kp, 'ki', g.ki, 'kd', g.kd, 'dt', dt);
        % Output limits come from the plant's actuator. Without them no rule
        % can ever saturate, and a comparison that cannot wind up is not
        % comparing the thing that actually separates these rules.
        [lo, hi] = pl.actuatorLimits();
        if isfinite(lo) && isfinite(hi)
            cfg.limits.use_output_limits = true;
            cfg.limits.output_min = lo;
            cfg.limits.output_max = hi;
        end
        c = pidx.PID(cfg);
        if g.tf > 0, c.setDerivativeFilter(g.tf); end
        res = simlab.Sim(pl, c, sc).run();
        m = res.metrics;
        iae = m.iae; os = m.overshoot; ts = m.settlingTime;
        surv = double(logical(m.stable));
    end
end

function g = gainsToStruct(g, dt, lo, hi)
    g.dt = dt;
    if isfinite(lo) && isfinite(hi)
        g.outMin = lo;
        g.outMax = hi;
    end
    if g.tf > 0
        g.tf = g.tf;   %#ok<*NASGU>  % kept so runOne() applies the filter
    end
end

function model = relayIdentify(plant, dt, o)
% Identify (Ku, Pu) with a relay, because that is what the target would do.
%
% Inventing a (K, tau, L) -> (Ku, Pu) conversion here would make the FREQ
% rules look better than they are: the conversion would be exact, and on the
% real plant it never is.
    K = pidx.Const;
    model = [];

    cfg = simlab.AutoTune.configDefault(K.IDENT_RELAY);
    cfg.rule = K.RULE_ZN;              % any FREQ rule; the model is what matters
    cfg.output_step = max(0.05 * plantActuatorSpan(plant), 1e-3);
    cfg.output_min = 0;
    cfg.output_max = 0;                % no clamp during identification
    cfg.timeout_s = 60 * max(1, plant.tau());
    cfg.skip_stabilize = true;

    at = simlab.AutoTune(cfg);
    c = pidx.PID(pidx.config('dt', dt));
    sp = plant.steadyStateGain() * cfg.output_step;
    at.start(c, sp);

    y = 0;
    nMax = round(cfg.timeout_s / dt);
    for k = 1:nMax
        u = at.update(y, dt);
        y = plant.update(u, dt);
        if ~at.isRunning()
            break;
        end
    end
    plant.reset();

    [rc, res] = at.getResult();
    if rc == K.OK
        model = res.model;
    end
end

function span = plantActuatorSpan(plant)
    [lo, hi] = plant.actuatorLimits();
    if isfinite(lo) && isfinite(hi)
        span = hi - lo;
    else
        span = 1;
    end
end

function s = ruleName(rule)
    names = {'ZN', 'TYREUS_LUYBEN', 'PESSEN', 'SOME_OVERSHOOT', ...
             'NO_OVERSHOOT', 'AMIGO_FREQ', 'COHEN_COON', 'AMIGO_STEP', ...
             'IMC', 'CUSTOM'};
    if rule >= 0 && rule < numel(names)
        s = names{rule + 1};
    else
        s = '?';
    end
end

function r = pearson(x, y)
    x = x(:) - mean(x(:));
    y = y(:) - mean(y(:));
    d = sqrt(sum(x.^2) * sum(y.^2));
    if d > 0
        r = sum(x .* y) / d;
    else
        r = NaN;
    end
end

function o = fillOpt(o, name, default)
    if isfield(opt, name) && ~isempty(opt.(name))
        o.(name) = opt.(name);
    else
        o.(name) = default;
    end
end

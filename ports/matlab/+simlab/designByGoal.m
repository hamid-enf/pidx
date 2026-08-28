function d = designByGoal(plant, goal, opt)
%SIMLAB.DESIGNBYGOAL  "Overshoot under 5%, as fast as possible" -> gains.
%
%   D = SIMLAB.DESIGNBYGOAL(PLANT, GOAL)
%   D = SIMLAB.DESIGNBYGOAL(PLANT, GOAL, 'model', m, 'dt', 0.01)
%
% A tuning rule gives you what a table says. This gives you what you asked
% for: a set of constraints and an objective, searched over the one-parameter
% family that actually spans the speed/robustness trade-off.
%
% GOAL - every field optional; set the ones that matter
%   .maxOvershoot     %              hard constraint
%   .maxSettling      s              hard constraint
%   .maxMs            -              hard constraint. Peak |S|; the single
%                                    best robustness number there is
%   .minPhaseMargin   deg            hard constraint
%   .maxSteadyError   eng units      hard constraint
%   .maxIAE           -              hard constraint
%   .minDelayMargin   s              hard constraint. How much MORE dead time
%                                    the loop survives - the number to compare
%                                    against how wrong your dead-time estimate is
%   .maxControlEffort eng units      hard constraint on mean |u|
%   .objective        'iae' | 'settling' | 'rise'. Default 'iae'
%
% HOW IT SEARCHES
%   The candidate family is IMC / lambda tuning, because lambda IS the
%   speed/robustness dial: raising it slows the loop and buys margin, lowering
%   it does the opposite. Sweeping it is not a hack - it is the whole design
%   space of a well-posed FOPDT loop, and every other rule is a particular
%   choice of lambda.
%
%   The sweep is geometric (0.05..4 times max(0.5L, 0.2T), 'nLambda' points)
%   because the useful range spans an order of magnitude and a linear sweep
%   would put almost all its points in the part that does not matter.
%
%   Two-stage evaluation, because simulation is the expensive part:
%     1. screen every candidate with simlab.sensitivity - pure arithmetic,
%        no simulation. Anything over maxMs or under minPhaseMargin is
%        rejected here and never simulated.
%     2. simulate the survivors and apply the time-domain constraints.
%
%   If 'searchBeta' is on, the overshoot constraint is additionally tried
%   against beta in {1.0, 0.8, 0.6, 0.4}. Lowering beta reduces SETPOINT
%   overshoot without touching disturbance rejection at all - that is the
%   entire point of 2DOF, and reaching for the integrator instead is the
%   classic way to trade away something you did not need to trade.
%
% WHAT IT RETURNS
%   D.gains        the chosen kp/ki/kd/tf/beta/lambda
%   D.feasible     false when nothing met every constraint
%   D.best         the chosen candidate's full evaluation
%   D.candidates   every candidate, with its metrics and which constraints it
%                  broke - so you can see the trade-off rather than take the
%                  answer on trust
%   D.diagnosis    plain language. When nothing is feasible it names the
%                  binding constraint and says what to relax, because
%                  "infeasible" with no reason is not an answer
%   D.model        the model the design is based on, and whether it was
%                  identified or assumed
%
% WHAT IT WILL NOT DO
%   It will not hand you an infeasible design and call it the best it could
%   do. If your overshoot and your settling time cannot both be met on this
%   plant, that is a fact about the plant, and the diagnosis says which of the
%   two is asking for more than the dead time allows.

    if nargin < 3, opt = struct(); end
    K = pidx.Const;

    o = fillOpt(opt, 'model', []);
    o = fillOpt(o, 'dt', []);
    o = fillOpt(o, 'scenario', []);
    o = fillOpt(o, 'structure', K.STRUCT_PID);
    o = fillOpt(o, 'nLambda', 25);
    o = fillOpt(o, 'searchBeta', true);
    o = fillOpt(o, 'verbose', true);

    dt = o.dt;
    if isempty(dt), dt = plant.dt; end
    if ~(dt > 0), dt = max(plant.tau() / 40, 1e-4); end

    % ---- the model ----
    model = o.model;
    if isempty(model)
        model = pidx.plantModel(K.MODEL_FOPDT, plant.steadyStateGain(), ...
            plant.tau(), plant.transportDelay());
        d.modelWasIdentified = false;
    else
        d.modelWasIdentified = true;
    end
    if model.kind ~= K.MODEL_FOPDT
        error('simlab:designByGoal:model', ...
              ['designByGoal sweeps the IMC lambda family, which needs a ' ...
               'FOPDT model. Identify one with simlab.identify or ' ...
               'simlab.AutoTune(IDENT_STEP) and pass it as ''model''.']);
    end
    d.model = model;

    sc = o.scenario;
    if isempty(sc)
        sc = simlab.Scenario.presets('stepResponse', ...
            'sp', defaultSp(plant), ...
            'tEnd', 12 * (model.t + model.l));
    end
    d.scenarioName = sc.name;

    % ---- the lambda sweep ----
    lam0 = max(0.5 * model.l, 0.2 * model.t);
    lambdas = lam0 * logspace(log10(0.05), log10(4.0), o.nLambda);
    betas = 1.0;
    if o.searchBeta && isfield(goal, 'maxOvershoot')
        betas = [1.0, 0.8, 0.6, 0.4];
    end

    [lo, hi] = plant.actuatorLimits();
    haveLimits = isfinite(lo) && isfinite(hi);

    cand = struct('lambda', {}, 'beta', {}, 'kp', {}, 'ki', {}, 'kd', {}, ...
        'tf', {}, 'Ms', {}, 'pm', {}, 'gm', {}, 'delayMargin', {}, ...
        'iae', {}, 'overshoot', {}, 'settling', {}, 'rise', {}, ...
        'ssError', {}, 'controlEffort', {}, 'violations', {}, ...
        'simulated', {});
    nCand = 0;
    nScreened = 0;

    for ib = 1:numel(betas)
        for il = 1:numel(lambdas)
            lam = lambdas(il);
            beta = betas(ib);

            [rc, g] = pidx.ruleApply(K.RULE_IMC, model, o.structure, lam);
            if rc ~= K.OK
                continue;
            end
            if ~isfinite(g.kp) || g.kp <= 0
                continue;
            end

            % ---- stage 1: cheap screen, no simulation ----
            try
                s = simlab.sensitivity(plant, struct('kp', g.kp, ...
                    'ki', g.ki, 'kd', g.kd), struct('dt', dt, ...
                    'tf', g.tf, 'beta', beta));
            catch
                continue;
            end

            v = {};
            if isfield(goal, 'maxMs') && s.Ms > goal.maxMs
                v{end + 1} = sprintf('Ms %.2f > %.2f', s.Ms, goal.maxMs); %#ok<AGROW>
            end
            if isfield(goal, 'minPhaseMargin') && (isnan(s.pm) || s.pm < goal.minPhaseMargin)
                v{end + 1} = sprintf('PM %.0f deg < %.0f deg', s.pm, goal.minPhaseMargin); %#ok<AGROW>
            end
            if isfield(goal, 'minDelayMargin') && (isnan(s.delayMargin) || s.delayMargin < goal.minDelayMargin)
                v{end + 1} = sprintf('delay margin %.3g s < %.3g s', s.delayMargin, goal.minDelayMargin); %#ok<AGROW>
            end
            if ~isempty(v)
                % Rejected before any simulation: that is what makes a 100
                % point sweep affordable.
                nScreened = nScreened + 1;
                nCand = nCand + 1;
                cand(nCand) = candidate(g, lam, beta, s, v, false); %#ok<AGROW>
                continue;
            end

            % ---- stage 2: simulate ----
            cfg = pidx.config('kp', g.kp, 'ki', g.ki, 'kd', g.kd, 'dt', dt);
            if g.tf > 0, cfg.filter.tf = g.tf; end
            cfg.weight.beta = beta;
            if haveLimits
                cfg.limits.use_output_limits = true;
                cfg.limits.output_min = lo;
                cfg.limits.output_max = hi;
            end
            try
                r = simlab.Sim(plant, pidx.PID(cfg), sc).run();
            catch
                continue;
            end
            m = r.metrics;

            v = {};
            if isfield(goal, 'maxOvershoot') && ~(m.overshoot <= goal.maxOvershoot)
                v{end + 1} = sprintf('overshoot %.1f%% > %.1f%%', m.overshoot, goal.maxOvershoot); %#ok<AGROW>
            end
            if isfield(goal, 'maxSettling') && ~(m.settlingTime <= goal.maxSettling)
                v{end + 1} = sprintf('settling %.4g s > %.4g s', m.settlingTime, goal.maxSettling); %#ok<AGROW>
            end
            if isfield(goal, 'maxIAE') && ~(m.iae <= goal.maxIAE)
                v{end + 1} = sprintf('IAE %.4g > %.4g', m.iae, goal.maxIAE); %#ok<AGROW>
            end
            if isfield(goal, 'maxSteadyError') && ~(abs(m.ssError) <= goal.maxSteadyError)
                v{end + 1} = sprintf('steady error %.4g > %.4g', abs(m.ssError), goal.maxSteadyError); %#ok<AGROW>
            end
            if isfield(goal, 'maxControlEffort') && ~(m.controlEffort <= goal.maxControlEffort)
                v{end + 1} = sprintf('mean |u| %.4g > %.4g', m.controlEffort, goal.maxControlEffort); %#ok<AGROW>
            end
            if ~logical(m.stable)
                v{end + 1} = 'the loop is not stable';
            end

            nCand = nCand + 1;
            c = candidate(g, lam, beta, s, v, true);
            c.iae = m.iae;
            c.overshoot = m.overshoot;
            c.settling = m.settlingTime;
            c.rise = m.riseTime;
            c.ssError = m.ssError;
            c.controlEffort = m.controlEffort;
            cand(nCand) = c; %#ok<AGROW>

            if o.verbose && mod(nCand, 10) == 0
                fprintf('  designByGoal: %d candidates, %d simulated, %d screened out\n', ...
                    nCand, nCand - nScreened, nScreened);
            end
        end
    end

    d.candidates = cand;
    d.nCandidates = nCand;
    d.nScreened = nScreened;
    d.nSimulated = nCand - nScreened;

    % ---- choose ----
    okIdx = [];
    for i = 1:numel(cand)
        if cand(i).simulated && isempty(cand(i).violations)
            okIdx(end + 1) = i; %#ok<AGROW>
        end
    end
    d.feasible = ~isempty(okIdx);

    obj = 'iae';
    if isfield(goal, 'objective') && ~isempty(goal.objective)
        obj = lower(goal.objective);
    end
    d.objective = obj;

    if d.feasible
        switch obj
            case 'settling'
                vals = [cand(okIdx).settling];
            case 'rise'
                vals = [cand(okIdx).rise];
            otherwise
                vals = [cand(okIdx).iae];
        end
        % A NaN (a metric the scenario could not measure) must not win by
        % being smallest, which is what a naive min() would do.
        vals(isnan(vals)) = inf;
        [~, j] = min(vals);
        bi = okIdx(j);
        d.best = cand(bi);
        d.gains = struct('kp', cand(bi).kp, 'ki', cand(bi).ki, ...
            'kd', cand(bi).kd, 'tf', cand(bi).tf, 'beta', cand(bi).beta, ...
            'lambda', cand(bi).lambda);
        d.diagnosis = sprintf( ...
            ['feasible. %s: lambda = %.5g s (beta = %.2f) gives %s %.4g ' ...
             'with overshoot %.1f%%, settling %.4g s, Ms %.2f and a delay ' ...
             'margin of %.4g s. %d of %d candidates met every constraint.'], ...
            obj, cand(bi).lambda, cand(bi).beta, obj, vals(j), ...
            cand(bi).overshoot, cand(bi).settling, cand(bi).Ms, ...
            cand(bi).delayMargin, numel(okIdx), numel(cand));
    else
        d.best = [];
        d.gains = closest(cand, goal, obj);
        d.diagnosis = infeasibleDiagnosis(cand, goal, model, obj);
    end

    if o.verbose
        printDesign(d);
    end
end

% ---------------------------------------------------------------------------

function c = candidate(g, lam, beta, s, v, simulated)
    c = struct();
    c.lambda = lam;
    c.beta = beta;
    c.kp = g.kp; c.ki = g.ki; c.kd = g.kd; c.tf = g.tf;
    c.Ms = s.Ms;
    c.pm = s.pm;
    c.gm = s.gm;
    c.delayMargin = s.delayMargin;
    c.iae = NaN; c.overshoot = NaN; c.settling = NaN; c.rise = NaN;
    c.ssError = NaN; c.controlEffort = NaN;
    c.violations = v;
    c.simulated = simulated;
end

function g = closest(cand, goal, obj)
% Nothing met every constraint. Return the candidate that broke the fewest,
% and among those the best on the objective - labelled, never silently.
    best = [];
    for i = 1:numel(cand)
        if ~cand(i).simulated
            continue;
        end
        nv = numel(cand(i).violations);
        if isempty(best) || nv < best.nViolations
            best = cand(i);
            best.nViolations = nv;
        end
    end
    if isempty(best)
        g = [];
        return;
    end
    g = struct('kp', best.kp, 'ki', best.ki, 'kd', best.kd, ...
        'tf', best.tf, 'beta', best.beta, 'lambda', best.lambda);
    %#ok<NASGU>
    goal; obj;
end

function s = infeasibleDiagnosis(cand, goal, model, obj)
% Name the binding constraint. "Infeasible" on its own is not an answer;
% "your settling time asks for a bandwidth the dead time does not allow" is.
    tally = struct();
    nSim = 0;
    for i = 1:numel(cand)
        if ~cand(i).simulated
            continue;
        end
        nSim = nSim + 1;
        for j = 1:numel(cand(i).violations)
            v = cand(i).violations{j};
            % The first word of the violation text is the constraint name.
            % keyName rather than matlab.lang.makeValidName, which is R2014a+
            % and absent from some Octave builds.
            key = simlab_tests.keyName(strtok(v, ' '));
            if isfield(tally, key)
                tally.(key) = tally.(key) + 1;
            else
                tally.(key) = 1;
            end
        end
    end

    bits = {};
    bits{end + 1} = sprintf( ...
        'INFEASIBLE: no candidate met every constraint (%d simulated).', nSim);

    % Which constraint bit hardest?
    fn = fieldnames(tally);
    if ~isempty(fn)
        counts = zeros(numel(fn), 1);
        for i = 1:numel(fn)
            counts(i) = tally.(fn{i});
        end
        [~, j] = max(counts);
        bits{end + 1} = sprintf('binding constraint: %s (%d of %d candidates).', ...
            fn{j}, counts(j), nSim);
    end

    % What the plant actually allows. This is the part that turns a refusal
    % into something the user can act on.
    if isfield(goal, 'maxSettling')
        % A loop with dead time L cannot settle much faster than a few times
        % L, no matter the gains - the information about the step has not
        % arrived yet.
        floorSettle = 3 * model.l;
        if goal.maxSettling < floorSettle
            bits{end + 1} = sprintf( ...
                ['the requested settling time %.4g s is below about 3*L = ' ...
                 '%.4g s. The dead time is %.4g s: the loop cannot correct ' ...
                 'for an error it has not seen yet. Relax the settling time ' ...
                 'or reduce the dead time physically.'], ...
                goal.maxSettling, floorSettle, model.l);
        end
    end
    if isfield(goal, 'maxOvershoot') && goal.maxOvershoot < 1
        bits{end + 1} = sprintf( ...
            ['overshoot below %.1f%% with L/T = %.2f needs beta well below 1 ' ...
             '(the 2DOF setpoint weight) or a slower loop. If both were ' ...
             'already searched, the plant is the limit.'], ...
            goal.maxOvershoot, model.l / model.t);
    end
    if isfield(goal, 'minDelayMargin') && ...
       goal.minDelayMargin > 0.5 * model.l
        bits{end + 1} = sprintf( ...
            ['a delay margin of %.4g s is more than half the plant''s own ' ...
             'dead time (%.4g s). That is a demanding robustness target; ' ...
             'expect to pay for it in speed.'], goal.minDelayMargin, model.l);
    end
    bits{end + 1} = 'The closest candidate is returned in .gains, labelled.';

    s = strjoin(bits, sprintf('\n'));
    %#ok<NASGU>
    obj;
end

function printDesign(d)
    fprintf('\n================ design by goal ================\n');
    fprintf('  model: K = %.5g, T = %.5g s, L = %.5g s  (%s)\n', ...
        d.model.k, d.model.t, d.model.l, ...
        tern(d.modelWasIdentified, 'identified', 'ASSUMED from the plant'));
    fprintf('  candidates: %d total, %d screened out on margins, %d simulated\n', ...
        d.nCandidates, d.nScreened, d.nSimulated);
    if d.feasible
        fprintf('  chosen: Kp = %.6g, Ki = %.6g, Kd = %.6g, Tf = %.6g, beta = %.2f\n', ...
            d.gains.kp, d.gains.ki, d.gains.kd, d.gains.tf, d.gains.beta);
        fprintf('          lambda = %.6g s\n', d.gains.lambda);
        fprintf('  %s\n', d.diagnosis);
    else
        fprintf('  %s\n', d.diagnosis);
    end
    fprintf('\n  %-10s %-6s %-10s %-10s %-7s %-9s %-7s %-6s\n', ...
        'lambda', 'beta', 'Kp', 'Ki', 'OS %', 'settle', 'Ms', 'PM');
    show = d.candidates;
    if numel(show) > 12
        % Show the ends of the sweep: the aggressive end and the robust end
        % are where the interesting trade-off is, and the middle is monotone.
        idx = [1:min(6, numel(show)), max(1, numel(show) - 5):numel(show)];
        show = show(unique(idx));
    end
    for i = 1:numel(show)
        c = show(i);
        if ~c.simulated
            fprintf('  %-10.4g %-6.2f %-10.4g %-10.4g  (screened out: %s)\n', ...
                c.lambda, c.beta, c.kp, c.ki, strjoin(c.violations, ', '));
            continue;
        end
        mark = '';
        if ~isempty(c.violations)
            mark = '  x';
        end
        fprintf('  %-10.4g %-6.2f %-10.4g %-10.4g %-7.2f %-9.4g %-7.2f %-6.0f%s\n', ...
            c.lambda, c.beta, c.kp, c.ki, c.overshoot, c.settling, ...
            c.Ms, c.pm, mark);
    end
    fprintf('================================================\n\n');
end

function sp = defaultSp(plant)
% Half the actuator span mapped through the plant gain: a step that is large
% enough to be meaningful and small enough not to spend the whole run
% saturated.
    [lo, hi] = plant.actuatorLimits();
    if ~isfinite(lo), lo = 0; end
    if ~isfinite(hi), hi = 1; end
    sp = 0.5 * (hi - lo) * plant.steadyStateGain();
    if ~isfinite(sp) || sp == 0
        sp = 1;
    end
end

function s = tern(c, a, b)
    if c, s = a; else, s = b; end
end

function o = fillOpt(o, name, default)
    if ~isfield(o, name) || isempty(o.(name))
        o.(name) = default;
    end
end

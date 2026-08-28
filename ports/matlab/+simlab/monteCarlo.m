function r = monteCarlo(plant, gains, opt)
%SIMLAB.MONTECARLO  How often does this tuning survive a plant that is wrong?
%
%   R = SIMLAB.MONTECARLO(PLANT, GAINS)
%   R = SIMLAB.MONTECARLO(PLANT, GAINS, 'nRuns', 100, 'spread', 2)
%
% A step response against a perfect model is a number with no error bars. The
% model came from an identification, and identifications are 10-30% wrong.
% This study asks the only question that decides whether the gains are safe:
% over a cloud of plants around the nominal one, how many still work?
%
% WHAT IS PERTURBED
%   Each run rebuilds the plant with
%       K  <- K  * f1,   tau <- tau * f2,   L <- L * f3
%   where the factors are drawn from 10^U with U uniform on
%   [-log10(spread), +log10(spread)]. 'spread' = 2 therefore means each
%   parameter ranges over 0.5x .. 2x, which brackets what a relay or step
%   identification actually delivers.
%
% This is the same design as sim/sim_robust.c, which is where the finding in
% README.md comes from: ranking nine rules by IAE against a PERFECT model
% correlates rho = -0.586 with ranking them by survival when the plant is
% wrong. Ziegler-Nichols is 1st on a perfect model and 7th when it is not.
% Reproduce it here with simlab.compareRules before trusting any rule.
%
% OPTIONS
%   'nRuns'    number of plants. Default 60 - enough for a share to be
%              meaningful, small enough to finish while you watch.
%   'spread'   parameter range as a factor. Default 2.
%   'scenario' a simlab.Scenario. Default presets('disturbance').
%   'gainsSpread' also perturb the controller by this factor (default 1 =
%              do not). Use it to ask "what if I detune by 30%".
%   'seed'     RNG seed. Default 1, so the study repeats exactly.
%   'verbose'  print progress. Default true.
%
% RESULT
%   R.table    one row per run: [f1 f2 f3 stable iae overshoot settling]
%   R.share    fraction stable and settled
%   R.iae      median / worst / IQR of IAE over the stable runs
%   R.worst    the row that was worst, for re-running and plotting
%   R.nominal  the same run at the nominal plant, for reference

    if nargin < 3, opt = struct(); end
    o = fillOpt(opt, 'nRuns', 60);
    o = fillOpt(o, 'spread', 2.0);
    o = fillOpt(o, 'scenario', []);
    o = fillOpt(o, 'gainsSpread', 1.0);
    o = fillOpt(o, 'seed', 1);
    o = fillOpt(o, 'verbose', true);
    o = fillOpt(o, 'dt', []);

    rng(o.seed, 'twister');

    sc = o.scenario;
    if isempty(sc)
        sc = simlab.Scenario.presets('disturbance');
    end
    n = o.nRuns;
    tab = nan(n, 7);
    logs = cell(n, 1);

    lo = -log10(o.spread);
    hi = -lo;

    for i = 1:n
        f = 10 .^ (lo + (hi - lo) * rand(1, 3));
        pl = perturb(plant, f);

        g = gains;
        if o.gainsSpread > 1
            fg = 10 .^ (-log10(o.gainsSpread) + ...
                        2 * log10(o.gainsSpread) * rand(1, 3));
            g.kp = gains.kp * fg(1);
            g.ki = gains.ki * fg(2);
            g.kd = gains.kd * fg(3);
        end

        res = runOne(pl, g, sc, o.dt);
        m = res.metrics;
        tab(i, :) = [f, m.stable, m.iae, m.overshoot, m.settlingTime];
        logs{i} = res;

        if o.verbose && mod(i, max(1, floor(n / 10))) == 0
            fprintf('  monteCarlo: %d/%d runs, %d%% survived so far\n', ...
                    i, n, round(100 * mean(tab(1:i, 4))));
        end
    end

    r = struct();
    r.table = tab;
    r.factors = tab(:, 1:3);
    r.stable = logical(tab(:, 4));
    r.share = mean(r.stable);
    r.nRuns = n;
    r.spread = o.spread;
    r.scenarioName = sc.name;
    r.logs = logs;

    iae = tab(r.stable, 5);
    if isempty(iae)
        r.iae.median = NaN; r.iae.worst = NaN; r.iae.iqr = NaN;
    else
        r.iae.median = median(iae);
        r.iae.worst = max(iae);
        r.iae.iqr = iqr(iae);
    end

    [~, iw] = max(tab(:, 5));
    r.worstRow = tab(iw, :);
    r.worstPlant = tab(iw, 1:3);
    r.worstLog = logs{iw};

    r.nominal = runOne(plant, gains, sc, o.dt).metrics;

    if o.verbose
        fprintf(['  monteCarlo: %.0f%% of %d plants stable; median IAE ' ...
                 '%.4g, worst %.4g (at K x%.2f, tau x%.2f, L x%.2f)\n'], ...
                100 * r.share, n, r.iae.median, r.iae.worst, ...
                r.worstPlant(1), r.worstPlant(2), r.worstPlant(3));
    end
end

% ---------------------------------------------------------------------------

function o = fillOpt(o, name, default)
    if ~isfield(o, name) || isempty(o.(name))
        o.(name) = default;
    end
end

function pl = perturb(plant, f)
% Rebuild the plant with scaled K, tau, L.
%
% Rebuilt rather than mutated: a plant that has been stepped carries state,
% and a study whose runs are not independent is not a study.
    switch plant.kind
        case 'fopdt'
            pl = simlab.Plant('fopdt', 'name', plant.name, ...
                'k', plant.modelParam('k') * f(1), ...
                'tau', plant.tau() * f(2), ...
                'l', plant.transportDelay() * f(3));
        case 'dc_motor'
            % Scaling a motor's "K, tau" means scaling the speed gain and the
            % mechanical time constant. Kt moves the gain, J moves tau_m, and
            % both leave the electrical pole L/R alone - which is right,
            % because the electrical dynamics are not what a speed-loop
            % identification is uncertain about.
            pl = simlab.Plant('dc_motor', 'name', plant.name, ...
                'r', plant.modelParam('r'), ...
                'ind', plant.modelParam('l_elec'), ...
                'ke', plant.modelParam('ke'), ...
                'kt', plant.modelParam('kt') * f(1), ...
                'j', plant.modelParam('j') * f(2), ...
                'b', plant.modelParam('b'), ...
                'coulomb', plant.modelParam('coulomb'), ...
                'l', plant.transportDelay() * f(3));
        case 'linear'
            % Scale the DC gain by f(1) and stretch the time axis by f(2),
            % which moves every pole by 1/f(2). Delay scales by f(3).
            pl = simlab.Plant('linear', 'name', plant.name, ...
                'num', plant.numerator() * f(1), ...
                'den', stretchDen(plant.denominator(), f(2)), ...
                'l', plant.transportDelay() * f(3));
        otherwise
            error('simlab:monteCarlo:custom', ...
                  ['monteCarlo cannot perturb a ''custom'' plant: it has ' ...
                   'no declared parameters to scale. Perturb it yourself ' ...
                   'from a ''custom'' event in the scenario.']);
    end
    % The sensor and actuator chains are copied so every run sees the same
    % hardware; only the process itself moves.
    pl = copyChains(plant, pl);
end

function den = stretchDen(den, factor)
% Multiply the polynomial's time axis: p(s) -> p(s/factor).
%
% For a denominator [a2 a1 a0] of s^2 this is [a2/f^2, a1/f, a0], i.e. the
% poles move by 1/factor. Scaling each coefficient by factor^(degree - index)
% is the same statement and needs no roots().
    n = numel(den);
    for i = 1:n
        den(i) = den(i) / factor^(n - i);
    end
end

function pl = copyChains(src, dst)
% Re-apply the sensor and actuator configuration of SRC onto DST.
%
% dst is a handle object, so the setters below mutate it in place; but the
% caller writes `pl = copyChains(...)` and MATLAB requires the output to be
% assigned on EVERY path - the missing assignment was a runtime error the
% first time Monte Carlo ran under real MATLAB.
    pl = dst;
    [lo, hi] = src.actuatorLimits();
    if isfinite(lo) || isfinite(hi)
        dst.setActuatorLimits(lo, hi);
    end
    dst.setActuatorDeadband(src.actuatorParam('deadband'));
    dst.setActuatorSlew(src.actuatorParam('slew'));
    dst.setActuatorQuantisation(src.actuatorParam('quant'));
    dst.setActuatorFn(src.actuatorParam('fn'));
    dst.setNoise(src.sensorParam('sigma'));
    b = src.sensorParam('bits');
    if b > 0
        dst.setAdcBits(b, src.sensorParam('qmin'), src.sensorParam('qmax'));
    end
    dst.setSensorGainBias(src.sensorParam('gain'), src.sensorParam('bias'));
    dst.setSensorDeadband(src.sensorParam('deadband'));
    dst.setSensorRateLimit(src.sensorParam('rate'));
    dst.setSensorDelay(src.sensorParam('delay'));
    dst.setDropout(src.sensorParam('dropout'));
end


function m = runOne(pl, g, sc, dt)
    cfg = pidx.config('kp', g.kp, 'ki', g.ki, 'kd', g.kd);
    if isfield(g, 'dt') && ~isempty(g.dt)
        cfg.core.sample_time = g.dt;
    end
    if dt > 0
        cfg.core.sample_time = dt;
    end
    if isfield(g, 'outMin') && isfield(g, 'outMax')
        cfg.limits.use_output_limits = true;
        cfg.limits.output_min = g.outMin;
        cfg.limits.output_max = g.outMax;
    end
    if isfield(g, 'awMode')
        cfg.integral.mode = g.awMode;
    end
    if isfield(g, 'tf') && g.tf > 0
        cfg.filter.tf = g.tf;
    end
    if isfield(g, 'beta')
        cfg.weight.beta = g.beta;
    end

    c = pidx.PID(cfg);
    if isfield(g, 'tf') && g.tf > 0
        c.setDerivativeFilter(g.tf);
    end
    res = simlab.Sim(pl, c, sc).run();
    res.metrics.stable = logical(res.metrics.stable);
end

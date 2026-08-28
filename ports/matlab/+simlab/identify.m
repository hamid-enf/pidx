function m = identify(data, opt)
%SIMLAB.IDENTIFY  Fit a FOPDT model to a measured open-loop step response.
%
%   M = SIMLAB.IDENTIFY(DATA)
%   M = SIMLAB.IDENTIFY(DATA, 'uStep', 30, 'u0', 20, 'settleBand', 0.005)
%
%   DATA is a struct with fields .t, .y and optionally .u (column or row
%   vectors, uniformly sampled - check .dt). Or a matrix [t y] / [t y u].
%   Or a CSV file name, read by simlab.readStepData.
%
% THE METHOD IS THE ONE THE TARGET RUNS
%   Area and moment integration, exactly as pidt_analyze_step() in
%   src/pid_autotune.c. For G(s) = K exp(-Ls)/(1+Ts) the first two moments of
%   the residual e(t) = y_inf - y(t) of the normalised response are
%
%       A1 = integral e(t) dt       = L + T
%       M1 = integral t*e(t) dt     = L^2/2 + L*T + T^2
%
%   which inverts in closed form:  T = sqrt(2*M1 - A1^2),  L = A1 - T.
%
%   The area method is used in preference to the classical two-point
%   (28.3%/63.2%) fit because both integrals average over the WHOLE transient:
%   a single noisy sample cannot move the answer, whereas a two-point fit
%   reads the model off exactly two samples and needs a final-value estimate
%   that is already accurate while the response is still moving.
%
%   The moment arm is evaluated at the interval MIDPOINT (t - dt/2). This is
%   not cosmetic: the analytic subtraction uses integral of t dt = te^2/2, and
%   only the midpoint sum matches that exactly. Sampling at the right endpoint
%   instead leaves the first moment short by te*dt/2 per unit of dy - a
%   deficit that GROWS with test length, so a longer and more careful
%   experiment would produce a WORSE model.
%
% WHY YOU WOULD RUN THIS INSTEAD OF THE AUTO-TUNER
%   You already have the data. Maybe the process cannot be disturbed again,
%   maybe the step test was run last month by someone else, maybe you have a
%   CSV from the SCADA historian. This gives you the same model the tuner
%   would have produced, from data you already paid for.
%
% FIELDS OF M (a pidx.plantModel, so every tuning rule accepts it)
%   .k, .t, .l        the FOPDT parameters
%   .quality          0..100, the same score the C computes
%   .noise_sigma      estimated measurement noise
%   .fit              struct with the fit diagnostics:
%       .a1, .m1      the two moments, normalised
%       .tEnd         the time the response was declared settled
%       .y0, .yInf    the levels the fit is measured between
%       .t632         the 63.2% crossing, for cross-checking against L+T
%       .covered      tEnd/(L+T) - below 5 the fit is extrapolating
%       .residual     the fit error over the whole transient
%   .warnings         cell array of plain-language problems
%
% WHAT THIS WILL NOT DO
%   It will not invent a model from a response that is not first order. If the
%   moments are inconsistent with a FOPDT - a second-order or an integrating
%   process, a response truncated mid-rise - the radicand goes negative and
%   the fit is REFUSED with a warning naming the likely cause, rather than
%   returning a plausible-looking model that will detune your loop.

    if nargin < 2, opt = struct(); end
    o = fillOpt(opt, 'uStep', []);
    o = fillOpt(o, 'u0', []);
    m = pidx.plantModel(pidx.Const.MODEL_FOPDT);
    m.quality = uint8(0);

    if ischar(data)
        data = simlab.readStepData(data);
    end
    [t, y, u, dt] = unpack(data);

    w = {};
    n = numel(t);
    if n < 20
        error('simlab:identify:tooShort', ...
              '%d samples is not a step response', n);
    end
    if dt <= 0
        error('simlab:identify:badDt', 'sample interval must be > 0');
    end

    % ---- the step amplitude ----
    %
    % From the command if it was logged AND it actually moves. A historian
    % export often starts AFTER the step, in which case u is a constant and
    % its span is zero - the amplitude must then come from 'uStep'. Refusing
    % to guess when neither is available is the point: K = dy/du, and a du
    % taken from the wrong place scales every gain the rules produce.
    duU = [];
    if ~isempty(u)
        duU = max(u) - min(u);
        if duU <= 0
            duU = [];
        end
    end
    if ~isempty(duU)
        du = duU;
        if ~isempty(o.uStep) && abs(abs(o.uStep) - du) > 0.02 * du
            w{end + 1} = sprintf( ...
                ['the logged command spans %.6g but ''uStep'' says %.6g. ' ...
                 'Using the logged command.'], du, o.uStep);
        end
    elseif ~isempty(o.uStep)
        du = abs(o.uStep);
    else
        du = [];
    end
    if isempty(du)
        error('simlab:identify:noStep', ...
              ['no step amplitude: the data has no moving .u column and ' ...
               '''uStep'' was not given. K = dy/du needs du.']);
    end

    % ---- step time ----
    % The moment integrals must start at the STEP instant, not at the
    % departure: the residual (y_inf - y) is full sized during the dead time,
    % and that flat stretch IS the dead time the fit solves for. Start them
    % at the departure instead and A1 comes up short by ~L, which is exactly
    % the failure the first real test run showed (L = 0.5 against 12).
    % With a logged command the step instant is known exactly; without one
    % only the departure is available, and L then comes out short - say so.
    if ~isempty(u) && (max(u) - min(u)) > 0
        ddu = find(abs(diff(u)) > 1e-12 * max(1, max(abs(u))), 1, 'first');
        if isempty(ddu)
            kStep = 1;
        else
            kStep = ddu;               % last sample before the step
        end
    else
        kStep = [];
        w{end + 1} = ['no logged command: the step instant is estimated as ' ...
            'the response departure, so the dead time comes out short by the ' ...
            'detection lag. Log u for an exact L.'];
    end

    % ---- pre-step baseline ----
    %
    % Averaged over the samples before the response starts to move, not taken
    % from the first sample: a single noisy sample would shift y0, and y0
    % appears in the denominator of the normalisation.
    kMove = findMoving(t, y);
    if kMove < 3
        % No quiet run-up to average over. Two samples is the most that
        % exists, and saying so is better than quietly using one.
        y0 = mean(y(1:min(2, numel(y))));
        w{end + 1} = ['the response starts moving in the first two samples, ' ...
            'so y0 is averaged over just those. A step test with a quiet ' ...
            'run-up gives a better baseline.'];
    else
        y0 = mean(y(1:kMove - 1));
    end
    if ~isempty(kStep)
        if kStep > 2
            y0 = mean(y(1:kStep - 1));
        else
            y0 = y(1);
        end
    end

    % ---- settling: the same criterion the C uses ----
    %
    % Within settleBand of the running final-value estimate AND flat, held for
    % a quarter of the elapsed test time. Both thresholds are floored at the
    % measured noise, because a threshold tighter than the noise can never be
    % met and the only possible outcome would be a timeout.
    sigma = noiseEstimate(y);
    ySettled = y(end);
    ySlow = y(1);
    ySlowPrev = y(1);
    settleTimer = 0;
    yAcc = 0;
    yAccN = 0;
    kEnd = n;

    for k = 2:n
        elapsed = t(k) - t(kMove);
        if elapsed <= 0
            ySlowPrev = y(k);
            ySlow = y(k);
            continue;
        end

        % Running final-value estimate, time constant a fifth of the elapsed
        % test time so it adapts to the plant.
        tau = elapsed * 0.2;
        a = tau / (tau + dt);
        ySettled = a * ySettled + (1 - a) * y(k);

        % Flatness on a FILTERED signal. On raw data, (y[k]-y[k-1])/dt is
        % dominated by noise: at 1% noise and dt = 10 ms the apparent slope is
        % order 1 unit/s even when the process is perfectly still.
        tauS = elapsed * 0.05;
        as = tauS / (tauS + dt);
        ySlowPrev = ySlow;
        ySlow = as * ySlow + (1 - as) * y(k);

        slope = abs(ySlow - ySlowPrev) / dt;
        total = abs(ySettled - y0);
        remaining = abs(ySettled - ySlow);
        slopeThr = max(total * 0.001, 1e-9);
        nearThr = total * o_band(opt);
        jitter = (1 - as) * sigma / dt;
        slopeThr = max(slopeThr, 3 * jitter);
        nearThr = max(nearThr, 2 * sigma);

        if total > 0 && slope < slopeThr && remaining < nearThr
            settleTimer = settleTimer + dt;
            yAcc = yAcc + y(k);
            yAccN = yAccN + 1;
        else
            settleTimer = 0;
            yAcc = 0;
            yAccN = 0;
        end

        % Do not declare victory before the process has reacted: right after
        % the step both relative thresholds are tiny and noise alone can
        % satisfy them.
        if abs(ySlow - y0) < 10 * sigma
            settleTimer = 0;
            yAcc = 0;
            yAccN = 0;
        end

        if settleTimer > 0.25 * elapsed && settleTimer > 20 * dt && yAccN > 0
            % Commit the settle-window MEAN as the final value. Not the
            % lagging low-pass estimate: A1 = te - area/dy is far more
            % sensitive to dy than to the integrals themselves.
            ySettled = yAcc / yAccN;
            kEnd = k;
            break;
        end
    end

    if kEnd >= n
        w{end + 1} = ['the response never satisfied the settling criterion ' ...
            'within the record. The fit uses the whole trace, so a tail that ' ...
            'is still moving biases T low and L high. Record longer.'];
    end

    dy = ySettled - y0;
    if abs(dy) < 5 * sigma
        error('simlab:identify:noResponse', ...
              ['the response moves by %.6g, which is within %.0f sigma of ' ...
               'the noise (%.6g). There is no step response here to fit.'], ...
              dy, 5, sigma);
    end

    k = dy / du;

    % ---- the two moments, midpoint arm, over the settled window ----
    % Anchored at the step instant when it is known. The settling loop above
    % is anchored at the departure, which is the right clock for flatness;
    % the moments need the step clock, or the dead time never enters A1.
    if isempty(kStep)
        kRef = kMove;
    else
        kRef = kStep;
    end
    te = t(kEnd) - t(kRef);
    area1 = 0;
    moment1 = 0;
    for kk = (kRef + 1):kEnd
        yAvg = 0.5 * ((y(kk) - y0) + (y(kk - 1) - y0));
        tMid = (t(kk) - t(kRef)) - 0.5 * dt;
        area1 = area1 + yAvg * dt;
        moment1 = moment1 + tMid * yAvg * dt;
    end

    a1 = (dy * te - area1) / dy;
    m1 = (dy * te * te * 0.5 - moment1) / dy;
    rad = 2 * m1 - a1 * a1;

    if ~(rad > 0) || ~(a1 > 0)
        error('simlab:identify:notFopdt', ...
              ['the moments are inconsistent with a first-order model ' ...
               '(A1 = %.6g, 2*M1 - A1^2 = %.6g). The usual causes, in order: ' ...
               'the process is second order or integrating; the record was ' ...
               'cut off mid-rise; or the step was not a step. This function ' ...
               'will not return a model it cannot support.'], a1, rad);
    end

    tt = sqrt(rad);
    l = a1 - tt;
    if l < 0
        % A negative dead time is physically impossible; it means the process
        % has essentially no transport delay. Floor it and let the quality
        % score reflect the strained fit.
        w{end + 1} = sprintf( ...
            ['the fit wants a negative dead time (%.6g s), which means the ' ...
             'process has essentially none. Floored at zero.'], l);
        l = 0;
    end

    m.k = k;
    m.t = tt;
    m.l = l;
    m.noise_sigma = sigma;

    % ---- the 63.2% crossing, as a cross-check only ----
    %
    % For a true FOPDT it falls at t = L + T. A large disagreement means the
    % process is not first order, and the caller deserves to know the model
    % they are about to tune from is an approximation. It is NOT used in the
    % fit, and it is not used in the quality score either: it is measured
    % against the running estimate of the final value, which lags, so it
    % always fires early by an amount that depends on the plant.
    t632 = NaN;
    for kk = kMove:kEnd
        if abs(dy) > 0 && abs(y(kk) - y0) / abs(dy) >= 0.632
            t632 = t(kk) - t(kMove);
            break;
        end
    end

    % ---- quality: the same two indicators the C uses ----
    ratio = l / tt;
    q = 100;
    if ratio < 0.05
        q = q - 40;              % dead time barely resolvable
    elseif ratio > 2.0
        q = q - 45;              % dead-time dominant, FOPDT strained
    end
    span = l + tt;
    covered = te / span;
    if covered < 5.0
        d = min((5.0 - covered) * 10.0, 45.0);
        q = q - d;
    end
    m.quality = uint8(max(min(q, 100), 0));

    if ~isnan(t632) && abs(t632 - (l + tt)) > 0.25 * (l + tt)
        w{end + 1} = sprintf( ...
            ['the 63.2%% crossing is at %.4g s but L+T is %.4g s. For a true ' ...
             'FOPDT they coincide. The process is probably not first order, ' ...
             'so treat K, T and L as an approximation.'], t632, l + tt);
    end
    if covered < 5
        w{end + 1} = sprintf( ...
            ['the record covers only %.1f time constants. Below 5 the fit is ' ...
             'extrapolating more than it is measuring - record longer.'], ...
            covered);
    end
    if ratio > 2
        w{end + 1} = sprintf( ...
            'L/T = %.2f is dead-time dominant. Cohen-Coon is not valid here; use IMC or AMIGO.', ratio);
    end

    % ---- fit residual, so the number is checkable ----
    resid = zeros(kEnd - kMove + 1, 1);
    for kk = kMove:kEnd
        tau_ = t(kk) - t(kMove);
        if tau_ < l
            yHat = y0;
        else
            yHat = y0 + dy * (1 - exp(-(tau_ - l) / tt));
        end
        resid(kk - kMove + 1) = y(kk) - yHat;
    end

    m.fit = struct('a1', a1, 'm1', m1, 'tEnd', te, 'y0', y0, ...
        'yInf', ySettled, 't632', t632, 'covered', covered, ...
        'residual', resid, 'kMove', kMove, 'kEnd', kEnd, ...
        'sigma', sigma, 'dt', dt, 'du', du);
    m.warnings = w;
end

% ---------------------------------------------------------------------------

function b = o_band(opt)
    if isfield(opt, 'settleBand') && ~isempty(opt.settleBand)
        b = opt.settleBand;
    else
        b = 0.005;
    end
end

function [t, y, u, dt] = unpack(data)
% Accept a struct, a matrix, or a filename. One place that knows the shapes,
% so the rest of the function deals with three vectors and nothing else.
    u = [];
    if isstruct(data)
        if ~isfield(data, 't') || ~isfield(data, 'y')
            error('simlab:identify:fields', ...
                  'the data struct needs .t and .y');
        end
        t = data.t(:);
        y = data.y(:);
        if isfield(data, 'u') && ~isempty(data.u)
            u = data.u(:);
        end
    elseif isnumeric(data)
        if size(data, 2) < 2
            error('simlab:identify:shape', ...
                  'a matrix needs at least [t y] columns');
        end
        t = data(:, 1);
        y = data(:, 2);
        if size(data, 2) >= 3
            u = data(:, 3);
        end
    else
        error('simlab:identify:type', ...
              'data must be a struct, a matrix, or a CSV file name');
    end

    d = diff(t);
    dt = median(d);
    if isempty(dt) || ~(dt > 0)
        error('simlab:identify:badTime', 'the time column must increase');
    end
    % Uniform sampling is what the moment integrals assume. A historian export
    % that dropped rows would silently change te and therefore the whole fit,
    % so say so rather than integrating over a grid that is not there.
    if max(abs(d - dt)) > 0.05 * dt
        warning('simlab:identify:nonUniform', ...
                ['the sample interval varies by more than 5%% (median %.6g s). ' ...
                 'The moment integrals assume a uniform grid; resample the ' ...
                 'data for a trustworthy fit.'], dt);
    end
end

function k = findMoving(t, y) %#ok<INUSL>
% The first sample at which the response has clearly left the baseline.
%
% "Clearly" means beyond ten times the sample-to-sample noise, so a noisy
% baseline is not mistaken for the start of the rise - which would stretch te
% and with it every parameter.
    sigma = noiseEstimate(y);
    y0 = median(y(1:max(2, floor(0.05 * numel(y)))));
    thr = max(10 * sigma, 1e-12);
    idx = find(abs(y - y0) > thr, 1, 'first');
    if isempty(idx)
        k = 1;
    else
        k = idx;
    end
end

function s = noiseEstimate(y)
% Mean absolute sample-to-sample change, scaled to a standard deviation.
%
% For a still process that difference is pure noise, and for white noise
% E|y[k]-y[k-1]| = sigma*sqrt(2/pi)*sqrt(2). Taking it over the whole record
% over-estimates during the rise, which is the safe direction: every
% threshold floored by it stays above the noise.
    % For white noise the difference of two samples has standard deviation
    % sigma*sqrt(2), and the mean absolute value of a zero-mean Gaussian is
    % its standard deviation times sqrt(2/pi). So
    %     E|y[k]-y[k-1]| = sigma * sqrt(2) * sqrt(2/pi) = sigma * 2/sqrt(pi)
    % and dividing by 2/sqrt(pi) recovers sigma.
    d = abs(diff(y));
    s = mean(d) * sqrt(pi) / 2;
    if ~isfinite(s) || s < 0
        s = 0;
    end
end

function o = fillOpt(o, name, default)
    if ~isfield(o, name) || isempty(o.(name))
        o.(name) = default;
    end
end

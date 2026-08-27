function m = metrics(r, opt)
%SIMLAB.METRICS  Turn a simulation log into the numbers a datasheet quotes.
%
%   M = SIMLAB.METRICS(R) where R is the struct returned by simlab.Sim.run().
%   M = SIMLAB.METRICS(R, 'band', 0.05) uses a 5% settling band.
%
% Every field is either a number or NaN. NaN means "the scenario did not
% contain the event this metric needs" - a step-response overshoot for a run
% with no step, say. It never means zero, and it is never invented: a metric
% computed over a window that does not exist is a lie with three decimals.
%
% FIELDS
%   riseTime        10% -> 90% of the first setpoint step [s]
%   overshoot       peak excursion beyond the target, % of the step
%   undershoot      dip below the pre-step level, % of the step
%   settlingTime    last exit from the +/-band around the target [s]
%   ssError         mean error over the final steady window
%   ssStd           std-dev over that window - noise, not offset
%   iae, ise, itae  integral of |e|, e^2, t*|e| over the whole run
%   iaeStep         IAE from the first step onwards
%   tv              total variation of u, sum |du| - actuator wear
%   controlEffort   mean |u|
%   uPeak           max |u|
%   satFraction     share of samples with the output saturated
%   stable          false if the tail is diverging or non-finite
%   stepIndex, stepTime, stepFrom, stepTo   the step the metrics describe
%
% WHY THE STEP IS DETECTED RATHER THAN ASSUMED
%   A scenario is a script, not a step. metrics() finds the first change in
%   the effective setpoint and measures from there, so the same function
%   scores a bare step response and a forty-event script.

    if nargin < 2, opt = struct(); end
    if ~isfield(opt, 'band'),   opt.band = 0.02;   end
    if ~isfield(opt, 'tail'),   opt.tail = 0.20;   end
    if ~isfield(opt, 'quiet'),  opt.quiet = 0.50;  end

    m = nanStruct();

    t = r.t(:).';
    y = r.y(:).';
    sp = r.r(:).';
    u = r.u(:).';
    dt = r.dt;
    n = numel(t);

    % ---------------- integral metrics: always defined ----------------
    e = sp - y;
    m.iae = trapz(t, abs(e));
    m.ise = trapz(t, e.^2);
    m.itae = trapz(t, t .* abs(e));

    % ---------------- actuator metrics: always defined ----------------
    if ~isempty(u) && numel(u) == n
        m.tv = sum(abs(diff(u)));
        m.controlEffort = mean(abs(u));
        m.uPeak = max(abs(u));
    end
    if isfield(r, 'flags') && ~isempty(r.flags)
        sat = bitand(double(r.flags), pidx.Const.FLAG_SATURATED) ~= 0;
        m.satFraction = mean(sat);
    end

    % ---------------- find the first setpoint step ----------------
    dsp = diff(sp);
    kStep = find(abs(dsp) > 1e-12 * max(1, max(abs(sp))), 1, 'first');
    if isempty(kStep)
        % No step: report the steady state of whatever the run did, and
        % leave the transient metrics NaN.
        m.ssError = mean(e(max(1, round((1 - opt.tail) * n)):n));
        m.ssStd = std(y(max(1, round((1 - opt.tail) * n)):n));
        m.stable = isStable(y, max(1, abs(sp(end))), opt);
        return;
    end

    k0 = kStep;                    % last sample before the step
    k1 = kStep + 1;                % first sample at the new setpoint
    y0 = sp(k0);
    y1 = sp(k1);
    step = y1 - y0;

    m.stepIndex = k1;
    m.stepTime = t(k1);
    m.stepFrom = y0;
    m.stepTo = y1;

    yt = y(k1:n);
    tt = t(k1:n);
    tn = tt - t(k1);

    % ---------------- rise time, 10% to 90% ----------------
    lo = y0 + 0.10 * step;
    hi = y0 + 0.90 * step;
    if step > 0
        iLo = find(yt >= lo, 1, 'first');
        iHi = find(yt >= hi, 1, 'first');
    else
        iLo = find(yt <= lo, 1, 'first');
        iHi = find(yt <= hi, 1, 'first');
    end
    if ~isempty(iLo) && ~isempty(iHi) && iHi > iLo
        m.riseTime = tn(iHi) - tn(iLo);
    end

    % ---------------- overshoot / undershoot ----------------
    % Overshoot is the excursion BEYOND the target in the direction of the
    % step; undershoot is the dip below the starting level. A controller that
    % undershoots before rising is not "negative overshoot", it is a
    % different symptom, so it gets its own field.
    if step > 0
        m.overshoot = 100 * max(0, max(yt) - y1) / abs(step);
        m.undershoot = 100 * max(0, y0 - min(yt)) / abs(step);
    else
        m.overshoot = 100 * max(0, y1 - min(yt)) / abs(step);
        m.undershoot = 100 * max(0, max(yt) - y0) / abs(step);
    end

    % ---------------- settling time ----------------
    % The LAST exit from the band, not the first entry: a response that
    % dips back out at t = 40 s has not settled at t = 8 s, and quoting the
    % first entry is the standard way to make a bad loop look good.
    outside = abs(yt - y1) > opt.band * abs(step);
    iOut = find(outside, 1, 'last');
    if isempty(iOut)
        m.settlingTime = 0;
    elseif iOut >= numel(yt)
        m.settlingTime = Inf;      % never settled
    else
        m.settlingTime = tn(iOut + 1);
    end

    % ---------------- steady state ----------------
    % Measured over the quietest final window. If a later event in the
    % scenario is still moving the process, the window is short and ssError
    % says so by being large - which is the truth.
    kW = max(1, round(opt.tail * numel(yt)));
    tailY = yt(end - kW + 1:end);
    tailE = e(k1 + numel(yt) - kW:numel(e));
    m.ssError = mean(tailE);
    m.ssStd = std(tailY);

    % ---------------- IAE from the step ----------------
    m.iaeStep = trapz(tn, abs(y1 - yt));

    m.stable = isStable(y, max(1, abs(step)), opt);
end

% ---------------------------------------------------------------------------

function m = nanStruct()
    m = struct( ...
        'riseTime', NaN, 'overshoot', NaN, 'undershoot', NaN, ...
        'settlingTime', NaN, 'ssError', NaN, 'ssStd', NaN, ...
        'iae', NaN, 'ise', NaN, 'itae', NaN, 'iaeStep', NaN, ...
        'tv', NaN, 'controlEffort', NaN, 'uPeak', NaN, ...
        'satFraction', NaN, 'stable', false, ...
        'stepIndex', NaN, 'stepTime', NaN, 'stepFrom', NaN, ...
        'stepTo', NaN);
end

function ok = isStable(y, scale, opt)
    % Diverged, non-finite, or still swinging by more than half the step at
    % the end of the run. Deliberately crude: this flag answers "should I
    % even look at the other numbers", not "what is the stability margin" -
    % simlab.sensitivity answers that one properly.
    ok = true;
    if ~all(isfinite(y))
        ok = false;
        return;
    end
    k = max(1, round(0.5 * numel(y)));
    tail = y(k:end);
    if max(abs(tail)) > 1e6 * max(1, scale)
        ok = false;
        return;
    end
    if (max(tail) - min(tail)) > opt.quiet * 2 * scale
        ok = false;
    end
end

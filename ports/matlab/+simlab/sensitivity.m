function s = sensitivity(plant, gains, opt)
%SIMLAB.SENSITIVITY  Frequency-domain margins of the loop you just tuned.
%
%   S = SIMLAB.SENSITIVITY(PLANT, GAINS)
%   S = SIMLAB.SENSITIVITY(PLANT, GAINS, 'dt', 0.01, 'beta', 0.7)
%
% A time-domain simulation tells you what THIS plant did. It cannot tell you
% how much the plant is allowed to be wrong before the loop stops working,
% and that is the question that decides whether the gains are safe to flash.
% This function answers it.
%
% WHAT IS COMPUTED
%   L(jw) = C(jw) * G(jw) * exp(-jw*(L + dt/2))
%
%   C(jw)  the PIDX control law in the frequency domain:
%          Kp*beta + Ki/(jw) + Kd*(jw)/(1 + jw*Tf)
%   G(jw)  the plant, from its poles and zeros - no toolbox needed
%   L      the transport delay, PLUS dt/2 for the zero-order hold. Omitting
%          the half-sample hold delay makes every discrete loop look more
%          robust than it is, and the error grows with dt: at dt = 0.1 s on a
%          loop crossing at 5 rad/s it is 14 degrees of phase you did not
%          account for.
%
% FIELDS
%   Ms          peak |S| = 1/|1+L|. The single best robustness number:
%               Ms < 1.4 comfortable, 1.4-2.0 acceptable, > 2.0 fragile.
%   Mt          peak |T|, the resonant peak of the closed loop.
%   gm, pm      gain margin [x] and phase margin [deg].
%   gmFreq, pmFreq, wc   where they occur [rad/s].
%   delayMargin extra dead time the loop tolerates [s].
%   bandwidth   -3 dB frequency of T [rad/s].
%   Ms          also reported as a gain-margin equivalent 1/(Ms-1).
%   w, L, S, T  the grids and curves, for plotting.
%   warnings    cell array of plain-language problems found.
%
% WHY NOT margin(loop_tf)
%   That needs the Control System Toolbox, and it would also silently drop
%   the sample-and-hold delay. This is ~60 lines of arithmetic that runs
%   anywhere and states its own assumptions.

    if nargin < 3, opt = struct(); end
    if ~isfield(opt, 'dt'),      opt.dt = [];      end
    if ~isfield(opt, 'beta'),    opt.beta = 1.0;   end
    if ~isfield(opt, 'gamma'),   opt.gamma = 0.0;  end
    if ~isfield(opt, 'tf'),      opt.tf = [];      end
    if ~isfield(opt, 'lpfTau'),  opt.lpfTau = 0;   end
    if ~isfield(opt, 'nPoints'), opt.nPoints = 2000; end

    K = pidx.Const;
    s = struct();
    s.warnings = {};

    kp = gains.kp; ki = gains.ki; kd = gains.kd;

    % ---- sample time: from the option, else the plant's last dt ----------
    dt = opt.dt;
    if isempty(dt) || ~(dt > 0)
        dt = plant.dt;
    end
    if ~(dt > 0)
        dt = 0.01;
        s.warnings{end + 1} = ...
            'no dt available; assumed 0.01 s. Pass ''dt'' to be sure.';
    end
    s.dt = dt;

    % ---- derivative filter, exactly as the core resolves it --------------
    % An explicit Tf always wins; otherwise Tf = Kd/(N*Kp) with N = 10.
    if isempty(opt.tf)
        if kd > 0 && kp > 0
            tf = kd / (K.DEFAULT_N_FILTER * kp);
        else
            tf = 0;
        end
    else
        tf = opt.tf;
    end
    s.tf = tf;

    % ---- plant model -> poles, zeros, gain, delay ------------------------
    % Straight from the plant, which is why a 'custom' plant has to be
    % identified before it can be analysed: there is no declared transfer
    % function to read.
    [z, p, k0, Ld] = plant.polesZeros();
    s.warnings = plantAnalysisWarnings(plant);

    % ---- frequency grid --------------------------------------------------
    % Span six decades centred on where the loop is likely to cross. Without
    % a guess we start from the plant's own corner frequencies, which is
    % enough to bracket any sane design.
    corners = [1 / max(Ld, 1e-6), 1e3];
    if ~isempty(p)
        pc = abs(p);
        pc = pc(pc > 1e-9);
        if ~isempty(pc), corners = [corners, pc(:).']; end
    end
    wlo = min(corners) / 1e3;
    whi = min(max(corners) * 1e3, pi / dt);
    w = logspace(log10(max(wlo, 1e-6)), log10(whi), opt.nPoints);
    jw = 1i * w;

    % ---- controller ------------------------------------------------------
    C = kp * opt.beta + ki ./ jw + kd * jw ./ (1 + jw * tf);
    % gamma only scales the setpoint path, so it does not enter L; it is
    % recorded so a report can state what was assumed.
    s.beta = opt.beta;
    s.gamma = opt.gamma;

    % ---- plant -----------------------------------------------------------
    num = k0 * ones(size(jw));
    for i = 1:numel(z)
        num = num .* (jw - z(i));
    end
    den = ones(size(jw));
    for i = 1:numel(p)
        den = den .* (jw - p(i));
    end
    G = num ./ den;

    % ---- dead time + zero-order hold -------------------------------------
    tauExtra = Ld + 0.5 * dt;
    Lw = C .* G .* exp(-jw * tauExtra);

    % ---- measurement filter, if the loop has one -------------------------
    if opt.lpfTau > 0
        Lw = Lw ./ (1 + jw * opt.lpfTau);
    end

    S = 1 ./ (1 + Lw);
    T = Lw ./ (1 + Lw);

    s.w = w; s.L = Lw; s.S = S; s.T = T;
    s.deadTime = Ld;
    s.holdDelay = 0.5 * dt;

    % ---- peak sensitivity ------------------------------------------------
    [s.Ms, iMs] = max(abs(S));
    s.MsFreq = w(iMs);
    [s.Mt, iMt] = max(abs(T));
    s.MtFreq = w(iMt);

    % ---- gain margin: |L| = 1 where phase = -180 deg ---------------------
    ph = unwrap(angle(Lw));
    idx = find(ph < -pi, 1, 'first');
    if ~isempty(idx) && idx > 1
        % Interpolate the crossing so the answer does not depend on the grid.
        f = (-pi - ph(idx - 1)) / (ph(idx) - ph(idx - 1));
        wc = exp(log(w(idx - 1)) + f * (log(w(idx)) - log(w(idx - 1))));
        mag = abs(interp1(log(w), log(abs(Lw)), log(wc)));
        s.gm = 1 / mag;
        s.gmFreq = wc;
    else
        s.gm = Inf;
        s.gmFreq = NaN;
        s.warnings{end + 1} = ...
            'phase never reaches -180 deg on the grid: gain margin is infinite, which usually means the grid is too narrow.';
    end

    % ---- phase margin: phase where |L| = 1 -------------------------------
    lg = log(abs(Lw));
    idx = find(lg < 0, 1, 'first');
    if ~isempty(idx) && idx > 1
        f = (0 - lg(idx - 1)) / (lg(idx) - lg(idx - 1));
        wc = exp(log(w(idx - 1)) + f * (log(w(idx)) - log(w(idx - 1))));
        phc = interp1(log(w), ph, log(wc));
        s.pm = 180 * (pi + phc) / pi;
        s.pmFreq = wc;
        s.wc = wc;
    else
        s.pm = NaN;
        s.pmFreq = NaN;
        s.wc = NaN;
        s.warnings{end + 1} = ...
            '|L| never falls below 1 on the grid: the loop may be open-loop unstable or the grid is wrong.';
    end

    % ---- bandwidth: |T| = 1/sqrt(2) of its low-frequency value -----------
    T0 = abs(T(1));
    idx = find(abs(T) < T0 / sqrt(2), 1, 'first');
    if ~isempty(idx)
        s.bandwidth = w(idx);
    else
        s.bandwidth = NaN;
    end

    % ---- delay margin ----------------------------------------------------
    % Extra dead time the loop survives. At the gain-crossover frequency the
    % phase budget left is exactly the phase margin, so tau_max = pm/wc. This
    % is the number to compare against how wrong your dead-time estimate is.
    if ~isnan(s.pm) && s.pm > 0 && ~isnan(s.wc)
        s.delayMargin = (s.pm * pi / 180) / s.wc;
    else
        s.delayMargin = NaN;
    end

    % ---- plain-language verdict ------------------------------------------
    s.verdict = verdict(s);
end

% ---------------------------------------------------------------------------

function v = verdict(s)
% A verdict in words, because "Ms = 1.83" does not tell an operator what to
% do. The thresholds are the conventional ones (Astrom & Hagglund): Ms 1.4
% for a comfortable loop, 2.0 as the limit of what is normally accepted.
    bits = {};
    if s.Ms < 1.4
        bits{end + 1} = sprintf('Ms = %.2f: comfortable', s.Ms);
    elseif s.Ms < 2.0
        bits{end + 1} = sprintf('Ms = %.2f: acceptable, but do not push the gains further', s.Ms);
    else
        bits{end + 1} = sprintf('Ms = %.2f: FRAGILE - detune (raise Ti, lower Kp)', s.Ms);
    end
    if ~isnan(s.pm)
        if s.pm < 30
            bits{end + 1} = sprintf('phase margin %.0f deg is below the usual 30 deg floor', s.pm);
        else
            bits{end + 1} = sprintf('phase margin %.0f deg', s.pm);
        end
    end
    if ~isinf(s.gm) && ~isnan(s.gm)
        bits{end + 1} = sprintf('gain margin %.1fx', s.gm);
    end
    if ~isnan(s.delayMargin)
        bits{end + 1} = sprintf('tolerates %.3g s more dead time', s.delayMargin);
    end
    v = strjoin(bits, '; ');
end

function warn = plantAnalysisWarnings(plant)
% What the analysis is NOT seeing. Stating it is cheaper than a reader
% discovering it after the plant behaves differently from the Bode plot.
    warn = plant.analysisCaveats();
    [~, pz] = plant.polesZeros();
    if any(abs(pz) < 1e-9)
        warn{end + 1} = ['the plant has a pole at the origin ' ...
            '(integrating): the phase margin is still meaningful, the gain ' ...
            'margin is not.'];
    end
end

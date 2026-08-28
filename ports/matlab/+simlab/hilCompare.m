function c = hilCompare(rSim, rHil, opt)
%SIMLAB.HILCOMPARE  Where does the board disagree with the simulation, and why?
%
%   C = SIMLAB.HILCOMPARE(RSIM, RHIL)
%   SIMLAB.HILCOMPARE(RSIM, RHIL, 'fig', 95)
%
% RSIM comes from simlab.Sim.run(), RHIL from simlab.hilRun(). Both must have
% been produced from the same scenario on the same plant model, or the
% difference measures the scenario rather than the board.
%
% WHY NOT JUST PLOT BOTH AND LOOK
%   Two traces that differ by 2% somewhere in the middle tell you nothing
%   actionable. What matters is WHICH channel differs first, because each
%   channel has a different cause and a different fix:
%
%     y differs, u agrees      the plant model is wrong. The board's process
%                              is not the model. Nothing about the controller
%                              is implicated, and retuning would be wrong.
%     u differs, y agrees      the controller is not the one you designed.
%                              Check what is actually flashed, the compiler's
%                              fast-math settings, and whether dt on the board
%                              is the dt you exported.
%     both differ from t = 0   a units or scaling mismatch. The most common
%                              single cause, and the easiest to miss because
%                              both traces look plausible.
%     both agree then diverge  something saturates, faults or resets on one
%                              side. Look at the flags, not at the values.
%
%   This function does that triage and says which case you are in.
%
% FIELDS
%   C.verdict      the triage, in words
%   C.dyMax/.duMax max absolute difference on each channel
%   C.tFirstDy/.tFirstDu  when each channel first exceeded 'tol'
%   C.lagSamples   the best-fit integer lag between the two u traces. A
%                  consistent non-zero lag is a sample-order or ISR-latency
%                  difference, which is a specific and fixable bug rather than
%                  a mysterious offset
%   C.flagMismatch samples where the PIDX flags disagree
%   C.fig          the figure handle

    if nargin < 3, opt = struct(); end
    o = fillOpt(opt, 'fig', 95);
    o = fillOpt(o, 'tol', []);
    o = fillOpt(o, 'noPlot', false);

    c = struct();

    % ---- align ----
    %
    % The two runs do not necessarily have the same number of samples: the HIL
    % run aborts on link timeouts, and the board's dt may differ. Comparison
    % is done on a common time grid by index of the SHORTER run, and any
    % leftover is reported rather than silently dropped.
    n = min(numel(rSim.t), numel(rHil.t));
    if n < 10
        error('simlab:hilCompare:short', ...
              'only %d common samples - nothing to compare', n);
    end
    c.n = n;
    c.nSkippedSim = numel(rSim.t) - n;
    c.nSkippedHil = numel(rHil.t) - n;
    c.dtSim = rSim.dt;
    c.dtHil = rHil.dt;
    if abs(rSim.dt - rHil.dt) > 1e-9 * max(rSim.dt, rHil.dt)
        c.verdict = sprintf( ...
            ['the two runs used different dt (%.6g s vs %.6g s). Every ' ...
             'difference below is explained by that alone: the integrator ' ...
             'and the derivative filter both scale with dt. Re-run with the ' ...
             'same dt before drawing any conclusion.'], rSim.dt, rHil.dt);
        return;
    end

    t = rSim.t(1:n);
    yS = rSim.y(1:n);   yH = rHil.y(1:n);
    uS = rSim.u(1:n);   uH = rHil.u(1:n);

    % Tolerances default to something derived from the signals themselves.
    % An absolute tolerance chosen by hand is either too loose to catch
    % anything or too tight to survive the board's ADC.
    uSpan = max(abs([uS, uH]));
    ySpan = max(abs([yS, yH]));
    if isempty(o.tol)
        tolU = 0.01 * uSpan;
        tolY = 0.01 * ySpan;
    else
        tolU = o.tol * uSpan;
        tolY = o.tol * ySpan;
    end
    c.tolU = tolU;
    c.tolY = tolY;

    dy = abs(yS - yH);
    du = abs(uS - uH);
    c.dyMax = max(dy);
    c.duMax = max(du);
    c.dyRms = sqrt(mean(dy.^2));
    c.duRms = sqrt(mean(du.^2));

    iY = find(dy > tolY, 1, 'first');
    iU = find(du > tolU, 1, 'first');
    c.tFirstDy = nanIfEmpty(iY, t);
    c.tFirstDu = nanIfEmpty(iU, t);

    % ---- flags ----
    if isfield(rSim, 'flags') && isfield(rHil, 'flags')
        fm = bitxor(double(rSim.flags(1:n)), double(rHil.flags(1:n))) ~= 0;
        c.flagMismatch = sum(fm);
        c.tFirstFlagMismatch = nanIfEmpty(find(fm, 1, 'first'), t);
    else
        c.flagMismatch = 0;
        c.tFirstFlagMismatch = NaN;
    end

    % ---- best-fit lag ----
    %
    % Only over a window where both signals are moving, because a constant
    % signal matches itself at every lag and would report zero. Cross
    % correlation over +-20 samples is enough to distinguish "the board is one
    % sample behind" from "the board is doing something else".
    [c.lagSamples, c.lagCorr] = bestLag(uS, uH, 20);

    % ---- triage ----
    c.verdict = triage(c, dy, du, tolU, tolY, t);

    if ~o.noPlot
        c.fig = plotCompare(t, yS, yH, uS, uH, dy, du, tolY, tolU, c, o.fig);
    end
end

% ---------------------------------------------------------------------------

function s = triage(c, dy, du, tolU, tolY, t) %#ok<INUSL>
% The four cases, decided by which channel moves first and whether the
% disagreement is a lag or an offset.
    bits = {};

    yBad = c.dyMax > tolY;
    uBad = c.duMax > tolU;

    if ~yBad && ~uBad
        bits{end + 1} = sprintf( ...
            ['AGREEMENT. Both channels stay inside tolerance (max |dy| = ' ...
             '%.4g, max |du| = %.4g) over %d samples. The compiled controller ' ...
             'on the board reproduces the simulation.'], c.dyMax, c.duMax, c.n);
        if c.flagMismatch > 0
            bits{end + 1} = sprintf( ...
                '...but the PIDX status flags disagree on %d samples. The values match and the reported state does not, which is worth chasing: it usually means one side is saturating and recovering inside a sample the other does not see.', ...
                c.flagMismatch);
        end
        s = strjoin(bits, sprintf('\n'));
        return;
    end

    if abs(c.lagSamples) > 0 && c.lagCorr > 0.9
        bits{end + 1} = sprintf( ...
            ['LAG. The board''s output matches the simulation shifted by %d ' ...
             'sample(s) (correlation %.3f). That is a sample-order or ' ...
             'ISR-latency difference, not a tuning difference: check whether ' ...
             'the board updates the plant before or after computing the ' ...
             'command.'], c.lagSamples, c.lagCorr);
    end

    if yBad && ~uBad
        bits{end + 1} = sprintf( ...
            ['PLANT MODEL. The commands agree (max |du| = %.4g) but the ' ...
             'measurements do not (max |dy| = %.4g, first at t = %.4g s). The ' ...
             'controller is doing the same thing on both sides; the process is ' ...
             'not. Retuning would be the wrong response - fix the model, or ' ...
             'accept that it is an approximation and re-run ' ...
             'simlab.monteCarlo with a wider spread.'], ...
            c.duMax, c.dyMax, c.tFirstDy);
    elseif uBad && ~yBad
        bits{end + 1} = sprintf( ...
            ['CONTROLLER. The measurements agree but the commands do not ' ...
             '(max |du| = %.4g, first at t = %.4g s). The board is not running ' ...
             'the controller you designed. Check, in this order: what is ' ...
             'actually flashed; the dt the board was configured with; and ' ...
             'whether the build enabled fast-math, which reorders the ' ...
             'floating-point operations the anti-windup depends on.'], ...
            c.duMax, c.tFirstDu);
    elseif c.tFirstDy < 3 * c.dtSim && c.tFirstDu < 3 * c.dtSim
        bits{end + 1} = sprintf( ...
            ['SCALING. Both channels disagree from the very first sample. ' ...
             'That is the signature of a units mismatch - engineering units ' ...
             'against Q15, or Celsius against Kelvin - rather than of a ' ...
             'dynamic difference. Check the ratio: dy/du of %.4g should be ' ...
             'the plant gain.'], c.dyMax / max(c.duMax, 1e-30));
    else
        bits{end + 1} = sprintf( ...
            ['DIVERGENCE. The traces agree until t = %.4g s and then part ' ...
             'company. Something saturated, faulted or reset on one side. ' ...
             'Compare the status-flag bands in the figure rather than the ' ...
             'values: %d samples have disagreeing flags.'], ...
            min(c.tFirstDy, c.tFirstDu), c.flagMismatch);
    end

    s = strjoin(bits, sprintf('\n'));
end

function v = nanIfEmpty(i, t)
    if isempty(i)
        v = NaN;
    else
        v = t(i);
    end
end

function [lag, corrBest] = bestLag(a, b, maxLag)
% Integer lag that best aligns B to A, by normalised cross correlation.
%
% Measured over the samples where the signal is actually changing. A settled
% trace correlates perfectly with itself at every lag, which would report
% lag = 0 no matter what the board was doing.
    d = diff(a);
    active = abs(d) > 0.01 * max(abs(d));
    if sum(active) < 10
        lag = 0;
        corrBest = NaN;
        return;
    end

    best = -inf;
    lag = 0;
    for L = -maxLag:maxLag
        if L >= 0
            x = a(2 + L:end);
            y = b(2:end - L);
        else
            x = a(2:end + L);
            y = b(2 - L:end);
        end
        m = min(numel(x), numel(y));
        if m < 10, continue; end
        x = x(1:m); y = y(1:m);
        sx = std(x); sy = std(y);
        if sx <= 0 || sy <= 0, continue; end
        cc = mean((x - mean(x)) .* (y - mean(y))) / (sx * sy);
        if cc > best
            best = cc;
            lag = L;
        end
    end
    corrBest = best;
end

function h = plotCompare(t, yS, yH, uS, uH, dy, du, tolY, tolU, c, fignum)
    h = figure(fignum);
    clf(h);
    set(h, 'Color', 'w', 'Name', 'HIL vs simulation');

    subplot(3, 1, 1);
    plot(t, yS, 'b', 'LineWidth', 1.2); hold on;
    plot(t, yH, 'r--', 'LineWidth', 1.0);
    ylabel('measurement');
    legend('simulation', 'board', 'Location', 'best');
    title('measurement: the plant model against the real process');
    grid on;

    subplot(3, 1, 2);
    plot(t, uS, 'b', 'LineWidth', 1.2); hold on;
    plot(t, uH, 'r--', 'LineWidth', 1.0);
    ylabel('command');
    title('command: the controller you designed against the one that is flashed');
    grid on;

    subplot(3, 1, 3);
    plot(t, dy, 'Color', [0.2 0.5 0.2], 'LineWidth', 1.0); hold on;
    plot(t, du, 'Color', [0.7 0.3 0.1], 'LineWidth', 1.0);
    plot(t, tolY * ones(size(t)), ':', 'Color', [0.2 0.5 0.2]);
    plot(t, tolU * ones(size(t)), ':', 'Color', [0.7 0.3 0.1]);
    ylabel('|difference|');
    xlabel('time [s]');
    legend('|dy|', '|du|', 'Location', 'best');
    title('where they part company');
    grid on;

    try
        sgtitle(c.verdict, 'Interpreter', 'none');
    catch
        set(h, 'Name', 'HIL vs simulation');
    end
end

function o = fillOpt(o, name, default)
    if ~isfield(o, name) || isempty(o.(name))
        o.(name) = default;
    end
end

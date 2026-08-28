function h = plot(r, opt)
%SIMLAB.PLOT  One figure that explains what the loop did and why.
%
%   SIMLAB.PLOT(R)              % R from simlab.Sim.run()
%   H = SIMLAB.PLOT(R, 'fig', 3, 'showTrue', true)
%
% Six panels, because a step response alone hides most of what matters:
%
%   1  setpoint vs measurement (and, optionally, the plant's true output -
%      the difference between the two lines IS your sensor chain)
%   2  control effort: the command, the pre-saturation sum, and what the
%      actuator actually delivered. When the first two separate, you are
%      saturated; when the second and third separate, the actuator is
%      rate- or deadband-limited.
%   3  P, I and D terms. Windup is visible here before it is visible in the
%      output: the I trace runs away while the output sits on the rail.
%   4  error
%   5  PIDX status flags as coloured bands. Saturation, faults, ramping and
%      slew-limiting, exactly as the controller reported them - not
%      re-derived from the data.
%   6  the active gains, which only move if gain scheduling is attached.
%
% OPTIONS
%   'fig'       figure number. Default 91.
%   'showTrue'  overlay the plant's true output. Default true.
%   'events'    mark scenario events with vertical lines. Default true.
%   'title'     figure title. Default the scenario name.
%
% Plain figure/subplot rather than uifigure on purpose: this has to work in
% Octave and in a MATLAB without App Designer, because a plot you cannot
% produce on the build machine is not part of the tool.

    if nargin < 2, opt = struct(); end
    showTrue = getOpt(opt, 'showTrue', true);
    events = getOpt(opt, 'events', true);
    fignum = getOpt(opt, 'fig', 91);
    ttl = getOpt(opt, 'title', []);

    h = figure(fignum);
    clf(h);
    set(h, 'Color', 'w', 'Name', 'PIDX simlab');

    t = r.t(:);
    n = numel(t);

    % ---------------- panel 1: tracking ----------------
    ax1 = subplot(3, 2, 1, 'Parent', h);
    plot(t, r.r(:), 'k--', 'LineWidth', 1.2); hold(ax1, 'on');
    if showTrue && isfield(r, 'yTrue') && any(r.yTrue ~= r.y)
        plot(t, r.yTrue(:), 'Color', [0.55 0.75 0.55], 'LineWidth', 1.0);
    end
    plot(t, r.y(:), 'b', 'LineWidth', 1.4);
    ylabel(ax1, 'process variable');
    title(ax1, 'setpoint vs measurement');
    legend(ax1, legendFor(r, showTrue), 'Location', 'best', 'FontSize', 7);
    grid(ax1, 'on');
    hold(ax1, 'off');

    % ---------------- panel 2: control effort ----------------
    ax2 = subplot(3, 2, 2, 'Parent', h);
    plot(t, r.uRaw(:), 'Color', [0.8 0.5 0.5], 'LineWidth', 0.9); hold(ax2, 'on');
    if isfield(r, 'uPlant') && any(abs(r.uPlant - r.u) > 0)
        plot(t, r.uPlant(:), 'Color', [0.4 0.7 0.4], 'LineWidth', 1.0);
    end
    plot(t, r.u(:), 'r', 'LineWidth', 1.3);
    ylabel(ax2, 'control output');
    title(ax2, sprintf('control effort   (TV = %.4g, peak %.4g)', ...
                       r.metrics.tv, r.metrics.uPeak));
    grid(ax2, 'on');
    hold(ax2, 'off');

    % ---------------- panel 3: P, I, D ----------------
    ax3 = subplot(3, 2, 3, 'Parent', h);
    plot(t, r.p(:), 'r', t, r.i(:), 'b', t, r.d(:), 'Color', [0 0.6 0]);
    if any(abs(r.ff(:)) > 0)
        hold(ax3, 'on');
        plot(t, r.ff(:), 'm');
        hold(ax3, 'off');
    end
    ylabel(ax3, 'term');
    title(ax3, 'P, I, D (and FF) in output units');
    legend(ax3, {'P', 'I', 'D'}, 'Location', 'best', 'FontSize', 7);
    grid(ax3, 'on');

    % ---------------- panel 4: error ----------------
    ax4 = subplot(3, 2, 4, 'Parent', h);
    plot(t, r.e(:), 'Color', [0.2 0.2 0.6]);
    ylabel(ax4, 'error');
    title(ax4, sprintf('error   (IAE = %.4g, steady %.3g)', ...
                       r.metrics.iae, r.metrics.ssError));
    grid(ax4, 'on');

    % ---------------- panel 5: flags ----------------
    ax5 = subplot(3, 2, 5, 'Parent', h);
    plotFlagBands(ax5, r);

    % ---------------- panel 6: gains ----------------
    ax6 = subplot(3, 2, 6, 'Parent', h);
    scheduling = any(abs(diff(r.kp(:))) > 0) || any(abs(diff(r.ki(:))) > 0);
    if scheduling
        plot(t, r.kp(:), 'r', t, r.ki(:), 'b', t, r.kd(:), 'Color', [0 0.6 0]);
        legend(ax6, {'Kp', 'Ki', 'Kd'}, 'Location', 'best', 'FontSize', 7);
        title(ax6, 'active gains (gain scheduling is attached)');
    else
        plot(t, r.u(:), 'Color', [0.6 0.6 0.6]);
        title(ax6, sprintf('gains fixed: Kp=%.4g  Ki=%.4g  Kd=%.4g', ...
                           r.kp(1), r.ki(1), r.kd(1)));
    end
    ylabel(ax6, 'gain');
    grid(ax6, 'on');

    for ax = [ax1, ax2, ax3, ax4, ax6]
        xlabel(ax, 'time [s]');
    end
    xlabel(ax5, 'time [s]');

    if events && isfield(r, 'scenario')
        markEvents(h, r);
    end

    if isempty(ttl)
        ttl = sprintf('PIDX simlab  -  rise %.4g s, OS %.1f%%, settle %.4g s, stable %d', ...
            r.metrics.riseTime, r.metrics.overshoot, ...
            r.metrics.settlingTime, logical(r.metrics.stable));
    end
    sgtitleOrTitle(h, ttl);
end

% ---------------------------------------------------------------------------

function plotFlagBands(ax, r)
% Draw the PIDX status flags as coloured bands.
%
% The flags come from the controller, not from a re-derivation: if the
% controller said it was saturated, the band says so, even in a case where
% guessing from the data would give a different answer.
    K = pidx.Const;
    cla(ax);
    hold(ax, 'on');
    t = r.t(:);
    fl = double(r.flags(:));
    rows = [K.FLAG_SATURATED_HIGH, K.FLAG_SATURATED_LOW, K.FLAG_FAULT, ...
            K.FLAG_INTEGRAL_LIMITED, K.FLAG_SP_RAMPING, ...
            K.FLAG_OUTPUT_SLEWING, K.FLAG_MANUAL, K.FLAG_SENSOR_INVALID];
    names = {'saturated high', 'saturated low', 'FAULT', 'integral limited', ...
             'sp ramping', 'output slewing', 'manual', 'sensor invalid'};
    cols = [0.85 0.2 0.2; 0.9 0.5 0.2; 0.5 0 0.5; 0.2 0.4 0.8; ...
            0.2 0.7 0.4; 0.7 0.7 0.2; 0.5 0.5 0.5; 0.9 0.2 0.6];

    shown = {};
    y = 0;
    for i = 1:numel(rows)
        on = bitand(uint32(fl), uint32(rows(i))) ~= 0;
        if ~any(on)
            continue;
        end
        y = y + 1;
        % Build rectangles rather than a stair plot: a band that is one
        % sample wide must still be visible.
        idx = find(on);
        groups = groupRuns(idx);
        for g = 1:size(groups, 1)
            k0 = groups(g, 1);
            k1 = groups(g, 2);
            w = t(k1) - t(k0);
            if w <= 0, w = max(r.dt, 1e-9); end
            rectangle('Position', [t(k0), y - 0.35, w, 0.7], ...
                      'FaceColor', cols(i, :), 'EdgeColor', 'none');
        end
        shown{end + 1} = names{i}; %#ok<AGROW>
    end
    if y == 0
        text(0.5, 0.5, 'no flags raised', 'Units', 'normalized', ...
             'HorizontalAlignment', 'center', 'Color', [0.4 0.4 0.4]);
    else
        set(ax, 'YTick', 1:y, 'YTickLabel', shown);
    end
    if t(end) > t(1)
        xlim(ax, [t(1), t(end)]);
    else
        xlim(ax, [t(1) - 1, t(1) + 1]);
    end
    ylim(ax, [0.3, max(y, 1) + 0.4]);
    title(ax, sprintf('PIDX status flags   (%.1f%% of samples saturated)', ...
                      100 * r.metrics.satFraction));
    grid(ax, 'on');
    hold(ax, 'off');
end

function g = groupRuns(idx)
% [first last] of each run of consecutive indices.
    g = zeros(0, 2);
    if isempty(idx), return; end
    s = idx(1);
    p = idx(1);
    for i = 2:numel(idx)
        if idx(i) == p + 1
            p = idx(i);
        else
            g(end + 1, :) = [s, p]; %#ok<AGROW>
            s = idx(i);
            p = idx(i);
        end
    end
    g(end + 1, :) = [s, p];
end

function markEvents(h, r)
% Vertical lines at the scenario events, labelled.
%
% Parsed from the transcript the Sim stored, so the plot and the report can
% never disagree about when something happened.
    if ~isfield(r, 'scenario') || isempty(r.scenario), return; end
    lines = regexp(r.scenario, '\n', 'split');
    ax = findall(h, 'Type', 'axes');
    for i = 1:numel(lines)
        tok = regexp(lines{i}, 't=\s*([0-9.eE+-]+)\s+(\S+)\s*(.*)', 'tokens', 'once');
        if isempty(tok), continue; end
        te = str2double(tok{1});
        kind = tok{2};
        lab = strtrim(tok{3});
        if strcmp(kind, 'setpoint') || strcmp(kind, 'spRamp') || ...
           strcmp(kind, 'loadStep') || strcmp(kind, 'stuck') || ...
           strcmp(kind, 'noise') || strcmp(kind, 'disturb') || ...
           strcmp(kind, 'plantGain') || strcmp(kind, 'mode')
            for j = 1:numel(ax)
                hold(ax(j), 'on');
                plot(ax(j), [te te], ylim(ax(j)), ':', ...
                     'Color', [0.5 0.5 0.5], 'LineWidth', 0.8);
                hold(ax(j), 'off');
            end
            yl = ylim(ax(1));
            text(ax(1), te, yl(1) + 0.02 * (yl(2) - yl(1)), ...
                 sprintf(' %s %s', kind, lab), 'FontSize', 6, ...
                 'Rotation', 90, 'Color', [0.3 0.3 0.3], ...
                 'VerticalAlignment', 'bottom');
        end
    end
end

function leg = legendFor(r, showTrue)
    leg = {'setpoint', 'measurement'};
    if showTrue && isfield(r, 'yTrue') && any(r.yTrue ~= r.y)
        leg = {'setpoint', 'true plant output', 'measurement'};
    end
end

function sgtitleOrTitle(h, ttl)
% sgtitle() is R2018b+. Fall back to annotating the first axes so this works
% everywhere rather than erroring on an older release.
    try
        sgtitle(ttl, 'Interpreter', 'none');
    catch
        set(h, 'Name', ttl);
    end
end

function v = getOpt(opt, name, default)
    if isfield(opt, name) && ~isempty(opt.(name))
        v = opt.(name);
    else
        v = default;
    end
end

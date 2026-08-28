function story = explain(r, plant, cfg, opt)
%SIMLAB.EXPLAIN  The loop as a story: what goes where, and WHY.
%
%   SIMLAB.EXPLAIN(R, PLANT, CFG)          draws the flow and prints the story
%   STORY = SIMLAB.EXPLAIN(R, PLANT, CFG)  returns the story as text, no figure
%   ... , struct('fig', 96)                figure number
%
% A step response shows WHAT happened. This shows WHERE each signal came
% from, WHERE it went, and WHY the stage between them exists - with the
% numbers THIS run produced, not a generic textbook diagram:
%
%   r --beta--> P --+
%   e --------> I --+--> u_raw --limits--> u --actuator--> uPlant
%   y --Tf----> D --+                                      |
%     ^                                                    v
%     +--- sensor <--- yTrue <--- plant(L, tau, K) <-------+
%
% and a seven-panel trace strip in the SAME order, so every arrow of the
% diagram has a plot under it: the gap between two consecutive panels IS the
% stage between the signals, annotated with what it contributed.
%
% The story lines are the deliverable for a hand-over: "the output saturated
% 43% of the samples because the actuator ends at 100 while the loop asked
% for 147; BACK_CALCULATION bled the integrator so recovery took 23 s" - the
% kind of sentence a tuning session should leave behind.

    if nargin < 4, opt = struct(); end
    K = pidx.Const;

    dt = r.dt;
    n = numel(r.t);
    t = r.t(:);

    % ---- stage numbers, measured from THIS run ----
    sat = bitand(double(r.flags), K.FLAG_SATURATED) ~= 0;
    satPct = 100 * mean(sat);
    nDelay = round(plant.transportDelay() / dt);
    sensorGap = r.y(:) - r.yTrue(:);
    % measure the sensor noise on the settled tail, where the plant is quiet
    kTail = max(1, round(0.7 * n)):n;
    noiseMeas = std(sensorGap(kTail));
    actGap = max(abs(r.u(:) - r.uPlant(:)));
    ssErr = r.e(end);
    iFinal = r.i(end);
    pPeak = max(abs(r.p(:)));
    iPeak = max(abs(r.i(:)));
    dPeak = max(abs(r.d(:)));
    askedMax = max(abs(r.uRaw(:)));

    % ---- the story ----
    story = {};
    story{end+1} = sprintf('SETPOINT r = %.6g', r.r(max(2, end)));
    story{end+1} = sprintf(['  -> P term, weighted by beta = %.2f. WHY: beta < 1 ' ...
        'softens the setpoint kick without touching disturbance rejection. ' ...
        'P peaked at %.6g.'], cfg.weight.beta, pPeak);
    story{end+1} = sprintf(['  -> D term through Tf, on the %s. WHY: derivative ' ...
        'on the measurement avoids the kick a step in r would otherwise ' ...
        'inject. D peaked at %.6g.'], derivName(cfg.filter.derivative_mode), dPeak);
    story{end+1} = sprintf(['INTEGRATOR ended at %.6g and left a steady error ' ...
        'of %.6g. WHY: P alone always leaves an offset; I exists to eat it. ' ...
        'If the offset is not ~0, the loop never reached the target or I is ' ...
        'disabled.'], iFinal, ssErr);
    story{end+1} = sprintf(['SUM asked for %.6g but the ACTUATOR ENDS at ' ...
        '%.6g: the output was saturated %.1f%% of the samples. WHY: physics ' ...
        '- no controller can command more than the hardware delivers; ' ...
        'anti-windup (%s) exists so the integrator does not remember the ' ...
        'impossible.'], askedMax, cfg.limits.output_max, satPct, ...
        awName(cfg.integral.mode));
    if actGap > 0
        story{end+1} = sprintf(['ACTUATOR CHAIN (deadband/slew/quantisation) ' ...
            'changed the command by up to %.6g between u and what the plant ' ...
            'received. WHY: real drives are not ideal wires.'], actGap);
    else
        story{end+1} = 'ACTUATOR CHAIN: ideal in this run (u == uPlant).';
    end
    story{end+1} = sprintf(['PLANT K=%.4g tau=%.4g s L=%.4g s turned u into ' ...
        'yTrue. WHY the response cannot start before %.0f samples: the dead ' ...
        'time is transport, not tuning - no gains can beat it.'], ...
        plant.steadyStateGain(), plant.tau(), plant.transportDelay(), nDelay);
    story{end+1} = sprintf(['SENSOR CHAIN added %.6g of spread (noise sigma ' ...
        '%.6g, %d-bit ADC over [%.6g, %.6g]). WHY it matters: the D term ' ...
        'amplifies exactly this, and the loop closes on yMeas, not yTrue.'], ...
        noiseMeas, plant.sensorParam('sigma'), plant.sensorParam('bits'), ...
        plant.sensorParam('qmin'), plant.sensorParam('qmax'));
    story{end+1} = sprintf(['CLOSE THE LOOP: y goes back to the top. Every ' ...
        'gap you see between two traces above is one of the stages this ' ...
        'story names - if a number on the board disagrees with MATLAB, walk ' ...
        'this list and find which stage the board has that the model does ' ...
        'not.']);

    % Interactive calls (no output asked) draw; scripted calls draw only when
    % a figure number is given, so the test suite stays headless.
    drawIt = (nargout == 0) || (isfield(opt, 'fig') && ~isempty(opt.fig));
    if drawIt
        drawFlow(r, plant, cfg, satPct, nDelay, noiseMeas, actGap, opt);
    end
    if nargout == 0
        for k = 1:numel(story)
            fprintf('%s\n', story{k});
        end
    end
end

% ===========================================================================

function drawFlow(r, plant, cfg, satPct, nDelay, noiseMeas, actGap, opt)
    fignum = 96;
    if isfield(opt, 'fig') && ~isempty(opt.fig), fignum = opt.fig; end
    h = figure(fignum);
    clf(h);
    set(h, 'Color', [0.97 0.97 0.98], 'Name', 'PIDX simlab - the loop, explained');

    % ---- the diagram ----
    ax = axes('Parent', h, 'Units', 'normalized', 'Position', ...
        [0.03 0.72 0.94 0.25], 'Visible', 'off', 'xlim', [0 100], 'ylim', [0 10]);
    box(ax,  2, 4, 12, 4, 'r (setpoint)', [0.85 0.92 1.0]);
    box(ax, 20, 7, 12, 3, sprintf('P  x%.3g (beta %.2f)', cfg.core.kp, cfg.weight.beta), [1 0.93 0.8]);
    box(ax, 20, 4, 12, 3, sprintf('I  x%.3g', cfg.core.ki), [0.85 1 0.85]);
    box(ax, 20, 1, 12, 3, sprintf('D  x%.3g / Tf', cfg.core.kd), [1 0.88 0.94]);
    box(ax, 38, 4, 10, 4, 'sum', [0.93 0.93 0.93]);
    box(ax, 52, 4, 13, 4, sprintf('limits [%.4g, %.4g]\nsat %.0f%%', ...
        cfg.limits.output_min, cfg.limits.output_max, satPct), [1 0.85 0.8]);
    box(ax, 69, 4, 12, 4, 'actuator chain', [1 0.95 0.8]);
    box(ax, 84, 4, 14, 4, sprintf('plant\nK %.3g tau %.3g\nL %.3g (%d smp)', ...
        plant.steadyStateGain(), plant.tau(), plant.transportDelay(), nDelay), ...
        [0.85 0.9 1.0]);
    box(ax, 52, 0.2, 20, 2.6, sprintf('sensor  +noise %.3g, %d-bit', ...
        noiseMeas, plant.sensorParam('bits')), [0.95 0.9 1.0]);
    arrow(ax, 14, 6, 20, 8);  arrow(ax, 14, 5.5, 20, 5.5);
    arrow(ax, 14, 5, 20, 2.5);
    arrow(ax, 32, 8, 41, 6.5); arrow(ax, 32, 5.5, 41, 6); arrow(ax, 32, 2.5, 41, 5.5);
    arrow(ax, 48, 6, 52, 6);  arrow(ax, 65, 6, 69, 6);  arrow(ax, 81, 6, 84, 6);
    arrow(ax, 91, 4, 91, 1.5); arrow(ax, 72, 1.5, 72, 1.5);
    arrow(ax, 52, 1.5, 72, 1.5); % plant out -> sensor (leftward drawn below)
    text(ax, 78, 2.4, 'yTrue ->', 'FontSize', 8, 'Color', [0.3 0.3 0.5]);
    text(ax, 44, 2.4, '-> y (feedback)', 'FontSize', 8, 'Color', [0.3 0.3 0.5]);
    text(ax, 2, 9.2, 'what goes where, and why - the story below names every gap', ...
        'FontSize', 10, 'FontWeight', 'bold');

    % ---- the trace strip, in signal order ----
    t = r.t(:);
    panels = { ...
        {'r and shaped setpoint',      t, r.r,        'the target: where the story starts'}; ...
        {'e = r - y',                  t, r.e,        'what the loop must correct'}; ...
        {'P, I, D terms',              t, [r.p, r.i, r.d], 'the three answers to e'}; ...
        {'uRaw vs u (limits)',         t, [r.uRaw, r.u],   'physics: the actuator has ends'}; ...
        {'u vs uPlant (actuator)',     t, [r.u, r.uPlant], 'the chain between command and plant'}; ...
        {'uPlant -> yTrue (plant)',    t, r.yTrue,    'gain, lag, and the dead time'}; ...
        {'yTrue vs yMeas (sensor)',    t, [r.yTrue, r.y], 'the loop closes on yMeas, not yTrue'}; ...
    };
    for i = 1:numel(panels)
        ax = subplot(4, 2, i + 1, 'Parent', h);
        plot(ax, t, panels{i}{3}, 'LineWidth', 1.1);
        title(ax, panels{i}{1}, 'FontSize', 9);
        text(ax, 0.02, 0.92, panels{i}{4}, 'Units', 'normalized', ...
            'FontSize', 8, 'Color', [0.25 0.3 0.45]);
        grid(ax, 'on');
        if i < numel(panels), xlabel(ax, 't [s]', 'FontSize', 8); end
    end
end

function box(ax, x, y, w, h, txt, col)
    rectangle('Parent', ax, 'Position', [x y w h], 'FaceColor', col, ...
        'EdgeColor', [0.3 0.35 0.45], 'LineWidth', 1);
    text(ax, x + w / 2, y + h / 2, txt, 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', 'FontSize', 8, 'FontWeight', 'bold');
end

function arrow(ax, x1, y1, x2, y2)
    plot(ax, [x1 x2], [y1 y2], '-', 'Color', [0.3 0.35 0.45], 'LineWidth', 1);
    plot(ax, x2, y2, '>', 'Color', [0.3 0.35 0.45], 'MarkerSize', 5);
end

function s = derivName(m)
    K = pidx.Const;
    switch m
        case K.DERIV_ON_ERROR,          s = 'error';
        case K.DERIV_ON_WEIGHTED_ERROR, s = 'weighted error';
        otherwise,                      s = 'measurement';
    end
end

function s = awName(m)
    K = pidx.Const;
    switch m
        case K.AW_NONE,             s = 'NONE';
        case K.AW_CONDITIONAL,      s = 'CONDITIONAL';
        case K.AW_BACK_CALCULATION, s = 'BACK_CALCULATION';
        case K.AW_TRACKING,         s = 'TRACKING';
        otherwise,                  s = 'CLAMP';
    end
end

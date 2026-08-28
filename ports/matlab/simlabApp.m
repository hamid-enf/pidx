function simlabApp()
%SIMLABAPP  Graphical workbench for the PIDX simulation tool.
%
%   simlabApp
%
% Five tabs over one engine: everything here calls the same +simlab functions
% the scripts and the test suite use, so a number seen in the GUI is a number
% the tests would vouch for.
%
% LAYOUT NOTES
%   Every control uses NORMALIZED units, so the window can be resized and
%   nothing overflows it - the first real session showed the old pixel layout
%   spilling the export panel past the window edge. Sections sit in titled
%   panels with one palette; the status line at the bottom is where every
%   auto-filled decision is announced.
%
%   Runs in plain MATLAB (classic uicontrol, no App Designer) and in Octave
%   with a graphics toolkit.

    K = pidx.Const;
    S = struct();

    fig = figure('Name', 'PIDX simlab', 'NumberTitle', 'off', ...
        'MenuBar', 'none', 'ToolBar', 'none', ...
        'Color', PAL.bg, 'Position', [60 60 1180 760], ...
        'Resize', 'on');
    S.f = fig;

    % ---- header band ----
    uicontrol('Parent', fig, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0 0.955 1 0.045], 'BackgroundColor', PAL.dark, ...
        'ForegroundColor', 'w', 'FontSize', 12, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'left', 'String', ...
        sprintf('    PIDX simlab  -  simulation & tuning workbench    library %s', ...
                K.VERSION_STRING));

    % ---- status line ----
    S.status = uicontrol('Parent', fig, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0 0 1 0.035], 'BackgroundColor', PAL.status, ...
        'HorizontalAlignment', 'left', 'FontSize', 10, 'String', ' ');

    % ---- tabs ----
    tg = uitabgroup('Parent', fig, 'Units', 'normalized', ...
        'Position', [0.008 0.045 0.984 0.902]);
    S.tabPlant = uitab(tg, 'Title', '1  Plant');
    S.tabCtrl = uitab(tg, 'Title', '2  Controller');
    S.tabScen = uitab(tg, 'Title', '3  Scenario');
    S.tabRun = uitab(tg, 'Title', '4  Run');
    S.tabExport = uitab(tg, 'Title', '5  Export');

    S = buildPlantTab(S);
    S = buildCtrlTab(S);
    S = buildScenTab(S);
    S = buildRunTab(S);
    S = buildExportTab(S);

    setappdata(fig, 'simlabState', S);
    refreshAll(fig);
end

% ===========================================================================
% Palette and control factory - one look everywhere
% ===========================================================================

function P = palette()
    P.bg = [0.955 0.960 0.970];     % window / panel background
    P.dark = [0.100 0.200 0.340];   % header
    P.status = [0.880 0.910 0.950]; % status line
    P.edge = [0.760 0.790 0.830];   % panel borders
    P.btn = [0.860 0.900 0.950];    % ordinary buttons
    P.go = [0.130 0.420 0.660];     % primary action
    P.warn = [0.850 0.550 0.200];
end

function P = PAL
    % MATLAB cannot call a function before it is defined in a script-like
    % file, but local functions are visible to the whole file, so the
    % palette is just a function; this one-line alias keeps call sites short.
    P = palette();
end

function h = panel(parent, title, x, y, w, ht)
    P = PAL;
    h = uipanel('Parent', parent, 'Title', title, 'Units', 'normalized', ...
        'Position', [x y w ht], 'BackgroundColor', P.bg, ...
        'BorderColor', P.edge, 'BorderType', 'line', ...
        'FontSize', 10, 'FontWeight', 'bold', 'TitlePosition', 'lefttop');
end

function h = lbl(parent, txt, x, y, w)
    P = PAL;
    h = uicontrol('Parent', parent, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [x y w 0.05], 'String', txt, ...
        'HorizontalAlignment', 'left', 'BackgroundColor', P.bg, ...
        'FontSize', 10);
end

function h = ed(parent, x, y, w, def)
    h = uicontrol('Parent', parent, 'Style', 'edit', 'Units', 'normalized', ...
        'Position', [x y w 0.052], 'String', def, ...
        'HorizontalAlignment', 'right', 'BackgroundColor', 'w', ...
        'FontSize', 10);
end

function h = pop(parent, items, x, y, w)
    h = uicontrol('Parent', parent, 'Style', 'popupmenu', ...
        'Units', 'normalized', 'Position', [x y w 0.058], ...
        'String', items, 'BackgroundColor', 'w', 'FontSize', 10);
end

function h = btn(parent, txt, x, y, w, ht, cb, primary)
    P = PAL;
    if nargin >= 8 && primary
        bg = P.go; fg = 'w';
    else
        bg = P.btn; fg = 'k';
    end
    h = uicontrol('Parent', parent, 'Style', 'pushbutton', ...
        'Units', 'normalized', 'Position', [x y w ht], 'String', txt, ...
        'Callback', cb, 'BackgroundColor', bg, 'ForegroundColor', fg, ...
        'FontSize', 10, 'FontWeight', 'bold');
end

function h = infoBox(parent, x, y, w, ht)
    h = uicontrol('Parent', parent, 'Style', 'edit', 'Units', 'normalized', ...
        'Position', [x y w ht], 'Max', 2, 'Enable', 'inactive', ...
        'BackgroundColor', 'w', 'HorizontalAlignment', 'left', ...
        'FontSize', 9, 'FontName', 'Consolas');
end

function s = wrap(s, width)
% Wrap long lines so the info boxes never clip a compile command in half.
    if nargin < 2, width = 96; end
    out = {};
    for k = 1:numel(s)
        line = s{k};
        while numel(line) > width
            cut = find(line(1:width) == ' ', 1, 'last');
            if isempty(cut), cut = width; end
            out{end + 1} = line(1:cut); %#ok<AGROW>
            line = ['    ', line(cut + 1:end)];
        end
        out{end + 1} = line; %#ok<AGROW>
    end
    s = out;
end

% ===========================================================================
% Tab 1 - Plant
% ===========================================================================

function S = buildPlantTab(S)
    p = S.tabPlant;

    pa = panel(p, 'Process model', 0.015, 0.52, 0.46, 0.45);
    lbl(pa, 'preset', 0.04, 0.86, 0.2);
    S.plantPreset = pop(pa, {'heater', 'extruder', 'flowValve', 'pressure', ...
        'level', 'dcMotor', 'servoPos', 'slowThermal'}, 0.26, 0.85, 0.4);
    rows = {'K (static gain)', 'K', '0'; ...
            'tau (time constant, s)', 'tau', '0'; ...
            'L (dead time, s)', 'L', '0'};
    y = 0.70;
    for i = 1:size(rows, 1)
        lbl(pa, rows{i, 1}, 0.04, y, 0.45);
        S.(rows{i, 2}) = ed(pa, 0.55, y, 0.22, rows{i, 3});
        y = y - 0.13;
    end
    btn(pa, 'Apply plant', 0.04, 0.10, 0.35, 0.10, @(s, e) cbApplyPlant(s), true);
    btn(pa, 'Open-loop step test', 0.45, 0.10, 0.42, 0.10, @(s, e) cbOpenLoop(s));

    pb = panel(p, 'Actuator & sensor realism', 0.015, 0.03, 0.46, 0.46);
    rowsB = {'actuator min', 'umin', '0'; 'actuator max', 'umax', '0'; ...
             'noise sigma', 'sigma', '0'; 'ADC bits', 'bits', '0'; ...
             'ADC min', 'adcmin', '0'; 'ADC max', 'adcmax', '0'; ...
             'sensor delay (s)', 'sdelay', '0'};
    y = 0.84;
    for i = 1:size(rowsB, 1)
        lbl(pb, rowsB{i, 1}, 0.04, y, 0.42);
        S.(rowsB{i, 2}) = ed(pb, 0.55, y, 0.22, rowsB{i, 3});
        y = y - 0.12;
    end
    btn(pb, 'Model info', 0.04, 0.02, 0.30, 0.10, @(s, e) cbPlantInfo(s));

    lbl(p, 'what the model is, and what a linear analysis cannot see', ...
        0.49, 0.965, 0.5);
    S.plantInfo = infoBox(p, 0.49, 0.03, 0.495, 0.93);
end

% ===========================================================================
% Tab 2 - Controller
% ===========================================================================

function S = buildCtrlTab(S)
    p = S.tabCtrl;

    pa = panel(p, 'Gains & timing', 0.015, 0.52, 0.46, 0.45);
    rows = {'dt (sample time, s)', 'dt', '0'; 'Kp', 'Kp', '0'; ...
            'Ki', 'Ki', '0'; 'Kd', 'Kd', '0'; 'Tf (0 = Td/10)', 'Tf', '0'};
    y = 0.82;
    for i = 1:size(rows, 1)
        lbl(pa, rows{i, 1}, 0.04, y, 0.45);
        S.(rows{i, 2}) = ed(pa, 0.55, y, 0.22, rows{i, 3});
        y = y - 0.13;
    end
    lbl(pa, 'integration', 0.04, 0.14, 0.3);
    S.imethod = pop(pa, {'backward Euler', 'trapezoidal'}, 0.36, 0.13, 0.4);

    pb = panel(p, 'Structure, limits, anti-windup', 0.015, 0.03, 0.46, 0.46);
    rowsB = {'out min', 'omin', '0'; 'out max', 'omax', '0'; ...
             'beta (2DOF)', 'beta', '1'; 'gamma', 'gamma', '0'; ...
             'input LPF tau', 'lpf', '0'};
    y = 0.84;
    for i = 1:size(rowsB, 1)
        lbl(pb, rowsB{i, 1}, 0.04, y, 0.42);
        S.(rowsB{i, 2}) = ed(pb, 0.55, y, 0.22, rowsB{i, 3});
        y = y - 0.115;
    end
    lbl(pb, 'anti-windup', 0.04, 0.24, 0.3);
    S.aw = pop(pb, {'CLAMP', 'CONDITIONAL', 'BACK_CALCULATION', ...
        'TRACKING', 'NONE'}, 0.36, 0.23, 0.5);
    lbl(pb, 'derivative on', 0.04, 0.12, 0.3);
    S.dmode = pop(pb, {'measurement', 'error', 'weighted error'}, ...
        0.36, 0.11, 0.5);

    pc = panel(p, 'Actions', 0.49, 0.72, 0.22, 0.25);
    btn(pc, 'Apply controller', 0.06, 0.62, 0.88, 0.22, @(s, e) cbApplyCtrl(s), true);
    btn(pc, 'Identify (auto-tune)', 0.06, 0.36, 0.88, 0.22, @(s, e) cbTune(s));
    btn(pc, 'Suggest gains (IMC)', 0.06, 0.10, 0.88, 0.22, @(s, e) cbSuggest(s));

    lbl(p, 'what will be applied, and why', 0.725, 0.965, 0.27);
    S.ctrlInfo = infoBox(p, 0.725, 0.03, 0.26, 0.93);
end

% ===========================================================================
% Tab 3 - Scenario
% ===========================================================================

function S = buildScenTab(S)
    p = S.tabScen;

    pa = panel(p, 'Preset & horizon', 0.015, 0.52, 0.46, 0.45);
    lbl(pa, 'preset', 0.04, 0.86, 0.2);
    S.scPreset = pop(pa, {'stepResponse', 'disturbance', 'windup', 'noise', ...
        'sensorFault', 'agingPlant', 'setpointProfile'}, 0.26, 0.85, 0.45);
    rows = {'setpoint', 'scSp', '100'; 'step at t (s)', 'scT', '1'; ...
            'duration (s)', 'scEnd', '60'};
    y = 0.68;
    for i = 1:size(rows, 1)
        lbl(pa, rows{i, 1}, 0.04, y, 0.45);
        S.(rows{i, 2}) = ed(pa, 0.55, y, 0.22, rows{i, 3});
        y = y - 0.14;
    end
    btn(pa, 'Load preset', 0.04, 0.10, 0.35, 0.11, @(s, e) cbScPreset(s), true);

    pb = panel(p, 'Extra events', 0.015, 0.03, 0.46, 0.46);
    rowsB = {'disturbance size', 'scDist', '0'; ...
             'disturbance at t (s)', 'scDistT', '30'; ...
             'noise sigma', 'scNoise', '0'; ...
             'noise from t (s)', 'scNoiseT', '30'};
    y = 0.80;
    for i = 1:size(rowsB, 1)
        lbl(pb, rowsB{i, 1}, 0.04, y, 0.45);
        S.(rowsB{i, 2}) = ed(pb, 0.55, y, 0.22, rowsB{i, 3});
        y = y - 0.14;
    end
    btn(pb, 'Build custom scenario', 0.04, 0.08, 0.5, 0.12, @(s, e) cbBuildScen(s));

    lbl(p, 'the script of events, in the order they fire', 0.49, 0.965, 0.5);
    S.scList = infoBox(p, 0.49, 0.03, 0.495, 0.93);
end

% ===========================================================================
% Tab 4 - Run
% ===========================================================================

function S = buildRunTab(S)
    p = S.tabRun;

    pa = panel(p, 'Actions', 0.015, 0.80, 0.97, 0.17);
    btn(pa, 'RUN', 0.015, 0.25, 0.14, 0.5, @(s, e) cbRun(s), true);
    btn(pa, 'Monte Carlo', 0.175, 0.25, 0.16, 0.5, @(s, e) cbMC(s));
    btn(pa, 'Compare 9 rules', 0.355, 0.25, 0.16, 0.5, @(s, e) cbRules(s));
    btn(pa, 'Re-plot', 0.535, 0.25, 0.12, 0.5, @(s, e) cbReplot(s));

    S.runInfo = infoBox(p, 0.015, 0.03, 0.97, 0.75);
end

% ===========================================================================
% Tab 5 - Export
% ===========================================================================

function S = buildExportTab(S)
    p = S.tabExport;

    pa = panel(p, 'Target', 0.015, 0.55, 0.40, 0.42);
    lbl(pa, 'output directory', 0.04, 0.84, 0.3);
    S.expDir = ed(pa, 0.34, 0.84, 0.5, fullfile(pwd, 'simlab_export'));
    S.expDir = S.expDir;  % edit control; left-aligned text reads better
    set(S.expDir, 'HorizontalAlignment', 'left');
    btn(pa, 'browse...', 0.85, 0.84, 0.11, 0.06, @(s, e) cbBrowse(s));
    lbl(pa, 'C symbol prefix', 0.04, 0.66, 0.3);
    S.expSym = ed(pa, 0.34, 0.66, 0.3, 'pidxLoop');
    set(S.expSym, 'HorizontalAlignment', 'left');
    lbl(pa, 'PIDX profile', 0.04, 0.48, 0.3);
    S.expProf = pop(pa, {'FULL', 'PROCESS', 'MOTION', 'MINIMAL'}, ...
        0.34, 0.47, 0.3);
    btn(pa, 'Export C for STM32CubeIDE', 0.04, 0.16, 0.6, 0.13, ...
        @(s, e) cbExport(s), true);
    btn(pa, 'Also write CSV + JSON report', 0.04, 0.03, 0.6, 0.10, ...
        @(s, e) cbReport(s));

    lbl(p, 'what was written, and how to wire it', 0.435, 0.965, 0.55);
    S.expInfo = infoBox(p, 0.435, 0.03, 0.55, 0.93);
end

function S = get_(src)
    f = ancestor(src, 'figure');
    S = getappdata(f, 'simlabState');
    S.f = f;
end

function put_(src, S)
    setappdata(ancestor(src, 'figure'), 'simlabState', S);
end

function say(S, fmt, varargin)
    set(S.status, 'String', sprintf(fmt, varargin{:}));
    drawnow;
end

% ===========================================================================
% Tab 1: plant
% ===========================================================================

function cbPreset(src)
    S = get_(src);
    names = get(src, 'String');
    which = names{get(src, 'Value')};
    pl = simlab.Plant.presets(which);
    S.plant = pl;
    [lo, hi] = pl.actuatorLimits();
    set(S.K, 'String', num2str(pl.steadyStateGain(), '%.6g'));
    set(S.tau, 'String', num2str(pl.tau(), '%.6g'));
    set(S.L, 'String', num2str(pl.transportDelay(), '%.6g'));
    set(S.umin, 'String', num2str(lo, '%.6g'));
    set(S.umax, 'String', num2str(hi, '%.6g'));
    set(S.sigma, 'String', num2str(pl.sensorParam('sigma'), '%.6g'));
    set(S.bits, 'String', num2str(pl.sensorParam('bits')));
    set(S.adcmin, 'String', num2str(pl.sensorParam('qmin'), '%.6g'));
    set(S.adcmax, 'String', num2str(pl.sensorParam('qmax'), '%.6g'));
    set(S.sdelay, 'String', num2str(pl.sensorParam('delay'), '%.6g'));
    % Suggest a sample rate instead of leaving a zero that dead-ends the
    % first Apply: a twentieth of the dominant time constant, floored.
    set(S.dt, 'String', num2str(max(pl.tau() / 20, 1e-4), '%.6g'));
    say(S, 'preset "%s" loaded - press Apply plant to use it', which);
    put_(src, S);
end

function cbApplyPlant(src)
    S = get_(src);
    K = str2double(get(S.K, 'String'));
    tau = str2double(get(S.tau, 'String'));
    L = str2double(get(S.L, 'String'));
    umin = str2double(get(S.umin, 'String'));
    umax = str2double(get(S.umax, 'String'));
    sigma = str2double(get(S.sigma, 'String'));
    bits = round(str2double(get(S.bits, 'String')));
    adcmin = str2double(get(S.adcmin, 'String'));
    adcmax = str2double(get(S.adcmax, 'String'));
    sdelay = str2double(get(S.sdelay, 'String'));

    if any(isnan([K, tau, L])) || tau <= 0
        say(S, 'K and tau must be numbers and tau must be > 0');
        return;
    end
    S.plant = simlab.Plant('fopdt', 'name', 'custom', 'k', K, ...
        'tau', tau, 'l', max(L, 0));
    if umax > umin
        S.plant.setActuatorLimits(umin, umax);
    end
    if sigma > 0, S.plant.setNoise(sigma); end
    if bits > 0 && adcmax > adcmin
        S.plant.setAdcBits(bits, adcmin, adcmax);
    end
    if sdelay > 0, S.plant.setSensorDelay(sdelay); end

    S.result = [];      % an old result described a different plant
    S.sens = [];
    showPlantInfo(S);
    say(S, 'plant applied: K=%.4g tau=%.4g L=%.4g', K, tau, max(L, 0));
    put_(src, S);
end

function showPlantInfo(S)
    % (lines are wrapped at the end so long caveats never clip)
    if isempty(S.plant)
        set(S.plantInfo, 'String', 'no plant yet');
        return;
    end
    pl = S.plant;
    L = {};
    L{end + 1} = sprintf('name        %s', pl.name);
    L{end + 1} = sprintf('kind        %s', pl.kind);
    L{end + 1} = sprintf('K           %.6g', pl.steadyStateGain());
    L{end + 1} = sprintf('tau         %.6g s', pl.tau());
    L{end + 1} = sprintf('dead time   %.6g s', pl.transportDelay());
    [z, p, k0, ld] = pl.polesZeros(); %#ok<ASGLU>
    L{end + 1} = sprintf('poles       %s', mat2str(p, 5));
    L{end + 1} = sprintf('zeros       %s', mat2str(z, 5));
    [lo, hi] = pl.actuatorLimits();
    L{end + 1} = sprintf('actuator    [%.6g, %.6g]', lo, hi);
    L{end + 1} = sprintf('noise       sigma %.6g', pl.sensorParam('sigma'));
    L{end + 1} = sprintf('ADC         %d bits over [%.6g, %.6g]', ...
        pl.sensorParam('bits'), pl.sensorParam('qmin'), pl.sensorParam('qmax'));
    L{end + 1} = sprintf('sensor dly  %.6g s', pl.sensorParam('delay'));
    L{end + 1} = '';
    L{end + 1} = 'what a linear analysis does NOT see:';
    cav = pl.analysisCaveats();
    for i = 1:numel(cav)
        L{end + 1} = sprintf('  - %s', cav{i});
    end
    set(S.plantInfo, 'String', strjoin(wrap(L), sprintf('\n')));
end

function cbPlantInfo(src)
    S = get_(src);
    if isempty(S.plant)
        say(S, 'apply a plant first');
        return;
    end
    showPlantInfo(S);
    say(S, 'model info refreshed');
end

function cbOpenLoop(src)
    S = get_(src);
    if isempty(S.plant)
        say(S, 'apply a plant first');
        return;
    end
    % Driving the plant from a controller held in MANUAL at a fixed output IS
    % an open-loop step test - the same thing the step identification does.
    dt = suggestedDt(S.plant);
    c = pidx.PID(pidx.config('dt', dt));
    c.setMode(pidx.Const.MODE_MANUAL);
    [lo, hi] = S.plant.actuatorLimits();
    if ~isfinite(hi), hi = 1; end
    if ~isfinite(lo), lo = 0; end
    c.setManualOutput(lo + 0.5 * (hi - lo));
    sc = simlab.Scenario('open-loop step', defaultHorizon(S.plant));
    sc.setpoint(0, 0);
    r = simlab.Sim(S.plant, c, sc).run();
    simlab.plot(r, struct('fig', 90, 'title', ...
        'open-loop step: measure K, tau and L off this'));
    say(S, 'open-loop step plotted (figure 90)');
end

% ===========================================================================
% Tab 2: controller
% ===========================================================================

function cbApplyCtrl(src)
    S = get_(src);
    dt = str2double(get(S.dt, 'String'));
    if ~(dt > 0)
        say(S, 'dt must be a number greater than zero');
        return;
    end
    cfg = pidx.config('kp', num(get(S.Kp)), 'ki', num(get(S.Ki)), ...
        'kd', num(get(S.Kd)), 'dt', dt);
    omin = num(get(S.omin));
    omax = num(get(S.omax));
    if omax > omin
        cfg.limits.use_output_limits = true;
        cfg.limits.output_min = omin;
        cfg.limits.output_max = omax;
    end
    K = pidx.Const;
    awIds = [K.AW_CLAMP, K.AW_CONDITIONAL, K.AW_BACK_CALCULATION, ...
             K.AW_TRACKING, K.AW_NONE];
    cfg.integral.mode = awIds(get(S.aw, 'Value'));
    dIds = [K.DERIV_ON_MEASUREMENT, K.DERIV_ON_ERROR, ...
            K.DERIV_ON_WEIGHTED_ERROR];
    cfg.filter.derivative_mode = dIds(get(S.dmode, 'Value'));
    iIds = [K.INTEGRATION_BACKWARD_EULER, K.INTEGRATION_TRAPEZOIDAL];
    cfg.core.integration = iIds(get(S.imethod, 'Value'));
    cfg.filter.tf = max(num(get(S.Tf)), 0);
    cfg.filter.input_lpf_tau = max(num(get(S.lpf)), 0);
    cfg.weight.beta = num(get(S.beta));
    cfg.weight.gamma = num(get(S.gamma));
    if cfg.weight.beta == 0 && isempty(strtrim(get(S.beta, 'String')))
        cfg.weight.beta = 1;
    end

    try
        S.ctrl = pidx.PID(cfg);
    catch err
        say(S, 'controller rejected: %s', err.message);
        return;
    end
    S.cfg = cfg;
    S.result = [];
    showCtrlInfo(S, '');
    say(S, 'controller applied: Kp=%.5g Ki=%.5g Kd=%.5g dt=%.4g', ...
        cfg.core.kp, cfg.core.ki, cfg.core.kd, dt);
    put_(src, S);
end

function cbSuggest(src)
    S = get_(src);
    if isempty(S.plant)
        say(S, 'apply a plant first');
        return;
    end
    K = pidx.Const;
    dt = num(get(S.dt, 'String'));
    if ~(dt > 0), dt = suggestedDt(S.plant); end
    m = pidx.plantModel(K.MODEL_FOPDT, S.plant.steadyStateGain(), ...
        S.plant.tau(), S.plant.transportDelay());
    [rc, g] = pidx.ruleApply(K.RULE_IMC, m, K.STRUCT_PID, 0);
    if rc ~= K.OK
        say(S, 'IMC could not be applied: %s', K.statusToString(rc));
        return;
    end
    set(S.dt, 'String', num2str(dt, '%.6g'));
    set(S.Kp, 'String', num2str(g.kp, '%.6g'));
    set(S.Ki, 'String', num2str(g.ki, '%.6g'));
    set(S.Kd, 'String', num2str(g.kd, '%.6g'));
    set(S.Tf, 'String', num2str(g.tf, '%.6g'));
    showCtrlInfo(S, sprintf( ...
        'IMC / lambda tuning on the nominal model.\n\n  K = %.6g  T = %.6g s  L = %.6g s\n  lambda = max(0.5*L, 0.2*T) = %.6g s\n\nThis is the TRUE model of the simulation plant, which you do not have on the board. Use Identify to get an honest model from data.', ...
        m.k, m.t, m.l, max(0.5 * m.l, 0.2 * m.t)));
    say(S, 'IMC gains suggested from the nominal model - press Apply controller');
    put_(src, S);
end

function cbTune(src)
    S = get_(src);
    if isempty(S.plant)
        say(S, 'apply a plant first');
        return;
    end
    K = pidx.Const;
    dt = num(get(S.dt, 'String'));
    if ~(dt > 0), dt = suggestedDt(S.plant); end

    choice = questdlg( ...
        ['Relay: closed loop, keeps the process near its setpoint, gives ' ...
         '(Ku, Pu). Step: open loop, drives the process away from its ' ...
         'operating point, gives a full FOPDT model (K, T, L).'], ...
        'Identification method', 'Relay', 'Step', 'Cancel', 'Relay');
    if isempty(choice) || strcmp(choice, 'Cancel')
        return;
    end
    if strcmp(choice, 'Relay')
        ident = K.IDENT_RELAY;
        rule = K.RULE_TYREUS_LUYBEN;
    else
        ident = K.IDENT_STEP;
        rule = K.RULE_AMIGO_STEP;
    end

    cfgT = simlab.AutoTune.configDefault(ident);
    cfgT.rule = rule;
    [lo, hi] = S.plant.actuatorLimits();
    if ~isfinite(lo), lo = 0; end
    if ~isfinite(hi), hi = 100; end
    span = hi - lo;
    cfgT.output_step = 0.2 * span;
    cfgT.hysteresis = max(2 * S.plant.sensorParam('sigma'), 0);
    cfgT.bias = lo + 0.5 * span;
    cfgT.auto_bias = false;
    cfgT.output_min = lo;
    cfgT.output_max = hi;
    cfgT.timeout_s = 20 * max(S.plant.tau(), 1);

    at = simlab.AutoTune(cfgT);
    c = pidx.PID(pidx.config('dt', dt));
    sp = S.plant.steadyStateGain() * cfgT.bias;
    S.plant.reset();
    at.start(c, sp);

    y = 0;
    nMax = round(cfgT.timeout_s / dt);
    tr = zeros(nMax, 3);
    for k = 1:nMax
        u = at.update(y, dt);
        y = S.plant.update(u, dt);
        tr(k, :) = [k * dt, y, u];
        if mod(k, max(1, floor(nMax / 100))) == 0
            say(S, 'identifying... %s %d%%', ...
                simlab.AutoTune.stateToString(at.getState()), at.getProgress());
        end
        if ~at.isRunning(), break; end
    end
    tr = tr(1:k, :);

    figure(94); clf;
    plot(tr(:, 1), tr(:, 2), 'b', 'LineWidth', 1.2); hold on;
    plot(tr(:, 1), tr(:, 3), 'r', 'LineWidth', 0.9);
    legend('measurement', 'injected output', 'Location', 'best');
    grid on;
    title(sprintf('%s identification  -  %s', choice, ...
        simlab.AutoTune.stateToString(at.getState())));
    xlabel('time [s]');

    [rc, res] = at.getResult();
    if rc ~= K.OK
        showCtrlInfo(S, sprintf('identification FAILED: %s\n\nThe tuner refuses to return gains it does not believe. See docs/14_autotune.md.', ...
            K.statusToString(rc)));
        say(S, 'identification failed: %s', K.statusToString(rc));
        return;
    end

    S.tune = res;
    g = res.gains;
    set(S.dt, 'String', num2str(dt, '%.6g'));
    set(S.Kp, 'String', num2str(g.kp, '%.6g'));
    set(S.Ki, 'String', num2str(g.ki, '%.6g'));
    set(S.Kd, 'String', num2str(g.kd, '%.6g'));
    set(S.Tf, 'String', num2str(g.tf, '%.6g'));
    set(S.omin, 'String', num2str(lo, '%.6g'));
    set(S.omax, 'String', num2str(hi, '%.6g'));

    txt = {};
    txt{end + 1} = sprintf('identification: %s', choice);
    if res.model.kind == K.MODEL_FREQ
        txt{end + 1} = sprintf('  Ku = %.6g', res.model.ku);
        txt{end + 1} = sprintf('  Pu = %.6g s', res.model.pu);
    else
        txt{end + 1} = sprintf('  K  = %.6g', res.model.k);
        txt{end + 1} = sprintf('  T  = %.6g s', res.model.t);
        txt{end + 1} = sprintf('  L  = %.6g s', res.model.l);
    end
    txt{end + 1} = sprintf('  quality %d/100, %d cycle(s), asymmetry %.3f', ...
        double(res.model.quality), res.cycles_used, res.asymmetry);
    txt{end + 1} = '';
    txt{end + 1} = 'gains:';
    txt{end + 1} = sprintf('  Kp = %.6g   Ti = %.6g s', g.kp, g.ti);
    txt{end + 1} = sprintf('  Ki = %.6g   Td = %.6g s', g.ki, g.td);
    txt{end + 1} = sprintf('  Kd = %.6g   Tf = %.6g s', g.kd, g.tf);
    txt{end + 1} = '';
    txt{end + 1} = 'Press "Apply controller", then run the scenario.';
    if res.asymmetric
        txt{end + 1} = '';
        txt{end + 1} = 'WARNING: asymmetry above 0.30 - the plant is';
        txt{end + 1} = 'nonlinear or the bias is wrong at this point.';
    end
    showCtrlInfo(S, strjoin(txt, sprintf('\n')));
    say(S, 'identified. Kp=%.5g Ki=%.5g Kd=%.5g - press Apply controller', ...
        g.kp, g.ki, g.kd);
    put_(src, S);
end

function showCtrlInfo(S, txt)
    if isempty(S.cfg)
        set(S.ctrlInfo, 'String', txt);
        return;
    end
    c = S.cfg;
    L = {};
    if ~isempty(txt)
        L{end + 1} = txt;
        L{end + 1} = '';
        L{end + 1} = '---------------------------------------------';
    end
    L{end + 1} = sprintf('dt        %.6g s   (%.4g Hz)', c.core.sample_time, ...
        1 / c.core.sample_time);
    L{end + 1} = sprintf('Kp        %.6g', c.core.kp);
    if c.core.ki > 0
        L{end + 1} = sprintf('Ki        %.6g   (Ti = %.6g s)', c.core.ki, ...
            c.core.kp / c.core.ki);
    else
        L{end + 1} = 'Ki        0   (no integral action: steady-state error will remain)';
    end
    if c.core.kd > 0
        L{end + 1} = sprintf('Kd        %.6g   (Td = %.6g s)', c.core.kd, ...
            c.core.kd / c.core.kp);
    else
        L{end + 1} = 'Kd        0';
    end
    L{end + 1} = sprintf('Tf        %.6g s', c.filter.tf);
    L{end + 1} = sprintf('beta/gamma %.3g / %.3g', c.weight.beta, c.weight.gamma);
    if c.limits.use_output_limits
        L{end + 1} = sprintf('output    [%.6g, %.6g]', c.limits.output_min, ...
            c.limits.output_max);
    else
        L{end + 1} = 'output    UNLIMITED - the integrator can wind up without bound';
    end
    set(S.ctrlInfo, 'String', strjoin(wrap(L), sprintf('\n')));
end

% ===========================================================================
% Tab 3: scenario
% ===========================================================================

function cbScPreset(src)
    S = get_(src);
    names = get(src, 'String');
    which = names{get(src, 'Value')};
    sp = num(get(S.scSp));
    tEnd = num(get(S.scEnd));
    if ~(tEnd > 0)
        % An empty or zero duration field must not produce a zero-length
        % scenario - that is the "0 samples" dead-end, renamed.
        tEnd = defaultHorizon(S.plant);
        say(S, 'duration was not usable - using %.5g s (12x tau+L)', tEnd);
    end
    S.scenario = simlab.Scenario.presets(which, 'sp', sp, 'tEnd', tEnd);
    set(S.scList, 'String', strjoin(wrap(cellstr(S.scenario.describe())), sprintf('\n')));
    say(S, 'scenario "%s" built - %d events over %.4g s', which, ...
        S.scenario.nEvents, S.scenario.tEnd);
    put_(src, S);
end

function cbBuildScen(src)
    S = get_(src);
    sp = num(get(S.scSp));
    t0 = num(get(S.scT));
    tEnd = num(get(S.scEnd));
    if ~(tEnd > 0)
        tEnd = defaultHorizon(S.plant);
    end
    sc = simlab.Scenario('custom', tEnd);
    sc.setpoint(0, 0);
    sc.setpoint(sp, t0);
    if num(get(S.scDist)) ~= 0
        sc.loadStep(num(get(S.scDist)), num(get(S.scDistT)));
    end
    if num(get(S.scNoise)) > 0
        sc.noise(num(get(S.scNoise)), num(get(S.scNoiseT)));
    end
    S.scenario = sc;
    set(S.scList, 'String', sc.describe());
    say(S, 'custom scenario built - %d events over %.4g s', sc.nEvents, sc.tEnd);
    put_(src, S);
end

% ===========================================================================
% Tab 4: run
% ===========================================================================

function cbRun(src)
    S = get_(src);
    if isempty(S.plant)
        say(S, 'apply a plant first (tab 1)');
        return;
    end
    % A Run pressed on a half-configured session must still produce a result,
    % with what was auto-filled printed in the status line. Dead-ends are how
    % a workbench loses its user.
    if isempty(S.ctrl)
        [S.ctrl, S.cfg] = makeDefaultController(S);
        say(S, 'no controller applied - using IMC on the nominal model: Kp=%.5g Ki=%.5g Kd=%.5g', ...
            S.cfg.core.kp, S.cfg.core.ki, S.cfg.core.kd);
    end
    dt = S.ctrl.getSampleTime();
    if isempty(S.scenario) || ~(S.scenario.tEnd >= 2 * dt)
        sp = defaultSetpoint(S.plant);
        T = max(defaultHorizon(S.plant), 10 * dt);
        S.scenario = simlab.Scenario.presets('stepResponse', 'sp', sp, 'tEnd', T);
        say(S, 'no usable scenario - built a step to %.5g over %.5g s', sp, T);
    end
    say(S, 'running...');
    S.plant.reset();
    S.result = simlab.Sim(S.plant, S.ctrl, S.scenario).run();
    try
        S.sens = simlab.sensitivity(S.plant, struct('kp', S.cfg.core.kp, ...
            'ki', S.cfg.core.ki, 'kd', S.cfg.core.kd), ...
            struct('dt', S.cfg.core.sample_time, 'tf', S.cfg.filter.tf, ...
                   'lpfTau', S.cfg.filter.input_lpf_tau));
    catch err
        S.sens = [];
        say(S, 'ran, but the frequency analysis failed: %s', err.message);
    end
    simlab.plot(S.result, struct('fig', 91));
    if ~isempty(S.sens)
        simlab.plotSensitivity(S.sens, struct('fig', 92));
    end
    showRunInfo(S);
    say(S, 'done - overshoot %.1f%%, settling %.4g s', ...
        S.result.metrics.overshoot, S.result.metrics.settlingTime);
    put_(src, S);
end

function cbReplot(src)
    S = get_(src);
    if isempty(S.result)
        say(S, 'nothing to re-plot yet');
        return;
    end
    simlab.plot(S.result, struct('fig', 91));
    if ~isempty(S.sens)
        simlab.plotSensitivity(S.sens, struct('fig', 92));
    end
    say(S, 're-plotted');
end

function cbMC(src)
    S = get_(src);
    if isempty(S.plant) || isempty(S.cfg)
        say(S, 'a plant and a controller are required');
        return;
    end
    n = inputdlg('How many perturbed plants?', 'Monte Carlo', 1, {'60'});
    if isempty(n)
        return;          % Cancel was pressed
    end
    n = str2double(n{1});
    if isnan(n) || n < 2, n = 60; end

    g = struct('kp', S.cfg.core.kp, 'ki', S.cfg.core.ki, ...
        'kd', S.cfg.core.kd, 'dt', S.cfg.core.sample_time);
    [lo, hi] = S.plant.actuatorLimits();
    if isfinite(lo) && isfinite(hi)
        g.outMin = lo; g.outMax = hi;
    end
    if S.cfg.filter.tf > 0, g.tf = S.cfg.filter.tf; end
    sc = S.scenario;
    if isempty(sc), sc = simlab.Scenario.presets('disturbance'); end

    say(S, 'running %d perturbed plants...', n);
    mc = simlab.monteCarlo(S.plant, g, struct('nRuns', n, 'spread', 2, ...
        'scenario', sc, 'dt', S.cfg.core.sample_time, 'verbose', false));

    L = {};
    L{end + 1} = sprintf('MONTE CARLO - %d plants, each with K, tau and L', n);
    L{end + 1} = 'scaled independently over 0.5x .. 2x';
    L{end + 1} = '';
    L{end + 1} = sprintf('  survived   %.0f%%', 100 * mc.share);
    L{end + 1} = sprintf('  median IAE %.6g', mc.iae.median);
    L{end + 1} = sprintf('  worst IAE  %.6g', mc.iae.worst);
    L{end + 1} = sprintf('  IQR        %.6g', mc.iae.iqr);
    L{end + 1} = sprintf('  worst at   K x%.2f, tau x%.2f, L x%.2f', ...
        mc.worstPlant(1), mc.worstPlant(2), mc.worstPlant(3));
    L{end + 1} = sprintf('  nominal    IAE %.6g, OS %.1f%%, settle %.4g s', ...
        mc.nominal.iae, mc.nominal.overshoot, mc.nominal.settlingTime);
    L{end + 1} = '';
    if mc.share < 0.9
        L{end + 1} = 'BELOW 90%: these gains depend on the model being right.';
        L{end + 1} = 'Detune (lower Kp, raise Ti) or improve the model.';
    else
        L{end + 1} = 'At or above 90%: the tuning survives a model that is';
        L{end + 1} = 'up to 2x wrong in any parameter.';
    end
    set(S.runInfo, 'String', strjoin(wrap(L), sprintf('\n')));
    say(S, 'Monte Carlo done: %.0f%% survived', 100 * mc.share);
    put_(src, S);
end

function cbRules(src)
    S = get_(src);
    if isempty(S.plant)
        say(S, 'apply a plant first');
        return;
    end
    choice = questdlg( ...
        ['exact: the identified model IS the plant (how fast could this be). ' ...
         'robust: plants perturbed 0.5x..2x (how often will it work). ' ...
         'noisy: exact model, much worse sensor.'], ...
        'Compare the nine tuning rules', 'robust', 'exact', 'noisy', 'robust');
    if isempty(choice), return; end
    dt = num(get(S.dt, 'String'));
    if ~(dt > 0), dt = suggestedDt(S.plant); end
    sc = S.scenario;
    if isempty(sc), sc = simlab.Scenario.presets('stepResponse'); end

    say(S, 'comparing nine rules, this takes a moment...');
    cr = simlab.compareRules(S.plant, struct('mode', choice, 'nRuns', 20, ...
        'dt', dt, 'scenario', sc, 'verbose', false));
    simlab.plotRules(cr, struct('fig', 93));

    L = {};
    L{end + 1} = sprintf('TUNING RULES on this plant  [mode: %s]', cr.mode);
    L{end + 1} = '';
    L{end + 1} = sprintf('%-16s %-10s %-10s %-10s %-8s %-8s', ...
        'rule', 'Kp', 'Ki', 'IAE', 'OS %', 'surv %');
    t = cr.table;
    for i = 1:numel(t.ok)
        if ~t.ok(i)
            L{end + 1} = sprintf('%-16s %s', t.name{i}, t.note{i});
            continue;
        end
        L{end + 1} = sprintf('%-16s %-10.4g %-10.4g %-10.4g %-8.3g %-8.0f', ...
            t.name{i}, t.kp(i), t.ki(i), t.iae(i), t.overshoot(i), ...
            100 * t.survival(i));
    end
    L{end + 1} = '';
    L{end + 1} = sprintf('Spearman rho between the two rankings: %.3f', cr.spearman);
    L{end + 1} = cr.rankCorrelationNote;
    L{end + 1} = '';
    L{end + 1} = cr.recommend;
    set(S.runInfo, 'String', strjoin(wrap(L), sprintf('\n')));
    say(S, 'rule comparison done - see figure 93');
    put_(src, S);
end

function showRunInfo(S)
    m = S.result.metrics;
    L = {};
    L{end + 1} = 'METRICS';
    L{end + 1} = sprintf('  rise 10-90%%   %12.5g s', m.riseTime);
    L{end + 1} = sprintf('  overshoot     %12.5g %%', m.overshoot);
    L{end + 1} = sprintf('  settling 2%%   %12.5g s', m.settlingTime);
    L{end + 1} = sprintf('  steady error  %12.5g', m.ssError);
    L{end + 1} = sprintf('  IAE / ITAE    %12.5g / %.5g', m.iae, m.itae);
    L{end + 1} = sprintf('  TV(u)         %12.5g', m.tv);
    L{end + 1} = sprintf('  peak u        %12.5g', m.uPeak);
    L{end + 1} = sprintf('  saturated     %12.5g %% of samples', ...
        100 * m.satFraction);
    L{end + 1} = sprintf('  stable        %12s', mat2str(logical(m.stable)));
    if ~isempty(S.sens)
        s = S.sens;
        L{end + 1} = '';
        L{end + 1} = 'LOOP MARGINS (linear analysis)';
        L{end + 1} = sprintf('  Ms            %12.4g   (<1.4 good, >2 fragile)', s.Ms);
        L{end + 1} = sprintf('  gain margin   %12.4g x', s.gm);
        L{end + 1} = sprintf('  phase margin  %12.4g deg', s.pm);
        L{end + 1} = sprintf('  delay margin  %12.4g s', s.delayMargin);
        L{end + 1} = sprintf('  bandwidth     %12.4g rad/s', s.bandwidth);
        L{end + 1} = sprintf('  %s', s.verdict);
        for i = 1:numel(s.warnings)
            L{end + 1} = sprintf('  ! %s', s.warnings{i});
        end
    end
    L{end + 1} = '';
    L{end + 1} = 'SCENARIO';
    L{end + 1} = S.result.scenario;
    set(S.runInfo, 'String', strjoin(wrap(L), sprintf('\n')));
end

% ===========================================================================
% Tab 5: export
% ===========================================================================

function cbBrowse(src)
    S = get_(src);
    d = uigetdir(get(S.expDir, 'String'), 'output directory');
    if ischar(d) && ~isempty(d)
        set(S.expDir, 'String', d);
    end
end

function cbExport(src)
    S = get_(src);
    if isempty(S.plant) || isempty(S.cfg)
        say(S, 'a plant and a controller are required before exporting');
        return;
    end
    profs = get(S.expProf, 'String');
    out = simlab.exportSTM32(S.plant, S.cfg, struct( ...
        'dir', get(S.expDir, 'String'), 'symbol', get(S.expSym, 'String'), ...
        'profile', profs{get(S.expProf, 'Value')}, 'result', S.result, ...
        'sens', S.sens, 'tune', S.tune, 'quiet', true));

    L = {};
    L{end + 1} = 'WRITTEN';
    L{end + 1} = sprintf('  %s', out.header);
    L{end + 1} = sprintf('  %s', out.source);
    L{end + 1} = '';
    L{end + 1} = 'COMPILE CHECK';
    L{end + 1} = sprintf('  %s', out.compileCmd);
    L{end + 1} = '';
    if isempty(out.dropped)
        L{end + 1} = 'every configured feature is present in the generated code.';
    else
        L{end + 1} = sprintf('NOT EXPORTED - absent from PIDX_PROFILE_%s:', ...
            profs{get(S.expProf, 'Value')});
        for i = 1:numel(out.dropped)
            L{end + 1} = sprintf('  - %s', out.dropped{i});
        end
        L{end + 1} = '';
        L{end + 1} = 'The loop will run WITHOUT those features. Either switch';
        L{end + 1} = 'profile or drop them from the controller.';
    end
    L{end + 1} = '';
    L{end + 1} = 'IN YOUR CUBEIDE PROJECT';
    L{end + 1} = '  1. add the PIDX src/*.c files to the project';
    L{end + 1} = '  2. put include/ on the include path';
    L{end + 1} = '  3. copy these two files into Core/Src';
    L{end + 1} = sprintf('  4. call %s_init() once, after HAL_Init()', out.symbol);
    L{end + 1} = sprintf('  5. call %s_tick(y) from the timer ISR at %.0f Hz', ...
        out.symbol, 1 / S.cfg.core.sample_time);
    L{end + 1} = '';
    L{end + 1} = 'The generated file is verified to compile and to reproduce';
    L{end + 1} = 'the MATLAB output by simlab_tests/test_export.m.';
    set(S.expInfo, 'String', strjoin(wrap(L), sprintf('\n')));
    say(S, 'exported to %s', out.dir);
    put_(src, S);
end

function cbReport(src)
    S = get_(src);
    if isempty(S.result)
        say(S, 'run a simulation first');
        return;
    end
    simlab.exportReport(S.result, struct('dir', get(S.expDir, 'String'), ...
        'name', get(S.expSym, 'String'), 'cfg', S.cfg, 'plant', S.plant, ...
        'sens', S.sens, 'tune', S.tune));
    say(S, 'report written');
end

% ===========================================================================
% Helpers
% ===========================================================================

function refreshAll(f)
    S = getappdata(f, 'simlabState');
    S.f = f;
    % Load the heater preset so the window is not empty on first open, and
    % say so - a default that arrives silently gets mistaken for a setting.
    names = get(S.plantPreset, 'String');
    set(S.plantPreset, 'Value', 1);
    cbPreset(S.plantPreset);
    S = getappdata(f, 'simlabState');
    say(S, 'preset "%s" loaded - press Apply plant, then set the controller', ...
        names{1});
end

function dt = suggestedDt(pl)
    % A twentieth of the dominant time constant, floored so a fast plant does
    % not produce an absurdly small step. An INTEGRATING plant (tank level,
    % servo position) has no dominant time constant - tau() is Inf - and Inf
    % is not a sample time: pidx.PID rejects it with ERR_INVALID_DT. Fall
    % back to 0.1 s and let the operator refine.
    tau = pl.tau();
    if ~isfinite(tau) || tau <= 0
        dt = 0.1;
    else
        dt = max(tau / 20, 1e-4);
    end
end

function T = defaultHorizon(pl)
    % A scenario long enough to see the whole response: 12 time constants
    % plus the dead time. Integrating plants get a finite stand-in, because a
    % horizon of Inf would make every run a zero-sample run.
    tau = pl.tau();
    L = pl.transportDelay();
    if ~isfinite(tau) || tau <= 0, tau = 10; end
    if ~isfinite(L)   || L < 0,    L = 0;   end
    T = max(12 * (tau + L), 1);
end

function sp = defaultSetpoint(pl)
    % Half the actuator span mapped through the plant gain: large enough to
    % be meaningful, small enough not to spend the run saturated.
    [lo, hi] = pl.actuatorLimits();
    if ~isfinite(lo), lo = 0; end
    if ~isfinite(hi), hi = 1; end
    k = pl.steadyStateGain();
    if ~isfinite(k) || k <= 0, k = 1; end
    sp = 0.5 * (hi - lo) * k;
    if ~isfinite(sp) || sp == 0, sp = 1; end
end

function [ctrl, cfg] = makeDefaultController(S)
    % IMC on the nominal model, so a Run pressed before any tuning still has
    % a controller. A starting point that is printed, not a hidden default.
    K = pidx.Const;
    dt = suggestedDt(S.plant);
    m = pidx.plantModel(K.MODEL_FOPDT, S.plant.steadyStateGain(), ...
        S.plant.tau(), S.plant.transportDelay());
    g = [];
    if isfinite(S.plant.steadyStateGain()) && isfinite(S.plant.tau())
        [rc, g] = pidx.ruleApply(K.RULE_IMC, m, K.STRUCT_PID, 0);
        if rc ~= K.OK, g = []; end
    end
    if isempty(g)
        g = struct('kp', 1.0, 'ki', 0.1, 'kd', 0, 'tf', 0);
    end
    cfg = pidx.config('kp', g.kp, 'ki', g.ki, 'kd', g.kd, 'dt', dt);
    if g.tf > 0, cfg.filter.tf = g.tf; end
    [lo, hi] = S.plant.actuatorLimits();
    if isfinite(lo) && isfinite(hi)
        cfg.limits.use_output_limits = true;
        cfg.limits.output_min = lo;
        cfg.limits.output_max = hi;
    end
    ctrl = pidx.PID(cfg);
end

function v = num(str)
    v = str2double(str);
    if isnan(v), v = 0; end
end

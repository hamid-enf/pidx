function out = exportSTM32(plant, cfg, opt)
%SIMLAB.EXPORTSTM32  Turn a verified MATLAB configuration into C that compiles.
%
%   OUT = SIMLAB.EXPORTSTM32(PLANT, CFG)
%   OUT = SIMLAB.EXPORTSTM32(PLANT, CFG, 'result', r, 'dir', 'out', ...
%                            'symbol', 'tempLoop', 'profile', 'FULL')
%
% This is the reason the rest of the tool exists. You tune in MATLAB because
% iterating there takes seconds; you flash C because that is what the STM32
% runs. The gap between those two is where gains get mistyped, a filter gets
% forgotten, and a loop that measured beautifully in simulation behaves
% differently on the board. This function closes the gap mechanically.
%
% WHAT IS WRITTEN
%   <dir>/pidx_tuning_<symbol>.h    every number as a #define, with units,
%                                   plus the simulation metrics as comments
%   <dir>/pidx_tuning_<symbol>.c    <symbol>_init() and <symbol>_tick(y),
%                                   built from the real PIDX API
%
% Both files compile as-is against this repository:
%   gcc -std=c99 -DPIDX_PROFILE_FULL -Iinclude -c pidx_tuning_<symbol>.c
% The generated C is verified that way by simlab_tests/test_export.m, which
% compiles it and runs the exported loop against the exported plant model.
% That is the check that the export is not a paraphrase.
%
% OPTIONS
%   'dir'      output directory. Default './simlab_export'.
%   'symbol'   C identifier prefix. Default 'pidxLoop'.
%   'profile'  PIDX profile the target is built with: MINIMAL, MOTION,
%              PROCESS, FULL. Default FULL. Features the profile does not
%              contain are written as a #warning comment instead of code, so
%              the file still compiles - a silently dropped feature is the
%              worst possible outcome of an export.
%   'result'   a simlab.Sim result. Its metrics go into the header as the
%              record of what these gains were measured to do.
%   'sens'     a simlab.sensitivity result. Ms and the margins go in too.
%   'gains'    gains to override cfg.core with (e.g. from an auto-tune).
%   'extra'    extra text lines for the file header.
%   'cascade'  a simlab.Cascade, exported instead of a single loop.
%
% WHAT IS *NOT* DONE, DELIBERATELY
%   No HAL code is generated. Which timer, which ADC channel and which ISR
%   owns the loop are decisions about your board, and a generator that
%   guesses them produces code you have to read anyway. The generated
%   <symbol>_tick() takes the measurement and returns the command, which is
%   the two lines that connect to your ISR.

    if nargin < 3, opt = struct(); end
    o = fillOpt(opt, 'dir', fullfile(pwd, 'simlab_export'));
    o = fillOpt(o, 'symbol', 'pidxLoop');
    o = fillOpt(o, 'profile', 'FULL');
    o = fillOpt(o, 'result', []);
    o = fillOpt(o, 'sens', []);
    o = fillOpt(o, 'gains', []);
    o = fillOpt(o, 'extra', {});
    o = fillOpt(o, 'cascade', []);
    o = fillOpt(o, 'quiet', false);

    if ~isempty(o.gains)
        cfg.core.kp = o.gains.kp;
        cfg.core.ki = o.gains.ki;
        cfg.core.kd = o.gains.kd;
        if isfield(o.gains, 'tf') && o.gains.tf > 0
            cfg.filter.tf = o.gains.tf;
        end
    end

    if ~exist(o.dir, 'dir')
        mkdir(o.dir);
    end

    guard = upper(['PIDX_TUNING_' o.symbol '_H']);
    hdrPath = fullfile(o.dir, sprintf('pidx_tuning_%s.h', o.symbol));
    srcPath = fullfile(o.dir, sprintf('pidx_tuning_%s.c', o.symbol));

    [hdr, o] = buildHeader(plant, cfg, o, guard);
    [src, o] = buildSource(cfg, o);

    fid = fopen(hdrPath, 'w');
    fprintf(fid, '%s', hdr);
    fclose(fid);
    fid = fopen(srcPath, 'w');
    fprintf(fid, '%s', src);
    fclose(fid);

    out = struct();
    out.header = hdrPath;
    out.source = srcPath;
    out.dir = o.dir;
    out.symbol = o.symbol;
    out.headerText = hdr;
    out.sourceText = src;
    out.dropped = o.dropped;
    out.compileCmd = sprintf( ...
        'gcc -std=c99 -DPIDX_PROFILE_%s -I<repo>/include -c %s', ...
        o.profile, srcPath);

    if ~o.quiet
        fprintf('\nsimlab.exportSTM32: wrote\n  %s\n  %s\n', hdrPath, srcPath);
        fprintf('  compile check: %s\n', out.compileCmd);
        if ~isempty(o.dropped)
            fprintf('  NOT EXPORTED (absent from PIDX_PROFILE_%s):\n', o.profile);
            for i = 1:numel(o.dropped)
                fprintf('    - %s\n', o.dropped{i});
            end
        end
    end
end

% ===========================================================================
% Header
% ===========================================================================

function [txt, o] = buildHeader(plant, cfg, o, guard)
    % An anonymous function cannot contain an assignment, so the line
    % accumulator is a NESTED function: it shares this function's L and
    % appends to it. (The first version wrote `a = @(...) L{end+1} = ...`,
    % which is a parse error MATLAB reports as "Incorrect use of '='".)
    L = {};
    function a(varargin)
        L{end + 1} = strjoin(cellfun(@toChar, varargin, ...
            'UniformOutput', false), '');
    end

    a('/**');
    a(' * @file    pidx_tuning_', o.symbol, '.h');
    a(' * @brief   Tuned parameters for the "', o.symbol, '" loop.');
    a(' *');
    a(' * GENERATED BY simlab.exportSTM32 on ', datestr(now, 'yyyy-mm-dd HH:MM:SS'), '.');
    a(' * Do not edit the numbers by hand: edit the MATLAB configuration and');
    a(' * re-export, so the simulation record below stays true.');
    a(' *');
    a(' * Library : PIDX ', pidx.Const.VERSION_STRING);
    a(' * Profile : PIDX_PROFILE_', o.profile);
    a(' * Plant   : ', plant.name, ' (', plant.kind, ')');
    a(' */');
    a('#ifndef ', guard);
    a('#define ', guard);
    a('');
    a('/* ---- loop rate ------------------------------------------------ */');
    a('#define ', upper(o.symbol), '_DT        ', num(cfg.core.sample_time), 'f   /* [s] */');
    a('#define ', upper(o.symbol), '_HZ        ', num(1 / cfg.core.sample_time), 'f');
    a('');
    a('/* ---- gains, parallel form ------------------------------------- */');
    a('/* u = Kp*e + Ki*int(e) + Kd*de/dt,  Ki = Kp/Ti,  Kd = Kp*Td       */');
    a('#define ', upper(o.symbol), '_KP        ', num(cfg.core.kp), 'f');
    a('#define ', upper(o.symbol), '_KI        ', num(cfg.core.ki), 'f');
    a('#define ', upper(o.symbol), '_KD        ', num(cfg.core.kd), 'f');
    if cfg.core.ki > 0
        a('#define ', upper(o.symbol), '_TI        ', num(cfg.core.kp / cfg.core.ki), 'f  /* [s] */');
    end
    if cfg.core.kd > 0 && cfg.core.kp > 0
        a('#define ', upper(o.symbol), '_TD        ', num(cfg.core.kd / cfg.core.kp), 'f  /* [s] */');
    end
    a('#define ', upper(o.symbol), '_TF        ', num(cfg.filter.tf), 'f  /* derivative filter [s] */');
    a('#define ', upper(o.symbol), '_NFILTER   ', num(cfg.filter.n_filter), 'f  /* Tf = Kd/(N*Kp) when TF == 0 */');
    a('');
    a('/* ---- output --------------------------------------------------- */');
    if cfg.limits.use_output_limits
        a('#define ', upper(o.symbol), '_OUT_MIN   ', num(cfg.limits.output_min), 'f');
        a('#define ', upper(o.symbol), '_OUT_MAX   ', num(cfg.limits.output_max), 'f');
    else
        a('/* no output limits: the actuator is assumed not to saturate.    */');
        a('/* If it does, set these and re-export - an integrator that can  */');
        a('/* demand more than the actuator delivers is windup waiting.     */');
    end
    a('');
    a('/* ---- structure ------------------------------------------------ */');
    a('#define ', upper(o.symbol), '_DIRECTION ', directionName(cfg.core.direction));
    a('#define ', upper(o.symbol), '_AW_MODE   ', awName(cfg.integral.mode));
    a('#define ', upper(o.symbol), '_AW_KT     ', num(cfg.integral.kt), 'f  /* 0 = derived */');
    a('#define ', upper(o.symbol), '_DERIV_MODE ', derivName(cfg.filter.derivative_mode));
    a('#define ', upper(o.symbol), '_INTEGRATION ', integName(cfg.core.integration));
    a('#define ', upper(o.symbol), '_BETA      ', num(cfg.weight.beta), 'f');
    a('#define ', upper(o.symbol), '_GAMMA     ', num(cfg.weight.gamma), 'f');
    a('');

    if ~isempty(o.result)
        m = o.result.metrics;
        a('/* ---- measured in simulation ----------------------------------- */');
        a('/* scenario: ', resultName(o.result), ' */');
        a('/*   rise 10-90%   ', num(m.riseTime), ' s');
        a('/*   overshoot     ', num(m.overshoot), ' %');
        a('/*   settling 2%   ', num(m.settlingTime), ' s');
        a('/*   IAE           ', num(m.iae));
        a('/*   steady error  ', num(m.ssError));
        a('/*   TV(u)         ', num(m.tv));
        a('/*   saturated     ', num(100 * m.satFraction), ' % of samples');
        a('/*   stable        ', mat2str(logical(m.stable)));
        a('/*');
        a(' * These numbers describe the MATLAB plant model, not your board.');
        a(' * They are the baseline to compare the board against: if the real');
        a(' * loop settles in twice this time, the model is wrong, not the');
        a(' * controller.');
        a(' */');
        a('');
    end

    if ~isempty(o.sens)
        s = o.sens;
        a('/* ---- loop margins (linear analysis) ---------------------------- */');
        a('/*   Ms              ', num(s.Ms), '   (< 1.4 comfortable, > 2 fragile)');
        a('/*   gain margin     ', num(s.gm), ' x');
        a('/*   phase margin    ', num(s.pm), ' deg');
        a('/*   delay margin    ', num(s.delayMargin), ' s');
        a('/*   bandwidth       ', num(s.bandwidth), ' rad/s');
        a('/*   assumes dt = ', num(s.dt), ' s and holds the ZOH half-sample delay');
        a(' */');
        a('');
    end

    if ~isempty(o.tune)
        t = o.tune;
        a('/* ---- identification the gains came from ------------------- */');
        if t.model.kind == pidx.Const.MODEL_FREQ
            a('/*   relay: Ku = ', num(t.model.ku), ', Pu = ', num(t.model.pu), ' s');
        else
            a('/*   step: K = ', num(t.model.k), ', T = ', num(t.model.t), ...
              ' s, L = ', num(t.model.l), ' s');
        end
        a('/*   quality ', num(double(t.model.quality)), '/100, asymmetry ', ...
          num(t.asymmetry), ', experiment ', num(t.elapsed_s), ' s');
        a('/*   Re-run the identification on the board: the model here is the');
        a('/*   simulation plant''s, and yours will differ. */');
        a('');
    end

    % Plant description, so the person reading the header in six months can
    % tell what the gains were tuned against.
    a('/* ---- plant the gains were tuned against ------------------------ */');
    switch plant.kind
        case 'fopdt'
            a('/*   G(s) = ', num(plant.modelParam('k')), ' * exp(-', ...
              num(plant.transportDelay()), '*s) / (1 + ', num(plant.tau()), '*s)');
        case 'dc_motor'
            a('/*   DC motor  R=', num(plant.modelParam('r')), ...
              ' L=', num(plant.modelParam('l_elec')), ...
              ' Ke=', num(plant.modelParam('ke')), ...
              ' Kt=', num(plant.modelParam('kt')));
            a('/*               J=', num(plant.modelParam('j')), ...
              ' B=', num(plant.modelParam('b')), ...
              ' Coulomb=', num(plant.modelParam('coulomb')));
        case 'linear'
            a('/*   G(s) = [', strjoin(arrayfun(@num, plant.numerator(), 'UniformOutput', false), ' '), '] / [', ...
              strjoin(arrayfun(@num, plant.denominator(), 'UniformOutput', false), ' '), ']');
        otherwise
            a('/*   custom model - see the MATLAB configuration');
    end
    cav = plant.analysisCaveats();
    for i = 1:numel(cav)
        a('/*   note: ', cav{i});
    end
    a(' */');
    a('');

    for i = 1:numel(o.extra)
        a('/* ', o.extra{i}, ' */');
    end

    a('/* ---- API ------------------------------------------------------ */');
    a('#ifdef __cplusplus');
    a('extern "C" {');
    a('#endif');
    a('');
    a('/** Configure the loop. Call once, after the HAL is up. */');
    a('void ', o.symbol, '_init(void);');
    a('');
    a('/** One control cycle. Call from the timer ISR at ', num(1 / cfg.core.sample_time), ' Hz.');
    a(' *  @param y  the measurement');
    a(' *  @return   the actuator command');
    a(' */');
    a('float ', o.symbol, '_tick(float y);');
    a('');
    a('/* ---- inspection, used by the HIL firmware and by your own ---- */');
    a('/*      diagnostics. Not needed to run the loop.                ---- */');
    a('PID_Handle *', o.symbol, '_handle(void);');
    a('');
    a('/** The PIDX status flags of the last cycle, exactly as the library set them. */');
    a('unsigned int ', o.symbol, '_flags(void);');
    a('');
    a('/** The output before saturation and slew limiting. */');
    a('float ', o.symbol, '_unsat(void);');
    a('');
    a('#ifdef __cplusplus');
    a('}');
    a('#endif');
    a('');
    a('#endif /* ', guard, ' */');
    a('');

    txt = [strjoin(L, sprintf('\n')), sprintf('\n')];
end

% ===========================================================================
% Source
% ===========================================================================

function [txt, o] = buildSource(cfg, o)
    L = {};
    function a(s)
        L{end + 1} = s;
    end
    K = pidx.Const;
    P = upper(o.symbol);

    dropped = {};

    a('/**');
    a(' * @file    pidx_tuning_', o.symbol, '.c');
    a(' * @brief   Generated by simlab.exportSTM32. Rebuild the PIDX sources');
    a(' *          into your project and call ', o.symbol, '_init() once.');
    a(' *');
    a(' * WIRING');
    a(' *   void TIM2_IRQHandler(void) {');
    a(' *       if (LL_TIM_IsActiveFlag_UPDATE(TIM2)) {');
    a(' *           LL_TIM_ClearFlag_UPDATE(TIM2);');
    a(' *           float y = read_sensor();');
    a(' *           float u = ', o.symbol, '_tick(y);');
    a(' *           write_actuator(u);');
    a(' *       }');
    a(' *   }');
    a(' */');
    a('#include "pidx/pid.h"');
    a('#include "pidx_tuning_', o.symbol, '.h"');
    a('');
    a('static PID_Handle ', o.symbol, ';');
    a('');
    a('void ', o.symbol, '_init(void)');
    a('{');
    a('    PID_Config cfg;');
    a('    (void)PID_ConfigDefault(&cfg);');
    a('');
    a('    cfg.core.kp = ', P, '_KP;');
    a('    cfg.core.ki = ', P, '_KI;');
    a('    cfg.core.kd = ', P, '_KD;');
    a('    cfg.core.sample_time = ', P, '_DT;');
    a('    cfg.core.direction = ', P, '_DIRECTION;');
    a('    cfg.core.integration = ', P, '_INTEGRATION;');
    a('');
    if cfg.limits.use_output_limits
        a('    cfg.limits.use_output_limits = true;');
        a('    cfg.limits.output_min = ', P, '_OUT_MIN;');
        a('    cfg.limits.output_max = ', P, '_OUT_MAX;');
        a('');
    end
    if cfg.limits.use_integral_limits
        a('    cfg.limits.use_integral_limits = true;');
        a('    cfg.limits.integral_min = ', num(cfg.limits.integral_min), 'f;');
        a('    cfg.limits.integral_max = ', num(cfg.limits.integral_max), 'f;');
        a('');
    end
    a('    cfg.integral.mode = ', P, '_AW_MODE;');
    a('    cfg.integral.kt = ', P, '_AW_KT;');
    a('    cfg.filter.derivative_mode = ', P, '_DERIV_MODE;');
    a('    cfg.filter.tf = ', P, '_TF;');
    a('    cfg.filter.n_filter = ', P, '_NFILTER;');
    a('    cfg.weight.beta = ', P, '_BETA;');
    a('    cfg.weight.gamma = ', P, '_GAMMA;');
    a('');

    % ---- features that need a module the profile may not have ----
    if cfg.integral.separation_threshold > 0
        a('    cfg.integral.separation_threshold = ', ...
          num(cfg.integral.separation_threshold), 'f;');
    end
    if cfg.integral.deadband > 0
        a('    cfg.integral.deadband = ', num(cfg.integral.deadband), 'f;');
    end
    if ~cfg.integral.enabled
        a('    cfg.integral.enabled = false;');
    end

    if cfg.feedforward.enabled
        if ~profileHas(o.profile, 'FEEDFORWARD')
            dropped{end + 1} = 'feedforward';
            a('    /* NOT EXPORTED: feedforward needs PIDX_ENABLE_FEEDFORWARD,');
            a('     * which PIDX_PROFILE_', o.profile, ' does not enable. */');
        else
            a('    cfg.feedforward.enabled = true;');
            a('    cfg.feedforward.value = ', num(cfg.feedforward.value), 'f;');
            a('    cfg.feedforward.gain = ', num(cfg.feedforward.gain), 'f;');
            a('    /* A feedforward CALLBACK cannot be exported: it is a MATLAB');
            a('     * function handle. Set cfg.feedforward.fn here in C, or use');
            a('     * the static value above. */');
        end
    end

    if cfg.shaper.sp_rate_max > 0 || cfg.shaper.out_slew_max > 0
        if ~profileHas(o.profile, 'SHAPER')
            dropped{end + 1} = 'setpoint/output shaper';
            a('    /* NOT EXPORTED: shaping needs PIDX_ENABLE_SHAPER,');
            a('     * which PIDX_PROFILE_', o.profile, ' does not enable. */');
        else
            if cfg.shaper.sp_rate_max > 0
                a('    cfg.shaper.sp_rate_max = ', num(cfg.shaper.sp_rate_max), 'f;');
                a('    cfg.shaper.sp_accel = ', num(cfg.shaper.sp_accel), 'f;');
                a('    cfg.shaper.sp_decel = ', num(cfg.shaper.sp_decel), 'f;');
            end
            if cfg.shaper.out_slew_max > 0
                a('    cfg.shaper.out_slew_max = ', num(cfg.shaper.out_slew_max), 'f;');
            end
        end
    end

    if cfg.filter.input_lpf_tau > 0
        if ~profileHas(o.profile, 'INPUT_FILTER')
            dropped{end + 1} = 'input low-pass filter';
            a('    /* NOT EXPORTED: the input filter needs');
            a('     * PIDX_ENABLE_INPUT_FILTER. */');
        else
            a('    cfg.filter.input_lpf_tau = ', num(cfg.filter.input_lpf_tau), 'f;');
        end
    end

    if cfg.safety.enabled
        if ~profileHas(o.profile, 'SAFETY')
            dropped{end + 1} = 'sensor validation / fail-safe';
            a('    /* NOT EXPORTED: safety needs PIDX_ENABLE_SAFETY,');
            a('     * which PIDX_PROFILE_', o.profile, ' does not enable.');
            a('     * The loop will run with NO sensor validation. */');
        else
            a('    cfg.safety.enabled = true;');
            a('    cfg.safety.meas_min = ', num(cfg.safety.meas_min), 'f;');
            a('    cfg.safety.meas_max = ', num(cfg.safety.meas_max), 'f;');
            a('    cfg.safety.meas_rate_max = ', num(cfg.safety.meas_rate_max), 'f;');
            a('    cfg.safety.failsafe_output = ', num(cfg.safety.failsafe_output), 'f;');
            a('    cfg.safety.fault_persist_n = ', num(cfg.safety.fault_persist_n), 'u;');
            a('    cfg.safety.auto_recover = ', bool(cfg.safety.auto_recover), ';');
        end
    end
    a('');
    a('    (void)PID_Init(&', o.symbol, ', &cfg);');
    a('}');
    a('');
    a('float ', o.symbol, '_tick(float y)');
    a('{');
    a('    return PID_Update(&', o.symbol, ', y);');
    a('}');
    a('');
    a('PID_Handle *', o.symbol, '_handle(void)');
    a('{');
    a('    return &', o.symbol, ';');
    a('}');
    a('');
    a('/* Reported from the library, not re-derived here. If the controller says');
    a(' * it saturated, that is what a caller should be told even in a case where');
    a(' * guessing from the data would say otherwise. */');
    a('unsigned int ', o.symbol, '_flags(void)');
    a('{');
    a('    return (unsigned int)PID_GetFlags(&', o.symbol, ');');
    a('}');
    a('');
    a('float ', o.symbol, '_unsat(void)');
    a('{');
    a('#if PIDX_ENABLE_DIAGNOSTICS');
    a('    PID_Status s;');
    a('    if (PID_GetStatus(&', o.symbol, ', &s) == PID_OK) {');
    a('        return s.output_unsat;');
    a('    }');
    a('#endif');
    a('    return PID_GetOutput(&', o.symbol, ');');
    a('}');
    a('');

    txt = [strjoin(L, sprintf('\n')), sprintf('\n')];
    o.dropped = dropped;
end

% ===========================================================================
% Helpers
% ===========================================================================

function s = toChar(v)
% The header builder concatenates numbers and strings freely; this is the one
% place that turns a number into text so the rest can stay readable.
    if ischar(v)
        s = v;
    else
        s = num2str(v);
    end
end

function s = resultName(r)
    if isfield(r, 'scenario') && ischar(r.scenario)
        % r.scenario is the whole transcript; the first line names it.
        nl = find(r.scenario == newline, 1, 'first');
        if isempty(nl), nl = numel(r.scenario); end
        s = strtrim(r.scenario(1:nl - 1));
    else
        s = 'unknown';
    end
end

function ok = profileHas(profile, feature)
% Which subsystems each compile-time profile enables, from pid_conf.h.
%
% Read from the header would be better; hard-coded here because parsing a C
% preprocessor in MATLAB is more machinery than this deserves, and the table
% is checked against pid_conf.h by simlab_tests/test_export.m.
    switch upper(feature)
        case 'FEEDFORWARD'
            ok = ~strcmpi(profile, 'MINIMAL');
        case 'SHAPER'
            ok = ~strcmpi(profile, 'MINIMAL');
        case 'INPUT_FILTER'
            ok = ~strcmpi(profile, 'MINIMAL');
        case 'SAFETY'
            ok = ~strcmpi(profile, 'MINIMAL');
        case 'GAIN_SCHED'
            ok = ~strcmpi(profile, 'MINIMAL');
        otherwise
            ok = true;
    end
end

function s = num(v)
% Format a number for C. %.10g round-trips a float32 exactly, which is what
% the target uses, and avoids printing 0.1 as 0.10000000000000001.
    if isempty(v) || ~isfinite(v)
        s = '0.0';
        return;
    end
    s = sprintf('%.10g', v);
end

function s = bool(v)
    if v, s = 'true'; else, s = 'false'; end
end

function s = directionName(d)
    if d == pidx.Const.REVERSE
        s = 'PID_REVERSE';
    else
        s = 'PID_DIRECT';
    end
end

function s = awName(m)
    K = pidx.Const;
    switch m
        case K.AW_NONE,             s = 'PID_AW_NONE';
        case K.AW_CLAMP,            s = 'PID_AW_CLAMP';
        case K.AW_CONDITIONAL,      s = 'PID_AW_CONDITIONAL';
        case K.AW_BACK_CALCULATION, s = 'PID_AW_BACK_CALCULATION';
        case K.AW_TRACKING,         s = 'PID_AW_TRACKING';
        otherwise,                  s = 'PID_AW_CLAMP';
    end
end

function s = derivName(m)
    K = pidx.Const;
    switch m
        case K.DERIV_ON_MEASUREMENT,    s = 'PID_DERIV_ON_MEASUREMENT';
        case K.DERIV_ON_ERROR,          s = 'PID_DERIV_ON_ERROR';
        case K.DERIV_ON_WEIGHTED_ERROR, s = 'PID_DERIV_ON_WEIGHTED_ERROR';
        otherwise,                      s = 'PID_DERIV_ON_MEASUREMENT';
    end
end

function s = integName(m)
    if m == pidx.Const.INTEGRATION_TRAPEZOIDAL
        s = 'PID_INTEGRATION_TRAPEZOIDAL';
    else
        s = 'PID_INTEGRATION_BACKWARD_EULER';
    end
end

function o = fillOpt(o, name, default)
    if ~isfield(o, name) || isempty(o.(name))
        o.(name) = default;
    end
end

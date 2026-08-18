function conform_m(scenario_file)
%CONFORM_M  Conformance runner for the MATLAB/Octave port.
%
%   Reads the scenario file described in ports/SPEC_conformance.md and writes
%   the CSV compared against the C reference. Mirrors the dispatch in
%   ports/c_ref/conform_c.c exactly.
%
%   Usage:  octave-cli --quiet --eval "conform_m('../compare/scenarios.txt')"

    K = pidx.Const;

    fid = fopen(scenario_file, 'r');
    if fid < 0
        error('conform_m:open', 'cannot open %s', scenario_file);
    end

    printf('scenario,k,cmd,rc,output,setpoint,error,p,i,d,ff,unsat,flags,last_error\n');

    st = struct();
    st.cfg = pidx.config();
    st.pid = pidx.PID();
    st.sched_points = [];
    st.scenario = 'none';
    st.row = 0;
    st.inited = false;

    while true
        line = fgetl(fid);
        if ~ischar(line)
            break;
        end
        t = strsplit(strtrim(line));
        t = t(~cellfun(@isempty, t));
        if isempty(t) || t{1}(1) == '#'
            continue;
        end

        switch t{1}
            case 'scenario'
                st.cfg = pidx.config();
                st.pid = pidx.PID();
                st.sched_points = [];
                st.inited = false;
                st.row = 0;
                if numel(t) > 1
                    st.scenario = t{2};
                else
                    st.scenario = '?';
                end
            case 'init'
                rc = st.pid.init(st.cfg);
                st.inited = (rc == K.OK);
                st = emit(st, 'init', rc, 0, NaN, NaN, NaN, NaN, NaN, NaN, ...
                          NaN, 0, 0);
            case 'end'
                st.inited = false;
            otherwise
                if ~st.inited
                    st = doConfig(st, t);
                else
                    st = doRun(st, t);
                end
        end
    end

    fclose(fid);
end

% ===========================================================================

function s = fmtNum(v)
    % Render with full round-trip precision, spelling non-finite values the
    % same way every other port does.
    if isnan(v)
        s = 'nan';
    elseif isinf(v)
        if v > 0
            s = 'inf';
        else
            s = '-inf';
        end
    else
        s = sprintf('%.17g', v);
    end
end

function st = emit(st, cmd, rc, out, sp, err, p, i, d, ff, unsat, flags, last)
    printf('%s,%d,%s,%d,%s,%s,%s,%s,%s,%s,%s,%s,%d,%d\n', ...
        st.scenario, st.row, cmd, rc, ...
        fmtNum(out), fmtNum(sp), fmtNum(err), fmtNum(p), fmtNum(i), ...
        fmtNum(d), fmtNum(ff), fmtNum(unsat), flags, last);
    st.row = st.row + 1;
end

function st = emitUpdate(st, cmd, rc, out)
    s = st.pid.getStatus();
    if isempty(s)
        st = emit(st, cmd, rc, out, NaN, NaN, NaN, NaN, NaN, NaN, NaN, ...
                  0, st.pid.peekLastError());
    else
        st = emit(st, cmd, rc, out, s.setpoint_shaped, s.error, s.p_term, ...
                  s.i_term, s.d_term, s.ff_term, s.output_unsat, ...
                  s.flags, st.pid.peekLastError());
    end
end

function v = num(tok)
    % Parse a scenario number, including the non-finite spellings.
    if strcmp(tok, 'nan')
        v = NaN;
    elseif strcmp(tok, 'inf')
        v = Inf;
    elseif strcmp(tok, '-inf')
        v = -Inf;
    else
        v = str2double(tok);
    end
end

function v = f(t, i)
    if i <= numel(t)
        v = num(t{i});
    else
        v = 0.0;
    end
end

function v = n(t, i)
    if i <= numel(t)
        v = round(str2double(t{i}));
    else
        v = 0;
    end
end

% ===========================================================================

function st = doConfig(st, t)
    c = t{1};
    switch c
        case 'gains'
            st.cfg.core.kp = f(t, 2);
            st.cfg.core.ki = f(t, 3);
            st.cfg.core.kd = f(t, 4);
        case 'dt'
            st.cfg.core.sample_time = f(t, 2);
        case 'direction'
            st.cfg.core.direction = n(t, 2);
        case 'mode'
            st.cfg.core.mode = n(t, 2);
        case 'integration'
            st.cfg.core.integration = n(t, 2);
        case 'outlim'
            st.cfg.limits.use_output_limits = true;
            st.cfg.limits.output_min = f(t, 2);
            st.cfg.limits.output_max = f(t, 3);
        case 'intlim'
            st.cfg.limits.use_integral_limits = true;
            st.cfg.limits.integral_min = f(t, 2);
            st.cfg.limits.integral_max = f(t, 3);
        case 'dtlim'
            st.cfg.limits.dt_min = f(t, 2);
            st.cfg.limits.dt_max = f(t, 3);
        case 'aw'
            st.cfg.integral.mode = n(t, 2);
            st.cfg.integral.kt = f(t, 3);
        case 'separation'
            st.cfg.integral.separation_threshold = f(t, 2);
        case 'deadband'
            st.cfg.integral.deadband = f(t, 2);
        case 'ienable'
            st.cfg.integral.enabled = (n(t, 2) ~= 0);
        case 'dmode'
            st.cfg.filter.derivative_mode = n(t, 2);
        case 'tf'
            st.cfg.filter.tf = f(t, 2);
        case 'nfilter'
            st.cfg.filter.n_filter = f(t, 2);
        case 'inlpf'
            st.cfg.filter.input_lpf_tau = f(t, 2);
        case 'weights'
            st.cfg.weight.beta = f(t, 2);
            st.cfg.weight.gamma = f(t, 3);
        case 'ff'
            st.cfg.feedforward.enabled = (n(t, 2) ~= 0);
            st.cfg.feedforward.value = f(t, 3);
            st.cfg.feedforward.gain = f(t, 4);
        case 'shaper'
            st.cfg.shaper.sp_rate_max = f(t, 2);
            st.cfg.shaper.sp_accel = f(t, 3);
            st.cfg.shaper.sp_decel = f(t, 4);
            st.cfg.shaper.out_slew_max = f(t, 5);
        case 'safety'
            st.cfg.safety.enabled = (n(t, 2) ~= 0);
            st.cfg.safety.meas_min = f(t, 3);
            st.cfg.safety.meas_max = f(t, 4);
            st.cfg.safety.meas_rate_max = f(t, 5);
            st.cfg.safety.failsafe_output = f(t, 6);
            st.cfg.safety.fault_persist_n = n(t, 7);
            st.cfg.safety.auto_recover = (n(t, 8) ~= 0);
        otherwise
            error('conform_m:cmd', 'unknown config cmd: %s', c);
    end
end

% ===========================================================================

function st = doRun(st, t)
    K = pidx.Const;
    c = t{1};
    p = st.pid;

    switch c
        case 'u'
            out = p.updateDt(f(t, 2), f(t, 3));
            st = emitUpdate(st, 'u', p.peekLastError(), out);
        case 'un'
            out = p.update(f(t, 2));
            st = emitUpdate(st, 'un', p.peekLastError(), out);
        case 'ufast'
            out = p.updateFast(f(t, 2));
            st = emit(st, 'ufast', p.peekLastError(), out, ...
                      p.getSetpoint(), NaN, NaN, p.getIntegrator(), ...
                      NaN, NaN, NaN, 0, p.peekLastError());
        case 'uex'
            inp = struct('measurement', f(t, 2), 'dt', f(t, 3), ...
                         'setpoint', f(t, 4), 'feedforward', f(t, 5), ...
                         'tracking', f(t, 6), 'schedule_var', f(t, 7));
            [out, rc] = p.updateEx(inp);
            st = emitUpdate(st, 'uex', rc, out);
        case 'sp',         p.setSetpoint(f(t, 2));
        case 'spimm',      p.setSetpointImmediate(f(t, 2));
        case 'setmode',    p.setMode(n(t, 2));
        case 'manual',     p.setManualOutput(f(t, 2));
        case 'setgains',   p.setGains(f(t, 2), f(t, 3), f(t, 4));
        case 'rescale'
            p.setGainsRescaleIntegral(f(t, 2), f(t, 3), f(t, 4));
        case 'setaw',      p.setAntiWindup(n(t, 2), f(t, 3));
        case 'setoutlim',  p.setOutputLimits(f(t, 2), f(t, 3));
        case 'clroutlim',  p.clearOutputLimits();
        case 'setintlim',  p.setIntegralLimits(f(t, 2), f(t, 3));
        case 'setint',     p.setIntegrator(f(t, 2));
        case 'track',      p.setTrackingInput(f(t, 2));
        case 'setdmode',   p.setDerivativeMode(n(t, 2));
        case 'settf',      p.setDerivativeFilter(f(t, 2));
        case 'setn',       p.setDerivativeFilterN(f(t, 2));
        case 'setdir',     p.setDirection(n(t, 2));
        case 'setweights', p.setWeights(f(t, 2), f(t, 3));
        case 'setff',      p.setFeedforward(f(t, 2));
        case 'setramp',    p.setSetpointRamp(f(t, 2), f(t, 3), f(t, 4));
        case 'setslew',    p.setOutputSlewRate(f(t, 2));
        case 'setinlpf',   p.setInputFilter(f(t, 2));
        case 'setsep',     p.setIntegralSeparation(f(t, 2));
        case 'setdb',      p.setIntegralDeadband(f(t, 2));
        case 'setienable', p.enableIntegral(n(t, 2) ~= 0);
        case 'setdtnom',   p.setSampleTime(f(t, 2));
        case 'reset',      p.reset();
        case 'clearfault', p.clearFault();
        case 'schedpoints'
            cnt = n(t, 2);
            pts = zeros(cnt, 4);
            for i = 1:cnt
                base = 3 + (i - 1) * 4;
                pts(i, 1) = f(t, base);
                pts(i, 2) = f(t, base + 1);
                pts(i, 3) = f(t, base + 2);
                pts(i, 4) = f(t, base + 3);
            end
            st.sched_points = pts;
        case 'schedcfg'
            if ~isempty(st.sched_points)
                sch = pidx.GainSchedule();
                sch.init(st.sched_points, n(t, 2), n(t, 3));
                sch.setHysteresis(f(t, 4));
                pidx.GainSchedule.attach(p, sch);
            end
        case 'schedvar'
            pidx.GainSchedule.setVar(p, f(t, 2));
        case 'rule'
            kind = n(t, 4);
            if kind == K.MODEL_FREQ
                mdl = pidx.plantModel(kind, f(t, 5), f(t, 6));
            else
                mdl = pidx.plantModel(kind, f(t, 5), f(t, 6), f(t, 7));
            end
            [rc, g] = pidx.ruleApply(n(t, 2), mdl, n(t, 3), f(t, 8));
            st = emit(st, 'rule', rc, g.kp, g.ki, g.kd, g.ti, g.td, ...
                      g.tf, NaN, NaN, 0, 0);
        otherwise
            error('conform_m:cmd', 'unknown run cmd: %s', c);
    end
end

function out = exportReport(r, opt)
%SIMLAB.EXPORTREPORT  Write a simulation run to CSV + JSON for the record.
%
%   OUT = SIMLAB.EXPORTREPORT(R)                 % R from simlab.Sim.run()
%   OUT = SIMLAB.EXPORTREPORT(R, 'dir', 'runs', 'name', 'heater_v3')
%
% The C file from exportSTM32 is what you flash. This is what you attach to
% the change request: the numbers those gains were measured against, in a
% form that survives the loss of the MATLAB session that produced them.
%
% FILES
%   <dir>/<name>.csv    one row per sample: t, r, y, yTrue, u, uPlant, uRaw,
%                       p, i, d, ff, e, flags, kp, ki, kd
%   <dir>/<name>.json   scenario transcript, controller configuration, plant
%                       description, metrics, margins, tool versions
%
% The JSON is written by hand rather than through jsonencode() so this runs
% on any MATLAB release and under Octave. It is deliberately boring: no
% nested objects that a parser might trip over, every number in full
% precision, every string escaped.
%
% OPTIONS
%   'dir'    default './simlab_export'
%   'name'   default derived from the timestamp
%   'sens'   a simlab.sensitivity result to include
%   'cfg'    the pidx config the controller was built from
%   'plant'  the simlab.Plant, for its description and caveats
%   'tune'   a simlab.AutoTune result to include

    if nargin < 2, opt = struct(); end
    o = fillOpt(opt, 'dir', fullfile(pwd, 'simlab_export'));
    o = fillOpt(o, 'name', sprintf('run_%s', datestr(now, 'yyyymmdd_HHMMSS')));
    o = fillOpt(o, 'sens', []);
    o = fillOpt(o, 'cfg', []);
    o = fillOpt(o, 'plant', []);
    o = fillOpt(o, 'tune', []);

    if ~exist(o.dir, 'dir')
        mkdir(o.dir);
    end
    csvPath = fullfile(o.dir, [o.name, '.csv']);
    jsonPath = fullfile(o.dir, [o.name, '.json']);

    writeCsv(csvPath, r);
    writeJson(jsonPath, r, o);

    out = struct('csv', csvPath, 'json', jsonPath, 'name', o.name);
    fprintf('simlab.exportReport: %s\n                   %s\n', csvPath, jsonPath);
end

% ---------------------------------------------------------------------------

function writeCsv(path, r)
    cols = {'t', 'r', 'y', 'yTrue', 'u', 'uPlant', 'uRaw', 'p', 'i', 'd', ...
            'ff', 'e', 'flags', 'kp', 'ki', 'kd'};
    n = numel(r.t);
    M = nan(n, numel(cols));
    for j = 1:numel(cols)
        if isfield(r, cols{j}) && numel(r.(cols{j})) == n
            M(:, j) = double(r.(cols{j}))(:);
        end
    end
    fid = fopen(path, 'w');
    fprintf(fid, '%s\n', strjoin(cols, ','));
    % %.9g keeps the file small and still round-trips a float32, which is the
    % precision the target actually has.
    for k = 1:n
        fprintf(fid, '%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u,%.9g,%.9g,%.9g\n', ...
            M(k, 1), M(k, 2), M(k, 3), M(k, 4), M(k, 5), M(k, 6), M(k, 7), ...
            M(k, 8), M(k, 9), M(k, 10), M(k, 11), M(k, 12), ...
            uint32(M(k, 13)), M(k, 14), M(k, 15), M(k, 16));
    end
    fclose(fid);
end

function writeJson(path, r, o)
    % Same rule as exportSTM32: an anonymous function cannot contain an
    % assignment, so the line accumulator is a nested function.
    L = {};
    function a(s)
        L{end + 1} = s;
    end

    a('{');
    a('  "tool": "simlab.exportReport",');
    a(sprintf('  "generated": "%s",', datestr(now, 'yyyy-mm-dd HH:MM:SS')));
    a(sprintf('  "pidxVersion": "%s",', pidx.Const.VERSION_STRING));
    a(sprintf('  "matlab": "%s",', escapeChar(version)));
    a(sprintf('  "dt": %s,', jnum(r.dt)));
    a(sprintf('  "samples": %d,', numel(r.t)));

    a('  "scenario": {');
    a(sprintf('    "transcript": "%s"', escapeChar(transcript(r))));
    a('  },');

    a('  "metrics": {');
    m = r.metrics;
    fn = fieldnames(m);
    for i = 1:numel(fn)
        v = m.(fn{i});
        if islogical(v)
            a(sprintf('    "%s": %s%s', fn{i}, mat2str(v), comma(i, fn)));
        elseif ischar(v)
            a(sprintf('    "%s": "%s"%s', fn{i}, escapeChar(v), comma(i, fn)));
        else
            a(sprintf('    "%s": %s%s', fn{i}, jnum(v), comma(i, fn)));
        end
    end
    a('  },');

    if ~isempty(o.cfg)
        a('  "controller": {');
        c = o.cfg;
        a(sprintf('    "kp": %s, "ki": %s, "kd": %s,', ...
            jnum(c.core.kp), jnum(c.core.ki), jnum(c.core.kd)));
        a(sprintf('    "dt": %s, "direction": %d, "integration": %d,', ...
            jnum(c.core.sample_time), c.core.direction, c.core.integration));
        a(sprintf('    "outputLimits": [%s, %s], "useOutputLimits": %s,', ...
            jnum(c.limits.output_min), jnum(c.limits.output_max), ...
            jbool(c.limits.use_output_limits)));
        a(sprintf('    "antiWindup": %d, "kt": %s,', c.integral.mode, jnum(c.integral.kt)));
        a(sprintf('    "derivativeMode": %d, "tf": %s, "nFilter": %s,', ...
            c.filter.derivative_mode, jnum(c.filter.tf), jnum(c.filter.n_filter)));
        a(sprintf('    "inputLpfTau": %s,', jnum(c.filter.input_lpf_tau)));
        a(sprintf('    "beta": %s, "gamma": %s,', jnum(c.weight.beta), jnum(c.weight.gamma)));
        a(sprintf('    "integralSeparation": %s, "integralDeadband": %s,', ...
            jnum(c.integral.separation_threshold), jnum(c.integral.deadband)));
        a(sprintf('    "spRateMax": %s, "spAccel": %s, "outSlewMax": %s,', ...
            jnum(c.shaper.sp_rate_max), jnum(c.shaper.sp_accel), ...
            jnum(c.shaper.out_slew_max)));
        a(sprintf('    "safety": %s, "measMin": %s, "measMax": %s, "failsafe": %s', ...
            jbool(c.safety.enabled), jnum(c.safety.meas_min), ...
            jnum(c.safety.meas_max), jnum(c.safety.failsafe_output)));
        a('  },');
    end

    if ~isempty(o.plant)
        a('  "plant": {');
        a(sprintf('    "name": "%s", "kind": "%s",', ...
            escapeChar(o.plant.name), o.plant.kind));
        [z, p, k, l] = o.plant.polesZeros();
        a(sprintf('    "gain": %s, "tau": %s, "deadTime": %s,', ...
            jnum(k), jnum(o.plant.tau()), jnum(l)));
        a(sprintf('    "poles": [%s], "zeros": [%s],', ...
            strjoin(arrayfun(@jnum, p(:).', 'UniformOutput', false), ', '), ...
            strjoin(arrayfun(@jnum, z(:).', 'UniformOutput', false), ', ')));
        [lo, hi] = o.plant.actuatorLimits();
        a(sprintf('    "actuator": [%s, %s],', jnum(lo), jnum(hi)));
        a(sprintf('    "noiseSigma": %s, "adcBits": %d,', ...
            jnum(o.plant.sensorParam('sigma')), o.plant.sensorParam('bits')));
        cav = o.plant.analysisCaveats();
        a('    "caveats": [');
        for i = 1:numel(cav)
            a(sprintf('      "%s"%s', escapeChar(cav{i}), comma(i, cav)));
        end
        a('    ]');
        a('  },');
    end

    if ~isempty(o.sens)
        s = o.sens;
        a('  "margins": {');
        a(sprintf('    "Ms": %s, "MsFreq": %s, "Mt": %s,', ...
            jnum(s.Ms), jnum(s.MsFreq), jnum(s.Mt)));
        a(sprintf('    "gainMargin": %s, "phaseMarginDeg": %s,', ...
            jnum(s.gm), jnum(s.pm)));
        a(sprintf('    "delayMargin": %s, "bandwidth": %s,', ...
            jnum(s.delayMargin), jnum(s.bandwidth)));
        a(sprintf('    "dt": %s, "deadTime": %s, "holdDelay": %s,', ...
            jnum(s.dt), jnum(s.deadTime), jnum(s.holdDelay)));
        a(sprintf('    "verdict": "%s"', escapeChar(s.verdict)));
        a('  },');
    end

    if ~isempty(o.tune)
        t = o.tune;
        a('  "autoTune": {');
        a(sprintf('    "modelKind": %d, "K": %s, "T": %s, "L": %s,', ...
            t.model.kind, jnum(t.model.k), jnum(t.model.t), jnum(t.model.l)));
        a(sprintf('    "Ku": %s, "Pu": %s, "quality": %d,', ...
            jnum(t.model.ku), jnum(t.model.pu), double(t.model.quality)));
        a(sprintf('    "kp": %s, "ki": %s, "kd": %s, "tf": %s,', ...
            jnum(t.gains.kp), jnum(t.gains.ki), jnum(t.gains.kd), ...
            jnum(t.gains.tf)));
        a(sprintf('    "asymmetry": %s, "elapsed": %s', ...
            jnum(t.asymmetry), jnum(t.elapsed_s)));
        a('  }');
    end

    a('}');
    fid = fopen(path, 'w');
    fprintf(fid, '%s\n', strjoin(L, sprintf('\n')));
    fclose(fid);
end

% ---------------------------------------------------------------------------

function s = comma(i, list)
% Trailing comma handling: JSON has no "and the last one has none".
    if i < numel(list)
        s = ',';
    else
        s = '';
    end
end

function s = jnum(v)
% A JSON number. NaN and Inf are not JSON, so they become null - which is the
% honest encoding, and the reason a reader can tell "not measured" from 0.
    if isempty(v)
        s = 'null';
    elseif isinf(v)
        s = 'null';
    elseif isnan(v)
        s = 'null';
    else
        s = sprintf('%.17g', v);
    end
end

function s = jbool(v)
    if v, s = 'true'; else, s = 'false'; end
end

function s = escapeChar(s)
% Minimal JSON string escaping. Backslash and quote are the two that break a
% parser; control characters are stripped because a transcript can contain a
% newline and JSON strings may not.
    s = strrep(s, '\', '\\');
    s = strrep(s, '"', '\"');
    s(s < 32) = ' ';
    s(s == 127) = ' ';
end

function s = transcript(r)
    if isfield(r, 'scenario') && ischar(r.scenario)
        s = strrep(r.scenario, sprintf('\n'), ' | ');
    else
        s = '';
    end
end

function o = fillOpt(o, name, default)
    if ~isfield(o, name) || isempty(o.(name))
        o.(name) = default;
    end
end

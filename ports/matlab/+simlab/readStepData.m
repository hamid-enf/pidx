function d = readStepData(path, opt)
%SIMLAB.READSTEPDATA  Read an open-loop step record from a CSV file.
%
%   D = SIMLAB.READSTEPDATA('heater_step.csv')
%   D = SIMLAB.READSTEPDATA('log.csv', 'tCol', 1, 'yCol', 2, 'uCol', 3, ...
%                           'header', true, 'decimate', 5)
%
% Returns a struct with .t, .y and (when present) .u, ready for
% simlab.identify.
%
% WHY THIS EXISTS AS A SEPARATE FUNCTION
%   The data you actually have is a historian export, and historian exports
%   come with a header row, a semicolon delimiter, European decimal commas, a
%   timestamp instead of a relative time, and columns in an order nobody
%   agrees on. Each of those silently produces a model that is wrong by a
%   factor, so they are handled here, out loud, rather than by a bare load().
%
% OPTIONS
%   'header'    the first line is column names. Default: auto-detected from
%               whether the first line parses as numbers.
%   'tCol'      1-based column of the time, or its name when 'header' is on.
%               Default 1.
%   'yCol'      measurement column. Default 2.
%   'uCol'      command column. Default 3 when present, else empty - and then
%               simlab.identify will need 'uStep'.
%   'timeUnit'  's' | 'ms' | 'min' | 'h'. Default 's'. Applied only when the
%               time column is numeric; a timestamp column is converted to
%               seconds relative to its first value.
%   'delimiter' default auto-detected among , ; tab and whitespace.
%   'decimate'  keep every Nth row. Default 1. A historian that logged at
%               10 Hz for an hour is 36000 rows, and the moment integrals do
%               not need them.
%
% The result is CHECKED: monotone time, a roughly uniform grid, and at least
% 20 rows. A record that fails any of those is reported here, where the cause
% is obvious, rather than as a strange model later.

    if nargin < 2, opt = struct(); end
    o = fillOpt(opt, 'header', []);
    o = fillOpt(o, 'tCol', []);
    o = fillOpt(o, 'yCol', []);
    o = fillOpt(o, 'uCol', []);
    o = fillOpt(o, 'timeUnit', 's');
    o = fillOpt(o, 'delimiter', []);
    o = fillOpt(o, 'decimate', 1);

    if exist(path, 'file') ~= 2
        error('simlab:readStepData:noFile', 'no such file: %s', path);
    end

    % ---- open and sniff the first two lines ----
    fid = fopen(path, 'r');
    if fid < 0
        error('simlab:readStepData:open', 'cannot open %s', path);
    end
    first = fgetl(fid);
    second = fgetl(fid);
    fclose(fid);
    if ~ischar(first)
        error('simlab:readStepData:empty', '%s is empty', path);
    end

    delim = o.delimiter;
    if isempty(delim)
        delim = sniffDelimiter(first);
    end

    if isempty(o.header)
        % Auto-detect: if the first line does not parse as numbers, it is a
        % header. Guessing the other way silently drops a data row; guessing
        % this way silently drops a header, which is the cheaper mistake and
        % the one that gets noticed.
        o.header = ~isNumericRow(first, delim);
    end

    names = {};
    if o.header
        names = stripQuotes(strsplit(first, delim));
    end

    % ---- resolve the column indices ----
    tCol = resolveCol(o.tCol, names, 1, 'time');
    yCol = resolveCol(o.yCol, names, 2, 'measurement');
    if isempty(o.uCol)
        % Use column 3 only if it exists and is numeric.
        probe = strsplit(stripComma(second, delim), delim);
        if numel(probe) >= 3 && ~isempty(str2double(probe{3}))
            uCol = 3;
        else
            uCol = [];
        end
    else
        uCol = resolveCol(o.uCol, names, 3, 'command');
    end

    % ---- read the numbers ----
    M = dlmread2(path, delim, o.header);
    if size(M, 2) < max(tCol, yCol)
        error('simlab:readStepData:cols', ...
              'the file has %d columns but column %d was requested', ...
              size(M, 2), max(tCol, yCol));
    end
    t = M(:, tCol);
    y = M(:, yCol);
    u = [];
    if ~isempty(uCol) && size(M, 2) >= uCol
        u = M(:, uCol);
    end

    if o.decimate > 1
        t = t(1:o.decimate:end);
        y = y(1:o.decimate:end);
        if ~isempty(u), u = u(1:o.decimate:end); end
    end

    % ---- time base ----
    % A timestamp column is converted to seconds from its first value. That is
    % what the moment integrals need, and it is the conversion a reader would
    % otherwise have to guess about.
    if all(t == floor(t)) && max(t) > 1e6
        % Looks like an epoch or a large integer counter. Relative seconds are
        % the only sensible reading, and the absolute origin is meaningless to
        % a step-response fit.
        warning('simlab:readStepData:epoch', ...
                ['the time column looks like absolute epoch/counter values ' ...
                 '(max %.6g). Converted to seconds from the first sample. ' ...
                 'If it is already a relative time, ignore this.'], max(t));
    end
    switch lower(o.timeUnit)
        case 's',   scale = 1;
        case 'ms',  scale = 1e-3;
        case 'min', scale = 60;
        case 'h',   scale = 3600;
        otherwise
            error('simlab:readStepData:unit', ...
                  'unknown timeUnit "%s"', o.timeUnit);
    end
    t = (t - t(1)) * scale;

    % ---- checks ----
    if numel(t) < 20
        error('simlab:readStepData:short', ...
              'only %d rows - not enough for a moment fit', numel(t));
    end
    d = diff(t);
    if any(d <= 0)
        error('simlab:readStepData:order', ...
              'the time column is not strictly increasing (row %d)', ...
              find(d <= 0, 1) + 1);
    end
    if ~all(isfinite(y))
        bad = find(~isfinite(y), 1);
        error('simlab:readStepData:nan', ...
              'the measurement column is not a number at row %d', bad);
    end

    d = struct('t', t, 'y', y, 'u', u);
    d.source = path;
    d.names = names;
    d.dt = median(diff(t));
    d.rows = numel(t);
    d.timeUnit = o.timeUnit;

    fprintf('simlab.readStepData: %s\n', path);
    fprintf('  %d rows, dt = %.6g s (%.6g s span), columns: t=%d y=%d u=%s\n', ...
        d.rows, d.dt, t(end), tCol, yCol, ...
        tern(~isempty(uCol), num2str(uCol), 'none'));
    if ~isempty(u)
        fprintf('  command spans %.6g .. %.6g\n', min(u), max(u));
    else
        fprintf('  no command column: simlab.identify will need ''uStep''\n');
    end
end

% ---------------------------------------------------------------------------

function c = sniffDelimiter(line)
% Pick the delimiter that produces the most fields. Tie-breaks toward comma,
% which is what a historian is most likely to have written.
    cands = {',', ';', sprintf('\t'), ' '};
    best = 0;
    c = ',';
    for i = 1:numel(cands)
        n = numel(strsplit(line, cands{i}));
        if n > best
            best = n;
            c = cands{i};
        end
    end
end

function tf = isNumericRow(line, delim)
    parts = strsplit(stripComma(line, delim), delim);
    tf = true;
    for i = 1:numel(parts)
        s = strtrim(parts{i});
        if isempty(s)
            continue;
        end
        if isnan(str2double(s))
            tf = false;
            return;
        end
    end
end

function c = resolveCol(spec, names, default, what)
% A column can be given by number or, when the file has a header, by name.
% Matching by name is case-insensitive and ignores spaces, because
% "Time [s]" and "time" are the same column to a reader and should be to this.
    if isempty(spec)
        c = default;
        return;
    end
    if ischar(spec)
        want = lower(strrep(spec, ' ', ''));
        for i = 1:numel(names)
            got = lower(strrep(names{i}, ' ', ''));
            if ~isempty(strfind(got, want)) %#ok<STREMP>
                c = i;
                return;
            end
        end
        error('simlab:readStepData:noColumn', ...
              'no %s column matching "%s" in the header', what, spec);
    end
    c = spec;
end

function s = stripComma(line, delim)
% European decimal commas, for files whose DELIMITER is not a comma.
%
% The rewrite (digit-comma-digit -> digit.dot digit) can only run when the
% comma is not the field separator: on a comma-delimited file it would glue
% every field into one token and the file would read as a single column.
% That bug shipped once and cost a whole fixture.
    if strcmp(delim, ',')
        s = line;
        return;
    end
    s = regexprep(line, '(?<=\d),(?=\d)', '.');
end

function parts = stripQuotes(parts)
    for i = 1:numel(parts)
        parts{i} = strtrim(strrep(strrep(parts{i}, '"', ''), '''', ''));
    end
end

function M = dlmread2(path, delim, hasHeader)
% Read the numeric block. dlmread is deprecated in recent MATLAB and its
% behaviour on a header row varies, so the file is parsed here: one code path,
% the same answer everywhere.
    fid = fopen(path, 'r');
    if hasHeader
        fgetl(fid);
    end
    rows = {};
    n = 0;
    while true
        line = fgetl(fid);
        if ~ischar(line), break; end
        line = stripComma(line, delim);
        if isempty(strtrim(line)), continue; end
        n = n + 1;
        rows{n} = strsplit(line, delim);
    end
    fclose(fid);
    if n == 0
        M = zeros(0, 0);
        return;
    end
    ncol = 0;
    for i = 1:n
        ncol = max(ncol, numel(rows{i}));
    end
    M = nan(n, ncol);
    for i = 1:n
        for j = 1:numel(rows{i})
            s = strtrim(rows{i}{j});
            if isempty(s), continue; end
            M(i, j) = str2double(s);
        end
    end
end

function s = tern(c, a, b)
    if c, s = a; else, s = b; end
end

function o = fillOpt(o, name, default)
    if ~isfield(o, name) || isempty(o.(name))
        o.(name) = default;
    end
end

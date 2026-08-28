function results = test_suite(varargin)
%SIMLAB_TESTS.TEST_SUITE  Run every simlab test and report.
%
%   test_suite                 run everything
%   test_suite('plant')        run only tests whose name contains 'plant'
%   test_suite('verbose', true)
%
% Returns a struct with .passed, .failed and .details, and throws if anything
% failed, so it can be used as a gate:
%
%   cd ports/matlab/simlab_tests && octave-cli --quiet --eval "test_suite"
%
% WHAT THE NUMERIC TESTS COMPARE AGAINST
%   simlab_tests/c_reference.csv, produced by tools/matlab_ref/matlab_ref.c
%   running the SAME scenarios through the C library in double precision.
%   The C library is the oracle for this repository - it is what the STM32
%   runs - so a disagreement means the MATLAB port is wrong, not that the
%   tolerance is too tight. Tolerances below are 1e-9 relative, which is
%   far above the double-precision noise these computations actually produce
%   and far below any logic difference.
%
%   Regenerate the reference with:  cd ../../../tools/matlab_ref && make run
%
% HONEST LIMIT
%   These tests were written against a C oracle that was built and run, but
%   the MATLAB itself was not executed by the author: no MATLAB or Octave
%   interpreter was available where the tool was written. Run this suite
%   before trusting any number the tool gives you. That is what it is for.

    if nargin < 1, varargin = {}; end
    filter = '';
    verbose = false;
    for i = 1:2:numel(varargin)
        switch lower(varargin{i})
            case 'filter',  filter = varargin{i + 1};
            case 'verbose', verbose = logical(varargin{i + 1});
        end
    end
    if numel(varargin) == 1 && ischar(varargin{1})
        filter = varargin{1};
    end

    % Absolute paths, so the suite can be started from any working directory
    % rather than only from inside simlab_tests/.
    here = fileparts(mfilename('fullpath'));
    addpath(fullfile(here, '..'));      % the +simlab and +simlab_tests packages
    addpath(here);                      % this file and the other test files

    T = struct();
    T.stack = {};
    T.passed = 0;
    T.failed = 0;
    T.details = {};
    T.verbose = verbose;
    T.ref = loadReference(fullfile(here, 'c_reference.csv'));

    tests = { ...
        'test_plant', ...
        'test_sim', ...
        'test_metrics', ...
        'test_scenario', ...
        'test_autotune', ...
        'test_cascade', ...
        'test_fixed', ...
        'test_compareFixed', ...
        'test_identify', ...
        'test_design', ...
        'test_monteCarlo', ...
        'test_readStepData', ...
        'test_sensitivity', ...
        'test_rules', ...
        'test_export'};

    fprintf('\n================ simlab test suite ================\n');
    if isempty(T.ref)
        fprintf(['NOTE: c_reference.csv not found - the numeric ' ...
                 'comparisons against the C library will be SKIPPED.\n' ...
                 '      Build them with: cd tools/matlab_ref && make run\n']);
    else
        fprintf('C reference: %d values from tools/matlab_ref\n', ...
                numel(fieldnames(T.ref)));
    end
    fprintf('===================================================\n\n');

    for i = 1:numel(tests)
        name = tests{i};
        if ~isempty(filter) && isempty(strfind(lower(name), lower(filter)))
            continue;   %#ok<ISMT>  % strfind, not contains: works under Octave
        end
        fprintf('%-22s ', name);
        before = T.passed;
        beforeF = T.failed;
        T.stack{end + 1} = name; %#ok<AGROW>
        try
            fh = str2func(name);
            T = fh(T);
            if T.failed > beforeF
                fprintf('FAIL (%d/%d)\n', T.passed - before, ...
                        T.failed - beforeF);
            else
                fprintf('ok   (%d checks)\n', T.passed - before);
            end
        catch err
            T.failed = T.failed + 1;
            fprintf('ERROR\n');
            where = '';
            nf = min(3, numel(err.stack));
            for jf = 1:nf
                where = [where, sprintf(' [%s:%d]', ...
                    err.stack(jf).name, err.stack(jf).line)]; %#ok<AGROW>
            end
            T.details{end + 1} = sprintf('%s: %s%s', name, err.message, where); %#ok<AGROW>
            if verbose
                % getReport is MATLAB-only; under Octave the message and the
                % stack are all there is, and all a reader needs.
                try
                    fprintf('   %s\n', getReport(err, 'basic'));
                catch
                    fprintf('   %s\n', err.message);
                end
            end
        end
        T.stack(end) = [];
    end

    fprintf('\n---------------------------------------------------\n');
    fprintf('  %d checks passed, %d failed\n', T.passed, T.failed);
    if ~isempty(T.details)
        fprintf('\n  failures:\n');
        for i = 1:numel(T.details)
            fprintf('    - %s\n', T.details{i});
        end
    end
    fprintf('---------------------------------------------------\n');

    results = T;
    if T.failed > 0
        error('simlab:test_suite:failures', '%d check(s) failed', T.failed);
    end
end

% ===========================================================================
% Reference file
% ===========================================================================

function ref = loadReference(path)
% Read key,value pairs. Returns an empty struct when the file is absent, so
% the caller can skip the numeric tests and SAY so rather than passing them.
    ref = struct();
    if ~exist(path, 'file')
        return;
    end
    fid = fopen(path, 'r');
    while true
        line = fgetl(fid);
        if ~ischar(line), break; end
        if isempty(line) || line(1) == '#'
            continue;
        end
        k = strfind(line, ',');
        if isempty(k), continue; end
        key = strrep(line(1:k(1) - 1), '.', '_');
        key = simlab_tests.keyName(key);
        val = str2double(line(k(1) + 1:end));
        ref.(key) = val;
    end
    fclose(fid);
end


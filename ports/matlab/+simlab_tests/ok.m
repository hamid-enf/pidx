function T = ok(T, cond, fmt, varargin)
%SIMLAB_TESTS.OK  Record one assertion.
%
% A test framework in four functions rather than a dependency on
% matlab.unittest: the suite has to run under Octave and under a MATLAB with
% no toolboxes, and it has to print a line per check so a failure is readable
% without a debugger.
    if nargin > 2
        msg = sprintf(fmt, varargin{:});
    else
        msg = '(unnamed check)';
    end
    if ~isfield(T, 'stack') || isempty(T.stack)
        where = 'unknown test';
    else
        where = T.stack{end};
    end
    if cond
        T.passed = T.passed + 1;
        if isfield(T, 'verbose') && T.verbose
            fprintf('\n      ok  %s', msg);
        end
    else
        T.failed = T.failed + 1;
        T.details{end + 1} = sprintf('[%s] %s', where, msg); %#ok<AGROW>
        fprintf('\n    FAIL [%s]: %s', where, msg);
    end
end

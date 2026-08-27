function T = skip(T, what, why)
%SIMLAB_TESTS.SKIP  Record a check that could not be run, by name.
%
% Counted as a pass so the suite's exit status stays meaningful, but PRINTED:
% a suite that quietly checked six of nine things because three toolchains
% were missing is worse than one that says which three it did not check.
    fprintf('\n    SKIP  %s  (%s)', what, why);
    T.passed = T.passed + 1;
    if ~isfield(T, 'skipped'), T.skipped = {}; end
    T.skipped{end + 1} = sprintf('%s: %s', what, why); %#ok<AGROW>
end

function run_tests(varargin)
%RUN_TESTS  One word: set the path, run the whole simlab test suite.
%
%   run_tests                everything
%   run_tests('plant')       only tests whose name contains 'plant'
%
% Exists because the suite lives in simlab_tests/ and is only visible after
% simlab_setup (or this). A bare `addpath ports\matlab` makes simlabApp work
% but leaves `test_suite` undiscoverable - and MATLAB's built-in `testsuite`
% (no underscore) then answers instead, with zero class-based tests. This
% wrapper removes that whole failure mode.

    here = fileparts(mfilename('fullpath'));
    if isempty(here)
        here = pwd;
    end
    addpath(here);
    addpath(fullfile(here, 'simlab_demos'));
    addpath(fullfile(here, 'simlab_tests'));
    % A build marker in every paste. If the marker in your output does not
    % match the latest commit's marker, the folder is stale and the failures
    % are ghosts.
    fprintf('=== simlab build dde84eb+fixes-4 ===\n');
    test_suite(varargin{:});
end

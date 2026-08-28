%SIMLAB_SETUP  Put the PIDX MATLAB tool on the path.
%
%   cd <repo>/ports/matlab
%   simlab_setup
%
% Adds this directory to the MATLAB path so that the two packages - +pidx
% (the verified controller port) and +simlab (the simulation tool built on
% it) - and the test suite become visible. A package directory must be a
% CHILD of a directory on the path, never on the path itself, which is why
% this adds ports/matlab rather than ports/matlab/+simlab.
%
% After this:
%   simlab_wizard            guided console workflow
%   simlabApp                graphical interface
%   simlab_demos.demo_quick  five-minute tour, no interaction
%   cd simlab_tests && test_suite

    here = fileparts(mfilename('fullpath'));
    if isempty(here)
        here = pwd;
    end
    addpath(here);
    addpath(fullfile(here, 'simlab_demos'));
    addpath(fullfile(here, 'simlab_tests'));

    fprintf('\n');
    fprintf('  PIDX MATLAB tool\n');
    fprintf('  ----------------\n');
    fprintf('  path      %s\n', here);
    fprintf('  library   PIDX %s\n', pidx.Const.VERSION_STRING);
    fprintf('  runtime   %s\n', version);
    fprintf('\n');
    fprintf('  start here:\n');
    fprintf('    simlab_demos.demo_quick     five-minute tour\n');
    fprintf('    simlab_wizard               guided console workflow\n');
    fprintf('    simlabApp                   graphical interface\n');
    fprintf('    test_suite                  verify everything\n');
    fprintf('\n');

    refFile = fullfile(here, 'simlab_tests', 'c_reference.csv');
    if exist(refFile, 'file') ~= 2
        fprintf('  NOTE: simlab_tests/c_reference.csv is missing, so the\n');
        fprintf('        numeric comparisons against the C library will be\n');
        fprintf('        skipped. Build them with:\n');
        fprintf('          cd %s && make run\n', ...
            fullfile(here, '..', '..', 'tools', 'matlab_ref'));
        fprintf('\n');
    end

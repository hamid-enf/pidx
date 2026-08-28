function T = eq(T, got, want, what, varargin)
%SIMLAB_TESTS.EQ  Exact comparison, for integers and flags.
% Accepts sprintf-style extra arguments like ok(), because several call
% sites label the check with numbers; an arity mismatch there is a parse
% error at runtime, which is how this was found.
    if nargin > 4
        label = sprintf(what, varargin{:});
    else
        label = what;
    end
    T = simlab_tests.ok(T, got == want, '%s: got %s, expected %s', ...
        label, num2str(got), num2str(want));
end

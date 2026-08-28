function T = near(T, got, want, tol, what, varargin)
%SIMLAB_TESTS.NEAR  Relative comparison against a reference value.
%
% The denominator floor of 1 keeps an expected value of zero from making
% every tolerance meaningless, and keeps a tiny expected value from failing
% on ordinary double-precision noise.
    if nargin < 4 || isempty(tol), tol = 1e-9; end
    d = abs(got - want) / max([1, abs(got), abs(want)]);
    if nargin > 5 && ~isempty(varargin)
        full = sprintf(what, varargin{:});
    else
        full = what;
    end
    T = simlab_tests.ok(T, d <= tol, ...
        '%s: got %.17g, expected %.17g (rel diff %.3g, tolerance %.3g)', ...
        full, got, want, d, tol);
end

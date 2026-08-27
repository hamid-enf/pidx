function T = eq(T, got, want, what)
%SIMLAB_TESTS.EQ  Exact comparison, for integers and flags.
    T = simlab_tests.ok(T, got == want, '%s: got %s, expected %s', ...
        what, num2str(got), num2str(want));
end

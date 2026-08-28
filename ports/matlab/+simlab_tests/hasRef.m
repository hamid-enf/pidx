function ok = hasRef(T)
%SIMLAB_TESTS.HASREF  True when the C reference table was loaded.
%
% Tests use this to SKIP a numeric comparison rather than to run it against
% nothing. A skipped comparison is printed by name, so a suite that could not
% reach its oracle does not read like one that passed.
    ok = isfield(T, 'ref') && ~isempty(fieldnames(T.ref));
end

function v = refGet(T, key, default)
%SIMLAB_TESTS.REFGET  Read a value from the C reference table.
%
% Returns DEFAULT (normally NaN) when the key or the whole file is missing,
% so a test can skip a comparison and say so rather than compare against
% nothing and pass.
    if nargin < 3, default = NaN; end
    if ~isfield(T, 'ref') || isempty(fieldnames(T.ref))
        v = default;
        return;
    end
    k = simlab_tests.keyName(key);
    if isfield(T.ref, k)
        v = T.ref.(k);
    else
        v = default;
    end
end

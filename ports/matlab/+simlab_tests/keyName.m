function k = keyName(key)
%SIMLAB_TESTS.KEYNAME  Turn a reference key into a valid struct field name.
%
% Hand-rolled rather than matlab.lang.makeValidName, which is R2014a+ and
% absent from some Octave builds. The reference keys are [A-Za-z0-9_.], so
% mapping the punctuation to underscores is the whole job.
    k = key;
    k(~isstrprop(k, 'alphanum')) = '_';
    if ~isempty(k) && (isstrprop(k(1), 'digit') || k(1) == '_')
        k = ['k', k];
    end
end

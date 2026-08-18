function m = plantModel(kind, a, b, c)
%PIDX.PLANTMODEL  Build an identified-plant struct for the tuning rules.
%
%   M = PIDX.PLANTMODEL(MODEL_FREQ,  KU, PU)     one Nyquist point
%   M = PIDX.PLANTMODEL(MODEL_FOPDT, K,  T,  L)  G(s)=K*exp(-L*s)/(1+T*s)
%
%   The intermediate data type that joins identification to tuning. Keeping
%   the two stages separate - and making the model explicit - is what lets
%   pidx.ruleApply reject a rule that cannot be evaluated from the data you
%   actually have, instead of inventing a conversion.

    K = pidx.Const;
    if nargin < 1, kind = K.MODEL_NONE; end
    if nargin < 2, a = 0; end
    if nargin < 3, b = 0; end
    if nargin < 4, c = 0; end

    m = struct('kind', kind, 'ku', 0, 'pu', 0, 'k', 0, 't', 0, 'l', 0, ...
               'noise_sigma', 0, 'quality', 0);

    if kind == K.MODEL_FREQ
        m.ku = a;      % ultimate gain
        m.pu = b;      % ultimate period [s]
    elseif kind == K.MODEL_FOPDT
        m.k = a;       % static gain
        m.t = b;       % time constant [s]
        m.l = c;       % dead time [s]
    end
end

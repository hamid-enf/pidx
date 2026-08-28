classdef Plant < handle
    % SIMLAB.PLANT  A discrete-time process model with an actuator and a
    % sensor chain wrapped around it.
    %
    % WHY THIS IS A CLASS AND NOT A TRANSFER FUNCTION
    %   The controller under test is PIDX, which is a sample-by-sample state
    %   machine. Comparing it against a continuous tf/ss model would answer a
    %   different question. This plant is stepped at exactly the controller's
    %   dt, so saturation, dead time, quantisation and a failing sensor all
    %   land where they land on the real target.
    %
    % THE THREE STAGES, in the order a real signal passes through them:
    %
    %     u_cmd -->[ actuator ]--> u_plant -->[ model ]--> y_true -->[ sensor ]--> y_meas
    %
    %   actuator : saturation, deadband, rate limit, quantisation (PWM
    %              resolution, DAC steps), optional user nonlinearity
    %   model    : 'linear' (any tf, ZOH-discretised) | 'fopdt' (exact)
    %              | 'dc_motor' (2-state, sub-stepped) | 'custom' (your own
    %              function handle)
    %   sensor   : gain/bias, Gaussian noise, ADC quantisation, deadband,
    %              rate limit, transport delay, stuck-at and dropout faults
    %
    % Every stage is OFF by default. A plant built with only a model is a pure
    % plant, and `yMeas == yTrue` to the last bit.
    %
    % EXAMPLE - a heater with 30 s dead time and a noisy thermocouple:
    %   pl = simlab.Plant('fopdt', 'k', 2.0, 't', 45, 'l', 12);
    %   pl.setNoise(0.15);
    %   pl.setAdcBits(12, 0, 300);
    %   y  = pl.update(50, 0.1);        % one 100 ms sample
    %
    % EXAMPLE - the DC motor from examples_stm32/03_motor_speed, with the same
    % physical constants, so the MATLAB study and the STM32 example agree:
    %   pl = simlab.Plant.presets('dcMotor');
    %   pl.setLoad(0.004, 2.0);         % 4 mNm load torque from t = 2 s
    %
    % CUSTOM MODEL - integrate it yourself, PIDX does not care how:
    %   f = @(x, u, dt) [ -x(1)/0.5 + 2*u ; x(1) ];   % dx/dt
    %   pl = simlab.Plant('custom', 'f', f, 'n', 2, 'yIndex', 2, ...
    %                     'x0', [0 0], 'substeps', 10);
    %
    % HONEST LIMITS
    %   'linear' is discretised with c2d(...,'zoh'), which needs the Control
    %   System Toolbox. 'fopdt', 'dc_motor' and 'custom' are hand-integrated
    %   and need nothing. If c2d is missing the constructor says so and
    %   suggests 'fopdt' rather than quietly returning a wrong model.

    properties (SetAccess = private)
        kind = 'fopdt';       % 'linear' | 'fopdt' | 'dc_motor' | 'custom'
        name = 'unnamed plant';
        dt = 0;               % dt of the last update(); 0 before the first
        t  = 0;               % plant time, advanced by update()
        nUpdates = 0;

        % What the controller thinks it is controlling, and what it is
        % really controlling. Both are recorded every sample so a plot can
        % show exactly how much the sensor chain is lying.
        yTrue = 0;
        yMeas = 0;
        uCmd = 0;             % command as the controller issued it
        uPlant = 0;           % command after the actuator chain
    end

    properties (Access = private)
        % ---- model: fopdt ----
        p_k = 1; p_tau = 1; p_L = 0;

        % ---- model: linear ----
        p_num = [1]; p_den = [1 1];
        ad_a = []; ad_b = [];     % discretised state matrices
        ad_c = []; ad_d = [];
        ad_dt = 0;
        xs = [];                  % discrete state

        % ---- model: dc motor ----
        m_R = 1; m_L = 5e-4; m_Ke = 0.05; m_Kt = 0.05;
        m_J = 1e-4; m_B = 2e-3; m_coulomb = 3e-3;
        m_i = 0; m_w = 0; m_theta = 0;
        m_load = 0; m_loadT0 = inf;
        m_substeps = 0;           % 0 -> derived from L/R

        % ---- model: custom ----
        f_custom = []; n_state = 1; y_index = 1; x0 = 0; cust_sub = 1;
        x_cust = 0;

        % ---- shared: input delay + internal state of the delay line ----
        delay_s = 0; delay_n = 0;
        u_buf = [];               % ring buffer of plant inputs

        % ---- actuator chain ----
        a_umin = -inf; a_umax = inf; a_sat = false;
        a_dead = 0;
        a_slew = 0; a_prev = 0; a_slew_primed = false;
        a_quant = 0;
        a_fn = [];

        % ---- sensor chain ----
        s_gain = 1; s_bias = 0;
        s_sigma = 0;
        s_qmin = 0; s_qmax = 0; s_bits = 0;   % 0 -> no quantisation
        s_dead = 0; s_last_good = 0; s_primed = false;
        s_rate = 0; s_prev_out = 0;
        s_delay_s = 0; s_delay_n = 0; s_buf = [];
        s_stuck = false; s_stuck_val = 0;
        s_dropout_p = 0;
        s_rng_seeded = false;
    end

    methods
        function o = Plant(kind, varargin)
            % PLANT(KIND, 'name', value, ...) - see the class help.
            if nargin < 1, kind = 'fopdt'; end
            o.kind = lower(kind);
            switch o.kind
                case {'linear', 'fopdt', 'dc_motor', 'custom'}
                otherwise
                    error('simlab:Plant:badKind', ...
                          'unknown plant kind "%s" (linear|fopdt|dc_motor|custom)', ...
                          kind);
            end
            o = o.set(varargin{:});
            o.rebuildDiscrete();
        end

        function o = set(o, varargin)
            % SET('name', value, ...) - fluent parameter setter.
            % Unknown names are an error, not a silent no-op: a typo in a
            % parameter name is the single easiest way to tune a controller
            % against a plant that is not the one you described.
            for i = 1:2:numel(varargin)
                nm = lower(varargin{i});
                v  = varargin{i + 1};
                switch nm
                    case 'name',      o.name = v;
                    case 'k',         o.p_k = v;
                    case 'tau',       o.p_tau = v;
                    case 't',         o.p_tau = v;      % alias, matches C
                    case 'l',         o.p_L = v;
                    case 'deadtime',  o.p_L = v;
                    case 'num',       o.p_num = v(:).';
                    case 'den',       o.p_den = v(:).';
                    case {'r', 'resistance'},   o.m_R = v;
                    case {'ind', 'inductance'}, o.m_L = v;
                    case 'ke',        o.m_Ke = v;
                    case 'kt',        o.m_Kt = v;
                    case {'j', 'inertia'},      o.m_J = v;
                    case 'b',         o.m_B = v;
                    case 'coulomb',   o.m_coulomb = v;
                    case 'substeps',  o.m_substeps = max(0, round(v));
                    case 'f',         o.f_custom = v;
                    case 'n',         o.n_state = v;
                    case 'yindex',    o.y_index = v;
                    case 'x0',        o.x0 = v(:);
                    case 'delay',     o.delay_s = max(0, v);
                    otherwise
                        error('simlab:Plant:badParam', ...
                              'unknown parameter "%s"', varargin{i});
                end
            end
            o.rebuildDiscrete();
        end

        % ==========================================================
        % Chain configuration
        % ==========================================================

        function o = setActuatorLimits(o, lo, hi)
            o.a_umin = lo; o.a_umax = hi;
            o.a_sat = isfinite(lo) || isfinite(hi);
        end

        function o = setActuatorDeadband(o, band)
            % Symmetric deadband around zero: |u| < band/2 is not delivered.
            % Models stiction and a valve that needs a minimum drive.
            o.a_dead = max(0, band);
        end

        function o = setActuatorSlew(o, rate)
            % Max |du/dt| [unit/s]. A real drive cannot step its output.
            o.a_slew = max(0, rate);
        end

        function o = setActuatorQuantisation(o, step)
            % Round the delivered command to a multiple of STEP.
            % step = supply/range models a PWM with a fixed resolution.
            o.a_quant = max(0, step);
        end

        function o = setActuatorFn(o, fn)
            % u_plant = fn(u_cmd). Escape hatch for hysteresis, backlash or a
            % valve characteristic. Applied AFTER limits and slew.
            o.a_fn = fn;
        end

        function o = setNoise(o, sigma)
            % Zero-mean Gaussian measurement noise, standard deviation SIGMA.
            % Seeded deterministically - see reset().
            o.s_sigma = max(0, sigma);
        end

        function o = setAdcBits(o, bits, lo, hi)
            % Quantise the measurement to a BITS-bit converter spanning
            % [lo, hi]. bits = 0 disables. This is what turns a smooth
            % simulation into the staircase the derivative term actually sees.
            o.s_bits = max(0, bits);
            o.s_qmin = lo; o.s_qmax = hi;
        end

        function o = setSensorGainBias(o, gain, bias)
            % Calibration error: y = gain*y_true + bias.
            o.s_gain = gain; o.s_bias = bias;
        end

        function o = setSensorDeadband(o, band)
            % Report the last accepted value while the change is < band.
            o.s_dead = max(0, band);
        end

        function o = setSensorRateLimit(o, rate)
            o.s_rate = max(0, rate);
        end

        function o = setSensorDelay(o, seconds)
            % Pure transport delay on the measurement, in seconds. Rounded to
            % a whole number of samples - which is exactly what a sampled
            % system does.
            o.s_delay_s = max(0, seconds);
            o.rebuildDelays();
        end

        function o = setStuckAt(o, value)
            % Freeze the sensor at VALUE. [] releases it.
            if isempty(value)
                o.s_stuck = false;
            else
                o.s_stuck = true; o.s_stuck_val = value;
            end
        end

        function o = setDropout(o, prob)
            % Each sample, with probability PROB the sensor repeats its
            % previous value (a dropped conversion / a missed CAN frame).
            o.s_dropout_p = min(max(prob, 0), 1);
        end

        function o = setLoad(o, torque, t0)
            % DC motor only: constant load torque from time t0 [s].
            o.m_load = torque; o.m_loadT0 = t0;
        end

        % ==========================================================
        % Presets
        % ==========================================================

        function o = reset(o)
            % Return every state variable to zero and rewind time. The random
            % stream is re-seeded, so a scenario replays bit-for-bit: two runs
            % with the same gains must differ only if the gains differ.
            o.t = 0; o.dt = 0; o.nUpdates = 0;
            o.yTrue = 0; o.yMeas = 0; o.uCmd = 0; o.uPlant = 0;

            % fopdt holds ONE state; a linear plant holds one per pole.
            % zeros(size(o.ad_a,1),1) gives the FOPDT an EMPTY state (ad_a is
            % empty until a c2d runs) and the first update dies on xs(1).
            % stepLinear() widens this scalar whenever the linear model needs
            % more, so a plain 0 is the right seed for every kind.
            o.xs = 0;
            o.m_i = 0; o.m_w = 0; o.m_theta = 0;
            o.x_cust = o.x0;

            o.rebuildDelays();
            o.a_prev = 0; o.a_slew_primed = false;
            o.s_last_good = 0; o.s_primed = false; o.s_prev_out = 0;
            rng(12345, 'twister');
            o.s_rng_seeded = true;
        end

        function y = update(o, u_cmd, dt)
            % One sample: actuator -> model -> sensor. Returns the value the
            % controller would read.
            if ~(dt > 0) || ~isfinite(dt)
                error('simlab:Plant:badDt', 'dt must be finite and > 0');
            end
            if ~o.s_rng_seeded, o.reset(); end
            % The delay lines are sized by dt, which is not known at
            % construction. A plant built with L = 12 s and stepped for the
            % first time at dt = 0.05 must build its 240-sample buffer HERE;
            % without this the very first study ran an UNDELAYED plant, which
            % is a different process - and the auto-tune tests proved it.
            if o.dt ~= dt
                o.dt = dt;
                o.rebuildDelays();
            end
            o.dt = dt;
            o.uCmd = u_cmd;

            % ---- actuator chain ----
            u = u_cmd;
            if o.a_sat
                u = min(max(u, o.a_umin), o.a_umax);
            end
            if o.a_slew > 0
                if ~o.a_slew_primed
                    o.a_prev = u; o.a_slew_primed = true;
                else
                    lim = o.a_slew * dt;
                    d = u - o.a_prev;
                    if d >  lim, u = o.a_prev + lim;
                    elseif d < -lim, u = o.a_prev - lim;
                    end
                    o.a_prev = u;
                end
            end
            if o.a_dead > 0
                half = 0.5 * o.a_dead;
                if abs(u) < half, u = 0; end
            end
            if o.a_quant > 0
                u = o.a_quant * round(u / o.a_quant);
            end
            if ~isempty(o.a_fn)
                u = o.a_fn(u);
            end
            o.uPlant = u;

            % ---- input delay ----
            u_plant = o.applyInputDelay(u);

            % ---- model ----
            switch o.kind
                case 'fopdt',    y_true = o.stepFopdt(u_plant, dt);
                case 'linear',   y_true = o.stepLinear(u_plant, dt);
                case 'dc_motor', y_true = o.stepMotor(u_plant, dt);
                otherwise,       y_true = o.stepCustom(u_plant, dt);
            end
            o.yTrue = y_true;

            % ---- sensor chain ----
            y = y_true * o.s_gain + o.s_bias;

            if o.s_stuck
                y = o.s_stuck_val;
            elseif o.s_dropout_p > 0 && o.s_primed && rand() < o.s_dropout_p
                y = o.s_prev_out;
            else
                if o.s_sigma > 0
                    y = y + o.s_sigma * randn();
                end
                if o.s_bits > 0
                    q = (o.s_qmax - o.s_qmin) / (2^o.s_bits - 1);
                    y = o.s_qmin + q * round((y - o.s_qmin) / q);
                end
                if o.s_rate > 0 && o.s_primed
                    lim = o.s_rate * dt;
                    d = y - o.s_prev_out;
                    if d >  lim, y = o.s_prev_out + lim;
                    elseif d < -lim, y = o.s_prev_out - lim;
                    end
                end
                if o.s_dead > 0 && o.s_primed
                    if abs(y - o.s_last_good) < o.s_dead
                        y = o.s_last_good;
                    else
                        o.s_last_good = y;
                    end
                else
                    o.s_last_good = y;
                end
            end
            o.s_prev_out = y;
            o.s_primed = true;

            y = o.applySensorDelay(y);

            o.yMeas = y;
            o.t = o.t + dt;
            o.nUpdates = o.nUpdates + 1;
        end

        function st = motorState(o)
            % [current; speed; position] - for plotting and for cascades.
            st = [o.m_i; o.m_w; o.m_theta];
        end

        function v = state(o, name)
            % STATE(NAME) - one internal variable by name.
            %
            % A cascade measures a different variable at every level, and a
            % custom model has states with no obvious meaning. Naming them is
            % how the measurement function stays readable:
            %
            %   measFn = @(pl) [pl.state('theta'); pl.state('speed')];
            %
            % Reading state directly from outside would need the fields to be
            % public, and a half-updated state is worse than a private one.
            switch lower(name)
                case {'i', 'current'},  v = o.m_i;
                case {'w', 'speed'},    v = o.m_w;
                case {'theta', 'position'}, v = o.m_theta;
                case {'x', 'y', 'output'},  v = o.xs;
                case 'custom',          v = o.x_cust;
                case 'true',            v = o.yTrue;
                case 'meas',            v = o.yMeas;
                otherwise
                    error('simlab:Plant:badState', ...
                          'unknown state "%s"', name);
            end
        end

        function [z, p, k, l] = polesZeros(o)
            % [ZEROS, POLES, GAIN, DEADTIME] of the model, for analysis.
            %
            % A public accessor rather than public fields: the state variables
            % have to stay private so nothing can half-update them, but the
            % model description is exactly what a frequency-domain analysis
            % needs. Dead time is returned as an exact delay, not as a Pade
            % approximation - see simlab.sensitivity for why that matters.
            z = [];
            l = o.p_L + o.delay_s;
            switch o.kind
                case 'fopdt'
                    p = -1 / o.p_tau;
                    k = o.p_k;
                case 'dc_motor'
                    % Speed over armature voltage, from the model's two state
                    % equations:
                    %   W(s)/V(s) = Kt / (L*J*s^2 + (L*B + R*J)*s
                    %                             + (R*B + Kt*Ke))
                    a2 = o.m_L * o.m_J;
                    a1 = o.m_L * o.m_B + o.m_R * o.m_J;
                    a0 = o.m_R * o.m_B + o.m_Kt * o.m_Ke;
                    p = roots([a2, a1, a0]);
                    k = o.m_Kt / a0;
                case 'linear'
                    [z, p, k] = simlab.Plant.tf2zpk(o.p_num, o.p_den);
                otherwise
                    error('simlab:Plant:noTf', ...
                          ['a ''custom'' plant has no declared transfer ' ...
                           'function. Identify a FOPDT model with ' ...
                           'simlab.AutoTune and analyse that.']);
            end
        end

        function o = degradeSensor(o, noiseFactor, bitsLost)
            % DEGRADESENSOR(NOISEFACTOR, BITSLOST) - make the sensor worse,
            % in place, and leave the process alone.
            %
            % This is the "how much does D actually cost me" experiment: the
            % plant is identical, so any change in the result is the
            % controller's response to noise and nothing else.
            if nargin < 2, noiseFactor = 1; end
            if nargin < 3, bitsLost = 0; end
            if o.s_sigma > 0
                o.s_sigma = o.s_sigma * noiseFactor;
            end
            if o.s_bits > bitsLost
                o.s_bits = o.s_bits - bitsLost;
            elseif o.s_bits > 0
                o.s_bits = 1;
            end
        end

        function v = modelParam(o, name)
            % Read a model parameter. The state itself stays private - half
            % of a state update is worse than none - but a Monte Carlo study
            % has to be able to read K, tau and L in order to scale them.
            switch lower(name)
                case {'k', 'gain'},       v = o.p_k;
                case {'tau', 't'},        v = o.p_tau;
                case {'l', 'deadtime'},   v = o.p_L;
                case {'r', 'resistance'}, v = o.m_R;
                case {'l_elec', 'ind'},   v = o.m_L;
                case 'ke',                v = o.m_Ke;
                case 'kt',                v = o.m_Kt;
                case {'j', 'inertia'},    v = o.m_J;
                case 'b',                 v = o.m_B;
                case 'coulomb',           v = o.m_coulomb;
                case 'load',              v = o.m_load;
                case 'loadt0',            v = o.m_loadT0;
                otherwise
                    error('simlab:Plant:badModelParam', ...
                          'unknown model parameter "%s"', name);
            end
        end

        function v = actuatorParam(o, name)
            switch lower(name)
                case 'deadband', v = o.a_dead;
                case 'slew',     v = o.a_slew;
                case 'quant',    v = o.a_quant;
                case 'fn',       v = o.a_fn;
                case 'umin',     v = o.a_umin;
                case 'umax',     v = o.a_umax;
                otherwise
                    error('simlab:Plant:badParam', ...
                          'unknown actuator parameter "%s"', name);
            end
        end

        function [lo, hi] = actuatorLimits(o)
            lo = o.a_umin; hi = o.a_umax;
        end

        function v = sensorParam(o, name)
            switch lower(name)
                case 'sigma',    v = o.s_sigma;
                case 'bits',     v = o.s_bits;
                case 'qmin',     v = o.s_qmin;
                case 'qmax',     v = o.s_qmax;
                case 'gain',     v = o.s_gain;
                case 'bias',     v = o.s_bias;
                case 'deadband', v = o.s_dead;
                case 'rate',     v = o.s_rate;
                case 'delay',    v = o.s_delay_s;
                case 'dropout',  v = o.s_dropout_p;
                otherwise
                    error('simlab:Plant:badParam', ...
                          'unknown sensor parameter "%s"', name);
            end
        end

        function v = numerator(o)
            % Continuous numerator polynomial, highest power first.
            v = o.p_num;
        end

        function v = denominator(o)
            v = o.p_den;
        end

        function v = transportDelay(o)
            % Dead time configured on the plant, in seconds. Note this is the
            % INPUT delay; the sensor's own delay is sensorParam('delay').
            v = o.delay_s;
        end

        function v = tau(o)
            % FOPDT time constant. For any other kind, the dominant pole's
            % time constant, which is the number a tuning rule would want.
            if strcmp(o.kind, 'fopdt')
                v = o.p_tau;
            else
                [~, p] = o.polesZeros();
                p = p(isfinite(p));
                p = p(abs(p) > 1e-12);
                if isempty(p)
                    v = Inf;
                else
                    v = 1 / max(abs(p));
                end
            end
        end

        function w = analysisCaveats(o)
            % What a LINEAR analysis of this plant does not see.
            %
            % Returned as a cell array of sentences. simlab.sensitivity and
            % the exported report both embed it, because margins computed
            % around a saturation or a deadband are not wrong so much as
            % incomplete, and the reader deserves to be told which.
            w = {};
            if o.a_sat
                w{end + 1} = sprintf('actuator saturates at [%.6g, %.6g] - not in a linear margin', ...
                                     o.a_umin, o.a_umax);
            end
            if o.a_dead > 0
                w{end + 1} = sprintf('actuator deadband %.6g - the loop will limit-cycle if the gains are high', o.a_dead);
            end
            if o.a_quant > 0
                w{end + 1} = sprintf('actuator quantised in steps of %.6g', o.a_quant);
            end
            if ~isempty(o.a_fn)
                w{end + 1} = 'actuator has a user nonlinearity (setActuatorFn)';
            end
            if o.s_bits > 0
                w{end + 1} = sprintf('measurement quantised to %d bits', o.s_bits);
            end
            if o.s_sigma > 0
                w{end + 1} = sprintf('measurement noise sigma %.6g', o.s_sigma);
            end
            if o.s_delay_s > 0
                w{end + 1} = sprintf('sensor transport delay %.6g s - ADD this to the model dead time before trusting a margin', o.s_delay_s);
            end
            if o.s_stuck
                w{end + 1} = 'sensor is currently stuck at a fixed value';
            end
            if strcmp(o.kind, 'dc_motor')
                w{end + 1} = ['DC motor: analysed as the linear speed/voltage ' ...
                              'model. Coulomb friction and load torque are not in it.'];
            end
        end

        function y = steadyStateGain(o)
            % Static gain of the model. Used by feedforward and by the
            % sanity check that warns when Kp looks dimensionally wrong.
            switch o.kind
                case 'fopdt'
                    y = o.p_k;
                case 'dc_motor'
                    % w_inf = (Kt*V - friction) / (B + Kt*Ke/R) at no load
                    y = o.m_Kt / (o.m_B + o.m_Kt * o.m_Ke / o.m_R);
                case 'linear'
                    y = sum(o.p_num) / sum(o.p_den);
                otherwise
                    y = NaN;   % a custom model has no declared gain
            end
        end
    end

    % ==================================================================
    % Model implementations
    % ==================================================================

    methods (Access = private)

        function rebuildDiscrete(o)
            % Discretise the linear model at the dt it will be run with.
            %
            % The dt is not known at construction time, so the matrices are
            % rebuilt lazily by stepLinear() whenever dt changes. Doing it
            % here too (with ad_dt = 0) simply clears the cache.
            o.ad_dt = 0;
            o.rebuildDelays();
        end

        function rebuildDelays(o)
            n = 0;
            if o.dt > 0
                n = round(o.delay_s / o.dt);
            end
            if n ~= o.delay_n
                o.delay_n = n;
                o.u_buf = zeros(n, 1);
            end
            ns = 0;
            if o.dt > 0
                ns = round(o.s_delay_s / o.dt);
            end
            if ns ~= o.s_delay_n
                o.s_delay_n = ns;
                o.s_buf = zeros(ns, 1);
            end
        end

        function u_out = applyInputDelay(o, u)
            % Integer-sample delay line. A fractional part is rounded, which
            % is the honest thing: the plant is sampled, so a delay of 1.4
            % samples is not distinguishable from 1 sample at this dt.
            if o.delay_n <= 0
                u_out = u;
                return;
            end
            u_out = o.u_buf(1);
            o.u_buf(1:end-1) = o.u_buf(2:end);
            o.u_buf(end) = u;
        end

        function y_out = applySensorDelay(o, y)
            if o.s_delay_n <= 0
                y_out = y;
                return;
            end
            y_out = o.s_buf(1);
            o.s_buf(1:end-1) = o.s_buf(2:end);
            o.s_buf(end) = y;
        end

        function y = stepFopdt(o, u, dt)
            % G(s) = K exp(-L s) / (1 + tau s), exact backward-Euler pole.
            %
            % Backward Euler rather than an exact exponential on purpose: it
            % is unconditionally stable at any dt, and at dt << tau it agrees
            % with exp(-dt/tau) to O(dt^2), which is far below the modelling
            % error of a FOPDT fit anyway.
            a = o.p_tau / (o.p_tau + dt);
            o.xs = a * o.xs + (1 - a) * (o.p_k * u);
            y = o.xs(1);
        end

        function y = stepLinear(o, u, dt)
            % ZOH discretisation of an arbitrary tf, cached per dt.
            if isempty(o.ad_a) || abs(o.ad_dt - dt) > 1e-15 * max(1, dt)
                if ~license('test', 'Control_Toolbox') && ~exist('c2d', 'file')
                    error('simlab:Plant:noCst', ...
                          ['simlab.Plant(''linear'') needs c2d (Control System ' ...
                           'Toolbox). Use ''fopdt'' or ''custom'', which are ' ...
                           'hand-integrated and need no toolbox.']);
                end
                sys = tf(o.p_num, o.p_den);
                d = c2d(sys, dt, 'zoh');
                [o.ad_a, o.ad_b, o.ad_c, o.ad_d] = ssdata(d);
                if isempty(o.xs) || numel(o.xs) ~= size(o.ad_a, 1)
                    % fopdt holds ONE state; a linear plant holds one per pole.
            % zeros(size(o.ad_a,1),1) gives the FOPDT an EMPTY state (ad_a is
            % empty until a c2d runs) and the first update dies on xs(1).
            % stepLinear() widens this scalar whenever the linear model needs
            % more, so a plain 0 is the right seed for every kind.
            o.xs = 0;
                end
                o.ad_dt = dt;
            end
            y = o.ad_c * o.xs + o.ad_d * u;
            o.xs = o.ad_a * o.xs + o.ad_b * u;
        end

        function y = stepMotor(o, v, dt)
            % The DC motor of examples_stm32/03_motor_speed, same equations
            % and the same sub-stepping rule, so a MATLAB study and the STM32
            % example are describing the same machine.
            %
            %   L di/dt = v - R i - Ke w
            %   J dw/dt = Kt i - B w - T_load - T_coulomb*sign(w)
            %   d(theta)/dt = w
            tau_elec = o.m_L / o.m_R;
            if o.m_substeps > 0
                n = o.m_substeps;
            else
                % Each sub-step at most a tenth of the electrical time
                % constant - below that the explicit Euler is inside its
                % stability margin with room to spare.
                n = floor(dt / (0.1 * tau_elec)) + 1;
            end
            h = dt / n;
            tl = 0;
            if o.t >= o.m_loadT0, tl = o.m_load; end

            for i = 1:n
                di = (v - o.m_R * o.m_i - o.m_Ke * o.m_w) / o.m_L;
                o.m_i = o.m_i + di * h;

                if o.m_w > 0.01
                    tc = o.m_coulomb;
                elseif o.m_w < -0.01
                    tc = -o.m_coulomb;
                else
                    tc = 0;
                end
                dw = (o.m_Kt * o.m_i - o.m_B * o.m_w - tl - tc) / o.m_J;
                o.m_w = o.m_w + dw * h;
                o.m_theta = o.m_theta + o.m_w * h;
            end
            y = o.m_w;
        end

        function y = stepCustom(o, u, dt)
            % x' = f(x, u, dt); y = x(y_index). Sub-stepped for stiffness.
            h = dt / o.cust_sub;
            for i = 1:o.cust_sub
                dx = o.f_custom(o.x_cust, u, h);
                o.x_cust = o.x_cust + dx(:) * h;
            end
            y = o.x_cust(o.y_index);
        end
    end

    % ==================================================================
    % Preset library
    % ==================================================================

    methods (Static)
        function pl = presets(which, varargin)
            % PRESETS(NAME) - ready-made plants used by the demos and tests.
            %
            %   'heater'     FOPDT  K=2    tau=45   L=12     [degC / %]
            %   'extruder'   FOPDT  K=80   tau=40   L=8      [degC / %]
            %   'flowValve'  FOPDT  K=1.2  tau=0.8  L=0.15   [l/min / V]
            %   'pressure'   FOPDT  K=3.5  tau=2.0  L=0.25   [bar / V]
            %   'level'      linear integrating tank, 1/(A s), A = 0.05
            %   'dcMotor'    the motor of examples_stm32/03_motor_speed
            %   'servoPos'   second order 1/(s(0.02 s + 1))
            %   'slowThermal' FOPDT K=1 tau=300 L=90, for patient studies
            %
            % Extra 'name', value pairs are passed to set(), so
            % presets('heater', 'k', 3) is a heater with the wrong gain -
            % which is exactly the situation a robustness study needs.
            switch lower(which)
                case 'heater'
                    pl = simlab.Plant('fopdt', 'name', 'heater', ...
                        'k', 2.0, 'tau', 45, 'l', 12);
                    pl.setActuatorLimits(0, 100);
                    pl.setAdcBits(12, 0, 300);
                    pl.setNoise(0.15);
                case 'extruder'
                    pl = simlab.Plant('fopdt', 'name', 'extruder', ...
                        'k', 80, 'tau', 40, 'l', 8);
                    pl.setActuatorLimits(0, 100);
                case 'flowvalve'
                    pl = simlab.Plant('fopdt', 'name', 'flow valve', ...
                        'k', 1.2, 'tau', 0.8, 'l', 0.15);
                    pl.setActuatorLimits(0, 10);
                    pl.setNoise(0.005);
                case 'pressure'
                    pl = simlab.Plant('fopdt', 'name', 'pressure', ...
                        'k', 3.5, 'tau', 2.0, 'l', 0.25);
                    pl.setActuatorLimits(0, 10);
                case 'level'
                    pl = simlab.Plant('linear', 'name', 'tank level', ...
                        'num', 1, 'den', [0.05 0]);
                    pl.setActuatorLimits(0, 1);
                case 'dcmotor'
                    pl = simlab.Plant('dc_motor', 'name', 'DC motor', ...
                        'r', 1.0, 'ind', 5e-4, 'ke', 0.05, 'kt', 0.05, ...
                        'j', 1e-4, 'b', 2e-3, 'coulomb', 3e-3);
                    pl.setActuatorLimits(-24, 24);
                case 'servopos'
                    pl = simlab.Plant('linear', 'name', 'servo position', ...
                        'num', 1, 'den', [0.02 1 0]);
                    pl.setActuatorLimits(-10, 10);
                case 'slowthermal'
                    pl = simlab.Plant('fopdt', 'name', 'slow thermal', ...
                        'k', 1.0, 'tau', 300, 'l', 90);
                otherwise
                    error('simlab:Plant:badPreset', ...
                          'unknown preset "%s"', which);
            end
            if ~isempty(varargin)
                pl = pl.set(varargin{:});
            end
        end

        function [z, p, k] = tf2zpk(num, den)
            % Roots of numerator and denominator plus the leading-coefficient
            % ratio. Same answer as the toolbox tf2zp, with no toolbox.
            num = num(:).';
            den = den(:).';
            % Strip leading zeros: roots() would otherwise pad the answer with
            % spurious zeros at the origin.
            i1 = find(abs(num) > 0, 1, 'first');
            i2 = find(abs(den) > 0, 1, 'first');
            if isempty(i1)
                z = [];
                kn = 0;
            else
                kn = num(i1);
                if numel(num) > i1
                    z = roots(num(i1:end));
                else
                    z = [];
                end
            end
            if isempty(i2)
                error('simlab:Plant:badDen', 'denominator is all zeros');
            end
            kd = den(i2);
            if numel(den) > i2
                p = roots(den(i2:end));
            else
                p = [];
            end
            k = kn / kd;
        end

        function pl = fromIdentified(model)
            % FROMIDENTIFIED(M) - build a FOPDT plant from a pidx.plantModel
            % struct, i.e. from whatever the auto-tuner just measured. Useful
            % for replaying a tuning session, and for the Monte Carlo study
            % where each plant is a perturbed identification.
            if ~strcmp(model.kind, 'fopdt')
                error('simlab:Plant:needFopdt', ...
                      'fromIdentified needs a MODEL_FOPDT model');
            end
            pl = simlab.Plant('fopdt', 'name', 'identified', ...
                'k', model.k, 'tau', model.t, 'l', model.l);
        end
    end
end

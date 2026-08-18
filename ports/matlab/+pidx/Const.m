classdef Const
    % PIDX.CONST  Status codes, enumerations, feature bits and status flags.
    %
    % Mirrors include/pidx/pid_status.h and include/pidx/pid_types.h. Values
    % are exposed as constant properties rather than MATLAB enumeration
    % classes so that they compare and print as the same integers the C
    % library uses, and so the code runs unchanged under GNU Octave.
    %
    % Names keep their English API spelling deliberately: pidx.Const.AW_CLAMP
    % is the same identifier as PID_AW_CLAMP in the C headers.

    properties (Constant)
        % -- version ----------------------------------------------------
        VERSION_STRING = '1.0.0';
        VERSION_NUM    = 10000;      % MAJOR*10000 + MINOR*100 + PATCH
        CONFIG_ABI_VERSION = 1;

        % -- defaults ---------------------------------------------------
        DEFAULT_SAMPLE_TIME = 0.01;
        DEFAULT_N_FILTER    = 10.0;
        HUGE_F              = 1.0e30;

        GAINSCHED_MAX_POINTS = 16;
        CASCADE_MAX_LOOPS    = 4;

        % -- status codes -----------------------------------------------
        OK                      = 0;
        ERR_NULL                = 1;
        ERR_NOT_INIT            = 2;
        ERR_INVALID_CONFIG      = 3;
        ERR_INVALID_GAIN        = 4;
        ERR_INVALID_LIMIT       = 5;
        ERR_INVALID_DT          = 6;
        ERR_INVALID_MODE        = 7;
        ERR_INVALID_PARAM       = 8;
        ERR_NAN_INPUT           = 9;
        ERR_INF_INPUT           = 10;
        ERR_SENSOR_RANGE        = 11;
        ERR_SENSOR_RATE         = 12;
        ERR_UNSUPPORTED         = 13;
        ERR_BUSY                = 14;
        ERR_TUNE_TIMEOUT        = 15;
        ERR_TUNE_UNSTABLE       = 16;
        ERR_TUNE_NO_OSCILLATION = 17;
        ERR_TUNE_MODEL_MISMATCH = 18;
        ERR_TUNE_ABORTED        = 19;
        ERR_TUNE_VALIDATION     = 20;

        % -- direction --------------------------------------------------
        DIRECT  = 0;
        REVERSE = 1;

        % -- mode -------------------------------------------------------
        MODE_MANUAL    = 0;
        MODE_AUTOMATIC = 1;
        MODE_HOLD      = 2;

        % -- anti-windup ------------------------------------------------
        AW_NONE             = 0;
        AW_CLAMP            = 1;
        AW_CONDITIONAL      = 2;
        AW_BACK_CALCULATION = 3;
        AW_TRACKING         = 4;

        % -- derivative source ------------------------------------------
        DERIV_ON_MEASUREMENT    = 0;
        DERIV_ON_ERROR          = 1;
        DERIV_ON_WEIGHTED_ERROR = 2;

        % -- integration ------------------------------------------------
        INTEGRATION_BACKWARD_EULER = 0;
        INTEGRATION_TRAPEZOIDAL    = 1;

        % -- gain schedule ----------------------------------------------
        SCHED_SRC_SETPOINT    = 0;
        SCHED_SRC_MEASUREMENT = 1;
        SCHED_SRC_ERROR       = 2;
        SCHED_SRC_ABS_ERROR   = 3;
        SCHED_SRC_OUTPUT      = 4;
        SCHED_SRC_EXTERNAL    = 5;

        SCHED_INTERP_LINEAR = 0;
        SCHED_INTERP_SMOOTH = 1;
        SCHED_INTERP_HOLD   = 2;

        % -- runtime feature mask ---------------------------------------
        FEAT_INTEGRAL       = 1;        % 1 << 0
        FEAT_DERIVATIVE     = 2;
        FEAT_D_FILTER       = 4;
        FEAT_OUTPUT_LIMIT   = 8;
        FEAT_INTEGRAL_LIMIT = 16;
        FEAT_FEEDFORWARD    = 32;
        FEAT_SP_SHAPER      = 64;
        FEAT_OUT_SHAPER     = 128;
        FEAT_INPUT_FILTER   = 256;
        FEAT_SAFETY         = 512;
        FEAT_GAIN_SCHED     = 1024;
        FEAT_DIAGNOSTICS    = 2048;
        FEAT_TELEMETRY      = 4096;

        % Everything the fast path does not implement.
        FEAT_ADVANCED_MASK = 32 + 64 + 128 + 256 + 512 + 1024 + 2048 + 4096;

        % -- per-cycle status flags -------------------------------------
        FLAG_SATURATED_HIGH   = 1;      % 1 << 0
        FLAG_SATURATED_LOW    = 2;
        FLAG_INTEGRAL_ACTIVE  = 4;
        FLAG_INTEGRAL_LIMITED = 8;
        FLAG_FAULT            = 16;
        FLAG_MANUAL           = 32;
        FLAG_TUNING           = 64;
        FLAG_DT_VIOLATION     = 128;
        FLAG_SENSOR_INVALID   = 256;
        FLAG_SP_RAMPING       = 512;
        FLAG_OUTPUT_SLEWING   = 1024;

        FLAG_SATURATED = 3;             % HIGH | LOW

        % -- tuning rules -----------------------------------------------
        RULE_ZN             = 0;
        RULE_TYREUS_LUYBEN  = 1;
        RULE_PESSEN         = 2;
        RULE_SOME_OVERSHOOT = 3;
        RULE_NO_OVERSHOOT   = 4;
        RULE_AMIGO_FREQ     = 5;
        RULE_COHEN_COON     = 6;
        RULE_AMIGO_STEP     = 7;
        RULE_IMC            = 8;
        RULE_CUSTOM         = 9;

        STRUCT_P   = 0;
        STRUCT_PI  = 1;
        STRUCT_PID = 2;

        MODEL_NONE  = 0;
        MODEL_FREQ  = 1;
        MODEL_FOPDT = 2;

        IDENT_RELAY = 0;
        IDENT_STEP  = 1;
    end

    methods (Static)
        function s = statusToString(code)
            % Human-readable name of a status code. Never errors.
            names = { ...
                'OK', 'null pointer', 'not initialised', 'invalid config', ...
                'invalid gain', 'invalid limit', 'invalid dt', ...
                'invalid mode', 'invalid parameter', 'NaN input', ...
                'Inf input', 'sensor out of range', 'sensor rate exceeded', ...
                'unsupported', 'busy', 'tune timeout', 'tune unstable', ...
                'tune: no oscillation', 'tune: model mismatch', ...
                'tune aborted', 'tune: validation failed'};
            if code >= 0 && code < numel(names)
                s = names{code + 1};
            else
                s = '?';
            end
        end
    end
end

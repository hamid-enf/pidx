/**
 * @file    matlab_ref.c
 * @brief   Reference numbers for the MATLAB simulation tool.
 *
 * The MATLAB package ports/matlab/+simlab contains a port of the auto-tune
 * state machine and of the cascade coordinator. A port that is only reviewed
 * by eye is not a port, so this tool runs the SAME scenarios through the C
 * library - the oracle for this repository - and prints the numbers the
 * MATLAB tests assert against.
 *
 * It is built in double precision, exactly like ports/c_ref/conform_c.c, so
 * that a disagreement is a logic difference and not float rounding.
 *
 * Build and run (from tools/matlab_ref):
 *     make
 *     ./matlab_ref > ref.csv
 *
 * Output format: one "key,value" pair per line, %.17g. Lines starting with
 * '#' are comments. simlab_tests/test_suite.m reads this file when it is
 * present and skips the numeric assertions when it is not, so the MATLAB
 * tests never depend on a C toolchain being installed.
 */

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "pidx/pid.h"
#include "pidx/pid_autotune.h"
#include "pidx/pid_cascade.h"
#include "pidx/pid_fixed.h"

/* ======================================================================== */
/* Plant models - the same equations as simlab.Plant                         */
/* ======================================================================== */

typedef struct {
    double k;
    double tau;
    double l;        /* dead time, in whole samples */
    double state;
    double *dbuf;    /* delay line */
    int     dn;
    int     di;
} Fopdt;

static void fopdt_init(Fopdt *p, double k, double tau, double l, double dt)
{
    int n = (int)(l / dt + 0.5);
    p->k = k;
    p->tau = tau;
    p->l = l;
    p->state = 0.0;
    p->dn = n;
    p->di = 0;
    if (n > 0) {
        static double buf[4096];
        memset(buf, 0, sizeof(buf));
        p->dbuf = buf;
    } else {
        p->dbuf = NULL;
    }
}

static double fopdt_step(Fopdt *p, double u, double dt)
{
    double a = p->tau / (p->tau + dt);
    double up = u;

    if (p->dn > 0) {
        up = p->dbuf[p->di];
        p->dbuf[p->di] = u;
        p->di = (p->di + 1) % p->dn;
    }
    p->state = a * p->state + (1.0 - a) * (p->k * up);
    return p->state;
}

/* ======================================================================== */
/* Helpers                                                                   */
/* ======================================================================== */

static void emit(const char *key, double v)
{
    printf("%s,%.17g\n", key, v);
}

static void emit_i(const char *key, long v)
{
    printf("%s,%ld\n", key, v);
}

static PID_Handle mk_pid(double kp, double ki, double kd, double dt)
{
    PID_Config cfg;
    PID_Handle h;

    (void)PID_ConfigDefault(&cfg);
    cfg.core.kp = kp;
    cfg.core.ki = ki;
    cfg.core.kd = kd;
    cfg.core.sample_time = dt;
    (void)PID_Init(&h, &cfg);
    return h;
}

/* ======================================================================== */
/* 1. Closed-loop step response - the core of simlab.Sim                     */
/* ======================================================================== */
/*
 * The sample order is the one simlab.Sim uses: the plant advances under the
 * command the controller issued last cycle, then the controller reads the
 * result. Getting this order wrong silently adds a sample of dead time, so
 * the two implementations must agree on it exactly.
 */
static void scenario_step(const char *tag)
{
    const double dt = 0.1;
    const int    n  = 300;
    Fopdt plant;
    PID_Handle pid;
    double y = 0.0;
    double u = 0.0;
    double sp;
    int k;
    char key[128];

    fopdt_init(&plant, 2.0, 45.0, 12.0, dt);
    pid = mk_pid(3.0, 0.08, 0.0, dt);
    (void)PID_SetOutputLimits(&pid, 0.0, 100.0);

    for (k = 0; k < n; ++k) {
        sp = (k >= 10) ? 100.0 : 0.0;
        (void)PID_SetSetpoint(&pid, sp);

        y = fopdt_step(&plant, u, dt);
        u = PID_Update(&pid, y);

        if ((k % 25) == 0) {
            snprintf(key, sizeof(key), "%s.t%03d.y", tag, k);
            emit(key, y);
            snprintf(key, sizeof(key), "%s.t%03d.u", tag, k);
            emit(key, u);
        }
    }
    snprintf(key, sizeof(key), "%s.finalY", tag);
    emit(key, y);
    snprintf(key, sizeof(key), "%s.finalU", tag);
    emit(key, u);
    snprintf(key, sizeof(key), "%s.integrator", tag);
    emit(key, PID_GetIntegrator(&pid));
}

/* ======================================================================== */
/* 2. Relay auto-tune                                                        */
/* ======================================================================== */

static void scenario_relay(const char *tag)
{
    const double dt = 0.05;
    const double sp = 100.0;
    Fopdt plant;
    PID_Handle pid;
    PID_AutoTune t;
    PID_AutoTuneConfig cfg;
    PID_AutoTuneResult r;
    double y = 0.0;
    double u;
    int k;
    char key[128];

    fopdt_init(&plant, 2.0, 45.0, 12.0, dt);
    pid = mk_pid(0.0, 0.0, 0.0, dt);

    (void)PID_AutoTune_ConfigDefault(&cfg, PID_IDENT_RELAY);
    cfg.output_step = 20.0;
    cfg.hysteresis = 0.5;
    cfg.bias = 50.0;
    cfg.auto_bias = false;
    cfg.output_min = 0.0;
    cfg.output_max = 100.0;
    cfg.timeout_s = 600.0;
    cfg.skip_stabilize = true;

    if (PID_AutoTune_Init(&t, &cfg) != PID_OK) {
        printf("# relay init failed\n");
        return;
    }
    (void)PID_AutoTune_Start(&t, &pid, sp);

    for (k = 0; k < 40000; ++k) {
        u = PID_AutoTune_Update(&t, y, dt);
        y = fopdt_step(&plant, u, dt);
        if (!PID_AutoTune_IsRunning(&t)) {
            break;
        }
    }

    snprintf(key, sizeof(key), "%s.state", tag);
    emit_i(key, (long)PID_AutoTune_GetState(&t));
    snprintf(key, sizeof(key), "%s.progress", tag);
    emit_i(key, (long)PID_AutoTune_GetProgress(&t));

    if (PID_AutoTune_GetResult(&t, &r) == PID_OK) {
        snprintf(key, sizeof(key), "%s.modelKind", tag);
        emit_i(key, (long)r.model.kind);
        snprintf(key, sizeof(key), "%s.ku", tag);
        emit(key, r.model.ku);
        snprintf(key, sizeof(key), "%s.pu", tag);
        emit(key, r.model.pu);
        snprintf(key, sizeof(key), "%s.quality", tag);
        emit_i(key, (long)r.model.quality);
        snprintf(key, sizeof(key), "%s.amplitude", tag);
        emit(key, r.amplitude);
        snprintf(key, sizeof(key), "%s.periodSpread", tag);
        emit(key, r.period_spread);
        snprintf(key, sizeof(key), "%s.ampSpread", tag);
        emit(key, r.amp_spread);
        snprintf(key, sizeof(key), "%s.asymmetry", tag);
        emit(key, r.asymmetry);
        snprintf(key, sizeof(key), "%s.cyclesUsed", tag);
        emit_i(key, (long)r.cycles_used);
        snprintf(key, sizeof(key), "%s.kp", tag);
        emit(key, r.gains.kp);
        snprintf(key, sizeof(key), "%s.ki", tag);
        emit(key, r.gains.ki);
        snprintf(key, sizeof(key), "%s.kd", tag);
        emit(key, r.gains.kd);
        snprintf(key, sizeof(key), "%s.ti", tag);
        emit(key, r.gains.ti);
        snprintf(key, sizeof(key), "%s.td", tag);
        emit(key, r.gains.td);
        snprintf(key, sizeof(key), "%s.tf", tag);
        emit(key, r.gains.tf);
    } else {
        snprintf(key, sizeof(key), "%s.error", tag);
        emit_i(key, (long)PID_AutoTune_GetError(&t));
    }
}

/* ======================================================================== */
/* 3. Step auto-tune                                                         */
/* ======================================================================== */

static void scenario_step_tune(const char *tag)
{
    const double dt = 0.05;
    Fopdt plant;
    PID_Handle pid;
    PID_AutoTune t;
    PID_AutoTuneConfig cfg;
    PID_AutoTuneResult r;
    double y = 0.0;
    double u;
    int k;
    char key[128];

    fopdt_init(&plant, 2.0, 45.0, 12.0, dt);
    pid = mk_pid(0.0, 0.0, 0.0, dt);

    (void)PID_AutoTune_ConfigDefault(&cfg, PID_IDENT_STEP);
    cfg.output_step = 30.0;
    cfg.bias = 20.0;
    cfg.auto_bias = false;
    cfg.output_min = 0.0;
    cfg.output_max = 100.0;
    cfg.timeout_s = 900.0;
    cfg.skip_stabilize = true;

    if (PID_AutoTune_Init(&t, &cfg) != PID_OK) {
        printf("# step-tune init failed\n");
        return;
    }
    (void)PID_AutoTune_Start(&t, &pid, 100.0);

    for (k = 0; k < 60000; ++k) {
        u = PID_AutoTune_Update(&t, y, dt);
        y = fopdt_step(&plant, u, dt);
        if (!PID_AutoTune_IsRunning(&t)) {
            break;
        }
    }

    snprintf(key, sizeof(key), "%s.state", tag);
    emit_i(key, (long)PID_AutoTune_GetState(&t));

    if (PID_AutoTune_GetResult(&t, &r) == PID_OK) {
        snprintf(key, sizeof(key), "%s.modelKind", tag);
        emit_i(key, (long)r.model.kind);
        snprintf(key, sizeof(key), "%s.k", tag);
        emit(key, r.model.k);
        snprintf(key, sizeof(key), "%s.t", tag);
        emit(key, r.model.t);
        snprintf(key, sizeof(key), "%s.l", tag);
        emit(key, r.model.l);
        snprintf(key, sizeof(key), "%s.quality", tag);
        emit_i(key, (long)r.model.quality);
        snprintf(key, sizeof(key), "%s.kp", tag);
        emit(key, r.gains.kp);
        snprintf(key, sizeof(key), "%s.ki", tag);
        emit(key, r.gains.ki);
        snprintf(key, sizeof(key), "%s.kd", tag);
        emit(key, r.gains.kd);
        snprintf(key, sizeof(key), "%s.tf", tag);
        emit(key, r.gains.tf);
    } else {
        snprintf(key, sizeof(key), "%s.error", tag);
        emit_i(key, (long)PID_AutoTune_GetError(&t));
    }
}

/* ======================================================================== */
/* 4. A relay tune that must FAIL, and fail for the right reason              */
/* ======================================================================== */
/*
 * A test that only checks the happy path proves nothing about the guards. On
 * a plant with no dead time and a very small dt the limit cycle period falls
 * below the 20-samples-per-period floor and the tune must be rejected with
 * ERR_TUNE_VALIDATION rather than returning gains nobody should flash.
 */
static void scenario_relay_reject(const char *tag)
{
    const double dt = 0.05;
    Fopdt plant;
    PID_Handle pid;
    PID_AutoTune t;
    PID_AutoTuneConfig cfg;
    PID_AutoTuneResult r;
    double y = 0.0;
    double u;
    int k;
    char key[128];

    /* tau = 0.4 s, no dead time: Pu comes out near 2 s, which is 40 samples -
     * that would pass. So make the loop far faster than the sample time by
     * using a tiny time constant instead. */
    fopdt_init(&plant, 2.0, 0.05, 0.0, dt);
    pid = mk_pid(0.0, 0.0, 0.0, dt);

    (void)PID_AutoTune_ConfigDefault(&cfg, PID_IDENT_RELAY);
    cfg.output_step = 20.0;
    cfg.hysteresis = 0.0;
    cfg.bias = 50.0;
    cfg.auto_bias = false;
    cfg.output_min = 0.0;
    cfg.output_max = 100.0;
    cfg.timeout_s = 60.0;
    cfg.skip_stabilize = true;

    (void)PID_AutoTune_Init(&t, &cfg);
    (void)PID_AutoTune_Start(&t, &pid, 100.0);

    for (k = 0; k < 40000; ++k) {
        u = PID_AutoTune_Update(&t, y, dt);
        y = fopdt_step(&plant, u, dt);
        if (!PID_AutoTune_IsRunning(&t)) {
            break;
        }
    }

    snprintf(key, sizeof(key), "%s.state", tag);
    emit_i(key, (long)PID_AutoTune_GetState(&t));
    snprintf(key, sizeof(key), "%s.error", tag);
    emit_i(key, (long)PID_AutoTune_GetError(&t));
    if (PID_AutoTune_GetResult(&t, &r) != PID_OK) {
        snprintf(key, sizeof(key), "%s.resultCode", tag);
        emit_i(key, (long)r.code);
    }
}

/* ======================================================================== */
/* 5. Model/rule mismatch is rejected, never fudged                          */
/* ======================================================================== */

static void scenario_mismatch(void)
{
    PID_PlantModel m;
    PID_Gains g;
    char key[128];

    memset(&m, 0, sizeof(m));
    m.kind = PID_MODEL_FREQ;
    m.ku = 2.0;
    m.pu = 4.0;

    snprintf(key, sizeof(key), "mismatch.imcFromFreq");
    emit_i(key, (long)PID_TuneRule_Apply(PID_RULE_IMC, &m, PID_STRUCT_PID,
                                         0.0, &g));

    memset(&m, 0, sizeof(m));
    m.kind = PID_MODEL_FOPDT;
    m.k = 2.0;
    m.t = 45.0;
    m.l = 12.0;

    snprintf(key, sizeof(key), "mismatch.znFromFopdt");
    emit_i(key, (long)PID_TuneRule_Apply(PID_RULE_ZN, &m, PID_STRUCT_PID,
                                         0.0, &g));

    snprintf(key, sizeof(key), "mismatch.requiredModelIMC");
    emit_i(key, (long)PID_TuneRule_RequiredModel(PID_RULE_IMC));
    snprintf(key, sizeof(key), "mismatch.requiredModelZN");
    emit_i(key, (long)PID_TuneRule_RequiredModel(PID_RULE_ZN));
}

/* ======================================================================== */
/* 6. Cascade: two levels, inner saturating                                  */
/* ======================================================================== */
/*
 * The point of the cascade test is the backward pass. The inner loop is
 * deliberately given an actuator it cannot escape from, so the outer
 * integrator has to be corrected; if the correction is missing or one sample
 * late, the outer integrator runs away and the final value shows it.
 */
static void scenario_cascade(const char *tag)
{
    const double dt = 0.001;
    const int    n  = 6000;
    PID_Handle outer;
    PID_Handle inner;
    PID_Handle *loops[2];
    PID_Cascade c;
    PID_Config cfg;
    double meas[2];
    double yInner = 0.0;
    double yOuter = 0.0;
    double u = 0.0;
    double sp;
    char key[128];
    int k;

    (void)PID_ConfigDefault(&cfg);
    cfg.core.kp = 0.5;
    cfg.core.ki = 2.0;
    cfg.core.kd = 0.0;
    cfg.core.sample_time = 0.01;      /* outer: 10x slower */
    (void)PID_Init(&outer, &cfg);
    (void)PID_SetOutputLimits(&outer, -50.0, 50.0);

    (void)PID_ConfigDefault(&cfg);
    cfg.core.kp = 1.0;
    cfg.core.ki = 20.0;
    cfg.core.kd = 0.0;
    cfg.core.sample_time = 0.001;
    (void)PID_Init(&inner, &cfg);
    (void)PID_SetOutputLimits(&inner, -10.0, 10.0);

    loops[0] = &outer;
    loops[1] = &inner;
    if (PID_Cascade_Init(&c, loops, 2) != PID_OK) {
        printf("# cascade init failed\n");
        return;
    }
    (void)PID_Cascade_ConfigLevel(&c, 0, 10, -20.0, 20.0);
    (void)PID_Cascade_ConfigLevel(&c, 1, 1, 0.0, 0.0);

    for (k = 0; k < n; ++k) {
        sp = (k >= 500) ? 30.0 : 0.0;   /* unreachable: the inner saturates */
        meas[0] = yOuter;
        meas[1] = yInner;

        u = PID_Cascade_Update(&c, meas, sp, dt);

        /* inner plant: fast first order, saturating actuator */
        yInner += (0.001 / 0.01) * (u - yInner);
        /* outer plant: slow first order driven by the inner variable */
        yOuter += (0.001 / 0.2) * (yInner - yOuter);
    }

    snprintf(key, sizeof(key), "%s.yOuter", tag);
    emit(key, yOuter);
    snprintf(key, sizeof(key), "%s.yInner", tag);
    emit(key, yInner);
    snprintf(key, sizeof(key), "%s.u", tag);
    emit(key, u);
    snprintf(key, sizeof(key), "%s.integratorOuter", tag);
    emit(key, PID_GetIntegrator(&outer));
    snprintf(key, sizeof(key), "%s.integratorInner", tag);
    emit(key, PID_GetIntegrator(&inner));
    snprintf(key, sizeof(key), "%s.saturated", tag);
    emit_i(key, PID_Cascade_IsSaturated(&c) ? 1L : 0L);

    {
        PID_Float min_ratio = 0.0;
        uint8_t worst = 0;
        snprintf(key, sizeof(key), "%s.validate", tag);
        emit_i(key, (long)PID_Cascade_Validate(&c, &min_ratio, &worst));
        snprintf(key, sizeof(key), "%s.minRatio", tag);
        emit(key, (double)min_ratio);
    }
}


/* ======================================================================== */
/* 7. Fixed-point Q15 controller                                             */
/* ======================================================================== */
/*
 * The measurement sequence is a fixed formula rather than a file, so both
 * sides generate it identically without an I/O step. It is quantised to Q15
 * before use, because that quantisation is part of what is being compared.
 */

static int16_t q_meas(int k)
{
    double t = (double)k * 0.001;
    double y = 0.4 * (1.0 - exp(-t / 0.05)) + 0.15 * sin((double)k * 0.07);
    double q = y * 32768.0;
    q = (q < 0.0) ? (q - 0.5) : (q + 0.5);
    q = (double)(int32_t)q;
    if (q > 32767.0)  { q = 32767.0; }
    if (q < -32768.0) { q = -32768.0; }
    return (int16_t)q;
}

static void scenario_q15(const char *tag)
{
    PIDq_Handle h;
    PIDq_Config cfg;
    char key[128];
    int k;
    int16_t u;

    (void)PIDq_ConfigDefault(&cfg);
    cfg.kp_q16 = PIDQ_F_TO_Q16(1.5);
    cfg.ki_q16 = PIDQ_F_TO_Q16(4.0);
    cfg.kd_q16 = PIDQ_F_TO_Q16(0.01);
    cfg.dt_us = 1000U;
    cfg.tf_us = 2000U;
    cfg.out_min_q15 = PIDQ_F_TO_Q15(-0.9);
    cfg.out_max_q15 = PIDQ_F_TO_Q15(0.9);
    cfg.i_min_q15 = PIDQ_F_TO_Q15(-0.9);
    cfg.i_max_q15 = PIDQ_F_TO_Q15(0.9);
    cfg.aw_mode = (uint8_t)PIDQ_AW_BACK_CALC;
    cfg.bc_shift = 4U;

    if (PIDq_Init(&h, &cfg) != PID_OK) {
        printf("# q15 init failed\n");
        return;
    }
    (void)PIDq_SetSetpoint(&h, PIDQ_F_TO_Q15(0.5));

    for (k = 0; k < 200; ++k) {
        if (k == 100) {
            (void)PIDq_SetSetpoint(&h, PIDQ_F_TO_Q15(-0.3));
        }
        u = PIDq_Update(&h, q_meas(k));
        if ((k % 10) == 0) {
            snprintf(key, sizeof(key), "%s.u%03d", tag, k);
            emit_i(key, (long)u);
        }
    }
    snprintf(key, sizeof(key), "%s.finalU", tag);
    emit_i(key, (long)u);
    snprintf(key, sizeof(key), "%s.integral", tag);
    emit_i(key, (long)PIDq_GetIntegral(&h));
    snprintf(key, sizeof(key), "%s.saturated", tag);
    emit_i(key, PIDq_IsSaturated(&h) ? 1L : 0L);
}

/*
 * Integral resolution death, demonstrated rather than asserted.
 *
 * One LSB of error, Ki = 0.5, dt = 1 ms: the increment is 1.5e-8 in real
 * units, which is 0.0005 of a Q15 LSB. A Q15 accumulator would round it to
 * zero and never move. The Q30 accumulator accumulates it correctly, and
 * after enough samples the output LSB finally moves. The number of samples
 * it takes is the quantity worth knowing before you choose a gain format.
 */
static void scenario_q15_resolution(const char *tag)
{
    PIDq_Handle h;
    PIDq_Config cfg;
    char key[128];
    int k;
    int first_move = -1;
    int16_t u = 0;
    int16_t prev = 0;

    (void)PIDq_ConfigDefault(&cfg);
    cfg.kp_q16 = 0;                     /* pure integrator: only I can move */
    cfg.ki_q16 = PIDQ_F_TO_Q16(0.5);
    cfg.kd_q16 = 0;
    cfg.dt_us = 1000U;
    cfg.out_min_q15 = PIDQ_F_TO_Q15(-0.9);
    cfg.out_max_q15 = PIDQ_F_TO_Q15(0.9);

    if (PIDq_Init(&h, &cfg) != PID_OK) {
        printf("# q15 resolution init failed\n");
        return;
    }
    (void)PIDq_SetSetpoint(&h, (int16_t)1);      /* one LSB of error */

    for (k = 0; k < 200000; ++k) {
        u = PIDq_Update(&h, (int16_t)0);
        if ((first_move < 0) && (u != prev)) {
            first_move = k;
        }
        prev = u;
    }
    snprintf(key, sizeof(key), "%s.firstMoveSample", tag);
    emit_i(key, (long)first_move);
    snprintf(key, sizeof(key), "%s.finalU", tag);
    emit_i(key, (long)u);
    snprintf(key, sizeof(key), "%s.integral", tag);
    emit_i(key, (long)PIDq_GetIntegral(&h));
}

/*
 * A tune that the format cannot represent. Ki*dt >= 2.0 means the loop would
 * integrate more than full scale in a single sample, which is always a
 * configuration error, and PIDq_Init must refuse it rather than saturate
 * every sample and look like it is working.
 */
static void scenario_q15_reject(const char *tag)
{
    PIDq_Handle h;
    PIDq_Config cfg;
    char key[128];

    (void)PIDq_ConfigDefault(&cfg);
    cfg.ki_q16 = (int32_t)2000000;      /* Ki ~ 30.5, dt = 1 ms */
    cfg.dt_us = 100000U;                /* 100 ms: Ki*dt ~ 3.05 > 2.0 */

    snprintf(key, sizeof(key), "%s.initRc", tag);
    emit_i(key, (long)PIDq_Init(&h, &cfg));

    /* Separation at or below the deadband leaves no window in which the
     * integrator can run at all; that is refused too. */
    (void)PIDq_ConfigDefault(&cfg);
    cfg.deadband_q15 = 100U;
    cfg.separation_q15 = 50U;
    snprintf(key, sizeof(key), "%s.separationRc", tag);
    emit_i(key, (long)PIDq_Init(&h, &cfg));

    /* Back-calculation with a zero shift would feed the whole saturation
     * error back in one sample and ring. */
    (void)PIDq_ConfigDefault(&cfg);
    cfg.aw_mode = (uint8_t)PIDQ_AW_BACK_CALC;
    cfg.bc_shift = 0U;
    snprintf(key, sizeof(key), "%s.bcShiftRc", tag);
    emit_i(key, (long)PIDq_Init(&h, &cfg));
}

/* Bumpless manual -> automatic transfer. */
static void scenario_q15_bumpless(const char *tag)
{
    PIDq_Handle h;
    PIDq_Config cfg;
    char key[128];
    int16_t u_manual;
    int16_t u_auto;

    (void)PIDq_ConfigDefault(&cfg);
    cfg.kp_q16 = PIDQ_F_TO_Q16(2.0);
    cfg.ki_q16 = PIDQ_F_TO_Q16(1.0);
    cfg.dt_us = 1000U;
    cfg.mode = (uint8_t)PIDQ_MODE_MANUAL;

    (void)PIDq_Init(&h, &cfg);
    (void)PIDq_SetSetpoint(&h, PIDQ_F_TO_Q15(0.5));
    (void)PIDq_SetManualOutput(&h, PIDQ_F_TO_Q15(0.25));

    u_manual = PIDq_Update(&h, PIDQ_F_TO_Q15(0.1));
    (void)PIDq_SetMode(&h, PIDQ_MODE_AUTOMATIC);
    u_auto = PIDq_Update(&h, PIDQ_F_TO_Q15(0.1));

    snprintf(key, sizeof(key), "%s.uManual", tag);
    emit_i(key, (long)u_manual);
    snprintf(key, sizeof(key), "%s.uAfterSwitch", tag);
    emit_i(key, (long)u_auto);
    snprintf(key, sizeof(key), "%s.bumpless", tag);
    emit_i(key, (u_manual == u_auto) ? 1L : 0L);
}

/* ======================================================================== */
/* main                                                                      */
/* ======================================================================== */

int main(void)
{
    printf("# PIDX reference values for the MATLAB simlab tests.\n");
    printf("# Generated by tools/matlab_ref/matlab_ref.c against the C library.\n");
    printf("# Library version: %s\n", PID_GetVersion());

    scenario_step("step");
    scenario_relay("relay");
    scenario_step_tune("steptune");
    scenario_relay_reject("relayreject");
    scenario_mismatch();
    scenario_cascade("cascade");
    scenario_q15("q15");
    scenario_q15_resolution("q15res");
    scenario_q15_reject("q15reject");
    scenario_q15_bumpless("q15bump");

    return 0;
}

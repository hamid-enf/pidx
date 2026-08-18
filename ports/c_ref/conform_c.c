/**
 * @file    conform_c.c
 * @brief   Reference conformance runner for the cross-language comparison.
 *
 * Reads the scenario file described in ports/SPEC_conformance.md, drives the
 * real PIDX core with it, and writes the CSV that every other port must
 * reproduce. This is the oracle: the C library defines the behaviour, this
 * program only transcribes it.
 *
 * Built with PIDX_USE_DOUBLE=1 so that the comparison against Python, Octave
 * and C# - all of which are natively double - measures algorithmic
 * differences instead of float rounding. See SPEC section 1.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pidx/pid.h"
#include "pidx/pid_gainsched.h"
#include "pidx/pid_autotune.h"

#define MAX_LINE   1024
#define MAX_TOK      64

/* ---------------------------------------------------------------------- */
/* Parsing helpers                                                         */
/* ---------------------------------------------------------------------- */

/**
 * strtod plus the spellings the scenario format allows for non-finite values.
 * strtod already accepts "nan"/"inf" on glibc, but not every libm does, and a
 * silent 0.0 here would turn a NaN-path test into a nonsense pass.
 */
static double parse_num(const char *s)
{
    if (strcmp(s, "nan") == 0)  { double z = 0.0; return z / z; }
    if (strcmp(s, "inf") == 0)  { double z = 0.0; return  1.0 / z; }
    if (strcmp(s, "-inf") == 0) { double z = 0.0; return -1.0 / z; }
    return strtod(s, NULL);
}

static PID_Float arg_f(char **tok, int ntok, int i)
{
    return (i < ntok) ? (PID_Float)parse_num(tok[i]) : PID_ZERO;
}

static int arg_i(char **tok, int ntok, int i)
{
    return (i < ntok) ? (int)strtol(tok[i], NULL, 10) : 0;
}

/* ---------------------------------------------------------------------- */
/* Output                                                                  */
/* ---------------------------------------------------------------------- */

/**
 * Print one double with full round-trip precision, spelling non-finite values
 * exactly as the other ports do. printf("%.17g") renders NaN as "nan" or
 * "-nan" depending on the sign bit, and the sign of a NaN is not meaningful
 * here - normalising it keeps the CSVs byte-comparable.
 */
static void put_num(double v)
{
    if (v != v) {
        fputs("nan", stdout);
    } else if (v > 1.7976931348623157e308) {
        fputs("inf", stdout);
    } else if (v < -1.7976931348623157e308) {
        fputs("-inf", stdout);
    } else {
        printf("%.17g", v);
    }
}

static const char *g_scenario = "none";
static int g_row = 0;

/** Emit one CSV row. Pass NaN for any diagnostic the caller cannot supply. */
static void emit(const char *cmd, int rc, double out, double sp, double err,
                 double p, double i, double d, double ff, double unsat,
                 unsigned flags, int last_err)
{
    printf("%s,%d,%s,%d,", g_scenario, g_row, cmd, rc);
    put_num(out);   putchar(',');
    put_num(sp);    putchar(',');
    put_num(err);   putchar(',');
    put_num(p);     putchar(',');
    put_num(i);     putchar(',');
    put_num(d);     putchar(',');
    put_num(ff);    putchar(',');
    put_num(unsat); putchar(',');
    printf("%u,%d\n", flags, last_err);
    g_row++;
}

/* ---------------------------------------------------------------------- */
/* State                                                                   */
/* ---------------------------------------------------------------------- */

static PID_Handle       g_h;
static PID_Config       g_cfg;
static PID_GainSchedule g_sched;
static PID_GainPoint    g_pts[PIDX_GAINSCHED_MAX_POINTS];
static int              g_have_sched;
static int              g_inited;

static double nan_val(void) { double z = 0.0; return z / z; }

/** Emit a row from the handle's diagnostic snapshot after an update call. */
static void emit_update(const char *cmd, PID_StatusCode rc, PID_Float out)
{
    PID_Status s;
    if (PID_GetStatus(&g_h, &s) == PID_OK) {
        emit(cmd, (int)rc, (double)out, (double)s.setpoint_shaped,
             (double)s.error, (double)s.p_term, (double)s.i_term,
             (double)s.d_term, (double)s.ff_term, (double)s.output_unsat,
             (unsigned)s.flags, (int)PID_PeekLastError(&g_h));
    } else {
        const double q = nan_val();
        emit(cmd, (int)rc, (double)out, q, q, q, q, q, q, q,
             0U, (int)PID_PeekLastError(&g_h));
    }
}

/* ---------------------------------------------------------------------- */
/* Command dispatch                                                        */
/* ---------------------------------------------------------------------- */

static void do_config(char **t, int n)
{
    const char *c = t[0];

    if (strcmp(c, "gains") == 0) {
        g_cfg.core.kp = arg_f(t, n, 1);
        g_cfg.core.ki = arg_f(t, n, 2);
        g_cfg.core.kd = arg_f(t, n, 3);
    } else if (strcmp(c, "dt") == 0) {
        g_cfg.core.sample_time = arg_f(t, n, 1);
    } else if (strcmp(c, "direction") == 0) {
        g_cfg.core.direction = (PID_Direction)arg_i(t, n, 1);
    } else if (strcmp(c, "mode") == 0) {
        g_cfg.core.mode = (PID_Mode)arg_i(t, n, 1);
    } else if (strcmp(c, "integration") == 0) {
        g_cfg.core.integration = (PID_IntegrationMethod)arg_i(t, n, 1);
    } else if (strcmp(c, "outlim") == 0) {
        g_cfg.limits.use_output_limits = true;
        g_cfg.limits.output_min = arg_f(t, n, 1);
        g_cfg.limits.output_max = arg_f(t, n, 2);
    } else if (strcmp(c, "intlim") == 0) {
        g_cfg.limits.use_integral_limits = true;
        g_cfg.limits.integral_min = arg_f(t, n, 1);
        g_cfg.limits.integral_max = arg_f(t, n, 2);
    } else if (strcmp(c, "dtlim") == 0) {
        g_cfg.limits.dt_min = arg_f(t, n, 1);
        g_cfg.limits.dt_max = arg_f(t, n, 2);
    } else if (strcmp(c, "aw") == 0) {
        g_cfg.integral.mode = (PID_AntiWindup)arg_i(t, n, 1);
        g_cfg.integral.kt = arg_f(t, n, 2);
    } else if (strcmp(c, "separation") == 0) {
        g_cfg.integral.separation_threshold = arg_f(t, n, 1);
    } else if (strcmp(c, "deadband") == 0) {
        g_cfg.integral.deadband = arg_f(t, n, 1);
    } else if (strcmp(c, "ienable") == 0) {
        g_cfg.integral.enabled = (arg_i(t, n, 1) != 0);
    } else if (strcmp(c, "dmode") == 0) {
        g_cfg.filter.derivative_mode = (PID_DerivativeMode)arg_i(t, n, 1);
    } else if (strcmp(c, "tf") == 0) {
        g_cfg.filter.tf = arg_f(t, n, 1);
    } else if (strcmp(c, "nfilter") == 0) {
        g_cfg.filter.n_filter = arg_f(t, n, 1);
    } else if (strcmp(c, "inlpf") == 0) {
        g_cfg.filter.input_lpf_tau = arg_f(t, n, 1);
    } else if (strcmp(c, "weights") == 0) {
        g_cfg.weight.beta = arg_f(t, n, 1);
        g_cfg.weight.gamma = arg_f(t, n, 2);
    } else if (strcmp(c, "ff") == 0) {
        g_cfg.feedforward.enabled = (arg_i(t, n, 1) != 0);
        g_cfg.feedforward.value = arg_f(t, n, 2);
        g_cfg.feedforward.gain = arg_f(t, n, 3);
    } else if (strcmp(c, "shaper") == 0) {
        g_cfg.shaper.sp_rate_max = arg_f(t, n, 1);
        g_cfg.shaper.sp_accel = arg_f(t, n, 2);
        g_cfg.shaper.sp_decel = arg_f(t, n, 3);
        g_cfg.shaper.out_slew_max = arg_f(t, n, 4);
    } else if (strcmp(c, "safety") == 0) {
        g_cfg.safety.enabled = (arg_i(t, n, 1) != 0);
        g_cfg.safety.meas_min = arg_f(t, n, 2);
        g_cfg.safety.meas_max = arg_f(t, n, 3);
        g_cfg.safety.meas_rate_max = arg_f(t, n, 4);
        g_cfg.safety.failsafe_output = arg_f(t, n, 5);
        g_cfg.safety.fault_persist_n = (uint8_t)arg_i(t, n, 6);
        g_cfg.safety.auto_recover = (arg_i(t, n, 7) != 0);
    } else {
        fprintf(stderr, "unknown config cmd: %s\n", c);
        exit(2);
    }
}

static void do_run(char **t, int n)
{
    const char *c = t[0];
    const double q = nan_val();

    if (strcmp(c, "u") == 0) {
        const PID_Float m = arg_f(t, n, 1);
        const PID_Float dt = arg_f(t, n, 2);
        const PID_Float out = PID_UpdateDt(&g_h, m, dt);
        emit_update("u", PID_PeekLastError(&g_h), out);
    } else if (strcmp(c, "un") == 0) {
        const PID_Float out = PID_Update(&g_h, arg_f(t, n, 1));
        emit_update("un", PID_PeekLastError(&g_h), out);
    } else if (strcmp(c, "ufast") == 0) {
        /* The fast path does not fill the snapshot by design, so only the
         * fields it genuinely produces are reported. */
        const PID_Float out = PID_UpdateFast(&g_h, arg_f(t, n, 1));
        emit("ufast", (int)PID_PeekLastError(&g_h), (double)out,
             (double)PID_GetSetpoint(&g_h), q, q,
             (double)PID_GetIntegrator(&g_h), q, q, q,
             0U, (int)PID_PeekLastError(&g_h));
    } else if (strcmp(c, "uex") == 0) {
        PID_Input in;
        PID_StatusCode rc = PID_OK;
        PID_Float out;
        PID_InputInit(&in);
        in.measurement  = arg_f(t, n, 1);
        in.dt           = arg_f(t, n, 2);
        in.setpoint     = arg_f(t, n, 3);
        in.feedforward  = arg_f(t, n, 4);
        in.tracking     = arg_f(t, n, 5);
        in.schedule_var = arg_f(t, n, 6);
        out = PID_UpdateEx(&g_h, &in, &rc);
        emit_update("uex", rc, out);
    } else if (strcmp(c, "sp") == 0) {
        (void)PID_SetSetpoint(&g_h, arg_f(t, n, 1));
    } else if (strcmp(c, "spimm") == 0) {
        PID_SetSetpointImmediate(&g_h, arg_f(t, n, 1));
    } else if (strcmp(c, "setmode") == 0) {
        (void)PID_SetMode(&g_h, (PID_Mode)arg_i(t, n, 1));
    } else if (strcmp(c, "manual") == 0) {
        (void)PID_SetManualOutput(&g_h, arg_f(t, n, 1));
    } else if (strcmp(c, "setgains") == 0) {
        (void)PID_SetGains(&g_h, arg_f(t, n, 1), arg_f(t, n, 2), arg_f(t, n, 3));
    } else if (strcmp(c, "rescale") == 0) {
        (void)PID_SetGainsRescaleIntegral(&g_h, arg_f(t, n, 1),
                                          arg_f(t, n, 2), arg_f(t, n, 3));
    } else if (strcmp(c, "setaw") == 0) {
        (void)PID_SetAntiWindup(&g_h, (PID_AntiWindup)arg_i(t, n, 1),
                                arg_f(t, n, 2));
    } else if (strcmp(c, "setoutlim") == 0) {
        (void)PID_SetOutputLimits(&g_h, arg_f(t, n, 1), arg_f(t, n, 2));
    } else if (strcmp(c, "clroutlim") == 0) {
        (void)PID_ClearOutputLimits(&g_h);
    } else if (strcmp(c, "setintlim") == 0) {
        (void)PID_SetIntegralLimits(&g_h, arg_f(t, n, 1), arg_f(t, n, 2));
    } else if (strcmp(c, "setint") == 0) {
        (void)PID_SetIntegrator(&g_h, arg_f(t, n, 1));
    } else if (strcmp(c, "track") == 0) {
        (void)PID_SetTrackingInput(&g_h, arg_f(t, n, 1));
    } else if (strcmp(c, "setdmode") == 0) {
        (void)PID_SetDerivativeMode(&g_h, (PID_DerivativeMode)arg_i(t, n, 1));
    } else if (strcmp(c, "settf") == 0) {
        (void)PID_SetDerivativeFilter(&g_h, arg_f(t, n, 1));
    } else if (strcmp(c, "setn") == 0) {
        (void)PID_SetDerivativeFilterN(&g_h, arg_f(t, n, 1));
    } else if (strcmp(c, "setdir") == 0) {
        (void)PID_SetDirection(&g_h, (PID_Direction)arg_i(t, n, 1));
    } else if (strcmp(c, "setweights") == 0) {
        (void)PID_SetWeights(&g_h, arg_f(t, n, 1), arg_f(t, n, 2));
    } else if (strcmp(c, "setff") == 0) {
        (void)PID_SetFeedforward(&g_h, arg_f(t, n, 1));
    } else if (strcmp(c, "setramp") == 0) {
        (void)PID_SetSetpointRamp(&g_h, arg_f(t, n, 1), arg_f(t, n, 2),
                                  arg_f(t, n, 3));
    } else if (strcmp(c, "setslew") == 0) {
        (void)PID_SetOutputSlewRate(&g_h, arg_f(t, n, 1));
    } else if (strcmp(c, "setinlpf") == 0) {
        (void)PID_SetInputFilter(&g_h, arg_f(t, n, 1));
    } else if (strcmp(c, "setsep") == 0) {
        (void)PID_SetIntegralSeparation(&g_h, arg_f(t, n, 1));
    } else if (strcmp(c, "setdb") == 0) {
        (void)PID_SetIntegralDeadband(&g_h, arg_f(t, n, 1));
    } else if (strcmp(c, "setienable") == 0) {
        (void)PID_EnableIntegral(&g_h, arg_i(t, n, 1) != 0);
    } else if (strcmp(c, "setdtnom") == 0) {
        (void)PID_SetSampleTime(&g_h, arg_f(t, n, 1));
    } else if (strcmp(c, "reset") == 0) {
        (void)PID_Reset(&g_h);
    } else if (strcmp(c, "clearfault") == 0) {
        (void)PID_ClearFault(&g_h);
    } else if (strcmp(c, "schedpoints") == 0) {
        const int cnt = arg_i(t, n, 1);
        int i;
        for (i = 0; i < cnt; ++i) {
            g_pts[i].x  = arg_f(t, n, 2 + (i * 4));
            g_pts[i].kp = arg_f(t, n, 3 + (i * 4));
            g_pts[i].ki = arg_f(t, n, 4 + (i * 4));
            g_pts[i].kd = arg_f(t, n, 5 + (i * 4));
        }
        g_have_sched = cnt;
    } else if (strcmp(c, "schedcfg") == 0) {
        if (g_have_sched > 0) {
            (void)PID_GainSched_Init(&g_sched, g_pts, (uint8_t)g_have_sched,
                                     (PID_SchedSource)arg_i(t, n, 1),
                                     (PID_SchedInterp)arg_i(t, n, 2));
            (void)PID_GainSched_SetHysteresis(&g_sched, arg_f(t, n, 3));
            (void)PID_GainSched_Attach(&g_h, &g_sched);
        }
    } else if (strcmp(c, "schedvar") == 0) {
        (void)PID_GainSched_SetVar(&g_h, arg_f(t, n, 1));
    } else if (strcmp(c, "rule") == 0) {
        PID_PlantModel m;
        PID_Gains g;
        PID_StatusCode rc;
        memset(&m, 0, sizeof(m));
        memset(&g, 0, sizeof(g));
        m.kind = (PID_ModelKind)arg_i(t, n, 3);
        if (m.kind == PID_MODEL_FREQ) {
            m.ku = arg_f(t, n, 4);
            m.pu = arg_f(t, n, 5);
        } else {
            m.k = arg_f(t, n, 4);
            m.t = arg_f(t, n, 5);
            m.l = arg_f(t, n, 6);
        }
        rc = PID_TuneRule_Apply((PID_TuneRule)arg_i(t, n, 1), &m,
                                (PID_TuneStructure)arg_i(t, n, 2),
                                arg_f(t, n, 7), &g);
        emit("rule", (int)rc, (double)g.kp, (double)g.ki, (double)g.kd,
             (double)g.ti, (double)g.td, (double)g.tf, q, q, 0U, 0);
    } else {
        fprintf(stderr, "unknown run cmd: %s\n", c);
        exit(2);
    }
}

/* ---------------------------------------------------------------------- */
/* Main                                                                    */
/* ---------------------------------------------------------------------- */

int main(int argc, char **argv)
{
    FILE *f;
    char line[MAX_LINE];

    if (argc < 2) {
        fprintf(stderr, "usage: %s <scenario-file>\n", argv[0]);
        return 1;
    }
    f = fopen(argv[1], "r");
    if (f == NULL) {
        fprintf(stderr, "cannot open %s\n", argv[1]);
        return 1;
    }

    printf("scenario,k,cmd,rc,output,setpoint,error,p,i,d,ff,unsat,"
           "flags,last_error\n");

    while (fgets(line, (int)sizeof(line), f) != NULL) {
        char *tok[MAX_TOK];
        int ntok = 0;
        char *p = strtok(line, " \t\r\n");

        while ((p != NULL) && (ntok < MAX_TOK)) {
            tok[ntok++] = p;
            p = strtok(NULL, " \t\r\n");
        }
        if ((ntok == 0) || (tok[0][0] == '#')) {
            continue;
        }

        if (strcmp(tok[0], "scenario") == 0) {
            static char namebuf[128];
            (void)PID_ConfigDefault(&g_cfg);
            memset(&g_h, 0, sizeof(g_h));
            g_have_sched = 0;
            g_inited = 0;
            g_row = 0;
            strncpy(namebuf, (ntok > 1) ? tok[1] : "?", sizeof(namebuf) - 1U);
            namebuf[sizeof(namebuf) - 1U] = '\0';
            g_scenario = namebuf;
        } else if (strcmp(tok[0], "init") == 0) {
            const PID_StatusCode rc = PID_Init(&g_h, &g_cfg);
            const double q = nan_val();
            g_inited = (rc == PID_OK);
            emit("init", (int)rc, 0.0, q, q, q, q, q, q, q, 0U, 0);
        } else if (strcmp(tok[0], "end") == 0) {
            g_inited = 0;
        } else if (g_inited == 0) {
            do_config(tok, ntok);
        } else {
            do_run(tok, ntok);
        }
    }

    fclose(f);
    return 0;
}

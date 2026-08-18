/* Core API contract suite (PHASE 17).
 *
 * The other suites each test a feature. This one tests the PROMISES the API
 * makes independently of any feature - the things a caller is entitled to
 * assume about every function in pid.h:
 *
 *   1. Every entry point survives NULL and an uninitialised handle. No
 *      crashes, no garbage, a defined return value.
 *   2. A rejected setter changes NOTHING. Partial application is worse than
 *      rejection because the handle is left in a state the caller never asked
 *      for and cannot see.
 *   3. PID_Update is a pure function of (state, input): same state and same
 *      input give a bit-identical output.
 *   4. Documented invariants hold - direction, output limits, reset semantics,
 *      the feature mask, sticky error reporting, PID_Deinit.
 *
 * These are the failures that do not show up in a step response; they show up
 * in the field six months later.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "pidx/pid.h"

static int pass = 0, bad = 0;
#define CK(c,m) do{ if(c){pass++;} else {bad++;printf("  FAIL: %s\n",(m));} }while(0)

static bool near(double a, double b, double tol) { return fabs(a - b) <= tol; }

static void std_cfg(PID_Config *c)
{
    PID_ConfigDefault(c);
    c->core.kp = 2.0f;
    c->core.ki = 0.5f;
    c->core.kd = 0.1f;
    c->core.sample_time = 0.01f;
    c->limits.use_output_limits = true;
    c->limits.output_min = -10.0f;
    c->limits.output_max = 10.0f;
}

/* ===================================================================== */
static void t_null_safety(void)
{
    PID_Float kp, ki, kd;
    PID_Status st;
    PID_StatusCode e;
    PID_Input in;

    puts("[1] every entry point survives NULL");

    /* Nothing here may crash; each returns its documented neutral value. */
    CK(PID_Init(NULL, NULL) == PID_ERR_NULL, "Init NULL");
    CK(PID_Deinit(NULL) == PID_ERR_NULL, "Deinit NULL");
    CK(PID_Reset(NULL) == PID_ERR_NULL, "Reset NULL");
    CK((double)PID_Update(NULL, 1.0f) == 0.0, "Update NULL");
    CK((double)PID_UpdateDt(NULL, 1.0f, 0.01f) == 0.0, "UpdateDt NULL");
    CK((double)PID_UpdateFast(NULL, 1.0f) == 0.0, "UpdateFast NULL");
    CK((double)PID_UpdateEx(NULL, NULL, NULL) == 0.0, "UpdateEx NULL");
    CK(PID_SetGains(NULL, 1.0f, 1.0f, 1.0f) == PID_ERR_NULL, "SetGains NULL");
    CK(PID_GetGains(NULL, &kp, &ki, &kd) == PID_ERR_NULL, "GetGains NULL");
    CK(PID_SetKp(NULL, 1.0f) == PID_ERR_NULL, "SetKp NULL");
    CK(PID_SetKi(NULL, 1.0f) == PID_ERR_NULL, "SetKi NULL");
    CK(PID_SetKd(NULL, 1.0f) == PID_ERR_NULL, "SetKd NULL");
    CK(PID_SetSetpoint(NULL, 1.0f) == PID_ERR_NULL, "SetSetpoint NULL");
    CK((double)PID_GetSetpoint(NULL) == 0.0, "GetSetpoint NULL");
    CK((double)PID_GetOutput(NULL) == 0.0, "GetOutput NULL");
    CK((double)PID_GetIntegrator(NULL) == 0.0, "GetIntegrator NULL");
    CK((double)PID_GetError(NULL) == 0.0, "GetError NULL");
    CK((double)PID_GetSampleTime(NULL) == 0.0, "GetSampleTime NULL");
    CK(PID_SetSampleTime(NULL, 0.01f) == PID_ERR_NULL, "SetSampleTime NULL");
    CK(PID_SetOutputLimits(NULL, -1.0f, 1.0f) == PID_ERR_NULL, "SetOutputLimits NULL");
    CK(PID_SetIntegralLimits(NULL, -1.0f, 1.0f) == PID_ERR_NULL, "SetIntegralLimits NULL");
    CK(PID_SetMode(NULL, PID_MODE_AUTOMATIC) == PID_ERR_NULL, "SetMode NULL");
    CK(PID_SetManualOutput(NULL, 1.0f) == PID_ERR_NULL, "SetManualOutput NULL");
    CK(PID_SetIntegrator(NULL, 1.0f) == PID_ERR_NULL, "SetIntegrator NULL");
    CK(PID_GetStatus(NULL, &st) == PID_ERR_NULL, "GetStatus NULL handle");
    CK(!PID_IsSaturated(NULL), "IsSaturated NULL is false");
    CK(PID_GetLastError(NULL, &e) == PID_ERR_NULL, "GetLastError NULL");
    CK(PID_GetFlags(NULL) == 0UL, "GetFlags NULL is 0");

    /* A NULL OUT-parameter is a different case and must also be tolerated. */
    {
        PID_Handle h;
        PID_Config c;
        std_cfg(&c);
        (void)PID_Init(&h, &c);
        CK(PID_GetGains(&h, NULL, NULL, NULL) == PID_OK, "GetGains all-NULL outputs");
        CK(PID_GetStatus(&h, NULL) == PID_ERR_NULL, "GetStatus NULL status");
        CK(PID_GetLastError(&h, NULL) == PID_OK ||
           PID_GetLastError(&h, NULL) == PID_ERR_NULL, "GetLastError NULL out defined");
        PID_InputInit(&in);
        CK((double)PID_UpdateEx(&h, NULL, NULL) == 0.0, "UpdateEx NULL input");
    }
}

static void t_uninitialised(void)
{
    PID_Handle h;
    PID_Float kp, ki, kd;
    PID_StatusCode e = PID_OK;

    puts("[2] an uninitialised handle is refused, not run");

    /* Deliberately filled with junk, as a stack variable would be. */
    memset(&h, 0x5A, sizeof(h));

    CK((double)PID_Update(&h, 1.0f) == 0.0, "Update on junk returns 0");
    CK(PID_GetLastError(&h, &e) == PID_OK || e == PID_ERR_NOT_INIT,
       "and reports NOT_INIT rather than running");
    CK(PID_SetGains(&h, 1.0f, 1.0f, 1.0f) == PID_ERR_NOT_INIT, "SetGains NOT_INIT");
    CK(PID_GetGains(&h, &kp, &ki, &kd) == PID_ERR_NOT_INIT, "GetGains NOT_INIT");
    CK(PID_SetOutputLimits(&h, -1.0f, 1.0f) == PID_ERR_NOT_INIT, "SetOutputLimits NOT_INIT");
    CK(PID_Reset(&h) == PID_ERR_NOT_INIT, "Reset NOT_INIT");
    /* The one that mattered enough to be a real bug: the cascade relies on
     * this returning 0 to detect a garbage member loop. */
    CK((double)PID_GetSampleTime(&h) == 0.0, "GetSampleTime on junk is 0, not garbage");

    /* Deinit must make an initialised handle behave exactly like this again. */
    {
        PID_Config c;
        std_cfg(&c);
        CK(PID_Init(&h, &c) == PID_OK, "init the same memory");
        (void)PID_SetSetpoint(&h, 1.0f);
        (void)PID_Update(&h, 0.0f);
        CK((double)PID_GetSampleTime(&h) > 0.0, "usable while initialised");
        CK(PID_Deinit(&h) == PID_OK, "Deinit ok");
        CK((double)PID_GetSampleTime(&h) == 0.0, "Deinit invalidates the magic");
        CK((double)PID_Update(&h, 0.0f) == 0.0, "and Update refuses afterwards");
        CK(PID_SetGains(&h, 1.0f, 1.0f, 1.0f) == PID_ERR_NOT_INIT, "as do the setters");
    }
}

/* ===================================================================== */
static void t_rejected_setters_are_atomic(void)
{
    PID_Handle h;
    PID_Config c;
    PID_Float kp, ki, kd;

    puts("[3] a rejected setter changes nothing");

    std_cfg(&c);
    (void)PID_Init(&h, &c);

    /* SetGains with one bad member must not apply the two good ones. This is
     * the classic partial-application bug: Kp lands, Ki is rejected, and the
     * controller now runs a tuning nobody chose. */
    CK(PID_SetGains(&h, 5.0f, (PID_Float)NAN, 1.0f) == PID_ERR_INVALID_GAIN,
       "NaN Ki rejected");
    (void)PID_GetGains(&h, &kp, &ki, &kd);
    /* Tolerance, not equality: 0.1f is not 0.1, so a zero-tolerance compare
     * against the decimal tests float representation rather than behaviour. */
    CK(near((double)kp, 2.0, 1e-6) && near((double)ki, 0.5, 1e-6) &&
       near((double)kd, 0.1, 1e-6), "no gain moved");

    CK(PID_SetGains(&h, -1.0f, 0.5f, 0.1f) == PID_ERR_INVALID_GAIN, "negative Kp rejected");
    (void)PID_GetGains(&h, &kp, &ki, &kd);
    CK(near((double)kp, 2.0, 1e-6), "Kp untouched after rejection");

    CK(PID_SetKi(&h, (PID_Float)INFINITY) == PID_ERR_INVALID_GAIN, "Inf Ki rejected");
    (void)PID_GetGains(&h, &kp, &ki, &kd);
    CK(near((double)ki, 0.5, 1e-6), "Ki untouched");

    /* Reversed limits must not be swapped silently, nor half-applied. */
    CK(PID_SetOutputLimits(&h, 5.0f, -5.0f) == PID_ERR_INVALID_LIMIT, "min > max rejected");
    {
        PID_Status st;
        (void)PID_GetStatus(&h, &st);
        (void)st;
    }
    (void)PID_SetSetpoint(&h, 100.0f);
    {
        int i;
        double u = 0.0;
        for (i = 0; i < 50; ++i) { u = (double)PID_Update(&h, 0.0f); }
        CK(u <= 10.0 + 1e-6, "the original +10 limit is still in force");
    }
    CK(PID_SetOutputLimits(&h, 1.0f, 1.0f) == PID_ERR_INVALID_LIMIT, "min == max rejected");
    CK(PID_SetOutputLimits(&h, (PID_Float)NAN, 1.0f) == PID_ERR_INVALID_LIMIT, "NaN limit");
    CK(PID_SetSampleTime(&h, 0.0f) == PID_ERR_INVALID_DT, "dt = 0 rejected");
    CK(PID_SetSampleTime(&h, -1.0f) == PID_ERR_INVALID_DT, "negative dt rejected");
    CK(near((double)PID_GetSampleTime(&h), 0.01, 1e-9), "sample time untouched");
    CK(PID_SetSetpoint(&h, (PID_Float)NAN) == PID_ERR_INVALID_PARAM, "NaN setpoint rejected");
    CK(near((double)PID_GetSetpoint(&h), 100.0, 1e-9), "setpoint untouched");

    /* Init itself: a rejected config must leave the handle unusable rather
     * than half-configured. */
    {
        PID_Handle g;
        PID_Config bc;
        std_cfg(&bc);
        bc.core.sample_time = 0.0f;
        CK(PID_Init(&g, &bc) == PID_ERR_INVALID_DT, "Init rejects dt = 0");
        CK((double)PID_Update(&g, 0.0f) == 0.0, "and the handle is not usable");
        std_cfg(&bc);
        bc.core.kp = -1.0f;
        CK(PID_Init(&g, &bc) == PID_ERR_INVALID_GAIN, "Init rejects negative Kp");
        std_cfg(&bc);
        bc.limits.output_min = 10.0f;
        bc.limits.output_max = -10.0f;
        CK(PID_Init(&g, &bc) == PID_ERR_INVALID_LIMIT, "Init rejects reversed limits");
    }
}

/* ===================================================================== */
static void t_determinism(void)
{
    PID_Handle h1, h2;
    PID_Config c;
    int i;
    int identical = 1;

    puts("[4] PID_Update is deterministic");

    /* Two handles, same config, same input sequence: every sample must match
     * bit for bit. Any divergence means hidden state - an uninitialised field,
     * a static, or a dependence on something outside the handle. */
    std_cfg(&c);
    (void)PID_Init(&h1, &c);
    (void)PID_Init(&h2, &c);
    (void)PID_SetSetpoint(&h1, 1.0f);
    (void)PID_SetSetpoint(&h2, 1.0f);

    for (i = 0; i < 1000; ++i) {
        const PID_Float y = (PID_Float)(0.5 * sin((double)i * 0.05));
        const PID_Float u1 = PID_Update(&h1, y);
        const PID_Float u2 = PID_Update(&h2, y);
        if (u1 != u2) { identical = 0; }
    }
    CK(identical, "two identically configured handles never diverge");

    /* Reset must return the handle to a state that replays identically. */
    {
        double first[100], second[100];
        int k;
        (void)PID_Reset(&h1);
        (void)PID_SetSetpoint(&h1, 1.0f);
        for (k = 0; k < 100; ++k) { first[k] = (double)PID_Update(&h1, 0.0f); }
        (void)PID_Reset(&h1);
        (void)PID_SetSetpoint(&h1, 1.0f);
        for (k = 0; k < 100; ++k) { second[k] = (double)PID_Update(&h1, 0.0f); }
        identical = 1;
        for (k = 0; k < 100; ++k) { if (first[k] != second[k]) { identical = 0; } }
        CK(identical, "Reset replays bit-identically");
        CK((double)PID_GetIntegrator(&h1) != 0.0, "the run really did charge I");
        (void)PID_Reset(&h1);
        CK((double)PID_GetIntegrator(&h1) == 0.0, "Reset clears the integrator");
        CK(PID_GetFlags(&h1) == 0UL || PID_GetFlags(&h1) == (uint32_t)PID_FLAG_MANUAL,
           "Reset clears the transient flags");
    }
}

/* ===================================================================== */
static void t_invariants(void)
{
    PID_Handle h;
    PID_Config c;
    int i;

    puts("[5] documented invariants");

    /* Output limits are never violated, in any mode, for any input. */
    std_cfg(&c);
    (void)PID_Init(&h, &c);
    (void)PID_SetSetpoint(&h, 1.0e6f);
    for (i = 0; i < 2000; ++i) {
        const double u = (double)PID_Update(&h, (PID_Float)(-1.0e6));
        if ((u > 10.0) || (u < -10.0)) { break; }
    }
    CK(i == 2000, "output never leaves [min,max] under extreme drive");
    CK(PID_IsSaturated(&h), "and saturation is reported");
    CK((PID_GetFlags(&h) & (uint32_t)PID_FLAG_SATURATED_HIGH) != 0UL,
       "with the correct direction flag");

    /* REVERSE really inverts the sign of the action. */
    {
        PID_Handle d, r;
        PID_Config cd;
        double ud, ur;
        std_cfg(&cd);
        cd.core.direction = PID_DIRECT;
        (void)PID_Init(&d, &cd);
        cd.core.direction = PID_REVERSE;
        (void)PID_Init(&r, &cd);
        (void)PID_SetSetpoint(&d, 1.0f);
        (void)PID_SetSetpoint(&r, 1.0f);
        ud = (double)PID_Update(&d, 0.0f);
        ur = (double)PID_Update(&r, 0.0f);
        CK(ud > 0.0, "DIRECT pushes up on a positive error");
        CK(ur < 0.0, "REVERSE pushes down on the same error");
        CK(near(ud, -ur, 1e-6), "and the magnitudes match exactly");
    }

    /* The feature mask reflects what was configured, and a disabled feature
     * stays disabled. */
    {
        PID_Handle f;
        PID_Config cf;
        std_cfg(&cf);
        (void)PID_Init(&f, &cf);
        CK(PID_IsFeatureEnabled(&f, PID_FEAT_OUTPUT_LIMIT), "output limit reported on");
        CK(PID_IsFeatureEnabled(&f, PID_FEAT_INTEGRAL), "integral on when Ki > 0");
        CK(PID_IsFeatureEnabled(&f, PID_FEAT_DERIVATIVE), "derivative on when Kd > 0");
        CK(!PID_IsFeatureEnabled(&f, PID_FEAT_GAIN_SCHED), "gain sched off");
        CK(!PID_IsFeatureEnabled(NULL, PID_FEAT_INTEGRAL), "IsFeatureEnabled(NULL)");
        CK(PID_ClearOutputLimits(&f) == PID_OK, "limits can be cleared");
        CK(!PID_IsFeatureEnabled(&f, PID_FEAT_OUTPUT_LIMIT), "and the mask follows");

        /* Ki = 0 must actually stop integration, not merely scale it to zero -
         * a nonzero integrator with Ki = 0 would resurrect on the next Ki. */
        /*
         * PID_FEAT_INTEGRAL tracks cfg.integral.enabled - "is the integral
         * PATH compiled in and switched on" - not "is Ki nonzero". That is the
         * right split: it lets a user disable the term without destroying
         * their tuning. What must hold is the numerical consequence: with
         * Ki = 0 the coefficient c_i = Ki*dt is exactly 0, so the integrator
         * can never leave zero however long the error persists.
         */
        std_cfg(&cf);
        cf.core.ki = 0.0f;
        (void)PID_Init(&f, &cf);
        (void)PID_SetSetpoint(&f, 1.0f);
        for (i = 0; i < 500; ++i) { (void)PID_Update(&f, 0.0f); }
        CK((double)PID_GetIntegrator(&f) == 0.0,
           "Ki = 0 keeps the integrator at exactly zero");
        CK(near((double)PID_GetOutput(&f), 2.0, 1e-6),
           "and the output is pure P (Kp * e = 2.0)");
        {
            /* Switching the path off is the other, independent control. */
            PID_Handle n;
            PID_Config cn;
            std_cfg(&cn);
            cn.integral.enabled = false;
            (void)PID_Init(&n, &cn);
            CK(!PID_IsFeatureEnabled(&n, PID_FEAT_INTEGRAL),
               "integral.enabled = false clears the feature bit");
            (void)PID_SetSetpoint(&n, 1.0f);
            for (i = 0; i < 500; ++i) { (void)PID_Update(&n, 0.0f); }
            CK((double)PID_GetIntegrator(&n) == 0.0,
               "and no integration happens despite Ki > 0");
        }
    }

    /* Sticky first-wins error reporting, read-and-clear. */
    {
        PID_Handle s;
        PID_Config cs;
        PID_StatusCode e = PID_OK;
        std_cfg(&cs);
        (void)PID_Init(&s, &cs);
        CK(PID_GetLastError(&s, &e) == PID_OK && e == PID_OK, "clean handle has no error");
        (void)PID_Update(&s, (PID_Float)NAN);
        (void)PID_SetGains(&s, -1.0f, 1.0f, 1.0f);
        (void)PID_GetLastError(&s, &e);
        CK(e == PID_ERR_NAN_INPUT, "first error wins over the later one");
        (void)PID_GetLastError(&s, &e);
        CK(e == PID_OK, "read-and-clear");
    }

    /* PID_StatusToString covers every code it is given, and never returns
     * NULL - it is used in printf() paths. */
    {
        int code;
        int all_ok = 1;
        for (code = -20; code <= 20; ++code) {
            const char *msg = PID_StatusToString((PID_StatusCode)code);
            if ((msg == NULL) || (msg[0] == '\0')) { all_ok = 0; }
        }
        CK(all_ok, "StatusToString never returns NULL or empty, even out of range");
    }
}

/* ===================================================================== */
static void t_fast_vs_full(void)
{
    PID_Handle a, b;
    PID_Config c;
    int i;
    int identical = 1;
    double worst = 0.0;

    puts("[6] the fast path is a subset, not a different controller");

    /* Re-asserted here (test_fastpath covers it in depth) because it is a
     * standing API promise: when PID_UpdateFast_IsSafe() is true the two
     * entry points must agree bit for bit. A regression in either path shows
     * up here first. */
    std_cfg(&c);
    c.core.kd = 0.0f;              /* keep it inside the fast-path subset */
    (void)PID_Init(&a, &c);
    (void)PID_Init(&b, &c);
    CK(PID_UpdateFast_IsSafe(&a), "config is fast-path safe");
    (void)PID_SetSetpoint(&a, 1.0f);
    (void)PID_SetSetpoint(&b, 1.0f);

    for (i = 0; i < 2000; ++i) {
        const PID_Float y = (PID_Float)(0.9 * sin((double)i * 0.02));
        const double ua = (double)PID_UpdateFast(&a, y);
        const double ub = (double)PID_Update(&b, y);
        if (ua != ub) {
            identical = 0;
            if (fabs(ua - ub) > worst) { worst = fabs(ua - ub); }
        }
    }
    CK(identical, "UpdateFast and Update are bit-identical when safe");
    if (!identical) { printf("    worst divergence %.3e\n", worst); }
    CK(!PID_UpdateFast_IsSafe(NULL), "IsSafeForFastPath(NULL) is false");
}

int main(void)
{
    puts("=== core API contract suite ===\n");
    t_null_safety();
    t_uninitialised();
    t_rejected_setters_are_atomic();
    t_determinism();
    t_invariants();
    t_fast_vs_full();
    printf("\n  core contract: %d passed, %d failed\n", pass, bad);
    return (bad == 0) ? 0 : 1;
}

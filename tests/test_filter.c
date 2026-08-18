/* Standalone filter suite (PHASE 17).
 *
 * pid_filter.c is the one module a user is most likely to reach for outside
 * the controller, and it had no dedicated suite: it was only exercised
 * indirectly through PID_Update's input filter. These tests pin the analytic
 * behaviour of each filter against closed-form values rather than against
 * "whatever it printed last time":
 *
 *   LPF1        pole a = tau/(tau+dt), step response 1-a^k, DC gain exactly 1
 *   MovingAvg   exact linear phase, exact rejection of a period-N square wave
 *   Median3     complete removal of an isolated spike, one-sample lag on ramps
 *   RateLimiter |dy/dt| <= rate_max, symmetric, dt-scaled
 *   deadband    continuity of the subtract form at the band edge
 *
 * Where a property is claimed in the header comment, it is asserted here.
 */
#include <stdio.h>
#include <math.h>
#include <float.h>

#include "pidx/pid_filter.h"

static int pass = 0, bad = 0;
#define CK(c,m) do{ if(c){pass++;} else {bad++;printf("  FAIL: %s\n",(m));} }while(0)

static bool close_to(double a, double b, double tol)
{
    return fabs(a - b) <= tol;
}

/* ===================================================================== */
static void t_lpf1(void)
{
    PID_LPF1 f;
    const double dt = 0.001;
    const double tau = 0.01;
    const double a = tau / (tau + dt);
    int k;
    double y = 0.0;

    puts("[1] first-order low-pass");

    CK(PID_LPF1_Init(NULL, 0.01f, 0.001f) == PID_ERR_NULL, "LPF1_Init NULL");
    CK(PID_LPF1_Init(&f, -1.0f, 0.001f) == PID_ERR_INVALID_PARAM, "LPF1 negative tau");
    CK(PID_LPF1_Init(&f, 0.01f, 0.0f) == PID_ERR_INVALID_DT, "LPF1 dt=0");
    CK(PID_LPF1_Init(&f, 0.01f, -0.001f) == PID_ERR_INVALID_DT, "LPF1 dt<0");
    CK(PID_LPF1_Init(&f, (PID_Float)NAN, 0.001f) == PID_ERR_INVALID_PARAM, "LPF1 NaN tau");

    CK(PID_LPF1_Init(&f, (PID_Float)tau, (PID_Float)dt) == PID_OK, "LPF1_Init ok");
    CK(close_to((double)f.a, a, 1e-6), "pole equals tau/(tau+dt)");

    /* The first sample seeds the state: an exponential filter that has to ramp
     * from zero would report a 100% error on a warm sensor at power-up. */
    CK(close_to((double)PID_LPF1_Update(&f, 5.0f), 5.0, 1e-6), "first sample seeds state");

    /* Step from 5 to 6. Analytic: y_k = 6 - 1*a^k. */
    for (k = 1; k <= 40; ++k) {
        y = (double)PID_LPF1_Update(&f, 6.0f);
        if (k == 1 || k == 10 || k == 40) {
            const double want = 6.0 - pow(a, (double)k);
            CK(close_to(y, want, 2e-5), "step response matches 1-a^k");
        }
    }

    /* DC gain is exactly one: sum of the coefficients is a + (1-a). */
    for (k = 0; k < 5000; ++k) { y = (double)PID_LPF1_Update(&f, 6.0f); }
    CK(close_to(y, 6.0, 1e-4), "DC gain is unity (settles on the input)");

    /* tau = 0 must be a pure pass-through, not a divide-by-zero. */
    {
        PID_LPF1 g;
        CK(PID_LPF1_Init(&g, 0.0f, 0.001f) == PID_OK, "tau=0 accepted");
        CK((double)g.a == 0.0, "tau=0 gives pole 0");
        (void)PID_LPF1_Update(&g, 1.0f);
        CK(close_to((double)PID_LPF1_Update(&g, 7.0f), 7.0, 1e-9), "tau=0 passes through");
    }

    /* Cutoff form: tau = 1/(2 pi fc). */
    {
        PID_LPF1 g;
        const double fc = 20.0;
        const double two_pi = 6.283185307179586;
        CK(PID_LPF1_Init(&g, 0.0f, (PID_Float)dt) == PID_OK, "init before cutoff");
        CK(PID_LPF1_SetCutoff(NULL, (PID_Float)fc, (PID_Float)dt) == PID_ERR_NULL,
           "SetCutoff NULL");
        CK(PID_LPF1_SetCutoff(&g, (PID_Float)fc, (PID_Float)dt) == PID_OK, "SetCutoff ok");
        CK(close_to((double)g.tau, 1.0 / (two_pi * fc), 1e-6), "tau = 1/(2 pi fc)");
        CK(PID_LPF1_SetCutoff(&g, 0.0f, (PID_Float)dt) == PID_ERR_INVALID_PARAM,
           "fc=0 rejected");
    }

    /* A non-finite sample must not poison the state. This is the whole reason
     * PID_LPF1_Update exists next to the inline pidf_lpf1_step. */
    {
        double before, after;
        before = (double)PID_LPF1_Update(&f, 6.0f);
        after  = (double)PID_LPF1_Update(&f, (PID_Float)NAN);
        CK(close_to(before, after, 0.0), "NaN sample holds the previous state");
        after = (double)PID_LPF1_Update(&f, (PID_Float)INFINITY);
        CK(close_to(before, after, 0.0), "Inf sample holds the previous state");
        CK(isfinite(after), "state still finite after bad samples");
    }

    /* Reset forgets history; the next sample re-seeds. */
    CK(PID_LPF1_Reset(&f) == PID_OK, "reset ok");
    CK(close_to((double)PID_LPF1_Update(&f, -3.0f), -3.0, 1e-9), "re-seeds after reset");

    /* SetTau keeps the state (retuning a live filter must not bump it). */
    {
        const double keep = (double)PID_LPF1_Update(&f, -3.0f);
        CK(PID_LPF1_SetTau(&f, 0.05f, (PID_Float)dt) == PID_OK, "SetTau ok");
        CK(close_to((double)f.state, keep, 0.0), "SetTau preserves state");
    }
}

/* ===================================================================== */
static void t_movavg(void)
{
    PID_Float buf[8];
    PID_MovingAvg f;
    int i;
    double y = 0.0;

    puts("[2] moving average");

    CK(PID_MovingAvg_Init(&f, NULL, 8U) == PID_ERR_NULL, "MovingAvg NULL buffer");
    CK(PID_MovingAvg_Init(NULL, buf, 8U) == PID_ERR_NULL, "MovingAvg NULL self");
    CK(PID_MovingAvg_Init(&f, buf, 0U) == PID_ERR_INVALID_PARAM, "MovingAvg size 0");
    CK(PID_MovingAvg_Init(&f, buf, 8U) == PID_OK, "MovingAvg init");

    /* Before the window fills it averages what it has - not what it wishes it
     * had. Feeding 1,2,3 must read 1, 1.5, 2. */
    CK(close_to((double)PID_MovingAvg_Update(&f, 1.0f), 1.0, 1e-6), "partial window 1");
    CK(close_to((double)PID_MovingAvg_Update(&f, 2.0f), 1.5, 1e-6), "partial window 2");
    CK(close_to((double)PID_MovingAvg_Update(&f, 3.0f), 2.0, 1e-6), "partial window 3");

    /* Constant input: exact, no drift. */
    for (i = 0; i < 100; ++i) { y = (double)PID_MovingAvg_Update(&f, 4.0f); }
    CK(close_to(y, 4.0, 1e-6), "constant input averages exactly");

    /* The headline property: a square wave whose period equals the window is
     * annihilated. An LPF only attenuates it. */
    for (i = 0; i < 200; ++i) {
        y = (double)PID_MovingAvg_Update(&f, ((i % 8) < 4) ? 1.0f : -1.0f);
    }
    CK(close_to(y, 0.0, 1e-6), "period-N square wave fully rejected");

    /* Sum re-derivation must keep long runs exact. 200k samples of 1e-3 added
     * to a large offset is exactly the case where a naive running sum drifts. */
    {
        PID_MovingAvg g;
        PID_Float gb[4];
        (void)PID_MovingAvg_Init(&g, gb, 4U);
        for (i = 0; i < 200000; ++i) { y = (double)PID_MovingAvg_Update(&g, 1000.001f); }
        CK(close_to(y, 1000.001, 1e-3), "no unbounded sum drift over 200k samples");
    }

    /* Non-finite input is substituted with zero rather than poisoning the sum
     * forever - documented, and it is why the average dips instead of NaNing. */
    {
        double a1, a2;
        a1 = (double)PID_MovingAvg_Update(&f, (PID_Float)NAN);
        CK(isfinite(a1), "NaN does not make the average NaN");
        for (i = 0; i < 20; ++i) { a2 = (double)PID_MovingAvg_Update(&f, 1.0f); }
        CK(close_to(a2, 1.0, 1e-6), "recovers fully once the window rolls over");
    }

    CK(PID_MovingAvg_Reset(&f) == PID_OK, "reset ok");
    CK(close_to((double)PID_MovingAvg_Update(&f, 9.0f), 9.0, 1e-6), "reset clears window");
}

/* ===================================================================== */
static void t_median3(void)
{
    PID_Median3 f;
    int i;
    double y;

    puts("[3] median-of-3 despiker");

    CK(PID_Median3_Init(NULL) == PID_ERR_NULL, "Median3 NULL");
    CK(PID_Median3_Init(&f) == PID_OK, "Median3 init");

    /* First two samples pass through: there is no median of one. */
    CK(close_to((double)PID_Median3_Update(&f, 1.0f), 1.0, 0.0), "sample 1 passes");
    CK(close_to((double)PID_Median3_Update(&f, 1.0f), 1.0, 0.0), "sample 2 passes");

    /* Steady signal with one enormous isolated spike. The spike must vanish
     * completely - this is the property a low-pass cannot deliver. */
    y = (double)PID_Median3_Update(&f, 1000.0f);
    CK(close_to(y, 1.0, 0.0), "isolated spike removed entirely");
    y = (double)PID_Median3_Update(&f, 1.0f);
    CK(close_to(y, 1.0, 0.0), "no ringing after the spike");

    /* A ramp is passed with exactly one sample of lag, no distortion. */
    {
        PID_Median3 g;
        double prev_in = 0.0;
        (void)PID_Median3_Init(&g);
        for (i = 0; i < 10; ++i) {
            const double in = (double)i;
            y = (double)PID_Median3_Update(&g, (PID_Float)in);
            if (i >= 3) {
                CK(close_to(y, prev_in, 1e-9), "ramp delayed by exactly one sample");
            }
            prev_in = in;
        }
    }

    /* Two consecutive outliers are NOT removed: a 3-tap median can only kill
     * runs of length 1. Asserting the limitation stops anyone claiming more. */
    {
        PID_Median3 g;
        (void)PID_Median3_Init(&g);
        for (i = 0; i < 5; ++i) { (void)PID_Median3_Update(&g, 1.0f); }
        (void)PID_Median3_Update(&g, 50.0f);
        y = (double)PID_Median3_Update(&g, 50.0f);
        CK(close_to(y, 50.0, 0.0), "a 2-sample burst passes (documented limit)");
    }

    /* NaN must not enter the history. */
    {
        PID_Median3 g;
        (void)PID_Median3_Init(&g);
        for (i = 0; i < 4; ++i) { (void)PID_Median3_Update(&g, 2.0f); }
        y = (double)PID_Median3_Update(&g, (PID_Float)NAN);
        CK(isfinite(y), "NaN input yields a finite output");
        y = (double)PID_Median3_Update(&g, 2.0f);
        CK(close_to(y, 2.0, 1e-9), "history uncontaminated by NaN");
    }
}

/* ===================================================================== */
static void t_ratelimiter(void)
{
    PID_RateLimiter f;
    const double dt = 0.01;
    int i;
    double y = 0.0;

    puts("[4] rate limiter");

    CK(PID_RateLimiter_Init(NULL, 1.0f) == PID_ERR_NULL, "RateLimiter NULL");
    CK(PID_RateLimiter_Init(&f, -1.0f) == PID_ERR_INVALID_PARAM, "negative rate");
    CK(PID_RateLimiter_Init(&f, (PID_Float)NAN) == PID_ERR_INVALID_PARAM, "NaN rate");
    CK(PID_RateLimiter_Init(&f, 10.0f) == PID_OK, "init 10 unit/s");

    /* Priming: the first sample is adopted, otherwise every limiter would slew
     * up from zero on the first call. */
    CK(close_to((double)PID_RateLimiter_Update(&f, 100.0f, (PID_Float)dt), 100.0, 0.0),
       "first sample adopted (primed)");

    /* From 100, target 200, rate 10/s, dt 10 ms => 0.1 per step. */
    y = (double)PID_RateLimiter_Update(&f, 200.0f, (PID_Float)dt);
    CK(close_to(y, 100.1, 1e-5), "step size is rate_max*dt");
    for (i = 0; i < 9; ++i) { y = (double)PID_RateLimiter_Update(&f, 200.0f, (PID_Float)dt); }
    CK(close_to(y, 101.0, 1e-4), "ten steps advance exactly 1.0");

    /* Symmetry downwards. */
    y = (double)PID_RateLimiter_Update(&f, 0.0f, (PID_Float)dt);
    CK(close_to(y, 100.9, 1e-4), "downward step equally limited");

    /* A move smaller than one step lands exactly, with no overshoot and no
     * limit cycle around the target. */
    (void)PID_RateLimiter_Reset(&f, 5.0f);
    y = (double)PID_RateLimiter_Update(&f, 5.05f, (PID_Float)dt);
    CK(close_to(y, 5.05, 1e-6), "small move lands exactly");
    y = (double)PID_RateLimiter_Update(&f, 5.05f, (PID_Float)dt);
    CK(close_to(y, 5.05, 1e-6), "stays parked, no dither");

    /* rate_max = 0 documents as pass-through, not as "frozen". */
    {
        PID_RateLimiter g;
        (void)PID_RateLimiter_Init(&g, 0.0f);
        (void)PID_RateLimiter_Update(&g, 0.0f, (PID_Float)dt);
        CK(close_to((double)PID_RateLimiter_Update(&g, 999.0f, (PID_Float)dt), 999.0, 0.0),
           "rate_max=0 is pass-through");
    }

    /* Bad dt holds the output rather than moving by an unknown amount. */
    (void)PID_RateLimiter_Reset(&f, 1.0f);
    CK(close_to((double)PID_RateLimiter_Update(&f, 99.0f, 0.0f), 1.0, 0.0), "dt=0 holds");
    CK(close_to((double)PID_RateLimiter_Update(&f, 99.0f, -1.0f), 1.0, 0.0), "dt<0 holds");
    CK(close_to((double)PID_RateLimiter_Update(&f, (PID_Float)NAN, (PID_Float)dt), 1.0, 0.0),
       "NaN input holds");

    /* The rate bound must hold for any dt, including a very large one. */
    {
        PID_RateLimiter g;
        double y0, y1;
        (void)PID_RateLimiter_Init(&g, 2.0f);
        (void)PID_RateLimiter_Update(&g, 0.0f, 0.1f);
        y0 = 0.0;
        y1 = (double)PID_RateLimiter_Update(&g, 1000.0f, 3.0f);
        CK(close_to(y1 - y0, 6.0, 1e-5), "|dy| = rate_max*dt for a 3 s gap");
    }
}

/* ===================================================================== */
static void t_deadband(void)
{
    const double w = 0.5;
    double a, b;

    puts("[5] deadband");

    CK(close_to((double)pidf_deadband(0.4f, (PID_Float)w, true), 0.0, 0.0), "inside band -> 0");
    CK(close_to((double)pidf_deadband(-0.4f, (PID_Float)w, true), 0.0, 0.0), "inside band, negative");
    CK(close_to((double)pidf_deadband(0.5f, (PID_Float)w, true), 0.0, 0.0), "edge is inside");

    /* Continuity is the entire point of the subtract form: approaching the
     * edge from outside must approach zero, not jump by w. */
    a = (double)pidf_deadband(0.5001f, (PID_Float)w, true);
    CK(fabs(a) < 1e-3, "subtract form is continuous at the edge");
    b = (double)pidf_deadband(0.5001f, (PID_Float)w, false);
    CK(close_to(b, 0.5001, 1e-6), "hard form steps by the full width");

    CK(close_to((double)pidf_deadband(2.0f, (PID_Float)w, true), 1.5, 1e-6), "subtract above");
    CK(close_to((double)pidf_deadband(-2.0f, (PID_Float)w, true), -1.5, 1e-6), "subtract below");
    CK(close_to((double)pidf_deadband(2.0f, (PID_Float)w, false), 2.0, 1e-6), "hard above");

    /* Zero and negative widths must disable the band, not invert it.
     * Compare against the float literal widened back to double: 0.01f is not
     * 0.01, and a zero-tolerance compare against the decimal would fail on the
     * representation, not on the behaviour. */
    CK((double)pidf_deadband(0.01f, 0.0f, true) == (double)(PID_Float)0.01f,
       "width 0 disables");
    CK((double)pidf_deadband(0.01f, -1.0f, true) == (double)(PID_Float)0.01f,
       "negative width disables");
}

int main(void)
{
    puts("=== filter module suite ===\n");
    t_lpf1();
    t_movavg();
    t_median3();
    t_ratelimiter();
    t_deadband();
    printf("\n  filters: %d passed, %d failed\n", pass, bad);
    return (bad == 0) ? 0 : 1;
}

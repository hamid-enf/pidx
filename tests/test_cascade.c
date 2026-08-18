/* Cascade suite (PHASE 17).
 *
 * Example 06 showed the cascade WORKS (30x better disturbance rejection than a
 * single loop). This suite pins down the mechanics that the example only
 * exercises incidentally, and the failure modes it never reaches:
 *
 *   - multi-rate decimation: a decimated level must integrate over its OWN
 *     period, not the caller's dt, or it under-integrates by exactly the
 *     decimation factor; between runs it holds (zero-order hold);
 *   - setpoint range clamping between levels;
 *   - back-propagated anti-windup - including the directional rule, which is
 *     the part that turns into a lock-up if it is written symmetrically;
 *   - AW_FREEZE vs AW_BACK_CALC vs AW_NONE, on the same plant, in numbers;
 *   - bumpless auto/manual across the whole chain, not just the inner loop;
 *   - PID_Cascade_Validate's timescale-separation check.
 *
 * The plant is a two-state chain, y_inner integrating the actuator and y_outer
 * integrating y_inner - the canonical current/velocity or flow/level pair.
 */
#include <stdio.h>
#include <math.h>

#include "pidx/pid.h"
#include "pidx/pid_cascade.h"

static int pass = 0, bad = 0;
#define CK(c,m) do{ if(c){pass++;} else {bad++;printf("  FAIL: %s\n",(m));} }while(0)

static bool near(double a, double b, double tol) { return fabs(a - b) <= tol; }

/* ---- plant: y_in' = (k_in*u - y_in)/tau_in ,  y_out' = y_in ------------- */
typedef struct { double y_in, y_out; } Plant;

static void plant_step(Plant *p, double u, double dt, double u_min, double u_max)
{
    if (u > u_max) { u = u_max; }
    if (u < u_min) { u = u_min; }
    p->y_in  += ((2.0 * u) - p->y_in) / 0.02 * dt;   /* fast: tau 20 ms */
    p->y_out += p->y_in * dt;                        /* integrator      */
}

/* ---- loop construction -------------------------------------------------- */
static void mk_loop(PID_Handle *h, double kp, double ki, double kd,
                    double ts, double omin, double omax)
{
    PID_Config c;
    PID_ConfigDefault(&c);
    c.core.kp = (PID_Float)kp;
    c.core.ki = (PID_Float)ki;
    c.core.kd = (PID_Float)kd;
    c.core.sample_time = (PID_Float)ts;
    c.limits.use_output_limits = true;
    c.limits.output_min = (PID_Float)omin;
    c.limits.output_max = (PID_Float)omax;
    (void)PID_Init(h, &c);
}

/* ===================================================================== */
static void t_init(void)
{
    PID_Handle a, b, cc;
    PID_Handle *loops[3];
    PID_Handle *two[2];
    PID_Cascade cas;
    PID_Float ratio;
    uint8_t worst;

    puts("[1] construction and validation");

    mk_loop(&a, 1.0, 0.5, 0.0, 0.050, -10.0, 10.0);
    mk_loop(&b, 4.0, 20.0, 0.0, 0.005, -24.0, 24.0);
    mk_loop(&cc, 2.0, 10.0, 0.0, 0.001, -1.0, 1.0);
    loops[0] = &a; loops[1] = &b; loops[2] = &cc;
    two[0] = &a; two[1] = &b;

    CK(PID_Cascade_Init(NULL, loops, 2U) == PID_ERR_NULL, "Init NULL cascade");
    CK(PID_Cascade_Init(&cas, NULL, 2U) == PID_ERR_NULL, "Init NULL array");
    CK(PID_Cascade_Init(&cas, loops, 1U) == PID_ERR_INVALID_PARAM,
       "one loop is not a cascade");
    CK(PID_Cascade_Init(&cas, loops, (uint8_t)(PIDX_CASCADE_MAX_LOOPS + 1U))
       == PID_ERR_INVALID_PARAM, "too many loops");

    /* An uninitialised handle in the array must be caught at construction, not
     * at the first update in the ISR. */
    {
        PID_Handle junk;
        PID_Handle *mixed[2];
        PID_Cascade tmp;
        mixed[0] = &a; mixed[1] = &junk;
        junk.init_magic = 0U;
        CK(PID_Cascade_Init(&tmp, mixed, 2U) == PID_ERR_NOT_INIT,
           "uninitialised member loop rejected");
        mixed[1] = NULL;
        CK(PID_Cascade_Init(&tmp, mixed, 2U) == PID_ERR_NULL, "NULL member rejected");
    }

    CK(PID_Cascade_Init(&cas, loops, 3U) == PID_OK, "3-level init ok");
    CK(PID_Cascade_GetLoop(&cas, 0U) == &a, "level 0 is the outermost");
    CK(PID_Cascade_GetLoop(&cas, 2U) == &cc, "level 2 is the innermost");
    CK(PID_Cascade_GetLoop(&cas, 3U) == NULL, "index past the end returns NULL");
    CK(PID_Cascade_GetLoop(NULL, 0U) == NULL, "GetLoop(NULL)");

    /* Init derives back-calculation gain Ki/Kp of the OUTER loop = 0.5/1 . */
    CK(near((double)cas.aw_gain, 0.5, 1e-6), "aw_gain = Ki/Kp of loop 0");
    CK(cas.aw_mode == (uint8_t)PID_CASCADE_AW_BACK_CALC, "BACK_CALC by default");

    /* Timescale separation: 50/5 = 10 and 5/1 = 5, so the worst is 5 - fine. */
    CK(PID_Cascade_Validate(&cas, &ratio, &worst) == PID_OK, "3x separation met");
    CK(near((double)ratio, 5.0, 1e-4), "worst ratio is 5 (levels 1->2)");
    CK(worst == 1U, "and it is reported at the right level");

    /* Now break it: an inner loop barely faster than its parent. */
    {
        PID_Handle slow;
        PID_Handle *pair[2];
        PID_Cascade bad_cas;
        mk_loop(&slow, 1.0, 1.0, 0.0, 0.040, -1.0, 1.0);
        pair[0] = &a; pair[1] = &slow;
        (void)PID_Cascade_Init(&bad_cas, pair, 2U);
        CK(PID_Cascade_Validate(&bad_cas, &ratio, &worst) == PID_ERR_INVALID_PARAM,
           "1.25x separation rejected");
        CK(near((double)ratio, 1.25, 1e-4), "and the actual ratio is reported");
    }

    /* Decimation participates in the effective period: outer 50 ms with
     * decimation 4 is really 200 ms against a 5 ms inner loop. */
    (void)PID_Cascade_Init(&cas, two, 2U);
    CK(PID_Cascade_ConfigLevel(&cas, 0U, 4U, 0.0f, 0.0f) == PID_OK, "ConfigLevel ok");
    CK(PID_Cascade_Validate(&cas, &ratio, NULL) == PID_OK, "still valid");
    CK(near((double)ratio, 40.0, 1e-3), "decimation counts toward the period");
    CK(PID_Cascade_ConfigLevel(&cas, 9U, 1U, 0.0f, 0.0f) == PID_ERR_INVALID_PARAM,
       "ConfigLevel index checked");
    CK(PID_Cascade_ConfigLevel(NULL, 0U, 1U, 0.0f, 0.0f) == PID_ERR_NULL,
       "ConfigLevel NULL");

    CK(PID_Cascade_SetAntiWindup(&cas, (PID_CascadeAntiWindup)7, 1.0f)
       == PID_ERR_INVALID_PARAM, "unknown AW mode rejected");
    CK(PID_Cascade_SetAntiWindup(&cas, PID_CASCADE_AW_FREEZE, (PID_Float)NAN)
       == PID_ERR_INVALID_PARAM, "NaN aw_gain rejected");
    CK(PID_Cascade_SetAntiWindup(&cas, PID_CASCADE_AW_BACK_CALC, 0.0f) == PID_OK,
       "aw_gain <= 0 keeps the derived value");
    CK(near((double)cas.aw_gain, 0.5, 1e-6), "derived gain preserved");
}

/* ===================================================================== */
static void t_bad_inputs(void)
{
    PID_Handle a, b;
    PID_Handle *loops[2];
    PID_Cascade cas;
    PID_Float m[2];
    double u_good;

    puts("[2] bad inputs hold the last output");

    mk_loop(&a, 1.0, 0.5, 0.0, 0.010, -10.0, 10.0);
    mk_loop(&b, 4.0, 20.0, 0.0, 0.001, -24.0, 24.0);
    loops[0] = &a; loops[1] = &b;
    (void)PID_Cascade_Init(&cas, loops, 2U);

    m[0] = 0.0f; m[1] = 0.0f;
    u_good = (double)PID_Cascade_Update(&cas, m, 1.0f, 0.001f);
    CK(u_good != 0.0, "a good update produces drive");

    CK((double)PID_Cascade_Update(&cas, NULL, 1.0f, 0.001f) == u_good,
       "NULL measurements holds");
    CK(PID_Cascade_GetLastError(&cas) == PID_ERR_NULL, "and reports it");
    CK((double)PID_Cascade_Update(&cas, m, 1.0f, 0.0f) == u_good, "dt=0 holds");
    CK(PID_Cascade_GetLastError(&cas) == PID_ERR_INVALID_DT, "reported as INVALID_DT");
    CK((double)PID_Cascade_Update(&cas, m, 1.0f, (PID_Float)NAN) == u_good, "NaN dt holds");
    /* pidc_set_error is sticky FIRST-wins, so the pending INVALID_DT has to be
     * read out before the next code can be observed. Forgetting this reads the
     * previous fault and looks like the new one was never reported. */
    CK(PID_Cascade_GetLastError(&cas) == PID_ERR_INVALID_DT, "NaN dt also INVALID_DT");
    CK((double)PID_Cascade_Update(&cas, m, (PID_Float)INFINITY, 0.001f) == u_good,
       "Inf setpoint holds");
    CK(PID_Cascade_GetLastError(&cas) == PID_ERR_NAN_INPUT, "reported as NAN_INPUT");
    CK((double)PID_Cascade_Update(NULL, m, 1.0f, 0.001f) == 0.0, "Update(NULL) is 0");

    /* GetLastError is read-and-clear, like the core's. */
    CK(PID_Cascade_GetLastError(&cas) == PID_OK, "error cleared after reading");
    CK(PID_Cascade_GetLastError(NULL) == PID_ERR_NULL, "GetLastError(NULL)");

    /* A per-loop fault must surface at the cascade level - the caller should
     * not have to poll every handle. */
    m[1] = (PID_Float)NAN;
    (void)PID_Cascade_Update(&cas, m, 1.0f, 0.001f);
    CK(PID_Cascade_GetLastError(&cas) == PID_ERR_NAN_INPUT,
       "an inner loop's NaN measurement surfaces at the cascade");
}

/* ===================================================================== */
static void t_decimation(void)
{
    PID_Handle a, b;
    PID_Handle *loops[2];
    PID_Cascade cas;
    PID_Float m[2];
    double sp_seen[12];
    double i_dec, i_plain;
    int i;

    puts("[3] multi-rate decimation");

    mk_loop(&a, 1.0, 2.0, 0.0, 0.005, -10.0, 10.0);
    mk_loop(&b, 4.0, 20.0, 0.0, 0.001, -24.0, 24.0);
    loops[0] = &a; loops[1] = &b;
    (void)PID_Cascade_Init(&cas, loops, 2U);
    (void)PID_Cascade_ConfigLevel(&cas, 0U, 5U, 0.0f, 0.0f);

    m[0] = 0.0f; m[1] = 0.0f;

    /* The inner setpoint (= outer command) must change only on every 5th call
     * and hold in between. */
    for (i = 0; i < 12; ++i) {
        (void)PID_Cascade_Update(&cas, m, 1.0f, 0.001f);
        sp_seen[i] = (double)PID_Cascade_GetLevelSetpoint(&cas, 1U);
    }
    CK(sp_seen[0] == 0.0, "outer has not run yet on call 1");
    CK(sp_seen[3] == sp_seen[0], "held through call 4");
    CK(sp_seen[4] != sp_seen[3], "outer runs on call 5");
    CK(sp_seen[5] == sp_seen[4] && sp_seen[8] == sp_seen[4], "held for 4 more calls");
    CK(sp_seen[9] != sp_seen[8], "and again on call 10");

    /* The decimated level must integrate over 5*dt, not dt. Compare its
     * integrator against the same loop run undecimated at 5x the dt: after the
     * same amount of SIMULATED time the two must agree, not differ by 5x. */
    (void)PID_Cascade_Reset(&cas);
    (void)PID_SetIntegrator(&a, 0.0f);
    for (i = 0; i < 500; ++i) { (void)PID_Cascade_Update(&cas, m, 1.0f, 0.001f); }
    i_dec = (double)PID_GetIntegrator(&a);

    {
        PID_Handle a2, b2;
        PID_Handle *l2[2];
        PID_Cascade c2;
        mk_loop(&a2, 1.0, 2.0, 0.0, 0.005, -10.0, 10.0);
        mk_loop(&b2, 4.0, 20.0, 0.0, 0.001, -24.0, 24.0);
        l2[0] = &a2; l2[1] = &b2;
        (void)PID_Cascade_Init(&c2, l2, 2U);
        for (i = 0; i < 100; ++i) { (void)PID_Cascade_Update(&c2, m, 1.0f, 0.005f); }
        i_plain = (double)PID_GetIntegrator(&a2);
    }
    printf("    outer integrator: decimated %.6f vs undecimated %.6f\n",
           i_dec, i_plain);
    CK(fabs(i_dec - i_plain) < 1e-4,
       "a decimated level integrates over its own period, not the caller's dt");

    /* decimation 0 and 1 must both mean "every call". */
    {
        PID_Cascade c3;
        PID_Handle a3, b3;
        PID_Handle *l3[2];
        double s0, s1;
        mk_loop(&a3, 1.0, 2.0, 0.0, 0.005, -10.0, 10.0);
        mk_loop(&b3, 4.0, 20.0, 0.0, 0.001, -24.0, 24.0);
        l3[0] = &a3; l3[1] = &b3;
        (void)PID_Cascade_Init(&c3, l3, 2U);
        (void)PID_Cascade_ConfigLevel(&c3, 0U, 0U, 0.0f, 0.0f);
        (void)PID_Cascade_Update(&c3, m, 1.0f, 0.001f);
        s0 = (double)PID_Cascade_GetLevelSetpoint(&c3, 1U);
        (void)PID_Cascade_Update(&c3, m, 1.0f, 0.001f);
        s1 = (double)PID_Cascade_GetLevelSetpoint(&c3, 1U);
        CK((s0 != 0.0) && (s1 != s0), "decimation 0 runs every call");
    }
}

/* ===================================================================== */
static void t_sp_clamp(void)
{
    PID_Handle a, b;
    PID_Handle *loops[2];
    PID_Cascade cas;
    PID_Float m[2];
    int i;

    puts("[4] inter-level setpoint clamp");

    mk_loop(&a, 5.0, 10.0, 0.0, 0.001, -100.0, 100.0);
    mk_loop(&b, 4.0, 20.0, 0.0, 0.001, -24.0, 24.0);
    loops[0] = &a; loops[1] = &b;
    (void)PID_Cascade_Init(&cas, loops, 2U);

    /* Default sp_min = sp_max = 0 disables the clamp: a huge outer command
     * must reach the inner loop untouched. */
    m[0] = 0.0f; m[1] = 0.0f;
    for (i = 0; i < 50; ++i) { (void)PID_Cascade_Update(&cas, m, 10.0f, 0.001f); }
    CK((double)PID_Cascade_GetLevelSetpoint(&cas, 1U) > 5.0,
       "clamp disabled by default (min == max)");

    /* Now clamp the inner setpoint to +/-2 - a velocity limit, physically. */
    (void)PID_Cascade_Reset(&cas);
    (void)PID_Cascade_ConfigLevel(&cas, 0U, 1U, -2.0f, 2.0f);
    for (i = 0; i < 200; ++i) { (void)PID_Cascade_Update(&cas, m, 10.0f, 0.001f); }
    CK(near((double)PID_Cascade_GetLevelSetpoint(&cas, 1U), 2.0, 1e-6),
       "inner setpoint clamped to sp_max");

    for (i = 0; i < 400; ++i) { (void)PID_Cascade_Update(&cas, m, -10.0f, 0.001f); }
    CK(near((double)PID_Cascade_GetLevelSetpoint(&cas, 1U), -2.0, 1e-6),
       "and to sp_min in the other direction");

    /* The clamp must also feed anti-windup: with the command pinned at 2.0 for
     * hundreds of samples the outer integrator must NOT run away. */
    (void)PID_Cascade_Reset(&cas);
    for (i = 0; i < 2000; ++i) { (void)PID_Cascade_Update(&cas, m, 10.0f, 0.001f); }
    printf("    outer integrator against a pinned clamp: %.4f\n",
           (double)PID_GetIntegrator(&a));
    CK(fabs((double)PID_GetIntegrator(&a)) < 20.0,
       "clamped command is back-propagated, so the outer loop does not wind up");
}

/* ===================================================================== */
/* Run the two-state plant to a step, return peak overshoot in y_out.       */
static double run_step(PID_CascadeAntiWindup aw, double *peak_i, double *settle_s)
{
    PID_Handle a, b;
    PID_Handle *loops[2];
    PID_Cascade cas;
    Plant p;
    PID_Float m[2];
    const double dt = 0.001;
    const double target = 1.0;
    double peak = 0.0, pk_i = 0.0;
    int i, settled = -1;

    mk_loop(&a, 3.0, 6.0, 0.0, 0.005, -100.0, 100.0);
    mk_loop(&b, 2.0, 10.0, 0.0, 0.001, -1.0, 1.0);   /* actuator +/-1 */
    loops[0] = &a; loops[1] = &b;
    (void)PID_Cascade_Init(&cas, loops, 2U);
    (void)PID_Cascade_ConfigLevel(&cas, 0U, 5U, 0.0f, 0.0f);
    (void)PID_Cascade_SetAntiWindup(&cas, aw, 0.0f);

    p.y_in = 0.0; p.y_out = 0.0;

    for (i = 0; i < 8000; ++i) {
        double u;
        m[0] = (PID_Float)p.y_out;
        m[1] = (PID_Float)p.y_in;
        u = (double)PID_Cascade_Update(&cas, m, (PID_Float)target, (PID_Float)dt);
        plant_step(&p, u, dt, -1.0, 1.0);

        if (fabs((double)PID_GetIntegrator(&a)) > pk_i) {
            pk_i = fabs((double)PID_GetIntegrator(&a));
        }
        if (p.y_out > peak) { peak = p.y_out; }
        if ((settled < 0) && (i > 100) && (fabs(p.y_out - target) < 0.02)) {
            settled = i;
        }
        if ((settled >= 0) && (fabs(p.y_out - target) > 0.02)) {
            settled = -1;   /* left the band again */
        }
    }
    *peak_i = pk_i;
    *settle_s = (settled < 0) ? -1.0 : ((double)settled * dt);
    return (peak - target) / target * 100.0;
}

static void t_antiwindup(void)
{
    double os_none, os_bc, os_fr;
    double pi_none, pi_bc, pi_fr;
    double ts_none, ts_bc, ts_fr;

    puts("[5] back-propagated anti-windup, measured");

    /*
     * The plant's actuator saturates at +/-1 while the outer loop asks for a
     * unit step in a double integrator: the inner loop is pinned at the rail
     * for the whole acceleration phase. That is precisely the situation where
     * outer-loop windup shows up as overshoot.
     */
    os_none = run_step(PID_CASCADE_AW_NONE,      &pi_none, &ts_none);
    os_bc   = run_step(PID_CASCADE_AW_BACK_CALC, &pi_bc,   &ts_bc);
    os_fr   = run_step(PID_CASCADE_AW_FREEZE,    &pi_fr,   &ts_fr);

    printf("    AW_NONE      overshoot %6.2f %%  peak|I| %6.3f  settle %6.3f s\n",
           os_none, pi_none, ts_none);
    printf("    AW_BACK_CALC overshoot %6.2f %%  peak|I| %6.3f  settle %6.3f s\n",
           os_bc, pi_bc, ts_bc);
    printf("    AW_FREEZE    overshoot %6.2f %%  peak|I| %6.3f  settle %6.3f s\n",
           os_fr, pi_fr, ts_fr);

    CK(os_none > 0.0, "the unprotected cascade does overshoot");
    CK(pi_bc < pi_none, "BACK_CALC holds a smaller integrator than NONE");
    CK(pi_fr < pi_none, "FREEZE holds a smaller integrator than NONE");
    CK(os_bc < os_none, "BACK_CALC overshoots less than NONE");
    CK(os_fr <= os_none, "FREEZE is no worse than NONE");
    CK(ts_bc > 0.0 && ts_bc <= ts_none, "BACK_CALC settles no later than NONE");
    CK(os_bc < 30.0, "and the protected overshoot is actually usable");
}

static void t_aw_direction(void)
{
    PID_Handle a, b;
    PID_Handle *loops[2];
    PID_Cascade cas;
    Plant p;
    PID_Float m[2];
    const double dt = 0.001;
    double i_pinned, i_after, i_end;
    int i;

    puts("[6] anti-windup must not lock the loop up");

    /*
     * The directional rule: a child pinned at its UPPER rail must still let its
     * parent integrate DOWNWARDS. A symmetric implementation - correcting
     * whenever the child is saturated, regardless of which way the parent is
     * pushing - traps the pair: the outer loop can never unwind and the
     * actuator stays at the rail forever. This is the single most common
     * cascade anti-windup bug, so it gets an explicit test.
     *
     * A LIVE plant is essential here. Against a frozen measurement the child
     * can never recover, so the parent's integrator is legitimately pinned by
     * anti-windup in both phases and the test proves nothing about direction.
     */
    mk_loop(&a, 3.0, 6.0, 0.0, 0.001, -100.0, 100.0);
    mk_loop(&b, 2.0, 10.0, 0.0, 0.001, -1.0, 1.0);
    loops[0] = &a; loops[1] = &b;
    (void)PID_Cascade_Init(&cas, loops, 2U);

    p.y_in = 0.0; p.y_out = 0.0;

    /* Phase 1: an unreachable climb. The actuator sits at its upper rail for
     * the whole run and the outer integrator is held by back-propagation. */
    for (i = 0; i < 4000; ++i) {
        m[0] = (PID_Float)p.y_out; m[1] = (PID_Float)p.y_in;
        plant_step(&p, (double)PID_Cascade_Update(&cas, m, 50.0f, (PID_Float)dt),
                   dt, -1.0, 1.0);
    }
    i_pinned = (double)PID_GetIntegrator(&a);
    CK(PID_IsSaturated(&b), "inner loop is pinned at its rail");
    CK(PID_Cascade_IsSaturated(&cas), "and the cascade reports it");
    CK(p.y_in > 1.5, "the plant really is running away upwards");

    /* Phase 2: the operator reverses the demand. The child is STILL pinned
     * high for the first samples, but the parent must be free to unwind. */
    for (i = 0; i < 200; ++i) {
        m[0] = (PID_Float)p.y_out; m[1] = (PID_Float)p.y_in;
        plant_step(&p, (double)PID_Cascade_Update(&cas, m, -50.0f, (PID_Float)dt),
                   dt, -1.0, 1.0);
    }
    i_after = (double)PID_GetIntegrator(&a);
    printf("    outer I: %.4f pinned -> %.4f after a reversal\n", i_pinned, i_after);
    CK(i_after < i_pinned, "the parent unwinds while the child is still pinned");

    /* And it must go all the way through zero, not stall at some floor. */
    for (i = 0; i < 4000; ++i) {
        m[0] = (PID_Float)p.y_out; m[1] = (PID_Float)p.y_in;
        plant_step(&p, (double)PID_Cascade_Update(&cas, m, -50.0f, (PID_Float)dt),
                   dt, -1.0, 1.0);
    }
    i_end = (double)PID_GetIntegrator(&a);
    printf("    and %.4f after the reversal completes (u %.3f, y_in %.3f)\n",
           i_end, (double)PID_Cascade_GetOutput(&cas), p.y_in);
    CK(i_end < 0.0, "passes through zero - no lock-up");
    CK((double)PID_Cascade_GetOutput(&cas) < 0.0, "the actuator reverses too");
    CK(p.y_in < 0.0, "and the plant actually turns around");
}

/* ===================================================================== */
static void t_modes(void)
{
    PID_Handle a, b;
    PID_Handle *loops[2];
    PID_Cascade cas;
    Plant p;
    PID_Float m[2];
    double u_last, u_manual, u_resume, jump;
    int i;

    puts("[7] mode switching is bumpless across the whole chain");

    mk_loop(&a, 3.0, 6.0, 0.0, 0.001, -100.0, 100.0);
    mk_loop(&b, 2.0, 10.0, 0.0, 0.001, -1.0, 1.0);
    loops[0] = &a; loops[1] = &b;
    (void)PID_Cascade_Init(&cas, loops, 2U);

    CK(PID_Cascade_SetMode(NULL, PID_MODE_MANUAL) == PID_ERR_NULL, "SetMode NULL");
    CK(PID_Cascade_SetMode(&cas, (PID_Mode)42) == PID_ERR_INVALID_MODE,
       "unknown mode rejected");

    p.y_in = 0.0; p.y_out = 0.0;
    for (i = 0; i < 3000; ++i) {
        m[0] = (PID_Float)p.y_out; m[1] = (PID_Float)p.y_in;
        plant_step(&p, (double)PID_Cascade_Update(&cas, m, 0.5f, 0.001f), 0.001,
                   -1.0, 1.0);
    }
    u_last = (double)PID_Cascade_GetOutput(&cas);
    CK(fabs(p.y_out - 0.5) < 0.05, "chain is tracking before the switch");

    /* AUTOMATIC -> MANUAL must not move the actuator by itself. */
    CK(PID_Cascade_SetMode(&cas, PID_MODE_MANUAL) == PID_OK, "to MANUAL");
    m[0] = (PID_Float)p.y_out; m[1] = (PID_Float)p.y_in;
    u_manual = (double)PID_Cascade_Update(&cas, m, 0.5f, 0.001f);
    printf("    u %.6f -> %.6f entering MANUAL\n", u_last, u_manual);
    CK(fabs(u_manual - u_last) < 0.02, "entering MANUAL does not bump the actuator");

    /*
     * Drive it by hand for a while, letting the plant drift away.
     *
     * 0.02 rather than a big excursion, deliberately. The inner loop's
     * integrator lives in output units and is bounded by the +/-1 actuator
     * range, so a manual command that implies I outside that range CANNOT be
     * back-solved - the library reports exactly that (PID_FLAG_INTEGRAL_LIMITED
     * plus a sticky PID_ERR_INVALID_LIMIT) and the bump on resume is then real
     * and unavoidable, not a defect. That case is asserted separately below;
     * here we test the representable path, which is the one that must be
     * perfectly bumpless.
     */
    CK(PID_Cascade_SetManualOutput(&cas, 0.02f) == PID_OK, "manual output accepted");
    CK(PID_Cascade_SetManualOutput(&cas, (PID_Float)NAN) == PID_ERR_INVALID_PARAM,
       "NaN manual output rejected");
    for (i = 0; i < 300; ++i) {
        m[0] = (PID_Float)p.y_out; m[1] = (PID_Float)p.y_in;
        plant_step(&p, (double)PID_Cascade_Update(&cas, m, 0.5f, 0.001f), 0.001,
                   -1.0, 1.0);
    }
    CK(near((double)PID_Cascade_GetOutput(&cas), 0.02, 1e-3),
       "manual value is what reaches the actuator");
    CK(((unsigned long)PID_GetFlags(PID_Cascade_GetLoop(&cas, 1U)) &
        (unsigned long)PID_FLAG_INTEGRAL_LIMITED) == 0UL,
       "a representable manual value does not exhaust the integrator");
    u_last = (double)PID_Cascade_GetOutput(&cas);

    /* MANUAL -> AUTOMATIC: every level must back-solve, not just the inner
     * one. If only the inner loop is bumpless the outer loop's stale command
     * kicks the actuator on the first automatic sample. */
    CK(PID_Cascade_SetMode(&cas, PID_MODE_AUTOMATIC) == PID_OK, "back to AUTOMATIC");
    m[0] = (PID_Float)p.y_out; m[1] = (PID_Float)p.y_in;
    u_resume = (double)PID_Cascade_Update(&cas, m, 0.5f, 0.001f);
    jump = fabs(u_resume - u_last);
    printf("    u %.6f -> %.6f resuming AUTOMATIC (jump %.6f)\n",
           u_last, u_resume, jump);
    CK(jump < 0.02, "resuming AUTOMATIC does not bump the actuator");

    /*
     * The unrepresentable case, asserted rather than avoided. A manual command
     * of 0.30 against a +/-1 actuator drives the inner integrator to its rail;
     * the library must SAY so instead of pretending the transfer was bumpless.
     * This is the documented "poll before switching mode" contract.
     */
    {
        PID_Handle a2, b2;
        PID_Handle *l2[2];
        PID_Cascade c2;
        Plant q;
        PID_StatusCode e = PID_OK;
        PID_Handle *inner;

        mk_loop(&a2, 3.0, 6.0, 0.0, 0.001, -100.0, 100.0);
        mk_loop(&b2, 2.0, 10.0, 0.0, 0.001, -1.0, 1.0);
        l2[0] = &a2; l2[1] = &b2;
        (void)PID_Cascade_Init(&c2, l2, 2U);
        q.y_in = 0.0; q.y_out = 0.0;

        for (i = 0; i < 3000; ++i) {
            m[0] = (PID_Float)q.y_out; m[1] = (PID_Float)q.y_in;
            plant_step(&q, (double)PID_Cascade_Update(&c2, m, 0.5f, 0.001f), 0.001,
                       -1.0, 1.0);
        }
        (void)PID_Cascade_SetMode(&c2, PID_MODE_MANUAL);
        (void)PID_Cascade_SetManualOutput(&c2, 0.30f);
        for (i = 0; i < 2000; ++i) {
            m[0] = (PID_Float)q.y_out; m[1] = (PID_Float)q.y_in;
            plant_step(&q, (double)PID_Cascade_Update(&c2, m, 0.5f, 0.001f), 0.001,
                       -1.0, 1.0);
        }
        inner = PID_Cascade_GetLoop(&c2, 1U);
        CK(((unsigned long)PID_GetFlags(inner) &
            (unsigned long)PID_FLAG_INTEGRAL_LIMITED) != 0UL,
           "an unrepresentable manual value raises INTEGRAL_LIMITED");
        (void)PID_GetLastError(inner, &e);
        CK(e == PID_ERR_INVALID_LIMIT,
           "and a sticky INVALID_LIMIT the operator can poll before switching");
    }

    /*
     * HOLD freezes the INTEGRATOR, not the output. P and D keep responding -
     * that is the documented contract (pid_types.h): "all terms are computed
     * and the output is produced, but the integrator is frozen". A mode that
     * ignored the measurement entirely would be MANUAL, not HOLD.
     */
    CK(PID_Cascade_SetMode(&cas, PID_MODE_HOLD) == PID_OK, "to HOLD");
    {
        const double i_outer = (double)PID_GetIntegrator(&a);
        const double i_inner = (double)PID_GetIntegrator(&b);
        double u_hold;

        for (i = 0; i < 100; ++i) {
            m[0] = 9.0f; m[1] = 9.0f;   /* wildly wrong measurements */
            u_hold = (double)PID_Cascade_Update(&cas, m, 0.5f, 0.001f);
        }
        CK(near((double)PID_GetIntegrator(&a), i_outer, 1e-6),
           "HOLD freezes the outer integrator");
        CK(near((double)PID_GetIntegrator(&b), i_inner, 1e-6),
           "HOLD freezes the inner integrator");
        CK(u_hold < 0.0, "but P still reacts to a huge negative error");
    }

    /* Reset clears every level and the cascade's own state. */
    CK(PID_Cascade_Reset(&cas) == PID_OK, "Reset ok");
    CK((double)PID_Cascade_GetOutput(&cas) == 0.0, "output zeroed");
    CK((double)PID_GetIntegrator(&a) == 0.0 && (double)PID_GetIntegrator(&b) == 0.0,
       "every level's integrator cleared");
    CK(PID_Cascade_Reset(NULL) == PID_ERR_NULL, "Reset NULL");
}

/* ===================================================================== */
static void t_beats_single(void)
{
    PID_Handle a, b, single;
    PID_Handle *loops[2];
    PID_Cascade cas;
    Plant pc, ps;
    PID_Float m[2];
    const double dt = 0.001;
    double dip_c = 0.0, dip_s = 0.0;
    int i;

    puts("[8] the cascade earns its complexity");

    /*
     * Same plant, same outer tuning, same actuator limit. The only difference
     * is that the cascade closes a fast loop around y_in. A load disturbance
     * is injected into the inner state; the cascade should reject it before
     * the outer variable notices.
     */
    mk_loop(&a, 3.0, 6.0, 0.0, 0.001, -100.0, 100.0);
    mk_loop(&b, 2.0, 10.0, 0.0, 0.001, -1.0, 1.0);
    loops[0] = &a; loops[1] = &b;
    (void)PID_Cascade_Init(&cas, loops, 2U);
    mk_loop(&single, 3.0, 6.0, 0.0, 0.001, -1.0, 1.0);
    (void)PID_SetSetpoint(&single, 1.0f);

    pc.y_in = 0.0; pc.y_out = 0.0;
    ps.y_in = 0.0; ps.y_out = 0.0;

    for (i = 0; i < 20000; ++i) {
        double uc, us;

        m[0] = (PID_Float)pc.y_out; m[1] = (PID_Float)pc.y_in;
        uc = (double)PID_Cascade_Update(&cas, m, 1.0f, (PID_Float)dt);
        us = (double)PID_Update(&single, (PID_Float)ps.y_out);

        plant_step(&pc, uc, dt, -1.0, 1.0);
        plant_step(&ps, us, dt, -1.0, 1.0);

        /* After both have settled, kick the inner state once per sample for
         * 200 ms - a load torque, not a setpoint change. */
        if ((i > 12000) && (i < 12200)) {
            pc.y_in -= 0.5 * dt / 0.02;
            ps.y_in -= 0.5 * dt / 0.02;
        }
        if (i > 12000) {
            const double dc = fabs(pc.y_out - 1.0);
            const double ds = fabs(ps.y_out - 1.0);
            if (dc > dip_c) { dip_c = dc; }
            if (ds > dip_s) { dip_s = ds; }
        }
    }
    printf("    peak deviation after the load step: cascade %.5f, single %.5f (%.1fx)\n",
           dip_c, dip_s, dip_s / dip_c);
    CK(dip_c < dip_s, "the cascade rejects the inner disturbance better");
    CK(fabs(pc.y_out - 1.0) < 0.05, "cascade recovers to setpoint");
    CK(fabs(ps.y_out - 1.0) < 0.05, "single loop recovers too, just more slowly");
}

int main(void)
{
    puts("=== cascade suite ===\n");
    t_init();
    t_bad_inputs();
    t_decimation();
    t_sp_clamp();
    t_antiwindup();
    t_aw_direction();
    t_modes();
    t_beats_single();
    printf("\n  cascade: %d passed, %d failed\n", pass, bad);
    return (bad == 0) ? 0 : 1;
}

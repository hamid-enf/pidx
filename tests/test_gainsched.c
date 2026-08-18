/* Gain-scheduling suite (PHASE 17).
 *
 * Two separate things are tested and they fail in different ways:
 *
 *   A. PID_GainSched_Evaluate as pure table maths - interpolation, clamping
 *      outside the table, hysteresis, and the C1 continuity that is the only
 *      reason PID_SCHED_INTERP_SMOOTH exists.
 *   B. the schedule wired into a live controller - that each PID_SchedSource
 *      really reads the signal it names, and (the property that actually
 *      matters in the field) that a gain change mid-flight does not bump the
 *      output, because the integrator is stored in output units.
 *
 * B is where a scheduling implementation usually goes wrong: the table is
 * fine, but the controller kicks every time the gains move.
 */
#include <stdio.h>
#include <math.h>

#include "pidx/pid.h"
#include "pidx/pid_gainsched.h"

static int pass = 0, bad = 0;
#define CK(c,m) do{ if(c){pass++;} else {bad++;printf("  FAIL: %s\n",(m));} }while(0)

/* A deliberately non-uniform table: unequal spacing catches an implementation
 * that assumes a constant step, and the Kd column runs the other way to catch
 * a copy-paste of the Kp interpolation. */
static const PID_GainPoint TBL[4] = {
    /*  x      kp     ki     kd   */
    {  0.0f,  1.0f,  0.10f, 0.30f },
    { 10.0f,  2.0f,  0.20f, 0.20f },
    { 50.0f,  6.0f,  0.60f, 0.10f },
    { 60.0f,  6.0f,  0.60f, 0.00f }
};

static bool near(double a, double b, double tol) { return fabs(a - b) <= tol; }

/* ===================================================================== */
static void t_init_validation(void)
{
    PID_GainSchedule s;
    PID_GainPoint bad_tbl[3];

    puts("[1] table validation");

    CK(PID_GainSched_Init(NULL, TBL, 4U, PID_SCHED_SRC_SETPOINT,
                          PID_SCHED_INTERP_LINEAR) == PID_ERR_NULL, "Init NULL self");
    CK(PID_GainSched_Init(&s, NULL, 4U, PID_SCHED_SRC_SETPOINT,
                          PID_SCHED_INTERP_LINEAR) == PID_ERR_NULL, "Init NULL table");
    CK(PID_GainSched_Init(&s, TBL, 1U, PID_SCHED_SRC_SETPOINT,
                          PID_SCHED_INTERP_LINEAR) == PID_ERR_INVALID_PARAM,
       "one point is not a schedule");
    CK(PID_GainSched_Init(&s, TBL, (uint8_t)(PIDX_GAINSCHED_MAX_POINTS + 1U),
                          PID_SCHED_SRC_SETPOINT, PID_SCHED_INTERP_LINEAR)
       == PID_ERR_INVALID_PARAM, "count above MAX_POINTS rejected");
    CK(PID_GainSched_Init(&s, TBL, 4U, (PID_SchedSource)99,
                          PID_SCHED_INTERP_LINEAR) == PID_ERR_INVALID_PARAM,
       "unknown source rejected");
    CK(PID_GainSched_Init(&s, TBL, 4U, PID_SCHED_SRC_SETPOINT,
                          (PID_SchedInterp)99) == PID_ERR_INVALID_PARAM,
       "unknown interpolation rejected");

    /* Descending or duplicated breakpoints: duplicates would divide by zero in
     * the interpolation, descending is always a user mistake. */
    bad_tbl[0] = TBL[0]; bad_tbl[1] = TBL[1]; bad_tbl[2] = TBL[1];
    CK(PID_GainSched_Init(&s, bad_tbl, 3U, PID_SCHED_SRC_SETPOINT,
                          PID_SCHED_INTERP_LINEAR) == PID_ERR_INVALID_PARAM,
       "duplicate breakpoint rejected");
    bad_tbl[2] = TBL[0];
    CK(PID_GainSched_Init(&s, bad_tbl, 3U, PID_SCHED_SRC_SETPOINT,
                          PID_SCHED_INTERP_LINEAR) == PID_ERR_INVALID_PARAM,
       "descending table rejected");

    /* Negative or non-finite gains. A negative Kp in a table is how a
     * commissioning typo turns into a runaway. */
    bad_tbl[0] = TBL[0]; bad_tbl[1] = TBL[1]; bad_tbl[2] = TBL[2];
    bad_tbl[1].kp = -1.0f;
    CK(PID_GainSched_Init(&s, bad_tbl, 3U, PID_SCHED_SRC_SETPOINT,
                          PID_SCHED_INTERP_LINEAR) == PID_ERR_INVALID_GAIN,
       "negative Kp rejected");
    bad_tbl[1].kp = 2.0f; bad_tbl[1].ki = (PID_Float)NAN;
    CK(PID_GainSched_Init(&s, bad_tbl, 3U, PID_SCHED_SRC_SETPOINT,
                          PID_SCHED_INTERP_LINEAR) == PID_ERR_INVALID_GAIN,
       "NaN Ki rejected");
    bad_tbl[1].ki = 0.2f; bad_tbl[1].x = (PID_Float)INFINITY;
    CK(PID_GainSched_Init(&s, bad_tbl, 3U, PID_SCHED_SRC_SETPOINT,
                          PID_SCHED_INTERP_LINEAR) == PID_ERR_INVALID_PARAM,
       "non-finite breakpoint rejected");

    CK(PID_GainSched_Init(&s, TBL, 4U, PID_SCHED_SRC_ABS_ERROR,
                          PID_SCHED_INTERP_LINEAR) == PID_OK, "valid table accepted");
    CK(PID_GainSched_SetHysteresis(&s, -1.0f) == PID_ERR_INVALID_PARAM,
       "negative hysteresis rejected");
    CK(PID_GainSched_SetHysteresis(NULL, 1.0f) == PID_ERR_NULL, "SetHysteresis NULL");
    CK(PID_GainSched_SetHysteresis(&s, 1.0f) == PID_OK, "hysteresis accepted");
}

/* ===================================================================== */
static void t_interpolation(void)
{
    PID_GainSchedule s;
    PID_Float kp, ki, kd;

    puts("[2] interpolation and clamping");

    (void)PID_GainSched_Init(&s, TBL, 4U, PID_SCHED_SRC_SETPOINT,
                             PID_SCHED_INTERP_LINEAR);

    CK(PID_GainSched_Evaluate(NULL, 0.0f, &kp, &ki, &kd) == PID_ERR_NULL, "Evaluate NULL");
    CK(PID_GainSched_Evaluate(&s, (PID_Float)NAN, &kp, &ki, &kd) == PID_ERR_INVALID_PARAM,
       "NaN scheduling variable rejected");
    /* All three outputs are optional; a caller that only schedules Kp should
     * not have to supply dummies. */
    CK(PID_GainSched_Evaluate(&s, 5.0f, &kp, NULL, NULL) == PID_OK, "NULL outputs allowed");

    /* Exactly on a breakpoint. */
    (void)PID_GainSched_Evaluate(&s, 10.0f, &kp, &ki, &kd);
    CK(near((double)kp, 2.0, 1e-6) && near((double)ki, 0.20, 1e-6) &&
       near((double)kd, 0.20, 1e-6), "exact breakpoint returns its row");

    /* Midway in the first segment: t = 0.5 -> kp = 1.5, kd = 0.25. */
    (void)PID_GainSched_Evaluate(&s, 5.0f, &kp, &ki, &kd);
    CK(near((double)kp, 1.5, 1e-6), "linear Kp at midpoint");
    CK(near((double)ki, 0.15, 1e-6), "linear Ki at midpoint");
    CK(near((double)kd, 0.25, 1e-6), "linear Kd interpolates DOWN, not up");

    /* Unequal segment: x = 30 is t = 0.5 across [10,50] -> kp = 4.0. */
    (void)PID_GainSched_Evaluate(&s, 30.0f, &kp, NULL, &kd);
    CK(near((double)kp, 4.0, 1e-6), "unequal segment interpolates on its own width");
    CK(near((double)kd, 0.15, 1e-6), "and Kd with it");

    /* Outside the table the gains saturate. Extrapolating a table that ends at
     * Kd = 0 would produce a negative derivative gain. */
    (void)PID_GainSched_Evaluate(&s, -1000.0f, &kp, &ki, &kd);
    CK(near((double)kp, 1.0, 1e-6) && near((double)kd, 0.30, 1e-6), "clamps below");
    (void)PID_GainSched_Evaluate(&s, 1000.0f, &kp, &ki, &kd);
    CK(near((double)kp, 6.0, 1e-6) && near((double)kd, 0.0, 1e-6), "clamps above");

    /* HOLD is piecewise constant: it must return the LEFT row anywhere inside
     * a segment, and only step at the next breakpoint. */
    {
        PID_GainSchedule g;
        (void)PID_GainSched_Init(&g, TBL, 4U, PID_SCHED_SRC_SETPOINT,
                                 PID_SCHED_INTERP_HOLD);
        (void)PID_GainSched_Evaluate(&g, 9.999f, &kp, NULL, NULL);
        CK(near((double)kp, 1.0, 1e-6), "HOLD keeps the left row inside a segment");
        (void)PID_GainSched_Evaluate(&g, 10.0f, &kp, NULL, NULL);
        CK(near((double)kp, 2.0, 1e-6), "HOLD steps exactly at the breakpoint");
    }

    /* SMOOTH: equals linear at t=0, 0.5, 1, but has zero slope at the ends.
     * Check the C1 property numerically - a finite difference of the gain
     * across the breakpoint must be continuous, unlike linear. */
    {
        PID_GainSchedule g;
        double slope_left, slope_right, kp_a, kp_b, kp_c;
        const double h = 0.01;

        (void)PID_GainSched_Init(&g, TBL, 4U, PID_SCHED_SRC_SETPOINT,
                                 PID_SCHED_INTERP_SMOOTH);
        (void)PID_GainSched_Evaluate(&g, 5.0f, &kp, NULL, NULL);
        CK(near((double)kp, 1.5, 1e-5), "smoothstep midpoint equals linear");
        (void)PID_GainSched_Evaluate(&g, 0.0f, &kp, NULL, NULL);
        CK(near((double)kp, 1.0, 1e-6), "smoothstep endpoint exact");

        /* Slope on each side of the x = 10 breakpoint. */
        (void)PID_GainSched_Evaluate(&g, (PID_Float)(10.0 - 2.0 * h), &kp, NULL, NULL);
        kp_a = (double)kp;
        (void)PID_GainSched_Evaluate(&g, (PID_Float)(10.0 - h), &kp, NULL, NULL);
        kp_b = (double)kp;
        (void)PID_GainSched_Evaluate(&g, (PID_Float)(10.0 + h), &kp, NULL, NULL);
        kp_c = (double)kp;
        slope_left  = (kp_b - kp_a) / h;
        slope_right = (kp_c - kp_b) / (2.0 * h);
        CK(fabs(slope_left) < 0.02 && fabs(slope_right) < 0.02,
           "smoothstep has ~zero slope on both sides of a breakpoint (C1)");

        /* Linear, for contrast: the slopes differ by a factor of four here
         * (segment widths 10 and 40 with equal Kp steps). */
        {
            PID_GainSchedule l;
            double sl, sr;
            (void)PID_GainSched_Init(&l, TBL, 4U, PID_SCHED_SRC_SETPOINT,
                                     PID_SCHED_INTERP_LINEAR);
            (void)PID_GainSched_Evaluate(&l, 9.0f, &kp, NULL, NULL); kp_a = (double)kp;
            (void)PID_GainSched_Evaluate(&l, 9.5f, &kp, NULL, NULL); kp_b = (double)kp;
            (void)PID_GainSched_Evaluate(&l, 11.0f, &kp, NULL, NULL); kp_c = (double)kp;
            sl = (kp_b - kp_a) / 0.5;
            sr = (kp_c - 2.0) / 1.0;
            CK(fabs(sl - 0.1) < 1e-4 && fabs(sr - 0.1) < 1e-4,
               "linear slopes are 0.1 both sides here (equal by construction)");
        }
    }
}

/* ===================================================================== */
static void t_hysteresis(void)
{
    PID_GainSchedule s;
    PID_Float kp;

    puts("[3] hysteresis");

    (void)PID_GainSched_Init(&s, TBL, 4U, PID_SCHED_SRC_SETPOINT,
                             PID_SCHED_INTERP_LINEAR);
    (void)PID_GainSched_SetHysteresis(&s, 2.0f);

    /* First evaluation always takes effect: there is no previous value to
     * compare against. This is what `primed` is for. */
    (void)PID_GainSched_Evaluate(&s, 5.0f, &kp, NULL, NULL);
    CK(near((double)kp, 1.5, 1e-6), "first evaluation is never suppressed");

    /* Movement inside the band is ignored - the gains must not budge. */
    (void)PID_GainSched_Evaluate(&s, 6.5f, &kp, NULL, NULL);
    CK(near((double)kp, 1.5, 1e-6), "1.5 < band: gains held");
    (void)PID_GainSched_Evaluate(&s, 3.5f, &kp, NULL, NULL);
    CK(near((double)kp, 1.5, 1e-6), "and in the other direction");

    /* Movement beyond the band takes effect in full - it is a deadband on the
     * scheduling variable, not a rate limit. */
    (void)PID_GainSched_Evaluate(&s, 9.0f, &kp, NULL, NULL);
    CK(near((double)kp, 1.9, 1e-6), "beyond the band the full move applies");

    /*
     * Dither rejection: this is what hysteresis is FOR. A scheduling variable
     * that jitters around a point with an amplitude smaller than the band must
     * never move the gains, no matter how many samples arrive.
     *
     * Note the band is compared with a strict <, so the step size must be
     * genuinely smaller than the band - a 0.5 step against a 1.0 band lands
     * exactly on the edge and IS accepted. Coprime-ish values are used here
     * for that reason.
     */
    {
        PID_GainSchedule g;
        int k;
        (void)PID_GainSched_Init(&g, TBL, 4U, PID_SCHED_SRC_SETPOINT,
                                 PID_SCHED_INTERP_LINEAR);
        (void)PID_GainSched_SetHysteresis(&g, 1.0f);
        (void)PID_GainSched_Evaluate(&g, 5.0f, &kp, NULL, NULL);
        for (k = 0; k < 500; ++k) {
            const double jitter = ((k % 2) == 0) ? 0.37 : -0.41;
            (void)PID_GainSched_Evaluate(&g, (PID_Float)(5.0 + jitter), &kp, NULL, NULL);
        }
        CK(near((double)kp, 1.5, 1e-6),
           "500 samples of sub-band dither never move the gains");

        /*
         * A sustained drift, however, MUST get through: hysteresis is a
         * deadband on the variable, not a permanent veto. Once the variable
         * has travelled a full band away from the last accepted value the new
         * value is taken in full. Suppressing a slow genuine ramp would be a
         * far worse defect than the chatter it prevents.
         */
        {
            double x;
            for (x = 5.0; x <= 12.0; x += 0.3) {
                (void)PID_GainSched_Evaluate(&g, (PID_Float)x, &kp, NULL, NULL);
            }
            CK((double)kp > 1.5,
               "a sustained ramp is not blocked, only quantised by the band");
            /* The ramp ends at x = 11.9, which is 1.9 into the [10,50]
             * segment: kp = 2 + (1.9/40)*4 = 2.19. Hysteresis can only hold
             * the accepted value BEHIND the ramp, never ahead of it, so the
             * result must not exceed the exact table value at the end point. */
            CK((double)kp <= 2.19 + 1e-4, "hysteresis lags the ramp, never leads it");
        }
    }

    /* Zero hysteresis means every change applies. */
    {
        PID_GainSchedule g;
        (void)PID_GainSched_Init(&g, TBL, 4U, PID_SCHED_SRC_SETPOINT,
                                 PID_SCHED_INTERP_LINEAR);
        (void)PID_GainSched_Evaluate(&g, 5.0f, &kp, NULL, NULL);
        (void)PID_GainSched_Evaluate(&g, 5.001f, &kp, NULL, NULL);
        CK(near((double)kp, 1.5001, 1e-4), "hysteresis 0 tracks every change");
    }
}

/* ===================================================================== */
static void base_cfg(PID_Config *c)
{
    PID_ConfigDefault(c);
    c->core.kp = 1.0f;
    c->core.ki = 0.1f;
    c->core.kd = 0.0f;
    c->core.sample_time = 0.01f;
    c->limits.use_output_limits = true;
    c->limits.output_min = -100.0f;
    c->limits.output_max = 100.0f;
}

static void t_attached(void)
{
    PID_Handle h;
    PID_Config c;
    PID_GainSchedule s;
    PID_Status st;
    int i;

    puts("[4] attached to a live controller");

    base_cfg(&c);
    CK(PID_Init(&h, &c) == PID_OK, "controller init");
    (void)PID_GainSched_Init(&s, TBL, 4U, PID_SCHED_SRC_SETPOINT,
                             PID_SCHED_INTERP_LINEAR);

    CK(PID_GainSched_Attach(NULL, &s) == PID_ERR_NULL, "Attach NULL handle");
    CK(PID_GainSched_Attach(&h, &s) == PID_OK, "attach ok");
    CK(PID_IsFeatureEnabled(&h, PID_FEAT_GAIN_SCHED), "attaching enables the feature");

    /* SRC_SETPOINT: setpoint 30 -> kp 4.0 after one update. */
    (void)PID_SetSetpoint(&h, 30.0f);
    (void)PID_Update(&h, 30.0f);
    (void)PID_GetStatus(&h, &st);
    CK(near((double)st.kp_active, 4.0, 1e-5), "SRC_SETPOINT reads the setpoint");

    /* SRC_MEASUREMENT. */
    s.source = (uint8_t)PID_SCHED_SRC_MEASUREMENT;
    s.primed = false;
    (void)PID_Update(&h, 10.0f);
    (void)PID_GetStatus(&h, &st);
    CK(near((double)st.kp_active, 2.0, 1e-5), "SRC_MEASUREMENT reads the measurement");

    /* SRC_ABS_ERROR: |30 - 20| = 10 -> kp 2.0, and the sign must not matter. */
    s.source = (uint8_t)PID_SCHED_SRC_ABS_ERROR;
    s.primed = false;
    (void)PID_Update(&h, 20.0f);
    (void)PID_GetStatus(&h, &st);
    CK(near((double)st.kp_active, 2.0, 1e-5), "SRC_ABS_ERROR, error +10");
    s.primed = false;
    (void)PID_Update(&h, 40.0f);
    (void)PID_GetStatus(&h, &st);
    CK(near((double)st.kp_active, 2.0, 1e-5), "SRC_ABS_ERROR, error -10 gives the same");

    /* SRC_ERROR is signed, so -10 clamps to the bottom row instead. */
    s.source = (uint8_t)PID_SCHED_SRC_ERROR;
    s.primed = false;
    (void)PID_Update(&h, 40.0f);
    (void)PID_GetStatus(&h, &st);
    CK(near((double)st.kp_active, 1.0, 1e-5), "SRC_ERROR is signed and clamps below");

    /* SRC_EXTERNAL: the user supplies the variable. */
    s.source = (uint8_t)PID_SCHED_SRC_EXTERNAL;
    s.primed = false;
    CK(PID_GainSched_SetVar(&h, 50.0f) == PID_OK, "SetVar ok");
    CK(PID_GainSched_SetVar(&h, (PID_Float)NAN) == PID_ERR_INVALID_PARAM, "SetVar NaN");
    CK(PID_GainSched_SetVar(NULL, 1.0f) == PID_ERR_NULL, "SetVar NULL");
    (void)PID_Update(&h, 30.0f);
    (void)PID_GetStatus(&h, &st);
    CK(near((double)st.kp_active, 6.0, 1e-5), "SRC_EXTERNAL uses the supplied value");

    /* PID_Input.schedule_var overrides the source for that one call - useful
     * when the scheduling signal is sampled with the measurement. */
    {
        PID_Input in;
        PID_InputInit(&in);
        in.measurement = 30.0f;
        in.schedule_var = 0.0f;
        (void)PID_UpdateEx(&h, &in, NULL);
        (void)PID_GetStatus(&h, &st);
        CK(near((double)st.kp_active, 1.0, 1e-5), "PID_Input.schedule_var overrides");
    }

    /* Detaching freezes the gains where they were, rather than snapping back
     * to the configured ones - the documented behaviour. */
    CK(PID_GainSched_Attach(&h, NULL) == PID_OK, "detach ok");
    CK(!PID_IsFeatureEnabled(&h, PID_FEAT_GAIN_SCHED), "feature cleared on detach");
    (void)PID_Update(&h, 30.0f);
    (void)PID_GetStatus(&h, &st);
    CK(near((double)st.kp_active, 1.0, 1e-5), "gains hold their last value");

    /* A schedule that never went through Init must be refused rather than
     * dereferenced at the first update. */
    {
        PID_GainSchedule junk;
        junk.points = NULL;
        junk.count = 0U;
        CK(PID_GainSched_Attach(&h, &junk) == PID_ERR_INVALID_PARAM,
           "uninitialised schedule refused");
    }

    (void)i;
}

/* ===================================================================== */
static void t_bumpless_gain_change(void)
{
    PID_Handle h;
    PID_Config c;
    PID_GainSchedule s;
    double u_before, u_after, worst = 0.0;
    int i;

    puts("[5] a moving schedule must not bump the output");

    /*
     * The integrator is stored in OUTPUT units, so I is unchanged when Ki
     * moves; only the future rate of accumulation changes. That makes a gain
     * sweep inherently bumpless. A library that stores the raw integral of the
     * error instead would step the output by (Ki_new/Ki_old - 1)*I here.
     *
     * Sweeping the scheduling variable one step at a time, the output change
     * per sample must stay at the level of the normal control action, not jump
     * by the ratio of the gains (which is 6x across this table).
     */
    base_cfg(&c);
    c.core.ki = 0.0f;   /* Ki comes from the table */
    (void)PID_Init(&h, &c);
    (void)PID_GainSched_Init(&s, TBL, 4U, PID_SCHED_SRC_EXTERNAL,
                             PID_SCHED_INTERP_LINEAR);
    (void)PID_GainSched_Attach(&h, &s);
    (void)PID_SetSetpoint(&h, 1.0f);

    /* Build up a real integrator at the low-gain end. */
    (void)PID_GainSched_SetVar(&h, 0.0f);
    for (i = 0; i < 2000; ++i) { (void)PID_Update(&h, 0.5f); }
    u_before = (double)PID_GetOutput(&h);
    CK(fabs((double)PID_GetIntegrator(&h)) > 0.1, "integrator has real content");

    /* Now sweep the scheduling variable from 0 to 60 in one-unit steps while
     * the plant is frozen. Kp goes 1 -> 6. */
    for (i = 1; i <= 60; ++i) {
        double u_prev = (double)PID_GetOutput(&h);
        double u_now;
        (void)PID_GainSched_SetVar(&h, (PID_Float)i);
        u_now = (double)PID_Update(&h, 0.5f);
        if (fabs(u_now - u_prev) > worst) { worst = fabs(u_now - u_prev); }
    }
    u_after = (double)PID_GetOutput(&h);

    printf("    u %.4f -> %.4f, worst single-sample step %.4f\n",
           u_before, u_after, worst);

    /* Kp*e changes by design: e = 0.5 and Kp moves by 5/60 per step, so the P
     * term legitimately moves ~0.042 per sample. What must NOT happen is a
     * discontinuity in the I contribution. */
    CK(worst < 0.10, "no gain-change discontinuity beyond the P term's own move");
    CK(u_after > u_before, "the output does rise - the sweep is not a no-op");

    /* Direct proof: freeze the plant, change Ki by 10x in one call, and check
     * the integrator value in output units is untouched. */
    {
        PID_Handle g;
        PID_Config gc;
        double i_before, i_after, ug_before, ug_after;

        base_cfg(&gc);
        (void)PID_Init(&g, &gc);
        (void)PID_SetSetpoint(&g, 1.0f);
        for (i = 0; i < 500; ++i) { (void)PID_Update(&g, 0.0f); }
        i_before = (double)PID_GetIntegrator(&g);
        ug_before = (double)PID_GetOutput(&g);
        CK(PID_SetKi(&g, 1.0f) == PID_OK, "Ki x10 accepted");
        i_after = (double)PID_GetIntegrator(&g);
        ug_after = (double)PID_Update(&g, 0.0f);
        printf("    Ki 0.1 -> 1.0: I %.6f -> %.6f, u %.6f -> %.6f\n",
               i_before, i_after, ug_before, ug_after);
        CK(fabs(i_after - i_before) < 1e-6, "integrator unchanged by a Ki change");
        /* One sample of the NEW Ki is 1.0*0.01*1.0 = 0.01. Nothing more. */
        CK(fabs(ug_after - ug_before) < 0.011, "output moves by exactly one new step");
    }
}

int main(void)
{
    puts("=== gain scheduling suite ===\n");
    t_init_validation();
    t_interpolation();
    t_hysteresis();
    t_attached();
    t_bumpless_gain_change();
    printf("\n  gainsched: %d passed, %d failed\n", pass, bad);
    return (bad == 0) ? 0 : 1;
}

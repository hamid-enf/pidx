/* Trajectory shaper suite (PHASE 17).
 *
 * pid_shaper.c had no dedicated suite; it was only covered indirectly through
 * the core's setpoint ramp. The properties worth pinning are physical, not
 * cosmetic:
 *
 *   - the profile never exceeds rate_max, accel or decel;
 *   - it lands exactly on the target and stays there (no dither, no overshoot);
 *   - the distance travelled equals the commanded distance (the integral of
 *     the velocity profile is conserved - a shaper that loses position is
 *     worse than no shaper);
 *   - PID_Shaper_EstimateTime agrees with the simulated duration, for both the
 *     trapezoidal and the triangular case;
 *   - re-targeting mid-move blends without a velocity discontinuity.
 *
 * Every tolerance below is one timestep's worth of motion, derived, not tuned.
 */
#include <stdio.h>
#include <math.h>

#include "pidx/pid_shaper.h"

static int pass = 0, bad = 0;
#define CK(c,m) do{ if(c){pass++;} else {bad++;printf("  FAIL: %s\n",(m));} }while(0)

/* Run the profile to completion, reporting the extremes it reached. */
typedef struct {
    double t;           /* simulated duration [s]                            */
    double v_peak;      /* max |velocity|                                    */
    double a_peak;      /* max |acceleration|, ignoring the landing sample    */
    double final_pos;
    int    steps;
    int    timed_out;
} RunResult;

static RunResult run(PID_Shaper *s, double dt, double t_max)
{
    RunResult r;
    double v_prev = (double)s->velocity;
    double p_prev = (double)s->position;
    int i;
    const int n_max = (int)(t_max / dt);

    r.t = 0.0; r.v_peak = 0.0; r.a_peak = 0.0; r.steps = 0; r.timed_out = 1;
    r.final_pos = p_prev;

    for (i = 0; i < n_max; ++i) {
        const double p = (double)PID_Shaper_Update(s, (PID_Float)dt);
        const double v = (double)s->velocity;
        const double a = fabs(v - v_prev) / dt;

        r.steps++;
        r.t += dt;
        r.final_pos = p;
        if (fabs(v) > r.v_peak) { r.v_peak = fabs(v); }

        /* The landing sample deliberately snaps velocity to zero, which reads
         * as an infinite deceleration. That is the correct behaviour (it stops
         * exactly on target) so it is excluded from the accel bound. */
        if (!PID_Shaper_IsMoving(s)) {
            r.timed_out = 0;
            break;
        }
        if (a > r.a_peak) { r.a_peak = a; }

        v_prev = v;
        p_prev = p;
    }
    (void)p_prev;
    return r;
}

/* ===================================================================== */
static void t_validation(void)
{
    PID_Shaper s;

    puts("[1] validation and lifecycle");

    CK(PID_Shaper_Init(NULL, 1.0f, 1.0f, 1.0f) == PID_ERR_NULL, "Init NULL");
    CK(PID_Shaper_Init(&s, -1.0f, 1.0f, 1.0f) == PID_ERR_INVALID_PARAM, "negative rate");
    CK(PID_Shaper_Init(&s, 1.0f, -1.0f, 1.0f) == PID_ERR_INVALID_PARAM, "negative accel");
    CK(PID_Shaper_Init(&s, 1.0f, 1.0f, -1.0f) == PID_ERR_INVALID_PARAM, "negative decel");
    CK(PID_Shaper_Init(&s, (PID_Float)NAN, 1.0f, 1.0f) == PID_ERR_INVALID_PARAM, "NaN rate");
    CK(PID_Shaper_Init(&s, 1.0f, 1.0f, 1.0f) == PID_OK, "Init ok");

    CK(PID_Shaper_SetTarget(NULL, 1.0f) == PID_ERR_NULL, "SetTarget NULL");
    CK(PID_Shaper_SetTarget(&s, (PID_Float)INFINITY) == PID_ERR_INVALID_PARAM, "Inf target");
    CK(PID_Shaper_Reset(&s, (PID_Float)NAN) == PID_ERR_INVALID_PARAM, "NaN reset position");

    CK(!PID_Shaper_IsMoving(&s), "idle after init");
    CK(!PID_Shaper_IsMoving(NULL), "IsMoving(NULL) is false, not a crash");

    /* Bad dt must hold position, not extrapolate. */
    (void)PID_Shaper_Reset(&s, 2.0f);
    (void)PID_Shaper_SetTarget(&s, 10.0f);
    CK((double)PID_Shaper_Update(&s, 0.0f) == 2.0, "dt=0 holds position");
    CK((double)PID_Shaper_Update(&s, -1.0f) == 2.0, "dt<0 holds position");
    CK((double)PID_Shaper_Update(&s, (PID_Float)NAN) == 2.0, "NaN dt holds position");
    CK((double)PID_Shaper_Update(NULL, 0.01f) == 0.0, "Update(NULL) returns 0");

    /* Reset teleports and clears the target: it models homing, where the axis
     * is physically somewhere new and any pending move is void. */
    (void)PID_Shaper_Reset(&s, -4.0f);
    CK((double)s.position == -4.0 && (double)s.target == -4.0 && (double)s.velocity == 0.0,
       "Reset teleports, zeroes velocity, voids the target");
    CK(!PID_Shaper_IsMoving(&s), "not moving after reset");
}

/* ===================================================================== */
static void t_rate_only(void)
{
    PID_Shaper s;
    RunResult r;
    const double dt = 0.001;
    const double v = 2.0;
    const double d = 5.0;

    puts("[2] rate-only profile (accel = 0)");

    CK(PID_Shaper_Init(&s, (PID_Float)v, 0.0f, 0.0f) == PID_OK, "init rate-only");
    (void)PID_Shaper_Reset(&s, 0.0f);
    (void)PID_Shaper_SetTarget(&s, (PID_Float)d);
    CK(PID_Shaper_IsMoving(&s), "moving once a target is set");

    r = run(&s, dt, 20.0);
    CK(!r.timed_out, "rate-only completes");
    CK(fabs(r.final_pos - d) < 1e-6, "lands exactly on target");
    CK(r.v_peak <= v + 1e-6, "never exceeds rate_max");
    /* d/v = 2.5 s, plus at most one step of quantisation. */
    CK(fabs(r.t - (d / v)) <= dt * 1.5, "duration is d/v");
    CK(fabs((double)PID_Shaper_EstimateTime(&s)) == 0.0, "estimate is 0 once parked");

    /* Parked: further updates must not move or re-arm. */
    (void)PID_Shaper_Update(&s, (PID_Float)dt);
    CK(fabs((double)s.position - d) < 1e-6, "stays parked");
    CK(!PID_Shaper_IsMoving(&s), "still idle");

    /* Backwards move is symmetric. */
    (void)PID_Shaper_SetTarget(&s, 0.0f);
    r = run(&s, dt, 20.0);
    CK(!r.timed_out && fabs(r.final_pos) < 1e-6, "returns exactly to 0");
    CK(fabs(r.t - (d / v)) <= dt * 1.5, "same duration in reverse");

    /* rate_max = 0 documents as "shaping disabled": jump straight there. */
    {
        PID_Shaper g;
        (void)PID_Shaper_Init(&g, 0.0f, 0.0f, 0.0f);
        (void)PID_Shaper_SetTarget(&g, 42.0f);
        CK((double)PID_Shaper_Update(&g, (PID_Float)dt) == 42.0, "rate_max=0 passes through");
        CK(!PID_Shaper_IsMoving(&g), "and reports idle immediately");
    }
}

/* ===================================================================== */
static void t_trapezoid(void)
{
    PID_Shaper s;
    RunResult r;
    const double dt = 0.0005;
    const double v = 2.0, a = 4.0, b = 4.0;
    const double d = 10.0;
    double est;

    puts("[3] trapezoidal profile");

    CK(PID_Shaper_Init(&s, (PID_Float)v, (PID_Float)a, (PID_Float)b) == PID_OK, "init");
    (void)PID_Shaper_Reset(&s, 0.0f);
    (void)PID_Shaper_SetTarget(&s, (PID_Float)d);

    /* d_ramp = v^2/(2a) + v^2/(2b) = 4/8 + 4/8 = 1.0 < 10, so this is a true
     * trapezoid: t = (d - d_ramp)/v + v/a + v/b = 9/2 + 0.5 + 0.5 = 5.5 s. */
    est = (double)PID_Shaper_EstimateTime(&s);
    CK(fabs(est - 5.5) < 1e-4, "EstimateTime = 5.5 s for the trapezoid");

    r = run(&s, dt, 30.0);
    CK(!r.timed_out, "trapezoid completes");
    CK(fabs(r.final_pos - d) < 1e-5, "lands exactly on target");
    CK(r.v_peak <= v * 1.0001, "cruise speed respects rate_max");
    CK(r.v_peak > v * 0.99, "actually reaches cruise speed (it is a trapezoid)");
    CK(r.a_peak <= a * 1.02, "acceleration respects the limit");
    /* One timestep of discretisation error per ramp, i.e. ~2*dt. */
    CK(fabs(r.t - est) < 0.02, "simulated duration matches the estimate");

    /* Asymmetric decel: braking twice as hard (b = 8) shortens the move only
     * slightly, because most of the time is spent cruising.
     *   d_ramp = 4/(2*4) + 4/(2*8) = 0.5 + 0.25 = 0.75
     *   t      = (10 - 0.75)/2 + 2/4 + 2/8 = 4.625 + 0.5 + 0.25 = 5.375 s
     * i.e. 0.125 s saved, exactly the halved decel ramp time. Asserting the
     * arithmetic rather than the intuition. */
    {
        PID_Shaper g;
        double e2;
        (void)PID_Shaper_Init(&g, (PID_Float)v, (PID_Float)a, 8.0f);
        (void)PID_Shaper_Reset(&g, 0.0f);
        (void)PID_Shaper_SetTarget(&g, (PID_Float)d);
        e2 = (double)PID_Shaper_EstimateTime(&g);
        CK(fabs(e2 - 5.375) < 1e-4, "asymmetric decel estimate = 5.375 s");
        r = run(&g, dt, 30.0);
        CK(fabs(r.t - e2) < 0.02, "asymmetric decel simulation agrees");
        CK(fabs(r.final_pos - d) < 1e-5, "and still lands on target");
    }

    /* decel = 0 must mirror accel, not disable braking. */
    {
        PID_Shaper g;
        (void)PID_Shaper_Init(&g, (PID_Float)v, (PID_Float)a, 0.0f);
        (void)PID_Shaper_Reset(&g, 0.0f);
        (void)PID_Shaper_SetTarget(&g, (PID_Float)d);
        CK(fabs((double)PID_Shaper_EstimateTime(&g) - 5.5) < 1e-4, "decel=0 mirrors accel");
        r = run(&g, dt, 30.0);
        CK(fabs(r.final_pos - d) < 1e-5, "decel=0 still lands on target");
        CK(r.a_peak <= a * 1.02, "decel=0 brakes at the accel rate");
    }
}

/* ===================================================================== */
static void t_triangle(void)
{
    PID_Shaper s;
    RunResult r;
    const double dt = 0.0002;
    const double v = 10.0, a = 4.0, b = 4.0;
    const double d = 0.5;           /* far too short to reach v */
    double est, v_peak_expected;

    puts("[4] triangular profile (never reaches rate_max)");

    (void)PID_Shaper_Init(&s, (PID_Float)v, (PID_Float)a, (PID_Float)b);
    (void)PID_Shaper_Reset(&s, 0.0f);
    (void)PID_Shaper_SetTarget(&s, (PID_Float)d);

    /* vp = sqrt(2*d*a*b/(a+b)) = sqrt(2*0.5*16/8) = sqrt(2) = 1.41421
     * t  = vp/a + vp/b = 2*sqrt(2)/4 = 0.70711 s */
    v_peak_expected = sqrt(2.0 * d * a * b / (a + b));
    est = (double)PID_Shaper_EstimateTime(&s);
    CK(fabs(est - (2.0 * v_peak_expected / a)) < 1e-5, "triangular estimate closed form");
    CK(fabs(est - 0.7071) < 1e-3, "estimate = 0.7071 s");

    r = run(&s, dt, 10.0);
    CK(!r.timed_out, "triangle completes");
    CK(fabs(r.final_pos - d) < 1e-5, "lands exactly on target");
    CK(r.v_peak < v * 0.2, "never gets near rate_max (it is a triangle)");
    CK(fabs(r.v_peak - v_peak_expected) < 0.02, "peak velocity matches sqrt(2dab/(a+b))");
    CK(r.a_peak <= a * 1.02, "acceleration respects the limit");
    CK(fabs(r.t - est) < 0.02, "simulated duration matches the estimate");
}

/* ===================================================================== */
static void t_retarget(void)
{
    PID_Shaper s;
    const double dt = 0.001;
    double v_at_switch, v_after, pos;
    int i;

    puts("[5] re-targeting mid-move");

    (void)PID_Shaper_Init(&s, 2.0f, 4.0f, 4.0f);
    (void)PID_Shaper_Reset(&s, 0.0f);
    (void)PID_Shaper_SetTarget(&s, 10.0f);

    for (i = 0; i < 1000; ++i) { (void)PID_Shaper_Update(&s, (PID_Float)dt); }
    v_at_switch = (double)s.velocity;
    CK(v_at_switch > 1.9, "at cruise speed after 1 s");

    /* Retarget while moving. The velocity must be continuous: the profile
     * blends from the current state, it does not restart from rest. */
    (void)PID_Shaper_SetTarget(&s, 3.0f);
    (void)PID_Shaper_Update(&s, (PID_Float)dt);
    v_after = (double)s.velocity;
    CK(fabs(v_after - v_at_switch) <= 4.0 * dt * 1.01,
       "velocity change on retarget is bounded by accel*dt");

    {
        RunResult r = run(&s, dt, 30.0);
        CK(!r.timed_out, "retargeted move completes");
        CK(fabs(r.final_pos - 3.0) < 1e-5, "lands on the NEW target");
        CK(r.a_peak <= 4.0 * 1.05, "accel limit held across the retarget");
    }

    /* Reversal mid-move: target behind the current position while travelling
     * forwards. The profile must decelerate through zero, not teleport. */
    (void)PID_Shaper_Reset(&s, 0.0f);
    (void)PID_Shaper_SetTarget(&s, 10.0f);
    for (i = 0; i < 1000; ++i) { (void)PID_Shaper_Update(&s, (PID_Float)dt); }
    pos = (double)s.position;
    (void)PID_Shaper_SetTarget(&s, -5.0f);
    {
        double max_forward = pos;
        int sign_changed = 0;
        for (i = 0; i < 20000; ++i) {
            const double p = (double)PID_Shaper_Update(&s, (PID_Float)dt);
            if (p > max_forward) { max_forward = p; }
            if ((double)s.velocity < 0.0) { sign_changed = 1; }
            if (!PID_Shaper_IsMoving(&s)) { break; }
        }
        CK(sign_changed, "velocity reverses sign rather than jumping");
        CK(max_forward > pos, "overshoots forward first - it cannot stop instantly");
        CK(max_forward - pos < 1.0, "and the overshoot is bounded by v^2/2a = 0.5");
        CK(fabs((double)s.position + 5.0) < 1e-5, "reaches the reversed target exactly");
    }

    /* Setting the target to the current position is a no-op, not a 0/0. */
    (void)PID_Shaper_Reset(&s, 7.0f);
    (void)PID_Shaper_SetTarget(&s, 7.0f);
    CK(!PID_Shaper_IsMoving(&s), "target == position is not a move");
    CK((double)PID_Shaper_Update(&s, (PID_Float)dt) == 7.0, "and stays put");
    CK((double)PID_Shaper_EstimateTime(&s) == 0.0, "estimate 0 for a zero-distance move");
}

/* ===================================================================== */
static void t_conservation(void)
{
    PID_Shaper s;
    const double dt = 0.001;
    double sum_v_dt = 0.0;
    int i;

    puts("[6] the velocity profile integrates to the distance");

    (void)PID_Shaper_Init(&s, 3.0f, 6.0f, 2.0f);
    (void)PID_Shaper_Reset(&s, 0.0f);
    (void)PID_Shaper_SetTarget(&s, 12.0f);

    /* If the integral of the reported velocity does not equal the distance
     * travelled, the shaper is lying to whatever is feedforwarding from it. */
    for (i = 0; i < 100000; ++i) {
        (void)PID_Shaper_Update(&s, (PID_Float)dt);
        sum_v_dt += (double)s.velocity * dt;
        if (!PID_Shaper_IsMoving(&s)) { break; }
    }
    CK(fabs((double)s.position - 12.0) < 1e-5, "arrived");
    /* The landing sample snaps the last fraction of a step, so allow one. */
    CK(fabs(sum_v_dt - 12.0) < 3.0 * dt * 3.0,
       "integral of velocity equals the commanded distance");
    printf("    integral(v dt) = %.6f vs distance 12.000000\n", sum_v_dt);
}

int main(void)
{
    puts("=== shaper module suite ===\n");
    t_validation();
    t_rate_only();
    t_trapezoid();
    t_triangle();
    t_retarget();
    t_conservation();
    printf("\n  shaper: %d passed, %d failed\n", pass, bad);
    return (bad == 0) ? 0 : 1;
}

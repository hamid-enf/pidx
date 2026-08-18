/**
 * @file    sim_common.c
 * @brief   Implementation of the shared simulation infrastructure.
 */
#include "sim_common.h"

#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "ex_plant.h"

/* ======================================================================== */
/* Metrics                                                                   */
/* ======================================================================== */

void sim_metrics_init(Sim_Metrics *m, double y0, double target, double band)
{
    memset(m, 0, sizeof(*m));
    m->y0 = y0;
    m->target = target;
    m->band = (band > 0.0) ? band : 0.02;
    m->peak = y0;
    m->y_final = y0;
    m->u_min = 1e300;
    m->u_max = -1e300;
}

void sim_metrics_update(Sim_Metrics *m, double y, double u, double dt)
{
    const double span = m->target - m->y0;
    const double e = m->target - y;
    const double ae = fabs(e);

    m->t += dt;
    m->n++;
    m->y_final = y;

    /* Rise time is measured on the commanded change, so it is meaningful even
     * when the loop never reaches the target. */
    if (span != 0.0) {
        const double frac = (y - m->y0) / span;
        if (!m->got_10 && (frac >= 0.1)) { m->t_10 = m->t; m->got_10 = true; }
        if (!m->got_90 && (frac >= 0.9)) {
            m->t_90 = m->t;
            m->got_90 = true;
            m->t_rise = m->t_90 - m->t_10;
        }
    }

    /* Overshoot and peak, signed by the direction of the step. */
    if (span >= 0.0) {
        if (y > m->peak) { m->peak = y; m->t_peak = m->t; }
    } else {
        if (y < m->peak) { m->peak = y; m->t_peak = m->t; }
    }
    if (span != 0.0) {
        const double os = (m->peak - m->target) / span * 100.0;
        m->overshoot = (os > 0.0) ? os : 0.0;
    }

    if (ae > m->e_peak_abs) { m->e_peak_abs = ae; }

    /* Settling time is the LAST exit from the band, not the first entry:
     * a response that enters the band and then leaves again has not settled. */
    if (span != 0.0) {
        if (ae > fabs(span) * m->band) { m->t_settle = m->t; }
    }

    m->iae  += ae * dt;
    m->ise  += e * e * dt;
    m->itae += m->t * ae * dt;

    if (m->primed) { m->u_tv += fabs(u - m->u_prev); }
    m->u_prev = u;
    m->primed = true;

    if (u < m->u_min) { m->u_min = u; }
    if (u > m->u_max) { m->u_max = u; }
}

double sim_metrics_sse(const Sim_Metrics *m)
{
    return m->target - m->y_final;
}

void sim_metrics_header(void)
{
    printf("  %-22s %8s %8s %8s %9s %9s %9s %8s\n",
           "case", "rise[s]", "sett[s]", "OS[%]", "IAE", "ITAE", "u_TV", "sse");
}

void sim_metrics_report(const Sim_Metrics *m, const char *label)
{
    printf("  %-22s %8.3f %8.3f %8.2f %9.3f %9.1f %9.2f %8.4f\n",
           label, m->t_rise, m->t_settle, m->overshoot,
           m->iae, m->itae, m->u_tv, sim_metrics_sse(m));
}

/* ======================================================================== */
/* CSV                                                                       */
/* ======================================================================== */

bool sim_csv_open(Sim_Csv *c, const char *path, const char *header)
{
    c->f = fopen(path, "w");
    if (c->f == NULL) {
        fprintf(stderr, "  ! cannot write %s\n", path);
        return false;
    }
    fprintf(c->f, "%s\n", header);
    return true;
}

void sim_csv_row(Sim_Csv *c, const char *fmt, ...)
{
    va_list ap;
    if ((c == NULL) || (c->f == NULL)) { return; }
    va_start(ap, fmt);
    (void)vfprintf(c->f, fmt, ap);
    va_end(ap);
    fputc('\n', c->f);
}

void sim_csv_close(Sim_Csv *c)
{
    if ((c != NULL) && (c->f != NULL)) {
        (void)fclose(c->f);
        c->f = NULL;
    }
}

bool sim_mkresults(void)
{
    /* 0755; EEXIST is success, which is the common case on a re-run. */
    if ((mkdir("results", 0755) != 0)) {
        FILE *probe = fopen("results/.probe", "w");
        if (probe == NULL) { return false; }
        (void)fclose(probe);
        (void)remove("results/.probe");
    }
    return true;
}

/* ======================================================================== */
/* Plant bank                                                                */
/* ======================================================================== */

static const Sim_PlantSpec PLANTS[SIM_PLANT_COUNT_] = {
    /* name          K     T     L      dt      horizon  sp   */
    { "easy L/T=0.1",  2.0,  1.0,  0.1,  0.01,    20.0,  1.0 },
    { "typical L/T=0.3", 2.0, 1.0, 0.3,  0.01,    25.0,  1.0 },
    { "hard L/T=1.0",  1.0,  1.0,  1.0,  0.01,    40.0,  1.0 },
    { "slow L/T=0.1",  1.0, 10.0,  1.0,  0.05,   150.0,  1.0 },
    { "furnace L/T=0.2", 3.0, 20.0, 4.0, 0.10,   400.0,  1.0 }
};

const Sim_PlantSpec *sim_plant_spec(Sim_PlantId id)
{
    if ((int)id < 0 || (int)id >= (int)SIM_PLANT_COUNT_) { return &PLANTS[0]; }
    return &PLANTS[id];
}

/* ======================================================================== */
/* Runner                                                                    */
/* ======================================================================== */

void sim_runopts_default(Sim_RunOpts *o)
{
    memset(o, 0, sizeof(*o));
    o->gain_mismatch = 1.0;
    o->tau_mismatch = 1.0;
    o->delay_mismatch = 1.0;
    o->u_min = -1e30;
    o->u_max = 1e30;
}

bool sim_run_step(PID_Handle *h, Sim_PlantId id, const Sim_RunOpts *o,
                  Sim_Metrics *out)
{
    const Sim_PlantSpec *sp = sim_plant_spec(id);
    Sim_RunOpts def;
    EX_Fopdt plant;
    double t = 0.0;
    long i;
    long steps;

    if (o == NULL) { sim_runopts_default(&def); o = &def; }

    steps = (long)(sp->horizon / sp->dt);

    /* The TRUE plant, which may deliberately differ from the model the
     * controller was tuned on. */
    ex_fopdt_init(&plant,
                  sp->k * o->gain_mismatch,
                  sp->t * o->tau_mismatch,
                  sp->l * o->delay_mismatch,
                  sp->dt, 0.0, 0.0);

    ex_noise_seed(12345U);
    sim_metrics_init(out, 0.0, sp->setpoint, 0.02);
    (void)PID_Reset(h);
    (void)PID_SetSetpoint(h, (PID_Float)sp->setpoint);

    for (i = 0; i < steps; ++i) {
        double y = plant.y;
        double u;

        if (o->noise_sigma > 0.0) { y += ex_noise_gauss(o->noise_sigma); }
        if (o->adc_bits > 0) {
            y = ex_adc_quantise(y, o->adc_lo, o->adc_hi, o->adc_bits);
        }

        u = (double)PID_UpdateDt(h, (PID_Float)y, (PID_Float)sp->dt);

        if (!isfinite(u) || !isfinite(plant.y)) {
            return false;   /* diverged - reported, not hidden */
        }

        {
            /* The load disturbance enters at the plant INPUT, which is where
             * real load changes act; injecting it on the output would instead
             * be a sensor fault and is a different experiment. */
            double u_plant = u;
            if ((o->dist_time > 0.0) && (t >= o->dist_time)) {
                u_plant += o->dist_size;
            }
            (void)ex_fopdt_step(&plant, u_plant, sp->dt);
        }

        sim_metrics_update(out, plant.y, u, sp->dt);

        if ((o->trace != NULL) && ((i % 2) == 0)) {
            sim_csv_row(o->trace, "%s,%.4f,%.6f,%.6f,%.6f",
                        (o->trace_tag != NULL) ? o->trace_tag : "run",
                        t, sp->setpoint, plant.y, u);
        }
        t += sp->dt;
    }
    return true;
}

/* ======================================================================== */
/* Exact plant analysis and controller construction                          */
/* ======================================================================== */

void sim_exact_ku_pu(double k, double t, double l, double *ku, double *pu)
{
    const double pi = 3.14159265358979323846;
    double lo = 1e-9, hi = 1.0, w;
    int i;

    while ((atan(hi * t) + (hi * l)) < pi) { hi *= 2.0; }
    for (i = 0; i < 200; ++i) {
        w = 0.5 * (lo + hi);
        if ((atan(w * t) + (w * l)) < pi) { lo = w; } else { hi = w; }
    }
    w = 0.5 * (lo + hi);
    *ku = sqrt(1.0 + (w * t * w * t)) / k;
    *pu = 2.0 * pi / w;
}

void sim_build_model(double k, double t, double l, PID_ModelKind kind,
                     PID_PlantModel *m)
{
    memset(m, 0, sizeof(*m));
    m->kind = kind;
    m->quality = 100U;
    if (kind == PID_MODEL_FOPDT) {
        m->k = (PID_Float)k;
        m->t = (PID_Float)t;
        m->l = (PID_Float)l;
    } else {
        double ku, pu;
        sim_exact_ku_pu(k, t, l, &ku, &pu);
        m->ku = (PID_Float)ku;
        m->pu = (PID_Float)pu;
    }
}

double sim_imc_lambda(double t, double l)
{
    const double eight_l = 8.0 * l;
    return 0.5 * ((t > eight_l) ? t : eight_l);
}

bool sim_make_controller(PID_Handle *h, const PID_Gains *g, double dt)
{
    PID_Config c;

    if (!isfinite((double)g->kp) || !isfinite((double)g->ki) ||
        !isfinite((double)g->kd) || ((double)g->kp <= 0.0)) {
        return false;
    }

    PID_ConfigDefault(&c);
    c.core.kp = g->kp;
    c.core.ki = g->ki;
    c.core.kd = g->kd;
    c.core.sample_time = (PID_Float)dt;

    /* A real actuator, so windup is part of the comparison rather than being
     * assumed away. u in [-2, 2] with a unit setpoint leaves headroom for an
     * aggressive rule but still bites when one goes wild. */
    c.limits.use_output_limits = true;
    c.limits.output_min = -2.0f;
    c.limits.output_max = 2.0f;
    c.integral.mode = PID_AW_BACK_CALCULATION;

    /* Derivative filter: rules report td, and an unfiltered D on a sampled
     * loop is not implementable. tf = td/N with N = 10 is the standard
     * choice; the library derives it when tf is left at 0. */
    c.filter.tf = g->tf;
    c.filter.n_filter = 10.0f;
    c.filter.derivative_mode = PID_DERIV_ON_MEASUREMENT;

    return (PID_Init(h, &c) == PID_OK);
}

/** @file ex_plant.c  Reference plant models for the examples. See ex_plant.h. */
#include "ex_plant.h"

#include <math.h>
#include <string.h>

/* ======================================================================== */
/* FOPDT                                                                     */
/* ======================================================================== */

void ex_fopdt_init(EX_Fopdt *p, double k, double tau, double l,
                   double dt, double y0, double bias)
{
    size_t n;

    memset(p, 0, sizeof(*p));
    p->k    = k;
    p->tau  = (tau > 1e-9) ? tau : 1e-9;
    p->l    = l;
    p->bias = bias;
    p->y    = y0;

    /* Round the dead time to whole samples. A half-sample rounding error is
     * irrelevant next to the +/-20% dead-time accuracy of any identification
     * method, and a fractional delay line would only add interpolation
     * artefacts to a reference model. */
    n = (size_t)((l / dt) + 0.5);
    if (n >= EX_DELAY_MAX) { n = EX_DELAY_MAX - 1U; }
    p->n   = n;
    p->idx = 0U;
}

double ex_fopdt_step(EX_Fopdt *p, double u, double dt)
{
    double u_delayed;

    if (p->n == 0U) {
        u_delayed = u;
    } else {
        u_delayed = p->buf[p->idx];   /* oldest sample */
        p->buf[p->idx] = u;
        p->idx = (p->idx + 1U) % p->n;
    }

    /* Exact discretisation of the first-order lag over one step:
     *   y[k+1] = y_inf + (y[k] - y_inf) * exp(-dt/tau)
     * rather than forward Euler, so the model stays correct even when the
     * caller uses a coarse dt. */
    {
        double y_inf = p->bias + (p->k * u_delayed);
        double a     = exp(-dt / p->tau);
        p->y = y_inf + ((p->y - y_inf) * a);
    }
    return p->y;
}

/* ======================================================================== */
/* DC motor                                                                  */
/* ======================================================================== */

void ex_motor_init(EX_Motor *m)
{
    memset(m, 0, sizeof(*m));
    m->l  = 0.5e-3;      /* 0.5 mH  -> tau_e = L/R = 0.5 ms  */
    m->r  = 1.0;         /* 1 ohm                            */
    m->ke = 0.05;        /* V/(rad/s)                        */
    m->kt = 0.05;        /* Nm/A  (equal to Ke in SI)        */
    m->j  = 1.0e-4;      /* kg m^2 -> tau_m = J/B = 50 ms    */
    m->b  = 2.0e-3;
    m->coulomb = 3.0e-3; /* Nm, breaks any purely P controller */
    m->load    = 0.0;
}

void ex_motor_step(EX_Motor *m, double v, double dt)
{
    /* Sub-step so the electrical dynamics stay resolved: with tau_e = 0.5 ms
     * and a caller dt of 1 ms, a single Euler step would be unstable. */
    const double h_max = 20e-6;
    int steps = (int)ceil(dt / h_max);
    double h;
    int i;

    if (steps < 1) { steps = 1; }
    if (steps > 2000) { steps = 2000; }
    h = dt / (double)steps;

    for (i = 0; i < steps; i++) {
        double di = (v - (m->r * m->i) - (m->ke * m->w)) / m->l;
        double torque = (m->kt * m->i) - (m->b * m->w) - m->load;

        /* Coulomb friction opposes motion and, at rest, cancels any torque
         * smaller than itself. Modelled with a small velocity threshold to
         * avoid the sign() chatter of an ideal discontinuity. */
        if (fabs(m->w) > 1e-3) {
            torque -= (m->w > 0.0) ? m->coulomb : -m->coulomb;
        } else if (fabs(torque) <= m->coulomb) {
            torque = 0.0;
            m->w   = 0.0;
        } else {
            torque -= (torque > 0.0) ? m->coulomb : -m->coulomb;
        }

        m->i  += di * h;
        m->w  += (torque / m->j) * h;
        m->th += m->w * h;
    }
}

/* ======================================================================== */
/* Heater                                                                    */
/* ======================================================================== */

void ex_heater_init(EX_Heater *p, double t_amb)
{
    memset(p, 0, sizeof(*p));
    p->t     = t_amb;
    p->t_amb = t_amb;
    /* Sized so that 180 C is reachable at roughly 60% duty: full power
     * would settle near 420 C on convection alone. A plant that cannot reach
     * the example's setpoint makes every controller look identical, because
     * they all just saturate. */
    p->c     = 40.0;     /* J/C  -> tau = C/h = 40 s at low temperature */
    p->p     = 400.0;    /* W at u = 1                                  */
    p->h     = 1.0;      /* W/C                                         */
    p->rad   = 2.0e-9;   /* W/K^4, ~30% of the loss at 180 C            */
}

double ex_heater_step(EX_Heater *p, double u, double dt)
{
    double tk;
    double tak;
    double loss;
    double dtdt;

    /* A resistive heater cannot cool and cannot exceed full power. Clamping
     * here keeps the MODEL physical no matter what the controller asks for;
     * it is not a substitute for configuring the controller's own output
     * limits, which is what tells the anti-windup logic where the actuator
     * actually ends. */
    if (u < 0.0) { u = 0.0; }
    if (u > 1.0) { u = 1.0; }

    tk   = p->t + 273.15;
    tak  = p->t_amb + 273.15;
    loss = (p->h * (p->t - p->t_amb))
         + (p->rad * ((tk * tk * tk * tk) - (tak * tak * tak * tak)));
    dtdt = ((p->p * u) - loss) / p->c;

    p->t += dtdt * dt;
    return p->t;
}

/* ======================================================================== */
/* Noise and quantisation                                                    */
/* ======================================================================== */

static unsigned int ex_rng = 12345U;

void ex_noise_seed(unsigned int seed)
{
    ex_rng = seed;
}

static double ex_uniform(void)
{
    /* Numerical Recipes LCG. Not cryptographic, but reproducible on every
     * platform, which is the only property an example needs. */
    ex_rng = (ex_rng * 1664525U) + 1013904223U;
    return ((double)(ex_rng >> 8) / (double)(1U << 24)) - 0.5;
}

double ex_noise(double amp)
{
    return ex_uniform() * 2.0 * amp;
}

double ex_noise_gauss(double sigma)
{
    /* Sum of 4 uniforms: variance 4/12 of a unit-width uniform, so scaling by
     * sqrt(3) gives unit variance. Good enough for sensor noise, and far
     * cheaper and more portable than Box-Muller. */
    double s = ex_uniform() + ex_uniform() + ex_uniform() + ex_uniform();
    return s * 1.7320508 * sigma;
}

double ex_adc_quantise(double x, double lo, double hi, int bits)
{
    double span  = hi - lo;
    double steps;
    double q;

    if ((span <= 0.0) || (bits <= 0) || (bits > 24)) {
        return x;
    }
    steps = (double)((1L << bits) - 1L);
    if (x < lo) { x = lo; }
    if (x > hi) { x = hi; }

    q = floor((((x - lo) / span) * steps) + 0.5);
    return lo + ((q / steps) * span);
}

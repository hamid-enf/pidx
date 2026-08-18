/**
 * @file    ex_plant.h
 * @brief   Reference plant models used by the PIDX examples.
 *
 * These are NOT part of the library. They exist so every example is a real,
 * runnable, closed-loop program on the host instead of a code fragment that
 * has never been executed.
 *
 * The models are written in double precision on purpose: the plant is the
 * "physics", and it should be more accurate than the controller under test so
 * that anything you see in the response comes from the controller, not from
 * the simulation. The controller boundary is always float (PID_Float).
 *
 * Everything here is deterministic - the noise source is a fixed-seed LCG - so
 * two runs of the same example produce byte-identical output. A flaky example
 * is a useless example.
 */
#ifndef PIDX_EX_PLANT_H
#define PIDX_EX_PLANT_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* 1. First-order plus dead time (FOPDT)                                     */
/* ======================================================================== */

#define EX_DELAY_MAX 4096

/**
 * tau * dy/dt = -(y - bias) + K * u(t - L)
 *
 * The workhorse model for process control: heaters, tanks, flows, most things
 * with a pipe in them. Dead time is a true transport delay implemented as a
 * ring buffer, not a Pade approximation, because a rational approximation of
 * dead time hides exactly the instability that dead time causes.
 */
typedef struct {
    double y;                     /**< Process value.                        */
    double k;                     /**< Static gain [y per u].                */
    double tau;                   /**< Time constant [s].                    */
    double l;                     /**< Dead time [s].                        */
    double bias;                  /**< Value with zero input (ambient).      */
    double buf[EX_DELAY_MAX];     /**< Transport delay line.                 */
    size_t n;                     /**< Delay length in samples.              */
    size_t idx;
} EX_Fopdt;

void   ex_fopdt_init(EX_Fopdt *p, double k, double tau, double l,
                     double dt, double y0, double bias);
double ex_fopdt_step(EX_Fopdt *p, double u, double dt);

/* ======================================================================== */
/* 2. DC motor (electrical + mechanical)                                     */
/* ======================================================================== */

/**
 * Two coupled states, which is what makes a real cascade meaningful:
 *
 *   L * di/dt = v - R*i - Ke*w          (electrical, fast: tau_e = L/R)
 *   J * dw/dt = Kt*i - B*w - tau_load   (mechanical, slow: tau_m = J/B)
 *   dth/dt    = w
 *
 * Default parameters are a small brushed motor: tau_e = 0.5 ms, tau_m = 50 ms.
 * The 100:1 separation between them is exactly why a current loop can run at
 * 20 kHz inside a velocity loop at 1 kHz.
 */
typedef struct {
    double i;        /**< Armature current [A].                              */
    double w;        /**< Shaft speed [rad/s].                               */
    double th;       /**< Shaft position [rad].                              */
    double l;        /**< Inductance [H].                                    */
    double r;        /**< Resistance [ohm].                                  */
    double ke;       /**< Back-EMF constant [V/(rad/s)].                     */
    double kt;       /**< Torque constant [Nm/A].                            */
    double j;        /**< Inertia [kg m^2].                                  */
    double b;        /**< Viscous friction [Nm/(rad/s)].                     */
    double coulomb;  /**< Coulomb friction torque [Nm], the classic cause of
                      *   steady-state error that integral action must beat. */
    double load;     /**< External load torque [Nm].                         */
} EX_Motor;

void ex_motor_init(EX_Motor *m);

/**
 * Advance by @p dt seconds, sub-stepping internally so that the electrical
 * time constant stays resolved even when the caller's dt is 1 ms.
 * @param v Terminal voltage [V].
 */
void ex_motor_step(EX_Motor *m, double v, double dt);

/* ======================================================================== */
/* 3. Heater with a nonlinear loss term                                      */
/* ======================================================================== */

/**
 * C * dT/dt = P*u - h*(T - Tamb) - eps*(T^4 - Tamb^4) * scale
 *
 * The radiative term makes the process gain fall as temperature rises, which
 * is the honest reason gain scheduling exists: a controller tuned at 40 C is
 * detuned at 200 C. Used by examples 02 and 10.
 *
 * u is clamped to [0, 1] inside the model: a resistive heater cannot cool.
 */
typedef struct {
    double t;        /**< Temperature [C].                                   */
    double t_amb;    /**< Ambient [C].                                       */
    double c;        /**< Heat capacity [J/C].                               */
    double p;        /**< Heater power at u = 1 [W].                         */
    double h;        /**< Convective loss [W/C].                             */
    double rad;      /**< Radiative coefficient; 0 makes the plant linear.   */
} EX_Heater;

void   ex_heater_init(EX_Heater *p, double t_amb);
double ex_heater_step(EX_Heater *p, double u, double dt);

/* ======================================================================== */
/* 4. Deterministic noise and quantisation                                   */
/* ======================================================================== */

/** Reset the noise generator. Same seed -> same sequence, always. */
void   ex_noise_seed(unsigned int seed);
/** Uniform white noise in [-amp, +amp]. */
double ex_noise(double amp);
/** Approximately Gaussian noise (sum of 4 uniforms), std-dev ~= sigma. */
double ex_noise_gauss(double sigma);

/**
 * Quantise to an ADC grid: @p bits over the span [lo, hi].
 * Sensor quantisation is what turns an unfiltered derivative term into a
 * square wave, so the examples that use a D term also use this.
 */
double ex_adc_quantise(double x, double lo, double hi, int bits);

#ifdef __cplusplus
}
#endif

#endif /* PIDX_EX_PLANT_H */

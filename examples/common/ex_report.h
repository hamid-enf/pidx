/**
 * @file    ex_report.h
 * @brief   Step-response metrics and ASCII plotting for the examples.
 *
 * Not part of the library. Every example ends by printing numbers that can be
 * checked rather than a wall of samples, so "it works" is a measurement and
 * not an impression.
 */
#ifndef PIDX_EX_REPORT_H
#define PIDX_EX_REPORT_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Running step-response analyser.
 *
 * Feed it every sample of a step from @p y0 to @p target and it computes the
 * classic figures of merit. All of them are defined against the COMMANDED
 * target, not against the final value, so a controller with steady-state
 * error is scored honestly instead of being graded on its own curve.
 */
typedef struct {
    double y0;            /**< Value when the step was applied.              */
    double target;
    double t;             /**< Accumulated time.                             */
    double t_rise;        /**< 10% -> 90% of the commanded change [s].       */
    double t_10;
    double t_90;
    double t_settle;      /**< Last time |y - target| left the band [s].     */
    double overshoot;     /**< Percent of the commanded change.              */
    double peak;
    double iae;           /**< Integral of |e| dt.                           */
    double ise;
    double itae;
    double u_travel;      /**< Integral of |du| - actuator wear proxy.       */
    double u_min;
    double u_max;
    double band;          /**< Settling band, fraction of the change.        */
    double y_final;
    double u_prev;
    bool   got_10;
    bool   got_90;
    bool   primed;
    long   samples;
} EX_Step;

/**
 * @param band Settling band as a fraction of the step size, e.g. 0.02 for the
 *             usual 2% criterion.
 */
void ex_step_init(EX_Step *s, double y0, double target, double band);

/** Feed one sample: process value @p y, actuator command @p u. */
void ex_step_update(EX_Step *s, double y, double u, double dt);

/** Steady-state error against the commanded target (signed). */
double ex_step_sserror(const EX_Step *s);

/** Print a one-line labelled summary. */
void ex_step_report(const EX_Step *s, const char *label);

/** Print the header that lines up with ex_step_report(). */
void ex_step_header(void);

/* ======================================================================== */
/* ASCII plot                                                                */
/* ======================================================================== */

/**
 * Draw a series as an ASCII chart, so an example is readable in a terminal
 * with no plotting dependency. @p n samples are decimated to @p width columns.
 */
void ex_plot(const double *y, int n, int width, int height,
             const char *label);

/** Two series on the same axes: '*' for @p a, '+' for @p b. */
void ex_plot2(const double *a, const double *b, int n, int width, int height,
              const char *label_a, const char *label_b);

#ifdef __cplusplus
}
#endif

#endif /* PIDX_EX_REPORT_H */

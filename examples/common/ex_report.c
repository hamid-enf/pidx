/** @file ex_report.c  Step metrics and ASCII plots. See ex_report.h. */
#include "ex_report.h"

#include <stdio.h>
#include <math.h>
#include <string.h>

void ex_step_init(EX_Step *s, double y0, double target, double band)
{
    memset(s, 0, sizeof(*s));
    s->y0      = y0;
    s->target  = target;
    s->band    = (band > 0.0) ? band : 0.02;
    s->peak    = y0;
    s->y_final = y0;
    s->u_min   =  1e30;
    s->u_max   = -1e30;
}

void ex_step_update(EX_Step *s, double y, double u, double dt)
{
    double change = s->target - s->y0;
    double e      = s->target - y;
    double adir   = (change >= 0.0) ? 1.0 : -1.0;
    double prog   = (change != 0.0) ? ((y - s->y0) / change) : 0.0;

    s->t += dt;
    s->samples++;
    s->y_final = y;

    s->iae  += fabs(e) * dt;
    s->ise  += e * e * dt;
    s->itae += s->t * fabs(e) * dt;

    if (s->primed) {
        s->u_travel += fabs(u - s->u_prev);
    }
    s->u_prev = u;
    s->primed = true;
    if (u < s->u_min) { s->u_min = u; }
    if (u > s->u_max) { s->u_max = u; }

    if (!s->got_10 && (prog >= 0.10)) { s->t_10 = s->t; s->got_10 = true; }
    if (!s->got_90 && (prog >= 0.90)) {
        s->t_90 = s->t;
        s->got_90 = true;
        s->t_rise = s->t_90 - s->t_10;
    }

    /* Peak in the direction of travel, so a downward step is scored on its
     * undershoot rather than on wherever it started. */
    if (((y - s->peak) * adir) > 0.0) {
        s->peak = y;
    }
    if (change != 0.0) {
        double os = ((s->peak - s->target) / change) * 100.0;
        if (os > s->overshoot) { s->overshoot = os; }
    }

    /* Settling time is the LAST exit from the band, which is the definition
     * that catches a loop that re-leaves the band later; taking the first
     * entry would score an oscillating controller as settled. */
    if (fabs(e) > (fabs(change) * s->band)) {
        s->t_settle = s->t;
    }
}

double ex_step_sserror(const EX_Step *s)
{
    return s->y_final - s->target;
}

void ex_step_header(void)
{
    printf("  %-22s %8s %8s %8s %9s %9s %8s\n",
           "case", "rise[s]", "sett[s]", "OS[%]", "ss.err", "IAE", "u_trav");
    printf("  %-22s %8s %8s %8s %9s %9s %8s\n",
           "----------------------", "--------", "--------", "--------",
           "---------", "---------", "--------");
}

void ex_step_report(const EX_Step *s, const char *label)
{
    printf("  %-22s %8.3f %8.3f %8.2f %9.4f %9.3f %8.2f\n",
           label,
           s->got_90 ? s->t_rise : -1.0,
           s->t_settle,
           s->overshoot,
           ex_step_sserror(s),
           s->iae,
           s->u_travel);
}

/* ======================================================================== */
/* ASCII plot                                                                */
/* ======================================================================== */

static void ex_plot_impl(const double *a, const double *b, int n,
                         int width, int height,
                         const char *la, const char *lb)
{
#define EX_PLOT_MAXW 100
#define EX_PLOT_MAXH 30
    static char canvas[EX_PLOT_MAXH][EX_PLOT_MAXW + 1];
    double lo = 1e30, hi = -1e30;
    int r, c, i;

    if ((n <= 1) || (width < 10) || (height < 3)) { return; }
    if (width  > EX_PLOT_MAXW) { width  = EX_PLOT_MAXW; }
    if (height > EX_PLOT_MAXH) { height = EX_PLOT_MAXH; }

    for (i = 0; i < n; i++) {
        if (a[i] < lo) { lo = a[i]; }
        if (a[i] > hi) { hi = a[i]; }
        if (b != NULL) {
            if (b[i] < lo) { lo = b[i]; }
            if (b[i] > hi) { hi = b[i]; }
        }
    }
    if ((hi - lo) < 1e-12) { hi = lo + 1.0; }

    for (r = 0; r < height; r++) {
        for (c = 0; c < width; c++) { canvas[r][c] = ' '; }
        canvas[r][width] = '\0';
    }

    for (c = 0; c < width; c++) {
        /* Decimate by picking the sample at the middle of each column's span,
         * not by averaging: averaging would hide the overshoot peak, which is
         * usually the whole point of looking at the plot. */
        i = (int)(((double)c / (double)(width - 1)) * (double)(n - 1));
        r = (int)(((hi - a[i]) / (hi - lo)) * (double)(height - 1) + 0.5);
        if (r < 0) { r = 0; }
        if (r >= height) { r = height - 1; }
        canvas[r][c] = '*';

        if (b != NULL) {
            r = (int)(((hi - b[i]) / (hi - lo)) * (double)(height - 1) + 0.5);
            if (r < 0) { r = 0; }
            if (r >= height) { r = height - 1; }
            if (canvas[r][c] == '*') { canvas[r][c] = '#'; }
            else                     { canvas[r][c] = '+'; }
        }
    }

    if (b != NULL) {
        printf("  %s ('*')  vs  %s ('+')   ['#' = both]\n", la, lb);
    } else {
        printf("  %s\n", la);
    }
    for (r = 0; r < height; r++) {
        double v = hi - (((hi - lo) * (double)r) / (double)(height - 1));
        printf("  %9.3f |%s|\n", v, canvas[r]);
    }
    printf("  %9s +", "");
    for (c = 0; c < width; c++) { putchar('-'); }
    printf("+\n");
}

void ex_plot(const double *y, int n, int width, int height, const char *label)
{
    ex_plot_impl(y, NULL, n, width, height, label, NULL);
}

void ex_plot2(const double *a, const double *b, int n, int width, int height,
              const char *label_a, const char *label_b)
{
    ex_plot_impl(a, b, n, width, height, label_a, label_b);
}

/**
 * Example 02 - Temperature control driving a PWM heater
 * =====================================================
 * The Intermediate API: PID_Config, output limits, anti-windup, integral
 * separation, a measurement filter and a quantised sensor.
 *
 * WHAT THIS SHOWS
 *   - PID_ConfigDefault + PID_Init instead of PID_InitDefault
 *   - output limits matched to a real actuator (a PWM duty of 0..1)
 *   - why an unlimited integrator is a bug and not a tuning preference
 *   - the three anti-windup strategies compared on the same plant, same gains
 *   - integral separation, and why it is not a substitute for anti-windup
 *   - PID_SetInputFilter against ADC quantisation noise
 *
 * THE CENTRAL POINT
 *   A heater cannot cool. The actuator saturates at 0 for the whole first
 *   part of a cool-down, and any integrator that keeps accumulating during
 *   that time has to be un-wound before the controller can respond again.
 *   That is windup, and the numbers at the bottom quantify it.
 */
#include <stdio.h>
#include <math.h>

#include "pidx/pid.h"
#include "ex_plant.h"
#include "ex_report.h"

#define DT      0.5              /* 2 Hz - a heater does not need more     */
#define N_UP    600              /* 300 s heating                          */
#define N_DOWN  600              /* 300 s at a much lower target           */
#define N_TOTAL (N_UP + N_DOWN)

/* ADC: 12-bit over 0..300 C. The LSB is 0.073 C, which is enough to make a
 * raw derivative term useless - see the filter comparison at the end. */
#define ADC_BITS 12
#define ADC_LO   0.0
#define ADC_HI   300.0

/**
 * One closed-loop run with a given anti-windup strategy.
 * Everything else - gains, limits, plant, setpoint profile - is identical, so
 * the only thing the comparison table measures is the strategy itself.
 */
static void run(PID_AntiWindup aw, PID_Float kt, const char *label,
                double *i_peak_out, double *recover_out, EX_Step *up_out)
{
    PID_Handle pid;
    PID_Config cfg;
    EX_Heater  plant;
    EX_Step    up;
    double     i_peak = 0.0;
    double     recover_time = -1.0;
    int        i;

    ex_heater_init(&plant, 20.0);
    ex_noise_seed(4242U);

    PID_ConfigDefault(&cfg);
    cfg.core.kp          = 0.05f;     /* duty per degree                     */
    cfg.core.ki          = 0.002f;    /* duty per degree-second              */
    cfg.core.kd          = 0.30f;     /* duty-second per degree              */
    cfg.core.sample_time = (PID_Float)DT;
    cfg.core.direction   = PID_DIRECT;         /* more duty -> hotter        */

    /* A PWM duty cycle. There is no such thing as -0.3 duty, and asking for
     * 1.4 just means "full on". Telling the controller its real authority is
     * what makes anti-windup and bumpless transfer possible at all. */
    cfg.limits.use_output_limits = true;
    cfg.limits.output_min = 0.0f;
    cfg.limits.output_max = 1.0f;

    cfg.integral.mode = aw;
    cfg.integral.kt   = kt;

    /* Derivative on measurement (the default) plus a filter: with a 12-bit
     * ADC, d/dt of the raw signal is 0.073 C per 0.5 s = 0.15 C/s of pure
     * quantisation noise, multiplied by Kd. */
    cfg.filter.derivative_mode = PID_DERIV_ON_MEASUREMENT;
    cfg.filter.tf              = 2.0f;
    cfg.filter.input_lpf_tau   = 1.0f;

    if (PID_Init(&pid, &cfg) != PID_OK) {
        printf("  %-18s INIT FAILED\n", label);
        return;
    }
    PID_SetSetpoint(&pid, 180.0f);
    ex_step_init(&up, 20.0, 180.0, 0.02);

    for (i = 0; i < N_TOTAL; i++) {
        double y_true = plant.t;
        double y_adc  = ex_adc_quantise(y_true + ex_noise_gauss(0.05),
                                        ADC_LO, ADC_HI, ADC_BITS);
        PID_Float u;

        if (i == N_UP) {
            /* Step DOWN to 60 C. The heater is now the wrong actuator: the
             * plant can only cool at its own rate and the output sits at 0
             * for a long time. This is where windup shows up. */
            PID_SetSetpoint(&pid, 60.0f);
        }

        u = PID_Update(&pid, (PID_Float)y_adc);
        (void)ex_heater_step(&plant, (double)u, DT);

        if (i < N_UP) {
            ex_step_update(&up, y_true, (double)u, DT);
        } else {
            double integ = (double)PID_GetIntegrator(&pid);
            if (fabs(integ) > i_peak) { i_peak = fabs(integ); }
            /* "Recovered" = the controller is off the limit and modulating
             * again, i.e. it has authority to respond to a disturbance. */
            if ((recover_time < 0.0) && (i > N_UP + 4)
                && (u > 0.001f) && (u < 0.999f)) {
                recover_time = (double)(i - N_UP) * DT;
            }
        }
    }

    *i_peak_out   = i_peak;
    *recover_out  = recover_time;
    *up_out       = up;
}

int main(void)
{
    double  i_peak[4], recover[4];
    EX_Step up[4];

    printf("Example 02 - Temperature / PWM heater\n");
    printf("  plant: 40 J/C, 400 W heater, convective + radiative loss\n");
    printf("  actuator: PWM duty 0..1  (a heater cannot cool)\n");
    printf("  sensor: 12-bit ADC over 0..300 C, sigma = 0.05 C\n");
    printf("  profile: 20 -> 180 C for 300 s, then step down to 60 C\n\n");

    run(PID_AW_NONE,             0.0f, "none",     &i_peak[0], &recover[0], &up[0]);
    run(PID_AW_CLAMP,            0.0f, "clamp",    &i_peak[1], &recover[1], &up[1]);
    run(PID_AW_CONDITIONAL,      0.0f, "cond",     &i_peak[2], &recover[2], &up[2]);
    run(PID_AW_BACK_CALCULATION, 1.0f, "back-calc",&i_peak[3], &recover[3], &up[3]);

    printf("  Heat-up phase (identical for all four - nothing saturates long):\n");
    ex_step_header();
    ex_step_report(&up[0], "AW none");
    ex_step_report(&up[1], "AW clamp");
    ex_step_report(&up[2], "AW conditional");
    ex_step_report(&up[3], "AW back-calculation");

    printf("\n  Cool-down phase - this is where the strategies differ:\n");
    printf("  %-22s %14s %16s\n", "anti-windup", "peak |I|", "time to regain");
    printf("  %-22s %14s %16s\n", "", "[duty]", "authority [s]");
    printf("  %-22s %14s %16s\n", "----------------------",
           "--------------", "----------------");
    {
        const char *nm[4] = { "none", "clamp", "conditional",
                              "back-calculation" };
        int k;
        for (k = 0; k < 4; k++) {
            if (recover[k] < 0.0) {
                printf("  %-22s %14.4f %16s\n", nm[k], i_peak[k], "never");
            } else {
                printf("  %-22s %14.4f %16.1f\n", nm[k], i_peak[k], recover[k]);
            }
        }
    }

    printf("\n  Reading the table:\n"
           "    - 'none' keeps integrating while the output is pinned at 0,\n"
           "      so |I| grows without bound and the loop stays saturated.\n"
           "    - 'clamp' bounds |I| to the output range; recovery is bounded\n"
           "      but the integrator still has to be walked back.\n"
           "    - 'conditional' stops integrating in the wrong direction, so\n"
           "      there is nothing to unwind.\n"
           "    - 'back-calculation' bleeds the excess off continuously at a\n"
           "      rate set by Kt; it is the fastest recovery here.\n");

    /* ------------------------------------------------------------------ */
    /* Derivative filtering against sensor quantisation                    */
    /* ------------------------------------------------------------------ */
    printf("\n  Derivative filter vs 12-bit quantisation (same 300 s heat-up):\n");
    {
        const PID_Float tfs[6] = { 0.0f, 0.25f, 0.5f, 1.0f, 2.0f, 8.0f };
        int k;

        printf("  %-22s %12s %12s %10s\n",
               "Tf [s]", "u travel", "IAE", "final [C]");
        printf("  %-22s %12s %12s %10s\n", "----------------------",
               "------------", "------------", "----------");

        for (k = 0; k < 6; k++) {
            PID_Handle pid;
            PID_Config cfg;
            EX_Heater  plant;
            EX_Step    m;
            char       lbl[32];
            int        i;

            ex_heater_init(&plant, 20.0);
            ex_noise_seed(4242U);

            PID_ConfigDefault(&cfg);
            cfg.core.kp = 0.05f;
            cfg.core.ki = 0.002f;
            cfg.core.kd = 0.30f;
            cfg.core.sample_time = (PID_Float)DT;
            cfg.limits.use_output_limits = true;
            cfg.limits.output_min = 0.0f;   /* omitting this leaves the
                                             * default -inf, and the loop
                                             * commands negative duty */
            cfg.limits.output_max = 1.0f;
            cfg.integral.mode = PID_AW_BACK_CALCULATION;
            cfg.integral.kt   = 1.0f;
            cfg.filter.tf      = tfs[k];
            /* n_filter would otherwise supply a fallback Tf = Kd/(N*Kp) when
             * tf == 0, and the "unfiltered" row would silently be filtered at
             * 0.6 s. Zeroing it makes tf mean exactly what it says. */
            cfg.filter.n_filter = 0.0f;
            /* No input pre-filter here, so the comparison isolates Tf. */
            (void)PID_Init(&pid, &cfg);
            PID_SetSetpoint(&pid, 180.0f);
            ex_step_init(&m, 20.0, 180.0, 0.02);

            for (i = 0; i < N_UP; i++) {
                double y_adc = ex_adc_quantise(plant.t + ex_noise_gauss(0.05),
                                               ADC_LO, ADC_HI, ADC_BITS);
                PID_Float u = PID_Update(&pid, (PID_Float)y_adc);
                (void)ex_heater_step(&plant, (double)u, DT);
                ex_step_update(&m, plant.t, (double)u, DT);
            }
            if (tfs[k] == 0.0f) {
                snprintf(lbl, sizeof(lbl), "0.00 (truly unfiltered)");
            } else {
                snprintf(lbl, sizeof(lbl), "%.2f  (Tf/dt = %.1f)",
                         (double)tfs[k], (double)tfs[k] / DT);
            }
            printf("  %-22s %12.2f %12.2f %10.2f\n",
                   lbl, m.u_travel, m.iae, plant.t);
        }
        printf("\n  'u travel' is the integral of |du| - a direct proxy for\n"
               "  actuator wear and for switching losses in the PWM stage.\n\n"
               "  This table is not a smooth trade-off, and the reason matters.\n"
               "  The filter pole is c_da = Tf/(Tf+dt), so with dt = 0.5 s a Tf\n"
               "  below about one sample barely filters at all while still\n"
               "  adding phase lag: Kd/(Tf+dt) makes the derivative gain LARGER\n"
               "  as Tf shrinks. Below Tf ~ dt the loop is not merely noisy, it\n"
               "  is unstable, and the plant never reaches the setpoint at all\n"
               "  (see the final-temperature column). Useful filtering starts\n"
               "  around Tf = 2*dt; the classic N-based rule Tf = Kd/(N*Kp)\n"
               "  with N = 10 gives 0.6 s here, just over one sample.\n");
    }

    return 0;
}

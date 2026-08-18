/* PHASE 13 - safety layer verification.
 *
 * Every check here asserts a specific documented behaviour, not merely that
 * the code runs. Where a number can be derived by hand it is derived by hand.
 */
#include <stdio.h>
#include <math.h>
#include "pidx/pid.h"

static int pass = 0, bad = 0;
#define CK(c,m) do{ if(c){pass++;} else {bad++;printf("  FAIL: %s\n",(m));} }while(0)

static double plant(double y, double u, double dt)
{
    return y + dt * ((1.0 * u - y) / 1.0);
}

int main(void)
{
    const double dt = 0.01;

    /* ---- 1. out-of-range measurement latches only after N samples ------- */
    {
        PID_Handle h; PID_Config c;
        PID_ConfigDefault(&c);
        c.core.kp = 1.0f; c.core.ki = 0.5f;
        c.core.sample_time = (PID_Float)dt;
        c.safety.enabled = true;
        c.safety.meas_min = -50.0f; c.safety.meas_max = 50.0f;
        c.safety.failsafe_output = -7.0f;
        c.safety.fault_persist_n = 3U;
        PID_Init(&h, &c);
        PID_SetSetpoint(&h, 1.0f);

        for (int i = 0; i < 50; i++) { (void)PID_Update(&h, 0.0f); }
        CK(!PID_IsFaulted(&h), "no fault on valid data");

        (void)PID_Update(&h, 999.0f);
        CK(!PID_IsFaulted(&h), "1 bad sample must not latch (persist_n=3)");
        (void)PID_Update(&h, 999.0f);
        CK(!PID_IsFaulted(&h), "2 bad samples must not latch");
        PID_Float u3 = PID_Update(&h, 999.0f);
        CK(PID_IsFaulted(&h), "3rd bad sample latches the fault");
        CK(u3 == -7.0f, "output switches to failsafe value");
        printf("  latch after 3 samples, failsafe u=%.2f\n", (double)u3);

        /* Latched fault must persist even once the sensor recovers. */
        for (int i = 0; i < 20; i++) { (void)PID_Update(&h, 0.0f); }
        CK(PID_IsFaulted(&h), "fault stays latched without auto_recover");
        CK(PID_GetOutput(&h) == -7.0f, "failsafe held while latched");

        CK(PID_ClearFault(&h) == PID_OK, "ClearFault accepted");
        CK(!PID_IsFaulted(&h), "fault cleared explicitly");
    }

    /* ---- 2. auto_recover releases the latch by itself ------------------- */
    {
        PID_Handle h; PID_Config c;
        PID_ConfigDefault(&c);
        c.core.kp = 1.0f; c.core.ki = 0.5f;
        c.core.sample_time = (PID_Float)dt;
        c.safety.enabled = true;
        c.safety.meas_min = -50.0f; c.safety.meas_max = 50.0f;
        c.safety.failsafe_output = 0.0f;
        c.safety.fault_persist_n = 2U;
        c.safety.auto_recover = true;
        PID_Init(&h, &c);
        PID_SetSetpoint(&h, 1.0f);
        for (int i = 0; i < 20; i++) { (void)PID_Update(&h, 0.0f); }
        (void)PID_Update(&h, 999.0f);
        (void)PID_Update(&h, 999.0f);
        CK(PID_IsFaulted(&h), "fault latched with persist_n=2");
        (void)PID_Update(&h, 0.0f);
        CK(!PID_IsFaulted(&h), "auto_recover clears the latch");
        printf("  auto_recover restored control\n");
    }

    /* ---- 3. rate limit catches a sensor jump inside the valid range ----- */
    {
        PID_Handle h; PID_Config c;
        PID_ConfigDefault(&c);
        c.core.kp = 1.0f;
        c.core.sample_time = (PID_Float)dt;
        c.safety.enabled = true;
        c.safety.meas_min = -1000.0f; c.safety.meas_max = 1000.0f;
        /* 100 units/s * 0.01 s = 1.0 unit allowed per sample. */
        c.safety.meas_rate_max = 100.0f;
        c.safety.fault_persist_n = 1U;
        c.safety.failsafe_output = 0.0f;
        PID_Init(&h, &c);
        PID_SetSetpoint(&h, 0.0f);

        (void)PID_Update(&h, 0.0f);
        (void)PID_Update(&h, 0.5f);            /* 0.5 <= 1.0 : fine */
        CK(!PID_IsFaulted(&h), "step within rate budget accepted");
        (void)PID_Update(&h, 20.0f);           /* jump of 19.5 : rejected */
        CK(PID_IsFaulted(&h), "sensor jump rejected while still in range");
        PID_StatusCode code = PID_OK;
        (void)PID_GetLastError(&h, &code);
        CK(code == PID_ERR_SENSOR_RATE, "SENSOR_RATE reported");
        printf("  rate guard fired, code=%d\n", (int)code);
    }

    /* ---- 4. safety disabled costs nothing and never trips --------------- */
    {
        PID_Handle h; PID_Config c;
        PID_ConfigDefault(&c);
        c.core.kp = 1.0f;
        c.core.sample_time = (PID_Float)dt;
        c.safety.enabled = false;
        PID_Init(&h, &c);
        for (int i = 0; i < 10; i++) { (void)PID_Update(&h, 1e6f); }
        CK(!PID_IsFaulted(&h), "no fault machinery when safety is off");
    }

    /* ---- 5. NaN/Inf are refused regardless of the safety switch --------- */
    {
        PID_Handle h; PID_Config c;
        PID_ConfigDefault(&c);
        c.core.kp = 1.0f; c.core.ki = 1.0f;
        c.core.sample_time = (PID_Float)dt;
        c.safety.enabled = false;
        PID_Init(&h, &c);
        PID_SetSetpoint(&h, 1.0f);
        double y = 0.0;
        for (int i = 0; i < 100; i++) {
            y = plant(y, (double)PID_Update(&h, (PID_Float)y), dt);
        }
        PID_Float held = PID_GetOutput(&h);
        PID_Float integ = PID_GetIntegrator(&h);

        (void)PID_Update(&h, (PID_Float)(0.0 / 0.0));
        CK(PID_GetOutput(&h) == held, "NaN must not move the output");
        CK(PID_GetIntegrator(&h) == integ, "NaN must not move the integrator");

        (void)PID_Update(&h, (PID_Float)(1.0 / 0.0));
        CK(PID_GetOutput(&h) == held, "Inf must not move the output");
        CK(PID_GetIntegrator(&h) == integ, "Inf must not move the integrator");
        printf("  output frozen at %.6f across NaN and Inf\n", (double)held);
    }

    /* ---- 6. failsafe output is itself validated ------------------------- */
    {
        PID_Handle h; PID_Config c;
        PID_ConfigDefault(&c);
        c.core.sample_time = (PID_Float)dt;
        PID_Init(&h, &c);
        PID_SafetyConfig sc;
        sc.enabled = true;
        sc.meas_min = 0.0f; sc.meas_max = 10.0f;
        sc.meas_rate_max = 0.0f;
        sc.failsafe_output = (PID_Float)(0.0 / 0.0);
        sc.fault_persist_n = 1U; sc.auto_recover = false;
        CK(PID_SetSafety(&h, &sc) == PID_ERR_INVALID_PARAM,
           "NaN failsafe output rejected");
        sc.failsafe_output = 0.0f;
        sc.meas_min = 10.0f; sc.meas_max = 5.0f;
        CK(PID_SetSafety(&h, &sc) == PID_ERR_INVALID_LIMIT ||
           PID_SetSafety(&h, &sc) == PID_ERR_INVALID_PARAM,
           "inverted measurement range rejected");
        sc.meas_min = 0.0f; sc.meas_max = 10.0f;
        CK(PID_SetSafety(&h, &sc) == PID_OK, "valid safety config accepted");
    }

    printf("\nsafety: %d passed, %d failed\n", pass, bad);
    return bad != 0;
}

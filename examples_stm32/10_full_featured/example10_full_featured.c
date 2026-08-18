/**
 * ====================================================================
 * مثال 10 — Full Featured (همه قابلیت‌ها یکجا)
 * ====================================================================
 *
 * این مثال همه قابلیت‌های PIDX را در یک کنترلر نشان می‌دهد:
 *
 *   1. 2DOF: beta < 1 برای کاهش overshoot بدون ضعیف کردن رد اغتشاش
 *   2. Feedforward: تخمین ولتاژ پایه (مدل معکوس)
 *   3. Setpoint Shaper: حرکت نرم (مثل آسانسور)
 *   4. Gain Scheduling: بهره‌ها با سرعت تغییر می‌کنند
 *   5. Safety: محدوده سنسور + نرخ + auto-recovery
 *   6. Anti-Windup: Back-Calculation
 *   7. Integral limits
 *   8. Telemetry: ثبت داده
 *
 * سناریو:
 *   موتور DC با انکودر به 100 rad/s فرمان می‌گیرد.
 *   Gains بر اساس سرعت تغییر می‌کنند (Gain Scheduling).
 *   در t=1.5s بار ضربه‌ای 15mNm اعمال می‌شود.
 *   در t=2.0s سنسور خراب می‌شود (صفر می‌دهد) — Safety وارد می‌شود.
 *   در t=2.5s سنسور برمی‌گردد — auto-recovery.
 *
 * پیاده‌سازی STM32:
 *   TIM3 با 1kHz → example10_tick()
 *   while(1) → example10_report() هر 1 ثانیه
 *
 * ====================================================================
 */

#include <stdio.h>
#include <math.h>

#include "pidx/pid.h"
#if PIDX_ENABLE_GAIN_SCHED
#include "pidx/pid_gainsched.h"
#endif
#if PIDX_ENABLE_DIAGNOSTICS
#include "pidx/pid_diag.h"
#endif

/* ===== پیکربندی ===== */
#define CTRL_DT             0.001f      /* 1kHz */
#define CTRL_SETPOINT       100.0f      /* rad/s */

#define V_MAX               24.0f

/* ===== موتور DC ===== */
#define MOTOR_R     1.0f
#define MOTOR_L     0.0005f
#define MOTOR_KE    0.05f
#define MOTOR_KT    0.05f
#define MOTOR_J     0.0001f
#define MOTOR_B     0.002f
#define MOTOR_COULOMB 0.003f

/* ===== وضعیت ===== */
static struct {
    PID_Handle pid;

    /* موتور */
    float motor_i, motor_w, motor_theta;
    float load_torque;

    float output;
    uint32_t tick;

    /* سنسور */
    float sensor_raw;      /* خروجی سنسور (قبل از safety) */
    int   sensor_faulted;

    /* Gain scheduling */
    #if PIDX_ENABLE_GAIN_SCHED
    PID_GainSchedule sched;
    PID_GainPoint    sched_points[5];
    #endif

} ctx;

/* ================================================================
 * مدل موتور با زیرگام
 * ================================================================ */
static void motor_step(float v, float dt)
{
    float tau_elec = MOTOR_L / MOTOR_R;
    int n = (int)(dt / (0.1f * tau_elec)) + 1;
    float dt_sub = dt / (float)n;

    for (int i = 0; i < n; i++) {
        float di_dt = (v - MOTOR_R * ctx.motor_i - MOTOR_KE * ctx.motor_w) / MOTOR_L;
        ctx.motor_i += di_dt * dt_sub;

        float coulomb = (ctx.motor_w > 0.01f) ? MOTOR_COULOMB :
                        (ctx.motor_w < -0.01f) ? -MOTOR_COULOMB : 0.0f;
        float dw_dt = (MOTOR_KT * ctx.motor_i - MOTOR_B * ctx.motor_w
                       - ctx.load_torque - coulomb) / MOTOR_J;
        ctx.motor_w += dw_dt * dt_sub;
        ctx.motor_theta += ctx.motor_w * dt_sub;
    }
}

/* ================================================================
 * Feedforward: V_ff = (R*B/Kt + Ke)*w + R*T_coulomb/Kt = 0.09w + 0.06
 * ================================================================ */
static float motor_ff(float setpoint, float measurement, void *p)
{
    (void)p; (void)measurement;
    return (MOTOR_R * MOTOR_B / MOTOR_KT + MOTOR_KE) * setpoint
         + (MOTOR_R * MOTOR_COULOMB / MOTOR_KT);
}

/* ================================================================
 * Gain Scheduling — بهره‌ها بر اساس سرعت
 *
 * سرعت کم → اصطکاک نسبی بیشتر → Kp بیشتر
 * سرعت زیاد → اینرسی کمک می‌کند → Kp کمتر
 *
 * جدول: { سرعت, Kp, Ki, Kd }
 * ================================================================ */
static void setup_gain_schedule(void)
{
    #if PIDX_ENABLE_GAIN_SCHED
    ctx.sched_points[0] = (PID_GainPoint){  0.0f, 0.80f, 12.0f, 0.008f };
    ctx.sched_points[1] = (PID_GainPoint){ 30.0f, 0.60f, 10.0f, 0.006f };
    ctx.sched_points[2] = (PID_GainPoint){ 60.0f, 0.50f,  8.0f, 0.005f };
    ctx.sched_points[3] = (PID_GainPoint){ 90.0f, 0.40f,  6.0f, 0.003f };
    ctx.sched_points[4] = (PID_GainPoint){200.0f, 0.30f,  4.0f, 0.002f };

    PID_GainSched_Init(&ctx.sched, ctx.sched_points, 5,
                        PID_SCHED_SRC_MEASUREMENT,
                        PID_SCHED_INTERP_LINEAR);
    PID_GainSched_Attach(&ctx.pid, &ctx.sched);
    #endif
}

/* ================================================================
 * سنسور انکودر
 * ================================================================ */
static float read_encoder(void)
{
    static float last = 0.0f;
    float speed = (ctx.motor_theta - last) / CTRL_DT;
    last = ctx.motor_theta;
    return speed;
}

/* ================================================================
 * راه‌اندازی — همه قابلیت‌ها
 * ================================================================ */
void example10_init(void)
{
    PID_Config cfg;

    ctx.motor_i = ctx.motor_w = ctx.motor_theta = 0.0f;
    ctx.load_torque = 0.0f;
    ctx.tick = 0;
    ctx.sensor_faulted = 0;

    PID_ConfigDefault(&cfg);

    /* ---- Core ---- */
    cfg.core.kp = 0.50f;
    cfg.core.ki = 6.0f;
    cfg.core.kd = 0.005f;
    cfg.core.sample_time = CTRL_DT;
    cfg.core.direction = PID_DIRECT;

    /* ---- Limits ---- */
    cfg.limits.use_output_limits = true;
    cfg.limits.output_min = -V_MAX;
    cfg.limits.output_max =  V_MAX;
    cfg.limits.use_integral_limits = true;
    cfg.limits.integral_min = -3.0f;
    cfg.limits.integral_max =  3.0f;

    /* ---- Filter: derivative on measurement + TF ---- */
    cfg.filter.derivative_mode = PID_DERIV_ON_MEASUREMENT;
    cfg.filter.tf = 0.005f;

    /* ---- Integral / AW ---- */
    cfg.integral.mode = PID_AW_BACK_CALCULATION;
    cfg.integral.kt = 10.0f;

    /* ---- 2DOF: beta=0.7 کاهش overshoot ---- */
    cfg.weight.beta  = 0.7f;
    cfg.weight.gamma = 0.0f;

    /* ---- Feedforward ---- */
    cfg.feedforward.enabled = true;
    cfg.feedforward.fn = motor_ff;
    cfg.feedforward.gain = 1.0f;

    /* ---- Shaper: مسیر نرم ---- */
    cfg.shaper.sp_rate_max = 100.0f;    /* rad/s */
    cfg.shaper.sp_accel    = 500.0f;    /* rad/s^2 */

    /* ---- Safety: محدوده + نرخ + خودبازیابی ---- */
    cfg.safety.enabled = true;
    cfg.safety.meas_min = -10.0f;
    cfg.safety.meas_max = 300.0f;
    cfg.safety.meas_rate_max = 5000.0f; /* rad/s^2 — جهش غیرطبیعی */
    cfg.safety.failsafe_output = 0.0f;  /* خروجی ایمن هنگام خطا */
    cfg.safety.fault_persist_n = 5;     /* 5 نمونه خطا تا latch */
    cfg.safety.auto_recover = true;     /* خودبازیابی بعد از رفع خطا */

    PID_Init(&ctx.pid, &cfg);

    /* ---- Gain Scheduling ---- */
    setup_gain_schedule();

    /* ---- Setpoint ---- */
    PID_SetSetpoint(&ctx.pid, CTRL_SETPOINT);

    /* ---- Prime مشتق ---- */
    PID_SetSetpoint(&ctx.pid, 0.0f);
    for (int i = 0; i < 50; i++)
        PID_Update(&ctx.pid, 0.0f);
    PID_SetSetpoint(&ctx.pid, CTRL_SETPOINT);

    printf("\r\n============================================\r\n");
    printf("مثال 10 — Full Featured PID\r\n");
    printf("============================================\r\n");
    printf("  ✓ 2DOF (beta=0.7)\r\n");
    printf("  ✓ Feedforward: V_ff = 0.09w + 0.06\r\n");
    printf("  ✓ Setpoint Shaper (100 rad/s, 500 rad/s^2)\r\n");
    printf("  ✓ Gain Scheduling (5 نقطه، linear)\r\n");
    printf("  ✓ Safety (range + rate + auto-recovery)\r\n");
    printf("  ✓ Back-Calculation AW + Integral limits\r\n");
    printf("--------------------------------------------\r\n");
    printf("%4s | %8s | %8s | %8s | %8s | %s\r\n",
           "t", "هدف", "سرعت", "خروجی", "gains", "رویداد");
    printf("--------------------------------------------\r\n");
}

/* ================================================================
 * تیک (1kHz)
 * ================================================================ */
void example10_tick(void)
{
    ctx.tick++;

    /* ---- سناریو: بار و خطا ---- */
    if (ctx.tick == 1500) {
        ctx.load_torque = 0.015f;   /* بار 15mNm در t=1.5s */
        printf(">>> بار 15mNm اعمال شد <<<\r\n");
    }
    if (ctx.tick == 2000) {
        ctx.sensor_faulted = 1;     /* خطای سنسور در t=2.0s */
        printf(">>> سنسور خراب شد (Safety فعال می‌شود) <<<\r\n");
    }
    if (ctx.tick == 2500) {
        ctx.sensor_faulted = 0;     /* سنسور برگشت t=2.5s */
        printf(">>> سنسور برگشت (auto-recovery) <<<\r\n");
    }

    /* ---- 1. موتور ---- */
    motor_step(ctx.output, CTRL_DT);

    /* ---- 2. سنسور ----
     *
     * خطای سنسور واقعی: قرائت خارج از محدوده (نه ثابت 0!)
     * چون سنسور ثابت 0 داخل محدوده [-10,300] است و
     * Safety آن را تشخیص نمی‌دهد — درست هم هست،
     * موتور ممکن است واقعاً متوقف شود.
     *
     * قرائت خراب: مقدار تصادفی بین 500 تا 900
     * (خارج از meas_max=300) → Range violation → Fault
     */
    ctx.sensor_raw = read_encoder();
    float meas;
    if (ctx.sensor_faulted) {
        static unsigned int seed = 999;
        seed = seed * 1664525 + 1013904223;
        float garbage = 500.0f + (float)(int)(seed % 400) * 1.0f;
        meas = garbage;   /* خارج از محدوده! */
    } else {
        meas = ctx.sensor_raw;
    }

    /* ---- 3. PID با همه قابلیت‌ها ---- */
    ctx.output = PID_Update(&ctx.pid, meas);

    /* ---- 4. گزارش ---- */
    if (ctx.tick % 200 == 0) {
        float sp = PID_GetSetpoint(&ctx.pid);
        float kp, ki, kd;
        PID_GetGains(&ctx.pid, &kp, &ki, &kd);
        uint16_t flags = PID_GetFlags(&ctx.pid);

        printf("%4.1f | %8.1f | %8.1f | %8.2f | %5.2f/%4.1f/%4.3f | %s\r\n",
               (float)ctx.tick * CTRL_DT,
               sp, ctx.sensor_raw, ctx.output,
               kp, ki, kd,
               (flags & PID_FLAG_FAULT) ? "FAULT!" :
               (flags & PID_FLAG_SP_RAMPING) ? "ramping" : "");
    }

    /* ---- 5. پایان ---- */
    if (ctx.tick >= 4000) {
        printf("--------------------------------------------\r\n");
        printf("OK. سرعت نهایی: %.1f rad/s (هدف %.0f)\r\n",
               ctx.motor_w, CTRL_SETPOINT);
        printf("============================================\r\n\r\n");
    }
}


/* ================================================================== */
#ifdef TEST_ON_PC
int main(void)
{
    example10_init();
    for (int i = 0; i < 4000; i++) example10_tick();
    return 0;
}
#endif

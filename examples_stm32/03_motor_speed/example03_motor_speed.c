/**
 * ====================================================================
 * مثال ۳ — کنترل سرعت موتور DC با 2DOF
 * ====================================================================
 *
 * ██ کاربرد ██
 *   کنترل سرعت موتور DC با انکودر:
 *   - رباتیک، نوار نقاله، فن، پمپ
 *   - محرک دوطرفه: −۲۴ ولت تا +۲۴ ولت
 *   - سنسور: انکودر ۱۰۲۴ پالس
 *
 * ██ مفاهیم جدید ██
 *   ۱. Derivative Kick:
 *      وقتی setpoint ناگهان تغییر می‌کنه (پله)،
 *      مشتق روی خطا یه ضربه بزرگ می‌زنه.
 *      راه‌حل: derivative روی measurement (پیش‌فرض).
 *
 *   ۲. Setpoint Weighting (β):
 *      ضرب Kp در β*sp به‌جای sp
 *      β=1: کلاسیک (overshoot داره)
 *      β<1: overshoot کمتر، بدون تغییر پایداری
 *
 *   ۳. کوپل Coulomb:
 *      موتور DC به ولتاژ ثابت برای غلبه بر اصطکاک نیاز داره
 *      — اگه انتگرال نداشته باشی، خطای ماندگار می‌مونه
 *
 * ██ سناریو ██
 *   - موتور به ۱۰۰ رادیان بر ثانیه (±۹۵۵ RPM) فرمان می‌گیره
 *   - بعد از ۲ ثانیه یه بار ضربه‌ای (load step) اعمال می‌شه
 *   - مقایسه PD (بدون I) و PID (با I) در برابر بار
 *
 * ██ خروجی مورد انتظار ██
 *   - Derivative روی measurement: پالس صفر روی D
 *   - Derivative روی error: ضربه ۲۴ ولتی روی D
 *   - PD: بعد از بار، خطای ماندگار (نمی‌تونه برگرده)
 *   - PID: بعد از بار، برمی‌گرده به هدف
 *
 * ====================================================================
 */

#include <stdio.h>
#include <math.h>

/* ===== مسیر هدر PIDX ===== */
#include "pidx/pid.h"

/* ===== پیکربندی ===== */
#define CTRL_DT             0.001f   /* ۱ کیلوهرتز */
#define CTRL_KP             0.30f
#define CTRL_KI             8.0f     /* برای PID */
#define CTRL_KD             0.002f
#define CTRL_TF             0.005f   /* فیلتر مشتق ۵ میلی‌ثانیه */

#define CTRL_SETPOINT       100.0f   /* rad/s */
#define V_SUPPLY            24.0f    /* ولتاژ تغذیه */

/* ===== موتور DC ===== */
#define MOTOR_R     1.0f             /* مقاومت آرمیچر [اهم] */
#define MOTOR_L     0.0005f          /* اندوکتانس [هانری] */
#define MOTOR_KE    0.05f            /* ثابت EMF [V/(rad/s)] */
#define MOTOR_KT    0.05f            /* ثابت گشتاور [Nm/A] */
#define MOTOR_J     0.0001f          /* اینرسی [kg·m²] */
#define MOTOR_B     0.002f           /* ویسکوزیته [N·m·s] */
#define MOTOR_COULOMB 0.003f         /* اصطکاک کولن [Nm] */

/* ===== وضعیت برنامه ===== */
static struct {
    PID_Handle pid;

    /* موتور */
    float motor_i;      /* جریان آرمیچر [A] */
    float motor_w;      /* سرعت [rad/s] */
    float motor_theta;  /* موقعیت [rad] */
    float load_torque;  /* گشتاور بار [Nm] */

    float output;       /* ولتاژ فرمان [-24..+24] */
    float d_term;       /* ترم مشتق (برای نمایش) */

    /* انکودر */
    float enc_last_counts;

    uint32_t tick;

} ctx;

/* ================================================================
 * مدل موتور DC — با زیرگام (sub-stepping) برای پایداری عددی
 *
 *   دیفرانسیل حالت:
 *     L*di/dt = V - R*i - Ke*w
 *     J*dw/dt = Kt*i - B*w - T_load - T_coulomb*sign(w)
 *     dθ/dt = w
 *
 *   ★ زیرگام: چون τ_elec = L/R = 0.5ms < dt=1ms
 *   حلقه الکتریکی را با گام‌های کوچکتر حل می‌کنیم
 * ================================================================ */
static void motor_step(float v, float dt)
{
    /* تعداد زیرگام‌ها: طوری که هر زیرگام ≤ ۰.۱ * τ_elec */
    float tau_elec = MOTOR_L / MOTOR_R;
    int n = (int)(dt / (0.1f * tau_elec)) + 1;
    float dt_sub = dt / (float)n;

    for (int i = 0; i < n; i++) {
        /* الکتریکی */
        float di_dt = (v - MOTOR_R * ctx.motor_i - MOTOR_KE * ctx.motor_w) / MOTOR_L;
        ctx.motor_i += di_dt * dt_sub;

        /* مکانیکی */
        float coulomb = (ctx.motor_w > 0.01f) ? MOTOR_COULOMB :
                        (ctx.motor_w < -0.01f) ? -MOTOR_COULOMB : 0.0f;
        float dw_dt = (MOTOR_KT * ctx.motor_i - MOTOR_B * ctx.motor_w
                       - ctx.load_torque - coulomb) / MOTOR_J;
        ctx.motor_w += dw_dt * dt_sub;

        ctx.motor_theta += ctx.motor_w * dt_sub;
    }
}

/* ================================================================
 * شبیه‌سازی انکودر — سرعت از شمارش پالس
 *
 * تفکیک‌پذیری: هر کانت = ۱.۵۳ rad/s LSB در ۱ کیلوهرتز
 * ================================================================ */
#define ENCODER_CPR 1024.0f

static float read_encoder_speed(void)
{
    float counts_per_rad = (4.0f * ENCODER_CPR) / (2.0f * 3.14159265f);
    float counts_now = floorf(ctx.motor_theta * counts_per_rad);
    float delta = (counts_now - ctx.enc_last_counts) / counts_per_rad;
    ctx.enc_last_counts = counts_now;

    float speed = delta / CTRL_DT;

    /* نویز کوانتیزاسیون انکودر */
    static unsigned int seed = 777;
    seed = seed * 1664525 + 1013904223;
    float noise = (float)(int)seed * 0.01f / 2147483648.0f;

    return speed + noise;
}

/* ================================================================
 * راه‌اندازی با حالت Derivative مشخص
 *
 * d_mode = PID_DERIV_ON_MEASUREMENT (پیش‌فرض، بدون kick)
 * d_mode = PID_DERIV_ON_ERROR (ضربه می‌زنه)
 * ================================================================ */
void example03_init(PID_DerivativeMode d_mode, float beta, int use_integral)
{
    PID_Config cfg;

    ctx.motor_i = 0.0f;
    ctx.motor_w = 0.0f;
    ctx.motor_theta = 0.0f;
    ctx.load_torque = 0.0f;
    ctx.enc_last_counts = 0.0f;
    ctx.tick = 0;

    PID_ConfigDefault(&cfg);
    cfg.core.kp = CTRL_KP;
    cfg.core.ki = use_integral ? CTRL_KI : 0.0f;
    cfg.core.kd = CTRL_KD;
    cfg.core.sample_time = CTRL_DT;

    /* محدودیت خروجی: پل H دوطرفه */
    cfg.limits.use_output_limits = true;
    cfg.limits.output_min = -V_SUPPLY;
    cfg.limits.output_max =  V_SUPPLY;

    cfg.integral.mode = PID_AW_BACK_CALCULATION;
    cfg.integral.kt   = 20.0f;

    cfg.filter.derivative_mode = d_mode;
    cfg.filter.tf = CTRL_TF;
    cfg.filter.n_filter = 0.0f;   /* tf دقیقاً همونیه که تنظیم کردیم */

    cfg.weight.beta  = beta;
    cfg.weight.gamma = 0.0f;

    PID_Init(&ctx.pid, &cfg);
    PID_SetSetpoint(&ctx.pid, CTRL_SETPOINT);

    printf("\r\n");
    printf("============================================\r\n");
    printf("مثال ۳ — سرعت موتور DC\r\n");
    printf("حالت D: %s  β=%.1f  Ki=%s\r\n",
           d_mode == PID_DERIV_ON_MEASUREMENT ? "روی اندازه‌گیری" : "روی خطا",
           beta,
           use_integral ? "فعال" : "صفر (PD)");
    printf("============================================\r\n");
    printf("%4s | %8s | %8s | %8s\r\n", "زمان", "هدف", "سرعت", "ولتاژ");
    printf("--------------------------------------------\r\n");
}

/* ================================================================
 * تیک کنترلر
 * ================================================================ */
void example03_tick(void)
{
    ctx.tick++;

    /* اعمال بار ضربه‌ای در t=2.0 ثانیه */
    if (ctx.tick == 2000) {
        ctx.load_torque = 0.010f;   /* ۱۰ mNm */
        printf(">>> بار ضربه‌ای اعمال شد <<<\r\n");
    }

    /* ۱. به‌روزرسانی موتور */
    motor_step(ctx.output, CTRL_DT);

    /* ۲. خوندن سنسور */
    float speed = read_encoder_speed();

    /* ۳. محاسبه PID */
    float control = PID_Update(&ctx.pid, speed);
    ctx.output = control;

    /* ۴. ذخیره D_term برای نمایش */
    #if PIDX_ENABLE_DIAGNOSTICS
    PID_Status st;
    PID_GetStatus(&ctx.pid, &st);
    ctx.d_term = st.d_term;
    #else
    ctx.d_term = 0;
    #endif

    /* گزارش */
    if (ctx.tick % 100 == 0) {   /* هر ۰.۱ ثانیه */
        printf("%4.1f | %8.0f | %8.1f | %8.2f\r\n",
               (float)ctx.tick * CTRL_DT,
               CTRL_SETPOINT, speed, control);
    }

    /* پایان در ۴ ثانیه */
    if (ctx.tick >= 4000) {
        printf("--------------------------------------------\r\n");
        printf("✅ مثال ۳ تمام شد. سرعت نهایی: %.1f rad/s\r\n",
               ctx.motor_w);
        printf("============================================\r\n\r\n");
    }
}


/* ================================================================== */
#ifdef TEST_ON_PC
int main(void)
{
    /* تست ۱: Derivative روی اندازه‌گیری (بدون kick) */
    example03_init(PID_DERIV_ON_MEASUREMENT, 1.0f, 1);
    for (int i = 0; i < 4000; i++) example03_tick();

    /* تست ۲: Derivative روی خطا (با kick) */
    example03_init(PID_DERIV_ON_ERROR, 1.0f, 1);
    for (int i = 0; i < 4000; i++) example03_tick();

    return 0;
}
#endif
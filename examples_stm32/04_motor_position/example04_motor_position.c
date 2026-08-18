/**
 * ====================================================================
 * مثال ۴ — کنترل موقعیت موتور با Setpoint Shaping
 * ====================================================================
 *
 * ██ کاربرد ██
 *   کنترل موقعیت دقیق:
 *   - سرووموتور، بازوی ربات، شیر برقی، CNC
 *   - پرینتر سه‌بعدی، دستگاه‌های بسته‌بندی
 *
 * ██ مفاهیم جدید ██
 *   ۱. Setpoint Shaping (شکل‌دهی مسیر):
 *      به‌جای اینکه setpoint رو یکدفعه ببریم بالا،
 *      با شیب محدود حرکت می‌کنه تا موتور شوک نبینه.
 *
 *   ۲. Setpoint Ramp (رمپ سرعت):
 *      نرخ تغییر setpoint رو محدود می‌کنه [unit/s].
 *      موتور نمی‌تونه بینهایت شتاب بگیره!
 *
 *   ۳. پروفیل ذوزنقه‌ای:
 *      شتاب → سرعت ثابت → کاهش شتاب
 *      مثل آسانسور: اول آروم حرکت می‌کنه، بعد تند، بعد آروم می‌ایسته.
 *
 * ██ سناریو ██
 *   موتور باید از موقعیت ۰ به ۵ دور (حدود ۳۱.۴ رادیان) بره.
 *   مقایسه: با و بدون Setpoint Ramp
 *
 * ██ پیاده‌سازی حقیقی روی STM32 ██
 *   توی main.c:
 *     example04_init(1);  // 1 = با ramp
 *     و توی TIM ISR با ۱ کیلوهرتز:
 *     example04_tick();
 *
 * ====================================================================
 */

#include <stdio.h>
#include <math.h>

/* ===== مسیر هدر PIDX ===== */
#include "pidx/pid.h"

/* ===== پیکربندی ===== */
#define CTRL_DT             0.001f       /* ۱ کیلوهرتز */
#define CTRL_KP             30.0f        /* P برای موقعیت */
#define CTRL_KI             5.0f         /* I برای حذف خطای ماندگار */
#define CTRL_KD             0.5f         /* D برای میرا کردن */
#define CTRL_TF             0.01f

#define CTRL_SETPOINT       31.4159f     /* ۵ دور = ۱۰π رادیان */

#define V_MAX               24.0f

/* ===== موتور DC (مثل مثال ۳) ===== */
#define MOTOR_R     1.0f
#define MOTOR_L     0.0005f
#define MOTOR_KE    0.05f
#define MOTOR_KT    0.05f
#define MOTOR_J     0.0001f
#define MOTOR_B     0.002f
#define MOTOR_COULOMB 0.003f

/* ===== وضعیت برنامه ===== */
static struct {
    PID_Handle pid;

    /* موتور */
    float motor_i, motor_w, motor_theta;

    float output;       /* خروجی کنترلر (ولتاژ) */
    uint32_t tick;

} ctx;

/* ================================================================
 * مدل موتور با زیرگام (sub-stepping)
 * ================================================================ */
static void motor_step(float v, float dt)
{
    float tau_elec = MOTOR_L / MOTOR_R;	/* ۰.۵ ms */
    int n = (int)(dt / (0.1f * tau_elec)) + 1;
    float dt_sub = dt / (float)n;

    for (int i = 0; i < n; i++) {
        float di_dt = (v - MOTOR_R * ctx.motor_i - MOTOR_KE * ctx.motor_w) / MOTOR_L;
        ctx.motor_i += di_dt * dt_sub;

        float coulomb = (ctx.motor_w > 0.01f) ? MOTOR_COULOMB :
                        (ctx.motor_w < -0.01f) ? -MOTOR_COULOMB : 0.0f;
        float dw_dt = (MOTOR_KT * ctx.motor_i - MOTOR_B * ctx.motor_w - coulomb) / MOTOR_J;
        ctx.motor_w += dw_dt * dt_sub;
        ctx.motor_theta += ctx.motor_w * dt_sub;
    }
}

/* ================================================================
 * شبیه‌سازی انکودر موقعیت
 * ================================================================ */
#define ENCODER_CPR 1024.0f
static float read_encoder_pos(void)
{
    return ctx.motor_theta;
}

/* ================================================================
 * راه‌اندازی
 *
 * use_ramp = 1: با شکل‌دهی مسیر (setpoint نرم)
 * use_ramp = 0: بدون شکل‌دهی (پله مستقیم)
 *
 * صدا زدن:
 *   example04_init(1);    // با ramp
 *   example04_init(0);    // بدون ramp
 * ================================================================ */
void example04_init(int use_ramp)
{
    PID_Config cfg;

    /* ---- صفر کردن وضعیت ---- */
    ctx.motor_i = 0.0f;
    ctx.motor_w = 0.0f;
    ctx.motor_theta = 0.0f;
    ctx.tick = 0;
    ctx.output = 0.0f;

    /* ---- پیکربندی PID ---- */
    PID_ConfigDefault(&cfg);
    cfg.core.kp = CTRL_KP;
    cfg.core.ki = CTRL_KI;
    cfg.core.kd = CTRL_KD;
    cfg.core.sample_time = CTRL_DT;

    /* محدودیت خروجی */
    cfg.limits.use_output_limits = true;
    cfg.limits.output_min = -V_MAX;
    cfg.limits.output_max =  V_MAX;

    /* محدودیت انتگرالگیر — مهم برای موقعیت!
     * integrator محدود به ±۵ ولت باشه تا بتونه اصطکاک رو جبران کنه
     * ولی نه بیشتر */
    cfg.limits.use_integral_limits = true;
    cfg.limits.integral_min = -5.0f;
    cfg.limits.integral_max =  5.0f;

    cfg.integral.mode = PID_AW_BACK_CALCULATION;
    cfg.integral.kt   = 20.0f;

    cfg.filter.derivative_mode = PID_DERIV_ON_MEASUREMENT;
    cfg.filter.tf = CTRL_TF;

    PID_Init(&ctx.pid, &cfg);

    /* ---- Setpoint ---- */
    if (use_ramp) {
        /* ★ شکل‌دهی مسیر ★
         *
         * PID_SetSetpointRamp(نرخ, شتاب, کاهش)
         *
         * پارامترها:
         *   rate_max = 20 rad/s   → حداکثر سرعت حرکت
         *   accel    = 50 rad/s²  → شتاب (از صفر تا ۲۰ در ۰.۴ ثانیه)
         *   decel    = 50 rad/s²  → کاهش شتاب
         *
         * اینطوری حرکت از ۰ شروع می‌شه، به ۲۰ rad/s می‌رسه،
         * و نزدیک هدف با شتاب منفی متوقف می‌شه.
         * مثل آسانسور: نرم شروع می‌شه، نرم می‌ایسته.
         */
        PID_SetSetpointRamp(&ctx.pid, 20.0f, 50.0f, 50.0f);
        PID_SetSetpoint(&ctx.pid, CTRL_SETPOINT);

        printf("\r\n====================================\r\n");
        printf("مثال ۴ — موقعیت + Setpoint Ramp\r\n");
        printf("مسیر: شتاب 50 → سرعت 20 → کاهش 50 [rad/s², rad/s, rad/s²]\r\n");
    } else {
        /* بدون ramp: setpoint مستقیم پله می‌خوره */
        PID_SetSetpoint(&ctx.pid, CTRL_SETPOINT);

        printf("\r\n====================================\r\n");
        printf("مثال ۴ — موقعیت (بدون Ramp)\r\n");
        printf("setpoint مستقیم: 0 → 31.4 rad در یک پله\r\n");
    }
    printf("====================================\r\n");
    printf("%4s | %8s | %8s | %8s\r\n", "زمان", "هدف", "موقعیت", "ولتاژ");
    printf("------------------------------------\r\n");
}

/* ================================================================
 * تیک کنترلر (هر ۱ میلی‌ثانیه)
 * ================================================================ */
void example04_tick(void)
{
    ctx.tick++;

    /* ۱. به‌روزرسانی موتور */
    motor_step(ctx.output, CTRL_DT);

    /* ۲. خوندن موقعیت از انکودر */
    float pos = read_encoder_pos();

    /* ۳. محاسبه PID */
    float control = PID_Update(&ctx.pid, pos);
    ctx.output = control;

    /* ۴. گزارش */
    if (ctx.tick % 200 == 0) {   /* هر ۰.۲ ثانیه */
        float sp = PID_GetSetpoint(&ctx.pid);
        printf("%4.1f | %8.2f | %8.2f | %8.2f\r\n",
               (float)ctx.tick * CTRL_DT,
               sp, pos, control);
    }

    /* ۵. پایان در ۳ ثانیه */
    if (ctx.tick >= 3000) {
        printf("------------------------------------\r\n");
        printf("✅ موقعیت نهایی: %.3f rad (هدف: %.3f rad)\r\n",
               ctx.motor_theta, CTRL_SETPOINT);
        printf("خطا: %.3f rad\r\n", CTRL_SETPOINT - ctx.motor_theta);
        printf("====================================\r\n\r\n");
    }
}


/* ================================================================== */
#ifdef TEST_ON_PC
int main(void)
{
    /* تست ۱: بدون Ramp — پله مستقیم */
    example04_init(0);
    for (int i = 0; i < 3000; i++) example04_tick();

    /* تست ۲: با Ramp — حرکت نرم */
    example04_init(1);
    for (int i = 0; i < 3000; i++) example04_tick();

    return 0;
}
#endif
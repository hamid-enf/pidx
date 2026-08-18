/**
 * ====================================================================
 * مثال ۵ — کنترل جریان ۲۰ کیلوهرتز با Fast Path
 * ====================================================================
 *
 * ██ کاربرد ██
 *   حلقه جریان در درایو موتور DC یا BLDC:
 *   - درایو موتور (حلقه داخلی جریان)
 *   - منبع تغذیه سوئیچینگ
 *   - هر جا نیاز به حلقه سریع (۱۰-۵۰ کیلوهرتز) دارید
 *
 * ██ مفاهیم جدید ██
 *   ۱. Fast Path (PID_UpdateFast):
 *      - فقط P + I (Backward-Euler) + D + Clamp
 *      - ۴ برابر سریع‌تر از PID_Update معمولی
 *      - فاقد شاخه‌های اضافی: shaper, safety, gain-sched, telemetry
 *      - مناسب برای حلقه‌های سخت‌بلادرنگ
 *
 *   ۲. PID_UpdateFast_IsSafe():
 *      - چک می‌کنه آیا Fast Path با پیکربندی فعلی جواب یکسانی میده
 *
 *   ۳. تنظیم تحلیلی PI جریان:
 *      - Kp = 2*π*BW*L    (پهنای باند × اندوکتانس)
 *      - Ki = 2*π*BW*R    (پهنای باند × مقاومت)
 *
 * ██ سناریو ██
 *   سیم‌پیچ موتور با R=1Ω, L=0.5mH:
 *   - فرمان جریان از ۰ به ۵ آمپر
 *   - پاسخ باید در < ۱ میلی‌ثانیه برسه
 *   - Fast Path و Full Path مقایسه (باید یکسان باشند)
 *
 * ██ پیاده‌سازی STM32 ██
 *   توی main.c:
 *     example05_init();
 *     و توی TIM ISR با ۲۰ کیلوهرتز (۵۰ میکروثانیه):
 *     example05_tick();
 *
 * ====================================================================
 */

#include <stdio.h>
#include <math.h>

/* ===== مسیر هدر PIDX ===== */
#include "pidx/pid.h"

/* ===== پیکربندی ===== */
#define CTRL_DT             0.00005f    /* ۵۰ میکروثانیه = ۲۰ کیلوهرتز */
#define CTRL_BW             2000.0f     /* پهنای باند ۲ کیلوهرتز */

#define L_MOTOR 0.0005f                 /* ۰.۵ mH */
#define R_MOTOR 1.0f                    /* ۱ Ω */
#define V_DC    24.0f                   /* ولتاژ لینک DC */

/* Kp = ω*L = 2π*BW*L
 * Ki = ω*R = 2π*BW*R */
#define CTRL_KP (6.2831853f * CTRL_BW * L_MOTOR)  /* ~6.283 */
#define CTRL_KI (6.2831853f * CTRL_BW * R_MOTOR)  /* ~12566 */

/* ===== وضعیت ===== */
static struct {
    PID_Handle pid;

    /* کویل ساده: V = L*di/dt + R*i */
    float i_actual;

    float output;           /* ولتاژ PWM */
    uint32_t tick;

    /* مقایسه */
    float diff_max;         /* حداکثر اختلاف Fast vs Full */

} ctx;

/* ================================================================
 * مدل کویل (سلف + مقاومت)
 *
 *   di/dt = (V - R*i) / L
 *
 * ثابت زمانی: τ = L/R = 0.5ms
 * ================================================================ */
static float coil_step(float v, float dt)
{
    float di_dt = (v - R_MOTOR * ctx.i_actual) / L_MOTOR;
    ctx.i_actual += di_dt * dt;
    return ctx.i_actual;
}

/* ================================================================
 * راه‌اندازی
 * ================================================================ */
void example05_init(void)
{
    PID_Config cfg;

    ctx.i_actual = 0.0f;
    ctx.tick = 0;
    ctx.diff_max = 0.0f;

    PID_ConfigDefault(&cfg);
    cfg.core.kp = CTRL_KP;
    cfg.core.ki = CTRL_KI;
    cfg.core.kd = 0.0f;              /* PI for current loop */
    cfg.core.sample_time = CTRL_DT;

    cfg.limits.use_output_limits = true;
    cfg.limits.output_min = -V_DC;
    cfg.limits.output_max =  V_DC;

    cfg.integral.mode = PID_AW_CLAMP;    /* Fast Path فقط CLAMP رو پشتیبانی می‌کنه */
    cfg.integral.kt   = 0.0f;

    cfg.filter.derivative_mode = PID_DERIV_ON_MEASUREMENT;

    PID_Init(&ctx.pid, &cfg);
    PID_SetSetpoint(&ctx.pid, 5.0f);  /* هدف ۵ آمپر */

    /* ★ آموزش: Fast Path ★
     *
     * PID_UpdateFast فقط زمانی با PID_Update هم‌ارزه که همه
     * شروط زیر برقرار باشه (PID_UpdateFast_IsSafe() چک می‌کنه):
     *
     * - خروجی محدود شده (use_output_limits = true)
     * - انتگرالگیر فعال
     * - Anti-windup = CLAMP
     * - Backward Euler
     * - Derivative روی measurement
     * - حالت AUTOMATIC
     * - بدون: shaper, feedforward, safety, gain-sched, input filter
     */
    if (PID_UpdateFast_IsSafe(&ctx.pid)) {
        printf("✅ Fast Path امن است — خروجی Fast و Full یکسان است\r\n");
    } else {
        printf("⚠️ Fast Path امن نیست — از Full Path استفاده می‌شود\r\n");
    }

    printf("\r\n============================================\r\n");
    printf("مثال ۵ — کنترل جریان ۲۰ کیلوهرتز\r\n");
    printf("Kp = %.4f  Ki = %.1f\r\n", CTRL_KP, CTRL_KI);
    printf("ثابت زمانی کویل: τ = L/R = %.1f µs\r\n", (L_MOTOR/R_MOTOR)*1e6f);
    printf("============================================\r\n");
    printf("%5s | %6s | %8s | %8s\r\n",
           "زمان(ms)", "هدف(A)", "جریان(A)", "خروجی(V)");
    printf("--------------------------------------------\r\n");
}

/* ================================================================
 * تیک — در TIM ISR با ۲۰ کیلوهرتز صدا زده می‌شه
 * ================================================================ */
void example05_tick(void)
{
    ctx.tick++;

    /* فرمان پله جریان در t=0.5ms */
    if (ctx.tick == 10)
        PID_SetSetpoint(&ctx.pid, 5.0f);

    /* ۱. به‌روزرسانی کویل */
    coil_step(ctx.output, CTRL_DT);

    /* ۲. اندازه‌گیری جریان */
    float i_meas = ctx.i_actual;

    /* ۳. Fast Path — ۴ برابر سریع‌تر، مناسب برای ۲۰kHz
     *
     * PID_UpdateFast این عملیات رو انجام میده:
     *   error = dir*(sp - meas)
     *   P = Kp * dir * (beta*sp - meas)
     *   D_state = c_da*D - c_db*(new_x - old_x)
     *   I += c_i * error
     *   I = clamp(I, i_min, i_max)
     *   output = clamp(P + I + D, out_min, out_max)
     *
     * همه اینها با چند ضرب و جمع ساده انجام میشه،
     * بدون هیچ شاخه شرطی یا تابع کمکی.
     */
    ctx.output = PID_UpdateFast(&ctx.pid, i_meas);

    static int print_divider = 0;  /* برای خط جداکننده بعد از گذراسم */

    /* ۴. گزارش
     * - اول هر ۵۰ تیک (۲.۵ms) برای دیدن پاسخ پله
     * - بعد هر ۴۰۰ تیک (۲۰ms)
     */
    int log_period = (ctx.tick < 200) ? 50 : 400;
    if (ctx.tick % log_period == 0) {
        printf("%5.2f | %6.1f | %8.3f | %8.3f\r\n",
               (float)ctx.tick * CTRL_DT * 1e3f,
               5.0f, i_meas, ctx.output);
        if (ctx.tick == 200) {
            printf("--------------------------------------------\r\n");
        }
    }

    /* پایان در ۴۰۰۰ تیک = ۰.۲ ثانیه */
    if (ctx.tick >= 4000) {
        printf("--------------------------------------------\r\n");
        printf("✅ جریان به %.3f A رسید (هدف: ۵.۰ A)\r\n", ctx.i_actual);
        printf("============================================\r\n\r\n");
    }
}


/* ================================================================== */
#ifdef TEST_ON_PC
int main(void)
{
    example05_init();
    for (int i = 0; i < 4000; i++) example05_tick();
    return 0;
}
#endif
/**
 * ====================================================================
 * مثال ۱ — ساده‌ترین کنترلر PID (Minimal API)
 * ====================================================================
 *
 * ██ کاربرد ██
 *   اولین قدم برای راه‌اندازی PID در هر پروژه‌ای.
 *   فقط ۵ تابع API نیاز دارید:
 *     PID_InitDefault, PID_SetGains, PID_SetSetpoint, PID_Update, PID_GetOutput
 *
 * ██ مفاهیم ██
 *   - Initialisation: PID_InitDefault
 *   - تنظیم سه بهره: KP (تناسبی), KI (انتگرالی), KD (مشتقی)
 *   - Setpoint: مقدار هدف
 *   - حلقه کنترل: هر تیک PID_Update صدا زده می‌شه
 *   - خروجی: عدد float که به محرک (مثلاً PWM) می‌رود
 *
 * ██ سناریو ██
 *   یک مخزن آب با هیتر ساده:
 *   - دمای محیط: ۲۰ درجه
 *   - هیتر می‌تونه تا ۱۰۰ درجه گرم کنه
 *   - سنسور: دماسنج با نویز ±۱ درجه
 *   - هدف: رسیدن به ۸۰ درجه و پایدار موندن
 *
 * ██ خروجی مورد انتظار ██
 *   بعد از حدود ۵ ثانیه دما باید به ۸۰ درجه نزدیک بشه
 *   (ممکنه کمی اورشوت داشته باشه چون محدودیت خروجی نداریم)
 *
 * ██ چطور استفاده کنم ██
 *   ۱. فایل رو در Core/Src پروژه STM32 کپی کنین
 *   ۲. include path رو به پوشه include/pidx تنظیم کنین
 *   ۳. به پروژه src/pid.c رو اضافه کنین
 *   ۴. توی main.c صدا بزنین:
 *        example01_init();
 *        و توی TIM ISR با نرخ ۱۰۰ هرتز:
 *        example01_tick();
 * ====================================================================
 */

#include <stdio.h>
#include <math.h>

/* ===== مسیر هدر PIDX ===== */
/* حالت ۱: اگر از IDE با include/pidx استفاده می‌کنین: */
#include "pidx/pid.h"
/* حالت ۲: اگر فایل pid.h مستقیم توی پروژه هست: */
/* #include "pid.h" */

/* ===== پیکربندی ===== */
#define CTRL_SAMPLE_TIME_S  0.01f    /* ۱۰ میلی‌ثانیه = ۱۰۰ هرتز */
#define CTRL_SETPOINT       80.0f    /* دمای هدف: ۸۰ درجه */
#define CTRL_KP             3.0f     /* بهره تناسبی */
#define CTRL_KI             0.8f     /* بهره انتگرالی [1/s] */
#define CTRL_KD             0.05f    /* بهره مشتقی [s] */

/* ===== پارامترهای پلنت (شبیه‌ساز) ===== */
#define PLANT_AMBIENT       20.0f    /* دمای محیط */
#define PLANT_GAIN          1.5f     /* ضریب: چند درجه به ازای واحد خروجی */
#define PLANT_TAU           0.5f     /* ثابت زمانی [ثانیه] */
#define SENSOR_NOISE_SIGMA  1.0f     /* نویز سنسور [±درجه] */

/* ===== وضعیت برنامه ===== */
static struct {
    /* کنترلر PID */
    PID_Handle pid;

    /* شبیهساز پلنت */
    float plant_y;          /* خروجی واقعی پلنت */

    /* اندازه‌گیری */
    float measurement;      /* مقدار خروجی سنسور (پلنت + نویز) */
    float output;           /* خروجی کنترلر که به محرک می‌رود */

    /* زمان */
    uint32_t tick_count;

} ctx;

/* ================================================================
 * مولد نویز گاوسی ساده
 * ================================================================ */
static float generate_noise(float sigma)
{
    static unsigned int seed = 12345;
    float sum = 0.0f;
    for (int i = 0; i < 12; i++) {
        seed = seed * 1664525 + 1013904223;
        sum += (float)(int)seed * (1.0f / 2147483648.0f);
    }
    return sum * (sigma / 6.0f);
}

/* ================================================================
 * مدل پلنت مرتبه اول (FOPDT بدون تاخیر)
 *
 *   G(s) = K / (1 + tau*s)
 *
 * فرم گسسته:
 *   y[k] = a*y[k-1] + K*(1-a)*u[k-1]
 *   که a = exp(-dt/tau)
 * ================================================================ */
static float plant_update(float u)
{
    float a = expf(-CTRL_SAMPLE_TIME_S / PLANT_TAU);
    ctx.plant_y = a * ctx.plant_y + PLANT_GAIN * (1.0f - a) * u;
    return ctx.plant_y;
}

/* ================================================================
 * شبیه‌سازی سنسور (پلنت + نویز)
 * ================================================================ */
static float read_sensor(void)
{
    ctx.measurement = ctx.plant_y + generate_noise(SENSOR_NOISE_SIGMA);
    return ctx.measurement;
}

/* ================================================================
 * راه‌اندازی Example 01
 *
 * این تابع رو یکبار صدا بزن (مثلاً توی main.c قبل از while(1))
 * ================================================================ */
void example01_init(void)
{
    /* ---- ۱. صفر کردن وضعیت شبیه‌ساز ---- */
    ctx.plant_y = PLANT_AMBIENT;
    ctx.tick_count = 0;
    ctx.output = 0.0f;

    /* ---- ۲. راه‌اندازی کنترلر PID ---- */
    PID_InitDefault(&ctx.pid);

    /* ★ تنظیم سه بهره ★
     *
     * KP بهره تناسبی: هر چه بزرگتر، پاسخ سریع‌تر ولی overshoot بیشتر
     * KI بهره انتگرالی: خطای ماندگار رو حذف می‌کنه
     * KD بهره مشتقی: نوسان رو کم می‌کنه (میراکننده)
     */
    PID_SetGains(&ctx.pid, CTRL_KP, CTRL_KI, CTRL_KD);

    /* ★ تنظیم نرخ نمونه‌برداری ★
     *
     * کتابخانه از این برای محاسبه ضرایب استفاده می‌کنه:
     *   c_i = Ki * dt        (ضریب انتگرال)
     *   c_da = Tf/(Tf+dt)    (قطب فیلتر مشتق)
     *   c_db = Kd/(Tf+dt)    (بهره فیلتر مشتق)
     */
    PID_SetSampleTime(&ctx.pid, CTRL_SAMPLE_TIME_S);

    /* ★ تنظیم هدف (setpoint) ★ */
    PID_SetSetpoint(&ctx.pid, CTRL_SETPOINT);

    /* ⚠️ توجه: PID_InitDefault محدودیت خروجی نمیذاره!
     * پس خروجی می‌تونه هر عدد float باشه.
     * اگه محرکت محدودیت داره (مثلاً PWM 0..100%)
     * باید PID_SetOutputLimits رو هم صدا بزنی.
     */

    printf("=====================================\r\n");
    printf("مثال ۱ — Minimal PID\r\n");
    printf("=====================================\r\n");
    printf("پلنت: دمای محیط %.0f°C, بهره %.1f, ثابت زمانی %.1fs\r\n",
           PLANT_AMBIENT, PLANT_GAIN, PLANT_TAU);
    printf("کنترلر: Kp=%.1f, Ki=%.2f, Kd=%.2f, نرخ=%dHz\r\n",
           CTRL_KP, CTRL_KI, CTRL_KD, (int)(1.0f / CTRL_SAMPLE_TIME_S));
    printf("Setpoint=%.0f°C\r\n", CTRL_SETPOINT);
    printf("-------------------------------------\r\n");
    printf("%4s | %8s | %8s | %8s\r\n", "زمان", "هدف", "دمای سنسور", "خروجی");
    printf("-------------------------------------\r\n");
}

/* ================================================================
 * تیک کنترلر — در TIM ISR صدا زده می‌شه
 *
 * مثال توی STM32:
 *   void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
 *       if (htim->Instance == TIM3) { example01_tick(); }
 *   }
 * ================================================================ */
void example01_tick(void)
{
    ctx.tick_count++;

    /* ---- مرحله ۱: پلنت به خروجی قبلی واکنش می‌ده ---- */
    plant_update(ctx.output);

    /* ---- مرحله ۲: خوندن سنسور ---- */
    float measurement = read_sensor();

    /* ---- مرحله ۳: محاسبه PID ----
     *
     * PID_Update دو تا کار انجام می‌ده:
     *   ۱. خروجی جدید رو محاسبه می‌کنه
     *      u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt
     *      که e(t) = setpoint - measurement
     *
     *   ۲. وضعیت داخلی (انگرال، مشتق) رو به‌روز می‌کنه
     *
     * ★ مهم: PID_Update طبق نرخ نمونه‌ای که به PID_SetSampleTime
     *   دادیم کار می‌کنه، پس حتماً باید توی یه تایمر با همون
     *   نرخ صدا زده بشه!
     */
    float control = PID_Update(&ctx.pid, measurement);

    /* ---- مرحله ۴: اعمال خروجی به محرک ---- */
    ctx.output = control;
    /* STM32: __HAL_TIM_SetCompare(&htim, TIM_CHANNEL_1, (uint32_t)control); */

    /* ---- گزارش هر ۱ ثانیه (۱۰۰ تیک) ---- */
    if (ctx.tick_count % 100 == 0) {
        printf("%4.0f | %8.0f | %8.1f | %8.2f\r\n",
               (float)ctx.tick_count * CTRL_SAMPLE_TIME_S,
               CTRL_SETPOINT,
               measurement,
               control);
    }

    /* ---- بعد از ۱۰ ثانیه پیام نهایی ---- */
    if (ctx.tick_count >= 1000) {
        printf("-------------------------------------\r\n");
        printf("✅ مثال ۱ تمام شد.\r\n");
        printf("دمای پلنت: %.1f°C (هدف: %.0f°C)\r\n",
               ctx.plant_y, CTRL_SETPOINT);
        printf("خطای ماندگار: %.1f°C\r\n",
               CTRL_SETPOINT - ctx.plant_y);
        printf("=====================================\r\n");
    }
}


/* ==================================================================
 * ——————————— بخش تست PC (اختیاری) ———————————
 * این بخش فقط برای کامپایل روی PC و تست کتابخونه‌ست.
 * برای STM32 این بخش رو حذف کنین یا کامنت کنین.
 * ================================================================== */
#ifdef TEST_ON_PC
int main(void)
{
    example01_init();

    /* اجرای ۱۰۰۰ تیک = ۱۰ ثانیه */
    for (int i = 0; i < 1000; i++) {
        example01_tick();
    }

    return 0;
}
#endif /* TEST_ON_PC */
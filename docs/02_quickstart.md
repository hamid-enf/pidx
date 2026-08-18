# ۰۲ — شروع سریع

هدف: از صفر تا یک حلقهٔ بستهٔ کارکن. همهٔ قطعه‌کدهای این صفحه کامپایل‌شده و
اجرا شده‌اند.

---

## گام ۱ — اضافه کردن به پروژه

هیچ build system‌ای لازم نیست:

1. `src/*.c` را به پروژه اضافه کنید.
2. `include/` را به مسیر include اضافه کنید.
3. `#include "pidx/pid.h"`.

همین. نه CMake اجباری، نه کتابخانهٔ خارجی، نه `math.h` در مسیر داغ.

```bash
gcc -std=c99 -Iinclude your_app.c src/*.c -lm -o app
```

> `-lm` فقط به خاطر ماژول auto-tune (`sqrtf`) لازم است. با
> `-DPIDX_PROFILE_MINIMAL` به آن هم نیازی نیست.

---

## گام ۲ — کمینهٔ کارکن

```c
#include "pidx/pid.h"

static PID_Handle pid;

void control_init(void)
{
    PID_InitDefault(&pid);                    /* پیش‌فرض‌های امن */
    PID_SetGains(&pid, 2.0f, 0.5f, 0.1f);     /* Kp, Ki, Kd */
    PID_SetSampleTime(&pid, 0.01f);           /* ۱۰ میلی‌ثانیه */
    PID_SetOutputLimits(&pid, 0.0f, 100.0f);  /* حد واقعی محرک */
    PID_SetSetpoint(&pid, 60.0f);
}

void control_tick(void)      /* هر ۱۰ میلی‌ثانیه صدا زده شود */
{
    float y = read_sensor();
    float u = PID_Update(&pid, y);
    write_actuator(u);
}
```

**چهار نکته که ۹۰٪ مشکلات از آن‌هاست:**

۱. **`PID_SetOutputLimits` را واقعاً ست کنید.** حد محرکتان را بدهید (مثلاً
   `0..100` برای یک PWM درصدی). بدون آن، anti-windup نمی‌داند اشباع کجاست.

۲. **`control_tick` را با نرخ ثابت صدا بزنید.** اگر نمی‌توانید، از
   `PID_UpdateDt()` استفاده کنید و `dt` واقعی را بدهید.

۳. **خروجی برگشتی را واقعاً به محرک بدهید.** اگر جایی clamp اضافه‌ای می‌کنید
   که کتابخانه از آن خبر ندارد، anti-windup کور می‌شود.

۴. **مقدار برگشتی توابع config را چک کنید** (حداقل هنگام توسعه):

```c
if (PID_SetOutputLimits(&pid, 0.0f, 100.0f) != PID_OK) {
    /* پیکربندی رد شد - ادامه ندهید */
}
```

---

## گام ۳ — انتخاب بهره‌ها

اگر هیچ ایده‌ای ندارید، بگذارید کتابخانه پیدا کند:

```c
#include "pidx/pid_autotune.h"

static PID_AutoTune tuner;

void tune_start(void)
{
    PID_AutoTuneConfig tc;
    PID_AutoTune_ConfigDefault(&tc, PID_IDENT_STEP);

    tc.rule        = PID_RULE_AMIGO_STEP;   /* بهترین معاملهٔ کلی */
    tc.structure   = PID_STRUCT_PID;
    tc.output_step = 0.5f;      /* اندازهٔ پله روی خروجی */
    tc.output_min  = 0.0f;
    tc.output_max  = 1.0f;
    tc.meas_min    = 10.0f;     /* محدودهٔ ایمن فرایند */
    tc.meas_max    = 200.0f;
    tc.timeout_s   = 600.0f;

    PID_AutoTune_Init(&tuner, &tc);
    PID_AutoTune_Start(&tuner, &pid, 60.0f);
}

void control_tick(void)    /* همان تیک، حالا با tuner */
{
    float y = read_sensor();

    if (PID_AutoTune_IsRunning(&tuner)) {
        write_actuator(PID_AutoTune_Update(&tuner, y, 0.01f));
        if (PID_AutoTune_IsComplete(&tuner)) {
            PID_AutoTune_Apply(&tuner, &pid);   /* بدون ضربه */
        }
    } else {
        write_actuator(PID_Update(&pid, y));
    }
}
```

**هیچ `while` مسدودکننده‌ای نیست.** ماشین حالت است؛ بقیهٔ سیستم بی‌وقفه
کار می‌کند.

### چرا `AMIGO_STEP`؟

چون ۸۱۰ اجرای شبیه‌سازی نشان داد بهترین معامله است: رتبهٔ ۳ در کارایی، رتبهٔ ۳
در مقاومت، و **کمترین بدترین‌حالت IAE** در کل مطالعه. جزئیات در §۱۴.

⚠️ **`PID_RULE_ZN` را به‌عنوان پیش‌فرض انتخاب نکنید.** روی مدل دقیق اول است
ولی وقتی مدل ±۳۰٪ خطا داشته باشد (یعنی همیشه) هفتم می‌شود.

---

## گام ۴ — تنظیم دستی، اگر ترجیح می‌دهید

ترتیبی که واقعاً جواب می‌دهد:

1. `Ki = 0`, `Kd = 0`. `Kp` را بالا ببرید تا پاسخ سریع شود ولی نوسان نکند.
2. `Ki` را کم‌کم اضافه کنید تا خطای ماندگار صفر شود. اگر اورشوت زیاد شد،
   `Ki` را کم کنید نه `Kp` را.
3. `Kd` فقط اگر لازم است. **روی سیگنال نویزی `Kd` معمولاً ضرر است** مگر با
   فیلتر (§۰۹).

> اورشوت زیاد معمولاً تقصیر `Ki` است نه `Kp`. در آزمایش‌های ما کم‌کردن `Kp`
> در یک حلقهٔ integral-dominated اورشوت را **بدتر** کرد. (§۹.۱۰.۲ سند معماری.)

---

## گام ۵ — چیزهایی که احتمالاً بعداً می‌خواهید

```c
/* حلقه را دستی/خودکار کنید - بدون ضربه به محرک */
PID_SetMode(&pid, PID_MODE_MANUAL);
PID_SetManualOutput(&pid, 42.0f);
PID_SetMode(&pid, PID_MODE_AUTOMATIC);   /* bumpless */

/* setpoint را نرم کنید تا محرک شوک نخورد */
PID_SetSetpointRamp(&pid, 5.0f, 0.0f, 0.0f);  /* نرخ، شتاب، کاهش */

/* مشتق را روی اندازه‌گیری بگیرید تا پرش setpoint مشتق نزند */
PID_SetDerivativeMode(&pid, PID_DERIV_ON_MEASUREMENT);

/* فیلتر مشتق - تقریباً همیشه لازم */
PID_SetDerivativeFilterN(&pid, 10.0f);
```

---

## عیب‌یابی سریع

| نشانه | علت محتمل |
|---|---|
| خروجی همیشه صفر | `PID_SetGains()` صدا زده نشده (بهره‌ها از صفر شروع می‌شوند) |
| خروجی می‌چسبد به حد بالا | windup — `PID_SetOutputLimits()` را ست کنید |
| خطای ماندگار نمی‌رود | `Ki == 0`، یا setpoint خارج از توان محرک |
| خروجی پرنویز | `Kd` روی سیگنال نویزی — §۰۹ |
| ضربه هنگام تغییر بهره | از `PID_SetGainsRescaleIntegral()` استفاده کنید |
| `PID_Init` کد ۵ برمی‌گرداند | `invalid limit` — back-calculation بدون حد خروجی |

جدول کامل در §۲۰.

---

## قدم بعدی

- §۰۳ — ریاضیات پشت این کد
- §۰۴ تا §۰۷ — API لایه به لایه
- §۱۸ — یکپارچه‌سازی STM32 (تایمر، ADC، ISR)
- `examples/` — ده مثال کامپایل‌شونده

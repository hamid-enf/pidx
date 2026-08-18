# ۰۵ — API سطح ۲: متوسط

## حدها

```c
PID_SetOutputLimits(&pid, 0.0f, 100.0f);
PID_ClearOutputLimits(&pid);
PID_SetIntegralLimits(&pid, -50.0f, 50.0f);
```

**حد خروجی را همیشه ست کنید.** بدون آن anti-windup نمی‌داند اشباع کجاست، و
`PID_AW_BACK_CALCULATION` اصلاً مقداردهی نمی‌شود (کد ۵).

## نرخ نمونه‌برداری

```c
PID_SetSampleTime(&pid, 0.01f);        /* برای PID_Update */
PID_Float dt = PID_GetSampleTime(&pid);

/* یا dt را هر بار بدهید: */
float u = PID_UpdateDt(&pid, y, actual_dt);
```

`PID_UpdateDt` وقتی مفید است که نرخ واقعاً متغیر است (مثلاً یک تسک RTOS با
jitter). اگر `dt` عوض نشده باشد، ضرایب کش‌شده استفاده می‌شوند و هزینه فقط
یک مقایسه است.

**اندازه‌گیری‌شده:** `dt` ثابت ~۱۲.۲ ns، `dt` متغیر ~۱۶.۷ ns — یعنی
بازمحاسبه حدود ۳۸٪ گران‌تر است.

## Anti-windup

```c
PID_SetAntiWindup(&pid, PID_AW_BACK_CALCULATION, 1.0f);
```

جزئیات کامل در §۰۸.

## مشتق

```c
PID_SetDerivativeMode(&pid, PID_DERIV_ON_MEASUREMENT);
PID_SetDerivativeFilterN(&pid, 10.0f);     /* Tf = Td/N */
PID_SetDerivativeFilter(&pid, 0.02f);      /* یا مستقیم Tf */
```

جزئیات در §۰۹.

## جهت عمل

```c
PID_SetDirection(&pid, PID_DIRECT);    /* خروجی↑ ⇒ اندازه‌گیری↑ */
PID_SetDirection(&pid, PID_REVERSE);   /* خروجی↑ ⇒ اندازه‌گیری↓ */
```

`REVERSE` برای محرک‌های سرمایشی: وقتی دما بالا می‌رود، خروجی باید بالا برود.

**از بهرهٔ منفی برای این کار استفاده نکنید** — رد می‌شود.

## حالت‌ها

```c
PID_SetMode(&pid, PID_MODE_MANUAL);
PID_SetManualOutput(&pid, 42.0f);
PID_SetMode(&pid, PID_MODE_AUTOMATIC);   /* bumpless */

PID_Mode m = PID_GetMode(&pid);
```

در حالت MANUAL انتگرال‌گر خروجی دستی را **دنبال می‌کند**، پس برگشت به
AUTOMATIC بدون ضربه است.

`PID_MODE_HOLD` هم هست: انتگرال‌گر **فریز** می‌شود ولی P و D زنده می‌مانند.

## بهره‌های تکی

```c
PID_SetKp(&pid, 2.0f);
PID_SetKi(&pid, 0.5f);
PID_SetKd(&pid, 0.1f);

PID_Float kp, ki, kd;
PID_GetGains(&pid, &kp, &ki, &kd);
```

> یکی از نه نقص پیداشده: `PID_GetGains` روی یک handle نامعتبر `PID_OK`
> برمی‌گرداند و مقادیر آشغال پشتهٔ فراخوان را دست‌نخورده می‌گذاشت. حالا
> اعتبارسنجی می‌شود.

## تغییر بهره با حفظ خروجی

```c
PID_SetGainsRescaleIntegral(&pid, new_kp, new_ki, new_kd);
```

انتگرال‌گر را طوری مقیاس می‌کند که خروجی **دقیقاً** ثابت بماند. برای وقتی
که در حال کار، بهره‌ها را عوض می‌کنید.

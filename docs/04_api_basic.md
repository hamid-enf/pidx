# ۰۴ — API سطح ۱: مقدماتی

شش تابع. اگر فقط همین‌ها را یاد بگیرید، ۸۰٪ کاربردها پوشش داده می‌شود.

## `PID_InitDefault`

```c
PID_StatusCode PID_InitDefault(PID_Handle *h);
```

پیش‌فرض‌های امن: `Kp=Ki=Kd=0`، `dt=10ms`، عمل مستقیم، حالت خودکار، مشتق روی
اندازه‌گیری، anti-windup از نوع clamp، بدون حد خروجی، `beta=1`, `gamma=0`.

**بهره‌ها عمداً صفرند** — کنترلر تا `PID_SetGains()` خروجی صفر می‌دهد. یک
نقطهٔ شروع بی‌اثر و امن، نه یک کنترلر تصادفی که محرک را تکان می‌دهد.

## `PID_SetGains`

```c
PID_StatusCode PID_SetGains(PID_Handle *h, PID_Float kp, PID_Float ki, PID_Float kd);
```

هر سه با هم، **بدون ضربه**. چون انتگرال‌گر در واحد خروجی ذخیره می‌شود (§۰۳)،
تغییر بهره خروجی را نمی‌پراند.

بهره‌های منفی رد می‌شوند (`PID_ERR_INVALID_GAIN`). برای عمل معکوس از
`PID_SetDirection()` استفاده کنید، نه بهرهٔ منفی.

## `PID_SetSetpoint`

```c
PID_StatusCode PID_SetSetpoint(PID_Handle *h, PID_Float sp);
```

اگر shaper فعال باشد، setpoint به‌تدریج به مقصد می‌رود. برای پرش فوری:
`PID_SetSetpointImmediate()`.

## `PID_Update`

```c
PID_Float PID_Update(PID_Handle *h, PID_Float measurement);
```

از `sample_time` پیکربندی‌شده استفاده می‌کند. **باید با نرخ ثابت صدا زده
شود** — معمولاً از یک ISR تایمر.

## `PID_Reset`

```c
PID_StatusCode PID_Reset(PID_Handle *h);
```

انتگرال‌گر، حالت مشتق و پرچم‌ها را صفر می‌کند. بهره‌ها و پیکربندی دست‌نخورده
می‌مانند.

بعد از توقف طولانی حلقه، `PID_Reset()` را صدا بزنید وگرنه انتگرال‌گر کهنه
یک ضربه می‌سازد.

## `PID_GetOutput`

```c
PID_Float PID_GetOutput(const PID_Handle *h);
```

آخرین خروجی، بدون محاسبهٔ مجدد.

---

## کد کامل

```c
static PID_Handle pid;

void setup(void)
{
    PID_InitDefault(&pid);
    PID_SetGains(&pid, 2.0f, 0.5f, 0.1f);
    PID_SetSampleTime(&pid, 0.01f);
    PID_SetOutputLimits(&pid, 0.0f, 100.0f);
    PID_SetSetpoint(&pid, 60.0f);
}

void timer_isr(void)   /* هر ۱۰ms */
{
    write_actuator(PID_Update(&pid, read_sensor()));
}
```

## کدهای بازگشتی

همهٔ توابع پیکربندی `PID_StatusCode` برمی‌گردانند. **حداقل در زمان توسعه
چکشان کنید:**

| کد | معنا |
|---|---|
| `PID_OK` | موفق |
| `PID_ERR_NULL` | اشاره‌گر NULL |
| `PID_ERR_INVALID_GAIN` | بهرهٔ منفی یا غیرمتناهی |
| `PID_ERR_INVALID_DT` | `dt <= 0` یا خارج از بازهٔ مجاز |
| `PID_ERR_INVALID_LIMIT` | `min >= max` یا ترکیب ناسازگار |
| `PID_ERR_NOT_INIT` | handle مقداردهی نشده |

`PID_StatusToString(code)` رشتهٔ خوانا می‌دهد.

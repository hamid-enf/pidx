# ۰۷ — API سطح ۴: Expert

## مسیر سریع

```c
PID_Float PID_UpdateFast(PID_Handle *h, PID_Float measurement);
bool      PID_UpdateFast_IsSafe(const PID_Handle *h);
```

کمینهٔ مطلق: خطا، P با وزن، انتگرال، مشتق فیلترشده، جمع، clamp. **بدون**
بررسی حالت، بدون shaper، بدون ایمنی، بدون تله‌متری، بدون بررسی NaN.

**اندازه‌گیری‌شده:** ~۲.۹۵ در برابر ~۱۲.۱ ns — **حدود ۴.۱ برابر سریع‌تر**.

### قرارداد

```c
if (!PID_UpdateFast_IsSafe(&pid)) {
    /* پیکربندی شما قابلیتی دارد که مسیر سریع نادیده می‌گیرد.
     * استفاده از UpdateFast نتیجهٔ متفاوتی می‌دهد. */
}
```

**همیشه این را در زمان راه‌اندازی چک کنید.** اگر `false` برگرداند و باز هم
`UpdateFast` استفاده کنید، نتیجه بی‌سروصدا با `PID_Update` فرق می‌کند.

### چرا branchless است

`PID_UpdateFast` حتی وقتی `Ki=0` است، حساب انتگرال را انجام می‌دهد. این
عمدی است: **زمان اجرای ثابت** برای یک ISR سخت‌بلادرنگ ارزشمندتر از چند سیکل
صرفه‌جویی است.

شاهدش در benchmark: پیکربندی P-only دقیقاً همان زمان پیکربندی کامل را
می‌گیرد (اختلاف داخل نویز اجرا‌به‌اجرا).

### تنها بررسی باقی‌مانده

`h == NULL`. هزینه‌اش ۱۳ بایت و **هیچ زمان قابل‌اندازه‌گیری** (شاخه کاملاً
پیش‌بینی می‌شود). یک segfault در ISR چیزی نیست که بشود با آن معامله کرد.

## `PID_UpdateEx`

```c
PID_Float PID_UpdateEx(PID_Handle *h, const PID_Input *in, PID_StatusCode *err);
```

کامل‌ترین شکل: setpoint، اندازه‌گیری، `dt`، feedforward خارجی و سیگنال
tracking را یکجا می‌گیرد و کد خطا را جدا برمی‌گرداند.

```c
PID_Input in;
PID_InputInit(&in);          /* حتماً - وگرنه فیلدهای آشغال */
in.measurement = y;
in.setpoint    = sp;
in.dt          = dt;

PID_StatusCode err;
float u = PID_UpdateEx(&pid, &in, &err);
```

## قوانین تنظیم سفارشی

```c
static PID_StatusCode my_rule(const PID_PlantModel *m, PID_TuneStructure s,
                              PID_Gains *out, void *ctx)
{
    (void)s; (void)ctx;
    if (m->kind != PID_MODEL_FOPDT) { return PID_ERR_TUNE_MODEL_MISMATCH; }
    out->kp = 0.5f / m->k;
    out->ti = m->t;
    out->ki = out->kp / out->ti;
    out->kd = 0.0f;
    return PID_OK;
}

PID_AutoTune_RegisterRule(&tuner, my_rule, NULL);
tc.rule = PID_RULE_CUSTOM;
```

## تنظیم مجدد نظارت‌شده

```c
PID_AutoTune_Retune(&tuner, &pid);
```

از مدل ذخیره‌شده دوباره بهره تولید می‌کند **بدون** تحریک دوبارهٔ پلنت.

## چیدمان حافظه

```c
/* روی STM32 در CCMRAM بگذارید - دسترسی سریع‌تر، بدون رقابت با DMA */
__attribute__((section(".ccmram"))) static PID_Handle pid;
```

| ساختار | اندازه |
|---|---|
| `PID_Handle` | ۳۴۴ B |
| `PID_Config` | ۱۸۴ B |
| `PID_Status` | ۷۶ B |
| `PID_AutoTune` | ۴۰۰ B |
| `PIDq_Handle` | ۸۰ B |

`PID_Config` فقط در زمان `PID_Init` لازم است — می‌شود روی پشته گذاشتش و بعد
رهایش کرد.

## نسخه و ABI

```c
/* PIDX_VERSION_NUM = MAJOR*10000 + MINOR*100 + PATCH  →  1.0.0 = 10000 */
#if PIDX_VERSION_NUM < 10000
#error "PIDX 1.0.0 or newer required"
#endif

cfg.abi_version = PIDX_CONFIG_ABI_VERSION;   /* ConfigDefault خودش می‌زند */
```

API سطح ۱ (مقدماتی) در MAJOR 1 **قفل** است و تغییر نمی‌کند.

اگر `PID_Config` بین نسخه‌ها عوض شود، `PID_Init` پیکربندی قدیمی را رد
می‌کند به‌جای اینکه بایت‌ها را اشتباه تفسیر کند.

# ۰۶ — API سطح ۳: پیشرفته

## وزن‌دهی setpoint (2-DOF)

```c
PID_SetWeights(&pid, 0.5f, 0.0f);   /* beta, gamma - هر دو در [0,2] */
```

§۱۰.

## Feedforward

```c
PID_SetFeedforward(&pid, 0.5f);
PID_SetFeedforwardFn(&pid, my_ff, ctx, 1.0f);
```

§۱۱.

## شکل‌دهی مسیر

```c
PID_SetSetpointRamp(&pid, 5.0f, 2.0f, 2.0f);  /* نرخ، شتاب، کاهش */
PID_SetOutputSlewRate(&pid, 20.0f);           /* حد |du/dt| */
```

با شتاب غیرصفر، پروفایل سرعت ذوزنقه‌ای ساخته می‌شود که در فاصلهٔ
$v^2/(2a)$ ترمز می‌کند و بدون اورشوت می‌نشیند.

> ⚠️ **اندازه‌گیری‌شده:** slew limiting روی یک سروو ۱ کیلوهرتز نتیجه را
> **بدتر** کرد. این ابزار برای محافظت از سخت‌افزار است، نه بهبود پاسخ.

## فیلتر ورودی

```c
PID_SetInputFilter(&pid, 0.05f);   /* ثابت زمانی LPF */
```

تأخیر فاز اضافه می‌کند. اول فیلتر مشتق را امتحان کنید (§۰۹).

## جداسازی انتگرال

```c
PID_SetIntegralSeparation(&pid, 10.0f);   /* |e| > 10 ⇒ انباشت نکن */
```

§۰۸.

## ایمنی

```c
cfg.safety.enabled         = true;
cfg.safety.meas_min        = -10.0f;
cfg.safety.meas_max        = 150.0f;
cfg.safety.meas_rate_max   = 50.0f;
cfg.safety.failsafe_output = 0.0f;
cfg.safety.fault_persist_n = 3;
cfg.safety.auto_recover    = true;
```

§۱۵.

## وضعیت و پرچم‌ها

```c
uint16_t flags = PID_GetFlags(&pid);

PID_Status st;
PID_GetStatus(&pid, &st);      /* P، I، D، خطا، خروجی خام و... */

PID_StatusCode code;
PID_GetLastError(&pid, &code);
PID_ClearError(&pid);
```

`PID_GetStatus` برای دیباگ عالی است: سهم هر جمله را جدا می‌بینید و می‌فهمید
کدام‌شان دارد حلقه را می‌راند.

## پیکربندی یکجا

به‌جای ده فراخوانی جدا:

```c
PID_Config cfg;
PID_ConfigDefault(&cfg);          /* abi_version را هم مهر می‌زند */

cfg.core.kp = 2.0f;
cfg.core.ki = 0.5f;
cfg.core.kd = 0.1f;
cfg.core.sample_time = 0.01f;
cfg.limits.use_output_limits = true;
cfg.limits.output_min = 0.0f;
cfg.limits.output_max = 100.0f;
cfg.integral.mode = PID_AW_BACK_CALCULATION;
cfg.integral.kt = 1.0f;
cfg.filter.derivative_mode = PID_DERIV_ON_MEASUREMENT;
cfg.filter.n_filter = 10.0f;
cfg.weight.beta = 0.7f;

if (PID_Init(&pid, &cfg) != PID_OK) { /* رد شد */ }
```

**`PID_Init` قبل از هر تغییری کل پیکربندی را اعتبارسنجی می‌کند.** اگر رد
شود، handle دست‌نخورده می‌ماند — نه نیمه‌پیکربندی.

### مسیر فیلدها

| گروه | فیلدها |
|---|---|
| `cfg.core` | `kp, ki, kd, sample_time, direction, mode, integration` |
| `cfg.limits` | `use_output_limits, output_min/max, use_integral_limits, integral_min/max, dt_min/max` |
| `cfg.integral` | `mode` (anti-windup)، `kt, separation_threshold, deadband, enabled` |
| `cfg.filter` | `derivative_mode, tf, n_filter, input_lpf_tau` |
| `cfg.weight` | `beta, gamma` |
| `cfg.feedforward` | `fn, value, ctx, gain` |
| `cfg.shaper` | `sp_rate_max, sp_accel, sp_decel, out_slew_max` |
| `cfg.safety` | `enabled, meas_min/max, meas_rate_max, failsafe_output, fault_persist_n, auto_recover` |

> ⚠️ anti-windup در `cfg.integral.mode` است، نه `cfg.limits`.

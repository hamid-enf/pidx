# ۲۰ — رفع اشکال

## جدول سریع

| نشانه | علت محتمل | راه‌حل |
|---|---|---|
| خروجی همیشه صفر | بهره‌ها هنوز صفرند | `PID_SetGains()` |
| خروجی همیشه صفر | حالت MANUAL با خروجی صفر | `PID_SetMode(AUTOMATIC)` |
| خروجی به حد چسبیده | windup | `PID_SetOutputLimits()` + anti-windup |
| خروجی به حد چسبیده | setpoint دست‌نیافتنی | محرک را بزرگ‌تر کنید |
| خطای ماندگار نمی‌رود | `Ki == 0` | `Ki` بدهید |
| خطای ماندگار نمی‌رود | deadband بزرگ‌تر از خطا | deadband را کم کنید |
| نوسان پیوسته | `Kp` زیاد | `Kp` را نصف کنید |
| نوسان پیوسته | تأخیر مرده زیاد | نرخ نمونه یا ساختار cascade |
| خروجی پرنویز | `Kd` روی سیگنال نویزی | فیلتر مشتق (§۰۹) |
| اورشوت زیاد | `Ki` زیاد (نه `Kp`!) | `Ti` را بکشید |
| ضربه هنگام تغییر بهره | — | `PID_SetGainsRescaleIntegral()` |
| ضربه هنگام MANUAL→AUTO | — | نباید بشود؛ باگ گزارش کنید |
| `PID_Init` کد ۵ می‌دهد | back-calc بدون حد | `use_output_limits = true` |
| `PID_Init` کد ۱۳ می‌دهد | قابلیت کامپایل نشده | پروفایل را عوض کنید |
| auto-tune timeout | `bias` غلط | `auto_bias = true` |
| `UpdateFast` نتیجهٔ متفاوت | قابلیتی که رد می‌شود | `PID_UpdateFast_IsSafe()` |

## عیب‌یابی گام‌به‌گام

### ۱. اول کد خطا را بخوانید

```c
PID_StatusCode code;
PID_GetLastError(&pid, &code);
printf("%s\n", PID_StatusToString(code));
PID_ClearError(&pid);
```

خطا **چسبان** و **اولین** است، نه آخرین — چون خطای اول معمولاً علت است و
بقیه معلول.

### ۲. سهم هر جمله را ببینید

```c
PID_Status st;
PID_GetStatus(&pid, &st);
printf("P=%.3f I=%.3f D=%.3f  u_raw=%.3f u=%.3f\n",
       st.p_term, st.i_term, st.d_term, st.output_raw, st.output);
```

این تشخیصی‌ترین کار ممکن است:

- **`I` خیلی بزرگ** ⇒ windup. حد خروجی و anti-windup را چک کنید.
- **`D` پرنویز** ⇒ فیلتر مشتق کم است.
- **`u_raw` خیلی بزرگ‌تر از `u`** ⇒ دائم در اشباع.
- **`P` غالب و نوسانی** ⇒ `Kp` زیاد.

### ۳. پرچم‌ها

```c
uint16_t f = PID_GetFlags(&pid);
```

`PID_FLAG_SATURATED_HIGH/LOW`، `PID_FLAG_INTEGRAL_LIMITED`،
`PID_FLAG_SENSOR_INVALID`، `PID_FLAG_FAULT`.

### ۴. سنجه‌ها

```c
static PID_LoopMetrics met;
PID_Metrics_Reset(&met);
/* هر نمونه: */ PID_Metrics_Update(&met, &pid);

float duty = PID_Metrics_SaturationDuty(&met);
```

**`SaturationDuty` بالای ۲۰٪ یعنی مشکل تنظیم نیست، مشکل سایز محرک است.**
هیچ مجموعه بهره‌ای این را درست نمی‌کند.

---

## مشکلات رایج به تفصیل

### «حلقه نوسان می‌کند ولی بهره‌ها را کم کردم»

اگر کم کردن `Kp` کمک نکرد، احتمالاً مشکل از `Ki` است. یک حلقهٔ
integral-dominated با کم کردن `Kp` **بدتر** می‌شود.

اندازه‌گیری‌شده: با `Ti = Pu/2` ثابت، کم کردن `Kp` از `0.60Ku` تا `0.10Ku`
اورشوت را ۴۴.۲٪ → ۴۱.۶٪ کرد (تقریباً هیچ). کِش دادن `Ti` تا `4Pu` اورشوت را
به **۰.۰٪** رساند.

**پس: `Ti` را بکشید، نه `Kp` را کم کنید.**

### «auto-tune همیشه timeout می‌شود»

۹۵٪ مواقع: `bias` روی خروجی‌ای نیست که setpoint را نگه دارد، پس حلقه هرگز
از setpoint عبور نمی‌کند و رله روی یک ریل می‌چسبد.

```c
tc.auto_bias = true;    /* خروجی فعلی کنترلر را بگیر */
```

یا دستی: `tc.bias = setpoint / process_gain;`

**۵٪ باقی:** setpoint واقعاً دست‌نیافتنی است. اگر هیتر شما حداکثر به ۲۲ درجه
می‌رسد و هدف ۶۰ است، هیچ تنظیمی جوابگو نیست. کتابخانه درست گزارش می‌دهد.

### «نتایج روی PC و روی برد فرق دارند»

- `double` در برابر `float`: روی PC ممکن است `double` استفاده شود.
- FPU خاموش روی برد.
- نرخ نمونهٔ واقعی با آنچه فکر می‌کنید فرق دارد — با یک GPIO toggle چک کنید.
- ADC نویزی‌تر از شبیه‌سازی است.

### «بعد از مدتی خروجی عجیب می‌شود»

- سرریز شمارندهٔ زمان. `PIDs_DeltaUs()` این را درست مدیریت می‌کند؛ حساب
  دستی معمولاً نه.
- انتگرال‌گر به حد رسیده و `PID_FLAG_INTEGRAL_LIMITED` روشن است.
- در fixed-point: مقیاس‌بندی اشتباه و سرریز Q15.

### «`PID_UpdateFast` نتیجهٔ متفاوتی می‌دهد»

طبق قرارداد. مسیر سریع shaper، ایمنی، gain scheduling، feedforward و
بررسی NaN را **رد می‌کند**.

```c
if (!PID_UpdateFast_IsSafe(&pid)) { /* از PID_Update استفاده کنید */ }
```

---

## کدهای وضعیت کامل

| کد | معنا |
|---|---|
| `PID_OK` | موفق |
| `PID_ERR_NULL` | اشاره‌گر NULL |
| `PID_ERR_NOT_INIT` | handle از `PID_Init` رد نشده |
| `PID_ERR_INVALID_CONFIG` | پیکربندی اعتبارسنجی نشد |
| `PID_ERR_INVALID_GAIN` | بهره NaN/Inf یا علامت غیرمجاز |
| `PID_ERR_INVALID_LIMIT` | `min >= max` یا حد NaN |
| `PID_ERR_INVALID_DT` | `dt <= 0` یا خارج از `[dt_min, dt_max]` |
| `PID_ERR_INVALID_MODE` | مقدار `PID_Mode` ناشناخته |
| `PID_ERR_INVALID_PARAM` | آرگومان خارج از بازه |
| `PID_ERR_NAN_INPUT` | اندازه‌گیری یا setpoint برابر NaN |
| `PID_ERR_INF_INPUT` | ورودی بی‌نهایت |
| `PID_ERR_SENSOR_RANGE` | خارج از محدودهٔ پیکربندی‌شده |
| `PID_ERR_SENSOR_RATE` | پرش سریع‌تر از حد مجاز |
| `PID_ERR_UNSUPPORTED` | قابلیت کامپایل نشده |
| `PID_ERR_BUSY` | عملیات در وضعیت فعلی مجاز نیست |
| `PID_ERR_TUNE_TIMEOUT` | بودجهٔ زمانی auto-tune تمام شد |
| `PID_ERR_TUNE_UNSTABLE` | نوسان واگرا شد |
| `PID_ERR_TUNE_NO_OSCILLATION` | رله نوسان قابل‌استفاده نساخت |
| `PID_ERR_TUNE_MODEL_MISMATCH` | قانون مدل دیگری می‌خواهد |
| `PID_ERR_TUNE_ABORTED` | لغو توسط کاربر/watchdog |
| `PID_ERR_TUNE_VALIDATION` | بهره‌های تولیدشده معقول نبودند |

`PID_StatusToString(code)` رشتهٔ خوانا می‌دهد.

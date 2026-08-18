# ۲۱ — مرجع کامل API

این فایل **به‌صورت خودکار از هدرها استخراج شده** تا هرگز با کد واگرا نشود.
برای توضیح مفهومی هر بخش، به فصل مربوطه مراجعه کنید.

قرارداد نام‌گذاری:

| پیشوند | معنا |
|---|---|
| `PID_` | API عمومی |
| `PIDq_` | ممیز ثابت |
| `PIDs_` | لایهٔ STM32 |
| `PIDp_` | لایهٔ POSIX |
| `PIDX_` | ماکروی پیکربندی |
| `pidm_/pidf_/pidc_/pidd_/pidt_/pidp_` | داخلی — استفاده نکنید |


## `pid.h` — هستهٔ کنترلر

```c
PID_StatusCode PID_InitDefault(PID_Handle *h);
PID_StatusCode PID_SetGains(
        PID_Handle *h, PID_Float kp, PID_Float ki, PID_Float kd);
PID_StatusCode PID_SetSetpoint(PID_Handle *h, PID_Float setpoint);
PID_Float PID_Update(PID_Handle *h, PID_Float measurement);
PID_StatusCode PID_Reset(PID_Handle *h);
PID_Float PID_GetOutput(const PID_Handle *h);
PID_StatusCode PID_ConfigDefault(PID_Config *cfg);
PID_StatusCode PID_Init(PID_Handle *h, const PID_Config *cfg);
PID_StatusCode PID_Deinit(PID_Handle *h);
PID_StatusCode PID_SetSampleTime(PID_Handle *h, PID_Float dt);
PID_Float PID_GetSampleTime(const PID_Handle *h);
PID_Float PID_UpdateDt(PID_Handle *h, PID_Float measurement, PID_Float dt);
PID_StatusCode PID_SetOutputLimits(
        PID_Handle *h, PID_Float min, PID_Float max);
PID_StatusCode PID_ClearOutputLimits(PID_Handle *h);
PID_StatusCode PID_SetIntegralLimits(
        PID_Handle *h, PID_Float min, PID_Float max);
PID_StatusCode PID_SetAntiWindup(
        PID_Handle *h, PID_AntiWindup mode, PID_Float kt);
PID_StatusCode PID_SetDerivativeMode(PID_Handle *h, PID_DerivativeMode mode);
PID_StatusCode PID_SetDerivativeFilter(PID_Handle *h, PID_Float tf);
PID_StatusCode PID_SetDerivativeFilterN(PID_Handle *h, PID_Float n);
PID_StatusCode PID_SetDirection(PID_Handle *h, PID_Direction dir);
PID_StatusCode PID_SetMode(PID_Handle *h, PID_Mode mode);
PID_Mode PID_GetMode(const PID_Handle *h);
PID_StatusCode PID_SetManualOutput(PID_Handle *h, PID_Float output);
PID_Float PID_GetManualOutput(const PID_Handle *h);
PID_StatusCode PID_SetSetpointRamp(
        PID_Handle *h, PID_Float rate_max, PID_Float accel, PID_Float decel);
PID_StatusCode PID_SetOutputSlewRate(PID_Handle *h, PID_Float slew_max);
PID_StatusCode PID_SetInputFilter(PID_Handle *h, PID_Float tau);
void PID_InputInit(PID_Input *in);
PID_Float PID_UpdateEx(
        PID_Handle *h, const PID_Input *in, PID_StatusCode *err);
PID_Float PID_UpdateFast(PID_Handle *h, PID_Float measurement);
bool PID_UpdateFast_IsSafe(const PID_Handle *h);
PID_StatusCode PID_SetKp(PID_Handle *h, PID_Float kp);
PID_StatusCode PID_SetKi(PID_Handle *h, PID_Float ki);
PID_StatusCode PID_SetKd(PID_Handle *h, PID_Float kd);
PID_StatusCode PID_GetGains(
        const PID_Handle *h, PID_Float *kp, PID_Float *ki, PID_Float *kd);
PID_StatusCode PID_SetGainsRescaleIntegral(
        PID_Handle *h, PID_Float kp, PID_Float ki, PID_Float kd);
PID_StatusCode PID_SetWeights(PID_Handle *h, PID_Float beta, PID_Float gamma);
PID_StatusCode PID_SetFeedforward(PID_Handle *h, PID_Float ff);
PID_StatusCode PID_SetFeedforwardFn(
        PID_Handle *h, PID_FeedforwardFn fn, void *ctx, PID_Float gain);
PID_StatusCode PID_SetIntegralSeparation(PID_Handle *h, PID_Float threshold);
PID_StatusCode PID_SetIntegralDeadband(PID_Handle *h, PID_Float db);
PID_StatusCode PID_EnableIntegral(PID_Handle *h, bool enable);
PID_StatusCode PID_SetIntegrator(PID_Handle *h, PID_Float value);
PID_Float PID_GetIntegrator(const PID_Handle *h);
PID_StatusCode PID_SetTrackingInput(PID_Handle *h, PID_Float u_track);
PID_StatusCode PID_SetIntegrationMethod(
        PID_Handle *h, PID_IntegrationMethod m);
PID_StatusCode PID_SetSafety(PID_Handle *h, const PID_SafetyConfig *sc);
PID_StatusCode PID_SetFaultOutput(PID_Handle *h, PID_Float output);
PID_StatusCode PID_ClearFault(PID_Handle *h);
bool PID_IsFaulted(const PID_Handle *h);
PID_StatusCode PID_EnableFeature(PID_Handle *h, uint32_t mask, bool enable);
bool PID_IsFeatureEnabled(const PID_Handle *h, uint32_t mask);
uint16_t PID_GetFlags(const PID_Handle *h);
bool PID_IsSaturated(const PID_Handle *h);
PID_Float PID_GetError(const PID_Handle *h);
PID_Float PID_GetSetpoint(const PID_Handle *h);
PID_StatusCode PID_GetLastError(PID_Handle *h, PID_StatusCode *code);
PID_StatusCode PID_PeekLastError(const PID_Handle *h);
PID_StatusCode PID_ClearError(PID_Handle *h);
PID_StatusCode PID_GetStatus(const PID_Handle *h, PID_Status *out);
const char * PID_StatusToString(PID_StatusCode code);
const char * PID_GetVersion(void);
```

## `pid_autotune.h` — تنظیم خودکار

```c
typedef PID_StatusCode(
        *PID_TuneRuleFn)(const PID_PlantModel *m, PID_TuneStructure s, PID_Gains *out, void *ctx);
PID_StatusCode PID_TuneRule_Apply(
        PID_TuneRule rule, const PID_PlantModel *model, PID_TuneStructure structure, PID_Float lambda, PID_Gains *out);
PID_ModelKind PID_TuneRule_RequiredModel(PID_TuneRule rule);
const char * PID_TuneRule_Name(PID_TuneRule rule);
const char * PID_TuneStateToString(PID_TuneState s);
PID_StatusCode PID_AutoTune_ConfigDefault(
        PID_AutoTuneConfig *cfg, PID_IdentMethod ident);
PID_StatusCode PID_AutoTune_Init(
        PID_AutoTune *t, const PID_AutoTuneConfig *cfg);
PID_StatusCode PID_AutoTune_Start(
        PID_AutoTune *t, PID_Handle *h, PID_Float sp);
PID_Float PID_AutoTune_Update(
        PID_AutoTune *t, PID_Float measurement, PID_Float dt);
PID_StatusCode PID_AutoTune_Abort(PID_AutoTune *t);
bool PID_AutoTune_IsComplete(const PID_AutoTune *t);
bool PID_AutoTune_IsRunning(const PID_AutoTune *t);
PID_TuneState PID_AutoTune_GetState(const PID_AutoTune *t);
PID_StatusCode PID_AutoTune_GetError(const PID_AutoTune *t);
uint8_t PID_AutoTune_GetProgress(const PID_AutoTune *t);
PID_StatusCode PID_AutoTune_GetResult(
        const PID_AutoTune *t, PID_AutoTuneResult *r);
PID_StatusCode PID_AutoTune_Apply(PID_AutoTune *t, PID_Handle *h);
PID_StatusCode PID_AutoTune_RegisterRule(
        PID_AutoTune *t, PID_TuneRuleFn fn, void *ctx);
PID_StatusCode PID_AutoTune_Retune(
        PID_AutoTune *t, PID_TuneRule rule, PID_TuneStructure structure);
```

## `pid_cascade.h` — کنترل آبشاری

```c
PID_StatusCode PID_Cascade_Init(
        PID_Cascade *c, PID_Handle *const *loops, uint8_t n);
PID_StatusCode PID_Cascade_ConfigLevel(
        PID_Cascade *c, uint8_t index, uint16_t decimation, PID_Float sp_min, PID_Float sp_max);
PID_StatusCode PID_Cascade_SetAntiWindup(
        PID_Cascade *c, PID_CascadeAntiWindup mode, PID_Float aw_gain);
PID_Float PID_Cascade_Update(
        PID_Cascade *c, const PID_Float *measurements, PID_Float setpoint, PID_Float dt);
PID_StatusCode PID_Cascade_SetMode(PID_Cascade *c, PID_Mode mode);
PID_StatusCode PID_Cascade_SetManualOutput(PID_Cascade *c, PID_Float output);
PID_StatusCode PID_Cascade_Reset(PID_Cascade *c);
PID_Float PID_Cascade_GetOutput(const PID_Cascade *c);
PID_Float PID_Cascade_GetLevelSetpoint(const PID_Cascade *c, uint8_t index);
PID_Handle * PID_Cascade_GetLoop(const PID_Cascade *c, uint8_t index);
bool PID_Cascade_IsSaturated(const PID_Cascade *c);
PID_StatusCode PID_Cascade_GetLastError(PID_Cascade *c);
PID_StatusCode PID_Cascade_Validate(
        const PID_Cascade *c, PID_Float *min_ratio, uint8_t *worst_index);
```

## `pid_gainsched.h` — Gain scheduling

```c
PID_StatusCode PID_GainSched_Init(
        PID_GainSchedule *s, const PID_GainPoint *points, uint8_t count, PID_SchedSource source, PID_SchedInterp interp);
PID_StatusCode PID_GainSched_SetHysteresis(
        PID_GainSchedule *s, PID_Float band);
PID_StatusCode PID_GainSched_Attach(PID_Handle *h, PID_GainSchedule *s);
PID_StatusCode PID_GainSched_SetVar(PID_Handle *h, PID_Float value);
PID_StatusCode PID_GainSched_Evaluate(
        PID_GainSchedule *s, PID_Float x, PID_Float *kp, PID_Float *ki, PID_Float *kd);
```

## `pid_shaper.h` — شکل‌دهی مسیر

```c
PID_StatusCode PID_Shaper_Init(
        PID_Shaper *s, PID_Float rate_max, PID_Float accel, PID_Float decel);
PID_StatusCode PID_Shaper_SetTarget(PID_Shaper *s, PID_Float target);
PID_StatusCode PID_Shaper_Reset(PID_Shaper *s, PID_Float position);
PID_Float PID_Shaper_Update(PID_Shaper *s, PID_Float dt);
bool PID_Shaper_IsMoving(const PID_Shaper *s);
PID_Float PID_Shaper_EstimateTime(const PID_Shaper *s);
```

## `pid_filter.h` — فیلترها

```c
PID_StatusCode PID_LPF1_Init(PID_LPF1 *f, PID_Float tau, PID_Float dt);
PID_StatusCode PID_LPF1_SetTau(PID_LPF1 *f, PID_Float tau, PID_Float dt);
PID_StatusCode PID_LPF1_SetCutoff(PID_LPF1 *f, PID_Float fc_hz, PID_Float dt);
PID_Float PID_LPF1_Update(PID_LPF1 *f, PID_Float x);
PID_StatusCode PID_LPF1_Reset(PID_LPF1 *f);
PID_StatusCode PID_MovingAvg_Init(
        PID_MovingAvg *f, PID_Float *buffer, uint16_t size);
PID_Float PID_MovingAvg_Update(PID_MovingAvg *f, PID_Float x);
PID_StatusCode PID_MovingAvg_Reset(PID_MovingAvg *f);
PID_StatusCode PID_Median3_Init(PID_Median3 *f);
PID_Float PID_Median3_Update(PID_Median3 *f, PID_Float x);
PID_StatusCode PID_RateLimiter_Init(PID_RateLimiter *f, PID_Float rate_max);
PID_Float PID_RateLimiter_Update(
        PID_RateLimiter *f, PID_Float x, PID_Float dt);
PID_StatusCode PID_RateLimiter_Reset(PID_RateLimiter *f, PID_Float value);
```

## `pid_diag.h` — تشخیص و تله‌متری

```c
PID_StatusCode PID_Telemetry_Init(
        PID_Telemetry *t, PID_TelemetryRecord *storage, uint16_t capacity);
PID_StatusCode PID_Telemetry_Attach(PID_Handle *h, PID_Telemetry *t);
PID_StatusCode PID_Telemetry_Read(PID_Telemetry *t, PID_TelemetryRecord *out);
uint16_t PID_Telemetry_Count(const PID_Telemetry *t);
uint16_t PID_Telemetry_Dropped(PID_Telemetry *t);
PID_StatusCode PID_Telemetry_Flush(PID_Telemetry *t);
PID_StatusCode PID_Metrics_Reset(PID_LoopMetrics *m);
PID_StatusCode PID_Metrics_Update(PID_LoopMetrics *m, const PID_Handle *h);
PID_Float PID_Metrics_MeanAbsError(const PID_LoopMetrics *m);
PID_Float PID_Metrics_SaturationDuty(const PID_LoopMetrics *m);
PID_Float PID_Metrics_OscillationRate(const PID_LoopMetrics *m);
```

## `pid_fixed.h` — ممیز ثابت (Q15/Q31)

```c
PID_StatusCode PIDq_ConfigDefault(PIDq_Config *cfg);
PID_StatusCode PIDq_Init(PIDq_Handle *h, const PIDq_Config *cfg);
PID_StatusCode PIDq_Deinit(PIDq_Handle *h);
PID_StatusCode PIDq_Reset(PIDq_Handle *h);
int16_t PIDq_Update(PIDq_Handle *h, int16_t measurement_q15);
PID_StatusCode PIDq_SetGains(
        PIDq_Handle *h, int32_t kp_q16, int32_t ki_q16, int32_t kd_q16);
PID_StatusCode PIDq_SetSetpoint(PIDq_Handle *h, int16_t sp_q15);
int16_t PIDq_GetSetpoint(const PIDq_Handle *h);
PID_StatusCode PIDq_SetMode(PIDq_Handle *h, PIDq_Mode mode);
PIDq_Mode PIDq_GetMode(const PIDq_Handle *h);
PID_StatusCode PIDq_SetManualOutput(PIDq_Handle *h, int16_t u_q15);
int16_t PIDq_GetManualOutput(const PIDq_Handle *h);
PID_StatusCode PIDq_SetOutputLimits(
        PIDq_Handle *h, int16_t min_q15, int16_t max_q15);
int16_t PIDq_GetOutput(const PIDq_Handle *h);
int16_t PIDq_GetIntegral(const PIDq_Handle *h);
bool PIDq_IsSaturated(const PIDq_Handle *h);
bool PIDq_SelfTest(void);
```

## `pid_types.h` — انواع داده

```c
typedef PID_Float(
        *PID_FeedforwardFn)(PID_Float setpoint, PID_Float measurement, void *ctx);
```

## `pid_stm32.h` — لایهٔ STM32 (اختیاری)

```c
PID_StatusCode PIDs_TimebaseInitTim(TIM_TypeDef *tim, uint32_t timer_clk_hz);
PID_StatusCode PIDs_TimebaseInitDwt(uint32_t core_clk_hz);
PID_StatusCode PIDs_TimebaseInitCallback(
        uint32_t (*fn)(void), uint32_t counter_mask);
uint32_t PIDs_CounterMask(void);
bool PIDs_TimebaseReady(void);
uint32_t PIDs_NowUs32(void);
uint32_t PIDs_DeltaUs(uint32_t earlier, uint32_t later);
uint64_t PIDs_NowUs(void);
PID_Float PIDs_Now(void);
void PIDs_DelayUs(uint32_t us);
PID_StatusCode PIDs_RateInit(PIDs_Rate *r, uint32_t period_us);
bool PIDs_RateElapsed(PIDs_Rate *r);
PID_StatusCode PIDs_CycleInit(void);
uint32_t PIDs_Cycles(void);
void PIDs_CycleReset(PIDs_CycleStat *s);
void PIDs_CycleStart(PIDs_CycleStat *s);
void PIDs_CycleStop(PIDs_CycleStat *s);
PID_Float PIDs_CycleMean(const PIDs_CycleStat *s);
PID_Float PIDs_CyclesToUs(uint32_t cycles);
PID_StatusCode PIDs_IsrMonitorInit(PIDs_IsrMonitor *m, uint32_t nominal_us);
void PIDs_IsrEnter(PIDs_IsrMonitor *m);
void PIDs_IsrExit(PIDs_IsrMonitor *m);
PID_Float PIDs_IsrLoadPercent(const PIDs_IsrMonitor *m);
uint32_t PIDs_EnterCritical(void);
void PIDs_ExitCritical(uint32_t state);
```

## `pid_posix.h` — لایهٔ POSIX (اختیاری)

```c
double PIDp_Now(void);
uint64_t PIDp_NowUs(void);
void PIDp_SleepUs(uint64_t us);
PID_StatusCode PIDp_LoopInit(PIDp_Loop *lp, uint64_t period_us);
double PIDp_LoopWait(PIDp_Loop *lp);
double PIDp_LoopMeanRate(const PIDp_Loop *lp);
void PIDp_TimerReset(PIDp_Timer *t);
void PIDp_TimerStart(PIDp_Timer *t);
void PIDp_TimerStop(PIDp_Timer *t);
double PIDp_TimerMeanUs(const PIDp_Timer *t);
```

---

مجموع: **182** تابع عمومی.

> این فایل از روی هدرها تولید می‌شود. اگر هدری عوض شد، دوباره
> تولیدش کنید تا مرجع کهنه نماند.
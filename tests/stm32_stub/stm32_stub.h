/* Minimal CMSIS look-alike used ONLY to compile and logic-test
 * platform/stm32/pid_stm32.c on the host.
 *
 * This is not an emulator. It gives the platform layer the same register
 * layout, macros and intrinsics that a real CMSIS device header would, backed
 * by plain memory the test can poke, so that the wrap-extension arithmetic,
 * the rate driver, the profiler and the ISR monitor can be exercised without
 * silicon. Anything that depends on the actual bus behaviour of a TIM (shadow
 * register timing, for example) is NOT covered here and has to be verified on
 * hardware.
 */
#ifndef PIDX_STM32_STUB_H
#define PIDX_STM32_STUB_H

#include <stdint.h>

/* ---- TIM ------------------------------------------------------------- */

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
} TIM_TypeDef;

extern TIM_TypeDef pidx_stub_tim2_inst;
#define TIM2  (&pidx_stub_tim2_inst)

/* ---- DWT / CoreDebug --------------------------------------------------- */

typedef struct {
    volatile uint32_t CTRL;
    volatile uint32_t CYCCNT;
    volatile uint32_t LAR;
} DWT_Type;

typedef struct {
    volatile uint32_t DEMCR;
} CoreDebug_Type;

extern DWT_Type       pidx_stub_dwt_inst;
extern CoreDebug_Type pidx_stub_coredebug_inst;

/* Every access advances CYCCNT by pidx_stub_cyc_step, so the counter behaves
 * like a running one and pids_dwt_enable()'s "is it actually counting?" probe
 * can succeed or fail on demand. */
extern uint32_t pidx_stub_cyc_step;
DWT_Type *pidx_stub_dwt(void);

#define DWT        (pidx_stub_dwt())
#define CoreDebug  (&pidx_stub_coredebug_inst)

#define CoreDebug_DEMCR_TRCENA_Msk   (1UL << 24)
#define DWT_CTRL_CYCCNTENA_Msk       (1UL << 0)

/* ---- intrinsics -------------------------------------------------------- */

extern uint32_t pidx_stub_primask;
extern uint32_t pidx_stub_basepri;
extern uint32_t pidx_stub_crit_depth;   /* max observed nesting, for the test */
extern uint32_t pidx_stub_crit_now;

static inline uint32_t __get_PRIMASK(void) { return pidx_stub_primask; }

static inline void __disable_irq(void)
{
    pidx_stub_primask = 1UL;
    pidx_stub_crit_now++;
    if (pidx_stub_crit_now > pidx_stub_crit_depth) {
        pidx_stub_crit_depth = pidx_stub_crit_now;
    }
}

static inline void __set_PRIMASK(uint32_t v)
{
    pidx_stub_primask = v;
    if (pidx_stub_crit_now > 0UL) { pidx_stub_crit_now--; }
}

static inline uint32_t __get_BASEPRI(void)       { return pidx_stub_basepri; }
static inline void     __set_BASEPRI(uint32_t v) { pidx_stub_basepri = v; }

#endif /* PIDX_STM32_STUB_H */

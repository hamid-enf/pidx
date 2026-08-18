/* Backing storage for the CMSIS look-alike. See stm32_stub.h. */
#include "stm32_stub.h"

TIM_TypeDef    pidx_stub_tim2_inst      = {0};
DWT_Type       pidx_stub_dwt_inst       = {0};
CoreDebug_Type pidx_stub_coredebug_inst = {0};

uint32_t pidx_stub_cyc_step   = 1UL;
uint32_t pidx_stub_primask    = 0UL;
uint32_t pidx_stub_basepri    = 0UL;
uint32_t pidx_stub_crit_depth = 0UL;
uint32_t pidx_stub_crit_now   = 0UL;

DWT_Type *pidx_stub_dwt(void)
{
    pidx_stub_dwt_inst.CYCCNT += pidx_stub_cyc_step;
    return &pidx_stub_dwt_inst;
}

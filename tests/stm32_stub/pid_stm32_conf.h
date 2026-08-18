/* Host-test configuration for the STM32 platform layer.
 * Mirrors pid_stm32_conf_template.h but points at the stub device header.
 * Every knob is overridable from the command line so the build gate can also
 * compile the Cortex-M0 shape (no DWT) and the BASEPRI critical section. */
#ifndef PIDX_PID_STM32_CONF_H
#define PIDX_PID_STM32_CONF_H

#define PIDX_STM32_DEVICE_HEADER     "stm32_stub.h"

#ifndef PIDX_STM32_TIMEBASE
#define PIDX_STM32_TIMEBASE          PIDX_STM32_TB_TIM
#endif
#ifndef PIDX_STM32_TIM
#define PIDX_STM32_TIM               TIM2
#endif
#ifndef PIDX_STM32_TIMER_CLK_HZ
#define PIDX_STM32_TIMER_CLK_HZ      84000000UL
#endif
#ifndef PIDX_STM32_CORE_CLK_HZ
#define PIDX_STM32_CORE_CLK_HZ       168000000UL
#endif
#ifndef PIDX_STM32_HAS_DWT
#define PIDX_STM32_HAS_DWT           1
#endif
#ifndef PIDX_STM32_DWT_HAS_LAR
#define PIDX_STM32_DWT_HAS_LAR       0
#endif
#ifndef PIDX_STM32_CRITICAL_BASEPRI
#define PIDX_STM32_CRITICAL_BASEPRI  0
#endif

#endif

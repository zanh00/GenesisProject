#include <stdint.h>
#include <stdbool.h>
#include "../../Drivers/STM32H7xx_HAL_Driver/inc/stm32h7xx_hal_rcc.h"
#include "../../Drivers/STM32H7xx_HAL_Driver/inc/stm32h7xx_hal_tim.h"
#include "../../Drivers/CMSIS/Device/ST/STM32H7xx/Include/stm32h7xx.h"
#include "../../Drivers/CMSIS/Device/ST/STM32H7xx/Include/stm32h755xx.h"


uint32_t ClockHandling_GetTimerClkFreq(TIM_HandleTypeDef *htim);

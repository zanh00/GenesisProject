#include "ClockHandling.h"
#ifndef __IO
#define __IO volatile
#endif

uint32_t ClockHandling_GetTimerClkFreq(TIM_HandleTypeDef *htim)
{
    uint32_t pclk           = 0;
    uint32_t timerClock     = 0;
    uint32_t apbPrescaler   = 0;
    uint32_t timpre         = 0;

    if(htim->Instance == TIM17 || htim->Instance == TIM16 || htim->Instance == TIM15 
        || htim->Instance == TIM8 || htim->Instance == TIM1)
    {
        pclk = HAL_RCC_GetPCLK2Freq();
        apbPrescaler = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE2) >> 8;
    }
    else
    {
        pclk = HAL_RCC_GetPCLK1Freq();
        apbPrescaler = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) >> 4;
    }

    // Get TIMPRE setting
    timpre = (RCC->CFGR & RCC_CFGR_TIMPRE) >> 15; // TIMPRE is bit 15

    switch (apbPrescaler)
    {
    case 0b000:
    case 0b001:
    case 0b010:
    case 0b011:
        timerClock = pclk;
        break;

    case 0b100:
        timerClock = pclk * 2;
        break;

    case 0b101:
        if( timpre == 0 )
        {
            timerClock = pclk * 2;
        }
        else
        {
            timerClock = pclk * 4;
        }
        break;

    case 0b110:
        if(timpre == 0)
        {
            timerClock = pclk * 2;
        }
        else
        {
            timerClock = pclk * 4;
        }
        break;

    case 0b111:
        if(timpre == 0)
        {
            timerClock = pclk * 2;
        }
        else
        {
            timerClock = pclk * 4;
        }
        break;
    default:
        break;
    }

    return timerClock;
}

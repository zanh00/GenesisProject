//////////////////////////////////////////////////////////////////////////////
/*
 * ClockHandling.c
 *
 *  Created on: July 13, 2024
 *      Author: Žan Hertiš
 */
 ////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Includes 
//////////////////////////////////////////////////////////////////////////////

#include "ClockHandling.h"
#ifndef __IO
#define __IO volatile
#endif

//////////////////////////////////////////////////////////////////////////////
// Defines 
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Function Definitions 
//////////////////////////////////////////////////////////////////////////////

uint32_t ClockHandling_GetTimerClkFreq(TIM_HandleTypeDef *htim)
{
    uint32_t pclk           = 0;
    uint32_t timerClock     = 0;
    uint32_t apbPrescaler   = 0;
    uint32_t timpre         = 0;

    if( htim->Instance == TIM17 || htim->Instance == TIM16 || htim->Instance == TIM15 
        || htim->Instance == TIM8 || htim->Instance == TIM1 )
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

void ClockHandling_PWMInit(ClockHandling_PWMConfig_t *config)
{
    uint8_t     presc           = 0u;
    uint32_t    ARR             = 0u;   // auto reload register
    uint32_t    timClk          = 0u;
    uint32_t    internalTimClk  = ClockHandling_GetTimerClkFreq(config->htim);
    uint32_t    sysclk          = HAL_RCC_GetSysClockFreq();

    if( sysclk < 100000000 ) // 100MHz
    {
        presc = 4;
    }
    else
    {
        presc = 8;
    }

    timClk  = internalTimClk / presc;
    ARR     = timClk / config->frequency;

    config->CCRmin = (uint32_t)((config->Dmin * ARR) / 100);
    config->CCRmax = (uint32_t)((config->Dmax * ARR) / 100);

    config->htim->Init.Prescaler    = presc - 1;
    config->htim->Init.Period       = ARR;

    if( HAL_TIM_Base_Init(config->htim) != HAL_OK )
    {
        Error_Handler();
    }
}

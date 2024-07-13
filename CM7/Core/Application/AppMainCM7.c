#include "AppMainCM7.h"

static void Steering_PWMInit(const uint32_t internalTimerClock, const uint32_t sysclk, uint32_t* const CCRmin, uint32_t* const CCRmax);

void AppCM7_Main()
{
    const uint32_t timerClk = ClockHandling_GetTimerClkFreq(&htim2);
    const uint32_t sysClk   = HAL_RCC_GetSysClockFreq();
    uint32_t CCRmin = 0; 
    uint32_t CCRmax = 0;

    Steering_PWMInit(timerClk, sysClk, &CCRmin, &CCRmax);

    TIM2->CCR1 = CCRmax;
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    while(1)
    {

    }
}

static void Steering_PWMInit(const uint32_t internalTimerClock, const uint32_t sysclk, uint32_t* const CCRmin, uint32_t* const CCRmax)
{
    uint8_t     presc   = 0u;
    uint8_t     freq    = 50u;
    uint8_t     Dmin    = 5u;
    uint8_t     Dmax    = 10u;
    uint32_t    ARR     = 0u;
    uint32_t    timClk  = 0u;

    if( sysclk < 100000000) // 100MHz
    {
        presc = 4;
    }
    else
    {
        presc = 8;
    }

    timClk  = internalTimerClock / presc;
    ARR     = timClk / freq;

    *CCRmin = (Dmin * ARR) / 100;
    *CCRmax = (Dmax * ARR) / 100;

    htim2.Init.Prescaler    = presc - 1;
    htim2.Init.Period       = ARR;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
      Error_Handler();
    }
}

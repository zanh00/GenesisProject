//////////////////////////////////////////////////////////////////////////////
/*
 * SpeedEstimation.c
 *
 *  Created on: Oct 25, 2024
 *      Author: Žan Hertiš
 */
 ////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Includes 
//////////////////////////////////////////////////////////////////////////////

#include "SpeedEstimation.h"

//////////////////////////////////////////////////////////////////////////////
// Global Variables 
//////////////////////////////////////////////////////////////////////////////
DMA_BUFFER  static uint32_t gSpeedEstimation_TimeCapturesBurfferd   [SPEEDESTIMATION_SAMPLE_COUNT];
            static uint32_t gSpeedEstimation_TimeCaptures           [SPEEDESTIMATION_SAMPLE_COUNT] = {0};

//////////////////////////////////////////////////////////////////////////////
// Function prototypes 
//////////////////////////////////////////////////////////////////////////////

static float    SpeedEstimation_CalculateSensorOutputFreq   (const uint32_t timerFreq);
static float    SpeedEstimation_CalculateSpeed              (const uint32_t sensOutFreq);
static uint32_t SpeedEstimation_TIM_Init                    (const uint32_t timerPeripherialClk);
static bool     SpeedEstimation_NewValuesCaptured           (void* prevCaptures);

//////////////////////////////////////////////////////////////////////////////
// FreeRTOS Task
//////////////////////////////////////////////////////////////////////////////

void SpeedEstimation_Task(void* pvParameters)
{
    uint32_t            timerClk    = ClockHandling_GetTimerClkFreq(&htim5);
    volatile float               speed       = 0;
    TickType_t          lastWakeTime;
    const TickType_t    taskPeriod  = pdMS_TO_TICKS(SPEEDESTIMATION_PERIOD_MS);
    HAL_StatusTypeDef   status      = HAL_OK;
    
    static uint32_t prevCaptures[SPEEDESTIMATION_SAMPLE_COUNT] = {0};
    
    // In case of larger peripherial frequencies, we reduce the timer frequency
    timerClk        = SpeedEstimation_TIM_Init(timerClk);
    lastWakeTime    = xTaskGetTickCount();

    status = HAL_TIM_IC_Start_DMA(&htim5, TIM_CHANNEL_4, gSpeedEstimation_TimeCapturesBurfferd, SPEEDESTIMATION_PERIOD_MS);
    if( status != HAL_OK )
    {
        Error_Handler();
    }

    while(1)
    {
        if( SpeedEstimation_NewValuesCaptured(prevCaptures) )
        {
            float sensOutFreq   = SpeedEstimation_CalculateSensorOutputFreq(timerClk);
            speed               = SpeedEstimation_CalculateSpeed(sensOutFreq);
        }
        else
        {
            speed = 0;
        }

        //xQueueOverwrite(q_speed, &speed);

        //vTaskDelayUntil(&lastWakeTime, taskPeriod);
        vTaskDelay(50);
    }
}

//////////////////////////////////////////////////////////////////////////////
// Function Definitions 
//////////////////////////////////////////////////////////////////////////////
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if( htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4 )
    {
        //memcpy(gSpeedEstimation_TimeCaptures, gSpeedEstimation_TimeCapturesBurfferd, sizeof(gSpeedEstimation_TimeCaptures));
    }
}

static float SpeedEstimation_CalculateSensorOutputFreq(const uint32_t timerFreq)
{
    double  avgPeriod = 0;
    float   freq      = 0;

    for( uint8_t i = 0; i < SPEEDESTIMATION_SAMPLE_COUNT - 1; i++ )
    {
        avgPeriod += gSpeedEstimation_TimeCaptures[i+1] - gSpeedEstimation_TimeCaptures[i];
    }

    avgPeriod = avgPeriod / (SPEEDESTIMATION_SAMPLE_COUNT - 1);
    freq = (1 / avgPeriod) * timerFreq;
    
    return freq;
}

static float SpeedEstimation_CalculateSpeed(const uint32_t sensOutFreq)
{
    float speed = 0;
    speed = SPEEDESTIMATION_COEFICIENT * sensOutFreq;
    return speed;
}

static uint32_t SpeedEstimation_TIM_Init(const uint32_t timerPeripherialClk)
{
    uint32_t usedTimerClk = timerPeripherialClk;

    if( timerPeripherialClk > 200000000 ) // greater than 200 MHz
    {
        htim5.Init.Prescaler = 1;
        if( HAL_TIM_Base_Init(&htim5) != HAL_OK )
        {
            Error_Handler();
        }

        usedTimerClk = timerPeripherialClk / 2;
    }

    return usedTimerClk;
}

static bool SpeedEstimation_NewValuesCaptured(void* prevCaptures)
{
    bool areNewValues = false;
    //int32_t result = memcmp(gSpeedEstimation_TimeCaptures, prevCaptures, sizeof(gSpeedEstimation_TimeCaptures));
    int32_t result = 0;
    if( result != 0 )
    {
        areNewValues = true;
        memcpy(prevCaptures, gSpeedEstimation_TimeCaptures, sizeof(gSpeedEstimation_TimeCaptures));
    }

    return areNewValues;
}

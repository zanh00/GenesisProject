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

#include "AppMainCM7.h"
#include "SpeedEstimation.h"
#include "ProjectConfig.h"
#include "ClockHandling.h"

//////////////////////////////////////////////////////////////////////////////
// Defines 
//////////////////////////////////////////////////////////////////////////////

#define DMA_BUFFER __attribute__((section(".sram1")))

#define SPEEDESTIMATION_SAMPLE_COUNT    4

#define TYRE_DIAMETE                0.165
#define GEAR_WHEEL_TEETH            20

#define SPEEDESTIMATION_COEFICIENT  ((TYRE_DIAMETE * PI) / GEAR_WHEEL_TEETH)

//////////////////////////////////////////////////////////////////////////////
// Global Variables 
//////////////////////////////////////////////////////////////////////////////

// buffer in sram1 where dma copies data from timer registers
DMA_BUFFER  static uint32_t gSpeedEstimation_TimeCapturesBurfferd   [SPEEDESTIMATION_SAMPLE_COUNT];

// buffer where ISR copies data from dma buffer to be processed
            static uint32_t gSpeedEstimation_TimeCaptures           [SPEEDESTIMATION_SAMPLE_COUNT] = {0};
// measured vehicle speed
            static float            gSpeed = 0;

//////////////////////////////////////////////////////////////////////////////
// Function prototypes 
//////////////////////////////////////////////////////////////////////////////

static float    SpeedEstimation_CalculateSensorOutputFreq   (const uint32_t timerFreq);
static float    SpeedEstimation_CalculateSpeed              (const uint32_t sensOutFreq);
static uint32_t SpeedEstimation_TIM_Init                    (const uint32_t timerPeripherialClk);
static bool     SpeedEstimation_NewValuesCaptured           (void* prevCaptures);
static void     SpeedEstimation_SendSpeedToDiagnostic       (TimerHandle_t xTimer);


//////////////////////////////////////////////////////////////////////////////
// FreeRTOS Task
//////////////////////////////////////////////////////////////////////////////

/**
 *  FreeRTOS task for speed measurement. Task firstly initializes the capture and compare timer
 *  and its dma controller. It than perfroms speed measuremnets and puts them in a queue with 
 *  the defined taskPeriod
 * 
 *  @param taskPeriod   task execution period. Can be set with SPEEDESTIMATION_PERIOD_MS in SpeedEstimation.h
 */
void SpeedEstimation_Task(void* pvParameters)
{
    uint32_t            timerClk    = ClockHandling_GetTimerClkFreq(&htim5);        // initial timer peripherial clock
    TickType_t          lastWakeTime;                                               // used for determining task period. Initialized at the start, 
                                                                                    // later handled by the rtos api. Leave as is.
    const TickType_t    taskPeriod  = pdMS_TO_TICKS(SPEEDESTIMATION_PERIOD_MS);     // task execution period in ticks
    HAL_StatusTypeDef   status      = HAL_OK;                                       // HAL status return variable
    
    static uint32_t prevCaptures[SPEEDESTIMATION_SAMPLE_COUNT] = {0};               // stores previously used time capture values (in calculations) and
                                                                                    // is used to compare it with current ones (so that we don't duplicate calcs.)
    TimerHandle_t    t_DiagPeriod;

    // In case of larger peripherial frequencies, we reduce the timer frequency
    timerClk        = SpeedEstimation_TIM_Init(timerClk);
    lastWakeTime    = xTaskGetTickCount();

    status = HAL_TIM_IC_Start_DMA(&htim5, TIM_CHANNEL_4, gSpeedEstimation_TimeCapturesBurfferd, SPEEDESTIMATION_SAMPLE_COUNT);
    if( status != HAL_OK )
    {
        Error_Handler();
    }

    t_DiagPeriod = xTimerCreate("Speed diag timer", pdMS_TO_TICKS(DIAGNOSTIC_SPEED_PERIOD_MS), pdTRUE, NULL, SpeedEstimation_SendSpeedToDiagnostic);

    if( t_DiagPeriod != NULL )
    {
        xTimerStart(t_DiagPeriod, portMAX_DELAY);
    }

    xEventGroupSetBits(e_statusFlags, SF_SPEED_ESTIMATION_TASK_ACTIVE);
    
    while(1)
    {
        /* 
            If the timer captured values are not the same as in previous execution, calculate new speed,
            otherwise the rotation is slow enough that we can assume speed as 0. 
        */
        if( SpeedEstimation_NewValuesCaptured(prevCaptures) )
        {
            float sensOutFreq   = SpeedEstimation_CalculateSensorOutputFreq(timerClk);
            gSpeed              = SpeedEstimation_CalculateSpeed(sensOutFreq);
        }
        else
        {
            gSpeed = 0;
        }

        // Write the speed to the queue
        xQueueOverwrite(q_speed, &gSpeed);

        // Sets the delay time which is equal to taskPeriod - task execution time
        vTaskDelayUntil(&lastWakeTime, taskPeriod);
    }
}

//////////////////////////////////////////////////////////////////////////////
// Function Definitions 
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
/**
 *  ISR callback function, that is called when dma transfers the number of samples determined
 *  by the SPEEDESTIMATION_SAMPLE_COUNT to the dma buffer. Function than copies the values to the 
 *  other buffer and defers the procesing to the rtos task.
 *  
 *  TODO: We probably need to protect the gSpeedEstimation_TimeCaptures buffer with the mutex.
 */
//////////////////////////////////////////////////////////////////////////////
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if( htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4 )
    {
        memcpy(gSpeedEstimation_TimeCaptures, gSpeedEstimation_TimeCapturesBurfferd, sizeof(gSpeedEstimation_TimeCaptures));
    }
}

//////////////////////////////////////////////////////////////////////////////
/**
 *  Function calculates the frequency produced by the sensor. It computes the average
 *  period of all the sample and multiplies by the timer frequncy.
 * 
 *  @param      timerFreq    Timer frequency
 * 
 *  @return     computed sensor oupt frequency.
 * 
 */
//////////////////////////////////////////////////////////////////////////////
static float SpeedEstimation_CalculateSensorOutputFreq(const uint32_t timerFreq)
{
    double  avgPeriod = 0;
    float   freq      = 0;

    taskENTER_CRITICAL();
    for( uint8_t i = 0; i < SPEEDESTIMATION_SAMPLE_COUNT - 1; i++ )
    {
        avgPeriod += gSpeedEstimation_TimeCaptures[i+1] - gSpeedEstimation_TimeCaptures[i];
    }
    taskEXIT_CRITICAL();

    avgPeriod = avgPeriod / (SPEEDESTIMATION_SAMPLE_COUNT - 1);
    freq = (1 / avgPeriod) * timerFreq;
    
    return freq;
}

//////////////////////////////////////////////////////////////////////////////
/**
 *  Calculates the speed by multipling the sensor oupt frequency by the
 *  SPEEDESTIMATION_COEFICIENT which is determined by the wheel parameters.
 * 
 *  @param      sensOutFreq     output frequency of the speed sensor
 * 
 *  @return     speed [m/s]
 */
//////////////////////////////////////////////////////////////////////////////
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

//////////////////////////////////////////////////////////////////////////////
/**
 *  Checks if the stored values in the buffer are different than previously. 
 *  If they are it returns true and stores the new values, if not, returns false
 * 
 *  @param      prevCaptures    Pointer to previus capture values
 * 
 *  @return     True if values are new, false otherwise.
 */
//////////////////////////////////////////////////////////////////////////////
static bool SpeedEstimation_NewValuesCaptured(void* prevCaptures)
{
    bool areNewValues = false;
    int32_t result = memcmp(gSpeedEstimation_TimeCaptures, prevCaptures, sizeof(gSpeedEstimation_TimeCaptures));

    if( result != 0 )
    {
        areNewValues = true;
        memcpy(prevCaptures, gSpeedEstimation_TimeCaptures, sizeof(gSpeedEstimation_TimeCaptures));
    }

    return areNewValues;
}

//////////////////////////////////////////////////////////////////////////////
/**
 * FreeRTOS periodic timer function that sends speed to diagnostic queue to be
 * sent to esp. Speed is converted from m/s to km/h
 */
//////////////////////////////////////////////////////////////////////////////
static void SpeedEstimation_SendSpeedToDiagnostic(TimerHandle_t xTimer)
{
    Message_t msgToSend;

    msgToSend.Id        = ID_VEHICLE_SPEED;
    // conversion from m/s to km/h
    msgToSend.Data.F  = (gSpeed * 3.6f);

    xQueueSendToBack(q_DiagnosticData, &msgToSend, 0);
}

//////////////////////////////////////////////////////////////////////////////
/*
 *  AppMainCM7.c
 *  
 *  Main application body
 * 
 *  Created on: Jul 13, 2024
 *      Author: Žan Hertiš
 */
 ////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Includes 
//////////////////////////////////////////////////////////////////////////////
#include "AppMainCM7.h"
#include "ClockHandling.h"
#include "Serializer.h"
#include "SpeedEstimation.h"
#include "EspComms.h"
#include "LongitudinalControl.h"
#include "ProjectConfig.h"

//////////////////////////////////////////////////////////////////////////////
// Defines 
//////////////////////////////////////////////////////////////////////////////

#define NO_DELAY                    0
#define SEND_STATUS_FLAG_PERIOD     pdMS_TO_TICKS(500)
#define MAIN_TASK_PERIOD_MS         50

//////////////////////////////////////////////////////////////////////////////
// Global Variables 
//////////////////////////////////////////////////////////////////////////////


TimerHandle_t       t_statusTimer;

QueueHandle_t       q_UserCommand           = NULL;
QueueHandle_t       q_DiagnosticData        = NULL;
QueueHandle_t       q_LongitudinalTaskData  = NULL;
QueueHandle_t       q_speed                 = NULL;

EventGroupHandle_t  e_commandFlags          = NULL;
EventGroupHandle_t  e_statusFlags           = NULL;



//////////////////////////////////////////////////////////////////////////////
// Function prototypes 
//////////////////////////////////////////////////////////////////////////////

static  void Steering_PWMInit               (const uint32_t internalTimerClock, const uint32_t sysclk, uint32_t* const CCRmin, uint32_t* const CCRmax);
        void vApplicationStackOverflowHook  (TaskHandle_t xTask, char *pcTaskName);
static  void SendStatusUpdateCallback       (TimerHandle_t xTimer);

/**
 * We want main task to be executing at a fixed period of 50 ms. This simplyfies things since the 
 * task will be working with multiple queues and sincronising the task on this multiple queues would
 * be challenging. 
 * At the same time we need to make sure not to overload the esp communication task. 
 * The execution threads (steering and motor control) should report there status 
 * via event flags. Since one event group can hold 24 flags in our example, this should be enough
 * for all tasks to notify the main task about there status with the same event group.
 * Main task will set the command flags (again, same for all tasks), execution tasks will poll the
 * parts of the event group intended for that task and update their status flag. Main task will than
 * be able to see if the staus flags match with the requested command and be able to determine any errors.
 * The command flags will include if particular task will have to report any diagnostic data. If yes, than
 * all the tasks will put the data in the same queue (Message structure with ID) and the main task will be
 * sending those messages one by one to the esp comms task. The queu will be of a fixed size and if a task will
 * not be able to write to the queue do to it being full, it will set the overload flag, that will notify
 * the user that data is being lost. 
 * 
 */

//////////////////////////////////////////////////////////////////////////////
// FreeRTOS Task
//////////////////////////////////////////////////////////////////////////////

void Main_Task(void* pvParameters)
{
    EventBits_t commandFlags = 0;
    BaseType_t  result;
    Message_t   messageForEsp;
    TickType_t  lastWakeTime;

    //const TickType_t    taskPeriod  = pdMS_TO_TICKS(MAIN_TASK_PERIOD_MS);
    const TickType_t    taskPeriod  = pdMS_TO_TICKS(1000);

    q_UserCommand           = xQueueCreate(1, sizeof(uint32_t));
    q_DiagnosticData        = xQueueCreate(5, sizeof(Message_t));
    q_LongitudinalTaskData  = xQueueCreate(5, sizeof(Message_t));
    q_speed                 = xQueueCreate(1, sizeof(float));

    t_statusTimer = xTimerCreate("Status timer", SEND_STATUS_FLAG_PERIOD, pdTRUE, NULL, SendStatusUpdateCallback);

    if( t_statusTimer != NULL )
    {
        xTimerStart(t_statusTimer, portMAX_DELAY);
    }

    e_commandFlags  = xEventGroupCreate();
    e_statusFlags   = xEventGroupCreate();

    if( (e_commandFlags == NULL) || (e_statusFlags == NULL) )
    {
        Error_Handler();
    }

    // create other tasks
    if( (xTaskCreate(SpeedEstimation_Task, "Speed task", 128, NULL, 2, NULL)) != pdPASS )
    {
        Error_Handler();
    }

    if( (xTaskCreate(EspComms_ReceiverTask, "Receiver task", 128, NULL, 4, NULL)) != pdPASS )
    {
        Error_Handler();
    }

    if( (xTaskCreate(EspComms_TransmitterTask, "Transmitter task", 128, NULL, 1, NULL)) != pdPASS )
    {
        Error_Handler();
    }

    // if( (xTaskCreate(LongitudinalControl_Task, "Speed control task", 128, NULL, 1, NULL)) != pdPASS )
    // {
    //     Error_Handler();
    // }

    lastWakeTime = xTaskGetTickCount();

    uint32_t speed;

    while(1)
    {
        result = xQueueReceive(q_UserCommand, &commandFlags, NO_DELAY);

        if( result == pdPASS )
        {
            commandFlags = xEventGroupSetBits(e_commandFlags, commandFlags);
        }

        xQueuePeek(q_speed, &speed, 0);
        BaseType_t result = xQueueSendToBack(q_DiagnosticData, &speed, 0);

        // Sets the delay time which is equal to taskPeriod - task execution time
        vTaskDelayUntil(&lastWakeTime, taskPeriod);
    }
}

//////////////////////////////////////////////////////////////////////////////
// Function Definitions 
//////////////////////////////////////////////////////////////////////////////

void AppCM7_Main()
{
    const uint32_t timer2Clk = ClockHandling_GetTimerClkFreq(&htim2);
    const uint32_t sysClk   = HAL_RCC_GetSysClockFreq();
    uint32_t CCRmin = 0; 
    uint32_t CCRmax = 0;

    //Steering_PWMInit(timer2Clk, sysClk, &CCRmin, &CCRmax);
    // TIM2->CCR1 = CCRmax;
    // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);


    if( (xTaskCreate(Main_Task, "Main task", 512, NULL, 4, NULL)) != pdPASS )
    {
        Error_Handler();
    }

    vTaskStartScheduler();

    while(1)
    {

    }
}

static void SendStatusUpdateCallback(TimerHandle_t xTimer)
{
    Message_t   messageToSend;
    BaseType_t  status;

    messageToSend.Data.U32  = xEventGroupGetBits(e_statusFlags);
    messageToSend.Id        = ID_STATUS_FLAG;

    status = xQueueSendToBack(q_DiagnosticData, &messageToSend, NO_DELAY);

    if( status == errQUEUE_FULL )
    {
        xEventGroupSetBits(e_statusFlags, SF_ESP_TRANSMISSION_OVERLOD);
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

    if( sysclk < 100000000 ) // 100MHz
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
    if( HAL_TIM_Base_Init(&htim2) != HAL_OK )
    {
      Error_Handler();
    }
}


void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    while(1)
    {

    }
}


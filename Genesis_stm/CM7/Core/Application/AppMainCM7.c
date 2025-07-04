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
#include <stdio.h>

#include "AppMainCM7.h"
#include "ClockHandling.h"
#include "Serializer.h"
#include "SpeedEstimation.h"
#include "EspComms.h"
#include "LongitudinalControl.h"
#include "LateralControl.h"
#include "JetsonComms.h"
#include "ProjectConfig.h"

//////////////////////////////////////////////////////////////////////////////
// Defines 
//////////////////////////////////////////////////////////////////////////////

#define NO_DELAY                    0
#define SEND_STATUS_FLAG_PERIOD     pdMS_TO_TICKS(1000)
#define MAIN_TASK_PERIOD_MS         50

//////////////////////////////////////////////////////////////////////////////
// Global Variables 
//////////////////////////////////////////////////////////////////////////////


TimerHandle_t       t_statusTimer;

QueueHandle_t       q_UserCommand           = NULL;
QueueHandle_t       q_DiagnosticData        = NULL;
QueueHandle_t       q_LongitudinalTaskData  = NULL;
QueueHandle_t       q_Curvature             = NULL;
QueueHandle_t       q_LateralDeviation      = NULL;
QueueHandle_t       q_RelativeYawAngle      = NULL;
QueueHandle_t       q_speed                 = NULL;
QueueHandle_t       q_ManualSteerAngle      = NULL;

EventGroupHandle_t  e_commandFlags          = NULL;
EventGroupHandle_t  e_statusFlags           = NULL;



//////////////////////////////////////////////////////////////////////////////
// Function prototypes 
//////////////////////////////////////////////////////////////////////////////

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
 * parts of the event group intended for that task.
 * The command flags will include if particular task will have to report any diagnostic data. If yes, than
 * all the tasks will put the data in the same queue (Message structure with ID) that will be directly read
 * and sent out by the esp transmitter task.
 * 
 * IMPORTANT:
 * ! CPU2 is currently not used - Disabled in option bytes
 * 
 */

//////////////////////////////////////////////////////////////////////////////
// FreeRTOS Task
//////////////////////////////////////////////////////////////////////////////

void Main_Task(void* pvParameters)
{
    EventBits_t commandFlags = 0;
    BaseType_t  result;
    TickType_t  lastWakeTime;

    const TickType_t    taskPeriod  = pdMS_TO_TICKS(MAIN_TASK_PERIOD_MS);

    q_UserCommand           = xQueueCreate(1, sizeof(uint32_t));
    q_DiagnosticData        = xQueueCreate(8, sizeof(Message_t));
    q_LongitudinalTaskData  = xQueueCreate(5, sizeof(Message_t));
    q_Curvature             = xQueueCreate(1, sizeof(float));
    q_LateralDeviation      = xQueueCreate(1, sizeof(float));
    q_RelativeYawAngle      = xQueueCreate(1, sizeof(float));
    q_speed                 = xQueueCreate(1, sizeof(float));
    q_ManualSteerAngle      = xQueueCreate(1, sizeof(uint32_t));

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

    if( (xTaskCreate(EspComms_ReceiverTask, "Receiver task", 128, NULL, 3, NULL)) != pdPASS )
    {
        Error_Handler();
    }

    if( (xTaskCreate(EspComms_TransmitterTask, "Transmitter task", 128, NULL, 1, NULL)) != pdPASS )
    {
        Error_Handler();
    }

    if( (xTaskCreate(LongitudinalControl_Task, "Speed control task", 128, NULL, 1, NULL)) != pdPASS )
    {
        Error_Handler();
    }

    if( (xTaskCreate(LateralControl_Task, "Steer control task", 4096, NULL, 4, NULL)) != pdPASS )
    {
        Error_Handler();
    }

    if( (xTaskCreate(JetsonComms_Task, "Jetson task", 128, NULL, 1, NULL)) != pdPASS )
    {
        Error_Handler();
    }

    lastWakeTime = xTaskGetTickCount();

    while(1)
    {
        result = xQueueReceive(q_UserCommand, &commandFlags, NO_DELAY);

        if( result == pdPASS )
        {
            // xEventGroupSetBits will only set bits but will not clear any. To clear
            // bits we need to invert the command flag and call clearbits funciton.
            (void)xEventGroupSetBits(e_commandFlags, commandFlags);
            // The 8 MSB bits are used by the kernel and must not be cleared
            commandFlags = (~commandFlags) & 0x00FFFFFF;
            (void)xEventGroupClearBits(e_commandFlags, commandFlags);
            
        }

        // Sets the delay time which is equal to taskPeriod - task execution time
        vTaskDelayUntil(&lastWakeTime, taskPeriod);
    }
}

//////////////////////////////////////////////////////////////////////////////
// Function Definitions 
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
/**
 * Application starting point. Function is called from main fucntion generated
 * by cubeMX in Src/main.c. 
 * Function creates main task and starts the scheduler.
 * 
 * @return  void
 * 
 */
//////////////////////////////////////////////////////////////////////////////
void AppCM7_Main()
{
    if( (xTaskCreate(Main_Task, "Main task", 512, NULL, 3, NULL)) != pdPASS )
    {
        Error_Handler();
    }

    vTaskStartScheduler();

    while(1)
    {

    }
}

//////////////////////////////////////////////////////////////////////////////
/**
 * This is a freeRTOS software timer callback function that periodically sends
 * status update to the esp device.
 * 
 * @param[in]   xTimer  Timer handle
 * 
 * @return      void
 * 
 */
//////////////////////////////////////////////////////////////////////////////
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



void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    printf("Stack overflow in task: %s\n", pcTaskName);
    while(1)
    {

    }
}

void vApplicationMallocFailedHook( void )
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    while(1)
    {

    }
}
//////////////////////////////////////////////////////////////////////////////
/**
 * This is an UART Receive complete callback function that is common for all
 * UART peripherial. It is called by the UART ISR. As such it is used both by 
 * EspComms and JetsonComms modules. Function checks which UART triggered the 
 * callback and calls respective function that processes the data.
 * 
 * @param[in]   huart   pointer to UART handle
 * 
 * @return      void
 * 
 */
//////////////////////////////////////////////////////////////////////////////
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if( huart == &huart1 )
    {
        JetsonComms_UART1RxCallback();
    }
    else if( huart == &huart2 )
    {
        EspComms_UART2RxCallback();
    }
}


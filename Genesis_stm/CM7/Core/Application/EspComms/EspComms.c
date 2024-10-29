//////////////////////////////////////////////////////////////////////////////
/*
 *  EspComms.h
 *  
 *  Module dedictated to communictaion with the esp device.
 * 
 *  Created on: Oct 27, 2024
 *      Author: Žan Hertiš
 */
 ////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Includes 
//////////////////////////////////////////////////////////////////////////////

#include "EspComms.h"

//////////////////////////////////////////////////////////////////////////////
// Defines 
//////////////////////////////////////////////////////////////////////////////

#define EVENT_TX_COMPLETE               (1 << 0)
#define EVENT_RX_COMPLETE               (1 << 1)

#define ESP_COMMS_CONNECTION_TIMEMOUT   pdMS_TO_TICKS(1000)
#define U32_MAX_VALUE                   4294967295

typedef struct EspComms
{
    bool    waitForStartByte;
    uint8_t rxBuffer[ESP_PACKET_SIZE - 1];
} EspComms_t;

//////////////////////////////////////////////////////////////////////////////
// Global Variables 
//////////////////////////////////////////////////////////////////////////////

EspComms_t gEspComms = {
    .waitForStartByte   = false,
    .rxBuffer           = {0}
};

DMA_BUFFER uint8_t gDmaTxBuffer[ESP_PACKET_SIZE]        = {0};
DMA_BUFFER uint8_t gDmaRxBuffer[ESP_PACKET_SIZE - 1]    = {0};

EventGroupHandle_t e_uartFlags = NULL;


//////////////////////////////////////////////////////////////////////////////
// Function prototypes 
//////////////////////////////////////////////////////////////////////////////

static void EspComms_OnTransferRequest  (void);
static void EspComms_OnMessageReceived  (TickType_t* const lastCommsCheck_ticks);
static bool EspComms_ticksToTimeout     (const TickType_t lastCheck, TickType_t* const delayUnitlTimeout);


//////////////////////////////////////////////////////////////////////////////
// FreeRTOS Task
//////////////////////////////////////////////////////////////////////////////

void EspComms_Task(void* pvParameters)
{
    TickType_t      delayUntilTimeout_ticks = ESP_COMMS_CONNECTION_TIMEMOUT;
    TickType_t      lastCommsCheck_ticks    = xTaskGetTickCount();
    EventBits_t     eventBits;
    bool            isTimedOut              = false;


    e_uartFlags = xEventGroupCreate();

    if( e_uartFlags == NULL )
    {
        Error_Handler();
    }

    gEspComms.waitForStartByte = true;
    HAL_UART_Receive_IT(&huart2, gDmaRxBuffer, 1);

    while(1)
    {
        eventBits = xEventGroupWaitBits(e_uartFlags, EVENT_TX_REQUEST | EVENT_RX_COMPLETE, pdFALSE, pdFALSE, delayUntilTimeout_ticks);

        if( ( eventBits & EVENT_TX_REQUEST ) != 0 )    // only EVENT_TX_REQUEST is set
        {
            if(( eventBits & EVENT_TX_COMPLETE ) != 0 )     // transmision of previous data has been completed
            {
                EspComms_OnTransferRequest();
                eventBits = xEventGroupClearBits(e_uartFlags, EVENT_TX_REQUEST);
                eventBits = xEventGroupClearBits(e_uartFlags, EVENT_TX_COMPLETE);
            }
        }

        if( ( eventBits & EVENT_RX_COMPLETE ) != 0 )   // only EVENT_RX_COMPLETE is set
        {
            EspComms_OnMessageReceived(&lastCommsCheck_ticks);
            eventBits = xEventGroupClearBits(e_uartFlags, EVENT_RX_COMPLETE);

        }

        isTimedOut = EspComms_ticksToTimeout(lastCommsCheck_ticks, &delayUntilTimeout_ticks);

        if( isTimedOut )
        {
            //TODO: Send some message to main task
        }
    }
}

/**
 * We would need event groups to be able to unblock a single esp task from either rx or tx callback function.
 * But with event groups there is a question of defering procesing to the deamon task
 * 
 * Also we need to make sure that periodic comms checks are being performed on a required interval and are successful.
 * Idealy we don't want a vTaskDelay function but rather to just block on waiting for event with a timeout 
 * that equals the remianing time for when comms check must arrive (with tolerance).
 */

//////////////////////////////////////////////////////////////////////////////
// Function Definitions 
//////////////////////////////////////////////////////////////////////////////

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BaseType_t result;

    if( huart != &huart2 )
    {
        return;
    }

    if( gEspComms.waitForStartByte )
    {
        if( gDmaRxBuffer[0] == 'Z' )
        {
            gEspComms.waitForStartByte = false;
            HAL_UART_Receive_DMA(&huart2, gDmaRxBuffer, sizeof(gDmaRxBuffer));
        }

        HAL_UART_Receive_IT(&huart2, gDmaRxBuffer, 1);
    }
    else
    {
        memcpy(gEspComms.rxBuffer, gDmaRxBuffer, sizeof(gDmaRxBuffer));
        gEspComms.waitForStartByte = true;
        HAL_UART_Receive_IT(&huart2, gDmaRxBuffer, 1);

        result = xEventGroupSetBitsFromISR(e_uartFlags, EVENT_RX_COMPLETE, &xHigherPriorityTaskWoken);

        if( result != pdFAIL )
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BaseType_t result;

    if( huart == &huart2 )
    {
        result = xEventGroupSetBitsFromISR(e_uartFlags, EVENT_TX_COMPLETE, &xHigherPriorityTaskWoken);

        if( result != pdFAIL )
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

static void EspComms_OnTransferRequest(void)
{
    Message_t           messageToSend;
    BaseType_t          result;
    HAL_StatusTypeDef   status = HAL_OK;

    result = xQueueReceive(q_messageForEsp, &messageToSend, NULL);

    if( result == pdPASS )
    {
        if( Serializer_SerializeForESP(messageToSend.Id, messageToSend.Data.U32, gDmaTxBuffer) )
        {
            status = HAL_UART_Transmit_DMA(&huart2, gDmaTxBuffer, sizeof(gDmaTxBuffer));

            if( status != HAL_OK)
            {
                // Uart Error //TODO:
            }
        }
        else
        {
            // error, data can't be serialized //TODO:
        }

    }
    else
    {
        // queue empty (shouldn't be) //TODO:
    }

}

static void EspComms_OnMessageReceived(TickType_t* const lastCommsCheck_ticks)
{
    Message_t   receivedMessage;

    if( Seriazlizer_Deserialize(gEspComms.rxBuffer, &(receivedMessage.Id), &(receivedMessage.Data.U32)) )
    {
        if( receivedMessage.Id == 4 )
        {
            *lastCommsCheck_ticks = xTaskGetTickCount();
        }

        //TODO: switch all other possible IDs and send them to appropriate queues 


    }
}

static bool EspComms_ticksToTimeout(const TickType_t lastCheck, TickType_t* const delayUnitlTimeout)
{
    const   TickType_t  currentTick             = xTaskGetTickCount();
            TickType_t  ticksElapsed            = 0;
            bool        isConnectionTimemedOut  = false;

    // check for overflow
    if( currentTick < lastCheck )
    {
        ticksElapsed = U32_MAX_VALUE - lastCheck + currentTick;
    }
    else
    {
        ticksElapsed = currentTick - lastCheck;
    }

    if( ticksElapsed >= ESP_COMMS_CONNECTION_TIMEMOUT )
    {
        isConnectionTimemedOut  = true;
        *delayUnitlTimeout      = 0;
    }
    else
    {
        *delayUnitlTimeout = ESP_COMMS_CONNECTION_TIMEMOUT - ticksElapsed;
    }

    return isConnectionTimemedOut;
}
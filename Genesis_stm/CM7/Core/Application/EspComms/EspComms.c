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
#define WAIT_FOR_QUEUE_MS               pdMS_TO_TICKS(10)

typedef struct EspComms
{
    bool    waitForStartByte;
    uint8_t rxBuffer[SERIALIZER_PACKET_SIZE - 1];
} EspComms_t;

//////////////////////////////////////////////////////////////////////////////
// Global Variables 
//////////////////////////////////////////////////////////////////////////////

EspComms_t gEspComms = {
    .waitForStartByte   = false,
    .rxBuffer           = {0}
};

DMA_BUFFER uint8_t gDmaTxBuffer[SERIALIZER_PACKET_SIZE]        = {0};
DMA_BUFFER uint8_t gDmaRxBuffer[SERIALIZER_PACKET_SIZE - 1]    = {0};

EventGroupHandle_t e_uartFlags = NULL;


//////////////////////////////////////////////////////////////////////////////
// Function prototypes 
//////////////////////////////////////////////////////////////////////////////

static bool EspComms_OnTransferRequest  (Message_t* messageToSendd);
static void EspComms_OnMessageReceived  (TickType_t* const lastCommsCheck_ticks);
static bool EspComms_ticksToTimeout     (const TickType_t lastCheck, TickType_t* const delayUnitlTimeout);


//////////////////////////////////////////////////////////////////////////////
// FreeRTOS Task
//////////////////////////////////////////////////////////////////////////////

void EspComms_ReceiverTask(void* pvParameters)
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
        eventBits = xEventGroupWaitBits(e_uartFlags, EVENT_RX_COMPLETE, pdFALSE, pdFALSE, delayUntilTimeout_ticks);

        if( ( eventBits & EVENT_RX_COMPLETE ) != 0 )   // EVENT_RX_COMPLETE is set
        {
            EspComms_OnMessageReceived(&lastCommsCheck_ticks);
            eventBits = xEventGroupClearBits(e_uartFlags, EVENT_RX_COMPLETE);
        }

        isTimedOut = EspComms_ticksToTimeout(lastCommsCheck_ticks, &delayUntilTimeout_ticks);

        /*
            In case of a timeout we set the timeout flag and we delay the task indefenetly,
            or until some packet is received.
            NOTE: Known limitation: if the application is left running in the timeout state, than
            once the tick timer makes a full circle a false acquisition can be astablished for the 
            duration of ESP_COMMS_CONNECTION_TIMEMOUT. This is due to the timer resolution and
            the way the timeout is being calculated.  
        
        */        
        if( isTimedOut )
        {
            xEventGroupSetBits(e_statusFlags, SF_ESP_COMMUNICTAION_TIMEOUT);
            delayUntilTimeout_ticks = portMAX_DELAY;
        }
        else
        {
            // clear the comms timeout flag
            xEventGroupClearBits(e_statusFlags, SF_ESP_COMMUNICTAION_TIMEOUT);
        }
    }
}

void EspComms_TransmitterTask(void* pveParameters)
{
    BaseType_t  result;
    Message_t   message;
    EventBits_t eventFlags;
    bool        messageSent = false;

    xEventGroupSetBits(e_uartFlags, EVENT_TX_COMPLETE);

    while(1)
    {
        result = xQueueReceive(q_DiagnosticData, &message, portMAX_DELAY);

        if( result == pdPASS )
        {
            eventFlags  = xEventGroupWaitBits(e_uartFlags, EVENT_TX_COMPLETE, pdTRUE, pdFALSE, portMAX_DELAY);
            messageSent = EspComms_OnTransferRequest(&message);

            // if message was sent of to uart we add some delay in order not to overload esp since it is generaly a much slower device
            if( messageSent )
            {
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////
// Function Definitions 
//////////////////////////////////////////////////////////////////////////////

static bool EspComms_OnTransferRequest(Message_t* messageToSend)
{
    BaseType_t          result;
    HAL_StatusTypeDef   status      = HAL_OK;
    bool                messageSent  = false;

    if( Serializer_SerializeForESP(messageToSend->Id, messageToSend->Data.U32, gDmaTxBuffer) )
    {
        status = HAL_UART_Transmit_DMA(&huart2, gDmaTxBuffer, sizeof(gDmaTxBuffer));

        if( status != HAL_OK )
        {
            // Uart Error //TODO:
        }
        else
        {
            messageSent = true;
        }
    }
    else
    {
        // error, data can't be serialized //TODO:
    }

    return messageSent;
}

static void EspComms_OnMessageReceived(TickType_t* const lastCommsCheck_ticks)
{
    Message_t   receivedMessage;

    if( Seriazlizer_Deserialize(gEspComms.rxBuffer, &(receivedMessage.Id), &(receivedMessage.Data.U32)) )
    {
        switch (receivedMessage.Id)
        {
        case ID_COMMAND_FLAG:
            xQueueOverwrite(q_UserCommand, &(receivedMessage.Data.U32));
            break;
        case ID_LONGITUDINAL_AUTOMODE_DIRECTION_SELECTION:
        case ID_LONGITUDINAL_SET_ACCELERATION:
        case ID_LONGITUDINAL_MANUAL_CONTROL:
            if( xQueueSendToBack(q_LongitudinalTaskData, &receivedMessage, WAIT_FOR_QUEUE_MS) != pdTRUE )
            {
                //TODO: Longitudinal task overload
            }
            break;
        
        default:
            //TODO: set unkonw ID Flag
            return; // unrecognised ID -> exit the function
            break;
        }

        // any valid message received counts as a communications check
        *lastCommsCheck_ticks = xTaskGetTickCount();
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);

        //TODO: switch all other possible IDs and send them to appropriate queues 

    }
    else
    {
        // message not valid
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
        ticksElapsed = UINT32_MAX - lastCheck + currentTick;
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
        else
        {
            HAL_UART_Receive_IT(&huart2, gDmaRxBuffer, 1);
        }
        
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
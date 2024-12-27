//////////////////////////////////////////////////////////////////////////////
/*
 *  JetsonComms.c
 *  
 *  Created on: Dec 21, 2024
 *      Author: Žan Hertiš
 */
 ////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
// Includes 
//////////////////////////////////////////////////////////////////////////////

#include "JetsonComms.h"
#include "Serializer.h"
#include <string.h>

//////////////////////////////////////////////////////////////////////////////
// Defines 
//////////////////////////////////////////////////////////////////////////////

#define EVENT_RX_COMPLETE               (1 << 1)

// Tick period after which if no message is received form jetson, the not active flag is set
#define JETSON_COMMS_NOT_ACITVE_TICKS   pdMS_TO_TICKS(500)

typedef struct JetsonComms
{
    bool    waitForStartByte;
    uint8_t rxBuffer[SERIALIZER_PACKET_SIZE - 1];
} JetsonComms_t;

//////////////////////////////////////////////////////////////////////////////
// Global Variables 
//////////////////////////////////////////////////////////////////////////////

JetsonComms_t gJetsonComms = {
    .waitForStartByte   = false,
    .rxBuffer           = {0}
};

DMA_BUFFER uint8_t gJetsonDMABuffer[SERIALIZER_PACKET_SIZE - 1] = {0};

EventGroupHandle_t e_uart1Flags = NULL;

//////////////////////////////////////////////////////////////////////////////
// Function prototypes 
//////////////////////////////////////////////////////////////////////////////

static void JetsonComms_OnMessageReceived(void);

//////////////////////////////////////////////////////////////////////////////
// FreeRTOS Task
//////////////////////////////////////////////////////////////////////////////
void JetsonComms_Task(void *pvParameters)
{
    EventBits_t eventBits;
    bool        timeout     = false;    

    e_uart1Flags = xEventGroupCreate();

    if( e_uart1Flags == NULL )
    {
        Error_Handler();
    }

    gJetsonComms.waitForStartByte = true;
    memset(&gJetsonDMABuffer[0], 0, sizeof(gJetsonDMABuffer));
    HAL_UART_Receive_IT(&huart1, gJetsonDMABuffer, 1);
    
    while(1)
    {
        eventBits = xEventGroupWaitBits(e_uart1Flags, EVENT_RX_COMPLETE, pdFALSE, pdFALSE, JETSON_COMMS_NOT_ACITVE_TICKS);

        if( (eventBits & EVENT_RX_COMPLETE) != 0 )  // EVENT_RX_COMPLETE is set
        {
            JetsonComms_OnMessageReceived();
            eventBits = xEventGroupClearBits(e_uart1Flags, EVENT_RX_COMPLETE);
            if( timeout )
            {
                xEventGroupClearBits(e_statusFlags, SF_JETSON_COMMUNICTAION_TIMEOUT);
                timeout = false;
            }
        }
        else
        {
            xEventGroupSetBits(e_statusFlags, SF_JETSON_COMMUNICTAION_TIMEOUT);
            timeout = true;
        }


    }
}
//////////////////////////////////////////////////////////////////////////////
// Function Definitions 
//////////////////////////////////////////////////////////////////////////////

static void JetsonComms_OnMessageReceived(void)
{
    Message_t rxMessage;

    if( Seriazlizer_Deserialize(gJetsonComms.rxBuffer, &(rxMessage.Id), &(rxMessage.Data.U32)) )
    {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1);
        switch( rxMessage.Id )
        {
            case ID_CURVATURE:
                xQueueOverwrite(q_Curvature, &rxMessage.Data.F);
                break;
            case ID_LATERAL_DEVIATION:
                xQueueOverwrite(q_LateralDeviation, &rxMessage.Data.F);
                break;
            default:
                // Unrecognised ID
                break;
        }
    }
    else
    {
        // invalid message
    }

}


void JetsonComms_UART1RxCallback(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BaseType_t result;

    if( gJetsonComms.waitForStartByte )
    {
        if( gJetsonDMABuffer[0] == 'Z' )
        {
            gJetsonComms.waitForStartByte = false;
            HAL_UART_Receive_DMA(&huart1, gJetsonDMABuffer, sizeof(gJetsonDMABuffer));
        }
        else
        {
            HAL_UART_Receive_IT(&huart1, gJetsonDMABuffer, 1);
        }
        
    }
    else
    {
        memcpy(gJetsonComms.rxBuffer, gJetsonDMABuffer, sizeof(gJetsonDMABuffer));
        gJetsonComms.waitForStartByte = true;
        HAL_UART_Receive_IT(&huart1, gJetsonDMABuffer, 1);

        result = xEventGroupSetBitsFromISR(e_uart1Flags, EVENT_RX_COMPLETE, &xHigherPriorityTaskWoken);

        if( result != pdFAIL )
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}


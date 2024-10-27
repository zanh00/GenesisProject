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

typedef struct EspComms
{
    bool    waitForStartByte;
    uint8_t rxBuffer[ESP_PACKET_SIZE - 1];
    bool    rxFull;
    uint8_t txBuffer[ESP_PACKET_SIZE];
    bool    txFull;
} EspComms_t;

//////////////////////////////////////////////////////////////////////////////
// Global Variables 
//////////////////////////////////////////////////////////////////////////////

EspComms_t gEspComms = {
    .waitForStartByte   = false,
    .rxBuffer           = {0},
    .rxFull             = {0},
    .txBuffer           = {0},
    .txFull             = {0}
};

DMA_BUFFER uint8_t gDmaTxBuffer[ESP_PACKET_SIZE]        = {0};
DMA_BUFFER uint8_t gDmaRxBuffer[ESP_PACKET_SIZE - 1]    = {0};

//////////////////////////////////////////////////////////////////////////////
// Function prototypes 
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// FreeRTOS Task
//////////////////////////////////////////////////////////////////////////////

void EspComms_Task(void* pvParameters)
{


    while(1)
    {

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
    if( huart != &huart2)
    {
        return;
    }

    if( gEspComms.waitForStartByte )
    {
        if( gEspComms.rxBuffer[0] == 'Z' )
        {
            gEspComms.waitForStartByte = false;
            HAL_UART_Receive_DMA(&huart2, gEspComms.rxBuffer, sizeof(gEspComms.rxBuffer));
        }
    }
    else
    {
        gEspComms.rxFull = true;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if( huart == &huart2 )
    {
        gEspComms.txFull = false;
    }
}
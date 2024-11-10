//////////////////////////////////////////////////////////////////////////////
/*
 *  LongitudinalControl.c
 *  
 *  Module for controlling vehicle speed with the PID control.
 * 
 *  PID controller used in here can be found on github:
 *  https://github.com/tcleg/PID_Controller.git
 * 
 *  Created on: Oct 30, 2024
 *      Author: Žan Hertiš
 */
 ////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Includes 
//////////////////////////////////////////////////////////////////////////////

#include "LongitudinalControl.h"



//////////////////////////////////////////////////////////////////////////////
// Defines 
//////////////////////////////////////////////////////////////////////////////

#define SOFTWARE_REV_REGISTER_ADDR      0x7
#define DRIVER_SOFTWARE_REV             13
#define WAIT_MODE_EVENT                 pdMS_TO_TICKS(200)

#define EVENT_MANUAL_DRIVE              (1 << 2)
#define EVENT_LANE_KEEP_MODE            (1 << 3)

typedef enum
{
    e_STOP = 0,
    e_FORWARD,
    e_BACKWARD
} Direction_e

//////////////////////////////////////////////////////////////////////////////
// Global Variables 
//////////////////////////////////////////////////////////////////////////////

            uint16_t            gDevAddress = 0xB0 << 1;
DMA_BUFFER  uint8_t             gDmaTxBuffer;
            SemaphoreHandle_t   txSemphr;

//////////////////////////////////////////////////////////////////////////////
// Function prototypes 
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// FreeRTOS Task
//////////////////////////////////////////////////////////////////////////////

/*
    In the initialization we establish connection by getting the revision number of the md03 driver.
    This can also be periodicaly checked to make sure that we have active connection with the driver.
    One of the command values is direction select (0-forward; 1-backwards; default is 0). Another 
    command value is the acceleration( 0-fastest, 255-slowest ). In the automatic (lane keeping) mode,
    the motor output is determined by the PID controller based on the requested speed. In manual mode,
    another command value must be periodicaly sent. 1 means vehicle will move slowly with the preditermined
    speed forwared, 2 in reverse (not the same data as the direction selection for automatic mode). If 0
    or nothing is send withtin the required period it means stop. Manual mode is not controled by the PID.
    Manual or automatic mode is set via the command flag(mutualy exclusive).

    We need direction protection. Meaning that if vehcile is moving forward, it must wirst stop before it
    can go backwards.

 */

void LongitudinalControl_Task(void* pvParameters)
{
    float           requestedSpeed  = 0;
    Direction_e     direction       = e_STOP;
    EventBits_t     events;

    txSemphr = xSemaphoreCreateBinary();

    if( txSemphr == NULL )
    {
        Error_Handler();
    }

    // We initiali give the semaphore so that a transmission function can take it. From that point on
    // the semaphore is given back by the interrupt routine when tx finishes.
    xSemaphoreGive(txSemphr);

    while( LongitudinalControl_CheckDriverConnection() == false )
    {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    while(1)
    {
        events = xEventGroupWaitBits(e_commandFlags, EVENT_MANUAL_DRIVE | EVENT_LANE_KEEP_MODE, pdFALSE, pdFALSE, WAIT_MODE_EVENT);

        if( (events & EVENT_MANUAL_DRIVE) != 0 )
        {
            

        }
        else if( (events & EVENT_LANE_KEEP_MODE) != 0 )
        {

        }
        else
        {
            // empty the queue
        }

    }
}

//////////////////////////////////////////////////////////////////////////////
// Function Definitions 
//////////////////////////////////////////////////////////////////////////////



void LogitudinalControl_WriteToDriver(const uint8_t memAddr, const uint8_t data)
{
    
    if( xSemaphoreTake(txSemphr, portMAX_DELAY) == pdTRUE )
    {
        gDmaTxBuffer = data;
        HAL_I2C_Mem_Write_DMA(&hi2c1, gDevAddress, (uint16_t)memAddr, 1, &gDmaTxBuffer, 1);
    }
    
}

bool LongitudinalControl_CheckDriverConnection()
{
    uint8_t softWareRev = 0;
    bool    result      = false;

    HAL_I2C_Mem_Read(&hi2c1, gDevAddress, SOFTWARE_REV_REGISTER_ADDR, 1, &softWareRev, 1, HAL_MAX_DELAY);

    if( softWareRev == DRIVER_SOFTWARE_REV )
    {
        result = true;
    }

    return result;
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if( hi2c != &hi2c1 )
    {
        return;
    }

    BaseType_t xHigerPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(txSemphr, &xHigerPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigerPriorityTaskWoken);
}
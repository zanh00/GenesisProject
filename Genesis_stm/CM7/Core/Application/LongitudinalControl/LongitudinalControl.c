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

#define WAIT_MODE_EVENT                 pdMS_TO_TICKS(200)
#define MANUAL_COMMAND_TIMEOUT_TICKS    pdMS_TO_TICKS(200)

#define SOFTWARE_REV_REGISTER_ADDR      0x7
#define DRIVER_COMMAND_REGISTER         0
#define DRIVER_SPEED_REGISTER           2
#define DRIVER_ACCELERATION_REGISTER    3

#define DRIVER_SOFTWARE_REV             13

#define STOP_VEHICLE_ACCELERATION       128
#define MANUAL_CONTROL_ACCELERATION     180
#define MANUAL_CONTROL_SPEED            30

#define EVENT_MANUAL_DRIVE              (1 << 2)
#define EVENT_LANE_KEEP_MODE            (1 << 3)

typedef enum
{
    eSTOP = 0,
    eFORWARD,
    eBACKWARD
} Direction_e;

typedef struct
{
    float       speed;
    float       requestedSpeed;
    Direction_e direction;
    Direction_e manualDriveCommand;
    uint32_t    lastManualCommand_ticks;
    uint8_t     desiredAcceleration;
} LongitduindalControl_Handle_t;

//////////////////////////////////////////////////////////////////////////////
// Global Variables 
//////////////////////////////////////////////////////////////////////////////

            uint16_t            gDevAddress = 0xB0 << 1;
DMA_BUFFER  uint8_t             gDmaTxBuffer;
            SemaphoreHandle_t   txSemphr;

LongitduindalControl_Handle_t   gLongitudinalControl = {
    .speed                      = 0,
    .requestedSpeed             = 0,
    .direction                  = eSTOP,
    .manualDriveCommand         = 0,
    .lastManualCommand_ticks    = 0,
    .desiredAcceleration        = 0
};

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

        LongitudinalControl_ReadData();

        if( (events & EVENT_MANUAL_DRIVE) != 0 )
        {
            LongitudinalControl_ManualControl();
        }
        else if( (events & EVENT_LANE_KEEP_MODE) != 0 )
        {

        }
        else
        {

        }

    }
}

//////////////////////////////////////////////////////////////////////////////
// Function Definitions 
//////////////////////////////////////////////////////////////////////////////



void LongitudinalControl_WriteToDriver(const uint8_t memAddr, const uint8_t data)
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

void LongitudinalControl_StopTheVehicle()
{
    LongitudinalControl_WriteToDriver(DRIVER_ACCELERATION_REGISTER, STOP_VEHICLE_ACCELERATION);
    LongitudinalControl_WriteToDriver(DRIVER_SPEED_REGISTER, 0);
}

void LongitudinalControl_MoveForward()
{
    LongitudinalControl_WriteToDriver(DRIVER_SPEED_REGISTER, MANUAL_CONTROL_SPEED);
    LongitudinalControl_WriteToDriver(DRIVER_COMMAND_REGISTER, 1); // 1 -> forward
}

void LongitudinalControl_MoveBackward()
{
    LongitudinalControl_WriteToDriver(DRIVER_SPEED_REGISTER, MANUAL_CONTROL_SPEED);
    LongitudinalControl_WriteToDriver(DRIVER_COMMAND_REGISTER, 2); // 1 -> revese
}

void LongitudinalControl_ReadData()
{
    Message_t receivedData;

    // read the queue as long as there is data in it
    while( xQueueReceive(q_LongitudinalTaskData, &receivedData, 0) == pdPASS )
    {
        switch(receivedData.Id)
        {
        case ID_LONGITUDINAL_SET_ACCELERATION:
            gLongitudinalControl.desiredAcceleration = receivedData.Data.U32;
            break;
        case ID_LONGITUDINAL_MANUAL_CONTROL:
            gLongitudinalControl.manualDriveCommand         = (Direction_e)receivedData.Data.U32;
            gLongitudinalControl.lastManualCommand_ticks    = xTaskGetTickCount();
            break;
        case ID_LONGITUDINAL_REQUESTED_SPEED:
            gLongitudinalControl.requestedSpeed = receivedData.Data.U32;
            break;
        default:
            // Error
            break;
        }

    }

    xQueuePeek(q_speed, &gLongitudinalControl.speed, 0);
}

void LongitudinalControl_ManualControl()
{
    const uint32_t currentTicks = xTaskGetTickCount();

    if( (currentTicks - gLongitudinalControl.lastManualCommand_ticks) > MANUAL_COMMAND_TIMEOUT_TICKS )
    {
        gLongitudinalControl.manualDriveCommand = eSTOP;
    }

    switch(gLongitudinalControl.direction)
    {
    case eSTOP:
        gLongitudinalControl.direction = LongitudinalControl_StateStopped();
        break;
    case eFORWARD:
        gLongitudinalControl.direction = LongitudinalControl_StateForward();
        break;
    case eBACKWARD:
        gLongitudinalControl.direction = LongitudinalControl_StateBackwards();
        break;
    default:
        // Error
        break;
    }
}

Direction_e LongitudinalControl_StateStopped()
{
    Direction_e retState = eSTOP;

    if( gLongitudinalControl.manualDriveCommand == eFORWARD )
    {
        LongitudinalControl_MoveForward();
        retState = eFORWARD;
    }
    else if( gLongitudinalControl.manualDriveCommand == eBACKWARD )
    {
        LongitudinalControl_MoveBackward();
        retState = eBACKWARD;
    }
    else
    {
        LongitudinalControl_StopTheVehicle();
    }

    return retState;
}

Direction_e LongitudinalControl_StateForward()
{
    Direction_e retState = eSTOP;

    if( gLongitudinalControl.manualDriveCommand == eFORWARD )
    {
        LongitudinalControl_MoveForward();
        retState = eFORWARD;
    }
    else if( (gLongitudinalControl.manualDriveCommand == eSTOP) || (gLongitudinalControl.manualDriveCommand == eBACKWARD) )
    {
        LongitudinalControl_StopTheVehicle();

        if( gLongitudinalControl.speed == 0 )
        {
            retState = eSTOP;
        }
        else
        {
            retState = eFORWARD;
        }
    }

    return retState;
}

Direction_e LongitudinalControl_StateBackwards()
{
    Direction_e retState = eSTOP;

    if( gLongitudinalControl.manualDriveCommand == eBACKWARD )
    {
        LongitudinalControl_MoveBackward();
        retState = eBACKWARD;
    }
    else if( (gLongitudinalControl.manualDriveCommand == eSTOP) || (gLongitudinalControl.manualDriveCommand == eFORWARD) )
    {
        LongitudinalControl_StopTheVehicle();
        
        if( gLongitudinalControl.speed == 0 )
        {
            retState = eSTOP;
        }
        else
        {
            retState = eBACKWARD;
        }
    }

    return retState;
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
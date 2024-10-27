//////////////////////////////////////////////////////////////////////////////
/*
 * SpeedEstimation.h
 *
 *  Created on: Oct 25, 2024
 *      Author: Žan Hertiš
 */
 ////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Includes 
//////////////////////////////////////////////////////////////////////////////

#include "tim.h"
#include "ClockHandling.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <string.h>

//////////////////////////////////////////////////////////////////////////////
// Defines 
//////////////////////////////////////////////////////////////////////////////

#define DMA_BUFFER __attribute__((section(".sram1")))

#define SPEEDESTIMATION_SAMPLE_COUNT    4

#define TYRE_DIAMETE                0.165
#define GEAR_WHEEL_TEETH            20
#define PI                          3.14159

#define SPEEDESTIMATION_COEFICIENT  ((TYRE_DIAMETE * PI) / GEAR_WHEEL_TEETH)

#define SPEEDESTIMATION_PERIOD_MS   50

//////////////////////////////////////////////////////////////////////////////
// Global Variables 
//////////////////////////////////////////////////////////////////////////////

extern QueueHandle_t q_speed;

//////////////////////////////////////////////////////////////////////////////
// FreeRTOS Task
//////////////////////////////////////////////////////////////////////////////

void SpeedEstimation_Task(void* pvParameters);
void Idle_Task(void* pvParameters);
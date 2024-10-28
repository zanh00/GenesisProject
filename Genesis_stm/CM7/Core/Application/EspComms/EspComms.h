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

#ifndef ESP_COMMS_H
#define ESP_COMMS_H


//////////////////////////////////////////////////////////////////////////////
// Includes 
//////////////////////////////////////////////////////////////////////////////

#include "Serializer.h"
#include "dma.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

#include <string.h>

//////////////////////////////////////////////////////////////////////////////
// Defines 
//////////////////////////////////////////////////////////////////////////////

#define DMA_BUFFER __attribute__((section(".sram1")))

#define EVENT_TX_REQUEST        (1 << 2)

typedef union {
    uint32_t    U32;
    float       F;
} DataUnion_t;

typedef struct Message
{
    uint8_t     Id;
    DataUnion_t Data;
} Message_t;



//////////////////////////////////////////////////////////////////////////////
// Global Variables 
//////////////////////////////////////////////////////////////////////////////

extern QueueHandle_t q_massageForEsp;

//////////////////////////////////////////////////////////////////////////////
// FreeRTOS Task
//////////////////////////////////////////////////////////////////////////////


#endif /*  ESP_COMMS_H  */

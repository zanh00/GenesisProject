//////////////////////////////////////////////////////////////////////////////
/*
 *  LongitudinalControl.h
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

#ifndef LONGITUDINAL_CONTROL
#define LONGITUDINAL_CONTROL


//////////////////////////////////////////////////////////////////////////////
// Includes 
//////////////////////////////////////////////////////////////////////////////

#include "AppMainCM7.h"
#include "ProjectConfig.h"
#include "i2c.h"

//////////////////////////////////////////////////////////////////////////////
// Defines 
//////////////////////////////////////////////////////////////////////////////

#define DMA_BUFFER __attribute__((section(".sram1")))

//////////////////////////////////////////////////////////////////////////////
// Global Variables 
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// FreeRTOS Task
//////////////////////////////////////////////////////////////////////////////


#endif /* LONGITUDINAL_CONTROL */
//////////////////////////////////////////////////////////////////////////////
/*
 * ProjectConfig.h
 *
 * Project confiugration defines
 * 
 *  Created on: Oct 30, 2024
 *      Author: Žan Hertiš
 */
 ////////////////////////////////////////////////////////////////////////////


#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

//////////////////////////////////////////////////////////////////////////////
// Timings
//////////////////////////////////////////////////////////////////////////////

#define LONGITUDINAL_CONTROL_PERIOD_MS  50
#define LATERAL_CONTROL_PERIOD_MS       50
#define SPEEDESTIMATION_PERIOD_MS       50

//////////////////////////////////////////////////////////////////////////////
// General
//////////////////////////////////////////////////////////////////////////////

#define PI                              3.14159

//Maximum wheel steer angle in either direction [deg]
#define MAX_STEER_ANGLE_DEG             30

//////////////////////////////////////////////////////////////////////////////
// ESP <-> STM Message Library
//////////////////////////////////////////////////////////////////////////////

#define ID_COMMAND_FLAG                                 1
#define ID_REQUESTED_DIAGNOSTIC                         2
#define ID_LONGITUDINAL_REQUESTED_SPEED                 3
#define ID_PERIODIC_COMMS_CHECHK                        4
#define ID_STATUS_FLAG                                  5
#define ID_LONGITUDINAL_AUTOMODE_DIRECTION_SELECTION    6
#define ID_LONGITUDINAL_SET_ACCELERATION                7
#define ID_LONGITUDINAL_MANUAL_CONTROL                  8  

//////////////////////////////////////////////////////////////////////////////
// Status Flags
//////////////////////////////////////////////////////////////////////////////

#define SF_SPEED_ESTIMATION_TASK_ACTIVE (1 << 1)
#define SF_LATERAL_CONTROL_TASK_ACTIVE  (1 << 2)
#define SF_LONG_CONTROL_TASK_ACTIVE     (1 << 3)
#define SF_ESP_TRANSMISSION_OVERLOD     (1 << 5)
#define SF_ESP_COMMUNICTAION_TIMOUT     (1 << 6)




#endif /* PROJECT_CONFIG_H */
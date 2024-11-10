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
// ESP <-> STM Message Library
//////////////////////////////////////////////////////////////////////////////

#define ID_COMMAND_FLAG                                 1
#define ID_REQUESTED_DIAGNOSTIC                         2
#define ID_LONGITUDINAL_REQUESTED_SPEED                              3
#define ID_PERIODIC_COMMS_CHECHK                        4
#define ID_STATUS_FLAG                                  5
#define ID_LONGITUDINAL_AUTOMODE_DIRECTION_SELECTION    6
#define ID_LONGITUDINAL_SET_ACCELERATION                7
#define ID_LONGITUDINAL_MANUAL_CONTROL                  8  

//////////////////////////////////////////////////////////////////////////////
// Status Flags
//////////////////////////////////////////////////////////////////////////////

#define SF_ESP_TRANSMISSION_OVERLOD     (1 << 5)
#define SF_ESP_COMMUNICTAION_TIMOUT     (1 << 6)




#endif /* PROJECT_CONFIG_H */
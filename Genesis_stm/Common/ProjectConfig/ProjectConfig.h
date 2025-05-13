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

#define LONGITUDINAL_CONTROL_PERIOD_MS          50
#define LATERAL_CONTROL_PERIOD_MS               20
#define SPEEDESTIMATION_PERIOD_MS               50

// Period in which vehicle speed measurement will be sent out to diagnostic queue
#define DIAGNOSTIC_SPEED_PERIOD_MS              1000

// Period with which lateral contorl module diagnostic data will be sent to diagnostic queue
#define DIAGNOSTIC_LATERAL_CONTROL_PERIOD_MS    1000

//////////////////////////////////////////////////////////////////////////////
// General
//////////////////////////////////////////////////////////////////////////////

#define PI                              3.14159

//Maximum wheel steer angle in either direction [deg]
#define MAX_STEER_ANGLE_DEG             30

// Wheateher to use the motor driver over I2C or analog control
#define MOTOR_DRIVER_USE_I2C            0

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
#define ID_LATERAL_CONTROL_MANUAL_STEER_ANGLE           9  
#define ID_LATERAL_CONTROL_STEER_ANGLE                  10  
#define ID_VEHICLE_SPEED                                130

//////////////////////////////////////////////////////////////////////////////
// Jetson <-> STM Message Library
//////////////////////////////////////////////////////////////////////////////

#define ID_CURVATURE                                    128
#define ID_LATERAL_DEVIATION                            129
#define ID_RELATIVE_YAW_ANGLE                           131

//////////////////////////////////////////////////////////////////////////////
// Status Flags
//////////////////////////////////////////////////////////////////////////////

#define SF_SPEED_ESTIMATION_TASK_ACTIVE (1 << 1)
#define SF_LATERAL_CONTROL_TASK_ACTIVE  (1 << 2)
#define SF_LONG_CONTROL_TASK_ACTIVE     (1 << 3)
#define SF_ESP_TRANSMISSION_OVERLOD     (1 << 5)
#define SF_ESP_COMMUNICTAION_TIMEOUT    (1 << 6)
#define SF_JETSON_COMMUNICTAION_TIMEOUT (1 << 7)

//////////////////////////////////////////////////////////////////////////////
// Command Flags
//////////////////////////////////////////////////////////////////////////////

#define COMMAND_MANUAL_DRIVE                    (1 << 2)
#define COMMAND_LANE_KEEP_MODE                  (1 << 3)
#define COMMAND_ENABLE_CURVATURE_DIAG           (1 << 4)
#define COMMAND_ENABLE_LATERAL_DEVIATION_DIAG   (1 << 5)
#define COMMAND_ENABLE_STEER_ANGLE_DIAG         (1 << 6)
#define COMMAND_ENABLE_REL_YAW_ANGLE_DIAG       (1 << 7)



#endif /* PROJECT_CONFIG_H */
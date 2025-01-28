//////////////////////////////////////////////////////////////////////////////
/*
 *  LateralControl.c
 *  
 *  Module for controlling vehicle steering angle with MPC controller.
 * 
 *  Created on: Dec 1, 2024
 *      Author: Žan Hertiš
 */
 ////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Includes 
//////////////////////////////////////////////////////////////////////////////

#include "AppMainCM7.h"
#include "ClockHandling.h"
#include "LateralControl.h"
#include "LateralController.h"
#include "ProjectConfig.h"
#include <math.h>

//////////////////////////////////////////////////////////////////////////////
// Defines 
//////////////////////////////////////////////////////////////////////////////

#define MAX_STEER_ANGLE_RAD             (MAX_STEER_ANGLE_DEG * (PI / 180.0))

/*
    Max servo angle: 90° -> 10% and 5% duty cycle (either direction).
    Position 0° -> 7,5% duty cycle. The below two calculations represent
    max and min duty cycles coresponding to maxmimum allowed steering angle
*/
#define STEERING_ANGLE_MAX_DUTY_CYCLE   (((MAX_STEER_ANGLE_RAD / (PI/2.0)) * (10.0 - 5.0)) + 7.5)
#define STEERING_ANGLE_MIN_DUTY_CYCLE   (((-MAX_STEER_ANGLE_RAD / (PI/2.0)) * (10.0 - 5.0)) + 7.5)

typedef enum
{
    eMODE_MANUAL = 0,
    eMODE_LANE_KEEP
} LateralControlMode_e;

typedef struct LateralControlData
{
    struct
    {
        float curvature;
        float lateralDeviation;
        float velocity;
    } Input;
    struct 
    {
        float angle;
    } Output;

    float                   manualSteerAngle;
    LateralControlMode_e    mode;
    
} LateralControlData_t;

//////////////////////////////////////////////////////////////////////////////
// Global Variables 
//////////////////////////////////////////////////////////////////////////////

LateralControlData_t gData = {{0}, {0}, 0, 0};

//////////////////////////////////////////////////////////////////////////////
// Function prototypes 
//////////////////////////////////////////////////////////////////////////////

static void     LateralControl_Step             (LateralControlData_t* const data);
static void     LateralControl_ReadData         (LateralControlData_t* const data);
static float    LateralControl_DegToRad         (const int32_t deg);
static void     LateralControl_SendDiagnostic   (TimerHandle_t xTimer);
static void     LateralControl_SetSteerAngle    (const uint32_t angleCCR);
static void     Steering_PWMInit                (const uint32_t intTimClk, const uint32_t sysclk, const uint8_t Dmin, const uint8_t Dmax, uint32_t* const CCRmin, uint32_t* const CCRmax);
static uint32_t LateralControl_AngleToCCR       (const float angle, const float angleMin, const float angleMax, const uint32_t CCRmin, const uint32_t CCRmax);
static double   Constrain                       (const float value, const float min, const float max);

//////////////////////////////////////////////////////////////////////////////
// FreeRTOS Task
//////////////////////////////////////////////////////////////////////////////

void LateralControl_Task(void* pvParameter)
{
    EventBits_t             commandFlags;
    TickType_t              lastWakeTime;
    uint32_t                CCRmin          = 0; 
    uint32_t                CCRmax          = 0;
    const uint32_t          timer2Clk       = ClockHandling_GetTimerClkFreq(&htim2);
    const uint32_t          sysClk          = HAL_RCC_GetSysClockFreq();
    const TickType_t        taskPeriod      = pdMS_TO_TICKS(LATERAL_CONTROL_PERIOD_MS);
    TimerHandle_t           t_DiagPeriod;

     

    Steering_PWMInit(timer2Clk, sysClk, STEERING_ANGLE_MIN_DUTY_CYCLE, STEERING_ANGLE_MAX_DUTY_CYCLE, &CCRmin, &CCRmax);

    // We set the wheel to neutral 0° position at the start
    TIM2->CCR1 = (CCRmax + CCRmin) / 2;
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    rtU.lateral_deviation       = 0;
    rtU.relative_yaw_angle      = 0;
    rtU.curvature               = 0;
    rtU.longitudinalvelocity    = 0;

    LateralController_initialize();

    lastWakeTime = xTaskGetTickCount();

    t_DiagPeriod = xTimerCreate("Status timer", pdMS_TO_TICKS(DIAGNOSTIC_LATERAL_CONTROL_PERIOD_MS), pdTRUE, NULL, LateralControl_SendDiagnostic);

    if( t_DiagPeriod != NULL )
    {
        xTimerStart(t_DiagPeriod, portMAX_DELAY);
    }

    xEventGroupSetBits(e_statusFlags, SF_LATERAL_CONTROL_TASK_ACTIVE);

    while(1)
    {
        uint32_t steerCCR = 0;
        commandFlags = xEventGroupWaitBits(e_commandFlags, COMMAND_MANUAL_DRIVE | COMMAND_LANE_KEEP_MODE, pdFALSE, pdFALSE, 0);

        LateralControl_ReadData(&gData);

        if( (commandFlags & COMMAND_MANUAL_DRIVE) != 0 )
        {
            gData.mode = eMODE_MANUAL;
            steerCCR = LateralControl_AngleToCCR(gData.manualSteerAngle, -MAX_STEER_ANGLE_RAD, MAX_STEER_ANGLE_RAD, CCRmin, CCRmax);
            LateralControl_SetSteerAngle(steerCCR);
        }
        else if( (commandFlags & COMMAND_LANE_KEEP_MODE) != 0 )
        {
            gData.mode = eMODE_LANE_KEEP;
            LateralControl_Step(&gData);

            // The value should already be constrained in the model...but just in case
            gData.Output.angle  = Constrain(gData.Output.angle, -MAX_STEER_ANGLE_RAD, MAX_STEER_ANGLE_RAD);
            steerCCR            = LateralControl_AngleToCCR(gData.Output.angle, -MAX_STEER_ANGLE_RAD, MAX_STEER_ANGLE_RAD, CCRmin, CCRmax);
            LateralControl_SetSteerAngle(steerCCR);
        }
        
        vTaskDelayUntil(&lastWakeTime, taskPeriod);
    }
}

//////////////////////////////////////////////////////////////////////////////
// Function Definitions 
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
/**
 * Function sets the values required by the MPC model, calls the step fucntion
 * and returns steering angle result.
 * 
 * @param[in]       latDeviation    lateral deviation [m]
 * @param[in]       relYawAngle     relative Yaw angle [rad]
 * @param[in]       velocity        vehicle velocity [m/s]
 * 
 * @return          requested steering angle
 */
//////////////////////////////////////////////////////////////////////////////
static void LateralControl_Step(LateralControlData_t* const data)
{
    rtU.lateral_deviation       = data->Input.lateralDeviation;
    rtU.relative_yaw_angle      = 0;                            // TODO: Implement relative yaw angle
    rtU.curvature               = data->Input.curvature;
    rtU.longitudinalvelocity    = data->Input.velocity;

    LateralController_step();

    data->Output.angle = rtY.steering_angle;
}

//////////////////////////////////////////////////////////////////////////////
/**
 * Function sets the steering angle of the vehicle.
 * 
 * @param[in]       angleCCR    steering angle in CCR value
 * 
 * @return          void
 */
//////////////////////////////////////////////////////////////////////////////
static void LateralControl_SetSteerAngle(const uint32_t angleCCR)
{
    TIM2->CCR1 = angleCCR;
}

//////////////////////////////////////////////////////////////////////////////
/**
 * Function reads the data from the queues.
 * 
 * @param[out]      data    Lateral control data structure
 * 
 * @return          void
 */
//////////////////////////////////////////////////////////////////////////////
static void LateralControl_ReadData(LateralControlData_t* const data)
{
    uint32_t manualSteerAngleTemp = 30;

    xQueuePeek(q_Curvature, &(data->Input.curvature), 0);
    xQueuePeek(q_LateralDeviation, &(data->Input.lateralDeviation), 0);
    xQueuePeek(q_ManualSteerAngle, &manualSteerAngleTemp, 0);
    // steering angle is shifted by 30 degrees to avoid having to use int32 values in the messages
    // manualSteerAngleTemp: 0-> -30°; 60 -> 30°
    gData.manualSteerAngle = LateralControl_DegToRad(manualSteerAngleTemp - 30);

}

//////////////////////////////////////////////////////////////////////////////
/**
 * Function converts degrees to radians.
 * 
 * @param[in]       deg     angle in degrees
 * 
 * @return          angle in radians
 */
//////////////////////////////////////////////////////////////////////////////
static float LateralControl_DegToRad(const int32_t deg)
{
    return deg * (PI / 180.0);
}

//////////////////////////////////////////////////////////////////////////////
/**
 * Function converts radians to degrees. And shifts the value by 30 degrees to
 * get the steering angle in the range of 0-60° so that we don't have to use 
 * int32 values in the messages.
 * TODO: This is a temporary solution and should be made to work with int32 values
 * 
 * @param[in]       rad     angle in radians
 * 
 * @return          angle in degrees
 */
//////////////////////////////////////////////////////////////////////////////
static uint32_t LateralControl_RadToDeg(const float rad)
{
    const int32_t deg =  rad * (180.0 / PI);

    return deg + 30;
}

//////////////////////////////////////////////////////////////////////////////
/**
 * Function sends out diagnostic data specified by the command flags.
 * 
 * @param[in]       xTimer      Timer handle
 * return           void
 */
//////////////////////////////////////////////////////////////////////////////
static void LateralControl_SendDiagnostic(TimerHandle_t xTimer)
{
    EventBits_t flags = xEventGroupGetBits(e_commandFlags);
    Message_t   diagData;

    if( (flags & COMMAND_ENABLE_CURVATURE_DIAG) != 0 )
    {
        diagData.Id = ID_CURVATURE;
        diagData.Data.F = gData.Input.curvature;
        xQueueSendToBack(q_DiagnosticData, &diagData, 0);
    }

    if( (flags & COMMAND_ENABLE_LATERAL_DEVIATION_DIAG) != 0 )
    {
        diagData.Id = ID_LATERAL_DEVIATION;
        diagData.Data.F = gData.Input.lateralDeviation;
        xQueueSendToBack(q_DiagnosticData, &diagData, 0);
    }

    if( (flags & COMMAND_ENABLE_STEER_ANGLE_DIAG) != 0 )
    {
        diagData.Id = ID_LATERAL_CONTROL_STEER_ANGLE;

        if( gData.mode == eMODE_LANE_KEEP )
        {
            diagData.Data.U32 = LateralControl_RadToDeg(gData.Output.angle);
        }
        else if ( gData.mode == eMODE_MANUAL )
        {
            diagData.Data.U32 = LateralControl_RadToDeg(gData.manualSteerAngle);
        }

        xQueueSendToBack(q_DiagnosticData, &diagData, 0);
    }
}

//////////////////////////////////////////////////////////////////////////////
/**
 * Fucntion configures pwm timer and calculates max and min allowed CCRmin 
 * and CCR max.
 * 
 * @param[in]       intTimClk       Internal timer clock in Hz
 * @param[in]       sysclk          System clock in Hz
 * @param[in]       Dmin            Min allowed servo duty cycle
 * @param[in]       Dmax            Maximum allowed servo duty cycle
 * @param[out]      CCRmin          capture and compare register value coresponidng
 *                                  to minimum duty cylce
 * @param[out]      CCRmax          capture and compare register value coresponidng
 *                                  to maximum duty cylce
 * 
 * @return          void
 */
//////////////////////////////////////////////////////////////////////////////
static void Steering_PWMInit(const uint32_t intTimClk, const uint32_t sysclk, const uint8_t Dmin, const uint8_t Dmax, uint32_t* const CCRmin, uint32_t* const CCRmax)
{
    uint8_t     presc   = 0u;
    uint8_t     freq    = 50u;  // 50Hz
    uint32_t    ARR     = 0u;   // auto reload register
    uint32_t    timClk  = 0u;

    if( sysclk < 100000000 ) // 100MHz
    {
        presc = 4;
    }
    else
    {
        presc = 8;
    }

    timClk  = intTimClk / presc;
    ARR     = timClk / freq;

    *CCRmin = (Dmin * ARR) / 100;
    *CCRmax = (Dmax * ARR) / 100;

    htim2.Init.Prescaler    = presc - 1;
    htim2.Init.Period       = ARR;

    if( HAL_TIM_Base_Init(&htim2) != HAL_OK )
    {
        Error_Handler();
    }
}

//////////////////////////////////////////////////////////////////////////////
/**
 * Function calculates steering servo pwm CCR value depending on the input angle
 * 
 * @param[in]       angle       input angle [rad] 
 * @param[in]       angleMin    Max steering  angle in one direction [rad] 
 * @param[in]       angleMax    Max steering  angle in other direction [rad] 
 * @param[in]       CCRmin      min duty cycle CCR value
 * @param[in]       CCRmax      max duty cycle CCR value
 * 
 * @return          current angle CCR value.      
 */
//////////////////////////////////////////////////////////////////////////////
static uint32_t LateralControl_AngleToCCR(const float angle, const float angleMin, const float angleMax, const uint32_t CCRmin, const uint32_t CCRmax)
{
    uint32_t CCRout = CCRmin + ((angle - angleMin) / (angleMax - angleMin)) * (CCRmax - CCRmin);

    return CCRout;
}

//////////////////////////////////////////////////////////////////////////////
/**
 * Function constrains the input value to the boundaries defined by the min
 * and max limits
 * 
 * @param[in]       value       value to be constrained
 * @param[in]       min         lower value limit
 * @param[in]       max         upper value limit
 * 
 * @return      constrained value.
 */
//////////////////////////////////////////////////////////////////////////////
static double Constrain(const float value, const float min, const float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

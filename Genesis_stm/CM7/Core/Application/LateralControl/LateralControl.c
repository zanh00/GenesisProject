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
#include "LateralControl.h"
#include "MPCmodel/adaptive_mpc_curvature_model_deployable.h"
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

//////////////////////////////////////////////////////////////////////////////
// Global Variables 
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Function prototypes 
//////////////////////////////////////////////////////////////////////////////

static void     Steering_PWMInit    (const uint32_t intTimClk, const uint32_t sysclk, const uint8_t Dmin, const uint8_t Dmax, uint32_t* const CCRmin, uint32_t* const CCRmax);
static uint32_t AngleToCCR          (const double angle, const double angleMin, const double angleMax, const uint32_t CCRmin, const uint32_t CCRmax);
static double   LateralControl_Step (const double latDeviation, const double relYawAngle, const double velocity);

//////////////////////////////////////////////////////////////////////////////
// FreeRTOS Task
//////////////////////////////////////////////////////////////////////////////

void LateralControl_Task(void* pvParameter)
{
    EventBits_t     events;
    TickType_t      lastWakeTime;
    uint32_t        CCRmin          = 0; 
    uint32_t        CCRmax          = 0;
    const uint32_t  timer2Clk       = ClockHandling_GetTimerClkFreq(&htim2);
    const uint32_t  sysClk          = HAL_RCC_GetSysClockFreq();

    const TickType_t taskPeriod  = pdMS_TO_TICKS(LATERAL_CONTROL_PERIOD_MS);

    Steering_PWMInit(timer2Clk, sysClk, STEERING_ANGLE_MIN_DUTY_CYCLE, STEERING_ANGLE_MAX_DUTY_CYCLE, CCRmin, CCRmax);

    // We set the wheel to neutral 0° position at the start
    TIM2->CCR1 = (CCRmax + CCRmin) / 2;
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    rtU.lateral_deviation   = 0;
    rtU.relative_yaw_angle  = 0;
    rtU.Velocity            = 0;

    adaptive_mpc_curvature_model_deployable_initialize();

    xEventGroupSetBits(e_statusFlags, SF_LATERAL_CONTROL_TASK_ACTIVE);

    while(1)
    {

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
static double LateralControl_Step(const double latDeviation, const double relYawAngle, const double velocity)
{
    double steerAngle = 0;

    rtU.lateral_deviation   = latDeviation;
    rtU.relative_yaw_angle  = relYawAngle;
    rtU.Velocity            = velocity;

    adaptive_mpc_curvature_model_deployable_step();

    steerAngle = rtY.Steeringangle;

    return steerAngle;
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
static uint32_t AngleToCCR(const double angle, const double angleMin, const double angleMax, const uint32_t CCRmin, const uint32_t CCRmax)
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
static double Constrain(const double value, const double min, const double max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

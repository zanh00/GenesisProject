/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: adaptive_mpc_curvature_model_deployable.h
 *
 * Code generated for Simulink model 'adaptive_mpc_curvature_model_deployable'.
 *
 * Model version                  : 1.11
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Fri Dec  6 17:21:10 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef adaptive_mpc_curvature_model_deployable_h_
#define adaptive_mpc_curvature_model_deployable_h_
#ifndef adaptive_mpc_curvature_model_deployable_COMMON_INCLUDES_
#define adaptive_mpc_curvature_model_deployable_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif            /* adaptive_mpc_curvature_model_deployable_COMMON_INCLUDES_ */

#include <string.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef rtmGetTStart
#define rtmGetTStart(rtm)              ((rtm)->Timing.tStart)
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T xk1[6];                       /* '<S32>/FixedHorizonOptimizer' */
  real_T Pk1[36];                      /* '<S32>/FixedHorizonOptimizer' */
  real_T State_DSTATE[4];              /* '<S34>/State' */
  real_T LastPcov_PreviousInput[36];   /* '<S4>/LastPcov' */
  real_T last_x_PreviousInput[6];      /* '<S4>/last_x' */
  real_T Memory_PreviousInput[2];      /* '<S3>/Memory' */
  real_T Memory1_PreviousInput[4];     /* '<S3>/Memory1' */
  real_T c_Hv[1860];
  real_T Su[1800];
  real_T u_scale;                      /* '<S4>/u_scale' */
  real_T u;                            /* '<S32>/FixedHorizonOptimizer' */
  real_T last_mv_DSTATE;               /* '<S4>/last_mv' */
  real_T Memory_PreviousInput_o;       /* '<Root>/Memory' */
  boolean_T iAout[8];                  /* '<S32>/FixedHorizonOptimizer' */
  boolean_T Memory_PreviousInput_l[8]; /* '<S4>/Memory' */
} DW;

/* Continuous states (default storage) */
typedef struct {
  real_T IntegratorLimited_CSTATE;     /* '<Root>/Integrator Limited' */
} X;

/* State derivatives (default storage) */
typedef struct {
  real_T IntegratorLimited_CSTATE;     /* '<Root>/Integrator Limited' */
} XDot;

/* State disabled  */
typedef struct {
  boolean_T IntegratorLimited_CSTATE;  /* '<Root>/Integrator Limited' */
} XDis;

#ifndef ODE3_INTG
#define ODE3_INTG

/* ODE3 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[3];                        /* derivatives */
} ODE3_IntgData;

#endif

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: lastPcov
   * Referenced by: '<S4>/LastPcov'
   */
  real_T LastPcov_InitialCondition[36];
} ConstP;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T Velocity;                     /* '<Root>/Velocity' */
  real_T lateral_deviation;            /* '<Root>/Inport' */
  real_T relative_yaw_angle;           /* '<Root>/Inport1' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T Steeringangle;                /* '<Root>/Steering angle' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[1];
  real_T odeF[3][1];
  ODE3_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    time_T tStart;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Continuous states (default storage) */
extern X rtX;

/* Disabled states (default storage) */
extern XDis rtXDis;

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void adaptive_mpc_curvature_model_deployable_initialize(void);
extern void adaptive_mpc_curvature_model_deployable_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S4>/Constant' : Unused code path elimination
 * Block '<S4>/Floor' : Unused code path elimination
 * Block '<S4>/Floor1' : Unused code path elimination
 * Block '<S5>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S6>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S7>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S8>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S9>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S10>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S11>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S12>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S13>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S14>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S15>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S16>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S17>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S18>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S19>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S20>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S21>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S22>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S23>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S24>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S25>/Vector Dimension Check' : Unused code path elimination
 * Block '<S26>/Vector Dimension Check' : Unused code path elimination
 * Block '<S27>/Vector Dimension Check' : Unused code path elimination
 * Block '<S28>/Vector Dimension Check' : Unused code path elimination
 * Block '<S29>/Vector Dimension Check' : Unused code path elimination
 * Block '<S30>/Vector Dimension Check' : Unused code path elimination
 * Block '<S4>/Min' : Unused code path elimination
 * Block '<S31>/Vector Dimension Check' : Unused code path elimination
 * Block '<S4>/useq_scale' : Unused code path elimination
 * Block '<S4>/useq_scale1' : Unused code path elimination
 * Block '<S4>/ym_zero' : Unused code path elimination
 * Block '<S1>/m_zero' : Unused code path elimination
 * Block '<S1>/p_zero' : Unused code path elimination
 * Block '<S34>/RSdx' : Unused code path elimination
 * Block '<S4>/Reshape' : Reshape block reduction
 * Block '<S4>/Reshape1' : Reshape block reduction
 * Block '<S4>/Reshape2' : Reshape block reduction
 * Block '<S4>/Reshape3' : Reshape block reduction
 * Block '<S4>/Reshape4' : Reshape block reduction
 * Block '<S4>/Reshape5' : Reshape block reduction
 * Block '<S4>/ymin_scale2' : Eliminated nontunable gain of 1
 * Block '<S34>/RSx' : Reshape block reduction
 * Block '<S34>/RSy' : Reshape block reduction
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'adaptive_mpc_curvature_model_deployable'
 * '<S1>'   : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller'
 * '<S2>'   : 'adaptive_mpc_curvature_model_deployable/MATLAB Function'
 * '<S3>'   : 'adaptive_mpc_curvature_model_deployable/Plant'
 * '<S4>'   : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC'
 * '<S5>'   : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/MPC Matrix Signal Check'
 * '<S6>'   : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/MPC Matrix Signal Check A'
 * '<S7>'   : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/MPC Matrix Signal Check B'
 * '<S8>'   : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/MPC Matrix Signal Check C'
 * '<S9>'   : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/MPC Matrix Signal Check D'
 * '<S10>'  : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/MPC Matrix Signal Check DX'
 * '<S11>'  : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/MPC Matrix Signal Check U'
 * '<S12>'  : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/MPC Matrix Signal Check X'
 * '<S13>'  : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/MPC Matrix Signal Check Y'
 * '<S14>'  : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/MPC Matrix Signal Check1'
 * '<S15>'  : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/MPC Matrix Signal Check2'
 * '<S16>'  : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/MPC Preview Signal Check'
 * '<S17>'  : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/MPC Preview Signal Check1'
 * '<S18>'  : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/MPC Preview Signal Check2'
 * '<S19>'  : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/MPC Preview Signal Check3'
 * '<S20>'  : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/MPC Preview Signal Check4'
 * '<S21>'  : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/MPC Preview Signal Check5'
 * '<S22>'  : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/MPC Preview Signal Check6'
 * '<S23>'  : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/MPC Preview Signal Check7'
 * '<S24>'  : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/MPC Preview Signal Check8'
 * '<S25>'  : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/MPC Scalar Signal Check'
 * '<S26>'  : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/MPC Scalar Signal Check1'
 * '<S27>'  : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/MPC Scalar Signal Check2'
 * '<S28>'  : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/MPC Vector Signal Check'
 * '<S29>'  : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/MPC Vector Signal Check1'
 * '<S30>'  : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/MPC Vector Signal Check6'
 * '<S31>'  : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/moorx'
 * '<S32>'  : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/optimizer'
 * '<S33>'  : 'adaptive_mpc_curvature_model_deployable/Adaptive MPC Controller/MPC/optimizer/FixedHorizonOptimizer'
 * '<S34>'  : 'adaptive_mpc_curvature_model_deployable/Plant/Discrete Varying State Space'
 * '<S35>'  : 'adaptive_mpc_curvature_model_deployable/Plant/MATLAB Function'
 */
#endif                          /* adaptive_mpc_curvature_model_deployable_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

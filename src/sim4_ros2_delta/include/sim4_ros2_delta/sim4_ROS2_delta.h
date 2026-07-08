//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: sim4_ROS2_delta.h
//
// Code generated for Simulink model 'sim4_ROS2_delta'.
//
// Model version                  : 1.230
// Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
// C/C++ source code generated on : Thu Jul  2 13:41:44 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: AMD->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef sim4_ROS2_delta_h_
#define sim4_ROS2_delta_h_
#include "rtwtypes.h"
#include "rtw_extmode.h"
#include "sysran_types.h"
#include "slros2_initialize.h"
#include "sim4_ROS2_delta_types.h"

extern "C"
{

#include "rtGetNaN.h"

}

extern "C"
{

#include "rt_nonfinite.h"

}

#include <stddef.h>

// Macros for accessing real-time model data structure
#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWExtModeInfo
#define rtmGetRTWExtModeInfo(rtm)      ((rtm)->extModeInfo)
#endif

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
#define rtmGetT(rtm)                   ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                (&(rtm)->Timing.taskTime0)
#endif

// Block signals (default storage)
struct B_sim4_ROS2_delta_T {
  SL_Bus_custom_messages_DeltaTarget In1;// '<S7>/In1'
  SL_Bus_custom_messages_DeltaTarget rtb_SourceBlock_o2_m;
  SL_Bus_custom_messages_DeltaJointAngles BusAssignment1;// '<Root>/Bus Assignment1' 
  SL_Bus_geometry_msgs_PointStamped In1_i;// '<S8>/In1'
  SL_Bus_geometry_msgs_PointStamped rtb_SourceBlock_o2_n_c;
  char_T b_zeroDelimTopic[27];
  char_T b_zeroDelimTopic_k[25];
  char_T b_zeroDelimTopic_c[22];
  real_T x_mm; // '<Root>/SigConversion_InsertedFor_Bus Selector2_at_outport_0'
  real_T y_mm; // '<Root>/SigConversion_InsertedFor_Bus Selector2_at_outport_1'
  real_T z_mm; // '<Root>/SigConversion_InsertedFor_Bus Selector2_at_outport_2'
  real_T TmpSignalConversionAtTAQSigLogg[3];
  // '<Root>/TmpSignal ConversionAtTAQSigLogging_InsertedFor_Mux2_at_outport_0Inport1' 
  real_T x;    // '<Root>/SigConversion_InsertedFor_Bus Selector3_at_outport_0'
  real_T y;    // '<Root>/SigConversion_InsertedFor_Bus Selector3_at_outport_1'
  real_T z;    // '<Root>/SigConversion_InsertedFor_Bus Selector3_at_outport_2'
  real_T Sum3[3];                      // '<Root>/Sum3'
  real_T x_pred;                       // '<Root>/MATLAB Function1'
  real_T y_pred;                       // '<Root>/MATLAB Function1'
  real_T z_pred;                       // '<Root>/MATLAB Function1'
  real_T st1;
  real_T t1;
  real_T st2;
  real_T t2;
  real_T t3;
  real_T theta1;
  real_T theta2;
  real_T theta3;
  real_T d;
  real_T d1;
  real_T d2;
  real_T b_y1;
  boolean_T ik_valid;                  // '<Root>/MATLAB Function3'
};

// Block states (default storage) for system '<Root>'
struct DW_sim4_ROS2_delta_T {
  ros_slros2_internal_block_Pub_T obj; // '<S4>/SinkBlock'
  ros_slros2_internal_block_Sub_T obj_d;// '<S6>/SourceBlock'
  ros_slros2_internal_block_Sub_T obj_l;// '<S5>/SourceBlock'
  struct {
    void *LoggedData[2];
  } Scope1_PWORK;                      // '<Root>/Scope1'

  int8_T EnabledSubsystem_SubsysRanBC; // '<S6>/Enabled Subsystem'
  int8_T EnabledSubsystem_SubsysRanBC_a;// '<S5>/Enabled Subsystem'
  boolean_T doneDoubleBufferReInit;    // '<Root>/MATLAB Function3'
  boolean_T doneDoubleBufferReInit_m;  // '<Root>/MATLAB Function1'
};

// Real-time Model Data Structure
struct tag_RTM_sim4_ROS2_delta_T {
  const char_T *errorStatus;
  RTWExtModeInfo *extModeInfo;

  //
  //  Sizes:
  //  The following substructure contains sizes information
  //  for many of the model attributes such as inputs, outputs,
  //  dwork, sample times, etc.

  struct {
    uint32_T checksums[4];
  } Sizes;

  //
  //  SpecialInfo:
  //  The following substructure contains special information
  //  related to other components that are dependent on RTW.

  struct {
    const void *mappingInfo;
  } SpecialInfo;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    time_T taskTime0;
    uint32_T clockTick0;
    time_T stepSize0;
    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

// Block signals (default storage)
#ifdef __cplusplus

extern "C"
{

#endif

  extern struct B_sim4_ROS2_delta_T sim4_ROS2_delta_B;

#ifdef __cplusplus

}

#endif

// Block states (default storage)
extern struct DW_sim4_ROS2_delta_T sim4_ROS2_delta_DW;

#ifdef __cplusplus

extern "C"
{

#endif

  // Model entry point functions
  extern void sim4_ROS2_delta_initialize(void);
  extern void sim4_ROS2_delta_step(void);
  extern void sim4_ROS2_delta_terminate(void);

#ifdef __cplusplus

}

#endif

// Real-time Model object
#ifdef __cplusplus

extern "C"
{

#endif

  extern RT_MODEL_sim4_ROS2_delta_T *const sim4_ROS2_delta_M;

#ifdef __cplusplus

}

#endif

extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/Gain1' : Eliminated nontunable gain of 1


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'sim4_ROS2_delta'
//  '<S1>'   : 'sim4_ROS2_delta/Blank Message1'
//  '<S2>'   : 'sim4_ROS2_delta/MATLAB Function1'
//  '<S3>'   : 'sim4_ROS2_delta/MATLAB Function3'
//  '<S4>'   : 'sim4_ROS2_delta/Publish1'
//  '<S5>'   : 'sim4_ROS2_delta/Subscribe2'
//  '<S6>'   : 'sim4_ROS2_delta/Subscribe3'
//  '<S7>'   : 'sim4_ROS2_delta/Subscribe2/Enabled Subsystem'
//  '<S8>'   : 'sim4_ROS2_delta/Subscribe3/Enabled Subsystem'

#endif                                 // sim4_ROS2_delta_h_

//
// File trailer for generated code.
//
// [EOF]
//

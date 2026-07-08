//
//  ext_mode_types.h
//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  Code generation for model "sim4_ROS2_delta".
//
//  Model version              : 1.230
//  Simulink Coder version : 25.2 (R2025b) 28-Jul-2025
//  C++ source code generated on : Thu Jul  2 13:41:44 2026
//
//  Target selection: ert.tlc
//  Embedded hardware selection: AMD->x86-64 (Windows64)
//  Code generation objectives: Unspecified
//  Validation result: Not run


#ifndef EXT_MODE_TYPES_H
#define EXT_MODE_TYPES_H
#include "rtwtypes.h"

//* External Mode Event ID
typedef uint16_T extmodeEventId_T;

//* External Mode real time, measured using the target's clock
typedef uint32_T extmodeRealTime_T;

//* External Mode simulation time
typedef time_T extmodeSimulationTime_T;

//* External Mode double data type
typedef real_T extmodeDouble_T;

//* External Mode classic trigger signal
typedef real_T extmodeClassicTriggerSignal_T;

//* Run the simulation forever (infinite simulation end time)
#define EXTMODE_SIMULATION_RUN_FOREVER ((extmodeSimulationTime_T) -1)

// Define max value of 32-bit unsigned type
#define XCP_UINT32_MAX                 0xFFFFFFFFU

//* External Mode Base rate event id
#define EXTMODE_BASE_RATE_EVENT_ID     0

//* External Mode Max Event Id
#define EXTMODE_MAX_EVENT_ID           0xFF
#endif                                 // EXT_MODE_TYPES_H

// [EOF] ext_mode_types.h

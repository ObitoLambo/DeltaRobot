//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: sim4_ROS2_delta.cpp
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
#include "sim4_ROS2_delta.h"
#include "rtwtypes.h"
#include "sim4_ROS2_delta_types.h"
#include <emmintrin.h>
#include <string.h>
#include <math.h>
#include "sim4_ROS2_delta_private.h"
#include "rmw/qos_profiles.h"
#include <stddef.h>

extern "C"
{

#include "rt_nonfinite.h"

}

#include "rt_defines.h"

// Block signals (default storage)
B_sim4_ROS2_delta_T sim4_ROS2_delta_B;

// Block states (default storage)
DW_sim4_ROS2_delta_T sim4_ROS2_delta_DW;

// Real-time model
RT_MODEL_sim4_ROS2_delta_T sim4_ROS2_delta_M_ = RT_MODEL_sim4_ROS2_delta_T();
RT_MODEL_sim4_ROS2_delta_T *const sim4_ROS2_delta_M = &sim4_ROS2_delta_M_;

// Forward declaration for local functions
static void sim4_ROS2_delta_angleYZ(real_T x0, real_T b_y0, real_T z0, real_T e,
  real_T f, real_T re, real_T rf, real_T *status, real_T *theta);
static void sim4_ROS2__Subscriber_setupImpl(const
  ros_slros2_internal_block_Sub_T *obj);
static void sim4_ROS_Subscriber_setupImpl_m(const
  ros_slros2_internal_block_Sub_T *obj);
static void sim4_ROS2_d_Publisher_setupImpl(const
  ros_slros2_internal_block_Pub_T *obj);
real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(static_cast<real_T>(tmp), static_cast<real_T>(tmp_0));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

// Function for MATLAB Function: '<Root>/MATLAB Function3'
static void sim4_ROS2_delta_angleYZ(real_T x0, real_T b_y0, real_T z0, real_T e,
  real_T f, real_T re, real_T rf, real_T *status, real_T *theta)
{
  real_T a;
  real_T b;
  real_T d;
  real_T yj;
  sim4_ROS2_delta_B.b_y1 = -0.28867513459481292 * f;
  b_y0 -= 0.28867513459481292 * e;
  a = (((((x0 * x0 + b_y0 * b_y0) + z0 * z0) + rf * rf) - re * re) -
       sim4_ROS2_delta_B.b_y1 * sim4_ROS2_delta_B.b_y1) / (2.0 * z0);
  b = (sim4_ROS2_delta_B.b_y1 - b_y0) / z0;
  d = b * sim4_ROS2_delta_B.b_y1 + a;
  yj = b * b;
  d = (yj * rf + rf) * rf - d * d;
  if (d < 0.0) {
    *status = -1.0;
    *theta = 0.0;
  } else {
    yj = ((sim4_ROS2_delta_B.b_y1 - a * b) - sqrt(d)) / (yj + 1.0);
    *theta = rt_atan2d_snf(-(b * yj + a), sim4_ROS2_delta_B.b_y1 - yj) *
      57.295779513082323;
    if (yj > sim4_ROS2_delta_B.b_y1) {
      *theta += 180.0;
    }

    *status = 0.0;
  }
}

static void sim4_ROS2__Subscriber_setupImpl(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[25] = "/delta/matlab/target_xyz";
  qos_profile = rmw_qos_profile_default;

  // Start for MATLABSystem: '<S5>/SourceBlock'
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 25; i++) {
    // Start for MATLABSystem: '<S5>/SourceBlock'
    sim4_ROS2_delta_B.b_zeroDelimTopic_k[i] = b_zeroDelimTopic[i];
  }

  Sub_sim4_ROS2_delta_75.createSubscriber(&sim4_ROS2_delta_B.b_zeroDelimTopic_k
    [0], qos_profile);
}

static void sim4_ROS_Subscriber_setupImpl_m(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[22] = "/delta/ee_position_mm";
  qos_profile = rmw_qos_profile_default;

  // Start for MATLABSystem: '<S6>/SourceBlock'
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 22; i++) {
    // Start for MATLABSystem: '<S6>/SourceBlock'
    sim4_ROS2_delta_B.b_zeroDelimTopic_c[i] = b_zeroDelimTopic[i];
  }

  Sub_sim4_ROS2_delta_76.createSubscriber(&sim4_ROS2_delta_B.b_zeroDelimTopic_c
    [0], qos_profile);
}

static void sim4_ROS2_d_Publisher_setupImpl(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[27] = "/delta/matlab/joint_thetas";
  qos_profile = rmw_qos_profile_default;

  // Start for MATLABSystem: '<S4>/SinkBlock'
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 27; i++) {
    // Start for MATLABSystem: '<S4>/SinkBlock'
    sim4_ROS2_delta_B.b_zeroDelimTopic[i] = b_zeroDelimTopic[i];
  }

  Pub_sim4_ROS2_delta_72.createPublisher(&sim4_ROS2_delta_B.b_zeroDelimTopic[0],
    qos_profile);
}

// Model step function
void sim4_ROS2_delta_step(void)
{
  __m128d tmp;
  boolean_T b_varargout_1;

  // Reset subsysRan breadcrumbs
  srClearBC(sim4_ROS2_delta_DW.EnabledSubsystem_SubsysRanBC_a);

  // Reset subsysRan breadcrumbs
  srClearBC(sim4_ROS2_delta_DW.EnabledSubsystem_SubsysRanBC);

  // MATLABSystem: '<S5>/SourceBlock'
  b_varargout_1 = Sub_sim4_ROS2_delta_75.getLatestMessage
    (&sim4_ROS2_delta_B.rtb_SourceBlock_o2_m);

  // Outputs for Enabled SubSystem: '<S5>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S7>/Enable'

  // Start for MATLABSystem: '<S5>/SourceBlock'
  if (b_varargout_1) {
    // SignalConversion generated from: '<S7>/In1'
    sim4_ROS2_delta_B.In1 = sim4_ROS2_delta_B.rtb_SourceBlock_o2_m;
    srUpdateBC(sim4_ROS2_delta_DW.EnabledSubsystem_SubsysRanBC_a);
  }

  // End of Start for MATLABSystem: '<S5>/SourceBlock'
  // End of Outputs for SubSystem: '<S5>/Enabled Subsystem'

  // SignalConversion generated from: '<Root>/Bus Selector2'
  sim4_ROS2_delta_B.x_mm = sim4_ROS2_delta_B.In1.x_mm;

  // SignalConversion generated from: '<Root>/Bus Selector2'
  sim4_ROS2_delta_B.y_mm = sim4_ROS2_delta_B.In1.y_mm;

  // SignalConversion generated from: '<Root>/Bus Selector2'
  sim4_ROS2_delta_B.z_mm = sim4_ROS2_delta_B.In1.z_mm;

  // MATLAB Function: '<Root>/MATLAB Function1'
  sim4_ROS2_delta_B.x_pred = sim4_ROS2_delta_B.x_mm;
  sim4_ROS2_delta_B.z_pred = sim4_ROS2_delta_B.z_mm;
  sim4_ROS2_delta_B.y_pred = sim4_ROS2_delta_B.y_mm + 13.0;

  // SignalConversion generated from: '<Root>/Mux2'
  sim4_ROS2_delta_B.TmpSignalConversionAtTAQSigLogg[0] =
    sim4_ROS2_delta_B.x_pred;
  sim4_ROS2_delta_B.TmpSignalConversionAtTAQSigLogg[1] =
    sim4_ROS2_delta_B.y_pred;
  sim4_ROS2_delta_B.TmpSignalConversionAtTAQSigLogg[2] =
    sim4_ROS2_delta_B.z_pred;

  // MATLABSystem: '<S6>/SourceBlock'
  b_varargout_1 = Sub_sim4_ROS2_delta_76.getLatestMessage
    (&sim4_ROS2_delta_B.rtb_SourceBlock_o2_n_c);

  // Outputs for Enabled SubSystem: '<S6>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S8>/Enable'

  // Start for MATLABSystem: '<S6>/SourceBlock'
  if (b_varargout_1) {
    // SignalConversion generated from: '<S8>/In1'
    sim4_ROS2_delta_B.In1_i = sim4_ROS2_delta_B.rtb_SourceBlock_o2_n_c;
    srUpdateBC(sim4_ROS2_delta_DW.EnabledSubsystem_SubsysRanBC);
  }

  // End of Start for MATLABSystem: '<S6>/SourceBlock'
  // End of Outputs for SubSystem: '<S6>/Enabled Subsystem'

  // SignalConversion generated from: '<Root>/Bus Selector3'
  sim4_ROS2_delta_B.x = sim4_ROS2_delta_B.In1_i.point.x;

  // SignalConversion generated from: '<Root>/Bus Selector3'
  sim4_ROS2_delta_B.y = sim4_ROS2_delta_B.In1_i.point.y;

  // SignalConversion generated from: '<Root>/Bus Selector3'
  sim4_ROS2_delta_B.z = sim4_ROS2_delta_B.In1_i.point.z;

  // Sum: '<Root>/Sum2'
  tmp = _mm_set_pd(sim4_ROS2_delta_B.y, sim4_ROS2_delta_B.x);
  tmp = _mm_add_pd(_mm_sub_pd(_mm_loadu_pd
    (&sim4_ROS2_delta_B.TmpSignalConversionAtTAQSigLogg[0]), tmp), tmp);

  // Sum: '<Root>/Sum3' incorporates:
  //   Sum: '<Root>/Sum2'

  _mm_storeu_pd(&sim4_ROS2_delta_B.Sum3[0], tmp);
  sim4_ROS2_delta_B.Sum3[2] =
    (sim4_ROS2_delta_B.TmpSignalConversionAtTAQSigLogg[2] - sim4_ROS2_delta_B.z)
    + sim4_ROS2_delta_B.z;

  // MATLAB Function: '<Root>/MATLAB Function3'
  sim4_ROS2_delta_B.theta1 = 0.0;
  sim4_ROS2_delta_B.theta2 = 0.0;
  sim4_ROS2_delta_B.theta3 = 0.0;
  sim4_ROS2_delta_B.ik_valid = false;

  // BusAssignment: '<Root>/Bus Assignment1'
  memset(&sim4_ROS2_delta_B.BusAssignment1, 0, sizeof
         (SL_Bus_custom_messages_DeltaJointAngles));

  // MATLAB Function: '<Root>/MATLAB Function3'
  if ((!(fabs(sim4_ROS2_delta_B.Sum3[0]) > 250.0)) && (!(fabs
        (sim4_ROS2_delta_B.Sum3[1]) > 250.0)) && ((!(sim4_ROS2_delta_B.Sum3[2] <
         -670.0)) && (!(sim4_ROS2_delta_B.Sum3[2] > -323.0)))) {
    sim4_ROS2_delta_angleYZ(sim4_ROS2_delta_B.Sum3[0], sim4_ROS2_delta_B.Sum3[1],
      sim4_ROS2_delta_B.Sum3[2], 35.0, 157.0, 400.0, 200.0,
      &sim4_ROS2_delta_B.st1, &sim4_ROS2_delta_B.t1);
    if (!(sim4_ROS2_delta_B.st1 != 0.0)) {
      sim4_ROS2_delta_B.st1 = sim4_ROS2_delta_B.Sum3[0] * -0.5;
      sim4_ROS2_delta_B.d = sim4_ROS2_delta_B.Sum3[1] * 0.8660254037844386;
      sim4_ROS2_delta_B.d1 = sim4_ROS2_delta_B.Sum3[1] * -0.5;
      sim4_ROS2_delta_B.d2 = sim4_ROS2_delta_B.Sum3[0] * 0.8660254037844386;
      sim4_ROS2_delta_angleYZ(sim4_ROS2_delta_B.st1 + sim4_ROS2_delta_B.d,
        sim4_ROS2_delta_B.d1 - sim4_ROS2_delta_B.d2, sim4_ROS2_delta_B.Sum3[2],
        35.0, 157.0, 400.0, 200.0, &sim4_ROS2_delta_B.st2, &sim4_ROS2_delta_B.t2);
      if (!(sim4_ROS2_delta_B.st2 != 0.0)) {
        sim4_ROS2_delta_angleYZ(sim4_ROS2_delta_B.st1 - sim4_ROS2_delta_B.d,
          sim4_ROS2_delta_B.d1 + sim4_ROS2_delta_B.d2, sim4_ROS2_delta_B.Sum3[2],
          35.0, 157.0, 400.0, 200.0, &sim4_ROS2_delta_B.st2,
          &sim4_ROS2_delta_B.t3);
        if ((!(sim4_ROS2_delta_B.st2 != 0.0)) && (!(sim4_ROS2_delta_B.t1 < -5.0))
            && (!(sim4_ROS2_delta_B.t1 > 90.0)) && (!(sim4_ROS2_delta_B.t2 <
              -5.0)) && (!(sim4_ROS2_delta_B.t2 > 90.0)) &&
            (!(sim4_ROS2_delta_B.t3 < -5.0)) && (!(sim4_ROS2_delta_B.t3 > 90.0)))
        {
          sim4_ROS2_delta_B.theta1 = sim4_ROS2_delta_B.t1;
          sim4_ROS2_delta_B.theta2 = sim4_ROS2_delta_B.t2;
          sim4_ROS2_delta_B.theta3 = sim4_ROS2_delta_B.t3;
          sim4_ROS2_delta_B.ik_valid = true;
        }
      }
    }
  }

  // BusAssignment: '<Root>/Bus Assignment1'
  sim4_ROS2_delta_B.BusAssignment1.theta1_deg = sim4_ROS2_delta_B.theta1;
  sim4_ROS2_delta_B.BusAssignment1.theta2_deg = sim4_ROS2_delta_B.theta2;
  sim4_ROS2_delta_B.BusAssignment1.theta3_deg = sim4_ROS2_delta_B.theta3;
  sim4_ROS2_delta_B.BusAssignment1.ik_valid = sim4_ROS2_delta_B.ik_valid;

  // MATLABSystem: '<S4>/SinkBlock'
  Pub_sim4_ROS2_delta_72.publish(&sim4_ROS2_delta_B.BusAssignment1);

  // Update absolute time for base rate
  // The "clockTick0" counts the number of times the code of this task has
  //  been executed. The absolute time is the multiplication of "clockTick0"
  //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
  //  overflow during the application lifespan selected.

  sim4_ROS2_delta_M->Timing.taskTime0 =
    ((time_T)(++sim4_ROS2_delta_M->Timing.clockTick0)) *
    sim4_ROS2_delta_M->Timing.stepSize0;
}

// Model initialize function
void sim4_ROS2_delta_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));
  rtmSetTFinal(sim4_ROS2_delta_M, -1);
  sim4_ROS2_delta_M->Timing.stepSize0 = 0.2;

  // External mode info
  sim4_ROS2_delta_M->Sizes.checksums[0] = (1594700945U);
  sim4_ROS2_delta_M->Sizes.checksums[1] = (621383653U);
  sim4_ROS2_delta_M->Sizes.checksums[2] = (2893097899U);
  sim4_ROS2_delta_M->Sizes.checksums[3] = (958291256U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[8];
    sim4_ROS2_delta_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    systemRan[2] = &rtAlwaysEnabled;
    systemRan[3] = &rtAlwaysEnabled;
    systemRan[4] = (sysRanDType *)
      &sim4_ROS2_delta_DW.EnabledSubsystem_SubsysRanBC_a;
    systemRan[5] = &rtAlwaysEnabled;
    systemRan[6] = (sysRanDType *)
      &sim4_ROS2_delta_DW.EnabledSubsystem_SubsysRanBC;
    systemRan[7] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(sim4_ROS2_delta_M->extModeInfo,
      &sim4_ROS2_delta_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(sim4_ROS2_delta_M->extModeInfo,
                        sim4_ROS2_delta_M->Sizes.checksums);
    rteiSetTPtr(sim4_ROS2_delta_M->extModeInfo, rtmGetTPtr(sim4_ROS2_delta_M));
  }

  // Start for MATLABSystem: '<S5>/SourceBlock'
  sim4_ROS2_delta_DW.obj_l.QOSAvoidROSNamespaceConventions = false;
  sim4_ROS2_delta_DW.obj_l.matlabCodegenIsDeleted = false;
  sim4_ROS2_delta_DW.obj_l.isSetupComplete = false;
  sim4_ROS2_delta_DW.obj_l.isInitialized = 1;
  sim4_ROS2__Subscriber_setupImpl(&sim4_ROS2_delta_DW.obj_l);
  sim4_ROS2_delta_DW.obj_l.isSetupComplete = true;

  // Start for MATLABSystem: '<S6>/SourceBlock'
  sim4_ROS2_delta_DW.obj_d.QOSAvoidROSNamespaceConventions = false;
  sim4_ROS2_delta_DW.obj_d.matlabCodegenIsDeleted = false;
  sim4_ROS2_delta_DW.obj_d.isSetupComplete = false;
  sim4_ROS2_delta_DW.obj_d.isInitialized = 1;
  sim4_ROS_Subscriber_setupImpl_m(&sim4_ROS2_delta_DW.obj_d);
  sim4_ROS2_delta_DW.obj_d.isSetupComplete = true;

  // Start for MATLABSystem: '<S4>/SinkBlock'
  sim4_ROS2_delta_DW.obj.QOSAvoidROSNamespaceConventions = false;
  sim4_ROS2_delta_DW.obj.matlabCodegenIsDeleted = false;
  sim4_ROS2_delta_DW.obj.isSetupComplete = false;
  sim4_ROS2_delta_DW.obj.isInitialized = 1;
  sim4_ROS2_d_Publisher_setupImpl(&sim4_ROS2_delta_DW.obj);
  sim4_ROS2_delta_DW.obj.isSetupComplete = true;
}

// Model terminate function
void sim4_ROS2_delta_terminate(void)
{
  // Terminate for MATLABSystem: '<S5>/SourceBlock'
  if (!sim4_ROS2_delta_DW.obj_l.matlabCodegenIsDeleted) {
    sim4_ROS2_delta_DW.obj_l.matlabCodegenIsDeleted = true;
    if ((sim4_ROS2_delta_DW.obj_l.isInitialized == 1) &&
        sim4_ROS2_delta_DW.obj_l.isSetupComplete) {
      Sub_sim4_ROS2_delta_75.resetSubscriberPtr();//();
    }
  }

  // End of Terminate for MATLABSystem: '<S5>/SourceBlock'

  // Terminate for MATLABSystem: '<S6>/SourceBlock'
  if (!sim4_ROS2_delta_DW.obj_d.matlabCodegenIsDeleted) {
    sim4_ROS2_delta_DW.obj_d.matlabCodegenIsDeleted = true;
    if ((sim4_ROS2_delta_DW.obj_d.isInitialized == 1) &&
        sim4_ROS2_delta_DW.obj_d.isSetupComplete) {
      Sub_sim4_ROS2_delta_76.resetSubscriberPtr();//();
    }
  }

  // End of Terminate for MATLABSystem: '<S6>/SourceBlock'
  // Terminate for MATLABSystem: '<S4>/SinkBlock'
  if (!sim4_ROS2_delta_DW.obj.matlabCodegenIsDeleted) {
    sim4_ROS2_delta_DW.obj.matlabCodegenIsDeleted = true;
    if ((sim4_ROS2_delta_DW.obj.isInitialized == 1) &&
        sim4_ROS2_delta_DW.obj.isSetupComplete) {
      Pub_sim4_ROS2_delta_72.resetPublisherPtr();//();
    }
  }

  // End of Terminate for MATLABSystem: '<S4>/SinkBlock'
}

//
// File trailer for generated code.
//
// [EOF]
//

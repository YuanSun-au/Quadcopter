/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: crazyflie.c
 *
 * Code generated for Simulink model 'crazyflie'.
 *
 * Model version                  : 1.263
 * Simulink Coder version         : 9.0 (R2018b) 24-May-2018
 * C/C++ source code generated on : Sun May 26 15:50:22 2019
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "crazyflie.h"

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Model step function */
void crazyflie_step(void)
{
  real_T scale;
  real_T absxk;
  real_T t;
  real_T rtb_Gain2;
  real_T rtb_Saturation[4];
  int32_T i;
  real_T tmp;
  real_T tmp_0;
  real_T tmp_1;
  real_T tmp_2;
  real_T u0;

  /* MATLAB Function: '<S1>/MATLAB Function' incorporates:
   *  Gain: '<Root>/Gain1'
   *  Inport: '<Root>/Acc_x'
   *  Inport: '<Root>/Acc_y'
   *  Inport: '<Root>/Acc_z'
   */
  /* Normalization Part */
  /* MATLAB Function 'Complementary filter/MATLAB Function': '<S2>:1' */
  /* '<S2>:1:4' Acceleration_Vector = [Acc_x,Acc_y,Acc_z]'; */
  /* '<S2>:1:5' Normalized_Acceleration = Acceleration_Vector / norm(Acceleration_Vector); */
  scale = 3.3121686421112381E-170;
  absxk = fabs(-rtU.Acc_x);
  if (absxk > 3.3121686421112381E-170) {
    rtb_Gain2 = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    rtb_Gain2 = t * t;
  }

  absxk = fabs(rtU.Acc_y);
  if (absxk > scale) {
    t = scale / absxk;
    rtb_Gain2 = rtb_Gain2 * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    rtb_Gain2 += t * t;
  }

  absxk = fabs(rtU.Acc_z);
  if (absxk > scale) {
    t = scale / absxk;
    rtb_Gain2 = rtb_Gain2 * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    rtb_Gain2 += t * t;
  }

  rtb_Gain2 = scale * sqrt(rtb_Gain2);
  absxk = rtU.Acc_y / rtb_Gain2;
  t = rtU.Acc_z / rtb_Gain2;

  /* Sum: '<S1>/Sum1' incorporates:
   *  Delay: '<S1>/Delay'
   *  Gain: '<Root>/Gain'
   *  Gain: '<Root>/Gain1'
   *  Gain: '<S1>/Gain1'
   *  Gain: '<S1>/High Pass Filter (0.98)'
   *  Gain: '<S1>/Low Pass Filter (0.02)'
   *  Inport: '<Root>/Acc_x'
   *  Inport: '<Root>/Gyro_x'
   *  Inport: '<Root>/Gyro_y'
   *  MATLAB Function: '<S1>/MATLAB Function'
   *  Sum: '<S1>/Sum'
   */
  /* '<S2>:1:6' fx_bar=Normalized_Acceleration(1); */
  /* '<S2>:1:7' fy_bar=Normalized_Acceleration(2); */
  /* '<S2>:1:8' fz_bar=Normalized_Acceleration(3); */
  /* Pitch and Roll Angles */
  /* '<S2>:1:10' phi = atan2d(fy_bar,fz_bar); */
  /* '<S2>:1:11' theta = atan2d(-fx_bar,((fy_bar^2+fz_bar^2)^0.5)); */
  scale = (0.01 * rtU.Gyro_x + rtDW.Delay_DSTATE[0]) * 0.98 + 57.295779513082323
    * atan2(absxk, t) * 0.02;
  rtb_Gain2 = atan2(-(-rtU.Acc_x / rtb_Gain2), sqrt(absxk * absxk + t * t)) *
    57.295779513082323 * 0.02 + (0.01 * -rtU.Gyro_y + rtDW.Delay_DSTATE[1]) *
    0.98;

  /* SignalConversion: '<Root>/TmpSignal ConversionAtGain4Inport1' incorporates:
   *  Gain: '<Root>/Coversion1'
   *  Gain: '<Root>/Coversion3'
   *  Inport: '<Root>/Ref_Pitch'
   *  Inport: '<Root>/Ref_Roll'
   */
  absxk = 0.017453292519943295 * rtU.Ref_Roll;
  t = 0.017453292519943295 * rtU.Ref_Pitch;

  /* Gain: '<Root>/Coversion' incorporates:
   *  Gain: '<Root>/Gain3'
   *  Inport: '<Root>/Gyro_x'
   *  Inport: '<Root>/Gyro_y'
   */
  tmp = 0.017453292519943295 * scale;
  tmp_0 = 0.017453292519943295 * -rtb_Gain2;
  tmp_1 = 0.017453292519943295 * rtU.Gyro_x;
  tmp_2 = 0.017453292519943295 * rtU.Gyro_y;
  for (i = 0; i < 4; i++) {
    /* Gain: '<Root>/-K*x' */
    u0 = rtConstP.Kx_Gain[i + 12] * tmp_2 + (rtConstP.Kx_Gain[i + 8] * tmp_1 +
      (rtConstP.Kx_Gain[i + 4] * tmp_0 + rtConstP.Kx_Gain[i] * tmp));

    /* Saturate: '<Root>/Saturation' incorporates:
     *  Gain: '<Root>/Gain4'
     *  Gain: '<Root>/torque2uint'
     *  Sum: '<Root>/Sum1'
     *  Sum: '<Root>/Sum2'
     */
    u0 = ((rtConstP.Gain4_Gain[i + 4] * t + rtConstP.Gain4_Gain[i] * absxk) + u0)
      * 445368.67142371729 + 2500.0;
    if (u0 > 65000.0) {
      rtb_Saturation[i] = 65000.0;
    } else if (u0 < 0.0) {
      rtb_Saturation[i] = 0.0;
    } else {
      rtb_Saturation[i] = u0;
    }

    /* End of Saturate: '<Root>/Saturation' */
  }

  /* DataTypeConversion: '<Root>/ToUint16' */
  if (rtb_Saturation[0] < 65536.0) {
    /* Outport: '<Root>/Motor_1' */
    rtY.Motor_1 = (uint16_T)rtb_Saturation[0];
  } else {
    /* Outport: '<Root>/Motor_1' */
    rtY.Motor_1 = MAX_uint16_T;
  }

  /* End of DataTypeConversion: '<Root>/ToUint16' */

  /* DataTypeConversion: '<Root>/ToUint16_1' */
  if (rtb_Saturation[1] < 65536.0) {
    /* Outport: '<Root>/Motor_2' */
    rtY.Motor_2 = (uint16_T)rtb_Saturation[1];
  } else {
    /* Outport: '<Root>/Motor_2' */
    rtY.Motor_2 = MAX_uint16_T;
  }

  /* End of DataTypeConversion: '<Root>/ToUint16_1' */

  /* DataTypeConversion: '<Root>/ToUint16_2' */
  if (rtb_Saturation[2] < 65536.0) {
    /* Outport: '<Root>/Motor_3' */
    rtY.Motor_3 = (uint16_T)rtb_Saturation[2];
  } else {
    /* Outport: '<Root>/Motor_3' */
    rtY.Motor_3 = MAX_uint16_T;
  }

  /* End of DataTypeConversion: '<Root>/ToUint16_2' */

  /* DataTypeConversion: '<Root>/ToUint16_3' */
  if (rtb_Saturation[3] < 65536.0) {
    /* Outport: '<Root>/Motor_4' */
    rtY.Motor_4 = (uint16_T)rtb_Saturation[3];
  } else {
    /* Outport: '<Root>/Motor_4' */
    rtY.Motor_4 = MAX_uint16_T;
  }

  /* End of DataTypeConversion: '<Root>/ToUint16_3' */

  /* Outport: '<Root>/Log1' */
  rtY.Log1 = scale;

  /* Outport: '<Root>/Log2' incorporates:
   *  Gain: '<Root>/Gain3'
   */
  rtY.Log2 = -rtb_Gain2;

  /* Outport: '<Root>/Log3' incorporates:
   *  Inport: '<Root>/Gyro_x'
   */
  rtY.Log3 = rtU.Gyro_x;

  /* Outport: '<Root>/Log4' incorporates:
   *  Inport: '<Root>/Gyro_y'
   */
  rtY.Log4 = rtU.Gyro_y;

  /* Update for Delay: '<S1>/Delay' */
  rtDW.Delay_DSTATE[0] = scale;
  rtDW.Delay_DSTATE[1] = rtb_Gain2;
}

/* Model initialize function */
void crazyflie_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void crazyflie_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

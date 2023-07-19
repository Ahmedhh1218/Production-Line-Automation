/* Simscape target specific file.
 * This file is generated for the Simscape network associated with the solver block 'V1_2P/Body Configuration/Solver Configuration'.
 */

#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"
#include "sm_CTarget.h"

void V1_2P_4d7d5d8d_1_setTargets(const RuntimeDerivedValuesBundle *rtdv, CTarget
  *targets)
{
  (void) rtdv;
  (void) targets;
}

void V1_2P_4d7d5d8d_1_resetAsmStateVector(const void *mech, double *state)
{
  double xx[2];
  (void) mech;
  xx[0] = 0.0;
  xx[1] = 1.0;
  state[0] = xx[0];
  state[1] = xx[0];
  state[2] = xx[0];
  state[3] = xx[0];
  state[4] = xx[0];
  state[5] = xx[1];
  state[6] = xx[0];
  state[7] = xx[0];
  state[8] = xx[0];
  state[9] = xx[0];
  state[10] = xx[0];
  state[11] = xx[0];
  state[12] = xx[0];
  state[13] = xx[0];
  state[14] = xx[0];
  state[15] = xx[0];
  state[16] = xx[0];
  state[17] = xx[0];
  state[18] = xx[1];
  state[19] = xx[0];
  state[20] = xx[0];
  state[21] = xx[0];
  state[22] = xx[0];
  state[23] = xx[0];
  state[24] = xx[0];
  state[25] = xx[0];
  state[26] = xx[0];
  state[27] = xx[0];
  state[28] = xx[0];
  state[29] = xx[0];
  state[30] = xx[0];
  state[31] = xx[0];
  state[32] = xx[0];
  state[33] = xx[0];
}

void V1_2P_4d7d5d8d_1_initializeTrackedAngleState(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const int *modeVector, const double
  *motionData, double *state)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
  (void) modeVector;
  (void) motionData;
}

void V1_2P_4d7d5d8d_1_computeDiscreteState(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, double *state)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
}

void V1_2P_4d7d5d8d_1_adjustPosition(const void *mech, const double *dofDeltas,
  double *state)
{
  double xx[17];
  (void) mech;
  xx[0] = state[5];
  xx[1] = state[6];
  xx[2] = state[7];
  xx[3] = state[8];
  xx[4] = dofDeltas[4];
  xx[5] = dofDeltas[5];
  xx[6] = dofDeltas[6];
  pm_math_Quaternion_compDeriv_ra(xx + 0, xx + 4, xx + 7);
  xx[0] = state[5] + xx[7];
  xx[1] = state[6] + xx[8];
  xx[2] = state[7] + xx[9];
  xx[3] = state[8] + xx[10];
  xx[4] = 1.0e-64;
  xx[5] = sqrt(xx[0] * xx[0] + xx[1] * xx[1] + xx[2] * xx[2] + xx[3] * xx[3]);
  if (xx[4] > xx[5])
    xx[5] = xx[4];
  xx[6] = state[18];
  xx[7] = state[19];
  xx[8] = state[20];
  xx[9] = state[21];
  xx[10] = dofDeltas[10];
  xx[11] = dofDeltas[11];
  xx[12] = dofDeltas[12];
  pm_math_Quaternion_compDeriv_ra(xx + 6, xx + 10, xx + 13);
  xx[6] = state[18] + xx[13];
  xx[7] = state[19] + xx[14];
  xx[8] = state[20] + xx[15];
  xx[9] = state[21] + xx[16];
  xx[10] = sqrt(xx[6] * xx[6] + xx[7] * xx[7] + xx[8] * xx[8] + xx[9] * xx[9]);
  if (xx[4] > xx[10])
    xx[10] = xx[4];
  state[0] = state[0] + dofDeltas[0];
  state[2] = state[2] + dofDeltas[1];
  state[3] = state[3] + dofDeltas[2];
  state[4] = state[4] + dofDeltas[3];
  state[5] = xx[0] / xx[5];
  state[6] = xx[1] / xx[5];
  state[7] = xx[2] / xx[5];
  state[8] = xx[3] / xx[5];
  state[15] = state[15] + dofDeltas[7];
  state[16] = state[16] + dofDeltas[8];
  state[17] = state[17] + dofDeltas[9];
  state[18] = xx[6] / xx[10];
  state[19] = xx[7] / xx[10];
  state[20] = xx[8] / xx[10];
  state[21] = xx[9] / xx[10];
  state[28] = state[28] + dofDeltas[13];
  state[30] = state[30] + dofDeltas[14];
  state[32] = state[32] + dofDeltas[15];
}

static void perturbAsmJointPrimitiveState_0_0(double mag, double *state)
{
  state[0] = state[0] + mag;
}

static void perturbAsmJointPrimitiveState_0_0v(double mag, double *state)
{
  state[0] = state[0] + mag;
  state[1] = state[1] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_1_0(double mag, double *state)
{
  state[2] = state[2] + mag;
}

static void perturbAsmJointPrimitiveState_1_0v(double mag, double *state)
{
  state[2] = state[2] + mag;
  state[9] = state[9] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_1_1(double mag, double *state)
{
  state[3] = state[3] + mag;
}

static void perturbAsmJointPrimitiveState_1_1v(double mag, double *state)
{
  state[3] = state[3] + mag;
  state[10] = state[10] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_1_2(double mag, double *state)
{
  state[4] = state[4] + mag;
}

static void perturbAsmJointPrimitiveState_1_2v(double mag, double *state)
{
  state[4] = state[4] + mag;
  state[11] = state[11] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_1_3(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[0] = state[5];
  xx[1] = state[6];
  xx[2] = state[7];
  xx[3] = state[8];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 0, xx + 4);
  state[5] = xx[4];
  state[6] = xx[5];
  state[7] = xx[6];
  state[8] = xx[7];
}

static void perturbAsmJointPrimitiveState_1_3v(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[3] = state[5];
  xx[4] = state[6];
  xx[5] = state[7];
  xx[6] = state[8];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 3, xx + 7);
  state[5] = xx[7];
  state[6] = xx[8];
  state[7] = xx[9];
  state[8] = xx[10];
  state[12] = state[12] + 1.2 * mag;
  state[13] = state[13] - xx[2];
  state[14] = state[14] + 0.9 * mag;
}

static void perturbAsmJointPrimitiveState_2_0(double mag, double *state)
{
  state[15] = state[15] + mag;
}

static void perturbAsmJointPrimitiveState_2_0v(double mag, double *state)
{
  state[15] = state[15] + mag;
  state[22] = state[22] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_2_1(double mag, double *state)
{
  state[16] = state[16] + mag;
}

static void perturbAsmJointPrimitiveState_2_1v(double mag, double *state)
{
  state[16] = state[16] + mag;
  state[23] = state[23] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_2_2(double mag, double *state)
{
  state[17] = state[17] + mag;
}

static void perturbAsmJointPrimitiveState_2_2v(double mag, double *state)
{
  state[17] = state[17] + mag;
  state[24] = state[24] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_2_3(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[0] = state[18];
  xx[1] = state[19];
  xx[2] = state[20];
  xx[3] = state[21];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 0, xx + 4);
  state[18] = xx[4];
  state[19] = xx[5];
  state[20] = xx[6];
  state[21] = xx[7];
}

static void perturbAsmJointPrimitiveState_2_3v(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[3] = state[18];
  xx[4] = state[19];
  xx[5] = state[20];
  xx[6] = state[21];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 3, xx + 7);
  state[18] = xx[7];
  state[19] = xx[8];
  state[20] = xx[9];
  state[21] = xx[10];
  state[25] = state[25] + 1.2 * mag;
  state[26] = state[26] - xx[2];
  state[27] = state[27] + 0.9 * mag;
}

static void perturbAsmJointPrimitiveState_3_0(double mag, double *state)
{
  state[28] = state[28] + mag;
}

static void perturbAsmJointPrimitiveState_3_0v(double mag, double *state)
{
  state[28] = state[28] + mag;
  state[29] = state[29] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_4_0(double mag, double *state)
{
  state[30] = state[30] + mag;
}

static void perturbAsmJointPrimitiveState_4_0v(double mag, double *state)
{
  state[30] = state[30] + mag;
  state[31] = state[31] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_5_0(double mag, double *state)
{
  state[32] = state[32] + mag;
}

static void perturbAsmJointPrimitiveState_5_0v(double mag, double *state)
{
  state[32] = state[32] + mag;
  state[33] = state[33] - 0.875 * mag;
}

void V1_2P_4d7d5d8d_1_perturbAsmJointPrimitiveState(const void *mech, size_t
  stageIdx, size_t primIdx, double mag, boolean_T doPerturbVelocity, double
  *state)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) mag;
  (void) doPerturbVelocity;
  (void) state;
  switch ((stageIdx * 6 + primIdx) * 2 + (doPerturbVelocity ? 1 : 0))
  {
   case 0:
    perturbAsmJointPrimitiveState_0_0(mag, state);
    break;

   case 1:
    perturbAsmJointPrimitiveState_0_0v(mag, state);
    break;

   case 12:
    perturbAsmJointPrimitiveState_1_0(mag, state);
    break;

   case 13:
    perturbAsmJointPrimitiveState_1_0v(mag, state);
    break;

   case 14:
    perturbAsmJointPrimitiveState_1_1(mag, state);
    break;

   case 15:
    perturbAsmJointPrimitiveState_1_1v(mag, state);
    break;

   case 16:
    perturbAsmJointPrimitiveState_1_2(mag, state);
    break;

   case 17:
    perturbAsmJointPrimitiveState_1_2v(mag, state);
    break;

   case 18:
    perturbAsmJointPrimitiveState_1_3(mag, state);
    break;

   case 19:
    perturbAsmJointPrimitiveState_1_3v(mag, state);
    break;

   case 24:
    perturbAsmJointPrimitiveState_2_0(mag, state);
    break;

   case 25:
    perturbAsmJointPrimitiveState_2_0v(mag, state);
    break;

   case 26:
    perturbAsmJointPrimitiveState_2_1(mag, state);
    break;

   case 27:
    perturbAsmJointPrimitiveState_2_1v(mag, state);
    break;

   case 28:
    perturbAsmJointPrimitiveState_2_2(mag, state);
    break;

   case 29:
    perturbAsmJointPrimitiveState_2_2v(mag, state);
    break;

   case 30:
    perturbAsmJointPrimitiveState_2_3(mag, state);
    break;

   case 31:
    perturbAsmJointPrimitiveState_2_3v(mag, state);
    break;

   case 36:
    perturbAsmJointPrimitiveState_3_0(mag, state);
    break;

   case 37:
    perturbAsmJointPrimitiveState_3_0v(mag, state);
    break;

   case 48:
    perturbAsmJointPrimitiveState_4_0(mag, state);
    break;

   case 49:
    perturbAsmJointPrimitiveState_4_0v(mag, state);
    break;

   case 60:
    perturbAsmJointPrimitiveState_5_0(mag, state);
    break;

   case 61:
    perturbAsmJointPrimitiveState_5_0v(mag, state);
    break;
  }
}

static void computePosDofBlendMatrix_1_3(const double *state, int partialType,
  double *matrix)
{
  double xx[20];
  xx[0] = 9.87654321;
  xx[1] = 2.0;
  xx[2] = xx[1] * (state[6] * state[7] - state[5] * state[8]);
  xx[3] = xx[2] * xx[2];
  xx[4] = 1.0;
  xx[5] = (state[5] * state[5] + state[6] * state[6]) * xx[1] - xx[4];
  xx[6] = xx[5] * xx[5];
  xx[7] = sqrt(xx[3] + xx[6]);
  xx[8] = xx[7] == 0.0 ? 0.0 : - xx[2] / xx[7];
  xx[9] = xx[6] + xx[3];
  xx[3] = sqrt(xx[9]);
  xx[6] = xx[3] == 0.0 ? 0.0 : xx[5] / xx[3];
  xx[10] = 0.0;
  xx[11] = (state[6] * state[8] + state[5] * state[7]) * xx[1];
  xx[1] = sqrt(xx[9] + xx[11] * xx[11]);
  xx[12] = xx[1] == 0.0 ? 0.0 : xx[5] / xx[1];
  xx[14] = xx[8];
  xx[15] = xx[6];
  xx[16] = xx[10];
  xx[17] = xx[8];
  xx[18] = xx[8];
  xx[19] = xx[12];
  xx[6] = xx[13 + (partialType)];
  xx[8] = xx[7] == 0.0 ? 0.0 : xx[5] / xx[7];
  xx[7] = xx[3] == 0.0 ? 0.0 : xx[2] / xx[3];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[2] / xx[1];
  xx[13] = xx[8];
  xx[14] = xx[7];
  xx[15] = xx[10];
  xx[16] = xx[8];
  xx[17] = xx[8];
  xx[18] = xx[3];
  xx[2] = xx[12 + (partialType)];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[11] / xx[1];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[4];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[18] = xx[3];
  xx[1] = xx[12 + (partialType)];
  xx[3] = xx[11] * xx[5];
  xx[5] = sqrt(xx[9] * xx[9] + xx[3] * xx[3]);
  xx[7] = xx[5] == 0.0 ? 0.0 : xx[9] / xx[5];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[7];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[7] = xx[11 + (partialType)];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[10];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[8] = xx[11 + (partialType)];
  xx[9] = xx[5] == 0.0 ? 0.0 : xx[3] / xx[5];
  xx[12] = xx[4];
  xx[13] = xx[4];
  xx[14] = xx[10];
  xx[15] = xx[9];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[0] = xx[11 + (partialType)];
  matrix[0] = xx[6];
  matrix[1] = xx[2];
  matrix[2] = xx[1];
  matrix[3] = xx[7];
  matrix[4] = xx[8];
  matrix[5] = xx[0];
  matrix[6] = xx[8];
  matrix[7] = xx[8];
  matrix[8] = xx[8];
}

static void computePosDofBlendMatrix_2_3(const double *state, int partialType,
  double *matrix)
{
  double xx[20];
  xx[0] = 9.87654321;
  xx[1] = 2.0;
  xx[2] = xx[1] * (state[19] * state[20] - state[18] * state[21]);
  xx[3] = xx[2] * xx[2];
  xx[4] = 1.0;
  xx[5] = (state[18] * state[18] + state[19] * state[19]) * xx[1] - xx[4];
  xx[6] = xx[5] * xx[5];
  xx[7] = sqrt(xx[3] + xx[6]);
  xx[8] = xx[7] == 0.0 ? 0.0 : - xx[2] / xx[7];
  xx[9] = xx[6] + xx[3];
  xx[3] = sqrt(xx[9]);
  xx[6] = xx[3] == 0.0 ? 0.0 : xx[5] / xx[3];
  xx[10] = 0.0;
  xx[11] = (state[19] * state[21] + state[18] * state[20]) * xx[1];
  xx[1] = sqrt(xx[9] + xx[11] * xx[11]);
  xx[12] = xx[1] == 0.0 ? 0.0 : xx[5] / xx[1];
  xx[14] = xx[8];
  xx[15] = xx[6];
  xx[16] = xx[10];
  xx[17] = xx[8];
  xx[18] = xx[8];
  xx[19] = xx[12];
  xx[6] = xx[13 + (partialType)];
  xx[8] = xx[7] == 0.0 ? 0.0 : xx[5] / xx[7];
  xx[7] = xx[3] == 0.0 ? 0.0 : xx[2] / xx[3];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[2] / xx[1];
  xx[13] = xx[8];
  xx[14] = xx[7];
  xx[15] = xx[10];
  xx[16] = xx[8];
  xx[17] = xx[8];
  xx[18] = xx[3];
  xx[2] = xx[12 + (partialType)];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[11] / xx[1];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[4];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[18] = xx[3];
  xx[1] = xx[12 + (partialType)];
  xx[3] = xx[11] * xx[5];
  xx[5] = sqrt(xx[9] * xx[9] + xx[3] * xx[3]);
  xx[7] = xx[5] == 0.0 ? 0.0 : xx[9] / xx[5];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[7];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[7] = xx[11 + (partialType)];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[10];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[8] = xx[11 + (partialType)];
  xx[9] = xx[5] == 0.0 ? 0.0 : xx[3] / xx[5];
  xx[12] = xx[4];
  xx[13] = xx[4];
  xx[14] = xx[10];
  xx[15] = xx[9];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[0] = xx[11 + (partialType)];
  matrix[0] = xx[6];
  matrix[1] = xx[2];
  matrix[2] = xx[1];
  matrix[3] = xx[7];
  matrix[4] = xx[8];
  matrix[5] = xx[0];
  matrix[6] = xx[8];
  matrix[7] = xx[8];
  matrix[8] = xx[8];
}

void V1_2P_4d7d5d8d_1_computePosDofBlendMatrix(const void *mech, size_t stageIdx,
  size_t primIdx, const double *state, int partialType, double *matrix)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) state;
  (void) partialType;
  (void) matrix;
  switch ((stageIdx * 6 + primIdx))
  {
   case 9:
    computePosDofBlendMatrix_1_3(state, partialType, matrix);
    break;

   case 15:
    computePosDofBlendMatrix_2_3(state, partialType, matrix);
    break;
  }
}

static void computeVelDofBlendMatrix_1_3(const double *state, int partialType,
  double *matrix)
{
  double xx[15];
  (void) state;
  xx[0] = 9.87654321;
  xx[1] = 0.0;
  xx[2] = 1.0;
  xx[4] = xx[1];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[2];
  xx[10] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[2];
  xx[9] = xx[1];
  xx[11] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[2];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[12] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[13] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[14] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[0] = xx[3 + (partialType)];
  matrix[0] = xx[10];
  matrix[1] = xx[11];
  matrix[2] = xx[12];
  matrix[3] = xx[13];
  matrix[4] = xx[14];
  matrix[5] = xx[0];
  matrix[6] = xx[13];
  matrix[7] = xx[13];
  matrix[8] = xx[13];
}

static void computeVelDofBlendMatrix_2_3(const double *state, int partialType,
  double *matrix)
{
  double xx[15];
  (void) state;
  xx[0] = 9.87654321;
  xx[1] = 0.0;
  xx[2] = 1.0;
  xx[4] = xx[1];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[2];
  xx[10] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[2];
  xx[9] = xx[1];
  xx[11] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[2];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[12] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[13] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[14] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[0] = xx[3 + (partialType)];
  matrix[0] = xx[10];
  matrix[1] = xx[11];
  matrix[2] = xx[12];
  matrix[3] = xx[13];
  matrix[4] = xx[14];
  matrix[5] = xx[0];
  matrix[6] = xx[13];
  matrix[7] = xx[13];
  matrix[8] = xx[13];
}

void V1_2P_4d7d5d8d_1_computeVelDofBlendMatrix(const void *mech, size_t stageIdx,
  size_t primIdx, const double *state, int partialType, double *matrix)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) state;
  (void) partialType;
  (void) matrix;
  switch ((stageIdx * 6 + primIdx))
  {
   case 9:
    computeVelDofBlendMatrix_1_3(state, partialType, matrix);
    break;

   case 15:
    computeVelDofBlendMatrix_2_3(state, partialType, matrix);
    break;
  }
}

static void projectPartiallyTargetedPos_1_3(const double *origState, int
  partialType, double *state)
{
  boolean_T bb[1];
  double xx[24];
  xx[0] = state[6] * state[8];
  xx[1] = state[5] * state[7];
  xx[2] = 2.0;
  xx[3] = (xx[0] + xx[1]) * xx[2];
  xx[4] = fabs(xx[3]) > 1.0 ? atan2(xx[3], 0.0) : asin(xx[3]);
  xx[5] = origState[6] * origState[8];
  xx[6] = origState[5] * origState[7];
  xx[7] = (xx[5] + xx[6]) * xx[2];
  xx[8] = fabs(xx[7]) > 1.0 ? atan2(xx[7], 0.0) : asin(xx[7]);
  xx[9] = xx[4];
  xx[10] = xx[4];
  xx[11] = xx[8];
  xx[12] = xx[8];
  xx[13] = xx[4];
  xx[14] = xx[4];
  xx[15] = xx[8];
  xx[16] = xx[9 + (partialType)];
  xx[9] = cos(xx[16]);
  xx[10] = 0.99999999999999;
  bb[0] = !(fabs(xx[3]) > xx[10]);
  xx[3] = state[6] * state[7];
  xx[11] = state[5] * state[8];
  xx[12] = state[5] * state[5];
  xx[13] = 1.0;
  xx[14] = (xx[12] + state[6] * state[6]) * xx[2] - xx[13];
  xx[15] = - (xx[2] * (xx[3] - xx[11]));
  xx[14] = (xx[15] == 0.0 && xx[14] == 0.0) ? 0.0 : atan2(xx[15], xx[14]);
  if (xx[4] < 0.0)
    xx[15] = -1.0;
  else if (xx[4] > 0.0)
    xx[15] = +1.0;
  else
    xx[15] = 0.0;
  xx[4] = (xx[12] + state[8] * state[8]) * xx[2] - xx[13];
  xx[17] = - (xx[2] * (state[7] * state[8] - state[5] * state[6]));
  xx[4] = (xx[17] == 0.0 && xx[4] == 0.0) ? 0.0 : atan2(xx[17], xx[4]);
  xx[12] = 0.5;
  xx[17] = - (xx[2] * (xx[0] - xx[1]) * xx[15]);
  xx[18] = (xx[3] + xx[11]) * xx[2] * xx[15];
  xx[17] = (xx[18] == 0.0 && xx[17] == 0.0) ? 0.0 : atan2(xx[18], xx[17]);
  xx[0] = bb[0] ? xx[4] : xx[12] * xx[17];
  xx[1] = bb[0] ? xx[14] : xx[15] * xx[0];
  bb[0] = !(fabs(xx[7]) > xx[10]);
  xx[3] = origState[6] * origState[7];
  xx[4] = origState[5] * origState[8];
  xx[7] = origState[5] * origState[5];
  xx[10] = (xx[7] + origState[6] * origState[6]) * xx[2] - xx[13];
  xx[11] = - (xx[2] * (xx[3] - xx[4]));
  xx[10] = (xx[11] == 0.0 && xx[10] == 0.0) ? 0.0 : atan2(xx[11], xx[10]);
  if (xx[8] < 0.0)
    xx[11] = -1.0;
  else if (xx[8] > 0.0)
    xx[11] = +1.0;
  else
    xx[11] = 0.0;
  xx[8] = (xx[7] + origState[8] * origState[8]) * xx[2] - xx[13];
  xx[14] = - (xx[2] * (origState[7] * origState[8] - origState[5] * origState[6]));
  xx[8] = (xx[14] == 0.0 && xx[8] == 0.0) ? 0.0 : atan2(xx[14], xx[8]);
  xx[7] = - (xx[2] * (xx[5] - xx[6]) * xx[11]);
  xx[13] = (xx[3] + xx[4]) * xx[2] * xx[11];
  xx[7] = (xx[13] == 0.0 && xx[7] == 0.0) ? 0.0 : atan2(xx[13], xx[7]);
  xx[2] = bb[0] ? xx[8] : xx[12] * xx[7];
  xx[3] = bb[0] ? xx[10] : xx[11] * xx[2];
  xx[17] = xx[1];
  xx[18] = xx[1];
  xx[19] = xx[1];
  xx[20] = xx[1];
  xx[21] = xx[3];
  xx[22] = xx[3];
  xx[23] = xx[3];
  xx[1] = xx[17 + (partialType)];
  xx[3] = cos(xx[1]);
  xx[4] = sin(xx[1]);
  xx[1] = sin(xx[16]);
  xx[10] = xx[0];
  xx[11] = xx[2];
  xx[12] = xx[0];
  xx[13] = xx[2];
  xx[14] = xx[0];
  xx[15] = xx[2];
  xx[16] = xx[0];
  xx[0] = xx[10 + (partialType)];
  xx[2] = cos(xx[0]);
  xx[5] = sin(xx[0]);
  xx[0] = xx[3] * xx[5];
  xx[6] = xx[3] * xx[2];
  xx[10] = xx[9] * xx[3];
  xx[11] = - (xx[9] * xx[4]);
  xx[12] = xx[1];
  xx[13] = xx[2] * xx[4] + xx[0] * xx[1];
  xx[14] = xx[6] - xx[1] * xx[5] * xx[4];
  xx[15] = - (xx[9] * xx[5]);
  xx[16] = xx[4] * xx[5] - xx[6] * xx[1];
  xx[17] = xx[0] + xx[2] * xx[1] * xx[4];
  xx[18] = xx[9] * xx[2];
  pm_math_Quaternion_Matrix3x3Ctor_ra(xx + 10, xx + 0);
  state[5] = xx[0];
  state[6] = xx[1];
  state[7] = xx[2];
  state[8] = xx[3];
}

static void projectPartiallyTargetedPos_2_3(const double *origState, int
  partialType, double *state)
{
  boolean_T bb[1];
  double xx[24];
  xx[0] = state[19] * state[21];
  xx[1] = state[18] * state[20];
  xx[2] = 2.0;
  xx[3] = (xx[0] + xx[1]) * xx[2];
  xx[4] = fabs(xx[3]) > 1.0 ? atan2(xx[3], 0.0) : asin(xx[3]);
  xx[5] = origState[19] * origState[21];
  xx[6] = origState[18] * origState[20];
  xx[7] = (xx[5] + xx[6]) * xx[2];
  xx[8] = fabs(xx[7]) > 1.0 ? atan2(xx[7], 0.0) : asin(xx[7]);
  xx[9] = xx[4];
  xx[10] = xx[4];
  xx[11] = xx[8];
  xx[12] = xx[8];
  xx[13] = xx[4];
  xx[14] = xx[4];
  xx[15] = xx[8];
  xx[16] = xx[9 + (partialType)];
  xx[9] = cos(xx[16]);
  xx[10] = 0.99999999999999;
  bb[0] = !(fabs(xx[3]) > xx[10]);
  xx[3] = state[19] * state[20];
  xx[11] = state[18] * state[21];
  xx[12] = state[18] * state[18];
  xx[13] = 1.0;
  xx[14] = (xx[12] + state[19] * state[19]) * xx[2] - xx[13];
  xx[15] = - (xx[2] * (xx[3] - xx[11]));
  xx[14] = (xx[15] == 0.0 && xx[14] == 0.0) ? 0.0 : atan2(xx[15], xx[14]);
  if (xx[4] < 0.0)
    xx[15] = -1.0;
  else if (xx[4] > 0.0)
    xx[15] = +1.0;
  else
    xx[15] = 0.0;
  xx[4] = (xx[12] + state[21] * state[21]) * xx[2] - xx[13];
  xx[17] = - (xx[2] * (state[20] * state[21] - state[18] * state[19]));
  xx[4] = (xx[17] == 0.0 && xx[4] == 0.0) ? 0.0 : atan2(xx[17], xx[4]);
  xx[12] = 0.5;
  xx[17] = - (xx[2] * (xx[0] - xx[1]) * xx[15]);
  xx[18] = (xx[3] + xx[11]) * xx[2] * xx[15];
  xx[17] = (xx[18] == 0.0 && xx[17] == 0.0) ? 0.0 : atan2(xx[18], xx[17]);
  xx[0] = bb[0] ? xx[4] : xx[12] * xx[17];
  xx[1] = bb[0] ? xx[14] : xx[15] * xx[0];
  bb[0] = !(fabs(xx[7]) > xx[10]);
  xx[3] = origState[19] * origState[20];
  xx[4] = origState[18] * origState[21];
  xx[7] = origState[18] * origState[18];
  xx[10] = (xx[7] + origState[19] * origState[19]) * xx[2] - xx[13];
  xx[11] = - (xx[2] * (xx[3] - xx[4]));
  xx[10] = (xx[11] == 0.0 && xx[10] == 0.0) ? 0.0 : atan2(xx[11], xx[10]);
  if (xx[8] < 0.0)
    xx[11] = -1.0;
  else if (xx[8] > 0.0)
    xx[11] = +1.0;
  else
    xx[11] = 0.0;
  xx[8] = (xx[7] + origState[21] * origState[21]) * xx[2] - xx[13];
  xx[14] = - (xx[2] * (origState[20] * origState[21] - origState[18] *
                       origState[19]));
  xx[8] = (xx[14] == 0.0 && xx[8] == 0.0) ? 0.0 : atan2(xx[14], xx[8]);
  xx[7] = - (xx[2] * (xx[5] - xx[6]) * xx[11]);
  xx[13] = (xx[3] + xx[4]) * xx[2] * xx[11];
  xx[7] = (xx[13] == 0.0 && xx[7] == 0.0) ? 0.0 : atan2(xx[13], xx[7]);
  xx[2] = bb[0] ? xx[8] : xx[12] * xx[7];
  xx[3] = bb[0] ? xx[10] : xx[11] * xx[2];
  xx[17] = xx[1];
  xx[18] = xx[1];
  xx[19] = xx[1];
  xx[20] = xx[1];
  xx[21] = xx[3];
  xx[22] = xx[3];
  xx[23] = xx[3];
  xx[1] = xx[17 + (partialType)];
  xx[3] = cos(xx[1]);
  xx[4] = sin(xx[1]);
  xx[1] = sin(xx[16]);
  xx[10] = xx[0];
  xx[11] = xx[2];
  xx[12] = xx[0];
  xx[13] = xx[2];
  xx[14] = xx[0];
  xx[15] = xx[2];
  xx[16] = xx[0];
  xx[0] = xx[10 + (partialType)];
  xx[2] = cos(xx[0]);
  xx[5] = sin(xx[0]);
  xx[0] = xx[3] * xx[5];
  xx[6] = xx[3] * xx[2];
  xx[10] = xx[9] * xx[3];
  xx[11] = - (xx[9] * xx[4]);
  xx[12] = xx[1];
  xx[13] = xx[2] * xx[4] + xx[0] * xx[1];
  xx[14] = xx[6] - xx[1] * xx[5] * xx[4];
  xx[15] = - (xx[9] * xx[5]);
  xx[16] = xx[4] * xx[5] - xx[6] * xx[1];
  xx[17] = xx[0] + xx[2] * xx[1] * xx[4];
  xx[18] = xx[9] * xx[2];
  pm_math_Quaternion_Matrix3x3Ctor_ra(xx + 10, xx + 0);
  state[18] = xx[0];
  state[19] = xx[1];
  state[20] = xx[2];
  state[21] = xx[3];
}

void V1_2P_4d7d5d8d_1_projectPartiallyTargetedPos(const void *mech, size_t
  stageIdx, size_t primIdx, const double *origState, int partialType, double
  *state)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) origState;
  (void) partialType;
  (void) state;
  switch ((stageIdx * 6 + primIdx))
  {
   case 9:
    projectPartiallyTargetedPos_1_3(origState, partialType, state);
    break;

   case 15:
    projectPartiallyTargetedPos_2_3(origState, partialType, state);
    break;
  }
}

void V1_2P_4d7d5d8d_1_propagateMotion(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const double *state, double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[45];
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  xx[0] = 0.0;
  xx[1] = 1.0;
  xx[2] = 0.6546837452583718;
  xx[3] = 0.2671875627615766;
  xx[4] = xx[2];
  xx[5] = xx[2];
  xx[6] = xx[3];
  xx[7] = - xx[3];
  xx[8] = - state[5];
  xx[9] = - state[6];
  xx[10] = - state[7];
  xx[11] = - state[8];
  pm_math_Quaternion_compose_ra(xx + 4, xx + 8, xx + 12);
  xx[2] = 0.02;
  xx[3] = xx[2] * state[7];
  xx[8] = xx[2] * state[6];
  xx[9] = 2.0;
  xx[16] = state[2] + (xx[3] * state[5] + xx[8] * state[8]) * xx[9];
  xx[17] = state[3] - xx[9] * (xx[8] * state[5] - xx[3] * state[8]);
  xx[18] = state[4] - (xx[8] * state[6] + xx[3] * state[7]) * xx[9] + xx[2];
  pm_math_Quaternion_xform_ra(xx + 4, xx + 16, xx + 19);
  xx[3] = 0.44080001915388;
  xx[8] = - 0.5528972265384762;
  xx[22] = xx[3];
  xx[23] = - xx[3];
  xx[24] = xx[8];
  xx[25] = xx[8];
  xx[26] = - state[18];
  xx[27] = - state[19];
  xx[28] = - state[20];
  xx[29] = - state[21];
  pm_math_Quaternion_compose_ra(xx + 22, xx + 26, xx + 30);
  xx[3] = 0.02500000000000001;
  xx[8] = xx[3] * state[20];
  xx[10] = xx[3] * state[19];
  xx[16] = state[15] + (xx[8] * state[18] + xx[10] * state[21]) * xx[9];
  xx[17] = state[16] - xx[9] * (xx[10] * state[18] - xx[8] * state[21]);
  xx[18] = state[17] - (xx[10] * state[19] + xx[8] * state[20]) * xx[9] + xx[3];
  pm_math_Quaternion_xform_ra(xx + 22, xx + 16, xx + 26);
  xx[8] = 0.5937433897368622;
  xx[10] = 0.477332231671813;
  xx[11] = 0.734586783585558 - state[28];
  xx[16] = - xx[1];
  xx[17] = 5.010366908599573e-3 + state[30];
  xx[18] = 0.5;
  xx[29] = - xx[18];
  xx[34] = state[9];
  xx[35] = state[10];
  xx[36] = state[11];
  pm_math_Quaternion_xform_ra(xx + 4, xx + 34, xx + 37);
  pm_math_Quaternion_inverseXform_ra(xx + 12, xx + 37, xx + 4);
  xx[34] = state[22];
  xx[35] = state[23];
  xx[36] = state[24];
  pm_math_Quaternion_xform_ra(xx + 22, xx + 34, xx + 37);
  pm_math_Quaternion_inverseXform_ra(xx + 30, xx + 37, xx + 22);
  xx[7] = - state[29];
  xx[34] = xx[29];
  xx[35] = xx[18];
  xx[36] = xx[29];
  xx[25] = xx[18] * state[29];
  xx[37] = xx[18] * state[31];
  xx[38] = xx[37] + xx[25];
  xx[39] = - xx[25];
  xx[40] = - xx[38];
  xx[41] = - xx[37];
  pm_math_Vector3_cross_ra(xx + 34, xx + 39, xx + 42);
  motionData[0] = xx[0];
  motionData[1] = xx[1];
  motionData[2] = xx[0];
  motionData[3] = xx[0];
  motionData[4] = 0.3254478146933229;
  motionData[5] = 0.4374504150307555;
  motionData[6] = 0.4681639892349409 - state[0];
  motionData[7] = xx[12];
  motionData[8] = xx[13];
  motionData[9] = xx[14];
  motionData[10] = xx[15];
  motionData[11] = xx[19] + 0.3254478146933178;
  motionData[12] = xx[20] + 0.5004504150307565;
  motionData[13] = xx[21] + 0.6031639892349404;
  motionData[14] = xx[30];
  motionData[15] = xx[31];
  motionData[16] = xx[32];
  motionData[17] = xx[33];
  motionData[18] = xx[26] + 0.325447814693318;
  motionData[19] = xx[27] + 0.4104504150307564;
  motionData[20] = xx[28] + 0.6031639892349409;
  motionData[21] = xx[1];
  motionData[22] = xx[0];
  motionData[23] = xx[0];
  motionData[24] = xx[0];
  motionData[25] = xx[8];
  motionData[26] = xx[10];
  motionData[27] = xx[11];
  motionData[28] = xx[16];
  motionData[29] = xx[0];
  motionData[30] = xx[0];
  motionData[31] = xx[0];
  motionData[32] = xx[17];
  motionData[33] = xx[0];
  motionData[34] = xx[0];
  motionData[35] = xx[29];
  motionData[36] = xx[29];
  motionData[37] = xx[18];
  motionData[38] = xx[29];
  motionData[39] = - 0.2373059419521393;
  motionData[40] = 0.140968183358948 - state[32];
  motionData[41] = - 0.01542279435070693;
  motionData[42] = xx[16];
  motionData[43] = xx[0];
  motionData[44] = xx[0];
  motionData[45] = xx[0];
  motionData[46] = xx[17] + xx[8];
  motionData[47] = xx[10];
  motionData[48] = xx[11];
  motionData[49] = xx[18];
  motionData[50] = xx[18];
  motionData[51] = xx[29];
  motionData[52] = xx[18];
  motionData[53] = xx[17] + 0.356437447784723;
  motionData[54] = 0.6183004150307611 - state[32];
  motionData[55] = 0.719163989234851 - state[28];
  motionData[56] = xx[0];
  motionData[57] = xx[0];
  motionData[58] = xx[0];
  motionData[59] = xx[0];
  motionData[60] = xx[0];
  motionData[61] = state[1];
  motionData[62] = state[12];
  motionData[63] = state[13];
  motionData[64] = state[14];
  motionData[65] = xx[4] + xx[2] * state[13];
  motionData[66] = xx[5] - xx[2] * state[12];
  motionData[67] = xx[6];
  motionData[68] = state[25];
  motionData[69] = state[26];
  motionData[70] = state[27];
  motionData[71] = xx[22] + xx[3] * state[26];
  motionData[72] = xx[23] - xx[3] * state[25];
  motionData[73] = xx[24];
  motionData[74] = xx[0];
  motionData[75] = xx[0];
  motionData[76] = xx[0];
  motionData[77] = xx[0];
  motionData[78] = xx[0];
  motionData[79] = xx[7];
  motionData[80] = xx[0];
  motionData[81] = xx[0];
  motionData[82] = xx[0];
  motionData[83] = state[31];
  motionData[84] = xx[0];
  motionData[85] = xx[7];
  motionData[86] = xx[0];
  motionData[87] = xx[0];
  motionData[88] = xx[0];
  motionData[89] = state[31] + xx[9] * (xx[42] - 0.25 * state[29]);
  motionData[90] = (xx[43] - xx[38] * xx[18]) * xx[9];
  motionData[91] = xx[9] * (xx[44] - xx[18] * xx[37]) - state[29] + state[33];
}

size_t V1_2P_4d7d5d8d_1_computeAssemblyError(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, const int *modeVector,
  const double *motionData, double *error)
{
  (void) mech;
  (void)rtdv;
  (void) modeVector;
  (void) motionData;
  (void) error;
  switch (constraintIdx)
  {
  }

  return 0;
}

size_t V1_2P_4d7d5d8d_1_computeAssemblyJacobian(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, boolean_T
  forVelocitySatisfaction, const double *state, const int *modeVector, const
  double *motionData, double *J)
{
  (void) mech;
  (void) rtdv;
  (void) state;
  (void) modeVector;
  (void) forVelocitySatisfaction;
  (void) motionData;
  (void) J;
  switch (constraintIdx)
  {
  }

  return 0;
}

size_t V1_2P_4d7d5d8d_1_computeFullAssemblyJacobian(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const double *state, const int *modeVector,
  const double *motionData, double *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
  (void) modeVector;
  (void) motionData;
  (void) J;
  return 0;
}

boolean_T V1_2P_4d7d5d8d_1_isInKinematicSingularity(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, const int *modeVector,
  const double *motionData)
{
  (void) mech;
  (void) rtdv;
  (void) modeVector;
  (void) motionData;
  switch (constraintIdx)
  {
  }

  return 0;
}

void V1_2P_4d7d5d8d_1_convertStateVector(const void *asmMech, const
  RuntimeDerivedValuesBundle *rtdv, const void *simMech, const double *asmState,
  const int *asmModeVector, const int *simModeVector, double *simState)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) asmMech;
  (void) rtdvd;
  (void) rtdvi;
  (void) simMech;
  (void) asmModeVector;
  (void) simModeVector;
  simState[0] = asmState[0];
  simState[1] = asmState[1];
  simState[2] = asmState[2];
  simState[3] = asmState[3];
  simState[4] = asmState[4];
  simState[5] = asmState[5];
  simState[6] = asmState[6];
  simState[7] = asmState[7];
  simState[8] = asmState[8];
  simState[9] = asmState[9];
  simState[10] = asmState[10];
  simState[11] = asmState[11];
  simState[12] = asmState[12];
  simState[13] = asmState[13];
  simState[14] = asmState[14];
  simState[15] = asmState[15];
  simState[16] = asmState[16];
  simState[17] = asmState[17];
  simState[18] = asmState[18];
  simState[19] = asmState[19];
  simState[20] = asmState[20];
  simState[21] = asmState[21];
  simState[22] = asmState[22];
  simState[23] = asmState[23];
  simState[24] = asmState[24];
  simState[25] = asmState[25];
  simState[26] = asmState[26];
  simState[27] = asmState[27];
  simState[28] = asmState[28];
  simState[29] = asmState[29];
  simState[30] = asmState[30];
  simState[31] = asmState[31];
  simState[32] = asmState[32];
  simState[33] = asmState[33];
}

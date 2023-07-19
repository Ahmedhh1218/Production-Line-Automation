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
#include "V1_2P_4d7d5d8d_1_geometries.h"

PmfMessageId V1_2P_4d7d5d8d_1_checkDynamics(const RuntimeDerivedValuesBundle
  *rtdv, const double *state, const double *input, const double *inputDot, const
  double *inputDdot, const double *discreteState, const int *modeVector, double *
  errorResult, NeuDiagnosticManager *neDiagMgr)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[23];
  (void) rtdvd;
  (void) rtdvi;
  (void) inputDot;
  (void) inputDdot;
  (void) discreteState;
  (void) modeVector;
  (void) neDiagMgr;
  xx[0] = 0.44080001915388;
  xx[1] = - 0.5528972265384762;
  xx[2] = xx[0];
  xx[3] = - xx[0];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = - state[16];
  xx[7] = - state[17];
  xx[8] = - state[18];
  xx[9] = - state[19];
  pm_math_Quaternion_compose_ra(xx + 2, xx + 6, xx + 10);
  xx[0] = 0.03499999999999999;
  xx[1] = xx[0] * xx[12];
  xx[6] = xx[0] * xx[11];
  xx[0] = 2.0;
  xx[7] = 0.02500000000000001;
  xx[8] = xx[7] * state[18];
  xx[9] = xx[7] * state[17];
  xx[14] = state[13] + (xx[8] * state[16] + xx[9] * state[19]) * xx[0];
  xx[15] = state[14] - xx[0] * (xx[9] * state[16] - xx[8] * state[19]);
  xx[16] = state[15] - (xx[9] * state[17] + xx[8] * state[18]) * xx[0] + xx[7];
  pm_math_Quaternion_xform_ra(xx + 2, xx + 14, xx + 7);
  xx[2] = input[1] + 0.3614478146933225;
  xx[3] = (xx[10] * xx[1] + xx[13] * xx[6]) * xx[0] + xx[7] - xx[2] +
    0.360447814693318;
  xx[4] = 0.6183004150307611 - input[0];
  xx[5] = xx[0] * (xx[13] * xx[1] - xx[10] * xx[6]) + xx[8] - xx[4] +
    0.5564504150307564;
  xx[7] = 0.719163989234851 - input[2];
  xx[8] = xx[9] - (xx[11] * xx[6] + xx[12] * xx[1]) * xx[0] - xx[7] +
    0.6881639892349409;
  xx[1] = sqrt(xx[3] * xx[3] + xx[5] * xx[5] + xx[8] * xx[8]);
  if (xx[1] == 0.0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:ForceUndefined",
      "'V1_2P/Suction/Internal Force' force is undefined since base and follower origins are coincident.",
      neDiagMgr);
  }

  xx[1] = input[964] / xx[1];
  xx[3] = 0.6546837452583718;
  xx[5] = 0.2671875627615766;
  xx[8] = xx[3];
  xx[9] = xx[3];
  xx[10] = xx[5];
  xx[11] = - xx[5];
  xx[12] = - state[3];
  xx[13] = - state[4];
  xx[14] = - state[5];
  xx[15] = - state[6];
  pm_math_Quaternion_compose_ra(xx + 8, xx + 12, xx + 16);
  xx[3] = 9.999999999999998e-3;
  xx[5] = xx[3] * xx[18];
  xx[6] = xx[3] * xx[17];
  xx[3] = 0.02;
  xx[12] = xx[3] * state[5];
  xx[13] = xx[3] * state[4];
  xx[20] = state[0] + (xx[12] * state[3] + xx[13] * state[6]) * xx[0];
  xx[21] = state[1] - xx[0] * (xx[13] * state[3] - xx[12] * state[6]);
  xx[22] = state[2] - (xx[13] * state[4] + xx[12] * state[5]) * xx[0] + xx[3];
  pm_math_Quaternion_xform_ra(xx + 8, xx + 20, xx + 12);
  xx[3] = (xx[16] * xx[5] + xx[19] * xx[6]) * xx[0] + xx[12] - xx[2] +
    0.3604478146933178;
  xx[2] = xx[0] * (xx[19] * xx[5] - xx[16] * xx[6]) + xx[13] - xx[4] +
    0.6464504150307565;
  xx[4] = xx[14] - (xx[17] * xx[6] + xx[18] * xx[5]) * xx[0] - xx[7] +
    0.6631639892349405;
  xx[0] = sqrt(xx[3] * xx[3] + xx[2] * xx[2] + xx[4] * xx[4]);
  if (xx[0] == 0.0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:ForceUndefined",
      "'V1_2P/Suction/Internal Force1' force is undefined since base and follower origins are coincident.",
      neDiagMgr);
  }

  xx[0] = input[965] / xx[0];
  errorResult[0] = xx[1] + xx[0];
  return NULL;
}

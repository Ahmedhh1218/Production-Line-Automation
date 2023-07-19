/* Simscape target specific file.
 * This file is generated for the Simscape network associated with the solver block 'V1_2P/Body Configuration/Solver Configuration'.
 */

#ifdef MATLAB_MEX_FILE
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
#endif

#include "nesl_rtw.h"
#include "V1_2P_4d7d5d8d_1.h"
#include "V1_2P_4d7d5d8d_1_gateway.h"

void V1_2P_4d7d5d8d_1_gateway(void)
{
  NeModelParameters modelparams = { (NeSolverType) 0, 0.001, 1, 0, 0.001, 0, 0,
    0, 0, (SscLoggingSetting) 0, 590510347, 0, };

  NeSolverParameters solverparams = { 0, 0, 1, 0, 0, 0.001, 1e-06, 1e-09, 0, 0,
    100, 0, 1, 0, 1e-09, 0, (NeLocalSolverChoice) 0, 0.001, 0, 3, 2, 0, 2,
    (NeLinearAlgebraChoice) 0, (NeEquationFormulationChoice) 0, 1024, 1, 0.001,
    (NePartitionStorageMethod) 0, 1024, (NePartitionMethod) 0, };

  const NeOutputParameters* outputparameters = NULL;
  NeDae* dae;
  size_t numOutputs = 0;
  int* rtpDaes = NULL;
  int rtwLogDaes[1] = { 0 };

  {
    static const NeOutputParameters outputparameters_init[] = { { 0, 0, }, };

    outputparameters = outputparameters_init;
    numOutputs = sizeof(outputparameters_init)/sizeof(outputparameters_init[0]);
  }

  V1_2P_4d7d5d8d_1_dae(&dae,
                       &modelparams,
                       &solverparams);
  nesl_register_simulator_group(
    "V1_2P/Body Configuration/Solver Configuration_1",
    1,
    &dae,
    &solverparams,
    &modelparams,
    numOutputs,
    outputparameters,
    0,
    rtpDaes,
    1,
    rtwLogDaes);
}

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

PmfMessageId V1_2P_4d7d5d8d_1_compDerivs(const RuntimeDerivedValuesBundle *rtdv,
  const int *eqnEnableFlags, const double *state, const int *modeVector, const
  double *input, const double *inputDot, const double *inputDdot, const double
  *discreteState, double *deriv, double *errorResult, NeuDiagnosticManager
  *neDiagMgr)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  int ii[1];
  double xx[289];
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) modeVector;
  (void) inputDot;
  (void) inputDdot;
  (void) discreteState;
  (void) neDiagMgr;
  xx[0] = state[3];
  xx[1] = state[4];
  xx[2] = state[5];
  xx[3] = state[6];
  xx[4] = state[10];
  xx[5] = state[11];
  xx[6] = state[12];
  pm_math_Quaternion_compDeriv_ra(xx + 0, xx + 4, xx + 7);
  xx[0] = 0.7144432252221142;
  xx[1] = 2.0;
  xx[2] = 0.6546837452583718;
  xx[3] = 0.2671875627615766;
  xx[11] = xx[2];
  xx[12] = xx[2];
  xx[13] = xx[3];
  xx[14] = - xx[3];
  xx[15] = - state[3];
  xx[16] = - state[4];
  xx[17] = - state[5];
  xx[18] = - state[6];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 15, xx + 19);
  xx[2] = 0.699693417100821;
  xx[3] = xx[2] * xx[21];
  xx[15] = - xx[3];
  xx[16] = xx[0] * xx[22] + xx[2] * xx[20];
  xx[17] = xx[0] * xx[21];
  xx[23] = xx[15];
  xx[24] = xx[16];
  xx[25] = - xx[17];
  pm_math_Vector3_cross_ra(xx + 20, xx + 23, xx + 26);
  xx[18] = xx[19] * xx[21];
  xx[23] = xx[0] + xx[1] * (xx[26] + xx[2] * xx[18]);
  xx[0] = xx[1] * (xx[27] - xx[19] * xx[16]);
  xx[16] = (xx[19] * xx[17] + xx[28]) * xx[1] - xx[2];
  xx[24] = xx[23];
  xx[25] = xx[0];
  xx[26] = xx[16];
  xx[17] = 0.07853981633974483;
  xx[27] = xx[23] * xx[17];
  xx[28] = xx[17] * xx[0];
  xx[29] = xx[16] * xx[17];
  xx[16] = 0.7144432252221145;
  xx[30] = xx[2] * xx[22] - xx[16] * xx[20];
  xx[31] = xx[16] * xx[21];
  xx[32] = xx[30];
  xx[33] = xx[15];
  pm_math_Vector3_cross_ra(xx + 20, xx + 31, xx + 34);
  xx[15] = xx[2] + xx[1] * (xx[34] - xx[16] * xx[18]);
  xx[2] = xx[15] * xx[17];
  xx[18] = xx[1] * (xx[35] - xx[19] * xx[30]);
  xx[30] = xx[17] * xx[18];
  xx[31] = xx[16] + (xx[19] * xx[3] + xx[36]) * xx[1];
  xx[3] = xx[31] * xx[17];
  xx[32] = xx[2];
  xx[33] = xx[30];
  xx[34] = xx[3];
  xx[16] = pm_math_Vector3_dot_ra(xx + 24, xx + 32);
  xx[35] = (xx[19] * xx[22] + xx[20] * xx[21]) * xx[1];
  xx[36] = - (xx[17] * xx[35]);
  xx[37] = 1.0;
  xx[38] = (xx[22] * xx[22] + xx[20] * xx[20]) * xx[1] - xx[37];
  xx[39] = xx[38] * xx[17];
  xx[40] = xx[1] * (xx[19] * xx[20] - xx[21] * xx[22]);
  xx[41] = xx[17] * xx[40];
  xx[42] = xx[36];
  xx[43] = xx[39];
  xx[44] = xx[41];
  xx[45] = pm_math_Vector3_dot_ra(xx + 24, xx + 42);
  xx[46] = 1.570796326794897e-3;
  xx[47] = - (xx[46] * xx[0]);
  xx[0] = xx[23] * xx[46];
  xx[23] = 0.0;
  xx[48] = xx[15];
  xx[49] = xx[18];
  xx[50] = xx[31];
  xx[31] = pm_math_Vector3_dot_ra(xx + 48, xx + 42);
  xx[51] = - (xx[46] * xx[18]);
  xx[18] = xx[15] * xx[46];
  xx[52] = - xx[35];
  xx[53] = xx[38];
  xx[54] = xx[40];
  xx[15] = - (xx[38] * xx[46]);
  xx[38] = - (xx[46] * xx[35]);
  xx[35] = 5.415974835094905e-5;
  xx[40] = 2.454369260617026e-5;
  xx[55] = pm_math_Vector3_dot_ra(xx + 24, xx + 27);
  xx[56] = xx[16];
  xx[57] = xx[45];
  xx[58] = xx[47];
  xx[59] = xx[0];
  xx[60] = xx[23];
  xx[61] = xx[16];
  xx[62] = pm_math_Vector3_dot_ra(xx + 48, xx + 32);
  xx[63] = xx[31];
  xx[64] = xx[51];
  xx[65] = xx[18];
  xx[66] = xx[23];
  xx[67] = xx[45];
  xx[68] = xx[31];
  xx[69] = pm_math_Vector3_dot_ra(xx + 52, xx + 42);
  xx[70] = xx[15];
  xx[71] = xx[38];
  xx[72] = xx[23];
  xx[73] = xx[47];
  xx[74] = xx[51];
  xx[75] = xx[15];
  xx[76] = xx[35];
  xx[77] = xx[23];
  xx[78] = xx[23];
  xx[79] = xx[0];
  xx[80] = xx[18];
  xx[81] = xx[38];
  xx[82] = xx[23];
  xx[83] = xx[35];
  xx[84] = xx[23];
  xx[85] = xx[23];
  xx[86] = xx[23];
  xx[87] = xx[23];
  xx[88] = xx[23];
  xx[89] = xx[23];
  xx[90] = xx[40];
  ii[0] = factorSymmetricPosDef(xx + 55, 6, xx + 91);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'V1_2P/Body Configuration/SixDOF1' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[31] = state[7];
  xx[32] = state[8];
  xx[33] = state[9];
  pm_math_Quaternion_xform_ra(xx + 11, xx + 31, xx + 42);
  pm_math_Quaternion_inverseXform_ra(xx + 19, xx + 42, xx + 31);
  xx[0] = 0.02;
  xx[42] = xx[31] + xx[0] * state[11];
  xx[43] = xx[32] - xx[0] * state[10];
  xx[44] = xx[33];
  pm_math_Vector3_cross_ra(xx + 4, xx + 42, xx + 91);
  xx[42] = - xx[31];
  xx[43] = - xx[32];
  xx[44] = - xx[33];
  pm_math_Vector3_cross_ra(xx + 4, xx + 42, xx + 31);
  xx[15] = 0.9238795325112867;
  xx[16] = 0.3826834323650898;
  xx[18] = xx[16] * input[59];
  xx[34] = xx[16] * input[58];
  xx[35] = input[58] - (xx[15] * xx[18] + xx[16] * xx[34]) * xx[1];
  xx[38] = xx[16] * input[71];
  xx[42] = xx[16] * input[70];
  xx[43] = input[70] - (xx[15] * xx[38] + xx[16] * xx[42]) * xx[1];
  xx[44] = xx[16] * input[83];
  xx[45] = xx[16] * input[82];
  xx[47] = input[82] - (xx[15] * xx[44] + xx[16] * xx[45]) * xx[1];
  xx[51] = xx[16] * input[95];
  xx[94] = xx[16] * input[94];
  xx[95] = input[94] - (xx[15] * xx[51] + xx[16] * xx[94]) * xx[1];
  xx[96] = xx[16] * input[155];
  xx[97] = xx[16] * input[154];
  xx[98] = input[154] - (xx[15] * xx[96] + xx[16] * xx[97]) * xx[1];
  xx[99] = xx[16] * input[167];
  xx[100] = xx[16] * input[166];
  xx[101] = input[166] - (xx[15] * xx[99] + xx[16] * xx[100]) * xx[1];
  xx[102] = xx[16] * input[179];
  xx[103] = xx[16] * input[178];
  xx[104] = input[178] - (xx[15] * xx[102] + xx[16] * xx[103]) * xx[1];
  xx[105] = xx[16] * input[191];
  xx[106] = xx[16] * input[190];
  xx[107] = input[190] - (xx[15] * xx[105] + xx[16] * xx[106]) * xx[1];
  xx[108] = xx[16] * input[251];
  xx[109] = xx[16] * input[250];
  xx[110] = input[250] - (xx[15] * xx[108] + xx[16] * xx[109]) * xx[1];
  xx[111] = xx[16] * input[263];
  xx[112] = xx[16] * input[262];
  xx[113] = input[262] - (xx[15] * xx[111] + xx[16] * xx[112]) * xx[1];
  xx[114] = xx[16] * input[275];
  xx[115] = xx[16] * input[274];
  xx[116] = input[274] - (xx[15] * xx[114] + xx[16] * xx[115]) * xx[1];
  xx[117] = xx[16] * input[287];
  xx[118] = xx[16] * input[286];
  xx[119] = input[286] - (xx[15] * xx[117] + xx[16] * xx[118]) * xx[1];
  xx[120] = xx[16] * input[347];
  xx[121] = xx[16] * input[346];
  xx[122] = input[346] - (xx[15] * xx[120] + xx[16] * xx[121]) * xx[1];
  xx[123] = xx[16] * input[359];
  xx[124] = xx[16] * input[358];
  xx[125] = input[358] - (xx[15] * xx[123] + xx[16] * xx[124]) * xx[1];
  xx[126] = xx[16] * input[371];
  xx[127] = xx[16] * input[370];
  xx[128] = input[370] - (xx[15] * xx[126] + xx[16] * xx[127]) * xx[1];
  xx[129] = xx[16] * input[383];
  xx[130] = xx[16] * input[382];
  xx[131] = input[382] - (xx[15] * xx[129] + xx[16] * xx[130]) * xx[1];
  xx[132] = xx[16] * input[442];
  xx[133] = xx[16] * input[443];
  xx[134] = input[442] - (xx[16] * xx[132] - xx[15] * xx[133]) * xx[1];
  xx[135] = xx[16] * input[454];
  xx[136] = xx[16] * input[455];
  xx[137] = input[454] - (xx[16] * xx[135] - xx[15] * xx[136]) * xx[1];
  xx[138] = xx[16] * input[466];
  xx[139] = xx[16] * input[467];
  xx[140] = input[466] - (xx[16] * xx[138] - xx[15] * xx[139]) * xx[1];
  xx[141] = xx[16] * input[478];
  xx[142] = xx[16] * input[479];
  xx[143] = input[478] - (xx[16] * xx[141] - xx[15] * xx[142]) * xx[1];
  xx[144] = xx[16] * input[490];
  xx[145] = xx[16] * input[491];
  xx[146] = input[490] - (xx[16] * xx[144] - xx[15] * xx[145]) * xx[1];
  xx[147] = xx[16] * input[502];
  xx[148] = xx[16] * input[503];
  xx[149] = input[502] - (xx[16] * xx[147] - xx[15] * xx[148]) * xx[1];
  xx[150] = xx[16] * input[514];
  xx[151] = xx[16] * input[515];
  xx[152] = input[514] - (xx[16] * xx[150] - xx[15] * xx[151]) * xx[1];
  xx[153] = xx[16] * input[526];
  xx[154] = xx[16] * input[527];
  xx[155] = input[526] - (xx[16] * xx[153] - xx[15] * xx[154]) * xx[1];
  xx[156] = xx[16] * input[538];
  xx[157] = xx[16] * input[539];
  xx[158] = input[538] - (xx[16] * xx[156] - xx[15] * xx[157]) * xx[1];
  xx[159] = xx[16] * input[550];
  xx[160] = xx[16] * input[551];
  xx[161] = input[550] - (xx[16] * xx[159] - xx[15] * xx[160]) * xx[1];
  xx[162] = xx[16] * input[562];
  xx[163] = xx[16] * input[563];
  xx[164] = input[562] - (xx[16] * xx[162] - xx[15] * xx[163]) * xx[1];
  xx[165] = xx[16] * input[574];
  xx[166] = xx[16] * input[575];
  xx[167] = input[574] - (xx[16] * xx[165] - xx[15] * xx[166]) * xx[1];
  xx[168] = xx[16] * input[586];
  xx[169] = xx[16] * input[587];
  xx[170] = input[586] - (xx[16] * xx[168] - xx[15] * xx[169]) * xx[1];
  xx[171] = xx[16] * input[598];
  xx[172] = xx[16] * input[599];
  xx[173] = input[598] - (xx[16] * xx[171] - xx[15] * xx[172]) * xx[1];
  xx[174] = xx[16] * input[610];
  xx[175] = xx[16] * input[611];
  xx[176] = input[610] - (xx[16] * xx[174] - xx[15] * xx[175]) * xx[1];
  xx[177] = xx[16] * input[622];
  xx[178] = xx[16] * input[623];
  xx[179] = input[622] - (xx[16] * xx[177] - xx[15] * xx[178]) * xx[1];
  xx[180] = xx[16] * input[634];
  xx[181] = xx[16] * input[635];
  xx[182] = input[634] - (xx[16] * xx[180] - xx[15] * xx[181]) * xx[1];
  xx[183] = xx[16] * input[646];
  xx[184] = xx[16] * input[647];
  xx[185] = input[646] - (xx[16] * xx[183] - xx[15] * xx[184]) * xx[1];
  xx[186] = xx[16] * input[658];
  xx[187] = xx[16] * input[659];
  xx[188] = input[658] - (xx[16] * xx[186] - xx[15] * xx[187]) * xx[1];
  xx[189] = xx[16] * input[670];
  xx[190] = xx[16] * input[671];
  xx[191] = input[670] - (xx[16] * xx[189] - xx[15] * xx[190]) * xx[1];
  xx[192] = xx[16] * input[682];
  xx[193] = xx[16] * input[683];
  xx[194] = input[682] - (xx[16] * xx[192] - xx[15] * xx[193]) * xx[1];
  xx[195] = xx[16] * input[694];
  xx[196] = xx[16] * input[695];
  xx[197] = input[694] - (xx[16] * xx[195] - xx[15] * xx[196]) * xx[1];
  xx[198] = xx[16] * input[706];
  xx[199] = xx[16] * input[707];
  xx[200] = input[706] - (xx[16] * xx[198] - xx[15] * xx[199]) * xx[1];
  xx[201] = xx[16] * input[718];
  xx[202] = xx[16] * input[719];
  xx[203] = input[718] - (xx[16] * xx[201] - xx[15] * xx[202]) * xx[1];
  xx[204] = xx[16] * input[730];
  xx[205] = xx[16] * input[731];
  xx[206] = input[730] - (xx[16] * xx[204] - xx[15] * xx[205]) * xx[1];
  xx[207] = xx[16] * input[742];
  xx[208] = xx[16] * input[743];
  xx[209] = input[742] - (xx[16] * xx[207] - xx[15] * xx[208]) * xx[1];
  xx[210] = xx[16] * input[754];
  xx[211] = xx[16] * input[755];
  xx[212] = input[754] - (xx[16] * xx[210] - xx[15] * xx[211]) * xx[1];
  xx[213] = xx[16] * input[766];
  xx[214] = xx[16] * input[767];
  xx[215] = input[766] - (xx[16] * xx[213] - xx[15] * xx[214]) * xx[1];
  xx[216] = xx[16] * input[778];
  xx[217] = xx[16] * input[779];
  xx[218] = input[778] - (xx[16] * xx[216] - xx[15] * xx[217]) * xx[1];
  xx[219] = xx[16] * input[790];
  xx[220] = xx[16] * input[791];
  xx[221] = input[790] - (xx[16] * xx[219] - xx[15] * xx[220]) * xx[1];
  xx[222] = xx[16] * input[802];
  xx[223] = xx[16] * input[803];
  xx[224] = input[802] - (xx[16] * xx[222] - xx[15] * xx[223]) * xx[1];
  xx[225] = xx[16] * input[814];
  xx[226] = xx[16] * input[815];
  xx[227] = input[814] - (xx[16] * xx[225] - xx[15] * xx[226]) * xx[1];
  xx[228] = 9.999999999999998e-3;
  xx[229] = xx[228] * xx[21];
  xx[230] = xx[228] * xx[20];
  xx[231] = xx[0] * state[5];
  xx[232] = xx[0] * state[4];
  xx[233] = state[0] + (xx[231] * state[3] + xx[232] * state[6]) * xx[1];
  xx[234] = state[1] - xx[1] * (xx[232] * state[3] - xx[231] * state[6]);
  xx[235] = state[2] - (xx[232] * state[4] + xx[231] * state[5]) * xx[1] + xx[0];
  pm_math_Quaternion_xform_ra(xx + 11, xx + 233, xx + 236);
  xx[11] = input[1] + 0.3614478146933225;
  xx[12] = (xx[19] * xx[229] + xx[22] * xx[230]) * xx[1] + xx[236] - xx[11] +
    0.3604478146933178;
  xx[13] = 0.6183004150307611 - input[0];
  xx[14] = xx[1] * (xx[22] * xx[229] - xx[19] * xx[230]) + xx[237] - xx[13] +
    0.6464504150307565;
  xx[231] = 0.719163989234851 - input[2];
  xx[232] = xx[238] - (xx[20] * xx[230] + xx[21] * xx[229]) * xx[1] - xx[231] +
    0.6631639892349405;
  xx[229] = sqrt(xx[12] * xx[12] + xx[14] * xx[14] + xx[232] * xx[232]);
  if (xx[229] == 0.0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:ForceUndefined",
      "'V1_2P/Suction/Internal Force1' force is undefined since base and follower origins are coincident.",
      neDiagMgr);
  }

  xx[229] = input[965] / xx[229];
  xx[233] = xx[229] * xx[12];
  xx[234] = xx[229] * xx[14];
  xx[235] = xx[229] * xx[232];
  pm_math_Quaternion_inverseXform_ra(xx + 19, xx + 233, xx + 236);
  xx[12] = (xx[91] + xx[31]) * xx[17] - (xx[35] + xx[43] + xx[47] + xx[95] + xx
    [98] + xx[101] + xx[104] + xx[107] + xx[110] + xx[113] + xx[116] + xx[119] +
    xx[122] + xx[125] + xx[128] + xx[131] + xx[134] + xx[137] + xx[140] + xx[143]
    + xx[146] + xx[149] + xx[152] + xx[155] + xx[158] + xx[161] + xx[164] + xx
    [167] + xx[170] + xx[173] + xx[176] + xx[179] + xx[182] + xx[185] + xx[188]
    + xx[191] + xx[194] + xx[197] + xx[200] + xx[203] + xx[206] + xx[209] + xx
    [212] + xx[215] + xx[218] + xx[221] + xx[224] + xx[227] + input[838] +
    input[862] + input[922] + input[934] + input[946] + input[958] + xx[236]);
  xx[14] = input[59] + xx[1] * (xx[15] * xx[34] - xx[16] * xx[18]);
  xx[18] = input[71] + xx[1] * (xx[15] * xx[42] - xx[16] * xx[38]);
  xx[34] = input[83] + xx[1] * (xx[15] * xx[45] - xx[16] * xx[44]);
  xx[38] = input[95] + xx[1] * (xx[15] * xx[94] - xx[16] * xx[51]);
  xx[42] = input[155] + xx[1] * (xx[15] * xx[97] - xx[16] * xx[96]);
  xx[44] = input[167] + xx[1] * (xx[15] * xx[100] - xx[16] * xx[99]);
  xx[45] = input[179] + xx[1] * (xx[15] * xx[103] - xx[16] * xx[102]);
  xx[51] = input[191] + xx[1] * (xx[15] * xx[106] - xx[16] * xx[105]);
  xx[94] = input[251] + xx[1] * (xx[15] * xx[109] - xx[16] * xx[108]);
  xx[96] = input[263] + xx[1] * (xx[15] * xx[112] - xx[16] * xx[111]);
  xx[97] = input[275] + xx[1] * (xx[15] * xx[115] - xx[16] * xx[114]);
  xx[99] = input[287] + xx[1] * (xx[15] * xx[118] - xx[16] * xx[117]);
  xx[100] = input[347] + xx[1] * (xx[15] * xx[121] - xx[16] * xx[120]);
  xx[102] = input[359] + xx[1] * (xx[15] * xx[124] - xx[16] * xx[123]);
  xx[103] = input[371] + xx[1] * (xx[15] * xx[127] - xx[16] * xx[126]);
  xx[105] = input[383] + xx[1] * (xx[15] * xx[130] - xx[16] * xx[129]);
  xx[106] = input[443] - xx[1] * (xx[15] * xx[132] + xx[16] * xx[133]);
  xx[108] = input[455] - xx[1] * (xx[15] * xx[135] + xx[16] * xx[136]);
  xx[109] = input[467] - xx[1] * (xx[15] * xx[138] + xx[16] * xx[139]);
  xx[111] = input[479] - xx[1] * (xx[15] * xx[141] + xx[16] * xx[142]);
  xx[112] = input[491] - xx[1] * (xx[15] * xx[144] + xx[16] * xx[145]);
  xx[114] = input[503] - xx[1] * (xx[15] * xx[147] + xx[16] * xx[148]);
  xx[115] = input[515] - xx[1] * (xx[15] * xx[150] + xx[16] * xx[151]);
  xx[117] = input[527] - xx[1] * (xx[15] * xx[153] + xx[16] * xx[154]);
  xx[118] = input[539] - xx[1] * (xx[15] * xx[156] + xx[16] * xx[157]);
  xx[120] = input[551] - xx[1] * (xx[15] * xx[159] + xx[16] * xx[160]);
  xx[121] = input[563] - xx[1] * (xx[15] * xx[162] + xx[16] * xx[163]);
  xx[123] = input[575] - xx[1] * (xx[15] * xx[165] + xx[16] * xx[166]);
  xx[124] = input[587] - xx[1] * (xx[15] * xx[168] + xx[16] * xx[169]);
  xx[126] = input[599] - xx[1] * (xx[15] * xx[171] + xx[16] * xx[172]);
  xx[127] = input[611] - xx[1] * (xx[15] * xx[174] + xx[16] * xx[175]);
  xx[129] = input[623] - xx[1] * (xx[15] * xx[177] + xx[16] * xx[178]);
  xx[130] = input[635] - xx[1] * (xx[15] * xx[180] + xx[16] * xx[181]);
  xx[132] = input[647] - xx[1] * (xx[15] * xx[183] + xx[16] * xx[184]);
  xx[133] = input[659] - xx[1] * (xx[15] * xx[186] + xx[16] * xx[187]);
  xx[135] = input[671] - xx[1] * (xx[15] * xx[189] + xx[16] * xx[190]);
  xx[136] = input[683] - xx[1] * (xx[15] * xx[192] + xx[16] * xx[193]);
  xx[138] = input[695] - xx[1] * (xx[15] * xx[195] + xx[16] * xx[196]);
  xx[139] = input[707] - xx[1] * (xx[15] * xx[198] + xx[16] * xx[199]);
  xx[141] = input[719] - xx[1] * (xx[15] * xx[201] + xx[16] * xx[202]);
  xx[142] = input[731] - xx[1] * (xx[15] * xx[204] + xx[16] * xx[205]);
  xx[144] = input[743] - xx[1] * (xx[15] * xx[207] + xx[16] * xx[208]);
  xx[145] = input[755] - xx[1] * (xx[15] * xx[210] + xx[16] * xx[211]);
  xx[147] = input[767] - xx[1] * (xx[15] * xx[213] + xx[16] * xx[214]);
  xx[148] = input[779] - xx[1] * (xx[15] * xx[216] + xx[16] * xx[217]);
  xx[150] = input[791] - xx[1] * (xx[15] * xx[219] + xx[16] * xx[220]);
  xx[151] = input[803] - xx[1] * (xx[15] * xx[222] + xx[16] * xx[223]);
  xx[153] = input[815] - xx[1] * (xx[15] * xx[225] + xx[16] * xx[226]);
  xx[154] = (xx[92] + xx[32]) * xx[17] - (xx[14] + xx[18] + xx[34] + xx[38] +
    xx[42] + xx[44] + xx[45] + xx[51] + xx[94] + xx[96] + xx[97] + xx[99] + xx
    [100] + xx[102] + xx[103] + xx[105] + xx[106] + xx[108] + xx[109] + xx[111]
    + xx[112] + xx[114] + xx[115] + xx[117] + xx[118] + xx[120] + xx[121] + xx
    [123] + xx[124] + xx[126] + xx[127] + xx[129] + xx[130] + xx[132] + xx[133]
    + xx[135] + xx[136] + xx[138] + xx[139] + xx[141] + xx[142] + xx[144] + xx
    [145] + xx[147] + xx[148] + xx[150] + xx[151] + xx[153] + input[839] +
    input[863] + input[923] + input[935] + input[947] + input[959] + xx[237]);
  xx[232] = xx[12];
  xx[233] = xx[154];
  xx[234] = (xx[93] + xx[33]) * xx[17] - (input[60] + input[72] + input[84] +
    input[96] + input[156] + input[168] + input[180] + input[192] + input[252] +
    input[264] + input[276] + input[288] + input[348] + input[360] + input[372]
    + input[384] + input[444] + input[456] + input[468] + input[480] + input[492]
    + input[504] + input[516] + input[528] + input[540] + input[552] + input[564]
    + input[576] + input[588] + input[600] + input[612] + input[624] + input[636]
    + input[648] + input[660] + input[672] + input[684] + input[696] + input[708]
    + input[720] + input[732] + input[744] + input[756] + input[768] + input[780]
    + input[792] + input[804] + input[816] + input[840] + input[864] + input[924]
    + input[936] + input[948] + input[960] + xx[238]);
  xx[17] = 2.274382181505112e-5;
  xx[31] = 2.274382181505111e-5;
  xx[91] = xx[17] * state[10];
  xx[92] = xx[31] * state[11];
  xx[93] = xx[40] * state[12];
  pm_math_Vector3_cross_ra(xx + 4, xx + 91, xx + 238);
  xx[4] = 0.02121320343559643;
  xx[5] = xx[16] * input[62];
  xx[6] = xx[16] * input[61];
  xx[32] = xx[16] * input[74];
  xx[33] = xx[16] * input[73];
  xx[91] = xx[16] * input[86];
  xx[92] = xx[16] * input[85];
  xx[93] = xx[16] * input[98];
  xx[156] = xx[16] * input[97];
  xx[157] = xx[16] * input[158];
  xx[159] = xx[16] * input[157];
  xx[160] = xx[16] * input[170];
  xx[162] = xx[16] * input[169];
  xx[163] = xx[16] * input[182];
  xx[165] = xx[16] * input[181];
  xx[166] = xx[16] * input[194];
  xx[168] = xx[16] * input[193];
  xx[169] = xx[16] * input[254];
  xx[171] = xx[16] * input[253];
  xx[172] = xx[16] * input[266];
  xx[174] = xx[16] * input[265];
  xx[175] = xx[16] * input[278];
  xx[177] = xx[16] * input[277];
  xx[178] = xx[16] * input[290];
  xx[180] = xx[16] * input[289];
  xx[181] = xx[16] * input[350];
  xx[183] = xx[16] * input[349];
  xx[184] = xx[16] * input[362];
  xx[186] = xx[16] * input[361];
  xx[187] = xx[16] * input[374];
  xx[189] = xx[16] * input[373];
  xx[190] = xx[16] * input[386];
  xx[192] = xx[16] * input[385];
  xx[193] = xx[16] * input[445];
  xx[195] = xx[16] * input[446];
  xx[196] = 0.0107;
  xx[198] = xx[16] * input[457];
  xx[199] = xx[16] * input[458];
  xx[201] = xx[16] * input[469];
  xx[202] = xx[16] * input[470];
  xx[204] = xx[16] * input[481];
  xx[205] = xx[16] * input[482];
  xx[207] = xx[16] * input[493];
  xx[208] = xx[16] * input[494];
  xx[210] = xx[16] * input[505];
  xx[211] = xx[16] * input[506];
  xx[213] = xx[16] * input[517];
  xx[214] = xx[16] * input[518];
  xx[216] = xx[16] * input[529];
  xx[217] = xx[16] * input[530];
  xx[219] = xx[16] * input[541];
  xx[220] = xx[16] * input[542];
  xx[222] = xx[16] * input[553];
  xx[223] = xx[16] * input[554];
  xx[225] = xx[16] * input[565];
  xx[226] = xx[16] * input[566];
  xx[230] = xx[16] * input[577];
  xx[235] = xx[16] * input[578];
  xx[241] = xx[16] * input[589];
  xx[242] = xx[16] * input[590];
  xx[243] = xx[16] * input[601];
  xx[244] = xx[16] * input[602];
  xx[245] = xx[16] * input[613];
  xx[246] = xx[16] * input[614];
  xx[247] = xx[16] * input[625];
  xx[248] = xx[16] * input[626];
  xx[249] = xx[16] * input[637];
  xx[250] = xx[16] * input[638];
  xx[251] = xx[16] * input[649];
  xx[252] = xx[16] * input[650];
  xx[253] = xx[16] * input[661];
  xx[254] = xx[16] * input[662];
  xx[255] = xx[16] * input[673];
  xx[256] = xx[16] * input[674];
  xx[257] = xx[16] * input[685];
  xx[258] = xx[16] * input[686];
  xx[259] = xx[16] * input[697];
  xx[260] = xx[16] * input[698];
  xx[261] = xx[16] * input[709];
  xx[262] = xx[16] * input[710];
  xx[263] = xx[16] * input[721];
  xx[264] = xx[16] * input[722];
  xx[265] = xx[16] * input[733];
  xx[266] = xx[16] * input[734];
  xx[267] = xx[16] * input[745];
  xx[268] = xx[16] * input[746];
  xx[269] = xx[16] * input[757];
  xx[270] = xx[16] * input[758];
  xx[271] = xx[16] * input[769];
  xx[272] = xx[16] * input[770];
  xx[273] = xx[16] * input[781];
  xx[274] = xx[16] * input[782];
  xx[275] = 9.299999999999999e-3;
  xx[276] = xx[16] * input[793];
  xx[277] = xx[16] * input[794];
  xx[278] = xx[16] * input[805];
  xx[279] = xx[16] * input[806];
  xx[280] = xx[16] * input[817];
  xx[281] = xx[16] * input[818];
  xx[282] = 0.03;
  xx[283] = - pm_math_Vector3_dot_ra(xx + 24, xx + 232);
  xx[284] = - pm_math_Vector3_dot_ra(xx + 48, xx + 232);
  xx[285] = - pm_math_Vector3_dot_ra(xx + 52, xx + 232);
  xx[286] = - (xx[238] - (xx[4] * input[60] + xx[14] * xx[0] + input[61] - (xx
    [15] * xx[5] + xx[16] * xx[6]) * xx[1]) - (input[73] - (xx[15] * xx[32] +
    xx[16] * xx[33]) * xx[1] + xx[18] * xx[0]) - (xx[34] * xx[0] - xx[4] *
    input[84] + input[85] - (xx[15] * xx[91] + xx[16] * xx[92]) * xx[1]) -
               (input[97] - (xx[15] * xx[93] + xx[16] * xx[156]) * xx[1] + xx[38]
                * xx[0]) - (xx[4] * input[156] + xx[42] * xx[0] + input[157] -
    (xx[15] * xx[157] + xx[16] * xx[159]) * xx[1]) - (input[169] - (xx[15] * xx
    [160] + xx[16] * xx[162]) * xx[1] + xx[44] * xx[0]) - (xx[45] * xx[0] - xx[4]
    * input[180] + input[181] - (xx[15] * xx[163] + xx[16] * xx[165]) * xx[1]) -
               (input[193] - (xx[15] * xx[166] + xx[16] * xx[168]) * xx[1] + xx
                [51] * xx[0]) - (xx[4] * input[252] + xx[94] * xx[0] + input[253]
    - (xx[15] * xx[169] + xx[16] * xx[171]) * xx[1]) - (input[265] - (xx[15] *
    xx[172] + xx[16] * xx[174]) * xx[1] + xx[96] * xx[0]) - (xx[97] * xx[0] -
    xx[4] * input[276] + input[277] - (xx[15] * xx[175] + xx[16] * xx[177]) *
    xx[1]) - (input[289] - (xx[15] * xx[178] + xx[16] * xx[180]) * xx[1] + xx[99]
              * xx[0]) - (xx[4] * input[348] + xx[100] * xx[0] + input[349] -
    (xx[15] * xx[181] + xx[16] * xx[183]) * xx[1]) - (input[361] - (xx[15] * xx
    [184] + xx[16] * xx[186]) * xx[1] + xx[102] * xx[0]) - (xx[103] * xx[0] -
    xx[4] * input[372] + input[373] - (xx[15] * xx[187] + xx[16] * xx[189]) *
    xx[1]) - (input[385] - (xx[15] * xx[190] + xx[16] * xx[192]) * xx[1] + xx
              [105] * xx[0]) - (input[445] - (xx[16] * xx[193] - xx[15] * xx[195])
    * xx[1] - xx[106] * xx[196]) - (input[457] - (xx[16] * xx[198] - xx[15] *
    xx[199]) * xx[1] - (xx[4] * input[456] + xx[108] * xx[196])) - (input[469] -
    (xx[16] * xx[201] - xx[15] * xx[202]) * xx[1] - xx[109] * xx[196]) - (xx[4] *
    input[480] - xx[111] * xx[196] + input[481] - (xx[16] * xx[204] - xx[15] *
    xx[205]) * xx[1]) - (input[493] - (xx[16] * xx[207] - xx[15] * xx[208]) *
    xx[1] - xx[112] * xx[196]) - (input[505] - (xx[16] * xx[210] - xx[15] * xx
    [211]) * xx[1] - (xx[4] * input[504] + xx[114] * xx[196])) - (input[517] -
    (xx[16] * xx[213] - xx[15] * xx[214]) * xx[1] - xx[115] * xx[196]) - (xx[4] *
    input[528] - xx[117] * xx[196] + input[529] - (xx[16] * xx[216] - xx[15] *
    xx[217]) * xx[1]) - (input[541] - (xx[16] * xx[219] - xx[15] * xx[220]) *
    xx[1] - xx[118] * xx[196]) - (input[553] - (xx[16] * xx[222] - xx[15] * xx
    [223]) * xx[1] - (xx[4] * input[552] + xx[120] * xx[196])) - (input[565] -
    (xx[16] * xx[225] - xx[15] * xx[226]) * xx[1] - xx[121] * xx[196]) - (xx[4] *
    input[576] - xx[123] * xx[196] + input[577] - (xx[16] * xx[230] - xx[15] *
    xx[235]) * xx[1]) - (input[589] - (xx[16] * xx[241] - xx[15] * xx[242]) *
    xx[1] - xx[124] * xx[196]) - (input[601] - (xx[16] * xx[243] - xx[15] * xx
    [244]) * xx[1] - (xx[4] * input[600] + xx[126] * xx[196])) - (input[613] -
    (xx[16] * xx[245] - xx[15] * xx[246]) * xx[1] - xx[127] * xx[196]) - (xx[4] *
    input[624] - xx[129] * xx[196] + input[625] - (xx[16] * xx[247] - xx[15] *
    xx[248]) * xx[1]) - (input[637] - (xx[16] * xx[249] - xx[15] * xx[250]) *
    xx[1] - xx[130] * xx[196]) - (input[649] - (xx[16] * xx[251] - xx[15] * xx
    [252]) * xx[1] - (xx[4] * input[648] + xx[132] * xx[196])) - (input[661] -
    (xx[16] * xx[253] - xx[15] * xx[254]) * xx[1] - xx[133] * xx[196]) - (xx[4] *
    input[672] - xx[135] * xx[196] + input[673] - (xx[16] * xx[255] - xx[15] *
    xx[256]) * xx[1]) - (input[685] - (xx[16] * xx[257] - xx[15] * xx[258]) *
    xx[1] - xx[136] * xx[196]) - (input[697] - (xx[16] * xx[259] - xx[15] * xx
    [260]) * xx[1] - (xx[4] * input[696] + xx[138] * xx[196])) - (input[709] -
    (xx[16] * xx[261] - xx[15] * xx[262]) * xx[1] - xx[139] * xx[196]) - (xx[4] *
    input[720] - xx[141] * xx[196] + input[721] - (xx[16] * xx[263] - xx[15] *
    xx[264]) * xx[1]) - (input[733] - (xx[16] * xx[265] - xx[15] * xx[266]) *
    xx[1] - xx[142] * xx[196]) - (input[745] - (xx[16] * xx[267] - xx[15] * xx
    [268]) * xx[1] - (xx[4] * input[744] + xx[144] * xx[196])) - (input[757] -
    (xx[16] * xx[269] - xx[15] * xx[270]) * xx[1] - xx[145] * xx[196]) - (xx[4] *
    input[768] - xx[147] * xx[196] + input[769] - (xx[16] * xx[271] - xx[15] *
    xx[272]) * xx[1]) - (input[781] - (xx[16] * xx[273] - xx[15] * xx[274]) *
    xx[1] + xx[148] * xx[275]) - (xx[150] * xx[275] - xx[4] * input[792] +
    input[793] - (xx[16] * xx[276] - xx[15] * xx[277]) * xx[1]) - (input[805] -
    (xx[16] * xx[278] - xx[15] * xx[279]) * xx[1] + xx[151] * xx[275]) - (xx[4] *
    input[816] + xx[153] * xx[275] + input[817] - (xx[16] * xx[280] - xx[15] *
    xx[281]) * xx[1]) - input[841] - input[865] - (input[925] + xx[282] * input
    [923]) - (input[937] + xx[282] * input[935]) - (input[949] + xx[282] *
    input[947]) - (input[961] + xx[282] * input[959]) + xx[228] * xx[237] - xx[0]
               * xx[154]);
  xx[287] = - (xx[239] - (input[62] + xx[1] * (xx[15] * xx[6] - xx[16] * xx[5])
    - xx[0] * xx[35]) - (input[74] + xx[1] * (xx[15] * xx[33] - xx[16] * xx[32])
    - (xx[0] * xx[43] + xx[4] * input[72])) - (input[86] + xx[1] * (xx[15] * xx
    [92] - xx[16] * xx[91]) - xx[0] * xx[47]) - (xx[4] * input[96] - xx[0] * xx
    [95] + input[98] + xx[1] * (xx[15] * xx[156] - xx[16] * xx[93])) - (input
    [158] + xx[1] * (xx[15] * xx[159] - xx[16] * xx[157]) - xx[0] * xx[98]) -
               (input[170] + xx[1] * (xx[15] * xx[162] - xx[16] * xx[160]) -
                (xx[0] * xx[101] + xx[4] * input[168])) - (input[182] + xx[1] *
    (xx[15] * xx[165] - xx[16] * xx[163]) - xx[0] * xx[104]) - (xx[4] * input
    [192] - xx[0] * xx[107] + input[194] + xx[1] * (xx[15] * xx[168] - xx[16] *
    xx[166])) - (input[254] + xx[1] * (xx[15] * xx[171] - xx[16] * xx[169]) -
                 xx[0] * xx[110]) - (input[266] + xx[1] * (xx[15] * xx[174] -
    xx[16] * xx[172]) - (xx[0] * xx[113] + xx[4] * input[264])) - (input[278] +
    xx[1] * (xx[15] * xx[177] - xx[16] * xx[175]) - xx[0] * xx[116]) - (xx[4] *
    input[288] - xx[0] * xx[119] + input[290] + xx[1] * (xx[15] * xx[180] - xx
    [16] * xx[178])) - (input[350] + xx[1] * (xx[15] * xx[183] - xx[16] * xx[181])
                        - xx[0] * xx[122]) - (input[362] + xx[1] * (xx[15] * xx
    [186] - xx[16] * xx[184]) - (xx[0] * xx[125] + xx[4] * input[360])) -
               (input[374] + xx[1] * (xx[15] * xx[189] - xx[16] * xx[187]) - xx
                [0] * xx[128]) - (xx[4] * input[384] - xx[0] * xx[131] + input
    [386] + xx[1] * (xx[15] * xx[192] - xx[16] * xx[190])) - (xx[196] * xx[134]
    - xx[4] * input[444] + input[446] - xx[1] * (xx[15] * xx[193] + xx[16] * xx
    [195])) - (xx[196] * xx[137] + input[458] - xx[1] * (xx[15] * xx[198] + xx
    [16] * xx[199])) - (xx[196] * xx[140] + xx[4] * input[468] + input[470] -
                        xx[1] * (xx[15] * xx[201] + xx[16] * xx[202])) - (xx[196]
    * xx[143] + input[482] - xx[1] * (xx[15] * xx[204] + xx[16] * xx[205])) -
               (xx[196] * xx[146] - xx[4] * input[492] + input[494] - xx[1] *
                (xx[15] * xx[207] + xx[16] * xx[208])) - (xx[196] * xx[149] +
    input[506] - xx[1] * (xx[15] * xx[210] + xx[16] * xx[211])) - (xx[196] * xx
    [152] + xx[4] * input[516] + input[518] - xx[1] * (xx[15] * xx[213] + xx[16]
    * xx[214])) - (xx[196] * xx[155] + input[530] - xx[1] * (xx[15] * xx[216] +
    xx[16] * xx[217])) - (xx[196] * xx[158] - xx[4] * input[540] + input[542] -
    xx[1] * (xx[15] * xx[219] + xx[16] * xx[220])) - (xx[196] * xx[161] + input
    [554] - xx[1] * (xx[15] * xx[222] + xx[16] * xx[223])) - (xx[196] * xx[164]
    + xx[4] * input[564] + input[566] - xx[1] * (xx[15] * xx[225] + xx[16] * xx
    [226])) - (xx[196] * xx[167] + input[578] - xx[1] * (xx[15] * xx[230] + xx
    [16] * xx[235])) - (xx[196] * xx[170] - xx[4] * input[588] + input[590] -
                        xx[1] * (xx[15] * xx[241] + xx[16] * xx[242])) - (xx[196]
    * xx[173] + input[602] - xx[1] * (xx[15] * xx[243] + xx[16] * xx[244])) -
               (xx[196] * xx[176] + xx[4] * input[612] + input[614] - xx[1] *
                (xx[15] * xx[245] + xx[16] * xx[246])) - (xx[196] * xx[179] +
    input[626] - xx[1] * (xx[15] * xx[247] + xx[16] * xx[248])) - (xx[196] * xx
    [182] - xx[4] * input[636] + input[638] - xx[1] * (xx[15] * xx[249] + xx[16]
    * xx[250])) - (xx[196] * xx[185] + input[650] - xx[1] * (xx[15] * xx[251] +
    xx[16] * xx[252])) - (xx[196] * xx[188] + xx[4] * input[660] + input[662] -
    xx[1] * (xx[15] * xx[253] + xx[16] * xx[254])) - (xx[196] * xx[191] + input
    [674] - xx[1] * (xx[15] * xx[255] + xx[16] * xx[256])) - (xx[196] * xx[194]
    - xx[4] * input[684] + input[686] - xx[1] * (xx[15] * xx[257] + xx[16] * xx
    [258])) - (xx[196] * xx[197] + input[698] - xx[1] * (xx[15] * xx[259] + xx
    [16] * xx[260])) - (xx[196] * xx[200] + xx[4] * input[708] + input[710] -
                        xx[1] * (xx[15] * xx[261] + xx[16] * xx[262])) - (xx[196]
    * xx[203] + input[722] - xx[1] * (xx[15] * xx[263] + xx[16] * xx[264])) -
               (xx[196] * xx[206] - xx[4] * input[732] + input[734] - xx[1] *
                (xx[15] * xx[265] + xx[16] * xx[266])) - (xx[196] * xx[209] +
    input[746] - xx[1] * (xx[15] * xx[267] + xx[16] * xx[268])) - (xx[196] * xx
    [212] + xx[4] * input[756] + input[758] - xx[1] * (xx[15] * xx[269] + xx[16]
    * xx[270])) - (xx[196] * xx[215] + input[770] - xx[1] * (xx[15] * xx[271] +
    xx[16] * xx[272])) - (input[782] - xx[1] * (xx[15] * xx[273] + xx[16] * xx
    [274]) - (xx[275] * xx[218] + xx[4] * input[780])) - (input[794] - xx[1] *
    (xx[15] * xx[276] + xx[16] * xx[277]) - xx[275] * xx[221]) - (xx[4] * input
    [804] - xx[275] * xx[224] + input[806] - xx[1] * (xx[15] * xx[278] + xx[16] *
    xx[279])) - (input[818] - xx[1] * (xx[15] * xx[280] + xx[16] * xx[281]) -
                 xx[275] * xx[227]) - input[842] - input[866] - (input[926] -
    xx[282] * input[922]) - (input[938] - xx[282] * input[934]) - (input[950] -
    xx[282] * input[946]) - (input[962] - xx[282] * input[958]) - xx[228] * xx
               [236] + xx[0] * xx[12]);
  xx[288] = input[963] - (xx[240] - (input[63] - xx[4] * xx[35]) - (xx[18] * xx
    [4] + input[75]) - (input[87] + xx[4] * xx[47]) - (input[99] - xx[38] * xx[4])
    - (input[159] - xx[4] * xx[98]) - (xx[44] * xx[4] + input[171]) - (input[183]
    + xx[4] * xx[104]) - (input[195] - xx[51] * xx[4]) - (input[255] - xx[4] *
    xx[110]) - (xx[96] * xx[4] + input[267]) - (input[279] + xx[4] * xx[116]) -
    (input[291] - xx[99] * xx[4]) - (input[351] - xx[4] * xx[122]) - (xx[102] *
    xx[4] + input[363]) - (input[375] + xx[4] * xx[128]) - (input[387] - xx[105]
    * xx[4]) - (xx[106] * xx[4] + input[447]) - (input[459] + xx[4] * xx[137]) -
    (input[471] - xx[109] * xx[4]) - (input[483] - xx[4] * xx[143]) - (xx[112] *
    xx[4] + input[495]) - (input[507] + xx[4] * xx[149]) - (input[519] - xx[115]
    * xx[4]) - (input[531] - xx[4] * xx[155]) - (xx[118] * xx[4] + input[543]) -
    (input[555] + xx[4] * xx[161]) - (input[567] - xx[121] * xx[4]) - (input[579]
    - xx[4] * xx[167]) - (xx[124] * xx[4] + input[591]) - (input[603] + xx[4] *
    xx[173]) - (input[615] - xx[127] * xx[4]) - (input[627] - xx[4] * xx[179]) -
    (xx[130] * xx[4] + input[639]) - (input[651] + xx[4] * xx[185]) - (input[663]
    - xx[133] * xx[4]) - (input[675] - xx[4] * xx[191]) - (xx[136] * xx[4] +
    input[687]) - (input[699] + xx[4] * xx[197]) - (input[711] - xx[139] * xx[4])
    - (input[723] - xx[4] * xx[203]) - (xx[142] * xx[4] + input[735]) - (input
    [747] + xx[4] * xx[209]) - (input[759] - xx[145] * xx[4]) - (input[771] -
    xx[4] * xx[215]) - (xx[148] * xx[4] + input[783]) - (input[795] + xx[4] *
    xx[221]) - (input[807] - xx[151] * xx[4]) - (input[819] - xx[4] * xx[227]) -
    input[843] - input[867] - input[927] - input[939] - input[951]);
  solveSymmetricPosDef(xx + 55, xx + 283, 6, 1, xx + 47, xx + 91);
  xx[91] = xx[23];
  xx[92] = xx[23];
  xx[93] = xx[23];
  xx[94] = xx[17];
  xx[95] = xx[23];
  xx[96] = xx[23];
  xx[97] = xx[23];
  xx[98] = xx[23];
  xx[99] = xx[23];
  xx[100] = xx[23];
  xx[101] = xx[31];
  xx[102] = xx[23];
  xx[103] = xx[23];
  xx[104] = xx[23];
  xx[105] = xx[23];
  xx[106] = xx[23];
  xx[107] = xx[23];
  xx[108] = xx[40];
  xx[109] = xx[27];
  xx[110] = xx[2];
  xx[111] = xx[36];
  xx[112] = xx[23];
  xx[113] = xx[46];
  xx[114] = xx[23];
  xx[115] = xx[28];
  xx[116] = xx[30];
  xx[117] = xx[39];
  xx[118] = - xx[46];
  xx[119] = xx[23];
  xx[120] = xx[23];
  xx[121] = xx[29];
  xx[122] = xx[3];
  xx[123] = xx[41];
  xx[124] = xx[23];
  xx[125] = xx[23];
  xx[126] = xx[23];
  solveSymmetricPosDef(xx + 55, xx + 91, 6, 6, xx + 127, xx + 24);
  xx[2] = xx[145];
  xx[3] = xx[151];
  xx[4] = xx[157];
  xx[0] = 9.800000000000001;
  xx[5] = xx[0] * xx[22];
  xx[6] = xx[0] * xx[20];
  xx[14] = (xx[19] * xx[5] + xx[21] * xx[6]) * xx[1];
  xx[15] = xx[0] - (xx[22] * xx[5] + xx[20] * xx[6]) * xx[1];
  xx[16] = xx[1] * (xx[21] * xx[5] - xx[19] * xx[6]);
  xx[17] = xx[146];
  xx[18] = xx[152];
  xx[19] = xx[158];
  xx[20] = xx[147];
  xx[21] = xx[153];
  xx[22] = xx[159];
  xx[24] = xx[148];
  xx[25] = xx[154];
  xx[26] = xx[160];
  xx[27] = xx[149];
  xx[28] = xx[155];
  xx[29] = xx[161];
  xx[30] = xx[150];
  xx[31] = xx[156];
  xx[32] = xx[162];
  xx[33] = state[16];
  xx[34] = state[17];
  xx[35] = state[18];
  xx[36] = state[19];
  xx[38] = state[23];
  xx[39] = state[24];
  xx[40] = state[25];
  pm_math_Quaternion_compDeriv_ra(xx + 33, xx + 38, xx + 41);
  xx[5] = 0.44080001915388;
  xx[6] = - 0.5528972265384762;
  xx[33] = xx[5];
  xx[34] = - xx[5];
  xx[35] = xx[6];
  xx[36] = xx[6];
  xx[53] = - state[16];
  xx[54] = - state[17];
  xx[55] = - state[18];
  xx[56] = - state[19];
  pm_math_Quaternion_compose_ra(xx + 33, xx + 53, xx + 57);
  xx[5] = 0.9748684321931496;
  xx[6] = xx[5] * xx[59];
  xx[12] = 0.2227813724557561;
  xx[45] = xx[12] * xx[60] + xx[5] * xx[58];
  xx[46] = xx[12] * xx[59];
  xx[53] = xx[6];
  xx[54] = - xx[45];
  xx[55] = xx[46];
  pm_math_Vector3_cross_ra(xx + 58, xx + 53, xx + 61);
  xx[53] = xx[57] * xx[59];
  xx[54] = xx[1] * (xx[61] - xx[5] * xx[53]) - xx[12];
  xx[12] = xx[1] * (xx[62] + xx[57] * xx[45]);
  xx[45] = xx[5] + (xx[63] - xx[57] * xx[46]) * xx[1];
  xx[61] = xx[54];
  xx[62] = xx[12];
  xx[63] = xx[45];
  xx[46] = 0.09817477042468103;
  xx[55] = xx[54] * xx[46];
  xx[56] = xx[46] * xx[12];
  xx[64] = xx[45] * xx[46];
  xx[65] = xx[55];
  xx[66] = xx[56];
  xx[67] = xx[64];
  xx[45] = 0.2227813724557562;
  xx[68] = xx[5] * xx[60] - xx[45] * xx[58];
  xx[69] = xx[45] * xx[59];
  xx[70] = xx[68];
  xx[71] = - xx[6];
  pm_math_Vector3_cross_ra(xx + 58, xx + 69, xx + 72);
  xx[69] = xx[5] + xx[1] * (xx[72] - xx[45] * xx[53]);
  xx[5] = xx[69] * xx[46];
  xx[53] = xx[1] * (xx[73] - xx[57] * xx[68]);
  xx[68] = xx[46] * xx[53];
  xx[70] = xx[45] + (xx[57] * xx[6] + xx[74]) * xx[1];
  xx[6] = xx[70] * xx[46];
  xx[71] = xx[5];
  xx[72] = xx[68];
  xx[73] = xx[6];
  xx[45] = pm_math_Vector3_dot_ra(xx + 61, xx + 71);
  xx[74] = (xx[57] * xx[60] + xx[58] * xx[59]) * xx[1];
  xx[75] = xx[46] * xx[74];
  xx[76] = xx[37] - (xx[60] * xx[60] + xx[58] * xx[58]) * xx[1];
  xx[37] = xx[46] * xx[76];
  xx[77] = xx[1] * (xx[59] * xx[60] - xx[57] * xx[58]);
  xx[78] = xx[46] * xx[77];
  xx[79] = xx[75];
  xx[80] = xx[37];
  xx[81] = xx[78];
  xx[82] = pm_math_Vector3_dot_ra(xx + 61, xx + 79);
  xx[83] = 2.454369260617027e-3;
  xx[84] = - (xx[83] * xx[12]);
  xx[12] = xx[54] * xx[83];
  xx[85] = xx[69];
  xx[86] = xx[53];
  xx[87] = xx[70];
  xx[54] = pm_math_Vector3_dot_ra(xx + 85, xx + 79);
  xx[70] = - (xx[83] * xx[53]);
  xx[53] = xx[69] * xx[83];
  xx[88] = xx[74];
  xx[89] = xx[76];
  xx[90] = xx[77];
  xx[69] = - (xx[83] * xx[76]);
  xx[76] = xx[83] * xx[74];
  xx[74] = 9.715211656609066e-5;
  xx[77] = 3.067961575771283e-5;
  xx[91] = pm_math_Vector3_dot_ra(xx + 61, xx + 65);
  xx[92] = xx[45];
  xx[93] = xx[82];
  xx[94] = xx[84];
  xx[95] = xx[12];
  xx[96] = xx[23];
  xx[97] = xx[45];
  xx[98] = pm_math_Vector3_dot_ra(xx + 85, xx + 71);
  xx[99] = xx[54];
  xx[100] = xx[70];
  xx[101] = xx[53];
  xx[102] = xx[23];
  xx[103] = xx[82];
  xx[104] = xx[54];
  xx[105] = pm_math_Vector3_dot_ra(xx + 88, xx + 79);
  xx[106] = xx[69];
  xx[107] = xx[76];
  xx[108] = xx[23];
  xx[109] = xx[84];
  xx[110] = xx[70];
  xx[111] = xx[69];
  xx[112] = xx[74];
  xx[113] = xx[23];
  xx[114] = xx[23];
  xx[115] = xx[12];
  xx[116] = xx[53];
  xx[117] = xx[76];
  xx[118] = xx[23];
  xx[119] = xx[74];
  xx[120] = xx[23];
  xx[121] = xx[23];
  xx[122] = xx[23];
  xx[123] = xx[23];
  xx[124] = xx[23];
  xx[125] = xx[23];
  xx[126] = xx[77];
  ii[0] = factorSymmetricPosDef(xx + 91, 6, xx + 69);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'V1_2P/Body Configuration/SixDOF' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[65] = state[20];
  xx[66] = state[21];
  xx[67] = state[22];
  pm_math_Quaternion_xform_ra(xx + 33, xx + 65, xx + 69);
  pm_math_Quaternion_inverseXform_ra(xx + 57, xx + 69, xx + 65);
  xx[12] = 0.02500000000000001;
  xx[69] = xx[65] + xx[12] * state[24];
  xx[70] = xx[66] - xx[12] * state[23];
  xx[71] = xx[67];
  pm_math_Vector3_cross_ra(xx + 38, xx + 69, xx + 72);
  xx[69] = - xx[65];
  xx[70] = - xx[66];
  xx[71] = - xx[67];
  pm_math_Vector3_cross_ra(xx + 38, xx + 69, xx + 65);
  xx[45] = 0.03499999999999999;
  xx[53] = xx[45] * xx[59];
  xx[54] = xx[45] * xx[58];
  xx[69] = xx[12] * state[18];
  xx[70] = xx[12] * state[17];
  xx[79] = state[13] + (xx[69] * state[16] + xx[70] * state[19]) * xx[1];
  xx[80] = state[14] - xx[1] * (xx[70] * state[16] - xx[69] * state[19]);
  xx[81] = state[15] - (xx[70] * state[17] + xx[69] * state[18]) * xx[1] + xx[12];
  pm_math_Quaternion_xform_ra(xx + 33, xx + 79, xx + 69);
  xx[33] = (xx[57] * xx[53] + xx[60] * xx[54]) * xx[1] + xx[69] - xx[11] +
    0.360447814693318;
  xx[11] = xx[1] * (xx[60] * xx[53] - xx[57] * xx[54]) + xx[70] - xx[13] +
    0.5564504150307564;
  xx[13] = xx[71] - (xx[58] * xx[54] + xx[59] * xx[53]) * xx[1] - xx[231] +
    0.6881639892349409;
  xx[34] = sqrt(xx[33] * xx[33] + xx[11] * xx[11] + xx[13] * xx[13]);
  if (xx[34] == 0.0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:ForceUndefined",
      "'V1_2P/Suction/Internal Force' force is undefined since base and follower origins are coincident.",
      neDiagMgr);
  }

  xx[34] = input[964] / xx[34];
  xx[69] = xx[34] * xx[33];
  xx[70] = xx[34] * xx[11];
  xx[71] = xx[34] * xx[13];
  pm_math_Quaternion_inverseXform_ra(xx + 57, xx + 69, xx + 79);
  xx[11] = (xx[72] + xx[65]) * xx[46] - (input[10] + input[22] + input[34] +
    input[46] + input[106] + input[118] + input[130] + input[142] + input[202] +
    input[214] + input[226] + input[238] + input[298] + input[310] + input[322]
    + input[334] + input[394] + input[406] + input[418] + input[430] + input[484]
    + input[496] + input[508] + input[520] + input[826] + input[850] + input[874]
    + input[886] + input[898] + input[910] + xx[79]);
  xx[13] = (xx[73] + xx[66]) * xx[46] - (input[11] + input[23] + input[35] +
    input[47] + input[107] + input[119] + input[131] + input[143] + input[203] +
    input[215] + input[227] + input[239] + input[299] + input[311] + input[323]
    + input[335] + input[395] + input[407] + input[419] + input[431] + input[485]
    + input[497] + input[509] + input[521] + input[827] + input[851] + input[875]
    + input[887] + input[899] + input[911] + xx[80]);
  xx[69] = xx[11];
  xx[70] = xx[13];
  xx[71] = (xx[74] + xx[67]) * xx[46] - (input[12] + input[24] + input[36] +
    input[48] + input[108] + input[120] + input[132] + input[144] + input[204] +
    input[216] + input[228] + input[240] + input[300] + input[312] + input[324]
    + input[336] + input[396] + input[408] + input[420] + input[432] + input[486]
    + input[498] + input[510] + input[522] + input[828] + input[852] + input[876]
    + input[888] + input[900] + input[912] + xx[81]);
  xx[33] = 3.579288505066496e-5;
  xx[65] = xx[33] * state[23];
  xx[66] = xx[33] * state[24];
  xx[67] = xx[77] * state[25];
  pm_math_Vector3_cross_ra(xx + 38, xx + 65, xx + 72);
  xx[35] = 0.015;
  xx[36] = - 0.01600000000000001;
  xx[38] = xx[35];
  xx[39] = xx[35];
  xx[40] = xx[36];
  xx[65] = input[10];
  xx[66] = input[11];
  xx[67] = input[12];
  pm_math_Vector3_cross_ra(xx + 38, xx + 65, xx + 127);
  xx[46] = - xx[35];
  xx[65] = xx[35];
  xx[66] = xx[46];
  xx[67] = xx[36];
  xx[130] = input[22];
  xx[131] = input[23];
  xx[132] = input[24];
  pm_math_Vector3_cross_ra(xx + 65, xx + 130, xx + 133);
  xx[130] = xx[46];
  xx[131] = xx[46];
  xx[132] = xx[36];
  xx[136] = input[34];
  xx[137] = input[35];
  xx[138] = input[36];
  pm_math_Vector3_cross_ra(xx + 130, xx + 136, xx + 139);
  xx[136] = xx[46];
  xx[137] = xx[35];
  xx[138] = xx[36];
  xx[142] = input[46];
  xx[143] = input[47];
  xx[144] = input[48];
  pm_math_Vector3_cross_ra(xx + 136, xx + 142, xx + 145);
  xx[142] = input[106];
  xx[143] = input[107];
  xx[144] = input[108];
  pm_math_Vector3_cross_ra(xx + 38, xx + 142, xx + 148);
  xx[142] = input[118];
  xx[143] = input[119];
  xx[144] = input[120];
  pm_math_Vector3_cross_ra(xx + 65, xx + 142, xx + 151);
  xx[142] = input[130];
  xx[143] = input[131];
  xx[144] = input[132];
  pm_math_Vector3_cross_ra(xx + 130, xx + 142, xx + 154);
  xx[142] = input[142];
  xx[143] = input[143];
  xx[144] = input[144];
  pm_math_Vector3_cross_ra(xx + 136, xx + 142, xx + 157);
  xx[142] = input[202];
  xx[143] = input[203];
  xx[144] = input[204];
  pm_math_Vector3_cross_ra(xx + 38, xx + 142, xx + 160);
  xx[142] = input[214];
  xx[143] = input[215];
  xx[144] = input[216];
  pm_math_Vector3_cross_ra(xx + 65, xx + 142, xx + 163);
  xx[142] = input[226];
  xx[143] = input[227];
  xx[144] = input[228];
  pm_math_Vector3_cross_ra(xx + 130, xx + 142, xx + 166);
  xx[142] = input[238];
  xx[143] = input[239];
  xx[144] = input[240];
  pm_math_Vector3_cross_ra(xx + 136, xx + 142, xx + 169);
  xx[142] = input[298];
  xx[143] = input[299];
  xx[144] = input[300];
  pm_math_Vector3_cross_ra(xx + 38, xx + 142, xx + 172);
  xx[142] = input[310];
  xx[143] = input[311];
  xx[144] = input[312];
  pm_math_Vector3_cross_ra(xx + 65, xx + 142, xx + 175);
  xx[142] = input[322];
  xx[143] = input[323];
  xx[144] = input[324];
  pm_math_Vector3_cross_ra(xx + 130, xx + 142, xx + 178);
  xx[142] = input[334];
  xx[143] = input[335];
  xx[144] = input[336];
  pm_math_Vector3_cross_ra(xx + 136, xx + 142, xx + 181);
  xx[142] = input[394];
  xx[143] = input[395];
  xx[144] = input[396];
  pm_math_Vector3_cross_ra(xx + 38, xx + 142, xx + 184);
  xx[38] = input[406];
  xx[39] = input[407];
  xx[40] = input[408];
  pm_math_Vector3_cross_ra(xx + 65, xx + 38, xx + 142);
  xx[38] = input[418];
  xx[39] = input[419];
  xx[40] = input[420];
  pm_math_Vector3_cross_ra(xx + 130, xx + 38, xx + 65);
  xx[38] = input[430];
  xx[39] = input[431];
  xx[40] = input[432];
  pm_math_Vector3_cross_ra(xx + 136, xx + 38, xx + 130);
  xx[35] = 0.02600000000000001;
  xx[36] = 6.938893903907228e-18;
  xx[38] = 4.999999999999999e-3;
  xx[39] = - 5.000000000000011e-3;
  xx[136] = xx[38];
  xx[137] = xx[38];
  xx[138] = xx[39];
  xx[187] = input[874];
  xx[188] = input[875];
  xx[189] = input[876];
  pm_math_Vector3_cross_ra(xx + 136, xx + 187, xx + 190);
  xx[40] = - xx[38];
  xx[136] = xx[38];
  xx[137] = xx[40];
  xx[138] = xx[39];
  xx[187] = input[886];
  xx[188] = input[887];
  xx[189] = input[888];
  pm_math_Vector3_cross_ra(xx + 136, xx + 187, xx + 193);
  xx[136] = xx[40];
  xx[137] = xx[40];
  xx[138] = xx[39];
  xx[187] = input[898];
  xx[188] = input[899];
  xx[189] = input[900];
  pm_math_Vector3_cross_ra(xx + 136, xx + 187, xx + 196);
  xx[136] = xx[40];
  xx[137] = xx[38];
  xx[138] = xx[39];
  xx[38] = input[910];
  xx[39] = input[911];
  xx[40] = input[912];
  pm_math_Vector3_cross_ra(xx + 136, xx + 38, xx + 187);
  xx[199] = - pm_math_Vector3_dot_ra(xx + 61, xx + 69);
  xx[200] = - pm_math_Vector3_dot_ra(xx + 85, xx + 69);
  xx[201] = - pm_math_Vector3_dot_ra(xx + 88, xx + 69);
  xx[202] = - (xx[72] - (xx[127] + input[13]) - (xx[133] + input[25]) - (xx[139]
    + input[37]) - (xx[145] + input[49]) - (xx[148] + input[109]) - (xx[151] +
    input[121]) - (xx[154] + input[133]) - (xx[157] + input[145]) - (xx[160] +
    input[205]) - (xx[163] + input[217]) - (xx[166] + input[229]) - (xx[169] +
    input[241]) - (xx[172] + input[301]) - (xx[175] + input[313]) - (xx[178] +
    input[325]) - (xx[181] + input[337]) - (xx[184] + input[397]) - (xx[142] +
    input[409]) - (xx[65] + input[421]) - (xx[130] + input[433]) - (input[487] +
    xx[35] * input[485]) - (input[499] + xx[35] * input[497]) - (input[511] +
    xx[35] * input[509]) - (input[523] + xx[35] * input[521]) - (input[829] +
    xx[36] * input[827]) - (input[853] + xx[36] * input[851]) - (xx[190] +
    input[877]) - (xx[193] + input[889]) - (xx[196] + input[901]) - (xx[187] +
    input[913]) + xx[45] * xx[80] - xx[12] * xx[13]);
  xx[203] = - (xx[73] - (xx[128] + input[14]) - (xx[134] + input[26]) - (xx[140]
    + input[38]) - (xx[146] + input[50]) - (xx[149] + input[110]) - (xx[152] +
    input[122]) - (xx[155] + input[134]) - (xx[158] + input[146]) - (xx[161] +
    input[206]) - (xx[164] + input[218]) - (xx[167] + input[230]) - (xx[170] +
    input[242]) - (xx[173] + input[302]) - (xx[176] + input[314]) - (xx[179] +
    input[326]) - (xx[182] + input[338]) - (xx[185] + input[398]) - (xx[143] +
    input[410]) - (xx[66] + input[422]) - (xx[131] + input[434]) - (input[488] -
    xx[35] * input[484]) - (input[500] - xx[35] * input[496]) - (input[512] -
    xx[35] * input[508]) - (input[524] - xx[35] * input[520]) - (input[830] -
    xx[36] * input[826]) - (input[854] - xx[36] * input[850]) - (xx[191] +
    input[878]) - (xx[194] + input[890]) - (xx[197] + input[902]) - (xx[188] +
    input[914]) - xx[45] * xx[79] + xx[12] * xx[11]);
  xx[204] = xx[189] + input[915] - (xx[74] - (xx[129] + input[15]) - (xx[135] +
    input[27]) - (xx[141] + input[39]) - (xx[147] + input[51]) - (xx[150] +
    input[111]) - (xx[153] + input[123]) - (xx[156] + input[135]) - (xx[159] +
    input[147]) - (xx[162] + input[207]) - (xx[165] + input[219]) - (xx[168] +
    input[231]) - (xx[171] + input[243]) - (xx[174] + input[303]) - (xx[177] +
    input[315]) - (xx[180] + input[327]) - (xx[183] + input[339]) - (xx[186] +
    input[399]) - (xx[144] + input[411]) - (xx[67] + input[423]) - (xx[132] +
    input[435]) - input[489] - input[501] - input[513] - input[525] - input[831]
    - input[855] - (xx[192] + input[879]) - (xx[195] + input[891]) - (xx[198] +
    input[903]));
  solveSymmetricPosDef(xx + 91, xx + 199, 6, 1, xx + 69, xx + 84);
  xx[127] = xx[23];
  xx[128] = xx[23];
  xx[129] = xx[23];
  xx[130] = xx[33];
  xx[131] = xx[23];
  xx[132] = xx[23];
  xx[133] = xx[23];
  xx[134] = xx[23];
  xx[135] = xx[23];
  xx[136] = xx[23];
  xx[137] = xx[33];
  xx[138] = xx[23];
  xx[139] = xx[23];
  xx[140] = xx[23];
  xx[141] = xx[23];
  xx[142] = xx[23];
  xx[143] = xx[23];
  xx[144] = xx[77];
  xx[145] = xx[55];
  xx[146] = xx[5];
  xx[147] = xx[75];
  xx[148] = xx[23];
  xx[149] = xx[83];
  xx[150] = xx[23];
  xx[151] = xx[56];
  xx[152] = xx[68];
  xx[153] = xx[37];
  xx[154] = - xx[83];
  xx[155] = xx[23];
  xx[156] = xx[23];
  xx[157] = xx[64];
  xx[158] = xx[6];
  xx[159] = xx[78];
  xx[160] = xx[23];
  xx[161] = xx[23];
  xx[162] = xx[23];
  solveSymmetricPosDef(xx + 91, xx + 127, 6, 6, xx + 163, xx + 35);
  xx[11] = xx[181];
  xx[12] = xx[187];
  xx[13] = xx[193];
  xx[5] = xx[0] * xx[60];
  xx[6] = xx[0] * xx[58];
  xx[35] = (xx[57] * xx[5] + xx[59] * xx[6]) * xx[1];
  xx[36] = xx[0] - (xx[60] * xx[5] + xx[58] * xx[6]) * xx[1];
  xx[37] = xx[1] * (xx[59] * xx[5] - xx[57] * xx[6]);
  xx[38] = xx[182];
  xx[39] = xx[188];
  xx[40] = xx[194];
  xx[53] = xx[183];
  xx[54] = xx[189];
  xx[55] = xx[195];
  xx[56] = xx[184];
  xx[57] = xx[190];
  xx[58] = xx[196];
  xx[59] = xx[185];
  xx[60] = xx[191];
  xx[61] = xx[197];
  xx[62] = xx[186];
  xx[63] = xx[192];
  xx[64] = xx[198];
  deriv[0] = state[7];
  deriv[1] = state[8];
  deriv[2] = state[9];
  deriv[3] = xx[7];
  deriv[4] = xx[8];
  deriv[5] = xx[9];
  deriv[6] = xx[10];
  deriv[7] = xx[47] - pm_math_Vector3_dot_ra(xx + 2, xx + 14);
  deriv[8] = xx[48] - pm_math_Vector3_dot_ra(xx + 17, xx + 14);
  deriv[9] = xx[49] - pm_math_Vector3_dot_ra(xx + 20, xx + 14);
  deriv[10] = xx[50] - pm_math_Vector3_dot_ra(xx + 24, xx + 14);
  deriv[11] = xx[51] - pm_math_Vector3_dot_ra(xx + 27, xx + 14);
  deriv[12] = xx[52] - pm_math_Vector3_dot_ra(xx + 30, xx + 14);
  deriv[13] = state[20];
  deriv[14] = state[21];
  deriv[15] = state[22];
  deriv[16] = xx[41];
  deriv[17] = xx[42];
  deriv[18] = xx[43];
  deriv[19] = xx[44];
  deriv[20] = xx[69] - pm_math_Vector3_dot_ra(xx + 11, xx + 35);
  deriv[21] = xx[70] - pm_math_Vector3_dot_ra(xx + 38, xx + 35);
  deriv[22] = xx[71] - pm_math_Vector3_dot_ra(xx + 53, xx + 35);
  deriv[23] = xx[72] - pm_math_Vector3_dot_ra(xx + 56, xx + 35);
  deriv[24] = xx[73] - pm_math_Vector3_dot_ra(xx + 59, xx + 35);
  deriv[25] = xx[74] - pm_math_Vector3_dot_ra(xx + 62, xx + 35);
  errorResult[0] = xx[34] + xx[229];
  return NULL;
}

PmfMessageId V1_2P_4d7d5d8d_1_numJacPerturbLoBounds(const
  RuntimeDerivedValuesBundle *rtdv, const int *eqnEnableFlags, const double
  *state, const int *modeVector, const double *input, const double *inputDot,
  const double *inputDdot, const double *discreteState, double *bounds, double
  *errorResult, NeuDiagnosticManager *neDiagMgr)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[2];
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) state;
  (void) modeVector;
  (void) input;
  (void) inputDot;
  (void) inputDdot;
  (void) discreteState;
  (void) neDiagMgr;
  xx[0] = 1.0e-9;
  xx[1] = 1.0e-8;
  bounds[0] = xx[0];
  bounds[1] = xx[0];
  bounds[2] = xx[0];
  bounds[3] = xx[1];
  bounds[4] = xx[1];
  bounds[5] = xx[1];
  bounds[6] = xx[1];
  bounds[7] = xx[0];
  bounds[8] = xx[0];
  bounds[9] = xx[0];
  bounds[10] = xx[1];
  bounds[11] = xx[1];
  bounds[12] = xx[1];
  bounds[13] = xx[0];
  bounds[14] = xx[0];
  bounds[15] = xx[0];
  bounds[16] = xx[1];
  bounds[17] = xx[1];
  bounds[18] = xx[1];
  bounds[19] = xx[1];
  bounds[20] = xx[0];
  bounds[21] = xx[0];
  bounds[22] = xx[0];
  bounds[23] = xx[1];
  bounds[24] = xx[1];
  bounds[25] = xx[1];
  errorResult[0] = 0.0;
  return NULL;
}

PmfMessageId V1_2P_4d7d5d8d_1_numJacPerturbHiBounds(const
  RuntimeDerivedValuesBundle *rtdv, const int *eqnEnableFlags, const double
  *state, const int *modeVector, const double *input, const double *inputDot,
  const double *inputDdot, const double *discreteState, double *bounds, double
  *errorResult, NeuDiagnosticManager *neDiagMgr)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[2];
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) state;
  (void) modeVector;
  (void) input;
  (void) inputDot;
  (void) inputDdot;
  (void) discreteState;
  (void) neDiagMgr;
  xx[0] = +pmf_get_inf();
  xx[1] = 0.1;
  bounds[0] = xx[0];
  bounds[1] = xx[0];
  bounds[2] = xx[0];
  bounds[3] = xx[1];
  bounds[4] = xx[1];
  bounds[5] = xx[1];
  bounds[6] = xx[1];
  bounds[7] = xx[0];
  bounds[8] = xx[0];
  bounds[9] = xx[0];
  bounds[10] = xx[0];
  bounds[11] = xx[0];
  bounds[12] = xx[0];
  bounds[13] = xx[0];
  bounds[14] = xx[0];
  bounds[15] = xx[0];
  bounds[16] = xx[1];
  bounds[17] = xx[1];
  bounds[18] = xx[1];
  bounds[19] = xx[1];
  bounds[20] = xx[0];
  bounds[21] = xx[0];
  bounds[22] = xx[0];
  bounds[23] = xx[0];
  bounds[24] = xx[0];
  bounds[25] = xx[0];
  errorResult[0] = 0.0;
  return NULL;
}

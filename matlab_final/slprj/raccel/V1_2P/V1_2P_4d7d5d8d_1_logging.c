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

PmfMessageId V1_2P_4d7d5d8d_1_recordLog(const RuntimeDerivedValuesBundle *rtdv,
  const int *eqnEnableFlags, const double *state, const int *modeVector, const
  double *input, const double *inputDot, const double *inputDdot, double
  *logVector, double *errorResult, NeuDiagnosticManager *neDiagMgr)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  int ii[1];
  double xx[286];
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) modeVector;
  (void) inputDot;
  (void) neDiagMgr;
  xx[0] = 57.29577951308232;
  xx[1] = 0.7144432252221142;
  xx[2] = 2.0;
  xx[3] = 0.6546837452583718;
  xx[4] = 0.2671875627615766;
  xx[5] = xx[3];
  xx[6] = xx[3];
  xx[7] = xx[4];
  xx[8] = - xx[4];
  xx[9] = - state[5];
  xx[10] = - state[6];
  xx[11] = - state[7];
  xx[12] = - state[8];
  pm_math_Quaternion_compose_ra(xx + 5, xx + 9, xx + 13);
  xx[3] = 0.699693417100821;
  xx[4] = xx[3] * xx[15];
  xx[9] = - xx[4];
  xx[10] = xx[1] * xx[16] + xx[3] * xx[14];
  xx[11] = xx[1] * xx[15];
  xx[17] = xx[9];
  xx[18] = xx[10];
  xx[19] = - xx[11];
  pm_math_Vector3_cross_ra(xx + 14, xx + 17, xx + 20);
  xx[12] = xx[13] * xx[15];
  xx[17] = xx[1] + xx[2] * (xx[20] + xx[3] * xx[12]);
  xx[1] = xx[2] * (xx[21] - xx[13] * xx[10]);
  xx[10] = (xx[13] * xx[11] + xx[22]) * xx[2] - xx[3];
  xx[18] = xx[17];
  xx[19] = xx[1];
  xx[20] = xx[10];
  xx[11] = 0.07853981633974483;
  xx[21] = xx[17] * xx[11];
  xx[22] = xx[11] * xx[1];
  xx[23] = xx[10] * xx[11];
  xx[10] = 0.7144432252221145;
  xx[24] = xx[3] * xx[16] - xx[10] * xx[14];
  xx[25] = xx[10] * xx[15];
  xx[26] = xx[24];
  xx[27] = xx[9];
  pm_math_Vector3_cross_ra(xx + 14, xx + 25, xx + 28);
  xx[9] = xx[3] + xx[2] * (xx[28] - xx[10] * xx[12]);
  xx[3] = xx[9] * xx[11];
  xx[12] = xx[2] * (xx[29] - xx[13] * xx[24]);
  xx[24] = xx[11] * xx[12];
  xx[25] = xx[10] + (xx[13] * xx[4] + xx[30]) * xx[2];
  xx[4] = xx[25] * xx[11];
  xx[26] = xx[3];
  xx[27] = xx[24];
  xx[28] = xx[4];
  xx[10] = pm_math_Vector3_dot_ra(xx + 18, xx + 26);
  xx[29] = (xx[13] * xx[16] + xx[14] * xx[15]) * xx[2];
  xx[30] = - (xx[11] * xx[29]);
  xx[31] = 1.0;
  xx[32] = (xx[16] * xx[16] + xx[14] * xx[14]) * xx[2] - xx[31];
  xx[33] = xx[32] * xx[11];
  xx[34] = xx[2] * (xx[13] * xx[14] - xx[15] * xx[16]);
  xx[35] = xx[11] * xx[34];
  xx[36] = xx[30];
  xx[37] = xx[33];
  xx[38] = xx[35];
  xx[39] = pm_math_Vector3_dot_ra(xx + 18, xx + 36);
  xx[40] = 1.570796326794897e-3;
  xx[41] = - (xx[40] * xx[1]);
  xx[1] = xx[17] * xx[40];
  xx[17] = 0.0;
  xx[42] = xx[9];
  xx[43] = xx[12];
  xx[44] = xx[25];
  xx[25] = pm_math_Vector3_dot_ra(xx + 42, xx + 36);
  xx[45] = - (xx[40] * xx[12]);
  xx[12] = xx[9] * xx[40];
  xx[46] = - xx[29];
  xx[47] = xx[32];
  xx[48] = xx[34];
  xx[9] = - (xx[32] * xx[40]);
  xx[32] = - (xx[40] * xx[29]);
  xx[29] = 5.415974835094905e-5;
  xx[34] = 2.454369260617026e-5;
  xx[49] = pm_math_Vector3_dot_ra(xx + 18, xx + 21);
  xx[50] = xx[10];
  xx[51] = xx[39];
  xx[52] = xx[41];
  xx[53] = xx[1];
  xx[54] = xx[17];
  xx[55] = xx[10];
  xx[56] = pm_math_Vector3_dot_ra(xx + 42, xx + 26);
  xx[57] = xx[25];
  xx[58] = xx[45];
  xx[59] = xx[12];
  xx[60] = xx[17];
  xx[61] = xx[39];
  xx[62] = xx[25];
  xx[63] = pm_math_Vector3_dot_ra(xx + 46, xx + 36);
  xx[64] = xx[9];
  xx[65] = xx[32];
  xx[66] = xx[17];
  xx[67] = xx[41];
  xx[68] = xx[45];
  xx[69] = xx[9];
  xx[70] = xx[29];
  xx[71] = xx[17];
  xx[72] = xx[17];
  xx[73] = xx[1];
  xx[74] = xx[12];
  xx[75] = xx[32];
  xx[76] = xx[17];
  xx[77] = xx[29];
  xx[78] = xx[17];
  xx[79] = xx[17];
  xx[80] = xx[17];
  xx[81] = xx[17];
  xx[82] = xx[17];
  xx[83] = xx[17];
  xx[84] = xx[34];
  ii[0] = factorSymmetricPosDef(xx + 49, 6, xx + 85);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'V1_2P/Body Configuration/SixDOF1' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[25] = state[12];
  xx[26] = state[13];
  xx[27] = state[14];
  xx[36] = state[9];
  xx[37] = state[10];
  xx[38] = state[11];
  pm_math_Quaternion_xform_ra(xx + 5, xx + 36, xx + 85);
  pm_math_Quaternion_inverseXform_ra(xx + 13, xx + 85, xx + 36);
  xx[1] = 0.02;
  xx[85] = xx[36] + xx[1] * state[13];
  xx[86] = xx[37] - xx[1] * state[12];
  xx[87] = xx[38];
  pm_math_Vector3_cross_ra(xx + 25, xx + 85, xx + 88);
  xx[85] = - xx[36];
  xx[86] = - xx[37];
  xx[87] = - xx[38];
  pm_math_Vector3_cross_ra(xx + 25, xx + 85, xx + 36);
  xx[9] = 0.9238795325112867;
  xx[10] = 0.3826834323650898;
  xx[12] = xx[10] * input[59];
  xx[28] = xx[10] * input[58];
  xx[29] = input[58] - (xx[9] * xx[12] + xx[10] * xx[28]) * xx[2];
  xx[32] = xx[10] * input[71];
  xx[39] = xx[10] * input[70];
  xx[41] = input[70] - (xx[9] * xx[32] + xx[10] * xx[39]) * xx[2];
  xx[45] = xx[10] * input[83];
  xx[85] = xx[10] * input[82];
  xx[86] = input[82] - (xx[9] * xx[45] + xx[10] * xx[85]) * xx[2];
  xx[87] = xx[10] * input[95];
  xx[91] = xx[10] * input[94];
  xx[92] = input[94] - (xx[9] * xx[87] + xx[10] * xx[91]) * xx[2];
  xx[93] = xx[10] * input[155];
  xx[94] = xx[10] * input[154];
  xx[95] = input[154] - (xx[9] * xx[93] + xx[10] * xx[94]) * xx[2];
  xx[96] = xx[10] * input[167];
  xx[97] = xx[10] * input[166];
  xx[98] = input[166] - (xx[9] * xx[96] + xx[10] * xx[97]) * xx[2];
  xx[99] = xx[10] * input[179];
  xx[100] = xx[10] * input[178];
  xx[101] = input[178] - (xx[9] * xx[99] + xx[10] * xx[100]) * xx[2];
  xx[102] = xx[10] * input[191];
  xx[103] = xx[10] * input[190];
  xx[104] = input[190] - (xx[9] * xx[102] + xx[10] * xx[103]) * xx[2];
  xx[105] = xx[10] * input[251];
  xx[106] = xx[10] * input[250];
  xx[107] = input[250] - (xx[9] * xx[105] + xx[10] * xx[106]) * xx[2];
  xx[108] = xx[10] * input[263];
  xx[109] = xx[10] * input[262];
  xx[110] = input[262] - (xx[9] * xx[108] + xx[10] * xx[109]) * xx[2];
  xx[111] = xx[10] * input[275];
  xx[112] = xx[10] * input[274];
  xx[113] = input[274] - (xx[9] * xx[111] + xx[10] * xx[112]) * xx[2];
  xx[114] = xx[10] * input[287];
  xx[115] = xx[10] * input[286];
  xx[116] = input[286] - (xx[9] * xx[114] + xx[10] * xx[115]) * xx[2];
  xx[117] = xx[10] * input[347];
  xx[118] = xx[10] * input[346];
  xx[119] = input[346] - (xx[9] * xx[117] + xx[10] * xx[118]) * xx[2];
  xx[120] = xx[10] * input[359];
  xx[121] = xx[10] * input[358];
  xx[122] = input[358] - (xx[9] * xx[120] + xx[10] * xx[121]) * xx[2];
  xx[123] = xx[10] * input[371];
  xx[124] = xx[10] * input[370];
  xx[125] = input[370] - (xx[9] * xx[123] + xx[10] * xx[124]) * xx[2];
  xx[126] = xx[10] * input[383];
  xx[127] = xx[10] * input[382];
  xx[128] = input[382] - (xx[9] * xx[126] + xx[10] * xx[127]) * xx[2];
  xx[129] = xx[10] * input[442];
  xx[130] = xx[10] * input[443];
  xx[131] = input[442] - (xx[10] * xx[129] - xx[9] * xx[130]) * xx[2];
  xx[132] = xx[10] * input[454];
  xx[133] = xx[10] * input[455];
  xx[134] = input[454] - (xx[10] * xx[132] - xx[9] * xx[133]) * xx[2];
  xx[135] = xx[10] * input[466];
  xx[136] = xx[10] * input[467];
  xx[137] = input[466] - (xx[10] * xx[135] - xx[9] * xx[136]) * xx[2];
  xx[138] = xx[10] * input[478];
  xx[139] = xx[10] * input[479];
  xx[140] = input[478] - (xx[10] * xx[138] - xx[9] * xx[139]) * xx[2];
  xx[141] = xx[10] * input[490];
  xx[142] = xx[10] * input[491];
  xx[143] = input[490] - (xx[10] * xx[141] - xx[9] * xx[142]) * xx[2];
  xx[144] = xx[10] * input[502];
  xx[145] = xx[10] * input[503];
  xx[146] = input[502] - (xx[10] * xx[144] - xx[9] * xx[145]) * xx[2];
  xx[147] = xx[10] * input[514];
  xx[148] = xx[10] * input[515];
  xx[149] = input[514] - (xx[10] * xx[147] - xx[9] * xx[148]) * xx[2];
  xx[150] = xx[10] * input[526];
  xx[151] = xx[10] * input[527];
  xx[152] = input[526] - (xx[10] * xx[150] - xx[9] * xx[151]) * xx[2];
  xx[153] = xx[10] * input[538];
  xx[154] = xx[10] * input[539];
  xx[155] = input[538] - (xx[10] * xx[153] - xx[9] * xx[154]) * xx[2];
  xx[156] = xx[10] * input[550];
  xx[157] = xx[10] * input[551];
  xx[158] = input[550] - (xx[10] * xx[156] - xx[9] * xx[157]) * xx[2];
  xx[159] = xx[10] * input[562];
  xx[160] = xx[10] * input[563];
  xx[161] = input[562] - (xx[10] * xx[159] - xx[9] * xx[160]) * xx[2];
  xx[162] = xx[10] * input[574];
  xx[163] = xx[10] * input[575];
  xx[164] = input[574] - (xx[10] * xx[162] - xx[9] * xx[163]) * xx[2];
  xx[165] = xx[10] * input[586];
  xx[166] = xx[10] * input[587];
  xx[167] = input[586] - (xx[10] * xx[165] - xx[9] * xx[166]) * xx[2];
  xx[168] = xx[10] * input[598];
  xx[169] = xx[10] * input[599];
  xx[170] = input[598] - (xx[10] * xx[168] - xx[9] * xx[169]) * xx[2];
  xx[171] = xx[10] * input[610];
  xx[172] = xx[10] * input[611];
  xx[173] = input[610] - (xx[10] * xx[171] - xx[9] * xx[172]) * xx[2];
  xx[174] = xx[10] * input[622];
  xx[175] = xx[10] * input[623];
  xx[176] = input[622] - (xx[10] * xx[174] - xx[9] * xx[175]) * xx[2];
  xx[177] = xx[10] * input[634];
  xx[178] = xx[10] * input[635];
  xx[179] = input[634] - (xx[10] * xx[177] - xx[9] * xx[178]) * xx[2];
  xx[180] = xx[10] * input[646];
  xx[181] = xx[10] * input[647];
  xx[182] = input[646] - (xx[10] * xx[180] - xx[9] * xx[181]) * xx[2];
  xx[183] = xx[10] * input[658];
  xx[184] = xx[10] * input[659];
  xx[185] = input[658] - (xx[10] * xx[183] - xx[9] * xx[184]) * xx[2];
  xx[186] = xx[10] * input[670];
  xx[187] = xx[10] * input[671];
  xx[188] = input[670] - (xx[10] * xx[186] - xx[9] * xx[187]) * xx[2];
  xx[189] = xx[10] * input[682];
  xx[190] = xx[10] * input[683];
  xx[191] = input[682] - (xx[10] * xx[189] - xx[9] * xx[190]) * xx[2];
  xx[192] = xx[10] * input[694];
  xx[193] = xx[10] * input[695];
  xx[194] = input[694] - (xx[10] * xx[192] - xx[9] * xx[193]) * xx[2];
  xx[195] = xx[10] * input[706];
  xx[196] = xx[10] * input[707];
  xx[197] = input[706] - (xx[10] * xx[195] - xx[9] * xx[196]) * xx[2];
  xx[198] = xx[10] * input[718];
  xx[199] = xx[10] * input[719];
  xx[200] = input[718] - (xx[10] * xx[198] - xx[9] * xx[199]) * xx[2];
  xx[201] = xx[10] * input[730];
  xx[202] = xx[10] * input[731];
  xx[203] = input[730] - (xx[10] * xx[201] - xx[9] * xx[202]) * xx[2];
  xx[204] = xx[10] * input[742];
  xx[205] = xx[10] * input[743];
  xx[206] = input[742] - (xx[10] * xx[204] - xx[9] * xx[205]) * xx[2];
  xx[207] = xx[10] * input[754];
  xx[208] = xx[10] * input[755];
  xx[209] = input[754] - (xx[10] * xx[207] - xx[9] * xx[208]) * xx[2];
  xx[210] = xx[10] * input[766];
  xx[211] = xx[10] * input[767];
  xx[212] = input[766] - (xx[10] * xx[210] - xx[9] * xx[211]) * xx[2];
  xx[213] = xx[10] * input[778];
  xx[214] = xx[10] * input[779];
  xx[215] = input[778] - (xx[10] * xx[213] - xx[9] * xx[214]) * xx[2];
  xx[216] = xx[10] * input[790];
  xx[217] = xx[10] * input[791];
  xx[218] = input[790] - (xx[10] * xx[216] - xx[9] * xx[217]) * xx[2];
  xx[219] = xx[10] * input[802];
  xx[220] = xx[10] * input[803];
  xx[221] = input[802] - (xx[10] * xx[219] - xx[9] * xx[220]) * xx[2];
  xx[222] = xx[10] * input[814];
  xx[223] = xx[10] * input[815];
  xx[224] = input[814] - (xx[10] * xx[222] - xx[9] * xx[223]) * xx[2];
  xx[225] = 9.999999999999998e-3;
  xx[226] = xx[225] * xx[15];
  xx[227] = xx[225] * xx[14];
  xx[228] = xx[1] * state[7];
  xx[229] = xx[1] * state[6];
  xx[230] = state[2] + (xx[228] * state[5] + xx[229] * state[8]) * xx[2];
  xx[231] = state[3] - xx[2] * (xx[229] * state[5] - xx[228] * state[8]);
  xx[232] = state[4] - (xx[229] * state[6] + xx[228] * state[7]) * xx[2] + xx[1];
  pm_math_Quaternion_xform_ra(xx + 5, xx + 230, xx + 233);
  xx[5] = state[30] + 0.3614478146933225;
  xx[6] = (xx[13] * xx[226] + xx[16] * xx[227]) * xx[2] + xx[233] - xx[5] +
    0.3604478146933178;
  xx[7] = 0.6183004150307611 - state[32];
  xx[8] = xx[2] * (xx[16] * xx[226] - xx[13] * xx[227]) + xx[234] - xx[7] +
    0.6464504150307565;
  xx[228] = 0.719163989234851 - state[28];
  xx[229] = xx[235] - (xx[14] * xx[227] + xx[15] * xx[226]) * xx[2] - xx[228] +
    0.6631639892349405;
  xx[226] = sqrt(xx[6] * xx[6] + xx[8] * xx[8] + xx[229] * xx[229]);
  if (xx[226] == 0.0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:ForceUndefined",
      "'V1_2P/Suction/Internal Force1' force is undefined since base and follower origins are coincident.",
      neDiagMgr);
  }

  xx[226] = input[965] / xx[226];
  xx[230] = xx[226] * xx[6];
  xx[231] = xx[226] * xx[8];
  xx[232] = xx[226] * xx[229];
  pm_math_Quaternion_inverseXform_ra(xx + 13, xx + 230, xx + 233);
  xx[6] = (xx[88] + xx[36]) * xx[11] - (xx[29] + xx[41] + xx[86] + xx[92] + xx
    [95] + xx[98] + xx[101] + xx[104] + xx[107] + xx[110] + xx[113] + xx[116] +
    xx[119] + xx[122] + xx[125] + xx[128] + xx[131] + xx[134] + xx[137] + xx[140]
    + xx[143] + xx[146] + xx[149] + xx[152] + xx[155] + xx[158] + xx[161] + xx
    [164] + xx[167] + xx[170] + xx[173] + xx[176] + xx[179] + xx[182] + xx[185]
    + xx[188] + xx[191] + xx[194] + xx[197] + xx[200] + xx[203] + xx[206] + xx
    [209] + xx[212] + xx[215] + xx[218] + xx[221] + xx[224] + input[838] +
    input[862] + input[922] + input[934] + input[946] + input[958] + xx[233]);
  xx[8] = input[59] + xx[2] * (xx[9] * xx[28] - xx[10] * xx[12]);
  xx[12] = input[71] + xx[2] * (xx[9] * xx[39] - xx[10] * xx[32]);
  xx[28] = input[83] + xx[2] * (xx[9] * xx[85] - xx[10] * xx[45]);
  xx[32] = input[95] + xx[2] * (xx[9] * xx[91] - xx[10] * xx[87]);
  xx[39] = input[155] + xx[2] * (xx[9] * xx[94] - xx[10] * xx[93]);
  xx[45] = input[167] + xx[2] * (xx[9] * xx[97] - xx[10] * xx[96]);
  xx[85] = input[179] + xx[2] * (xx[9] * xx[100] - xx[10] * xx[99]);
  xx[87] = input[191] + xx[2] * (xx[9] * xx[103] - xx[10] * xx[102]);
  xx[91] = input[251] + xx[2] * (xx[9] * xx[106] - xx[10] * xx[105]);
  xx[93] = input[263] + xx[2] * (xx[9] * xx[109] - xx[10] * xx[108]);
  xx[94] = input[275] + xx[2] * (xx[9] * xx[112] - xx[10] * xx[111]);
  xx[96] = input[287] + xx[2] * (xx[9] * xx[115] - xx[10] * xx[114]);
  xx[97] = input[347] + xx[2] * (xx[9] * xx[118] - xx[10] * xx[117]);
  xx[99] = input[359] + xx[2] * (xx[9] * xx[121] - xx[10] * xx[120]);
  xx[100] = input[371] + xx[2] * (xx[9] * xx[124] - xx[10] * xx[123]);
  xx[102] = input[383] + xx[2] * (xx[9] * xx[127] - xx[10] * xx[126]);
  xx[103] = input[443] - xx[2] * (xx[9] * xx[129] + xx[10] * xx[130]);
  xx[105] = input[455] - xx[2] * (xx[9] * xx[132] + xx[10] * xx[133]);
  xx[106] = input[467] - xx[2] * (xx[9] * xx[135] + xx[10] * xx[136]);
  xx[108] = input[479] - xx[2] * (xx[9] * xx[138] + xx[10] * xx[139]);
  xx[109] = input[491] - xx[2] * (xx[9] * xx[141] + xx[10] * xx[142]);
  xx[111] = input[503] - xx[2] * (xx[9] * xx[144] + xx[10] * xx[145]);
  xx[112] = input[515] - xx[2] * (xx[9] * xx[147] + xx[10] * xx[148]);
  xx[114] = input[527] - xx[2] * (xx[9] * xx[150] + xx[10] * xx[151]);
  xx[115] = input[539] - xx[2] * (xx[9] * xx[153] + xx[10] * xx[154]);
  xx[117] = input[551] - xx[2] * (xx[9] * xx[156] + xx[10] * xx[157]);
  xx[118] = input[563] - xx[2] * (xx[9] * xx[159] + xx[10] * xx[160]);
  xx[120] = input[575] - xx[2] * (xx[9] * xx[162] + xx[10] * xx[163]);
  xx[121] = input[587] - xx[2] * (xx[9] * xx[165] + xx[10] * xx[166]);
  xx[123] = input[599] - xx[2] * (xx[9] * xx[168] + xx[10] * xx[169]);
  xx[124] = input[611] - xx[2] * (xx[9] * xx[171] + xx[10] * xx[172]);
  xx[126] = input[623] - xx[2] * (xx[9] * xx[174] + xx[10] * xx[175]);
  xx[127] = input[635] - xx[2] * (xx[9] * xx[177] + xx[10] * xx[178]);
  xx[129] = input[647] - xx[2] * (xx[9] * xx[180] + xx[10] * xx[181]);
  xx[130] = input[659] - xx[2] * (xx[9] * xx[183] + xx[10] * xx[184]);
  xx[132] = input[671] - xx[2] * (xx[9] * xx[186] + xx[10] * xx[187]);
  xx[133] = input[683] - xx[2] * (xx[9] * xx[189] + xx[10] * xx[190]);
  xx[135] = input[695] - xx[2] * (xx[9] * xx[192] + xx[10] * xx[193]);
  xx[136] = input[707] - xx[2] * (xx[9] * xx[195] + xx[10] * xx[196]);
  xx[138] = input[719] - xx[2] * (xx[9] * xx[198] + xx[10] * xx[199]);
  xx[139] = input[731] - xx[2] * (xx[9] * xx[201] + xx[10] * xx[202]);
  xx[141] = input[743] - xx[2] * (xx[9] * xx[204] + xx[10] * xx[205]);
  xx[142] = input[755] - xx[2] * (xx[9] * xx[207] + xx[10] * xx[208]);
  xx[144] = input[767] - xx[2] * (xx[9] * xx[210] + xx[10] * xx[211]);
  xx[145] = input[779] - xx[2] * (xx[9] * xx[213] + xx[10] * xx[214]);
  xx[147] = input[791] - xx[2] * (xx[9] * xx[216] + xx[10] * xx[217]);
  xx[148] = input[803] - xx[2] * (xx[9] * xx[219] + xx[10] * xx[220]);
  xx[150] = input[815] - xx[2] * (xx[9] * xx[222] + xx[10] * xx[223]);
  xx[151] = (xx[89] + xx[37]) * xx[11] - (xx[8] + xx[12] + xx[28] + xx[32] + xx
    [39] + xx[45] + xx[85] + xx[87] + xx[91] + xx[93] + xx[94] + xx[96] + xx[97]
    + xx[99] + xx[100] + xx[102] + xx[103] + xx[105] + xx[106] + xx[108] + xx
    [109] + xx[111] + xx[112] + xx[114] + xx[115] + xx[117] + xx[118] + xx[120]
    + xx[121] + xx[123] + xx[124] + xx[126] + xx[127] + xx[129] + xx[130] + xx
    [132] + xx[133] + xx[135] + xx[136] + xx[138] + xx[139] + xx[141] + xx[142]
    + xx[144] + xx[145] + xx[147] + xx[148] + xx[150] + input[839] + input[863]
    + input[923] + input[935] + input[947] + input[959] + xx[234]);
  xx[229] = xx[6];
  xx[230] = xx[151];
  xx[231] = (xx[90] + xx[38]) * xx[11] - (input[60] + input[72] + input[84] +
    input[96] + input[156] + input[168] + input[180] + input[192] + input[252] +
    input[264] + input[276] + input[288] + input[348] + input[360] + input[372]
    + input[384] + input[444] + input[456] + input[468] + input[480] + input[492]
    + input[504] + input[516] + input[528] + input[540] + input[552] + input[564]
    + input[576] + input[588] + input[600] + input[612] + input[624] + input[636]
    + input[648] + input[660] + input[672] + input[684] + input[696] + input[708]
    + input[720] + input[732] + input[744] + input[756] + input[768] + input[780]
    + input[792] + input[804] + input[816] + input[840] + input[864] + input[924]
    + input[936] + input[948] + input[960] + xx[235]);
  xx[11] = 2.274382181505112e-5;
  xx[36] = 2.274382181505111e-5;
  xx[88] = xx[11] * state[12];
  xx[89] = xx[36] * state[13];
  xx[90] = xx[34] * state[14];
  pm_math_Vector3_cross_ra(xx + 25, xx + 88, xx + 235);
  xx[25] = 0.02121320343559643;
  xx[26] = xx[10] * input[62];
  xx[27] = xx[10] * input[61];
  xx[37] = xx[10] * input[74];
  xx[38] = xx[10] * input[73];
  xx[88] = xx[10] * input[86];
  xx[89] = xx[10] * input[85];
  xx[90] = xx[10] * input[98];
  xx[153] = xx[10] * input[97];
  xx[154] = xx[10] * input[158];
  xx[156] = xx[10] * input[157];
  xx[157] = xx[10] * input[170];
  xx[159] = xx[10] * input[169];
  xx[160] = xx[10] * input[182];
  xx[162] = xx[10] * input[181];
  xx[163] = xx[10] * input[194];
  xx[165] = xx[10] * input[193];
  xx[166] = xx[10] * input[254];
  xx[168] = xx[10] * input[253];
  xx[169] = xx[10] * input[266];
  xx[171] = xx[10] * input[265];
  xx[172] = xx[10] * input[278];
  xx[174] = xx[10] * input[277];
  xx[175] = xx[10] * input[290];
  xx[177] = xx[10] * input[289];
  xx[178] = xx[10] * input[350];
  xx[180] = xx[10] * input[349];
  xx[181] = xx[10] * input[362];
  xx[183] = xx[10] * input[361];
  xx[184] = xx[10] * input[374];
  xx[186] = xx[10] * input[373];
  xx[187] = xx[10] * input[386];
  xx[189] = xx[10] * input[385];
  xx[190] = xx[10] * input[445];
  xx[192] = xx[10] * input[446];
  xx[193] = 0.0107;
  xx[195] = xx[10] * input[457];
  xx[196] = xx[10] * input[458];
  xx[198] = xx[10] * input[469];
  xx[199] = xx[10] * input[470];
  xx[201] = xx[10] * input[481];
  xx[202] = xx[10] * input[482];
  xx[204] = xx[10] * input[493];
  xx[205] = xx[10] * input[494];
  xx[207] = xx[10] * input[505];
  xx[208] = xx[10] * input[506];
  xx[210] = xx[10] * input[517];
  xx[211] = xx[10] * input[518];
  xx[213] = xx[10] * input[529];
  xx[214] = xx[10] * input[530];
  xx[216] = xx[10] * input[541];
  xx[217] = xx[10] * input[542];
  xx[219] = xx[10] * input[553];
  xx[220] = xx[10] * input[554];
  xx[222] = xx[10] * input[565];
  xx[223] = xx[10] * input[566];
  xx[227] = xx[10] * input[577];
  xx[232] = xx[10] * input[578];
  xx[238] = xx[10] * input[589];
  xx[239] = xx[10] * input[590];
  xx[240] = xx[10] * input[601];
  xx[241] = xx[10] * input[602];
  xx[242] = xx[10] * input[613];
  xx[243] = xx[10] * input[614];
  xx[244] = xx[10] * input[625];
  xx[245] = xx[10] * input[626];
  xx[246] = xx[10] * input[637];
  xx[247] = xx[10] * input[638];
  xx[248] = xx[10] * input[649];
  xx[249] = xx[10] * input[650];
  xx[250] = xx[10] * input[661];
  xx[251] = xx[10] * input[662];
  xx[252] = xx[10] * input[673];
  xx[253] = xx[10] * input[674];
  xx[254] = xx[10] * input[685];
  xx[255] = xx[10] * input[686];
  xx[256] = xx[10] * input[697];
  xx[257] = xx[10] * input[698];
  xx[258] = xx[10] * input[709];
  xx[259] = xx[10] * input[710];
  xx[260] = xx[10] * input[721];
  xx[261] = xx[10] * input[722];
  xx[262] = xx[10] * input[733];
  xx[263] = xx[10] * input[734];
  xx[264] = xx[10] * input[745];
  xx[265] = xx[10] * input[746];
  xx[266] = xx[10] * input[757];
  xx[267] = xx[10] * input[758];
  xx[268] = xx[10] * input[769];
  xx[269] = xx[10] * input[770];
  xx[270] = xx[10] * input[781];
  xx[271] = xx[10] * input[782];
  xx[272] = 9.299999999999999e-3;
  xx[273] = xx[10] * input[793];
  xx[274] = xx[10] * input[794];
  xx[275] = xx[10] * input[805];
  xx[276] = xx[10] * input[806];
  xx[277] = xx[10] * input[817];
  xx[278] = xx[10] * input[818];
  xx[279] = 0.03;
  xx[280] = - pm_math_Vector3_dot_ra(xx + 18, xx + 229);
  xx[281] = - pm_math_Vector3_dot_ra(xx + 42, xx + 229);
  xx[282] = - pm_math_Vector3_dot_ra(xx + 46, xx + 229);
  xx[283] = - (xx[235] - (xx[25] * input[60] + xx[8] * xx[1] + input[61] - (xx[9]
    * xx[26] + xx[10] * xx[27]) * xx[2]) - (input[73] - (xx[9] * xx[37] + xx[10]
    * xx[38]) * xx[2] + xx[12] * xx[1]) - (xx[28] * xx[1] - xx[25] * input[84] +
    input[85] - (xx[9] * xx[88] + xx[10] * xx[89]) * xx[2]) - (input[97] - (xx[9]
    * xx[90] + xx[10] * xx[153]) * xx[2] + xx[32] * xx[1]) - (xx[25] * input[156]
    + xx[39] * xx[1] + input[157] - (xx[9] * xx[154] + xx[10] * xx[156]) * xx[2])
               - (input[169] - (xx[9] * xx[157] + xx[10] * xx[159]) * xx[2] +
                  xx[45] * xx[1]) - (xx[85] * xx[1] - xx[25] * input[180] +
    input[181] - (xx[9] * xx[160] + xx[10] * xx[162]) * xx[2]) - (input[193] -
    (xx[9] * xx[163] + xx[10] * xx[165]) * xx[2] + xx[87] * xx[1]) - (xx[25] *
    input[252] + xx[91] * xx[1] + input[253] - (xx[9] * xx[166] + xx[10] * xx
    [168]) * xx[2]) - (input[265] - (xx[9] * xx[169] + xx[10] * xx[171]) * xx[2]
                       + xx[93] * xx[1]) - (xx[94] * xx[1] - xx[25] * input[276]
    + input[277] - (xx[9] * xx[172] + xx[10] * xx[174]) * xx[2]) - (input[289] -
    (xx[9] * xx[175] + xx[10] * xx[177]) * xx[2] + xx[96] * xx[1]) - (xx[25] *
    input[348] + xx[97] * xx[1] + input[349] - (xx[9] * xx[178] + xx[10] * xx
    [180]) * xx[2]) - (input[361] - (xx[9] * xx[181] + xx[10] * xx[183]) * xx[2]
                       + xx[99] * xx[1]) - (xx[100] * xx[1] - xx[25] * input[372]
    + input[373] - (xx[9] * xx[184] + xx[10] * xx[186]) * xx[2]) - (input[385] -
    (xx[9] * xx[187] + xx[10] * xx[189]) * xx[2] + xx[102] * xx[1]) - (input[445]
    - (xx[10] * xx[190] - xx[9] * xx[192]) * xx[2] - xx[103] * xx[193]) -
               (input[457] - (xx[10] * xx[195] - xx[9] * xx[196]) * xx[2] - (xx
    [25] * input[456] + xx[105] * xx[193])) - (input[469] - (xx[10] * xx[198] -
    xx[9] * xx[199]) * xx[2] - xx[106] * xx[193]) - (xx[25] * input[480] - xx
    [108] * xx[193] + input[481] - (xx[10] * xx[201] - xx[9] * xx[202]) * xx[2])
               - (input[493] - (xx[10] * xx[204] - xx[9] * xx[205]) * xx[2] -
                  xx[109] * xx[193]) - (input[505] - (xx[10] * xx[207] - xx[9] *
    xx[208]) * xx[2] - (xx[25] * input[504] + xx[111] * xx[193])) - (input[517]
    - (xx[10] * xx[210] - xx[9] * xx[211]) * xx[2] - xx[112] * xx[193]) - (xx[25]
    * input[528] - xx[114] * xx[193] + input[529] - (xx[10] * xx[213] - xx[9] *
    xx[214]) * xx[2]) - (input[541] - (xx[10] * xx[216] - xx[9] * xx[217]) * xx
    [2] - xx[115] * xx[193]) - (input[553] - (xx[10] * xx[219] - xx[9] * xx[220])
    * xx[2] - (xx[25] * input[552] + xx[117] * xx[193])) - (input[565] - (xx[10]
    * xx[222] - xx[9] * xx[223]) * xx[2] - xx[118] * xx[193]) - (xx[25] * input
    [576] - xx[120] * xx[193] + input[577] - (xx[10] * xx[227] - xx[9] * xx[232])
    * xx[2]) - (input[589] - (xx[10] * xx[238] - xx[9] * xx[239]) * xx[2] - xx
                [121] * xx[193]) - (input[601] - (xx[10] * xx[240] - xx[9] * xx
    [241]) * xx[2] - (xx[25] * input[600] + xx[123] * xx[193])) - (input[613] -
    (xx[10] * xx[242] - xx[9] * xx[243]) * xx[2] - xx[124] * xx[193]) - (xx[25] *
    input[624] - xx[126] * xx[193] + input[625] - (xx[10] * xx[244] - xx[9] *
    xx[245]) * xx[2]) - (input[637] - (xx[10] * xx[246] - xx[9] * xx[247]) * xx
    [2] - xx[127] * xx[193]) - (input[649] - (xx[10] * xx[248] - xx[9] * xx[249])
    * xx[2] - (xx[25] * input[648] + xx[129] * xx[193])) - (input[661] - (xx[10]
    * xx[250] - xx[9] * xx[251]) * xx[2] - xx[130] * xx[193]) - (xx[25] * input
    [672] - xx[132] * xx[193] + input[673] - (xx[10] * xx[252] - xx[9] * xx[253])
    * xx[2]) - (input[685] - (xx[10] * xx[254] - xx[9] * xx[255]) * xx[2] - xx
                [133] * xx[193]) - (input[697] - (xx[10] * xx[256] - xx[9] * xx
    [257]) * xx[2] - (xx[25] * input[696] + xx[135] * xx[193])) - (input[709] -
    (xx[10] * xx[258] - xx[9] * xx[259]) * xx[2] - xx[136] * xx[193]) - (xx[25] *
    input[720] - xx[138] * xx[193] + input[721] - (xx[10] * xx[260] - xx[9] *
    xx[261]) * xx[2]) - (input[733] - (xx[10] * xx[262] - xx[9] * xx[263]) * xx
    [2] - xx[139] * xx[193]) - (input[745] - (xx[10] * xx[264] - xx[9] * xx[265])
    * xx[2] - (xx[25] * input[744] + xx[141] * xx[193])) - (input[757] - (xx[10]
    * xx[266] - xx[9] * xx[267]) * xx[2] - xx[142] * xx[193]) - (xx[25] * input
    [768] - xx[144] * xx[193] + input[769] - (xx[10] * xx[268] - xx[9] * xx[269])
    * xx[2]) - (input[781] - (xx[10] * xx[270] - xx[9] * xx[271]) * xx[2] + xx
                [145] * xx[272]) - (xx[147] * xx[272] - xx[25] * input[792] +
    input[793] - (xx[10] * xx[273] - xx[9] * xx[274]) * xx[2]) - (input[805] -
    (xx[10] * xx[275] - xx[9] * xx[276]) * xx[2] + xx[148] * xx[272]) - (xx[25] *
    input[816] + xx[150] * xx[272] + input[817] - (xx[10] * xx[277] - xx[9] *
    xx[278]) * xx[2]) - input[841] - input[865] - (input[925] + xx[279] * input
    [923]) - (input[937] + xx[279] * input[935]) - (input[949] + xx[279] *
    input[947]) - (input[961] + xx[279] * input[959]) + xx[225] * xx[234] - xx[1]
               * xx[151]);
  xx[284] = - (xx[236] - (input[62] + xx[2] * (xx[9] * xx[27] - xx[10] * xx[26])
    - xx[1] * xx[29]) - (input[74] + xx[2] * (xx[9] * xx[38] - xx[10] * xx[37])
    - (xx[1] * xx[41] + xx[25] * input[72])) - (input[86] + xx[2] * (xx[9] * xx
    [89] - xx[10] * xx[88]) - xx[1] * xx[86]) - (xx[25] * input[96] - xx[1] *
    xx[92] + input[98] + xx[2] * (xx[9] * xx[153] - xx[10] * xx[90])) - (input
    [158] + xx[2] * (xx[9] * xx[156] - xx[10] * xx[154]) - xx[1] * xx[95]) -
               (input[170] + xx[2] * (xx[9] * xx[159] - xx[10] * xx[157]) - (xx
    [1] * xx[98] + xx[25] * input[168])) - (input[182] + xx[2] * (xx[9] * xx[162]
    - xx[10] * xx[160]) - xx[1] * xx[101]) - (xx[25] * input[192] - xx[1] * xx
    [104] + input[194] + xx[2] * (xx[9] * xx[165] - xx[10] * xx[163])) - (input
    [254] + xx[2] * (xx[9] * xx[168] - xx[10] * xx[166]) - xx[1] * xx[107]) -
               (input[266] + xx[2] * (xx[9] * xx[171] - xx[10] * xx[169]) - (xx
    [1] * xx[110] + xx[25] * input[264])) - (input[278] + xx[2] * (xx[9] * xx
    [174] - xx[10] * xx[172]) - xx[1] * xx[113]) - (xx[25] * input[288] - xx[1] *
    xx[116] + input[290] + xx[2] * (xx[9] * xx[177] - xx[10] * xx[175])) -
               (input[350] + xx[2] * (xx[9] * xx[180] - xx[10] * xx[178]) - xx[1]
                * xx[119]) - (input[362] + xx[2] * (xx[9] * xx[183] - xx[10] *
    xx[181]) - (xx[1] * xx[122] + xx[25] * input[360])) - (input[374] + xx[2] *
    (xx[9] * xx[186] - xx[10] * xx[184]) - xx[1] * xx[125]) - (xx[25] * input
    [384] - xx[1] * xx[128] + input[386] + xx[2] * (xx[9] * xx[189] - xx[10] *
    xx[187])) - (xx[193] * xx[131] - xx[25] * input[444] + input[446] - xx[2] *
                 (xx[9] * xx[190] + xx[10] * xx[192])) - (xx[193] * xx[134] +
    input[458] - xx[2] * (xx[9] * xx[195] + xx[10] * xx[196])) - (xx[193] * xx
    [137] + xx[25] * input[468] + input[470] - xx[2] * (xx[9] * xx[198] + xx[10]
    * xx[199])) - (xx[193] * xx[140] + input[482] - xx[2] * (xx[9] * xx[201] +
    xx[10] * xx[202])) - (xx[193] * xx[143] - xx[25] * input[492] + input[494] -
    xx[2] * (xx[9] * xx[204] + xx[10] * xx[205])) - (xx[193] * xx[146] + input
    [506] - xx[2] * (xx[9] * xx[207] + xx[10] * xx[208])) - (xx[193] * xx[149] +
    xx[25] * input[516] + input[518] - xx[2] * (xx[9] * xx[210] + xx[10] * xx
    [211])) - (xx[193] * xx[152] + input[530] - xx[2] * (xx[9] * xx[213] + xx[10]
    * xx[214])) - (xx[193] * xx[155] - xx[25] * input[540] + input[542] - xx[2] *
                   (xx[9] * xx[216] + xx[10] * xx[217])) - (xx[193] * xx[158] +
    input[554] - xx[2] * (xx[9] * xx[219] + xx[10] * xx[220])) - (xx[193] * xx
    [161] + xx[25] * input[564] + input[566] - xx[2] * (xx[9] * xx[222] + xx[10]
    * xx[223])) - (xx[193] * xx[164] + input[578] - xx[2] * (xx[9] * xx[227] +
    xx[10] * xx[232])) - (xx[193] * xx[167] - xx[25] * input[588] + input[590] -
    xx[2] * (xx[9] * xx[238] + xx[10] * xx[239])) - (xx[193] * xx[170] + input
    [602] - xx[2] * (xx[9] * xx[240] + xx[10] * xx[241])) - (xx[193] * xx[173] +
    xx[25] * input[612] + input[614] - xx[2] * (xx[9] * xx[242] + xx[10] * xx
    [243])) - (xx[193] * xx[176] + input[626] - xx[2] * (xx[9] * xx[244] + xx[10]
    * xx[245])) - (xx[193] * xx[179] - xx[25] * input[636] + input[638] - xx[2] *
                   (xx[9] * xx[246] + xx[10] * xx[247])) - (xx[193] * xx[182] +
    input[650] - xx[2] * (xx[9] * xx[248] + xx[10] * xx[249])) - (xx[193] * xx
    [185] + xx[25] * input[660] + input[662] - xx[2] * (xx[9] * xx[250] + xx[10]
    * xx[251])) - (xx[193] * xx[188] + input[674] - xx[2] * (xx[9] * xx[252] +
    xx[10] * xx[253])) - (xx[193] * xx[191] - xx[25] * input[684] + input[686] -
    xx[2] * (xx[9] * xx[254] + xx[10] * xx[255])) - (xx[193] * xx[194] + input
    [698] - xx[2] * (xx[9] * xx[256] + xx[10] * xx[257])) - (xx[193] * xx[197] +
    xx[25] * input[708] + input[710] - xx[2] * (xx[9] * xx[258] + xx[10] * xx
    [259])) - (xx[193] * xx[200] + input[722] - xx[2] * (xx[9] * xx[260] + xx[10]
    * xx[261])) - (xx[193] * xx[203] - xx[25] * input[732] + input[734] - xx[2] *
                   (xx[9] * xx[262] + xx[10] * xx[263])) - (xx[193] * xx[206] +
    input[746] - xx[2] * (xx[9] * xx[264] + xx[10] * xx[265])) - (xx[193] * xx
    [209] + xx[25] * input[756] + input[758] - xx[2] * (xx[9] * xx[266] + xx[10]
    * xx[267])) - (xx[193] * xx[212] + input[770] - xx[2] * (xx[9] * xx[268] +
    xx[10] * xx[269])) - (input[782] - xx[2] * (xx[9] * xx[270] + xx[10] * xx
    [271]) - (xx[272] * xx[215] + xx[25] * input[780])) - (input[794] - xx[2] *
    (xx[9] * xx[273] + xx[10] * xx[274]) - xx[272] * xx[218]) - (xx[25] * input
    [804] - xx[272] * xx[221] + input[806] - xx[2] * (xx[9] * xx[275] + xx[10] *
    xx[276])) - (input[818] - xx[2] * (xx[9] * xx[277] + xx[10] * xx[278]) - xx
                 [272] * xx[224]) - input[842] - input[866] - (input[926] - xx
    [279] * input[922]) - (input[938] - xx[279] * input[934]) - (input[950] -
    xx[279] * input[946]) - (input[962] - xx[279] * input[958]) - xx[225] * xx
               [233] + xx[1] * xx[6]);
  xx[285] = input[963] - (xx[237] - (input[63] - xx[25] * xx[29]) - (xx[12] *
    xx[25] + input[75]) - (input[87] + xx[25] * xx[86]) - (input[99] - xx[32] *
    xx[25]) - (input[159] - xx[25] * xx[95]) - (xx[45] * xx[25] + input[171]) -
    (input[183] + xx[25] * xx[101]) - (input[195] - xx[87] * xx[25]) - (input
    [255] - xx[25] * xx[107]) - (xx[93] * xx[25] + input[267]) - (input[279] +
    xx[25] * xx[113]) - (input[291] - xx[96] * xx[25]) - (input[351] - xx[25] *
    xx[119]) - (xx[99] * xx[25] + input[363]) - (input[375] + xx[25] * xx[125])
    - (input[387] - xx[102] * xx[25]) - (xx[103] * xx[25] + input[447]) -
    (input[459] + xx[25] * xx[134]) - (input[471] - xx[106] * xx[25]) - (input
    [483] - xx[25] * xx[140]) - (xx[109] * xx[25] + input[495]) - (input[507] +
    xx[25] * xx[146]) - (input[519] - xx[112] * xx[25]) - (input[531] - xx[25] *
    xx[152]) - (xx[115] * xx[25] + input[543]) - (input[555] + xx[25] * xx[158])
    - (input[567] - xx[118] * xx[25]) - (input[579] - xx[25] * xx[164]) - (xx
    [121] * xx[25] + input[591]) - (input[603] + xx[25] * xx[170]) - (input[615]
    - xx[124] * xx[25]) - (input[627] - xx[25] * xx[176]) - (xx[127] * xx[25] +
    input[639]) - (input[651] + xx[25] * xx[182]) - (input[663] - xx[130] * xx
    [25]) - (input[675] - xx[25] * xx[188]) - (xx[133] * xx[25] + input[687]) -
    (input[699] + xx[25] * xx[194]) - (input[711] - xx[136] * xx[25]) - (input
    [723] - xx[25] * xx[200]) - (xx[139] * xx[25] + input[735]) - (input[747] +
    xx[25] * xx[206]) - (input[759] - xx[142] * xx[25]) - (input[771] - xx[25] *
    xx[212]) - (xx[145] * xx[25] + input[783]) - (input[795] + xx[25] * xx[218])
    - (input[807] - xx[148] * xx[25]) - (input[819] - xx[25] * xx[224]) - input
    [843] - input[867] - input[927] - input[939] - input[951]);
  solveSymmetricPosDef(xx + 49, xx + 280, 6, 1, xx + 41, xx + 85);
  xx[85] = xx[17];
  xx[86] = xx[17];
  xx[87] = xx[17];
  xx[88] = xx[11];
  xx[89] = xx[17];
  xx[90] = xx[17];
  xx[91] = xx[17];
  xx[92] = xx[17];
  xx[93] = xx[17];
  xx[94] = xx[17];
  xx[95] = xx[36];
  xx[96] = xx[17];
  xx[97] = xx[17];
  xx[98] = xx[17];
  xx[99] = xx[17];
  xx[100] = xx[17];
  xx[101] = xx[17];
  xx[102] = xx[34];
  xx[103] = xx[21];
  xx[104] = xx[3];
  xx[105] = xx[30];
  xx[106] = xx[17];
  xx[107] = xx[40];
  xx[108] = xx[17];
  xx[109] = xx[22];
  xx[110] = xx[24];
  xx[111] = xx[33];
  xx[112] = - xx[40];
  xx[113] = xx[17];
  xx[114] = xx[17];
  xx[115] = xx[23];
  xx[116] = xx[4];
  xx[117] = xx[35];
  xx[118] = xx[17];
  xx[119] = xx[17];
  xx[120] = xx[17];
  solveSymmetricPosDef(xx + 49, xx + 85, 6, 6, xx + 121, xx + 18);
  xx[8] = xx[139];
  xx[9] = xx[145];
  xx[10] = xx[151];
  xx[1] = 9.800000000000001;
  xx[3] = xx[1] * xx[16];
  xx[4] = xx[1] * xx[14];
  xx[18] = (xx[13] * xx[3] + xx[15] * xx[4]) * xx[2];
  xx[19] = xx[1] - (xx[16] * xx[3] + xx[14] * xx[4]) * xx[2];
  xx[20] = xx[2] * (xx[15] * xx[3] - xx[13] * xx[4]);
  xx[11] = xx[140];
  xx[12] = xx[146];
  xx[13] = xx[152];
  xx[14] = xx[141];
  xx[15] = xx[147];
  xx[16] = xx[153];
  xx[21] = xx[142];
  xx[22] = xx[148];
  xx[23] = xx[154];
  xx[24] = xx[143];
  xx[25] = xx[149];
  xx[26] = xx[155];
  xx[27] = xx[144];
  xx[28] = xx[150];
  xx[29] = xx[156];
  xx[3] = 0.44080001915388;
  xx[4] = - 0.5528972265384762;
  xx[32] = xx[3];
  xx[33] = - xx[3];
  xx[34] = xx[4];
  xx[35] = xx[4];
  xx[36] = - state[18];
  xx[37] = - state[19];
  xx[38] = - state[20];
  xx[39] = - state[21];
  pm_math_Quaternion_compose_ra(xx + 32, xx + 36, xx + 47);
  xx[3] = 0.9748684321931496;
  xx[4] = xx[3] * xx[49];
  xx[6] = 0.2227813724557561;
  xx[30] = xx[6] * xx[50] + xx[3] * xx[48];
  xx[36] = xx[6] * xx[49];
  xx[37] = xx[4];
  xx[38] = - xx[30];
  xx[39] = xx[36];
  pm_math_Vector3_cross_ra(xx + 48, xx + 37, xx + 51);
  xx[37] = xx[47] * xx[49];
  xx[38] = xx[2] * (xx[51] - xx[3] * xx[37]) - xx[6];
  xx[6] = xx[2] * (xx[52] + xx[47] * xx[30]);
  xx[30] = xx[3] + (xx[53] - xx[47] * xx[36]) * xx[2];
  xx[51] = xx[38];
  xx[52] = xx[6];
  xx[53] = xx[30];
  xx[36] = 0.09817477042468103;
  xx[39] = xx[38] * xx[36];
  xx[40] = xx[36] * xx[6];
  xx[54] = xx[30] * xx[36];
  xx[55] = xx[39];
  xx[56] = xx[40];
  xx[57] = xx[54];
  xx[30] = 0.2227813724557562;
  xx[58] = xx[3] * xx[50] - xx[30] * xx[48];
  xx[59] = xx[30] * xx[49];
  xx[60] = xx[58];
  xx[61] = - xx[4];
  pm_math_Vector3_cross_ra(xx + 48, xx + 59, xx + 62);
  xx[59] = xx[3] + xx[2] * (xx[62] - xx[30] * xx[37]);
  xx[3] = xx[59] * xx[36];
  xx[37] = xx[2] * (xx[63] - xx[47] * xx[58]);
  xx[58] = xx[36] * xx[37];
  xx[60] = xx[30] + (xx[47] * xx[4] + xx[64]) * xx[2];
  xx[4] = xx[60] * xx[36];
  xx[61] = xx[3];
  xx[62] = xx[58];
  xx[63] = xx[4];
  xx[30] = pm_math_Vector3_dot_ra(xx + 51, xx + 61);
  xx[64] = (xx[47] * xx[50] + xx[48] * xx[49]) * xx[2];
  xx[65] = xx[36] * xx[64];
  xx[66] = xx[31] - (xx[50] * xx[50] + xx[48] * xx[48]) * xx[2];
  xx[31] = xx[36] * xx[66];
  xx[67] = xx[2] * (xx[49] * xx[50] - xx[47] * xx[48]);
  xx[68] = xx[36] * xx[67];
  xx[69] = xx[65];
  xx[70] = xx[31];
  xx[71] = xx[68];
  xx[72] = pm_math_Vector3_dot_ra(xx + 51, xx + 69);
  xx[73] = 2.454369260617027e-3;
  xx[74] = - (xx[73] * xx[6]);
  xx[6] = xx[38] * xx[73];
  xx[75] = xx[59];
  xx[76] = xx[37];
  xx[77] = xx[60];
  xx[38] = pm_math_Vector3_dot_ra(xx + 75, xx + 69);
  xx[60] = - (xx[73] * xx[37]);
  xx[37] = xx[59] * xx[73];
  xx[78] = xx[64];
  xx[79] = xx[66];
  xx[80] = xx[67];
  xx[59] = - (xx[73] * xx[66]);
  xx[66] = xx[73] * xx[64];
  xx[64] = 9.715211656609066e-5;
  xx[67] = 3.067961575771283e-5;
  xx[81] = pm_math_Vector3_dot_ra(xx + 51, xx + 55);
  xx[82] = xx[30];
  xx[83] = xx[72];
  xx[84] = xx[74];
  xx[85] = xx[6];
  xx[86] = xx[17];
  xx[87] = xx[30];
  xx[88] = pm_math_Vector3_dot_ra(xx + 75, xx + 61);
  xx[89] = xx[38];
  xx[90] = xx[60];
  xx[91] = xx[37];
  xx[92] = xx[17];
  xx[93] = xx[72];
  xx[94] = xx[38];
  xx[95] = pm_math_Vector3_dot_ra(xx + 78, xx + 69);
  xx[96] = xx[59];
  xx[97] = xx[66];
  xx[98] = xx[17];
  xx[99] = xx[74];
  xx[100] = xx[60];
  xx[101] = xx[59];
  xx[102] = xx[64];
  xx[103] = xx[17];
  xx[104] = xx[17];
  xx[105] = xx[6];
  xx[106] = xx[37];
  xx[107] = xx[66];
  xx[108] = xx[17];
  xx[109] = xx[64];
  xx[110] = xx[17];
  xx[111] = xx[17];
  xx[112] = xx[17];
  xx[113] = xx[17];
  xx[114] = xx[17];
  xx[115] = xx[17];
  xx[116] = xx[67];
  ii[0] = factorSymmetricPosDef(xx + 81, 6, xx + 59);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'V1_2P/Body Configuration/SixDOF' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[55] = state[25];
  xx[56] = state[26];
  xx[57] = state[27];
  xx[59] = state[22];
  xx[60] = state[23];
  xx[61] = state[24];
  pm_math_Quaternion_xform_ra(xx + 32, xx + 59, xx + 62);
  pm_math_Quaternion_inverseXform_ra(xx + 47, xx + 62, xx + 59);
  xx[6] = 0.02500000000000001;
  xx[62] = xx[59] + xx[6] * state[26];
  xx[63] = xx[60] - xx[6] * state[25];
  xx[64] = xx[61];
  pm_math_Vector3_cross_ra(xx + 55, xx + 62, xx + 69);
  xx[62] = - xx[59];
  xx[63] = - xx[60];
  xx[64] = - xx[61];
  pm_math_Vector3_cross_ra(xx + 55, xx + 62, xx + 59);
  xx[30] = 0.03499999999999999;
  xx[37] = xx[30] * xx[49];
  xx[38] = xx[30] * xx[48];
  xx[62] = xx[6] * state[20];
  xx[63] = xx[6] * state[19];
  xx[117] = state[15] + (xx[62] * state[18] + xx[63] * state[21]) * xx[2];
  xx[118] = state[16] - xx[2] * (xx[63] * state[18] - xx[62] * state[21]);
  xx[119] = state[17] - (xx[63] * state[19] + xx[62] * state[20]) * xx[2] + xx[6];
  pm_math_Quaternion_xform_ra(xx + 32, xx + 117, xx + 62);
  xx[32] = (xx[47] * xx[37] + xx[50] * xx[38]) * xx[2] + xx[62] - xx[5] +
    0.360447814693318;
  xx[5] = xx[2] * (xx[50] * xx[37] - xx[47] * xx[38]) + xx[63] - xx[7] +
    0.5564504150307564;
  xx[7] = xx[64] - (xx[48] * xx[38] + xx[49] * xx[37]) * xx[2] - xx[228] +
    0.6881639892349409;
  xx[33] = sqrt(xx[32] * xx[32] + xx[5] * xx[5] + xx[7] * xx[7]);
  if (xx[33] == 0.0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:ForceUndefined",
      "'V1_2P/Suction/Internal Force' force is undefined since base and follower origins are coincident.",
      neDiagMgr);
  }

  xx[33] = input[964] / xx[33];
  xx[62] = xx[33] * xx[32];
  xx[63] = xx[33] * xx[5];
  xx[64] = xx[33] * xx[7];
  pm_math_Quaternion_inverseXform_ra(xx + 47, xx + 62, xx + 117);
  xx[5] = (xx[69] + xx[59]) * xx[36] - (input[10] + input[22] + input[34] +
    input[46] + input[106] + input[118] + input[130] + input[142] + input[202] +
    input[214] + input[226] + input[238] + input[298] + input[310] + input[322]
    + input[334] + input[394] + input[406] + input[418] + input[430] + input[484]
    + input[496] + input[508] + input[520] + input[826] + input[850] + input[874]
    + input[886] + input[898] + input[910] + xx[117]);
  xx[7] = (xx[70] + xx[60]) * xx[36] - (input[11] + input[23] + input[35] +
    input[47] + input[107] + input[119] + input[131] + input[143] + input[203] +
    input[215] + input[227] + input[239] + input[299] + input[311] + input[323]
    + input[335] + input[395] + input[407] + input[419] + input[431] + input[485]
    + input[497] + input[509] + input[521] + input[827] + input[851] + input[875]
    + input[887] + input[899] + input[911] + xx[118]);
  xx[62] = xx[5];
  xx[63] = xx[7];
  xx[64] = (xx[71] + xx[61]) * xx[36] - (input[12] + input[24] + input[36] +
    input[48] + input[108] + input[120] + input[132] + input[144] + input[204] +
    input[216] + input[228] + input[240] + input[300] + input[312] + input[324]
    + input[336] + input[396] + input[408] + input[420] + input[432] + input[486]
    + input[498] + input[510] + input[522] + input[828] + input[852] + input[876]
    + input[888] + input[900] + input[912] + xx[119]);
  xx[32] = 3.579288505066496e-5;
  xx[34] = xx[32] * state[25];
  xx[35] = xx[32] * state[26];
  xx[36] = xx[67] * state[27];
  pm_math_Vector3_cross_ra(xx + 55, xx + 34, xx + 59);
  xx[34] = 0.015;
  xx[35] = - 0.01600000000000001;
  xx[36] = xx[34];
  xx[37] = xx[34];
  xx[38] = xx[35];
  xx[55] = input[10];
  xx[56] = input[11];
  xx[57] = input[12];
  pm_math_Vector3_cross_ra(xx + 36, xx + 55, xx + 69);
  xx[55] = - xx[34];
  xx[119] = xx[34];
  xx[120] = xx[55];
  xx[121] = xx[35];
  xx[122] = input[22];
  xx[123] = input[23];
  xx[124] = input[24];
  pm_math_Vector3_cross_ra(xx + 119, xx + 122, xx + 125);
  xx[122] = xx[55];
  xx[123] = xx[55];
  xx[124] = xx[35];
  xx[128] = input[34];
  xx[129] = input[35];
  xx[130] = input[36];
  pm_math_Vector3_cross_ra(xx + 122, xx + 128, xx + 131);
  xx[128] = xx[55];
  xx[129] = xx[34];
  xx[130] = xx[35];
  xx[55] = input[46];
  xx[56] = input[47];
  xx[57] = input[48];
  pm_math_Vector3_cross_ra(xx + 128, xx + 55, xx + 134);
  xx[55] = input[106];
  xx[56] = input[107];
  xx[57] = input[108];
  pm_math_Vector3_cross_ra(xx + 36, xx + 55, xx + 137);
  xx[55] = input[118];
  xx[56] = input[119];
  xx[57] = input[120];
  pm_math_Vector3_cross_ra(xx + 119, xx + 55, xx + 140);
  xx[55] = input[130];
  xx[56] = input[131];
  xx[57] = input[132];
  pm_math_Vector3_cross_ra(xx + 122, xx + 55, xx + 143);
  xx[55] = input[142];
  xx[56] = input[143];
  xx[57] = input[144];
  pm_math_Vector3_cross_ra(xx + 128, xx + 55, xx + 146);
  xx[55] = input[202];
  xx[56] = input[203];
  xx[57] = input[204];
  pm_math_Vector3_cross_ra(xx + 36, xx + 55, xx + 149);
  xx[55] = input[214];
  xx[56] = input[215];
  xx[57] = input[216];
  pm_math_Vector3_cross_ra(xx + 119, xx + 55, xx + 152);
  xx[55] = input[226];
  xx[56] = input[227];
  xx[57] = input[228];
  pm_math_Vector3_cross_ra(xx + 122, xx + 55, xx + 155);
  xx[55] = input[238];
  xx[56] = input[239];
  xx[57] = input[240];
  pm_math_Vector3_cross_ra(xx + 128, xx + 55, xx + 158);
  xx[55] = input[298];
  xx[56] = input[299];
  xx[57] = input[300];
  pm_math_Vector3_cross_ra(xx + 36, xx + 55, xx + 161);
  xx[55] = input[310];
  xx[56] = input[311];
  xx[57] = input[312];
  pm_math_Vector3_cross_ra(xx + 119, xx + 55, xx + 164);
  xx[55] = input[322];
  xx[56] = input[323];
  xx[57] = input[324];
  pm_math_Vector3_cross_ra(xx + 122, xx + 55, xx + 167);
  xx[55] = input[334];
  xx[56] = input[335];
  xx[57] = input[336];
  pm_math_Vector3_cross_ra(xx + 128, xx + 55, xx + 170);
  xx[55] = input[394];
  xx[56] = input[395];
  xx[57] = input[396];
  pm_math_Vector3_cross_ra(xx + 36, xx + 55, xx + 173);
  xx[34] = input[406];
  xx[35] = input[407];
  xx[36] = input[408];
  pm_math_Vector3_cross_ra(xx + 119, xx + 34, xx + 55);
  xx[34] = input[418];
  xx[35] = input[419];
  xx[36] = input[420];
  pm_math_Vector3_cross_ra(xx + 122, xx + 34, xx + 119);
  xx[34] = input[430];
  xx[35] = input[431];
  xx[36] = input[432];
  pm_math_Vector3_cross_ra(xx + 128, xx + 34, xx + 122);
  xx[34] = 0.02600000000000001;
  xx[35] = 6.938893903907228e-18;
  xx[36] = 4.999999999999999e-3;
  xx[37] = - 5.000000000000011e-3;
  xx[128] = xx[36];
  xx[129] = xx[36];
  xx[130] = xx[37];
  xx[176] = input[874];
  xx[177] = input[875];
  xx[178] = input[876];
  pm_math_Vector3_cross_ra(xx + 128, xx + 176, xx + 179);
  xx[38] = - xx[36];
  xx[128] = xx[36];
  xx[129] = xx[38];
  xx[130] = xx[37];
  xx[176] = input[886];
  xx[177] = input[887];
  xx[178] = input[888];
  pm_math_Vector3_cross_ra(xx + 128, xx + 176, xx + 182);
  xx[128] = xx[38];
  xx[129] = xx[38];
  xx[130] = xx[37];
  xx[176] = input[898];
  xx[177] = input[899];
  xx[178] = input[900];
  pm_math_Vector3_cross_ra(xx + 128, xx + 176, xx + 185);
  xx[128] = xx[38];
  xx[129] = xx[36];
  xx[130] = xx[37];
  xx[36] = input[910];
  xx[37] = input[911];
  xx[38] = input[912];
  pm_math_Vector3_cross_ra(xx + 128, xx + 36, xx + 176);
  xx[188] = - pm_math_Vector3_dot_ra(xx + 51, xx + 62);
  xx[189] = - pm_math_Vector3_dot_ra(xx + 75, xx + 62);
  xx[190] = - pm_math_Vector3_dot_ra(xx + 78, xx + 62);
  xx[191] = - (xx[59] - (xx[69] + input[13]) - (xx[125] + input[25]) - (xx[131]
    + input[37]) - (xx[134] + input[49]) - (xx[137] + input[109]) - (xx[140] +
    input[121]) - (xx[143] + input[133]) - (xx[146] + input[145]) - (xx[149] +
    input[205]) - (xx[152] + input[217]) - (xx[155] + input[229]) - (xx[158] +
    input[241]) - (xx[161] + input[301]) - (xx[164] + input[313]) - (xx[167] +
    input[325]) - (xx[170] + input[337]) - (xx[173] + input[397]) - (xx[55] +
    input[409]) - (xx[119] + input[421]) - (xx[122] + input[433]) - (input[487]
    + xx[34] * input[485]) - (input[499] + xx[34] * input[497]) - (input[511] +
    xx[34] * input[509]) - (input[523] + xx[34] * input[521]) - (input[829] +
    xx[35] * input[827]) - (input[853] + xx[35] * input[851]) - (xx[179] +
    input[877]) - (xx[182] + input[889]) - (xx[185] + input[901]) - (xx[176] +
    input[913]) + xx[30] * xx[118] - xx[6] * xx[7]);
  xx[192] = - (xx[60] - (xx[70] + input[14]) - (xx[126] + input[26]) - (xx[132]
    + input[38]) - (xx[135] + input[50]) - (xx[138] + input[110]) - (xx[141] +
    input[122]) - (xx[144] + input[134]) - (xx[147] + input[146]) - (xx[150] +
    input[206]) - (xx[153] + input[218]) - (xx[156] + input[230]) - (xx[159] +
    input[242]) - (xx[162] + input[302]) - (xx[165] + input[314]) - (xx[168] +
    input[326]) - (xx[171] + input[338]) - (xx[174] + input[398]) - (xx[56] +
    input[410]) - (xx[120] + input[422]) - (xx[123] + input[434]) - (input[488]
    - xx[34] * input[484]) - (input[500] - xx[34] * input[496]) - (input[512] -
    xx[34] * input[508]) - (input[524] - xx[34] * input[520]) - (input[830] -
    xx[35] * input[826]) - (input[854] - xx[35] * input[850]) - (xx[180] +
    input[878]) - (xx[183] + input[890]) - (xx[186] + input[902]) - (xx[177] +
    input[914]) - xx[30] * xx[117] + xx[6] * xx[5]);
  xx[193] = xx[178] + input[915] - (xx[61] - (xx[71] + input[15]) - (xx[127] +
    input[27]) - (xx[133] + input[39]) - (xx[136] + input[51]) - (xx[139] +
    input[111]) - (xx[142] + input[123]) - (xx[145] + input[135]) - (xx[148] +
    input[147]) - (xx[151] + input[207]) - (xx[154] + input[219]) - (xx[157] +
    input[231]) - (xx[160] + input[243]) - (xx[163] + input[303]) - (xx[166] +
    input[315]) - (xx[169] + input[327]) - (xx[172] + input[339]) - (xx[175] +
    input[399]) - (xx[57] + input[411]) - (xx[121] + input[423]) - (xx[124] +
    input[435]) - input[489] - input[501] - input[513] - input[525] - input[831]
    - input[855] - (xx[181] + input[879]) - (xx[184] + input[891]) - (xx[187] +
    input[903]));
  solveSymmetricPosDef(xx + 81, xx + 188, 6, 1, xx + 59, xx + 74);
  xx[117] = xx[17];
  xx[118] = xx[17];
  xx[119] = xx[17];
  xx[120] = xx[32];
  xx[121] = xx[17];
  xx[122] = xx[17];
  xx[123] = xx[17];
  xx[124] = xx[17];
  xx[125] = xx[17];
  xx[126] = xx[17];
  xx[127] = xx[32];
  xx[128] = xx[17];
  xx[129] = xx[17];
  xx[130] = xx[17];
  xx[131] = xx[17];
  xx[132] = xx[17];
  xx[133] = xx[17];
  xx[134] = xx[67];
  xx[135] = xx[39];
  xx[136] = xx[3];
  xx[137] = xx[65];
  xx[138] = xx[17];
  xx[139] = xx[73];
  xx[140] = xx[17];
  xx[141] = xx[40];
  xx[142] = xx[58];
  xx[143] = xx[31];
  xx[144] = - xx[73];
  xx[145] = xx[17];
  xx[146] = xx[17];
  xx[147] = xx[54];
  xx[148] = xx[4];
  xx[149] = xx[68];
  xx[150] = xx[17];
  xx[151] = xx[17];
  xx[152] = xx[17];
  solveSymmetricPosDef(xx + 81, xx + 117, 6, 6, xx + 153, xx + 34);
  xx[3] = xx[171];
  xx[4] = xx[177];
  xx[5] = xx[183];
  xx[6] = xx[1] * xx[50];
  xx[7] = xx[1] * xx[48];
  xx[30] = (xx[47] * xx[6] + xx[49] * xx[7]) * xx[2];
  xx[31] = xx[1] - (xx[50] * xx[6] + xx[48] * xx[7]) * xx[2];
  xx[32] = xx[2] * (xx[49] * xx[6] - xx[47] * xx[7]);
  xx[34] = xx[172];
  xx[35] = xx[178];
  xx[36] = xx[184];
  xx[37] = xx[173];
  xx[38] = xx[179];
  xx[39] = xx[185];
  xx[47] = xx[174];
  xx[48] = xx[180];
  xx[49] = xx[186];
  xx[50] = xx[175];
  xx[51] = xx[181];
  xx[52] = xx[187];
  xx[53] = xx[176];
  xx[54] = xx[182];
  xx[55] = xx[188];
  logVector[0] = state[0];
  logVector[1] = state[1];
  logVector[2] = state[2];
  logVector[3] = state[3];
  logVector[4] = state[4];
  logVector[5] = state[5];
  logVector[6] = state[6];
  logVector[7] = state[7];
  logVector[8] = state[8];
  logVector[9] = state[9];
  logVector[10] = state[10];
  logVector[11] = state[11];
  logVector[12] = xx[0] * state[12];
  logVector[13] = xx[0] * state[13];
  logVector[14] = xx[0] * state[14];
  logVector[15] = state[15];
  logVector[16] = state[16];
  logVector[17] = state[17];
  logVector[18] = state[18];
  logVector[19] = state[19];
  logVector[20] = state[20];
  logVector[21] = state[21];
  logVector[22] = state[22];
  logVector[23] = state[23];
  logVector[24] = state[24];
  logVector[25] = xx[0] * state[25];
  logVector[26] = xx[0] * state[26];
  logVector[27] = xx[0] * state[27];
  logVector[28] = state[28];
  logVector[29] = state[29];
  logVector[30] = state[30];
  logVector[31] = state[31];
  logVector[32] = state[32];
  logVector[33] = state[33];
  logVector[34] = inputDdot[3];
  logVector[35] = xx[41] - pm_math_Vector3_dot_ra(xx + 8, xx + 18);
  logVector[36] = xx[42] - pm_math_Vector3_dot_ra(xx + 11, xx + 18);
  logVector[37] = xx[43] - pm_math_Vector3_dot_ra(xx + 14, xx + 18);
  logVector[38] = xx[0] * (xx[44] - pm_math_Vector3_dot_ra(xx + 21, xx + 18));
  logVector[39] = xx[0] * (xx[45] - pm_math_Vector3_dot_ra(xx + 24, xx + 18));
  logVector[40] = xx[0] * (xx[46] - pm_math_Vector3_dot_ra(xx + 27, xx + 18));
  logVector[41] = xx[59] - pm_math_Vector3_dot_ra(xx + 3, xx + 30);
  logVector[42] = xx[60] - pm_math_Vector3_dot_ra(xx + 34, xx + 30);
  logVector[43] = xx[61] - pm_math_Vector3_dot_ra(xx + 37, xx + 30);
  logVector[44] = xx[0] * (xx[62] - pm_math_Vector3_dot_ra(xx + 47, xx + 30));
  logVector[45] = xx[0] * (xx[63] - pm_math_Vector3_dot_ra(xx + 50, xx + 30));
  logVector[46] = xx[0] * (xx[64] - pm_math_Vector3_dot_ra(xx + 53, xx + 30));
  logVector[47] = inputDdot[2];
  logVector[48] = inputDdot[1];
  logVector[49] = inputDdot[0];
  errorResult[0] = xx[33] + xx[226];
  return NULL;
}

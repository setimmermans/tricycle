//
//-------------------------------------------------------------
//
//	ROBOTRAN - Version 6.6 (build : february 22, 2008)
//
//	Copyright 
//	Universite catholique de Louvain 
//	Departement de Mecanique 
//	Unite de Production Mecanique et Machines 
//	2, Place du Levant 
//	1348 Louvain-la-Neuve 
//	http://www.robotran.be// 
//
//	==> Generation Date : Sat Oct 29 14:54:54 2016
//
//	==> Project name : tricycle
//	==> using XML input file 
//
//	==> Number of joints : 30
//
//	==> Function : F 8 : Constraints Vector (h) and Jacobian Matrix (Jac) 
//	==> Flops complexity : 574
//
//	==> Generation Time :  0.020 seconds
//	==> Post-Processing :  0.010 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
 
void mbs_cons_hJ(double *h,double **Jac,
MbsData *s, double tsim)

// double h[8];
// double Jac[8][30];
{ 
 
#include "mbs_cons_hJ_tricycle.h" 
#define q s->q 
#define qd s->qd 
#define qdd s->qdd 
 
 

// === begin imp_aux === 

// === end imp_aux === 

// ===== BEGIN task 0 ===== 

// = = Block_0_0_0_0_0_3 = = 
 
// Trigonometric Variables  

  C10 = cos(q[10]);
  S10 = sin(q[10]);

// = = Block_0_0_0_0_0_5 = = 
 
// Trigonometric Variables  

  C12 = cos(q[12]);
  S12 = sin(q[12]);
  C13 = cos(q[13]);
  S13 = sin(q[13]);
  C14 = cos(q[14]);
  S14 = sin(q[14]);
  C15 = cos(q[15]);
  S15 = sin(q[15]);

// = = Block_0_0_0_0_0_6 = = 
 
// Trigonometric Variables  

  C18 = cos(q[18]);
  S18 = sin(q[18]);
  C19 = cos(q[19]);
  S19 = sin(q[19]);
  C20 = cos(q[20]);
  S20 = sin(q[20]);
  C21 = cos(q[21]);
  S21 = sin(q[21]);

// = = Block_0_0_0_0_0_7 = = 
 
// Trigonometric Variables  

  C24 = cos(q[24]);
  S24 = sin(q[24]);
  C25 = cos(q[25]);
  S25 = sin(q[25]);

// = = Block_0_0_0_0_0_8 = = 
 
// Trigonometric Variables  

  C26 = cos(q[26]);
  S26 = sin(q[26]);

// = = Block_0_0_0_0_0_9 = = 
 
// Trigonometric Variables  

  C27 = cos(q[27]);
  S27 = sin(q[27]);
  C28 = cos(q[28]);
  S28 = sin(q[28]);

// = = Block_0_0_0_0_0_10 = = 
 
// Trigonometric Variables  

  C29 = cos(q[29]);
  S29 = sin(q[29]);
  C30 = cos(q[30]);
  S30 = sin(q[30]);

// = = Block_0_1_0_0_0_5 = = 
 
// Trigonometric Variables  

//
  S12p13 = C12*S13+S12*C13;
  C12p13 = C12*C13-S12*S13;
 
// Constraints and Constraints Jacobian 

  RO_3_413 = S12p13*S10;
  RO_3_613 = S12p13*C10;
  RO_3_714 = C12p13*S10*C14+C10*S14;
  RO_3_814 = -S12p13*C14;
  RO_3_914 = C12p13*C10*C14-S10*S14;
  RL_3_112 = s->dpt[3][11]*S10;
  RL_3_312 = s->dpt[3][11]*C10;
  RL_3_113 = s->dpt[2][16]*S10*S12;
  RL_3_213 = s->dpt[2][16]*C12;
  RL_3_313 = s->dpt[2][16]*C10*S12;
  RL_3_134 = RO_3_714*s->dpt[3][19];
  RL_3_234 = RO_3_814*s->dpt[3][19];
  RL_3_334 = RO_3_914*s->dpt[3][19];
  JT_3_134_10 = RL_3_312+RL_3_313+RL_3_334;
  JT_3_334_10 = -(RL_3_112+RL_3_113+RL_3_134);
  JT_3_134_12 = S10*(RL_3_213+RL_3_234);
  JT_3_234_12 = -(RL_3_134*S10+RL_3_334*C10+s->dpt[2][16]*S12);
  JT_3_334_12 = C10*(RL_3_213+RL_3_234);
  JT_3_134_13 = RL_3_234*S10;
  JT_3_234_13 = -(RL_3_134*S10+RL_3_334*C10);
  JT_3_334_13 = RL_3_234*C10;
  JT_3_134_14 = -(RL_3_234*RO_3_613-RL_3_334*C12p13);
  JT_3_234_14 = RL_3_134*RO_3_613-RL_3_334*RO_3_413;
  JT_3_334_14 = -(RL_3_134*C12p13-RL_3_234*RO_3_413);
  JT_3_134_15 = -(RL_3_234*RO_3_914-RL_3_334*RO_3_814);
  JT_3_234_15 = RL_3_134*RO_3_914-RL_3_334*RO_3_714;
  JT_3_334_15 = -(RL_3_134*RO_3_814-RL_3_234*RO_3_714);
//
  RO_7_413 = S12p13*S10;
  RO_7_613 = S12p13*C10;
  RO_7_713 = C12p13*S10;
  RO_7_913 = C12p13*C10;
  RO_7_114 = -(RO_7_713*S14-C10*C14);
  RO_7_214 = S12p13*S14;
  RO_7_314 = -(RO_7_913*S14+S10*C14);
  RO_7_714 = RO_7_713*C14+C10*S14;
  RO_7_814 = -S12p13*C14;
  RO_7_914 = RO_7_913*C14-S10*S14;
  RL_7_112 = s->dpt[3][11]*S10;
  RL_7_312 = s->dpt[3][11]*C10;
  RL_7_113 = s->dpt[2][16]*S10*S12;
  RL_7_213 = s->dpt[2][16]*C12;
  RL_7_313 = s->dpt[2][16]*C10*S12;
  RL_7_138 = RO_7_714*s->dpt[3][21]+s->dpt[1][21]*(RO_7_114*C15+RO_7_413*S15)-s->dpt[2][21]*(RO_7_114*S15-RO_7_413*C15);
  RL_7_238 = RO_7_814*s->dpt[3][21]+s->dpt[1][21]*(RO_7_214*C15+C12p13*S15)-s->dpt[2][21]*(RO_7_214*S15-C12p13*C15);
  RL_7_338 = RO_7_914*s->dpt[3][21]+s->dpt[1][21]*(RO_7_314*C15+RO_7_613*S15)-s->dpt[2][21]*(RO_7_314*S15-RO_7_613*C15);

// = = Block_0_1_0_0_0_6 = = 
 
// Trigonometric Variables  

//
  S18p19 = C18*S19+S18*C19;
  C18p19 = C18*C19-S18*S19;
 
// Constraints and Constraints Jacobian 

  RO_1_419 = S18p19*S10;
  RO_1_619 = S18p19*C10;
  RO_1_719 = C18p19*S10;
  RO_1_919 = C18p19*C10;
  RO_1_120 = -(RO_1_719*S20-C10*C20);
  RO_1_220 = S18p19*S20;
  RO_1_320 = -(RO_1_919*S20+S10*C20);
  RO_1_720 = RO_1_719*C20+C10*S20;
  RO_1_820 = -S18p19*C20;
  RO_1_920 = RO_1_919*C20-S10*S20;
  RL_1_118 = s->dpt[3][12]*S10;
  RL_1_318 = s->dpt[3][12]*C10;
  RL_1_119 = s->dpt[2][23]*S10*S18;
  RL_1_219 = s->dpt[2][23]*C18;
  RL_1_319 = s->dpt[2][23]*C10*S18;
  PO_1_119 = RL_1_118+RL_1_119+s->dpt[1][5];
  PO_1_219 = RL_1_219+s->dpt[2][12];
  PO_1_319 = RL_1_318+RL_1_319+s->dpt[3][5];
  JT_1_119_10 = RL_1_318+RL_1_319;
  JT_1_319_10 = -(RL_1_118+RL_1_119);
  JT_1_119_18 = RL_1_219*S10;
  JT_1_219_18 = -s->dpt[2][23]*S18;
  JT_1_319_18 = RL_1_219*C10;
  RL_1_132 = RO_1_720*s->dpt[3][26];
  RL_1_232 = RO_1_820*s->dpt[3][26];
  RL_1_332 = RO_1_920*s->dpt[3][26];
  JT_1_132_10 = JT_1_119_10+RL_1_332;
  JT_1_332_10 = JT_1_319_10-RL_1_132;
  JT_1_132_18 = JT_1_119_18+RL_1_232*S10;
  JT_1_232_18 = JT_1_219_18-RL_1_132*S10-RL_1_332*C10;
  JT_1_332_18 = JT_1_319_18+RL_1_232*C10;
  JT_1_132_19 = RL_1_232*S10;
  JT_1_232_19 = -(RL_1_132*S10+RL_1_332*C10);
  JT_1_332_19 = RL_1_232*C10;
  JT_1_132_20 = -(RL_1_232*RO_1_619-RL_1_332*C18p19);
  JT_1_232_20 = RL_1_132*RO_1_619-RL_1_332*RO_1_419;
  JT_1_332_20 = -(RL_1_132*C18p19-RL_1_232*RO_1_419);
  JT_1_132_21 = -(RL_1_232*RO_1_920-RL_1_332*RO_1_820);
  JT_1_232_21 = RL_1_132*RO_1_920-RL_1_332*RO_1_720;
  JT_1_332_21 = -(RL_1_132*RO_1_820-RL_1_232*RO_1_720);
//
  RL_4_135 = RO_1_720*s->dpt[3][28]+s->dpt[1][28]*(RO_1_120*C21+RO_1_419*S21)-s->dpt[2][28]*(RO_1_120*S21-RO_1_419*C21);
  RL_4_235 = RO_1_820*s->dpt[3][28]+s->dpt[1][28]*(RO_1_220*C21+C18p19*S21)-s->dpt[2][28]*(RO_1_220*S21-C18p19*C21);
  RL_4_335 = RO_1_920*s->dpt[3][28]+s->dpt[1][28]*(RO_1_320*C21+RO_1_619*S21)-s->dpt[2][28]*(RO_1_320*S21-RO_1_619*C21);

// = = Block_0_1_0_0_0_7 = = 
 
// Constraints and Constraints Jacobian 

//
  RO_5_125 = C24*C25;
  RO_5_325 = -S24*C25;
  RO_5_425 = -C24*S25;
  RO_5_625 = S24*S25;
  RL_5_136 = RO_5_125*s->dpt[1][31]+RO_5_425*s->dpt[2][31];
  RL_5_236 = s->dpt[1][31]*S25+s->dpt[2][31]*C25;
  RL_5_336 = RO_5_325*s->dpt[1][31]+RO_5_625*s->dpt[2][31];
//
  RL_6_137 = RO_5_125*s->dpt[1][30]+RO_5_425*s->dpt[2][30];
  RL_6_237 = s->dpt[1][30]*S25+s->dpt[2][30]*C25;
  RL_6_337 = RO_5_325*s->dpt[1][30]+RO_5_625*s->dpt[2][30];

// = = Block_0_1_0_0_0_9 = = 
 
// Constraints and Constraints Jacobian 

//
  RO_2_127 = C26*C27;
  RO_2_327 = -S26*C27;
  RL_2_133 = -s->dpt[2][34]*(C26*S27*C28-S26*S28);
  RL_2_233 = s->dpt[2][34]*C27*C28;
  RL_2_333 = s->dpt[2][34]*(C26*S28+S26*S27*C28);
  JT_2_133_27 = -RL_2_233*C26;
  JT_2_233_27 = RL_2_133*C26-RL_2_333*S26;
  JT_2_333_27 = RL_2_233*S26;
  JT_2_133_28 = -(RL_2_233*RO_2_327-RL_2_333*S27);
  JT_2_233_28 = RL_2_133*RO_2_327-RL_2_333*RO_2_127;
  JT_2_333_28 = -(RL_2_133*S27-RL_2_233*RO_2_127);

// = = Block_0_1_0_0_0_10 = = 
 
// Constraints and Constraints Jacobian 

//
  RO_0_129 = C26*C29;
  RO_0_329 = -S26*C29;
  RL_0_131 = -s->dpt[2][35]*(C26*S29*C30-S26*S30);
  RL_0_231 = s->dpt[2][35]*C29*C30;
  RL_0_331 = s->dpt[2][35]*(C26*S30+S26*S29*C30);
  JT_0_131_29 = -RL_0_231*C26;
  JT_0_231_29 = RL_0_131*C26-RL_0_331*S26;
  JT_0_331_29 = RL_0_231*S26;
  JT_0_131_30 = -(RL_0_231*RO_0_329-RL_0_331*S29);
  JT_0_231_30 = RL_0_131*RO_0_329-RL_0_331*RO_0_129;
  JT_0_331_30 = -(RL_0_131*S29-RL_0_231*RO_0_129);

// = = Block_0_1_0_0_1_0 = = 
 
// Constraints and Constraints Jacobian 

//
  h_1 = RL_0_131+s->dpt[1][7]-(PO_1_119+RL_1_132);
  h_2 = -(PO_1_219-RL_0_231+RL_1_232);
  h_3 = RL_0_331+s->dpt[3][7]-(PO_1_319+RL_1_332);
//
  h_4 = RL_2_133-RL_3_112-RL_3_113-RL_3_134-s->dpt[1][5]+s->dpt[1][7];
  h_5 = RL_2_233-RL_3_213-RL_3_234-s->dpt[2][11];
  h_6 = RL_2_333-RL_3_312-RL_3_313-RL_3_334-s->dpt[3][5]+s->dpt[3][7];
//
  Plp11 = PO_1_119+RL_4_135-RL_5_136-s->dpt[1][6];
  Plp21 = PO_1_219+RL_4_235-RL_5_236;
  Plp31 = PO_1_319+RL_4_335-RL_5_336-s->dpt[3][6];
  h_7 = (0.50)*(Plp11*Plp11+Plp21*Plp21+Plp31*Plp31-s->lrod[1]*s->lrod[1]);
//
  Jacu_7_10 = Plp11*(JT_1_119_10+RL_4_335)+Plp31*(JT_1_319_10-RL_4_135);
  Jacu_7_18 = Plp11*(JT_1_119_18+RL_4_235*S10)+Plp21*(JT_1_219_18-RL_4_135*S10-RL_4_335*C10)+Plp31*(JT_1_319_18+RL_4_235
 *C10);
  Jac_7_19 = -(Plp21*(RL_4_135*S10+RL_4_335*C10)-RL_4_235*(Plp11*S10+Plp31*C10));
  Jac_7_20 = -(Plp11*(RL_4_235*RO_1_619-RL_4_335*C18p19)-Plp21*(RL_4_135*RO_1_619-RL_4_335*RO_1_419)+Plp31*(RL_4_135*
 C18p19-RL_4_235*RO_1_419));
  Jac_7_21 = -(Plp11*(RL_4_235*RO_1_920-RL_4_335*RO_1_820)-Plp21*(RL_4_135*RO_1_920-RL_4_335*RO_1_720)+Plp31*(RL_4_135*
 RO_1_820-RL_4_235*RO_1_720));
  Jacu_7_24 = -(Plp11*RL_5_336-Plp31*RL_5_136);
  Jacu_7_25 = -(Plp21*(s->dpt[1][31]*C25-s->dpt[2][31]*S25)-RL_5_236*(Plp11*C24-Plp31*S24));
//
  Plp12 = RL_6_137-RL_7_112-RL_7_113-RL_7_138-s->dpt[1][5]+s->dpt[1][6];
  Plp22 = RL_6_237-RL_7_213-RL_7_238-s->dpt[2][11];
  Plp32 = RL_6_337-RL_7_312-RL_7_313-RL_7_338-s->dpt[3][5]+s->dpt[3][6];
  h_8 = (0.50)*(Plp12*Plp12+Plp22*Plp22+Plp32*Plp32-s->lrod[2]*s->lrod[2]);
//
  Jacu_8_10 = -(Plp12*(RL_7_312+RL_7_313+RL_7_338)-Plp32*(RL_7_112+RL_7_113+RL_7_138));
  Jacu_8_12 = Plp22*(RL_7_138*S10+RL_7_338*C10+s->dpt[2][16]*S12)-(RL_7_213+RL_7_238)*(Plp12*S10+Plp32*C10);
  Jac_8_13 = Plp22*(RL_7_138*S10+RL_7_338*C10)-RL_7_238*(Plp12*S10+Plp32*C10);
  Jac_8_14 = Plp12*(RL_7_238*RO_7_613-RL_7_338*C12p13)-Plp22*(RL_7_138*RO_7_613-RL_7_338*RO_7_413)+Plp32*(RL_7_138*
 C12p13-RL_7_238*RO_7_413);
  Jac_8_15 = Plp12*(RL_7_238*RO_7_914-RL_7_338*RO_7_814)-Plp22*(RL_7_138*RO_7_914-RL_7_338*RO_7_714)+Plp32*(RL_7_138*
 RO_7_814-RL_7_238*RO_7_714);
  Jacu_8_24 = Plp12*RL_6_337-Plp32*RL_6_137;
  Jacu_8_25 = Plp22*(s->dpt[1][30]*C25-s->dpt[2][30]*S25)-RL_6_237*(Plp12*C24-Plp32*S24);

// = = Block_0_3_0_0_0_0 = = 
 
// Symbolic Outputs  

  h[1] = h_1;
  h[2] = h_2;
  h[3] = h_3;
  h[4] = h_4;
  h[5] = h_5;
  h[6] = h_6;
  h[7] = h_7;
  h[8] = h_8;
  Jac[1][10] = -JT_1_132_10;
  Jac[1][18] = -JT_1_132_18;
  Jac[1][19] = -JT_1_132_19;
  Jac[1][20] = -JT_1_132_20;
  Jac[1][21] = -JT_1_132_21;
  Jac[1][26] = RL_0_331;
  Jac[1][29] = JT_0_131_29;
  Jac[1][30] = JT_0_131_30;
  Jac[2][18] = -JT_1_232_18;
  Jac[2][19] = -JT_1_232_19;
  Jac[2][20] = -JT_1_232_20;
  Jac[2][21] = -JT_1_232_21;
  Jac[2][29] = JT_0_231_29;
  Jac[2][30] = JT_0_231_30;
  Jac[3][10] = -JT_1_332_10;
  Jac[3][18] = -JT_1_332_18;
  Jac[3][19] = -JT_1_332_19;
  Jac[3][20] = -JT_1_332_20;
  Jac[3][21] = -JT_1_332_21;
  Jac[3][26] = -RL_0_131;
  Jac[3][29] = JT_0_331_29;
  Jac[3][30] = JT_0_331_30;
  Jac[4][10] = -JT_3_134_10;
  Jac[4][12] = -JT_3_134_12;
  Jac[4][13] = -JT_3_134_13;
  Jac[4][14] = -JT_3_134_14;
  Jac[4][15] = -JT_3_134_15;
  Jac[4][26] = RL_2_333;
  Jac[4][27] = JT_2_133_27;
  Jac[4][28] = JT_2_133_28;
  Jac[5][12] = -JT_3_234_12;
  Jac[5][13] = -JT_3_234_13;
  Jac[5][14] = -JT_3_234_14;
  Jac[5][15] = -JT_3_234_15;
  Jac[5][27] = JT_2_233_27;
  Jac[5][28] = JT_2_233_28;
  Jac[6][10] = -JT_3_334_10;
  Jac[6][12] = -JT_3_334_12;
  Jac[6][13] = -JT_3_334_13;
  Jac[6][14] = -JT_3_334_14;
  Jac[6][15] = -JT_3_334_15;
  Jac[6][26] = -RL_2_133;
  Jac[6][27] = JT_2_333_27;
  Jac[6][28] = JT_2_333_28;
  Jac[7][10] = Jacu_7_10;
  Jac[7][18] = Jacu_7_18;
  Jac[7][19] = Jac_7_19;
  Jac[7][20] = Jac_7_20;
  Jac[7][21] = Jac_7_21;
  Jac[7][24] = Jacu_7_24;
  Jac[7][25] = Jacu_7_25;
  Jac[8][10] = Jacu_8_10;
  Jac[8][12] = Jacu_8_12;
  Jac[8][13] = Jac_8_13;
  Jac[8][14] = Jac_8_14;
  Jac[8][15] = Jac_8_15;
  Jac[8][24] = Jacu_8_24;
  Jac[8][25] = Jacu_8_25;

// ====== END Task 0 ====== 


}
 


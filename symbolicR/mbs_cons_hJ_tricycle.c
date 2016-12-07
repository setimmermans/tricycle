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
//	==> Generation Date : Mon Dec  5 17:03:59 2016
//
//	==> Project name : tricycle
//	==> using XML input file 
//
//	==> Number of joints : 31
//
//	==> Function : F 8 : Constraints Vector (h) and Jacobian Matrix (Jac) 
//	==> Flops complexity : 659
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

// double h[11];
// double Jac[11][31];
{ 
 
#include "mbs_cons_hJ_tricycle.h" 
#define q s->q 
#define qd s->qd 
#define qdd s->qdd 
 
 

// === begin imp_aux === 

// === end imp_aux === 

// ===== BEGIN task 0 ===== 

// = = Block_0_0_0_0_0_2 = = 
 
// Trigonometric Variables  

  C8 = cos(q[8]);
  S8 = sin(q[8]);

// = = Block_0_0_0_0_0_3 = = 
 
// Trigonometric Variables  

  C11 = cos(q[11]);
  S11 = sin(q[11]);

// = = Block_0_0_0_0_0_4 = = 
 
// Trigonometric Variables  

  C12 = cos(q[12]);
  S12 = sin(q[12]);

// = = Block_0_0_0_0_0_5 = = 
 
// Trigonometric Variables  

  C13 = cos(q[13]);
  S13 = sin(q[13]);
  C14 = cos(q[14]);
  S14 = sin(q[14]);
  C15 = cos(q[15]);
  S15 = sin(q[15]);
  C16 = cos(q[16]);
  S16 = sin(q[16]);

// = = Block_0_0_0_0_0_6 = = 
 
// Trigonometric Variables  

  C19 = cos(q[19]);
  S19 = sin(q[19]);
  C20 = cos(q[20]);
  S20 = sin(q[20]);
  C21 = cos(q[21]);
  S21 = sin(q[21]);
  C22 = cos(q[22]);
  S22 = sin(q[22]);

// = = Block_0_0_0_0_0_7 = = 
 
// Trigonometric Variables  

  C25 = cos(q[25]);
  S25 = sin(q[25]);
  C26 = cos(q[26]);
  S26 = sin(q[26]);

// = = Block_0_0_0_0_0_8 = = 
 
// Trigonometric Variables  

  C27 = cos(q[27]);
  S27 = sin(q[27]);

// = = Block_0_0_0_0_0_9 = = 
 
// Trigonometric Variables  

  C28 = cos(q[28]);
  S28 = sin(q[28]);
  C29 = cos(q[29]);
  S29 = sin(q[29]);

// = = Block_0_0_0_0_0_10 = = 
 
// Trigonometric Variables  

  C30 = cos(q[30]);
  S30 = sin(q[30]);
  C31 = cos(q[31]);
  S31 = sin(q[31]);

// = = Block_0_1_0_0_0_2 = = 
 
// Constraints and Constraints Jacobian 

//
  RL_9_141 = s->dpt[1][9]*C8+s->dpt[3][9]*S8;
  RL_9_341 = -(s->dpt[1][9]*S8-s->dpt[3][9]*C8);

// = = Block_0_1_0_0_0_4 = = 
 
// Constraints and Constraints Jacobian 

//
  RL_10_242 = s->dpt[2][15]*C12-s->dpt[3][15]*S12;
  RL_10_342 = s->dpt[2][15]*S12+s->dpt[3][15]*C12;
//
  RL_12_244 = s->dpt[2][14]*C12-s->dpt[3][14]*S12;
  RL_12_344 = s->dpt[2][14]*S12+s->dpt[3][14]*C12;

// = = Block_0_1_0_0_0_5 = = 
 
// Trigonometric Variables  

//
  S13p14 = C13*S14+S13*C14;
  C13p14 = C13*C14-S13*S14;
 
// Constraints and Constraints Jacobian 

  RO_3_414 = S13p14*S11;
  RO_3_614 = S13p14*C11;
  RO_3_715 = C13p14*S11*C15+C11*S15;
  RO_3_815 = -S13p14*C15;
  RO_3_915 = C13p14*C11*C15-S11*S15;
  RL_3_113 = s->dpt[3][12]*S11;
  RL_3_313 = s->dpt[3][12]*C11;
  RL_3_114 = s->dpt[2][17]*S11*S13;
  RL_3_214 = s->dpt[2][17]*C13;
  RL_3_314 = s->dpt[2][17]*C11*S13;
  RL_3_135 = RO_3_715*s->dpt[3][20];
  RL_3_235 = RO_3_815*s->dpt[3][20];
  RL_3_335 = RO_3_915*s->dpt[3][20];
  JT_3_135_11 = RL_3_313+RL_3_314+RL_3_335;
  JT_3_335_11 = -(RL_3_113+RL_3_114+RL_3_135);
  JT_3_135_13 = S11*(RL_3_214+RL_3_235);
  JT_3_235_13 = -(RL_3_135*S11+RL_3_335*C11+s->dpt[2][17]*S13);
  JT_3_335_13 = C11*(RL_3_214+RL_3_235);
  JT_3_135_14 = RL_3_235*S11;
  JT_3_235_14 = -(RL_3_135*S11+RL_3_335*C11);
  JT_3_335_14 = RL_3_235*C11;
  JT_3_135_15 = -(RL_3_235*RO_3_614-RL_3_335*C13p14);
  JT_3_235_15 = RL_3_135*RO_3_614-RL_3_335*RO_3_414;
  JT_3_335_15 = -(RL_3_135*C13p14-RL_3_235*RO_3_414);
  JT_3_135_16 = -(RL_3_235*RO_3_915-RL_3_335*RO_3_815);
  JT_3_235_16 = RL_3_135*RO_3_915-RL_3_335*RO_3_715;
  JT_3_335_16 = -(RL_3_135*RO_3_815-RL_3_235*RO_3_715);
//
  RO_7_414 = S13p14*S11;
  RO_7_614 = S13p14*C11;
  RO_7_714 = C13p14*S11;
  RO_7_914 = C13p14*C11;
  RO_7_115 = -(RO_7_714*S15-C11*C15);
  RO_7_215 = S13p14*S15;
  RO_7_315 = -(RO_7_914*S15+S11*C15);
  RO_7_715 = RO_7_714*C15+C11*S15;
  RO_7_815 = -S13p14*C15;
  RO_7_915 = RO_7_914*C15-S11*S15;
  RL_7_113 = s->dpt[3][12]*S11;
  RL_7_313 = s->dpt[3][12]*C11;
  RL_7_114 = s->dpt[2][17]*S11*S13;
  RL_7_214 = s->dpt[2][17]*C13;
  RL_7_314 = s->dpt[2][17]*C11*S13;
  RL_7_139 = RO_7_715*s->dpt[3][22]+s->dpt[1][22]*(RO_7_115*C16+RO_7_414*S16)-s->dpt[2][22]*(RO_7_115*S16-RO_7_414*C16);
  RL_7_239 = RO_7_815*s->dpt[3][22]+s->dpt[1][22]*(RO_7_215*C16+C13p14*S16)-s->dpt[2][22]*(RO_7_215*S16-C13p14*C16);
  RL_7_339 = RO_7_915*s->dpt[3][22]+s->dpt[1][22]*(RO_7_315*C16+RO_7_614*S16)-s->dpt[2][22]*(RO_7_315*S16-RO_7_614*C16);
//
  RL_13_245 = s->dpt[2][18]*C13-s->dpt[3][18]*S13;
  RL_13_345 = s->dpt[2][18]*S13+s->dpt[3][18]*C13;

// = = Block_0_1_0_0_0_6 = = 
 
// Trigonometric Variables  

//
  S19p20 = C19*S20+S19*C20;
  C19p20 = C19*C20-S19*S20;
 
// Constraints and Constraints Jacobian 

  RO_1_420 = S19p20*S11;
  RO_1_620 = S19p20*C11;
  RO_1_720 = C19p20*S11;
  RO_1_920 = C19p20*C11;
  RO_1_121 = -(RO_1_720*S21-C11*C21);
  RO_1_221 = S19p20*S21;
  RO_1_321 = -(RO_1_920*S21+S11*C21);
  RO_1_721 = RO_1_720*C21+C11*S21;
  RO_1_821 = -S19p20*C21;
  RO_1_921 = RO_1_920*C21-S11*S21;
  RL_1_119 = s->dpt[3][13]*S11;
  RL_1_319 = s->dpt[3][13]*C11;
  RL_1_120 = s->dpt[2][24]*S11*S19;
  RL_1_220 = s->dpt[2][24]*C19;
  RL_1_320 = s->dpt[2][24]*C11*S19;
  PO_1_120 = RL_1_119+RL_1_120+s->dpt[1][5];
  PO_1_220 = RL_1_220+s->dpt[2][13];
  PO_1_320 = RL_1_319+RL_1_320+s->dpt[3][5];
  JT_1_120_11 = RL_1_319+RL_1_320;
  JT_1_320_11 = -(RL_1_119+RL_1_120);
  JT_1_120_19 = RL_1_220*S11;
  JT_1_220_19 = -s->dpt[2][24]*S19;
  JT_1_320_19 = RL_1_220*C11;
  RL_1_133 = RO_1_721*s->dpt[3][27];
  RL_1_233 = RO_1_821*s->dpt[3][27];
  RL_1_333 = RO_1_921*s->dpt[3][27];
  JT_1_133_11 = JT_1_120_11+RL_1_333;
  JT_1_333_11 = JT_1_320_11-RL_1_133;
  JT_1_133_19 = JT_1_120_19+RL_1_233*S11;
  JT_1_233_19 = JT_1_220_19-RL_1_133*S11-RL_1_333*C11;
  JT_1_333_19 = JT_1_320_19+RL_1_233*C11;
  JT_1_133_20 = RL_1_233*S11;
  JT_1_233_20 = -(RL_1_133*S11+RL_1_333*C11);
  JT_1_333_20 = RL_1_233*C11;
  JT_1_133_21 = -(RL_1_233*RO_1_620-RL_1_333*C19p20);
  JT_1_233_21 = RL_1_133*RO_1_620-RL_1_333*RO_1_420;
  JT_1_333_21 = -(RL_1_133*C19p20-RL_1_233*RO_1_420);
  JT_1_133_22 = -(RL_1_233*RO_1_921-RL_1_333*RO_1_821);
  JT_1_233_22 = RL_1_133*RO_1_921-RL_1_333*RO_1_721;
  JT_1_333_22 = -(RL_1_133*RO_1_821-RL_1_233*RO_1_721);
//
  RL_4_136 = RO_1_721*s->dpt[3][29]+s->dpt[1][29]*(RO_1_121*C22+RO_1_420*S22)-s->dpt[2][29]*(RO_1_121*S22-RO_1_420*C22);
  RL_4_236 = RO_1_821*s->dpt[3][29]+s->dpt[1][29]*(RO_1_221*C22+C19p20*S22)-s->dpt[2][29]*(RO_1_221*S22-C19p20*C22);
  RL_4_336 = RO_1_921*s->dpt[3][29]+s->dpt[1][29]*(RO_1_321*C22+RO_1_620*S22)-s->dpt[2][29]*(RO_1_321*S22-RO_1_620*C22);
//
  RL_11_243 = s->dpt[2][25]*C19-s->dpt[3][25]*S19;
  RL_11_343 = s->dpt[2][25]*S19+s->dpt[3][25]*C19;

// = = Block_0_1_0_0_0_7 = = 
 
// Constraints and Constraints Jacobian 

//
  RO_5_126 = C25*C26;
  RO_5_326 = -S25*C26;
  RO_5_426 = -C25*S26;
  RO_5_626 = S25*S26;
  RL_5_137 = RO_5_126*s->dpt[1][32]+RO_5_426*s->dpt[2][32];
  RL_5_237 = s->dpt[1][32]*S26+s->dpt[2][32]*C26;
  RL_5_337 = RO_5_326*s->dpt[1][32]+RO_5_626*s->dpt[2][32];
//
  RL_6_138 = RO_5_126*s->dpt[1][31]+RO_5_426*s->dpt[2][31];
  RL_6_238 = s->dpt[1][31]*S26+s->dpt[2][31]*C26;
  RL_6_338 = RO_5_326*s->dpt[1][31]+RO_5_626*s->dpt[2][31];

// = = Block_0_1_0_0_0_9 = = 
 
// Constraints and Constraints Jacobian 

//
  RO_2_128 = C27*C28;
  RO_2_328 = -S27*C28;
  RL_2_134 = -s->dpt[2][35]*(C27*S28*C29-S27*S29);
  RL_2_234 = s->dpt[2][35]*C28*C29;
  RL_2_334 = s->dpt[2][35]*(C27*S29+S27*S28*C29);
  JT_2_134_28 = -RL_2_234*C27;
  JT_2_234_28 = RL_2_134*C27-RL_2_334*S27;
  JT_2_334_28 = RL_2_234*S27;
  JT_2_134_29 = -(RL_2_234*RO_2_328-RL_2_334*S28);
  JT_2_234_29 = RL_2_134*RO_2_328-RL_2_334*RO_2_128;
  JT_2_334_29 = -(RL_2_134*S28-RL_2_234*RO_2_128);

// = = Block_0_1_0_0_0_10 = = 
 
// Constraints and Constraints Jacobian 

//
  RO_0_130 = C27*C30;
  RO_0_330 = -S27*C30;
  RL_0_132 = -s->dpt[2][36]*(C27*S30*C31-S27*S31);
  RL_0_232 = s->dpt[2][36]*C30*C31;
  RL_0_332 = s->dpt[2][36]*(C27*S31+S27*S30*C31);
  JT_0_132_30 = -RL_0_232*C27;
  JT_0_232_30 = RL_0_132*C27-RL_0_332*S27;
  JT_0_332_30 = RL_0_232*S27;
  JT_0_132_31 = -(RL_0_232*RO_0_330-RL_0_332*S30);
  JT_0_232_31 = RL_0_132*RO_0_330-RL_0_332*RO_0_130;
  JT_0_332_31 = -(RL_0_132*S30-RL_0_232*RO_0_130);

// = = Block_0_1_0_0_1_0 = = 
 
// Constraints and Constraints Jacobian 

//
  h_1 = RL_0_132+s->dpt[1][7]-(PO_1_120+RL_1_133);
  h_2 = -(PO_1_220-RL_0_232+RL_1_233);
  h_3 = RL_0_332+s->dpt[3][7]-(PO_1_320+RL_1_333);
//
  h_4 = RL_2_134-RL_3_113-RL_3_114-RL_3_135-s->dpt[1][5]+s->dpt[1][7];
  h_5 = RL_2_234-RL_3_214-RL_3_235-s->dpt[2][12];
  h_6 = RL_2_334-RL_3_313-RL_3_314-RL_3_335-s->dpt[3][5]+s->dpt[3][7];
//
  Plp11 = PO_1_120+RL_4_136-RL_5_137-s->dpt[1][6];
  Plp21 = PO_1_220+RL_4_236-RL_5_237;
  Plp31 = PO_1_320+RL_4_336-RL_5_337-s->dpt[3][6];
  h_7 = (0.50)*(Plp11*Plp11+Plp21*Plp21+Plp31*Plp31-s->lrod[1]*s->lrod[1]);
//
  Jacu_7_11 = Plp11*(JT_1_120_11+RL_4_336)+Plp31*(JT_1_320_11-RL_4_136);
  Jac_7_19 = Plp11*(JT_1_120_19+RL_4_236*S11)+Plp21*(JT_1_220_19-RL_4_136*S11-RL_4_336*C11)+Plp31*(JT_1_320_19+RL_4_236*
 C11);
  Jac_7_20 = -(Plp21*(RL_4_136*S11+RL_4_336*C11)-RL_4_236*(Plp11*S11+Plp31*C11));
  Jac_7_21 = -(Plp11*(RL_4_236*RO_1_620-RL_4_336*C19p20)-Plp21*(RL_4_136*RO_1_620-RL_4_336*RO_1_420)+Plp31*(RL_4_136*
 C19p20-RL_4_236*RO_1_420));
  Jac_7_22 = -(Plp11*(RL_4_236*RO_1_921-RL_4_336*RO_1_821)-Plp21*(RL_4_136*RO_1_921-RL_4_336*RO_1_721)+Plp31*(RL_4_136*
 RO_1_821-RL_4_236*RO_1_721));
  Jacu_7_25 = -(Plp11*RL_5_337-Plp31*RL_5_137);
  Jacu_7_26 = -(Plp21*(s->dpt[1][32]*C26-s->dpt[2][32]*S26)-RL_5_237*(Plp11*C25-Plp31*S25));
//
  Plp12 = RL_6_138-RL_7_113-RL_7_114-RL_7_139-s->dpt[1][5]+s->dpt[1][6];
  Plp22 = RL_6_238-RL_7_214-RL_7_239-s->dpt[2][12];
  Plp32 = RL_6_338-RL_7_313-RL_7_314-RL_7_339-s->dpt[3][5]+s->dpt[3][6];
  h_8 = (0.50)*(Plp12*Plp12+Plp22*Plp22+Plp32*Plp32-s->lrod[2]*s->lrod[2]);
//
  Jacu_8_11 = -(Plp12*(RL_7_313+RL_7_314+RL_7_339)-Plp32*(RL_7_113+RL_7_114+RL_7_139));
  Jac_8_13 = Plp22*(RL_7_139*S11+RL_7_339*C11+s->dpt[2][17]*S13)-(RL_7_214+RL_7_239)*(Plp12*S11+Plp32*C11);
  Jac_8_14 = Plp22*(RL_7_139*S11+RL_7_339*C11)-RL_7_239*(Plp12*S11+Plp32*C11);
  Jac_8_15 = Plp12*(RL_7_239*RO_7_614-RL_7_339*C13p14)-Plp22*(RL_7_139*RO_7_614-RL_7_339*RO_7_414)+Plp32*(RL_7_139*
 C13p14-RL_7_239*RO_7_414);
  Jac_8_16 = Plp12*(RL_7_239*RO_7_915-RL_7_339*RO_7_815)-Plp22*(RL_7_139*RO_7_915-RL_7_339*RO_7_715)+Plp32*(RL_7_139*
 RO_7_815-RL_7_239*RO_7_715);
  Jacu_8_25 = Plp12*RL_6_338-Plp32*RL_6_138;
  Jacu_8_26 = Plp22*(s->dpt[1][31]*C26-s->dpt[2][31]*S26)-RL_6_238*(Plp12*C25-Plp32*S25);
//
  Plp13 = -(RL_9_141+s->dpt[1][2]-s->dpt[1][3]);
  Plp33 = -(RL_9_341+s->dpt[3][2]-s->dpt[3][3]);
  h_9 = (0.50)*(Plp13*Plp13+Plp33*Plp33-s->lrod[3]*s->lrod[3]);
//
  Jac_9_8 = -(Plp13*RL_9_341-Plp33*RL_9_141);
//
  Plp24 = RL_10_242-RL_11_243-s->dpt[2][13];
  Plp34 = RL_10_342-RL_11_343-s->dpt[3][13];
  h_10 = (0.50)*(Plp24*Plp24+Plp34*Plp34-s->lrod[4]*s->lrod[4]);
//
  Jacu_10_12 = -(Plp24*RL_10_342-Plp34*RL_10_242);
  Jac_10_19 = Plp24*RL_11_343-Plp34*RL_11_243;
//
  Plp25 = RL_12_244-RL_13_245-s->dpt[2][12];
  Plp35 = RL_12_344-RL_13_345-s->dpt[3][12];
  h_11 = (0.50)*(Plp25*Plp25+Plp35*Plp35-s->lrod[5]*s->lrod[5]);
//
  Jacu_11_12 = -(Plp25*RL_12_344-Plp35*RL_12_244);
  Jac_11_13 = Plp25*RL_13_345-Plp35*RL_13_245;

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
  h[9] = h_9;
  h[10] = h_10;
  h[11] = h_11;
  Jac[1][11] = -JT_1_133_11;
  Jac[1][19] = -JT_1_133_19;
  Jac[1][20] = -JT_1_133_20;
  Jac[1][21] = -JT_1_133_21;
  Jac[1][22] = -JT_1_133_22;
  Jac[1][27] = RL_0_332;
  Jac[1][30] = JT_0_132_30;
  Jac[1][31] = JT_0_132_31;
  Jac[2][19] = -JT_1_233_19;
  Jac[2][20] = -JT_1_233_20;
  Jac[2][21] = -JT_1_233_21;
  Jac[2][22] = -JT_1_233_22;
  Jac[2][30] = JT_0_232_30;
  Jac[2][31] = JT_0_232_31;
  Jac[3][11] = -JT_1_333_11;
  Jac[3][19] = -JT_1_333_19;
  Jac[3][20] = -JT_1_333_20;
  Jac[3][21] = -JT_1_333_21;
  Jac[3][22] = -JT_1_333_22;
  Jac[3][27] = -RL_0_132;
  Jac[3][30] = JT_0_332_30;
  Jac[3][31] = JT_0_332_31;
  Jac[4][11] = -JT_3_135_11;
  Jac[4][13] = -JT_3_135_13;
  Jac[4][14] = -JT_3_135_14;
  Jac[4][15] = -JT_3_135_15;
  Jac[4][16] = -JT_3_135_16;
  Jac[4][27] = RL_2_334;
  Jac[4][28] = JT_2_134_28;
  Jac[4][29] = JT_2_134_29;
  Jac[5][13] = -JT_3_235_13;
  Jac[5][14] = -JT_3_235_14;
  Jac[5][15] = -JT_3_235_15;
  Jac[5][16] = -JT_3_235_16;
  Jac[5][28] = JT_2_234_28;
  Jac[5][29] = JT_2_234_29;
  Jac[6][11] = -JT_3_335_11;
  Jac[6][13] = -JT_3_335_13;
  Jac[6][14] = -JT_3_335_14;
  Jac[6][15] = -JT_3_335_15;
  Jac[6][16] = -JT_3_335_16;
  Jac[6][27] = -RL_2_134;
  Jac[6][28] = JT_2_334_28;
  Jac[6][29] = JT_2_334_29;
  Jac[7][11] = Jacu_7_11;
  Jac[7][19] = Jac_7_19;
  Jac[7][20] = Jac_7_20;
  Jac[7][21] = Jac_7_21;
  Jac[7][22] = Jac_7_22;
  Jac[7][25] = Jacu_7_25;
  Jac[7][26] = Jacu_7_26;
  Jac[8][11] = Jacu_8_11;
  Jac[8][13] = Jac_8_13;
  Jac[8][14] = Jac_8_14;
  Jac[8][15] = Jac_8_15;
  Jac[8][16] = Jac_8_16;
  Jac[8][25] = Jacu_8_25;
  Jac[8][26] = Jacu_8_26;
  Jac[9][8] = Jac_9_8;
  Jac[10][12] = Jacu_10_12;
  Jac[10][19] = Jac_10_19;
  Jac[11][12] = Jacu_11_12;
  Jac[11][13] = Jac_11_13;

// ====== END Task 0 ====== 


}
 


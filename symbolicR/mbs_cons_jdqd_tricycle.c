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
//	==> Generation Date : Mon Nov 28 17:34:56 2016
//
//	==> Project name : tricycle
//	==> using XML input file 
//
//	==> Number of joints : 31
//
//	==> Function : F18 : Constraints Quadratic Velocity Terms (Jdqd)
//	==> Flops complexity : 835
//
//	==> Generation Time :  0.010 seconds
//	==> Post-Processing :  0.020 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
 
void mbs_cons_jdqd(double *Jdqd,
MbsData *s, double tsim)

// double Jdqd[11];
{ 
 
#include "mbs_cons_jdqd_tricycle.h" 
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
  RL_10_242 = s->dpt[2][16]*C12-s->dpt[3][16]*S12;
  RL_10_342 = s->dpt[2][16]*S12+s->dpt[3][16]*C12;
//
  RL_12_244 = s->dpt[2][15]*C12-s->dpt[3][15]*S12;
  RL_12_344 = s->dpt[2][15]*S12+s->dpt[3][15]*C12;

// = = Block_0_1_0_0_0_5 = = 
 
// Trigonometric Variables  

//
  S13p14 = C13*S14+S13*C14;
  C13p14 = C13*C14-S13*S14;
 
// Constraints and Constraints Jacobian 

  RO_3_715 = C13p14*S11*C15+C11*S15;
  RO_3_815 = -S13p14*C15;
  RO_3_915 = C13p14*C11*C15-S11*S15;
  RL_3_214 = s->dpt[2][18]*C13;
  RL_3_135 = RO_3_715*s->dpt[3][21];
  RL_3_235 = RO_3_815*s->dpt[3][21];
  RL_3_335 = RO_3_915*s->dpt[3][21];
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
  RL_7_214 = s->dpt[2][18]*C13;
  RL_7_139 = RO_7_715*s->dpt[3][23]+s->dpt[1][23]*(RO_7_115*C16+RO_7_414*S16)-s->dpt[2][23]*(RO_7_115*S16-RO_7_414*C16);
  RL_7_239 = RO_7_815*s->dpt[3][23]+s->dpt[1][23]*(RO_7_215*C16+C13p14*S16)-s->dpt[2][23]*(RO_7_215*S16-C13p14*C16);
  RL_7_339 = RO_7_915*s->dpt[3][23]+s->dpt[1][23]*(RO_7_315*C16+RO_7_614*S16)-s->dpt[2][23]*(RO_7_315*S16-RO_7_614*C16);
//
  RL_13_245 = s->dpt[2][19]*C13-s->dpt[3][19]*S13;
  RL_13_345 = s->dpt[2][19]*S13+s->dpt[3][19]*C13;

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
  RL_1_220 = s->dpt[2][25]*C19;
  RL_1_133 = RO_1_721*s->dpt[3][28];
  RL_1_233 = RO_1_821*s->dpt[3][28];
  RL_1_333 = RO_1_921*s->dpt[3][28];
//
  RL_4_136 = RO_1_721*s->dpt[3][30]+s->dpt[1][30]*(RO_1_121*C22+RO_1_420*S22)-s->dpt[2][30]*(RO_1_121*S22-RO_1_420*C22);
  RL_4_236 = RO_1_821*s->dpt[3][30]+s->dpt[1][30]*(RO_1_221*C22+C19p20*S22)-s->dpt[2][30]*(RO_1_221*S22-C19p20*C22);
  RL_4_336 = RO_1_921*s->dpt[3][30]+s->dpt[1][30]*(RO_1_321*C22+RO_1_620*S22)-s->dpt[2][30]*(RO_1_321*S22-RO_1_620*C22);
//
  RL_11_243 = s->dpt[2][26]*C19-s->dpt[3][26]*S19;
  RL_11_343 = s->dpt[2][26]*S19+s->dpt[3][26]*C19;

// = = Block_0_1_0_0_0_7 = = 
 
// Constraints and Constraints Jacobian 

//
  RO_5_126 = C25*C26;
  RO_5_326 = -S25*C26;
  RO_5_426 = -C25*S26;
  RO_5_626 = S25*S26;
  RL_5_137 = RO_5_126*s->dpt[1][33]+RO_5_426*s->dpt[2][33];
  RL_5_237 = s->dpt[1][33]*S26+s->dpt[2][33]*C26;
  RL_5_337 = RO_5_326*s->dpt[1][33]+RO_5_626*s->dpt[2][33];
//
  RL_6_138 = RO_5_126*s->dpt[1][32]+RO_5_426*s->dpt[2][32];
  RL_6_238 = s->dpt[1][32]*S26+s->dpt[2][32]*C26;
  RL_6_338 = RO_5_326*s->dpt[1][32]+RO_5_626*s->dpt[2][32];

// = = Block_0_1_0_0_0_9 = = 
 
// Constraints and Constraints Jacobian 

//
  RL_2_134 = -s->dpt[2][36]*(C27*S28*C29-S27*S29);
  RL_2_234 = s->dpt[2][36]*C28*C29;
  RL_2_334 = s->dpt[2][36]*(C27*S29+S27*S28*C29);

// = = Block_0_1_0_0_0_10 = = 
 
// Constraints and Constraints Jacobian 

//
  RL_0_132 = -s->dpt[2][37]*(C27*S30*C31-S27*S31);
  RL_0_232 = s->dpt[2][37]*C30*C31;
  RL_0_332 = s->dpt[2][37]*(C27*S31+S27*S30*C31);

// = = Block_0_2_0_0_0_0 = = 
 
// Constraints Quadratic Terms 

//
  OM_0_130 = qd[30]*S27;
  OM_0_330 = qd[30]*C27;
  OM_0_131 = OM_0_130+qd[31]*C27*C30;
  OM_0_231 = qd[31]*S30;
  OM_0_331 = OM_0_330-qd[31]*S27*C30;
  Ompqp_0_131 = -OM_0_330*qd[31]*S30;
  Ompqp_0_231 = qd[30]*qd[31]*C30;
  Ompqp_0_331 = OM_0_130*qd[31]*S30;
  OR_0_132 = OM_0_231*RL_0_332-OM_0_331*RL_0_232;
  OR_0_232 = -(OM_0_131*RL_0_332-OM_0_331*RL_0_132);
  OR_0_332 = OM_0_131*RL_0_232-OM_0_231*RL_0_132;
//
  OM_1_119 = qd[19]*C11;
  OM_1_319 = -qd[19]*S11;
  OM_1_120 = C11*(qd[19]+qd[20]);
  OM_1_320 = OM_1_319-qd[20]*S11;
  OR_1_120 = -OM_1_319*RL_1_220;
  OR_1_220 = -qd[19]*s->dpt[2][25]*S19;
  OR_1_320 = OM_1_119*RL_1_220;
  Apqp_1_120 = -OM_1_319*OR_1_220;
  Apqp_1_220 = -(OM_1_119*OR_1_320-OM_1_319*OR_1_120);
  Apqp_1_320 = OM_1_119*OR_1_220;
  OM_1_121 = OM_1_120+RO_1_420*qd[21];
  OM_1_221 = qd[21]*C19p20;
  OM_1_321 = OM_1_320+RO_1_620*qd[21];
  OM_1_122 = OM_1_121+RO_1_721*qd[22];
  OM_1_222 = OM_1_221+RO_1_821*qd[22];
  OM_1_322 = OM_1_321+RO_1_921*qd[22];
  Ompqp_1_122 = -(OM_1_320*qd[21]*C19p20-qd[22]*(OM_1_221*RO_1_921-OM_1_321*RO_1_821));
  Ompqp_1_222 = -(qd[21]*S19p20*(qd[19]+qd[20])+qd[22]*(OM_1_121*RO_1_921-OM_1_321*RO_1_721));
  Ompqp_1_322 = OM_1_120*qd[21]*C19p20+qd[22]*(OM_1_121*RO_1_821-OM_1_221*RO_1_721);
  OR_1_133 = OM_1_222*RL_1_333-OM_1_322*RL_1_233;
  OR_1_233 = -(OM_1_122*RL_1_333-OM_1_322*RL_1_133);
  OR_1_333 = OM_1_122*RL_1_233-OM_1_222*RL_1_133;
//
  OM_2_128 = qd[28]*S27;
  OM_2_328 = qd[28]*C27;
  OM_2_129 = OM_2_128+qd[29]*C27*C28;
  OM_2_229 = qd[29]*S28;
  OM_2_329 = OM_2_328-qd[29]*S27*C28;
  Ompqp_2_129 = -OM_2_328*qd[29]*S28;
  Ompqp_2_229 = qd[28]*qd[29]*C28;
  Ompqp_2_329 = OM_2_128*qd[29]*S28;
  OR_2_134 = OM_2_229*RL_2_334-OM_2_329*RL_2_234;
  OR_2_234 = -(OM_2_129*RL_2_334-OM_2_329*RL_2_134);
  OR_2_334 = OM_2_129*RL_2_234-OM_2_229*RL_2_134;
//
  OM_3_113 = qd[13]*C11;
  OM_3_313 = -qd[13]*S11;
  OM_3_114 = C11*(qd[13]+qd[14]);
  OM_3_314 = OM_3_313-qd[14]*S11;
  OR_3_214 = -qd[13]*s->dpt[2][18]*S13;
  OM_3_115 = OM_3_114+qd[15]*S13p14*S11;
  OM_3_215 = qd[15]*C13p14;
  OM_3_315 = OM_3_314+qd[15]*S13p14*C11;
  OM_3_116 = OM_3_115+RO_3_715*qd[16];
  OM_3_216 = OM_3_215+RO_3_815*qd[16];
  OM_3_316 = OM_3_315+RO_3_915*qd[16];
  Ompqp_3_116 = -(OM_3_314*qd[15]*C13p14-qd[16]*(OM_3_215*RO_3_915-OM_3_315*RO_3_815));
  Ompqp_3_216 = -(qd[15]*S13p14*(qd[13]+qd[14])+qd[16]*(OM_3_115*RO_3_915-OM_3_315*RO_3_715));
  Ompqp_3_316 = OM_3_114*qd[15]*C13p14+qd[16]*(OM_3_115*RO_3_815-OM_3_215*RO_3_715);
  OR_3_135 = OM_3_216*RL_3_335-OM_3_316*RL_3_235;
  OR_3_235 = -(OM_3_116*RL_3_335-OM_3_316*RL_3_135);
  OR_3_335 = OM_3_116*RL_3_235-OM_3_216*RL_3_135;
//
  OR_4_136 = OM_1_222*RL_4_336-OM_1_322*RL_4_236;
  OR_4_236 = -(OM_1_122*RL_4_336-OM_1_322*RL_4_136);
  OR_4_336 = OM_1_122*RL_4_236-OM_1_222*RL_4_136;
  VI_4_136 = OR_1_120+OR_4_136;
  VI_4_236 = OR_1_220+OR_4_236;
  VI_4_336 = OR_1_320+OR_4_336;
//
  OM_5_126 = qd[26]*S25;
  OM_5_326 = qd[26]*C25;
  OR_5_137 = -OM_5_326*RL_5_237;
  OR_5_237 = -(OM_5_126*RL_5_337-OM_5_326*RL_5_137);
  OR_5_337 = OM_5_126*RL_5_237;
//
  OR_6_138 = -OM_5_326*RL_6_238;
  OR_6_238 = -(OM_5_126*RL_6_338-OM_5_326*RL_6_138);
  OR_6_338 = OM_5_126*RL_6_238;
//
  OM_7_113 = qd[13]*C11;
  OM_7_313 = -qd[13]*S11;
  OM_7_114 = C11*(qd[13]+qd[14]);
  OM_7_314 = OM_7_313-qd[14]*S11;
  OR_7_114 = -OM_7_313*RL_7_214;
  OR_7_214 = -qd[13]*s->dpt[2][18]*S13;
  OR_7_314 = OM_7_113*RL_7_214;
  OM_7_115 = OM_7_114+RO_7_414*qd[15];
  OM_7_215 = qd[15]*C13p14;
  OM_7_315 = OM_7_314+RO_7_614*qd[15];
  OM_7_116 = OM_7_115+RO_7_715*qd[16];
  OM_7_216 = OM_7_215+RO_7_815*qd[16];
  OM_7_316 = OM_7_315+RO_7_915*qd[16];
  Ompqp_7_116 = -(OM_7_314*qd[15]*C13p14-qd[16]*(OM_7_215*RO_7_915-OM_7_315*RO_7_815));
  Ompqp_7_216 = -(qd[15]*S13p14*(qd[13]+qd[14])+qd[16]*(OM_7_115*RO_7_915-OM_7_315*RO_7_715));
  Ompqp_7_316 = OM_7_114*qd[15]*C13p14+qd[16]*(OM_7_115*RO_7_815-OM_7_215*RO_7_715);
  OR_7_139 = OM_7_216*RL_7_339-OM_7_316*RL_7_239;
  OR_7_239 = -(OM_7_116*RL_7_339-OM_7_316*RL_7_139);
  OR_7_339 = OM_7_116*RL_7_239-OM_7_216*RL_7_139;
  VI_7_139 = OR_7_114+OR_7_139;
  VI_7_239 = OR_7_214+OR_7_239;
  VI_7_339 = OR_7_314+OR_7_339;
//
  OR_9_141 = RL_9_341*qd[8];
  OR_9_341 = -RL_9_141*qd[8];
//
  OR_10_242 = -RL_10_342*qd[12];
  OR_10_342 = RL_10_242*qd[12];
//
  OR_11_243 = -RL_11_343*qd[19];
  OR_11_343 = RL_11_243*qd[19];
//
  OR_12_244 = -RL_12_344*qd[12];
  OR_12_344 = RL_12_244*qd[12];
//
  OR_13_245 = -RL_13_345*qd[13];
  OR_13_345 = RL_13_245*qd[13];

// = = Block_0_2_0_0_0_1 = = 
 
// Constraints Quadratic Terms 

//
  jdqd1 = OM_0_231*OR_0_332-OM_0_331*OR_0_232+Ompqp_0_231*RL_0_332-Ompqp_0_331*RL_0_232-(Apqp_1_120+OM_1_222*OR_1_333-
 OM_1_322*OR_1_233+Ompqp_1_222*RL_1_333-Ompqp_1_322*RL_1_233);
  jdqd2 = -(Apqp_1_220+OM_0_131*OR_0_332-OM_0_331*OR_0_132-OM_1_122*OR_1_333+OM_1_322*OR_1_133+Ompqp_0_131*RL_0_332-
 Ompqp_0_331*RL_0_132-Ompqp_1_122*RL_1_333+Ompqp_1_322*RL_1_133);
  jdqd3 = OM_0_131*OR_0_232-OM_0_231*OR_0_132+Ompqp_0_131*RL_0_232-Ompqp_0_231*RL_0_132-(Apqp_1_320+OM_1_122*OR_1_233-
 OM_1_222*OR_1_133+Ompqp_1_122*RL_1_233-Ompqp_1_222*RL_1_133);
//
  jdqd4 = OM_2_229*OR_2_334-OM_2_329*OR_2_234-OM_3_216*OR_3_335+OM_3_313*OR_3_214+OM_3_316*OR_3_235+Ompqp_2_229*RL_2_334
 -Ompqp_2_329*RL_2_234-Ompqp_3_216*RL_3_335+Ompqp_3_316*RL_3_235;
  jdqd5 = -(OM_2_129*OR_2_334-OM_2_329*OR_2_134-OM_3_116*OR_3_335+OM_3_316*OR_3_135+Ompqp_2_129*RL_2_334-Ompqp_2_329*
 RL_2_134-Ompqp_3_116*RL_3_335+Ompqp_3_316*RL_3_135-RL_3_214*(OM_3_113*OM_3_113+OM_3_313*OM_3_313));
  jdqd6 = OM_2_129*OR_2_234-OM_2_229*OR_2_134-OM_3_113*OR_3_214-OM_3_116*OR_3_235+OM_3_216*OR_3_135+Ompqp_2_129*RL_2_234
 -Ompqp_2_229*RL_2_134-Ompqp_3_116*RL_3_235+Ompqp_3_216*RL_3_135;
//
  jdqd7 = (RL_4_136-RL_5_137+s->dpt[1][5]-s->dpt[1][6]+s->dpt[2][25]*S11*S19+s->dpt[3][14]*S11)*(Apqp_1_120+OM_1_222*
 OR_4_336-OM_1_322*OR_4_236+OM_5_326*OR_5_237+Ompqp_1_222*RL_4_336-Ompqp_1_322*RL_4_236)+(RL_4_336-RL_5_337+s->dpt[3][5]-
 s->dpt[3][6]+s->dpt[2][25]*C11*S19+s->dpt[3][14]*C11)*(Apqp_1_320+OM_1_122*OR_4_236-OM_1_222*OR_4_136-OM_5_126*OR_5_237+
 Ompqp_1_122*RL_4_236-Ompqp_1_222*RL_4_136)+(RL_1_220+RL_4_236-RL_5_237+s->dpt[2][14])*(Apqp_1_220-OM_1_122*OR_4_336+OM_1_322
 *OR_4_136+OM_5_126*OR_5_337-OM_5_326*OR_5_137-Ompqp_1_122*RL_4_336+Ompqp_1_322*RL_4_136)+(OR_5_137-VI_4_136)*(OR_5_137-
 VI_4_136)+(OR_5_237-VI_4_236)*(OR_5_237-VI_4_236)+(OR_5_337-VI_4_336)*(OR_5_337-VI_4_336);
//
  jdqd8 = -((OM_5_126*OR_6_338-OM_5_326*OR_6_138-OM_7_113*OR_7_314-OM_7_116*OR_7_339+OM_7_313*OR_7_114+OM_7_316*OR_7_139
 -Ompqp_7_116*RL_7_339+Ompqp_7_316*RL_7_139)*(RL_6_238-RL_7_214-RL_7_239-s->dpt[2][13])-(OM_5_126*OR_6_238-OM_7_113*OR_7_214-
 OM_7_116*OR_7_239+OM_7_216*OR_7_139-Ompqp_7_116*RL_7_239+Ompqp_7_216*RL_7_139)*(RL_6_338-RL_7_339-s->dpt[3][5]+s->dpt[3][6]-
 s->dpt[2][18]*C11*S13-s->dpt[3][13]*C11)-(OR_6_138-VI_7_139)*(OR_6_138-VI_7_139)-(OR_6_238-VI_7_239)*(OR_6_238-VI_7_239)-(
 OR_6_338-VI_7_339)*(OR_6_338-VI_7_339)+(OM_5_326*OR_6_238+OM_7_216*OR_7_339-OM_7_313*OR_7_214-OM_7_316*OR_7_239+Ompqp_7_216*
 RL_7_339-Ompqp_7_316*RL_7_239)*(RL_6_138-RL_7_139-s->dpt[1][5]+s->dpt[1][6]-s->dpt[2][18]*S11*S13-s->dpt[3][13]*S11));
//
  jdqd9 = OR_9_141*(OR_9_141-qd[8]*(RL_9_341+s->dpt[3][2]-s->dpt[3][3]))+OR_9_341*OR_9_341+OR_9_341*qd[8]*(RL_9_141+
 s->dpt[1][2]-s->dpt[1][3]);
//
  jdqd10 = (OR_10_242-OR_11_243)*(OR_10_242-OR_11_243)+(OR_10_342-OR_11_343)*(OR_10_342-OR_11_343)+(OR_10_242*qd[12]-
 OR_11_243*qd[19])*(RL_10_342-RL_11_343-s->dpt[3][14])-(OR_10_342*qd[12]-OR_11_343*qd[19])*(RL_10_242-RL_11_243-s->dpt[2][14]
 );
//
  jdqd11 = (OR_12_244-OR_13_245)*(OR_12_244-OR_13_245)+(OR_12_344-OR_13_345)*(OR_12_344-OR_13_345)+(OR_12_244*qd[12]-
 OR_13_245*qd[13])*(RL_12_344-RL_13_345-s->dpt[3][13])-(OR_12_344*qd[12]-OR_13_345*qd[13])*(RL_12_244-RL_13_245-s->dpt[2][13]
 );

// = = Block_0_3_0_0_0_0 = = 
 
// Symbolic Outputs  

  Jdqd[1] = jdqd1;
  Jdqd[2] = jdqd2;
  Jdqd[3] = jdqd3;
  Jdqd[4] = jdqd4;
  Jdqd[5] = jdqd5;
  Jdqd[6] = jdqd6;
  Jdqd[7] = jdqd7;
  Jdqd[8] = jdqd8;
  Jdqd[9] = jdqd9;
  Jdqd[10] = jdqd10;
  Jdqd[11] = jdqd11;

// ====== END Task 0 ====== 


}
 


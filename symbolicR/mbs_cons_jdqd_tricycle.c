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
//	==> Generation Date : Wed Nov  2 18:35:16 2016
//
//	==> Project name : tricycle
//	==> using XML input file 
//
//	==> Number of joints : 30
//
//	==> Function : F18 : Constraints Quadratic Velocity Terms (Jdqd)
//	==> Flops complexity : 731
//
//	==> Generation Time :  0.020 seconds
//	==> Post-Processing :  0.010 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
 
void mbs_cons_jdqd(double *Jdqd,
MbsData *s, double tsim)

// double Jdqd[8];
{ 
 
#include "mbs_cons_jdqd_tricycle.h" 
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

  RO_3_714 = C12p13*S10*C14+C10*S14;
  RO_3_814 = -S12p13*C14;
  RO_3_914 = C12p13*C10*C14-S10*S14;
  RL_3_213 = s->dpt[2][16]*C12;
  RL_3_134 = RO_3_714*s->dpt[3][19];
  RL_3_234 = RO_3_814*s->dpt[3][19];
  RL_3_334 = RO_3_914*s->dpt[3][19];
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
  RL_7_213 = s->dpt[2][16]*C12;
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
  RL_1_219 = s->dpt[2][23]*C18;
  RL_1_132 = RO_1_720*s->dpt[3][26];
  RL_1_232 = RO_1_820*s->dpt[3][26];
  RL_1_332 = RO_1_920*s->dpt[3][26];
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
  RL_2_133 = -s->dpt[2][34]*(C26*S27*C28-S26*S28);
  RL_2_233 = s->dpt[2][34]*C27*C28;
  RL_2_333 = s->dpt[2][34]*(C26*S28+S26*S27*C28);

// = = Block_0_1_0_0_0_10 = = 
 
// Constraints and Constraints Jacobian 

//
  RL_0_131 = -s->dpt[2][35]*(C26*S29*C30-S26*S30);
  RL_0_231 = s->dpt[2][35]*C29*C30;
  RL_0_331 = s->dpt[2][35]*(C26*S30+S26*S29*C30);

// = = Block_0_2_0_0_0_0 = = 
 
// Constraints Quadratic Terms 

//
  OM_0_129 = qd[29]*S26;
  OM_0_329 = qd[29]*C26;
  OM_0_130 = OM_0_129+qd[30]*C26*C29;
  OM_0_230 = qd[30]*S29;
  OM_0_330 = OM_0_329-qd[30]*S26*C29;
  Ompqp_0_130 = -OM_0_329*qd[30]*S29;
  Ompqp_0_230 = qd[29]*qd[30]*C29;
  Ompqp_0_330 = OM_0_129*qd[30]*S29;
  OR_0_131 = OM_0_230*RL_0_331-OM_0_330*RL_0_231;
  OR_0_231 = -(OM_0_130*RL_0_331-OM_0_330*RL_0_131);
  OR_0_331 = OM_0_130*RL_0_231-OM_0_230*RL_0_131;
//
  OM_1_118 = qd[18]*C10;
  OM_1_318 = -qd[18]*S10;
  OM_1_119 = C10*(qd[18]+qd[19]);
  OM_1_319 = OM_1_318-qd[19]*S10;
  OR_1_119 = -OM_1_318*RL_1_219;
  OR_1_219 = -qd[18]*s->dpt[2][23]*S18;
  OR_1_319 = OM_1_118*RL_1_219;
  Apqp_1_119 = -OM_1_318*OR_1_219;
  Apqp_1_219 = -(OM_1_118*OR_1_319-OM_1_318*OR_1_119);
  Apqp_1_319 = OM_1_118*OR_1_219;
  OM_1_120 = OM_1_119+RO_1_419*qd[20];
  OM_1_220 = qd[20]*C18p19;
  OM_1_320 = OM_1_319+RO_1_619*qd[20];
  OM_1_121 = OM_1_120+RO_1_720*qd[21];
  OM_1_221 = OM_1_220+RO_1_820*qd[21];
  OM_1_321 = OM_1_320+RO_1_920*qd[21];
  Ompqp_1_121 = -(OM_1_319*qd[20]*C18p19-qd[21]*(OM_1_220*RO_1_920-OM_1_320*RO_1_820));
  Ompqp_1_221 = -(qd[20]*S18p19*(qd[18]+qd[19])+qd[21]*(OM_1_120*RO_1_920-OM_1_320*RO_1_720));
  Ompqp_1_321 = OM_1_119*qd[20]*C18p19+qd[21]*(OM_1_120*RO_1_820-OM_1_220*RO_1_720);
  OR_1_132 = OM_1_221*RL_1_332-OM_1_321*RL_1_232;
  OR_1_232 = -(OM_1_121*RL_1_332-OM_1_321*RL_1_132);
  OR_1_332 = OM_1_121*RL_1_232-OM_1_221*RL_1_132;
//
  OM_2_127 = qd[27]*S26;
  OM_2_327 = qd[27]*C26;
  OM_2_128 = OM_2_127+qd[28]*C26*C27;
  OM_2_228 = qd[28]*S27;
  OM_2_328 = OM_2_327-qd[28]*S26*C27;
  Ompqp_2_128 = -OM_2_327*qd[28]*S27;
  Ompqp_2_228 = qd[27]*qd[28]*C27;
  Ompqp_2_328 = OM_2_127*qd[28]*S27;
  OR_2_133 = OM_2_228*RL_2_333-OM_2_328*RL_2_233;
  OR_2_233 = -(OM_2_128*RL_2_333-OM_2_328*RL_2_133);
  OR_2_333 = OM_2_128*RL_2_233-OM_2_228*RL_2_133;
//
  OM_3_112 = qd[12]*C10;
  OM_3_312 = -qd[12]*S10;
  OM_3_113 = C10*(qd[12]+qd[13]);
  OM_3_313 = OM_3_312-qd[13]*S10;
  OR_3_213 = -qd[12]*s->dpt[2][16]*S12;
  OM_3_114 = OM_3_113+qd[14]*S12p13*S10;
  OM_3_214 = qd[14]*C12p13;
  OM_3_314 = OM_3_313+qd[14]*S12p13*C10;
  OM_3_115 = OM_3_114+RO_3_714*qd[15];
  OM_3_215 = OM_3_214+RO_3_814*qd[15];
  OM_3_315 = OM_3_314+RO_3_914*qd[15];
  Ompqp_3_115 = -(OM_3_313*qd[14]*C12p13-qd[15]*(OM_3_214*RO_3_914-OM_3_314*RO_3_814));
  Ompqp_3_215 = -(qd[14]*S12p13*(qd[12]+qd[13])+qd[15]*(OM_3_114*RO_3_914-OM_3_314*RO_3_714));
  Ompqp_3_315 = OM_3_113*qd[14]*C12p13+qd[15]*(OM_3_114*RO_3_814-OM_3_214*RO_3_714);
  OR_3_134 = OM_3_215*RL_3_334-OM_3_315*RL_3_234;
  OR_3_234 = -(OM_3_115*RL_3_334-OM_3_315*RL_3_134);
  OR_3_334 = OM_3_115*RL_3_234-OM_3_215*RL_3_134;
//
  OR_4_135 = OM_1_221*RL_4_335-OM_1_321*RL_4_235;
  OR_4_235 = -(OM_1_121*RL_4_335-OM_1_321*RL_4_135);
  OR_4_335 = OM_1_121*RL_4_235-OM_1_221*RL_4_135;
  VI_4_135 = OR_1_119+OR_4_135;
  VI_4_235 = OR_1_219+OR_4_235;
  VI_4_335 = OR_1_319+OR_4_335;
//
  OM_5_125 = qd[25]*S24;
  OM_5_325 = qd[25]*C24;
  OR_5_136 = -OM_5_325*RL_5_236;
  OR_5_236 = -(OM_5_125*RL_5_336-OM_5_325*RL_5_136);
  OR_5_336 = OM_5_125*RL_5_236;
//
  OR_6_137 = -OM_5_325*RL_6_237;
  OR_6_237 = -(OM_5_125*RL_6_337-OM_5_325*RL_6_137);
  OR_6_337 = OM_5_125*RL_6_237;
//
  OM_7_112 = qd[12]*C10;
  OM_7_312 = -qd[12]*S10;
  OM_7_113 = C10*(qd[12]+qd[13]);
  OM_7_313 = OM_7_312-qd[13]*S10;
  OR_7_113 = -OM_7_312*RL_7_213;
  OR_7_213 = -qd[12]*s->dpt[2][16]*S12;
  OR_7_313 = OM_7_112*RL_7_213;
  OM_7_114 = OM_7_113+RO_7_413*qd[14];
  OM_7_214 = qd[14]*C12p13;
  OM_7_314 = OM_7_313+RO_7_613*qd[14];
  OM_7_115 = OM_7_114+RO_7_714*qd[15];
  OM_7_215 = OM_7_214+RO_7_814*qd[15];
  OM_7_315 = OM_7_314+RO_7_914*qd[15];
  Ompqp_7_115 = -(OM_7_313*qd[14]*C12p13-qd[15]*(OM_7_214*RO_7_914-OM_7_314*RO_7_814));
  Ompqp_7_215 = -(qd[14]*S12p13*(qd[12]+qd[13])+qd[15]*(OM_7_114*RO_7_914-OM_7_314*RO_7_714));
  Ompqp_7_315 = OM_7_113*qd[14]*C12p13+qd[15]*(OM_7_114*RO_7_814-OM_7_214*RO_7_714);
  OR_7_138 = OM_7_215*RL_7_338-OM_7_315*RL_7_238;
  OR_7_238 = -(OM_7_115*RL_7_338-OM_7_315*RL_7_138);
  OR_7_338 = OM_7_115*RL_7_238-OM_7_215*RL_7_138;
  VI_7_138 = OR_7_113+OR_7_138;
  VI_7_238 = OR_7_213+OR_7_238;
  VI_7_338 = OR_7_313+OR_7_338;

// = = Block_0_2_0_0_0_1 = = 
 
// Constraints Quadratic Terms 

//
  jdqd1 = OM_0_230*OR_0_331-OM_0_330*OR_0_231+Ompqp_0_230*RL_0_331-Ompqp_0_330*RL_0_231-(Apqp_1_119+OM_1_221*OR_1_332-
 OM_1_321*OR_1_232+Ompqp_1_221*RL_1_332-Ompqp_1_321*RL_1_232);
  jdqd2 = -(Apqp_1_219+OM_0_130*OR_0_331-OM_0_330*OR_0_131-OM_1_121*OR_1_332+OM_1_321*OR_1_132+Ompqp_0_130*RL_0_331-
 Ompqp_0_330*RL_0_131-Ompqp_1_121*RL_1_332+Ompqp_1_321*RL_1_132);
  jdqd3 = OM_0_130*OR_0_231-OM_0_230*OR_0_131+Ompqp_0_130*RL_0_231-Ompqp_0_230*RL_0_131-(Apqp_1_319+OM_1_121*OR_1_232-
 OM_1_221*OR_1_132+Ompqp_1_121*RL_1_232-Ompqp_1_221*RL_1_132);
//
  jdqd4 = OM_2_228*OR_2_333-OM_2_328*OR_2_233-OM_3_215*OR_3_334+OM_3_312*OR_3_213+OM_3_315*OR_3_234+Ompqp_2_228*RL_2_333
 -Ompqp_2_328*RL_2_233-Ompqp_3_215*RL_3_334+Ompqp_3_315*RL_3_234;
  jdqd5 = -(OM_2_128*OR_2_333-OM_2_328*OR_2_133-OM_3_115*OR_3_334+OM_3_315*OR_3_134+Ompqp_2_128*RL_2_333-Ompqp_2_328*
 RL_2_133-Ompqp_3_115*RL_3_334+Ompqp_3_315*RL_3_134-RL_3_213*(OM_3_112*OM_3_112+OM_3_312*OM_3_312));
  jdqd6 = OM_2_128*OR_2_233-OM_2_228*OR_2_133-OM_3_112*OR_3_213-OM_3_115*OR_3_234+OM_3_215*OR_3_134+Ompqp_2_128*RL_2_233
 -Ompqp_2_228*RL_2_133-Ompqp_3_115*RL_3_234+Ompqp_3_215*RL_3_134;
//
  jdqd7 = (RL_4_135-RL_5_136+s->dpt[1][5]-s->dpt[1][6]+s->dpt[2][23]*S10*S18+s->dpt[3][12]*S10)*(Apqp_1_119+OM_1_221*
 OR_4_335-OM_1_321*OR_4_235+OM_5_325*OR_5_236+Ompqp_1_221*RL_4_335-Ompqp_1_321*RL_4_235)+(RL_4_335-RL_5_336+s->dpt[3][5]-
 s->dpt[3][6]+s->dpt[2][23]*C10*S18+s->dpt[3][12]*C10)*(Apqp_1_319+OM_1_121*OR_4_235-OM_1_221*OR_4_135-OM_5_125*OR_5_236+
 Ompqp_1_121*RL_4_235-Ompqp_1_221*RL_4_135)+(RL_1_219+RL_4_235-RL_5_236+s->dpt[2][12])*(Apqp_1_219-OM_1_121*OR_4_335+OM_1_321
 *OR_4_135+OM_5_125*OR_5_336-OM_5_325*OR_5_136-Ompqp_1_121*RL_4_335+Ompqp_1_321*RL_4_135)+(OR_5_136-VI_4_135)*(OR_5_136-
 VI_4_135)+(OR_5_236-VI_4_235)*(OR_5_236-VI_4_235)+(OR_5_336-VI_4_335)*(OR_5_336-VI_4_335);
//
  jdqd8 = -((OM_5_125*OR_6_337-OM_5_325*OR_6_137-OM_7_112*OR_7_313-OM_7_115*OR_7_338+OM_7_312*OR_7_113+OM_7_315*OR_7_138
 -Ompqp_7_115*RL_7_338+Ompqp_7_315*RL_7_138)*(RL_6_237-RL_7_213-RL_7_238-s->dpt[2][11])-(OM_5_125*OR_6_237-OM_7_112*OR_7_213-
 OM_7_115*OR_7_238+OM_7_215*OR_7_138-Ompqp_7_115*RL_7_238+Ompqp_7_215*RL_7_138)*(RL_6_337-RL_7_338-s->dpt[3][5]+s->dpt[3][6]-
 s->dpt[2][16]*C10*S12-s->dpt[3][11]*C10)-(OR_6_137-VI_7_138)*(OR_6_137-VI_7_138)-(OR_6_237-VI_7_238)*(OR_6_237-VI_7_238)-(
 OR_6_337-VI_7_338)*(OR_6_337-VI_7_338)+(OM_5_325*OR_6_237+OM_7_215*OR_7_338-OM_7_312*OR_7_213-OM_7_315*OR_7_238+Ompqp_7_215*
 RL_7_338-Ompqp_7_315*RL_7_238)*(RL_6_137-RL_7_138-s->dpt[1][5]+s->dpt[1][6]-s->dpt[2][16]*S10*S12-s->dpt[3][11]*S10));

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

// ====== END Task 0 ====== 


}
 


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
//	==> Generation Date : Sun Dec 18 11:20:57 2016
//
//	==> Project name : tricycle
//	==> using XML input file 
//
//	==> Number of joints : 31
//
//	==> Function : F19 : External Forces
//	==> Flops complexity : 1593
//
//	==> Generation Time :  0.030 seconds
//	==> Post-Processing :  0.030 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
 
void mbs_extforces(double **frc,double **trq,
MbsData *s, double tsim)

// double frc[3][31];
// double trq[3][31];
{ 
double PxF1[4]; 
double RxF1[4][4]; 
double VxF1[4]; 
double OMxF1[4]; 
double AxF1[4]; 
double OMPxF1[4]; 
double *SWr1; 
double PxF2[4]; 
double RxF2[4][4]; 
double VxF2[4]; 
double OMxF2[4]; 
double AxF2[4]; 
double OMPxF2[4]; 
double *SWr2; 
double PxF3[4]; 
double RxF3[4][4]; 
double VxF3[4]; 
double OMxF3[4]; 
double AxF3[4]; 
double OMPxF3[4]; 
double *SWr3; 
 
#include "mbs_extforces_tricycle.h" 
#define q s->q 
#define qd s->qd 
#define qdd s->qdd 
 
 

// === begin imp_aux === 

// === end imp_aux === 

// ===== BEGIN task 0 ===== 
 
// Sensor Kinematics 



// = = Block_0_0_0_0_0_1 = = 
 
// Trigonometric Variables  

  C1 = cos(q[1]);
  S1 = sin(q[1]);
  C5 = cos(q[5]);
  S5 = sin(q[5]);
  C6 = cos(q[6]);
  S6 = sin(q[6]);
  C7 = cos(q[7]);
  S7 = sin(q[7]);

// = = Block_0_0_0_0_0_2 = = 
 
// Trigonometric Variables  

  C8 = cos(q[8]);
  S8 = sin(q[8]);
  C9 = cos(q[9]);
  S9 = sin(q[9]);

// = = Block_0_0_0_0_0_3 = = 
 
// Trigonometric Variables  

  C11 = cos(q[11]);
  S11 = sin(q[11]);

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
  C17 = cos(q[17]);
  S17 = sin(q[17]);
  C18 = cos(q[18]);
  S18 = sin(q[18]);

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
  C23 = cos(q[23]);
  S23 = sin(q[23]);
  C24 = cos(q[24]);
  S24 = sin(q[24]);

// = = Block_0_0_1_1_0_1 = = 
 
// Trigonometric Variables  

  C1p5 = C1*C5-S1*S5;
  S1p5 = C1*S5+S1*C5;
 
// Sensor Kinematics 


  ROcp11_46 = -S1p5*C6;
  ROcp11_56 = C1p5*C6;
  ROcp11_76 = S1p5*S6;
  ROcp11_86 = -C1p5*S6;
  ROcp11_17 = -(ROcp11_76*S7-C1p5*C7);
  ROcp11_27 = -(ROcp11_86*S7-S1p5*C7);
  ROcp11_77 = ROcp11_76*C7+C1p5*S7;
  ROcp11_87 = ROcp11_86*C7+S1p5*S7;
  RLcp11_12 = q[2]*C1;
  RLcp11_22 = q[2]*S1;
  ORcp11_12 = -qd[1]*RLcp11_22;
  ORcp11_22 = qd[1]*RLcp11_12;
  RLcp11_13 = -q[3]*S1;
  RLcp11_23 = q[3]*C1;
  ORcp11_13 = -qd[1]*RLcp11_23;
  ORcp11_23 = qd[1]*RLcp11_13;
  OMcp11_35 = qd[1]+qd[5];
  OMcp11_16 = qd[6]*C1p5;
  OMcp11_26 = qd[6]*S1p5;
  OMcp11_17 = OMcp11_16+qd[7]*ROcp11_46;
  OMcp11_27 = OMcp11_26+qd[7]*ROcp11_56;
  OMcp11_37 = OMcp11_35+qd[7]*S6;
  OPcp11_17 = -(qd[6]*OMcp11_35*S1p5-qd[7]*(OMcp11_26*S6-OMcp11_35*ROcp11_56)-qdd[6]*C1p5-qdd[7]*ROcp11_46);
  OPcp11_27 = qd[6]*OMcp11_35*C1p5-qd[7]*(OMcp11_16*S6-OMcp11_35*ROcp11_46)+qdd[6]*S1p5+qdd[7]*ROcp11_56;
  OPcp11_37 = qdd[5]+qd[6]*qd[7]*C6+qdd[7]*S6;

// = = Block_0_0_1_1_0_2 = = 
 
// Trigonometric Variables  

  S7p8 = C7*S8+S7*C8;
  C7p8 = C7*C8-S7*S8;
  S7p8p9 = C7p8*S9+S7p8*C9;
  C7p8p9 = C7p8*C9-S7p8*S9;
 
// Sensor Kinematics 


  ROcp11_18 = ROcp11_17*C8-ROcp11_77*S8;
  ROcp11_28 = ROcp11_27*C8-ROcp11_87*S8;
  ROcp11_78 = ROcp11_17*S8+ROcp11_77*C8;
  ROcp11_88 = ROcp11_27*S8+ROcp11_87*C8;
  ROcp11_19 = ROcp11_18*C9-ROcp11_78*S9;
  ROcp11_29 = ROcp11_28*C9-ROcp11_88*S9;
  ROcp11_39 = -S7p8p9*C6;
  ROcp11_79 = ROcp11_18*S9+ROcp11_78*C9;
  ROcp11_89 = ROcp11_28*S9+ROcp11_88*C9;
  ROcp11_99 = C7p8p9*C6;
  RLcp11_18 = ROcp11_17*s->dpt[1][2]+ROcp11_77*s->dpt[3][2];
  RLcp11_28 = ROcp11_27*s->dpt[1][2]+ROcp11_87*s->dpt[3][2];
  RLcp11_38 = -C6*(s->dpt[1][2]*S7-s->dpt[3][2]*C7);
  OMcp11_18 = OMcp11_17+qd[8]*ROcp11_46;
  OMcp11_28 = OMcp11_27+qd[8]*ROcp11_56;
  OMcp11_38 = OMcp11_37+qd[8]*S6;
  ORcp11_18 = OMcp11_27*RLcp11_38-OMcp11_37*RLcp11_28;
  ORcp11_28 = -(OMcp11_17*RLcp11_38-OMcp11_37*RLcp11_18);
  ORcp11_38 = OMcp11_17*RLcp11_28-OMcp11_27*RLcp11_18;
  OPcp11_18 = OPcp11_17+qd[8]*(OMcp11_27*S6-OMcp11_37*ROcp11_56)+qdd[8]*ROcp11_46;
  OPcp11_28 = OPcp11_27-qd[8]*(OMcp11_17*S6-OMcp11_37*ROcp11_46)+qdd[8]*ROcp11_56;
  OPcp11_38 = OPcp11_37+qd[8]*(OMcp11_17*ROcp11_56-OMcp11_27*ROcp11_46)+qdd[8]*S6;
  RLcp11_19 = ROcp11_18*s->dpt[1][8];
  RLcp11_29 = ROcp11_28*s->dpt[1][8];
  RLcp11_39 = -s->dpt[1][8]*S7p8*C6;
  ORcp11_19 = OMcp11_28*RLcp11_39-OMcp11_38*RLcp11_29;
  ORcp11_29 = -(OMcp11_18*RLcp11_39-OMcp11_38*RLcp11_19);
  ORcp11_39 = OMcp11_18*RLcp11_29-OMcp11_28*RLcp11_19;
  PxF1[1] = RLcp11_12+RLcp11_13+RLcp11_18+RLcp11_19;
  PxF1[2] = RLcp11_22+RLcp11_23+RLcp11_28+RLcp11_29;
  PxF1[3] = q[4]+RLcp11_38+RLcp11_39;
  RxF1[1][1] = ROcp11_19;
  RxF1[1][2] = ROcp11_29;
  RxF1[1][3] = ROcp11_39;
  RxF1[2][1] = ROcp11_46;
  RxF1[2][2] = ROcp11_56;
  RxF1[2][3] = S6;
  RxF1[3][1] = ROcp11_79;
  RxF1[3][2] = ROcp11_89;
  RxF1[3][3] = ROcp11_99;
  VxF1[1] = ORcp11_12+ORcp11_13+ORcp11_18+ORcp11_19+qd[2]*C1-qd[3]*S1;
  VxF1[2] = ORcp11_22+ORcp11_23+ORcp11_28+ORcp11_29+qd[2]*S1+qd[3]*C1;
  VxF1[3] = qd[4]+ORcp11_38+ORcp11_39;
  OMxF1[1] = OMcp11_18+qd[9]*ROcp11_46;
  OMxF1[2] = OMcp11_28+qd[9]*ROcp11_56;
  OMxF1[3] = OMcp11_38+qd[9]*S6;
  AxF1[1] = -(qd[1]*(ORcp11_22+ORcp11_23+(2.0)*qd[2]*S1+(2.0)*qd[3]*C1)-qdd[2]*C1+qdd[3]*S1-OMcp11_27*ORcp11_38-OMcp11_28*
 ORcp11_39+OMcp11_37*ORcp11_28+OMcp11_38*ORcp11_29-OPcp11_27*RLcp11_38-OPcp11_28*RLcp11_39+OPcp11_37*RLcp11_28+OPcp11_38*
 RLcp11_29);
  AxF1[2] = qd[1]*(ORcp11_12+ORcp11_13+(2.0)*qd[2]*C1-(2.0)*qd[3]*S1)+qdd[2]*S1+qdd[3]*C1-OMcp11_17*ORcp11_38-OMcp11_18*ORcp11_39+
 OMcp11_37*ORcp11_18+OMcp11_38*ORcp11_19-OPcp11_17*RLcp11_38-OPcp11_18*RLcp11_39+OPcp11_37*RLcp11_18+OPcp11_38*RLcp11_19;
  AxF1[3] = qdd[4]+OMcp11_17*ORcp11_28+OMcp11_18*ORcp11_29-OMcp11_27*ORcp11_18-OMcp11_28*ORcp11_19+OPcp11_17*RLcp11_28+
 OPcp11_18*RLcp11_29-OPcp11_27*RLcp11_18-OPcp11_28*RLcp11_19;
  OMPxF1[1] = OPcp11_18+qd[9]*(OMcp11_28*S6-OMcp11_38*ROcp11_56)+qdd[9]*ROcp11_46;
  OMPxF1[2] = OPcp11_28-qd[9]*(OMcp11_18*S6-OMcp11_38*ROcp11_46)+qdd[9]*ROcp11_56;
  OMPxF1[3] = OPcp11_38+qd[9]*(OMcp11_18*ROcp11_56-OMcp11_28*ROcp11_46)+qdd[9]*S6;
 
// Sensor Forces Computation 

  SWr1 = user_ExtForces(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc112 = ROcp11_19*SWr1[1]+ROcp11_29*SWr1[2]+ROcp11_39*SWr1[3];
  xfrc212 = ROcp11_46*SWr1[1]+ROcp11_56*SWr1[2]+SWr1[3]*S6;
  xfrc312 = ROcp11_79*SWr1[1]+ROcp11_89*SWr1[2]+ROcp11_99*SWr1[3];
  frc[1][9] = s->frc[1][9]+xfrc112;
  frc[2][9] = s->frc[2][9]+xfrc212;
  frc[3][9] = s->frc[3][9]+xfrc312;
  xtrq112 = ROcp11_19*SWr1[4]+ROcp11_29*SWr1[5]+ROcp11_39*SWr1[6];
  xtrq212 = ROcp11_46*SWr1[4]+ROcp11_56*SWr1[5]+SWr1[6]*S6;
  xtrq312 = ROcp11_79*SWr1[4]+ROcp11_89*SWr1[5]+ROcp11_99*SWr1[6];
  trq[1][9] = s->trq[1][9]+xtrq112-xfrc212*SWr1[9]+xfrc312*SWr1[8];
  trq[2][9] = s->trq[2][9]+xtrq212+xfrc112*SWr1[9]-xfrc312*SWr1[7];
  trq[3][9] = s->trq[3][9]+xtrq312-xfrc112*SWr1[8]+xfrc212*SWr1[7];

// = = Block_0_0_1_2_0_1 = = 
 
// Sensor Kinematics 


  ROcp12_46 = -S1p5*C6;
  ROcp12_56 = C1p5*C6;
  ROcp12_76 = S1p5*S6;
  ROcp12_86 = -C1p5*S6;
  ROcp12_17 = -(ROcp12_76*S7-C1p5*C7);
  ROcp12_27 = -(ROcp12_86*S7-S1p5*C7);
  ROcp12_77 = ROcp12_76*C7+C1p5*S7;
  ROcp12_87 = ROcp12_86*C7+S1p5*S7;
  RLcp12_12 = q[2]*C1;
  RLcp12_22 = q[2]*S1;
  ORcp12_12 = -qd[1]*RLcp12_22;
  ORcp12_22 = qd[1]*RLcp12_12;
  RLcp12_13 = -q[3]*S1;
  RLcp12_23 = q[3]*C1;
  ORcp12_13 = -qd[1]*RLcp12_23;
  ORcp12_23 = qd[1]*RLcp12_13;
  OMcp12_35 = qd[1]+qd[5];
  OMcp12_16 = qd[6]*C1p5;
  OMcp12_26 = qd[6]*S1p5;
  OMcp12_17 = OMcp12_16+qd[7]*ROcp12_46;
  OMcp12_27 = OMcp12_26+qd[7]*ROcp12_56;
  OMcp12_37 = OMcp12_35+qd[7]*S6;
  OPcp12_17 = -(qd[6]*OMcp12_35*S1p5-qd[7]*(OMcp12_26*S6-OMcp12_35*ROcp12_56)-qdd[6]*C1p5-qdd[7]*ROcp12_46);
  OPcp12_27 = qd[6]*OMcp12_35*C1p5-qd[7]*(OMcp12_16*S6-OMcp12_35*ROcp12_46)+qdd[6]*S1p5+qdd[7]*ROcp12_56;
  OPcp12_37 = qdd[5]+qd[6]*qd[7]*C6+qdd[7]*S6;

// = = Block_0_0_1_2_0_3 = = 
 
// Trigonometric Variables  

  S11p7 = C11*S7+S11*C7;
  C11p7 = C11*C7-S11*S7;
 
// Sensor Kinematics 


  ROcp12_111 = ROcp12_17*C11-ROcp12_77*S11;
  ROcp12_211 = ROcp12_27*C11-ROcp12_87*S11;
  ROcp12_311 = -S11p7*C6;
  ROcp12_711 = ROcp12_17*S11+ROcp12_77*C11;
  ROcp12_811 = ROcp12_27*S11+ROcp12_87*C11;
  ROcp12_911 = C11p7*C6;
  RLcp12_111 = ROcp12_17*s->dpt[1][5]+ROcp12_77*s->dpt[3][5];
  RLcp12_211 = ROcp12_27*s->dpt[1][5]+ROcp12_87*s->dpt[3][5];
  RLcp12_311 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
  ORcp12_111 = OMcp12_27*RLcp12_311-OMcp12_37*RLcp12_211;
  ORcp12_211 = -(OMcp12_17*RLcp12_311-OMcp12_37*RLcp12_111);
  ORcp12_311 = OMcp12_17*RLcp12_211-OMcp12_27*RLcp12_111;

// = = Block_0_0_1_2_0_5 = = 
 
// Sensor Kinematics 


  ROcp12_413 = ROcp12_46*C13+ROcp12_711*S13;
  ROcp12_513 = ROcp12_56*C13+ROcp12_811*S13;
  ROcp12_613 = ROcp12_911*S13+C13*S6;
  ROcp12_713 = -(ROcp12_46*S13-ROcp12_711*C13);
  ROcp12_813 = -(ROcp12_56*S13-ROcp12_811*C13);
  ROcp12_913 = ROcp12_911*C13-S13*S6;
  ROcp12_414 = ROcp12_413*C14+ROcp12_713*S14;
  ROcp12_514 = ROcp12_513*C14+ROcp12_813*S14;
  ROcp12_614 = ROcp12_613*C14+ROcp12_913*S14;
  ROcp12_714 = -(ROcp12_413*S14-ROcp12_713*C14);
  ROcp12_814 = -(ROcp12_513*S14-ROcp12_813*C14);
  ROcp12_914 = -(ROcp12_613*S14-ROcp12_913*C14);
  ROcp12_115 = ROcp12_111*C15-ROcp12_714*S15;
  ROcp12_215 = ROcp12_211*C15-ROcp12_814*S15;
  ROcp12_315 = ROcp12_311*C15-ROcp12_914*S15;
  ROcp12_715 = ROcp12_111*S15+ROcp12_714*C15;
  ROcp12_815 = ROcp12_211*S15+ROcp12_814*C15;
  ROcp12_915 = ROcp12_311*S15+ROcp12_914*C15;
  ROcp12_116 = ROcp12_115*C16+ROcp12_414*S16;
  ROcp12_216 = ROcp12_215*C16+ROcp12_514*S16;
  ROcp12_316 = ROcp12_315*C16+ROcp12_614*S16;
  ROcp12_416 = -(ROcp12_115*S16-ROcp12_414*C16);
  ROcp12_516 = -(ROcp12_215*S16-ROcp12_514*C16);
  ROcp12_616 = -(ROcp12_315*S16-ROcp12_614*C16);
  ROcp12_417 = ROcp12_416*C17+ROcp12_715*S17;
  ROcp12_517 = ROcp12_516*C17+ROcp12_815*S17;
  ROcp12_617 = ROcp12_616*C17+ROcp12_915*S17;
  ROcp12_717 = -(ROcp12_416*S17-ROcp12_715*C17);
  ROcp12_817 = -(ROcp12_516*S17-ROcp12_815*C17);
  ROcp12_917 = -(ROcp12_616*S17-ROcp12_915*C17);
  ROcp12_118 = ROcp12_116*C18-ROcp12_717*S18;
  ROcp12_218 = ROcp12_216*C18-ROcp12_817*S18;
  ROcp12_318 = ROcp12_316*C18-ROcp12_917*S18;
  ROcp12_718 = ROcp12_116*S18+ROcp12_717*C18;
  ROcp12_818 = ROcp12_216*S18+ROcp12_817*C18;
  ROcp12_918 = ROcp12_316*S18+ROcp12_917*C18;
  RLcp12_113 = ROcp12_46*s->dpt[2][12]+ROcp12_711*s->dpt[3][12];
  RLcp12_213 = ROcp12_56*s->dpt[2][12]+ROcp12_811*s->dpt[3][12];
  RLcp12_313 = ROcp12_911*s->dpt[3][12]+s->dpt[2][12]*S6;
  OMcp12_113 = OMcp12_17+qd[13]*ROcp12_111;
  OMcp12_213 = OMcp12_27+qd[13]*ROcp12_211;
  OMcp12_313 = OMcp12_37+qd[13]*ROcp12_311;
  ORcp12_113 = OMcp12_27*RLcp12_313-OMcp12_37*RLcp12_213;
  ORcp12_213 = -(OMcp12_17*RLcp12_313-OMcp12_37*RLcp12_113);
  ORcp12_313 = OMcp12_17*RLcp12_213-OMcp12_27*RLcp12_113;
  OPcp12_113 = OPcp12_17+qd[13]*(OMcp12_27*ROcp12_311-OMcp12_37*ROcp12_211)+qdd[13]*ROcp12_111;
  OPcp12_213 = OPcp12_27-qd[13]*(OMcp12_17*ROcp12_311-OMcp12_37*ROcp12_111)+qdd[13]*ROcp12_211;
  OPcp12_313 = OPcp12_37+qd[13]*(OMcp12_17*ROcp12_211-OMcp12_27*ROcp12_111)+qdd[13]*ROcp12_311;
  RLcp12_114 = ROcp12_413*s->dpt[2][17]+ROcp12_713*s->dpt[3][17];
  RLcp12_214 = ROcp12_513*s->dpt[2][17]+ROcp12_813*s->dpt[3][17];
  RLcp12_314 = ROcp12_613*s->dpt[2][17]+ROcp12_913*s->dpt[3][17];
  OMcp12_114 = OMcp12_113+qd[14]*ROcp12_111;
  OMcp12_214 = OMcp12_213+qd[14]*ROcp12_211;
  OMcp12_314 = OMcp12_313+qd[14]*ROcp12_311;
  ORcp12_114 = OMcp12_213*RLcp12_314-OMcp12_313*RLcp12_214;
  ORcp12_214 = -(OMcp12_113*RLcp12_314-OMcp12_313*RLcp12_114);
  ORcp12_314 = OMcp12_113*RLcp12_214-OMcp12_213*RLcp12_114;
  OMcp12_115 = OMcp12_114+qd[15]*ROcp12_414;
  OMcp12_215 = OMcp12_214+qd[15]*ROcp12_514;
  OMcp12_315 = OMcp12_314+qd[15]*ROcp12_614;
  OMcp12_116 = OMcp12_115+qd[16]*ROcp12_715;
  OMcp12_216 = OMcp12_215+qd[16]*ROcp12_815;
  OMcp12_316 = OMcp12_315+qd[16]*ROcp12_915;
  OPcp12_116 = OPcp12_113+qd[14]*(OMcp12_213*ROcp12_311-OMcp12_313*ROcp12_211)+qd[15]*(OMcp12_214*ROcp12_614-OMcp12_314*
 ROcp12_514)+qd[16]*(OMcp12_215*ROcp12_915-OMcp12_315*ROcp12_815)+qdd[14]*ROcp12_111+qdd[15]*ROcp12_414+qdd[16]*ROcp12_715;
  OPcp12_216 = OPcp12_213-qd[14]*(OMcp12_113*ROcp12_311-OMcp12_313*ROcp12_111)-qd[15]*(OMcp12_114*ROcp12_614-OMcp12_314*
 ROcp12_414)-qd[16]*(OMcp12_115*ROcp12_915-OMcp12_315*ROcp12_715)+qdd[14]*ROcp12_211+qdd[15]*ROcp12_514+qdd[16]*ROcp12_815;
  OPcp12_316 = OPcp12_313+qd[14]*(OMcp12_113*ROcp12_211-OMcp12_213*ROcp12_111)+qd[15]*(OMcp12_114*ROcp12_514-OMcp12_214*
 ROcp12_414)+qd[16]*(OMcp12_115*ROcp12_815-OMcp12_215*ROcp12_715)+qdd[14]*ROcp12_311+qdd[15]*ROcp12_614+qdd[16]*ROcp12_915;
  RLcp12_117 = ROcp12_116*s->dpt[1][21]+ROcp12_416*s->dpt[2][21]+ROcp12_715*s->dpt[3][21];
  RLcp12_217 = ROcp12_216*s->dpt[1][21]+ROcp12_516*s->dpt[2][21]+ROcp12_815*s->dpt[3][21];
  RLcp12_317 = ROcp12_316*s->dpt[1][21]+ROcp12_616*s->dpt[2][21]+ROcp12_915*s->dpt[3][21];
  ORcp12_117 = OMcp12_216*RLcp12_317-OMcp12_316*RLcp12_217;
  ORcp12_217 = -(OMcp12_116*RLcp12_317-OMcp12_316*RLcp12_117);
  ORcp12_317 = OMcp12_116*RLcp12_217-OMcp12_216*RLcp12_117;
  PxF2[1] = RLcp12_111+RLcp12_113+RLcp12_114+RLcp12_117+RLcp12_12+RLcp12_13;
  PxF2[2] = RLcp12_211+RLcp12_213+RLcp12_214+RLcp12_217+RLcp12_22+RLcp12_23;
  PxF2[3] = q[4]+RLcp12_311+RLcp12_313+RLcp12_314+RLcp12_317;
  RxF2[1][1] = ROcp12_118;
  RxF2[1][2] = ROcp12_218;
  RxF2[1][3] = ROcp12_318;
  RxF2[2][1] = ROcp12_417;
  RxF2[2][2] = ROcp12_517;
  RxF2[2][3] = ROcp12_617;
  RxF2[3][1] = ROcp12_718;
  RxF2[3][2] = ROcp12_818;
  RxF2[3][3] = ROcp12_918;
  VxF2[1] = ORcp12_111+ORcp12_113+ORcp12_114+ORcp12_117+ORcp12_12+ORcp12_13+qd[2]*C1-qd[3]*S1;
  VxF2[2] = ORcp12_211+ORcp12_213+ORcp12_214+ORcp12_217+ORcp12_22+ORcp12_23+qd[2]*S1+qd[3]*C1;
  VxF2[3] = qd[4]+ORcp12_311+ORcp12_313+ORcp12_314+ORcp12_317;
  OMxF2[1] = OMcp12_116+qd[18]*ROcp12_417;
  OMxF2[2] = OMcp12_216+qd[18]*ROcp12_517;
  OMxF2[3] = OMcp12_316+qd[18]*ROcp12_617;
  AxF2[1] = -(qd[1]*(ORcp12_22+ORcp12_23+(2.0)*qd[2]*S1+(2.0)*qd[3]*C1)-qdd[2]*C1+qdd[3]*S1-OMcp12_213*ORcp12_314-OMcp12_216*
 ORcp12_317-OMcp12_27*ORcp12_311-OMcp12_27*ORcp12_313+OMcp12_313*ORcp12_214+OMcp12_316*ORcp12_217+OMcp12_37*ORcp12_211+
 OMcp12_37*ORcp12_213-OPcp12_213*RLcp12_314-OPcp12_216*RLcp12_317-OPcp12_27*RLcp12_311-OPcp12_27*RLcp12_313+OPcp12_313*
 RLcp12_214+OPcp12_316*RLcp12_217+OPcp12_37*RLcp12_211+OPcp12_37*RLcp12_213);
  AxF2[2] = qd[1]*(ORcp12_12+ORcp12_13+(2.0)*qd[2]*C1-(2.0)*qd[3]*S1)+qdd[2]*S1+qdd[3]*C1-OMcp12_113*ORcp12_314-OMcp12_116*
 ORcp12_317-OMcp12_17*ORcp12_311-OMcp12_17*ORcp12_313+OMcp12_313*ORcp12_114+OMcp12_316*ORcp12_117+OMcp12_37*ORcp12_111+
 OMcp12_37*ORcp12_113-OPcp12_113*RLcp12_314-OPcp12_116*RLcp12_317-OPcp12_17*RLcp12_311-OPcp12_17*RLcp12_313+OPcp12_313*
 RLcp12_114+OPcp12_316*RLcp12_117+OPcp12_37*RLcp12_111+OPcp12_37*RLcp12_113;
  AxF2[3] = qdd[4]+OMcp12_113*ORcp12_214+OMcp12_116*ORcp12_217+OMcp12_17*ORcp12_211+OMcp12_17*ORcp12_213-OMcp12_213*
 ORcp12_114-OMcp12_216*ORcp12_117-OMcp12_27*ORcp12_111-OMcp12_27*ORcp12_113+OPcp12_113*RLcp12_214+OPcp12_116*RLcp12_217+
 OPcp12_17*RLcp12_211+OPcp12_17*RLcp12_213-OPcp12_213*RLcp12_114-OPcp12_216*RLcp12_117-OPcp12_27*RLcp12_111-OPcp12_27*
 RLcp12_113;
  OMPxF2[1] = OPcp12_116+qd[18]*(OMcp12_216*ROcp12_617-OMcp12_316*ROcp12_517)+qdd[18]*ROcp12_417;
  OMPxF2[2] = OPcp12_216-qd[18]*(OMcp12_116*ROcp12_617-OMcp12_316*ROcp12_417)+qdd[18]*ROcp12_517;
  OMPxF2[3] = OPcp12_316+qd[18]*(OMcp12_116*ROcp12_517-OMcp12_216*ROcp12_417)+qdd[18]*ROcp12_617;
 
// Sensor Forces Computation 

  SWr2 = user_ExtForces(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc113 = ROcp12_118*SWr2[1]+ROcp12_218*SWr2[2]+ROcp12_318*SWr2[3];
  xfrc213 = ROcp12_417*SWr2[1]+ROcp12_517*SWr2[2]+ROcp12_617*SWr2[3];
  xfrc313 = ROcp12_718*SWr2[1]+ROcp12_818*SWr2[2]+ROcp12_918*SWr2[3];
  frc[1][18] = s->frc[1][18]+xfrc113;
  frc[2][18] = s->frc[2][18]+xfrc213;
  frc[3][18] = s->frc[3][18]+xfrc313;
  xtrq113 = ROcp12_118*SWr2[4]+ROcp12_218*SWr2[5]+ROcp12_318*SWr2[6];
  xtrq213 = ROcp12_417*SWr2[4]+ROcp12_517*SWr2[5]+ROcp12_617*SWr2[6];
  xtrq313 = ROcp12_718*SWr2[4]+ROcp12_818*SWr2[5]+ROcp12_918*SWr2[6];
  trq[1][18] = s->trq[1][18]+xtrq113-xfrc213*SWr2[9]+xfrc313*SWr2[8];
  trq[2][18] = s->trq[2][18]+xtrq213+xfrc113*SWr2[9]-xfrc313*SWr2[7];
  trq[3][18] = s->trq[3][18]+xtrq313-xfrc113*SWr2[8]+xfrc213*SWr2[7];

// = = Block_0_0_1_3_0_1 = = 
 
// Sensor Kinematics 


  ROcp13_46 = -S1p5*C6;
  ROcp13_56 = C1p5*C6;
  ROcp13_76 = S1p5*S6;
  ROcp13_86 = -C1p5*S6;
  ROcp13_17 = -(ROcp13_76*S7-C1p5*C7);
  ROcp13_27 = -(ROcp13_86*S7-S1p5*C7);
  ROcp13_77 = ROcp13_76*C7+C1p5*S7;
  ROcp13_87 = ROcp13_86*C7+S1p5*S7;
  RLcp13_12 = q[2]*C1;
  RLcp13_22 = q[2]*S1;
  ORcp13_12 = -qd[1]*RLcp13_22;
  ORcp13_22 = qd[1]*RLcp13_12;
  RLcp13_13 = -q[3]*S1;
  RLcp13_23 = q[3]*C1;
  ORcp13_13 = -qd[1]*RLcp13_23;
  ORcp13_23 = qd[1]*RLcp13_13;
  OMcp13_35 = qd[1]+qd[5];
  OMcp13_16 = qd[6]*C1p5;
  OMcp13_26 = qd[6]*S1p5;
  OMcp13_17 = OMcp13_16+qd[7]*ROcp13_46;
  OMcp13_27 = OMcp13_26+qd[7]*ROcp13_56;
  OMcp13_37 = OMcp13_35+qd[7]*S6;
  OPcp13_17 = -(qd[6]*OMcp13_35*S1p5-qd[7]*(OMcp13_26*S6-OMcp13_35*ROcp13_56)-qdd[6]*C1p5-qdd[7]*ROcp13_46);
  OPcp13_27 = qd[6]*OMcp13_35*C1p5-qd[7]*(OMcp13_16*S6-OMcp13_35*ROcp13_46)+qdd[6]*S1p5+qdd[7]*ROcp13_56;
  OPcp13_37 = qdd[5]+qd[6]*qd[7]*C6+qdd[7]*S6;

// = = Block_0_0_1_3_0_3 = = 
 
// Sensor Kinematics 


  ROcp13_111 = ROcp13_17*C11-ROcp13_77*S11;
  ROcp13_211 = ROcp13_27*C11-ROcp13_87*S11;
  ROcp13_311 = -S11p7*C6;
  ROcp13_711 = ROcp13_17*S11+ROcp13_77*C11;
  ROcp13_811 = ROcp13_27*S11+ROcp13_87*C11;
  ROcp13_911 = C11p7*C6;
  RLcp13_111 = ROcp13_17*s->dpt[1][5]+ROcp13_77*s->dpt[3][5];
  RLcp13_211 = ROcp13_27*s->dpt[1][5]+ROcp13_87*s->dpt[3][5];
  RLcp13_311 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
  ORcp13_111 = OMcp13_27*RLcp13_311-OMcp13_37*RLcp13_211;
  ORcp13_211 = -(OMcp13_17*RLcp13_311-OMcp13_37*RLcp13_111);
  ORcp13_311 = OMcp13_17*RLcp13_211-OMcp13_27*RLcp13_111;

// = = Block_0_0_1_3_0_6 = = 
 
// Sensor Kinematics 


  ROcp13_419 = ROcp13_46*C19+ROcp13_711*S19;
  ROcp13_519 = ROcp13_56*C19+ROcp13_811*S19;
  ROcp13_619 = ROcp13_911*S19+C19*S6;
  ROcp13_719 = -(ROcp13_46*S19-ROcp13_711*C19);
  ROcp13_819 = -(ROcp13_56*S19-ROcp13_811*C19);
  ROcp13_919 = ROcp13_911*C19-S19*S6;
  ROcp13_420 = ROcp13_419*C20+ROcp13_719*S20;
  ROcp13_520 = ROcp13_519*C20+ROcp13_819*S20;
  ROcp13_620 = ROcp13_619*C20+ROcp13_919*S20;
  ROcp13_720 = -(ROcp13_419*S20-ROcp13_719*C20);
  ROcp13_820 = -(ROcp13_519*S20-ROcp13_819*C20);
  ROcp13_920 = -(ROcp13_619*S20-ROcp13_919*C20);
  ROcp13_121 = ROcp13_111*C21-ROcp13_720*S21;
  ROcp13_221 = ROcp13_211*C21-ROcp13_820*S21;
  ROcp13_321 = ROcp13_311*C21-ROcp13_920*S21;
  ROcp13_721 = ROcp13_111*S21+ROcp13_720*C21;
  ROcp13_821 = ROcp13_211*S21+ROcp13_820*C21;
  ROcp13_921 = ROcp13_311*S21+ROcp13_920*C21;
  ROcp13_122 = ROcp13_121*C22+ROcp13_420*S22;
  ROcp13_222 = ROcp13_221*C22+ROcp13_520*S22;
  ROcp13_322 = ROcp13_321*C22+ROcp13_620*S22;
  ROcp13_422 = -(ROcp13_121*S22-ROcp13_420*C22);
  ROcp13_522 = -(ROcp13_221*S22-ROcp13_520*C22);
  ROcp13_622 = -(ROcp13_321*S22-ROcp13_620*C22);
  ROcp13_423 = ROcp13_422*C23+ROcp13_721*S23;
  ROcp13_523 = ROcp13_522*C23+ROcp13_821*S23;
  ROcp13_623 = ROcp13_622*C23+ROcp13_921*S23;
  ROcp13_723 = -(ROcp13_422*S23-ROcp13_721*C23);
  ROcp13_823 = -(ROcp13_522*S23-ROcp13_821*C23);
  ROcp13_923 = -(ROcp13_622*S23-ROcp13_921*C23);
  ROcp13_124 = ROcp13_122*C24-ROcp13_723*S24;
  ROcp13_224 = ROcp13_222*C24-ROcp13_823*S24;
  ROcp13_324 = ROcp13_322*C24-ROcp13_923*S24;
  ROcp13_724 = ROcp13_122*S24+ROcp13_723*C24;
  ROcp13_824 = ROcp13_222*S24+ROcp13_823*C24;
  ROcp13_924 = ROcp13_322*S24+ROcp13_923*C24;
  RLcp13_119 = ROcp13_46*s->dpt[2][13]+ROcp13_711*s->dpt[3][13];
  RLcp13_219 = ROcp13_56*s->dpt[2][13]+ROcp13_811*s->dpt[3][13];
  RLcp13_319 = ROcp13_911*s->dpt[3][13]+s->dpt[2][13]*S6;
  OMcp13_119 = OMcp13_17+qd[19]*ROcp13_111;
  OMcp13_219 = OMcp13_27+qd[19]*ROcp13_211;
  OMcp13_319 = OMcp13_37+qd[19]*ROcp13_311;
  ORcp13_119 = OMcp13_27*RLcp13_319-OMcp13_37*RLcp13_219;
  ORcp13_219 = -(OMcp13_17*RLcp13_319-OMcp13_37*RLcp13_119);
  ORcp13_319 = OMcp13_17*RLcp13_219-OMcp13_27*RLcp13_119;
  OPcp13_119 = OPcp13_17+qd[19]*(OMcp13_27*ROcp13_311-OMcp13_37*ROcp13_211)+qdd[19]*ROcp13_111;
  OPcp13_219 = OPcp13_27-qd[19]*(OMcp13_17*ROcp13_311-OMcp13_37*ROcp13_111)+qdd[19]*ROcp13_211;
  OPcp13_319 = OPcp13_37+qd[19]*(OMcp13_17*ROcp13_211-OMcp13_27*ROcp13_111)+qdd[19]*ROcp13_311;
  RLcp13_120 = ROcp13_419*s->dpt[2][24]+ROcp13_719*s->dpt[3][24];
  RLcp13_220 = ROcp13_519*s->dpt[2][24]+ROcp13_819*s->dpt[3][24];
  RLcp13_320 = ROcp13_619*s->dpt[2][24]+ROcp13_919*s->dpt[3][24];
  OMcp13_120 = OMcp13_119+qd[20]*ROcp13_111;
  OMcp13_220 = OMcp13_219+qd[20]*ROcp13_211;
  OMcp13_320 = OMcp13_319+qd[20]*ROcp13_311;
  ORcp13_120 = OMcp13_219*RLcp13_320-OMcp13_319*RLcp13_220;
  ORcp13_220 = -(OMcp13_119*RLcp13_320-OMcp13_319*RLcp13_120);
  ORcp13_320 = OMcp13_119*RLcp13_220-OMcp13_219*RLcp13_120;
  OMcp13_121 = OMcp13_120+qd[21]*ROcp13_420;
  OMcp13_221 = OMcp13_220+qd[21]*ROcp13_520;
  OMcp13_321 = OMcp13_320+qd[21]*ROcp13_620;
  OMcp13_122 = OMcp13_121+qd[22]*ROcp13_721;
  OMcp13_222 = OMcp13_221+qd[22]*ROcp13_821;
  OMcp13_322 = OMcp13_321+qd[22]*ROcp13_921;
  OPcp13_122 = OPcp13_119+qd[20]*(OMcp13_219*ROcp13_311-OMcp13_319*ROcp13_211)+qd[21]*(OMcp13_220*ROcp13_620-OMcp13_320*
 ROcp13_520)+qd[22]*(OMcp13_221*ROcp13_921-OMcp13_321*ROcp13_821)+qdd[20]*ROcp13_111+qdd[21]*ROcp13_420+qdd[22]*ROcp13_721;
  OPcp13_222 = OPcp13_219-qd[20]*(OMcp13_119*ROcp13_311-OMcp13_319*ROcp13_111)-qd[21]*(OMcp13_120*ROcp13_620-OMcp13_320*
 ROcp13_420)-qd[22]*(OMcp13_121*ROcp13_921-OMcp13_321*ROcp13_721)+qdd[20]*ROcp13_211+qdd[21]*ROcp13_520+qdd[22]*ROcp13_821;
  OPcp13_322 = OPcp13_319+qd[20]*(OMcp13_119*ROcp13_211-OMcp13_219*ROcp13_111)+qd[21]*(OMcp13_120*ROcp13_520-OMcp13_220*
 ROcp13_420)+qd[22]*(OMcp13_121*ROcp13_821-OMcp13_221*ROcp13_721)+qdd[20]*ROcp13_311+qdd[21]*ROcp13_620+qdd[22]*ROcp13_921;
  RLcp13_123 = ROcp13_122*s->dpt[1][28]+ROcp13_422*s->dpt[2][28]+ROcp13_721*s->dpt[3][28];
  RLcp13_223 = ROcp13_222*s->dpt[1][28]+ROcp13_522*s->dpt[2][28]+ROcp13_821*s->dpt[3][28];
  RLcp13_323 = ROcp13_322*s->dpt[1][28]+ROcp13_622*s->dpt[2][28]+ROcp13_921*s->dpt[3][28];
  ORcp13_123 = OMcp13_222*RLcp13_323-OMcp13_322*RLcp13_223;
  ORcp13_223 = -(OMcp13_122*RLcp13_323-OMcp13_322*RLcp13_123);
  ORcp13_323 = OMcp13_122*RLcp13_223-OMcp13_222*RLcp13_123;
  PxF3[1] = RLcp13_111+RLcp13_119+RLcp13_12+RLcp13_120+RLcp13_123+RLcp13_13;
  PxF3[2] = RLcp13_211+RLcp13_219+RLcp13_22+RLcp13_220+RLcp13_223+RLcp13_23;
  PxF3[3] = q[4]+RLcp13_311+RLcp13_319+RLcp13_320+RLcp13_323;
  RxF3[1][1] = ROcp13_124;
  RxF3[1][2] = ROcp13_224;
  RxF3[1][3] = ROcp13_324;
  RxF3[2][1] = ROcp13_423;
  RxF3[2][2] = ROcp13_523;
  RxF3[2][3] = ROcp13_623;
  RxF3[3][1] = ROcp13_724;
  RxF3[3][2] = ROcp13_824;
  RxF3[3][3] = ROcp13_924;
  VxF3[1] = ORcp13_111+ORcp13_119+ORcp13_12+ORcp13_120+ORcp13_123+ORcp13_13+qd[2]*C1-qd[3]*S1;
  VxF3[2] = ORcp13_211+ORcp13_219+ORcp13_22+ORcp13_220+ORcp13_223+ORcp13_23+qd[2]*S1+qd[3]*C1;
  VxF3[3] = qd[4]+ORcp13_311+ORcp13_319+ORcp13_320+ORcp13_323;
  OMxF3[1] = OMcp13_122+qd[24]*ROcp13_423;
  OMxF3[2] = OMcp13_222+qd[24]*ROcp13_523;
  OMxF3[3] = OMcp13_322+qd[24]*ROcp13_623;
  AxF3[1] = -(qd[1]*(ORcp13_22+ORcp13_23+(2.0)*qd[2]*S1+(2.0)*qd[3]*C1)-qdd[2]*C1+qdd[3]*S1-OMcp13_219*ORcp13_320-OMcp13_222*
 ORcp13_323-OMcp13_27*ORcp13_311-OMcp13_27*ORcp13_319+OMcp13_319*ORcp13_220+OMcp13_322*ORcp13_223+OMcp13_37*ORcp13_211+
 OMcp13_37*ORcp13_219-OPcp13_219*RLcp13_320-OPcp13_222*RLcp13_323-OPcp13_27*RLcp13_311-OPcp13_27*RLcp13_319+OPcp13_319*
 RLcp13_220+OPcp13_322*RLcp13_223+OPcp13_37*RLcp13_211+OPcp13_37*RLcp13_219);
  AxF3[2] = qd[1]*(ORcp13_12+ORcp13_13+(2.0)*qd[2]*C1-(2.0)*qd[3]*S1)+qdd[2]*S1+qdd[3]*C1-OMcp13_119*ORcp13_320-OMcp13_122*
 ORcp13_323-OMcp13_17*ORcp13_311-OMcp13_17*ORcp13_319+OMcp13_319*ORcp13_120+OMcp13_322*ORcp13_123+OMcp13_37*ORcp13_111+
 OMcp13_37*ORcp13_119-OPcp13_119*RLcp13_320-OPcp13_122*RLcp13_323-OPcp13_17*RLcp13_311-OPcp13_17*RLcp13_319+OPcp13_319*
 RLcp13_120+OPcp13_322*RLcp13_123+OPcp13_37*RLcp13_111+OPcp13_37*RLcp13_119;
  AxF3[3] = qdd[4]+OMcp13_119*ORcp13_220+OMcp13_122*ORcp13_223+OMcp13_17*ORcp13_211+OMcp13_17*ORcp13_219-OMcp13_219*
 ORcp13_120-OMcp13_222*ORcp13_123-OMcp13_27*ORcp13_111-OMcp13_27*ORcp13_119+OPcp13_119*RLcp13_220+OPcp13_122*RLcp13_223+
 OPcp13_17*RLcp13_211+OPcp13_17*RLcp13_219-OPcp13_219*RLcp13_120-OPcp13_222*RLcp13_123-OPcp13_27*RLcp13_111-OPcp13_27*
 RLcp13_119;
  OMPxF3[1] = OPcp13_122+qd[24]*(OMcp13_222*ROcp13_623-OMcp13_322*ROcp13_523)+qdd[24]*ROcp13_423;
  OMPxF3[2] = OPcp13_222-qd[24]*(OMcp13_122*ROcp13_623-OMcp13_322*ROcp13_423)+qdd[24]*ROcp13_523;
  OMPxF3[3] = OPcp13_322+qd[24]*(OMcp13_122*ROcp13_523-OMcp13_222*ROcp13_423)+qdd[24]*ROcp13_623;
 
// Sensor Forces Computation 

  SWr3 = user_ExtForces(PxF3,RxF3,VxF3,OMxF3,AxF3,OMPxF3,s,tsim,3);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc114 = ROcp13_124*SWr3[1]+ROcp13_224*SWr3[2]+ROcp13_324*SWr3[3];
  xfrc214 = ROcp13_423*SWr3[1]+ROcp13_523*SWr3[2]+ROcp13_623*SWr3[3];
  xfrc314 = ROcp13_724*SWr3[1]+ROcp13_824*SWr3[2]+ROcp13_924*SWr3[3];
  frc[1][24] = s->frc[1][24]+xfrc114;
  frc[2][24] = s->frc[2][24]+xfrc214;
  frc[3][24] = s->frc[3][24]+xfrc314;
  xtrq114 = ROcp13_124*SWr3[4]+ROcp13_224*SWr3[5]+ROcp13_324*SWr3[6];
  xtrq214 = ROcp13_423*SWr3[4]+ROcp13_523*SWr3[5]+ROcp13_623*SWr3[6];
  xtrq314 = ROcp13_724*SWr3[4]+ROcp13_824*SWr3[5]+ROcp13_924*SWr3[6];
  trq[1][24] = s->trq[1][24]+xtrq114-xfrc214*SWr3[9]+xfrc314*SWr3[8];
  trq[2][24] = s->trq[2][24]+xtrq214+xfrc114*SWr3[9]-xfrc314*SWr3[7];
  trq[3][24] = s->trq[3][24]+xtrq314-xfrc114*SWr3[8]+xfrc214*SWr3[7];

// = = Block_0_0_1_3_1_0 = = 
 
// Symbolic Outputs  

  frc[1][7] = s->frc[1][7];
  frc[2][7] = s->frc[2][7];
  frc[3][7] = s->frc[3][7];
  frc[1][8] = s->frc[1][8];
  frc[2][8] = s->frc[2][8];
  frc[3][8] = s->frc[3][8];
  frc[1][10] = s->frc[1][10];
  frc[2][10] = s->frc[2][10];
  frc[3][10] = s->frc[3][10];
  frc[1][11] = s->frc[1][11];
  frc[2][11] = s->frc[2][11];
  frc[3][11] = s->frc[3][11];
  frc[1][12] = s->frc[1][12];
  frc[2][12] = s->frc[2][12];
  frc[3][12] = s->frc[3][12];
  frc[1][13] = s->frc[1][13];
  frc[2][13] = s->frc[2][13];
  frc[3][13] = s->frc[3][13];
  frc[1][16] = s->frc[1][16];
  frc[2][16] = s->frc[2][16];
  frc[3][16] = s->frc[3][16];
  frc[1][19] = s->frc[1][19];
  frc[2][19] = s->frc[2][19];
  frc[3][19] = s->frc[3][19];
  frc[1][22] = s->frc[1][22];
  frc[2][22] = s->frc[2][22];
  frc[3][22] = s->frc[3][22];
  frc[1][26] = s->frc[1][26];
  frc[2][26] = s->frc[2][26];
  frc[3][26] = s->frc[3][26];
  frc[1][27] = s->frc[1][27];
  frc[2][27] = s->frc[2][27];
  frc[3][27] = s->frc[3][27];
  frc[1][29] = s->frc[1][29];
  frc[2][29] = s->frc[2][29];
  frc[3][29] = s->frc[3][29];
  frc[1][31] = s->frc[1][31];
  frc[2][31] = s->frc[2][31];
  frc[3][31] = s->frc[3][31];
  trq[1][7] = s->trq[1][7];
  trq[2][7] = s->trq[2][7];
  trq[3][7] = s->trq[3][7];
  trq[1][8] = s->trq[1][8];
  trq[2][8] = s->trq[2][8];
  trq[3][8] = s->trq[3][8];
  trq[1][10] = s->trq[1][10];
  trq[2][10] = s->trq[2][10];
  trq[3][10] = s->trq[3][10];
  trq[1][11] = s->trq[1][11];
  trq[2][11] = s->trq[2][11];
  trq[3][11] = s->trq[3][11];
  trq[1][12] = s->trq[1][12];
  trq[2][12] = s->trq[2][12];
  trq[3][12] = s->trq[3][12];
  trq[1][13] = s->trq[1][13];
  trq[2][13] = s->trq[2][13];
  trq[3][13] = s->trq[3][13];
  trq[1][16] = s->trq[1][16];
  trq[2][16] = s->trq[2][16];
  trq[3][16] = s->trq[3][16];
  trq[1][19] = s->trq[1][19];
  trq[2][19] = s->trq[2][19];
  trq[3][19] = s->trq[3][19];
  trq[1][22] = s->trq[1][22];
  trq[2][22] = s->trq[2][22];
  trq[3][22] = s->trq[3][22];
  trq[1][26] = s->trq[1][26];
  trq[2][26] = s->trq[2][26];
  trq[3][26] = s->trq[3][26];
  trq[1][27] = s->trq[1][27];
  trq[2][27] = s->trq[2][27];
  trq[3][27] = s->trq[3][27];
  trq[1][29] = s->trq[1][29];
  trq[2][29] = s->trq[2][29];
  trq[3][29] = s->trq[3][29];
  trq[1][31] = s->trq[1][31];
  trq[2][31] = s->trq[2][31];
  trq[3][31] = s->trq[3][31];

// ====== END Task 0 ====== 


}
 


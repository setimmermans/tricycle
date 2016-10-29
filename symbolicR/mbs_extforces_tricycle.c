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
//	==> Generation Date : Sat Oct 29 14:54:55 2016
//
//	==> Project name : tricycle
//	==> using XML input file 
//
//	==> Number of joints : 30
//
//	==> Function : F19 : External Forces
//	==> Flops complexity : 1581
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

// double frc[3][30];
// double trq[3][30];
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
  C16 = cos(q[16]);
  S16 = sin(q[16]);
  C17 = cos(q[17]);
  S17 = sin(q[17]);

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
  C22 = cos(q[22]);
  S22 = sin(q[22]);
  C23 = cos(q[23]);
  S23 = sin(q[23]);

// = = Block_0_0_1_1_0_1 = = 
 
// Trigonometric Variables  

  C1p5 = C1*C5-S1*S5;
  S1p5 = C1*S5+S1*C5;
 
// Sensor Kinematics 


  ROcp10_46 = -S1p5*C6;
  ROcp10_56 = C1p5*C6;
  ROcp10_76 = S1p5*S6;
  ROcp10_86 = -C1p5*S6;
  ROcp10_17 = -(ROcp10_76*S7-C1p5*C7);
  ROcp10_27 = -(ROcp10_86*S7-S1p5*C7);
  ROcp10_77 = ROcp10_76*C7+C1p5*S7;
  ROcp10_87 = ROcp10_86*C7+S1p5*S7;
  RLcp10_12 = q[2]*C1;
  RLcp10_22 = q[2]*S1;
  ORcp10_12 = -qd[1]*RLcp10_22;
  ORcp10_22 = qd[1]*RLcp10_12;
  RLcp10_13 = -q[3]*S1;
  RLcp10_23 = q[3]*C1;
  ORcp10_13 = -qd[1]*RLcp10_23;
  ORcp10_23 = qd[1]*RLcp10_13;
  OMcp10_35 = qd[1]+qd[5];
  OMcp10_16 = qd[6]*C1p5;
  OMcp10_26 = qd[6]*S1p5;
  OMcp10_17 = OMcp10_16+qd[7]*ROcp10_46;
  OMcp10_27 = OMcp10_26+qd[7]*ROcp10_56;
  OMcp10_37 = OMcp10_35+qd[7]*S6;
  OPcp10_17 = -(qd[6]*OMcp10_35*S1p5-qd[7]*(OMcp10_26*S6-OMcp10_35*ROcp10_56)-qdd[6]*C1p5-qdd[7]*ROcp10_46);
  OPcp10_27 = qd[6]*OMcp10_35*C1p5-qd[7]*(OMcp10_16*S6-OMcp10_35*ROcp10_46)+qdd[6]*S1p5+qdd[7]*ROcp10_56;
  OPcp10_37 = qdd[5]+qd[6]*qd[7]*C6+qdd[7]*S6;

// = = Block_0_0_1_1_0_2 = = 
 
// Trigonometric Variables  

  S7p8 = C7*S8+S7*C8;
  C7p8 = C7*C8-S7*S8;
  S7p8p9 = C7p8*S9+S7p8*C9;
  C7p8p9 = C7p8*C9-S7p8*S9;
 
// Sensor Kinematics 


  ROcp10_18 = ROcp10_17*C8-ROcp10_77*S8;
  ROcp10_28 = ROcp10_27*C8-ROcp10_87*S8;
  ROcp10_78 = ROcp10_17*S8+ROcp10_77*C8;
  ROcp10_88 = ROcp10_27*S8+ROcp10_87*C8;
  ROcp10_19 = ROcp10_18*C9-ROcp10_78*S9;
  ROcp10_29 = ROcp10_28*C9-ROcp10_88*S9;
  ROcp10_39 = -S7p8p9*C6;
  ROcp10_79 = ROcp10_18*S9+ROcp10_78*C9;
  ROcp10_89 = ROcp10_28*S9+ROcp10_88*C9;
  ROcp10_99 = C7p8p9*C6;
  RLcp10_18 = ROcp10_17*s->dpt[1][2]+ROcp10_77*s->dpt[3][2];
  RLcp10_28 = ROcp10_27*s->dpt[1][2]+ROcp10_87*s->dpt[3][2];
  RLcp10_38 = -C6*(s->dpt[1][2]*S7-s->dpt[3][2]*C7);
  OMcp10_18 = OMcp10_17+qd[8]*ROcp10_46;
  OMcp10_28 = OMcp10_27+qd[8]*ROcp10_56;
  OMcp10_38 = OMcp10_37+qd[8]*S6;
  ORcp10_18 = OMcp10_27*RLcp10_38-OMcp10_37*RLcp10_28;
  ORcp10_28 = -(OMcp10_17*RLcp10_38-OMcp10_37*RLcp10_18);
  ORcp10_38 = OMcp10_17*RLcp10_28-OMcp10_27*RLcp10_18;
  OPcp10_18 = OPcp10_17+qd[8]*(OMcp10_27*S6-OMcp10_37*ROcp10_56)+qdd[8]*ROcp10_46;
  OPcp10_28 = OPcp10_27-qd[8]*(OMcp10_17*S6-OMcp10_37*ROcp10_46)+qdd[8]*ROcp10_56;
  OPcp10_38 = OPcp10_37+qd[8]*(OMcp10_17*ROcp10_56-OMcp10_27*ROcp10_46)+qdd[8]*S6;
  RLcp10_19 = ROcp10_18*s->dpt[1][8];
  RLcp10_29 = ROcp10_28*s->dpt[1][8];
  RLcp10_39 = -s->dpt[1][8]*S7p8*C6;
  ORcp10_19 = OMcp10_28*RLcp10_39-OMcp10_38*RLcp10_29;
  ORcp10_29 = -(OMcp10_18*RLcp10_39-OMcp10_38*RLcp10_19);
  ORcp10_39 = OMcp10_18*RLcp10_29-OMcp10_28*RLcp10_19;
  PxF1[1] = RLcp10_12+RLcp10_13+RLcp10_18+RLcp10_19;
  PxF1[2] = RLcp10_22+RLcp10_23+RLcp10_28+RLcp10_29;
  PxF1[3] = q[4]+RLcp10_38+RLcp10_39;
  RxF1[1][1] = ROcp10_19;
  RxF1[1][2] = ROcp10_29;
  RxF1[1][3] = ROcp10_39;
  RxF1[2][1] = ROcp10_46;
  RxF1[2][2] = ROcp10_56;
  RxF1[2][3] = S6;
  RxF1[3][1] = ROcp10_79;
  RxF1[3][2] = ROcp10_89;
  RxF1[3][3] = ROcp10_99;
  VxF1[1] = ORcp10_12+ORcp10_13+ORcp10_18+ORcp10_19+qd[2]*C1-qd[3]*S1;
  VxF1[2] = ORcp10_22+ORcp10_23+ORcp10_28+ORcp10_29+qd[2]*S1+qd[3]*C1;
  VxF1[3] = qd[4]+ORcp10_38+ORcp10_39;
  OMxF1[1] = OMcp10_18+qd[9]*ROcp10_46;
  OMxF1[2] = OMcp10_28+qd[9]*ROcp10_56;
  OMxF1[3] = OMcp10_38+qd[9]*S6;
  AxF1[1] = -(qd[1]*(ORcp10_22+ORcp10_23+(2.0)*qd[2]*S1+(2.0)*qd[3]*C1)-qdd[2]*C1+qdd[3]*S1-OMcp10_27*ORcp10_38-OMcp10_28*
 ORcp10_39+OMcp10_37*ORcp10_28+OMcp10_38*ORcp10_29-OPcp10_27*RLcp10_38-OPcp10_28*RLcp10_39+OPcp10_37*RLcp10_28+OPcp10_38*
 RLcp10_29);
  AxF1[2] = qd[1]*(ORcp10_12+ORcp10_13+(2.0)*qd[2]*C1-(2.0)*qd[3]*S1)+qdd[2]*S1+qdd[3]*C1-OMcp10_17*ORcp10_38-OMcp10_18*ORcp10_39+
 OMcp10_37*ORcp10_18+OMcp10_38*ORcp10_19-OPcp10_17*RLcp10_38-OPcp10_18*RLcp10_39+OPcp10_37*RLcp10_18+OPcp10_38*RLcp10_19;
  AxF1[3] = qdd[4]+OMcp10_17*ORcp10_28+OMcp10_18*ORcp10_29-OMcp10_27*ORcp10_18-OMcp10_28*ORcp10_19+OPcp10_17*RLcp10_28+
 OPcp10_18*RLcp10_29-OPcp10_27*RLcp10_18-OPcp10_28*RLcp10_19;
  OMPxF1[1] = OPcp10_18+qd[9]*(OMcp10_28*S6-OMcp10_38*ROcp10_56)+qdd[9]*ROcp10_46;
  OMPxF1[2] = OPcp10_28-qd[9]*(OMcp10_18*S6-OMcp10_38*ROcp10_46)+qdd[9]*ROcp10_56;
  OMPxF1[3] = OPcp10_38+qd[9]*(OMcp10_18*ROcp10_56-OMcp10_28*ROcp10_46)+qdd[9]*S6;
 
// Sensor Forces Computation 

  SWr1 = user_ExtForces(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc111 = ROcp10_19*SWr1[1]+ROcp10_29*SWr1[2]+ROcp10_39*SWr1[3];
  xfrc211 = ROcp10_46*SWr1[1]+ROcp10_56*SWr1[2]+SWr1[3]*S6;
  xfrc311 = ROcp10_79*SWr1[1]+ROcp10_89*SWr1[2]+ROcp10_99*SWr1[3];
  frc[1][9] = s->frc[1][9]+xfrc111;
  frc[2][9] = s->frc[2][9]+xfrc211;
  frc[3][9] = s->frc[3][9]+xfrc311;
  xtrq111 = ROcp10_19*SWr1[4]+ROcp10_29*SWr1[5]+ROcp10_39*SWr1[6];
  xtrq211 = ROcp10_46*SWr1[4]+ROcp10_56*SWr1[5]+SWr1[6]*S6;
  xtrq311 = ROcp10_79*SWr1[4]+ROcp10_89*SWr1[5]+ROcp10_99*SWr1[6];
  trq[1][9] = s->trq[1][9]+xtrq111-xfrc211*SWr1[9]+xfrc311*SWr1[8];
  trq[2][9] = s->trq[2][9]+xtrq211+xfrc111*SWr1[9]-xfrc311*SWr1[7];
  trq[3][9] = s->trq[3][9]+xtrq311-xfrc111*SWr1[8]+xfrc211*SWr1[7];

// = = Block_0_0_1_2_0_1 = = 
 
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

// = = Block_0_0_1_2_0_3 = = 
 
// Trigonometric Variables  

  S10p7 = C10*S7+S10*C7;
  C10p7 = C10*C7-S10*S7;
 
// Sensor Kinematics 


  ROcp11_110 = ROcp11_17*C10-ROcp11_77*S10;
  ROcp11_210 = ROcp11_27*C10-ROcp11_87*S10;
  ROcp11_310 = -S10p7*C6;
  ROcp11_710 = ROcp11_17*S10+ROcp11_77*C10;
  ROcp11_810 = ROcp11_27*S10+ROcp11_87*C10;
  ROcp11_910 = C10p7*C6;
  RLcp11_110 = ROcp11_17*s->dpt[1][5]+ROcp11_77*s->dpt[3][5];
  RLcp11_210 = ROcp11_27*s->dpt[1][5]+ROcp11_87*s->dpt[3][5];
  RLcp11_310 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
  ORcp11_110 = OMcp11_27*RLcp11_310-OMcp11_37*RLcp11_210;
  ORcp11_210 = -(OMcp11_17*RLcp11_310-OMcp11_37*RLcp11_110);
  ORcp11_310 = OMcp11_17*RLcp11_210-OMcp11_27*RLcp11_110;

// = = Block_0_0_1_2_0_5 = = 
 
// Sensor Kinematics 


  ROcp11_412 = ROcp11_46*C12+ROcp11_710*S12;
  ROcp11_512 = ROcp11_56*C12+ROcp11_810*S12;
  ROcp11_612 = ROcp11_910*S12+C12*S6;
  ROcp11_712 = -(ROcp11_46*S12-ROcp11_710*C12);
  ROcp11_812 = -(ROcp11_56*S12-ROcp11_810*C12);
  ROcp11_912 = ROcp11_910*C12-S12*S6;
  ROcp11_413 = ROcp11_412*C13+ROcp11_712*S13;
  ROcp11_513 = ROcp11_512*C13+ROcp11_812*S13;
  ROcp11_613 = ROcp11_612*C13+ROcp11_912*S13;
  ROcp11_713 = -(ROcp11_412*S13-ROcp11_712*C13);
  ROcp11_813 = -(ROcp11_512*S13-ROcp11_812*C13);
  ROcp11_913 = -(ROcp11_612*S13-ROcp11_912*C13);
  ROcp11_114 = ROcp11_110*C14-ROcp11_713*S14;
  ROcp11_214 = ROcp11_210*C14-ROcp11_813*S14;
  ROcp11_314 = ROcp11_310*C14-ROcp11_913*S14;
  ROcp11_714 = ROcp11_110*S14+ROcp11_713*C14;
  ROcp11_814 = ROcp11_210*S14+ROcp11_813*C14;
  ROcp11_914 = ROcp11_310*S14+ROcp11_913*C14;
  ROcp11_115 = ROcp11_114*C15+ROcp11_413*S15;
  ROcp11_215 = ROcp11_214*C15+ROcp11_513*S15;
  ROcp11_315 = ROcp11_314*C15+ROcp11_613*S15;
  ROcp11_415 = -(ROcp11_114*S15-ROcp11_413*C15);
  ROcp11_515 = -(ROcp11_214*S15-ROcp11_513*C15);
  ROcp11_615 = -(ROcp11_314*S15-ROcp11_613*C15);
  ROcp11_416 = ROcp11_415*C16+ROcp11_714*S16;
  ROcp11_516 = ROcp11_515*C16+ROcp11_814*S16;
  ROcp11_616 = ROcp11_615*C16+ROcp11_914*S16;
  ROcp11_716 = -(ROcp11_415*S16-ROcp11_714*C16);
  ROcp11_816 = -(ROcp11_515*S16-ROcp11_814*C16);
  ROcp11_916 = -(ROcp11_615*S16-ROcp11_914*C16);
  ROcp11_117 = ROcp11_115*C17-ROcp11_716*S17;
  ROcp11_217 = ROcp11_215*C17-ROcp11_816*S17;
  ROcp11_317 = ROcp11_315*C17-ROcp11_916*S17;
  ROcp11_717 = ROcp11_115*S17+ROcp11_716*C17;
  ROcp11_817 = ROcp11_215*S17+ROcp11_816*C17;
  ROcp11_917 = ROcp11_315*S17+ROcp11_916*C17;
  RLcp11_112 = ROcp11_46*s->dpt[2][11]+ROcp11_710*s->dpt[3][11];
  RLcp11_212 = ROcp11_56*s->dpt[2][11]+ROcp11_810*s->dpt[3][11];
  RLcp11_312 = ROcp11_910*s->dpt[3][11]+s->dpt[2][11]*S6;
  OMcp11_112 = OMcp11_17+qd[12]*ROcp11_110;
  OMcp11_212 = OMcp11_27+qd[12]*ROcp11_210;
  OMcp11_312 = OMcp11_37+qd[12]*ROcp11_310;
  ORcp11_112 = OMcp11_27*RLcp11_312-OMcp11_37*RLcp11_212;
  ORcp11_212 = -(OMcp11_17*RLcp11_312-OMcp11_37*RLcp11_112);
  ORcp11_312 = OMcp11_17*RLcp11_212-OMcp11_27*RLcp11_112;
  OPcp11_112 = OPcp11_17+qd[12]*(OMcp11_27*ROcp11_310-OMcp11_37*ROcp11_210)+qdd[12]*ROcp11_110;
  OPcp11_212 = OPcp11_27-qd[12]*(OMcp11_17*ROcp11_310-OMcp11_37*ROcp11_110)+qdd[12]*ROcp11_210;
  OPcp11_312 = OPcp11_37+qd[12]*(OMcp11_17*ROcp11_210-OMcp11_27*ROcp11_110)+qdd[12]*ROcp11_310;
  RLcp11_113 = ROcp11_412*s->dpt[2][16];
  RLcp11_213 = ROcp11_512*s->dpt[2][16];
  RLcp11_313 = ROcp11_612*s->dpt[2][16];
  OMcp11_113 = OMcp11_112+qd[13]*ROcp11_110;
  OMcp11_213 = OMcp11_212+qd[13]*ROcp11_210;
  OMcp11_313 = OMcp11_312+qd[13]*ROcp11_310;
  ORcp11_113 = OMcp11_212*RLcp11_313-OMcp11_312*RLcp11_213;
  ORcp11_213 = -(OMcp11_112*RLcp11_313-OMcp11_312*RLcp11_113);
  ORcp11_313 = OMcp11_112*RLcp11_213-OMcp11_212*RLcp11_113;
  OMcp11_114 = OMcp11_113+qd[14]*ROcp11_413;
  OMcp11_214 = OMcp11_213+qd[14]*ROcp11_513;
  OMcp11_314 = OMcp11_313+qd[14]*ROcp11_613;
  OMcp11_115 = OMcp11_114+qd[15]*ROcp11_714;
  OMcp11_215 = OMcp11_214+qd[15]*ROcp11_814;
  OMcp11_315 = OMcp11_314+qd[15]*ROcp11_914;
  OPcp11_115 = OPcp11_112+qd[13]*(OMcp11_212*ROcp11_310-OMcp11_312*ROcp11_210)+qd[14]*(OMcp11_213*ROcp11_613-OMcp11_313*
 ROcp11_513)+qd[15]*(OMcp11_214*ROcp11_914-OMcp11_314*ROcp11_814)+qdd[13]*ROcp11_110+qdd[14]*ROcp11_413+qdd[15]*ROcp11_714;
  OPcp11_215 = OPcp11_212-qd[13]*(OMcp11_112*ROcp11_310-OMcp11_312*ROcp11_110)-qd[14]*(OMcp11_113*ROcp11_613-OMcp11_313*
 ROcp11_413)-qd[15]*(OMcp11_114*ROcp11_914-OMcp11_314*ROcp11_714)+qdd[13]*ROcp11_210+qdd[14]*ROcp11_513+qdd[15]*ROcp11_814;
  OPcp11_315 = OPcp11_312+qd[13]*(OMcp11_112*ROcp11_210-OMcp11_212*ROcp11_110)+qd[14]*(OMcp11_113*ROcp11_513-OMcp11_213*
 ROcp11_413)+qd[15]*(OMcp11_114*ROcp11_814-OMcp11_214*ROcp11_714)+qdd[13]*ROcp11_310+qdd[14]*ROcp11_613+qdd[15]*ROcp11_914;
  RLcp11_116 = ROcp11_115*s->dpt[1][20]+ROcp11_415*s->dpt[2][20]+ROcp11_714*s->dpt[3][20];
  RLcp11_216 = ROcp11_215*s->dpt[1][20]+ROcp11_515*s->dpt[2][20]+ROcp11_814*s->dpt[3][20];
  RLcp11_316 = ROcp11_315*s->dpt[1][20]+ROcp11_615*s->dpt[2][20]+ROcp11_914*s->dpt[3][20];
  ORcp11_116 = OMcp11_215*RLcp11_316-OMcp11_315*RLcp11_216;
  ORcp11_216 = -(OMcp11_115*RLcp11_316-OMcp11_315*RLcp11_116);
  ORcp11_316 = OMcp11_115*RLcp11_216-OMcp11_215*RLcp11_116;
  PxF2[1] = RLcp11_110+RLcp11_112+RLcp11_113+RLcp11_116+RLcp11_12+RLcp11_13;
  PxF2[2] = RLcp11_210+RLcp11_212+RLcp11_213+RLcp11_216+RLcp11_22+RLcp11_23;
  PxF2[3] = q[4]+RLcp11_310+RLcp11_312+RLcp11_313+RLcp11_316;
  RxF2[1][1] = ROcp11_117;
  RxF2[1][2] = ROcp11_217;
  RxF2[1][3] = ROcp11_317;
  RxF2[2][1] = ROcp11_416;
  RxF2[2][2] = ROcp11_516;
  RxF2[2][3] = ROcp11_616;
  RxF2[3][1] = ROcp11_717;
  RxF2[3][2] = ROcp11_817;
  RxF2[3][3] = ROcp11_917;
  VxF2[1] = ORcp11_110+ORcp11_112+ORcp11_113+ORcp11_116+ORcp11_12+ORcp11_13+qd[2]*C1-qd[3]*S1;
  VxF2[2] = ORcp11_210+ORcp11_212+ORcp11_213+ORcp11_216+ORcp11_22+ORcp11_23+qd[2]*S1+qd[3]*C1;
  VxF2[3] = qd[4]+ORcp11_310+ORcp11_312+ORcp11_313+ORcp11_316;
  OMxF2[1] = OMcp11_115+qd[17]*ROcp11_416;
  OMxF2[2] = OMcp11_215+qd[17]*ROcp11_516;
  OMxF2[3] = OMcp11_315+qd[17]*ROcp11_616;
  AxF2[1] = -(qd[1]*(ORcp11_22+ORcp11_23+(2.0)*qd[2]*S1+(2.0)*qd[3]*C1)-qdd[2]*C1+qdd[3]*S1-OMcp11_212*ORcp11_313-OMcp11_215*
 ORcp11_316-OMcp11_27*ORcp11_310-OMcp11_27*ORcp11_312+OMcp11_312*ORcp11_213+OMcp11_315*ORcp11_216+OMcp11_37*ORcp11_210+
 OMcp11_37*ORcp11_212-OPcp11_212*RLcp11_313-OPcp11_215*RLcp11_316-OPcp11_27*RLcp11_310-OPcp11_27*RLcp11_312+OPcp11_312*
 RLcp11_213+OPcp11_315*RLcp11_216+OPcp11_37*RLcp11_210+OPcp11_37*RLcp11_212);
  AxF2[2] = qd[1]*(ORcp11_12+ORcp11_13+(2.0)*qd[2]*C1-(2.0)*qd[3]*S1)+qdd[2]*S1+qdd[3]*C1-OMcp11_112*ORcp11_313-OMcp11_115*
 ORcp11_316-OMcp11_17*ORcp11_310-OMcp11_17*ORcp11_312+OMcp11_312*ORcp11_113+OMcp11_315*ORcp11_116+OMcp11_37*ORcp11_110+
 OMcp11_37*ORcp11_112-OPcp11_112*RLcp11_313-OPcp11_115*RLcp11_316-OPcp11_17*RLcp11_310-OPcp11_17*RLcp11_312+OPcp11_312*
 RLcp11_113+OPcp11_315*RLcp11_116+OPcp11_37*RLcp11_110+OPcp11_37*RLcp11_112;
  AxF2[3] = qdd[4]+OMcp11_112*ORcp11_213+OMcp11_115*ORcp11_216+OMcp11_17*ORcp11_210+OMcp11_17*ORcp11_212-OMcp11_212*
 ORcp11_113-OMcp11_215*ORcp11_116-OMcp11_27*ORcp11_110-OMcp11_27*ORcp11_112+OPcp11_112*RLcp11_213+OPcp11_115*RLcp11_216+
 OPcp11_17*RLcp11_210+OPcp11_17*RLcp11_212-OPcp11_212*RLcp11_113-OPcp11_215*RLcp11_116-OPcp11_27*RLcp11_110-OPcp11_27*
 RLcp11_112;
  OMPxF2[1] = OPcp11_115+qd[17]*(OMcp11_215*ROcp11_616-OMcp11_315*ROcp11_516)+qdd[17]*ROcp11_416;
  OMPxF2[2] = OPcp11_215-qd[17]*(OMcp11_115*ROcp11_616-OMcp11_315*ROcp11_416)+qdd[17]*ROcp11_516;
  OMPxF2[3] = OPcp11_315+qd[17]*(OMcp11_115*ROcp11_516-OMcp11_215*ROcp11_416)+qdd[17]*ROcp11_616;
 
// Sensor Forces Computation 

  SWr2 = user_ExtForces(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc112 = ROcp11_117*SWr2[1]+ROcp11_217*SWr2[2]+ROcp11_317*SWr2[3];
  xfrc212 = ROcp11_416*SWr2[1]+ROcp11_516*SWr2[2]+ROcp11_616*SWr2[3];
  xfrc312 = ROcp11_717*SWr2[1]+ROcp11_817*SWr2[2]+ROcp11_917*SWr2[3];
  frc[1][17] = s->frc[1][17]+xfrc112;
  frc[2][17] = s->frc[2][17]+xfrc212;
  frc[3][17] = s->frc[3][17]+xfrc312;
  xtrq112 = ROcp11_117*SWr2[4]+ROcp11_217*SWr2[5]+ROcp11_317*SWr2[6];
  xtrq212 = ROcp11_416*SWr2[4]+ROcp11_516*SWr2[5]+ROcp11_616*SWr2[6];
  xtrq312 = ROcp11_717*SWr2[4]+ROcp11_817*SWr2[5]+ROcp11_917*SWr2[6];
  trq[1][17] = s->trq[1][17]+xtrq112-xfrc212*SWr2[9]+xfrc312*SWr2[8];
  trq[2][17] = s->trq[2][17]+xtrq212+xfrc112*SWr2[9]-xfrc312*SWr2[7];
  trq[3][17] = s->trq[3][17]+xtrq312-xfrc112*SWr2[8]+xfrc212*SWr2[7];

// = = Block_0_0_1_3_0_1 = = 
 
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

// = = Block_0_0_1_3_0_3 = = 
 
// Sensor Kinematics 


  ROcp12_110 = ROcp12_17*C10-ROcp12_77*S10;
  ROcp12_210 = ROcp12_27*C10-ROcp12_87*S10;
  ROcp12_310 = -S10p7*C6;
  ROcp12_710 = ROcp12_17*S10+ROcp12_77*C10;
  ROcp12_810 = ROcp12_27*S10+ROcp12_87*C10;
  ROcp12_910 = C10p7*C6;
  RLcp12_110 = ROcp12_17*s->dpt[1][5]+ROcp12_77*s->dpt[3][5];
  RLcp12_210 = ROcp12_27*s->dpt[1][5]+ROcp12_87*s->dpt[3][5];
  RLcp12_310 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
  ORcp12_110 = OMcp12_27*RLcp12_310-OMcp12_37*RLcp12_210;
  ORcp12_210 = -(OMcp12_17*RLcp12_310-OMcp12_37*RLcp12_110);
  ORcp12_310 = OMcp12_17*RLcp12_210-OMcp12_27*RLcp12_110;

// = = Block_0_0_1_3_0_6 = = 
 
// Sensor Kinematics 


  ROcp12_418 = ROcp12_46*C18+ROcp12_710*S18;
  ROcp12_518 = ROcp12_56*C18+ROcp12_810*S18;
  ROcp12_618 = ROcp12_910*S18+C18*S6;
  ROcp12_718 = -(ROcp12_46*S18-ROcp12_710*C18);
  ROcp12_818 = -(ROcp12_56*S18-ROcp12_810*C18);
  ROcp12_918 = ROcp12_910*C18-S18*S6;
  ROcp12_419 = ROcp12_418*C19+ROcp12_718*S19;
  ROcp12_519 = ROcp12_518*C19+ROcp12_818*S19;
  ROcp12_619 = ROcp12_618*C19+ROcp12_918*S19;
  ROcp12_719 = -(ROcp12_418*S19-ROcp12_718*C19);
  ROcp12_819 = -(ROcp12_518*S19-ROcp12_818*C19);
  ROcp12_919 = -(ROcp12_618*S19-ROcp12_918*C19);
  ROcp12_120 = ROcp12_110*C20-ROcp12_719*S20;
  ROcp12_220 = ROcp12_210*C20-ROcp12_819*S20;
  ROcp12_320 = ROcp12_310*C20-ROcp12_919*S20;
  ROcp12_720 = ROcp12_110*S20+ROcp12_719*C20;
  ROcp12_820 = ROcp12_210*S20+ROcp12_819*C20;
  ROcp12_920 = ROcp12_310*S20+ROcp12_919*C20;
  ROcp12_121 = ROcp12_120*C21+ROcp12_419*S21;
  ROcp12_221 = ROcp12_220*C21+ROcp12_519*S21;
  ROcp12_321 = ROcp12_320*C21+ROcp12_619*S21;
  ROcp12_421 = -(ROcp12_120*S21-ROcp12_419*C21);
  ROcp12_521 = -(ROcp12_220*S21-ROcp12_519*C21);
  ROcp12_621 = -(ROcp12_320*S21-ROcp12_619*C21);
  ROcp12_422 = ROcp12_421*C22+ROcp12_720*S22;
  ROcp12_522 = ROcp12_521*C22+ROcp12_820*S22;
  ROcp12_622 = ROcp12_621*C22+ROcp12_920*S22;
  ROcp12_722 = -(ROcp12_421*S22-ROcp12_720*C22);
  ROcp12_822 = -(ROcp12_521*S22-ROcp12_820*C22);
  ROcp12_922 = -(ROcp12_621*S22-ROcp12_920*C22);
  ROcp12_123 = ROcp12_121*C23-ROcp12_722*S23;
  ROcp12_223 = ROcp12_221*C23-ROcp12_822*S23;
  ROcp12_323 = ROcp12_321*C23-ROcp12_922*S23;
  ROcp12_723 = ROcp12_121*S23+ROcp12_722*C23;
  ROcp12_823 = ROcp12_221*S23+ROcp12_822*C23;
  ROcp12_923 = ROcp12_321*S23+ROcp12_922*C23;
  RLcp12_118 = ROcp12_46*s->dpt[2][12]+ROcp12_710*s->dpt[3][12];
  RLcp12_218 = ROcp12_56*s->dpt[2][12]+ROcp12_810*s->dpt[3][12];
  RLcp12_318 = ROcp12_910*s->dpt[3][12]+s->dpt[2][12]*S6;
  OMcp12_118 = OMcp12_17+qd[18]*ROcp12_110;
  OMcp12_218 = OMcp12_27+qd[18]*ROcp12_210;
  OMcp12_318 = OMcp12_37+qd[18]*ROcp12_310;
  ORcp12_118 = OMcp12_27*RLcp12_318-OMcp12_37*RLcp12_218;
  ORcp12_218 = -(OMcp12_17*RLcp12_318-OMcp12_37*RLcp12_118);
  ORcp12_318 = OMcp12_17*RLcp12_218-OMcp12_27*RLcp12_118;
  OPcp12_118 = OPcp12_17+qd[18]*(OMcp12_27*ROcp12_310-OMcp12_37*ROcp12_210)+qdd[18]*ROcp12_110;
  OPcp12_218 = OPcp12_27-qd[18]*(OMcp12_17*ROcp12_310-OMcp12_37*ROcp12_110)+qdd[18]*ROcp12_210;
  OPcp12_318 = OPcp12_37+qd[18]*(OMcp12_17*ROcp12_210-OMcp12_27*ROcp12_110)+qdd[18]*ROcp12_310;
  RLcp12_119 = ROcp12_418*s->dpt[2][23];
  RLcp12_219 = ROcp12_518*s->dpt[2][23];
  RLcp12_319 = ROcp12_618*s->dpt[2][23];
  OMcp12_119 = OMcp12_118+qd[19]*ROcp12_110;
  OMcp12_219 = OMcp12_218+qd[19]*ROcp12_210;
  OMcp12_319 = OMcp12_318+qd[19]*ROcp12_310;
  ORcp12_119 = OMcp12_218*RLcp12_319-OMcp12_318*RLcp12_219;
  ORcp12_219 = -(OMcp12_118*RLcp12_319-OMcp12_318*RLcp12_119);
  ORcp12_319 = OMcp12_118*RLcp12_219-OMcp12_218*RLcp12_119;
  OMcp12_120 = OMcp12_119+qd[20]*ROcp12_419;
  OMcp12_220 = OMcp12_219+qd[20]*ROcp12_519;
  OMcp12_320 = OMcp12_319+qd[20]*ROcp12_619;
  OMcp12_121 = OMcp12_120+qd[21]*ROcp12_720;
  OMcp12_221 = OMcp12_220+qd[21]*ROcp12_820;
  OMcp12_321 = OMcp12_320+qd[21]*ROcp12_920;
  OPcp12_121 = OPcp12_118+qd[19]*(OMcp12_218*ROcp12_310-OMcp12_318*ROcp12_210)+qd[20]*(OMcp12_219*ROcp12_619-OMcp12_319*
 ROcp12_519)+qd[21]*(OMcp12_220*ROcp12_920-OMcp12_320*ROcp12_820)+qdd[19]*ROcp12_110+qdd[20]*ROcp12_419+qdd[21]*ROcp12_720;
  OPcp12_221 = OPcp12_218-qd[19]*(OMcp12_118*ROcp12_310-OMcp12_318*ROcp12_110)-qd[20]*(OMcp12_119*ROcp12_619-OMcp12_319*
 ROcp12_419)-qd[21]*(OMcp12_120*ROcp12_920-OMcp12_320*ROcp12_720)+qdd[19]*ROcp12_210+qdd[20]*ROcp12_519+qdd[21]*ROcp12_820;
  OPcp12_321 = OPcp12_318+qd[19]*(OMcp12_118*ROcp12_210-OMcp12_218*ROcp12_110)+qd[20]*(OMcp12_119*ROcp12_519-OMcp12_219*
 ROcp12_419)+qd[21]*(OMcp12_120*ROcp12_820-OMcp12_220*ROcp12_720)+qdd[19]*ROcp12_310+qdd[20]*ROcp12_619+qdd[21]*ROcp12_920;
  RLcp12_122 = ROcp12_121*s->dpt[1][27]+ROcp12_421*s->dpt[2][27]+ROcp12_720*s->dpt[3][27];
  RLcp12_222 = ROcp12_221*s->dpt[1][27]+ROcp12_521*s->dpt[2][27]+ROcp12_820*s->dpt[3][27];
  RLcp12_322 = ROcp12_321*s->dpt[1][27]+ROcp12_621*s->dpt[2][27]+ROcp12_920*s->dpt[3][27];
  ORcp12_122 = OMcp12_221*RLcp12_322-OMcp12_321*RLcp12_222;
  ORcp12_222 = -(OMcp12_121*RLcp12_322-OMcp12_321*RLcp12_122);
  ORcp12_322 = OMcp12_121*RLcp12_222-OMcp12_221*RLcp12_122;
  PxF3[1] = RLcp12_110+RLcp12_118+RLcp12_119+RLcp12_12+RLcp12_122+RLcp12_13;
  PxF3[2] = RLcp12_210+RLcp12_218+RLcp12_219+RLcp12_22+RLcp12_222+RLcp12_23;
  PxF3[3] = q[4]+RLcp12_310+RLcp12_318+RLcp12_319+RLcp12_322;
  RxF3[1][1] = ROcp12_123;
  RxF3[1][2] = ROcp12_223;
  RxF3[1][3] = ROcp12_323;
  RxF3[2][1] = ROcp12_422;
  RxF3[2][2] = ROcp12_522;
  RxF3[2][3] = ROcp12_622;
  RxF3[3][1] = ROcp12_723;
  RxF3[3][2] = ROcp12_823;
  RxF3[3][3] = ROcp12_923;
  VxF3[1] = ORcp12_110+ORcp12_118+ORcp12_119+ORcp12_12+ORcp12_122+ORcp12_13+qd[2]*C1-qd[3]*S1;
  VxF3[2] = ORcp12_210+ORcp12_218+ORcp12_219+ORcp12_22+ORcp12_222+ORcp12_23+qd[2]*S1+qd[3]*C1;
  VxF3[3] = qd[4]+ORcp12_310+ORcp12_318+ORcp12_319+ORcp12_322;
  OMxF3[1] = OMcp12_121+qd[23]*ROcp12_422;
  OMxF3[2] = OMcp12_221+qd[23]*ROcp12_522;
  OMxF3[3] = OMcp12_321+qd[23]*ROcp12_622;
  AxF3[1] = -(qd[1]*(ORcp12_22+ORcp12_23+(2.0)*qd[2]*S1+(2.0)*qd[3]*C1)-qdd[2]*C1+qdd[3]*S1-OMcp12_218*ORcp12_319-OMcp12_221*
 ORcp12_322-OMcp12_27*ORcp12_310-OMcp12_27*ORcp12_318+OMcp12_318*ORcp12_219+OMcp12_321*ORcp12_222+OMcp12_37*ORcp12_210+
 OMcp12_37*ORcp12_218-OPcp12_218*RLcp12_319-OPcp12_221*RLcp12_322-OPcp12_27*RLcp12_310-OPcp12_27*RLcp12_318+OPcp12_318*
 RLcp12_219+OPcp12_321*RLcp12_222+OPcp12_37*RLcp12_210+OPcp12_37*RLcp12_218);
  AxF3[2] = qd[1]*(ORcp12_12+ORcp12_13+(2.0)*qd[2]*C1-(2.0)*qd[3]*S1)+qdd[2]*S1+qdd[3]*C1-OMcp12_118*ORcp12_319-OMcp12_121*
 ORcp12_322-OMcp12_17*ORcp12_310-OMcp12_17*ORcp12_318+OMcp12_318*ORcp12_119+OMcp12_321*ORcp12_122+OMcp12_37*ORcp12_110+
 OMcp12_37*ORcp12_118-OPcp12_118*RLcp12_319-OPcp12_121*RLcp12_322-OPcp12_17*RLcp12_310-OPcp12_17*RLcp12_318+OPcp12_318*
 RLcp12_119+OPcp12_321*RLcp12_122+OPcp12_37*RLcp12_110+OPcp12_37*RLcp12_118;
  AxF3[3] = qdd[4]+OMcp12_118*ORcp12_219+OMcp12_121*ORcp12_222+OMcp12_17*ORcp12_210+OMcp12_17*ORcp12_218-OMcp12_218*
 ORcp12_119-OMcp12_221*ORcp12_122-OMcp12_27*ORcp12_110-OMcp12_27*ORcp12_118+OPcp12_118*RLcp12_219+OPcp12_121*RLcp12_222+
 OPcp12_17*RLcp12_210+OPcp12_17*RLcp12_218-OPcp12_218*RLcp12_119-OPcp12_221*RLcp12_122-OPcp12_27*RLcp12_110-OPcp12_27*
 RLcp12_118;
  OMPxF3[1] = OPcp12_121+qd[23]*(OMcp12_221*ROcp12_622-OMcp12_321*ROcp12_522)+qdd[23]*ROcp12_422;
  OMPxF3[2] = OPcp12_221-qd[23]*(OMcp12_121*ROcp12_622-OMcp12_321*ROcp12_422)+qdd[23]*ROcp12_522;
  OMPxF3[3] = OPcp12_321+qd[23]*(OMcp12_121*ROcp12_522-OMcp12_221*ROcp12_422)+qdd[23]*ROcp12_622;
 
// Sensor Forces Computation 

  SWr3 = user_ExtForces(PxF3,RxF3,VxF3,OMxF3,AxF3,OMPxF3,s,tsim,3);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc113 = ROcp12_123*SWr3[1]+ROcp12_223*SWr3[2]+ROcp12_323*SWr3[3];
  xfrc213 = ROcp12_422*SWr3[1]+ROcp12_522*SWr3[2]+ROcp12_622*SWr3[3];
  xfrc313 = ROcp12_723*SWr3[1]+ROcp12_823*SWr3[2]+ROcp12_923*SWr3[3];
  frc[1][23] = s->frc[1][23]+xfrc113;
  frc[2][23] = s->frc[2][23]+xfrc213;
  frc[3][23] = s->frc[3][23]+xfrc313;
  xtrq113 = ROcp12_123*SWr3[4]+ROcp12_223*SWr3[5]+ROcp12_323*SWr3[6];
  xtrq213 = ROcp12_422*SWr3[4]+ROcp12_522*SWr3[5]+ROcp12_622*SWr3[6];
  xtrq313 = ROcp12_723*SWr3[4]+ROcp12_823*SWr3[5]+ROcp12_923*SWr3[6];
  trq[1][23] = s->trq[1][23]+xtrq113-xfrc213*SWr3[9]+xfrc313*SWr3[8];
  trq[2][23] = s->trq[2][23]+xtrq213+xfrc113*SWr3[9]-xfrc313*SWr3[7];
  trq[3][23] = s->trq[3][23]+xtrq313-xfrc113*SWr3[8]+xfrc213*SWr3[7];

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
  frc[1][15] = s->frc[1][15];
  frc[2][15] = s->frc[2][15];
  frc[3][15] = s->frc[3][15];
  frc[1][18] = s->frc[1][18];
  frc[2][18] = s->frc[2][18];
  frc[3][18] = s->frc[3][18];
  frc[1][21] = s->frc[1][21];
  frc[2][21] = s->frc[2][21];
  frc[3][21] = s->frc[3][21];
  frc[1][25] = s->frc[1][25];
  frc[2][25] = s->frc[2][25];
  frc[3][25] = s->frc[3][25];
  frc[1][26] = s->frc[1][26];
  frc[2][26] = s->frc[2][26];
  frc[3][26] = s->frc[3][26];
  frc[1][28] = s->frc[1][28];
  frc[2][28] = s->frc[2][28];
  frc[3][28] = s->frc[3][28];
  frc[1][30] = s->frc[1][30];
  frc[2][30] = s->frc[2][30];
  frc[3][30] = s->frc[3][30];
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
  trq[1][15] = s->trq[1][15];
  trq[2][15] = s->trq[2][15];
  trq[3][15] = s->trq[3][15];
  trq[1][18] = s->trq[1][18];
  trq[2][18] = s->trq[2][18];
  trq[3][18] = s->trq[3][18];
  trq[1][21] = s->trq[1][21];
  trq[2][21] = s->trq[2][21];
  trq[3][21] = s->trq[3][21];
  trq[1][25] = s->trq[1][25];
  trq[2][25] = s->trq[2][25];
  trq[3][25] = s->trq[3][25];
  trq[1][26] = s->trq[1][26];
  trq[2][26] = s->trq[2][26];
  trq[3][26] = s->trq[3][26];
  trq[1][28] = s->trq[1][28];
  trq[2][28] = s->trq[2][28];
  trq[3][28] = s->trq[3][28];
  trq[1][30] = s->trq[1][30];
  trq[2][30] = s->trq[2][30];
  trq[3][30] = s->trq[3][30];

// ====== END Task 0 ====== 


}
 


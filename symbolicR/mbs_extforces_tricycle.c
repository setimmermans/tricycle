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
//	==> Generation Date : Tue Nov  8 17:14:02 2016
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
//	==> Post-Processing :  0.020 seconds
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

  S10p7 = C10*S7+S10*C7;
  C10p7 = C10*C7-S10*S7;
 
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

// = = Block_0_0_1_2_0_5 = = 
 
// Sensor Kinematics 


  ROcp12_412 = ROcp12_46*C12+ROcp12_710*S12;
  ROcp12_512 = ROcp12_56*C12+ROcp12_810*S12;
  ROcp12_612 = ROcp12_910*S12+C12*S6;
  ROcp12_712 = -(ROcp12_46*S12-ROcp12_710*C12);
  ROcp12_812 = -(ROcp12_56*S12-ROcp12_810*C12);
  ROcp12_912 = ROcp12_910*C12-S12*S6;
  ROcp12_413 = ROcp12_412*C13+ROcp12_712*S13;
  ROcp12_513 = ROcp12_512*C13+ROcp12_812*S13;
  ROcp12_613 = ROcp12_612*C13+ROcp12_912*S13;
  ROcp12_713 = -(ROcp12_412*S13-ROcp12_712*C13);
  ROcp12_813 = -(ROcp12_512*S13-ROcp12_812*C13);
  ROcp12_913 = -(ROcp12_612*S13-ROcp12_912*C13);
  ROcp12_114 = ROcp12_110*C14-ROcp12_713*S14;
  ROcp12_214 = ROcp12_210*C14-ROcp12_813*S14;
  ROcp12_314 = ROcp12_310*C14-ROcp12_913*S14;
  ROcp12_714 = ROcp12_110*S14+ROcp12_713*C14;
  ROcp12_814 = ROcp12_210*S14+ROcp12_813*C14;
  ROcp12_914 = ROcp12_310*S14+ROcp12_913*C14;
  ROcp12_115 = ROcp12_114*C15+ROcp12_413*S15;
  ROcp12_215 = ROcp12_214*C15+ROcp12_513*S15;
  ROcp12_315 = ROcp12_314*C15+ROcp12_613*S15;
  ROcp12_415 = -(ROcp12_114*S15-ROcp12_413*C15);
  ROcp12_515 = -(ROcp12_214*S15-ROcp12_513*C15);
  ROcp12_615 = -(ROcp12_314*S15-ROcp12_613*C15);
  ROcp12_416 = ROcp12_415*C16+ROcp12_714*S16;
  ROcp12_516 = ROcp12_515*C16+ROcp12_814*S16;
  ROcp12_616 = ROcp12_615*C16+ROcp12_914*S16;
  ROcp12_716 = -(ROcp12_415*S16-ROcp12_714*C16);
  ROcp12_816 = -(ROcp12_515*S16-ROcp12_814*C16);
  ROcp12_916 = -(ROcp12_615*S16-ROcp12_914*C16);
  ROcp12_117 = ROcp12_115*C17-ROcp12_716*S17;
  ROcp12_217 = ROcp12_215*C17-ROcp12_816*S17;
  ROcp12_317 = ROcp12_315*C17-ROcp12_916*S17;
  ROcp12_717 = ROcp12_115*S17+ROcp12_716*C17;
  ROcp12_817 = ROcp12_215*S17+ROcp12_816*C17;
  ROcp12_917 = ROcp12_315*S17+ROcp12_916*C17;
  RLcp12_112 = ROcp12_46*s->dpt[2][11]+ROcp12_710*s->dpt[3][11];
  RLcp12_212 = ROcp12_56*s->dpt[2][11]+ROcp12_810*s->dpt[3][11];
  RLcp12_312 = ROcp12_910*s->dpt[3][11]+s->dpt[2][11]*S6;
  OMcp12_112 = OMcp12_17+qd[12]*ROcp12_110;
  OMcp12_212 = OMcp12_27+qd[12]*ROcp12_210;
  OMcp12_312 = OMcp12_37+qd[12]*ROcp12_310;
  ORcp12_112 = OMcp12_27*RLcp12_312-OMcp12_37*RLcp12_212;
  ORcp12_212 = -(OMcp12_17*RLcp12_312-OMcp12_37*RLcp12_112);
  ORcp12_312 = OMcp12_17*RLcp12_212-OMcp12_27*RLcp12_112;
  OPcp12_112 = OPcp12_17+qd[12]*(OMcp12_27*ROcp12_310-OMcp12_37*ROcp12_210)+qdd[12]*ROcp12_110;
  OPcp12_212 = OPcp12_27-qd[12]*(OMcp12_17*ROcp12_310-OMcp12_37*ROcp12_110)+qdd[12]*ROcp12_210;
  OPcp12_312 = OPcp12_37+qd[12]*(OMcp12_17*ROcp12_210-OMcp12_27*ROcp12_110)+qdd[12]*ROcp12_310;
  RLcp12_113 = ROcp12_412*s->dpt[2][16];
  RLcp12_213 = ROcp12_512*s->dpt[2][16];
  RLcp12_313 = ROcp12_612*s->dpt[2][16];
  OMcp12_113 = OMcp12_112+qd[13]*ROcp12_110;
  OMcp12_213 = OMcp12_212+qd[13]*ROcp12_210;
  OMcp12_313 = OMcp12_312+qd[13]*ROcp12_310;
  ORcp12_113 = OMcp12_212*RLcp12_313-OMcp12_312*RLcp12_213;
  ORcp12_213 = -(OMcp12_112*RLcp12_313-OMcp12_312*RLcp12_113);
  ORcp12_313 = OMcp12_112*RLcp12_213-OMcp12_212*RLcp12_113;
  OMcp12_114 = OMcp12_113+qd[14]*ROcp12_413;
  OMcp12_214 = OMcp12_213+qd[14]*ROcp12_513;
  OMcp12_314 = OMcp12_313+qd[14]*ROcp12_613;
  OMcp12_115 = OMcp12_114+qd[15]*ROcp12_714;
  OMcp12_215 = OMcp12_214+qd[15]*ROcp12_814;
  OMcp12_315 = OMcp12_314+qd[15]*ROcp12_914;
  OPcp12_115 = OPcp12_112+qd[13]*(OMcp12_212*ROcp12_310-OMcp12_312*ROcp12_210)+qd[14]*(OMcp12_213*ROcp12_613-OMcp12_313*
 ROcp12_513)+qd[15]*(OMcp12_214*ROcp12_914-OMcp12_314*ROcp12_814)+qdd[13]*ROcp12_110+qdd[14]*ROcp12_413+qdd[15]*ROcp12_714;
  OPcp12_215 = OPcp12_212-qd[13]*(OMcp12_112*ROcp12_310-OMcp12_312*ROcp12_110)-qd[14]*(OMcp12_113*ROcp12_613-OMcp12_313*
 ROcp12_413)-qd[15]*(OMcp12_114*ROcp12_914-OMcp12_314*ROcp12_714)+qdd[13]*ROcp12_210+qdd[14]*ROcp12_513+qdd[15]*ROcp12_814;
  OPcp12_315 = OPcp12_312+qd[13]*(OMcp12_112*ROcp12_210-OMcp12_212*ROcp12_110)+qd[14]*(OMcp12_113*ROcp12_513-OMcp12_213*
 ROcp12_413)+qd[15]*(OMcp12_114*ROcp12_814-OMcp12_214*ROcp12_714)+qdd[13]*ROcp12_310+qdd[14]*ROcp12_613+qdd[15]*ROcp12_914;
  RLcp12_116 = ROcp12_115*s->dpt[1][20]+ROcp12_415*s->dpt[2][20]+ROcp12_714*s->dpt[3][20];
  RLcp12_216 = ROcp12_215*s->dpt[1][20]+ROcp12_515*s->dpt[2][20]+ROcp12_814*s->dpt[3][20];
  RLcp12_316 = ROcp12_315*s->dpt[1][20]+ROcp12_615*s->dpt[2][20]+ROcp12_914*s->dpt[3][20];
  ORcp12_116 = OMcp12_215*RLcp12_316-OMcp12_315*RLcp12_216;
  ORcp12_216 = -(OMcp12_115*RLcp12_316-OMcp12_315*RLcp12_116);
  ORcp12_316 = OMcp12_115*RLcp12_216-OMcp12_215*RLcp12_116;
  PxF2[1] = RLcp12_110+RLcp12_112+RLcp12_113+RLcp12_116+RLcp12_12+RLcp12_13;
  PxF2[2] = RLcp12_210+RLcp12_212+RLcp12_213+RLcp12_216+RLcp12_22+RLcp12_23;
  PxF2[3] = q[4]+RLcp12_310+RLcp12_312+RLcp12_313+RLcp12_316;
  RxF2[1][1] = ROcp12_117;
  RxF2[1][2] = ROcp12_217;
  RxF2[1][3] = ROcp12_317;
  RxF2[2][1] = ROcp12_416;
  RxF2[2][2] = ROcp12_516;
  RxF2[2][3] = ROcp12_616;
  RxF2[3][1] = ROcp12_717;
  RxF2[3][2] = ROcp12_817;
  RxF2[3][3] = ROcp12_917;
  VxF2[1] = ORcp12_110+ORcp12_112+ORcp12_113+ORcp12_116+ORcp12_12+ORcp12_13+qd[2]*C1-qd[3]*S1;
  VxF2[2] = ORcp12_210+ORcp12_212+ORcp12_213+ORcp12_216+ORcp12_22+ORcp12_23+qd[2]*S1+qd[3]*C1;
  VxF2[3] = qd[4]+ORcp12_310+ORcp12_312+ORcp12_313+ORcp12_316;
  OMxF2[1] = OMcp12_115+qd[17]*ROcp12_416;
  OMxF2[2] = OMcp12_215+qd[17]*ROcp12_516;
  OMxF2[3] = OMcp12_315+qd[17]*ROcp12_616;
  AxF2[1] = -(qd[1]*(ORcp12_22+ORcp12_23+(2.0)*qd[2]*S1+(2.0)*qd[3]*C1)-qdd[2]*C1+qdd[3]*S1-OMcp12_212*ORcp12_313-OMcp12_215*
 ORcp12_316-OMcp12_27*ORcp12_310-OMcp12_27*ORcp12_312+OMcp12_312*ORcp12_213+OMcp12_315*ORcp12_216+OMcp12_37*ORcp12_210+
 OMcp12_37*ORcp12_212-OPcp12_212*RLcp12_313-OPcp12_215*RLcp12_316-OPcp12_27*RLcp12_310-OPcp12_27*RLcp12_312+OPcp12_312*
 RLcp12_213+OPcp12_315*RLcp12_216+OPcp12_37*RLcp12_210+OPcp12_37*RLcp12_212);
  AxF2[2] = qd[1]*(ORcp12_12+ORcp12_13+(2.0)*qd[2]*C1-(2.0)*qd[3]*S1)+qdd[2]*S1+qdd[3]*C1-OMcp12_112*ORcp12_313-OMcp12_115*
 ORcp12_316-OMcp12_17*ORcp12_310-OMcp12_17*ORcp12_312+OMcp12_312*ORcp12_113+OMcp12_315*ORcp12_116+OMcp12_37*ORcp12_110+
 OMcp12_37*ORcp12_112-OPcp12_112*RLcp12_313-OPcp12_115*RLcp12_316-OPcp12_17*RLcp12_310-OPcp12_17*RLcp12_312+OPcp12_312*
 RLcp12_113+OPcp12_315*RLcp12_116+OPcp12_37*RLcp12_110+OPcp12_37*RLcp12_112;
  AxF2[3] = qdd[4]+OMcp12_112*ORcp12_213+OMcp12_115*ORcp12_216+OMcp12_17*ORcp12_210+OMcp12_17*ORcp12_212-OMcp12_212*
 ORcp12_113-OMcp12_215*ORcp12_116-OMcp12_27*ORcp12_110-OMcp12_27*ORcp12_112+OPcp12_112*RLcp12_213+OPcp12_115*RLcp12_216+
 OPcp12_17*RLcp12_210+OPcp12_17*RLcp12_212-OPcp12_212*RLcp12_113-OPcp12_215*RLcp12_116-OPcp12_27*RLcp12_110-OPcp12_27*
 RLcp12_112;
  OMPxF2[1] = OPcp12_115+qd[17]*(OMcp12_215*ROcp12_616-OMcp12_315*ROcp12_516)+qdd[17]*ROcp12_416;
  OMPxF2[2] = OPcp12_215-qd[17]*(OMcp12_115*ROcp12_616-OMcp12_315*ROcp12_416)+qdd[17]*ROcp12_516;
  OMPxF2[3] = OPcp12_315+qd[17]*(OMcp12_115*ROcp12_516-OMcp12_215*ROcp12_416)+qdd[17]*ROcp12_616;
 
// Sensor Forces Computation 

  SWr2 = user_ExtForces(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc113 = ROcp12_117*SWr2[1]+ROcp12_217*SWr2[2]+ROcp12_317*SWr2[3];
  xfrc213 = ROcp12_416*SWr2[1]+ROcp12_516*SWr2[2]+ROcp12_616*SWr2[3];
  xfrc313 = ROcp12_717*SWr2[1]+ROcp12_817*SWr2[2]+ROcp12_917*SWr2[3];
  frc[1][17] = s->frc[1][17]+xfrc113;
  frc[2][17] = s->frc[2][17]+xfrc213;
  frc[3][17] = s->frc[3][17]+xfrc313;
  xtrq113 = ROcp12_117*SWr2[4]+ROcp12_217*SWr2[5]+ROcp12_317*SWr2[6];
  xtrq213 = ROcp12_416*SWr2[4]+ROcp12_516*SWr2[5]+ROcp12_616*SWr2[6];
  xtrq313 = ROcp12_717*SWr2[4]+ROcp12_817*SWr2[5]+ROcp12_917*SWr2[6];
  trq[1][17] = s->trq[1][17]+xtrq113-xfrc213*SWr2[9]+xfrc313*SWr2[8];
  trq[2][17] = s->trq[2][17]+xtrq213+xfrc113*SWr2[9]-xfrc313*SWr2[7];
  trq[3][17] = s->trq[3][17]+xtrq313-xfrc113*SWr2[8]+xfrc213*SWr2[7];

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


  ROcp13_110 = ROcp13_17*C10-ROcp13_77*S10;
  ROcp13_210 = ROcp13_27*C10-ROcp13_87*S10;
  ROcp13_310 = -S10p7*C6;
  ROcp13_710 = ROcp13_17*S10+ROcp13_77*C10;
  ROcp13_810 = ROcp13_27*S10+ROcp13_87*C10;
  ROcp13_910 = C10p7*C6;
  RLcp13_110 = ROcp13_17*s->dpt[1][5]+ROcp13_77*s->dpt[3][5];
  RLcp13_210 = ROcp13_27*s->dpt[1][5]+ROcp13_87*s->dpt[3][5];
  RLcp13_310 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
  ORcp13_110 = OMcp13_27*RLcp13_310-OMcp13_37*RLcp13_210;
  ORcp13_210 = -(OMcp13_17*RLcp13_310-OMcp13_37*RLcp13_110);
  ORcp13_310 = OMcp13_17*RLcp13_210-OMcp13_27*RLcp13_110;

// = = Block_0_0_1_3_0_6 = = 
 
// Sensor Kinematics 


  ROcp13_418 = ROcp13_46*C18+ROcp13_710*S18;
  ROcp13_518 = ROcp13_56*C18+ROcp13_810*S18;
  ROcp13_618 = ROcp13_910*S18+C18*S6;
  ROcp13_718 = -(ROcp13_46*S18-ROcp13_710*C18);
  ROcp13_818 = -(ROcp13_56*S18-ROcp13_810*C18);
  ROcp13_918 = ROcp13_910*C18-S18*S6;
  ROcp13_419 = ROcp13_418*C19+ROcp13_718*S19;
  ROcp13_519 = ROcp13_518*C19+ROcp13_818*S19;
  ROcp13_619 = ROcp13_618*C19+ROcp13_918*S19;
  ROcp13_719 = -(ROcp13_418*S19-ROcp13_718*C19);
  ROcp13_819 = -(ROcp13_518*S19-ROcp13_818*C19);
  ROcp13_919 = -(ROcp13_618*S19-ROcp13_918*C19);
  ROcp13_120 = ROcp13_110*C20-ROcp13_719*S20;
  ROcp13_220 = ROcp13_210*C20-ROcp13_819*S20;
  ROcp13_320 = ROcp13_310*C20-ROcp13_919*S20;
  ROcp13_720 = ROcp13_110*S20+ROcp13_719*C20;
  ROcp13_820 = ROcp13_210*S20+ROcp13_819*C20;
  ROcp13_920 = ROcp13_310*S20+ROcp13_919*C20;
  ROcp13_121 = ROcp13_120*C21+ROcp13_419*S21;
  ROcp13_221 = ROcp13_220*C21+ROcp13_519*S21;
  ROcp13_321 = ROcp13_320*C21+ROcp13_619*S21;
  ROcp13_421 = -(ROcp13_120*S21-ROcp13_419*C21);
  ROcp13_521 = -(ROcp13_220*S21-ROcp13_519*C21);
  ROcp13_621 = -(ROcp13_320*S21-ROcp13_619*C21);
  ROcp13_422 = ROcp13_421*C22+ROcp13_720*S22;
  ROcp13_522 = ROcp13_521*C22+ROcp13_820*S22;
  ROcp13_622 = ROcp13_621*C22+ROcp13_920*S22;
  ROcp13_722 = -(ROcp13_421*S22-ROcp13_720*C22);
  ROcp13_822 = -(ROcp13_521*S22-ROcp13_820*C22);
  ROcp13_922 = -(ROcp13_621*S22-ROcp13_920*C22);
  ROcp13_123 = ROcp13_121*C23-ROcp13_722*S23;
  ROcp13_223 = ROcp13_221*C23-ROcp13_822*S23;
  ROcp13_323 = ROcp13_321*C23-ROcp13_922*S23;
  ROcp13_723 = ROcp13_121*S23+ROcp13_722*C23;
  ROcp13_823 = ROcp13_221*S23+ROcp13_822*C23;
  ROcp13_923 = ROcp13_321*S23+ROcp13_922*C23;
  RLcp13_118 = ROcp13_46*s->dpt[2][12]+ROcp13_710*s->dpt[3][12];
  RLcp13_218 = ROcp13_56*s->dpt[2][12]+ROcp13_810*s->dpt[3][12];
  RLcp13_318 = ROcp13_910*s->dpt[3][12]+s->dpt[2][12]*S6;
  OMcp13_118 = OMcp13_17+qd[18]*ROcp13_110;
  OMcp13_218 = OMcp13_27+qd[18]*ROcp13_210;
  OMcp13_318 = OMcp13_37+qd[18]*ROcp13_310;
  ORcp13_118 = OMcp13_27*RLcp13_318-OMcp13_37*RLcp13_218;
  ORcp13_218 = -(OMcp13_17*RLcp13_318-OMcp13_37*RLcp13_118);
  ORcp13_318 = OMcp13_17*RLcp13_218-OMcp13_27*RLcp13_118;
  OPcp13_118 = OPcp13_17+qd[18]*(OMcp13_27*ROcp13_310-OMcp13_37*ROcp13_210)+qdd[18]*ROcp13_110;
  OPcp13_218 = OPcp13_27-qd[18]*(OMcp13_17*ROcp13_310-OMcp13_37*ROcp13_110)+qdd[18]*ROcp13_210;
  OPcp13_318 = OPcp13_37+qd[18]*(OMcp13_17*ROcp13_210-OMcp13_27*ROcp13_110)+qdd[18]*ROcp13_310;
  RLcp13_119 = ROcp13_418*s->dpt[2][23];
  RLcp13_219 = ROcp13_518*s->dpt[2][23];
  RLcp13_319 = ROcp13_618*s->dpt[2][23];
  OMcp13_119 = OMcp13_118+qd[19]*ROcp13_110;
  OMcp13_219 = OMcp13_218+qd[19]*ROcp13_210;
  OMcp13_319 = OMcp13_318+qd[19]*ROcp13_310;
  ORcp13_119 = OMcp13_218*RLcp13_319-OMcp13_318*RLcp13_219;
  ORcp13_219 = -(OMcp13_118*RLcp13_319-OMcp13_318*RLcp13_119);
  ORcp13_319 = OMcp13_118*RLcp13_219-OMcp13_218*RLcp13_119;
  OMcp13_120 = OMcp13_119+qd[20]*ROcp13_419;
  OMcp13_220 = OMcp13_219+qd[20]*ROcp13_519;
  OMcp13_320 = OMcp13_319+qd[20]*ROcp13_619;
  OMcp13_121 = OMcp13_120+qd[21]*ROcp13_720;
  OMcp13_221 = OMcp13_220+qd[21]*ROcp13_820;
  OMcp13_321 = OMcp13_320+qd[21]*ROcp13_920;
  OPcp13_121 = OPcp13_118+qd[19]*(OMcp13_218*ROcp13_310-OMcp13_318*ROcp13_210)+qd[20]*(OMcp13_219*ROcp13_619-OMcp13_319*
 ROcp13_519)+qd[21]*(OMcp13_220*ROcp13_920-OMcp13_320*ROcp13_820)+qdd[19]*ROcp13_110+qdd[20]*ROcp13_419+qdd[21]*ROcp13_720;
  OPcp13_221 = OPcp13_218-qd[19]*(OMcp13_118*ROcp13_310-OMcp13_318*ROcp13_110)-qd[20]*(OMcp13_119*ROcp13_619-OMcp13_319*
 ROcp13_419)-qd[21]*(OMcp13_120*ROcp13_920-OMcp13_320*ROcp13_720)+qdd[19]*ROcp13_210+qdd[20]*ROcp13_519+qdd[21]*ROcp13_820;
  OPcp13_321 = OPcp13_318+qd[19]*(OMcp13_118*ROcp13_210-OMcp13_218*ROcp13_110)+qd[20]*(OMcp13_119*ROcp13_519-OMcp13_219*
 ROcp13_419)+qd[21]*(OMcp13_120*ROcp13_820-OMcp13_220*ROcp13_720)+qdd[19]*ROcp13_310+qdd[20]*ROcp13_619+qdd[21]*ROcp13_920;
  RLcp13_122 = ROcp13_121*s->dpt[1][27]+ROcp13_421*s->dpt[2][27]+ROcp13_720*s->dpt[3][27];
  RLcp13_222 = ROcp13_221*s->dpt[1][27]+ROcp13_521*s->dpt[2][27]+ROcp13_820*s->dpt[3][27];
  RLcp13_322 = ROcp13_321*s->dpt[1][27]+ROcp13_621*s->dpt[2][27]+ROcp13_920*s->dpt[3][27];
  ORcp13_122 = OMcp13_221*RLcp13_322-OMcp13_321*RLcp13_222;
  ORcp13_222 = -(OMcp13_121*RLcp13_322-OMcp13_321*RLcp13_122);
  ORcp13_322 = OMcp13_121*RLcp13_222-OMcp13_221*RLcp13_122;
  PxF3[1] = RLcp13_110+RLcp13_118+RLcp13_119+RLcp13_12+RLcp13_122+RLcp13_13;
  PxF3[2] = RLcp13_210+RLcp13_218+RLcp13_219+RLcp13_22+RLcp13_222+RLcp13_23;
  PxF3[3] = q[4]+RLcp13_310+RLcp13_318+RLcp13_319+RLcp13_322;
  RxF3[1][1] = ROcp13_123;
  RxF3[1][2] = ROcp13_223;
  RxF3[1][3] = ROcp13_323;
  RxF3[2][1] = ROcp13_422;
  RxF3[2][2] = ROcp13_522;
  RxF3[2][3] = ROcp13_622;
  RxF3[3][1] = ROcp13_723;
  RxF3[3][2] = ROcp13_823;
  RxF3[3][3] = ROcp13_923;
  VxF3[1] = ORcp13_110+ORcp13_118+ORcp13_119+ORcp13_12+ORcp13_122+ORcp13_13+qd[2]*C1-qd[3]*S1;
  VxF3[2] = ORcp13_210+ORcp13_218+ORcp13_219+ORcp13_22+ORcp13_222+ORcp13_23+qd[2]*S1+qd[3]*C1;
  VxF3[3] = qd[4]+ORcp13_310+ORcp13_318+ORcp13_319+ORcp13_322;
  OMxF3[1] = OMcp13_121+qd[23]*ROcp13_422;
  OMxF3[2] = OMcp13_221+qd[23]*ROcp13_522;
  OMxF3[3] = OMcp13_321+qd[23]*ROcp13_622;
  AxF3[1] = -(qd[1]*(ORcp13_22+ORcp13_23+(2.0)*qd[2]*S1+(2.0)*qd[3]*C1)-qdd[2]*C1+qdd[3]*S1-OMcp13_218*ORcp13_319-OMcp13_221*
 ORcp13_322-OMcp13_27*ORcp13_310-OMcp13_27*ORcp13_318+OMcp13_318*ORcp13_219+OMcp13_321*ORcp13_222+OMcp13_37*ORcp13_210+
 OMcp13_37*ORcp13_218-OPcp13_218*RLcp13_319-OPcp13_221*RLcp13_322-OPcp13_27*RLcp13_310-OPcp13_27*RLcp13_318+OPcp13_318*
 RLcp13_219+OPcp13_321*RLcp13_222+OPcp13_37*RLcp13_210+OPcp13_37*RLcp13_218);
  AxF3[2] = qd[1]*(ORcp13_12+ORcp13_13+(2.0)*qd[2]*C1-(2.0)*qd[3]*S1)+qdd[2]*S1+qdd[3]*C1-OMcp13_118*ORcp13_319-OMcp13_121*
 ORcp13_322-OMcp13_17*ORcp13_310-OMcp13_17*ORcp13_318+OMcp13_318*ORcp13_119+OMcp13_321*ORcp13_122+OMcp13_37*ORcp13_110+
 OMcp13_37*ORcp13_118-OPcp13_118*RLcp13_319-OPcp13_121*RLcp13_322-OPcp13_17*RLcp13_310-OPcp13_17*RLcp13_318+OPcp13_318*
 RLcp13_119+OPcp13_321*RLcp13_122+OPcp13_37*RLcp13_110+OPcp13_37*RLcp13_118;
  AxF3[3] = qdd[4]+OMcp13_118*ORcp13_219+OMcp13_121*ORcp13_222+OMcp13_17*ORcp13_210+OMcp13_17*ORcp13_218-OMcp13_218*
 ORcp13_119-OMcp13_221*ORcp13_122-OMcp13_27*ORcp13_110-OMcp13_27*ORcp13_118+OPcp13_118*RLcp13_219+OPcp13_121*RLcp13_222+
 OPcp13_17*RLcp13_210+OPcp13_17*RLcp13_218-OPcp13_218*RLcp13_119-OPcp13_221*RLcp13_122-OPcp13_27*RLcp13_110-OPcp13_27*
 RLcp13_118;
  OMPxF3[1] = OPcp13_121+qd[23]*(OMcp13_221*ROcp13_622-OMcp13_321*ROcp13_522)+qdd[23]*ROcp13_422;
  OMPxF3[2] = OPcp13_221-qd[23]*(OMcp13_121*ROcp13_622-OMcp13_321*ROcp13_422)+qdd[23]*ROcp13_522;
  OMPxF3[3] = OPcp13_321+qd[23]*(OMcp13_121*ROcp13_522-OMcp13_221*ROcp13_422)+qdd[23]*ROcp13_622;
 
// Sensor Forces Computation 

  SWr3 = user_ExtForces(PxF3,RxF3,VxF3,OMxF3,AxF3,OMPxF3,s,tsim,3);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc114 = ROcp13_123*SWr3[1]+ROcp13_223*SWr3[2]+ROcp13_323*SWr3[3];
  xfrc214 = ROcp13_422*SWr3[1]+ROcp13_522*SWr3[2]+ROcp13_622*SWr3[3];
  xfrc314 = ROcp13_723*SWr3[1]+ROcp13_823*SWr3[2]+ROcp13_923*SWr3[3];
  frc[1][23] = s->frc[1][23]+xfrc114;
  frc[2][23] = s->frc[2][23]+xfrc214;
  frc[3][23] = s->frc[3][23]+xfrc314;
  xtrq114 = ROcp13_123*SWr3[4]+ROcp13_223*SWr3[5]+ROcp13_323*SWr3[6];
  xtrq214 = ROcp13_422*SWr3[4]+ROcp13_522*SWr3[5]+ROcp13_622*SWr3[6];
  xtrq314 = ROcp13_723*SWr3[4]+ROcp13_823*SWr3[5]+ROcp13_923*SWr3[6];
  trq[1][23] = s->trq[1][23]+xtrq114-xfrc214*SWr3[9]+xfrc314*SWr3[8];
  trq[2][23] = s->trq[2][23]+xtrq214+xfrc114*SWr3[9]-xfrc314*SWr3[7];
  trq[3][23] = s->trq[3][23]+xtrq314-xfrc114*SWr3[8]+xfrc214*SWr3[7];

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
 


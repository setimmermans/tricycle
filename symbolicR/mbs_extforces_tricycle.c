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
//	==> Generation Date : Tue Oct 11 17:37:02 2016
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


  ROcp3_46 = -S1p5*C6;
  ROcp3_56 = C1p5*C6;
  ROcp3_76 = S1p5*S6;
  ROcp3_86 = -C1p5*S6;
  ROcp3_17 = -(ROcp3_76*S7-C1p5*C7);
  ROcp3_27 = -(ROcp3_86*S7-S1p5*C7);
  ROcp3_77 = ROcp3_76*C7+C1p5*S7;
  ROcp3_87 = ROcp3_86*C7+S1p5*S7;
  RLcp3_12 = q[2]*C1;
  RLcp3_22 = q[2]*S1;
  ORcp3_12 = -qd[1]*RLcp3_22;
  ORcp3_22 = qd[1]*RLcp3_12;
  RLcp3_13 = -q[3]*S1;
  RLcp3_23 = q[3]*C1;
  ORcp3_13 = -qd[1]*RLcp3_23;
  ORcp3_23 = qd[1]*RLcp3_13;
  OMcp3_35 = qd[1]+qd[5];
  OMcp3_16 = qd[6]*C1p5;
  OMcp3_26 = qd[6]*S1p5;
  OMcp3_17 = OMcp3_16+qd[7]*ROcp3_46;
  OMcp3_27 = OMcp3_26+qd[7]*ROcp3_56;
  OMcp3_37 = OMcp3_35+qd[7]*S6;
  OPcp3_17 = -(qd[6]*OMcp3_35*S1p5-qd[7]*(OMcp3_26*S6-OMcp3_35*ROcp3_56)-qdd[6]*C1p5-qdd[7]*ROcp3_46);
  OPcp3_27 = qd[6]*OMcp3_35*C1p5-qd[7]*(OMcp3_16*S6-OMcp3_35*ROcp3_46)+qdd[6]*S1p5+qdd[7]*ROcp3_56;
  OPcp3_37 = qdd[5]+qd[6]*qd[7]*C6+qdd[7]*S6;

// = = Block_0_0_1_1_0_2 = = 
 
// Trigonometric Variables  

  S7p8 = C7*S8+S7*C8;
  C7p8 = C7*C8-S7*S8;
  S7p8p9 = C7p8*S9+S7p8*C9;
  C7p8p9 = C7p8*C9-S7p8*S9;
 
// Sensor Kinematics 


  ROcp3_18 = ROcp3_17*C8-ROcp3_77*S8;
  ROcp3_28 = ROcp3_27*C8-ROcp3_87*S8;
  ROcp3_78 = ROcp3_17*S8+ROcp3_77*C8;
  ROcp3_88 = ROcp3_27*S8+ROcp3_87*C8;
  ROcp3_19 = ROcp3_18*C9-ROcp3_78*S9;
  ROcp3_29 = ROcp3_28*C9-ROcp3_88*S9;
  ROcp3_39 = -S7p8p9*C6;
  ROcp3_79 = ROcp3_18*S9+ROcp3_78*C9;
  ROcp3_89 = ROcp3_28*S9+ROcp3_88*C9;
  ROcp3_99 = C7p8p9*C6;
  RLcp3_18 = ROcp3_17*s->dpt[1][2]+ROcp3_77*s->dpt[3][2];
  RLcp3_28 = ROcp3_27*s->dpt[1][2]+ROcp3_87*s->dpt[3][2];
  RLcp3_38 = -C6*(s->dpt[1][2]*S7-s->dpt[3][2]*C7);
  OMcp3_18 = OMcp3_17+qd[8]*ROcp3_46;
  OMcp3_28 = OMcp3_27+qd[8]*ROcp3_56;
  OMcp3_38 = OMcp3_37+qd[8]*S6;
  ORcp3_18 = OMcp3_27*RLcp3_38-OMcp3_37*RLcp3_28;
  ORcp3_28 = -(OMcp3_17*RLcp3_38-OMcp3_37*RLcp3_18);
  ORcp3_38 = OMcp3_17*RLcp3_28-OMcp3_27*RLcp3_18;
  OPcp3_18 = OPcp3_17+qd[8]*(OMcp3_27*S6-OMcp3_37*ROcp3_56)+qdd[8]*ROcp3_46;
  OPcp3_28 = OPcp3_27-qd[8]*(OMcp3_17*S6-OMcp3_37*ROcp3_46)+qdd[8]*ROcp3_56;
  OPcp3_38 = OPcp3_37+qd[8]*(OMcp3_17*ROcp3_56-OMcp3_27*ROcp3_46)+qdd[8]*S6;
  RLcp3_19 = ROcp3_18*s->dpt[1][8];
  RLcp3_29 = ROcp3_28*s->dpt[1][8];
  RLcp3_39 = -s->dpt[1][8]*S7p8*C6;
  ORcp3_19 = OMcp3_28*RLcp3_39-OMcp3_38*RLcp3_29;
  ORcp3_29 = -(OMcp3_18*RLcp3_39-OMcp3_38*RLcp3_19);
  ORcp3_39 = OMcp3_18*RLcp3_29-OMcp3_28*RLcp3_19;
  PxF1[1] = RLcp3_12+RLcp3_13+RLcp3_18+RLcp3_19;
  PxF1[2] = RLcp3_22+RLcp3_23+RLcp3_28+RLcp3_29;
  PxF1[3] = q[4]+RLcp3_38+RLcp3_39;
  RxF1[1][1] = ROcp3_19;
  RxF1[1][2] = ROcp3_29;
  RxF1[1][3] = ROcp3_39;
  RxF1[2][1] = ROcp3_46;
  RxF1[2][2] = ROcp3_56;
  RxF1[2][3] = S6;
  RxF1[3][1] = ROcp3_79;
  RxF1[3][2] = ROcp3_89;
  RxF1[3][3] = ROcp3_99;
  VxF1[1] = ORcp3_12+ORcp3_13+ORcp3_18+ORcp3_19+qd[2]*C1-qd[3]*S1;
  VxF1[2] = ORcp3_22+ORcp3_23+ORcp3_28+ORcp3_29+qd[2]*S1+qd[3]*C1;
  VxF1[3] = qd[4]+ORcp3_38+ORcp3_39;
  OMxF1[1] = OMcp3_18+qd[9]*ROcp3_46;
  OMxF1[2] = OMcp3_28+qd[9]*ROcp3_56;
  OMxF1[3] = OMcp3_38+qd[9]*S6;
  AxF1[1] = -(qd[1]*(ORcp3_22+ORcp3_23+(2.0)*qd[2]*S1+(2.0)*qd[3]*C1)-qdd[2]*C1+qdd[3]*S1-OMcp3_27*ORcp3_38-OMcp3_28*ORcp3_39+
 OMcp3_37*ORcp3_28+OMcp3_38*ORcp3_29-OPcp3_27*RLcp3_38-OPcp3_28*RLcp3_39+OPcp3_37*RLcp3_28+OPcp3_38*RLcp3_29);
  AxF1[2] = qd[1]*(ORcp3_12+ORcp3_13+(2.0)*qd[2]*C1-(2.0)*qd[3]*S1)+qdd[2]*S1+qdd[3]*C1-OMcp3_17*ORcp3_38-OMcp3_18*ORcp3_39+
 OMcp3_37*ORcp3_18+OMcp3_38*ORcp3_19-OPcp3_17*RLcp3_38-OPcp3_18*RLcp3_39+OPcp3_37*RLcp3_18+OPcp3_38*RLcp3_19;
  AxF1[3] = qdd[4]+OMcp3_17*ORcp3_28+OMcp3_18*ORcp3_29-OMcp3_27*ORcp3_18-OMcp3_28*ORcp3_19+OPcp3_17*RLcp3_28+OPcp3_18*
 RLcp3_29-OPcp3_27*RLcp3_18-OPcp3_28*RLcp3_19;
  OMPxF1[1] = OPcp3_18+qd[9]*(OMcp3_28*S6-OMcp3_38*ROcp3_56)+qdd[9]*ROcp3_46;
  OMPxF1[2] = OPcp3_28-qd[9]*(OMcp3_18*S6-OMcp3_38*ROcp3_46)+qdd[9]*ROcp3_56;
  OMPxF1[3] = OPcp3_38+qd[9]*(OMcp3_18*ROcp3_56-OMcp3_28*ROcp3_46)+qdd[9]*S6;
 
// Sensor Forces Computation 

  SWr1 = user_ExtForces(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc14 = ROcp3_19*SWr1[1]+ROcp3_29*SWr1[2]+ROcp3_39*SWr1[3];
  xfrc24 = ROcp3_46*SWr1[1]+ROcp3_56*SWr1[2]+SWr1[3]*S6;
  xfrc34 = ROcp3_79*SWr1[1]+ROcp3_89*SWr1[2]+ROcp3_99*SWr1[3];
  frc[1][9] = s->frc[1][9]+xfrc14;
  frc[2][9] = s->frc[2][9]+xfrc24;
  frc[3][9] = s->frc[3][9]+xfrc34;
  xtrq14 = ROcp3_19*SWr1[4]+ROcp3_29*SWr1[5]+ROcp3_39*SWr1[6];
  xtrq24 = ROcp3_46*SWr1[4]+ROcp3_56*SWr1[5]+SWr1[6]*S6;
  xtrq34 = ROcp3_79*SWr1[4]+ROcp3_89*SWr1[5]+ROcp3_99*SWr1[6];
  trq[1][9] = s->trq[1][9]+xtrq14-xfrc24*SWr1[9]+xfrc34*SWr1[8];
  trq[2][9] = s->trq[2][9]+xtrq24+xfrc14*SWr1[9]-xfrc34*SWr1[7];
  trq[3][9] = s->trq[3][9]+xtrq34-xfrc14*SWr1[8]+xfrc24*SWr1[7];

// = = Block_0_0_1_2_0_1 = = 
 
// Sensor Kinematics 


  ROcp4_46 = -S1p5*C6;
  ROcp4_56 = C1p5*C6;
  ROcp4_76 = S1p5*S6;
  ROcp4_86 = -C1p5*S6;
  ROcp4_17 = -(ROcp4_76*S7-C1p5*C7);
  ROcp4_27 = -(ROcp4_86*S7-S1p5*C7);
  ROcp4_77 = ROcp4_76*C7+C1p5*S7;
  ROcp4_87 = ROcp4_86*C7+S1p5*S7;
  RLcp4_12 = q[2]*C1;
  RLcp4_22 = q[2]*S1;
  ORcp4_12 = -qd[1]*RLcp4_22;
  ORcp4_22 = qd[1]*RLcp4_12;
  RLcp4_13 = -q[3]*S1;
  RLcp4_23 = q[3]*C1;
  ORcp4_13 = -qd[1]*RLcp4_23;
  ORcp4_23 = qd[1]*RLcp4_13;
  OMcp4_35 = qd[1]+qd[5];
  OMcp4_16 = qd[6]*C1p5;
  OMcp4_26 = qd[6]*S1p5;
  OMcp4_17 = OMcp4_16+qd[7]*ROcp4_46;
  OMcp4_27 = OMcp4_26+qd[7]*ROcp4_56;
  OMcp4_37 = OMcp4_35+qd[7]*S6;
  OPcp4_17 = -(qd[6]*OMcp4_35*S1p5-qd[7]*(OMcp4_26*S6-OMcp4_35*ROcp4_56)-qdd[6]*C1p5-qdd[7]*ROcp4_46);
  OPcp4_27 = qd[6]*OMcp4_35*C1p5-qd[7]*(OMcp4_16*S6-OMcp4_35*ROcp4_46)+qdd[6]*S1p5+qdd[7]*ROcp4_56;
  OPcp4_37 = qdd[5]+qd[6]*qd[7]*C6+qdd[7]*S6;

// = = Block_0_0_1_2_0_3 = = 
 
// Trigonometric Variables  

  S10p7 = C10*S7+S10*C7;
  C10p7 = C10*C7-S10*S7;
 
// Sensor Kinematics 


  ROcp4_110 = ROcp4_17*C10-ROcp4_77*S10;
  ROcp4_210 = ROcp4_27*C10-ROcp4_87*S10;
  ROcp4_310 = -S10p7*C6;
  ROcp4_710 = ROcp4_17*S10+ROcp4_77*C10;
  ROcp4_810 = ROcp4_27*S10+ROcp4_87*C10;
  ROcp4_910 = C10p7*C6;
  RLcp4_110 = ROcp4_17*s->dpt[1][5]+ROcp4_77*s->dpt[3][5];
  RLcp4_210 = ROcp4_27*s->dpt[1][5]+ROcp4_87*s->dpt[3][5];
  RLcp4_310 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
  ORcp4_110 = OMcp4_27*RLcp4_310-OMcp4_37*RLcp4_210;
  ORcp4_210 = -(OMcp4_17*RLcp4_310-OMcp4_37*RLcp4_110);
  ORcp4_310 = OMcp4_17*RLcp4_210-OMcp4_27*RLcp4_110;

// = = Block_0_0_1_2_0_5 = = 
 
// Sensor Kinematics 


  ROcp4_412 = ROcp4_46*C12+ROcp4_710*S12;
  ROcp4_512 = ROcp4_56*C12+ROcp4_810*S12;
  ROcp4_612 = ROcp4_910*S12+C12*S6;
  ROcp4_712 = -(ROcp4_46*S12-ROcp4_710*C12);
  ROcp4_812 = -(ROcp4_56*S12-ROcp4_810*C12);
  ROcp4_912 = ROcp4_910*C12-S12*S6;
  ROcp4_413 = ROcp4_412*C13+ROcp4_712*S13;
  ROcp4_513 = ROcp4_512*C13+ROcp4_812*S13;
  ROcp4_613 = ROcp4_612*C13+ROcp4_912*S13;
  ROcp4_713 = -(ROcp4_412*S13-ROcp4_712*C13);
  ROcp4_813 = -(ROcp4_512*S13-ROcp4_812*C13);
  ROcp4_913 = -(ROcp4_612*S13-ROcp4_912*C13);
  ROcp4_114 = ROcp4_110*C14-ROcp4_713*S14;
  ROcp4_214 = ROcp4_210*C14-ROcp4_813*S14;
  ROcp4_314 = ROcp4_310*C14-ROcp4_913*S14;
  ROcp4_714 = ROcp4_110*S14+ROcp4_713*C14;
  ROcp4_814 = ROcp4_210*S14+ROcp4_813*C14;
  ROcp4_914 = ROcp4_310*S14+ROcp4_913*C14;
  ROcp4_115 = ROcp4_114*C15+ROcp4_413*S15;
  ROcp4_215 = ROcp4_214*C15+ROcp4_513*S15;
  ROcp4_315 = ROcp4_314*C15+ROcp4_613*S15;
  ROcp4_415 = -(ROcp4_114*S15-ROcp4_413*C15);
  ROcp4_515 = -(ROcp4_214*S15-ROcp4_513*C15);
  ROcp4_615 = -(ROcp4_314*S15-ROcp4_613*C15);
  ROcp4_416 = ROcp4_415*C16+ROcp4_714*S16;
  ROcp4_516 = ROcp4_515*C16+ROcp4_814*S16;
  ROcp4_616 = ROcp4_615*C16+ROcp4_914*S16;
  ROcp4_716 = -(ROcp4_415*S16-ROcp4_714*C16);
  ROcp4_816 = -(ROcp4_515*S16-ROcp4_814*C16);
  ROcp4_916 = -(ROcp4_615*S16-ROcp4_914*C16);
  ROcp4_117 = ROcp4_115*C17-ROcp4_716*S17;
  ROcp4_217 = ROcp4_215*C17-ROcp4_816*S17;
  ROcp4_317 = ROcp4_315*C17-ROcp4_916*S17;
  ROcp4_717 = ROcp4_115*S17+ROcp4_716*C17;
  ROcp4_817 = ROcp4_215*S17+ROcp4_816*C17;
  ROcp4_917 = ROcp4_315*S17+ROcp4_916*C17;
  RLcp4_112 = ROcp4_46*s->dpt[2][11]+ROcp4_710*s->dpt[3][11];
  RLcp4_212 = ROcp4_56*s->dpt[2][11]+ROcp4_810*s->dpt[3][11];
  RLcp4_312 = ROcp4_910*s->dpt[3][11]+s->dpt[2][11]*S6;
  OMcp4_112 = OMcp4_17+qd[12]*ROcp4_110;
  OMcp4_212 = OMcp4_27+qd[12]*ROcp4_210;
  OMcp4_312 = OMcp4_37+qd[12]*ROcp4_310;
  ORcp4_112 = OMcp4_27*RLcp4_312-OMcp4_37*RLcp4_212;
  ORcp4_212 = -(OMcp4_17*RLcp4_312-OMcp4_37*RLcp4_112);
  ORcp4_312 = OMcp4_17*RLcp4_212-OMcp4_27*RLcp4_112;
  OPcp4_112 = OPcp4_17+qd[12]*(OMcp4_27*ROcp4_310-OMcp4_37*ROcp4_210)+qdd[12]*ROcp4_110;
  OPcp4_212 = OPcp4_27-qd[12]*(OMcp4_17*ROcp4_310-OMcp4_37*ROcp4_110)+qdd[12]*ROcp4_210;
  OPcp4_312 = OPcp4_37+qd[12]*(OMcp4_17*ROcp4_210-OMcp4_27*ROcp4_110)+qdd[12]*ROcp4_310;
  RLcp4_113 = ROcp4_412*s->dpt[2][16];
  RLcp4_213 = ROcp4_512*s->dpt[2][16];
  RLcp4_313 = ROcp4_612*s->dpt[2][16];
  OMcp4_113 = OMcp4_112+qd[13]*ROcp4_110;
  OMcp4_213 = OMcp4_212+qd[13]*ROcp4_210;
  OMcp4_313 = OMcp4_312+qd[13]*ROcp4_310;
  ORcp4_113 = OMcp4_212*RLcp4_313-OMcp4_312*RLcp4_213;
  ORcp4_213 = -(OMcp4_112*RLcp4_313-OMcp4_312*RLcp4_113);
  ORcp4_313 = OMcp4_112*RLcp4_213-OMcp4_212*RLcp4_113;
  OMcp4_114 = OMcp4_113+qd[14]*ROcp4_413;
  OMcp4_214 = OMcp4_213+qd[14]*ROcp4_513;
  OMcp4_314 = OMcp4_313+qd[14]*ROcp4_613;
  OMcp4_115 = OMcp4_114+qd[15]*ROcp4_714;
  OMcp4_215 = OMcp4_214+qd[15]*ROcp4_814;
  OMcp4_315 = OMcp4_314+qd[15]*ROcp4_914;
  OPcp4_115 = OPcp4_112+qd[13]*(OMcp4_212*ROcp4_310-OMcp4_312*ROcp4_210)+qd[14]*(OMcp4_213*ROcp4_613-OMcp4_313*ROcp4_513
 )+qd[15]*(OMcp4_214*ROcp4_914-OMcp4_314*ROcp4_814)+qdd[13]*ROcp4_110+qdd[14]*ROcp4_413+qdd[15]*ROcp4_714;
  OPcp4_215 = OPcp4_212-qd[13]*(OMcp4_112*ROcp4_310-OMcp4_312*ROcp4_110)-qd[14]*(OMcp4_113*ROcp4_613-OMcp4_313*ROcp4_413
 )-qd[15]*(OMcp4_114*ROcp4_914-OMcp4_314*ROcp4_714)+qdd[13]*ROcp4_210+qdd[14]*ROcp4_513+qdd[15]*ROcp4_814;
  OPcp4_315 = OPcp4_312+qd[13]*(OMcp4_112*ROcp4_210-OMcp4_212*ROcp4_110)+qd[14]*(OMcp4_113*ROcp4_513-OMcp4_213*ROcp4_413
 )+qd[15]*(OMcp4_114*ROcp4_814-OMcp4_214*ROcp4_714)+qdd[13]*ROcp4_310+qdd[14]*ROcp4_613+qdd[15]*ROcp4_914;
  RLcp4_116 = ROcp4_115*s->dpt[1][19]+ROcp4_415*s->dpt[2][19]+ROcp4_714*s->dpt[3][19];
  RLcp4_216 = ROcp4_215*s->dpt[1][19]+ROcp4_515*s->dpt[2][19]+ROcp4_814*s->dpt[3][19];
  RLcp4_316 = ROcp4_315*s->dpt[1][19]+ROcp4_615*s->dpt[2][19]+ROcp4_914*s->dpt[3][19];
  ORcp4_116 = OMcp4_215*RLcp4_316-OMcp4_315*RLcp4_216;
  ORcp4_216 = -(OMcp4_115*RLcp4_316-OMcp4_315*RLcp4_116);
  ORcp4_316 = OMcp4_115*RLcp4_216-OMcp4_215*RLcp4_116;
  PxF2[1] = RLcp4_110+RLcp4_112+RLcp4_113+RLcp4_116+RLcp4_12+RLcp4_13;
  PxF2[2] = RLcp4_210+RLcp4_212+RLcp4_213+RLcp4_216+RLcp4_22+RLcp4_23;
  PxF2[3] = q[4]+RLcp4_310+RLcp4_312+RLcp4_313+RLcp4_316;
  RxF2[1][1] = ROcp4_117;
  RxF2[1][2] = ROcp4_217;
  RxF2[1][3] = ROcp4_317;
  RxF2[2][1] = ROcp4_416;
  RxF2[2][2] = ROcp4_516;
  RxF2[2][3] = ROcp4_616;
  RxF2[3][1] = ROcp4_717;
  RxF2[3][2] = ROcp4_817;
  RxF2[3][3] = ROcp4_917;
  VxF2[1] = ORcp4_110+ORcp4_112+ORcp4_113+ORcp4_116+ORcp4_12+ORcp4_13+qd[2]*C1-qd[3]*S1;
  VxF2[2] = ORcp4_210+ORcp4_212+ORcp4_213+ORcp4_216+ORcp4_22+ORcp4_23+qd[2]*S1+qd[3]*C1;
  VxF2[3] = qd[4]+ORcp4_310+ORcp4_312+ORcp4_313+ORcp4_316;
  OMxF2[1] = OMcp4_115+qd[17]*ROcp4_416;
  OMxF2[2] = OMcp4_215+qd[17]*ROcp4_516;
  OMxF2[3] = OMcp4_315+qd[17]*ROcp4_616;
  AxF2[1] = -(qd[1]*(ORcp4_22+ORcp4_23+(2.0)*qd[2]*S1+(2.0)*qd[3]*C1)-qdd[2]*C1+qdd[3]*S1-OMcp4_212*ORcp4_313-OMcp4_215*ORcp4_316-
 OMcp4_27*ORcp4_310-OMcp4_27*ORcp4_312+OMcp4_312*ORcp4_213+OMcp4_315*ORcp4_216+OMcp4_37*ORcp4_210+OMcp4_37*ORcp4_212-
 OPcp4_212*RLcp4_313-OPcp4_215*RLcp4_316-OPcp4_27*RLcp4_310-OPcp4_27*RLcp4_312+OPcp4_312*RLcp4_213+OPcp4_315*RLcp4_216+
 OPcp4_37*RLcp4_210+OPcp4_37*RLcp4_212);
  AxF2[2] = qd[1]*(ORcp4_12+ORcp4_13+(2.0)*qd[2]*C1-(2.0)*qd[3]*S1)+qdd[2]*S1+qdd[3]*C1-OMcp4_112*ORcp4_313-OMcp4_115*ORcp4_316-
 OMcp4_17*ORcp4_310-OMcp4_17*ORcp4_312+OMcp4_312*ORcp4_113+OMcp4_315*ORcp4_116+OMcp4_37*ORcp4_110+OMcp4_37*ORcp4_112-
 OPcp4_112*RLcp4_313-OPcp4_115*RLcp4_316-OPcp4_17*RLcp4_310-OPcp4_17*RLcp4_312+OPcp4_312*RLcp4_113+OPcp4_315*RLcp4_116+
 OPcp4_37*RLcp4_110+OPcp4_37*RLcp4_112;
  AxF2[3] = qdd[4]+OMcp4_112*ORcp4_213+OMcp4_115*ORcp4_216+OMcp4_17*ORcp4_210+OMcp4_17*ORcp4_212-OMcp4_212*ORcp4_113-
 OMcp4_215*ORcp4_116-OMcp4_27*ORcp4_110-OMcp4_27*ORcp4_112+OPcp4_112*RLcp4_213+OPcp4_115*RLcp4_216+OPcp4_17*RLcp4_210+
 OPcp4_17*RLcp4_212-OPcp4_212*RLcp4_113-OPcp4_215*RLcp4_116-OPcp4_27*RLcp4_110-OPcp4_27*RLcp4_112;
  OMPxF2[1] = OPcp4_115+qd[17]*(OMcp4_215*ROcp4_616-OMcp4_315*ROcp4_516)+qdd[17]*ROcp4_416;
  OMPxF2[2] = OPcp4_215-qd[17]*(OMcp4_115*ROcp4_616-OMcp4_315*ROcp4_416)+qdd[17]*ROcp4_516;
  OMPxF2[3] = OPcp4_315+qd[17]*(OMcp4_115*ROcp4_516-OMcp4_215*ROcp4_416)+qdd[17]*ROcp4_616;
 
// Sensor Forces Computation 

  SWr2 = user_ExtForces(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc15 = ROcp4_117*SWr2[1]+ROcp4_217*SWr2[2]+ROcp4_317*SWr2[3];
  xfrc25 = ROcp4_416*SWr2[1]+ROcp4_516*SWr2[2]+ROcp4_616*SWr2[3];
  xfrc35 = ROcp4_717*SWr2[1]+ROcp4_817*SWr2[2]+ROcp4_917*SWr2[3];
  frc[1][17] = s->frc[1][17]+xfrc15;
  frc[2][17] = s->frc[2][17]+xfrc25;
  frc[3][17] = s->frc[3][17]+xfrc35;
  xtrq15 = ROcp4_117*SWr2[4]+ROcp4_217*SWr2[5]+ROcp4_317*SWr2[6];
  xtrq25 = ROcp4_416*SWr2[4]+ROcp4_516*SWr2[5]+ROcp4_616*SWr2[6];
  xtrq35 = ROcp4_717*SWr2[4]+ROcp4_817*SWr2[5]+ROcp4_917*SWr2[6];
  trq[1][17] = s->trq[1][17]+xtrq15-xfrc25*SWr2[9]+xfrc35*SWr2[8];
  trq[2][17] = s->trq[2][17]+xtrq25+xfrc15*SWr2[9]-xfrc35*SWr2[7];
  trq[3][17] = s->trq[3][17]+xtrq35-xfrc15*SWr2[8]+xfrc25*SWr2[7];

// = = Block_0_0_1_3_0_1 = = 
 
// Sensor Kinematics 


  ROcp5_46 = -S1p5*C6;
  ROcp5_56 = C1p5*C6;
  ROcp5_76 = S1p5*S6;
  ROcp5_86 = -C1p5*S6;
  ROcp5_17 = -(ROcp5_76*S7-C1p5*C7);
  ROcp5_27 = -(ROcp5_86*S7-S1p5*C7);
  ROcp5_77 = ROcp5_76*C7+C1p5*S7;
  ROcp5_87 = ROcp5_86*C7+S1p5*S7;
  RLcp5_12 = q[2]*C1;
  RLcp5_22 = q[2]*S1;
  ORcp5_12 = -qd[1]*RLcp5_22;
  ORcp5_22 = qd[1]*RLcp5_12;
  RLcp5_13 = -q[3]*S1;
  RLcp5_23 = q[3]*C1;
  ORcp5_13 = -qd[1]*RLcp5_23;
  ORcp5_23 = qd[1]*RLcp5_13;
  OMcp5_35 = qd[1]+qd[5];
  OMcp5_16 = qd[6]*C1p5;
  OMcp5_26 = qd[6]*S1p5;
  OMcp5_17 = OMcp5_16+qd[7]*ROcp5_46;
  OMcp5_27 = OMcp5_26+qd[7]*ROcp5_56;
  OMcp5_37 = OMcp5_35+qd[7]*S6;
  OPcp5_17 = -(qd[6]*OMcp5_35*S1p5-qd[7]*(OMcp5_26*S6-OMcp5_35*ROcp5_56)-qdd[6]*C1p5-qdd[7]*ROcp5_46);
  OPcp5_27 = qd[6]*OMcp5_35*C1p5-qd[7]*(OMcp5_16*S6-OMcp5_35*ROcp5_46)+qdd[6]*S1p5+qdd[7]*ROcp5_56;
  OPcp5_37 = qdd[5]+qd[6]*qd[7]*C6+qdd[7]*S6;

// = = Block_0_0_1_3_0_3 = = 
 
// Sensor Kinematics 


  ROcp5_110 = ROcp5_17*C10-ROcp5_77*S10;
  ROcp5_210 = ROcp5_27*C10-ROcp5_87*S10;
  ROcp5_310 = -S10p7*C6;
  ROcp5_710 = ROcp5_17*S10+ROcp5_77*C10;
  ROcp5_810 = ROcp5_27*S10+ROcp5_87*C10;
  ROcp5_910 = C10p7*C6;
  RLcp5_110 = ROcp5_17*s->dpt[1][5]+ROcp5_77*s->dpt[3][5];
  RLcp5_210 = ROcp5_27*s->dpt[1][5]+ROcp5_87*s->dpt[3][5];
  RLcp5_310 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
  ORcp5_110 = OMcp5_27*RLcp5_310-OMcp5_37*RLcp5_210;
  ORcp5_210 = -(OMcp5_17*RLcp5_310-OMcp5_37*RLcp5_110);
  ORcp5_310 = OMcp5_17*RLcp5_210-OMcp5_27*RLcp5_110;

// = = Block_0_0_1_3_0_6 = = 
 
// Sensor Kinematics 


  ROcp5_418 = ROcp5_46*C18+ROcp5_710*S18;
  ROcp5_518 = ROcp5_56*C18+ROcp5_810*S18;
  ROcp5_618 = ROcp5_910*S18+C18*S6;
  ROcp5_718 = -(ROcp5_46*S18-ROcp5_710*C18);
  ROcp5_818 = -(ROcp5_56*S18-ROcp5_810*C18);
  ROcp5_918 = ROcp5_910*C18-S18*S6;
  ROcp5_419 = ROcp5_418*C19+ROcp5_718*S19;
  ROcp5_519 = ROcp5_518*C19+ROcp5_818*S19;
  ROcp5_619 = ROcp5_618*C19+ROcp5_918*S19;
  ROcp5_719 = -(ROcp5_418*S19-ROcp5_718*C19);
  ROcp5_819 = -(ROcp5_518*S19-ROcp5_818*C19);
  ROcp5_919 = -(ROcp5_618*S19-ROcp5_918*C19);
  ROcp5_120 = ROcp5_110*C20-ROcp5_719*S20;
  ROcp5_220 = ROcp5_210*C20-ROcp5_819*S20;
  ROcp5_320 = ROcp5_310*C20-ROcp5_919*S20;
  ROcp5_720 = ROcp5_110*S20+ROcp5_719*C20;
  ROcp5_820 = ROcp5_210*S20+ROcp5_819*C20;
  ROcp5_920 = ROcp5_310*S20+ROcp5_919*C20;
  ROcp5_121 = ROcp5_120*C21+ROcp5_419*S21;
  ROcp5_221 = ROcp5_220*C21+ROcp5_519*S21;
  ROcp5_321 = ROcp5_320*C21+ROcp5_619*S21;
  ROcp5_421 = -(ROcp5_120*S21-ROcp5_419*C21);
  ROcp5_521 = -(ROcp5_220*S21-ROcp5_519*C21);
  ROcp5_621 = -(ROcp5_320*S21-ROcp5_619*C21);
  ROcp5_422 = ROcp5_421*C22+ROcp5_720*S22;
  ROcp5_522 = ROcp5_521*C22+ROcp5_820*S22;
  ROcp5_622 = ROcp5_621*C22+ROcp5_920*S22;
  ROcp5_722 = -(ROcp5_421*S22-ROcp5_720*C22);
  ROcp5_822 = -(ROcp5_521*S22-ROcp5_820*C22);
  ROcp5_922 = -(ROcp5_621*S22-ROcp5_920*C22);
  ROcp5_123 = ROcp5_121*C23-ROcp5_722*S23;
  ROcp5_223 = ROcp5_221*C23-ROcp5_822*S23;
  ROcp5_323 = ROcp5_321*C23-ROcp5_922*S23;
  ROcp5_723 = ROcp5_121*S23+ROcp5_722*C23;
  ROcp5_823 = ROcp5_221*S23+ROcp5_822*C23;
  ROcp5_923 = ROcp5_321*S23+ROcp5_922*C23;
  RLcp5_118 = ROcp5_46*s->dpt[2][12]+ROcp5_710*s->dpt[3][12];
  RLcp5_218 = ROcp5_56*s->dpt[2][12]+ROcp5_810*s->dpt[3][12];
  RLcp5_318 = ROcp5_910*s->dpt[3][12]+s->dpt[2][12]*S6;
  OMcp5_118 = OMcp5_17+qd[18]*ROcp5_110;
  OMcp5_218 = OMcp5_27+qd[18]*ROcp5_210;
  OMcp5_318 = OMcp5_37+qd[18]*ROcp5_310;
  ORcp5_118 = OMcp5_27*RLcp5_318-OMcp5_37*RLcp5_218;
  ORcp5_218 = -(OMcp5_17*RLcp5_318-OMcp5_37*RLcp5_118);
  ORcp5_318 = OMcp5_17*RLcp5_218-OMcp5_27*RLcp5_118;
  OPcp5_118 = OPcp5_17+qd[18]*(OMcp5_27*ROcp5_310-OMcp5_37*ROcp5_210)+qdd[18]*ROcp5_110;
  OPcp5_218 = OPcp5_27-qd[18]*(OMcp5_17*ROcp5_310-OMcp5_37*ROcp5_110)+qdd[18]*ROcp5_210;
  OPcp5_318 = OPcp5_37+qd[18]*(OMcp5_17*ROcp5_210-OMcp5_27*ROcp5_110)+qdd[18]*ROcp5_310;
  RLcp5_119 = ROcp5_418*s->dpt[2][22];
  RLcp5_219 = ROcp5_518*s->dpt[2][22];
  RLcp5_319 = ROcp5_618*s->dpt[2][22];
  OMcp5_119 = OMcp5_118+qd[19]*ROcp5_110;
  OMcp5_219 = OMcp5_218+qd[19]*ROcp5_210;
  OMcp5_319 = OMcp5_318+qd[19]*ROcp5_310;
  ORcp5_119 = OMcp5_218*RLcp5_319-OMcp5_318*RLcp5_219;
  ORcp5_219 = -(OMcp5_118*RLcp5_319-OMcp5_318*RLcp5_119);
  ORcp5_319 = OMcp5_118*RLcp5_219-OMcp5_218*RLcp5_119;
  OMcp5_120 = OMcp5_119+qd[20]*ROcp5_419;
  OMcp5_220 = OMcp5_219+qd[20]*ROcp5_519;
  OMcp5_320 = OMcp5_319+qd[20]*ROcp5_619;
  OMcp5_121 = OMcp5_120+qd[21]*ROcp5_720;
  OMcp5_221 = OMcp5_220+qd[21]*ROcp5_820;
  OMcp5_321 = OMcp5_320+qd[21]*ROcp5_920;
  OPcp5_121 = OPcp5_118+qd[19]*(OMcp5_218*ROcp5_310-OMcp5_318*ROcp5_210)+qd[20]*(OMcp5_219*ROcp5_619-OMcp5_319*ROcp5_519
 )+qd[21]*(OMcp5_220*ROcp5_920-OMcp5_320*ROcp5_820)+qdd[19]*ROcp5_110+qdd[20]*ROcp5_419+qdd[21]*ROcp5_720;
  OPcp5_221 = OPcp5_218-qd[19]*(OMcp5_118*ROcp5_310-OMcp5_318*ROcp5_110)-qd[20]*(OMcp5_119*ROcp5_619-OMcp5_319*ROcp5_419
 )-qd[21]*(OMcp5_120*ROcp5_920-OMcp5_320*ROcp5_720)+qdd[19]*ROcp5_210+qdd[20]*ROcp5_519+qdd[21]*ROcp5_820;
  OPcp5_321 = OPcp5_318+qd[19]*(OMcp5_118*ROcp5_210-OMcp5_218*ROcp5_110)+qd[20]*(OMcp5_119*ROcp5_519-OMcp5_219*ROcp5_419
 )+qd[21]*(OMcp5_120*ROcp5_820-OMcp5_220*ROcp5_720)+qdd[19]*ROcp5_310+qdd[20]*ROcp5_619+qdd[21]*ROcp5_920;
  RLcp5_122 = ROcp5_121*s->dpt[1][25]+ROcp5_421*s->dpt[2][25]+ROcp5_720*s->dpt[3][25];
  RLcp5_222 = ROcp5_221*s->dpt[1][25]+ROcp5_521*s->dpt[2][25]+ROcp5_820*s->dpt[3][25];
  RLcp5_322 = ROcp5_321*s->dpt[1][25]+ROcp5_621*s->dpt[2][25]+ROcp5_920*s->dpt[3][25];
  ORcp5_122 = OMcp5_221*RLcp5_322-OMcp5_321*RLcp5_222;
  ORcp5_222 = -(OMcp5_121*RLcp5_322-OMcp5_321*RLcp5_122);
  ORcp5_322 = OMcp5_121*RLcp5_222-OMcp5_221*RLcp5_122;
  PxF3[1] = RLcp5_110+RLcp5_118+RLcp5_119+RLcp5_12+RLcp5_122+RLcp5_13;
  PxF3[2] = RLcp5_210+RLcp5_218+RLcp5_219+RLcp5_22+RLcp5_222+RLcp5_23;
  PxF3[3] = q[4]+RLcp5_310+RLcp5_318+RLcp5_319+RLcp5_322;
  RxF3[1][1] = ROcp5_123;
  RxF3[1][2] = ROcp5_223;
  RxF3[1][3] = ROcp5_323;
  RxF3[2][1] = ROcp5_422;
  RxF3[2][2] = ROcp5_522;
  RxF3[2][3] = ROcp5_622;
  RxF3[3][1] = ROcp5_723;
  RxF3[3][2] = ROcp5_823;
  RxF3[3][3] = ROcp5_923;
  VxF3[1] = ORcp5_110+ORcp5_118+ORcp5_119+ORcp5_12+ORcp5_122+ORcp5_13+qd[2]*C1-qd[3]*S1;
  VxF3[2] = ORcp5_210+ORcp5_218+ORcp5_219+ORcp5_22+ORcp5_222+ORcp5_23+qd[2]*S1+qd[3]*C1;
  VxF3[3] = qd[4]+ORcp5_310+ORcp5_318+ORcp5_319+ORcp5_322;
  OMxF3[1] = OMcp5_121+qd[23]*ROcp5_422;
  OMxF3[2] = OMcp5_221+qd[23]*ROcp5_522;
  OMxF3[3] = OMcp5_321+qd[23]*ROcp5_622;
  AxF3[1] = -(qd[1]*(ORcp5_22+ORcp5_23+(2.0)*qd[2]*S1+(2.0)*qd[3]*C1)-qdd[2]*C1+qdd[3]*S1-OMcp5_218*ORcp5_319-OMcp5_221*ORcp5_322-
 OMcp5_27*ORcp5_310-OMcp5_27*ORcp5_318+OMcp5_318*ORcp5_219+OMcp5_321*ORcp5_222+OMcp5_37*ORcp5_210+OMcp5_37*ORcp5_218-
 OPcp5_218*RLcp5_319-OPcp5_221*RLcp5_322-OPcp5_27*RLcp5_310-OPcp5_27*RLcp5_318+OPcp5_318*RLcp5_219+OPcp5_321*RLcp5_222+
 OPcp5_37*RLcp5_210+OPcp5_37*RLcp5_218);
  AxF3[2] = qd[1]*(ORcp5_12+ORcp5_13+(2.0)*qd[2]*C1-(2.0)*qd[3]*S1)+qdd[2]*S1+qdd[3]*C1-OMcp5_118*ORcp5_319-OMcp5_121*ORcp5_322-
 OMcp5_17*ORcp5_310-OMcp5_17*ORcp5_318+OMcp5_318*ORcp5_119+OMcp5_321*ORcp5_122+OMcp5_37*ORcp5_110+OMcp5_37*ORcp5_118-
 OPcp5_118*RLcp5_319-OPcp5_121*RLcp5_322-OPcp5_17*RLcp5_310-OPcp5_17*RLcp5_318+OPcp5_318*RLcp5_119+OPcp5_321*RLcp5_122+
 OPcp5_37*RLcp5_110+OPcp5_37*RLcp5_118;
  AxF3[3] = qdd[4]+OMcp5_118*ORcp5_219+OMcp5_121*ORcp5_222+OMcp5_17*ORcp5_210+OMcp5_17*ORcp5_218-OMcp5_218*ORcp5_119-
 OMcp5_221*ORcp5_122-OMcp5_27*ORcp5_110-OMcp5_27*ORcp5_118+OPcp5_118*RLcp5_219+OPcp5_121*RLcp5_222+OPcp5_17*RLcp5_210+
 OPcp5_17*RLcp5_218-OPcp5_218*RLcp5_119-OPcp5_221*RLcp5_122-OPcp5_27*RLcp5_110-OPcp5_27*RLcp5_118;
  OMPxF3[1] = OPcp5_121+qd[23]*(OMcp5_221*ROcp5_622-OMcp5_321*ROcp5_522)+qdd[23]*ROcp5_422;
  OMPxF3[2] = OPcp5_221-qd[23]*(OMcp5_121*ROcp5_622-OMcp5_321*ROcp5_422)+qdd[23]*ROcp5_522;
  OMPxF3[3] = OPcp5_321+qd[23]*(OMcp5_121*ROcp5_522-OMcp5_221*ROcp5_422)+qdd[23]*ROcp5_622;
 
// Sensor Forces Computation 

  SWr3 = user_ExtForces(PxF3,RxF3,VxF3,OMxF3,AxF3,OMPxF3,s,tsim,3);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc16 = ROcp5_123*SWr3[1]+ROcp5_223*SWr3[2]+ROcp5_323*SWr3[3];
  xfrc26 = ROcp5_422*SWr3[1]+ROcp5_522*SWr3[2]+ROcp5_622*SWr3[3];
  xfrc36 = ROcp5_723*SWr3[1]+ROcp5_823*SWr3[2]+ROcp5_923*SWr3[3];
  frc[1][23] = s->frc[1][23]+xfrc16;
  frc[2][23] = s->frc[2][23]+xfrc26;
  frc[3][23] = s->frc[3][23]+xfrc36;
  xtrq16 = ROcp5_123*SWr3[4]+ROcp5_223*SWr3[5]+ROcp5_323*SWr3[6];
  xtrq26 = ROcp5_422*SWr3[4]+ROcp5_522*SWr3[5]+ROcp5_622*SWr3[6];
  xtrq36 = ROcp5_723*SWr3[4]+ROcp5_823*SWr3[5]+ROcp5_923*SWr3[6];
  trq[1][23] = s->trq[1][23]+xtrq16-xfrc26*SWr3[9]+xfrc36*SWr3[8];
  trq[2][23] = s->trq[2][23]+xtrq26+xfrc16*SWr3[9]-xfrc36*SWr3[7];
  trq[3][23] = s->trq[3][23]+xtrq36-xfrc16*SWr3[8]+xfrc26*SWr3[7];

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
 


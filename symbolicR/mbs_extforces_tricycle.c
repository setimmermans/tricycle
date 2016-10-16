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
//	==> Generation Date : Fri Oct 14 16:15:44 2016
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

// = = Block_0_0_1_1_0_2 = = 
 
// Trigonometric Variables  

  S7p8 = C7*S8+S7*C8;
  C7p8 = C7*C8-S7*S8;
  S7p8p9 = C7p8*S9+S7p8*C9;
  C7p8p9 = C7p8*C9-S7p8*S9;
 
// Sensor Kinematics 


  ROcp5_18 = ROcp5_17*C8-ROcp5_77*S8;
  ROcp5_28 = ROcp5_27*C8-ROcp5_87*S8;
  ROcp5_78 = ROcp5_17*S8+ROcp5_77*C8;
  ROcp5_88 = ROcp5_27*S8+ROcp5_87*C8;
  ROcp5_19 = ROcp5_18*C9-ROcp5_78*S9;
  ROcp5_29 = ROcp5_28*C9-ROcp5_88*S9;
  ROcp5_39 = -S7p8p9*C6;
  ROcp5_79 = ROcp5_18*S9+ROcp5_78*C9;
  ROcp5_89 = ROcp5_28*S9+ROcp5_88*C9;
  ROcp5_99 = C7p8p9*C6;
  RLcp5_18 = ROcp5_17*s->dpt[1][2]+ROcp5_77*s->dpt[3][2];
  RLcp5_28 = ROcp5_27*s->dpt[1][2]+ROcp5_87*s->dpt[3][2];
  RLcp5_38 = -C6*(s->dpt[1][2]*S7-s->dpt[3][2]*C7);
  OMcp5_18 = OMcp5_17+qd[8]*ROcp5_46;
  OMcp5_28 = OMcp5_27+qd[8]*ROcp5_56;
  OMcp5_38 = OMcp5_37+qd[8]*S6;
  ORcp5_18 = OMcp5_27*RLcp5_38-OMcp5_37*RLcp5_28;
  ORcp5_28 = -(OMcp5_17*RLcp5_38-OMcp5_37*RLcp5_18);
  ORcp5_38 = OMcp5_17*RLcp5_28-OMcp5_27*RLcp5_18;
  OPcp5_18 = OPcp5_17+qd[8]*(OMcp5_27*S6-OMcp5_37*ROcp5_56)+qdd[8]*ROcp5_46;
  OPcp5_28 = OPcp5_27-qd[8]*(OMcp5_17*S6-OMcp5_37*ROcp5_46)+qdd[8]*ROcp5_56;
  OPcp5_38 = OPcp5_37+qd[8]*(OMcp5_17*ROcp5_56-OMcp5_27*ROcp5_46)+qdd[8]*S6;
  RLcp5_19 = ROcp5_18*s->dpt[1][8];
  RLcp5_29 = ROcp5_28*s->dpt[1][8];
  RLcp5_39 = -s->dpt[1][8]*S7p8*C6;
  ORcp5_19 = OMcp5_28*RLcp5_39-OMcp5_38*RLcp5_29;
  ORcp5_29 = -(OMcp5_18*RLcp5_39-OMcp5_38*RLcp5_19);
  ORcp5_39 = OMcp5_18*RLcp5_29-OMcp5_28*RLcp5_19;
  PxF1[1] = RLcp5_12+RLcp5_13+RLcp5_18+RLcp5_19;
  PxF1[2] = RLcp5_22+RLcp5_23+RLcp5_28+RLcp5_29;
  PxF1[3] = q[4]+RLcp5_38+RLcp5_39;
  RxF1[1][1] = ROcp5_19;
  RxF1[1][2] = ROcp5_29;
  RxF1[1][3] = ROcp5_39;
  RxF1[2][1] = ROcp5_46;
  RxF1[2][2] = ROcp5_56;
  RxF1[2][3] = S6;
  RxF1[3][1] = ROcp5_79;
  RxF1[3][2] = ROcp5_89;
  RxF1[3][3] = ROcp5_99;
  VxF1[1] = ORcp5_12+ORcp5_13+ORcp5_18+ORcp5_19+qd[2]*C1-qd[3]*S1;
  VxF1[2] = ORcp5_22+ORcp5_23+ORcp5_28+ORcp5_29+qd[2]*S1+qd[3]*C1;
  VxF1[3] = qd[4]+ORcp5_38+ORcp5_39;
  OMxF1[1] = OMcp5_18+qd[9]*ROcp5_46;
  OMxF1[2] = OMcp5_28+qd[9]*ROcp5_56;
  OMxF1[3] = OMcp5_38+qd[9]*S6;
  AxF1[1] = -(qd[1]*(ORcp5_22+ORcp5_23+(2.0)*qd[2]*S1+(2.0)*qd[3]*C1)-qdd[2]*C1+qdd[3]*S1-OMcp5_27*ORcp5_38-OMcp5_28*ORcp5_39+
 OMcp5_37*ORcp5_28+OMcp5_38*ORcp5_29-OPcp5_27*RLcp5_38-OPcp5_28*RLcp5_39+OPcp5_37*RLcp5_28+OPcp5_38*RLcp5_29);
  AxF1[2] = qd[1]*(ORcp5_12+ORcp5_13+(2.0)*qd[2]*C1-(2.0)*qd[3]*S1)+qdd[2]*S1+qdd[3]*C1-OMcp5_17*ORcp5_38-OMcp5_18*ORcp5_39+
 OMcp5_37*ORcp5_18+OMcp5_38*ORcp5_19-OPcp5_17*RLcp5_38-OPcp5_18*RLcp5_39+OPcp5_37*RLcp5_18+OPcp5_38*RLcp5_19;
  AxF1[3] = qdd[4]+OMcp5_17*ORcp5_28+OMcp5_18*ORcp5_29-OMcp5_27*ORcp5_18-OMcp5_28*ORcp5_19+OPcp5_17*RLcp5_28+OPcp5_18*
 RLcp5_29-OPcp5_27*RLcp5_18-OPcp5_28*RLcp5_19;
  OMPxF1[1] = OPcp5_18+qd[9]*(OMcp5_28*S6-OMcp5_38*ROcp5_56)+qdd[9]*ROcp5_46;
  OMPxF1[2] = OPcp5_28-qd[9]*(OMcp5_18*S6-OMcp5_38*ROcp5_46)+qdd[9]*ROcp5_56;
  OMPxF1[3] = OPcp5_38+qd[9]*(OMcp5_18*ROcp5_56-OMcp5_28*ROcp5_46)+qdd[9]*S6;
 
// Sensor Forces Computation 

  SWr1 = user_ExtForces(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc16 = ROcp5_19*SWr1[1]+ROcp5_29*SWr1[2]+ROcp5_39*SWr1[3];
  xfrc26 = ROcp5_46*SWr1[1]+ROcp5_56*SWr1[2]+SWr1[3]*S6;
  xfrc36 = ROcp5_79*SWr1[1]+ROcp5_89*SWr1[2]+ROcp5_99*SWr1[3];
  frc[1][9] = s->frc[1][9]+xfrc16;
  frc[2][9] = s->frc[2][9]+xfrc26;
  frc[3][9] = s->frc[3][9]+xfrc36;
  xtrq16 = ROcp5_19*SWr1[4]+ROcp5_29*SWr1[5]+ROcp5_39*SWr1[6];
  xtrq26 = ROcp5_46*SWr1[4]+ROcp5_56*SWr1[5]+SWr1[6]*S6;
  xtrq36 = ROcp5_79*SWr1[4]+ROcp5_89*SWr1[5]+ROcp5_99*SWr1[6];
  trq[1][9] = s->trq[1][9]+xtrq16-xfrc26*SWr1[9]+xfrc36*SWr1[8];
  trq[2][9] = s->trq[2][9]+xtrq26+xfrc16*SWr1[9]-xfrc36*SWr1[7];
  trq[3][9] = s->trq[3][9]+xtrq36-xfrc16*SWr1[8]+xfrc26*SWr1[7];

// = = Block_0_0_1_2_0_1 = = 
 
// Sensor Kinematics 


  ROcp6_46 = -S1p5*C6;
  ROcp6_56 = C1p5*C6;
  ROcp6_76 = S1p5*S6;
  ROcp6_86 = -C1p5*S6;
  ROcp6_17 = -(ROcp6_76*S7-C1p5*C7);
  ROcp6_27 = -(ROcp6_86*S7-S1p5*C7);
  ROcp6_77 = ROcp6_76*C7+C1p5*S7;
  ROcp6_87 = ROcp6_86*C7+S1p5*S7;
  RLcp6_12 = q[2]*C1;
  RLcp6_22 = q[2]*S1;
  ORcp6_12 = -qd[1]*RLcp6_22;
  ORcp6_22 = qd[1]*RLcp6_12;
  RLcp6_13 = -q[3]*S1;
  RLcp6_23 = q[3]*C1;
  ORcp6_13 = -qd[1]*RLcp6_23;
  ORcp6_23 = qd[1]*RLcp6_13;
  OMcp6_35 = qd[1]+qd[5];
  OMcp6_16 = qd[6]*C1p5;
  OMcp6_26 = qd[6]*S1p5;
  OMcp6_17 = OMcp6_16+qd[7]*ROcp6_46;
  OMcp6_27 = OMcp6_26+qd[7]*ROcp6_56;
  OMcp6_37 = OMcp6_35+qd[7]*S6;
  OPcp6_17 = -(qd[6]*OMcp6_35*S1p5-qd[7]*(OMcp6_26*S6-OMcp6_35*ROcp6_56)-qdd[6]*C1p5-qdd[7]*ROcp6_46);
  OPcp6_27 = qd[6]*OMcp6_35*C1p5-qd[7]*(OMcp6_16*S6-OMcp6_35*ROcp6_46)+qdd[6]*S1p5+qdd[7]*ROcp6_56;
  OPcp6_37 = qdd[5]+qd[6]*qd[7]*C6+qdd[7]*S6;

// = = Block_0_0_1_2_0_3 = = 
 
// Trigonometric Variables  

  S10p7 = C10*S7+S10*C7;
  C10p7 = C10*C7-S10*S7;
 
// Sensor Kinematics 


  ROcp6_110 = ROcp6_17*C10-ROcp6_77*S10;
  ROcp6_210 = ROcp6_27*C10-ROcp6_87*S10;
  ROcp6_310 = -S10p7*C6;
  ROcp6_710 = ROcp6_17*S10+ROcp6_77*C10;
  ROcp6_810 = ROcp6_27*S10+ROcp6_87*C10;
  ROcp6_910 = C10p7*C6;
  RLcp6_110 = ROcp6_17*s->dpt[1][5]+ROcp6_77*s->dpt[3][5];
  RLcp6_210 = ROcp6_27*s->dpt[1][5]+ROcp6_87*s->dpt[3][5];
  RLcp6_310 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
  ORcp6_110 = OMcp6_27*RLcp6_310-OMcp6_37*RLcp6_210;
  ORcp6_210 = -(OMcp6_17*RLcp6_310-OMcp6_37*RLcp6_110);
  ORcp6_310 = OMcp6_17*RLcp6_210-OMcp6_27*RLcp6_110;

// = = Block_0_0_1_2_0_5 = = 
 
// Sensor Kinematics 


  ROcp6_412 = ROcp6_46*C12+ROcp6_710*S12;
  ROcp6_512 = ROcp6_56*C12+ROcp6_810*S12;
  ROcp6_612 = ROcp6_910*S12+C12*S6;
  ROcp6_712 = -(ROcp6_46*S12-ROcp6_710*C12);
  ROcp6_812 = -(ROcp6_56*S12-ROcp6_810*C12);
  ROcp6_912 = ROcp6_910*C12-S12*S6;
  ROcp6_413 = ROcp6_412*C13+ROcp6_712*S13;
  ROcp6_513 = ROcp6_512*C13+ROcp6_812*S13;
  ROcp6_613 = ROcp6_612*C13+ROcp6_912*S13;
  ROcp6_713 = -(ROcp6_412*S13-ROcp6_712*C13);
  ROcp6_813 = -(ROcp6_512*S13-ROcp6_812*C13);
  ROcp6_913 = -(ROcp6_612*S13-ROcp6_912*C13);
  ROcp6_114 = ROcp6_110*C14-ROcp6_713*S14;
  ROcp6_214 = ROcp6_210*C14-ROcp6_813*S14;
  ROcp6_314 = ROcp6_310*C14-ROcp6_913*S14;
  ROcp6_714 = ROcp6_110*S14+ROcp6_713*C14;
  ROcp6_814 = ROcp6_210*S14+ROcp6_813*C14;
  ROcp6_914 = ROcp6_310*S14+ROcp6_913*C14;
  ROcp6_115 = ROcp6_114*C15+ROcp6_413*S15;
  ROcp6_215 = ROcp6_214*C15+ROcp6_513*S15;
  ROcp6_315 = ROcp6_314*C15+ROcp6_613*S15;
  ROcp6_415 = -(ROcp6_114*S15-ROcp6_413*C15);
  ROcp6_515 = -(ROcp6_214*S15-ROcp6_513*C15);
  ROcp6_615 = -(ROcp6_314*S15-ROcp6_613*C15);
  ROcp6_416 = ROcp6_415*C16+ROcp6_714*S16;
  ROcp6_516 = ROcp6_515*C16+ROcp6_814*S16;
  ROcp6_616 = ROcp6_615*C16+ROcp6_914*S16;
  ROcp6_716 = -(ROcp6_415*S16-ROcp6_714*C16);
  ROcp6_816 = -(ROcp6_515*S16-ROcp6_814*C16);
  ROcp6_916 = -(ROcp6_615*S16-ROcp6_914*C16);
  ROcp6_117 = ROcp6_115*C17-ROcp6_716*S17;
  ROcp6_217 = ROcp6_215*C17-ROcp6_816*S17;
  ROcp6_317 = ROcp6_315*C17-ROcp6_916*S17;
  ROcp6_717 = ROcp6_115*S17+ROcp6_716*C17;
  ROcp6_817 = ROcp6_215*S17+ROcp6_816*C17;
  ROcp6_917 = ROcp6_315*S17+ROcp6_916*C17;
  RLcp6_112 = ROcp6_46*s->dpt[2][11]+ROcp6_710*s->dpt[3][11];
  RLcp6_212 = ROcp6_56*s->dpt[2][11]+ROcp6_810*s->dpt[3][11];
  RLcp6_312 = ROcp6_910*s->dpt[3][11]+s->dpt[2][11]*S6;
  OMcp6_112 = OMcp6_17+qd[12]*ROcp6_110;
  OMcp6_212 = OMcp6_27+qd[12]*ROcp6_210;
  OMcp6_312 = OMcp6_37+qd[12]*ROcp6_310;
  ORcp6_112 = OMcp6_27*RLcp6_312-OMcp6_37*RLcp6_212;
  ORcp6_212 = -(OMcp6_17*RLcp6_312-OMcp6_37*RLcp6_112);
  ORcp6_312 = OMcp6_17*RLcp6_212-OMcp6_27*RLcp6_112;
  OPcp6_112 = OPcp6_17+qd[12]*(OMcp6_27*ROcp6_310-OMcp6_37*ROcp6_210)+qdd[12]*ROcp6_110;
  OPcp6_212 = OPcp6_27-qd[12]*(OMcp6_17*ROcp6_310-OMcp6_37*ROcp6_110)+qdd[12]*ROcp6_210;
  OPcp6_312 = OPcp6_37+qd[12]*(OMcp6_17*ROcp6_210-OMcp6_27*ROcp6_110)+qdd[12]*ROcp6_310;
  RLcp6_113 = ROcp6_412*s->dpt[2][16];
  RLcp6_213 = ROcp6_512*s->dpt[2][16];
  RLcp6_313 = ROcp6_612*s->dpt[2][16];
  OMcp6_113 = OMcp6_112+qd[13]*ROcp6_110;
  OMcp6_213 = OMcp6_212+qd[13]*ROcp6_210;
  OMcp6_313 = OMcp6_312+qd[13]*ROcp6_310;
  ORcp6_113 = OMcp6_212*RLcp6_313-OMcp6_312*RLcp6_213;
  ORcp6_213 = -(OMcp6_112*RLcp6_313-OMcp6_312*RLcp6_113);
  ORcp6_313 = OMcp6_112*RLcp6_213-OMcp6_212*RLcp6_113;
  OMcp6_114 = OMcp6_113+qd[14]*ROcp6_413;
  OMcp6_214 = OMcp6_213+qd[14]*ROcp6_513;
  OMcp6_314 = OMcp6_313+qd[14]*ROcp6_613;
  OMcp6_115 = OMcp6_114+qd[15]*ROcp6_714;
  OMcp6_215 = OMcp6_214+qd[15]*ROcp6_814;
  OMcp6_315 = OMcp6_314+qd[15]*ROcp6_914;
  OPcp6_115 = OPcp6_112+qd[13]*(OMcp6_212*ROcp6_310-OMcp6_312*ROcp6_210)+qd[14]*(OMcp6_213*ROcp6_613-OMcp6_313*ROcp6_513
 )+qd[15]*(OMcp6_214*ROcp6_914-OMcp6_314*ROcp6_814)+qdd[13]*ROcp6_110+qdd[14]*ROcp6_413+qdd[15]*ROcp6_714;
  OPcp6_215 = OPcp6_212-qd[13]*(OMcp6_112*ROcp6_310-OMcp6_312*ROcp6_110)-qd[14]*(OMcp6_113*ROcp6_613-OMcp6_313*ROcp6_413
 )-qd[15]*(OMcp6_114*ROcp6_914-OMcp6_314*ROcp6_714)+qdd[13]*ROcp6_210+qdd[14]*ROcp6_513+qdd[15]*ROcp6_814;
  OPcp6_315 = OPcp6_312+qd[13]*(OMcp6_112*ROcp6_210-OMcp6_212*ROcp6_110)+qd[14]*(OMcp6_113*ROcp6_513-OMcp6_213*ROcp6_413
 )+qd[15]*(OMcp6_114*ROcp6_814-OMcp6_214*ROcp6_714)+qdd[13]*ROcp6_310+qdd[14]*ROcp6_613+qdd[15]*ROcp6_914;
  RLcp6_116 = ROcp6_115*s->dpt[1][19]+ROcp6_415*s->dpt[2][19]+ROcp6_714*s->dpt[3][19];
  RLcp6_216 = ROcp6_215*s->dpt[1][19]+ROcp6_515*s->dpt[2][19]+ROcp6_814*s->dpt[3][19];
  RLcp6_316 = ROcp6_315*s->dpt[1][19]+ROcp6_615*s->dpt[2][19]+ROcp6_914*s->dpt[3][19];
  ORcp6_116 = OMcp6_215*RLcp6_316-OMcp6_315*RLcp6_216;
  ORcp6_216 = -(OMcp6_115*RLcp6_316-OMcp6_315*RLcp6_116);
  ORcp6_316 = OMcp6_115*RLcp6_216-OMcp6_215*RLcp6_116;
  PxF2[1] = RLcp6_110+RLcp6_112+RLcp6_113+RLcp6_116+RLcp6_12+RLcp6_13;
  PxF2[2] = RLcp6_210+RLcp6_212+RLcp6_213+RLcp6_216+RLcp6_22+RLcp6_23;
  PxF2[3] = q[4]+RLcp6_310+RLcp6_312+RLcp6_313+RLcp6_316;
  RxF2[1][1] = ROcp6_117;
  RxF2[1][2] = ROcp6_217;
  RxF2[1][3] = ROcp6_317;
  RxF2[2][1] = ROcp6_416;
  RxF2[2][2] = ROcp6_516;
  RxF2[2][3] = ROcp6_616;
  RxF2[3][1] = ROcp6_717;
  RxF2[3][2] = ROcp6_817;
  RxF2[3][3] = ROcp6_917;
  VxF2[1] = ORcp6_110+ORcp6_112+ORcp6_113+ORcp6_116+ORcp6_12+ORcp6_13+qd[2]*C1-qd[3]*S1;
  VxF2[2] = ORcp6_210+ORcp6_212+ORcp6_213+ORcp6_216+ORcp6_22+ORcp6_23+qd[2]*S1+qd[3]*C1;
  VxF2[3] = qd[4]+ORcp6_310+ORcp6_312+ORcp6_313+ORcp6_316;
  OMxF2[1] = OMcp6_115+qd[17]*ROcp6_416;
  OMxF2[2] = OMcp6_215+qd[17]*ROcp6_516;
  OMxF2[3] = OMcp6_315+qd[17]*ROcp6_616;
  AxF2[1] = -(qd[1]*(ORcp6_22+ORcp6_23+(2.0)*qd[2]*S1+(2.0)*qd[3]*C1)-qdd[2]*C1+qdd[3]*S1-OMcp6_212*ORcp6_313-OMcp6_215*ORcp6_316-
 OMcp6_27*ORcp6_310-OMcp6_27*ORcp6_312+OMcp6_312*ORcp6_213+OMcp6_315*ORcp6_216+OMcp6_37*ORcp6_210+OMcp6_37*ORcp6_212-
 OPcp6_212*RLcp6_313-OPcp6_215*RLcp6_316-OPcp6_27*RLcp6_310-OPcp6_27*RLcp6_312+OPcp6_312*RLcp6_213+OPcp6_315*RLcp6_216+
 OPcp6_37*RLcp6_210+OPcp6_37*RLcp6_212);
  AxF2[2] = qd[1]*(ORcp6_12+ORcp6_13+(2.0)*qd[2]*C1-(2.0)*qd[3]*S1)+qdd[2]*S1+qdd[3]*C1-OMcp6_112*ORcp6_313-OMcp6_115*ORcp6_316-
 OMcp6_17*ORcp6_310-OMcp6_17*ORcp6_312+OMcp6_312*ORcp6_113+OMcp6_315*ORcp6_116+OMcp6_37*ORcp6_110+OMcp6_37*ORcp6_112-
 OPcp6_112*RLcp6_313-OPcp6_115*RLcp6_316-OPcp6_17*RLcp6_310-OPcp6_17*RLcp6_312+OPcp6_312*RLcp6_113+OPcp6_315*RLcp6_116+
 OPcp6_37*RLcp6_110+OPcp6_37*RLcp6_112;
  AxF2[3] = qdd[4]+OMcp6_112*ORcp6_213+OMcp6_115*ORcp6_216+OMcp6_17*ORcp6_210+OMcp6_17*ORcp6_212-OMcp6_212*ORcp6_113-
 OMcp6_215*ORcp6_116-OMcp6_27*ORcp6_110-OMcp6_27*ORcp6_112+OPcp6_112*RLcp6_213+OPcp6_115*RLcp6_216+OPcp6_17*RLcp6_210+
 OPcp6_17*RLcp6_212-OPcp6_212*RLcp6_113-OPcp6_215*RLcp6_116-OPcp6_27*RLcp6_110-OPcp6_27*RLcp6_112;
  OMPxF2[1] = OPcp6_115+qd[17]*(OMcp6_215*ROcp6_616-OMcp6_315*ROcp6_516)+qdd[17]*ROcp6_416;
  OMPxF2[2] = OPcp6_215-qd[17]*(OMcp6_115*ROcp6_616-OMcp6_315*ROcp6_416)+qdd[17]*ROcp6_516;
  OMPxF2[3] = OPcp6_315+qd[17]*(OMcp6_115*ROcp6_516-OMcp6_215*ROcp6_416)+qdd[17]*ROcp6_616;
 
// Sensor Forces Computation 

  SWr2 = user_ExtForces(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc17 = ROcp6_117*SWr2[1]+ROcp6_217*SWr2[2]+ROcp6_317*SWr2[3];
  xfrc27 = ROcp6_416*SWr2[1]+ROcp6_516*SWr2[2]+ROcp6_616*SWr2[3];
  xfrc37 = ROcp6_717*SWr2[1]+ROcp6_817*SWr2[2]+ROcp6_917*SWr2[3];
  frc[1][17] = s->frc[1][17]+xfrc17;
  frc[2][17] = s->frc[2][17]+xfrc27;
  frc[3][17] = s->frc[3][17]+xfrc37;
  xtrq17 = ROcp6_117*SWr2[4]+ROcp6_217*SWr2[5]+ROcp6_317*SWr2[6];
  xtrq27 = ROcp6_416*SWr2[4]+ROcp6_516*SWr2[5]+ROcp6_616*SWr2[6];
  xtrq37 = ROcp6_717*SWr2[4]+ROcp6_817*SWr2[5]+ROcp6_917*SWr2[6];
  trq[1][17] = s->trq[1][17]+xtrq17-xfrc27*SWr2[9]+xfrc37*SWr2[8];
  trq[2][17] = s->trq[2][17]+xtrq27+xfrc17*SWr2[9]-xfrc37*SWr2[7];
  trq[3][17] = s->trq[3][17]+xtrq37-xfrc17*SWr2[8]+xfrc27*SWr2[7];

// = = Block_0_0_1_3_0_1 = = 
 
// Sensor Kinematics 


  ROcp7_46 = -S1p5*C6;
  ROcp7_56 = C1p5*C6;
  ROcp7_76 = S1p5*S6;
  ROcp7_86 = -C1p5*S6;
  ROcp7_17 = -(ROcp7_76*S7-C1p5*C7);
  ROcp7_27 = -(ROcp7_86*S7-S1p5*C7);
  ROcp7_77 = ROcp7_76*C7+C1p5*S7;
  ROcp7_87 = ROcp7_86*C7+S1p5*S7;
  RLcp7_12 = q[2]*C1;
  RLcp7_22 = q[2]*S1;
  ORcp7_12 = -qd[1]*RLcp7_22;
  ORcp7_22 = qd[1]*RLcp7_12;
  RLcp7_13 = -q[3]*S1;
  RLcp7_23 = q[3]*C1;
  ORcp7_13 = -qd[1]*RLcp7_23;
  ORcp7_23 = qd[1]*RLcp7_13;
  OMcp7_35 = qd[1]+qd[5];
  OMcp7_16 = qd[6]*C1p5;
  OMcp7_26 = qd[6]*S1p5;
  OMcp7_17 = OMcp7_16+qd[7]*ROcp7_46;
  OMcp7_27 = OMcp7_26+qd[7]*ROcp7_56;
  OMcp7_37 = OMcp7_35+qd[7]*S6;
  OPcp7_17 = -(qd[6]*OMcp7_35*S1p5-qd[7]*(OMcp7_26*S6-OMcp7_35*ROcp7_56)-qdd[6]*C1p5-qdd[7]*ROcp7_46);
  OPcp7_27 = qd[6]*OMcp7_35*C1p5-qd[7]*(OMcp7_16*S6-OMcp7_35*ROcp7_46)+qdd[6]*S1p5+qdd[7]*ROcp7_56;
  OPcp7_37 = qdd[5]+qd[6]*qd[7]*C6+qdd[7]*S6;

// = = Block_0_0_1_3_0_3 = = 
 
// Sensor Kinematics 


  ROcp7_110 = ROcp7_17*C10-ROcp7_77*S10;
  ROcp7_210 = ROcp7_27*C10-ROcp7_87*S10;
  ROcp7_310 = -S10p7*C6;
  ROcp7_710 = ROcp7_17*S10+ROcp7_77*C10;
  ROcp7_810 = ROcp7_27*S10+ROcp7_87*C10;
  ROcp7_910 = C10p7*C6;
  RLcp7_110 = ROcp7_17*s->dpt[1][5]+ROcp7_77*s->dpt[3][5];
  RLcp7_210 = ROcp7_27*s->dpt[1][5]+ROcp7_87*s->dpt[3][5];
  RLcp7_310 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
  ORcp7_110 = OMcp7_27*RLcp7_310-OMcp7_37*RLcp7_210;
  ORcp7_210 = -(OMcp7_17*RLcp7_310-OMcp7_37*RLcp7_110);
  ORcp7_310 = OMcp7_17*RLcp7_210-OMcp7_27*RLcp7_110;

// = = Block_0_0_1_3_0_6 = = 
 
// Sensor Kinematics 


  ROcp7_418 = ROcp7_46*C18+ROcp7_710*S18;
  ROcp7_518 = ROcp7_56*C18+ROcp7_810*S18;
  ROcp7_618 = ROcp7_910*S18+C18*S6;
  ROcp7_718 = -(ROcp7_46*S18-ROcp7_710*C18);
  ROcp7_818 = -(ROcp7_56*S18-ROcp7_810*C18);
  ROcp7_918 = ROcp7_910*C18-S18*S6;
  ROcp7_419 = ROcp7_418*C19+ROcp7_718*S19;
  ROcp7_519 = ROcp7_518*C19+ROcp7_818*S19;
  ROcp7_619 = ROcp7_618*C19+ROcp7_918*S19;
  ROcp7_719 = -(ROcp7_418*S19-ROcp7_718*C19);
  ROcp7_819 = -(ROcp7_518*S19-ROcp7_818*C19);
  ROcp7_919 = -(ROcp7_618*S19-ROcp7_918*C19);
  ROcp7_120 = ROcp7_110*C20-ROcp7_719*S20;
  ROcp7_220 = ROcp7_210*C20-ROcp7_819*S20;
  ROcp7_320 = ROcp7_310*C20-ROcp7_919*S20;
  ROcp7_720 = ROcp7_110*S20+ROcp7_719*C20;
  ROcp7_820 = ROcp7_210*S20+ROcp7_819*C20;
  ROcp7_920 = ROcp7_310*S20+ROcp7_919*C20;
  ROcp7_121 = ROcp7_120*C21+ROcp7_419*S21;
  ROcp7_221 = ROcp7_220*C21+ROcp7_519*S21;
  ROcp7_321 = ROcp7_320*C21+ROcp7_619*S21;
  ROcp7_421 = -(ROcp7_120*S21-ROcp7_419*C21);
  ROcp7_521 = -(ROcp7_220*S21-ROcp7_519*C21);
  ROcp7_621 = -(ROcp7_320*S21-ROcp7_619*C21);
  ROcp7_422 = ROcp7_421*C22+ROcp7_720*S22;
  ROcp7_522 = ROcp7_521*C22+ROcp7_820*S22;
  ROcp7_622 = ROcp7_621*C22+ROcp7_920*S22;
  ROcp7_722 = -(ROcp7_421*S22-ROcp7_720*C22);
  ROcp7_822 = -(ROcp7_521*S22-ROcp7_820*C22);
  ROcp7_922 = -(ROcp7_621*S22-ROcp7_920*C22);
  ROcp7_123 = ROcp7_121*C23-ROcp7_722*S23;
  ROcp7_223 = ROcp7_221*C23-ROcp7_822*S23;
  ROcp7_323 = ROcp7_321*C23-ROcp7_922*S23;
  ROcp7_723 = ROcp7_121*S23+ROcp7_722*C23;
  ROcp7_823 = ROcp7_221*S23+ROcp7_822*C23;
  ROcp7_923 = ROcp7_321*S23+ROcp7_922*C23;
  RLcp7_118 = ROcp7_46*s->dpt[2][12]+ROcp7_710*s->dpt[3][12];
  RLcp7_218 = ROcp7_56*s->dpt[2][12]+ROcp7_810*s->dpt[3][12];
  RLcp7_318 = ROcp7_910*s->dpt[3][12]+s->dpt[2][12]*S6;
  OMcp7_118 = OMcp7_17+qd[18]*ROcp7_110;
  OMcp7_218 = OMcp7_27+qd[18]*ROcp7_210;
  OMcp7_318 = OMcp7_37+qd[18]*ROcp7_310;
  ORcp7_118 = OMcp7_27*RLcp7_318-OMcp7_37*RLcp7_218;
  ORcp7_218 = -(OMcp7_17*RLcp7_318-OMcp7_37*RLcp7_118);
  ORcp7_318 = OMcp7_17*RLcp7_218-OMcp7_27*RLcp7_118;
  OPcp7_118 = OPcp7_17+qd[18]*(OMcp7_27*ROcp7_310-OMcp7_37*ROcp7_210)+qdd[18]*ROcp7_110;
  OPcp7_218 = OPcp7_27-qd[18]*(OMcp7_17*ROcp7_310-OMcp7_37*ROcp7_110)+qdd[18]*ROcp7_210;
  OPcp7_318 = OPcp7_37+qd[18]*(OMcp7_17*ROcp7_210-OMcp7_27*ROcp7_110)+qdd[18]*ROcp7_310;
  RLcp7_119 = ROcp7_418*s->dpt[2][22];
  RLcp7_219 = ROcp7_518*s->dpt[2][22];
  RLcp7_319 = ROcp7_618*s->dpt[2][22];
  OMcp7_119 = OMcp7_118+qd[19]*ROcp7_110;
  OMcp7_219 = OMcp7_218+qd[19]*ROcp7_210;
  OMcp7_319 = OMcp7_318+qd[19]*ROcp7_310;
  ORcp7_119 = OMcp7_218*RLcp7_319-OMcp7_318*RLcp7_219;
  ORcp7_219 = -(OMcp7_118*RLcp7_319-OMcp7_318*RLcp7_119);
  ORcp7_319 = OMcp7_118*RLcp7_219-OMcp7_218*RLcp7_119;
  OMcp7_120 = OMcp7_119+qd[20]*ROcp7_419;
  OMcp7_220 = OMcp7_219+qd[20]*ROcp7_519;
  OMcp7_320 = OMcp7_319+qd[20]*ROcp7_619;
  OMcp7_121 = OMcp7_120+qd[21]*ROcp7_720;
  OMcp7_221 = OMcp7_220+qd[21]*ROcp7_820;
  OMcp7_321 = OMcp7_320+qd[21]*ROcp7_920;
  OPcp7_121 = OPcp7_118+qd[19]*(OMcp7_218*ROcp7_310-OMcp7_318*ROcp7_210)+qd[20]*(OMcp7_219*ROcp7_619-OMcp7_319*ROcp7_519
 )+qd[21]*(OMcp7_220*ROcp7_920-OMcp7_320*ROcp7_820)+qdd[19]*ROcp7_110+qdd[20]*ROcp7_419+qdd[21]*ROcp7_720;
  OPcp7_221 = OPcp7_218-qd[19]*(OMcp7_118*ROcp7_310-OMcp7_318*ROcp7_110)-qd[20]*(OMcp7_119*ROcp7_619-OMcp7_319*ROcp7_419
 )-qd[21]*(OMcp7_120*ROcp7_920-OMcp7_320*ROcp7_720)+qdd[19]*ROcp7_210+qdd[20]*ROcp7_519+qdd[21]*ROcp7_820;
  OPcp7_321 = OPcp7_318+qd[19]*(OMcp7_118*ROcp7_210-OMcp7_218*ROcp7_110)+qd[20]*(OMcp7_119*ROcp7_519-OMcp7_219*ROcp7_419
 )+qd[21]*(OMcp7_120*ROcp7_820-OMcp7_220*ROcp7_720)+qdd[19]*ROcp7_310+qdd[20]*ROcp7_619+qdd[21]*ROcp7_920;
  RLcp7_122 = ROcp7_121*s->dpt[1][25]+ROcp7_421*s->dpt[2][25]+ROcp7_720*s->dpt[3][25];
  RLcp7_222 = ROcp7_221*s->dpt[1][25]+ROcp7_521*s->dpt[2][25]+ROcp7_820*s->dpt[3][25];
  RLcp7_322 = ROcp7_321*s->dpt[1][25]+ROcp7_621*s->dpt[2][25]+ROcp7_920*s->dpt[3][25];
  ORcp7_122 = OMcp7_221*RLcp7_322-OMcp7_321*RLcp7_222;
  ORcp7_222 = -(OMcp7_121*RLcp7_322-OMcp7_321*RLcp7_122);
  ORcp7_322 = OMcp7_121*RLcp7_222-OMcp7_221*RLcp7_122;
  PxF3[1] = RLcp7_110+RLcp7_118+RLcp7_119+RLcp7_12+RLcp7_122+RLcp7_13;
  PxF3[2] = RLcp7_210+RLcp7_218+RLcp7_219+RLcp7_22+RLcp7_222+RLcp7_23;
  PxF3[3] = q[4]+RLcp7_310+RLcp7_318+RLcp7_319+RLcp7_322;
  RxF3[1][1] = ROcp7_123;
  RxF3[1][2] = ROcp7_223;
  RxF3[1][3] = ROcp7_323;
  RxF3[2][1] = ROcp7_422;
  RxF3[2][2] = ROcp7_522;
  RxF3[2][3] = ROcp7_622;
  RxF3[3][1] = ROcp7_723;
  RxF3[3][2] = ROcp7_823;
  RxF3[3][3] = ROcp7_923;
  VxF3[1] = ORcp7_110+ORcp7_118+ORcp7_119+ORcp7_12+ORcp7_122+ORcp7_13+qd[2]*C1-qd[3]*S1;
  VxF3[2] = ORcp7_210+ORcp7_218+ORcp7_219+ORcp7_22+ORcp7_222+ORcp7_23+qd[2]*S1+qd[3]*C1;
  VxF3[3] = qd[4]+ORcp7_310+ORcp7_318+ORcp7_319+ORcp7_322;
  OMxF3[1] = OMcp7_121+qd[23]*ROcp7_422;
  OMxF3[2] = OMcp7_221+qd[23]*ROcp7_522;
  OMxF3[3] = OMcp7_321+qd[23]*ROcp7_622;
  AxF3[1] = -(qd[1]*(ORcp7_22+ORcp7_23+(2.0)*qd[2]*S1+(2.0)*qd[3]*C1)-qdd[2]*C1+qdd[3]*S1-OMcp7_218*ORcp7_319-OMcp7_221*ORcp7_322-
 OMcp7_27*ORcp7_310-OMcp7_27*ORcp7_318+OMcp7_318*ORcp7_219+OMcp7_321*ORcp7_222+OMcp7_37*ORcp7_210+OMcp7_37*ORcp7_218-
 OPcp7_218*RLcp7_319-OPcp7_221*RLcp7_322-OPcp7_27*RLcp7_310-OPcp7_27*RLcp7_318+OPcp7_318*RLcp7_219+OPcp7_321*RLcp7_222+
 OPcp7_37*RLcp7_210+OPcp7_37*RLcp7_218);
  AxF3[2] = qd[1]*(ORcp7_12+ORcp7_13+(2.0)*qd[2]*C1-(2.0)*qd[3]*S1)+qdd[2]*S1+qdd[3]*C1-OMcp7_118*ORcp7_319-OMcp7_121*ORcp7_322-
 OMcp7_17*ORcp7_310-OMcp7_17*ORcp7_318+OMcp7_318*ORcp7_119+OMcp7_321*ORcp7_122+OMcp7_37*ORcp7_110+OMcp7_37*ORcp7_118-
 OPcp7_118*RLcp7_319-OPcp7_121*RLcp7_322-OPcp7_17*RLcp7_310-OPcp7_17*RLcp7_318+OPcp7_318*RLcp7_119+OPcp7_321*RLcp7_122+
 OPcp7_37*RLcp7_110+OPcp7_37*RLcp7_118;
  AxF3[3] = qdd[4]+OMcp7_118*ORcp7_219+OMcp7_121*ORcp7_222+OMcp7_17*ORcp7_210+OMcp7_17*ORcp7_218-OMcp7_218*ORcp7_119-
 OMcp7_221*ORcp7_122-OMcp7_27*ORcp7_110-OMcp7_27*ORcp7_118+OPcp7_118*RLcp7_219+OPcp7_121*RLcp7_222+OPcp7_17*RLcp7_210+
 OPcp7_17*RLcp7_218-OPcp7_218*RLcp7_119-OPcp7_221*RLcp7_122-OPcp7_27*RLcp7_110-OPcp7_27*RLcp7_118;
  OMPxF3[1] = OPcp7_121+qd[23]*(OMcp7_221*ROcp7_622-OMcp7_321*ROcp7_522)+qdd[23]*ROcp7_422;
  OMPxF3[2] = OPcp7_221-qd[23]*(OMcp7_121*ROcp7_622-OMcp7_321*ROcp7_422)+qdd[23]*ROcp7_522;
  OMPxF3[3] = OPcp7_321+qd[23]*(OMcp7_121*ROcp7_522-OMcp7_221*ROcp7_422)+qdd[23]*ROcp7_622;
 
// Sensor Forces Computation 

  SWr3 = user_ExtForces(PxF3,RxF3,VxF3,OMxF3,AxF3,OMPxF3,s,tsim,3);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc18 = ROcp7_123*SWr3[1]+ROcp7_223*SWr3[2]+ROcp7_323*SWr3[3];
  xfrc28 = ROcp7_422*SWr3[1]+ROcp7_522*SWr3[2]+ROcp7_622*SWr3[3];
  xfrc38 = ROcp7_723*SWr3[1]+ROcp7_823*SWr3[2]+ROcp7_923*SWr3[3];
  frc[1][23] = s->frc[1][23]+xfrc18;
  frc[2][23] = s->frc[2][23]+xfrc28;
  frc[3][23] = s->frc[3][23]+xfrc38;
  xtrq18 = ROcp7_123*SWr3[4]+ROcp7_223*SWr3[5]+ROcp7_323*SWr3[6];
  xtrq28 = ROcp7_422*SWr3[4]+ROcp7_522*SWr3[5]+ROcp7_622*SWr3[6];
  xtrq38 = ROcp7_723*SWr3[4]+ROcp7_823*SWr3[5]+ROcp7_923*SWr3[6];
  trq[1][23] = s->trq[1][23]+xtrq18-xfrc28*SWr3[9]+xfrc38*SWr3[8];
  trq[2][23] = s->trq[2][23]+xtrq28+xfrc18*SWr3[9]-xfrc38*SWr3[7];
  trq[3][23] = s->trq[3][23]+xtrq38-xfrc18*SWr3[8]+xfrc28*SWr3[7];

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
 


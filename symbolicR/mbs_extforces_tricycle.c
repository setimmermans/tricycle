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
//	==> Generation Date : Tue Oct  4 18:18:28 2016
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


  ROcp2_46 = -S1p5*C6;
  ROcp2_56 = C1p5*C6;
  ROcp2_76 = S1p5*S6;
  ROcp2_86 = -C1p5*S6;
  ROcp2_17 = -(ROcp2_76*S7-C1p5*C7);
  ROcp2_27 = -(ROcp2_86*S7-S1p5*C7);
  ROcp2_77 = ROcp2_76*C7+C1p5*S7;
  ROcp2_87 = ROcp2_86*C7+S1p5*S7;
  RLcp2_12 = q[2]*C1;
  RLcp2_22 = q[2]*S1;
  ORcp2_12 = -qd[1]*RLcp2_22;
  ORcp2_22 = qd[1]*RLcp2_12;
  RLcp2_13 = -q[3]*S1;
  RLcp2_23 = q[3]*C1;
  ORcp2_13 = -qd[1]*RLcp2_23;
  ORcp2_23 = qd[1]*RLcp2_13;
  OMcp2_35 = qd[1]+qd[5];
  OMcp2_16 = qd[6]*C1p5;
  OMcp2_26 = qd[6]*S1p5;
  OMcp2_17 = OMcp2_16+qd[7]*ROcp2_46;
  OMcp2_27 = OMcp2_26+qd[7]*ROcp2_56;
  OMcp2_37 = OMcp2_35+qd[7]*S6;
  OPcp2_17 = -(qd[6]*OMcp2_35*S1p5-qd[7]*(OMcp2_26*S6-OMcp2_35*ROcp2_56)-qdd[6]*C1p5-qdd[7]*ROcp2_46);
  OPcp2_27 = qd[6]*OMcp2_35*C1p5-qd[7]*(OMcp2_16*S6-OMcp2_35*ROcp2_46)+qdd[6]*S1p5+qdd[7]*ROcp2_56;
  OPcp2_37 = qdd[5]+qd[6]*qd[7]*C6+qdd[7]*S6;

// = = Block_0_0_1_1_0_2 = = 
 
// Trigonometric Variables  

  S7p8 = C7*S8+S7*C8;
  C7p8 = C7*C8-S7*S8;
  S7p8p9 = C7p8*S9+S7p8*C9;
  C7p8p9 = C7p8*C9-S7p8*S9;
 
// Sensor Kinematics 


  ROcp2_18 = ROcp2_17*C8-ROcp2_77*S8;
  ROcp2_28 = ROcp2_27*C8-ROcp2_87*S8;
  ROcp2_78 = ROcp2_17*S8+ROcp2_77*C8;
  ROcp2_88 = ROcp2_27*S8+ROcp2_87*C8;
  ROcp2_19 = ROcp2_18*C9-ROcp2_78*S9;
  ROcp2_29 = ROcp2_28*C9-ROcp2_88*S9;
  ROcp2_39 = -S7p8p9*C6;
  ROcp2_79 = ROcp2_18*S9+ROcp2_78*C9;
  ROcp2_89 = ROcp2_28*S9+ROcp2_88*C9;
  ROcp2_99 = C7p8p9*C6;
  RLcp2_18 = ROcp2_17*s->dpt[1][1]+ROcp2_77*s->dpt[3][1];
  RLcp2_28 = ROcp2_27*s->dpt[1][1]+ROcp2_87*s->dpt[3][1];
  RLcp2_38 = -C6*(s->dpt[1][1]*S7-s->dpt[3][1]*C7);
  OMcp2_18 = OMcp2_17+qd[8]*ROcp2_46;
  OMcp2_28 = OMcp2_27+qd[8]*ROcp2_56;
  OMcp2_38 = OMcp2_37+qd[8]*S6;
  ORcp2_18 = OMcp2_27*RLcp2_38-OMcp2_37*RLcp2_28;
  ORcp2_28 = -(OMcp2_17*RLcp2_38-OMcp2_37*RLcp2_18);
  ORcp2_38 = OMcp2_17*RLcp2_28-OMcp2_27*RLcp2_18;
  OPcp2_18 = OPcp2_17+qd[8]*(OMcp2_27*S6-OMcp2_37*ROcp2_56)+qdd[8]*ROcp2_46;
  OPcp2_28 = OPcp2_27-qd[8]*(OMcp2_17*S6-OMcp2_37*ROcp2_46)+qdd[8]*ROcp2_56;
  OPcp2_38 = OPcp2_37+qd[8]*(OMcp2_17*ROcp2_56-OMcp2_27*ROcp2_46)+qdd[8]*S6;
  RLcp2_19 = ROcp2_18*s->dpt[1][7];
  RLcp2_29 = ROcp2_28*s->dpt[1][7];
  RLcp2_39 = -s->dpt[1][7]*S7p8*C6;
  ORcp2_19 = OMcp2_28*RLcp2_39-OMcp2_38*RLcp2_29;
  ORcp2_29 = -(OMcp2_18*RLcp2_39-OMcp2_38*RLcp2_19);
  ORcp2_39 = OMcp2_18*RLcp2_29-OMcp2_28*RLcp2_19;
  PxF1[1] = RLcp2_12+RLcp2_13+RLcp2_18+RLcp2_19;
  PxF1[2] = RLcp2_22+RLcp2_23+RLcp2_28+RLcp2_29;
  PxF1[3] = q[4]+RLcp2_38+RLcp2_39;
  RxF1[1][1] = ROcp2_19;
  RxF1[1][2] = ROcp2_29;
  RxF1[1][3] = ROcp2_39;
  RxF1[2][1] = ROcp2_46;
  RxF1[2][2] = ROcp2_56;
  RxF1[2][3] = S6;
  RxF1[3][1] = ROcp2_79;
  RxF1[3][2] = ROcp2_89;
  RxF1[3][3] = ROcp2_99;
  VxF1[1] = ORcp2_12+ORcp2_13+ORcp2_18+ORcp2_19+qd[2]*C1-qd[3]*S1;
  VxF1[2] = ORcp2_22+ORcp2_23+ORcp2_28+ORcp2_29+qd[2]*S1+qd[3]*C1;
  VxF1[3] = qd[4]+ORcp2_38+ORcp2_39;
  OMxF1[1] = OMcp2_18+qd[9]*ROcp2_46;
  OMxF1[2] = OMcp2_28+qd[9]*ROcp2_56;
  OMxF1[3] = OMcp2_38+qd[9]*S6;
  AxF1[1] = -(qd[1]*(ORcp2_22+ORcp2_23+(2.0)*qd[2]*S1+(2.0)*qd[3]*C1)-qdd[2]*C1+qdd[3]*S1-OMcp2_27*ORcp2_38-OMcp2_28*ORcp2_39+
 OMcp2_37*ORcp2_28+OMcp2_38*ORcp2_29-OPcp2_27*RLcp2_38-OPcp2_28*RLcp2_39+OPcp2_37*RLcp2_28+OPcp2_38*RLcp2_29);
  AxF1[2] = qd[1]*(ORcp2_12+ORcp2_13+(2.0)*qd[2]*C1-(2.0)*qd[3]*S1)+qdd[2]*S1+qdd[3]*C1-OMcp2_17*ORcp2_38-OMcp2_18*ORcp2_39+
 OMcp2_37*ORcp2_18+OMcp2_38*ORcp2_19-OPcp2_17*RLcp2_38-OPcp2_18*RLcp2_39+OPcp2_37*RLcp2_18+OPcp2_38*RLcp2_19;
  AxF1[3] = qdd[4]+OMcp2_17*ORcp2_28+OMcp2_18*ORcp2_29-OMcp2_27*ORcp2_18-OMcp2_28*ORcp2_19+OPcp2_17*RLcp2_28+OPcp2_18*
 RLcp2_29-OPcp2_27*RLcp2_18-OPcp2_28*RLcp2_19;
  OMPxF1[1] = OPcp2_18+qd[9]*(OMcp2_28*S6-OMcp2_38*ROcp2_56)+qdd[9]*ROcp2_46;
  OMPxF1[2] = OPcp2_28-qd[9]*(OMcp2_18*S6-OMcp2_38*ROcp2_46)+qdd[9]*ROcp2_56;
  OMPxF1[3] = OPcp2_38+qd[9]*(OMcp2_18*ROcp2_56-OMcp2_28*ROcp2_46)+qdd[9]*S6;
 
// Sensor Forces Computation 

  SWr1 = user_ExtForces(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc13 = ROcp2_19*SWr1[1]+ROcp2_29*SWr1[2]+ROcp2_39*SWr1[3];
  xfrc23 = ROcp2_46*SWr1[1]+ROcp2_56*SWr1[2]+SWr1[3]*S6;
  xfrc33 = ROcp2_79*SWr1[1]+ROcp2_89*SWr1[2]+ROcp2_99*SWr1[3];
  frc[1][9] = s->frc[1][9]+xfrc13;
  frc[2][9] = s->frc[2][9]+xfrc23;
  frc[3][9] = s->frc[3][9]+xfrc33;
  xtrq13 = ROcp2_19*SWr1[4]+ROcp2_29*SWr1[5]+ROcp2_39*SWr1[6];
  xtrq23 = ROcp2_46*SWr1[4]+ROcp2_56*SWr1[5]+SWr1[6]*S6;
  xtrq33 = ROcp2_79*SWr1[4]+ROcp2_89*SWr1[5]+ROcp2_99*SWr1[6];
  trq[1][9] = s->trq[1][9]+xtrq13-xfrc23*SWr1[9]+xfrc33*SWr1[8];
  trq[2][9] = s->trq[2][9]+xtrq23+xfrc13*SWr1[9]-xfrc33*SWr1[7];
  trq[3][9] = s->trq[3][9]+xtrq33-xfrc13*SWr1[8]+xfrc23*SWr1[7];

// = = Block_0_0_1_2_0_1 = = 
 
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

// = = Block_0_0_1_2_0_3 = = 
 
// Trigonometric Variables  

  S10p7 = C10*S7+S10*C7;
  C10p7 = C10*C7-S10*S7;
 
// Sensor Kinematics 


  ROcp3_110 = ROcp3_17*C10-ROcp3_77*S10;
  ROcp3_210 = ROcp3_27*C10-ROcp3_87*S10;
  ROcp3_310 = -S10p7*C6;
  ROcp3_710 = ROcp3_17*S10+ROcp3_77*C10;
  ROcp3_810 = ROcp3_27*S10+ROcp3_87*C10;
  ROcp3_910 = C10p7*C6;
  RLcp3_110 = ROcp3_17*s->dpt[1][4]+ROcp3_77*s->dpt[3][4];
  RLcp3_210 = ROcp3_27*s->dpt[1][4]+ROcp3_87*s->dpt[3][4];
  RLcp3_310 = -C6*(s->dpt[1][4]*S7-s->dpt[3][4]*C7);
  ORcp3_110 = OMcp3_27*RLcp3_310-OMcp3_37*RLcp3_210;
  ORcp3_210 = -(OMcp3_17*RLcp3_310-OMcp3_37*RLcp3_110);
  ORcp3_310 = OMcp3_17*RLcp3_210-OMcp3_27*RLcp3_110;

// = = Block_0_0_1_2_0_5 = = 
 
// Sensor Kinematics 


  ROcp3_412 = ROcp3_46*C12+ROcp3_710*S12;
  ROcp3_512 = ROcp3_56*C12+ROcp3_810*S12;
  ROcp3_612 = ROcp3_910*S12+C12*S6;
  ROcp3_712 = -(ROcp3_46*S12-ROcp3_710*C12);
  ROcp3_812 = -(ROcp3_56*S12-ROcp3_810*C12);
  ROcp3_912 = ROcp3_910*C12-S12*S6;
  ROcp3_413 = ROcp3_412*C13+ROcp3_712*S13;
  ROcp3_513 = ROcp3_512*C13+ROcp3_812*S13;
  ROcp3_613 = ROcp3_612*C13+ROcp3_912*S13;
  ROcp3_713 = -(ROcp3_412*S13-ROcp3_712*C13);
  ROcp3_813 = -(ROcp3_512*S13-ROcp3_812*C13);
  ROcp3_913 = -(ROcp3_612*S13-ROcp3_912*C13);
  ROcp3_114 = ROcp3_110*C14-ROcp3_713*S14;
  ROcp3_214 = ROcp3_210*C14-ROcp3_813*S14;
  ROcp3_314 = ROcp3_310*C14-ROcp3_913*S14;
  ROcp3_714 = ROcp3_110*S14+ROcp3_713*C14;
  ROcp3_814 = ROcp3_210*S14+ROcp3_813*C14;
  ROcp3_914 = ROcp3_310*S14+ROcp3_913*C14;
  ROcp3_115 = ROcp3_114*C15+ROcp3_413*S15;
  ROcp3_215 = ROcp3_214*C15+ROcp3_513*S15;
  ROcp3_315 = ROcp3_314*C15+ROcp3_613*S15;
  ROcp3_415 = -(ROcp3_114*S15-ROcp3_413*C15);
  ROcp3_515 = -(ROcp3_214*S15-ROcp3_513*C15);
  ROcp3_615 = -(ROcp3_314*S15-ROcp3_613*C15);
  ROcp3_416 = ROcp3_415*C16+ROcp3_714*S16;
  ROcp3_516 = ROcp3_515*C16+ROcp3_814*S16;
  ROcp3_616 = ROcp3_615*C16+ROcp3_914*S16;
  ROcp3_716 = -(ROcp3_415*S16-ROcp3_714*C16);
  ROcp3_816 = -(ROcp3_515*S16-ROcp3_814*C16);
  ROcp3_916 = -(ROcp3_615*S16-ROcp3_914*C16);
  ROcp3_117 = ROcp3_115*C17-ROcp3_716*S17;
  ROcp3_217 = ROcp3_215*C17-ROcp3_816*S17;
  ROcp3_317 = ROcp3_315*C17-ROcp3_916*S17;
  ROcp3_717 = ROcp3_115*S17+ROcp3_716*C17;
  ROcp3_817 = ROcp3_215*S17+ROcp3_816*C17;
  ROcp3_917 = ROcp3_315*S17+ROcp3_916*C17;
  RLcp3_112 = ROcp3_46*s->dpt[2][10]+ROcp3_710*s->dpt[3][10];
  RLcp3_212 = ROcp3_56*s->dpt[2][10]+ROcp3_810*s->dpt[3][10];
  RLcp3_312 = ROcp3_910*s->dpt[3][10]+s->dpt[2][10]*S6;
  OMcp3_112 = OMcp3_17+qd[12]*ROcp3_110;
  OMcp3_212 = OMcp3_27+qd[12]*ROcp3_210;
  OMcp3_312 = OMcp3_37+qd[12]*ROcp3_310;
  ORcp3_112 = OMcp3_27*RLcp3_312-OMcp3_37*RLcp3_212;
  ORcp3_212 = -(OMcp3_17*RLcp3_312-OMcp3_37*RLcp3_112);
  ORcp3_312 = OMcp3_17*RLcp3_212-OMcp3_27*RLcp3_112;
  OPcp3_112 = OPcp3_17+qd[12]*(OMcp3_27*ROcp3_310-OMcp3_37*ROcp3_210)+qdd[12]*ROcp3_110;
  OPcp3_212 = OPcp3_27-qd[12]*(OMcp3_17*ROcp3_310-OMcp3_37*ROcp3_110)+qdd[12]*ROcp3_210;
  OPcp3_312 = OPcp3_37+qd[12]*(OMcp3_17*ROcp3_210-OMcp3_27*ROcp3_110)+qdd[12]*ROcp3_310;
  RLcp3_113 = ROcp3_412*s->dpt[2][15];
  RLcp3_213 = ROcp3_512*s->dpt[2][15];
  RLcp3_313 = ROcp3_612*s->dpt[2][15];
  OMcp3_113 = OMcp3_112+qd[13]*ROcp3_110;
  OMcp3_213 = OMcp3_212+qd[13]*ROcp3_210;
  OMcp3_313 = OMcp3_312+qd[13]*ROcp3_310;
  ORcp3_113 = OMcp3_212*RLcp3_313-OMcp3_312*RLcp3_213;
  ORcp3_213 = -(OMcp3_112*RLcp3_313-OMcp3_312*RLcp3_113);
  ORcp3_313 = OMcp3_112*RLcp3_213-OMcp3_212*RLcp3_113;
  OMcp3_114 = OMcp3_113+qd[14]*ROcp3_413;
  OMcp3_214 = OMcp3_213+qd[14]*ROcp3_513;
  OMcp3_314 = OMcp3_313+qd[14]*ROcp3_613;
  OMcp3_115 = OMcp3_114+qd[15]*ROcp3_714;
  OMcp3_215 = OMcp3_214+qd[15]*ROcp3_814;
  OMcp3_315 = OMcp3_314+qd[15]*ROcp3_914;
  OPcp3_115 = OPcp3_112+qd[13]*(OMcp3_212*ROcp3_310-OMcp3_312*ROcp3_210)+qd[14]*(OMcp3_213*ROcp3_613-OMcp3_313*ROcp3_513
 )+qd[15]*(OMcp3_214*ROcp3_914-OMcp3_314*ROcp3_814)+qdd[13]*ROcp3_110+qdd[14]*ROcp3_413+qdd[15]*ROcp3_714;
  OPcp3_215 = OPcp3_212-qd[13]*(OMcp3_112*ROcp3_310-OMcp3_312*ROcp3_110)-qd[14]*(OMcp3_113*ROcp3_613-OMcp3_313*ROcp3_413
 )-qd[15]*(OMcp3_114*ROcp3_914-OMcp3_314*ROcp3_714)+qdd[13]*ROcp3_210+qdd[14]*ROcp3_513+qdd[15]*ROcp3_814;
  OPcp3_315 = OPcp3_312+qd[13]*(OMcp3_112*ROcp3_210-OMcp3_212*ROcp3_110)+qd[14]*(OMcp3_113*ROcp3_513-OMcp3_213*ROcp3_413
 )+qd[15]*(OMcp3_114*ROcp3_814-OMcp3_214*ROcp3_714)+qdd[13]*ROcp3_310+qdd[14]*ROcp3_613+qdd[15]*ROcp3_914;
  RLcp3_116 = ROcp3_115*s->dpt[1][18]+ROcp3_415*s->dpt[2][18]+ROcp3_714*s->dpt[3][18];
  RLcp3_216 = ROcp3_215*s->dpt[1][18]+ROcp3_515*s->dpt[2][18]+ROcp3_814*s->dpt[3][18];
  RLcp3_316 = ROcp3_315*s->dpt[1][18]+ROcp3_615*s->dpt[2][18]+ROcp3_914*s->dpt[3][18];
  ORcp3_116 = OMcp3_215*RLcp3_316-OMcp3_315*RLcp3_216;
  ORcp3_216 = -(OMcp3_115*RLcp3_316-OMcp3_315*RLcp3_116);
  ORcp3_316 = OMcp3_115*RLcp3_216-OMcp3_215*RLcp3_116;
  PxF2[1] = RLcp3_110+RLcp3_112+RLcp3_113+RLcp3_116+RLcp3_12+RLcp3_13;
  PxF2[2] = RLcp3_210+RLcp3_212+RLcp3_213+RLcp3_216+RLcp3_22+RLcp3_23;
  PxF2[3] = q[4]+RLcp3_310+RLcp3_312+RLcp3_313+RLcp3_316;
  RxF2[1][1] = ROcp3_117;
  RxF2[1][2] = ROcp3_217;
  RxF2[1][3] = ROcp3_317;
  RxF2[2][1] = ROcp3_416;
  RxF2[2][2] = ROcp3_516;
  RxF2[2][3] = ROcp3_616;
  RxF2[3][1] = ROcp3_717;
  RxF2[3][2] = ROcp3_817;
  RxF2[3][3] = ROcp3_917;
  VxF2[1] = ORcp3_110+ORcp3_112+ORcp3_113+ORcp3_116+ORcp3_12+ORcp3_13+qd[2]*C1-qd[3]*S1;
  VxF2[2] = ORcp3_210+ORcp3_212+ORcp3_213+ORcp3_216+ORcp3_22+ORcp3_23+qd[2]*S1+qd[3]*C1;
  VxF2[3] = qd[4]+ORcp3_310+ORcp3_312+ORcp3_313+ORcp3_316;
  OMxF2[1] = OMcp3_115+qd[17]*ROcp3_416;
  OMxF2[2] = OMcp3_215+qd[17]*ROcp3_516;
  OMxF2[3] = OMcp3_315+qd[17]*ROcp3_616;
  AxF2[1] = -(qd[1]*(ORcp3_22+ORcp3_23+(2.0)*qd[2]*S1+(2.0)*qd[3]*C1)-qdd[2]*C1+qdd[3]*S1-OMcp3_212*ORcp3_313-OMcp3_215*ORcp3_316-
 OMcp3_27*ORcp3_310-OMcp3_27*ORcp3_312+OMcp3_312*ORcp3_213+OMcp3_315*ORcp3_216+OMcp3_37*ORcp3_210+OMcp3_37*ORcp3_212-
 OPcp3_212*RLcp3_313-OPcp3_215*RLcp3_316-OPcp3_27*RLcp3_310-OPcp3_27*RLcp3_312+OPcp3_312*RLcp3_213+OPcp3_315*RLcp3_216+
 OPcp3_37*RLcp3_210+OPcp3_37*RLcp3_212);
  AxF2[2] = qd[1]*(ORcp3_12+ORcp3_13+(2.0)*qd[2]*C1-(2.0)*qd[3]*S1)+qdd[2]*S1+qdd[3]*C1-OMcp3_112*ORcp3_313-OMcp3_115*ORcp3_316-
 OMcp3_17*ORcp3_310-OMcp3_17*ORcp3_312+OMcp3_312*ORcp3_113+OMcp3_315*ORcp3_116+OMcp3_37*ORcp3_110+OMcp3_37*ORcp3_112-
 OPcp3_112*RLcp3_313-OPcp3_115*RLcp3_316-OPcp3_17*RLcp3_310-OPcp3_17*RLcp3_312+OPcp3_312*RLcp3_113+OPcp3_315*RLcp3_116+
 OPcp3_37*RLcp3_110+OPcp3_37*RLcp3_112;
  AxF2[3] = qdd[4]+OMcp3_112*ORcp3_213+OMcp3_115*ORcp3_216+OMcp3_17*ORcp3_210+OMcp3_17*ORcp3_212-OMcp3_212*ORcp3_113-
 OMcp3_215*ORcp3_116-OMcp3_27*ORcp3_110-OMcp3_27*ORcp3_112+OPcp3_112*RLcp3_213+OPcp3_115*RLcp3_216+OPcp3_17*RLcp3_210+
 OPcp3_17*RLcp3_212-OPcp3_212*RLcp3_113-OPcp3_215*RLcp3_116-OPcp3_27*RLcp3_110-OPcp3_27*RLcp3_112;
  OMPxF2[1] = OPcp3_115+qd[17]*(OMcp3_215*ROcp3_616-OMcp3_315*ROcp3_516)+qdd[17]*ROcp3_416;
  OMPxF2[2] = OPcp3_215-qd[17]*(OMcp3_115*ROcp3_616-OMcp3_315*ROcp3_416)+qdd[17]*ROcp3_516;
  OMPxF2[3] = OPcp3_315+qd[17]*(OMcp3_115*ROcp3_516-OMcp3_215*ROcp3_416)+qdd[17]*ROcp3_616;
 
// Sensor Forces Computation 

  SWr2 = user_ExtForces(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc14 = ROcp3_117*SWr2[1]+ROcp3_217*SWr2[2]+ROcp3_317*SWr2[3];
  xfrc24 = ROcp3_416*SWr2[1]+ROcp3_516*SWr2[2]+ROcp3_616*SWr2[3];
  xfrc34 = ROcp3_717*SWr2[1]+ROcp3_817*SWr2[2]+ROcp3_917*SWr2[3];
  frc[1][17] = s->frc[1][17]+xfrc14;
  frc[2][17] = s->frc[2][17]+xfrc24;
  frc[3][17] = s->frc[3][17]+xfrc34;
  xtrq14 = ROcp3_117*SWr2[4]+ROcp3_217*SWr2[5]+ROcp3_317*SWr2[6];
  xtrq24 = ROcp3_416*SWr2[4]+ROcp3_516*SWr2[5]+ROcp3_616*SWr2[6];
  xtrq34 = ROcp3_717*SWr2[4]+ROcp3_817*SWr2[5]+ROcp3_917*SWr2[6];
  trq[1][17] = s->trq[1][17]+xtrq14-xfrc24*SWr2[9]+xfrc34*SWr2[8];
  trq[2][17] = s->trq[2][17]+xtrq24+xfrc14*SWr2[9]-xfrc34*SWr2[7];
  trq[3][17] = s->trq[3][17]+xtrq34-xfrc14*SWr2[8]+xfrc24*SWr2[7];

// = = Block_0_0_1_3_0_1 = = 
 
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

// = = Block_0_0_1_3_0_3 = = 
 
// Sensor Kinematics 


  ROcp4_110 = ROcp4_17*C10-ROcp4_77*S10;
  ROcp4_210 = ROcp4_27*C10-ROcp4_87*S10;
  ROcp4_310 = -S10p7*C6;
  ROcp4_710 = ROcp4_17*S10+ROcp4_77*C10;
  ROcp4_810 = ROcp4_27*S10+ROcp4_87*C10;
  ROcp4_910 = C10p7*C6;
  RLcp4_110 = ROcp4_17*s->dpt[1][4]+ROcp4_77*s->dpt[3][4];
  RLcp4_210 = ROcp4_27*s->dpt[1][4]+ROcp4_87*s->dpt[3][4];
  RLcp4_310 = -C6*(s->dpt[1][4]*S7-s->dpt[3][4]*C7);
  ORcp4_110 = OMcp4_27*RLcp4_310-OMcp4_37*RLcp4_210;
  ORcp4_210 = -(OMcp4_17*RLcp4_310-OMcp4_37*RLcp4_110);
  ORcp4_310 = OMcp4_17*RLcp4_210-OMcp4_27*RLcp4_110;

// = = Block_0_0_1_3_0_6 = = 
 
// Sensor Kinematics 


  ROcp4_418 = ROcp4_46*C18+ROcp4_710*S18;
  ROcp4_518 = ROcp4_56*C18+ROcp4_810*S18;
  ROcp4_618 = ROcp4_910*S18+C18*S6;
  ROcp4_718 = -(ROcp4_46*S18-ROcp4_710*C18);
  ROcp4_818 = -(ROcp4_56*S18-ROcp4_810*C18);
  ROcp4_918 = ROcp4_910*C18-S18*S6;
  ROcp4_419 = ROcp4_418*C19+ROcp4_718*S19;
  ROcp4_519 = ROcp4_518*C19+ROcp4_818*S19;
  ROcp4_619 = ROcp4_618*C19+ROcp4_918*S19;
  ROcp4_719 = -(ROcp4_418*S19-ROcp4_718*C19);
  ROcp4_819 = -(ROcp4_518*S19-ROcp4_818*C19);
  ROcp4_919 = -(ROcp4_618*S19-ROcp4_918*C19);
  ROcp4_120 = ROcp4_110*C20-ROcp4_719*S20;
  ROcp4_220 = ROcp4_210*C20-ROcp4_819*S20;
  ROcp4_320 = ROcp4_310*C20-ROcp4_919*S20;
  ROcp4_720 = ROcp4_110*S20+ROcp4_719*C20;
  ROcp4_820 = ROcp4_210*S20+ROcp4_819*C20;
  ROcp4_920 = ROcp4_310*S20+ROcp4_919*C20;
  ROcp4_121 = ROcp4_120*C21+ROcp4_419*S21;
  ROcp4_221 = ROcp4_220*C21+ROcp4_519*S21;
  ROcp4_321 = ROcp4_320*C21+ROcp4_619*S21;
  ROcp4_421 = -(ROcp4_120*S21-ROcp4_419*C21);
  ROcp4_521 = -(ROcp4_220*S21-ROcp4_519*C21);
  ROcp4_621 = -(ROcp4_320*S21-ROcp4_619*C21);
  ROcp4_422 = ROcp4_421*C22+ROcp4_720*S22;
  ROcp4_522 = ROcp4_521*C22+ROcp4_820*S22;
  ROcp4_622 = ROcp4_621*C22+ROcp4_920*S22;
  ROcp4_722 = -(ROcp4_421*S22-ROcp4_720*C22);
  ROcp4_822 = -(ROcp4_521*S22-ROcp4_820*C22);
  ROcp4_922 = -(ROcp4_621*S22-ROcp4_920*C22);
  ROcp4_123 = ROcp4_121*C23-ROcp4_722*S23;
  ROcp4_223 = ROcp4_221*C23-ROcp4_822*S23;
  ROcp4_323 = ROcp4_321*C23-ROcp4_922*S23;
  ROcp4_723 = ROcp4_121*S23+ROcp4_722*C23;
  ROcp4_823 = ROcp4_221*S23+ROcp4_822*C23;
  ROcp4_923 = ROcp4_321*S23+ROcp4_922*C23;
  RLcp4_118 = ROcp4_46*s->dpt[2][11]+ROcp4_710*s->dpt[3][11];
  RLcp4_218 = ROcp4_56*s->dpt[2][11]+ROcp4_810*s->dpt[3][11];
  RLcp4_318 = ROcp4_910*s->dpt[3][11]+s->dpt[2][11]*S6;
  OMcp4_118 = OMcp4_17+qd[18]*ROcp4_110;
  OMcp4_218 = OMcp4_27+qd[18]*ROcp4_210;
  OMcp4_318 = OMcp4_37+qd[18]*ROcp4_310;
  ORcp4_118 = OMcp4_27*RLcp4_318-OMcp4_37*RLcp4_218;
  ORcp4_218 = -(OMcp4_17*RLcp4_318-OMcp4_37*RLcp4_118);
  ORcp4_318 = OMcp4_17*RLcp4_218-OMcp4_27*RLcp4_118;
  OPcp4_118 = OPcp4_17+qd[18]*(OMcp4_27*ROcp4_310-OMcp4_37*ROcp4_210)+qdd[18]*ROcp4_110;
  OPcp4_218 = OPcp4_27-qd[18]*(OMcp4_17*ROcp4_310-OMcp4_37*ROcp4_110)+qdd[18]*ROcp4_210;
  OPcp4_318 = OPcp4_37+qd[18]*(OMcp4_17*ROcp4_210-OMcp4_27*ROcp4_110)+qdd[18]*ROcp4_310;
  RLcp4_119 = ROcp4_418*s->dpt[2][21];
  RLcp4_219 = ROcp4_518*s->dpt[2][21];
  RLcp4_319 = ROcp4_618*s->dpt[2][21];
  OMcp4_119 = OMcp4_118+qd[19]*ROcp4_110;
  OMcp4_219 = OMcp4_218+qd[19]*ROcp4_210;
  OMcp4_319 = OMcp4_318+qd[19]*ROcp4_310;
  ORcp4_119 = OMcp4_218*RLcp4_319-OMcp4_318*RLcp4_219;
  ORcp4_219 = -(OMcp4_118*RLcp4_319-OMcp4_318*RLcp4_119);
  ORcp4_319 = OMcp4_118*RLcp4_219-OMcp4_218*RLcp4_119;
  OMcp4_120 = OMcp4_119+qd[20]*ROcp4_419;
  OMcp4_220 = OMcp4_219+qd[20]*ROcp4_519;
  OMcp4_320 = OMcp4_319+qd[20]*ROcp4_619;
  OMcp4_121 = OMcp4_120+qd[21]*ROcp4_720;
  OMcp4_221 = OMcp4_220+qd[21]*ROcp4_820;
  OMcp4_321 = OMcp4_320+qd[21]*ROcp4_920;
  OPcp4_121 = OPcp4_118+qd[19]*(OMcp4_218*ROcp4_310-OMcp4_318*ROcp4_210)+qd[20]*(OMcp4_219*ROcp4_619-OMcp4_319*ROcp4_519
 )+qd[21]*(OMcp4_220*ROcp4_920-OMcp4_320*ROcp4_820)+qdd[19]*ROcp4_110+qdd[20]*ROcp4_419+qdd[21]*ROcp4_720;
  OPcp4_221 = OPcp4_218-qd[19]*(OMcp4_118*ROcp4_310-OMcp4_318*ROcp4_110)-qd[20]*(OMcp4_119*ROcp4_619-OMcp4_319*ROcp4_419
 )-qd[21]*(OMcp4_120*ROcp4_920-OMcp4_320*ROcp4_720)+qdd[19]*ROcp4_210+qdd[20]*ROcp4_519+qdd[21]*ROcp4_820;
  OPcp4_321 = OPcp4_318+qd[19]*(OMcp4_118*ROcp4_210-OMcp4_218*ROcp4_110)+qd[20]*(OMcp4_119*ROcp4_519-OMcp4_219*ROcp4_419
 )+qd[21]*(OMcp4_120*ROcp4_820-OMcp4_220*ROcp4_720)+qdd[19]*ROcp4_310+qdd[20]*ROcp4_619+qdd[21]*ROcp4_920;
  RLcp4_122 = ROcp4_121*s->dpt[1][24]+ROcp4_421*s->dpt[2][24]+ROcp4_720*s->dpt[3][24];
  RLcp4_222 = ROcp4_221*s->dpt[1][24]+ROcp4_521*s->dpt[2][24]+ROcp4_820*s->dpt[3][24];
  RLcp4_322 = ROcp4_321*s->dpt[1][24]+ROcp4_621*s->dpt[2][24]+ROcp4_920*s->dpt[3][24];
  ORcp4_122 = OMcp4_221*RLcp4_322-OMcp4_321*RLcp4_222;
  ORcp4_222 = -(OMcp4_121*RLcp4_322-OMcp4_321*RLcp4_122);
  ORcp4_322 = OMcp4_121*RLcp4_222-OMcp4_221*RLcp4_122;
  PxF3[1] = RLcp4_110+RLcp4_118+RLcp4_119+RLcp4_12+RLcp4_122+RLcp4_13;
  PxF3[2] = RLcp4_210+RLcp4_218+RLcp4_219+RLcp4_22+RLcp4_222+RLcp4_23;
  PxF3[3] = q[4]+RLcp4_310+RLcp4_318+RLcp4_319+RLcp4_322;
  RxF3[1][1] = ROcp4_123;
  RxF3[1][2] = ROcp4_223;
  RxF3[1][3] = ROcp4_323;
  RxF3[2][1] = ROcp4_422;
  RxF3[2][2] = ROcp4_522;
  RxF3[2][3] = ROcp4_622;
  RxF3[3][1] = ROcp4_723;
  RxF3[3][2] = ROcp4_823;
  RxF3[3][3] = ROcp4_923;
  VxF3[1] = ORcp4_110+ORcp4_118+ORcp4_119+ORcp4_12+ORcp4_122+ORcp4_13+qd[2]*C1-qd[3]*S1;
  VxF3[2] = ORcp4_210+ORcp4_218+ORcp4_219+ORcp4_22+ORcp4_222+ORcp4_23+qd[2]*S1+qd[3]*C1;
  VxF3[3] = qd[4]+ORcp4_310+ORcp4_318+ORcp4_319+ORcp4_322;
  OMxF3[1] = OMcp4_121+qd[23]*ROcp4_422;
  OMxF3[2] = OMcp4_221+qd[23]*ROcp4_522;
  OMxF3[3] = OMcp4_321+qd[23]*ROcp4_622;
  AxF3[1] = -(qd[1]*(ORcp4_22+ORcp4_23+(2.0)*qd[2]*S1+(2.0)*qd[3]*C1)-qdd[2]*C1+qdd[3]*S1-OMcp4_218*ORcp4_319-OMcp4_221*ORcp4_322-
 OMcp4_27*ORcp4_310-OMcp4_27*ORcp4_318+OMcp4_318*ORcp4_219+OMcp4_321*ORcp4_222+OMcp4_37*ORcp4_210+OMcp4_37*ORcp4_218-
 OPcp4_218*RLcp4_319-OPcp4_221*RLcp4_322-OPcp4_27*RLcp4_310-OPcp4_27*RLcp4_318+OPcp4_318*RLcp4_219+OPcp4_321*RLcp4_222+
 OPcp4_37*RLcp4_210+OPcp4_37*RLcp4_218);
  AxF3[2] = qd[1]*(ORcp4_12+ORcp4_13+(2.0)*qd[2]*C1-(2.0)*qd[3]*S1)+qdd[2]*S1+qdd[3]*C1-OMcp4_118*ORcp4_319-OMcp4_121*ORcp4_322-
 OMcp4_17*ORcp4_310-OMcp4_17*ORcp4_318+OMcp4_318*ORcp4_119+OMcp4_321*ORcp4_122+OMcp4_37*ORcp4_110+OMcp4_37*ORcp4_118-
 OPcp4_118*RLcp4_319-OPcp4_121*RLcp4_322-OPcp4_17*RLcp4_310-OPcp4_17*RLcp4_318+OPcp4_318*RLcp4_119+OPcp4_321*RLcp4_122+
 OPcp4_37*RLcp4_110+OPcp4_37*RLcp4_118;
  AxF3[3] = qdd[4]+OMcp4_118*ORcp4_219+OMcp4_121*ORcp4_222+OMcp4_17*ORcp4_210+OMcp4_17*ORcp4_218-OMcp4_218*ORcp4_119-
 OMcp4_221*ORcp4_122-OMcp4_27*ORcp4_110-OMcp4_27*ORcp4_118+OPcp4_118*RLcp4_219+OPcp4_121*RLcp4_222+OPcp4_17*RLcp4_210+
 OPcp4_17*RLcp4_218-OPcp4_218*RLcp4_119-OPcp4_221*RLcp4_122-OPcp4_27*RLcp4_110-OPcp4_27*RLcp4_118;
  OMPxF3[1] = OPcp4_121+qd[23]*(OMcp4_221*ROcp4_622-OMcp4_321*ROcp4_522)+qdd[23]*ROcp4_422;
  OMPxF3[2] = OPcp4_221-qd[23]*(OMcp4_121*ROcp4_622-OMcp4_321*ROcp4_422)+qdd[23]*ROcp4_522;
  OMPxF3[3] = OPcp4_321+qd[23]*(OMcp4_121*ROcp4_522-OMcp4_221*ROcp4_422)+qdd[23]*ROcp4_622;
 
// Sensor Forces Computation 

  SWr3 = user_ExtForces(PxF3,RxF3,VxF3,OMxF3,AxF3,OMPxF3,s,tsim,3);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc15 = ROcp4_123*SWr3[1]+ROcp4_223*SWr3[2]+ROcp4_323*SWr3[3];
  xfrc25 = ROcp4_422*SWr3[1]+ROcp4_522*SWr3[2]+ROcp4_622*SWr3[3];
  xfrc35 = ROcp4_723*SWr3[1]+ROcp4_823*SWr3[2]+ROcp4_923*SWr3[3];
  frc[1][23] = s->frc[1][23]+xfrc15;
  frc[2][23] = s->frc[2][23]+xfrc25;
  frc[3][23] = s->frc[3][23]+xfrc35;
  xtrq15 = ROcp4_123*SWr3[4]+ROcp4_223*SWr3[5]+ROcp4_323*SWr3[6];
  xtrq25 = ROcp4_422*SWr3[4]+ROcp4_522*SWr3[5]+ROcp4_622*SWr3[6];
  xtrq35 = ROcp4_723*SWr3[4]+ROcp4_823*SWr3[5]+ROcp4_923*SWr3[6];
  trq[1][23] = s->trq[1][23]+xtrq15-xfrc25*SWr3[9]+xfrc35*SWr3[8];
  trq[2][23] = s->trq[2][23]+xtrq25+xfrc15*SWr3[9]-xfrc35*SWr3[7];
  trq[3][23] = s->trq[3][23]+xtrq35-xfrc15*SWr3[8]+xfrc25*SWr3[7];

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
 


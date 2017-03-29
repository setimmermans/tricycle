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
//	==> Generation Date : Tue Mar 28 10:33:05 2017
//
//	==> Project name : tricycle
//	==> using XML input file 
//
//	==> Number of joints : 31
//
//	==> Function : F19 : External Forces
//	==> Flops complexity : 1775
//
//	==> Generation Time :  0.030 seconds
//	==> Post-Processing :  0.040 seconds
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
double PxF4[4]; 
double RxF4[4][4]; 
double VxF4[4]; 
double OMxF4[4]; 
double AxF4[4]; 
double OMPxF4[4]; 
double *SWr4; 
 
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
  ROcp11_37 = -C6*S7;
  ROcp11_77 = ROcp11_76*C7+C1p5*S7;
  ROcp11_87 = ROcp11_86*C7+S1p5*S7;
  ROcp11_97 = C6*C7;
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
  RLcp11_157 = ROcp11_17*s->dpt[1][4];
  RLcp11_257 = ROcp11_27*s->dpt[1][4];
  RLcp11_357 = ROcp11_37*s->dpt[1][4];
  ORcp11_157 = OMcp11_27*RLcp11_357-OMcp11_37*RLcp11_257;
  ORcp11_257 = -(OMcp11_17*RLcp11_357-OMcp11_37*RLcp11_157);
  ORcp11_357 = OMcp11_17*RLcp11_257-OMcp11_27*RLcp11_157;
  PxF1[1] = RLcp11_12+RLcp11_13+RLcp11_157;
  PxF1[2] = RLcp11_22+RLcp11_23+RLcp11_257;
  PxF1[3] = q[4]+RLcp11_357;
  RxF1[1][1] = ROcp11_17;
  RxF1[1][2] = ROcp11_27;
  RxF1[1][3] = ROcp11_37;
  RxF1[2][1] = ROcp11_46;
  RxF1[2][2] = ROcp11_56;
  RxF1[2][3] = S6;
  RxF1[3][1] = ROcp11_77;
  RxF1[3][2] = ROcp11_87;
  RxF1[3][3] = ROcp11_97;
  VxF1[1] = ORcp11_12+ORcp11_13+ORcp11_157+qd[2]*C1-qd[3]*S1;
  VxF1[2] = ORcp11_22+ORcp11_23+ORcp11_257+qd[2]*S1+qd[3]*C1;
  VxF1[3] = qd[4]+ORcp11_357;
  OMxF1[1] = OMcp11_17;
  OMxF1[2] = OMcp11_27;
  OMxF1[3] = OMcp11_37;
  AxF1[1] = -(qd[1]*(ORcp11_22+ORcp11_23+(2.0)*qd[2]*S1+(2.0)*qd[3]*C1)-qdd[2]*C1+qdd[3]*S1-OMcp11_27*ORcp11_357+OMcp11_37*
 ORcp11_257-OPcp11_27*RLcp11_357+OPcp11_37*RLcp11_257);
  AxF1[2] = qd[1]*(ORcp11_12+ORcp11_13+(2.0)*qd[2]*C1-(2.0)*qd[3]*S1)+qdd[2]*S1+qdd[3]*C1-OMcp11_17*ORcp11_357+OMcp11_37*
 ORcp11_157-OPcp11_17*RLcp11_357+OPcp11_37*RLcp11_157;
  AxF1[3] = qdd[4]+OMcp11_17*ORcp11_257-OMcp11_27*ORcp11_157+OPcp11_17*RLcp11_257-OPcp11_27*RLcp11_157;
  OMPxF1[1] = OPcp11_17;
  OMPxF1[2] = OPcp11_27;
  OMPxF1[3] = OPcp11_37;
 
// Sensor Forces Computation 

  SWr1 = user_ExtForces(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc112 = ROcp11_17*SWr1[1]+ROcp11_27*SWr1[2]+ROcp11_37*SWr1[3];
  xfrc212 = ROcp11_46*SWr1[1]+ROcp11_56*SWr1[2]+SWr1[3]*S6;
  xfrc312 = ROcp11_77*SWr1[1]+ROcp11_87*SWr1[2]+ROcp11_97*SWr1[3];
  frc[1][7] = s->frc[1][7]+xfrc112;
  frc[2][7] = s->frc[2][7]+xfrc212;
  frc[3][7] = s->frc[3][7]+xfrc312;
  xtrq112 = ROcp11_17*SWr1[4]+ROcp11_27*SWr1[5]+ROcp11_37*SWr1[6];
  xtrq212 = ROcp11_46*SWr1[4]+ROcp11_56*SWr1[5]+SWr1[6]*S6;
  xtrq312 = ROcp11_77*SWr1[4]+ROcp11_87*SWr1[5]+ROcp11_97*SWr1[6];
  trq[1][7] = s->trq[1][7]+xtrq112-xfrc212*(SWr1[9]-s->l[3][7])+xfrc312*SWr1[8];
  trq[2][7] = s->trq[2][7]+xtrq212+xfrc112*(SWr1[9]-s->l[3][7])-xfrc312*(SWr1[7]-s->l[1][7]);
  trq[3][7] = s->trq[3][7]+xtrq312-xfrc112*SWr1[8]+xfrc212*(SWr1[7]-s->l[1][7]);

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

// = = Block_0_0_1_2_0_2 = = 
 
// Trigonometric Variables  

  S7p8 = C7*S8+S7*C8;
  C7p8 = C7*C8-S7*S8;
  S7p8p9 = C7p8*S9+S7p8*C9;
  C7p8p9 = C7p8*C9-S7p8*S9;
 
// Sensor Kinematics 


  ROcp12_18 = ROcp12_17*C8-ROcp12_77*S8;
  ROcp12_28 = ROcp12_27*C8-ROcp12_87*S8;
  ROcp12_78 = ROcp12_17*S8+ROcp12_77*C8;
  ROcp12_88 = ROcp12_27*S8+ROcp12_87*C8;
  ROcp12_19 = ROcp12_18*C9-ROcp12_78*S9;
  ROcp12_29 = ROcp12_28*C9-ROcp12_88*S9;
  ROcp12_39 = -S7p8p9*C6;
  ROcp12_79 = ROcp12_18*S9+ROcp12_78*C9;
  ROcp12_89 = ROcp12_28*S9+ROcp12_88*C9;
  ROcp12_99 = C7p8p9*C6;
  RLcp12_18 = ROcp12_17*s->dpt[1][2]+ROcp12_77*s->dpt[3][2];
  RLcp12_28 = ROcp12_27*s->dpt[1][2]+ROcp12_87*s->dpt[3][2];
  RLcp12_38 = -C6*(s->dpt[1][2]*S7-s->dpt[3][2]*C7);
  OMcp12_18 = OMcp12_17+qd[8]*ROcp12_46;
  OMcp12_28 = OMcp12_27+qd[8]*ROcp12_56;
  OMcp12_38 = OMcp12_37+qd[8]*S6;
  ORcp12_18 = OMcp12_27*RLcp12_38-OMcp12_37*RLcp12_28;
  ORcp12_28 = -(OMcp12_17*RLcp12_38-OMcp12_37*RLcp12_18);
  ORcp12_38 = OMcp12_17*RLcp12_28-OMcp12_27*RLcp12_18;
  OPcp12_18 = OPcp12_17+qd[8]*(OMcp12_27*S6-OMcp12_37*ROcp12_56)+qdd[8]*ROcp12_46;
  OPcp12_28 = OPcp12_27-qd[8]*(OMcp12_17*S6-OMcp12_37*ROcp12_46)+qdd[8]*ROcp12_56;
  OPcp12_38 = OPcp12_37+qd[8]*(OMcp12_17*ROcp12_56-OMcp12_27*ROcp12_46)+qdd[8]*S6;
  RLcp12_19 = ROcp12_18*s->dpt[1][8];
  RLcp12_29 = ROcp12_28*s->dpt[1][8];
  RLcp12_39 = -s->dpt[1][8]*S7p8*C6;
  ORcp12_19 = OMcp12_28*RLcp12_39-OMcp12_38*RLcp12_29;
  ORcp12_29 = -(OMcp12_18*RLcp12_39-OMcp12_38*RLcp12_19);
  ORcp12_39 = OMcp12_18*RLcp12_29-OMcp12_28*RLcp12_19;
  PxF2[1] = RLcp12_12+RLcp12_13+RLcp12_18+RLcp12_19;
  PxF2[2] = RLcp12_22+RLcp12_23+RLcp12_28+RLcp12_29;
  PxF2[3] = q[4]+RLcp12_38+RLcp12_39;
  RxF2[1][1] = ROcp12_19;
  RxF2[1][2] = ROcp12_29;
  RxF2[1][3] = ROcp12_39;
  RxF2[2][1] = ROcp12_46;
  RxF2[2][2] = ROcp12_56;
  RxF2[2][3] = S6;
  RxF2[3][1] = ROcp12_79;
  RxF2[3][2] = ROcp12_89;
  RxF2[3][3] = ROcp12_99;
  VxF2[1] = ORcp12_12+ORcp12_13+ORcp12_18+ORcp12_19+qd[2]*C1-qd[3]*S1;
  VxF2[2] = ORcp12_22+ORcp12_23+ORcp12_28+ORcp12_29+qd[2]*S1+qd[3]*C1;
  VxF2[3] = qd[4]+ORcp12_38+ORcp12_39;
  OMxF2[1] = OMcp12_18+qd[9]*ROcp12_46;
  OMxF2[2] = OMcp12_28+qd[9]*ROcp12_56;
  OMxF2[3] = OMcp12_38+qd[9]*S6;
  AxF2[1] = -(qd[1]*(ORcp12_22+ORcp12_23+(2.0)*qd[2]*S1+(2.0)*qd[3]*C1)-qdd[2]*C1+qdd[3]*S1-OMcp12_27*ORcp12_38-OMcp12_28*
 ORcp12_39+OMcp12_37*ORcp12_28+OMcp12_38*ORcp12_29-OPcp12_27*RLcp12_38-OPcp12_28*RLcp12_39+OPcp12_37*RLcp12_28+OPcp12_38*
 RLcp12_29);
  AxF2[2] = qd[1]*(ORcp12_12+ORcp12_13+(2.0)*qd[2]*C1-(2.0)*qd[3]*S1)+qdd[2]*S1+qdd[3]*C1-OMcp12_17*ORcp12_38-OMcp12_18*ORcp12_39+
 OMcp12_37*ORcp12_18+OMcp12_38*ORcp12_19-OPcp12_17*RLcp12_38-OPcp12_18*RLcp12_39+OPcp12_37*RLcp12_18+OPcp12_38*RLcp12_19;
  AxF2[3] = qdd[4]+OMcp12_17*ORcp12_28+OMcp12_18*ORcp12_29-OMcp12_27*ORcp12_18-OMcp12_28*ORcp12_19+OPcp12_17*RLcp12_28+
 OPcp12_18*RLcp12_29-OPcp12_27*RLcp12_18-OPcp12_28*RLcp12_19;
  OMPxF2[1] = OPcp12_18+qd[9]*(OMcp12_28*S6-OMcp12_38*ROcp12_56)+qdd[9]*ROcp12_46;
  OMPxF2[2] = OPcp12_28-qd[9]*(OMcp12_18*S6-OMcp12_38*ROcp12_46)+qdd[9]*ROcp12_56;
  OMPxF2[3] = OPcp12_38+qd[9]*(OMcp12_18*ROcp12_56-OMcp12_28*ROcp12_46)+qdd[9]*S6;
 
// Sensor Forces Computation 

  SWr2 = user_ExtForces(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc113 = ROcp12_19*SWr2[1]+ROcp12_29*SWr2[2]+ROcp12_39*SWr2[3];
  xfrc213 = ROcp12_46*SWr2[1]+ROcp12_56*SWr2[2]+SWr2[3]*S6;
  xfrc313 = ROcp12_79*SWr2[1]+ROcp12_89*SWr2[2]+ROcp12_99*SWr2[3];
  frc[1][9] = s->frc[1][9]+xfrc113;
  frc[2][9] = s->frc[2][9]+xfrc213;
  frc[3][9] = s->frc[3][9]+xfrc313;
  xtrq113 = ROcp12_19*SWr2[4]+ROcp12_29*SWr2[5]+ROcp12_39*SWr2[6];
  xtrq213 = ROcp12_46*SWr2[4]+ROcp12_56*SWr2[5]+SWr2[6]*S6;
  xtrq313 = ROcp12_79*SWr2[4]+ROcp12_89*SWr2[5]+ROcp12_99*SWr2[6];
  trq[1][9] = s->trq[1][9]+xtrq113-xfrc213*SWr2[9]+xfrc313*SWr2[8];
  trq[2][9] = s->trq[2][9]+xtrq213+xfrc113*SWr2[9]-xfrc313*SWr2[7];
  trq[3][9] = s->trq[3][9]+xtrq313-xfrc113*SWr2[8]+xfrc213*SWr2[7];

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
 
// Trigonometric Variables  

  S11p7 = C11*S7+S11*C7;
  C11p7 = C11*C7-S11*S7;
 
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

// = = Block_0_0_1_3_0_5 = = 
 
// Sensor Kinematics 


  ROcp13_413 = ROcp13_46*C13+ROcp13_711*S13;
  ROcp13_513 = ROcp13_56*C13+ROcp13_811*S13;
  ROcp13_613 = ROcp13_911*S13+C13*S6;
  ROcp13_713 = -(ROcp13_46*S13-ROcp13_711*C13);
  ROcp13_813 = -(ROcp13_56*S13-ROcp13_811*C13);
  ROcp13_913 = ROcp13_911*C13-S13*S6;
  ROcp13_414 = ROcp13_413*C14+ROcp13_713*S14;
  ROcp13_514 = ROcp13_513*C14+ROcp13_813*S14;
  ROcp13_614 = ROcp13_613*C14+ROcp13_913*S14;
  ROcp13_714 = -(ROcp13_413*S14-ROcp13_713*C14);
  ROcp13_814 = -(ROcp13_513*S14-ROcp13_813*C14);
  ROcp13_914 = -(ROcp13_613*S14-ROcp13_913*C14);
  ROcp13_115 = ROcp13_111*C15-ROcp13_714*S15;
  ROcp13_215 = ROcp13_211*C15-ROcp13_814*S15;
  ROcp13_315 = ROcp13_311*C15-ROcp13_914*S15;
  ROcp13_715 = ROcp13_111*S15+ROcp13_714*C15;
  ROcp13_815 = ROcp13_211*S15+ROcp13_814*C15;
  ROcp13_915 = ROcp13_311*S15+ROcp13_914*C15;
  ROcp13_116 = ROcp13_115*C16+ROcp13_414*S16;
  ROcp13_216 = ROcp13_215*C16+ROcp13_514*S16;
  ROcp13_316 = ROcp13_315*C16+ROcp13_614*S16;
  ROcp13_416 = -(ROcp13_115*S16-ROcp13_414*C16);
  ROcp13_516 = -(ROcp13_215*S16-ROcp13_514*C16);
  ROcp13_616 = -(ROcp13_315*S16-ROcp13_614*C16);
  ROcp13_417 = ROcp13_416*C17+ROcp13_715*S17;
  ROcp13_517 = ROcp13_516*C17+ROcp13_815*S17;
  ROcp13_617 = ROcp13_616*C17+ROcp13_915*S17;
  ROcp13_717 = -(ROcp13_416*S17-ROcp13_715*C17);
  ROcp13_817 = -(ROcp13_516*S17-ROcp13_815*C17);
  ROcp13_917 = -(ROcp13_616*S17-ROcp13_915*C17);
  ROcp13_118 = ROcp13_116*C18-ROcp13_717*S18;
  ROcp13_218 = ROcp13_216*C18-ROcp13_817*S18;
  ROcp13_318 = ROcp13_316*C18-ROcp13_917*S18;
  ROcp13_718 = ROcp13_116*S18+ROcp13_717*C18;
  ROcp13_818 = ROcp13_216*S18+ROcp13_817*C18;
  ROcp13_918 = ROcp13_316*S18+ROcp13_917*C18;
  RLcp13_113 = ROcp13_46*s->dpt[2][12]+ROcp13_711*s->dpt[3][12];
  RLcp13_213 = ROcp13_56*s->dpt[2][12]+ROcp13_811*s->dpt[3][12];
  RLcp13_313 = ROcp13_911*s->dpt[3][12]+s->dpt[2][12]*S6;
  OMcp13_113 = OMcp13_17+qd[13]*ROcp13_111;
  OMcp13_213 = OMcp13_27+qd[13]*ROcp13_211;
  OMcp13_313 = OMcp13_37+qd[13]*ROcp13_311;
  ORcp13_113 = OMcp13_27*RLcp13_313-OMcp13_37*RLcp13_213;
  ORcp13_213 = -(OMcp13_17*RLcp13_313-OMcp13_37*RLcp13_113);
  ORcp13_313 = OMcp13_17*RLcp13_213-OMcp13_27*RLcp13_113;
  OPcp13_113 = OPcp13_17+qd[13]*(OMcp13_27*ROcp13_311-OMcp13_37*ROcp13_211)+qdd[13]*ROcp13_111;
  OPcp13_213 = OPcp13_27-qd[13]*(OMcp13_17*ROcp13_311-OMcp13_37*ROcp13_111)+qdd[13]*ROcp13_211;
  OPcp13_313 = OPcp13_37+qd[13]*(OMcp13_17*ROcp13_211-OMcp13_27*ROcp13_111)+qdd[13]*ROcp13_311;
  RLcp13_114 = ROcp13_413*s->dpt[2][17]+ROcp13_713*s->dpt[3][17];
  RLcp13_214 = ROcp13_513*s->dpt[2][17]+ROcp13_813*s->dpt[3][17];
  RLcp13_314 = ROcp13_613*s->dpt[2][17]+ROcp13_913*s->dpt[3][17];
  OMcp13_114 = OMcp13_113+qd[14]*ROcp13_111;
  OMcp13_214 = OMcp13_213+qd[14]*ROcp13_211;
  OMcp13_314 = OMcp13_313+qd[14]*ROcp13_311;
  ORcp13_114 = OMcp13_213*RLcp13_314-OMcp13_313*RLcp13_214;
  ORcp13_214 = -(OMcp13_113*RLcp13_314-OMcp13_313*RLcp13_114);
  ORcp13_314 = OMcp13_113*RLcp13_214-OMcp13_213*RLcp13_114;
  OMcp13_115 = OMcp13_114+qd[15]*ROcp13_414;
  OMcp13_215 = OMcp13_214+qd[15]*ROcp13_514;
  OMcp13_315 = OMcp13_314+qd[15]*ROcp13_614;
  OMcp13_116 = OMcp13_115+qd[16]*ROcp13_715;
  OMcp13_216 = OMcp13_215+qd[16]*ROcp13_815;
  OMcp13_316 = OMcp13_315+qd[16]*ROcp13_915;
  OPcp13_116 = OPcp13_113+qd[14]*(OMcp13_213*ROcp13_311-OMcp13_313*ROcp13_211)+qd[15]*(OMcp13_214*ROcp13_614-OMcp13_314*
 ROcp13_514)+qd[16]*(OMcp13_215*ROcp13_915-OMcp13_315*ROcp13_815)+qdd[14]*ROcp13_111+qdd[15]*ROcp13_414+qdd[16]*ROcp13_715;
  OPcp13_216 = OPcp13_213-qd[14]*(OMcp13_113*ROcp13_311-OMcp13_313*ROcp13_111)-qd[15]*(OMcp13_114*ROcp13_614-OMcp13_314*
 ROcp13_414)-qd[16]*(OMcp13_115*ROcp13_915-OMcp13_315*ROcp13_715)+qdd[14]*ROcp13_211+qdd[15]*ROcp13_514+qdd[16]*ROcp13_815;
  OPcp13_316 = OPcp13_313+qd[14]*(OMcp13_113*ROcp13_211-OMcp13_213*ROcp13_111)+qd[15]*(OMcp13_114*ROcp13_514-OMcp13_214*
 ROcp13_414)+qd[16]*(OMcp13_115*ROcp13_815-OMcp13_215*ROcp13_715)+qdd[14]*ROcp13_311+qdd[15]*ROcp13_614+qdd[16]*ROcp13_915;
  RLcp13_117 = ROcp13_116*s->dpt[1][21]+ROcp13_416*s->dpt[2][21]+ROcp13_715*s->dpt[3][21];
  RLcp13_217 = ROcp13_216*s->dpt[1][21]+ROcp13_516*s->dpt[2][21]+ROcp13_815*s->dpt[3][21];
  RLcp13_317 = ROcp13_316*s->dpt[1][21]+ROcp13_616*s->dpt[2][21]+ROcp13_915*s->dpt[3][21];
  ORcp13_117 = OMcp13_216*RLcp13_317-OMcp13_316*RLcp13_217;
  ORcp13_217 = -(OMcp13_116*RLcp13_317-OMcp13_316*RLcp13_117);
  ORcp13_317 = OMcp13_116*RLcp13_217-OMcp13_216*RLcp13_117;
  PxF3[1] = RLcp13_111+RLcp13_113+RLcp13_114+RLcp13_117+RLcp13_12+RLcp13_13;
  PxF3[2] = RLcp13_211+RLcp13_213+RLcp13_214+RLcp13_217+RLcp13_22+RLcp13_23;
  PxF3[3] = q[4]+RLcp13_311+RLcp13_313+RLcp13_314+RLcp13_317;
  RxF3[1][1] = ROcp13_118;
  RxF3[1][2] = ROcp13_218;
  RxF3[1][3] = ROcp13_318;
  RxF3[2][1] = ROcp13_417;
  RxF3[2][2] = ROcp13_517;
  RxF3[2][3] = ROcp13_617;
  RxF3[3][1] = ROcp13_718;
  RxF3[3][2] = ROcp13_818;
  RxF3[3][3] = ROcp13_918;
  VxF3[1] = ORcp13_111+ORcp13_113+ORcp13_114+ORcp13_117+ORcp13_12+ORcp13_13+qd[2]*C1-qd[3]*S1;
  VxF3[2] = ORcp13_211+ORcp13_213+ORcp13_214+ORcp13_217+ORcp13_22+ORcp13_23+qd[2]*S1+qd[3]*C1;
  VxF3[3] = qd[4]+ORcp13_311+ORcp13_313+ORcp13_314+ORcp13_317;
  OMxF3[1] = OMcp13_116+qd[18]*ROcp13_417;
  OMxF3[2] = OMcp13_216+qd[18]*ROcp13_517;
  OMxF3[3] = OMcp13_316+qd[18]*ROcp13_617;
  AxF3[1] = -(qd[1]*(ORcp13_22+ORcp13_23+(2.0)*qd[2]*S1+(2.0)*qd[3]*C1)-qdd[2]*C1+qdd[3]*S1-OMcp13_213*ORcp13_314-OMcp13_216*
 ORcp13_317-OMcp13_27*ORcp13_311-OMcp13_27*ORcp13_313+OMcp13_313*ORcp13_214+OMcp13_316*ORcp13_217+OMcp13_37*ORcp13_211+
 OMcp13_37*ORcp13_213-OPcp13_213*RLcp13_314-OPcp13_216*RLcp13_317-OPcp13_27*RLcp13_311-OPcp13_27*RLcp13_313+OPcp13_313*
 RLcp13_214+OPcp13_316*RLcp13_217+OPcp13_37*RLcp13_211+OPcp13_37*RLcp13_213);
  AxF3[2] = qd[1]*(ORcp13_12+ORcp13_13+(2.0)*qd[2]*C1-(2.0)*qd[3]*S1)+qdd[2]*S1+qdd[3]*C1-OMcp13_113*ORcp13_314-OMcp13_116*
 ORcp13_317-OMcp13_17*ORcp13_311-OMcp13_17*ORcp13_313+OMcp13_313*ORcp13_114+OMcp13_316*ORcp13_117+OMcp13_37*ORcp13_111+
 OMcp13_37*ORcp13_113-OPcp13_113*RLcp13_314-OPcp13_116*RLcp13_317-OPcp13_17*RLcp13_311-OPcp13_17*RLcp13_313+OPcp13_313*
 RLcp13_114+OPcp13_316*RLcp13_117+OPcp13_37*RLcp13_111+OPcp13_37*RLcp13_113;
  AxF3[3] = qdd[4]+OMcp13_113*ORcp13_214+OMcp13_116*ORcp13_217+OMcp13_17*ORcp13_211+OMcp13_17*ORcp13_213-OMcp13_213*
 ORcp13_114-OMcp13_216*ORcp13_117-OMcp13_27*ORcp13_111-OMcp13_27*ORcp13_113+OPcp13_113*RLcp13_214+OPcp13_116*RLcp13_217+
 OPcp13_17*RLcp13_211+OPcp13_17*RLcp13_213-OPcp13_213*RLcp13_114-OPcp13_216*RLcp13_117-OPcp13_27*RLcp13_111-OPcp13_27*
 RLcp13_113;
  OMPxF3[1] = OPcp13_116+qd[18]*(OMcp13_216*ROcp13_617-OMcp13_316*ROcp13_517)+qdd[18]*ROcp13_417;
  OMPxF3[2] = OPcp13_216-qd[18]*(OMcp13_116*ROcp13_617-OMcp13_316*ROcp13_417)+qdd[18]*ROcp13_517;
  OMPxF3[3] = OPcp13_316+qd[18]*(OMcp13_116*ROcp13_517-OMcp13_216*ROcp13_417)+qdd[18]*ROcp13_617;
 
// Sensor Forces Computation 

  SWr3 = user_ExtForces(PxF3,RxF3,VxF3,OMxF3,AxF3,OMPxF3,s,tsim,3);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc114 = ROcp13_118*SWr3[1]+ROcp13_218*SWr3[2]+ROcp13_318*SWr3[3];
  xfrc214 = ROcp13_417*SWr3[1]+ROcp13_517*SWr3[2]+ROcp13_617*SWr3[3];
  xfrc314 = ROcp13_718*SWr3[1]+ROcp13_818*SWr3[2]+ROcp13_918*SWr3[3];
  frc[1][18] = s->frc[1][18]+xfrc114;
  frc[2][18] = s->frc[2][18]+xfrc214;
  frc[3][18] = s->frc[3][18]+xfrc314;
  xtrq114 = ROcp13_118*SWr3[4]+ROcp13_218*SWr3[5]+ROcp13_318*SWr3[6];
  xtrq214 = ROcp13_417*SWr3[4]+ROcp13_517*SWr3[5]+ROcp13_617*SWr3[6];
  xtrq314 = ROcp13_718*SWr3[4]+ROcp13_818*SWr3[5]+ROcp13_918*SWr3[6];
  trq[1][18] = s->trq[1][18]+xtrq114-xfrc214*SWr3[9]+xfrc314*SWr3[8];
  trq[2][18] = s->trq[2][18]+xtrq214+xfrc114*SWr3[9]-xfrc314*SWr3[7];
  trq[3][18] = s->trq[3][18]+xtrq314-xfrc114*SWr3[8]+xfrc214*SWr3[7];

// = = Block_0_0_1_4_0_1 = = 
 
// Sensor Kinematics 


  ROcp14_46 = -S1p5*C6;
  ROcp14_56 = C1p5*C6;
  ROcp14_76 = S1p5*S6;
  ROcp14_86 = -C1p5*S6;
  ROcp14_17 = -(ROcp14_76*S7-C1p5*C7);
  ROcp14_27 = -(ROcp14_86*S7-S1p5*C7);
  ROcp14_77 = ROcp14_76*C7+C1p5*S7;
  ROcp14_87 = ROcp14_86*C7+S1p5*S7;
  RLcp14_12 = q[2]*C1;
  RLcp14_22 = q[2]*S1;
  ORcp14_12 = -qd[1]*RLcp14_22;
  ORcp14_22 = qd[1]*RLcp14_12;
  RLcp14_13 = -q[3]*S1;
  RLcp14_23 = q[3]*C1;
  ORcp14_13 = -qd[1]*RLcp14_23;
  ORcp14_23 = qd[1]*RLcp14_13;
  OMcp14_35 = qd[1]+qd[5];
  OMcp14_16 = qd[6]*C1p5;
  OMcp14_26 = qd[6]*S1p5;
  OMcp14_17 = OMcp14_16+qd[7]*ROcp14_46;
  OMcp14_27 = OMcp14_26+qd[7]*ROcp14_56;
  OMcp14_37 = OMcp14_35+qd[7]*S6;
  OPcp14_17 = -(qd[6]*OMcp14_35*S1p5-qd[7]*(OMcp14_26*S6-OMcp14_35*ROcp14_56)-qdd[6]*C1p5-qdd[7]*ROcp14_46);
  OPcp14_27 = qd[6]*OMcp14_35*C1p5-qd[7]*(OMcp14_16*S6-OMcp14_35*ROcp14_46)+qdd[6]*S1p5+qdd[7]*ROcp14_56;
  OPcp14_37 = qdd[5]+qd[6]*qd[7]*C6+qdd[7]*S6;

// = = Block_0_0_1_4_0_3 = = 
 
// Sensor Kinematics 


  ROcp14_111 = ROcp14_17*C11-ROcp14_77*S11;
  ROcp14_211 = ROcp14_27*C11-ROcp14_87*S11;
  ROcp14_311 = -S11p7*C6;
  ROcp14_711 = ROcp14_17*S11+ROcp14_77*C11;
  ROcp14_811 = ROcp14_27*S11+ROcp14_87*C11;
  ROcp14_911 = C11p7*C6;
  RLcp14_111 = ROcp14_17*s->dpt[1][5]+ROcp14_77*s->dpt[3][5];
  RLcp14_211 = ROcp14_27*s->dpt[1][5]+ROcp14_87*s->dpt[3][5];
  RLcp14_311 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
  ORcp14_111 = OMcp14_27*RLcp14_311-OMcp14_37*RLcp14_211;
  ORcp14_211 = -(OMcp14_17*RLcp14_311-OMcp14_37*RLcp14_111);
  ORcp14_311 = OMcp14_17*RLcp14_211-OMcp14_27*RLcp14_111;

// = = Block_0_0_1_4_0_6 = = 
 
// Sensor Kinematics 


  ROcp14_419 = ROcp14_46*C19+ROcp14_711*S19;
  ROcp14_519 = ROcp14_56*C19+ROcp14_811*S19;
  ROcp14_619 = ROcp14_911*S19+C19*S6;
  ROcp14_719 = -(ROcp14_46*S19-ROcp14_711*C19);
  ROcp14_819 = -(ROcp14_56*S19-ROcp14_811*C19);
  ROcp14_919 = ROcp14_911*C19-S19*S6;
  ROcp14_420 = ROcp14_419*C20+ROcp14_719*S20;
  ROcp14_520 = ROcp14_519*C20+ROcp14_819*S20;
  ROcp14_620 = ROcp14_619*C20+ROcp14_919*S20;
  ROcp14_720 = -(ROcp14_419*S20-ROcp14_719*C20);
  ROcp14_820 = -(ROcp14_519*S20-ROcp14_819*C20);
  ROcp14_920 = -(ROcp14_619*S20-ROcp14_919*C20);
  ROcp14_121 = ROcp14_111*C21-ROcp14_720*S21;
  ROcp14_221 = ROcp14_211*C21-ROcp14_820*S21;
  ROcp14_321 = ROcp14_311*C21-ROcp14_920*S21;
  ROcp14_721 = ROcp14_111*S21+ROcp14_720*C21;
  ROcp14_821 = ROcp14_211*S21+ROcp14_820*C21;
  ROcp14_921 = ROcp14_311*S21+ROcp14_920*C21;
  ROcp14_122 = ROcp14_121*C22+ROcp14_420*S22;
  ROcp14_222 = ROcp14_221*C22+ROcp14_520*S22;
  ROcp14_322 = ROcp14_321*C22+ROcp14_620*S22;
  ROcp14_422 = -(ROcp14_121*S22-ROcp14_420*C22);
  ROcp14_522 = -(ROcp14_221*S22-ROcp14_520*C22);
  ROcp14_622 = -(ROcp14_321*S22-ROcp14_620*C22);
  ROcp14_423 = ROcp14_422*C23+ROcp14_721*S23;
  ROcp14_523 = ROcp14_522*C23+ROcp14_821*S23;
  ROcp14_623 = ROcp14_622*C23+ROcp14_921*S23;
  ROcp14_723 = -(ROcp14_422*S23-ROcp14_721*C23);
  ROcp14_823 = -(ROcp14_522*S23-ROcp14_821*C23);
  ROcp14_923 = -(ROcp14_622*S23-ROcp14_921*C23);
  ROcp14_124 = ROcp14_122*C24-ROcp14_723*S24;
  ROcp14_224 = ROcp14_222*C24-ROcp14_823*S24;
  ROcp14_324 = ROcp14_322*C24-ROcp14_923*S24;
  ROcp14_724 = ROcp14_122*S24+ROcp14_723*C24;
  ROcp14_824 = ROcp14_222*S24+ROcp14_823*C24;
  ROcp14_924 = ROcp14_322*S24+ROcp14_923*C24;
  RLcp14_119 = ROcp14_46*s->dpt[2][13]+ROcp14_711*s->dpt[3][13];
  RLcp14_219 = ROcp14_56*s->dpt[2][13]+ROcp14_811*s->dpt[3][13];
  RLcp14_319 = ROcp14_911*s->dpt[3][13]+s->dpt[2][13]*S6;
  OMcp14_119 = OMcp14_17+qd[19]*ROcp14_111;
  OMcp14_219 = OMcp14_27+qd[19]*ROcp14_211;
  OMcp14_319 = OMcp14_37+qd[19]*ROcp14_311;
  ORcp14_119 = OMcp14_27*RLcp14_319-OMcp14_37*RLcp14_219;
  ORcp14_219 = -(OMcp14_17*RLcp14_319-OMcp14_37*RLcp14_119);
  ORcp14_319 = OMcp14_17*RLcp14_219-OMcp14_27*RLcp14_119;
  OPcp14_119 = OPcp14_17+qd[19]*(OMcp14_27*ROcp14_311-OMcp14_37*ROcp14_211)+qdd[19]*ROcp14_111;
  OPcp14_219 = OPcp14_27-qd[19]*(OMcp14_17*ROcp14_311-OMcp14_37*ROcp14_111)+qdd[19]*ROcp14_211;
  OPcp14_319 = OPcp14_37+qd[19]*(OMcp14_17*ROcp14_211-OMcp14_27*ROcp14_111)+qdd[19]*ROcp14_311;
  RLcp14_120 = ROcp14_419*s->dpt[2][24]+ROcp14_719*s->dpt[3][24];
  RLcp14_220 = ROcp14_519*s->dpt[2][24]+ROcp14_819*s->dpt[3][24];
  RLcp14_320 = ROcp14_619*s->dpt[2][24]+ROcp14_919*s->dpt[3][24];
  OMcp14_120 = OMcp14_119+qd[20]*ROcp14_111;
  OMcp14_220 = OMcp14_219+qd[20]*ROcp14_211;
  OMcp14_320 = OMcp14_319+qd[20]*ROcp14_311;
  ORcp14_120 = OMcp14_219*RLcp14_320-OMcp14_319*RLcp14_220;
  ORcp14_220 = -(OMcp14_119*RLcp14_320-OMcp14_319*RLcp14_120);
  ORcp14_320 = OMcp14_119*RLcp14_220-OMcp14_219*RLcp14_120;
  OMcp14_121 = OMcp14_120+qd[21]*ROcp14_420;
  OMcp14_221 = OMcp14_220+qd[21]*ROcp14_520;
  OMcp14_321 = OMcp14_320+qd[21]*ROcp14_620;
  OMcp14_122 = OMcp14_121+qd[22]*ROcp14_721;
  OMcp14_222 = OMcp14_221+qd[22]*ROcp14_821;
  OMcp14_322 = OMcp14_321+qd[22]*ROcp14_921;
  OPcp14_122 = OPcp14_119+qd[20]*(OMcp14_219*ROcp14_311-OMcp14_319*ROcp14_211)+qd[21]*(OMcp14_220*ROcp14_620-OMcp14_320*
 ROcp14_520)+qd[22]*(OMcp14_221*ROcp14_921-OMcp14_321*ROcp14_821)+qdd[20]*ROcp14_111+qdd[21]*ROcp14_420+qdd[22]*ROcp14_721;
  OPcp14_222 = OPcp14_219-qd[20]*(OMcp14_119*ROcp14_311-OMcp14_319*ROcp14_111)-qd[21]*(OMcp14_120*ROcp14_620-OMcp14_320*
 ROcp14_420)-qd[22]*(OMcp14_121*ROcp14_921-OMcp14_321*ROcp14_721)+qdd[20]*ROcp14_211+qdd[21]*ROcp14_520+qdd[22]*ROcp14_821;
  OPcp14_322 = OPcp14_319+qd[20]*(OMcp14_119*ROcp14_211-OMcp14_219*ROcp14_111)+qd[21]*(OMcp14_120*ROcp14_520-OMcp14_220*
 ROcp14_420)+qd[22]*(OMcp14_121*ROcp14_821-OMcp14_221*ROcp14_721)+qdd[20]*ROcp14_311+qdd[21]*ROcp14_620+qdd[22]*ROcp14_921;
  RLcp14_123 = ROcp14_122*s->dpt[1][28]+ROcp14_422*s->dpt[2][28]+ROcp14_721*s->dpt[3][28];
  RLcp14_223 = ROcp14_222*s->dpt[1][28]+ROcp14_522*s->dpt[2][28]+ROcp14_821*s->dpt[3][28];
  RLcp14_323 = ROcp14_322*s->dpt[1][28]+ROcp14_622*s->dpt[2][28]+ROcp14_921*s->dpt[3][28];
  ORcp14_123 = OMcp14_222*RLcp14_323-OMcp14_322*RLcp14_223;
  ORcp14_223 = -(OMcp14_122*RLcp14_323-OMcp14_322*RLcp14_123);
  ORcp14_323 = OMcp14_122*RLcp14_223-OMcp14_222*RLcp14_123;
  PxF4[1] = RLcp14_111+RLcp14_119+RLcp14_12+RLcp14_120+RLcp14_123+RLcp14_13;
  PxF4[2] = RLcp14_211+RLcp14_219+RLcp14_22+RLcp14_220+RLcp14_223+RLcp14_23;
  PxF4[3] = q[4]+RLcp14_311+RLcp14_319+RLcp14_320+RLcp14_323;
  RxF4[1][1] = ROcp14_124;
  RxF4[1][2] = ROcp14_224;
  RxF4[1][3] = ROcp14_324;
  RxF4[2][1] = ROcp14_423;
  RxF4[2][2] = ROcp14_523;
  RxF4[2][3] = ROcp14_623;
  RxF4[3][1] = ROcp14_724;
  RxF4[3][2] = ROcp14_824;
  RxF4[3][3] = ROcp14_924;
  VxF4[1] = ORcp14_111+ORcp14_119+ORcp14_12+ORcp14_120+ORcp14_123+ORcp14_13+qd[2]*C1-qd[3]*S1;
  VxF4[2] = ORcp14_211+ORcp14_219+ORcp14_22+ORcp14_220+ORcp14_223+ORcp14_23+qd[2]*S1+qd[3]*C1;
  VxF4[3] = qd[4]+ORcp14_311+ORcp14_319+ORcp14_320+ORcp14_323;
  OMxF4[1] = OMcp14_122+qd[24]*ROcp14_423;
  OMxF4[2] = OMcp14_222+qd[24]*ROcp14_523;
  OMxF4[3] = OMcp14_322+qd[24]*ROcp14_623;
  AxF4[1] = -(qd[1]*(ORcp14_22+ORcp14_23+(2.0)*qd[2]*S1+(2.0)*qd[3]*C1)-qdd[2]*C1+qdd[3]*S1-OMcp14_219*ORcp14_320-OMcp14_222*
 ORcp14_323-OMcp14_27*ORcp14_311-OMcp14_27*ORcp14_319+OMcp14_319*ORcp14_220+OMcp14_322*ORcp14_223+OMcp14_37*ORcp14_211+
 OMcp14_37*ORcp14_219-OPcp14_219*RLcp14_320-OPcp14_222*RLcp14_323-OPcp14_27*RLcp14_311-OPcp14_27*RLcp14_319+OPcp14_319*
 RLcp14_220+OPcp14_322*RLcp14_223+OPcp14_37*RLcp14_211+OPcp14_37*RLcp14_219);
  AxF4[2] = qd[1]*(ORcp14_12+ORcp14_13+(2.0)*qd[2]*C1-(2.0)*qd[3]*S1)+qdd[2]*S1+qdd[3]*C1-OMcp14_119*ORcp14_320-OMcp14_122*
 ORcp14_323-OMcp14_17*ORcp14_311-OMcp14_17*ORcp14_319+OMcp14_319*ORcp14_120+OMcp14_322*ORcp14_123+OMcp14_37*ORcp14_111+
 OMcp14_37*ORcp14_119-OPcp14_119*RLcp14_320-OPcp14_122*RLcp14_323-OPcp14_17*RLcp14_311-OPcp14_17*RLcp14_319+OPcp14_319*
 RLcp14_120+OPcp14_322*RLcp14_123+OPcp14_37*RLcp14_111+OPcp14_37*RLcp14_119;
  AxF4[3] = qdd[4]+OMcp14_119*ORcp14_220+OMcp14_122*ORcp14_223+OMcp14_17*ORcp14_211+OMcp14_17*ORcp14_219-OMcp14_219*
 ORcp14_120-OMcp14_222*ORcp14_123-OMcp14_27*ORcp14_111-OMcp14_27*ORcp14_119+OPcp14_119*RLcp14_220+OPcp14_122*RLcp14_223+
 OPcp14_17*RLcp14_211+OPcp14_17*RLcp14_219-OPcp14_219*RLcp14_120-OPcp14_222*RLcp14_123-OPcp14_27*RLcp14_111-OPcp14_27*
 RLcp14_119;
  OMPxF4[1] = OPcp14_122+qd[24]*(OMcp14_222*ROcp14_623-OMcp14_322*ROcp14_523)+qdd[24]*ROcp14_423;
  OMPxF4[2] = OPcp14_222-qd[24]*(OMcp14_122*ROcp14_623-OMcp14_322*ROcp14_423)+qdd[24]*ROcp14_523;
  OMPxF4[3] = OPcp14_322+qd[24]*(OMcp14_122*ROcp14_523-OMcp14_222*ROcp14_423)+qdd[24]*ROcp14_623;
 
// Sensor Forces Computation 

  SWr4 = user_ExtForces(PxF4,RxF4,VxF4,OMxF4,AxF4,OMPxF4,s,tsim,4);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc115 = ROcp14_124*SWr4[1]+ROcp14_224*SWr4[2]+ROcp14_324*SWr4[3];
  xfrc215 = ROcp14_423*SWr4[1]+ROcp14_523*SWr4[2]+ROcp14_623*SWr4[3];
  xfrc315 = ROcp14_724*SWr4[1]+ROcp14_824*SWr4[2]+ROcp14_924*SWr4[3];
  frc[1][24] = s->frc[1][24]+xfrc115;
  frc[2][24] = s->frc[2][24]+xfrc215;
  frc[3][24] = s->frc[3][24]+xfrc315;
  xtrq115 = ROcp14_124*SWr4[4]+ROcp14_224*SWr4[5]+ROcp14_324*SWr4[6];
  xtrq215 = ROcp14_423*SWr4[4]+ROcp14_523*SWr4[5]+ROcp14_623*SWr4[6];
  xtrq315 = ROcp14_724*SWr4[4]+ROcp14_824*SWr4[5]+ROcp14_924*SWr4[6];
  trq[1][24] = s->trq[1][24]+xtrq115-xfrc215*SWr4[9]+xfrc315*SWr4[8];
  trq[2][24] = s->trq[2][24]+xtrq215+xfrc115*SWr4[9]-xfrc315*SWr4[7];
  trq[3][24] = s->trq[3][24]+xtrq315-xfrc115*SWr4[8]+xfrc215*SWr4[7];

// = = Block_0_0_1_4_1_0 = = 
 
// Symbolic Outputs  

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
 


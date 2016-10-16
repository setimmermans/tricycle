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
//	==> Generation Date : Fri Oct 14 16:15:43 2016
//
//	==> Project name : tricycle
//	==> using XML input file 
//
//	==> Number of joints : 30
//
//	==> Function : F 6 : Sensors Kinematical Informations (sens) 
//	==> Flops complexity : 3873
//
//	==> Generation Time :  0.060 seconds
//	==> Post-Processing :  0.070 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
#include "mbs_sensor.h"
 
void  mbs_sensor(MbsSensor *sens, 
              MbsData *s,
              int isens)
{ 
 
#include "mbs_sensor_tricycle.h" 
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

// = = Block_0_0_0_1_0_1 = = 
 
// Trigonometric Variables  

  C1p5 = C1*C5-S1*S5;
  S1p5 = C1*S5+S1*C5;

// = = Block_0_0_0_3_0_3 = = 
 
// Trigonometric Variables  

  S10p7 = C10*S7+S10*C7;
  C10p7 = C10*C7-S10*S7;

// = = Block_0_0_0_6_0_2 = = 
 
// Trigonometric Variables  

  S7p8 = C7*S8+S7*C8;
  C7p8 = C7*C8-S7*S8;
  S7p8p9 = C7p8*S9+S7p8*C9;
  C7p8p9 = C7p8*C9-S7p8*S9;

// ====== END Task 0 ====== 

// ===== BEGIN task 1 ===== 
 
switch(isens)
{
 
// 
break;
case 1:
 


// = = Block_1_0_0_1_0_1 = = 
 
// Sensor Kinematics 


    ROcp0_46 = -S1p5*C6;
    ROcp0_56 = C1p5*C6;
    ROcp0_76 = S1p5*S6;
    ROcp0_86 = -C1p5*S6;
    ROcp0_17 = -(ROcp0_76*S7-C1p5*C7);
    ROcp0_27 = -(ROcp0_86*S7-S1p5*C7);
    ROcp0_37 = -C6*S7;
    ROcp0_77 = ROcp0_76*C7+C1p5*S7;
    ROcp0_87 = ROcp0_86*C7+S1p5*S7;
    ROcp0_97 = C6*C7;
    RLcp0_12 = q[2]*C1;
    RLcp0_22 = q[2]*S1;
    ORcp0_12 = -RLcp0_22*qd[1];
    ORcp0_22 = RLcp0_12*qd[1];
    RLcp0_13 = -q[3]*S1;
    RLcp0_23 = q[3]*C1;
    POcp0_13 = RLcp0_12+RLcp0_13;
    POcp0_23 = RLcp0_22+RLcp0_23;
    JTcp0_13_1 = -(RLcp0_22+RLcp0_23);
    JTcp0_23_1 = RLcp0_12+RLcp0_13;
    ORcp0_13 = -RLcp0_23*qd[1];
    ORcp0_23 = RLcp0_13*qd[1];
    VIcp0_13 = ORcp0_12+ORcp0_13+qd[2]*C1-qd[3]*S1;
    VIcp0_23 = ORcp0_22+ORcp0_23+qd[2]*S1+qd[3]*C1;
    ACcp0_13 = -(ORcp0_23*qd[1]-qdd[2]*C1+qdd[3]*S1+(2.0)*qd[1]*qd[3]*C1+qd[1]*(ORcp0_22+(2.0)*qd[2]*S1));
    ACcp0_23 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp0_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp0_13-(2.0)*qd[3]*S1);
    OMcp0_35 = qd[1]+qd[5];
    OMcp0_16 = qd[6]*C1p5;
    OMcp0_26 = qd[6]*S1p5;
    OMcp0_17 = OMcp0_16+ROcp0_46*qd[7];
    OMcp0_27 = OMcp0_26+ROcp0_56*qd[7];
    OMcp0_37 = OMcp0_35+qd[7]*S6;
    OPcp0_17 = -(OMcp0_35*qd[6]*S1p5-ROcp0_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp0_26*S6-OMcp0_35*ROcp0_56));
    OPcp0_27 = OMcp0_35*qd[6]*C1p5+ROcp0_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp0_16*S6-OMcp0_35*ROcp0_46);
    OPcp0_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_1_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp0_13;
    sens->P[2] = POcp0_23;
    sens->P[3] = q[4];
    sens->R[1][1] = ROcp0_17;
    sens->R[1][2] = ROcp0_27;
    sens->R[1][3] = ROcp0_37;
    sens->R[2][1] = ROcp0_46;
    sens->R[2][2] = ROcp0_56;
    sens->R[2][3] = S6;
    sens->R[3][1] = ROcp0_77;
    sens->R[3][2] = ROcp0_87;
    sens->R[3][3] = ROcp0_97;
    sens->V[1] = VIcp0_13;
    sens->V[2] = VIcp0_23;
    sens->V[3] = qd[4];
    sens->OM[1] = OMcp0_17;
    sens->OM[2] = OMcp0_27;
    sens->OM[3] = OMcp0_37;
    sens->J[1][1] = JTcp0_13_1;
    sens->J[1][2] = C1;
    sens->J[1][3] = -S1;
    sens->J[2][1] = JTcp0_23_1;
    sens->J[2][2] = S1;
    sens->J[2][3] = C1;
    sens->J[3][4] = (1.0);
    sens->J[4][6] = C1p5;
    sens->J[4][7] = ROcp0_46;
    sens->J[5][6] = S1p5;
    sens->J[5][7] = ROcp0_56;
    sens->J[6][1] = (1.0);
    sens->J[6][5] = (1.0);
    sens->J[6][7] = S6;
    sens->A[1] = ACcp0_13;
    sens->A[2] = ACcp0_23;
    sens->A[3] = qdd[4];
    sens->OMP[1] = OPcp0_17;
    sens->OMP[2] = OPcp0_27;
    sens->OMP[3] = OPcp0_37;
 
// 
break;
case 2:
 


// = = Block_1_0_0_2_0_1 = = 
 
// Sensor Kinematics 


    ROcp1_46 = -S1p5*C6;
    ROcp1_56 = C1p5*C6;
    ROcp1_76 = S1p5*S6;
    ROcp1_86 = -C1p5*S6;
    ROcp1_17 = -(ROcp1_76*S7-C1p5*C7);
    ROcp1_27 = -(ROcp1_86*S7-S1p5*C7);
    ROcp1_37 = -C6*S7;
    ROcp1_77 = ROcp1_76*C7+C1p5*S7;
    ROcp1_87 = ROcp1_86*C7+S1p5*S7;
    ROcp1_97 = C6*C7;
    RLcp1_12 = q[2]*C1;
    RLcp1_22 = q[2]*S1;
    ORcp1_12 = -RLcp1_22*qd[1];
    ORcp1_22 = RLcp1_12*qd[1];
    RLcp1_13 = -q[3]*S1;
    RLcp1_23 = q[3]*C1;
    ORcp1_13 = -RLcp1_23*qd[1];
    ORcp1_23 = RLcp1_13*qd[1];
    OMcp1_35 = qd[1]+qd[5];
    OMcp1_16 = qd[6]*C1p5;
    OMcp1_26 = qd[6]*S1p5;
    OMcp1_17 = OMcp1_16+ROcp1_46*qd[7];
    OMcp1_27 = OMcp1_26+ROcp1_56*qd[7];
    OMcp1_37 = OMcp1_35+qd[7]*S6;
    OPcp1_17 = -(OMcp1_35*qd[6]*S1p5-ROcp1_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp1_26*S6-OMcp1_35*ROcp1_56));
    OPcp1_27 = OMcp1_35*qd[6]*C1p5+ROcp1_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp1_16*S6-OMcp1_35*ROcp1_46);
    OPcp1_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;
    RLcp1_146 = ROcp1_17*s->dpt[1][4]+ROcp1_77*s->dpt[3][4];
    RLcp1_246 = ROcp1_27*s->dpt[1][4]+ROcp1_87*s->dpt[3][4];
    RLcp1_346 = ROcp1_37*s->dpt[1][4]+ROcp1_97*s->dpt[3][4];
    POcp1_146 = RLcp1_12+RLcp1_13+RLcp1_146;
    POcp1_246 = RLcp1_22+RLcp1_23+RLcp1_246;
    POcp1_346 = RLcp1_346+q[4];
    JTcp1_146_1 = -(RLcp1_22+RLcp1_23+RLcp1_246);
    JTcp1_246_1 = RLcp1_12+RLcp1_13+RLcp1_146;
    JTcp1_146_6 = RLcp1_346*S1p5;
    JTcp1_246_6 = -RLcp1_346*C1p5;
    JTcp1_346_6 = -(RLcp1_146*S1p5-RLcp1_246*C1p5);
    JTcp1_146_7 = -(RLcp1_246*S6-RLcp1_346*ROcp1_56);
    JTcp1_246_7 = RLcp1_146*S6-RLcp1_346*ROcp1_46;
    JTcp1_346_7 = -(RLcp1_146*ROcp1_56-RLcp1_246*ROcp1_46);
    ORcp1_146 = OMcp1_27*RLcp1_346-OMcp1_37*RLcp1_246;
    ORcp1_246 = -(OMcp1_17*RLcp1_346-OMcp1_37*RLcp1_146);
    ORcp1_346 = OMcp1_17*RLcp1_246-OMcp1_27*RLcp1_146;
    VIcp1_146 = ORcp1_12+ORcp1_13+ORcp1_146+qd[2]*C1-qd[3]*S1;
    VIcp1_246 = ORcp1_22+ORcp1_23+ORcp1_246+qd[2]*S1+qd[3]*C1;
    VIcp1_346 = ORcp1_346+qd[4];
    ACcp1_146 = OMcp1_27*ORcp1_346-OMcp1_37*ORcp1_246+OPcp1_27*RLcp1_346-OPcp1_37*RLcp1_246-ORcp1_23*qd[1]+qdd[2]*C1-
 qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp1_22+(2.0)*qd[2]*S1);
    ACcp1_246 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp1_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp1_13-(2.0)*qd[3]*S1)-OMcp1_17*ORcp1_346+OMcp1_37*
 ORcp1_146-OPcp1_17*RLcp1_346+OPcp1_37*RLcp1_146;
    ACcp1_346 = qdd[4]+OMcp1_17*ORcp1_246-OMcp1_27*ORcp1_146+OPcp1_17*RLcp1_246-OPcp1_27*RLcp1_146;

// = = Block_1_0_0_2_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp1_146;
    sens->P[2] = POcp1_246;
    sens->P[3] = POcp1_346;
    sens->R[1][1] = ROcp1_17;
    sens->R[1][2] = ROcp1_27;
    sens->R[1][3] = ROcp1_37;
    sens->R[2][1] = ROcp1_46;
    sens->R[2][2] = ROcp1_56;
    sens->R[2][3] = S6;
    sens->R[3][1] = ROcp1_77;
    sens->R[3][2] = ROcp1_87;
    sens->R[3][3] = ROcp1_97;
    sens->V[1] = VIcp1_146;
    sens->V[2] = VIcp1_246;
    sens->V[3] = VIcp1_346;
    sens->OM[1] = OMcp1_17;
    sens->OM[2] = OMcp1_27;
    sens->OM[3] = OMcp1_37;
    sens->J[1][1] = JTcp1_146_1;
    sens->J[1][2] = C1;
    sens->J[1][3] = -S1;
    sens->J[1][5] = -RLcp1_246;
    sens->J[1][6] = JTcp1_146_6;
    sens->J[1][7] = JTcp1_146_7;
    sens->J[2][1] = JTcp1_246_1;
    sens->J[2][2] = S1;
    sens->J[2][3] = C1;
    sens->J[2][5] = RLcp1_146;
    sens->J[2][6] = JTcp1_246_6;
    sens->J[2][7] = JTcp1_246_7;
    sens->J[3][4] = (1.0);
    sens->J[3][6] = JTcp1_346_6;
    sens->J[3][7] = JTcp1_346_7;
    sens->J[4][6] = C1p5;
    sens->J[4][7] = ROcp1_46;
    sens->J[5][6] = S1p5;
    sens->J[5][7] = ROcp1_56;
    sens->J[6][1] = (1.0);
    sens->J[6][5] = (1.0);
    sens->J[6][7] = S6;
    sens->A[1] = ACcp1_146;
    sens->A[2] = ACcp1_246;
    sens->A[3] = ACcp1_346;
    sens->OMP[1] = OPcp1_17;
    sens->OMP[2] = OPcp1_27;
    sens->OMP[3] = OPcp1_37;
 
// 
break;
case 3:
 


// = = Block_1_0_0_3_0_1 = = 
 
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
    ORcp2_12 = -RLcp2_22*qd[1];
    ORcp2_22 = RLcp2_12*qd[1];
    RLcp2_13 = -q[3]*S1;
    RLcp2_23 = q[3]*C1;
    ORcp2_13 = -RLcp2_23*qd[1];
    ORcp2_23 = RLcp2_13*qd[1];
    OMcp2_35 = qd[1]+qd[5];
    OMcp2_16 = qd[6]*C1p5;
    OMcp2_26 = qd[6]*S1p5;
    OMcp2_17 = OMcp2_16+ROcp2_46*qd[7];
    OMcp2_27 = OMcp2_26+ROcp2_56*qd[7];
    OMcp2_37 = OMcp2_35+qd[7]*S6;
    OPcp2_17 = -(OMcp2_35*qd[6]*S1p5-ROcp2_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp2_26*S6-OMcp2_35*ROcp2_56));
    OPcp2_27 = OMcp2_35*qd[6]*C1p5+ROcp2_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp2_16*S6-OMcp2_35*ROcp2_46);
    OPcp2_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_3_0_3 = = 
 
// Sensor Kinematics 


    ROcp2_110 = ROcp2_17*C10-ROcp2_77*S10;
    ROcp2_210 = ROcp2_27*C10-ROcp2_87*S10;
    ROcp2_310 = -S10p7*C6;
    ROcp2_710 = ROcp2_17*S10+ROcp2_77*C10;
    ROcp2_810 = ROcp2_27*S10+ROcp2_87*C10;
    ROcp2_910 = C10p7*C6;
    RLcp2_110 = ROcp2_17*s->dpt[1][5]+ROcp2_77*s->dpt[3][5];
    RLcp2_210 = ROcp2_27*s->dpt[1][5]+ROcp2_87*s->dpt[3][5];
    RLcp2_310 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp2_110 = OMcp2_27*RLcp2_310-OMcp2_37*RLcp2_210;
    ORcp2_210 = -(OMcp2_17*RLcp2_310-OMcp2_37*RLcp2_110);
    ORcp2_310 = OMcp2_17*RLcp2_210-OMcp2_27*RLcp2_110;

// = = Block_1_0_0_3_0_5 = = 
 
// Sensor Kinematics 


    ROcp2_412 = ROcp2_46*C12+ROcp2_710*S12;
    ROcp2_512 = ROcp2_56*C12+ROcp2_810*S12;
    ROcp2_612 = ROcp2_910*S12+C12*S6;
    ROcp2_712 = -(ROcp2_46*S12-ROcp2_710*C12);
    ROcp2_812 = -(ROcp2_56*S12-ROcp2_810*C12);
    ROcp2_912 = ROcp2_910*C12-S12*S6;
    ROcp2_413 = ROcp2_412*C13+ROcp2_712*S13;
    ROcp2_513 = ROcp2_512*C13+ROcp2_812*S13;
    ROcp2_613 = ROcp2_612*C13+ROcp2_912*S13;
    ROcp2_713 = -(ROcp2_412*S13-ROcp2_712*C13);
    ROcp2_813 = -(ROcp2_512*S13-ROcp2_812*C13);
    ROcp2_913 = -(ROcp2_612*S13-ROcp2_912*C13);
    ROcp2_114 = ROcp2_110*C14-ROcp2_713*S14;
    ROcp2_214 = ROcp2_210*C14-ROcp2_813*S14;
    ROcp2_314 = ROcp2_310*C14-ROcp2_913*S14;
    ROcp2_714 = ROcp2_110*S14+ROcp2_713*C14;
    ROcp2_814 = ROcp2_210*S14+ROcp2_813*C14;
    ROcp2_914 = ROcp2_310*S14+ROcp2_913*C14;
    ROcp2_115 = ROcp2_114*C15+ROcp2_413*S15;
    ROcp2_215 = ROcp2_214*C15+ROcp2_513*S15;
    ROcp2_315 = ROcp2_314*C15+ROcp2_613*S15;
    ROcp2_415 = -(ROcp2_114*S15-ROcp2_413*C15);
    ROcp2_515 = -(ROcp2_214*S15-ROcp2_513*C15);
    ROcp2_615 = -(ROcp2_314*S15-ROcp2_613*C15);
    ROcp2_416 = ROcp2_415*C16+ROcp2_714*S16;
    ROcp2_516 = ROcp2_515*C16+ROcp2_814*S16;
    ROcp2_616 = ROcp2_615*C16+ROcp2_914*S16;
    ROcp2_716 = -(ROcp2_415*S16-ROcp2_714*C16);
    ROcp2_816 = -(ROcp2_515*S16-ROcp2_814*C16);
    ROcp2_916 = -(ROcp2_615*S16-ROcp2_914*C16);
    ROcp2_117 = ROcp2_115*C17-ROcp2_716*S17;
    ROcp2_217 = ROcp2_215*C17-ROcp2_816*S17;
    ROcp2_317 = ROcp2_315*C17-ROcp2_916*S17;
    ROcp2_717 = ROcp2_115*S17+ROcp2_716*C17;
    ROcp2_817 = ROcp2_215*S17+ROcp2_816*C17;
    ROcp2_917 = ROcp2_315*S17+ROcp2_916*C17;
    RLcp2_112 = ROcp2_46*s->dpt[2][11]+ROcp2_710*s->dpt[3][11];
    RLcp2_212 = ROcp2_56*s->dpt[2][11]+ROcp2_810*s->dpt[3][11];
    RLcp2_312 = ROcp2_910*s->dpt[3][11]+s->dpt[2][11]*S6;
    OMcp2_112 = OMcp2_17+ROcp2_110*qd[12];
    OMcp2_212 = OMcp2_27+ROcp2_210*qd[12];
    OMcp2_312 = OMcp2_37+ROcp2_310*qd[12];
    ORcp2_112 = OMcp2_27*RLcp2_312-OMcp2_37*RLcp2_212;
    ORcp2_212 = -(OMcp2_17*RLcp2_312-OMcp2_37*RLcp2_112);
    ORcp2_312 = OMcp2_17*RLcp2_212-OMcp2_27*RLcp2_112;
    OPcp2_112 = OPcp2_17+ROcp2_110*qdd[12]+qd[12]*(OMcp2_27*ROcp2_310-OMcp2_37*ROcp2_210);
    OPcp2_212 = OPcp2_27+ROcp2_210*qdd[12]-qd[12]*(OMcp2_17*ROcp2_310-OMcp2_37*ROcp2_110);
    OPcp2_312 = OPcp2_37+ROcp2_310*qdd[12]+qd[12]*(OMcp2_17*ROcp2_210-OMcp2_27*ROcp2_110);
    RLcp2_113 = ROcp2_412*s->dpt[2][16];
    RLcp2_213 = ROcp2_512*s->dpt[2][16];
    RLcp2_313 = ROcp2_612*s->dpt[2][16];
    OMcp2_113 = OMcp2_112+ROcp2_110*qd[13];
    OMcp2_213 = OMcp2_212+ROcp2_210*qd[13];
    OMcp2_313 = OMcp2_312+ROcp2_310*qd[13];
    ORcp2_113 = OMcp2_212*RLcp2_313-OMcp2_312*RLcp2_213;
    ORcp2_213 = -(OMcp2_112*RLcp2_313-OMcp2_312*RLcp2_113);
    ORcp2_313 = OMcp2_112*RLcp2_213-OMcp2_212*RLcp2_113;
    OMcp2_114 = OMcp2_113+ROcp2_413*qd[14];
    OMcp2_214 = OMcp2_213+ROcp2_513*qd[14];
    OMcp2_314 = OMcp2_313+ROcp2_613*qd[14];
    OMcp2_115 = OMcp2_114+ROcp2_714*qd[15];
    OMcp2_215 = OMcp2_214+ROcp2_814*qd[15];
    OMcp2_315 = OMcp2_314+ROcp2_914*qd[15];
    OPcp2_115 = OPcp2_112+ROcp2_110*qdd[13]+ROcp2_413*qdd[14]+ROcp2_714*qdd[15]+qd[13]*(OMcp2_212*ROcp2_310-OMcp2_312*
 ROcp2_210)+qd[14]*(OMcp2_213*ROcp2_613-OMcp2_313*ROcp2_513)+qd[15]*(OMcp2_214*ROcp2_914-OMcp2_314*ROcp2_814);
    OPcp2_215 = OPcp2_212+ROcp2_210*qdd[13]+ROcp2_513*qdd[14]+ROcp2_814*qdd[15]-qd[13]*(OMcp2_112*ROcp2_310-OMcp2_312*
 ROcp2_110)-qd[14]*(OMcp2_113*ROcp2_613-OMcp2_313*ROcp2_413)-qd[15]*(OMcp2_114*ROcp2_914-OMcp2_314*ROcp2_714);
    OPcp2_315 = OPcp2_312+ROcp2_310*qdd[13]+ROcp2_613*qdd[14]+ROcp2_914*qdd[15]+qd[13]*(OMcp2_112*ROcp2_210-OMcp2_212*
 ROcp2_110)+qd[14]*(OMcp2_113*ROcp2_513-OMcp2_213*ROcp2_413)+qd[15]*(OMcp2_114*ROcp2_814-OMcp2_214*ROcp2_714);
    RLcp2_116 = ROcp2_115*s->dpt[1][19]+ROcp2_415*s->dpt[2][19]+ROcp2_714*s->dpt[3][19];
    RLcp2_216 = ROcp2_215*s->dpt[1][19]+ROcp2_515*s->dpt[2][19]+ROcp2_814*s->dpt[3][19];
    RLcp2_316 = ROcp2_315*s->dpt[1][19]+ROcp2_615*s->dpt[2][19]+ROcp2_914*s->dpt[3][19];
    POcp2_116 = RLcp2_110+RLcp2_112+RLcp2_113+RLcp2_116+RLcp2_12+RLcp2_13;
    POcp2_216 = RLcp2_210+RLcp2_212+RLcp2_213+RLcp2_216+RLcp2_22+RLcp2_23;
    POcp2_316 = RLcp2_310+RLcp2_312+RLcp2_313+RLcp2_316+q[4];
    JTcp2_116_1 = -(RLcp2_210+RLcp2_212+RLcp2_213+RLcp2_216+RLcp2_22+RLcp2_23);
    JTcp2_216_1 = RLcp2_110+RLcp2_112+RLcp2_113+RLcp2_116+RLcp2_12+RLcp2_13;
    JTcp2_116_5 = -(RLcp2_210+RLcp2_212+RLcp2_213+RLcp2_216);
    JTcp2_216_5 = RLcp2_110+RLcp2_112+RLcp2_113+RLcp2_116;
    JTcp2_116_6 = S1p5*(RLcp2_310+RLcp2_312+RLcp2_313+RLcp2_316);
    JTcp2_216_6 = -C1p5*(RLcp2_310+RLcp2_312+RLcp2_313+RLcp2_316);
    JTcp2_316_6 = C1p5*(RLcp2_210+RLcp2_212+RLcp2_213+RLcp2_216)-S1p5*(RLcp2_110+RLcp2_112)-S1p5*(RLcp2_113+RLcp2_116);
    JTcp2_116_7 = ROcp2_56*(RLcp2_310+RLcp2_312+RLcp2_313+RLcp2_316)-S6*(RLcp2_210+RLcp2_212)-S6*(RLcp2_213+RLcp2_216);
    JTcp2_216_7 = -(ROcp2_46*(RLcp2_310+RLcp2_312+RLcp2_313+RLcp2_316)-S6*(RLcp2_110+RLcp2_112)-S6*(RLcp2_113+RLcp2_116));
    JTcp2_316_7 = ROcp2_46*(RLcp2_210+RLcp2_212+RLcp2_213+RLcp2_216)-ROcp2_56*(RLcp2_110+RLcp2_112)-ROcp2_56*(RLcp2_113+
 RLcp2_116);
    JTcp2_116_8 = ROcp2_56*(RLcp2_312+RLcp2_313)-S6*(RLcp2_212+RLcp2_213)-RLcp2_216*S6+RLcp2_316*ROcp2_56;
    JTcp2_216_8 = RLcp2_116*S6-RLcp2_316*ROcp2_46-ROcp2_46*(RLcp2_312+RLcp2_313)+S6*(RLcp2_112+RLcp2_113);
    JTcp2_316_8 = ROcp2_46*(RLcp2_212+RLcp2_213)-ROcp2_56*(RLcp2_112+RLcp2_113)-RLcp2_116*ROcp2_56+RLcp2_216*ROcp2_46;
    JTcp2_116_9 = ROcp2_210*(RLcp2_313+RLcp2_316)-ROcp2_310*(RLcp2_213+RLcp2_216);
    JTcp2_216_9 = -(ROcp2_110*(RLcp2_313+RLcp2_316)-ROcp2_310*(RLcp2_113+RLcp2_116));
    JTcp2_316_9 = ROcp2_110*(RLcp2_213+RLcp2_216)-ROcp2_210*(RLcp2_113+RLcp2_116);
    JTcp2_116_10 = -(RLcp2_216*ROcp2_310-RLcp2_316*ROcp2_210);
    JTcp2_216_10 = RLcp2_116*ROcp2_310-RLcp2_316*ROcp2_110;
    JTcp2_316_10 = -(RLcp2_116*ROcp2_210-RLcp2_216*ROcp2_110);
    JTcp2_116_11 = -(RLcp2_216*ROcp2_613-RLcp2_316*ROcp2_513);
    JTcp2_216_11 = RLcp2_116*ROcp2_613-RLcp2_316*ROcp2_413;
    JTcp2_316_11 = -(RLcp2_116*ROcp2_513-RLcp2_216*ROcp2_413);
    JTcp2_116_12 = -(RLcp2_216*ROcp2_914-RLcp2_316*ROcp2_814);
    JTcp2_216_12 = RLcp2_116*ROcp2_914-RLcp2_316*ROcp2_714;
    JTcp2_316_12 = -(RLcp2_116*ROcp2_814-RLcp2_216*ROcp2_714);
    ORcp2_116 = OMcp2_215*RLcp2_316-OMcp2_315*RLcp2_216;
    ORcp2_216 = -(OMcp2_115*RLcp2_316-OMcp2_315*RLcp2_116);
    ORcp2_316 = OMcp2_115*RLcp2_216-OMcp2_215*RLcp2_116;
    VIcp2_116 = ORcp2_110+ORcp2_112+ORcp2_113+ORcp2_116+ORcp2_12+ORcp2_13+qd[2]*C1-qd[3]*S1;
    VIcp2_216 = ORcp2_210+ORcp2_212+ORcp2_213+ORcp2_216+ORcp2_22+ORcp2_23+qd[2]*S1+qd[3]*C1;
    VIcp2_316 = ORcp2_310+ORcp2_312+ORcp2_313+ORcp2_316+qd[4];
    ACcp2_116 = OMcp2_212*ORcp2_313+OMcp2_215*ORcp2_316+OMcp2_27*(ORcp2_310+ORcp2_312)-OMcp2_312*ORcp2_213-OMcp2_315*
 ORcp2_216-OMcp2_37*ORcp2_210-OMcp2_37*ORcp2_212+OPcp2_212*RLcp2_313+OPcp2_215*RLcp2_316+OPcp2_27*RLcp2_310+OPcp2_27*
 RLcp2_312-OPcp2_312*RLcp2_213-OPcp2_315*RLcp2_216-OPcp2_37*RLcp2_210-OPcp2_37*RLcp2_212-ORcp2_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*
 qd[1]*qd[3]*C1-qd[1]*(ORcp2_22+(2.0)*qd[2]*S1);
    ACcp2_216 = -(OMcp2_112*ORcp2_313+OMcp2_115*ORcp2_316+OMcp2_17*(ORcp2_310+ORcp2_312)-OMcp2_312*ORcp2_113-OMcp2_315*
 ORcp2_116-OMcp2_37*ORcp2_110-OMcp2_37*ORcp2_112+OPcp2_112*RLcp2_313+OPcp2_115*RLcp2_316+OPcp2_17*RLcp2_310+OPcp2_17*
 RLcp2_312-OPcp2_312*RLcp2_113-OPcp2_315*RLcp2_116-OPcp2_37*RLcp2_110-OPcp2_37*RLcp2_112-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp2_12+(2.0)
 *qd[2]*C1)-qd[1]*(ORcp2_13-(2.0)*qd[3]*S1));
    ACcp2_316 = qdd[4]+OMcp2_112*ORcp2_213+OMcp2_115*ORcp2_216+OMcp2_17*ORcp2_210+OMcp2_17*ORcp2_212-OMcp2_212*ORcp2_113-
 OMcp2_215*ORcp2_116-OMcp2_27*ORcp2_110-OMcp2_27*ORcp2_112+OPcp2_112*RLcp2_213+OPcp2_115*RLcp2_216+OPcp2_17*RLcp2_210+
 OPcp2_17*RLcp2_212-OPcp2_212*RLcp2_113-OPcp2_215*RLcp2_116-OPcp2_27*RLcp2_110-OPcp2_27*RLcp2_112;
    OMcp2_117 = OMcp2_115+ROcp2_416*qd[17];
    OMcp2_217 = OMcp2_215+ROcp2_516*qd[17];
    OMcp2_317 = OMcp2_315+ROcp2_616*qd[17];
    OPcp2_117 = OPcp2_115+ROcp2_416*qdd[17]+qd[17]*(OMcp2_215*ROcp2_616-OMcp2_315*ROcp2_516);
    OPcp2_217 = OPcp2_215+ROcp2_516*qdd[17]-qd[17]*(OMcp2_115*ROcp2_616-OMcp2_315*ROcp2_416);
    OPcp2_317 = OPcp2_315+ROcp2_616*qdd[17]+qd[17]*(OMcp2_115*ROcp2_516-OMcp2_215*ROcp2_416);

// = = Block_1_0_0_3_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp2_116;
    sens->P[2] = POcp2_216;
    sens->P[3] = POcp2_316;
    sens->R[1][1] = ROcp2_117;
    sens->R[1][2] = ROcp2_217;
    sens->R[1][3] = ROcp2_317;
    sens->R[2][1] = ROcp2_416;
    sens->R[2][2] = ROcp2_516;
    sens->R[2][3] = ROcp2_616;
    sens->R[3][1] = ROcp2_717;
    sens->R[3][2] = ROcp2_817;
    sens->R[3][3] = ROcp2_917;
    sens->V[1] = VIcp2_116;
    sens->V[2] = VIcp2_216;
    sens->V[3] = VIcp2_316;
    sens->OM[1] = OMcp2_117;
    sens->OM[2] = OMcp2_217;
    sens->OM[3] = OMcp2_317;
    sens->J[1][1] = JTcp2_116_1;
    sens->J[1][2] = C1;
    sens->J[1][3] = -S1;
    sens->J[1][5] = JTcp2_116_5;
    sens->J[1][6] = JTcp2_116_6;
    sens->J[1][7] = JTcp2_116_7;
    sens->J[1][10] = JTcp2_116_8;
    sens->J[1][12] = JTcp2_116_9;
    sens->J[1][13] = JTcp2_116_10;
    sens->J[1][14] = JTcp2_116_11;
    sens->J[1][15] = JTcp2_116_12;
    sens->J[2][1] = JTcp2_216_1;
    sens->J[2][2] = S1;
    sens->J[2][3] = C1;
    sens->J[2][5] = JTcp2_216_5;
    sens->J[2][6] = JTcp2_216_6;
    sens->J[2][7] = JTcp2_216_7;
    sens->J[2][10] = JTcp2_216_8;
    sens->J[2][12] = JTcp2_216_9;
    sens->J[2][13] = JTcp2_216_10;
    sens->J[2][14] = JTcp2_216_11;
    sens->J[2][15] = JTcp2_216_12;
    sens->J[3][4] = (1.0);
    sens->J[3][6] = JTcp2_316_6;
    sens->J[3][7] = JTcp2_316_7;
    sens->J[3][10] = JTcp2_316_8;
    sens->J[3][12] = JTcp2_316_9;
    sens->J[3][13] = JTcp2_316_10;
    sens->J[3][14] = JTcp2_316_11;
    sens->J[3][15] = JTcp2_316_12;
    sens->J[4][6] = C1p5;
    sens->J[4][7] = ROcp2_46;
    sens->J[4][10] = ROcp2_46;
    sens->J[4][12] = ROcp2_110;
    sens->J[4][13] = ROcp2_110;
    sens->J[4][14] = ROcp2_413;
    sens->J[4][15] = ROcp2_714;
    sens->J[4][16] = ROcp2_115;
    sens->J[4][17] = ROcp2_416;
    sens->J[5][6] = S1p5;
    sens->J[5][7] = ROcp2_56;
    sens->J[5][10] = ROcp2_56;
    sens->J[5][12] = ROcp2_210;
    sens->J[5][13] = ROcp2_210;
    sens->J[5][14] = ROcp2_513;
    sens->J[5][15] = ROcp2_814;
    sens->J[5][16] = ROcp2_215;
    sens->J[5][17] = ROcp2_516;
    sens->J[6][1] = (1.0);
    sens->J[6][5] = (1.0);
    sens->J[6][7] = S6;
    sens->J[6][10] = S6;
    sens->J[6][12] = ROcp2_310;
    sens->J[6][13] = ROcp2_310;
    sens->J[6][14] = ROcp2_613;
    sens->J[6][15] = ROcp2_914;
    sens->J[6][16] = ROcp2_315;
    sens->J[6][17] = ROcp2_616;
    sens->A[1] = ACcp2_116;
    sens->A[2] = ACcp2_216;
    sens->A[3] = ACcp2_316;
    sens->OMP[1] = OPcp2_117;
    sens->OMP[2] = OPcp2_217;
    sens->OMP[3] = OPcp2_317;
 
// 
break;
case 4:
 


// = = Block_1_0_0_4_0_1 = = 
 
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
    ORcp3_12 = -RLcp3_22*qd[1];
    ORcp3_22 = RLcp3_12*qd[1];
    RLcp3_13 = -q[3]*S1;
    RLcp3_23 = q[3]*C1;
    ORcp3_13 = -RLcp3_23*qd[1];
    ORcp3_23 = RLcp3_13*qd[1];
    OMcp3_35 = qd[1]+qd[5];
    OMcp3_16 = qd[6]*C1p5;
    OMcp3_26 = qd[6]*S1p5;
    OMcp3_17 = OMcp3_16+ROcp3_46*qd[7];
    OMcp3_27 = OMcp3_26+ROcp3_56*qd[7];
    OMcp3_37 = OMcp3_35+qd[7]*S6;
    OPcp3_17 = -(OMcp3_35*qd[6]*S1p5-ROcp3_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp3_26*S6-OMcp3_35*ROcp3_56));
    OPcp3_27 = OMcp3_35*qd[6]*C1p5+ROcp3_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp3_16*S6-OMcp3_35*ROcp3_46);
    OPcp3_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_4_0_3 = = 
 
// Sensor Kinematics 


    ROcp3_110 = ROcp3_17*C10-ROcp3_77*S10;
    ROcp3_210 = ROcp3_27*C10-ROcp3_87*S10;
    ROcp3_310 = -S10p7*C6;
    ROcp3_710 = ROcp3_17*S10+ROcp3_77*C10;
    ROcp3_810 = ROcp3_27*S10+ROcp3_87*C10;
    ROcp3_910 = C10p7*C6;
    RLcp3_110 = ROcp3_17*s->dpt[1][5]+ROcp3_77*s->dpt[3][5];
    RLcp3_210 = ROcp3_27*s->dpt[1][5]+ROcp3_87*s->dpt[3][5];
    RLcp3_310 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp3_110 = OMcp3_27*RLcp3_310-OMcp3_37*RLcp3_210;
    ORcp3_210 = -(OMcp3_17*RLcp3_310-OMcp3_37*RLcp3_110);
    ORcp3_310 = OMcp3_17*RLcp3_210-OMcp3_27*RLcp3_110;

// = = Block_1_0_0_4_0_6 = = 
 
// Sensor Kinematics 


    ROcp3_418 = ROcp3_46*C18+ROcp3_710*S18;
    ROcp3_518 = ROcp3_56*C18+ROcp3_810*S18;
    ROcp3_618 = ROcp3_910*S18+C18*S6;
    ROcp3_718 = -(ROcp3_46*S18-ROcp3_710*C18);
    ROcp3_818 = -(ROcp3_56*S18-ROcp3_810*C18);
    ROcp3_918 = ROcp3_910*C18-S18*S6;
    ROcp3_419 = ROcp3_418*C19+ROcp3_718*S19;
    ROcp3_519 = ROcp3_518*C19+ROcp3_818*S19;
    ROcp3_619 = ROcp3_618*C19+ROcp3_918*S19;
    ROcp3_719 = -(ROcp3_418*S19-ROcp3_718*C19);
    ROcp3_819 = -(ROcp3_518*S19-ROcp3_818*C19);
    ROcp3_919 = -(ROcp3_618*S19-ROcp3_918*C19);
    ROcp3_120 = ROcp3_110*C20-ROcp3_719*S20;
    ROcp3_220 = ROcp3_210*C20-ROcp3_819*S20;
    ROcp3_320 = ROcp3_310*C20-ROcp3_919*S20;
    ROcp3_720 = ROcp3_110*S20+ROcp3_719*C20;
    ROcp3_820 = ROcp3_210*S20+ROcp3_819*C20;
    ROcp3_920 = ROcp3_310*S20+ROcp3_919*C20;
    ROcp3_121 = ROcp3_120*C21+ROcp3_419*S21;
    ROcp3_221 = ROcp3_220*C21+ROcp3_519*S21;
    ROcp3_321 = ROcp3_320*C21+ROcp3_619*S21;
    ROcp3_421 = -(ROcp3_120*S21-ROcp3_419*C21);
    ROcp3_521 = -(ROcp3_220*S21-ROcp3_519*C21);
    ROcp3_621 = -(ROcp3_320*S21-ROcp3_619*C21);
    RLcp3_118 = ROcp3_46*s->dpt[2][12]+ROcp3_710*s->dpt[3][12];
    RLcp3_218 = ROcp3_56*s->dpt[2][12]+ROcp3_810*s->dpt[3][12];
    RLcp3_318 = ROcp3_910*s->dpt[3][12]+s->dpt[2][12]*S6;
    OMcp3_118 = OMcp3_17+ROcp3_110*qd[18];
    OMcp3_218 = OMcp3_27+ROcp3_210*qd[18];
    OMcp3_318 = OMcp3_37+ROcp3_310*qd[18];
    ORcp3_118 = OMcp3_27*RLcp3_318-OMcp3_37*RLcp3_218;
    ORcp3_218 = -(OMcp3_17*RLcp3_318-OMcp3_37*RLcp3_118);
    ORcp3_318 = OMcp3_17*RLcp3_218-OMcp3_27*RLcp3_118;
    OPcp3_118 = OPcp3_17+ROcp3_110*qdd[18]+qd[18]*(OMcp3_27*ROcp3_310-OMcp3_37*ROcp3_210);
    OPcp3_218 = OPcp3_27+ROcp3_210*qdd[18]-qd[18]*(OMcp3_17*ROcp3_310-OMcp3_37*ROcp3_110);
    OPcp3_318 = OPcp3_37+ROcp3_310*qdd[18]+qd[18]*(OMcp3_17*ROcp3_210-OMcp3_27*ROcp3_110);
    RLcp3_119 = ROcp3_418*s->dpt[2][22];
    RLcp3_219 = ROcp3_518*s->dpt[2][22];
    RLcp3_319 = ROcp3_618*s->dpt[2][22];
    OMcp3_119 = OMcp3_118+ROcp3_110*qd[19];
    OMcp3_219 = OMcp3_218+ROcp3_210*qd[19];
    OMcp3_319 = OMcp3_318+ROcp3_310*qd[19];
    ORcp3_119 = OMcp3_218*RLcp3_319-OMcp3_318*RLcp3_219;
    ORcp3_219 = -(OMcp3_118*RLcp3_319-OMcp3_318*RLcp3_119);
    ORcp3_319 = OMcp3_118*RLcp3_219-OMcp3_218*RLcp3_119;
    OMcp3_120 = OMcp3_119+ROcp3_419*qd[20];
    OMcp3_220 = OMcp3_219+ROcp3_519*qd[20];
    OMcp3_320 = OMcp3_319+ROcp3_619*qd[20];
    OMcp3_121 = OMcp3_120+ROcp3_720*qd[21];
    OMcp3_221 = OMcp3_220+ROcp3_820*qd[21];
    OMcp3_321 = OMcp3_320+ROcp3_920*qd[21];
    OPcp3_121 = OPcp3_118+ROcp3_110*qdd[19]+ROcp3_419*qdd[20]+ROcp3_720*qdd[21]+qd[19]*(OMcp3_218*ROcp3_310-OMcp3_318*
 ROcp3_210)+qd[20]*(OMcp3_219*ROcp3_619-OMcp3_319*ROcp3_519)+qd[21]*(OMcp3_220*ROcp3_920-OMcp3_320*ROcp3_820);
    OPcp3_221 = OPcp3_218+ROcp3_210*qdd[19]+ROcp3_519*qdd[20]+ROcp3_820*qdd[21]-qd[19]*(OMcp3_118*ROcp3_310-OMcp3_318*
 ROcp3_110)-qd[20]*(OMcp3_119*ROcp3_619-OMcp3_319*ROcp3_419)-qd[21]*(OMcp3_120*ROcp3_920-OMcp3_320*ROcp3_720);
    OPcp3_321 = OPcp3_318+ROcp3_310*qdd[19]+ROcp3_619*qdd[20]+ROcp3_920*qdd[21]+qd[19]*(OMcp3_118*ROcp3_210-OMcp3_218*
 ROcp3_110)+qd[20]*(OMcp3_119*ROcp3_519-OMcp3_219*ROcp3_419)+qd[21]*(OMcp3_120*ROcp3_820-OMcp3_220*ROcp3_720);
    RLcp3_148 = ROcp3_121*s->dpt[1][25]+ROcp3_421*s->dpt[2][25]+ROcp3_720*s->dpt[3][25];
    RLcp3_248 = ROcp3_221*s->dpt[1][25]+ROcp3_521*s->dpt[2][25]+ROcp3_820*s->dpt[3][25];
    RLcp3_348 = ROcp3_321*s->dpt[1][25]+ROcp3_621*s->dpt[2][25]+ROcp3_920*s->dpt[3][25];
    POcp3_148 = RLcp3_110+RLcp3_118+RLcp3_119+RLcp3_12+RLcp3_13+RLcp3_148;
    POcp3_248 = RLcp3_210+RLcp3_218+RLcp3_219+RLcp3_22+RLcp3_23+RLcp3_248;
    POcp3_348 = RLcp3_310+RLcp3_318+RLcp3_319+RLcp3_348+q[4];
    JTcp3_148_1 = -(RLcp3_210+RLcp3_218+RLcp3_219+RLcp3_22+RLcp3_23+RLcp3_248);
    JTcp3_248_1 = RLcp3_110+RLcp3_118+RLcp3_119+RLcp3_12+RLcp3_13+RLcp3_148;
    JTcp3_148_5 = -(RLcp3_210+RLcp3_218+RLcp3_219+RLcp3_248);
    JTcp3_248_5 = RLcp3_110+RLcp3_118+RLcp3_119+RLcp3_148;
    JTcp3_148_6 = S1p5*(RLcp3_310+RLcp3_318+RLcp3_319+RLcp3_348);
    JTcp3_248_6 = -C1p5*(RLcp3_310+RLcp3_318+RLcp3_319+RLcp3_348);
    JTcp3_348_6 = C1p5*(RLcp3_210+RLcp3_218+RLcp3_219+RLcp3_248)-S1p5*(RLcp3_110+RLcp3_118)-S1p5*(RLcp3_119+RLcp3_148);
    JTcp3_148_7 = ROcp3_56*(RLcp3_310+RLcp3_318+RLcp3_319+RLcp3_348)-S6*(RLcp3_210+RLcp3_218)-S6*(RLcp3_219+RLcp3_248);
    JTcp3_248_7 = -(ROcp3_46*(RLcp3_310+RLcp3_318+RLcp3_319+RLcp3_348)-S6*(RLcp3_110+RLcp3_118)-S6*(RLcp3_119+RLcp3_148));
    JTcp3_348_7 = ROcp3_46*(RLcp3_210+RLcp3_218+RLcp3_219+RLcp3_248)-ROcp3_56*(RLcp3_110+RLcp3_118)-ROcp3_56*(RLcp3_119+
 RLcp3_148);
    JTcp3_148_8 = ROcp3_56*(RLcp3_318+RLcp3_319)-S6*(RLcp3_218+RLcp3_219)-RLcp3_248*S6+RLcp3_348*ROcp3_56;
    JTcp3_248_8 = RLcp3_148*S6-RLcp3_348*ROcp3_46-ROcp3_46*(RLcp3_318+RLcp3_319)+S6*(RLcp3_118+RLcp3_119);
    JTcp3_348_8 = ROcp3_46*(RLcp3_218+RLcp3_219)-ROcp3_56*(RLcp3_118+RLcp3_119)-RLcp3_148*ROcp3_56+RLcp3_248*ROcp3_46;
    JTcp3_148_9 = ROcp3_210*(RLcp3_319+RLcp3_348)-ROcp3_310*(RLcp3_219+RLcp3_248);
    JTcp3_248_9 = -(ROcp3_110*(RLcp3_319+RLcp3_348)-ROcp3_310*(RLcp3_119+RLcp3_148));
    JTcp3_348_9 = ROcp3_110*(RLcp3_219+RLcp3_248)-ROcp3_210*(RLcp3_119+RLcp3_148);
    JTcp3_148_10 = -(RLcp3_248*ROcp3_310-RLcp3_348*ROcp3_210);
    JTcp3_248_10 = RLcp3_148*ROcp3_310-RLcp3_348*ROcp3_110;
    JTcp3_348_10 = -(RLcp3_148*ROcp3_210-RLcp3_248*ROcp3_110);
    JTcp3_148_11 = -(RLcp3_248*ROcp3_619-RLcp3_348*ROcp3_519);
    JTcp3_248_11 = RLcp3_148*ROcp3_619-RLcp3_348*ROcp3_419;
    JTcp3_348_11 = -(RLcp3_148*ROcp3_519-RLcp3_248*ROcp3_419);
    JTcp3_148_12 = -(RLcp3_248*ROcp3_920-RLcp3_348*ROcp3_820);
    JTcp3_248_12 = RLcp3_148*ROcp3_920-RLcp3_348*ROcp3_720;
    JTcp3_348_12 = -(RLcp3_148*ROcp3_820-RLcp3_248*ROcp3_720);
    ORcp3_148 = OMcp3_221*RLcp3_348-OMcp3_321*RLcp3_248;
    ORcp3_248 = -(OMcp3_121*RLcp3_348-OMcp3_321*RLcp3_148);
    ORcp3_348 = OMcp3_121*RLcp3_248-OMcp3_221*RLcp3_148;
    VIcp3_148 = ORcp3_110+ORcp3_118+ORcp3_119+ORcp3_12+ORcp3_13+ORcp3_148+qd[2]*C1-qd[3]*S1;
    VIcp3_248 = ORcp3_210+ORcp3_218+ORcp3_219+ORcp3_22+ORcp3_23+ORcp3_248+qd[2]*S1+qd[3]*C1;
    VIcp3_348 = ORcp3_310+ORcp3_318+ORcp3_319+ORcp3_348+qd[4];
    ACcp3_148 = OMcp3_218*ORcp3_319+OMcp3_221*ORcp3_348+OMcp3_27*(ORcp3_310+ORcp3_318)-OMcp3_318*ORcp3_219-OMcp3_321*
 ORcp3_248-OMcp3_37*ORcp3_210-OMcp3_37*ORcp3_218+OPcp3_218*RLcp3_319+OPcp3_221*RLcp3_348+OPcp3_27*RLcp3_310+OPcp3_27*
 RLcp3_318-OPcp3_318*RLcp3_219-OPcp3_321*RLcp3_248-OPcp3_37*RLcp3_210-OPcp3_37*RLcp3_218-ORcp3_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*
 qd[1]*qd[3]*C1-qd[1]*(ORcp3_22+(2.0)*qd[2]*S1);
    ACcp3_248 = -(OMcp3_118*ORcp3_319+OMcp3_121*ORcp3_348+OMcp3_17*(ORcp3_310+ORcp3_318)-OMcp3_318*ORcp3_119-OMcp3_321*
 ORcp3_148-OMcp3_37*ORcp3_110-OMcp3_37*ORcp3_118+OPcp3_118*RLcp3_319+OPcp3_121*RLcp3_348+OPcp3_17*RLcp3_310+OPcp3_17*
 RLcp3_318-OPcp3_318*RLcp3_119-OPcp3_321*RLcp3_148-OPcp3_37*RLcp3_110-OPcp3_37*RLcp3_118-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp3_12+(2.0)
 *qd[2]*C1)-qd[1]*(ORcp3_13-(2.0)*qd[3]*S1));
    ACcp3_348 = qdd[4]+OMcp3_118*ORcp3_219+OMcp3_121*ORcp3_248+OMcp3_17*ORcp3_210+OMcp3_17*ORcp3_218-OMcp3_218*ORcp3_119-
 OMcp3_221*ORcp3_148-OMcp3_27*ORcp3_110-OMcp3_27*ORcp3_118+OPcp3_118*RLcp3_219+OPcp3_121*RLcp3_248+OPcp3_17*RLcp3_210+
 OPcp3_17*RLcp3_218-OPcp3_218*RLcp3_119-OPcp3_221*RLcp3_148-OPcp3_27*RLcp3_110-OPcp3_27*RLcp3_118;

// = = Block_1_0_0_4_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp3_148;
    sens->P[2] = POcp3_248;
    sens->P[3] = POcp3_348;
    sens->R[1][1] = ROcp3_121;
    sens->R[1][2] = ROcp3_221;
    sens->R[1][3] = ROcp3_321;
    sens->R[2][1] = ROcp3_421;
    sens->R[2][2] = ROcp3_521;
    sens->R[2][3] = ROcp3_621;
    sens->R[3][1] = ROcp3_720;
    sens->R[3][2] = ROcp3_820;
    sens->R[3][3] = ROcp3_920;
    sens->V[1] = VIcp3_148;
    sens->V[2] = VIcp3_248;
    sens->V[3] = VIcp3_348;
    sens->OM[1] = OMcp3_121;
    sens->OM[2] = OMcp3_221;
    sens->OM[3] = OMcp3_321;
    sens->J[1][1] = JTcp3_148_1;
    sens->J[1][2] = C1;
    sens->J[1][3] = -S1;
    sens->J[1][5] = JTcp3_148_5;
    sens->J[1][6] = JTcp3_148_6;
    sens->J[1][7] = JTcp3_148_7;
    sens->J[1][10] = JTcp3_148_8;
    sens->J[1][18] = JTcp3_148_9;
    sens->J[1][19] = JTcp3_148_10;
    sens->J[1][20] = JTcp3_148_11;
    sens->J[1][21] = JTcp3_148_12;
    sens->J[2][1] = JTcp3_248_1;
    sens->J[2][2] = S1;
    sens->J[2][3] = C1;
    sens->J[2][5] = JTcp3_248_5;
    sens->J[2][6] = JTcp3_248_6;
    sens->J[2][7] = JTcp3_248_7;
    sens->J[2][10] = JTcp3_248_8;
    sens->J[2][18] = JTcp3_248_9;
    sens->J[2][19] = JTcp3_248_10;
    sens->J[2][20] = JTcp3_248_11;
    sens->J[2][21] = JTcp3_248_12;
    sens->J[3][4] = (1.0);
    sens->J[3][6] = JTcp3_348_6;
    sens->J[3][7] = JTcp3_348_7;
    sens->J[3][10] = JTcp3_348_8;
    sens->J[3][18] = JTcp3_348_9;
    sens->J[3][19] = JTcp3_348_10;
    sens->J[3][20] = JTcp3_348_11;
    sens->J[3][21] = JTcp3_348_12;
    sens->J[4][6] = C1p5;
    sens->J[4][7] = ROcp3_46;
    sens->J[4][10] = ROcp3_46;
    sens->J[4][18] = ROcp3_110;
    sens->J[4][19] = ROcp3_110;
    sens->J[4][20] = ROcp3_419;
    sens->J[4][21] = ROcp3_720;
    sens->J[5][6] = S1p5;
    sens->J[5][7] = ROcp3_56;
    sens->J[5][10] = ROcp3_56;
    sens->J[5][18] = ROcp3_210;
    sens->J[5][19] = ROcp3_210;
    sens->J[5][20] = ROcp3_519;
    sens->J[5][21] = ROcp3_820;
    sens->J[6][1] = (1.0);
    sens->J[6][5] = (1.0);
    sens->J[6][7] = S6;
    sens->J[6][10] = S6;
    sens->J[6][18] = ROcp3_310;
    sens->J[6][19] = ROcp3_310;
    sens->J[6][20] = ROcp3_619;
    sens->J[6][21] = ROcp3_920;
    sens->A[1] = ACcp3_148;
    sens->A[2] = ACcp3_248;
    sens->A[3] = ACcp3_348;
    sens->OMP[1] = OPcp3_121;
    sens->OMP[2] = OPcp3_221;
    sens->OMP[3] = OPcp3_321;
 
// 
break;
case 5:
 


// = = Block_1_0_0_5_0_1 = = 
 
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
    ORcp4_12 = -RLcp4_22*qd[1];
    ORcp4_22 = RLcp4_12*qd[1];
    RLcp4_13 = -q[3]*S1;
    RLcp4_23 = q[3]*C1;
    ORcp4_13 = -RLcp4_23*qd[1];
    ORcp4_23 = RLcp4_13*qd[1];
    OMcp4_35 = qd[1]+qd[5];
    OMcp4_16 = qd[6]*C1p5;
    OMcp4_26 = qd[6]*S1p5;
    OMcp4_17 = OMcp4_16+ROcp4_46*qd[7];
    OMcp4_27 = OMcp4_26+ROcp4_56*qd[7];
    OMcp4_37 = OMcp4_35+qd[7]*S6;
    OPcp4_17 = -(OMcp4_35*qd[6]*S1p5-ROcp4_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp4_26*S6-OMcp4_35*ROcp4_56));
    OPcp4_27 = OMcp4_35*qd[6]*C1p5+ROcp4_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp4_16*S6-OMcp4_35*ROcp4_46);
    OPcp4_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_5_0_3 = = 
 
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

// = = Block_1_0_0_5_0_6 = = 
 
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
    RLcp4_118 = ROcp4_46*s->dpt[2][12]+ROcp4_710*s->dpt[3][12];
    RLcp4_218 = ROcp4_56*s->dpt[2][12]+ROcp4_810*s->dpt[3][12];
    RLcp4_318 = ROcp4_910*s->dpt[3][12]+s->dpt[2][12]*S6;
    OMcp4_118 = OMcp4_17+ROcp4_110*qd[18];
    OMcp4_218 = OMcp4_27+ROcp4_210*qd[18];
    OMcp4_318 = OMcp4_37+ROcp4_310*qd[18];
    ORcp4_118 = OMcp4_27*RLcp4_318-OMcp4_37*RLcp4_218;
    ORcp4_218 = -(OMcp4_17*RLcp4_318-OMcp4_37*RLcp4_118);
    ORcp4_318 = OMcp4_17*RLcp4_218-OMcp4_27*RLcp4_118;
    OPcp4_118 = OPcp4_17+ROcp4_110*qdd[18]+qd[18]*(OMcp4_27*ROcp4_310-OMcp4_37*ROcp4_210);
    OPcp4_218 = OPcp4_27+ROcp4_210*qdd[18]-qd[18]*(OMcp4_17*ROcp4_310-OMcp4_37*ROcp4_110);
    OPcp4_318 = OPcp4_37+ROcp4_310*qdd[18]+qd[18]*(OMcp4_17*ROcp4_210-OMcp4_27*ROcp4_110);
    RLcp4_119 = ROcp4_418*s->dpt[2][22];
    RLcp4_219 = ROcp4_518*s->dpt[2][22];
    RLcp4_319 = ROcp4_618*s->dpt[2][22];
    OMcp4_119 = OMcp4_118+ROcp4_110*qd[19];
    OMcp4_219 = OMcp4_218+ROcp4_210*qd[19];
    OMcp4_319 = OMcp4_318+ROcp4_310*qd[19];
    ORcp4_119 = OMcp4_218*RLcp4_319-OMcp4_318*RLcp4_219;
    ORcp4_219 = -(OMcp4_118*RLcp4_319-OMcp4_318*RLcp4_119);
    ORcp4_319 = OMcp4_118*RLcp4_219-OMcp4_218*RLcp4_119;
    OMcp4_120 = OMcp4_119+ROcp4_419*qd[20];
    OMcp4_220 = OMcp4_219+ROcp4_519*qd[20];
    OMcp4_320 = OMcp4_319+ROcp4_619*qd[20];
    OMcp4_121 = OMcp4_120+ROcp4_720*qd[21];
    OMcp4_221 = OMcp4_220+ROcp4_820*qd[21];
    OMcp4_321 = OMcp4_320+ROcp4_920*qd[21];
    OPcp4_121 = OPcp4_118+ROcp4_110*qdd[19]+ROcp4_419*qdd[20]+ROcp4_720*qdd[21]+qd[19]*(OMcp4_218*ROcp4_310-OMcp4_318*
 ROcp4_210)+qd[20]*(OMcp4_219*ROcp4_619-OMcp4_319*ROcp4_519)+qd[21]*(OMcp4_220*ROcp4_920-OMcp4_320*ROcp4_820);
    OPcp4_221 = OPcp4_218+ROcp4_210*qdd[19]+ROcp4_519*qdd[20]+ROcp4_820*qdd[21]-qd[19]*(OMcp4_118*ROcp4_310-OMcp4_318*
 ROcp4_110)-qd[20]*(OMcp4_119*ROcp4_619-OMcp4_319*ROcp4_419)-qd[21]*(OMcp4_120*ROcp4_920-OMcp4_320*ROcp4_720);
    OPcp4_321 = OPcp4_318+ROcp4_310*qdd[19]+ROcp4_619*qdd[20]+ROcp4_920*qdd[21]+qd[19]*(OMcp4_118*ROcp4_210-OMcp4_218*
 ROcp4_110)+qd[20]*(OMcp4_119*ROcp4_519-OMcp4_219*ROcp4_419)+qd[21]*(OMcp4_120*ROcp4_820-OMcp4_220*ROcp4_720);
    RLcp4_122 = ROcp4_121*s->dpt[1][25]+ROcp4_421*s->dpt[2][25]+ROcp4_720*s->dpt[3][25];
    RLcp4_222 = ROcp4_221*s->dpt[1][25]+ROcp4_521*s->dpt[2][25]+ROcp4_820*s->dpt[3][25];
    RLcp4_322 = ROcp4_321*s->dpt[1][25]+ROcp4_621*s->dpt[2][25]+ROcp4_920*s->dpt[3][25];
    POcp4_122 = RLcp4_110+RLcp4_118+RLcp4_119+RLcp4_12+RLcp4_122+RLcp4_13;
    POcp4_222 = RLcp4_210+RLcp4_218+RLcp4_219+RLcp4_22+RLcp4_222+RLcp4_23;
    POcp4_322 = RLcp4_310+RLcp4_318+RLcp4_319+RLcp4_322+q[4];
    JTcp4_122_1 = -(RLcp4_210+RLcp4_218+RLcp4_219+RLcp4_22+RLcp4_222+RLcp4_23);
    JTcp4_222_1 = RLcp4_110+RLcp4_118+RLcp4_119+RLcp4_12+RLcp4_122+RLcp4_13;
    JTcp4_122_5 = -(RLcp4_210+RLcp4_218+RLcp4_219+RLcp4_222);
    JTcp4_222_5 = RLcp4_110+RLcp4_118+RLcp4_119+RLcp4_122;
    JTcp4_122_6 = S1p5*(RLcp4_310+RLcp4_318+RLcp4_319+RLcp4_322);
    JTcp4_222_6 = -C1p5*(RLcp4_310+RLcp4_318+RLcp4_319+RLcp4_322);
    JTcp4_322_6 = C1p5*(RLcp4_210+RLcp4_218+RLcp4_219+RLcp4_222)-S1p5*(RLcp4_110+RLcp4_118)-S1p5*(RLcp4_119+RLcp4_122);
    JTcp4_122_7 = ROcp4_56*(RLcp4_310+RLcp4_318+RLcp4_319+RLcp4_322)-S6*(RLcp4_210+RLcp4_218)-S6*(RLcp4_219+RLcp4_222);
    JTcp4_222_7 = -(ROcp4_46*(RLcp4_310+RLcp4_318+RLcp4_319+RLcp4_322)-S6*(RLcp4_110+RLcp4_118)-S6*(RLcp4_119+RLcp4_122));
    JTcp4_322_7 = ROcp4_46*(RLcp4_210+RLcp4_218+RLcp4_219+RLcp4_222)-ROcp4_56*(RLcp4_110+RLcp4_118)-ROcp4_56*(RLcp4_119+
 RLcp4_122);
    JTcp4_122_8 = ROcp4_56*(RLcp4_318+RLcp4_319)-S6*(RLcp4_218+RLcp4_219)-RLcp4_222*S6+RLcp4_322*ROcp4_56;
    JTcp4_222_8 = RLcp4_122*S6-RLcp4_322*ROcp4_46-ROcp4_46*(RLcp4_318+RLcp4_319)+S6*(RLcp4_118+RLcp4_119);
    JTcp4_322_8 = ROcp4_46*(RLcp4_218+RLcp4_219)-ROcp4_56*(RLcp4_118+RLcp4_119)-RLcp4_122*ROcp4_56+RLcp4_222*ROcp4_46;
    JTcp4_122_9 = ROcp4_210*(RLcp4_319+RLcp4_322)-ROcp4_310*(RLcp4_219+RLcp4_222);
    JTcp4_222_9 = -(ROcp4_110*(RLcp4_319+RLcp4_322)-ROcp4_310*(RLcp4_119+RLcp4_122));
    JTcp4_322_9 = ROcp4_110*(RLcp4_219+RLcp4_222)-ROcp4_210*(RLcp4_119+RLcp4_122);
    JTcp4_122_10 = -(RLcp4_222*ROcp4_310-RLcp4_322*ROcp4_210);
    JTcp4_222_10 = RLcp4_122*ROcp4_310-RLcp4_322*ROcp4_110;
    JTcp4_322_10 = -(RLcp4_122*ROcp4_210-RLcp4_222*ROcp4_110);
    JTcp4_122_11 = -(RLcp4_222*ROcp4_619-RLcp4_322*ROcp4_519);
    JTcp4_222_11 = RLcp4_122*ROcp4_619-RLcp4_322*ROcp4_419;
    JTcp4_322_11 = -(RLcp4_122*ROcp4_519-RLcp4_222*ROcp4_419);
    JTcp4_122_12 = -(RLcp4_222*ROcp4_920-RLcp4_322*ROcp4_820);
    JTcp4_222_12 = RLcp4_122*ROcp4_920-RLcp4_322*ROcp4_720;
    JTcp4_322_12 = -(RLcp4_122*ROcp4_820-RLcp4_222*ROcp4_720);
    ORcp4_122 = OMcp4_221*RLcp4_322-OMcp4_321*RLcp4_222;
    ORcp4_222 = -(OMcp4_121*RLcp4_322-OMcp4_321*RLcp4_122);
    ORcp4_322 = OMcp4_121*RLcp4_222-OMcp4_221*RLcp4_122;
    VIcp4_122 = ORcp4_110+ORcp4_118+ORcp4_119+ORcp4_12+ORcp4_122+ORcp4_13+qd[2]*C1-qd[3]*S1;
    VIcp4_222 = ORcp4_210+ORcp4_218+ORcp4_219+ORcp4_22+ORcp4_222+ORcp4_23+qd[2]*S1+qd[3]*C1;
    VIcp4_322 = ORcp4_310+ORcp4_318+ORcp4_319+ORcp4_322+qd[4];
    ACcp4_122 = OMcp4_218*ORcp4_319+OMcp4_221*ORcp4_322+OMcp4_27*(ORcp4_310+ORcp4_318)-OMcp4_318*ORcp4_219-OMcp4_321*
 ORcp4_222-OMcp4_37*ORcp4_210-OMcp4_37*ORcp4_218+OPcp4_218*RLcp4_319+OPcp4_221*RLcp4_322+OPcp4_27*RLcp4_310+OPcp4_27*
 RLcp4_318-OPcp4_318*RLcp4_219-OPcp4_321*RLcp4_222-OPcp4_37*RLcp4_210-OPcp4_37*RLcp4_218-ORcp4_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*
 qd[1]*qd[3]*C1-qd[1]*(ORcp4_22+(2.0)*qd[2]*S1);
    ACcp4_222 = -(OMcp4_118*ORcp4_319+OMcp4_121*ORcp4_322+OMcp4_17*(ORcp4_310+ORcp4_318)-OMcp4_318*ORcp4_119-OMcp4_321*
 ORcp4_122-OMcp4_37*ORcp4_110-OMcp4_37*ORcp4_118+OPcp4_118*RLcp4_319+OPcp4_121*RLcp4_322+OPcp4_17*RLcp4_310+OPcp4_17*
 RLcp4_318-OPcp4_318*RLcp4_119-OPcp4_321*RLcp4_122-OPcp4_37*RLcp4_110-OPcp4_37*RLcp4_118-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp4_12+(2.0)
 *qd[2]*C1)-qd[1]*(ORcp4_13-(2.0)*qd[3]*S1));
    ACcp4_322 = qdd[4]+OMcp4_118*ORcp4_219+OMcp4_121*ORcp4_222+OMcp4_17*ORcp4_210+OMcp4_17*ORcp4_218-OMcp4_218*ORcp4_119-
 OMcp4_221*ORcp4_122-OMcp4_27*ORcp4_110-OMcp4_27*ORcp4_118+OPcp4_118*RLcp4_219+OPcp4_121*RLcp4_222+OPcp4_17*RLcp4_210+
 OPcp4_17*RLcp4_218-OPcp4_218*RLcp4_119-OPcp4_221*RLcp4_122-OPcp4_27*RLcp4_110-OPcp4_27*RLcp4_118;
    OMcp4_123 = OMcp4_121+ROcp4_422*qd[23];
    OMcp4_223 = OMcp4_221+ROcp4_522*qd[23];
    OMcp4_323 = OMcp4_321+ROcp4_622*qd[23];
    OPcp4_123 = OPcp4_121+ROcp4_422*qdd[23]+qd[23]*(OMcp4_221*ROcp4_622-OMcp4_321*ROcp4_522);
    OPcp4_223 = OPcp4_221+ROcp4_522*qdd[23]-qd[23]*(OMcp4_121*ROcp4_622-OMcp4_321*ROcp4_422);
    OPcp4_323 = OPcp4_321+ROcp4_622*qdd[23]+qd[23]*(OMcp4_121*ROcp4_522-OMcp4_221*ROcp4_422);

// = = Block_1_0_0_5_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp4_122;
    sens->P[2] = POcp4_222;
    sens->P[3] = POcp4_322;
    sens->R[1][1] = ROcp4_123;
    sens->R[1][2] = ROcp4_223;
    sens->R[1][3] = ROcp4_323;
    sens->R[2][1] = ROcp4_422;
    sens->R[2][2] = ROcp4_522;
    sens->R[2][3] = ROcp4_622;
    sens->R[3][1] = ROcp4_723;
    sens->R[3][2] = ROcp4_823;
    sens->R[3][3] = ROcp4_923;
    sens->V[1] = VIcp4_122;
    sens->V[2] = VIcp4_222;
    sens->V[3] = VIcp4_322;
    sens->OM[1] = OMcp4_123;
    sens->OM[2] = OMcp4_223;
    sens->OM[3] = OMcp4_323;
    sens->J[1][1] = JTcp4_122_1;
    sens->J[1][2] = C1;
    sens->J[1][3] = -S1;
    sens->J[1][5] = JTcp4_122_5;
    sens->J[1][6] = JTcp4_122_6;
    sens->J[1][7] = JTcp4_122_7;
    sens->J[1][10] = JTcp4_122_8;
    sens->J[1][18] = JTcp4_122_9;
    sens->J[1][19] = JTcp4_122_10;
    sens->J[1][20] = JTcp4_122_11;
    sens->J[1][21] = JTcp4_122_12;
    sens->J[2][1] = JTcp4_222_1;
    sens->J[2][2] = S1;
    sens->J[2][3] = C1;
    sens->J[2][5] = JTcp4_222_5;
    sens->J[2][6] = JTcp4_222_6;
    sens->J[2][7] = JTcp4_222_7;
    sens->J[2][10] = JTcp4_222_8;
    sens->J[2][18] = JTcp4_222_9;
    sens->J[2][19] = JTcp4_222_10;
    sens->J[2][20] = JTcp4_222_11;
    sens->J[2][21] = JTcp4_222_12;
    sens->J[3][4] = (1.0);
    sens->J[3][6] = JTcp4_322_6;
    sens->J[3][7] = JTcp4_322_7;
    sens->J[3][10] = JTcp4_322_8;
    sens->J[3][18] = JTcp4_322_9;
    sens->J[3][19] = JTcp4_322_10;
    sens->J[3][20] = JTcp4_322_11;
    sens->J[3][21] = JTcp4_322_12;
    sens->J[4][6] = C1p5;
    sens->J[4][7] = ROcp4_46;
    sens->J[4][10] = ROcp4_46;
    sens->J[4][18] = ROcp4_110;
    sens->J[4][19] = ROcp4_110;
    sens->J[4][20] = ROcp4_419;
    sens->J[4][21] = ROcp4_720;
    sens->J[4][22] = ROcp4_121;
    sens->J[4][23] = ROcp4_422;
    sens->J[5][6] = S1p5;
    sens->J[5][7] = ROcp4_56;
    sens->J[5][10] = ROcp4_56;
    sens->J[5][18] = ROcp4_210;
    sens->J[5][19] = ROcp4_210;
    sens->J[5][20] = ROcp4_519;
    sens->J[5][21] = ROcp4_820;
    sens->J[5][22] = ROcp4_221;
    sens->J[5][23] = ROcp4_522;
    sens->J[6][1] = (1.0);
    sens->J[6][5] = (1.0);
    sens->J[6][7] = S6;
    sens->J[6][10] = S6;
    sens->J[6][18] = ROcp4_310;
    sens->J[6][19] = ROcp4_310;
    sens->J[6][20] = ROcp4_619;
    sens->J[6][21] = ROcp4_920;
    sens->J[6][22] = ROcp4_321;
    sens->J[6][23] = ROcp4_622;
    sens->A[1] = ACcp4_122;
    sens->A[2] = ACcp4_222;
    sens->A[3] = ACcp4_322;
    sens->OMP[1] = OPcp4_123;
    sens->OMP[2] = OPcp4_223;
    sens->OMP[3] = OPcp4_323;
 
// 
break;
case 6:
 


// = = Block_1_0_0_6_0_1 = = 
 
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
    ORcp5_12 = -RLcp5_22*qd[1];
    ORcp5_22 = RLcp5_12*qd[1];
    RLcp5_13 = -q[3]*S1;
    RLcp5_23 = q[3]*C1;
    ORcp5_13 = -RLcp5_23*qd[1];
    ORcp5_23 = RLcp5_13*qd[1];
    OMcp5_35 = qd[1]+qd[5];
    OMcp5_16 = qd[6]*C1p5;
    OMcp5_26 = qd[6]*S1p5;
    OMcp5_17 = OMcp5_16+ROcp5_46*qd[7];
    OMcp5_27 = OMcp5_26+ROcp5_56*qd[7];
    OMcp5_37 = OMcp5_35+qd[7]*S6;
    OPcp5_17 = -(OMcp5_35*qd[6]*S1p5-ROcp5_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp5_26*S6-OMcp5_35*ROcp5_56));
    OPcp5_27 = OMcp5_35*qd[6]*C1p5+ROcp5_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp5_16*S6-OMcp5_35*ROcp5_46);
    OPcp5_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_6_0_2 = = 
 
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
    OMcp5_18 = OMcp5_17+ROcp5_46*qd[8];
    OMcp5_28 = OMcp5_27+ROcp5_56*qd[8];
    OMcp5_38 = OMcp5_37+qd[8]*S6;
    ORcp5_18 = OMcp5_27*RLcp5_38-OMcp5_37*RLcp5_28;
    ORcp5_28 = -(OMcp5_17*RLcp5_38-OMcp5_37*RLcp5_18);
    ORcp5_38 = OMcp5_17*RLcp5_28-OMcp5_27*RLcp5_18;
    OPcp5_18 = OPcp5_17+ROcp5_46*qdd[8]+qd[8]*(OMcp5_27*S6-OMcp5_37*ROcp5_56);
    OPcp5_28 = OPcp5_27+ROcp5_56*qdd[8]-qd[8]*(OMcp5_17*S6-OMcp5_37*ROcp5_46);
    OPcp5_38 = OPcp5_37+qdd[8]*S6+qd[8]*(OMcp5_17*ROcp5_56-OMcp5_27*ROcp5_46);
    RLcp5_19 = ROcp5_18*s->dpt[1][8];
    RLcp5_29 = ROcp5_28*s->dpt[1][8];
    RLcp5_39 = -s->dpt[1][8]*S7p8*C6;
    POcp5_19 = RLcp5_12+RLcp5_13+RLcp5_18+RLcp5_19;
    POcp5_29 = RLcp5_22+RLcp5_23+RLcp5_28+RLcp5_29;
    POcp5_39 = RLcp5_38+RLcp5_39+q[4];
    OMcp5_19 = OMcp5_18+ROcp5_46*qd[9];
    OMcp5_29 = OMcp5_28+ROcp5_56*qd[9];
    OMcp5_39 = OMcp5_38+qd[9]*S6;
    ORcp5_19 = OMcp5_28*RLcp5_39-OMcp5_38*RLcp5_29;
    ORcp5_29 = -(OMcp5_18*RLcp5_39-OMcp5_38*RLcp5_19);
    ORcp5_39 = OMcp5_18*RLcp5_29-OMcp5_28*RLcp5_19;
    VIcp5_19 = ORcp5_12+ORcp5_13+ORcp5_18+ORcp5_19+qd[2]*C1-qd[3]*S1;
    VIcp5_29 = ORcp5_22+ORcp5_23+ORcp5_28+ORcp5_29+qd[2]*S1+qd[3]*C1;
    VIcp5_39 = ORcp5_38+ORcp5_39+qd[4];
    OPcp5_19 = OPcp5_18+ROcp5_46*qdd[9]+qd[9]*(OMcp5_28*S6-OMcp5_38*ROcp5_56);
    OPcp5_29 = OPcp5_28+ROcp5_56*qdd[9]-qd[9]*(OMcp5_18*S6-OMcp5_38*ROcp5_46);
    OPcp5_39 = OPcp5_38+qdd[9]*S6+qd[9]*(OMcp5_18*ROcp5_56-OMcp5_28*ROcp5_46);
    ACcp5_19 = OMcp5_27*ORcp5_38+OMcp5_28*ORcp5_39-OMcp5_37*ORcp5_28-OMcp5_38*ORcp5_29+OPcp5_27*RLcp5_38+OPcp5_28*RLcp5_39
 -OPcp5_37*RLcp5_28-OPcp5_38*RLcp5_29-ORcp5_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp5_22+(2.0)*qd[2]*S1);
    ACcp5_29 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp5_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp5_13-(2.0)*qd[3]*S1)-OMcp5_17*ORcp5_38+OMcp5_37*
 ORcp5_18-OPcp5_17*RLcp5_38+OPcp5_37*RLcp5_18-OMcp5_18*ORcp5_39+OMcp5_38*ORcp5_19-OPcp5_18*RLcp5_39+OPcp5_38*RLcp5_19;
    ACcp5_39 = qdd[4]+OMcp5_17*ORcp5_28+OMcp5_18*ORcp5_29-OMcp5_27*ORcp5_18-OMcp5_28*ORcp5_19+OPcp5_17*RLcp5_28+OPcp5_18*
 RLcp5_29-OPcp5_27*RLcp5_18-OPcp5_28*RLcp5_19;

// = = Block_1_0_0_6_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp5_19;
    sens->P[2] = POcp5_29;
    sens->P[3] = POcp5_39;
    sens->R[1][1] = ROcp5_19;
    sens->R[1][2] = ROcp5_29;
    sens->R[1][3] = ROcp5_39;
    sens->R[2][1] = ROcp5_46;
    sens->R[2][2] = ROcp5_56;
    sens->R[2][3] = S6;
    sens->R[3][1] = ROcp5_79;
    sens->R[3][2] = ROcp5_89;
    sens->R[3][3] = ROcp5_99;
    sens->V[1] = VIcp5_19;
    sens->V[2] = VIcp5_29;
    sens->V[3] = VIcp5_39;
    sens->OM[1] = OMcp5_19;
    sens->OM[2] = OMcp5_29;
    sens->OM[3] = OMcp5_39;
    sens->A[1] = ACcp5_19;
    sens->A[2] = ACcp5_29;
    sens->A[3] = ACcp5_39;
    sens->OMP[1] = OPcp5_19;
    sens->OMP[2] = OPcp5_29;
    sens->OMP[3] = OPcp5_39;
 
// 
break;
case 7:
 


// = = Block_1_0_0_7_0_1 = = 
 
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
    ORcp6_12 = -RLcp6_22*qd[1];
    ORcp6_22 = RLcp6_12*qd[1];
    RLcp6_13 = -q[3]*S1;
    RLcp6_23 = q[3]*C1;
    ORcp6_13 = -RLcp6_23*qd[1];
    ORcp6_23 = RLcp6_13*qd[1];
    OMcp6_35 = qd[1]+qd[5];
    OMcp6_16 = qd[6]*C1p5;
    OMcp6_26 = qd[6]*S1p5;
    OMcp6_17 = OMcp6_16+ROcp6_46*qd[7];
    OMcp6_27 = OMcp6_26+ROcp6_56*qd[7];
    OMcp6_37 = OMcp6_35+qd[7]*S6;
    OPcp6_17 = -(OMcp6_35*qd[6]*S1p5-ROcp6_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp6_26*S6-OMcp6_35*ROcp6_56));
    OPcp6_27 = OMcp6_35*qd[6]*C1p5+ROcp6_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp6_16*S6-OMcp6_35*ROcp6_46);
    OPcp6_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_7_0_3 = = 
 
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

// = = Block_1_0_0_7_0_5 = = 
 
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
    OMcp6_112 = OMcp6_17+ROcp6_110*qd[12];
    OMcp6_212 = OMcp6_27+ROcp6_210*qd[12];
    OMcp6_312 = OMcp6_37+ROcp6_310*qd[12];
    ORcp6_112 = OMcp6_27*RLcp6_312-OMcp6_37*RLcp6_212;
    ORcp6_212 = -(OMcp6_17*RLcp6_312-OMcp6_37*RLcp6_112);
    ORcp6_312 = OMcp6_17*RLcp6_212-OMcp6_27*RLcp6_112;
    OPcp6_112 = OPcp6_17+ROcp6_110*qdd[12]+qd[12]*(OMcp6_27*ROcp6_310-OMcp6_37*ROcp6_210);
    OPcp6_212 = OPcp6_27+ROcp6_210*qdd[12]-qd[12]*(OMcp6_17*ROcp6_310-OMcp6_37*ROcp6_110);
    OPcp6_312 = OPcp6_37+ROcp6_310*qdd[12]+qd[12]*(OMcp6_17*ROcp6_210-OMcp6_27*ROcp6_110);
    RLcp6_113 = ROcp6_412*s->dpt[2][16];
    RLcp6_213 = ROcp6_512*s->dpt[2][16];
    RLcp6_313 = ROcp6_612*s->dpt[2][16];
    OMcp6_113 = OMcp6_112+ROcp6_110*qd[13];
    OMcp6_213 = OMcp6_212+ROcp6_210*qd[13];
    OMcp6_313 = OMcp6_312+ROcp6_310*qd[13];
    ORcp6_113 = OMcp6_212*RLcp6_313-OMcp6_312*RLcp6_213;
    ORcp6_213 = -(OMcp6_112*RLcp6_313-OMcp6_312*RLcp6_113);
    ORcp6_313 = OMcp6_112*RLcp6_213-OMcp6_212*RLcp6_113;
    OMcp6_114 = OMcp6_113+ROcp6_413*qd[14];
    OMcp6_214 = OMcp6_213+ROcp6_513*qd[14];
    OMcp6_314 = OMcp6_313+ROcp6_613*qd[14];
    OMcp6_115 = OMcp6_114+ROcp6_714*qd[15];
    OMcp6_215 = OMcp6_214+ROcp6_814*qd[15];
    OMcp6_315 = OMcp6_314+ROcp6_914*qd[15];
    OPcp6_115 = OPcp6_112+ROcp6_110*qdd[13]+ROcp6_413*qdd[14]+ROcp6_714*qdd[15]+qd[13]*(OMcp6_212*ROcp6_310-OMcp6_312*
 ROcp6_210)+qd[14]*(OMcp6_213*ROcp6_613-OMcp6_313*ROcp6_513)+qd[15]*(OMcp6_214*ROcp6_914-OMcp6_314*ROcp6_814);
    OPcp6_215 = OPcp6_212+ROcp6_210*qdd[13]+ROcp6_513*qdd[14]+ROcp6_814*qdd[15]-qd[13]*(OMcp6_112*ROcp6_310-OMcp6_312*
 ROcp6_110)-qd[14]*(OMcp6_113*ROcp6_613-OMcp6_313*ROcp6_413)-qd[15]*(OMcp6_114*ROcp6_914-OMcp6_314*ROcp6_714);
    OPcp6_315 = OPcp6_312+ROcp6_310*qdd[13]+ROcp6_613*qdd[14]+ROcp6_914*qdd[15]+qd[13]*(OMcp6_112*ROcp6_210-OMcp6_212*
 ROcp6_110)+qd[14]*(OMcp6_113*ROcp6_513-OMcp6_213*ROcp6_413)+qd[15]*(OMcp6_114*ROcp6_814-OMcp6_214*ROcp6_714);
    RLcp6_116 = ROcp6_115*s->dpt[1][19]+ROcp6_415*s->dpt[2][19]+ROcp6_714*s->dpt[3][19];
    RLcp6_216 = ROcp6_215*s->dpt[1][19]+ROcp6_515*s->dpt[2][19]+ROcp6_814*s->dpt[3][19];
    RLcp6_316 = ROcp6_315*s->dpt[1][19]+ROcp6_615*s->dpt[2][19]+ROcp6_914*s->dpt[3][19];
    POcp6_116 = RLcp6_110+RLcp6_112+RLcp6_113+RLcp6_116+RLcp6_12+RLcp6_13;
    POcp6_216 = RLcp6_210+RLcp6_212+RLcp6_213+RLcp6_216+RLcp6_22+RLcp6_23;
    POcp6_316 = RLcp6_310+RLcp6_312+RLcp6_313+RLcp6_316+q[4];
    ORcp6_116 = OMcp6_215*RLcp6_316-OMcp6_315*RLcp6_216;
    ORcp6_216 = -(OMcp6_115*RLcp6_316-OMcp6_315*RLcp6_116);
    ORcp6_316 = OMcp6_115*RLcp6_216-OMcp6_215*RLcp6_116;
    VIcp6_116 = ORcp6_110+ORcp6_112+ORcp6_113+ORcp6_116+ORcp6_12+ORcp6_13+qd[2]*C1-qd[3]*S1;
    VIcp6_216 = ORcp6_210+ORcp6_212+ORcp6_213+ORcp6_216+ORcp6_22+ORcp6_23+qd[2]*S1+qd[3]*C1;
    VIcp6_316 = ORcp6_310+ORcp6_312+ORcp6_313+ORcp6_316+qd[4];
    ACcp6_116 = OMcp6_212*ORcp6_313+OMcp6_215*ORcp6_316+OMcp6_27*(ORcp6_310+ORcp6_312)-OMcp6_312*ORcp6_213-OMcp6_315*
 ORcp6_216-OMcp6_37*ORcp6_210-OMcp6_37*ORcp6_212+OPcp6_212*RLcp6_313+OPcp6_215*RLcp6_316+OPcp6_27*RLcp6_310+OPcp6_27*
 RLcp6_312-OPcp6_312*RLcp6_213-OPcp6_315*RLcp6_216-OPcp6_37*RLcp6_210-OPcp6_37*RLcp6_212-ORcp6_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*
 qd[1]*qd[3]*C1-qd[1]*(ORcp6_22+(2.0)*qd[2]*S1);
    ACcp6_216 = -(OMcp6_112*ORcp6_313+OMcp6_115*ORcp6_316+OMcp6_17*(ORcp6_310+ORcp6_312)-OMcp6_312*ORcp6_113-OMcp6_315*
 ORcp6_116-OMcp6_37*ORcp6_110-OMcp6_37*ORcp6_112+OPcp6_112*RLcp6_313+OPcp6_115*RLcp6_316+OPcp6_17*RLcp6_310+OPcp6_17*
 RLcp6_312-OPcp6_312*RLcp6_113-OPcp6_315*RLcp6_116-OPcp6_37*RLcp6_110-OPcp6_37*RLcp6_112-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp6_12+(2.0)
 *qd[2]*C1)-qd[1]*(ORcp6_13-(2.0)*qd[3]*S1));
    ACcp6_316 = qdd[4]+OMcp6_112*ORcp6_213+OMcp6_115*ORcp6_216+OMcp6_17*ORcp6_210+OMcp6_17*ORcp6_212-OMcp6_212*ORcp6_113-
 OMcp6_215*ORcp6_116-OMcp6_27*ORcp6_110-OMcp6_27*ORcp6_112+OPcp6_112*RLcp6_213+OPcp6_115*RLcp6_216+OPcp6_17*RLcp6_210+
 OPcp6_17*RLcp6_212-OPcp6_212*RLcp6_113-OPcp6_215*RLcp6_116-OPcp6_27*RLcp6_110-OPcp6_27*RLcp6_112;
    OMcp6_117 = OMcp6_115+ROcp6_416*qd[17];
    OMcp6_217 = OMcp6_215+ROcp6_516*qd[17];
    OMcp6_317 = OMcp6_315+ROcp6_616*qd[17];
    OPcp6_117 = OPcp6_115+ROcp6_416*qdd[17]+qd[17]*(OMcp6_215*ROcp6_616-OMcp6_315*ROcp6_516);
    OPcp6_217 = OPcp6_215+ROcp6_516*qdd[17]-qd[17]*(OMcp6_115*ROcp6_616-OMcp6_315*ROcp6_416);
    OPcp6_317 = OPcp6_315+ROcp6_616*qdd[17]+qd[17]*(OMcp6_115*ROcp6_516-OMcp6_215*ROcp6_416);

// = = Block_1_0_0_7_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp6_116;
    sens->P[2] = POcp6_216;
    sens->P[3] = POcp6_316;
    sens->R[1][1] = ROcp6_117;
    sens->R[1][2] = ROcp6_217;
    sens->R[1][3] = ROcp6_317;
    sens->R[2][1] = ROcp6_416;
    sens->R[2][2] = ROcp6_516;
    sens->R[2][3] = ROcp6_616;
    sens->R[3][1] = ROcp6_717;
    sens->R[3][2] = ROcp6_817;
    sens->R[3][3] = ROcp6_917;
    sens->V[1] = VIcp6_116;
    sens->V[2] = VIcp6_216;
    sens->V[3] = VIcp6_316;
    sens->OM[1] = OMcp6_117;
    sens->OM[2] = OMcp6_217;
    sens->OM[3] = OMcp6_317;
    sens->A[1] = ACcp6_116;
    sens->A[2] = ACcp6_216;
    sens->A[3] = ACcp6_316;
    sens->OMP[1] = OPcp6_117;
    sens->OMP[2] = OPcp6_217;
    sens->OMP[3] = OPcp6_317;
 
// 
break;
case 8:
 


// = = Block_1_0_0_8_0_1 = = 
 
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
    ORcp7_12 = -RLcp7_22*qd[1];
    ORcp7_22 = RLcp7_12*qd[1];
    RLcp7_13 = -q[3]*S1;
    RLcp7_23 = q[3]*C1;
    ORcp7_13 = -RLcp7_23*qd[1];
    ORcp7_23 = RLcp7_13*qd[1];
    OMcp7_35 = qd[1]+qd[5];
    OMcp7_16 = qd[6]*C1p5;
    OMcp7_26 = qd[6]*S1p5;
    OMcp7_17 = OMcp7_16+ROcp7_46*qd[7];
    OMcp7_27 = OMcp7_26+ROcp7_56*qd[7];
    OMcp7_37 = OMcp7_35+qd[7]*S6;
    OPcp7_17 = -(OMcp7_35*qd[6]*S1p5-ROcp7_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp7_26*S6-OMcp7_35*ROcp7_56));
    OPcp7_27 = OMcp7_35*qd[6]*C1p5+ROcp7_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp7_16*S6-OMcp7_35*ROcp7_46);
    OPcp7_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_8_0_3 = = 
 
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

// = = Block_1_0_0_8_0_6 = = 
 
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
    OMcp7_118 = OMcp7_17+ROcp7_110*qd[18];
    OMcp7_218 = OMcp7_27+ROcp7_210*qd[18];
    OMcp7_318 = OMcp7_37+ROcp7_310*qd[18];
    ORcp7_118 = OMcp7_27*RLcp7_318-OMcp7_37*RLcp7_218;
    ORcp7_218 = -(OMcp7_17*RLcp7_318-OMcp7_37*RLcp7_118);
    ORcp7_318 = OMcp7_17*RLcp7_218-OMcp7_27*RLcp7_118;
    OPcp7_118 = OPcp7_17+ROcp7_110*qdd[18]+qd[18]*(OMcp7_27*ROcp7_310-OMcp7_37*ROcp7_210);
    OPcp7_218 = OPcp7_27+ROcp7_210*qdd[18]-qd[18]*(OMcp7_17*ROcp7_310-OMcp7_37*ROcp7_110);
    OPcp7_318 = OPcp7_37+ROcp7_310*qdd[18]+qd[18]*(OMcp7_17*ROcp7_210-OMcp7_27*ROcp7_110);
    RLcp7_119 = ROcp7_418*s->dpt[2][22];
    RLcp7_219 = ROcp7_518*s->dpt[2][22];
    RLcp7_319 = ROcp7_618*s->dpt[2][22];
    OMcp7_119 = OMcp7_118+ROcp7_110*qd[19];
    OMcp7_219 = OMcp7_218+ROcp7_210*qd[19];
    OMcp7_319 = OMcp7_318+ROcp7_310*qd[19];
    ORcp7_119 = OMcp7_218*RLcp7_319-OMcp7_318*RLcp7_219;
    ORcp7_219 = -(OMcp7_118*RLcp7_319-OMcp7_318*RLcp7_119);
    ORcp7_319 = OMcp7_118*RLcp7_219-OMcp7_218*RLcp7_119;
    OMcp7_120 = OMcp7_119+ROcp7_419*qd[20];
    OMcp7_220 = OMcp7_219+ROcp7_519*qd[20];
    OMcp7_320 = OMcp7_319+ROcp7_619*qd[20];
    OMcp7_121 = OMcp7_120+ROcp7_720*qd[21];
    OMcp7_221 = OMcp7_220+ROcp7_820*qd[21];
    OMcp7_321 = OMcp7_320+ROcp7_920*qd[21];
    OPcp7_121 = OPcp7_118+ROcp7_110*qdd[19]+ROcp7_419*qdd[20]+ROcp7_720*qdd[21]+qd[19]*(OMcp7_218*ROcp7_310-OMcp7_318*
 ROcp7_210)+qd[20]*(OMcp7_219*ROcp7_619-OMcp7_319*ROcp7_519)+qd[21]*(OMcp7_220*ROcp7_920-OMcp7_320*ROcp7_820);
    OPcp7_221 = OPcp7_218+ROcp7_210*qdd[19]+ROcp7_519*qdd[20]+ROcp7_820*qdd[21]-qd[19]*(OMcp7_118*ROcp7_310-OMcp7_318*
 ROcp7_110)-qd[20]*(OMcp7_119*ROcp7_619-OMcp7_319*ROcp7_419)-qd[21]*(OMcp7_120*ROcp7_920-OMcp7_320*ROcp7_720);
    OPcp7_321 = OPcp7_318+ROcp7_310*qdd[19]+ROcp7_619*qdd[20]+ROcp7_920*qdd[21]+qd[19]*(OMcp7_118*ROcp7_210-OMcp7_218*
 ROcp7_110)+qd[20]*(OMcp7_119*ROcp7_519-OMcp7_219*ROcp7_419)+qd[21]*(OMcp7_120*ROcp7_820-OMcp7_220*ROcp7_720);
    RLcp7_122 = ROcp7_121*s->dpt[1][25]+ROcp7_421*s->dpt[2][25]+ROcp7_720*s->dpt[3][25];
    RLcp7_222 = ROcp7_221*s->dpt[1][25]+ROcp7_521*s->dpt[2][25]+ROcp7_820*s->dpt[3][25];
    RLcp7_322 = ROcp7_321*s->dpt[1][25]+ROcp7_621*s->dpt[2][25]+ROcp7_920*s->dpt[3][25];
    POcp7_122 = RLcp7_110+RLcp7_118+RLcp7_119+RLcp7_12+RLcp7_122+RLcp7_13;
    POcp7_222 = RLcp7_210+RLcp7_218+RLcp7_219+RLcp7_22+RLcp7_222+RLcp7_23;
    POcp7_322 = RLcp7_310+RLcp7_318+RLcp7_319+RLcp7_322+q[4];
    ORcp7_122 = OMcp7_221*RLcp7_322-OMcp7_321*RLcp7_222;
    ORcp7_222 = -(OMcp7_121*RLcp7_322-OMcp7_321*RLcp7_122);
    ORcp7_322 = OMcp7_121*RLcp7_222-OMcp7_221*RLcp7_122;
    VIcp7_122 = ORcp7_110+ORcp7_118+ORcp7_119+ORcp7_12+ORcp7_122+ORcp7_13+qd[2]*C1-qd[3]*S1;
    VIcp7_222 = ORcp7_210+ORcp7_218+ORcp7_219+ORcp7_22+ORcp7_222+ORcp7_23+qd[2]*S1+qd[3]*C1;
    VIcp7_322 = ORcp7_310+ORcp7_318+ORcp7_319+ORcp7_322+qd[4];
    ACcp7_122 = OMcp7_218*ORcp7_319+OMcp7_221*ORcp7_322+OMcp7_27*(ORcp7_310+ORcp7_318)-OMcp7_318*ORcp7_219-OMcp7_321*
 ORcp7_222-OMcp7_37*ORcp7_210-OMcp7_37*ORcp7_218+OPcp7_218*RLcp7_319+OPcp7_221*RLcp7_322+OPcp7_27*RLcp7_310+OPcp7_27*
 RLcp7_318-OPcp7_318*RLcp7_219-OPcp7_321*RLcp7_222-OPcp7_37*RLcp7_210-OPcp7_37*RLcp7_218-ORcp7_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*
 qd[1]*qd[3]*C1-qd[1]*(ORcp7_22+(2.0)*qd[2]*S1);
    ACcp7_222 = -(OMcp7_118*ORcp7_319+OMcp7_121*ORcp7_322+OMcp7_17*(ORcp7_310+ORcp7_318)-OMcp7_318*ORcp7_119-OMcp7_321*
 ORcp7_122-OMcp7_37*ORcp7_110-OMcp7_37*ORcp7_118+OPcp7_118*RLcp7_319+OPcp7_121*RLcp7_322+OPcp7_17*RLcp7_310+OPcp7_17*
 RLcp7_318-OPcp7_318*RLcp7_119-OPcp7_321*RLcp7_122-OPcp7_37*RLcp7_110-OPcp7_37*RLcp7_118-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp7_12+(2.0)
 *qd[2]*C1)-qd[1]*(ORcp7_13-(2.0)*qd[3]*S1));
    ACcp7_322 = qdd[4]+OMcp7_118*ORcp7_219+OMcp7_121*ORcp7_222+OMcp7_17*ORcp7_210+OMcp7_17*ORcp7_218-OMcp7_218*ORcp7_119-
 OMcp7_221*ORcp7_122-OMcp7_27*ORcp7_110-OMcp7_27*ORcp7_118+OPcp7_118*RLcp7_219+OPcp7_121*RLcp7_222+OPcp7_17*RLcp7_210+
 OPcp7_17*RLcp7_218-OPcp7_218*RLcp7_119-OPcp7_221*RLcp7_122-OPcp7_27*RLcp7_110-OPcp7_27*RLcp7_118;
    OMcp7_123 = OMcp7_121+ROcp7_422*qd[23];
    OMcp7_223 = OMcp7_221+ROcp7_522*qd[23];
    OMcp7_323 = OMcp7_321+ROcp7_622*qd[23];
    OPcp7_123 = OPcp7_121+ROcp7_422*qdd[23]+qd[23]*(OMcp7_221*ROcp7_622-OMcp7_321*ROcp7_522);
    OPcp7_223 = OPcp7_221+ROcp7_522*qdd[23]-qd[23]*(OMcp7_121*ROcp7_622-OMcp7_321*ROcp7_422);
    OPcp7_323 = OPcp7_321+ROcp7_622*qdd[23]+qd[23]*(OMcp7_121*ROcp7_522-OMcp7_221*ROcp7_422);

// = = Block_1_0_0_8_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp7_122;
    sens->P[2] = POcp7_222;
    sens->P[3] = POcp7_322;
    sens->R[1][1] = ROcp7_123;
    sens->R[1][2] = ROcp7_223;
    sens->R[1][3] = ROcp7_323;
    sens->R[2][1] = ROcp7_422;
    sens->R[2][2] = ROcp7_522;
    sens->R[2][3] = ROcp7_622;
    sens->R[3][1] = ROcp7_723;
    sens->R[3][2] = ROcp7_823;
    sens->R[3][3] = ROcp7_923;
    sens->V[1] = VIcp7_122;
    sens->V[2] = VIcp7_222;
    sens->V[3] = VIcp7_322;
    sens->OM[1] = OMcp7_123;
    sens->OM[2] = OMcp7_223;
    sens->OM[3] = OMcp7_323;
    sens->A[1] = ACcp7_122;
    sens->A[2] = ACcp7_222;
    sens->A[3] = ACcp7_322;
    sens->OMP[1] = OPcp7_123;
    sens->OMP[2] = OPcp7_223;
    sens->OMP[3] = OPcp7_323;

break;
default:
break;
}


// ====== END Task 1 ====== 


}
 


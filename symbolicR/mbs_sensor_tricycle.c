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
//	==> Generation Date : Wed Oct 12 11:55:11 2016
//
//	==> Project name : tricycle
//	==> using XML input file 
//
//	==> Number of joints : 30
//
//	==> Function : F 6 : Sensors Kinematical Informations (sens) 
//	==> Flops complexity : 2439
//
//	==> Generation Time :  0.040 seconds
//	==> Post-Processing :  0.040 seconds
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

// = = Block_0_0_0_4_0_2 = = 
 
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

// = = Block_1_0_0_3_0_6 = = 
 
// Sensor Kinematics 


    ROcp2_418 = ROcp2_46*C18+ROcp2_710*S18;
    ROcp2_518 = ROcp2_56*C18+ROcp2_810*S18;
    ROcp2_618 = ROcp2_910*S18+C18*S6;
    ROcp2_718 = -(ROcp2_46*S18-ROcp2_710*C18);
    ROcp2_818 = -(ROcp2_56*S18-ROcp2_810*C18);
    ROcp2_918 = ROcp2_910*C18-S18*S6;
    ROcp2_419 = ROcp2_418*C19+ROcp2_718*S19;
    ROcp2_519 = ROcp2_518*C19+ROcp2_818*S19;
    ROcp2_619 = ROcp2_618*C19+ROcp2_918*S19;
    ROcp2_719 = -(ROcp2_418*S19-ROcp2_718*C19);
    ROcp2_819 = -(ROcp2_518*S19-ROcp2_818*C19);
    ROcp2_919 = -(ROcp2_618*S19-ROcp2_918*C19);
    ROcp2_120 = ROcp2_110*C20-ROcp2_719*S20;
    ROcp2_220 = ROcp2_210*C20-ROcp2_819*S20;
    ROcp2_320 = ROcp2_310*C20-ROcp2_919*S20;
    ROcp2_720 = ROcp2_110*S20+ROcp2_719*C20;
    ROcp2_820 = ROcp2_210*S20+ROcp2_819*C20;
    ROcp2_920 = ROcp2_310*S20+ROcp2_919*C20;
    ROcp2_121 = ROcp2_120*C21+ROcp2_419*S21;
    ROcp2_221 = ROcp2_220*C21+ROcp2_519*S21;
    ROcp2_321 = ROcp2_320*C21+ROcp2_619*S21;
    ROcp2_421 = -(ROcp2_120*S21-ROcp2_419*C21);
    ROcp2_521 = -(ROcp2_220*S21-ROcp2_519*C21);
    ROcp2_621 = -(ROcp2_320*S21-ROcp2_619*C21);
    RLcp2_118 = ROcp2_46*s->dpt[2][12]+ROcp2_710*s->dpt[3][12];
    RLcp2_218 = ROcp2_56*s->dpt[2][12]+ROcp2_810*s->dpt[3][12];
    RLcp2_318 = ROcp2_910*s->dpt[3][12]+s->dpt[2][12]*S6;
    OMcp2_118 = OMcp2_17+ROcp2_110*qd[18];
    OMcp2_218 = OMcp2_27+ROcp2_210*qd[18];
    OMcp2_318 = OMcp2_37+ROcp2_310*qd[18];
    ORcp2_118 = OMcp2_27*RLcp2_318-OMcp2_37*RLcp2_218;
    ORcp2_218 = -(OMcp2_17*RLcp2_318-OMcp2_37*RLcp2_118);
    ORcp2_318 = OMcp2_17*RLcp2_218-OMcp2_27*RLcp2_118;
    OPcp2_118 = OPcp2_17+ROcp2_110*qdd[18]+qd[18]*(OMcp2_27*ROcp2_310-OMcp2_37*ROcp2_210);
    OPcp2_218 = OPcp2_27+ROcp2_210*qdd[18]-qd[18]*(OMcp2_17*ROcp2_310-OMcp2_37*ROcp2_110);
    OPcp2_318 = OPcp2_37+ROcp2_310*qdd[18]+qd[18]*(OMcp2_17*ROcp2_210-OMcp2_27*ROcp2_110);
    RLcp2_119 = ROcp2_418*s->dpt[2][22];
    RLcp2_219 = ROcp2_518*s->dpt[2][22];
    RLcp2_319 = ROcp2_618*s->dpt[2][22];
    OMcp2_119 = OMcp2_118+ROcp2_110*qd[19];
    OMcp2_219 = OMcp2_218+ROcp2_210*qd[19];
    OMcp2_319 = OMcp2_318+ROcp2_310*qd[19];
    ORcp2_119 = OMcp2_218*RLcp2_319-OMcp2_318*RLcp2_219;
    ORcp2_219 = -(OMcp2_118*RLcp2_319-OMcp2_318*RLcp2_119);
    ORcp2_319 = OMcp2_118*RLcp2_219-OMcp2_218*RLcp2_119;
    OMcp2_120 = OMcp2_119+ROcp2_419*qd[20];
    OMcp2_220 = OMcp2_219+ROcp2_519*qd[20];
    OMcp2_320 = OMcp2_319+ROcp2_619*qd[20];
    OMcp2_121 = OMcp2_120+ROcp2_720*qd[21];
    OMcp2_221 = OMcp2_220+ROcp2_820*qd[21];
    OMcp2_321 = OMcp2_320+ROcp2_920*qd[21];
    OPcp2_121 = OPcp2_118+ROcp2_110*qdd[19]+ROcp2_419*qdd[20]+ROcp2_720*qdd[21]+qd[19]*(OMcp2_218*ROcp2_310-OMcp2_318*
 ROcp2_210)+qd[20]*(OMcp2_219*ROcp2_619-OMcp2_319*ROcp2_519)+qd[21]*(OMcp2_220*ROcp2_920-OMcp2_320*ROcp2_820);
    OPcp2_221 = OPcp2_218+ROcp2_210*qdd[19]+ROcp2_519*qdd[20]+ROcp2_820*qdd[21]-qd[19]*(OMcp2_118*ROcp2_310-OMcp2_318*
 ROcp2_110)-qd[20]*(OMcp2_119*ROcp2_619-OMcp2_319*ROcp2_419)-qd[21]*(OMcp2_120*ROcp2_920-OMcp2_320*ROcp2_720);
    OPcp2_321 = OPcp2_318+ROcp2_310*qdd[19]+ROcp2_619*qdd[20]+ROcp2_920*qdd[21]+qd[19]*(OMcp2_118*ROcp2_210-OMcp2_218*
 ROcp2_110)+qd[20]*(OMcp2_119*ROcp2_519-OMcp2_219*ROcp2_419)+qd[21]*(OMcp2_120*ROcp2_820-OMcp2_220*ROcp2_720);
    RLcp2_147 = ROcp2_121*s->dpt[1][25]+ROcp2_421*s->dpt[2][25]+ROcp2_720*s->dpt[3][25];
    RLcp2_247 = ROcp2_221*s->dpt[1][25]+ROcp2_521*s->dpt[2][25]+ROcp2_820*s->dpt[3][25];
    RLcp2_347 = ROcp2_321*s->dpt[1][25]+ROcp2_621*s->dpt[2][25]+ROcp2_920*s->dpt[3][25];
    POcp2_147 = RLcp2_110+RLcp2_118+RLcp2_119+RLcp2_12+RLcp2_13+RLcp2_147;
    POcp2_247 = RLcp2_210+RLcp2_218+RLcp2_219+RLcp2_22+RLcp2_23+RLcp2_247;
    POcp2_347 = RLcp2_310+RLcp2_318+RLcp2_319+RLcp2_347+q[4];
    JTcp2_147_1 = -(RLcp2_210+RLcp2_218+RLcp2_219+RLcp2_22+RLcp2_23+RLcp2_247);
    JTcp2_247_1 = RLcp2_110+RLcp2_118+RLcp2_119+RLcp2_12+RLcp2_13+RLcp2_147;
    JTcp2_147_5 = -(RLcp2_210+RLcp2_218+RLcp2_219+RLcp2_247);
    JTcp2_247_5 = RLcp2_110+RLcp2_118+RLcp2_119+RLcp2_147;
    JTcp2_147_6 = S1p5*(RLcp2_310+RLcp2_318+RLcp2_319+RLcp2_347);
    JTcp2_247_6 = -C1p5*(RLcp2_310+RLcp2_318+RLcp2_319+RLcp2_347);
    JTcp2_347_6 = C1p5*(RLcp2_210+RLcp2_218+RLcp2_219+RLcp2_247)-S1p5*(RLcp2_110+RLcp2_118)-S1p5*(RLcp2_119+RLcp2_147);
    JTcp2_147_7 = ROcp2_56*(RLcp2_310+RLcp2_318+RLcp2_319+RLcp2_347)-S6*(RLcp2_210+RLcp2_218)-S6*(RLcp2_219+RLcp2_247);
    JTcp2_247_7 = -(ROcp2_46*(RLcp2_310+RLcp2_318+RLcp2_319+RLcp2_347)-S6*(RLcp2_110+RLcp2_118)-S6*(RLcp2_119+RLcp2_147));
    JTcp2_347_7 = ROcp2_46*(RLcp2_210+RLcp2_218+RLcp2_219+RLcp2_247)-ROcp2_56*(RLcp2_110+RLcp2_118)-ROcp2_56*(RLcp2_119+
 RLcp2_147);
    JTcp2_147_8 = ROcp2_56*(RLcp2_318+RLcp2_319)-S6*(RLcp2_218+RLcp2_219)-RLcp2_247*S6+RLcp2_347*ROcp2_56;
    JTcp2_247_8 = RLcp2_147*S6-RLcp2_347*ROcp2_46-ROcp2_46*(RLcp2_318+RLcp2_319)+S6*(RLcp2_118+RLcp2_119);
    JTcp2_347_8 = ROcp2_46*(RLcp2_218+RLcp2_219)-ROcp2_56*(RLcp2_118+RLcp2_119)-RLcp2_147*ROcp2_56+RLcp2_247*ROcp2_46;
    JTcp2_147_9 = ROcp2_210*(RLcp2_319+RLcp2_347)-ROcp2_310*(RLcp2_219+RLcp2_247);
    JTcp2_247_9 = -(ROcp2_110*(RLcp2_319+RLcp2_347)-ROcp2_310*(RLcp2_119+RLcp2_147));
    JTcp2_347_9 = ROcp2_110*(RLcp2_219+RLcp2_247)-ROcp2_210*(RLcp2_119+RLcp2_147);
    JTcp2_147_10 = -(RLcp2_247*ROcp2_310-RLcp2_347*ROcp2_210);
    JTcp2_247_10 = RLcp2_147*ROcp2_310-RLcp2_347*ROcp2_110;
    JTcp2_347_10 = -(RLcp2_147*ROcp2_210-RLcp2_247*ROcp2_110);
    JTcp2_147_11 = -(RLcp2_247*ROcp2_619-RLcp2_347*ROcp2_519);
    JTcp2_247_11 = RLcp2_147*ROcp2_619-RLcp2_347*ROcp2_419;
    JTcp2_347_11 = -(RLcp2_147*ROcp2_519-RLcp2_247*ROcp2_419);
    JTcp2_147_12 = -(RLcp2_247*ROcp2_920-RLcp2_347*ROcp2_820);
    JTcp2_247_12 = RLcp2_147*ROcp2_920-RLcp2_347*ROcp2_720;
    JTcp2_347_12 = -(RLcp2_147*ROcp2_820-RLcp2_247*ROcp2_720);
    ORcp2_147 = OMcp2_221*RLcp2_347-OMcp2_321*RLcp2_247;
    ORcp2_247 = -(OMcp2_121*RLcp2_347-OMcp2_321*RLcp2_147);
    ORcp2_347 = OMcp2_121*RLcp2_247-OMcp2_221*RLcp2_147;
    VIcp2_147 = ORcp2_110+ORcp2_118+ORcp2_119+ORcp2_12+ORcp2_13+ORcp2_147+qd[2]*C1-qd[3]*S1;
    VIcp2_247 = ORcp2_210+ORcp2_218+ORcp2_219+ORcp2_22+ORcp2_23+ORcp2_247+qd[2]*S1+qd[3]*C1;
    VIcp2_347 = ORcp2_310+ORcp2_318+ORcp2_319+ORcp2_347+qd[4];
    ACcp2_147 = OMcp2_218*ORcp2_319+OMcp2_221*ORcp2_347+OMcp2_27*(ORcp2_310+ORcp2_318)-OMcp2_318*ORcp2_219-OMcp2_321*
 ORcp2_247-OMcp2_37*ORcp2_210-OMcp2_37*ORcp2_218+OPcp2_218*RLcp2_319+OPcp2_221*RLcp2_347+OPcp2_27*RLcp2_310+OPcp2_27*
 RLcp2_318-OPcp2_318*RLcp2_219-OPcp2_321*RLcp2_247-OPcp2_37*RLcp2_210-OPcp2_37*RLcp2_218-ORcp2_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*
 qd[1]*qd[3]*C1-qd[1]*(ORcp2_22+(2.0)*qd[2]*S1);
    ACcp2_247 = -(OMcp2_118*ORcp2_319+OMcp2_121*ORcp2_347+OMcp2_17*(ORcp2_310+ORcp2_318)-OMcp2_318*ORcp2_119-OMcp2_321*
 ORcp2_147-OMcp2_37*ORcp2_110-OMcp2_37*ORcp2_118+OPcp2_118*RLcp2_319+OPcp2_121*RLcp2_347+OPcp2_17*RLcp2_310+OPcp2_17*
 RLcp2_318-OPcp2_318*RLcp2_119-OPcp2_321*RLcp2_147-OPcp2_37*RLcp2_110-OPcp2_37*RLcp2_118-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp2_12+(2.0)
 *qd[2]*C1)-qd[1]*(ORcp2_13-(2.0)*qd[3]*S1));
    ACcp2_347 = qdd[4]+OMcp2_118*ORcp2_219+OMcp2_121*ORcp2_247+OMcp2_17*ORcp2_210+OMcp2_17*ORcp2_218-OMcp2_218*ORcp2_119-
 OMcp2_221*ORcp2_147-OMcp2_27*ORcp2_110-OMcp2_27*ORcp2_118+OPcp2_118*RLcp2_219+OPcp2_121*RLcp2_247+OPcp2_17*RLcp2_210+
 OPcp2_17*RLcp2_218-OPcp2_218*RLcp2_119-OPcp2_221*RLcp2_147-OPcp2_27*RLcp2_110-OPcp2_27*RLcp2_118;

// = = Block_1_0_0_3_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp2_147;
    sens->P[2] = POcp2_247;
    sens->P[3] = POcp2_347;
    sens->R[1][1] = ROcp2_121;
    sens->R[1][2] = ROcp2_221;
    sens->R[1][3] = ROcp2_321;
    sens->R[2][1] = ROcp2_421;
    sens->R[2][2] = ROcp2_521;
    sens->R[2][3] = ROcp2_621;
    sens->R[3][1] = ROcp2_720;
    sens->R[3][2] = ROcp2_820;
    sens->R[3][3] = ROcp2_920;
    sens->V[1] = VIcp2_147;
    sens->V[2] = VIcp2_247;
    sens->V[3] = VIcp2_347;
    sens->OM[1] = OMcp2_121;
    sens->OM[2] = OMcp2_221;
    sens->OM[3] = OMcp2_321;
    sens->J[1][1] = JTcp2_147_1;
    sens->J[1][2] = C1;
    sens->J[1][3] = -S1;
    sens->J[1][5] = JTcp2_147_5;
    sens->J[1][6] = JTcp2_147_6;
    sens->J[1][7] = JTcp2_147_7;
    sens->J[1][10] = JTcp2_147_8;
    sens->J[1][18] = JTcp2_147_9;
    sens->J[1][19] = JTcp2_147_10;
    sens->J[1][20] = JTcp2_147_11;
    sens->J[1][21] = JTcp2_147_12;
    sens->J[2][1] = JTcp2_247_1;
    sens->J[2][2] = S1;
    sens->J[2][3] = C1;
    sens->J[2][5] = JTcp2_247_5;
    sens->J[2][6] = JTcp2_247_6;
    sens->J[2][7] = JTcp2_247_7;
    sens->J[2][10] = JTcp2_247_8;
    sens->J[2][18] = JTcp2_247_9;
    sens->J[2][19] = JTcp2_247_10;
    sens->J[2][20] = JTcp2_247_11;
    sens->J[2][21] = JTcp2_247_12;
    sens->J[3][4] = (1.0);
    sens->J[3][6] = JTcp2_347_6;
    sens->J[3][7] = JTcp2_347_7;
    sens->J[3][10] = JTcp2_347_8;
    sens->J[3][18] = JTcp2_347_9;
    sens->J[3][19] = JTcp2_347_10;
    sens->J[3][20] = JTcp2_347_11;
    sens->J[3][21] = JTcp2_347_12;
    sens->J[4][6] = C1p5;
    sens->J[4][7] = ROcp2_46;
    sens->J[4][10] = ROcp2_46;
    sens->J[4][18] = ROcp2_110;
    sens->J[4][19] = ROcp2_110;
    sens->J[4][20] = ROcp2_419;
    sens->J[4][21] = ROcp2_720;
    sens->J[5][6] = S1p5;
    sens->J[5][7] = ROcp2_56;
    sens->J[5][10] = ROcp2_56;
    sens->J[5][18] = ROcp2_210;
    sens->J[5][19] = ROcp2_210;
    sens->J[5][20] = ROcp2_519;
    sens->J[5][21] = ROcp2_820;
    sens->J[6][1] = (1.0);
    sens->J[6][5] = (1.0);
    sens->J[6][7] = S6;
    sens->J[6][10] = S6;
    sens->J[6][18] = ROcp2_310;
    sens->J[6][19] = ROcp2_310;
    sens->J[6][20] = ROcp2_619;
    sens->J[6][21] = ROcp2_920;
    sens->A[1] = ACcp2_147;
    sens->A[2] = ACcp2_247;
    sens->A[3] = ACcp2_347;
    sens->OMP[1] = OPcp2_121;
    sens->OMP[2] = OPcp2_221;
    sens->OMP[3] = OPcp2_321;
 
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

// = = Block_1_0_0_4_0_2 = = 
 
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
    OMcp3_18 = OMcp3_17+ROcp3_46*qd[8];
    OMcp3_28 = OMcp3_27+ROcp3_56*qd[8];
    OMcp3_38 = OMcp3_37+qd[8]*S6;
    ORcp3_18 = OMcp3_27*RLcp3_38-OMcp3_37*RLcp3_28;
    ORcp3_28 = -(OMcp3_17*RLcp3_38-OMcp3_37*RLcp3_18);
    ORcp3_38 = OMcp3_17*RLcp3_28-OMcp3_27*RLcp3_18;
    OPcp3_18 = OPcp3_17+ROcp3_46*qdd[8]+qd[8]*(OMcp3_27*S6-OMcp3_37*ROcp3_56);
    OPcp3_28 = OPcp3_27+ROcp3_56*qdd[8]-qd[8]*(OMcp3_17*S6-OMcp3_37*ROcp3_46);
    OPcp3_38 = OPcp3_37+qdd[8]*S6+qd[8]*(OMcp3_17*ROcp3_56-OMcp3_27*ROcp3_46);
    RLcp3_19 = ROcp3_18*s->dpt[1][8];
    RLcp3_29 = ROcp3_28*s->dpt[1][8];
    RLcp3_39 = -s->dpt[1][8]*S7p8*C6;
    POcp3_19 = RLcp3_12+RLcp3_13+RLcp3_18+RLcp3_19;
    POcp3_29 = RLcp3_22+RLcp3_23+RLcp3_28+RLcp3_29;
    POcp3_39 = RLcp3_38+RLcp3_39+q[4];
    OMcp3_19 = OMcp3_18+ROcp3_46*qd[9];
    OMcp3_29 = OMcp3_28+ROcp3_56*qd[9];
    OMcp3_39 = OMcp3_38+qd[9]*S6;
    ORcp3_19 = OMcp3_28*RLcp3_39-OMcp3_38*RLcp3_29;
    ORcp3_29 = -(OMcp3_18*RLcp3_39-OMcp3_38*RLcp3_19);
    ORcp3_39 = OMcp3_18*RLcp3_29-OMcp3_28*RLcp3_19;
    VIcp3_19 = ORcp3_12+ORcp3_13+ORcp3_18+ORcp3_19+qd[2]*C1-qd[3]*S1;
    VIcp3_29 = ORcp3_22+ORcp3_23+ORcp3_28+ORcp3_29+qd[2]*S1+qd[3]*C1;
    VIcp3_39 = ORcp3_38+ORcp3_39+qd[4];
    OPcp3_19 = OPcp3_18+ROcp3_46*qdd[9]+qd[9]*(OMcp3_28*S6-OMcp3_38*ROcp3_56);
    OPcp3_29 = OPcp3_28+ROcp3_56*qdd[9]-qd[9]*(OMcp3_18*S6-OMcp3_38*ROcp3_46);
    OPcp3_39 = OPcp3_38+qdd[9]*S6+qd[9]*(OMcp3_18*ROcp3_56-OMcp3_28*ROcp3_46);
    ACcp3_19 = OMcp3_27*ORcp3_38+OMcp3_28*ORcp3_39-OMcp3_37*ORcp3_28-OMcp3_38*ORcp3_29+OPcp3_27*RLcp3_38+OPcp3_28*RLcp3_39
 -OPcp3_37*RLcp3_28-OPcp3_38*RLcp3_29-ORcp3_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp3_22+(2.0)*qd[2]*S1);
    ACcp3_29 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp3_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp3_13-(2.0)*qd[3]*S1)-OMcp3_17*ORcp3_38+OMcp3_37*
 ORcp3_18-OPcp3_17*RLcp3_38+OPcp3_37*RLcp3_18-OMcp3_18*ORcp3_39+OMcp3_38*ORcp3_19-OPcp3_18*RLcp3_39+OPcp3_38*RLcp3_19;
    ACcp3_39 = qdd[4]+OMcp3_17*ORcp3_28+OMcp3_18*ORcp3_29-OMcp3_27*ORcp3_18-OMcp3_28*ORcp3_19+OPcp3_17*RLcp3_28+OPcp3_18*
 RLcp3_29-OPcp3_27*RLcp3_18-OPcp3_28*RLcp3_19;

// = = Block_1_0_0_4_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp3_19;
    sens->P[2] = POcp3_29;
    sens->P[3] = POcp3_39;
    sens->R[1][1] = ROcp3_19;
    sens->R[1][2] = ROcp3_29;
    sens->R[1][3] = ROcp3_39;
    sens->R[2][1] = ROcp3_46;
    sens->R[2][2] = ROcp3_56;
    sens->R[2][3] = S6;
    sens->R[3][1] = ROcp3_79;
    sens->R[3][2] = ROcp3_89;
    sens->R[3][3] = ROcp3_99;
    sens->V[1] = VIcp3_19;
    sens->V[2] = VIcp3_29;
    sens->V[3] = VIcp3_39;
    sens->OM[1] = OMcp3_19;
    sens->OM[2] = OMcp3_29;
    sens->OM[3] = OMcp3_39;
    sens->A[1] = ACcp3_19;
    sens->A[2] = ACcp3_29;
    sens->A[3] = ACcp3_39;
    sens->OMP[1] = OPcp3_19;
    sens->OMP[2] = OPcp3_29;
    sens->OMP[3] = OPcp3_39;
 
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

// = = Block_1_0_0_5_0_5 = = 
 
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
    OMcp4_112 = OMcp4_17+ROcp4_110*qd[12];
    OMcp4_212 = OMcp4_27+ROcp4_210*qd[12];
    OMcp4_312 = OMcp4_37+ROcp4_310*qd[12];
    ORcp4_112 = OMcp4_27*RLcp4_312-OMcp4_37*RLcp4_212;
    ORcp4_212 = -(OMcp4_17*RLcp4_312-OMcp4_37*RLcp4_112);
    ORcp4_312 = OMcp4_17*RLcp4_212-OMcp4_27*RLcp4_112;
    OPcp4_112 = OPcp4_17+ROcp4_110*qdd[12]+qd[12]*(OMcp4_27*ROcp4_310-OMcp4_37*ROcp4_210);
    OPcp4_212 = OPcp4_27+ROcp4_210*qdd[12]-qd[12]*(OMcp4_17*ROcp4_310-OMcp4_37*ROcp4_110);
    OPcp4_312 = OPcp4_37+ROcp4_310*qdd[12]+qd[12]*(OMcp4_17*ROcp4_210-OMcp4_27*ROcp4_110);
    RLcp4_113 = ROcp4_412*s->dpt[2][16];
    RLcp4_213 = ROcp4_512*s->dpt[2][16];
    RLcp4_313 = ROcp4_612*s->dpt[2][16];
    OMcp4_113 = OMcp4_112+ROcp4_110*qd[13];
    OMcp4_213 = OMcp4_212+ROcp4_210*qd[13];
    OMcp4_313 = OMcp4_312+ROcp4_310*qd[13];
    ORcp4_113 = OMcp4_212*RLcp4_313-OMcp4_312*RLcp4_213;
    ORcp4_213 = -(OMcp4_112*RLcp4_313-OMcp4_312*RLcp4_113);
    ORcp4_313 = OMcp4_112*RLcp4_213-OMcp4_212*RLcp4_113;
    OMcp4_114 = OMcp4_113+ROcp4_413*qd[14];
    OMcp4_214 = OMcp4_213+ROcp4_513*qd[14];
    OMcp4_314 = OMcp4_313+ROcp4_613*qd[14];
    OMcp4_115 = OMcp4_114+ROcp4_714*qd[15];
    OMcp4_215 = OMcp4_214+ROcp4_814*qd[15];
    OMcp4_315 = OMcp4_314+ROcp4_914*qd[15];
    OPcp4_115 = OPcp4_112+ROcp4_110*qdd[13]+ROcp4_413*qdd[14]+ROcp4_714*qdd[15]+qd[13]*(OMcp4_212*ROcp4_310-OMcp4_312*
 ROcp4_210)+qd[14]*(OMcp4_213*ROcp4_613-OMcp4_313*ROcp4_513)+qd[15]*(OMcp4_214*ROcp4_914-OMcp4_314*ROcp4_814);
    OPcp4_215 = OPcp4_212+ROcp4_210*qdd[13]+ROcp4_513*qdd[14]+ROcp4_814*qdd[15]-qd[13]*(OMcp4_112*ROcp4_310-OMcp4_312*
 ROcp4_110)-qd[14]*(OMcp4_113*ROcp4_613-OMcp4_313*ROcp4_413)-qd[15]*(OMcp4_114*ROcp4_914-OMcp4_314*ROcp4_714);
    OPcp4_315 = OPcp4_312+ROcp4_310*qdd[13]+ROcp4_613*qdd[14]+ROcp4_914*qdd[15]+qd[13]*(OMcp4_112*ROcp4_210-OMcp4_212*
 ROcp4_110)+qd[14]*(OMcp4_113*ROcp4_513-OMcp4_213*ROcp4_413)+qd[15]*(OMcp4_114*ROcp4_814-OMcp4_214*ROcp4_714);
    RLcp4_116 = ROcp4_115*s->dpt[1][19]+ROcp4_415*s->dpt[2][19]+ROcp4_714*s->dpt[3][19];
    RLcp4_216 = ROcp4_215*s->dpt[1][19]+ROcp4_515*s->dpt[2][19]+ROcp4_814*s->dpt[3][19];
    RLcp4_316 = ROcp4_315*s->dpt[1][19]+ROcp4_615*s->dpt[2][19]+ROcp4_914*s->dpt[3][19];
    POcp4_116 = RLcp4_110+RLcp4_112+RLcp4_113+RLcp4_116+RLcp4_12+RLcp4_13;
    POcp4_216 = RLcp4_210+RLcp4_212+RLcp4_213+RLcp4_216+RLcp4_22+RLcp4_23;
    POcp4_316 = RLcp4_310+RLcp4_312+RLcp4_313+RLcp4_316+q[4];
    ORcp4_116 = OMcp4_215*RLcp4_316-OMcp4_315*RLcp4_216;
    ORcp4_216 = -(OMcp4_115*RLcp4_316-OMcp4_315*RLcp4_116);
    ORcp4_316 = OMcp4_115*RLcp4_216-OMcp4_215*RLcp4_116;
    VIcp4_116 = ORcp4_110+ORcp4_112+ORcp4_113+ORcp4_116+ORcp4_12+ORcp4_13+qd[2]*C1-qd[3]*S1;
    VIcp4_216 = ORcp4_210+ORcp4_212+ORcp4_213+ORcp4_216+ORcp4_22+ORcp4_23+qd[2]*S1+qd[3]*C1;
    VIcp4_316 = ORcp4_310+ORcp4_312+ORcp4_313+ORcp4_316+qd[4];
    ACcp4_116 = OMcp4_212*ORcp4_313+OMcp4_215*ORcp4_316+OMcp4_27*(ORcp4_310+ORcp4_312)-OMcp4_312*ORcp4_213-OMcp4_315*
 ORcp4_216-OMcp4_37*ORcp4_210-OMcp4_37*ORcp4_212+OPcp4_212*RLcp4_313+OPcp4_215*RLcp4_316+OPcp4_27*RLcp4_310+OPcp4_27*
 RLcp4_312-OPcp4_312*RLcp4_213-OPcp4_315*RLcp4_216-OPcp4_37*RLcp4_210-OPcp4_37*RLcp4_212-ORcp4_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*
 qd[1]*qd[3]*C1-qd[1]*(ORcp4_22+(2.0)*qd[2]*S1);
    ACcp4_216 = -(OMcp4_112*ORcp4_313+OMcp4_115*ORcp4_316+OMcp4_17*(ORcp4_310+ORcp4_312)-OMcp4_312*ORcp4_113-OMcp4_315*
 ORcp4_116-OMcp4_37*ORcp4_110-OMcp4_37*ORcp4_112+OPcp4_112*RLcp4_313+OPcp4_115*RLcp4_316+OPcp4_17*RLcp4_310+OPcp4_17*
 RLcp4_312-OPcp4_312*RLcp4_113-OPcp4_315*RLcp4_116-OPcp4_37*RLcp4_110-OPcp4_37*RLcp4_112-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp4_12+(2.0)
 *qd[2]*C1)-qd[1]*(ORcp4_13-(2.0)*qd[3]*S1));
    ACcp4_316 = qdd[4]+OMcp4_112*ORcp4_213+OMcp4_115*ORcp4_216+OMcp4_17*ORcp4_210+OMcp4_17*ORcp4_212-OMcp4_212*ORcp4_113-
 OMcp4_215*ORcp4_116-OMcp4_27*ORcp4_110-OMcp4_27*ORcp4_112+OPcp4_112*RLcp4_213+OPcp4_115*RLcp4_216+OPcp4_17*RLcp4_210+
 OPcp4_17*RLcp4_212-OPcp4_212*RLcp4_113-OPcp4_215*RLcp4_116-OPcp4_27*RLcp4_110-OPcp4_27*RLcp4_112;
    OMcp4_117 = OMcp4_115+ROcp4_416*qd[17];
    OMcp4_217 = OMcp4_215+ROcp4_516*qd[17];
    OMcp4_317 = OMcp4_315+ROcp4_616*qd[17];
    OPcp4_117 = OPcp4_115+ROcp4_416*qdd[17]+qd[17]*(OMcp4_215*ROcp4_616-OMcp4_315*ROcp4_516);
    OPcp4_217 = OPcp4_215+ROcp4_516*qdd[17]-qd[17]*(OMcp4_115*ROcp4_616-OMcp4_315*ROcp4_416);
    OPcp4_317 = OPcp4_315+ROcp4_616*qdd[17]+qd[17]*(OMcp4_115*ROcp4_516-OMcp4_215*ROcp4_416);

// = = Block_1_0_0_5_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp4_116;
    sens->P[2] = POcp4_216;
    sens->P[3] = POcp4_316;
    sens->R[1][1] = ROcp4_117;
    sens->R[1][2] = ROcp4_217;
    sens->R[1][3] = ROcp4_317;
    sens->R[2][1] = ROcp4_416;
    sens->R[2][2] = ROcp4_516;
    sens->R[2][3] = ROcp4_616;
    sens->R[3][1] = ROcp4_717;
    sens->R[3][2] = ROcp4_817;
    sens->R[3][3] = ROcp4_917;
    sens->V[1] = VIcp4_116;
    sens->V[2] = VIcp4_216;
    sens->V[3] = VIcp4_316;
    sens->OM[1] = OMcp4_117;
    sens->OM[2] = OMcp4_217;
    sens->OM[3] = OMcp4_317;
    sens->A[1] = ACcp4_116;
    sens->A[2] = ACcp4_216;
    sens->A[3] = ACcp4_316;
    sens->OMP[1] = OPcp4_117;
    sens->OMP[2] = OPcp4_217;
    sens->OMP[3] = OPcp4_317;
 
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

// = = Block_1_0_0_6_0_3 = = 
 
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

// = = Block_1_0_0_6_0_6 = = 
 
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
    OMcp5_118 = OMcp5_17+ROcp5_110*qd[18];
    OMcp5_218 = OMcp5_27+ROcp5_210*qd[18];
    OMcp5_318 = OMcp5_37+ROcp5_310*qd[18];
    ORcp5_118 = OMcp5_27*RLcp5_318-OMcp5_37*RLcp5_218;
    ORcp5_218 = -(OMcp5_17*RLcp5_318-OMcp5_37*RLcp5_118);
    ORcp5_318 = OMcp5_17*RLcp5_218-OMcp5_27*RLcp5_118;
    OPcp5_118 = OPcp5_17+ROcp5_110*qdd[18]+qd[18]*(OMcp5_27*ROcp5_310-OMcp5_37*ROcp5_210);
    OPcp5_218 = OPcp5_27+ROcp5_210*qdd[18]-qd[18]*(OMcp5_17*ROcp5_310-OMcp5_37*ROcp5_110);
    OPcp5_318 = OPcp5_37+ROcp5_310*qdd[18]+qd[18]*(OMcp5_17*ROcp5_210-OMcp5_27*ROcp5_110);
    RLcp5_119 = ROcp5_418*s->dpt[2][22];
    RLcp5_219 = ROcp5_518*s->dpt[2][22];
    RLcp5_319 = ROcp5_618*s->dpt[2][22];
    OMcp5_119 = OMcp5_118+ROcp5_110*qd[19];
    OMcp5_219 = OMcp5_218+ROcp5_210*qd[19];
    OMcp5_319 = OMcp5_318+ROcp5_310*qd[19];
    ORcp5_119 = OMcp5_218*RLcp5_319-OMcp5_318*RLcp5_219;
    ORcp5_219 = -(OMcp5_118*RLcp5_319-OMcp5_318*RLcp5_119);
    ORcp5_319 = OMcp5_118*RLcp5_219-OMcp5_218*RLcp5_119;
    OMcp5_120 = OMcp5_119+ROcp5_419*qd[20];
    OMcp5_220 = OMcp5_219+ROcp5_519*qd[20];
    OMcp5_320 = OMcp5_319+ROcp5_619*qd[20];
    OMcp5_121 = OMcp5_120+ROcp5_720*qd[21];
    OMcp5_221 = OMcp5_220+ROcp5_820*qd[21];
    OMcp5_321 = OMcp5_320+ROcp5_920*qd[21];
    OPcp5_121 = OPcp5_118+ROcp5_110*qdd[19]+ROcp5_419*qdd[20]+ROcp5_720*qdd[21]+qd[19]*(OMcp5_218*ROcp5_310-OMcp5_318*
 ROcp5_210)+qd[20]*(OMcp5_219*ROcp5_619-OMcp5_319*ROcp5_519)+qd[21]*(OMcp5_220*ROcp5_920-OMcp5_320*ROcp5_820);
    OPcp5_221 = OPcp5_218+ROcp5_210*qdd[19]+ROcp5_519*qdd[20]+ROcp5_820*qdd[21]-qd[19]*(OMcp5_118*ROcp5_310-OMcp5_318*
 ROcp5_110)-qd[20]*(OMcp5_119*ROcp5_619-OMcp5_319*ROcp5_419)-qd[21]*(OMcp5_120*ROcp5_920-OMcp5_320*ROcp5_720);
    OPcp5_321 = OPcp5_318+ROcp5_310*qdd[19]+ROcp5_619*qdd[20]+ROcp5_920*qdd[21]+qd[19]*(OMcp5_118*ROcp5_210-OMcp5_218*
 ROcp5_110)+qd[20]*(OMcp5_119*ROcp5_519-OMcp5_219*ROcp5_419)+qd[21]*(OMcp5_120*ROcp5_820-OMcp5_220*ROcp5_720);
    RLcp5_122 = ROcp5_121*s->dpt[1][25]+ROcp5_421*s->dpt[2][25]+ROcp5_720*s->dpt[3][25];
    RLcp5_222 = ROcp5_221*s->dpt[1][25]+ROcp5_521*s->dpt[2][25]+ROcp5_820*s->dpt[3][25];
    RLcp5_322 = ROcp5_321*s->dpt[1][25]+ROcp5_621*s->dpt[2][25]+ROcp5_920*s->dpt[3][25];
    POcp5_122 = RLcp5_110+RLcp5_118+RLcp5_119+RLcp5_12+RLcp5_122+RLcp5_13;
    POcp5_222 = RLcp5_210+RLcp5_218+RLcp5_219+RLcp5_22+RLcp5_222+RLcp5_23;
    POcp5_322 = RLcp5_310+RLcp5_318+RLcp5_319+RLcp5_322+q[4];
    ORcp5_122 = OMcp5_221*RLcp5_322-OMcp5_321*RLcp5_222;
    ORcp5_222 = -(OMcp5_121*RLcp5_322-OMcp5_321*RLcp5_122);
    ORcp5_322 = OMcp5_121*RLcp5_222-OMcp5_221*RLcp5_122;
    VIcp5_122 = ORcp5_110+ORcp5_118+ORcp5_119+ORcp5_12+ORcp5_122+ORcp5_13+qd[2]*C1-qd[3]*S1;
    VIcp5_222 = ORcp5_210+ORcp5_218+ORcp5_219+ORcp5_22+ORcp5_222+ORcp5_23+qd[2]*S1+qd[3]*C1;
    VIcp5_322 = ORcp5_310+ORcp5_318+ORcp5_319+ORcp5_322+qd[4];
    ACcp5_122 = OMcp5_218*ORcp5_319+OMcp5_221*ORcp5_322+OMcp5_27*(ORcp5_310+ORcp5_318)-OMcp5_318*ORcp5_219-OMcp5_321*
 ORcp5_222-OMcp5_37*ORcp5_210-OMcp5_37*ORcp5_218+OPcp5_218*RLcp5_319+OPcp5_221*RLcp5_322+OPcp5_27*RLcp5_310+OPcp5_27*
 RLcp5_318-OPcp5_318*RLcp5_219-OPcp5_321*RLcp5_222-OPcp5_37*RLcp5_210-OPcp5_37*RLcp5_218-ORcp5_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*
 qd[1]*qd[3]*C1-qd[1]*(ORcp5_22+(2.0)*qd[2]*S1);
    ACcp5_222 = -(OMcp5_118*ORcp5_319+OMcp5_121*ORcp5_322+OMcp5_17*(ORcp5_310+ORcp5_318)-OMcp5_318*ORcp5_119-OMcp5_321*
 ORcp5_122-OMcp5_37*ORcp5_110-OMcp5_37*ORcp5_118+OPcp5_118*RLcp5_319+OPcp5_121*RLcp5_322+OPcp5_17*RLcp5_310+OPcp5_17*
 RLcp5_318-OPcp5_318*RLcp5_119-OPcp5_321*RLcp5_122-OPcp5_37*RLcp5_110-OPcp5_37*RLcp5_118-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp5_12+(2.0)
 *qd[2]*C1)-qd[1]*(ORcp5_13-(2.0)*qd[3]*S1));
    ACcp5_322 = qdd[4]+OMcp5_118*ORcp5_219+OMcp5_121*ORcp5_222+OMcp5_17*ORcp5_210+OMcp5_17*ORcp5_218-OMcp5_218*ORcp5_119-
 OMcp5_221*ORcp5_122-OMcp5_27*ORcp5_110-OMcp5_27*ORcp5_118+OPcp5_118*RLcp5_219+OPcp5_121*RLcp5_222+OPcp5_17*RLcp5_210+
 OPcp5_17*RLcp5_218-OPcp5_218*RLcp5_119-OPcp5_221*RLcp5_122-OPcp5_27*RLcp5_110-OPcp5_27*RLcp5_118;
    OMcp5_123 = OMcp5_121+ROcp5_422*qd[23];
    OMcp5_223 = OMcp5_221+ROcp5_522*qd[23];
    OMcp5_323 = OMcp5_321+ROcp5_622*qd[23];
    OPcp5_123 = OPcp5_121+ROcp5_422*qdd[23]+qd[23]*(OMcp5_221*ROcp5_622-OMcp5_321*ROcp5_522);
    OPcp5_223 = OPcp5_221+ROcp5_522*qdd[23]-qd[23]*(OMcp5_121*ROcp5_622-OMcp5_321*ROcp5_422);
    OPcp5_323 = OPcp5_321+ROcp5_622*qdd[23]+qd[23]*(OMcp5_121*ROcp5_522-OMcp5_221*ROcp5_422);

// = = Block_1_0_0_6_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp5_122;
    sens->P[2] = POcp5_222;
    sens->P[3] = POcp5_322;
    sens->R[1][1] = ROcp5_123;
    sens->R[1][2] = ROcp5_223;
    sens->R[1][3] = ROcp5_323;
    sens->R[2][1] = ROcp5_422;
    sens->R[2][2] = ROcp5_522;
    sens->R[2][3] = ROcp5_622;
    sens->R[3][1] = ROcp5_723;
    sens->R[3][2] = ROcp5_823;
    sens->R[3][3] = ROcp5_923;
    sens->V[1] = VIcp5_122;
    sens->V[2] = VIcp5_222;
    sens->V[3] = VIcp5_322;
    sens->OM[1] = OMcp5_123;
    sens->OM[2] = OMcp5_223;
    sens->OM[3] = OMcp5_323;
    sens->A[1] = ACcp5_122;
    sens->A[2] = ACcp5_222;
    sens->A[3] = ACcp5_322;
    sens->OMP[1] = OPcp5_123;
    sens->OMP[2] = OPcp5_223;
    sens->OMP[3] = OPcp5_323;

break;
default:
break;
}


// ====== END Task 1 ====== 


}
 


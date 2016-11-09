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
//	==> Generation Date : Wed Nov  9 18:03:58 2016
//
//	==> Project name : tricycle
//	==> using XML input file 
//
//	==> Number of joints : 30
//
//	==> Function : F 6 : Sensors Kinematical Informations (sens) 
//	==> Flops complexity : 7205
//
//	==> Generation Time :  0.100 seconds
//	==> Post-Processing :  0.160 seconds
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

// = = Block_0_0_0_3_0_2 = = 
 
// Trigonometric Variables  

  S7p8 = C7*S8+S7*C8;
  C7p8 = C7*C8-S7*S8;
  S7p8p9 = C7p8*S9+S7p8*C9;
  C7p8p9 = C7p8*C9-S7p8*S9;

// = = Block_0_0_0_4_0_3 = = 
 
// Trigonometric Variables  

  S10p7 = C10*S7+S10*C7;
  C10p7 = C10*C7-S10*S7;

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

// = = Block_1_0_0_3_0_2 = = 
 
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
    RLcp2_18 = ROcp2_17*s->dpt[1][2]+ROcp2_77*s->dpt[3][2];
    RLcp2_28 = ROcp2_27*s->dpt[1][2]+ROcp2_87*s->dpt[3][2];
    RLcp2_38 = -C6*(s->dpt[1][2]*S7-s->dpt[3][2]*C7);
    OMcp2_18 = OMcp2_17+ROcp2_46*qd[8];
    OMcp2_28 = OMcp2_27+ROcp2_56*qd[8];
    OMcp2_38 = OMcp2_37+qd[8]*S6;
    ORcp2_18 = OMcp2_27*RLcp2_38-OMcp2_37*RLcp2_28;
    ORcp2_28 = -(OMcp2_17*RLcp2_38-OMcp2_37*RLcp2_18);
    ORcp2_38 = OMcp2_17*RLcp2_28-OMcp2_27*RLcp2_18;
    OPcp2_18 = OPcp2_17+ROcp2_46*qdd[8]+qd[8]*(OMcp2_27*S6-OMcp2_37*ROcp2_56);
    OPcp2_28 = OPcp2_27+ROcp2_56*qdd[8]-qd[8]*(OMcp2_17*S6-OMcp2_37*ROcp2_46);
    OPcp2_38 = OPcp2_37+qdd[8]*S6+qd[8]*(OMcp2_17*ROcp2_56-OMcp2_27*ROcp2_46);
    RLcp2_19 = ROcp2_18*s->dpt[1][8];
    RLcp2_29 = ROcp2_28*s->dpt[1][8];
    RLcp2_39 = -s->dpt[1][8]*S7p8*C6;
    POcp2_19 = RLcp2_12+RLcp2_13+RLcp2_18+RLcp2_19;
    POcp2_29 = RLcp2_22+RLcp2_23+RLcp2_28+RLcp2_29;
    POcp2_39 = RLcp2_38+RLcp2_39+q[4];
    JTcp2_19_1 = -(RLcp2_22+RLcp2_23+RLcp2_28+RLcp2_29);
    JTcp2_29_1 = RLcp2_12+RLcp2_13+RLcp2_18+RLcp2_19;
    JTcp2_19_5 = -(RLcp2_28+RLcp2_29);
    JTcp2_29_5 = RLcp2_18+RLcp2_19;
    JTcp2_19_6 = S1p5*(RLcp2_38+RLcp2_39);
    JTcp2_29_6 = -C1p5*(RLcp2_38+RLcp2_39);
    JTcp2_39_6 = C1p5*(RLcp2_28+RLcp2_29)-S1p5*(RLcp2_18+RLcp2_19);
    JTcp2_19_7 = ROcp2_56*(RLcp2_38+RLcp2_39)-S6*(RLcp2_28+RLcp2_29);
    JTcp2_29_7 = -(ROcp2_46*(RLcp2_38+RLcp2_39)-S6*(RLcp2_18+RLcp2_19));
    JTcp2_39_7 = ROcp2_46*(RLcp2_28+RLcp2_29)-ROcp2_56*(RLcp2_18+RLcp2_19);
    JTcp2_19_8 = -(RLcp2_29*S6-RLcp2_39*ROcp2_56);
    JTcp2_29_8 = RLcp2_19*S6-RLcp2_39*ROcp2_46;
    JTcp2_39_8 = -(RLcp2_19*ROcp2_56-RLcp2_29*ROcp2_46);
    OMcp2_19 = OMcp2_18+ROcp2_46*qd[9];
    OMcp2_29 = OMcp2_28+ROcp2_56*qd[9];
    OMcp2_39 = OMcp2_38+qd[9]*S6;
    ORcp2_19 = OMcp2_28*RLcp2_39-OMcp2_38*RLcp2_29;
    ORcp2_29 = -(OMcp2_18*RLcp2_39-OMcp2_38*RLcp2_19);
    ORcp2_39 = OMcp2_18*RLcp2_29-OMcp2_28*RLcp2_19;
    VIcp2_19 = ORcp2_12+ORcp2_13+ORcp2_18+ORcp2_19+qd[2]*C1-qd[3]*S1;
    VIcp2_29 = ORcp2_22+ORcp2_23+ORcp2_28+ORcp2_29+qd[2]*S1+qd[3]*C1;
    VIcp2_39 = ORcp2_38+ORcp2_39+qd[4];
    OPcp2_19 = OPcp2_18+ROcp2_46*qdd[9]+qd[9]*(OMcp2_28*S6-OMcp2_38*ROcp2_56);
    OPcp2_29 = OPcp2_28+ROcp2_56*qdd[9]-qd[9]*(OMcp2_18*S6-OMcp2_38*ROcp2_46);
    OPcp2_39 = OPcp2_38+qdd[9]*S6+qd[9]*(OMcp2_18*ROcp2_56-OMcp2_28*ROcp2_46);
    ACcp2_19 = OMcp2_27*ORcp2_38+OMcp2_28*ORcp2_39-OMcp2_37*ORcp2_28-OMcp2_38*ORcp2_29+OPcp2_27*RLcp2_38+OPcp2_28*RLcp2_39
 -OPcp2_37*RLcp2_28-OPcp2_38*RLcp2_29-ORcp2_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp2_22+(2.0)*qd[2]*S1);
    ACcp2_29 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp2_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp2_13-(2.0)*qd[3]*S1)-OMcp2_17*ORcp2_38+OMcp2_37*
 ORcp2_18-OPcp2_17*RLcp2_38+OPcp2_37*RLcp2_18-OMcp2_18*ORcp2_39+OMcp2_38*ORcp2_19-OPcp2_18*RLcp2_39+OPcp2_38*RLcp2_19;
    ACcp2_39 = qdd[4]+OMcp2_17*ORcp2_28+OMcp2_18*ORcp2_29-OMcp2_27*ORcp2_18-OMcp2_28*ORcp2_19+OPcp2_17*RLcp2_28+OPcp2_18*
 RLcp2_29-OPcp2_27*RLcp2_18-OPcp2_28*RLcp2_19;

// = = Block_1_0_0_3_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp2_19;
    sens->P[2] = POcp2_29;
    sens->P[3] = POcp2_39;
    sens->R[1][1] = ROcp2_19;
    sens->R[1][2] = ROcp2_29;
    sens->R[1][3] = ROcp2_39;
    sens->R[2][1] = ROcp2_46;
    sens->R[2][2] = ROcp2_56;
    sens->R[2][3] = S6;
    sens->R[3][1] = ROcp2_79;
    sens->R[3][2] = ROcp2_89;
    sens->R[3][3] = ROcp2_99;
    sens->V[1] = VIcp2_19;
    sens->V[2] = VIcp2_29;
    sens->V[3] = VIcp2_39;
    sens->OM[1] = OMcp2_19;
    sens->OM[2] = OMcp2_29;
    sens->OM[3] = OMcp2_39;
    sens->J[1][1] = JTcp2_19_1;
    sens->J[1][2] = C1;
    sens->J[1][3] = -S1;
    sens->J[1][5] = JTcp2_19_5;
    sens->J[1][6] = JTcp2_19_6;
    sens->J[1][7] = JTcp2_19_7;
    sens->J[1][8] = JTcp2_19_8;
    sens->J[2][1] = JTcp2_29_1;
    sens->J[2][2] = S1;
    sens->J[2][3] = C1;
    sens->J[2][5] = JTcp2_29_5;
    sens->J[2][6] = JTcp2_29_6;
    sens->J[2][7] = JTcp2_29_7;
    sens->J[2][8] = JTcp2_29_8;
    sens->J[3][4] = (1.0);
    sens->J[3][6] = JTcp2_39_6;
    sens->J[3][7] = JTcp2_39_7;
    sens->J[3][8] = JTcp2_39_8;
    sens->J[4][6] = C1p5;
    sens->J[4][7] = ROcp2_46;
    sens->J[4][8] = ROcp2_46;
    sens->J[4][9] = ROcp2_46;
    sens->J[5][6] = S1p5;
    sens->J[5][7] = ROcp2_56;
    sens->J[5][8] = ROcp2_56;
    sens->J[5][9] = ROcp2_56;
    sens->J[6][1] = (1.0);
    sens->J[6][5] = (1.0);
    sens->J[6][7] = S6;
    sens->J[6][8] = S6;
    sens->J[6][9] = S6;
    sens->A[1] = ACcp2_19;
    sens->A[2] = ACcp2_29;
    sens->A[3] = ACcp2_39;
    sens->OMP[1] = OPcp2_19;
    sens->OMP[2] = OPcp2_29;
    sens->OMP[3] = OPcp2_39;
 
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

// = = Block_1_0_0_4_0_5 = = 
 
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
    RLcp3_112 = ROcp3_46*s->dpt[2][11]+ROcp3_710*s->dpt[3][11];
    RLcp3_212 = ROcp3_56*s->dpt[2][11]+ROcp3_810*s->dpt[3][11];
    RLcp3_312 = ROcp3_910*s->dpt[3][11]+s->dpt[2][11]*S6;
    OMcp3_112 = OMcp3_17+ROcp3_110*qd[12];
    OMcp3_212 = OMcp3_27+ROcp3_210*qd[12];
    OMcp3_312 = OMcp3_37+ROcp3_310*qd[12];
    ORcp3_112 = OMcp3_27*RLcp3_312-OMcp3_37*RLcp3_212;
    ORcp3_212 = -(OMcp3_17*RLcp3_312-OMcp3_37*RLcp3_112);
    ORcp3_312 = OMcp3_17*RLcp3_212-OMcp3_27*RLcp3_112;
    OPcp3_112 = OPcp3_17+ROcp3_110*qdd[12]+qd[12]*(OMcp3_27*ROcp3_310-OMcp3_37*ROcp3_210);
    OPcp3_212 = OPcp3_27+ROcp3_210*qdd[12]-qd[12]*(OMcp3_17*ROcp3_310-OMcp3_37*ROcp3_110);
    OPcp3_312 = OPcp3_37+ROcp3_310*qdd[12]+qd[12]*(OMcp3_17*ROcp3_210-OMcp3_27*ROcp3_110);
    RLcp3_113 = ROcp3_412*s->dpt[2][16];
    RLcp3_213 = ROcp3_512*s->dpt[2][16];
    RLcp3_313 = ROcp3_612*s->dpt[2][16];
    POcp3_113 = RLcp3_110+RLcp3_112+RLcp3_113+RLcp3_12+RLcp3_13;
    POcp3_213 = RLcp3_210+RLcp3_212+RLcp3_213+RLcp3_22+RLcp3_23;
    POcp3_313 = RLcp3_310+RLcp3_312+RLcp3_313+q[4];
    JTcp3_113_1 = -(RLcp3_210+RLcp3_212+RLcp3_213+RLcp3_22+RLcp3_23);
    JTcp3_213_1 = RLcp3_110+RLcp3_112+RLcp3_113+RLcp3_12+RLcp3_13;
    JTcp3_113_5 = -(RLcp3_210+RLcp3_212+RLcp3_213);
    JTcp3_213_5 = RLcp3_110+RLcp3_112+RLcp3_113;
    JTcp3_113_6 = S1p5*(RLcp3_310+RLcp3_312+RLcp3_313);
    JTcp3_213_6 = -C1p5*(RLcp3_310+RLcp3_312+RLcp3_313);
    JTcp3_313_6 = C1p5*(RLcp3_210+RLcp3_212)-S1p5*(RLcp3_110+RLcp3_112)-RLcp3_113*S1p5+RLcp3_213*C1p5;
    JTcp3_113_7 = ROcp3_56*(RLcp3_310+RLcp3_312)-S6*(RLcp3_210+RLcp3_212)-RLcp3_213*S6+RLcp3_313*ROcp3_56;
    JTcp3_213_7 = RLcp3_113*S6-RLcp3_313*ROcp3_46-ROcp3_46*(RLcp3_310+RLcp3_312)+S6*(RLcp3_110+RLcp3_112);
    JTcp3_313_7 = ROcp3_46*(RLcp3_210+RLcp3_212)-ROcp3_56*(RLcp3_110+RLcp3_112)-RLcp3_113*ROcp3_56+RLcp3_213*ROcp3_46;
    JTcp3_113_8 = ROcp3_56*(RLcp3_312+RLcp3_313)-S6*(RLcp3_212+RLcp3_213);
    JTcp3_213_8 = -(ROcp3_46*(RLcp3_312+RLcp3_313)-S6*(RLcp3_112+RLcp3_113));
    JTcp3_313_8 = ROcp3_46*(RLcp3_212+RLcp3_213)-ROcp3_56*(RLcp3_112+RLcp3_113);
    JTcp3_113_9 = -(RLcp3_213*ROcp3_310-RLcp3_313*ROcp3_210);
    JTcp3_213_9 = RLcp3_113*ROcp3_310-RLcp3_313*ROcp3_110;
    JTcp3_313_9 = -(RLcp3_113*ROcp3_210-RLcp3_213*ROcp3_110);
    OMcp3_113 = OMcp3_112+ROcp3_110*qd[13];
    OMcp3_213 = OMcp3_212+ROcp3_210*qd[13];
    OMcp3_313 = OMcp3_312+ROcp3_310*qd[13];
    ORcp3_113 = OMcp3_212*RLcp3_313-OMcp3_312*RLcp3_213;
    ORcp3_213 = -(OMcp3_112*RLcp3_313-OMcp3_312*RLcp3_113);
    ORcp3_313 = OMcp3_112*RLcp3_213-OMcp3_212*RLcp3_113;
    VIcp3_113 = ORcp3_110+ORcp3_112+ORcp3_113+ORcp3_12+ORcp3_13+qd[2]*C1-qd[3]*S1;
    VIcp3_213 = ORcp3_210+ORcp3_212+ORcp3_213+ORcp3_22+ORcp3_23+qd[2]*S1+qd[3]*C1;
    VIcp3_313 = ORcp3_310+ORcp3_312+ORcp3_313+qd[4];
    ACcp3_113 = OMcp3_212*ORcp3_313+OMcp3_27*(ORcp3_310+ORcp3_312)-OMcp3_312*ORcp3_213-OMcp3_37*ORcp3_210-OMcp3_37*
 ORcp3_212+OPcp3_212*RLcp3_313+OPcp3_27*RLcp3_310+OPcp3_27*RLcp3_312-OPcp3_312*RLcp3_213-OPcp3_37*RLcp3_210-OPcp3_37*
 RLcp3_212-ORcp3_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp3_22+(2.0)*qd[2]*S1);
    ACcp3_213 = -(OMcp3_112*ORcp3_313+OMcp3_17*(ORcp3_310+ORcp3_312)-OMcp3_312*ORcp3_113-OMcp3_37*ORcp3_110-OMcp3_37*
 ORcp3_112+OPcp3_112*RLcp3_313+OPcp3_17*RLcp3_310+OPcp3_17*RLcp3_312-OPcp3_312*RLcp3_113-OPcp3_37*RLcp3_110-OPcp3_37*
 RLcp3_112-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp3_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp3_13-(2.0)*qd[3]*S1));
    ACcp3_313 = qdd[4]+OMcp3_112*ORcp3_213+OMcp3_17*ORcp3_210+OMcp3_17*ORcp3_212-OMcp3_212*ORcp3_113-OMcp3_27*ORcp3_110-
 OMcp3_27*ORcp3_112+OPcp3_112*RLcp3_213+OPcp3_17*RLcp3_210+OPcp3_17*RLcp3_212-OPcp3_212*RLcp3_113-OPcp3_27*RLcp3_110-OPcp3_27
 *RLcp3_112;
    OMcp3_114 = OMcp3_113+ROcp3_413*qd[14];
    OMcp3_214 = OMcp3_213+ROcp3_513*qd[14];
    OMcp3_314 = OMcp3_313+ROcp3_613*qd[14];
    OMcp3_115 = OMcp3_114+ROcp3_714*qd[15];
    OMcp3_215 = OMcp3_214+ROcp3_814*qd[15];
    OMcp3_315 = OMcp3_314+ROcp3_914*qd[15];
    OPcp3_115 = OPcp3_112+ROcp3_110*qdd[13]+ROcp3_413*qdd[14]+ROcp3_714*qdd[15]+qd[13]*(OMcp3_212*ROcp3_310-OMcp3_312*
 ROcp3_210)+qd[14]*(OMcp3_213*ROcp3_613-OMcp3_313*ROcp3_513)+qd[15]*(OMcp3_214*ROcp3_914-OMcp3_314*ROcp3_814);
    OPcp3_215 = OPcp3_212+ROcp3_210*qdd[13]+ROcp3_513*qdd[14]+ROcp3_814*qdd[15]-qd[13]*(OMcp3_112*ROcp3_310-OMcp3_312*
 ROcp3_110)-qd[14]*(OMcp3_113*ROcp3_613-OMcp3_313*ROcp3_413)-qd[15]*(OMcp3_114*ROcp3_914-OMcp3_314*ROcp3_714);
    OPcp3_315 = OPcp3_312+ROcp3_310*qdd[13]+ROcp3_613*qdd[14]+ROcp3_914*qdd[15]+qd[13]*(OMcp3_112*ROcp3_210-OMcp3_212*
 ROcp3_110)+qd[14]*(OMcp3_113*ROcp3_513-OMcp3_213*ROcp3_413)+qd[15]*(OMcp3_114*ROcp3_814-OMcp3_214*ROcp3_714);

// = = Block_1_0_0_4_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp3_113;
    sens->P[2] = POcp3_213;
    sens->P[3] = POcp3_313;
    sens->R[1][1] = ROcp3_115;
    sens->R[1][2] = ROcp3_215;
    sens->R[1][3] = ROcp3_315;
    sens->R[2][1] = ROcp3_415;
    sens->R[2][2] = ROcp3_515;
    sens->R[2][3] = ROcp3_615;
    sens->R[3][1] = ROcp3_714;
    sens->R[3][2] = ROcp3_814;
    sens->R[3][3] = ROcp3_914;
    sens->V[1] = VIcp3_113;
    sens->V[2] = VIcp3_213;
    sens->V[3] = VIcp3_313;
    sens->OM[1] = OMcp3_115;
    sens->OM[2] = OMcp3_215;
    sens->OM[3] = OMcp3_315;
    sens->J[1][1] = JTcp3_113_1;
    sens->J[1][2] = C1;
    sens->J[1][3] = -S1;
    sens->J[1][5] = JTcp3_113_5;
    sens->J[1][6] = JTcp3_113_6;
    sens->J[1][7] = JTcp3_113_7;
    sens->J[1][10] = JTcp3_113_8;
    sens->J[1][12] = JTcp3_113_9;
    sens->J[2][1] = JTcp3_213_1;
    sens->J[2][2] = S1;
    sens->J[2][3] = C1;
    sens->J[2][5] = JTcp3_213_5;
    sens->J[2][6] = JTcp3_213_6;
    sens->J[2][7] = JTcp3_213_7;
    sens->J[2][10] = JTcp3_213_8;
    sens->J[2][12] = JTcp3_213_9;
    sens->J[3][4] = (1.0);
    sens->J[3][6] = JTcp3_313_6;
    sens->J[3][7] = JTcp3_313_7;
    sens->J[3][10] = JTcp3_313_8;
    sens->J[3][12] = JTcp3_313_9;
    sens->J[4][6] = C1p5;
    sens->J[4][7] = ROcp3_46;
    sens->J[4][10] = ROcp3_46;
    sens->J[4][12] = ROcp3_110;
    sens->J[4][13] = ROcp3_110;
    sens->J[4][14] = ROcp3_413;
    sens->J[4][15] = ROcp3_714;
    sens->J[5][6] = S1p5;
    sens->J[5][7] = ROcp3_56;
    sens->J[5][10] = ROcp3_56;
    sens->J[5][12] = ROcp3_210;
    sens->J[5][13] = ROcp3_210;
    sens->J[5][14] = ROcp3_513;
    sens->J[5][15] = ROcp3_814;
    sens->J[6][1] = (1.0);
    sens->J[6][5] = (1.0);
    sens->J[6][7] = S6;
    sens->J[6][10] = S6;
    sens->J[6][12] = ROcp3_310;
    sens->J[6][13] = ROcp3_310;
    sens->J[6][14] = ROcp3_613;
    sens->J[6][15] = ROcp3_914;
    sens->A[1] = ACcp3_113;
    sens->A[2] = ACcp3_213;
    sens->A[3] = ACcp3_313;
    sens->OMP[1] = OPcp3_115;
    sens->OMP[2] = OPcp3_215;
    sens->OMP[3] = OPcp3_315;
 
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
    RLcp4_149 = ROcp4_714*s->dpt[3][19];
    RLcp4_249 = ROcp4_814*s->dpt[3][19];
    RLcp4_349 = ROcp4_914*s->dpt[3][19];
    POcp4_149 = RLcp4_110+RLcp4_112+RLcp4_113+RLcp4_12+RLcp4_13+RLcp4_149;
    POcp4_249 = RLcp4_210+RLcp4_212+RLcp4_213+RLcp4_22+RLcp4_23+RLcp4_249;
    POcp4_349 = RLcp4_310+RLcp4_312+RLcp4_313+RLcp4_349+q[4];
    JTcp4_149_1 = -(RLcp4_210+RLcp4_212+RLcp4_213+RLcp4_22+RLcp4_23+RLcp4_249);
    JTcp4_249_1 = RLcp4_110+RLcp4_112+RLcp4_113+RLcp4_12+RLcp4_13+RLcp4_149;
    JTcp4_149_5 = -(RLcp4_210+RLcp4_212+RLcp4_213+RLcp4_249);
    JTcp4_249_5 = RLcp4_110+RLcp4_112+RLcp4_113+RLcp4_149;
    JTcp4_149_6 = S1p5*(RLcp4_310+RLcp4_312+RLcp4_313+RLcp4_349);
    JTcp4_249_6 = -C1p5*(RLcp4_310+RLcp4_312+RLcp4_313+RLcp4_349);
    JTcp4_349_6 = C1p5*(RLcp4_210+RLcp4_212+RLcp4_213+RLcp4_249)-S1p5*(RLcp4_110+RLcp4_112)-S1p5*(RLcp4_113+RLcp4_149);
    JTcp4_149_7 = ROcp4_56*(RLcp4_310+RLcp4_312+RLcp4_313+RLcp4_349)-S6*(RLcp4_210+RLcp4_212)-S6*(RLcp4_213+RLcp4_249);
    JTcp4_249_7 = -(ROcp4_46*(RLcp4_310+RLcp4_312+RLcp4_313+RLcp4_349)-S6*(RLcp4_110+RLcp4_112)-S6*(RLcp4_113+RLcp4_149));
    JTcp4_349_7 = ROcp4_46*(RLcp4_210+RLcp4_212+RLcp4_213+RLcp4_249)-ROcp4_56*(RLcp4_110+RLcp4_112)-ROcp4_56*(RLcp4_113+
 RLcp4_149);
    JTcp4_149_8 = ROcp4_56*(RLcp4_312+RLcp4_313)-S6*(RLcp4_212+RLcp4_213)-RLcp4_249*S6+RLcp4_349*ROcp4_56;
    JTcp4_249_8 = RLcp4_149*S6-RLcp4_349*ROcp4_46-ROcp4_46*(RLcp4_312+RLcp4_313)+S6*(RLcp4_112+RLcp4_113);
    JTcp4_349_8 = ROcp4_46*(RLcp4_212+RLcp4_213)-ROcp4_56*(RLcp4_112+RLcp4_113)-RLcp4_149*ROcp4_56+RLcp4_249*ROcp4_46;
    JTcp4_149_9 = ROcp4_210*(RLcp4_313+RLcp4_349)-ROcp4_310*(RLcp4_213+RLcp4_249);
    JTcp4_249_9 = -(ROcp4_110*(RLcp4_313+RLcp4_349)-ROcp4_310*(RLcp4_113+RLcp4_149));
    JTcp4_349_9 = ROcp4_110*(RLcp4_213+RLcp4_249)-ROcp4_210*(RLcp4_113+RLcp4_149);
    JTcp4_149_10 = -(RLcp4_249*ROcp4_310-RLcp4_349*ROcp4_210);
    JTcp4_249_10 = RLcp4_149*ROcp4_310-RLcp4_349*ROcp4_110;
    JTcp4_349_10 = -(RLcp4_149*ROcp4_210-RLcp4_249*ROcp4_110);
    JTcp4_149_11 = -(RLcp4_249*ROcp4_613-RLcp4_349*ROcp4_513);
    JTcp4_249_11 = RLcp4_149*ROcp4_613-RLcp4_349*ROcp4_413;
    JTcp4_349_11 = -(RLcp4_149*ROcp4_513-RLcp4_249*ROcp4_413);
    JTcp4_149_12 = -(RLcp4_249*ROcp4_914-RLcp4_349*ROcp4_814);
    JTcp4_249_12 = RLcp4_149*ROcp4_914-RLcp4_349*ROcp4_714;
    JTcp4_349_12 = -(RLcp4_149*ROcp4_814-RLcp4_249*ROcp4_714);
    ORcp4_149 = OMcp4_215*RLcp4_349-OMcp4_315*RLcp4_249;
    ORcp4_249 = -(OMcp4_115*RLcp4_349-OMcp4_315*RLcp4_149);
    ORcp4_349 = OMcp4_115*RLcp4_249-OMcp4_215*RLcp4_149;
    VIcp4_149 = ORcp4_110+ORcp4_112+ORcp4_113+ORcp4_12+ORcp4_13+ORcp4_149+qd[2]*C1-qd[3]*S1;
    VIcp4_249 = ORcp4_210+ORcp4_212+ORcp4_213+ORcp4_22+ORcp4_23+ORcp4_249+qd[2]*S1+qd[3]*C1;
    VIcp4_349 = ORcp4_310+ORcp4_312+ORcp4_313+ORcp4_349+qd[4];
    ACcp4_149 = OMcp4_212*ORcp4_313+OMcp4_215*ORcp4_349+OMcp4_27*(ORcp4_310+ORcp4_312)-OMcp4_312*ORcp4_213-OMcp4_315*
 ORcp4_249-OMcp4_37*ORcp4_210-OMcp4_37*ORcp4_212+OPcp4_212*RLcp4_313+OPcp4_215*RLcp4_349+OPcp4_27*RLcp4_310+OPcp4_27*
 RLcp4_312-OPcp4_312*RLcp4_213-OPcp4_315*RLcp4_249-OPcp4_37*RLcp4_210-OPcp4_37*RLcp4_212-ORcp4_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*
 qd[1]*qd[3]*C1-qd[1]*(ORcp4_22+(2.0)*qd[2]*S1);
    ACcp4_249 = -(OMcp4_112*ORcp4_313+OMcp4_115*ORcp4_349+OMcp4_17*(ORcp4_310+ORcp4_312)-OMcp4_312*ORcp4_113-OMcp4_315*
 ORcp4_149-OMcp4_37*ORcp4_110-OMcp4_37*ORcp4_112+OPcp4_112*RLcp4_313+OPcp4_115*RLcp4_349+OPcp4_17*RLcp4_310+OPcp4_17*
 RLcp4_312-OPcp4_312*RLcp4_113-OPcp4_315*RLcp4_149-OPcp4_37*RLcp4_110-OPcp4_37*RLcp4_112-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp4_12+(2.0)
 *qd[2]*C1)-qd[1]*(ORcp4_13-(2.0)*qd[3]*S1));
    ACcp4_349 = qdd[4]+OMcp4_112*ORcp4_213+OMcp4_115*ORcp4_249+OMcp4_17*ORcp4_210+OMcp4_17*ORcp4_212-OMcp4_212*ORcp4_113-
 OMcp4_215*ORcp4_149-OMcp4_27*ORcp4_110-OMcp4_27*ORcp4_112+OPcp4_112*RLcp4_213+OPcp4_115*RLcp4_249+OPcp4_17*RLcp4_210+
 OPcp4_17*RLcp4_212-OPcp4_212*RLcp4_113-OPcp4_215*RLcp4_149-OPcp4_27*RLcp4_110-OPcp4_27*RLcp4_112;

// = = Block_1_0_0_5_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp4_149;
    sens->P[2] = POcp4_249;
    sens->P[3] = POcp4_349;
    sens->R[1][1] = ROcp4_115;
    sens->R[1][2] = ROcp4_215;
    sens->R[1][3] = ROcp4_315;
    sens->R[2][1] = ROcp4_415;
    sens->R[2][2] = ROcp4_515;
    sens->R[2][3] = ROcp4_615;
    sens->R[3][1] = ROcp4_714;
    sens->R[3][2] = ROcp4_814;
    sens->R[3][3] = ROcp4_914;
    sens->V[1] = VIcp4_149;
    sens->V[2] = VIcp4_249;
    sens->V[3] = VIcp4_349;
    sens->OM[1] = OMcp4_115;
    sens->OM[2] = OMcp4_215;
    sens->OM[3] = OMcp4_315;
    sens->J[1][1] = JTcp4_149_1;
    sens->J[1][2] = C1;
    sens->J[1][3] = -S1;
    sens->J[1][5] = JTcp4_149_5;
    sens->J[1][6] = JTcp4_149_6;
    sens->J[1][7] = JTcp4_149_7;
    sens->J[1][10] = JTcp4_149_8;
    sens->J[1][12] = JTcp4_149_9;
    sens->J[1][13] = JTcp4_149_10;
    sens->J[1][14] = JTcp4_149_11;
    sens->J[1][15] = JTcp4_149_12;
    sens->J[2][1] = JTcp4_249_1;
    sens->J[2][2] = S1;
    sens->J[2][3] = C1;
    sens->J[2][5] = JTcp4_249_5;
    sens->J[2][6] = JTcp4_249_6;
    sens->J[2][7] = JTcp4_249_7;
    sens->J[2][10] = JTcp4_249_8;
    sens->J[2][12] = JTcp4_249_9;
    sens->J[2][13] = JTcp4_249_10;
    sens->J[2][14] = JTcp4_249_11;
    sens->J[2][15] = JTcp4_249_12;
    sens->J[3][4] = (1.0);
    sens->J[3][6] = JTcp4_349_6;
    sens->J[3][7] = JTcp4_349_7;
    sens->J[3][10] = JTcp4_349_8;
    sens->J[3][12] = JTcp4_349_9;
    sens->J[3][13] = JTcp4_349_10;
    sens->J[3][14] = JTcp4_349_11;
    sens->J[3][15] = JTcp4_349_12;
    sens->J[4][6] = C1p5;
    sens->J[4][7] = ROcp4_46;
    sens->J[4][10] = ROcp4_46;
    sens->J[4][12] = ROcp4_110;
    sens->J[4][13] = ROcp4_110;
    sens->J[4][14] = ROcp4_413;
    sens->J[4][15] = ROcp4_714;
    sens->J[5][6] = S1p5;
    sens->J[5][7] = ROcp4_56;
    sens->J[5][10] = ROcp4_56;
    sens->J[5][12] = ROcp4_210;
    sens->J[5][13] = ROcp4_210;
    sens->J[5][14] = ROcp4_513;
    sens->J[5][15] = ROcp4_814;
    sens->J[6][1] = (1.0);
    sens->J[6][5] = (1.0);
    sens->J[6][7] = S6;
    sens->J[6][10] = S6;
    sens->J[6][12] = ROcp4_310;
    sens->J[6][13] = ROcp4_310;
    sens->J[6][14] = ROcp4_613;
    sens->J[6][15] = ROcp4_914;
    sens->A[1] = ACcp4_149;
    sens->A[2] = ACcp4_249;
    sens->A[3] = ACcp4_349;
    sens->OMP[1] = OPcp4_115;
    sens->OMP[2] = OPcp4_215;
    sens->OMP[3] = OPcp4_315;
 
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

// = = Block_1_0_0_6_0_5 = = 
 
// Sensor Kinematics 


    ROcp5_412 = ROcp5_46*C12+ROcp5_710*S12;
    ROcp5_512 = ROcp5_56*C12+ROcp5_810*S12;
    ROcp5_612 = ROcp5_910*S12+C12*S6;
    ROcp5_712 = -(ROcp5_46*S12-ROcp5_710*C12);
    ROcp5_812 = -(ROcp5_56*S12-ROcp5_810*C12);
    ROcp5_912 = ROcp5_910*C12-S12*S6;
    ROcp5_413 = ROcp5_412*C13+ROcp5_712*S13;
    ROcp5_513 = ROcp5_512*C13+ROcp5_812*S13;
    ROcp5_613 = ROcp5_612*C13+ROcp5_912*S13;
    ROcp5_713 = -(ROcp5_412*S13-ROcp5_712*C13);
    ROcp5_813 = -(ROcp5_512*S13-ROcp5_812*C13);
    ROcp5_913 = -(ROcp5_612*S13-ROcp5_912*C13);
    ROcp5_114 = ROcp5_110*C14-ROcp5_713*S14;
    ROcp5_214 = ROcp5_210*C14-ROcp5_813*S14;
    ROcp5_314 = ROcp5_310*C14-ROcp5_913*S14;
    ROcp5_714 = ROcp5_110*S14+ROcp5_713*C14;
    ROcp5_814 = ROcp5_210*S14+ROcp5_813*C14;
    ROcp5_914 = ROcp5_310*S14+ROcp5_913*C14;
    ROcp5_115 = ROcp5_114*C15+ROcp5_413*S15;
    ROcp5_215 = ROcp5_214*C15+ROcp5_513*S15;
    ROcp5_315 = ROcp5_314*C15+ROcp5_613*S15;
    ROcp5_415 = -(ROcp5_114*S15-ROcp5_413*C15);
    ROcp5_515 = -(ROcp5_214*S15-ROcp5_513*C15);
    ROcp5_615 = -(ROcp5_314*S15-ROcp5_613*C15);
    RLcp5_112 = ROcp5_46*s->dpt[2][11]+ROcp5_710*s->dpt[3][11];
    RLcp5_212 = ROcp5_56*s->dpt[2][11]+ROcp5_810*s->dpt[3][11];
    RLcp5_312 = ROcp5_910*s->dpt[3][11]+s->dpt[2][11]*S6;
    OMcp5_112 = OMcp5_17+ROcp5_110*qd[12];
    OMcp5_212 = OMcp5_27+ROcp5_210*qd[12];
    OMcp5_312 = OMcp5_37+ROcp5_310*qd[12];
    ORcp5_112 = OMcp5_27*RLcp5_312-OMcp5_37*RLcp5_212;
    ORcp5_212 = -(OMcp5_17*RLcp5_312-OMcp5_37*RLcp5_112);
    ORcp5_312 = OMcp5_17*RLcp5_212-OMcp5_27*RLcp5_112;
    OPcp5_112 = OPcp5_17+ROcp5_110*qdd[12]+qd[12]*(OMcp5_27*ROcp5_310-OMcp5_37*ROcp5_210);
    OPcp5_212 = OPcp5_27+ROcp5_210*qdd[12]-qd[12]*(OMcp5_17*ROcp5_310-OMcp5_37*ROcp5_110);
    OPcp5_312 = OPcp5_37+ROcp5_310*qdd[12]+qd[12]*(OMcp5_17*ROcp5_210-OMcp5_27*ROcp5_110);
    RLcp5_113 = ROcp5_412*s->dpt[2][16];
    RLcp5_213 = ROcp5_512*s->dpt[2][16];
    RLcp5_313 = ROcp5_612*s->dpt[2][16];
    OMcp5_113 = OMcp5_112+ROcp5_110*qd[13];
    OMcp5_213 = OMcp5_212+ROcp5_210*qd[13];
    OMcp5_313 = OMcp5_312+ROcp5_310*qd[13];
    ORcp5_113 = OMcp5_212*RLcp5_313-OMcp5_312*RLcp5_213;
    ORcp5_213 = -(OMcp5_112*RLcp5_313-OMcp5_312*RLcp5_113);
    ORcp5_313 = OMcp5_112*RLcp5_213-OMcp5_212*RLcp5_113;
    OMcp5_114 = OMcp5_113+ROcp5_413*qd[14];
    OMcp5_214 = OMcp5_213+ROcp5_513*qd[14];
    OMcp5_314 = OMcp5_313+ROcp5_613*qd[14];
    OMcp5_115 = OMcp5_114+ROcp5_714*qd[15];
    OMcp5_215 = OMcp5_214+ROcp5_814*qd[15];
    OMcp5_315 = OMcp5_314+ROcp5_914*qd[15];
    OPcp5_115 = OPcp5_112+ROcp5_110*qdd[13]+ROcp5_413*qdd[14]+ROcp5_714*qdd[15]+qd[13]*(OMcp5_212*ROcp5_310-OMcp5_312*
 ROcp5_210)+qd[14]*(OMcp5_213*ROcp5_613-OMcp5_313*ROcp5_513)+qd[15]*(OMcp5_214*ROcp5_914-OMcp5_314*ROcp5_814);
    OPcp5_215 = OPcp5_212+ROcp5_210*qdd[13]+ROcp5_513*qdd[14]+ROcp5_814*qdd[15]-qd[13]*(OMcp5_112*ROcp5_310-OMcp5_312*
 ROcp5_110)-qd[14]*(OMcp5_113*ROcp5_613-OMcp5_313*ROcp5_413)-qd[15]*(OMcp5_114*ROcp5_914-OMcp5_314*ROcp5_714);
    OPcp5_315 = OPcp5_312+ROcp5_310*qdd[13]+ROcp5_613*qdd[14]+ROcp5_914*qdd[15]+qd[13]*(OMcp5_112*ROcp5_210-OMcp5_212*
 ROcp5_110)+qd[14]*(OMcp5_113*ROcp5_513-OMcp5_213*ROcp5_413)+qd[15]*(OMcp5_114*ROcp5_814-OMcp5_214*ROcp5_714);
    RLcp5_150 = ROcp5_115*s->dpt[1][20]+ROcp5_415*s->dpt[2][20]+ROcp5_714*s->dpt[3][20];
    RLcp5_250 = ROcp5_215*s->dpt[1][20]+ROcp5_515*s->dpt[2][20]+ROcp5_814*s->dpt[3][20];
    RLcp5_350 = ROcp5_315*s->dpt[1][20]+ROcp5_615*s->dpt[2][20]+ROcp5_914*s->dpt[3][20];
    POcp5_150 = RLcp5_110+RLcp5_112+RLcp5_113+RLcp5_12+RLcp5_13+RLcp5_150;
    POcp5_250 = RLcp5_210+RLcp5_212+RLcp5_213+RLcp5_22+RLcp5_23+RLcp5_250;
    POcp5_350 = RLcp5_310+RLcp5_312+RLcp5_313+RLcp5_350+q[4];
    JTcp5_150_1 = -(RLcp5_210+RLcp5_212+RLcp5_213+RLcp5_22+RLcp5_23+RLcp5_250);
    JTcp5_250_1 = RLcp5_110+RLcp5_112+RLcp5_113+RLcp5_12+RLcp5_13+RLcp5_150;
    JTcp5_150_5 = -(RLcp5_210+RLcp5_212+RLcp5_213+RLcp5_250);
    JTcp5_250_5 = RLcp5_110+RLcp5_112+RLcp5_113+RLcp5_150;
    JTcp5_150_6 = S1p5*(RLcp5_310+RLcp5_312+RLcp5_313+RLcp5_350);
    JTcp5_250_6 = -C1p5*(RLcp5_310+RLcp5_312+RLcp5_313+RLcp5_350);
    JTcp5_350_6 = C1p5*(RLcp5_210+RLcp5_212+RLcp5_213+RLcp5_250)-S1p5*(RLcp5_110+RLcp5_112)-S1p5*(RLcp5_113+RLcp5_150);
    JTcp5_150_7 = ROcp5_56*(RLcp5_310+RLcp5_312+RLcp5_313+RLcp5_350)-S6*(RLcp5_210+RLcp5_212)-S6*(RLcp5_213+RLcp5_250);
    JTcp5_250_7 = -(ROcp5_46*(RLcp5_310+RLcp5_312+RLcp5_313+RLcp5_350)-S6*(RLcp5_110+RLcp5_112)-S6*(RLcp5_113+RLcp5_150));
    JTcp5_350_7 = ROcp5_46*(RLcp5_210+RLcp5_212+RLcp5_213+RLcp5_250)-ROcp5_56*(RLcp5_110+RLcp5_112)-ROcp5_56*(RLcp5_113+
 RLcp5_150);
    JTcp5_150_8 = ROcp5_56*(RLcp5_312+RLcp5_313)-S6*(RLcp5_212+RLcp5_213)-RLcp5_250*S6+RLcp5_350*ROcp5_56;
    JTcp5_250_8 = RLcp5_150*S6-RLcp5_350*ROcp5_46-ROcp5_46*(RLcp5_312+RLcp5_313)+S6*(RLcp5_112+RLcp5_113);
    JTcp5_350_8 = ROcp5_46*(RLcp5_212+RLcp5_213)-ROcp5_56*(RLcp5_112+RLcp5_113)-RLcp5_150*ROcp5_56+RLcp5_250*ROcp5_46;
    JTcp5_150_9 = ROcp5_210*(RLcp5_313+RLcp5_350)-ROcp5_310*(RLcp5_213+RLcp5_250);
    JTcp5_250_9 = -(ROcp5_110*(RLcp5_313+RLcp5_350)-ROcp5_310*(RLcp5_113+RLcp5_150));
    JTcp5_350_9 = ROcp5_110*(RLcp5_213+RLcp5_250)-ROcp5_210*(RLcp5_113+RLcp5_150);
    JTcp5_150_10 = -(RLcp5_250*ROcp5_310-RLcp5_350*ROcp5_210);
    JTcp5_250_10 = RLcp5_150*ROcp5_310-RLcp5_350*ROcp5_110;
    JTcp5_350_10 = -(RLcp5_150*ROcp5_210-RLcp5_250*ROcp5_110);
    JTcp5_150_11 = -(RLcp5_250*ROcp5_613-RLcp5_350*ROcp5_513);
    JTcp5_250_11 = RLcp5_150*ROcp5_613-RLcp5_350*ROcp5_413;
    JTcp5_350_11 = -(RLcp5_150*ROcp5_513-RLcp5_250*ROcp5_413);
    JTcp5_150_12 = -(RLcp5_250*ROcp5_914-RLcp5_350*ROcp5_814);
    JTcp5_250_12 = RLcp5_150*ROcp5_914-RLcp5_350*ROcp5_714;
    JTcp5_350_12 = -(RLcp5_150*ROcp5_814-RLcp5_250*ROcp5_714);
    ORcp5_150 = OMcp5_215*RLcp5_350-OMcp5_315*RLcp5_250;
    ORcp5_250 = -(OMcp5_115*RLcp5_350-OMcp5_315*RLcp5_150);
    ORcp5_350 = OMcp5_115*RLcp5_250-OMcp5_215*RLcp5_150;
    VIcp5_150 = ORcp5_110+ORcp5_112+ORcp5_113+ORcp5_12+ORcp5_13+ORcp5_150+qd[2]*C1-qd[3]*S1;
    VIcp5_250 = ORcp5_210+ORcp5_212+ORcp5_213+ORcp5_22+ORcp5_23+ORcp5_250+qd[2]*S1+qd[3]*C1;
    VIcp5_350 = ORcp5_310+ORcp5_312+ORcp5_313+ORcp5_350+qd[4];
    ACcp5_150 = OMcp5_212*ORcp5_313+OMcp5_215*ORcp5_350+OMcp5_27*(ORcp5_310+ORcp5_312)-OMcp5_312*ORcp5_213-OMcp5_315*
 ORcp5_250-OMcp5_37*ORcp5_210-OMcp5_37*ORcp5_212+OPcp5_212*RLcp5_313+OPcp5_215*RLcp5_350+OPcp5_27*RLcp5_310+OPcp5_27*
 RLcp5_312-OPcp5_312*RLcp5_213-OPcp5_315*RLcp5_250-OPcp5_37*RLcp5_210-OPcp5_37*RLcp5_212-ORcp5_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*
 qd[1]*qd[3]*C1-qd[1]*(ORcp5_22+(2.0)*qd[2]*S1);
    ACcp5_250 = -(OMcp5_112*ORcp5_313+OMcp5_115*ORcp5_350+OMcp5_17*(ORcp5_310+ORcp5_312)-OMcp5_312*ORcp5_113-OMcp5_315*
 ORcp5_150-OMcp5_37*ORcp5_110-OMcp5_37*ORcp5_112+OPcp5_112*RLcp5_313+OPcp5_115*RLcp5_350+OPcp5_17*RLcp5_310+OPcp5_17*
 RLcp5_312-OPcp5_312*RLcp5_113-OPcp5_315*RLcp5_150-OPcp5_37*RLcp5_110-OPcp5_37*RLcp5_112-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp5_12+(2.0)
 *qd[2]*C1)-qd[1]*(ORcp5_13-(2.0)*qd[3]*S1));
    ACcp5_350 = qdd[4]+OMcp5_112*ORcp5_213+OMcp5_115*ORcp5_250+OMcp5_17*ORcp5_210+OMcp5_17*ORcp5_212-OMcp5_212*ORcp5_113-
 OMcp5_215*ORcp5_150-OMcp5_27*ORcp5_110-OMcp5_27*ORcp5_112+OPcp5_112*RLcp5_213+OPcp5_115*RLcp5_250+OPcp5_17*RLcp5_210+
 OPcp5_17*RLcp5_212-OPcp5_212*RLcp5_113-OPcp5_215*RLcp5_150-OPcp5_27*RLcp5_110-OPcp5_27*RLcp5_112;

// = = Block_1_0_0_6_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp5_150;
    sens->P[2] = POcp5_250;
    sens->P[3] = POcp5_350;
    sens->R[1][1] = ROcp5_115;
    sens->R[1][2] = ROcp5_215;
    sens->R[1][3] = ROcp5_315;
    sens->R[2][1] = ROcp5_415;
    sens->R[2][2] = ROcp5_515;
    sens->R[2][3] = ROcp5_615;
    sens->R[3][1] = ROcp5_714;
    sens->R[3][2] = ROcp5_814;
    sens->R[3][3] = ROcp5_914;
    sens->V[1] = VIcp5_150;
    sens->V[2] = VIcp5_250;
    sens->V[3] = VIcp5_350;
    sens->OM[1] = OMcp5_115;
    sens->OM[2] = OMcp5_215;
    sens->OM[3] = OMcp5_315;
    sens->J[1][1] = JTcp5_150_1;
    sens->J[1][2] = C1;
    sens->J[1][3] = -S1;
    sens->J[1][5] = JTcp5_150_5;
    sens->J[1][6] = JTcp5_150_6;
    sens->J[1][7] = JTcp5_150_7;
    sens->J[1][10] = JTcp5_150_8;
    sens->J[1][12] = JTcp5_150_9;
    sens->J[1][13] = JTcp5_150_10;
    sens->J[1][14] = JTcp5_150_11;
    sens->J[1][15] = JTcp5_150_12;
    sens->J[2][1] = JTcp5_250_1;
    sens->J[2][2] = S1;
    sens->J[2][3] = C1;
    sens->J[2][5] = JTcp5_250_5;
    sens->J[2][6] = JTcp5_250_6;
    sens->J[2][7] = JTcp5_250_7;
    sens->J[2][10] = JTcp5_250_8;
    sens->J[2][12] = JTcp5_250_9;
    sens->J[2][13] = JTcp5_250_10;
    sens->J[2][14] = JTcp5_250_11;
    sens->J[2][15] = JTcp5_250_12;
    sens->J[3][4] = (1.0);
    sens->J[3][6] = JTcp5_350_6;
    sens->J[3][7] = JTcp5_350_7;
    sens->J[3][10] = JTcp5_350_8;
    sens->J[3][12] = JTcp5_350_9;
    sens->J[3][13] = JTcp5_350_10;
    sens->J[3][14] = JTcp5_350_11;
    sens->J[3][15] = JTcp5_350_12;
    sens->J[4][6] = C1p5;
    sens->J[4][7] = ROcp5_46;
    sens->J[4][10] = ROcp5_46;
    sens->J[4][12] = ROcp5_110;
    sens->J[4][13] = ROcp5_110;
    sens->J[4][14] = ROcp5_413;
    sens->J[4][15] = ROcp5_714;
    sens->J[5][6] = S1p5;
    sens->J[5][7] = ROcp5_56;
    sens->J[5][10] = ROcp5_56;
    sens->J[5][12] = ROcp5_210;
    sens->J[5][13] = ROcp5_210;
    sens->J[5][14] = ROcp5_513;
    sens->J[5][15] = ROcp5_814;
    sens->J[6][1] = (1.0);
    sens->J[6][5] = (1.0);
    sens->J[6][7] = S6;
    sens->J[6][10] = S6;
    sens->J[6][12] = ROcp5_310;
    sens->J[6][13] = ROcp5_310;
    sens->J[6][14] = ROcp5_613;
    sens->J[6][15] = ROcp5_914;
    sens->A[1] = ACcp5_150;
    sens->A[2] = ACcp5_250;
    sens->A[3] = ACcp5_350;
    sens->OMP[1] = OPcp5_115;
    sens->OMP[2] = OPcp5_215;
    sens->OMP[3] = OPcp5_315;
 
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
    RLcp6_116 = ROcp6_115*s->dpt[1][20]+ROcp6_415*s->dpt[2][20]+ROcp6_714*s->dpt[3][20];
    RLcp6_216 = ROcp6_215*s->dpt[1][20]+ROcp6_515*s->dpt[2][20]+ROcp6_814*s->dpt[3][20];
    RLcp6_316 = ROcp6_315*s->dpt[1][20]+ROcp6_615*s->dpt[2][20]+ROcp6_914*s->dpt[3][20];
    POcp6_116 = RLcp6_110+RLcp6_112+RLcp6_113+RLcp6_116+RLcp6_12+RLcp6_13;
    POcp6_216 = RLcp6_210+RLcp6_212+RLcp6_213+RLcp6_216+RLcp6_22+RLcp6_23;
    POcp6_316 = RLcp6_310+RLcp6_312+RLcp6_313+RLcp6_316+q[4];
    JTcp6_116_1 = -(RLcp6_210+RLcp6_212+RLcp6_213+RLcp6_216+RLcp6_22+RLcp6_23);
    JTcp6_216_1 = RLcp6_110+RLcp6_112+RLcp6_113+RLcp6_116+RLcp6_12+RLcp6_13;
    JTcp6_116_5 = -(RLcp6_210+RLcp6_212+RLcp6_213+RLcp6_216);
    JTcp6_216_5 = RLcp6_110+RLcp6_112+RLcp6_113+RLcp6_116;
    JTcp6_116_6 = S1p5*(RLcp6_310+RLcp6_312+RLcp6_313+RLcp6_316);
    JTcp6_216_6 = -C1p5*(RLcp6_310+RLcp6_312+RLcp6_313+RLcp6_316);
    JTcp6_316_6 = C1p5*(RLcp6_210+RLcp6_212+RLcp6_213+RLcp6_216)-S1p5*(RLcp6_110+RLcp6_112)-S1p5*(RLcp6_113+RLcp6_116);
    JTcp6_116_7 = ROcp6_56*(RLcp6_310+RLcp6_312+RLcp6_313+RLcp6_316)-S6*(RLcp6_210+RLcp6_212)-S6*(RLcp6_213+RLcp6_216);
    JTcp6_216_7 = -(ROcp6_46*(RLcp6_310+RLcp6_312+RLcp6_313+RLcp6_316)-S6*(RLcp6_110+RLcp6_112)-S6*(RLcp6_113+RLcp6_116));
    JTcp6_316_7 = ROcp6_46*(RLcp6_210+RLcp6_212+RLcp6_213+RLcp6_216)-ROcp6_56*(RLcp6_110+RLcp6_112)-ROcp6_56*(RLcp6_113+
 RLcp6_116);
    JTcp6_116_8 = ROcp6_56*(RLcp6_312+RLcp6_313)-S6*(RLcp6_212+RLcp6_213)-RLcp6_216*S6+RLcp6_316*ROcp6_56;
    JTcp6_216_8 = RLcp6_116*S6-RLcp6_316*ROcp6_46-ROcp6_46*(RLcp6_312+RLcp6_313)+S6*(RLcp6_112+RLcp6_113);
    JTcp6_316_8 = ROcp6_46*(RLcp6_212+RLcp6_213)-ROcp6_56*(RLcp6_112+RLcp6_113)-RLcp6_116*ROcp6_56+RLcp6_216*ROcp6_46;
    JTcp6_116_9 = ROcp6_210*(RLcp6_313+RLcp6_316)-ROcp6_310*(RLcp6_213+RLcp6_216);
    JTcp6_216_9 = -(ROcp6_110*(RLcp6_313+RLcp6_316)-ROcp6_310*(RLcp6_113+RLcp6_116));
    JTcp6_316_9 = ROcp6_110*(RLcp6_213+RLcp6_216)-ROcp6_210*(RLcp6_113+RLcp6_116);
    JTcp6_116_10 = -(RLcp6_216*ROcp6_310-RLcp6_316*ROcp6_210);
    JTcp6_216_10 = RLcp6_116*ROcp6_310-RLcp6_316*ROcp6_110;
    JTcp6_316_10 = -(RLcp6_116*ROcp6_210-RLcp6_216*ROcp6_110);
    JTcp6_116_11 = -(RLcp6_216*ROcp6_613-RLcp6_316*ROcp6_513);
    JTcp6_216_11 = RLcp6_116*ROcp6_613-RLcp6_316*ROcp6_413;
    JTcp6_316_11 = -(RLcp6_116*ROcp6_513-RLcp6_216*ROcp6_413);
    JTcp6_116_12 = -(RLcp6_216*ROcp6_914-RLcp6_316*ROcp6_814);
    JTcp6_216_12 = RLcp6_116*ROcp6_914-RLcp6_316*ROcp6_714;
    JTcp6_316_12 = -(RLcp6_116*ROcp6_814-RLcp6_216*ROcp6_714);
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
    sens->J[1][1] = JTcp6_116_1;
    sens->J[1][2] = C1;
    sens->J[1][3] = -S1;
    sens->J[1][5] = JTcp6_116_5;
    sens->J[1][6] = JTcp6_116_6;
    sens->J[1][7] = JTcp6_116_7;
    sens->J[1][10] = JTcp6_116_8;
    sens->J[1][12] = JTcp6_116_9;
    sens->J[1][13] = JTcp6_116_10;
    sens->J[1][14] = JTcp6_116_11;
    sens->J[1][15] = JTcp6_116_12;
    sens->J[2][1] = JTcp6_216_1;
    sens->J[2][2] = S1;
    sens->J[2][3] = C1;
    sens->J[2][5] = JTcp6_216_5;
    sens->J[2][6] = JTcp6_216_6;
    sens->J[2][7] = JTcp6_216_7;
    sens->J[2][10] = JTcp6_216_8;
    sens->J[2][12] = JTcp6_216_9;
    sens->J[2][13] = JTcp6_216_10;
    sens->J[2][14] = JTcp6_216_11;
    sens->J[2][15] = JTcp6_216_12;
    sens->J[3][4] = (1.0);
    sens->J[3][6] = JTcp6_316_6;
    sens->J[3][7] = JTcp6_316_7;
    sens->J[3][10] = JTcp6_316_8;
    sens->J[3][12] = JTcp6_316_9;
    sens->J[3][13] = JTcp6_316_10;
    sens->J[3][14] = JTcp6_316_11;
    sens->J[3][15] = JTcp6_316_12;
    sens->J[4][6] = C1p5;
    sens->J[4][7] = ROcp6_46;
    sens->J[4][10] = ROcp6_46;
    sens->J[4][12] = ROcp6_110;
    sens->J[4][13] = ROcp6_110;
    sens->J[4][14] = ROcp6_413;
    sens->J[4][15] = ROcp6_714;
    sens->J[4][16] = ROcp6_115;
    sens->J[4][17] = ROcp6_416;
    sens->J[5][6] = S1p5;
    sens->J[5][7] = ROcp6_56;
    sens->J[5][10] = ROcp6_56;
    sens->J[5][12] = ROcp6_210;
    sens->J[5][13] = ROcp6_210;
    sens->J[5][14] = ROcp6_513;
    sens->J[5][15] = ROcp6_814;
    sens->J[5][16] = ROcp6_215;
    sens->J[5][17] = ROcp6_516;
    sens->J[6][1] = (1.0);
    sens->J[6][5] = (1.0);
    sens->J[6][7] = S6;
    sens->J[6][10] = S6;
    sens->J[6][12] = ROcp6_310;
    sens->J[6][13] = ROcp6_310;
    sens->J[6][14] = ROcp6_613;
    sens->J[6][15] = ROcp6_914;
    sens->J[6][16] = ROcp6_315;
    sens->J[6][17] = ROcp6_616;
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
    RLcp7_119 = ROcp7_418*s->dpt[2][23];
    RLcp7_219 = ROcp7_518*s->dpt[2][23];
    RLcp7_319 = ROcp7_618*s->dpt[2][23];
    POcp7_119 = RLcp7_110+RLcp7_118+RLcp7_119+RLcp7_12+RLcp7_13;
    POcp7_219 = RLcp7_210+RLcp7_218+RLcp7_219+RLcp7_22+RLcp7_23;
    POcp7_319 = RLcp7_310+RLcp7_318+RLcp7_319+q[4];
    JTcp7_119_1 = -(RLcp7_210+RLcp7_218+RLcp7_219+RLcp7_22+RLcp7_23);
    JTcp7_219_1 = RLcp7_110+RLcp7_118+RLcp7_119+RLcp7_12+RLcp7_13;
    JTcp7_119_5 = -(RLcp7_210+RLcp7_218+RLcp7_219);
    JTcp7_219_5 = RLcp7_110+RLcp7_118+RLcp7_119;
    JTcp7_119_6 = S1p5*(RLcp7_310+RLcp7_318+RLcp7_319);
    JTcp7_219_6 = -C1p5*(RLcp7_310+RLcp7_318+RLcp7_319);
    JTcp7_319_6 = C1p5*(RLcp7_210+RLcp7_218)-S1p5*(RLcp7_110+RLcp7_118)-RLcp7_119*S1p5+RLcp7_219*C1p5;
    JTcp7_119_7 = ROcp7_56*(RLcp7_310+RLcp7_318)-S6*(RLcp7_210+RLcp7_218)-RLcp7_219*S6+RLcp7_319*ROcp7_56;
    JTcp7_219_7 = RLcp7_119*S6-RLcp7_319*ROcp7_46-ROcp7_46*(RLcp7_310+RLcp7_318)+S6*(RLcp7_110+RLcp7_118);
    JTcp7_319_7 = ROcp7_46*(RLcp7_210+RLcp7_218)-ROcp7_56*(RLcp7_110+RLcp7_118)-RLcp7_119*ROcp7_56+RLcp7_219*ROcp7_46;
    JTcp7_119_8 = ROcp7_56*(RLcp7_318+RLcp7_319)-S6*(RLcp7_218+RLcp7_219);
    JTcp7_219_8 = -(ROcp7_46*(RLcp7_318+RLcp7_319)-S6*(RLcp7_118+RLcp7_119));
    JTcp7_319_8 = ROcp7_46*(RLcp7_218+RLcp7_219)-ROcp7_56*(RLcp7_118+RLcp7_119);
    JTcp7_119_9 = -(RLcp7_219*ROcp7_310-RLcp7_319*ROcp7_210);
    JTcp7_219_9 = RLcp7_119*ROcp7_310-RLcp7_319*ROcp7_110;
    JTcp7_319_9 = -(RLcp7_119*ROcp7_210-RLcp7_219*ROcp7_110);
    OMcp7_119 = OMcp7_118+ROcp7_110*qd[19];
    OMcp7_219 = OMcp7_218+ROcp7_210*qd[19];
    OMcp7_319 = OMcp7_318+ROcp7_310*qd[19];
    ORcp7_119 = OMcp7_218*RLcp7_319-OMcp7_318*RLcp7_219;
    ORcp7_219 = -(OMcp7_118*RLcp7_319-OMcp7_318*RLcp7_119);
    ORcp7_319 = OMcp7_118*RLcp7_219-OMcp7_218*RLcp7_119;
    VIcp7_119 = ORcp7_110+ORcp7_118+ORcp7_119+ORcp7_12+ORcp7_13+qd[2]*C1-qd[3]*S1;
    VIcp7_219 = ORcp7_210+ORcp7_218+ORcp7_219+ORcp7_22+ORcp7_23+qd[2]*S1+qd[3]*C1;
    VIcp7_319 = ORcp7_310+ORcp7_318+ORcp7_319+qd[4];
    ACcp7_119 = OMcp7_218*ORcp7_319+OMcp7_27*(ORcp7_310+ORcp7_318)-OMcp7_318*ORcp7_219-OMcp7_37*ORcp7_210-OMcp7_37*
 ORcp7_218+OPcp7_218*RLcp7_319+OPcp7_27*RLcp7_310+OPcp7_27*RLcp7_318-OPcp7_318*RLcp7_219-OPcp7_37*RLcp7_210-OPcp7_37*
 RLcp7_218-ORcp7_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp7_22+(2.0)*qd[2]*S1);
    ACcp7_219 = -(OMcp7_118*ORcp7_319+OMcp7_17*(ORcp7_310+ORcp7_318)-OMcp7_318*ORcp7_119-OMcp7_37*ORcp7_110-OMcp7_37*
 ORcp7_118+OPcp7_118*RLcp7_319+OPcp7_17*RLcp7_310+OPcp7_17*RLcp7_318-OPcp7_318*RLcp7_119-OPcp7_37*RLcp7_110-OPcp7_37*
 RLcp7_118-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp7_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp7_13-(2.0)*qd[3]*S1));
    ACcp7_319 = qdd[4]+OMcp7_118*ORcp7_219+OMcp7_17*ORcp7_210+OMcp7_17*ORcp7_218-OMcp7_218*ORcp7_119-OMcp7_27*ORcp7_110-
 OMcp7_27*ORcp7_118+OPcp7_118*RLcp7_219+OPcp7_17*RLcp7_210+OPcp7_17*RLcp7_218-OPcp7_218*RLcp7_119-OPcp7_27*RLcp7_110-OPcp7_27
 *RLcp7_118;
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

// = = Block_1_0_0_8_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp7_119;
    sens->P[2] = POcp7_219;
    sens->P[3] = POcp7_319;
    sens->R[1][1] = ROcp7_121;
    sens->R[1][2] = ROcp7_221;
    sens->R[1][3] = ROcp7_321;
    sens->R[2][1] = ROcp7_421;
    sens->R[2][2] = ROcp7_521;
    sens->R[2][3] = ROcp7_621;
    sens->R[3][1] = ROcp7_720;
    sens->R[3][2] = ROcp7_820;
    sens->R[3][3] = ROcp7_920;
    sens->V[1] = VIcp7_119;
    sens->V[2] = VIcp7_219;
    sens->V[3] = VIcp7_319;
    sens->OM[1] = OMcp7_121;
    sens->OM[2] = OMcp7_221;
    sens->OM[3] = OMcp7_321;
    sens->J[1][1] = JTcp7_119_1;
    sens->J[1][2] = C1;
    sens->J[1][3] = -S1;
    sens->J[1][5] = JTcp7_119_5;
    sens->J[1][6] = JTcp7_119_6;
    sens->J[1][7] = JTcp7_119_7;
    sens->J[1][10] = JTcp7_119_8;
    sens->J[1][18] = JTcp7_119_9;
    sens->J[2][1] = JTcp7_219_1;
    sens->J[2][2] = S1;
    sens->J[2][3] = C1;
    sens->J[2][5] = JTcp7_219_5;
    sens->J[2][6] = JTcp7_219_6;
    sens->J[2][7] = JTcp7_219_7;
    sens->J[2][10] = JTcp7_219_8;
    sens->J[2][18] = JTcp7_219_9;
    sens->J[3][4] = (1.0);
    sens->J[3][6] = JTcp7_319_6;
    sens->J[3][7] = JTcp7_319_7;
    sens->J[3][10] = JTcp7_319_8;
    sens->J[3][18] = JTcp7_319_9;
    sens->J[4][6] = C1p5;
    sens->J[4][7] = ROcp7_46;
    sens->J[4][10] = ROcp7_46;
    sens->J[4][18] = ROcp7_110;
    sens->J[4][19] = ROcp7_110;
    sens->J[4][20] = ROcp7_419;
    sens->J[4][21] = ROcp7_720;
    sens->J[5][6] = S1p5;
    sens->J[5][7] = ROcp7_56;
    sens->J[5][10] = ROcp7_56;
    sens->J[5][18] = ROcp7_210;
    sens->J[5][19] = ROcp7_210;
    sens->J[5][20] = ROcp7_519;
    sens->J[5][21] = ROcp7_820;
    sens->J[6][1] = (1.0);
    sens->J[6][5] = (1.0);
    sens->J[6][7] = S6;
    sens->J[6][10] = S6;
    sens->J[6][18] = ROcp7_310;
    sens->J[6][19] = ROcp7_310;
    sens->J[6][20] = ROcp7_619;
    sens->J[6][21] = ROcp7_920;
    sens->A[1] = ACcp7_119;
    sens->A[2] = ACcp7_219;
    sens->A[3] = ACcp7_319;
    sens->OMP[1] = OPcp7_121;
    sens->OMP[2] = OPcp7_221;
    sens->OMP[3] = OPcp7_321;
 
// 
break;
case 9:
 


// = = Block_1_0_0_9_0_1 = = 
 
// Sensor Kinematics 


    ROcp8_46 = -S1p5*C6;
    ROcp8_56 = C1p5*C6;
    ROcp8_76 = S1p5*S6;
    ROcp8_86 = -C1p5*S6;
    ROcp8_17 = -(ROcp8_76*S7-C1p5*C7);
    ROcp8_27 = -(ROcp8_86*S7-S1p5*C7);
    ROcp8_77 = ROcp8_76*C7+C1p5*S7;
    ROcp8_87 = ROcp8_86*C7+S1p5*S7;
    RLcp8_12 = q[2]*C1;
    RLcp8_22 = q[2]*S1;
    ORcp8_12 = -RLcp8_22*qd[1];
    ORcp8_22 = RLcp8_12*qd[1];
    RLcp8_13 = -q[3]*S1;
    RLcp8_23 = q[3]*C1;
    ORcp8_13 = -RLcp8_23*qd[1];
    ORcp8_23 = RLcp8_13*qd[1];
    OMcp8_35 = qd[1]+qd[5];
    OMcp8_16 = qd[6]*C1p5;
    OMcp8_26 = qd[6]*S1p5;
    OMcp8_17 = OMcp8_16+ROcp8_46*qd[7];
    OMcp8_27 = OMcp8_26+ROcp8_56*qd[7];
    OMcp8_37 = OMcp8_35+qd[7]*S6;
    OPcp8_17 = -(OMcp8_35*qd[6]*S1p5-ROcp8_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp8_26*S6-OMcp8_35*ROcp8_56));
    OPcp8_27 = OMcp8_35*qd[6]*C1p5+ROcp8_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp8_16*S6-OMcp8_35*ROcp8_46);
    OPcp8_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_9_0_3 = = 
 
// Sensor Kinematics 


    ROcp8_110 = ROcp8_17*C10-ROcp8_77*S10;
    ROcp8_210 = ROcp8_27*C10-ROcp8_87*S10;
    ROcp8_310 = -S10p7*C6;
    ROcp8_710 = ROcp8_17*S10+ROcp8_77*C10;
    ROcp8_810 = ROcp8_27*S10+ROcp8_87*C10;
    ROcp8_910 = C10p7*C6;
    RLcp8_110 = ROcp8_17*s->dpt[1][5]+ROcp8_77*s->dpt[3][5];
    RLcp8_210 = ROcp8_27*s->dpt[1][5]+ROcp8_87*s->dpt[3][5];
    RLcp8_310 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp8_110 = OMcp8_27*RLcp8_310-OMcp8_37*RLcp8_210;
    ORcp8_210 = -(OMcp8_17*RLcp8_310-OMcp8_37*RLcp8_110);
    ORcp8_310 = OMcp8_17*RLcp8_210-OMcp8_27*RLcp8_110;

// = = Block_1_0_0_9_0_6 = = 
 
// Sensor Kinematics 


    ROcp8_418 = ROcp8_46*C18+ROcp8_710*S18;
    ROcp8_518 = ROcp8_56*C18+ROcp8_810*S18;
    ROcp8_618 = ROcp8_910*S18+C18*S6;
    ROcp8_718 = -(ROcp8_46*S18-ROcp8_710*C18);
    ROcp8_818 = -(ROcp8_56*S18-ROcp8_810*C18);
    ROcp8_918 = ROcp8_910*C18-S18*S6;
    ROcp8_419 = ROcp8_418*C19+ROcp8_718*S19;
    ROcp8_519 = ROcp8_518*C19+ROcp8_818*S19;
    ROcp8_619 = ROcp8_618*C19+ROcp8_918*S19;
    ROcp8_719 = -(ROcp8_418*S19-ROcp8_718*C19);
    ROcp8_819 = -(ROcp8_518*S19-ROcp8_818*C19);
    ROcp8_919 = -(ROcp8_618*S19-ROcp8_918*C19);
    ROcp8_120 = ROcp8_110*C20-ROcp8_719*S20;
    ROcp8_220 = ROcp8_210*C20-ROcp8_819*S20;
    ROcp8_320 = ROcp8_310*C20-ROcp8_919*S20;
    ROcp8_720 = ROcp8_110*S20+ROcp8_719*C20;
    ROcp8_820 = ROcp8_210*S20+ROcp8_819*C20;
    ROcp8_920 = ROcp8_310*S20+ROcp8_919*C20;
    ROcp8_121 = ROcp8_120*C21+ROcp8_419*S21;
    ROcp8_221 = ROcp8_220*C21+ROcp8_519*S21;
    ROcp8_321 = ROcp8_320*C21+ROcp8_619*S21;
    ROcp8_421 = -(ROcp8_120*S21-ROcp8_419*C21);
    ROcp8_521 = -(ROcp8_220*S21-ROcp8_519*C21);
    ROcp8_621 = -(ROcp8_320*S21-ROcp8_619*C21);
    RLcp8_118 = ROcp8_46*s->dpt[2][12]+ROcp8_710*s->dpt[3][12];
    RLcp8_218 = ROcp8_56*s->dpt[2][12]+ROcp8_810*s->dpt[3][12];
    RLcp8_318 = ROcp8_910*s->dpt[3][12]+s->dpt[2][12]*S6;
    OMcp8_118 = OMcp8_17+ROcp8_110*qd[18];
    OMcp8_218 = OMcp8_27+ROcp8_210*qd[18];
    OMcp8_318 = OMcp8_37+ROcp8_310*qd[18];
    ORcp8_118 = OMcp8_27*RLcp8_318-OMcp8_37*RLcp8_218;
    ORcp8_218 = -(OMcp8_17*RLcp8_318-OMcp8_37*RLcp8_118);
    ORcp8_318 = OMcp8_17*RLcp8_218-OMcp8_27*RLcp8_118;
    OPcp8_118 = OPcp8_17+ROcp8_110*qdd[18]+qd[18]*(OMcp8_27*ROcp8_310-OMcp8_37*ROcp8_210);
    OPcp8_218 = OPcp8_27+ROcp8_210*qdd[18]-qd[18]*(OMcp8_17*ROcp8_310-OMcp8_37*ROcp8_110);
    OPcp8_318 = OPcp8_37+ROcp8_310*qdd[18]+qd[18]*(OMcp8_17*ROcp8_210-OMcp8_27*ROcp8_110);
    RLcp8_119 = ROcp8_418*s->dpt[2][23];
    RLcp8_219 = ROcp8_518*s->dpt[2][23];
    RLcp8_319 = ROcp8_618*s->dpt[2][23];
    OMcp8_119 = OMcp8_118+ROcp8_110*qd[19];
    OMcp8_219 = OMcp8_218+ROcp8_210*qd[19];
    OMcp8_319 = OMcp8_318+ROcp8_310*qd[19];
    ORcp8_119 = OMcp8_218*RLcp8_319-OMcp8_318*RLcp8_219;
    ORcp8_219 = -(OMcp8_118*RLcp8_319-OMcp8_318*RLcp8_119);
    ORcp8_319 = OMcp8_118*RLcp8_219-OMcp8_218*RLcp8_119;
    OMcp8_120 = OMcp8_119+ROcp8_419*qd[20];
    OMcp8_220 = OMcp8_219+ROcp8_519*qd[20];
    OMcp8_320 = OMcp8_319+ROcp8_619*qd[20];
    OMcp8_121 = OMcp8_120+ROcp8_720*qd[21];
    OMcp8_221 = OMcp8_220+ROcp8_820*qd[21];
    OMcp8_321 = OMcp8_320+ROcp8_920*qd[21];
    OPcp8_121 = OPcp8_118+ROcp8_110*qdd[19]+ROcp8_419*qdd[20]+ROcp8_720*qdd[21]+qd[19]*(OMcp8_218*ROcp8_310-OMcp8_318*
 ROcp8_210)+qd[20]*(OMcp8_219*ROcp8_619-OMcp8_319*ROcp8_519)+qd[21]*(OMcp8_220*ROcp8_920-OMcp8_320*ROcp8_820);
    OPcp8_221 = OPcp8_218+ROcp8_210*qdd[19]+ROcp8_519*qdd[20]+ROcp8_820*qdd[21]-qd[19]*(OMcp8_118*ROcp8_310-OMcp8_318*
 ROcp8_110)-qd[20]*(OMcp8_119*ROcp8_619-OMcp8_319*ROcp8_419)-qd[21]*(OMcp8_120*ROcp8_920-OMcp8_320*ROcp8_720);
    OPcp8_321 = OPcp8_318+ROcp8_310*qdd[19]+ROcp8_619*qdd[20]+ROcp8_920*qdd[21]+qd[19]*(OMcp8_118*ROcp8_210-OMcp8_218*
 ROcp8_110)+qd[20]*(OMcp8_119*ROcp8_519-OMcp8_219*ROcp8_419)+qd[21]*(OMcp8_120*ROcp8_820-OMcp8_220*ROcp8_720);
    RLcp8_153 = ROcp8_720*s->dpt[3][26];
    RLcp8_253 = ROcp8_820*s->dpt[3][26];
    RLcp8_353 = ROcp8_920*s->dpt[3][26];
    POcp8_153 = RLcp8_110+RLcp8_118+RLcp8_119+RLcp8_12+RLcp8_13+RLcp8_153;
    POcp8_253 = RLcp8_210+RLcp8_218+RLcp8_219+RLcp8_22+RLcp8_23+RLcp8_253;
    POcp8_353 = RLcp8_310+RLcp8_318+RLcp8_319+RLcp8_353+q[4];
    JTcp8_153_1 = -(RLcp8_210+RLcp8_218+RLcp8_219+RLcp8_22+RLcp8_23+RLcp8_253);
    JTcp8_253_1 = RLcp8_110+RLcp8_118+RLcp8_119+RLcp8_12+RLcp8_13+RLcp8_153;
    JTcp8_153_5 = -(RLcp8_210+RLcp8_218+RLcp8_219+RLcp8_253);
    JTcp8_253_5 = RLcp8_110+RLcp8_118+RLcp8_119+RLcp8_153;
    JTcp8_153_6 = S1p5*(RLcp8_310+RLcp8_318+RLcp8_319+RLcp8_353);
    JTcp8_253_6 = -C1p5*(RLcp8_310+RLcp8_318+RLcp8_319+RLcp8_353);
    JTcp8_353_6 = C1p5*(RLcp8_210+RLcp8_218+RLcp8_219+RLcp8_253)-S1p5*(RLcp8_110+RLcp8_118)-S1p5*(RLcp8_119+RLcp8_153);
    JTcp8_153_7 = ROcp8_56*(RLcp8_310+RLcp8_318+RLcp8_319+RLcp8_353)-S6*(RLcp8_210+RLcp8_218)-S6*(RLcp8_219+RLcp8_253);
    JTcp8_253_7 = -(ROcp8_46*(RLcp8_310+RLcp8_318+RLcp8_319+RLcp8_353)-S6*(RLcp8_110+RLcp8_118)-S6*(RLcp8_119+RLcp8_153));
    JTcp8_353_7 = ROcp8_46*(RLcp8_210+RLcp8_218+RLcp8_219+RLcp8_253)-ROcp8_56*(RLcp8_110+RLcp8_118)-ROcp8_56*(RLcp8_119+
 RLcp8_153);
    JTcp8_153_8 = ROcp8_56*(RLcp8_318+RLcp8_319)-S6*(RLcp8_218+RLcp8_219)-RLcp8_253*S6+RLcp8_353*ROcp8_56;
    JTcp8_253_8 = RLcp8_153*S6-RLcp8_353*ROcp8_46-ROcp8_46*(RLcp8_318+RLcp8_319)+S6*(RLcp8_118+RLcp8_119);
    JTcp8_353_8 = ROcp8_46*(RLcp8_218+RLcp8_219)-ROcp8_56*(RLcp8_118+RLcp8_119)-RLcp8_153*ROcp8_56+RLcp8_253*ROcp8_46;
    JTcp8_153_9 = ROcp8_210*(RLcp8_319+RLcp8_353)-ROcp8_310*(RLcp8_219+RLcp8_253);
    JTcp8_253_9 = -(ROcp8_110*(RLcp8_319+RLcp8_353)-ROcp8_310*(RLcp8_119+RLcp8_153));
    JTcp8_353_9 = ROcp8_110*(RLcp8_219+RLcp8_253)-ROcp8_210*(RLcp8_119+RLcp8_153);
    JTcp8_153_10 = -(RLcp8_253*ROcp8_310-RLcp8_353*ROcp8_210);
    JTcp8_253_10 = RLcp8_153*ROcp8_310-RLcp8_353*ROcp8_110;
    JTcp8_353_10 = -(RLcp8_153*ROcp8_210-RLcp8_253*ROcp8_110);
    JTcp8_153_11 = -(RLcp8_253*ROcp8_619-RLcp8_353*ROcp8_519);
    JTcp8_253_11 = RLcp8_153*ROcp8_619-RLcp8_353*ROcp8_419;
    JTcp8_353_11 = -(RLcp8_153*ROcp8_519-RLcp8_253*ROcp8_419);
    JTcp8_153_12 = -(RLcp8_253*ROcp8_920-RLcp8_353*ROcp8_820);
    JTcp8_253_12 = RLcp8_153*ROcp8_920-RLcp8_353*ROcp8_720;
    JTcp8_353_12 = -(RLcp8_153*ROcp8_820-RLcp8_253*ROcp8_720);
    ORcp8_153 = OMcp8_221*RLcp8_353-OMcp8_321*RLcp8_253;
    ORcp8_253 = -(OMcp8_121*RLcp8_353-OMcp8_321*RLcp8_153);
    ORcp8_353 = OMcp8_121*RLcp8_253-OMcp8_221*RLcp8_153;
    VIcp8_153 = ORcp8_110+ORcp8_118+ORcp8_119+ORcp8_12+ORcp8_13+ORcp8_153+qd[2]*C1-qd[3]*S1;
    VIcp8_253 = ORcp8_210+ORcp8_218+ORcp8_219+ORcp8_22+ORcp8_23+ORcp8_253+qd[2]*S1+qd[3]*C1;
    VIcp8_353 = ORcp8_310+ORcp8_318+ORcp8_319+ORcp8_353+qd[4];
    ACcp8_153 = OMcp8_218*ORcp8_319+OMcp8_221*ORcp8_353+OMcp8_27*(ORcp8_310+ORcp8_318)-OMcp8_318*ORcp8_219-OMcp8_321*
 ORcp8_253-OMcp8_37*ORcp8_210-OMcp8_37*ORcp8_218+OPcp8_218*RLcp8_319+OPcp8_221*RLcp8_353+OPcp8_27*RLcp8_310+OPcp8_27*
 RLcp8_318-OPcp8_318*RLcp8_219-OPcp8_321*RLcp8_253-OPcp8_37*RLcp8_210-OPcp8_37*RLcp8_218-ORcp8_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*
 qd[1]*qd[3]*C1-qd[1]*(ORcp8_22+(2.0)*qd[2]*S1);
    ACcp8_253 = -(OMcp8_118*ORcp8_319+OMcp8_121*ORcp8_353+OMcp8_17*(ORcp8_310+ORcp8_318)-OMcp8_318*ORcp8_119-OMcp8_321*
 ORcp8_153-OMcp8_37*ORcp8_110-OMcp8_37*ORcp8_118+OPcp8_118*RLcp8_319+OPcp8_121*RLcp8_353+OPcp8_17*RLcp8_310+OPcp8_17*
 RLcp8_318-OPcp8_318*RLcp8_119-OPcp8_321*RLcp8_153-OPcp8_37*RLcp8_110-OPcp8_37*RLcp8_118-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp8_12+(2.0)
 *qd[2]*C1)-qd[1]*(ORcp8_13-(2.0)*qd[3]*S1));
    ACcp8_353 = qdd[4]+OMcp8_118*ORcp8_219+OMcp8_121*ORcp8_253+OMcp8_17*ORcp8_210+OMcp8_17*ORcp8_218-OMcp8_218*ORcp8_119-
 OMcp8_221*ORcp8_153-OMcp8_27*ORcp8_110-OMcp8_27*ORcp8_118+OPcp8_118*RLcp8_219+OPcp8_121*RLcp8_253+OPcp8_17*RLcp8_210+
 OPcp8_17*RLcp8_218-OPcp8_218*RLcp8_119-OPcp8_221*RLcp8_153-OPcp8_27*RLcp8_110-OPcp8_27*RLcp8_118;

// = = Block_1_0_0_9_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp8_153;
    sens->P[2] = POcp8_253;
    sens->P[3] = POcp8_353;
    sens->R[1][1] = ROcp8_121;
    sens->R[1][2] = ROcp8_221;
    sens->R[1][3] = ROcp8_321;
    sens->R[2][1] = ROcp8_421;
    sens->R[2][2] = ROcp8_521;
    sens->R[2][3] = ROcp8_621;
    sens->R[3][1] = ROcp8_720;
    sens->R[3][2] = ROcp8_820;
    sens->R[3][3] = ROcp8_920;
    sens->V[1] = VIcp8_153;
    sens->V[2] = VIcp8_253;
    sens->V[3] = VIcp8_353;
    sens->OM[1] = OMcp8_121;
    sens->OM[2] = OMcp8_221;
    sens->OM[3] = OMcp8_321;
    sens->J[1][1] = JTcp8_153_1;
    sens->J[1][2] = C1;
    sens->J[1][3] = -S1;
    sens->J[1][5] = JTcp8_153_5;
    sens->J[1][6] = JTcp8_153_6;
    sens->J[1][7] = JTcp8_153_7;
    sens->J[1][10] = JTcp8_153_8;
    sens->J[1][18] = JTcp8_153_9;
    sens->J[1][19] = JTcp8_153_10;
    sens->J[1][20] = JTcp8_153_11;
    sens->J[1][21] = JTcp8_153_12;
    sens->J[2][1] = JTcp8_253_1;
    sens->J[2][2] = S1;
    sens->J[2][3] = C1;
    sens->J[2][5] = JTcp8_253_5;
    sens->J[2][6] = JTcp8_253_6;
    sens->J[2][7] = JTcp8_253_7;
    sens->J[2][10] = JTcp8_253_8;
    sens->J[2][18] = JTcp8_253_9;
    sens->J[2][19] = JTcp8_253_10;
    sens->J[2][20] = JTcp8_253_11;
    sens->J[2][21] = JTcp8_253_12;
    sens->J[3][4] = (1.0);
    sens->J[3][6] = JTcp8_353_6;
    sens->J[3][7] = JTcp8_353_7;
    sens->J[3][10] = JTcp8_353_8;
    sens->J[3][18] = JTcp8_353_9;
    sens->J[3][19] = JTcp8_353_10;
    sens->J[3][20] = JTcp8_353_11;
    sens->J[3][21] = JTcp8_353_12;
    sens->J[4][6] = C1p5;
    sens->J[4][7] = ROcp8_46;
    sens->J[4][10] = ROcp8_46;
    sens->J[4][18] = ROcp8_110;
    sens->J[4][19] = ROcp8_110;
    sens->J[4][20] = ROcp8_419;
    sens->J[4][21] = ROcp8_720;
    sens->J[5][6] = S1p5;
    sens->J[5][7] = ROcp8_56;
    sens->J[5][10] = ROcp8_56;
    sens->J[5][18] = ROcp8_210;
    sens->J[5][19] = ROcp8_210;
    sens->J[5][20] = ROcp8_519;
    sens->J[5][21] = ROcp8_820;
    sens->J[6][1] = (1.0);
    sens->J[6][5] = (1.0);
    sens->J[6][7] = S6;
    sens->J[6][10] = S6;
    sens->J[6][18] = ROcp8_310;
    sens->J[6][19] = ROcp8_310;
    sens->J[6][20] = ROcp8_619;
    sens->J[6][21] = ROcp8_920;
    sens->A[1] = ACcp8_153;
    sens->A[2] = ACcp8_253;
    sens->A[3] = ACcp8_353;
    sens->OMP[1] = OPcp8_121;
    sens->OMP[2] = OPcp8_221;
    sens->OMP[3] = OPcp8_321;
 
// 
break;
case 10:
 


// = = Block_1_0_0_10_0_1 = = 
 
// Sensor Kinematics 


    ROcp9_46 = -S1p5*C6;
    ROcp9_56 = C1p5*C6;
    ROcp9_76 = S1p5*S6;
    ROcp9_86 = -C1p5*S6;
    ROcp9_17 = -(ROcp9_76*S7-C1p5*C7);
    ROcp9_27 = -(ROcp9_86*S7-S1p5*C7);
    ROcp9_77 = ROcp9_76*C7+C1p5*S7;
    ROcp9_87 = ROcp9_86*C7+S1p5*S7;
    RLcp9_12 = q[2]*C1;
    RLcp9_22 = q[2]*S1;
    ORcp9_12 = -RLcp9_22*qd[1];
    ORcp9_22 = RLcp9_12*qd[1];
    RLcp9_13 = -q[3]*S1;
    RLcp9_23 = q[3]*C1;
    ORcp9_13 = -RLcp9_23*qd[1];
    ORcp9_23 = RLcp9_13*qd[1];
    OMcp9_35 = qd[1]+qd[5];
    OMcp9_16 = qd[6]*C1p5;
    OMcp9_26 = qd[6]*S1p5;
    OMcp9_17 = OMcp9_16+ROcp9_46*qd[7];
    OMcp9_27 = OMcp9_26+ROcp9_56*qd[7];
    OMcp9_37 = OMcp9_35+qd[7]*S6;
    OPcp9_17 = -(OMcp9_35*qd[6]*S1p5-ROcp9_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp9_26*S6-OMcp9_35*ROcp9_56));
    OPcp9_27 = OMcp9_35*qd[6]*C1p5+ROcp9_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp9_16*S6-OMcp9_35*ROcp9_46);
    OPcp9_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_10_0_3 = = 
 
// Sensor Kinematics 


    ROcp9_110 = ROcp9_17*C10-ROcp9_77*S10;
    ROcp9_210 = ROcp9_27*C10-ROcp9_87*S10;
    ROcp9_310 = -S10p7*C6;
    ROcp9_710 = ROcp9_17*S10+ROcp9_77*C10;
    ROcp9_810 = ROcp9_27*S10+ROcp9_87*C10;
    ROcp9_910 = C10p7*C6;
    RLcp9_110 = ROcp9_17*s->dpt[1][5]+ROcp9_77*s->dpt[3][5];
    RLcp9_210 = ROcp9_27*s->dpt[1][5]+ROcp9_87*s->dpt[3][5];
    RLcp9_310 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp9_110 = OMcp9_27*RLcp9_310-OMcp9_37*RLcp9_210;
    ORcp9_210 = -(OMcp9_17*RLcp9_310-OMcp9_37*RLcp9_110);
    ORcp9_310 = OMcp9_17*RLcp9_210-OMcp9_27*RLcp9_110;

// = = Block_1_0_0_10_0_6 = = 
 
// Sensor Kinematics 


    ROcp9_418 = ROcp9_46*C18+ROcp9_710*S18;
    ROcp9_518 = ROcp9_56*C18+ROcp9_810*S18;
    ROcp9_618 = ROcp9_910*S18+C18*S6;
    ROcp9_718 = -(ROcp9_46*S18-ROcp9_710*C18);
    ROcp9_818 = -(ROcp9_56*S18-ROcp9_810*C18);
    ROcp9_918 = ROcp9_910*C18-S18*S6;
    ROcp9_419 = ROcp9_418*C19+ROcp9_718*S19;
    ROcp9_519 = ROcp9_518*C19+ROcp9_818*S19;
    ROcp9_619 = ROcp9_618*C19+ROcp9_918*S19;
    ROcp9_719 = -(ROcp9_418*S19-ROcp9_718*C19);
    ROcp9_819 = -(ROcp9_518*S19-ROcp9_818*C19);
    ROcp9_919 = -(ROcp9_618*S19-ROcp9_918*C19);
    ROcp9_120 = ROcp9_110*C20-ROcp9_719*S20;
    ROcp9_220 = ROcp9_210*C20-ROcp9_819*S20;
    ROcp9_320 = ROcp9_310*C20-ROcp9_919*S20;
    ROcp9_720 = ROcp9_110*S20+ROcp9_719*C20;
    ROcp9_820 = ROcp9_210*S20+ROcp9_819*C20;
    ROcp9_920 = ROcp9_310*S20+ROcp9_919*C20;
    ROcp9_121 = ROcp9_120*C21+ROcp9_419*S21;
    ROcp9_221 = ROcp9_220*C21+ROcp9_519*S21;
    ROcp9_321 = ROcp9_320*C21+ROcp9_619*S21;
    ROcp9_421 = -(ROcp9_120*S21-ROcp9_419*C21);
    ROcp9_521 = -(ROcp9_220*S21-ROcp9_519*C21);
    ROcp9_621 = -(ROcp9_320*S21-ROcp9_619*C21);
    RLcp9_118 = ROcp9_46*s->dpt[2][12]+ROcp9_710*s->dpt[3][12];
    RLcp9_218 = ROcp9_56*s->dpt[2][12]+ROcp9_810*s->dpt[3][12];
    RLcp9_318 = ROcp9_910*s->dpt[3][12]+s->dpt[2][12]*S6;
    OMcp9_118 = OMcp9_17+ROcp9_110*qd[18];
    OMcp9_218 = OMcp9_27+ROcp9_210*qd[18];
    OMcp9_318 = OMcp9_37+ROcp9_310*qd[18];
    ORcp9_118 = OMcp9_27*RLcp9_318-OMcp9_37*RLcp9_218;
    ORcp9_218 = -(OMcp9_17*RLcp9_318-OMcp9_37*RLcp9_118);
    ORcp9_318 = OMcp9_17*RLcp9_218-OMcp9_27*RLcp9_118;
    OPcp9_118 = OPcp9_17+ROcp9_110*qdd[18]+qd[18]*(OMcp9_27*ROcp9_310-OMcp9_37*ROcp9_210);
    OPcp9_218 = OPcp9_27+ROcp9_210*qdd[18]-qd[18]*(OMcp9_17*ROcp9_310-OMcp9_37*ROcp9_110);
    OPcp9_318 = OPcp9_37+ROcp9_310*qdd[18]+qd[18]*(OMcp9_17*ROcp9_210-OMcp9_27*ROcp9_110);
    RLcp9_119 = ROcp9_418*s->dpt[2][23];
    RLcp9_219 = ROcp9_518*s->dpt[2][23];
    RLcp9_319 = ROcp9_618*s->dpt[2][23];
    OMcp9_119 = OMcp9_118+ROcp9_110*qd[19];
    OMcp9_219 = OMcp9_218+ROcp9_210*qd[19];
    OMcp9_319 = OMcp9_318+ROcp9_310*qd[19];
    ORcp9_119 = OMcp9_218*RLcp9_319-OMcp9_318*RLcp9_219;
    ORcp9_219 = -(OMcp9_118*RLcp9_319-OMcp9_318*RLcp9_119);
    ORcp9_319 = OMcp9_118*RLcp9_219-OMcp9_218*RLcp9_119;
    OMcp9_120 = OMcp9_119+ROcp9_419*qd[20];
    OMcp9_220 = OMcp9_219+ROcp9_519*qd[20];
    OMcp9_320 = OMcp9_319+ROcp9_619*qd[20];
    OMcp9_121 = OMcp9_120+ROcp9_720*qd[21];
    OMcp9_221 = OMcp9_220+ROcp9_820*qd[21];
    OMcp9_321 = OMcp9_320+ROcp9_920*qd[21];
    OPcp9_121 = OPcp9_118+ROcp9_110*qdd[19]+ROcp9_419*qdd[20]+ROcp9_720*qdd[21]+qd[19]*(OMcp9_218*ROcp9_310-OMcp9_318*
 ROcp9_210)+qd[20]*(OMcp9_219*ROcp9_619-OMcp9_319*ROcp9_519)+qd[21]*(OMcp9_220*ROcp9_920-OMcp9_320*ROcp9_820);
    OPcp9_221 = OPcp9_218+ROcp9_210*qdd[19]+ROcp9_519*qdd[20]+ROcp9_820*qdd[21]-qd[19]*(OMcp9_118*ROcp9_310-OMcp9_318*
 ROcp9_110)-qd[20]*(OMcp9_119*ROcp9_619-OMcp9_319*ROcp9_419)-qd[21]*(OMcp9_120*ROcp9_920-OMcp9_320*ROcp9_720);
    OPcp9_321 = OPcp9_318+ROcp9_310*qdd[19]+ROcp9_619*qdd[20]+ROcp9_920*qdd[21]+qd[19]*(OMcp9_118*ROcp9_210-OMcp9_218*
 ROcp9_110)+qd[20]*(OMcp9_119*ROcp9_519-OMcp9_219*ROcp9_419)+qd[21]*(OMcp9_120*ROcp9_820-OMcp9_220*ROcp9_720);
    RLcp9_154 = ROcp9_121*s->dpt[1][27]+ROcp9_421*s->dpt[2][27]+ROcp9_720*s->dpt[3][27];
    RLcp9_254 = ROcp9_221*s->dpt[1][27]+ROcp9_521*s->dpt[2][27]+ROcp9_820*s->dpt[3][27];
    RLcp9_354 = ROcp9_321*s->dpt[1][27]+ROcp9_621*s->dpt[2][27]+ROcp9_920*s->dpt[3][27];
    POcp9_154 = RLcp9_110+RLcp9_118+RLcp9_119+RLcp9_12+RLcp9_13+RLcp9_154;
    POcp9_254 = RLcp9_210+RLcp9_218+RLcp9_219+RLcp9_22+RLcp9_23+RLcp9_254;
    POcp9_354 = RLcp9_310+RLcp9_318+RLcp9_319+RLcp9_354+q[4];
    JTcp9_154_1 = -(RLcp9_210+RLcp9_218+RLcp9_219+RLcp9_22+RLcp9_23+RLcp9_254);
    JTcp9_254_1 = RLcp9_110+RLcp9_118+RLcp9_119+RLcp9_12+RLcp9_13+RLcp9_154;
    JTcp9_154_5 = -(RLcp9_210+RLcp9_218+RLcp9_219+RLcp9_254);
    JTcp9_254_5 = RLcp9_110+RLcp9_118+RLcp9_119+RLcp9_154;
    JTcp9_154_6 = S1p5*(RLcp9_310+RLcp9_318+RLcp9_319+RLcp9_354);
    JTcp9_254_6 = -C1p5*(RLcp9_310+RLcp9_318+RLcp9_319+RLcp9_354);
    JTcp9_354_6 = C1p5*(RLcp9_210+RLcp9_218+RLcp9_219+RLcp9_254)-S1p5*(RLcp9_110+RLcp9_118)-S1p5*(RLcp9_119+RLcp9_154);
    JTcp9_154_7 = ROcp9_56*(RLcp9_310+RLcp9_318+RLcp9_319+RLcp9_354)-S6*(RLcp9_210+RLcp9_218)-S6*(RLcp9_219+RLcp9_254);
    JTcp9_254_7 = -(ROcp9_46*(RLcp9_310+RLcp9_318+RLcp9_319+RLcp9_354)-S6*(RLcp9_110+RLcp9_118)-S6*(RLcp9_119+RLcp9_154));
    JTcp9_354_7 = ROcp9_46*(RLcp9_210+RLcp9_218+RLcp9_219+RLcp9_254)-ROcp9_56*(RLcp9_110+RLcp9_118)-ROcp9_56*(RLcp9_119+
 RLcp9_154);
    JTcp9_154_8 = ROcp9_56*(RLcp9_318+RLcp9_319)-S6*(RLcp9_218+RLcp9_219)-RLcp9_254*S6+RLcp9_354*ROcp9_56;
    JTcp9_254_8 = RLcp9_154*S6-RLcp9_354*ROcp9_46-ROcp9_46*(RLcp9_318+RLcp9_319)+S6*(RLcp9_118+RLcp9_119);
    JTcp9_354_8 = ROcp9_46*(RLcp9_218+RLcp9_219)-ROcp9_56*(RLcp9_118+RLcp9_119)-RLcp9_154*ROcp9_56+RLcp9_254*ROcp9_46;
    JTcp9_154_9 = ROcp9_210*(RLcp9_319+RLcp9_354)-ROcp9_310*(RLcp9_219+RLcp9_254);
    JTcp9_254_9 = -(ROcp9_110*(RLcp9_319+RLcp9_354)-ROcp9_310*(RLcp9_119+RLcp9_154));
    JTcp9_354_9 = ROcp9_110*(RLcp9_219+RLcp9_254)-ROcp9_210*(RLcp9_119+RLcp9_154);
    JTcp9_154_10 = -(RLcp9_254*ROcp9_310-RLcp9_354*ROcp9_210);
    JTcp9_254_10 = RLcp9_154*ROcp9_310-RLcp9_354*ROcp9_110;
    JTcp9_354_10 = -(RLcp9_154*ROcp9_210-RLcp9_254*ROcp9_110);
    JTcp9_154_11 = -(RLcp9_254*ROcp9_619-RLcp9_354*ROcp9_519);
    JTcp9_254_11 = RLcp9_154*ROcp9_619-RLcp9_354*ROcp9_419;
    JTcp9_354_11 = -(RLcp9_154*ROcp9_519-RLcp9_254*ROcp9_419);
    JTcp9_154_12 = -(RLcp9_254*ROcp9_920-RLcp9_354*ROcp9_820);
    JTcp9_254_12 = RLcp9_154*ROcp9_920-RLcp9_354*ROcp9_720;
    JTcp9_354_12 = -(RLcp9_154*ROcp9_820-RLcp9_254*ROcp9_720);
    ORcp9_154 = OMcp9_221*RLcp9_354-OMcp9_321*RLcp9_254;
    ORcp9_254 = -(OMcp9_121*RLcp9_354-OMcp9_321*RLcp9_154);
    ORcp9_354 = OMcp9_121*RLcp9_254-OMcp9_221*RLcp9_154;
    VIcp9_154 = ORcp9_110+ORcp9_118+ORcp9_119+ORcp9_12+ORcp9_13+ORcp9_154+qd[2]*C1-qd[3]*S1;
    VIcp9_254 = ORcp9_210+ORcp9_218+ORcp9_219+ORcp9_22+ORcp9_23+ORcp9_254+qd[2]*S1+qd[3]*C1;
    VIcp9_354 = ORcp9_310+ORcp9_318+ORcp9_319+ORcp9_354+qd[4];
    ACcp9_154 = OMcp9_218*ORcp9_319+OMcp9_221*ORcp9_354+OMcp9_27*(ORcp9_310+ORcp9_318)-OMcp9_318*ORcp9_219-OMcp9_321*
 ORcp9_254-OMcp9_37*ORcp9_210-OMcp9_37*ORcp9_218+OPcp9_218*RLcp9_319+OPcp9_221*RLcp9_354+OPcp9_27*RLcp9_310+OPcp9_27*
 RLcp9_318-OPcp9_318*RLcp9_219-OPcp9_321*RLcp9_254-OPcp9_37*RLcp9_210-OPcp9_37*RLcp9_218-ORcp9_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*
 qd[1]*qd[3]*C1-qd[1]*(ORcp9_22+(2.0)*qd[2]*S1);
    ACcp9_254 = -(OMcp9_118*ORcp9_319+OMcp9_121*ORcp9_354+OMcp9_17*(ORcp9_310+ORcp9_318)-OMcp9_318*ORcp9_119-OMcp9_321*
 ORcp9_154-OMcp9_37*ORcp9_110-OMcp9_37*ORcp9_118+OPcp9_118*RLcp9_319+OPcp9_121*RLcp9_354+OPcp9_17*RLcp9_310+OPcp9_17*
 RLcp9_318-OPcp9_318*RLcp9_119-OPcp9_321*RLcp9_154-OPcp9_37*RLcp9_110-OPcp9_37*RLcp9_118-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp9_12+(2.0)
 *qd[2]*C1)-qd[1]*(ORcp9_13-(2.0)*qd[3]*S1));
    ACcp9_354 = qdd[4]+OMcp9_118*ORcp9_219+OMcp9_121*ORcp9_254+OMcp9_17*ORcp9_210+OMcp9_17*ORcp9_218-OMcp9_218*ORcp9_119-
 OMcp9_221*ORcp9_154-OMcp9_27*ORcp9_110-OMcp9_27*ORcp9_118+OPcp9_118*RLcp9_219+OPcp9_121*RLcp9_254+OPcp9_17*RLcp9_210+
 OPcp9_17*RLcp9_218-OPcp9_218*RLcp9_119-OPcp9_221*RLcp9_154-OPcp9_27*RLcp9_110-OPcp9_27*RLcp9_118;

// = = Block_1_0_0_10_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp9_154;
    sens->P[2] = POcp9_254;
    sens->P[3] = POcp9_354;
    sens->R[1][1] = ROcp9_121;
    sens->R[1][2] = ROcp9_221;
    sens->R[1][3] = ROcp9_321;
    sens->R[2][1] = ROcp9_421;
    sens->R[2][2] = ROcp9_521;
    sens->R[2][3] = ROcp9_621;
    sens->R[3][1] = ROcp9_720;
    sens->R[3][2] = ROcp9_820;
    sens->R[3][3] = ROcp9_920;
    sens->V[1] = VIcp9_154;
    sens->V[2] = VIcp9_254;
    sens->V[3] = VIcp9_354;
    sens->OM[1] = OMcp9_121;
    sens->OM[2] = OMcp9_221;
    sens->OM[3] = OMcp9_321;
    sens->J[1][1] = JTcp9_154_1;
    sens->J[1][2] = C1;
    sens->J[1][3] = -S1;
    sens->J[1][5] = JTcp9_154_5;
    sens->J[1][6] = JTcp9_154_6;
    sens->J[1][7] = JTcp9_154_7;
    sens->J[1][10] = JTcp9_154_8;
    sens->J[1][18] = JTcp9_154_9;
    sens->J[1][19] = JTcp9_154_10;
    sens->J[1][20] = JTcp9_154_11;
    sens->J[1][21] = JTcp9_154_12;
    sens->J[2][1] = JTcp9_254_1;
    sens->J[2][2] = S1;
    sens->J[2][3] = C1;
    sens->J[2][5] = JTcp9_254_5;
    sens->J[2][6] = JTcp9_254_6;
    sens->J[2][7] = JTcp9_254_7;
    sens->J[2][10] = JTcp9_254_8;
    sens->J[2][18] = JTcp9_254_9;
    sens->J[2][19] = JTcp9_254_10;
    sens->J[2][20] = JTcp9_254_11;
    sens->J[2][21] = JTcp9_254_12;
    sens->J[3][4] = (1.0);
    sens->J[3][6] = JTcp9_354_6;
    sens->J[3][7] = JTcp9_354_7;
    sens->J[3][10] = JTcp9_354_8;
    sens->J[3][18] = JTcp9_354_9;
    sens->J[3][19] = JTcp9_354_10;
    sens->J[3][20] = JTcp9_354_11;
    sens->J[3][21] = JTcp9_354_12;
    sens->J[4][6] = C1p5;
    sens->J[4][7] = ROcp9_46;
    sens->J[4][10] = ROcp9_46;
    sens->J[4][18] = ROcp9_110;
    sens->J[4][19] = ROcp9_110;
    sens->J[4][20] = ROcp9_419;
    sens->J[4][21] = ROcp9_720;
    sens->J[5][6] = S1p5;
    sens->J[5][7] = ROcp9_56;
    sens->J[5][10] = ROcp9_56;
    sens->J[5][18] = ROcp9_210;
    sens->J[5][19] = ROcp9_210;
    sens->J[5][20] = ROcp9_519;
    sens->J[5][21] = ROcp9_820;
    sens->J[6][1] = (1.0);
    sens->J[6][5] = (1.0);
    sens->J[6][7] = S6;
    sens->J[6][10] = S6;
    sens->J[6][18] = ROcp9_310;
    sens->J[6][19] = ROcp9_310;
    sens->J[6][20] = ROcp9_619;
    sens->J[6][21] = ROcp9_920;
    sens->A[1] = ACcp9_154;
    sens->A[2] = ACcp9_254;
    sens->A[3] = ACcp9_354;
    sens->OMP[1] = OPcp9_121;
    sens->OMP[2] = OPcp9_221;
    sens->OMP[3] = OPcp9_321;
 
// 
break;
case 11:
 


// = = Block_1_0_0_11_0_1 = = 
 
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
    ORcp10_12 = -RLcp10_22*qd[1];
    ORcp10_22 = RLcp10_12*qd[1];
    RLcp10_13 = -q[3]*S1;
    RLcp10_23 = q[3]*C1;
    ORcp10_13 = -RLcp10_23*qd[1];
    ORcp10_23 = RLcp10_13*qd[1];
    OMcp10_35 = qd[1]+qd[5];
    OMcp10_16 = qd[6]*C1p5;
    OMcp10_26 = qd[6]*S1p5;
    OMcp10_17 = OMcp10_16+ROcp10_46*qd[7];
    OMcp10_27 = OMcp10_26+ROcp10_56*qd[7];
    OMcp10_37 = OMcp10_35+qd[7]*S6;
    OPcp10_17 = -(OMcp10_35*qd[6]*S1p5-ROcp10_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp10_26*S6-OMcp10_35*ROcp10_56));
    OPcp10_27 = OMcp10_35*qd[6]*C1p5+ROcp10_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp10_16*S6-OMcp10_35*ROcp10_46);
    OPcp10_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_11_0_3 = = 
 
// Sensor Kinematics 


    ROcp10_110 = ROcp10_17*C10-ROcp10_77*S10;
    ROcp10_210 = ROcp10_27*C10-ROcp10_87*S10;
    ROcp10_310 = -S10p7*C6;
    ROcp10_710 = ROcp10_17*S10+ROcp10_77*C10;
    ROcp10_810 = ROcp10_27*S10+ROcp10_87*C10;
    ROcp10_910 = C10p7*C6;
    RLcp10_110 = ROcp10_17*s->dpt[1][5]+ROcp10_77*s->dpt[3][5];
    RLcp10_210 = ROcp10_27*s->dpt[1][5]+ROcp10_87*s->dpt[3][5];
    RLcp10_310 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp10_110 = OMcp10_27*RLcp10_310-OMcp10_37*RLcp10_210;
    ORcp10_210 = -(OMcp10_17*RLcp10_310-OMcp10_37*RLcp10_110);
    ORcp10_310 = OMcp10_17*RLcp10_210-OMcp10_27*RLcp10_110;

// = = Block_1_0_0_11_0_6 = = 
 
// Sensor Kinematics 


    ROcp10_418 = ROcp10_46*C18+ROcp10_710*S18;
    ROcp10_518 = ROcp10_56*C18+ROcp10_810*S18;
    ROcp10_618 = ROcp10_910*S18+C18*S6;
    ROcp10_718 = -(ROcp10_46*S18-ROcp10_710*C18);
    ROcp10_818 = -(ROcp10_56*S18-ROcp10_810*C18);
    ROcp10_918 = ROcp10_910*C18-S18*S6;
    ROcp10_419 = ROcp10_418*C19+ROcp10_718*S19;
    ROcp10_519 = ROcp10_518*C19+ROcp10_818*S19;
    ROcp10_619 = ROcp10_618*C19+ROcp10_918*S19;
    ROcp10_719 = -(ROcp10_418*S19-ROcp10_718*C19);
    ROcp10_819 = -(ROcp10_518*S19-ROcp10_818*C19);
    ROcp10_919 = -(ROcp10_618*S19-ROcp10_918*C19);
    ROcp10_120 = ROcp10_110*C20-ROcp10_719*S20;
    ROcp10_220 = ROcp10_210*C20-ROcp10_819*S20;
    ROcp10_320 = ROcp10_310*C20-ROcp10_919*S20;
    ROcp10_720 = ROcp10_110*S20+ROcp10_719*C20;
    ROcp10_820 = ROcp10_210*S20+ROcp10_819*C20;
    ROcp10_920 = ROcp10_310*S20+ROcp10_919*C20;
    ROcp10_121 = ROcp10_120*C21+ROcp10_419*S21;
    ROcp10_221 = ROcp10_220*C21+ROcp10_519*S21;
    ROcp10_321 = ROcp10_320*C21+ROcp10_619*S21;
    ROcp10_421 = -(ROcp10_120*S21-ROcp10_419*C21);
    ROcp10_521 = -(ROcp10_220*S21-ROcp10_519*C21);
    ROcp10_621 = -(ROcp10_320*S21-ROcp10_619*C21);
    ROcp10_422 = ROcp10_421*C22+ROcp10_720*S22;
    ROcp10_522 = ROcp10_521*C22+ROcp10_820*S22;
    ROcp10_622 = ROcp10_621*C22+ROcp10_920*S22;
    ROcp10_722 = -(ROcp10_421*S22-ROcp10_720*C22);
    ROcp10_822 = -(ROcp10_521*S22-ROcp10_820*C22);
    ROcp10_922 = -(ROcp10_621*S22-ROcp10_920*C22);
    ROcp10_123 = ROcp10_121*C23-ROcp10_722*S23;
    ROcp10_223 = ROcp10_221*C23-ROcp10_822*S23;
    ROcp10_323 = ROcp10_321*C23-ROcp10_922*S23;
    ROcp10_723 = ROcp10_121*S23+ROcp10_722*C23;
    ROcp10_823 = ROcp10_221*S23+ROcp10_822*C23;
    ROcp10_923 = ROcp10_321*S23+ROcp10_922*C23;
    RLcp10_118 = ROcp10_46*s->dpt[2][12]+ROcp10_710*s->dpt[3][12];
    RLcp10_218 = ROcp10_56*s->dpt[2][12]+ROcp10_810*s->dpt[3][12];
    RLcp10_318 = ROcp10_910*s->dpt[3][12]+s->dpt[2][12]*S6;
    OMcp10_118 = OMcp10_17+ROcp10_110*qd[18];
    OMcp10_218 = OMcp10_27+ROcp10_210*qd[18];
    OMcp10_318 = OMcp10_37+ROcp10_310*qd[18];
    ORcp10_118 = OMcp10_27*RLcp10_318-OMcp10_37*RLcp10_218;
    ORcp10_218 = -(OMcp10_17*RLcp10_318-OMcp10_37*RLcp10_118);
    ORcp10_318 = OMcp10_17*RLcp10_218-OMcp10_27*RLcp10_118;
    OPcp10_118 = OPcp10_17+ROcp10_110*qdd[18]+qd[18]*(OMcp10_27*ROcp10_310-OMcp10_37*ROcp10_210);
    OPcp10_218 = OPcp10_27+ROcp10_210*qdd[18]-qd[18]*(OMcp10_17*ROcp10_310-OMcp10_37*ROcp10_110);
    OPcp10_318 = OPcp10_37+ROcp10_310*qdd[18]+qd[18]*(OMcp10_17*ROcp10_210-OMcp10_27*ROcp10_110);
    RLcp10_119 = ROcp10_418*s->dpt[2][23];
    RLcp10_219 = ROcp10_518*s->dpt[2][23];
    RLcp10_319 = ROcp10_618*s->dpt[2][23];
    OMcp10_119 = OMcp10_118+ROcp10_110*qd[19];
    OMcp10_219 = OMcp10_218+ROcp10_210*qd[19];
    OMcp10_319 = OMcp10_318+ROcp10_310*qd[19];
    ORcp10_119 = OMcp10_218*RLcp10_319-OMcp10_318*RLcp10_219;
    ORcp10_219 = -(OMcp10_118*RLcp10_319-OMcp10_318*RLcp10_119);
    ORcp10_319 = OMcp10_118*RLcp10_219-OMcp10_218*RLcp10_119;
    OMcp10_120 = OMcp10_119+ROcp10_419*qd[20];
    OMcp10_220 = OMcp10_219+ROcp10_519*qd[20];
    OMcp10_320 = OMcp10_319+ROcp10_619*qd[20];
    OMcp10_121 = OMcp10_120+ROcp10_720*qd[21];
    OMcp10_221 = OMcp10_220+ROcp10_820*qd[21];
    OMcp10_321 = OMcp10_320+ROcp10_920*qd[21];
    OPcp10_121 = OPcp10_118+ROcp10_110*qdd[19]+ROcp10_419*qdd[20]+ROcp10_720*qdd[21]+qd[19]*(OMcp10_218*ROcp10_310-
 OMcp10_318*ROcp10_210)+qd[20]*(OMcp10_219*ROcp10_619-OMcp10_319*ROcp10_519)+qd[21]*(OMcp10_220*ROcp10_920-OMcp10_320*
 ROcp10_820);
    OPcp10_221 = OPcp10_218+ROcp10_210*qdd[19]+ROcp10_519*qdd[20]+ROcp10_820*qdd[21]-qd[19]*(OMcp10_118*ROcp10_310-
 OMcp10_318*ROcp10_110)-qd[20]*(OMcp10_119*ROcp10_619-OMcp10_319*ROcp10_419)-qd[21]*(OMcp10_120*ROcp10_920-OMcp10_320*
 ROcp10_720);
    OPcp10_321 = OPcp10_318+ROcp10_310*qdd[19]+ROcp10_619*qdd[20]+ROcp10_920*qdd[21]+qd[19]*(OMcp10_118*ROcp10_210-
 OMcp10_218*ROcp10_110)+qd[20]*(OMcp10_119*ROcp10_519-OMcp10_219*ROcp10_419)+qd[21]*(OMcp10_120*ROcp10_820-OMcp10_220*
 ROcp10_720);
    RLcp10_122 = ROcp10_121*s->dpt[1][27]+ROcp10_421*s->dpt[2][27]+ROcp10_720*s->dpt[3][27];
    RLcp10_222 = ROcp10_221*s->dpt[1][27]+ROcp10_521*s->dpt[2][27]+ROcp10_820*s->dpt[3][27];
    RLcp10_322 = ROcp10_321*s->dpt[1][27]+ROcp10_621*s->dpt[2][27]+ROcp10_920*s->dpt[3][27];
    POcp10_122 = RLcp10_110+RLcp10_118+RLcp10_119+RLcp10_12+RLcp10_122+RLcp10_13;
    POcp10_222 = RLcp10_210+RLcp10_218+RLcp10_219+RLcp10_22+RLcp10_222+RLcp10_23;
    POcp10_322 = RLcp10_310+RLcp10_318+RLcp10_319+RLcp10_322+q[4];
    JTcp10_122_1 = -(RLcp10_210+RLcp10_218+RLcp10_219+RLcp10_22+RLcp10_222+RLcp10_23);
    JTcp10_222_1 = RLcp10_110+RLcp10_118+RLcp10_119+RLcp10_12+RLcp10_122+RLcp10_13;
    JTcp10_122_5 = -(RLcp10_210+RLcp10_218+RLcp10_219+RLcp10_222);
    JTcp10_222_5 = RLcp10_110+RLcp10_118+RLcp10_119+RLcp10_122;
    JTcp10_122_6 = S1p5*(RLcp10_310+RLcp10_318+RLcp10_319+RLcp10_322);
    JTcp10_222_6 = -C1p5*(RLcp10_310+RLcp10_318+RLcp10_319+RLcp10_322);
    JTcp10_322_6 = C1p5*(RLcp10_210+RLcp10_218+RLcp10_219+RLcp10_222)-S1p5*(RLcp10_110+RLcp10_118)-S1p5*(RLcp10_119+
 RLcp10_122);
    JTcp10_122_7 = ROcp10_56*(RLcp10_310+RLcp10_318+RLcp10_319+RLcp10_322)-S6*(RLcp10_210+RLcp10_218)-S6*(RLcp10_219+
 RLcp10_222);
    JTcp10_222_7 = -(ROcp10_46*(RLcp10_310+RLcp10_318+RLcp10_319+RLcp10_322)-S6*(RLcp10_110+RLcp10_118)-S6*(RLcp10_119+
 RLcp10_122));
    JTcp10_322_7 = ROcp10_46*(RLcp10_210+RLcp10_218+RLcp10_219+RLcp10_222)-ROcp10_56*(RLcp10_110+RLcp10_118)-ROcp10_56*(
 RLcp10_119+RLcp10_122);
    JTcp10_122_8 = ROcp10_56*(RLcp10_318+RLcp10_319)-S6*(RLcp10_218+RLcp10_219)-RLcp10_222*S6+RLcp10_322*ROcp10_56;
    JTcp10_222_8 = RLcp10_122*S6-RLcp10_322*ROcp10_46-ROcp10_46*(RLcp10_318+RLcp10_319)+S6*(RLcp10_118+RLcp10_119);
    JTcp10_322_8 = ROcp10_46*(RLcp10_218+RLcp10_219)-ROcp10_56*(RLcp10_118+RLcp10_119)-RLcp10_122*ROcp10_56+RLcp10_222*
 ROcp10_46;
    JTcp10_122_9 = ROcp10_210*(RLcp10_319+RLcp10_322)-ROcp10_310*(RLcp10_219+RLcp10_222);
    JTcp10_222_9 = -(ROcp10_110*(RLcp10_319+RLcp10_322)-ROcp10_310*(RLcp10_119+RLcp10_122));
    JTcp10_322_9 = ROcp10_110*(RLcp10_219+RLcp10_222)-ROcp10_210*(RLcp10_119+RLcp10_122);
    JTcp10_122_10 = -(RLcp10_222*ROcp10_310-RLcp10_322*ROcp10_210);
    JTcp10_222_10 = RLcp10_122*ROcp10_310-RLcp10_322*ROcp10_110;
    JTcp10_322_10 = -(RLcp10_122*ROcp10_210-RLcp10_222*ROcp10_110);
    JTcp10_122_11 = -(RLcp10_222*ROcp10_619-RLcp10_322*ROcp10_519);
    JTcp10_222_11 = RLcp10_122*ROcp10_619-RLcp10_322*ROcp10_419;
    JTcp10_322_11 = -(RLcp10_122*ROcp10_519-RLcp10_222*ROcp10_419);
    JTcp10_122_12 = -(RLcp10_222*ROcp10_920-RLcp10_322*ROcp10_820);
    JTcp10_222_12 = RLcp10_122*ROcp10_920-RLcp10_322*ROcp10_720;
    JTcp10_322_12 = -(RLcp10_122*ROcp10_820-RLcp10_222*ROcp10_720);
    ORcp10_122 = OMcp10_221*RLcp10_322-OMcp10_321*RLcp10_222;
    ORcp10_222 = -(OMcp10_121*RLcp10_322-OMcp10_321*RLcp10_122);
    ORcp10_322 = OMcp10_121*RLcp10_222-OMcp10_221*RLcp10_122;
    VIcp10_122 = ORcp10_110+ORcp10_118+ORcp10_119+ORcp10_12+ORcp10_122+ORcp10_13+qd[2]*C1-qd[3]*S1;
    VIcp10_222 = ORcp10_210+ORcp10_218+ORcp10_219+ORcp10_22+ORcp10_222+ORcp10_23+qd[2]*S1+qd[3]*C1;
    VIcp10_322 = ORcp10_310+ORcp10_318+ORcp10_319+ORcp10_322+qd[4];
    ACcp10_122 = OMcp10_218*ORcp10_319+OMcp10_221*ORcp10_322+OMcp10_27*(ORcp10_310+ORcp10_318)-OMcp10_318*ORcp10_219-
 OMcp10_321*ORcp10_222-OMcp10_37*ORcp10_210-OMcp10_37*ORcp10_218+OPcp10_218*RLcp10_319+OPcp10_221*RLcp10_322+OPcp10_27*
 RLcp10_310+OPcp10_27*RLcp10_318-OPcp10_318*RLcp10_219-OPcp10_321*RLcp10_222-OPcp10_37*RLcp10_210-OPcp10_37*RLcp10_218-
 ORcp10_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp10_22+(2.0)*qd[2]*S1);
    ACcp10_222 = -(OMcp10_118*ORcp10_319+OMcp10_121*ORcp10_322+OMcp10_17*(ORcp10_310+ORcp10_318)-OMcp10_318*ORcp10_119-
 OMcp10_321*ORcp10_122-OMcp10_37*ORcp10_110-OMcp10_37*ORcp10_118+OPcp10_118*RLcp10_319+OPcp10_121*RLcp10_322+OPcp10_17*
 RLcp10_310+OPcp10_17*RLcp10_318-OPcp10_318*RLcp10_119-OPcp10_321*RLcp10_122-OPcp10_37*RLcp10_110-OPcp10_37*RLcp10_118-qdd[2]
 *S1-qdd[3]*C1-qd[1]*(ORcp10_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp10_13-(2.0)*qd[3]*S1));
    ACcp10_322 = qdd[4]+OMcp10_118*ORcp10_219+OMcp10_121*ORcp10_222+OMcp10_17*ORcp10_210+OMcp10_17*ORcp10_218-OMcp10_218*
 ORcp10_119-OMcp10_221*ORcp10_122-OMcp10_27*ORcp10_110-OMcp10_27*ORcp10_118+OPcp10_118*RLcp10_219+OPcp10_121*RLcp10_222+
 OPcp10_17*RLcp10_210+OPcp10_17*RLcp10_218-OPcp10_218*RLcp10_119-OPcp10_221*RLcp10_122-OPcp10_27*RLcp10_110-OPcp10_27*
 RLcp10_118;
    OMcp10_123 = OMcp10_121+ROcp10_422*qd[23];
    OMcp10_223 = OMcp10_221+ROcp10_522*qd[23];
    OMcp10_323 = OMcp10_321+ROcp10_622*qd[23];
    OPcp10_123 = OPcp10_121+ROcp10_422*qdd[23]+qd[23]*(OMcp10_221*ROcp10_622-OMcp10_321*ROcp10_522);
    OPcp10_223 = OPcp10_221+ROcp10_522*qdd[23]-qd[23]*(OMcp10_121*ROcp10_622-OMcp10_321*ROcp10_422);
    OPcp10_323 = OPcp10_321+ROcp10_622*qdd[23]+qd[23]*(OMcp10_121*ROcp10_522-OMcp10_221*ROcp10_422);

// = = Block_1_0_0_11_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp10_122;
    sens->P[2] = POcp10_222;
    sens->P[3] = POcp10_322;
    sens->R[1][1] = ROcp10_123;
    sens->R[1][2] = ROcp10_223;
    sens->R[1][3] = ROcp10_323;
    sens->R[2][1] = ROcp10_422;
    sens->R[2][2] = ROcp10_522;
    sens->R[2][3] = ROcp10_622;
    sens->R[3][1] = ROcp10_723;
    sens->R[3][2] = ROcp10_823;
    sens->R[3][3] = ROcp10_923;
    sens->V[1] = VIcp10_122;
    sens->V[2] = VIcp10_222;
    sens->V[3] = VIcp10_322;
    sens->OM[1] = OMcp10_123;
    sens->OM[2] = OMcp10_223;
    sens->OM[3] = OMcp10_323;
    sens->J[1][1] = JTcp10_122_1;
    sens->J[1][2] = C1;
    sens->J[1][3] = -S1;
    sens->J[1][5] = JTcp10_122_5;
    sens->J[1][6] = JTcp10_122_6;
    sens->J[1][7] = JTcp10_122_7;
    sens->J[1][10] = JTcp10_122_8;
    sens->J[1][18] = JTcp10_122_9;
    sens->J[1][19] = JTcp10_122_10;
    sens->J[1][20] = JTcp10_122_11;
    sens->J[1][21] = JTcp10_122_12;
    sens->J[2][1] = JTcp10_222_1;
    sens->J[2][2] = S1;
    sens->J[2][3] = C1;
    sens->J[2][5] = JTcp10_222_5;
    sens->J[2][6] = JTcp10_222_6;
    sens->J[2][7] = JTcp10_222_7;
    sens->J[2][10] = JTcp10_222_8;
    sens->J[2][18] = JTcp10_222_9;
    sens->J[2][19] = JTcp10_222_10;
    sens->J[2][20] = JTcp10_222_11;
    sens->J[2][21] = JTcp10_222_12;
    sens->J[3][4] = (1.0);
    sens->J[3][6] = JTcp10_322_6;
    sens->J[3][7] = JTcp10_322_7;
    sens->J[3][10] = JTcp10_322_8;
    sens->J[3][18] = JTcp10_322_9;
    sens->J[3][19] = JTcp10_322_10;
    sens->J[3][20] = JTcp10_322_11;
    sens->J[3][21] = JTcp10_322_12;
    sens->J[4][6] = C1p5;
    sens->J[4][7] = ROcp10_46;
    sens->J[4][10] = ROcp10_46;
    sens->J[4][18] = ROcp10_110;
    sens->J[4][19] = ROcp10_110;
    sens->J[4][20] = ROcp10_419;
    sens->J[4][21] = ROcp10_720;
    sens->J[4][22] = ROcp10_121;
    sens->J[4][23] = ROcp10_422;
    sens->J[5][6] = S1p5;
    sens->J[5][7] = ROcp10_56;
    sens->J[5][10] = ROcp10_56;
    sens->J[5][18] = ROcp10_210;
    sens->J[5][19] = ROcp10_210;
    sens->J[5][20] = ROcp10_519;
    sens->J[5][21] = ROcp10_820;
    sens->J[5][22] = ROcp10_221;
    sens->J[5][23] = ROcp10_522;
    sens->J[6][1] = (1.0);
    sens->J[6][5] = (1.0);
    sens->J[6][7] = S6;
    sens->J[6][10] = S6;
    sens->J[6][18] = ROcp10_310;
    sens->J[6][19] = ROcp10_310;
    sens->J[6][20] = ROcp10_619;
    sens->J[6][21] = ROcp10_920;
    sens->J[6][22] = ROcp10_321;
    sens->J[6][23] = ROcp10_622;
    sens->A[1] = ACcp10_122;
    sens->A[2] = ACcp10_222;
    sens->A[3] = ACcp10_322;
    sens->OMP[1] = OPcp10_123;
    sens->OMP[2] = OPcp10_223;
    sens->OMP[3] = OPcp10_323;
 
// 
break;
case 12:
 


// = = Block_1_0_0_12_0_1 = = 
 
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
    ORcp11_12 = -RLcp11_22*qd[1];
    ORcp11_22 = RLcp11_12*qd[1];
    RLcp11_13 = -q[3]*S1;
    RLcp11_23 = q[3]*C1;
    ORcp11_13 = -RLcp11_23*qd[1];
    ORcp11_23 = RLcp11_13*qd[1];
    OMcp11_35 = qd[1]+qd[5];
    OMcp11_16 = qd[6]*C1p5;
    OMcp11_26 = qd[6]*S1p5;
    OMcp11_17 = OMcp11_16+ROcp11_46*qd[7];
    OMcp11_27 = OMcp11_26+ROcp11_56*qd[7];
    OMcp11_37 = OMcp11_35+qd[7]*S6;
    OPcp11_17 = -(OMcp11_35*qd[6]*S1p5-ROcp11_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp11_26*S6-OMcp11_35*ROcp11_56));
    OPcp11_27 = OMcp11_35*qd[6]*C1p5+ROcp11_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp11_16*S6-OMcp11_35*ROcp11_46);
    OPcp11_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_12_0_2 = = 
 
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
    OMcp11_18 = OMcp11_17+ROcp11_46*qd[8];
    OMcp11_28 = OMcp11_27+ROcp11_56*qd[8];
    OMcp11_38 = OMcp11_37+qd[8]*S6;
    ORcp11_18 = OMcp11_27*RLcp11_38-OMcp11_37*RLcp11_28;
    ORcp11_28 = -(OMcp11_17*RLcp11_38-OMcp11_37*RLcp11_18);
    ORcp11_38 = OMcp11_17*RLcp11_28-OMcp11_27*RLcp11_18;
    OPcp11_18 = OPcp11_17+ROcp11_46*qdd[8]+qd[8]*(OMcp11_27*S6-OMcp11_37*ROcp11_56);
    OPcp11_28 = OPcp11_27+ROcp11_56*qdd[8]-qd[8]*(OMcp11_17*S6-OMcp11_37*ROcp11_46);
    OPcp11_38 = OPcp11_37+qdd[8]*S6+qd[8]*(OMcp11_17*ROcp11_56-OMcp11_27*ROcp11_46);
    RLcp11_19 = ROcp11_18*s->dpt[1][8];
    RLcp11_29 = ROcp11_28*s->dpt[1][8];
    RLcp11_39 = -s->dpt[1][8]*S7p8*C6;
    POcp11_19 = RLcp11_12+RLcp11_13+RLcp11_18+RLcp11_19;
    POcp11_29 = RLcp11_22+RLcp11_23+RLcp11_28+RLcp11_29;
    POcp11_39 = RLcp11_38+RLcp11_39+q[4];
    OMcp11_19 = OMcp11_18+ROcp11_46*qd[9];
    OMcp11_29 = OMcp11_28+ROcp11_56*qd[9];
    OMcp11_39 = OMcp11_38+qd[9]*S6;
    ORcp11_19 = OMcp11_28*RLcp11_39-OMcp11_38*RLcp11_29;
    ORcp11_29 = -(OMcp11_18*RLcp11_39-OMcp11_38*RLcp11_19);
    ORcp11_39 = OMcp11_18*RLcp11_29-OMcp11_28*RLcp11_19;
    VIcp11_19 = ORcp11_12+ORcp11_13+ORcp11_18+ORcp11_19+qd[2]*C1-qd[3]*S1;
    VIcp11_29 = ORcp11_22+ORcp11_23+ORcp11_28+ORcp11_29+qd[2]*S1+qd[3]*C1;
    VIcp11_39 = ORcp11_38+ORcp11_39+qd[4];
    OPcp11_19 = OPcp11_18+ROcp11_46*qdd[9]+qd[9]*(OMcp11_28*S6-OMcp11_38*ROcp11_56);
    OPcp11_29 = OPcp11_28+ROcp11_56*qdd[9]-qd[9]*(OMcp11_18*S6-OMcp11_38*ROcp11_46);
    OPcp11_39 = OPcp11_38+qdd[9]*S6+qd[9]*(OMcp11_18*ROcp11_56-OMcp11_28*ROcp11_46);
    ACcp11_19 = OMcp11_27*ORcp11_38+OMcp11_28*ORcp11_39-OMcp11_37*ORcp11_28-OMcp11_38*ORcp11_29+OPcp11_27*RLcp11_38+
 OPcp11_28*RLcp11_39-OPcp11_37*RLcp11_28-OPcp11_38*RLcp11_29-ORcp11_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(
 ORcp11_22+(2.0)*qd[2]*S1);
    ACcp11_29 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp11_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp11_13-(2.0)*qd[3]*S1)-OMcp11_17*ORcp11_38+OMcp11_37*
 ORcp11_18-OPcp11_17*RLcp11_38+OPcp11_37*RLcp11_18-OMcp11_18*ORcp11_39+OMcp11_38*ORcp11_19-OPcp11_18*RLcp11_39+OPcp11_38*
 RLcp11_19;
    ACcp11_39 = qdd[4]+OMcp11_17*ORcp11_28+OMcp11_18*ORcp11_29-OMcp11_27*ORcp11_18-OMcp11_28*ORcp11_19+OPcp11_17*RLcp11_28
 +OPcp11_18*RLcp11_29-OPcp11_27*RLcp11_18-OPcp11_28*RLcp11_19;

// = = Block_1_0_0_12_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp11_19;
    sens->P[2] = POcp11_29;
    sens->P[3] = POcp11_39;
    sens->R[1][1] = ROcp11_19;
    sens->R[1][2] = ROcp11_29;
    sens->R[1][3] = ROcp11_39;
    sens->R[2][1] = ROcp11_46;
    sens->R[2][2] = ROcp11_56;
    sens->R[2][3] = S6;
    sens->R[3][1] = ROcp11_79;
    sens->R[3][2] = ROcp11_89;
    sens->R[3][3] = ROcp11_99;
    sens->V[1] = VIcp11_19;
    sens->V[2] = VIcp11_29;
    sens->V[3] = VIcp11_39;
    sens->OM[1] = OMcp11_19;
    sens->OM[2] = OMcp11_29;
    sens->OM[3] = OMcp11_39;
    sens->A[1] = ACcp11_19;
    sens->A[2] = ACcp11_29;
    sens->A[3] = ACcp11_39;
    sens->OMP[1] = OPcp11_19;
    sens->OMP[2] = OPcp11_29;
    sens->OMP[3] = OPcp11_39;
 
// 
break;
case 13:
 


// = = Block_1_0_0_13_0_1 = = 
 
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
    ORcp12_12 = -RLcp12_22*qd[1];
    ORcp12_22 = RLcp12_12*qd[1];
    RLcp12_13 = -q[3]*S1;
    RLcp12_23 = q[3]*C1;
    ORcp12_13 = -RLcp12_23*qd[1];
    ORcp12_23 = RLcp12_13*qd[1];
    OMcp12_35 = qd[1]+qd[5];
    OMcp12_16 = qd[6]*C1p5;
    OMcp12_26 = qd[6]*S1p5;
    OMcp12_17 = OMcp12_16+ROcp12_46*qd[7];
    OMcp12_27 = OMcp12_26+ROcp12_56*qd[7];
    OMcp12_37 = OMcp12_35+qd[7]*S6;
    OPcp12_17 = -(OMcp12_35*qd[6]*S1p5-ROcp12_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp12_26*S6-OMcp12_35*ROcp12_56));
    OPcp12_27 = OMcp12_35*qd[6]*C1p5+ROcp12_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp12_16*S6-OMcp12_35*ROcp12_46);
    OPcp12_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_13_0_3 = = 
 
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

// = = Block_1_0_0_13_0_5 = = 
 
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
    OMcp12_112 = OMcp12_17+ROcp12_110*qd[12];
    OMcp12_212 = OMcp12_27+ROcp12_210*qd[12];
    OMcp12_312 = OMcp12_37+ROcp12_310*qd[12];
    ORcp12_112 = OMcp12_27*RLcp12_312-OMcp12_37*RLcp12_212;
    ORcp12_212 = -(OMcp12_17*RLcp12_312-OMcp12_37*RLcp12_112);
    ORcp12_312 = OMcp12_17*RLcp12_212-OMcp12_27*RLcp12_112;
    OPcp12_112 = OPcp12_17+ROcp12_110*qdd[12]+qd[12]*(OMcp12_27*ROcp12_310-OMcp12_37*ROcp12_210);
    OPcp12_212 = OPcp12_27+ROcp12_210*qdd[12]-qd[12]*(OMcp12_17*ROcp12_310-OMcp12_37*ROcp12_110);
    OPcp12_312 = OPcp12_37+ROcp12_310*qdd[12]+qd[12]*(OMcp12_17*ROcp12_210-OMcp12_27*ROcp12_110);
    RLcp12_113 = ROcp12_412*s->dpt[2][16];
    RLcp12_213 = ROcp12_512*s->dpt[2][16];
    RLcp12_313 = ROcp12_612*s->dpt[2][16];
    OMcp12_113 = OMcp12_112+ROcp12_110*qd[13];
    OMcp12_213 = OMcp12_212+ROcp12_210*qd[13];
    OMcp12_313 = OMcp12_312+ROcp12_310*qd[13];
    ORcp12_113 = OMcp12_212*RLcp12_313-OMcp12_312*RLcp12_213;
    ORcp12_213 = -(OMcp12_112*RLcp12_313-OMcp12_312*RLcp12_113);
    ORcp12_313 = OMcp12_112*RLcp12_213-OMcp12_212*RLcp12_113;
    OMcp12_114 = OMcp12_113+ROcp12_413*qd[14];
    OMcp12_214 = OMcp12_213+ROcp12_513*qd[14];
    OMcp12_314 = OMcp12_313+ROcp12_613*qd[14];
    OMcp12_115 = OMcp12_114+ROcp12_714*qd[15];
    OMcp12_215 = OMcp12_214+ROcp12_814*qd[15];
    OMcp12_315 = OMcp12_314+ROcp12_914*qd[15];
    OPcp12_115 = OPcp12_112+ROcp12_110*qdd[13]+ROcp12_413*qdd[14]+ROcp12_714*qdd[15]+qd[13]*(OMcp12_212*ROcp12_310-
 OMcp12_312*ROcp12_210)+qd[14]*(OMcp12_213*ROcp12_613-OMcp12_313*ROcp12_513)+qd[15]*(OMcp12_214*ROcp12_914-OMcp12_314*
 ROcp12_814);
    OPcp12_215 = OPcp12_212+ROcp12_210*qdd[13]+ROcp12_513*qdd[14]+ROcp12_814*qdd[15]-qd[13]*(OMcp12_112*ROcp12_310-
 OMcp12_312*ROcp12_110)-qd[14]*(OMcp12_113*ROcp12_613-OMcp12_313*ROcp12_413)-qd[15]*(OMcp12_114*ROcp12_914-OMcp12_314*
 ROcp12_714);
    OPcp12_315 = OPcp12_312+ROcp12_310*qdd[13]+ROcp12_613*qdd[14]+ROcp12_914*qdd[15]+qd[13]*(OMcp12_112*ROcp12_210-
 OMcp12_212*ROcp12_110)+qd[14]*(OMcp12_113*ROcp12_513-OMcp12_213*ROcp12_413)+qd[15]*(OMcp12_114*ROcp12_814-OMcp12_214*
 ROcp12_714);
    RLcp12_116 = ROcp12_115*s->dpt[1][20]+ROcp12_415*s->dpt[2][20]+ROcp12_714*s->dpt[3][20];
    RLcp12_216 = ROcp12_215*s->dpt[1][20]+ROcp12_515*s->dpt[2][20]+ROcp12_814*s->dpt[3][20];
    RLcp12_316 = ROcp12_315*s->dpt[1][20]+ROcp12_615*s->dpt[2][20]+ROcp12_914*s->dpt[3][20];
    POcp12_116 = RLcp12_110+RLcp12_112+RLcp12_113+RLcp12_116+RLcp12_12+RLcp12_13;
    POcp12_216 = RLcp12_210+RLcp12_212+RLcp12_213+RLcp12_216+RLcp12_22+RLcp12_23;
    POcp12_316 = RLcp12_310+RLcp12_312+RLcp12_313+RLcp12_316+q[4];
    ORcp12_116 = OMcp12_215*RLcp12_316-OMcp12_315*RLcp12_216;
    ORcp12_216 = -(OMcp12_115*RLcp12_316-OMcp12_315*RLcp12_116);
    ORcp12_316 = OMcp12_115*RLcp12_216-OMcp12_215*RLcp12_116;
    VIcp12_116 = ORcp12_110+ORcp12_112+ORcp12_113+ORcp12_116+ORcp12_12+ORcp12_13+qd[2]*C1-qd[3]*S1;
    VIcp12_216 = ORcp12_210+ORcp12_212+ORcp12_213+ORcp12_216+ORcp12_22+ORcp12_23+qd[2]*S1+qd[3]*C1;
    VIcp12_316 = ORcp12_310+ORcp12_312+ORcp12_313+ORcp12_316+qd[4];
    ACcp12_116 = OMcp12_212*ORcp12_313+OMcp12_215*ORcp12_316+OMcp12_27*(ORcp12_310+ORcp12_312)-OMcp12_312*ORcp12_213-
 OMcp12_315*ORcp12_216-OMcp12_37*ORcp12_210-OMcp12_37*ORcp12_212+OPcp12_212*RLcp12_313+OPcp12_215*RLcp12_316+OPcp12_27*
 RLcp12_310+OPcp12_27*RLcp12_312-OPcp12_312*RLcp12_213-OPcp12_315*RLcp12_216-OPcp12_37*RLcp12_210-OPcp12_37*RLcp12_212-
 ORcp12_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp12_22+(2.0)*qd[2]*S1);
    ACcp12_216 = -(OMcp12_112*ORcp12_313+OMcp12_115*ORcp12_316+OMcp12_17*(ORcp12_310+ORcp12_312)-OMcp12_312*ORcp12_113-
 OMcp12_315*ORcp12_116-OMcp12_37*ORcp12_110-OMcp12_37*ORcp12_112+OPcp12_112*RLcp12_313+OPcp12_115*RLcp12_316+OPcp12_17*
 RLcp12_310+OPcp12_17*RLcp12_312-OPcp12_312*RLcp12_113-OPcp12_315*RLcp12_116-OPcp12_37*RLcp12_110-OPcp12_37*RLcp12_112-qdd[2]
 *S1-qdd[3]*C1-qd[1]*(ORcp12_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp12_13-(2.0)*qd[3]*S1));
    ACcp12_316 = qdd[4]+OMcp12_112*ORcp12_213+OMcp12_115*ORcp12_216+OMcp12_17*ORcp12_210+OMcp12_17*ORcp12_212-OMcp12_212*
 ORcp12_113-OMcp12_215*ORcp12_116-OMcp12_27*ORcp12_110-OMcp12_27*ORcp12_112+OPcp12_112*RLcp12_213+OPcp12_115*RLcp12_216+
 OPcp12_17*RLcp12_210+OPcp12_17*RLcp12_212-OPcp12_212*RLcp12_113-OPcp12_215*RLcp12_116-OPcp12_27*RLcp12_110-OPcp12_27*
 RLcp12_112;
    OMcp12_117 = OMcp12_115+ROcp12_416*qd[17];
    OMcp12_217 = OMcp12_215+ROcp12_516*qd[17];
    OMcp12_317 = OMcp12_315+ROcp12_616*qd[17];
    OPcp12_117 = OPcp12_115+ROcp12_416*qdd[17]+qd[17]*(OMcp12_215*ROcp12_616-OMcp12_315*ROcp12_516);
    OPcp12_217 = OPcp12_215+ROcp12_516*qdd[17]-qd[17]*(OMcp12_115*ROcp12_616-OMcp12_315*ROcp12_416);
    OPcp12_317 = OPcp12_315+ROcp12_616*qdd[17]+qd[17]*(OMcp12_115*ROcp12_516-OMcp12_215*ROcp12_416);

// = = Block_1_0_0_13_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp12_116;
    sens->P[2] = POcp12_216;
    sens->P[3] = POcp12_316;
    sens->R[1][1] = ROcp12_117;
    sens->R[1][2] = ROcp12_217;
    sens->R[1][3] = ROcp12_317;
    sens->R[2][1] = ROcp12_416;
    sens->R[2][2] = ROcp12_516;
    sens->R[2][3] = ROcp12_616;
    sens->R[3][1] = ROcp12_717;
    sens->R[3][2] = ROcp12_817;
    sens->R[3][3] = ROcp12_917;
    sens->V[1] = VIcp12_116;
    sens->V[2] = VIcp12_216;
    sens->V[3] = VIcp12_316;
    sens->OM[1] = OMcp12_117;
    sens->OM[2] = OMcp12_217;
    sens->OM[3] = OMcp12_317;
    sens->A[1] = ACcp12_116;
    sens->A[2] = ACcp12_216;
    sens->A[3] = ACcp12_316;
    sens->OMP[1] = OPcp12_117;
    sens->OMP[2] = OPcp12_217;
    sens->OMP[3] = OPcp12_317;
 
// 
break;
case 14:
 


// = = Block_1_0_0_14_0_1 = = 
 
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
    ORcp13_12 = -RLcp13_22*qd[1];
    ORcp13_22 = RLcp13_12*qd[1];
    RLcp13_13 = -q[3]*S1;
    RLcp13_23 = q[3]*C1;
    ORcp13_13 = -RLcp13_23*qd[1];
    ORcp13_23 = RLcp13_13*qd[1];
    OMcp13_35 = qd[1]+qd[5];
    OMcp13_16 = qd[6]*C1p5;
    OMcp13_26 = qd[6]*S1p5;
    OMcp13_17 = OMcp13_16+ROcp13_46*qd[7];
    OMcp13_27 = OMcp13_26+ROcp13_56*qd[7];
    OMcp13_37 = OMcp13_35+qd[7]*S6;
    OPcp13_17 = -(OMcp13_35*qd[6]*S1p5-ROcp13_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp13_26*S6-OMcp13_35*ROcp13_56));
    OPcp13_27 = OMcp13_35*qd[6]*C1p5+ROcp13_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp13_16*S6-OMcp13_35*ROcp13_46);
    OPcp13_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_14_0_3 = = 
 
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

// = = Block_1_0_0_14_0_6 = = 
 
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
    OMcp13_118 = OMcp13_17+ROcp13_110*qd[18];
    OMcp13_218 = OMcp13_27+ROcp13_210*qd[18];
    OMcp13_318 = OMcp13_37+ROcp13_310*qd[18];
    ORcp13_118 = OMcp13_27*RLcp13_318-OMcp13_37*RLcp13_218;
    ORcp13_218 = -(OMcp13_17*RLcp13_318-OMcp13_37*RLcp13_118);
    ORcp13_318 = OMcp13_17*RLcp13_218-OMcp13_27*RLcp13_118;
    OPcp13_118 = OPcp13_17+ROcp13_110*qdd[18]+qd[18]*(OMcp13_27*ROcp13_310-OMcp13_37*ROcp13_210);
    OPcp13_218 = OPcp13_27+ROcp13_210*qdd[18]-qd[18]*(OMcp13_17*ROcp13_310-OMcp13_37*ROcp13_110);
    OPcp13_318 = OPcp13_37+ROcp13_310*qdd[18]+qd[18]*(OMcp13_17*ROcp13_210-OMcp13_27*ROcp13_110);
    RLcp13_119 = ROcp13_418*s->dpt[2][23];
    RLcp13_219 = ROcp13_518*s->dpt[2][23];
    RLcp13_319 = ROcp13_618*s->dpt[2][23];
    OMcp13_119 = OMcp13_118+ROcp13_110*qd[19];
    OMcp13_219 = OMcp13_218+ROcp13_210*qd[19];
    OMcp13_319 = OMcp13_318+ROcp13_310*qd[19];
    ORcp13_119 = OMcp13_218*RLcp13_319-OMcp13_318*RLcp13_219;
    ORcp13_219 = -(OMcp13_118*RLcp13_319-OMcp13_318*RLcp13_119);
    ORcp13_319 = OMcp13_118*RLcp13_219-OMcp13_218*RLcp13_119;
    OMcp13_120 = OMcp13_119+ROcp13_419*qd[20];
    OMcp13_220 = OMcp13_219+ROcp13_519*qd[20];
    OMcp13_320 = OMcp13_319+ROcp13_619*qd[20];
    OMcp13_121 = OMcp13_120+ROcp13_720*qd[21];
    OMcp13_221 = OMcp13_220+ROcp13_820*qd[21];
    OMcp13_321 = OMcp13_320+ROcp13_920*qd[21];
    OPcp13_121 = OPcp13_118+ROcp13_110*qdd[19]+ROcp13_419*qdd[20]+ROcp13_720*qdd[21]+qd[19]*(OMcp13_218*ROcp13_310-
 OMcp13_318*ROcp13_210)+qd[20]*(OMcp13_219*ROcp13_619-OMcp13_319*ROcp13_519)+qd[21]*(OMcp13_220*ROcp13_920-OMcp13_320*
 ROcp13_820);
    OPcp13_221 = OPcp13_218+ROcp13_210*qdd[19]+ROcp13_519*qdd[20]+ROcp13_820*qdd[21]-qd[19]*(OMcp13_118*ROcp13_310-
 OMcp13_318*ROcp13_110)-qd[20]*(OMcp13_119*ROcp13_619-OMcp13_319*ROcp13_419)-qd[21]*(OMcp13_120*ROcp13_920-OMcp13_320*
 ROcp13_720);
    OPcp13_321 = OPcp13_318+ROcp13_310*qdd[19]+ROcp13_619*qdd[20]+ROcp13_920*qdd[21]+qd[19]*(OMcp13_118*ROcp13_210-
 OMcp13_218*ROcp13_110)+qd[20]*(OMcp13_119*ROcp13_519-OMcp13_219*ROcp13_419)+qd[21]*(OMcp13_120*ROcp13_820-OMcp13_220*
 ROcp13_720);
    RLcp13_122 = ROcp13_121*s->dpt[1][27]+ROcp13_421*s->dpt[2][27]+ROcp13_720*s->dpt[3][27];
    RLcp13_222 = ROcp13_221*s->dpt[1][27]+ROcp13_521*s->dpt[2][27]+ROcp13_820*s->dpt[3][27];
    RLcp13_322 = ROcp13_321*s->dpt[1][27]+ROcp13_621*s->dpt[2][27]+ROcp13_920*s->dpt[3][27];
    POcp13_122 = RLcp13_110+RLcp13_118+RLcp13_119+RLcp13_12+RLcp13_122+RLcp13_13;
    POcp13_222 = RLcp13_210+RLcp13_218+RLcp13_219+RLcp13_22+RLcp13_222+RLcp13_23;
    POcp13_322 = RLcp13_310+RLcp13_318+RLcp13_319+RLcp13_322+q[4];
    ORcp13_122 = OMcp13_221*RLcp13_322-OMcp13_321*RLcp13_222;
    ORcp13_222 = -(OMcp13_121*RLcp13_322-OMcp13_321*RLcp13_122);
    ORcp13_322 = OMcp13_121*RLcp13_222-OMcp13_221*RLcp13_122;
    VIcp13_122 = ORcp13_110+ORcp13_118+ORcp13_119+ORcp13_12+ORcp13_122+ORcp13_13+qd[2]*C1-qd[3]*S1;
    VIcp13_222 = ORcp13_210+ORcp13_218+ORcp13_219+ORcp13_22+ORcp13_222+ORcp13_23+qd[2]*S1+qd[3]*C1;
    VIcp13_322 = ORcp13_310+ORcp13_318+ORcp13_319+ORcp13_322+qd[4];
    ACcp13_122 = OMcp13_218*ORcp13_319+OMcp13_221*ORcp13_322+OMcp13_27*(ORcp13_310+ORcp13_318)-OMcp13_318*ORcp13_219-
 OMcp13_321*ORcp13_222-OMcp13_37*ORcp13_210-OMcp13_37*ORcp13_218+OPcp13_218*RLcp13_319+OPcp13_221*RLcp13_322+OPcp13_27*
 RLcp13_310+OPcp13_27*RLcp13_318-OPcp13_318*RLcp13_219-OPcp13_321*RLcp13_222-OPcp13_37*RLcp13_210-OPcp13_37*RLcp13_218-
 ORcp13_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp13_22+(2.0)*qd[2]*S1);
    ACcp13_222 = -(OMcp13_118*ORcp13_319+OMcp13_121*ORcp13_322+OMcp13_17*(ORcp13_310+ORcp13_318)-OMcp13_318*ORcp13_119-
 OMcp13_321*ORcp13_122-OMcp13_37*ORcp13_110-OMcp13_37*ORcp13_118+OPcp13_118*RLcp13_319+OPcp13_121*RLcp13_322+OPcp13_17*
 RLcp13_310+OPcp13_17*RLcp13_318-OPcp13_318*RLcp13_119-OPcp13_321*RLcp13_122-OPcp13_37*RLcp13_110-OPcp13_37*RLcp13_118-qdd[2]
 *S1-qdd[3]*C1-qd[1]*(ORcp13_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp13_13-(2.0)*qd[3]*S1));
    ACcp13_322 = qdd[4]+OMcp13_118*ORcp13_219+OMcp13_121*ORcp13_222+OMcp13_17*ORcp13_210+OMcp13_17*ORcp13_218-OMcp13_218*
 ORcp13_119-OMcp13_221*ORcp13_122-OMcp13_27*ORcp13_110-OMcp13_27*ORcp13_118+OPcp13_118*RLcp13_219+OPcp13_121*RLcp13_222+
 OPcp13_17*RLcp13_210+OPcp13_17*RLcp13_218-OPcp13_218*RLcp13_119-OPcp13_221*RLcp13_122-OPcp13_27*RLcp13_110-OPcp13_27*
 RLcp13_118;
    OMcp13_123 = OMcp13_121+ROcp13_422*qd[23];
    OMcp13_223 = OMcp13_221+ROcp13_522*qd[23];
    OMcp13_323 = OMcp13_321+ROcp13_622*qd[23];
    OPcp13_123 = OPcp13_121+ROcp13_422*qdd[23]+qd[23]*(OMcp13_221*ROcp13_622-OMcp13_321*ROcp13_522);
    OPcp13_223 = OPcp13_221+ROcp13_522*qdd[23]-qd[23]*(OMcp13_121*ROcp13_622-OMcp13_321*ROcp13_422);
    OPcp13_323 = OPcp13_321+ROcp13_622*qdd[23]+qd[23]*(OMcp13_121*ROcp13_522-OMcp13_221*ROcp13_422);

// = = Block_1_0_0_14_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp13_122;
    sens->P[2] = POcp13_222;
    sens->P[3] = POcp13_322;
    sens->R[1][1] = ROcp13_123;
    sens->R[1][2] = ROcp13_223;
    sens->R[1][3] = ROcp13_323;
    sens->R[2][1] = ROcp13_422;
    sens->R[2][2] = ROcp13_522;
    sens->R[2][3] = ROcp13_622;
    sens->R[3][1] = ROcp13_723;
    sens->R[3][2] = ROcp13_823;
    sens->R[3][3] = ROcp13_923;
    sens->V[1] = VIcp13_122;
    sens->V[2] = VIcp13_222;
    sens->V[3] = VIcp13_322;
    sens->OM[1] = OMcp13_123;
    sens->OM[2] = OMcp13_223;
    sens->OM[3] = OMcp13_323;
    sens->A[1] = ACcp13_122;
    sens->A[2] = ACcp13_222;
    sens->A[3] = ACcp13_322;
    sens->OMP[1] = OPcp13_123;
    sens->OMP[2] = OPcp13_223;
    sens->OMP[3] = OPcp13_323;

break;
default:
break;
}


// ====== END Task 1 ====== 


}
 


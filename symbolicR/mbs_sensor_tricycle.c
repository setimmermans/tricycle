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
//	==> Generation Date : Mon Dec  5 17:03:59 2016
//
//	==> Project name : tricycle
//	==> using XML input file 
//
//	==> Number of joints : 31
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

  S11p7 = C11*S7+S11*C7;
  C11p7 = C11*C7-S11*S7;

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
    RLcp1_147 = ROcp1_17*s->dpt[1][4]+ROcp1_77*s->dpt[3][4];
    RLcp1_247 = ROcp1_27*s->dpt[1][4]+ROcp1_87*s->dpt[3][4];
    RLcp1_347 = ROcp1_37*s->dpt[1][4]+ROcp1_97*s->dpt[3][4];
    POcp1_147 = RLcp1_12+RLcp1_13+RLcp1_147;
    POcp1_247 = RLcp1_22+RLcp1_23+RLcp1_247;
    POcp1_347 = RLcp1_347+q[4];
    JTcp1_147_1 = -(RLcp1_22+RLcp1_23+RLcp1_247);
    JTcp1_247_1 = RLcp1_12+RLcp1_13+RLcp1_147;
    JTcp1_147_6 = RLcp1_347*S1p5;
    JTcp1_247_6 = -RLcp1_347*C1p5;
    JTcp1_347_6 = -(RLcp1_147*S1p5-RLcp1_247*C1p5);
    JTcp1_147_7 = -(RLcp1_247*S6-RLcp1_347*ROcp1_56);
    JTcp1_247_7 = RLcp1_147*S6-RLcp1_347*ROcp1_46;
    JTcp1_347_7 = -(RLcp1_147*ROcp1_56-RLcp1_247*ROcp1_46);
    ORcp1_147 = OMcp1_27*RLcp1_347-OMcp1_37*RLcp1_247;
    ORcp1_247 = -(OMcp1_17*RLcp1_347-OMcp1_37*RLcp1_147);
    ORcp1_347 = OMcp1_17*RLcp1_247-OMcp1_27*RLcp1_147;
    VIcp1_147 = ORcp1_12+ORcp1_13+ORcp1_147+qd[2]*C1-qd[3]*S1;
    VIcp1_247 = ORcp1_22+ORcp1_23+ORcp1_247+qd[2]*S1+qd[3]*C1;
    VIcp1_347 = ORcp1_347+qd[4];
    ACcp1_147 = OMcp1_27*ORcp1_347-OMcp1_37*ORcp1_247+OPcp1_27*RLcp1_347-OPcp1_37*RLcp1_247-ORcp1_23*qd[1]+qdd[2]*C1-
 qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp1_22+(2.0)*qd[2]*S1);
    ACcp1_247 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp1_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp1_13-(2.0)*qd[3]*S1)-OMcp1_17*ORcp1_347+OMcp1_37*
 ORcp1_147-OPcp1_17*RLcp1_347+OPcp1_37*RLcp1_147;
    ACcp1_347 = qdd[4]+OMcp1_17*ORcp1_247-OMcp1_27*ORcp1_147+OPcp1_17*RLcp1_247-OPcp1_27*RLcp1_147;

// = = Block_1_0_0_2_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp1_147;
    sens->P[2] = POcp1_247;
    sens->P[3] = POcp1_347;
    sens->R[1][1] = ROcp1_17;
    sens->R[1][2] = ROcp1_27;
    sens->R[1][3] = ROcp1_37;
    sens->R[2][1] = ROcp1_46;
    sens->R[2][2] = ROcp1_56;
    sens->R[2][3] = S6;
    sens->R[3][1] = ROcp1_77;
    sens->R[3][2] = ROcp1_87;
    sens->R[3][3] = ROcp1_97;
    sens->V[1] = VIcp1_147;
    sens->V[2] = VIcp1_247;
    sens->V[3] = VIcp1_347;
    sens->OM[1] = OMcp1_17;
    sens->OM[2] = OMcp1_27;
    sens->OM[3] = OMcp1_37;
    sens->J[1][1] = JTcp1_147_1;
    sens->J[1][2] = C1;
    sens->J[1][3] = -S1;
    sens->J[1][5] = -RLcp1_247;
    sens->J[1][6] = JTcp1_147_6;
    sens->J[1][7] = JTcp1_147_7;
    sens->J[2][1] = JTcp1_247_1;
    sens->J[2][2] = S1;
    sens->J[2][3] = C1;
    sens->J[2][5] = RLcp1_147;
    sens->J[2][6] = JTcp1_247_6;
    sens->J[2][7] = JTcp1_247_7;
    sens->J[3][4] = (1.0);
    sens->J[3][6] = JTcp1_347_6;
    sens->J[3][7] = JTcp1_347_7;
    sens->J[4][6] = C1p5;
    sens->J[4][7] = ROcp1_46;
    sens->J[5][6] = S1p5;
    sens->J[5][7] = ROcp1_56;
    sens->J[6][1] = (1.0);
    sens->J[6][5] = (1.0);
    sens->J[6][7] = S6;
    sens->A[1] = ACcp1_147;
    sens->A[2] = ACcp1_247;
    sens->A[3] = ACcp1_347;
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


    ROcp3_111 = ROcp3_17*C11-ROcp3_77*S11;
    ROcp3_211 = ROcp3_27*C11-ROcp3_87*S11;
    ROcp3_311 = -S11p7*C6;
    ROcp3_711 = ROcp3_17*S11+ROcp3_77*C11;
    ROcp3_811 = ROcp3_27*S11+ROcp3_87*C11;
    ROcp3_911 = C11p7*C6;
    RLcp3_111 = ROcp3_17*s->dpt[1][5]+ROcp3_77*s->dpt[3][5];
    RLcp3_211 = ROcp3_27*s->dpt[1][5]+ROcp3_87*s->dpt[3][5];
    RLcp3_311 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp3_111 = OMcp3_27*RLcp3_311-OMcp3_37*RLcp3_211;
    ORcp3_211 = -(OMcp3_17*RLcp3_311-OMcp3_37*RLcp3_111);
    ORcp3_311 = OMcp3_17*RLcp3_211-OMcp3_27*RLcp3_111;

// = = Block_1_0_0_4_0_5 = = 
 
// Sensor Kinematics 


    ROcp3_413 = ROcp3_46*C13+ROcp3_711*S13;
    ROcp3_513 = ROcp3_56*C13+ROcp3_811*S13;
    ROcp3_613 = ROcp3_911*S13+C13*S6;
    ROcp3_713 = -(ROcp3_46*S13-ROcp3_711*C13);
    ROcp3_813 = -(ROcp3_56*S13-ROcp3_811*C13);
    ROcp3_913 = ROcp3_911*C13-S13*S6;
    ROcp3_414 = ROcp3_413*C14+ROcp3_713*S14;
    ROcp3_514 = ROcp3_513*C14+ROcp3_813*S14;
    ROcp3_614 = ROcp3_613*C14+ROcp3_913*S14;
    ROcp3_714 = -(ROcp3_413*S14-ROcp3_713*C14);
    ROcp3_814 = -(ROcp3_513*S14-ROcp3_813*C14);
    ROcp3_914 = -(ROcp3_613*S14-ROcp3_913*C14);
    ROcp3_115 = ROcp3_111*C15-ROcp3_714*S15;
    ROcp3_215 = ROcp3_211*C15-ROcp3_814*S15;
    ROcp3_315 = ROcp3_311*C15-ROcp3_914*S15;
    ROcp3_715 = ROcp3_111*S15+ROcp3_714*C15;
    ROcp3_815 = ROcp3_211*S15+ROcp3_814*C15;
    ROcp3_915 = ROcp3_311*S15+ROcp3_914*C15;
    ROcp3_116 = ROcp3_115*C16+ROcp3_414*S16;
    ROcp3_216 = ROcp3_215*C16+ROcp3_514*S16;
    ROcp3_316 = ROcp3_315*C16+ROcp3_614*S16;
    ROcp3_416 = -(ROcp3_115*S16-ROcp3_414*C16);
    ROcp3_516 = -(ROcp3_215*S16-ROcp3_514*C16);
    ROcp3_616 = -(ROcp3_315*S16-ROcp3_614*C16);
    RLcp3_113 = ROcp3_46*s->dpt[2][12]+ROcp3_711*s->dpt[3][12];
    RLcp3_213 = ROcp3_56*s->dpt[2][12]+ROcp3_811*s->dpt[3][12];
    RLcp3_313 = ROcp3_911*s->dpt[3][12]+s->dpt[2][12]*S6;
    OMcp3_113 = OMcp3_17+ROcp3_111*qd[13];
    OMcp3_213 = OMcp3_27+ROcp3_211*qd[13];
    OMcp3_313 = OMcp3_37+ROcp3_311*qd[13];
    ORcp3_113 = OMcp3_27*RLcp3_313-OMcp3_37*RLcp3_213;
    ORcp3_213 = -(OMcp3_17*RLcp3_313-OMcp3_37*RLcp3_113);
    ORcp3_313 = OMcp3_17*RLcp3_213-OMcp3_27*RLcp3_113;
    OPcp3_113 = OPcp3_17+ROcp3_111*qdd[13]+qd[13]*(OMcp3_27*ROcp3_311-OMcp3_37*ROcp3_211);
    OPcp3_213 = OPcp3_27+ROcp3_211*qdd[13]-qd[13]*(OMcp3_17*ROcp3_311-OMcp3_37*ROcp3_111);
    OPcp3_313 = OPcp3_37+ROcp3_311*qdd[13]+qd[13]*(OMcp3_17*ROcp3_211-OMcp3_27*ROcp3_111);
    RLcp3_114 = ROcp3_413*s->dpt[2][17];
    RLcp3_214 = ROcp3_513*s->dpt[2][17];
    RLcp3_314 = ROcp3_613*s->dpt[2][17];
    POcp3_114 = RLcp3_111+RLcp3_113+RLcp3_114+RLcp3_12+RLcp3_13;
    POcp3_214 = RLcp3_211+RLcp3_213+RLcp3_214+RLcp3_22+RLcp3_23;
    POcp3_314 = RLcp3_311+RLcp3_313+RLcp3_314+q[4];
    JTcp3_114_1 = -(RLcp3_211+RLcp3_213+RLcp3_214+RLcp3_22+RLcp3_23);
    JTcp3_214_1 = RLcp3_111+RLcp3_113+RLcp3_114+RLcp3_12+RLcp3_13;
    JTcp3_114_5 = -(RLcp3_211+RLcp3_213+RLcp3_214);
    JTcp3_214_5 = RLcp3_111+RLcp3_113+RLcp3_114;
    JTcp3_114_6 = S1p5*(RLcp3_311+RLcp3_313+RLcp3_314);
    JTcp3_214_6 = -C1p5*(RLcp3_311+RLcp3_313+RLcp3_314);
    JTcp3_314_6 = C1p5*(RLcp3_211+RLcp3_213)-S1p5*(RLcp3_111+RLcp3_113)-RLcp3_114*S1p5+RLcp3_214*C1p5;
    JTcp3_114_7 = ROcp3_56*(RLcp3_311+RLcp3_313)-S6*(RLcp3_211+RLcp3_213)-RLcp3_214*S6+RLcp3_314*ROcp3_56;
    JTcp3_214_7 = RLcp3_114*S6-RLcp3_314*ROcp3_46-ROcp3_46*(RLcp3_311+RLcp3_313)+S6*(RLcp3_111+RLcp3_113);
    JTcp3_314_7 = ROcp3_46*(RLcp3_211+RLcp3_213)-ROcp3_56*(RLcp3_111+RLcp3_113)-RLcp3_114*ROcp3_56+RLcp3_214*ROcp3_46;
    JTcp3_114_8 = ROcp3_56*(RLcp3_313+RLcp3_314)-S6*(RLcp3_213+RLcp3_214);
    JTcp3_214_8 = -(ROcp3_46*(RLcp3_313+RLcp3_314)-S6*(RLcp3_113+RLcp3_114));
    JTcp3_314_8 = ROcp3_46*(RLcp3_213+RLcp3_214)-ROcp3_56*(RLcp3_113+RLcp3_114);
    JTcp3_114_9 = -(RLcp3_214*ROcp3_311-RLcp3_314*ROcp3_211);
    JTcp3_214_9 = RLcp3_114*ROcp3_311-RLcp3_314*ROcp3_111;
    JTcp3_314_9 = -(RLcp3_114*ROcp3_211-RLcp3_214*ROcp3_111);
    OMcp3_114 = OMcp3_113+ROcp3_111*qd[14];
    OMcp3_214 = OMcp3_213+ROcp3_211*qd[14];
    OMcp3_314 = OMcp3_313+ROcp3_311*qd[14];
    ORcp3_114 = OMcp3_213*RLcp3_314-OMcp3_313*RLcp3_214;
    ORcp3_214 = -(OMcp3_113*RLcp3_314-OMcp3_313*RLcp3_114);
    ORcp3_314 = OMcp3_113*RLcp3_214-OMcp3_213*RLcp3_114;
    VIcp3_114 = ORcp3_111+ORcp3_113+ORcp3_114+ORcp3_12+ORcp3_13+qd[2]*C1-qd[3]*S1;
    VIcp3_214 = ORcp3_211+ORcp3_213+ORcp3_214+ORcp3_22+ORcp3_23+qd[2]*S1+qd[3]*C1;
    VIcp3_314 = ORcp3_311+ORcp3_313+ORcp3_314+qd[4];
    ACcp3_114 = OMcp3_213*ORcp3_314+OMcp3_27*(ORcp3_311+ORcp3_313)-OMcp3_313*ORcp3_214-OMcp3_37*ORcp3_211-OMcp3_37*
 ORcp3_213+OPcp3_213*RLcp3_314+OPcp3_27*RLcp3_311+OPcp3_27*RLcp3_313-OPcp3_313*RLcp3_214-OPcp3_37*RLcp3_211-OPcp3_37*
 RLcp3_213-ORcp3_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp3_22+(2.0)*qd[2]*S1);
    ACcp3_214 = -(OMcp3_113*ORcp3_314+OMcp3_17*(ORcp3_311+ORcp3_313)-OMcp3_313*ORcp3_114-OMcp3_37*ORcp3_111-OMcp3_37*
 ORcp3_113+OPcp3_113*RLcp3_314+OPcp3_17*RLcp3_311+OPcp3_17*RLcp3_313-OPcp3_313*RLcp3_114-OPcp3_37*RLcp3_111-OPcp3_37*
 RLcp3_113-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp3_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp3_13-(2.0)*qd[3]*S1));
    ACcp3_314 = qdd[4]+OMcp3_113*ORcp3_214+OMcp3_17*ORcp3_211+OMcp3_17*ORcp3_213-OMcp3_213*ORcp3_114-OMcp3_27*ORcp3_111-
 OMcp3_27*ORcp3_113+OPcp3_113*RLcp3_214+OPcp3_17*RLcp3_211+OPcp3_17*RLcp3_213-OPcp3_213*RLcp3_114-OPcp3_27*RLcp3_111-OPcp3_27
 *RLcp3_113;
    OMcp3_115 = OMcp3_114+ROcp3_414*qd[15];
    OMcp3_215 = OMcp3_214+ROcp3_514*qd[15];
    OMcp3_315 = OMcp3_314+ROcp3_614*qd[15];
    OMcp3_116 = OMcp3_115+ROcp3_715*qd[16];
    OMcp3_216 = OMcp3_215+ROcp3_815*qd[16];
    OMcp3_316 = OMcp3_315+ROcp3_915*qd[16];
    OPcp3_116 = OPcp3_113+ROcp3_111*qdd[14]+ROcp3_414*qdd[15]+ROcp3_715*qdd[16]+qd[14]*(OMcp3_213*ROcp3_311-OMcp3_313*
 ROcp3_211)+qd[15]*(OMcp3_214*ROcp3_614-OMcp3_314*ROcp3_514)+qd[16]*(OMcp3_215*ROcp3_915-OMcp3_315*ROcp3_815);
    OPcp3_216 = OPcp3_213+ROcp3_211*qdd[14]+ROcp3_514*qdd[15]+ROcp3_815*qdd[16]-qd[14]*(OMcp3_113*ROcp3_311-OMcp3_313*
 ROcp3_111)-qd[15]*(OMcp3_114*ROcp3_614-OMcp3_314*ROcp3_414)-qd[16]*(OMcp3_115*ROcp3_915-OMcp3_315*ROcp3_715);
    OPcp3_316 = OPcp3_313+ROcp3_311*qdd[14]+ROcp3_614*qdd[15]+ROcp3_915*qdd[16]+qd[14]*(OMcp3_113*ROcp3_211-OMcp3_213*
 ROcp3_111)+qd[15]*(OMcp3_114*ROcp3_514-OMcp3_214*ROcp3_414)+qd[16]*(OMcp3_115*ROcp3_815-OMcp3_215*ROcp3_715);

// = = Block_1_0_0_4_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp3_114;
    sens->P[2] = POcp3_214;
    sens->P[3] = POcp3_314;
    sens->R[1][1] = ROcp3_116;
    sens->R[1][2] = ROcp3_216;
    sens->R[1][3] = ROcp3_316;
    sens->R[2][1] = ROcp3_416;
    sens->R[2][2] = ROcp3_516;
    sens->R[2][3] = ROcp3_616;
    sens->R[3][1] = ROcp3_715;
    sens->R[3][2] = ROcp3_815;
    sens->R[3][3] = ROcp3_915;
    sens->V[1] = VIcp3_114;
    sens->V[2] = VIcp3_214;
    sens->V[3] = VIcp3_314;
    sens->OM[1] = OMcp3_116;
    sens->OM[2] = OMcp3_216;
    sens->OM[3] = OMcp3_316;
    sens->J[1][1] = JTcp3_114_1;
    sens->J[1][2] = C1;
    sens->J[1][3] = -S1;
    sens->J[1][5] = JTcp3_114_5;
    sens->J[1][6] = JTcp3_114_6;
    sens->J[1][7] = JTcp3_114_7;
    sens->J[1][11] = JTcp3_114_8;
    sens->J[1][13] = JTcp3_114_9;
    sens->J[2][1] = JTcp3_214_1;
    sens->J[2][2] = S1;
    sens->J[2][3] = C1;
    sens->J[2][5] = JTcp3_214_5;
    sens->J[2][6] = JTcp3_214_6;
    sens->J[2][7] = JTcp3_214_7;
    sens->J[2][11] = JTcp3_214_8;
    sens->J[2][13] = JTcp3_214_9;
    sens->J[3][4] = (1.0);
    sens->J[3][6] = JTcp3_314_6;
    sens->J[3][7] = JTcp3_314_7;
    sens->J[3][11] = JTcp3_314_8;
    sens->J[3][13] = JTcp3_314_9;
    sens->J[4][6] = C1p5;
    sens->J[4][7] = ROcp3_46;
    sens->J[4][11] = ROcp3_46;
    sens->J[4][13] = ROcp3_111;
    sens->J[4][14] = ROcp3_111;
    sens->J[4][15] = ROcp3_414;
    sens->J[4][16] = ROcp3_715;
    sens->J[5][6] = S1p5;
    sens->J[5][7] = ROcp3_56;
    sens->J[5][11] = ROcp3_56;
    sens->J[5][13] = ROcp3_211;
    sens->J[5][14] = ROcp3_211;
    sens->J[5][15] = ROcp3_514;
    sens->J[5][16] = ROcp3_815;
    sens->J[6][1] = (1.0);
    sens->J[6][5] = (1.0);
    sens->J[6][7] = S6;
    sens->J[6][11] = S6;
    sens->J[6][13] = ROcp3_311;
    sens->J[6][14] = ROcp3_311;
    sens->J[6][15] = ROcp3_614;
    sens->J[6][16] = ROcp3_915;
    sens->A[1] = ACcp3_114;
    sens->A[2] = ACcp3_214;
    sens->A[3] = ACcp3_314;
    sens->OMP[1] = OPcp3_116;
    sens->OMP[2] = OPcp3_216;
    sens->OMP[3] = OPcp3_316;
 
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


    ROcp4_111 = ROcp4_17*C11-ROcp4_77*S11;
    ROcp4_211 = ROcp4_27*C11-ROcp4_87*S11;
    ROcp4_311 = -S11p7*C6;
    ROcp4_711 = ROcp4_17*S11+ROcp4_77*C11;
    ROcp4_811 = ROcp4_27*S11+ROcp4_87*C11;
    ROcp4_911 = C11p7*C6;
    RLcp4_111 = ROcp4_17*s->dpt[1][5]+ROcp4_77*s->dpt[3][5];
    RLcp4_211 = ROcp4_27*s->dpt[1][5]+ROcp4_87*s->dpt[3][5];
    RLcp4_311 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp4_111 = OMcp4_27*RLcp4_311-OMcp4_37*RLcp4_211;
    ORcp4_211 = -(OMcp4_17*RLcp4_311-OMcp4_37*RLcp4_111);
    ORcp4_311 = OMcp4_17*RLcp4_211-OMcp4_27*RLcp4_111;

// = = Block_1_0_0_5_0_5 = = 
 
// Sensor Kinematics 


    ROcp4_413 = ROcp4_46*C13+ROcp4_711*S13;
    ROcp4_513 = ROcp4_56*C13+ROcp4_811*S13;
    ROcp4_613 = ROcp4_911*S13+C13*S6;
    ROcp4_713 = -(ROcp4_46*S13-ROcp4_711*C13);
    ROcp4_813 = -(ROcp4_56*S13-ROcp4_811*C13);
    ROcp4_913 = ROcp4_911*C13-S13*S6;
    ROcp4_414 = ROcp4_413*C14+ROcp4_713*S14;
    ROcp4_514 = ROcp4_513*C14+ROcp4_813*S14;
    ROcp4_614 = ROcp4_613*C14+ROcp4_913*S14;
    ROcp4_714 = -(ROcp4_413*S14-ROcp4_713*C14);
    ROcp4_814 = -(ROcp4_513*S14-ROcp4_813*C14);
    ROcp4_914 = -(ROcp4_613*S14-ROcp4_913*C14);
    ROcp4_115 = ROcp4_111*C15-ROcp4_714*S15;
    ROcp4_215 = ROcp4_211*C15-ROcp4_814*S15;
    ROcp4_315 = ROcp4_311*C15-ROcp4_914*S15;
    ROcp4_715 = ROcp4_111*S15+ROcp4_714*C15;
    ROcp4_815 = ROcp4_211*S15+ROcp4_814*C15;
    ROcp4_915 = ROcp4_311*S15+ROcp4_914*C15;
    ROcp4_116 = ROcp4_115*C16+ROcp4_414*S16;
    ROcp4_216 = ROcp4_215*C16+ROcp4_514*S16;
    ROcp4_316 = ROcp4_315*C16+ROcp4_614*S16;
    ROcp4_416 = -(ROcp4_115*S16-ROcp4_414*C16);
    ROcp4_516 = -(ROcp4_215*S16-ROcp4_514*C16);
    ROcp4_616 = -(ROcp4_315*S16-ROcp4_614*C16);
    RLcp4_113 = ROcp4_46*s->dpt[2][12]+ROcp4_711*s->dpt[3][12];
    RLcp4_213 = ROcp4_56*s->dpt[2][12]+ROcp4_811*s->dpt[3][12];
    RLcp4_313 = ROcp4_911*s->dpt[3][12]+s->dpt[2][12]*S6;
    OMcp4_113 = OMcp4_17+ROcp4_111*qd[13];
    OMcp4_213 = OMcp4_27+ROcp4_211*qd[13];
    OMcp4_313 = OMcp4_37+ROcp4_311*qd[13];
    ORcp4_113 = OMcp4_27*RLcp4_313-OMcp4_37*RLcp4_213;
    ORcp4_213 = -(OMcp4_17*RLcp4_313-OMcp4_37*RLcp4_113);
    ORcp4_313 = OMcp4_17*RLcp4_213-OMcp4_27*RLcp4_113;
    OPcp4_113 = OPcp4_17+ROcp4_111*qdd[13]+qd[13]*(OMcp4_27*ROcp4_311-OMcp4_37*ROcp4_211);
    OPcp4_213 = OPcp4_27+ROcp4_211*qdd[13]-qd[13]*(OMcp4_17*ROcp4_311-OMcp4_37*ROcp4_111);
    OPcp4_313 = OPcp4_37+ROcp4_311*qdd[13]+qd[13]*(OMcp4_17*ROcp4_211-OMcp4_27*ROcp4_111);
    RLcp4_114 = ROcp4_413*s->dpt[2][17];
    RLcp4_214 = ROcp4_513*s->dpt[2][17];
    RLcp4_314 = ROcp4_613*s->dpt[2][17];
    OMcp4_114 = OMcp4_113+ROcp4_111*qd[14];
    OMcp4_214 = OMcp4_213+ROcp4_211*qd[14];
    OMcp4_314 = OMcp4_313+ROcp4_311*qd[14];
    ORcp4_114 = OMcp4_213*RLcp4_314-OMcp4_313*RLcp4_214;
    ORcp4_214 = -(OMcp4_113*RLcp4_314-OMcp4_313*RLcp4_114);
    ORcp4_314 = OMcp4_113*RLcp4_214-OMcp4_213*RLcp4_114;
    OMcp4_115 = OMcp4_114+ROcp4_414*qd[15];
    OMcp4_215 = OMcp4_214+ROcp4_514*qd[15];
    OMcp4_315 = OMcp4_314+ROcp4_614*qd[15];
    OMcp4_116 = OMcp4_115+ROcp4_715*qd[16];
    OMcp4_216 = OMcp4_215+ROcp4_815*qd[16];
    OMcp4_316 = OMcp4_315+ROcp4_915*qd[16];
    OPcp4_116 = OPcp4_113+ROcp4_111*qdd[14]+ROcp4_414*qdd[15]+ROcp4_715*qdd[16]+qd[14]*(OMcp4_213*ROcp4_311-OMcp4_313*
 ROcp4_211)+qd[15]*(OMcp4_214*ROcp4_614-OMcp4_314*ROcp4_514)+qd[16]*(OMcp4_215*ROcp4_915-OMcp4_315*ROcp4_815);
    OPcp4_216 = OPcp4_213+ROcp4_211*qdd[14]+ROcp4_514*qdd[15]+ROcp4_815*qdd[16]-qd[14]*(OMcp4_113*ROcp4_311-OMcp4_313*
 ROcp4_111)-qd[15]*(OMcp4_114*ROcp4_614-OMcp4_314*ROcp4_414)-qd[16]*(OMcp4_115*ROcp4_915-OMcp4_315*ROcp4_715);
    OPcp4_316 = OPcp4_313+ROcp4_311*qdd[14]+ROcp4_614*qdd[15]+ROcp4_915*qdd[16]+qd[14]*(OMcp4_113*ROcp4_211-OMcp4_213*
 ROcp4_111)+qd[15]*(OMcp4_114*ROcp4_514-OMcp4_214*ROcp4_414)+qd[16]*(OMcp4_115*ROcp4_815-OMcp4_215*ROcp4_715);
    RLcp4_150 = ROcp4_715*s->dpt[3][20];
    RLcp4_250 = ROcp4_815*s->dpt[3][20];
    RLcp4_350 = ROcp4_915*s->dpt[3][20];
    POcp4_150 = RLcp4_111+RLcp4_113+RLcp4_114+RLcp4_12+RLcp4_13+RLcp4_150;
    POcp4_250 = RLcp4_211+RLcp4_213+RLcp4_214+RLcp4_22+RLcp4_23+RLcp4_250;
    POcp4_350 = RLcp4_311+RLcp4_313+RLcp4_314+RLcp4_350+q[4];
    JTcp4_150_1 = -(RLcp4_211+RLcp4_213+RLcp4_214+RLcp4_22+RLcp4_23+RLcp4_250);
    JTcp4_250_1 = RLcp4_111+RLcp4_113+RLcp4_114+RLcp4_12+RLcp4_13+RLcp4_150;
    JTcp4_150_5 = -(RLcp4_211+RLcp4_213+RLcp4_214+RLcp4_250);
    JTcp4_250_5 = RLcp4_111+RLcp4_113+RLcp4_114+RLcp4_150;
    JTcp4_150_6 = S1p5*(RLcp4_311+RLcp4_313+RLcp4_314+RLcp4_350);
    JTcp4_250_6 = -C1p5*(RLcp4_311+RLcp4_313+RLcp4_314+RLcp4_350);
    JTcp4_350_6 = C1p5*(RLcp4_211+RLcp4_213+RLcp4_214+RLcp4_250)-S1p5*(RLcp4_111+RLcp4_113)-S1p5*(RLcp4_114+RLcp4_150);
    JTcp4_150_7 = ROcp4_56*(RLcp4_311+RLcp4_313+RLcp4_314+RLcp4_350)-S6*(RLcp4_211+RLcp4_213)-S6*(RLcp4_214+RLcp4_250);
    JTcp4_250_7 = -(ROcp4_46*(RLcp4_311+RLcp4_313+RLcp4_314+RLcp4_350)-S6*(RLcp4_111+RLcp4_113)-S6*(RLcp4_114+RLcp4_150));
    JTcp4_350_7 = ROcp4_46*(RLcp4_211+RLcp4_213+RLcp4_214+RLcp4_250)-ROcp4_56*(RLcp4_111+RLcp4_113)-ROcp4_56*(RLcp4_114+
 RLcp4_150);
    JTcp4_150_8 = ROcp4_56*(RLcp4_313+RLcp4_314)-S6*(RLcp4_213+RLcp4_214)-RLcp4_250*S6+RLcp4_350*ROcp4_56;
    JTcp4_250_8 = RLcp4_150*S6-RLcp4_350*ROcp4_46-ROcp4_46*(RLcp4_313+RLcp4_314)+S6*(RLcp4_113+RLcp4_114);
    JTcp4_350_8 = ROcp4_46*(RLcp4_213+RLcp4_214)-ROcp4_56*(RLcp4_113+RLcp4_114)-RLcp4_150*ROcp4_56+RLcp4_250*ROcp4_46;
    JTcp4_150_9 = ROcp4_211*(RLcp4_314+RLcp4_350)-ROcp4_311*(RLcp4_214+RLcp4_250);
    JTcp4_250_9 = -(ROcp4_111*(RLcp4_314+RLcp4_350)-ROcp4_311*(RLcp4_114+RLcp4_150));
    JTcp4_350_9 = ROcp4_111*(RLcp4_214+RLcp4_250)-ROcp4_211*(RLcp4_114+RLcp4_150);
    JTcp4_150_10 = -(RLcp4_250*ROcp4_311-RLcp4_350*ROcp4_211);
    JTcp4_250_10 = RLcp4_150*ROcp4_311-RLcp4_350*ROcp4_111;
    JTcp4_350_10 = -(RLcp4_150*ROcp4_211-RLcp4_250*ROcp4_111);
    JTcp4_150_11 = -(RLcp4_250*ROcp4_614-RLcp4_350*ROcp4_514);
    JTcp4_250_11 = RLcp4_150*ROcp4_614-RLcp4_350*ROcp4_414;
    JTcp4_350_11 = -(RLcp4_150*ROcp4_514-RLcp4_250*ROcp4_414);
    JTcp4_150_12 = -(RLcp4_250*ROcp4_915-RLcp4_350*ROcp4_815);
    JTcp4_250_12 = RLcp4_150*ROcp4_915-RLcp4_350*ROcp4_715;
    JTcp4_350_12 = -(RLcp4_150*ROcp4_815-RLcp4_250*ROcp4_715);
    ORcp4_150 = OMcp4_216*RLcp4_350-OMcp4_316*RLcp4_250;
    ORcp4_250 = -(OMcp4_116*RLcp4_350-OMcp4_316*RLcp4_150);
    ORcp4_350 = OMcp4_116*RLcp4_250-OMcp4_216*RLcp4_150;
    VIcp4_150 = ORcp4_111+ORcp4_113+ORcp4_114+ORcp4_12+ORcp4_13+ORcp4_150+qd[2]*C1-qd[3]*S1;
    VIcp4_250 = ORcp4_211+ORcp4_213+ORcp4_214+ORcp4_22+ORcp4_23+ORcp4_250+qd[2]*S1+qd[3]*C1;
    VIcp4_350 = ORcp4_311+ORcp4_313+ORcp4_314+ORcp4_350+qd[4];
    ACcp4_150 = OMcp4_213*ORcp4_314+OMcp4_216*ORcp4_350+OMcp4_27*(ORcp4_311+ORcp4_313)-OMcp4_313*ORcp4_214-OMcp4_316*
 ORcp4_250-OMcp4_37*ORcp4_211-OMcp4_37*ORcp4_213+OPcp4_213*RLcp4_314+OPcp4_216*RLcp4_350+OPcp4_27*RLcp4_311+OPcp4_27*
 RLcp4_313-OPcp4_313*RLcp4_214-OPcp4_316*RLcp4_250-OPcp4_37*RLcp4_211-OPcp4_37*RLcp4_213-ORcp4_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*
 qd[1]*qd[3]*C1-qd[1]*(ORcp4_22+(2.0)*qd[2]*S1);
    ACcp4_250 = -(OMcp4_113*ORcp4_314+OMcp4_116*ORcp4_350+OMcp4_17*(ORcp4_311+ORcp4_313)-OMcp4_313*ORcp4_114-OMcp4_316*
 ORcp4_150-OMcp4_37*ORcp4_111-OMcp4_37*ORcp4_113+OPcp4_113*RLcp4_314+OPcp4_116*RLcp4_350+OPcp4_17*RLcp4_311+OPcp4_17*
 RLcp4_313-OPcp4_313*RLcp4_114-OPcp4_316*RLcp4_150-OPcp4_37*RLcp4_111-OPcp4_37*RLcp4_113-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp4_12+(2.0)
 *qd[2]*C1)-qd[1]*(ORcp4_13-(2.0)*qd[3]*S1));
    ACcp4_350 = qdd[4]+OMcp4_113*ORcp4_214+OMcp4_116*ORcp4_250+OMcp4_17*ORcp4_211+OMcp4_17*ORcp4_213-OMcp4_213*ORcp4_114-
 OMcp4_216*ORcp4_150-OMcp4_27*ORcp4_111-OMcp4_27*ORcp4_113+OPcp4_113*RLcp4_214+OPcp4_116*RLcp4_250+OPcp4_17*RLcp4_211+
 OPcp4_17*RLcp4_213-OPcp4_213*RLcp4_114-OPcp4_216*RLcp4_150-OPcp4_27*RLcp4_111-OPcp4_27*RLcp4_113;

// = = Block_1_0_0_5_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp4_150;
    sens->P[2] = POcp4_250;
    sens->P[3] = POcp4_350;
    sens->R[1][1] = ROcp4_116;
    sens->R[1][2] = ROcp4_216;
    sens->R[1][3] = ROcp4_316;
    sens->R[2][1] = ROcp4_416;
    sens->R[2][2] = ROcp4_516;
    sens->R[2][3] = ROcp4_616;
    sens->R[3][1] = ROcp4_715;
    sens->R[3][2] = ROcp4_815;
    sens->R[3][3] = ROcp4_915;
    sens->V[1] = VIcp4_150;
    sens->V[2] = VIcp4_250;
    sens->V[3] = VIcp4_350;
    sens->OM[1] = OMcp4_116;
    sens->OM[2] = OMcp4_216;
    sens->OM[3] = OMcp4_316;
    sens->J[1][1] = JTcp4_150_1;
    sens->J[1][2] = C1;
    sens->J[1][3] = -S1;
    sens->J[1][5] = JTcp4_150_5;
    sens->J[1][6] = JTcp4_150_6;
    sens->J[1][7] = JTcp4_150_7;
    sens->J[1][11] = JTcp4_150_8;
    sens->J[1][13] = JTcp4_150_9;
    sens->J[1][14] = JTcp4_150_10;
    sens->J[1][15] = JTcp4_150_11;
    sens->J[1][16] = JTcp4_150_12;
    sens->J[2][1] = JTcp4_250_1;
    sens->J[2][2] = S1;
    sens->J[2][3] = C1;
    sens->J[2][5] = JTcp4_250_5;
    sens->J[2][6] = JTcp4_250_6;
    sens->J[2][7] = JTcp4_250_7;
    sens->J[2][11] = JTcp4_250_8;
    sens->J[2][13] = JTcp4_250_9;
    sens->J[2][14] = JTcp4_250_10;
    sens->J[2][15] = JTcp4_250_11;
    sens->J[2][16] = JTcp4_250_12;
    sens->J[3][4] = (1.0);
    sens->J[3][6] = JTcp4_350_6;
    sens->J[3][7] = JTcp4_350_7;
    sens->J[3][11] = JTcp4_350_8;
    sens->J[3][13] = JTcp4_350_9;
    sens->J[3][14] = JTcp4_350_10;
    sens->J[3][15] = JTcp4_350_11;
    sens->J[3][16] = JTcp4_350_12;
    sens->J[4][6] = C1p5;
    sens->J[4][7] = ROcp4_46;
    sens->J[4][11] = ROcp4_46;
    sens->J[4][13] = ROcp4_111;
    sens->J[4][14] = ROcp4_111;
    sens->J[4][15] = ROcp4_414;
    sens->J[4][16] = ROcp4_715;
    sens->J[5][6] = S1p5;
    sens->J[5][7] = ROcp4_56;
    sens->J[5][11] = ROcp4_56;
    sens->J[5][13] = ROcp4_211;
    sens->J[5][14] = ROcp4_211;
    sens->J[5][15] = ROcp4_514;
    sens->J[5][16] = ROcp4_815;
    sens->J[6][1] = (1.0);
    sens->J[6][5] = (1.0);
    sens->J[6][7] = S6;
    sens->J[6][11] = S6;
    sens->J[6][13] = ROcp4_311;
    sens->J[6][14] = ROcp4_311;
    sens->J[6][15] = ROcp4_614;
    sens->J[6][16] = ROcp4_915;
    sens->A[1] = ACcp4_150;
    sens->A[2] = ACcp4_250;
    sens->A[3] = ACcp4_350;
    sens->OMP[1] = OPcp4_116;
    sens->OMP[2] = OPcp4_216;
    sens->OMP[3] = OPcp4_316;
 
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


    ROcp5_111 = ROcp5_17*C11-ROcp5_77*S11;
    ROcp5_211 = ROcp5_27*C11-ROcp5_87*S11;
    ROcp5_311 = -S11p7*C6;
    ROcp5_711 = ROcp5_17*S11+ROcp5_77*C11;
    ROcp5_811 = ROcp5_27*S11+ROcp5_87*C11;
    ROcp5_911 = C11p7*C6;
    RLcp5_111 = ROcp5_17*s->dpt[1][5]+ROcp5_77*s->dpt[3][5];
    RLcp5_211 = ROcp5_27*s->dpt[1][5]+ROcp5_87*s->dpt[3][5];
    RLcp5_311 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp5_111 = OMcp5_27*RLcp5_311-OMcp5_37*RLcp5_211;
    ORcp5_211 = -(OMcp5_17*RLcp5_311-OMcp5_37*RLcp5_111);
    ORcp5_311 = OMcp5_17*RLcp5_211-OMcp5_27*RLcp5_111;

// = = Block_1_0_0_6_0_5 = = 
 
// Sensor Kinematics 


    ROcp5_413 = ROcp5_46*C13+ROcp5_711*S13;
    ROcp5_513 = ROcp5_56*C13+ROcp5_811*S13;
    ROcp5_613 = ROcp5_911*S13+C13*S6;
    ROcp5_713 = -(ROcp5_46*S13-ROcp5_711*C13);
    ROcp5_813 = -(ROcp5_56*S13-ROcp5_811*C13);
    ROcp5_913 = ROcp5_911*C13-S13*S6;
    ROcp5_414 = ROcp5_413*C14+ROcp5_713*S14;
    ROcp5_514 = ROcp5_513*C14+ROcp5_813*S14;
    ROcp5_614 = ROcp5_613*C14+ROcp5_913*S14;
    ROcp5_714 = -(ROcp5_413*S14-ROcp5_713*C14);
    ROcp5_814 = -(ROcp5_513*S14-ROcp5_813*C14);
    ROcp5_914 = -(ROcp5_613*S14-ROcp5_913*C14);
    ROcp5_115 = ROcp5_111*C15-ROcp5_714*S15;
    ROcp5_215 = ROcp5_211*C15-ROcp5_814*S15;
    ROcp5_315 = ROcp5_311*C15-ROcp5_914*S15;
    ROcp5_715 = ROcp5_111*S15+ROcp5_714*C15;
    ROcp5_815 = ROcp5_211*S15+ROcp5_814*C15;
    ROcp5_915 = ROcp5_311*S15+ROcp5_914*C15;
    ROcp5_116 = ROcp5_115*C16+ROcp5_414*S16;
    ROcp5_216 = ROcp5_215*C16+ROcp5_514*S16;
    ROcp5_316 = ROcp5_315*C16+ROcp5_614*S16;
    ROcp5_416 = -(ROcp5_115*S16-ROcp5_414*C16);
    ROcp5_516 = -(ROcp5_215*S16-ROcp5_514*C16);
    ROcp5_616 = -(ROcp5_315*S16-ROcp5_614*C16);
    RLcp5_113 = ROcp5_46*s->dpt[2][12]+ROcp5_711*s->dpt[3][12];
    RLcp5_213 = ROcp5_56*s->dpt[2][12]+ROcp5_811*s->dpt[3][12];
    RLcp5_313 = ROcp5_911*s->dpt[3][12]+s->dpt[2][12]*S6;
    OMcp5_113 = OMcp5_17+ROcp5_111*qd[13];
    OMcp5_213 = OMcp5_27+ROcp5_211*qd[13];
    OMcp5_313 = OMcp5_37+ROcp5_311*qd[13];
    ORcp5_113 = OMcp5_27*RLcp5_313-OMcp5_37*RLcp5_213;
    ORcp5_213 = -(OMcp5_17*RLcp5_313-OMcp5_37*RLcp5_113);
    ORcp5_313 = OMcp5_17*RLcp5_213-OMcp5_27*RLcp5_113;
    OPcp5_113 = OPcp5_17+ROcp5_111*qdd[13]+qd[13]*(OMcp5_27*ROcp5_311-OMcp5_37*ROcp5_211);
    OPcp5_213 = OPcp5_27+ROcp5_211*qdd[13]-qd[13]*(OMcp5_17*ROcp5_311-OMcp5_37*ROcp5_111);
    OPcp5_313 = OPcp5_37+ROcp5_311*qdd[13]+qd[13]*(OMcp5_17*ROcp5_211-OMcp5_27*ROcp5_111);
    RLcp5_114 = ROcp5_413*s->dpt[2][17];
    RLcp5_214 = ROcp5_513*s->dpt[2][17];
    RLcp5_314 = ROcp5_613*s->dpt[2][17];
    OMcp5_114 = OMcp5_113+ROcp5_111*qd[14];
    OMcp5_214 = OMcp5_213+ROcp5_211*qd[14];
    OMcp5_314 = OMcp5_313+ROcp5_311*qd[14];
    ORcp5_114 = OMcp5_213*RLcp5_314-OMcp5_313*RLcp5_214;
    ORcp5_214 = -(OMcp5_113*RLcp5_314-OMcp5_313*RLcp5_114);
    ORcp5_314 = OMcp5_113*RLcp5_214-OMcp5_213*RLcp5_114;
    OMcp5_115 = OMcp5_114+ROcp5_414*qd[15];
    OMcp5_215 = OMcp5_214+ROcp5_514*qd[15];
    OMcp5_315 = OMcp5_314+ROcp5_614*qd[15];
    OMcp5_116 = OMcp5_115+ROcp5_715*qd[16];
    OMcp5_216 = OMcp5_215+ROcp5_815*qd[16];
    OMcp5_316 = OMcp5_315+ROcp5_915*qd[16];
    OPcp5_116 = OPcp5_113+ROcp5_111*qdd[14]+ROcp5_414*qdd[15]+ROcp5_715*qdd[16]+qd[14]*(OMcp5_213*ROcp5_311-OMcp5_313*
 ROcp5_211)+qd[15]*(OMcp5_214*ROcp5_614-OMcp5_314*ROcp5_514)+qd[16]*(OMcp5_215*ROcp5_915-OMcp5_315*ROcp5_815);
    OPcp5_216 = OPcp5_213+ROcp5_211*qdd[14]+ROcp5_514*qdd[15]+ROcp5_815*qdd[16]-qd[14]*(OMcp5_113*ROcp5_311-OMcp5_313*
 ROcp5_111)-qd[15]*(OMcp5_114*ROcp5_614-OMcp5_314*ROcp5_414)-qd[16]*(OMcp5_115*ROcp5_915-OMcp5_315*ROcp5_715);
    OPcp5_316 = OPcp5_313+ROcp5_311*qdd[14]+ROcp5_614*qdd[15]+ROcp5_915*qdd[16]+qd[14]*(OMcp5_113*ROcp5_211-OMcp5_213*
 ROcp5_111)+qd[15]*(OMcp5_114*ROcp5_514-OMcp5_214*ROcp5_414)+qd[16]*(OMcp5_115*ROcp5_815-OMcp5_215*ROcp5_715);
    RLcp5_151 = ROcp5_116*s->dpt[1][21]+ROcp5_416*s->dpt[2][21]+ROcp5_715*s->dpt[3][21];
    RLcp5_251 = ROcp5_216*s->dpt[1][21]+ROcp5_516*s->dpt[2][21]+ROcp5_815*s->dpt[3][21];
    RLcp5_351 = ROcp5_316*s->dpt[1][21]+ROcp5_616*s->dpt[2][21]+ROcp5_915*s->dpt[3][21];
    POcp5_151 = RLcp5_111+RLcp5_113+RLcp5_114+RLcp5_12+RLcp5_13+RLcp5_151;
    POcp5_251 = RLcp5_211+RLcp5_213+RLcp5_214+RLcp5_22+RLcp5_23+RLcp5_251;
    POcp5_351 = RLcp5_311+RLcp5_313+RLcp5_314+RLcp5_351+q[4];
    JTcp5_151_1 = -(RLcp5_211+RLcp5_213+RLcp5_214+RLcp5_22+RLcp5_23+RLcp5_251);
    JTcp5_251_1 = RLcp5_111+RLcp5_113+RLcp5_114+RLcp5_12+RLcp5_13+RLcp5_151;
    JTcp5_151_5 = -(RLcp5_211+RLcp5_213+RLcp5_214+RLcp5_251);
    JTcp5_251_5 = RLcp5_111+RLcp5_113+RLcp5_114+RLcp5_151;
    JTcp5_151_6 = S1p5*(RLcp5_311+RLcp5_313+RLcp5_314+RLcp5_351);
    JTcp5_251_6 = -C1p5*(RLcp5_311+RLcp5_313+RLcp5_314+RLcp5_351);
    JTcp5_351_6 = C1p5*(RLcp5_211+RLcp5_213+RLcp5_214+RLcp5_251)-S1p5*(RLcp5_111+RLcp5_113)-S1p5*(RLcp5_114+RLcp5_151);
    JTcp5_151_7 = ROcp5_56*(RLcp5_311+RLcp5_313+RLcp5_314+RLcp5_351)-S6*(RLcp5_211+RLcp5_213)-S6*(RLcp5_214+RLcp5_251);
    JTcp5_251_7 = -(ROcp5_46*(RLcp5_311+RLcp5_313+RLcp5_314+RLcp5_351)-S6*(RLcp5_111+RLcp5_113)-S6*(RLcp5_114+RLcp5_151));
    JTcp5_351_7 = ROcp5_46*(RLcp5_211+RLcp5_213+RLcp5_214+RLcp5_251)-ROcp5_56*(RLcp5_111+RLcp5_113)-ROcp5_56*(RLcp5_114+
 RLcp5_151);
    JTcp5_151_8 = ROcp5_56*(RLcp5_313+RLcp5_314)-S6*(RLcp5_213+RLcp5_214)-RLcp5_251*S6+RLcp5_351*ROcp5_56;
    JTcp5_251_8 = RLcp5_151*S6-RLcp5_351*ROcp5_46-ROcp5_46*(RLcp5_313+RLcp5_314)+S6*(RLcp5_113+RLcp5_114);
    JTcp5_351_8 = ROcp5_46*(RLcp5_213+RLcp5_214)-ROcp5_56*(RLcp5_113+RLcp5_114)-RLcp5_151*ROcp5_56+RLcp5_251*ROcp5_46;
    JTcp5_151_9 = ROcp5_211*(RLcp5_314+RLcp5_351)-ROcp5_311*(RLcp5_214+RLcp5_251);
    JTcp5_251_9 = -(ROcp5_111*(RLcp5_314+RLcp5_351)-ROcp5_311*(RLcp5_114+RLcp5_151));
    JTcp5_351_9 = ROcp5_111*(RLcp5_214+RLcp5_251)-ROcp5_211*(RLcp5_114+RLcp5_151);
    JTcp5_151_10 = -(RLcp5_251*ROcp5_311-RLcp5_351*ROcp5_211);
    JTcp5_251_10 = RLcp5_151*ROcp5_311-RLcp5_351*ROcp5_111;
    JTcp5_351_10 = -(RLcp5_151*ROcp5_211-RLcp5_251*ROcp5_111);
    JTcp5_151_11 = -(RLcp5_251*ROcp5_614-RLcp5_351*ROcp5_514);
    JTcp5_251_11 = RLcp5_151*ROcp5_614-RLcp5_351*ROcp5_414;
    JTcp5_351_11 = -(RLcp5_151*ROcp5_514-RLcp5_251*ROcp5_414);
    JTcp5_151_12 = -(RLcp5_251*ROcp5_915-RLcp5_351*ROcp5_815);
    JTcp5_251_12 = RLcp5_151*ROcp5_915-RLcp5_351*ROcp5_715;
    JTcp5_351_12 = -(RLcp5_151*ROcp5_815-RLcp5_251*ROcp5_715);
    ORcp5_151 = OMcp5_216*RLcp5_351-OMcp5_316*RLcp5_251;
    ORcp5_251 = -(OMcp5_116*RLcp5_351-OMcp5_316*RLcp5_151);
    ORcp5_351 = OMcp5_116*RLcp5_251-OMcp5_216*RLcp5_151;
    VIcp5_151 = ORcp5_111+ORcp5_113+ORcp5_114+ORcp5_12+ORcp5_13+ORcp5_151+qd[2]*C1-qd[3]*S1;
    VIcp5_251 = ORcp5_211+ORcp5_213+ORcp5_214+ORcp5_22+ORcp5_23+ORcp5_251+qd[2]*S1+qd[3]*C1;
    VIcp5_351 = ORcp5_311+ORcp5_313+ORcp5_314+ORcp5_351+qd[4];
    ACcp5_151 = OMcp5_213*ORcp5_314+OMcp5_216*ORcp5_351+OMcp5_27*(ORcp5_311+ORcp5_313)-OMcp5_313*ORcp5_214-OMcp5_316*
 ORcp5_251-OMcp5_37*ORcp5_211-OMcp5_37*ORcp5_213+OPcp5_213*RLcp5_314+OPcp5_216*RLcp5_351+OPcp5_27*RLcp5_311+OPcp5_27*
 RLcp5_313-OPcp5_313*RLcp5_214-OPcp5_316*RLcp5_251-OPcp5_37*RLcp5_211-OPcp5_37*RLcp5_213-ORcp5_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*
 qd[1]*qd[3]*C1-qd[1]*(ORcp5_22+(2.0)*qd[2]*S1);
    ACcp5_251 = -(OMcp5_113*ORcp5_314+OMcp5_116*ORcp5_351+OMcp5_17*(ORcp5_311+ORcp5_313)-OMcp5_313*ORcp5_114-OMcp5_316*
 ORcp5_151-OMcp5_37*ORcp5_111-OMcp5_37*ORcp5_113+OPcp5_113*RLcp5_314+OPcp5_116*RLcp5_351+OPcp5_17*RLcp5_311+OPcp5_17*
 RLcp5_313-OPcp5_313*RLcp5_114-OPcp5_316*RLcp5_151-OPcp5_37*RLcp5_111-OPcp5_37*RLcp5_113-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp5_12+(2.0)
 *qd[2]*C1)-qd[1]*(ORcp5_13-(2.0)*qd[3]*S1));
    ACcp5_351 = qdd[4]+OMcp5_113*ORcp5_214+OMcp5_116*ORcp5_251+OMcp5_17*ORcp5_211+OMcp5_17*ORcp5_213-OMcp5_213*ORcp5_114-
 OMcp5_216*ORcp5_151-OMcp5_27*ORcp5_111-OMcp5_27*ORcp5_113+OPcp5_113*RLcp5_214+OPcp5_116*RLcp5_251+OPcp5_17*RLcp5_211+
 OPcp5_17*RLcp5_213-OPcp5_213*RLcp5_114-OPcp5_216*RLcp5_151-OPcp5_27*RLcp5_111-OPcp5_27*RLcp5_113;

// = = Block_1_0_0_6_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp5_151;
    sens->P[2] = POcp5_251;
    sens->P[3] = POcp5_351;
    sens->R[1][1] = ROcp5_116;
    sens->R[1][2] = ROcp5_216;
    sens->R[1][3] = ROcp5_316;
    sens->R[2][1] = ROcp5_416;
    sens->R[2][2] = ROcp5_516;
    sens->R[2][3] = ROcp5_616;
    sens->R[3][1] = ROcp5_715;
    sens->R[3][2] = ROcp5_815;
    sens->R[3][3] = ROcp5_915;
    sens->V[1] = VIcp5_151;
    sens->V[2] = VIcp5_251;
    sens->V[3] = VIcp5_351;
    sens->OM[1] = OMcp5_116;
    sens->OM[2] = OMcp5_216;
    sens->OM[3] = OMcp5_316;
    sens->J[1][1] = JTcp5_151_1;
    sens->J[1][2] = C1;
    sens->J[1][3] = -S1;
    sens->J[1][5] = JTcp5_151_5;
    sens->J[1][6] = JTcp5_151_6;
    sens->J[1][7] = JTcp5_151_7;
    sens->J[1][11] = JTcp5_151_8;
    sens->J[1][13] = JTcp5_151_9;
    sens->J[1][14] = JTcp5_151_10;
    sens->J[1][15] = JTcp5_151_11;
    sens->J[1][16] = JTcp5_151_12;
    sens->J[2][1] = JTcp5_251_1;
    sens->J[2][2] = S1;
    sens->J[2][3] = C1;
    sens->J[2][5] = JTcp5_251_5;
    sens->J[2][6] = JTcp5_251_6;
    sens->J[2][7] = JTcp5_251_7;
    sens->J[2][11] = JTcp5_251_8;
    sens->J[2][13] = JTcp5_251_9;
    sens->J[2][14] = JTcp5_251_10;
    sens->J[2][15] = JTcp5_251_11;
    sens->J[2][16] = JTcp5_251_12;
    sens->J[3][4] = (1.0);
    sens->J[3][6] = JTcp5_351_6;
    sens->J[3][7] = JTcp5_351_7;
    sens->J[3][11] = JTcp5_351_8;
    sens->J[3][13] = JTcp5_351_9;
    sens->J[3][14] = JTcp5_351_10;
    sens->J[3][15] = JTcp5_351_11;
    sens->J[3][16] = JTcp5_351_12;
    sens->J[4][6] = C1p5;
    sens->J[4][7] = ROcp5_46;
    sens->J[4][11] = ROcp5_46;
    sens->J[4][13] = ROcp5_111;
    sens->J[4][14] = ROcp5_111;
    sens->J[4][15] = ROcp5_414;
    sens->J[4][16] = ROcp5_715;
    sens->J[5][6] = S1p5;
    sens->J[5][7] = ROcp5_56;
    sens->J[5][11] = ROcp5_56;
    sens->J[5][13] = ROcp5_211;
    sens->J[5][14] = ROcp5_211;
    sens->J[5][15] = ROcp5_514;
    sens->J[5][16] = ROcp5_815;
    sens->J[6][1] = (1.0);
    sens->J[6][5] = (1.0);
    sens->J[6][7] = S6;
    sens->J[6][11] = S6;
    sens->J[6][13] = ROcp5_311;
    sens->J[6][14] = ROcp5_311;
    sens->J[6][15] = ROcp5_614;
    sens->J[6][16] = ROcp5_915;
    sens->A[1] = ACcp5_151;
    sens->A[2] = ACcp5_251;
    sens->A[3] = ACcp5_351;
    sens->OMP[1] = OPcp5_116;
    sens->OMP[2] = OPcp5_216;
    sens->OMP[3] = OPcp5_316;
 
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


    ROcp6_111 = ROcp6_17*C11-ROcp6_77*S11;
    ROcp6_211 = ROcp6_27*C11-ROcp6_87*S11;
    ROcp6_311 = -S11p7*C6;
    ROcp6_711 = ROcp6_17*S11+ROcp6_77*C11;
    ROcp6_811 = ROcp6_27*S11+ROcp6_87*C11;
    ROcp6_911 = C11p7*C6;
    RLcp6_111 = ROcp6_17*s->dpt[1][5]+ROcp6_77*s->dpt[3][5];
    RLcp6_211 = ROcp6_27*s->dpt[1][5]+ROcp6_87*s->dpt[3][5];
    RLcp6_311 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp6_111 = OMcp6_27*RLcp6_311-OMcp6_37*RLcp6_211;
    ORcp6_211 = -(OMcp6_17*RLcp6_311-OMcp6_37*RLcp6_111);
    ORcp6_311 = OMcp6_17*RLcp6_211-OMcp6_27*RLcp6_111;

// = = Block_1_0_0_7_0_5 = = 
 
// Sensor Kinematics 


    ROcp6_413 = ROcp6_46*C13+ROcp6_711*S13;
    ROcp6_513 = ROcp6_56*C13+ROcp6_811*S13;
    ROcp6_613 = ROcp6_911*S13+C13*S6;
    ROcp6_713 = -(ROcp6_46*S13-ROcp6_711*C13);
    ROcp6_813 = -(ROcp6_56*S13-ROcp6_811*C13);
    ROcp6_913 = ROcp6_911*C13-S13*S6;
    ROcp6_414 = ROcp6_413*C14+ROcp6_713*S14;
    ROcp6_514 = ROcp6_513*C14+ROcp6_813*S14;
    ROcp6_614 = ROcp6_613*C14+ROcp6_913*S14;
    ROcp6_714 = -(ROcp6_413*S14-ROcp6_713*C14);
    ROcp6_814 = -(ROcp6_513*S14-ROcp6_813*C14);
    ROcp6_914 = -(ROcp6_613*S14-ROcp6_913*C14);
    ROcp6_115 = ROcp6_111*C15-ROcp6_714*S15;
    ROcp6_215 = ROcp6_211*C15-ROcp6_814*S15;
    ROcp6_315 = ROcp6_311*C15-ROcp6_914*S15;
    ROcp6_715 = ROcp6_111*S15+ROcp6_714*C15;
    ROcp6_815 = ROcp6_211*S15+ROcp6_814*C15;
    ROcp6_915 = ROcp6_311*S15+ROcp6_914*C15;
    ROcp6_116 = ROcp6_115*C16+ROcp6_414*S16;
    ROcp6_216 = ROcp6_215*C16+ROcp6_514*S16;
    ROcp6_316 = ROcp6_315*C16+ROcp6_614*S16;
    ROcp6_416 = -(ROcp6_115*S16-ROcp6_414*C16);
    ROcp6_516 = -(ROcp6_215*S16-ROcp6_514*C16);
    ROcp6_616 = -(ROcp6_315*S16-ROcp6_614*C16);
    ROcp6_417 = ROcp6_416*C17+ROcp6_715*S17;
    ROcp6_517 = ROcp6_516*C17+ROcp6_815*S17;
    ROcp6_617 = ROcp6_616*C17+ROcp6_915*S17;
    ROcp6_717 = -(ROcp6_416*S17-ROcp6_715*C17);
    ROcp6_817 = -(ROcp6_516*S17-ROcp6_815*C17);
    ROcp6_917 = -(ROcp6_616*S17-ROcp6_915*C17);
    ROcp6_118 = ROcp6_116*C18-ROcp6_717*S18;
    ROcp6_218 = ROcp6_216*C18-ROcp6_817*S18;
    ROcp6_318 = ROcp6_316*C18-ROcp6_917*S18;
    ROcp6_718 = ROcp6_116*S18+ROcp6_717*C18;
    ROcp6_818 = ROcp6_216*S18+ROcp6_817*C18;
    ROcp6_918 = ROcp6_316*S18+ROcp6_917*C18;
    RLcp6_113 = ROcp6_46*s->dpt[2][12]+ROcp6_711*s->dpt[3][12];
    RLcp6_213 = ROcp6_56*s->dpt[2][12]+ROcp6_811*s->dpt[3][12];
    RLcp6_313 = ROcp6_911*s->dpt[3][12]+s->dpt[2][12]*S6;
    OMcp6_113 = OMcp6_17+ROcp6_111*qd[13];
    OMcp6_213 = OMcp6_27+ROcp6_211*qd[13];
    OMcp6_313 = OMcp6_37+ROcp6_311*qd[13];
    ORcp6_113 = OMcp6_27*RLcp6_313-OMcp6_37*RLcp6_213;
    ORcp6_213 = -(OMcp6_17*RLcp6_313-OMcp6_37*RLcp6_113);
    ORcp6_313 = OMcp6_17*RLcp6_213-OMcp6_27*RLcp6_113;
    OPcp6_113 = OPcp6_17+ROcp6_111*qdd[13]+qd[13]*(OMcp6_27*ROcp6_311-OMcp6_37*ROcp6_211);
    OPcp6_213 = OPcp6_27+ROcp6_211*qdd[13]-qd[13]*(OMcp6_17*ROcp6_311-OMcp6_37*ROcp6_111);
    OPcp6_313 = OPcp6_37+ROcp6_311*qdd[13]+qd[13]*(OMcp6_17*ROcp6_211-OMcp6_27*ROcp6_111);
    RLcp6_114 = ROcp6_413*s->dpt[2][17];
    RLcp6_214 = ROcp6_513*s->dpt[2][17];
    RLcp6_314 = ROcp6_613*s->dpt[2][17];
    OMcp6_114 = OMcp6_113+ROcp6_111*qd[14];
    OMcp6_214 = OMcp6_213+ROcp6_211*qd[14];
    OMcp6_314 = OMcp6_313+ROcp6_311*qd[14];
    ORcp6_114 = OMcp6_213*RLcp6_314-OMcp6_313*RLcp6_214;
    ORcp6_214 = -(OMcp6_113*RLcp6_314-OMcp6_313*RLcp6_114);
    ORcp6_314 = OMcp6_113*RLcp6_214-OMcp6_213*RLcp6_114;
    OMcp6_115 = OMcp6_114+ROcp6_414*qd[15];
    OMcp6_215 = OMcp6_214+ROcp6_514*qd[15];
    OMcp6_315 = OMcp6_314+ROcp6_614*qd[15];
    OMcp6_116 = OMcp6_115+ROcp6_715*qd[16];
    OMcp6_216 = OMcp6_215+ROcp6_815*qd[16];
    OMcp6_316 = OMcp6_315+ROcp6_915*qd[16];
    OPcp6_116 = OPcp6_113+ROcp6_111*qdd[14]+ROcp6_414*qdd[15]+ROcp6_715*qdd[16]+qd[14]*(OMcp6_213*ROcp6_311-OMcp6_313*
 ROcp6_211)+qd[15]*(OMcp6_214*ROcp6_614-OMcp6_314*ROcp6_514)+qd[16]*(OMcp6_215*ROcp6_915-OMcp6_315*ROcp6_815);
    OPcp6_216 = OPcp6_213+ROcp6_211*qdd[14]+ROcp6_514*qdd[15]+ROcp6_815*qdd[16]-qd[14]*(OMcp6_113*ROcp6_311-OMcp6_313*
 ROcp6_111)-qd[15]*(OMcp6_114*ROcp6_614-OMcp6_314*ROcp6_414)-qd[16]*(OMcp6_115*ROcp6_915-OMcp6_315*ROcp6_715);
    OPcp6_316 = OPcp6_313+ROcp6_311*qdd[14]+ROcp6_614*qdd[15]+ROcp6_915*qdd[16]+qd[14]*(OMcp6_113*ROcp6_211-OMcp6_213*
 ROcp6_111)+qd[15]*(OMcp6_114*ROcp6_514-OMcp6_214*ROcp6_414)+qd[16]*(OMcp6_115*ROcp6_815-OMcp6_215*ROcp6_715);
    RLcp6_117 = ROcp6_116*s->dpt[1][21]+ROcp6_416*s->dpt[2][21]+ROcp6_715*s->dpt[3][21];
    RLcp6_217 = ROcp6_216*s->dpt[1][21]+ROcp6_516*s->dpt[2][21]+ROcp6_815*s->dpt[3][21];
    RLcp6_317 = ROcp6_316*s->dpt[1][21]+ROcp6_616*s->dpt[2][21]+ROcp6_915*s->dpt[3][21];
    POcp6_117 = RLcp6_111+RLcp6_113+RLcp6_114+RLcp6_117+RLcp6_12+RLcp6_13;
    POcp6_217 = RLcp6_211+RLcp6_213+RLcp6_214+RLcp6_217+RLcp6_22+RLcp6_23;
    POcp6_317 = RLcp6_311+RLcp6_313+RLcp6_314+RLcp6_317+q[4];
    JTcp6_117_1 = -(RLcp6_211+RLcp6_213+RLcp6_214+RLcp6_217+RLcp6_22+RLcp6_23);
    JTcp6_217_1 = RLcp6_111+RLcp6_113+RLcp6_114+RLcp6_117+RLcp6_12+RLcp6_13;
    JTcp6_117_5 = -(RLcp6_211+RLcp6_213+RLcp6_214+RLcp6_217);
    JTcp6_217_5 = RLcp6_111+RLcp6_113+RLcp6_114+RLcp6_117;
    JTcp6_117_6 = S1p5*(RLcp6_311+RLcp6_313+RLcp6_314+RLcp6_317);
    JTcp6_217_6 = -C1p5*(RLcp6_311+RLcp6_313+RLcp6_314+RLcp6_317);
    JTcp6_317_6 = C1p5*(RLcp6_211+RLcp6_213+RLcp6_214+RLcp6_217)-S1p5*(RLcp6_111+RLcp6_113)-S1p5*(RLcp6_114+RLcp6_117);
    JTcp6_117_7 = ROcp6_56*(RLcp6_311+RLcp6_313+RLcp6_314+RLcp6_317)-S6*(RLcp6_211+RLcp6_213)-S6*(RLcp6_214+RLcp6_217);
    JTcp6_217_7 = -(ROcp6_46*(RLcp6_311+RLcp6_313+RLcp6_314+RLcp6_317)-S6*(RLcp6_111+RLcp6_113)-S6*(RLcp6_114+RLcp6_117));
    JTcp6_317_7 = ROcp6_46*(RLcp6_211+RLcp6_213+RLcp6_214+RLcp6_217)-ROcp6_56*(RLcp6_111+RLcp6_113)-ROcp6_56*(RLcp6_114+
 RLcp6_117);
    JTcp6_117_8 = ROcp6_56*(RLcp6_313+RLcp6_314)-S6*(RLcp6_213+RLcp6_214)-RLcp6_217*S6+RLcp6_317*ROcp6_56;
    JTcp6_217_8 = RLcp6_117*S6-RLcp6_317*ROcp6_46-ROcp6_46*(RLcp6_313+RLcp6_314)+S6*(RLcp6_113+RLcp6_114);
    JTcp6_317_8 = ROcp6_46*(RLcp6_213+RLcp6_214)-ROcp6_56*(RLcp6_113+RLcp6_114)-RLcp6_117*ROcp6_56+RLcp6_217*ROcp6_46;
    JTcp6_117_9 = ROcp6_211*(RLcp6_314+RLcp6_317)-ROcp6_311*(RLcp6_214+RLcp6_217);
    JTcp6_217_9 = -(ROcp6_111*(RLcp6_314+RLcp6_317)-ROcp6_311*(RLcp6_114+RLcp6_117));
    JTcp6_317_9 = ROcp6_111*(RLcp6_214+RLcp6_217)-ROcp6_211*(RLcp6_114+RLcp6_117);
    JTcp6_117_10 = -(RLcp6_217*ROcp6_311-RLcp6_317*ROcp6_211);
    JTcp6_217_10 = RLcp6_117*ROcp6_311-RLcp6_317*ROcp6_111;
    JTcp6_317_10 = -(RLcp6_117*ROcp6_211-RLcp6_217*ROcp6_111);
    JTcp6_117_11 = -(RLcp6_217*ROcp6_614-RLcp6_317*ROcp6_514);
    JTcp6_217_11 = RLcp6_117*ROcp6_614-RLcp6_317*ROcp6_414;
    JTcp6_317_11 = -(RLcp6_117*ROcp6_514-RLcp6_217*ROcp6_414);
    JTcp6_117_12 = -(RLcp6_217*ROcp6_915-RLcp6_317*ROcp6_815);
    JTcp6_217_12 = RLcp6_117*ROcp6_915-RLcp6_317*ROcp6_715;
    JTcp6_317_12 = -(RLcp6_117*ROcp6_815-RLcp6_217*ROcp6_715);
    ORcp6_117 = OMcp6_216*RLcp6_317-OMcp6_316*RLcp6_217;
    ORcp6_217 = -(OMcp6_116*RLcp6_317-OMcp6_316*RLcp6_117);
    ORcp6_317 = OMcp6_116*RLcp6_217-OMcp6_216*RLcp6_117;
    VIcp6_117 = ORcp6_111+ORcp6_113+ORcp6_114+ORcp6_117+ORcp6_12+ORcp6_13+qd[2]*C1-qd[3]*S1;
    VIcp6_217 = ORcp6_211+ORcp6_213+ORcp6_214+ORcp6_217+ORcp6_22+ORcp6_23+qd[2]*S1+qd[3]*C1;
    VIcp6_317 = ORcp6_311+ORcp6_313+ORcp6_314+ORcp6_317+qd[4];
    ACcp6_117 = OMcp6_213*ORcp6_314+OMcp6_216*ORcp6_317+OMcp6_27*(ORcp6_311+ORcp6_313)-OMcp6_313*ORcp6_214-OMcp6_316*
 ORcp6_217-OMcp6_37*ORcp6_211-OMcp6_37*ORcp6_213+OPcp6_213*RLcp6_314+OPcp6_216*RLcp6_317+OPcp6_27*RLcp6_311+OPcp6_27*
 RLcp6_313-OPcp6_313*RLcp6_214-OPcp6_316*RLcp6_217-OPcp6_37*RLcp6_211-OPcp6_37*RLcp6_213-ORcp6_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*
 qd[1]*qd[3]*C1-qd[1]*(ORcp6_22+(2.0)*qd[2]*S1);
    ACcp6_217 = -(OMcp6_113*ORcp6_314+OMcp6_116*ORcp6_317+OMcp6_17*(ORcp6_311+ORcp6_313)-OMcp6_313*ORcp6_114-OMcp6_316*
 ORcp6_117-OMcp6_37*ORcp6_111-OMcp6_37*ORcp6_113+OPcp6_113*RLcp6_314+OPcp6_116*RLcp6_317+OPcp6_17*RLcp6_311+OPcp6_17*
 RLcp6_313-OPcp6_313*RLcp6_114-OPcp6_316*RLcp6_117-OPcp6_37*RLcp6_111-OPcp6_37*RLcp6_113-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp6_12+(2.0)
 *qd[2]*C1)-qd[1]*(ORcp6_13-(2.0)*qd[3]*S1));
    ACcp6_317 = qdd[4]+OMcp6_113*ORcp6_214+OMcp6_116*ORcp6_217+OMcp6_17*ORcp6_211+OMcp6_17*ORcp6_213-OMcp6_213*ORcp6_114-
 OMcp6_216*ORcp6_117-OMcp6_27*ORcp6_111-OMcp6_27*ORcp6_113+OPcp6_113*RLcp6_214+OPcp6_116*RLcp6_217+OPcp6_17*RLcp6_211+
 OPcp6_17*RLcp6_213-OPcp6_213*RLcp6_114-OPcp6_216*RLcp6_117-OPcp6_27*RLcp6_111-OPcp6_27*RLcp6_113;
    OMcp6_118 = OMcp6_116+ROcp6_417*qd[18];
    OMcp6_218 = OMcp6_216+ROcp6_517*qd[18];
    OMcp6_318 = OMcp6_316+ROcp6_617*qd[18];
    OPcp6_118 = OPcp6_116+ROcp6_417*qdd[18]+qd[18]*(OMcp6_216*ROcp6_617-OMcp6_316*ROcp6_517);
    OPcp6_218 = OPcp6_216+ROcp6_517*qdd[18]-qd[18]*(OMcp6_116*ROcp6_617-OMcp6_316*ROcp6_417);
    OPcp6_318 = OPcp6_316+ROcp6_617*qdd[18]+qd[18]*(OMcp6_116*ROcp6_517-OMcp6_216*ROcp6_417);

// = = Block_1_0_0_7_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp6_117;
    sens->P[2] = POcp6_217;
    sens->P[3] = POcp6_317;
    sens->R[1][1] = ROcp6_118;
    sens->R[1][2] = ROcp6_218;
    sens->R[1][3] = ROcp6_318;
    sens->R[2][1] = ROcp6_417;
    sens->R[2][2] = ROcp6_517;
    sens->R[2][3] = ROcp6_617;
    sens->R[3][1] = ROcp6_718;
    sens->R[3][2] = ROcp6_818;
    sens->R[3][3] = ROcp6_918;
    sens->V[1] = VIcp6_117;
    sens->V[2] = VIcp6_217;
    sens->V[3] = VIcp6_317;
    sens->OM[1] = OMcp6_118;
    sens->OM[2] = OMcp6_218;
    sens->OM[3] = OMcp6_318;
    sens->J[1][1] = JTcp6_117_1;
    sens->J[1][2] = C1;
    sens->J[1][3] = -S1;
    sens->J[1][5] = JTcp6_117_5;
    sens->J[1][6] = JTcp6_117_6;
    sens->J[1][7] = JTcp6_117_7;
    sens->J[1][11] = JTcp6_117_8;
    sens->J[1][13] = JTcp6_117_9;
    sens->J[1][14] = JTcp6_117_10;
    sens->J[1][15] = JTcp6_117_11;
    sens->J[1][16] = JTcp6_117_12;
    sens->J[2][1] = JTcp6_217_1;
    sens->J[2][2] = S1;
    sens->J[2][3] = C1;
    sens->J[2][5] = JTcp6_217_5;
    sens->J[2][6] = JTcp6_217_6;
    sens->J[2][7] = JTcp6_217_7;
    sens->J[2][11] = JTcp6_217_8;
    sens->J[2][13] = JTcp6_217_9;
    sens->J[2][14] = JTcp6_217_10;
    sens->J[2][15] = JTcp6_217_11;
    sens->J[2][16] = JTcp6_217_12;
    sens->J[3][4] = (1.0);
    sens->J[3][6] = JTcp6_317_6;
    sens->J[3][7] = JTcp6_317_7;
    sens->J[3][11] = JTcp6_317_8;
    sens->J[3][13] = JTcp6_317_9;
    sens->J[3][14] = JTcp6_317_10;
    sens->J[3][15] = JTcp6_317_11;
    sens->J[3][16] = JTcp6_317_12;
    sens->J[4][6] = C1p5;
    sens->J[4][7] = ROcp6_46;
    sens->J[4][11] = ROcp6_46;
    sens->J[4][13] = ROcp6_111;
    sens->J[4][14] = ROcp6_111;
    sens->J[4][15] = ROcp6_414;
    sens->J[4][16] = ROcp6_715;
    sens->J[4][17] = ROcp6_116;
    sens->J[4][18] = ROcp6_417;
    sens->J[5][6] = S1p5;
    sens->J[5][7] = ROcp6_56;
    sens->J[5][11] = ROcp6_56;
    sens->J[5][13] = ROcp6_211;
    sens->J[5][14] = ROcp6_211;
    sens->J[5][15] = ROcp6_514;
    sens->J[5][16] = ROcp6_815;
    sens->J[5][17] = ROcp6_216;
    sens->J[5][18] = ROcp6_517;
    sens->J[6][1] = (1.0);
    sens->J[6][5] = (1.0);
    sens->J[6][7] = S6;
    sens->J[6][11] = S6;
    sens->J[6][13] = ROcp6_311;
    sens->J[6][14] = ROcp6_311;
    sens->J[6][15] = ROcp6_614;
    sens->J[6][16] = ROcp6_915;
    sens->J[6][17] = ROcp6_316;
    sens->J[6][18] = ROcp6_617;
    sens->A[1] = ACcp6_117;
    sens->A[2] = ACcp6_217;
    sens->A[3] = ACcp6_317;
    sens->OMP[1] = OPcp6_118;
    sens->OMP[2] = OPcp6_218;
    sens->OMP[3] = OPcp6_318;
 
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


    ROcp7_111 = ROcp7_17*C11-ROcp7_77*S11;
    ROcp7_211 = ROcp7_27*C11-ROcp7_87*S11;
    ROcp7_311 = -S11p7*C6;
    ROcp7_711 = ROcp7_17*S11+ROcp7_77*C11;
    ROcp7_811 = ROcp7_27*S11+ROcp7_87*C11;
    ROcp7_911 = C11p7*C6;
    RLcp7_111 = ROcp7_17*s->dpt[1][5]+ROcp7_77*s->dpt[3][5];
    RLcp7_211 = ROcp7_27*s->dpt[1][5]+ROcp7_87*s->dpt[3][5];
    RLcp7_311 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp7_111 = OMcp7_27*RLcp7_311-OMcp7_37*RLcp7_211;
    ORcp7_211 = -(OMcp7_17*RLcp7_311-OMcp7_37*RLcp7_111);
    ORcp7_311 = OMcp7_17*RLcp7_211-OMcp7_27*RLcp7_111;

// = = Block_1_0_0_8_0_6 = = 
 
// Sensor Kinematics 


    ROcp7_419 = ROcp7_46*C19+ROcp7_711*S19;
    ROcp7_519 = ROcp7_56*C19+ROcp7_811*S19;
    ROcp7_619 = ROcp7_911*S19+C19*S6;
    ROcp7_719 = -(ROcp7_46*S19-ROcp7_711*C19);
    ROcp7_819 = -(ROcp7_56*S19-ROcp7_811*C19);
    ROcp7_919 = ROcp7_911*C19-S19*S6;
    ROcp7_420 = ROcp7_419*C20+ROcp7_719*S20;
    ROcp7_520 = ROcp7_519*C20+ROcp7_819*S20;
    ROcp7_620 = ROcp7_619*C20+ROcp7_919*S20;
    ROcp7_720 = -(ROcp7_419*S20-ROcp7_719*C20);
    ROcp7_820 = -(ROcp7_519*S20-ROcp7_819*C20);
    ROcp7_920 = -(ROcp7_619*S20-ROcp7_919*C20);
    ROcp7_121 = ROcp7_111*C21-ROcp7_720*S21;
    ROcp7_221 = ROcp7_211*C21-ROcp7_820*S21;
    ROcp7_321 = ROcp7_311*C21-ROcp7_920*S21;
    ROcp7_721 = ROcp7_111*S21+ROcp7_720*C21;
    ROcp7_821 = ROcp7_211*S21+ROcp7_820*C21;
    ROcp7_921 = ROcp7_311*S21+ROcp7_920*C21;
    ROcp7_122 = ROcp7_121*C22+ROcp7_420*S22;
    ROcp7_222 = ROcp7_221*C22+ROcp7_520*S22;
    ROcp7_322 = ROcp7_321*C22+ROcp7_620*S22;
    ROcp7_422 = -(ROcp7_121*S22-ROcp7_420*C22);
    ROcp7_522 = -(ROcp7_221*S22-ROcp7_520*C22);
    ROcp7_622 = -(ROcp7_321*S22-ROcp7_620*C22);
    RLcp7_119 = ROcp7_46*s->dpt[2][13]+ROcp7_711*s->dpt[3][13];
    RLcp7_219 = ROcp7_56*s->dpt[2][13]+ROcp7_811*s->dpt[3][13];
    RLcp7_319 = ROcp7_911*s->dpt[3][13]+s->dpt[2][13]*S6;
    OMcp7_119 = OMcp7_17+ROcp7_111*qd[19];
    OMcp7_219 = OMcp7_27+ROcp7_211*qd[19];
    OMcp7_319 = OMcp7_37+ROcp7_311*qd[19];
    ORcp7_119 = OMcp7_27*RLcp7_319-OMcp7_37*RLcp7_219;
    ORcp7_219 = -(OMcp7_17*RLcp7_319-OMcp7_37*RLcp7_119);
    ORcp7_319 = OMcp7_17*RLcp7_219-OMcp7_27*RLcp7_119;
    OPcp7_119 = OPcp7_17+ROcp7_111*qdd[19]+qd[19]*(OMcp7_27*ROcp7_311-OMcp7_37*ROcp7_211);
    OPcp7_219 = OPcp7_27+ROcp7_211*qdd[19]-qd[19]*(OMcp7_17*ROcp7_311-OMcp7_37*ROcp7_111);
    OPcp7_319 = OPcp7_37+ROcp7_311*qdd[19]+qd[19]*(OMcp7_17*ROcp7_211-OMcp7_27*ROcp7_111);
    RLcp7_120 = ROcp7_419*s->dpt[2][24];
    RLcp7_220 = ROcp7_519*s->dpt[2][24];
    RLcp7_320 = ROcp7_619*s->dpt[2][24];
    POcp7_120 = RLcp7_111+RLcp7_119+RLcp7_12+RLcp7_120+RLcp7_13;
    POcp7_220 = RLcp7_211+RLcp7_219+RLcp7_22+RLcp7_220+RLcp7_23;
    POcp7_320 = RLcp7_311+RLcp7_319+RLcp7_320+q[4];
    JTcp7_120_1 = -(RLcp7_211+RLcp7_219+RLcp7_22+RLcp7_220+RLcp7_23);
    JTcp7_220_1 = RLcp7_111+RLcp7_119+RLcp7_12+RLcp7_120+RLcp7_13;
    JTcp7_120_5 = -(RLcp7_211+RLcp7_219+RLcp7_220);
    JTcp7_220_5 = RLcp7_111+RLcp7_119+RLcp7_120;
    JTcp7_120_6 = S1p5*(RLcp7_311+RLcp7_319+RLcp7_320);
    JTcp7_220_6 = -C1p5*(RLcp7_311+RLcp7_319+RLcp7_320);
    JTcp7_320_6 = C1p5*(RLcp7_211+RLcp7_219)-S1p5*(RLcp7_111+RLcp7_119)-RLcp7_120*S1p5+RLcp7_220*C1p5;
    JTcp7_120_7 = ROcp7_56*(RLcp7_311+RLcp7_319)-S6*(RLcp7_211+RLcp7_219)-RLcp7_220*S6+RLcp7_320*ROcp7_56;
    JTcp7_220_7 = RLcp7_120*S6-RLcp7_320*ROcp7_46-ROcp7_46*(RLcp7_311+RLcp7_319)+S6*(RLcp7_111+RLcp7_119);
    JTcp7_320_7 = ROcp7_46*(RLcp7_211+RLcp7_219)-ROcp7_56*(RLcp7_111+RLcp7_119)-RLcp7_120*ROcp7_56+RLcp7_220*ROcp7_46;
    JTcp7_120_8 = ROcp7_56*(RLcp7_319+RLcp7_320)-S6*(RLcp7_219+RLcp7_220);
    JTcp7_220_8 = -(ROcp7_46*(RLcp7_319+RLcp7_320)-S6*(RLcp7_119+RLcp7_120));
    JTcp7_320_8 = ROcp7_46*(RLcp7_219+RLcp7_220)-ROcp7_56*(RLcp7_119+RLcp7_120);
    JTcp7_120_9 = -(RLcp7_220*ROcp7_311-RLcp7_320*ROcp7_211);
    JTcp7_220_9 = RLcp7_120*ROcp7_311-RLcp7_320*ROcp7_111;
    JTcp7_320_9 = -(RLcp7_120*ROcp7_211-RLcp7_220*ROcp7_111);
    OMcp7_120 = OMcp7_119+ROcp7_111*qd[20];
    OMcp7_220 = OMcp7_219+ROcp7_211*qd[20];
    OMcp7_320 = OMcp7_319+ROcp7_311*qd[20];
    ORcp7_120 = OMcp7_219*RLcp7_320-OMcp7_319*RLcp7_220;
    ORcp7_220 = -(OMcp7_119*RLcp7_320-OMcp7_319*RLcp7_120);
    ORcp7_320 = OMcp7_119*RLcp7_220-OMcp7_219*RLcp7_120;
    VIcp7_120 = ORcp7_111+ORcp7_119+ORcp7_12+ORcp7_120+ORcp7_13+qd[2]*C1-qd[3]*S1;
    VIcp7_220 = ORcp7_211+ORcp7_219+ORcp7_22+ORcp7_220+ORcp7_23+qd[2]*S1+qd[3]*C1;
    VIcp7_320 = ORcp7_311+ORcp7_319+ORcp7_320+qd[4];
    ACcp7_120 = OMcp7_219*ORcp7_320+OMcp7_27*(ORcp7_311+ORcp7_319)-OMcp7_319*ORcp7_220-OMcp7_37*ORcp7_211-OMcp7_37*
 ORcp7_219+OPcp7_219*RLcp7_320+OPcp7_27*RLcp7_311+OPcp7_27*RLcp7_319-OPcp7_319*RLcp7_220-OPcp7_37*RLcp7_211-OPcp7_37*
 RLcp7_219-ORcp7_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp7_22+(2.0)*qd[2]*S1);
    ACcp7_220 = -(OMcp7_119*ORcp7_320+OMcp7_17*(ORcp7_311+ORcp7_319)-OMcp7_319*ORcp7_120-OMcp7_37*ORcp7_111-OMcp7_37*
 ORcp7_119+OPcp7_119*RLcp7_320+OPcp7_17*RLcp7_311+OPcp7_17*RLcp7_319-OPcp7_319*RLcp7_120-OPcp7_37*RLcp7_111-OPcp7_37*
 RLcp7_119-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp7_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp7_13-(2.0)*qd[3]*S1));
    ACcp7_320 = qdd[4]+OMcp7_119*ORcp7_220+OMcp7_17*ORcp7_211+OMcp7_17*ORcp7_219-OMcp7_219*ORcp7_120-OMcp7_27*ORcp7_111-
 OMcp7_27*ORcp7_119+OPcp7_119*RLcp7_220+OPcp7_17*RLcp7_211+OPcp7_17*RLcp7_219-OPcp7_219*RLcp7_120-OPcp7_27*RLcp7_111-OPcp7_27
 *RLcp7_119;
    OMcp7_121 = OMcp7_120+ROcp7_420*qd[21];
    OMcp7_221 = OMcp7_220+ROcp7_520*qd[21];
    OMcp7_321 = OMcp7_320+ROcp7_620*qd[21];
    OMcp7_122 = OMcp7_121+ROcp7_721*qd[22];
    OMcp7_222 = OMcp7_221+ROcp7_821*qd[22];
    OMcp7_322 = OMcp7_321+ROcp7_921*qd[22];
    OPcp7_122 = OPcp7_119+ROcp7_111*qdd[20]+ROcp7_420*qdd[21]+ROcp7_721*qdd[22]+qd[20]*(OMcp7_219*ROcp7_311-OMcp7_319*
 ROcp7_211)+qd[21]*(OMcp7_220*ROcp7_620-OMcp7_320*ROcp7_520)+qd[22]*(OMcp7_221*ROcp7_921-OMcp7_321*ROcp7_821);
    OPcp7_222 = OPcp7_219+ROcp7_211*qdd[20]+ROcp7_520*qdd[21]+ROcp7_821*qdd[22]-qd[20]*(OMcp7_119*ROcp7_311-OMcp7_319*
 ROcp7_111)-qd[21]*(OMcp7_120*ROcp7_620-OMcp7_320*ROcp7_420)-qd[22]*(OMcp7_121*ROcp7_921-OMcp7_321*ROcp7_721);
    OPcp7_322 = OPcp7_319+ROcp7_311*qdd[20]+ROcp7_620*qdd[21]+ROcp7_921*qdd[22]+qd[20]*(OMcp7_119*ROcp7_211-OMcp7_219*
 ROcp7_111)+qd[21]*(OMcp7_120*ROcp7_520-OMcp7_220*ROcp7_420)+qd[22]*(OMcp7_121*ROcp7_821-OMcp7_221*ROcp7_721);

// = = Block_1_0_0_8_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp7_120;
    sens->P[2] = POcp7_220;
    sens->P[3] = POcp7_320;
    sens->R[1][1] = ROcp7_122;
    sens->R[1][2] = ROcp7_222;
    sens->R[1][3] = ROcp7_322;
    sens->R[2][1] = ROcp7_422;
    sens->R[2][2] = ROcp7_522;
    sens->R[2][3] = ROcp7_622;
    sens->R[3][1] = ROcp7_721;
    sens->R[3][2] = ROcp7_821;
    sens->R[3][3] = ROcp7_921;
    sens->V[1] = VIcp7_120;
    sens->V[2] = VIcp7_220;
    sens->V[3] = VIcp7_320;
    sens->OM[1] = OMcp7_122;
    sens->OM[2] = OMcp7_222;
    sens->OM[3] = OMcp7_322;
    sens->J[1][1] = JTcp7_120_1;
    sens->J[1][2] = C1;
    sens->J[1][3] = -S1;
    sens->J[1][5] = JTcp7_120_5;
    sens->J[1][6] = JTcp7_120_6;
    sens->J[1][7] = JTcp7_120_7;
    sens->J[1][11] = JTcp7_120_8;
    sens->J[1][19] = JTcp7_120_9;
    sens->J[2][1] = JTcp7_220_1;
    sens->J[2][2] = S1;
    sens->J[2][3] = C1;
    sens->J[2][5] = JTcp7_220_5;
    sens->J[2][6] = JTcp7_220_6;
    sens->J[2][7] = JTcp7_220_7;
    sens->J[2][11] = JTcp7_220_8;
    sens->J[2][19] = JTcp7_220_9;
    sens->J[3][4] = (1.0);
    sens->J[3][6] = JTcp7_320_6;
    sens->J[3][7] = JTcp7_320_7;
    sens->J[3][11] = JTcp7_320_8;
    sens->J[3][19] = JTcp7_320_9;
    sens->J[4][6] = C1p5;
    sens->J[4][7] = ROcp7_46;
    sens->J[4][11] = ROcp7_46;
    sens->J[4][19] = ROcp7_111;
    sens->J[4][20] = ROcp7_111;
    sens->J[4][21] = ROcp7_420;
    sens->J[4][22] = ROcp7_721;
    sens->J[5][6] = S1p5;
    sens->J[5][7] = ROcp7_56;
    sens->J[5][11] = ROcp7_56;
    sens->J[5][19] = ROcp7_211;
    sens->J[5][20] = ROcp7_211;
    sens->J[5][21] = ROcp7_520;
    sens->J[5][22] = ROcp7_821;
    sens->J[6][1] = (1.0);
    sens->J[6][5] = (1.0);
    sens->J[6][7] = S6;
    sens->J[6][11] = S6;
    sens->J[6][19] = ROcp7_311;
    sens->J[6][20] = ROcp7_311;
    sens->J[6][21] = ROcp7_620;
    sens->J[6][22] = ROcp7_921;
    sens->A[1] = ACcp7_120;
    sens->A[2] = ACcp7_220;
    sens->A[3] = ACcp7_320;
    sens->OMP[1] = OPcp7_122;
    sens->OMP[2] = OPcp7_222;
    sens->OMP[3] = OPcp7_322;
 
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


    ROcp8_111 = ROcp8_17*C11-ROcp8_77*S11;
    ROcp8_211 = ROcp8_27*C11-ROcp8_87*S11;
    ROcp8_311 = -S11p7*C6;
    ROcp8_711 = ROcp8_17*S11+ROcp8_77*C11;
    ROcp8_811 = ROcp8_27*S11+ROcp8_87*C11;
    ROcp8_911 = C11p7*C6;
    RLcp8_111 = ROcp8_17*s->dpt[1][5]+ROcp8_77*s->dpt[3][5];
    RLcp8_211 = ROcp8_27*s->dpt[1][5]+ROcp8_87*s->dpt[3][5];
    RLcp8_311 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp8_111 = OMcp8_27*RLcp8_311-OMcp8_37*RLcp8_211;
    ORcp8_211 = -(OMcp8_17*RLcp8_311-OMcp8_37*RLcp8_111);
    ORcp8_311 = OMcp8_17*RLcp8_211-OMcp8_27*RLcp8_111;

// = = Block_1_0_0_9_0_6 = = 
 
// Sensor Kinematics 


    ROcp8_419 = ROcp8_46*C19+ROcp8_711*S19;
    ROcp8_519 = ROcp8_56*C19+ROcp8_811*S19;
    ROcp8_619 = ROcp8_911*S19+C19*S6;
    ROcp8_719 = -(ROcp8_46*S19-ROcp8_711*C19);
    ROcp8_819 = -(ROcp8_56*S19-ROcp8_811*C19);
    ROcp8_919 = ROcp8_911*C19-S19*S6;
    ROcp8_420 = ROcp8_419*C20+ROcp8_719*S20;
    ROcp8_520 = ROcp8_519*C20+ROcp8_819*S20;
    ROcp8_620 = ROcp8_619*C20+ROcp8_919*S20;
    ROcp8_720 = -(ROcp8_419*S20-ROcp8_719*C20);
    ROcp8_820 = -(ROcp8_519*S20-ROcp8_819*C20);
    ROcp8_920 = -(ROcp8_619*S20-ROcp8_919*C20);
    ROcp8_121 = ROcp8_111*C21-ROcp8_720*S21;
    ROcp8_221 = ROcp8_211*C21-ROcp8_820*S21;
    ROcp8_321 = ROcp8_311*C21-ROcp8_920*S21;
    ROcp8_721 = ROcp8_111*S21+ROcp8_720*C21;
    ROcp8_821 = ROcp8_211*S21+ROcp8_820*C21;
    ROcp8_921 = ROcp8_311*S21+ROcp8_920*C21;
    ROcp8_122 = ROcp8_121*C22+ROcp8_420*S22;
    ROcp8_222 = ROcp8_221*C22+ROcp8_520*S22;
    ROcp8_322 = ROcp8_321*C22+ROcp8_620*S22;
    ROcp8_422 = -(ROcp8_121*S22-ROcp8_420*C22);
    ROcp8_522 = -(ROcp8_221*S22-ROcp8_520*C22);
    ROcp8_622 = -(ROcp8_321*S22-ROcp8_620*C22);
    RLcp8_119 = ROcp8_46*s->dpt[2][13]+ROcp8_711*s->dpt[3][13];
    RLcp8_219 = ROcp8_56*s->dpt[2][13]+ROcp8_811*s->dpt[3][13];
    RLcp8_319 = ROcp8_911*s->dpt[3][13]+s->dpt[2][13]*S6;
    OMcp8_119 = OMcp8_17+ROcp8_111*qd[19];
    OMcp8_219 = OMcp8_27+ROcp8_211*qd[19];
    OMcp8_319 = OMcp8_37+ROcp8_311*qd[19];
    ORcp8_119 = OMcp8_27*RLcp8_319-OMcp8_37*RLcp8_219;
    ORcp8_219 = -(OMcp8_17*RLcp8_319-OMcp8_37*RLcp8_119);
    ORcp8_319 = OMcp8_17*RLcp8_219-OMcp8_27*RLcp8_119;
    OPcp8_119 = OPcp8_17+ROcp8_111*qdd[19]+qd[19]*(OMcp8_27*ROcp8_311-OMcp8_37*ROcp8_211);
    OPcp8_219 = OPcp8_27+ROcp8_211*qdd[19]-qd[19]*(OMcp8_17*ROcp8_311-OMcp8_37*ROcp8_111);
    OPcp8_319 = OPcp8_37+ROcp8_311*qdd[19]+qd[19]*(OMcp8_17*ROcp8_211-OMcp8_27*ROcp8_111);
    RLcp8_120 = ROcp8_419*s->dpt[2][24];
    RLcp8_220 = ROcp8_519*s->dpt[2][24];
    RLcp8_320 = ROcp8_619*s->dpt[2][24];
    OMcp8_120 = OMcp8_119+ROcp8_111*qd[20];
    OMcp8_220 = OMcp8_219+ROcp8_211*qd[20];
    OMcp8_320 = OMcp8_319+ROcp8_311*qd[20];
    ORcp8_120 = OMcp8_219*RLcp8_320-OMcp8_319*RLcp8_220;
    ORcp8_220 = -(OMcp8_119*RLcp8_320-OMcp8_319*RLcp8_120);
    ORcp8_320 = OMcp8_119*RLcp8_220-OMcp8_219*RLcp8_120;
    OMcp8_121 = OMcp8_120+ROcp8_420*qd[21];
    OMcp8_221 = OMcp8_220+ROcp8_520*qd[21];
    OMcp8_321 = OMcp8_320+ROcp8_620*qd[21];
    OMcp8_122 = OMcp8_121+ROcp8_721*qd[22];
    OMcp8_222 = OMcp8_221+ROcp8_821*qd[22];
    OMcp8_322 = OMcp8_321+ROcp8_921*qd[22];
    OPcp8_122 = OPcp8_119+ROcp8_111*qdd[20]+ROcp8_420*qdd[21]+ROcp8_721*qdd[22]+qd[20]*(OMcp8_219*ROcp8_311-OMcp8_319*
 ROcp8_211)+qd[21]*(OMcp8_220*ROcp8_620-OMcp8_320*ROcp8_520)+qd[22]*(OMcp8_221*ROcp8_921-OMcp8_321*ROcp8_821);
    OPcp8_222 = OPcp8_219+ROcp8_211*qdd[20]+ROcp8_520*qdd[21]+ROcp8_821*qdd[22]-qd[20]*(OMcp8_119*ROcp8_311-OMcp8_319*
 ROcp8_111)-qd[21]*(OMcp8_120*ROcp8_620-OMcp8_320*ROcp8_420)-qd[22]*(OMcp8_121*ROcp8_921-OMcp8_321*ROcp8_721);
    OPcp8_322 = OPcp8_319+ROcp8_311*qdd[20]+ROcp8_620*qdd[21]+ROcp8_921*qdd[22]+qd[20]*(OMcp8_119*ROcp8_211-OMcp8_219*
 ROcp8_111)+qd[21]*(OMcp8_120*ROcp8_520-OMcp8_220*ROcp8_420)+qd[22]*(OMcp8_121*ROcp8_821-OMcp8_221*ROcp8_721);
    RLcp8_154 = ROcp8_721*s->dpt[3][27];
    RLcp8_254 = ROcp8_821*s->dpt[3][27];
    RLcp8_354 = ROcp8_921*s->dpt[3][27];
    POcp8_154 = RLcp8_111+RLcp8_119+RLcp8_12+RLcp8_120+RLcp8_13+RLcp8_154;
    POcp8_254 = RLcp8_211+RLcp8_219+RLcp8_22+RLcp8_220+RLcp8_23+RLcp8_254;
    POcp8_354 = RLcp8_311+RLcp8_319+RLcp8_320+RLcp8_354+q[4];
    JTcp8_154_1 = -(RLcp8_211+RLcp8_219+RLcp8_22+RLcp8_220+RLcp8_23+RLcp8_254);
    JTcp8_254_1 = RLcp8_111+RLcp8_119+RLcp8_12+RLcp8_120+RLcp8_13+RLcp8_154;
    JTcp8_154_5 = -(RLcp8_211+RLcp8_219+RLcp8_220+RLcp8_254);
    JTcp8_254_5 = RLcp8_111+RLcp8_119+RLcp8_120+RLcp8_154;
    JTcp8_154_6 = S1p5*(RLcp8_311+RLcp8_319+RLcp8_320+RLcp8_354);
    JTcp8_254_6 = -C1p5*(RLcp8_311+RLcp8_319+RLcp8_320+RLcp8_354);
    JTcp8_354_6 = C1p5*(RLcp8_211+RLcp8_219+RLcp8_220+RLcp8_254)-S1p5*(RLcp8_111+RLcp8_119)-S1p5*(RLcp8_120+RLcp8_154);
    JTcp8_154_7 = ROcp8_56*(RLcp8_311+RLcp8_319+RLcp8_320+RLcp8_354)-S6*(RLcp8_211+RLcp8_219)-S6*(RLcp8_220+RLcp8_254);
    JTcp8_254_7 = -(ROcp8_46*(RLcp8_311+RLcp8_319+RLcp8_320+RLcp8_354)-S6*(RLcp8_111+RLcp8_119)-S6*(RLcp8_120+RLcp8_154));
    JTcp8_354_7 = ROcp8_46*(RLcp8_211+RLcp8_219+RLcp8_220+RLcp8_254)-ROcp8_56*(RLcp8_111+RLcp8_119)-ROcp8_56*(RLcp8_120+
 RLcp8_154);
    JTcp8_154_8 = ROcp8_56*(RLcp8_319+RLcp8_320)-S6*(RLcp8_219+RLcp8_220)-RLcp8_254*S6+RLcp8_354*ROcp8_56;
    JTcp8_254_8 = RLcp8_154*S6-RLcp8_354*ROcp8_46-ROcp8_46*(RLcp8_319+RLcp8_320)+S6*(RLcp8_119+RLcp8_120);
    JTcp8_354_8 = ROcp8_46*(RLcp8_219+RLcp8_220)-ROcp8_56*(RLcp8_119+RLcp8_120)-RLcp8_154*ROcp8_56+RLcp8_254*ROcp8_46;
    JTcp8_154_9 = ROcp8_211*(RLcp8_320+RLcp8_354)-ROcp8_311*(RLcp8_220+RLcp8_254);
    JTcp8_254_9 = -(ROcp8_111*(RLcp8_320+RLcp8_354)-ROcp8_311*(RLcp8_120+RLcp8_154));
    JTcp8_354_9 = ROcp8_111*(RLcp8_220+RLcp8_254)-ROcp8_211*(RLcp8_120+RLcp8_154);
    JTcp8_154_10 = -(RLcp8_254*ROcp8_311-RLcp8_354*ROcp8_211);
    JTcp8_254_10 = RLcp8_154*ROcp8_311-RLcp8_354*ROcp8_111;
    JTcp8_354_10 = -(RLcp8_154*ROcp8_211-RLcp8_254*ROcp8_111);
    JTcp8_154_11 = -(RLcp8_254*ROcp8_620-RLcp8_354*ROcp8_520);
    JTcp8_254_11 = RLcp8_154*ROcp8_620-RLcp8_354*ROcp8_420;
    JTcp8_354_11 = -(RLcp8_154*ROcp8_520-RLcp8_254*ROcp8_420);
    JTcp8_154_12 = -(RLcp8_254*ROcp8_921-RLcp8_354*ROcp8_821);
    JTcp8_254_12 = RLcp8_154*ROcp8_921-RLcp8_354*ROcp8_721;
    JTcp8_354_12 = -(RLcp8_154*ROcp8_821-RLcp8_254*ROcp8_721);
    ORcp8_154 = OMcp8_222*RLcp8_354-OMcp8_322*RLcp8_254;
    ORcp8_254 = -(OMcp8_122*RLcp8_354-OMcp8_322*RLcp8_154);
    ORcp8_354 = OMcp8_122*RLcp8_254-OMcp8_222*RLcp8_154;
    VIcp8_154 = ORcp8_111+ORcp8_119+ORcp8_12+ORcp8_120+ORcp8_13+ORcp8_154+qd[2]*C1-qd[3]*S1;
    VIcp8_254 = ORcp8_211+ORcp8_219+ORcp8_22+ORcp8_220+ORcp8_23+ORcp8_254+qd[2]*S1+qd[3]*C1;
    VIcp8_354 = ORcp8_311+ORcp8_319+ORcp8_320+ORcp8_354+qd[4];
    ACcp8_154 = OMcp8_219*ORcp8_320+OMcp8_222*ORcp8_354+OMcp8_27*(ORcp8_311+ORcp8_319)-OMcp8_319*ORcp8_220-OMcp8_322*
 ORcp8_254-OMcp8_37*ORcp8_211-OMcp8_37*ORcp8_219+OPcp8_219*RLcp8_320+OPcp8_222*RLcp8_354+OPcp8_27*RLcp8_311+OPcp8_27*
 RLcp8_319-OPcp8_319*RLcp8_220-OPcp8_322*RLcp8_254-OPcp8_37*RLcp8_211-OPcp8_37*RLcp8_219-ORcp8_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*
 qd[1]*qd[3]*C1-qd[1]*(ORcp8_22+(2.0)*qd[2]*S1);
    ACcp8_254 = -(OMcp8_119*ORcp8_320+OMcp8_122*ORcp8_354+OMcp8_17*(ORcp8_311+ORcp8_319)-OMcp8_319*ORcp8_120-OMcp8_322*
 ORcp8_154-OMcp8_37*ORcp8_111-OMcp8_37*ORcp8_119+OPcp8_119*RLcp8_320+OPcp8_122*RLcp8_354+OPcp8_17*RLcp8_311+OPcp8_17*
 RLcp8_319-OPcp8_319*RLcp8_120-OPcp8_322*RLcp8_154-OPcp8_37*RLcp8_111-OPcp8_37*RLcp8_119-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp8_12+(2.0)
 *qd[2]*C1)-qd[1]*(ORcp8_13-(2.0)*qd[3]*S1));
    ACcp8_354 = qdd[4]+OMcp8_119*ORcp8_220+OMcp8_122*ORcp8_254+OMcp8_17*ORcp8_211+OMcp8_17*ORcp8_219-OMcp8_219*ORcp8_120-
 OMcp8_222*ORcp8_154-OMcp8_27*ORcp8_111-OMcp8_27*ORcp8_119+OPcp8_119*RLcp8_220+OPcp8_122*RLcp8_254+OPcp8_17*RLcp8_211+
 OPcp8_17*RLcp8_219-OPcp8_219*RLcp8_120-OPcp8_222*RLcp8_154-OPcp8_27*RLcp8_111-OPcp8_27*RLcp8_119;

// = = Block_1_0_0_9_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp8_154;
    sens->P[2] = POcp8_254;
    sens->P[3] = POcp8_354;
    sens->R[1][1] = ROcp8_122;
    sens->R[1][2] = ROcp8_222;
    sens->R[1][3] = ROcp8_322;
    sens->R[2][1] = ROcp8_422;
    sens->R[2][2] = ROcp8_522;
    sens->R[2][3] = ROcp8_622;
    sens->R[3][1] = ROcp8_721;
    sens->R[3][2] = ROcp8_821;
    sens->R[3][3] = ROcp8_921;
    sens->V[1] = VIcp8_154;
    sens->V[2] = VIcp8_254;
    sens->V[3] = VIcp8_354;
    sens->OM[1] = OMcp8_122;
    sens->OM[2] = OMcp8_222;
    sens->OM[3] = OMcp8_322;
    sens->J[1][1] = JTcp8_154_1;
    sens->J[1][2] = C1;
    sens->J[1][3] = -S1;
    sens->J[1][5] = JTcp8_154_5;
    sens->J[1][6] = JTcp8_154_6;
    sens->J[1][7] = JTcp8_154_7;
    sens->J[1][11] = JTcp8_154_8;
    sens->J[1][19] = JTcp8_154_9;
    sens->J[1][20] = JTcp8_154_10;
    sens->J[1][21] = JTcp8_154_11;
    sens->J[1][22] = JTcp8_154_12;
    sens->J[2][1] = JTcp8_254_1;
    sens->J[2][2] = S1;
    sens->J[2][3] = C1;
    sens->J[2][5] = JTcp8_254_5;
    sens->J[2][6] = JTcp8_254_6;
    sens->J[2][7] = JTcp8_254_7;
    sens->J[2][11] = JTcp8_254_8;
    sens->J[2][19] = JTcp8_254_9;
    sens->J[2][20] = JTcp8_254_10;
    sens->J[2][21] = JTcp8_254_11;
    sens->J[2][22] = JTcp8_254_12;
    sens->J[3][4] = (1.0);
    sens->J[3][6] = JTcp8_354_6;
    sens->J[3][7] = JTcp8_354_7;
    sens->J[3][11] = JTcp8_354_8;
    sens->J[3][19] = JTcp8_354_9;
    sens->J[3][20] = JTcp8_354_10;
    sens->J[3][21] = JTcp8_354_11;
    sens->J[3][22] = JTcp8_354_12;
    sens->J[4][6] = C1p5;
    sens->J[4][7] = ROcp8_46;
    sens->J[4][11] = ROcp8_46;
    sens->J[4][19] = ROcp8_111;
    sens->J[4][20] = ROcp8_111;
    sens->J[4][21] = ROcp8_420;
    sens->J[4][22] = ROcp8_721;
    sens->J[5][6] = S1p5;
    sens->J[5][7] = ROcp8_56;
    sens->J[5][11] = ROcp8_56;
    sens->J[5][19] = ROcp8_211;
    sens->J[5][20] = ROcp8_211;
    sens->J[5][21] = ROcp8_520;
    sens->J[5][22] = ROcp8_821;
    sens->J[6][1] = (1.0);
    sens->J[6][5] = (1.0);
    sens->J[6][7] = S6;
    sens->J[6][11] = S6;
    sens->J[6][19] = ROcp8_311;
    sens->J[6][20] = ROcp8_311;
    sens->J[6][21] = ROcp8_620;
    sens->J[6][22] = ROcp8_921;
    sens->A[1] = ACcp8_154;
    sens->A[2] = ACcp8_254;
    sens->A[3] = ACcp8_354;
    sens->OMP[1] = OPcp8_122;
    sens->OMP[2] = OPcp8_222;
    sens->OMP[3] = OPcp8_322;
 
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


    ROcp9_111 = ROcp9_17*C11-ROcp9_77*S11;
    ROcp9_211 = ROcp9_27*C11-ROcp9_87*S11;
    ROcp9_311 = -S11p7*C6;
    ROcp9_711 = ROcp9_17*S11+ROcp9_77*C11;
    ROcp9_811 = ROcp9_27*S11+ROcp9_87*C11;
    ROcp9_911 = C11p7*C6;
    RLcp9_111 = ROcp9_17*s->dpt[1][5]+ROcp9_77*s->dpt[3][5];
    RLcp9_211 = ROcp9_27*s->dpt[1][5]+ROcp9_87*s->dpt[3][5];
    RLcp9_311 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp9_111 = OMcp9_27*RLcp9_311-OMcp9_37*RLcp9_211;
    ORcp9_211 = -(OMcp9_17*RLcp9_311-OMcp9_37*RLcp9_111);
    ORcp9_311 = OMcp9_17*RLcp9_211-OMcp9_27*RLcp9_111;

// = = Block_1_0_0_10_0_6 = = 
 
// Sensor Kinematics 


    ROcp9_419 = ROcp9_46*C19+ROcp9_711*S19;
    ROcp9_519 = ROcp9_56*C19+ROcp9_811*S19;
    ROcp9_619 = ROcp9_911*S19+C19*S6;
    ROcp9_719 = -(ROcp9_46*S19-ROcp9_711*C19);
    ROcp9_819 = -(ROcp9_56*S19-ROcp9_811*C19);
    ROcp9_919 = ROcp9_911*C19-S19*S6;
    ROcp9_420 = ROcp9_419*C20+ROcp9_719*S20;
    ROcp9_520 = ROcp9_519*C20+ROcp9_819*S20;
    ROcp9_620 = ROcp9_619*C20+ROcp9_919*S20;
    ROcp9_720 = -(ROcp9_419*S20-ROcp9_719*C20);
    ROcp9_820 = -(ROcp9_519*S20-ROcp9_819*C20);
    ROcp9_920 = -(ROcp9_619*S20-ROcp9_919*C20);
    ROcp9_121 = ROcp9_111*C21-ROcp9_720*S21;
    ROcp9_221 = ROcp9_211*C21-ROcp9_820*S21;
    ROcp9_321 = ROcp9_311*C21-ROcp9_920*S21;
    ROcp9_721 = ROcp9_111*S21+ROcp9_720*C21;
    ROcp9_821 = ROcp9_211*S21+ROcp9_820*C21;
    ROcp9_921 = ROcp9_311*S21+ROcp9_920*C21;
    ROcp9_122 = ROcp9_121*C22+ROcp9_420*S22;
    ROcp9_222 = ROcp9_221*C22+ROcp9_520*S22;
    ROcp9_322 = ROcp9_321*C22+ROcp9_620*S22;
    ROcp9_422 = -(ROcp9_121*S22-ROcp9_420*C22);
    ROcp9_522 = -(ROcp9_221*S22-ROcp9_520*C22);
    ROcp9_622 = -(ROcp9_321*S22-ROcp9_620*C22);
    RLcp9_119 = ROcp9_46*s->dpt[2][13]+ROcp9_711*s->dpt[3][13];
    RLcp9_219 = ROcp9_56*s->dpt[2][13]+ROcp9_811*s->dpt[3][13];
    RLcp9_319 = ROcp9_911*s->dpt[3][13]+s->dpt[2][13]*S6;
    OMcp9_119 = OMcp9_17+ROcp9_111*qd[19];
    OMcp9_219 = OMcp9_27+ROcp9_211*qd[19];
    OMcp9_319 = OMcp9_37+ROcp9_311*qd[19];
    ORcp9_119 = OMcp9_27*RLcp9_319-OMcp9_37*RLcp9_219;
    ORcp9_219 = -(OMcp9_17*RLcp9_319-OMcp9_37*RLcp9_119);
    ORcp9_319 = OMcp9_17*RLcp9_219-OMcp9_27*RLcp9_119;
    OPcp9_119 = OPcp9_17+ROcp9_111*qdd[19]+qd[19]*(OMcp9_27*ROcp9_311-OMcp9_37*ROcp9_211);
    OPcp9_219 = OPcp9_27+ROcp9_211*qdd[19]-qd[19]*(OMcp9_17*ROcp9_311-OMcp9_37*ROcp9_111);
    OPcp9_319 = OPcp9_37+ROcp9_311*qdd[19]+qd[19]*(OMcp9_17*ROcp9_211-OMcp9_27*ROcp9_111);
    RLcp9_120 = ROcp9_419*s->dpt[2][24];
    RLcp9_220 = ROcp9_519*s->dpt[2][24];
    RLcp9_320 = ROcp9_619*s->dpt[2][24];
    OMcp9_120 = OMcp9_119+ROcp9_111*qd[20];
    OMcp9_220 = OMcp9_219+ROcp9_211*qd[20];
    OMcp9_320 = OMcp9_319+ROcp9_311*qd[20];
    ORcp9_120 = OMcp9_219*RLcp9_320-OMcp9_319*RLcp9_220;
    ORcp9_220 = -(OMcp9_119*RLcp9_320-OMcp9_319*RLcp9_120);
    ORcp9_320 = OMcp9_119*RLcp9_220-OMcp9_219*RLcp9_120;
    OMcp9_121 = OMcp9_120+ROcp9_420*qd[21];
    OMcp9_221 = OMcp9_220+ROcp9_520*qd[21];
    OMcp9_321 = OMcp9_320+ROcp9_620*qd[21];
    OMcp9_122 = OMcp9_121+ROcp9_721*qd[22];
    OMcp9_222 = OMcp9_221+ROcp9_821*qd[22];
    OMcp9_322 = OMcp9_321+ROcp9_921*qd[22];
    OPcp9_122 = OPcp9_119+ROcp9_111*qdd[20]+ROcp9_420*qdd[21]+ROcp9_721*qdd[22]+qd[20]*(OMcp9_219*ROcp9_311-OMcp9_319*
 ROcp9_211)+qd[21]*(OMcp9_220*ROcp9_620-OMcp9_320*ROcp9_520)+qd[22]*(OMcp9_221*ROcp9_921-OMcp9_321*ROcp9_821);
    OPcp9_222 = OPcp9_219+ROcp9_211*qdd[20]+ROcp9_520*qdd[21]+ROcp9_821*qdd[22]-qd[20]*(OMcp9_119*ROcp9_311-OMcp9_319*
 ROcp9_111)-qd[21]*(OMcp9_120*ROcp9_620-OMcp9_320*ROcp9_420)-qd[22]*(OMcp9_121*ROcp9_921-OMcp9_321*ROcp9_721);
    OPcp9_322 = OPcp9_319+ROcp9_311*qdd[20]+ROcp9_620*qdd[21]+ROcp9_921*qdd[22]+qd[20]*(OMcp9_119*ROcp9_211-OMcp9_219*
 ROcp9_111)+qd[21]*(OMcp9_120*ROcp9_520-OMcp9_220*ROcp9_420)+qd[22]*(OMcp9_121*ROcp9_821-OMcp9_221*ROcp9_721);
    RLcp9_155 = ROcp9_122*s->dpt[1][28]+ROcp9_422*s->dpt[2][28]+ROcp9_721*s->dpt[3][28];
    RLcp9_255 = ROcp9_222*s->dpt[1][28]+ROcp9_522*s->dpt[2][28]+ROcp9_821*s->dpt[3][28];
    RLcp9_355 = ROcp9_322*s->dpt[1][28]+ROcp9_622*s->dpt[2][28]+ROcp9_921*s->dpt[3][28];
    POcp9_155 = RLcp9_111+RLcp9_119+RLcp9_12+RLcp9_120+RLcp9_13+RLcp9_155;
    POcp9_255 = RLcp9_211+RLcp9_219+RLcp9_22+RLcp9_220+RLcp9_23+RLcp9_255;
    POcp9_355 = RLcp9_311+RLcp9_319+RLcp9_320+RLcp9_355+q[4];
    JTcp9_155_1 = -(RLcp9_211+RLcp9_219+RLcp9_22+RLcp9_220+RLcp9_23+RLcp9_255);
    JTcp9_255_1 = RLcp9_111+RLcp9_119+RLcp9_12+RLcp9_120+RLcp9_13+RLcp9_155;
    JTcp9_155_5 = -(RLcp9_211+RLcp9_219+RLcp9_220+RLcp9_255);
    JTcp9_255_5 = RLcp9_111+RLcp9_119+RLcp9_120+RLcp9_155;
    JTcp9_155_6 = S1p5*(RLcp9_311+RLcp9_319+RLcp9_320+RLcp9_355);
    JTcp9_255_6 = -C1p5*(RLcp9_311+RLcp9_319+RLcp9_320+RLcp9_355);
    JTcp9_355_6 = C1p5*(RLcp9_211+RLcp9_219+RLcp9_220+RLcp9_255)-S1p5*(RLcp9_111+RLcp9_119)-S1p5*(RLcp9_120+RLcp9_155);
    JTcp9_155_7 = ROcp9_56*(RLcp9_311+RLcp9_319+RLcp9_320+RLcp9_355)-S6*(RLcp9_211+RLcp9_219)-S6*(RLcp9_220+RLcp9_255);
    JTcp9_255_7 = -(ROcp9_46*(RLcp9_311+RLcp9_319+RLcp9_320+RLcp9_355)-S6*(RLcp9_111+RLcp9_119)-S6*(RLcp9_120+RLcp9_155));
    JTcp9_355_7 = ROcp9_46*(RLcp9_211+RLcp9_219+RLcp9_220+RLcp9_255)-ROcp9_56*(RLcp9_111+RLcp9_119)-ROcp9_56*(RLcp9_120+
 RLcp9_155);
    JTcp9_155_8 = ROcp9_56*(RLcp9_319+RLcp9_320)-S6*(RLcp9_219+RLcp9_220)-RLcp9_255*S6+RLcp9_355*ROcp9_56;
    JTcp9_255_8 = RLcp9_155*S6-RLcp9_355*ROcp9_46-ROcp9_46*(RLcp9_319+RLcp9_320)+S6*(RLcp9_119+RLcp9_120);
    JTcp9_355_8 = ROcp9_46*(RLcp9_219+RLcp9_220)-ROcp9_56*(RLcp9_119+RLcp9_120)-RLcp9_155*ROcp9_56+RLcp9_255*ROcp9_46;
    JTcp9_155_9 = ROcp9_211*(RLcp9_320+RLcp9_355)-ROcp9_311*(RLcp9_220+RLcp9_255);
    JTcp9_255_9 = -(ROcp9_111*(RLcp9_320+RLcp9_355)-ROcp9_311*(RLcp9_120+RLcp9_155));
    JTcp9_355_9 = ROcp9_111*(RLcp9_220+RLcp9_255)-ROcp9_211*(RLcp9_120+RLcp9_155);
    JTcp9_155_10 = -(RLcp9_255*ROcp9_311-RLcp9_355*ROcp9_211);
    JTcp9_255_10 = RLcp9_155*ROcp9_311-RLcp9_355*ROcp9_111;
    JTcp9_355_10 = -(RLcp9_155*ROcp9_211-RLcp9_255*ROcp9_111);
    JTcp9_155_11 = -(RLcp9_255*ROcp9_620-RLcp9_355*ROcp9_520);
    JTcp9_255_11 = RLcp9_155*ROcp9_620-RLcp9_355*ROcp9_420;
    JTcp9_355_11 = -(RLcp9_155*ROcp9_520-RLcp9_255*ROcp9_420);
    JTcp9_155_12 = -(RLcp9_255*ROcp9_921-RLcp9_355*ROcp9_821);
    JTcp9_255_12 = RLcp9_155*ROcp9_921-RLcp9_355*ROcp9_721;
    JTcp9_355_12 = -(RLcp9_155*ROcp9_821-RLcp9_255*ROcp9_721);
    ORcp9_155 = OMcp9_222*RLcp9_355-OMcp9_322*RLcp9_255;
    ORcp9_255 = -(OMcp9_122*RLcp9_355-OMcp9_322*RLcp9_155);
    ORcp9_355 = OMcp9_122*RLcp9_255-OMcp9_222*RLcp9_155;
    VIcp9_155 = ORcp9_111+ORcp9_119+ORcp9_12+ORcp9_120+ORcp9_13+ORcp9_155+qd[2]*C1-qd[3]*S1;
    VIcp9_255 = ORcp9_211+ORcp9_219+ORcp9_22+ORcp9_220+ORcp9_23+ORcp9_255+qd[2]*S1+qd[3]*C1;
    VIcp9_355 = ORcp9_311+ORcp9_319+ORcp9_320+ORcp9_355+qd[4];
    ACcp9_155 = OMcp9_219*ORcp9_320+OMcp9_222*ORcp9_355+OMcp9_27*(ORcp9_311+ORcp9_319)-OMcp9_319*ORcp9_220-OMcp9_322*
 ORcp9_255-OMcp9_37*ORcp9_211-OMcp9_37*ORcp9_219+OPcp9_219*RLcp9_320+OPcp9_222*RLcp9_355+OPcp9_27*RLcp9_311+OPcp9_27*
 RLcp9_319-OPcp9_319*RLcp9_220-OPcp9_322*RLcp9_255-OPcp9_37*RLcp9_211-OPcp9_37*RLcp9_219-ORcp9_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*
 qd[1]*qd[3]*C1-qd[1]*(ORcp9_22+(2.0)*qd[2]*S1);
    ACcp9_255 = -(OMcp9_119*ORcp9_320+OMcp9_122*ORcp9_355+OMcp9_17*(ORcp9_311+ORcp9_319)-OMcp9_319*ORcp9_120-OMcp9_322*
 ORcp9_155-OMcp9_37*ORcp9_111-OMcp9_37*ORcp9_119+OPcp9_119*RLcp9_320+OPcp9_122*RLcp9_355+OPcp9_17*RLcp9_311+OPcp9_17*
 RLcp9_319-OPcp9_319*RLcp9_120-OPcp9_322*RLcp9_155-OPcp9_37*RLcp9_111-OPcp9_37*RLcp9_119-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp9_12+(2.0)
 *qd[2]*C1)-qd[1]*(ORcp9_13-(2.0)*qd[3]*S1));
    ACcp9_355 = qdd[4]+OMcp9_119*ORcp9_220+OMcp9_122*ORcp9_255+OMcp9_17*ORcp9_211+OMcp9_17*ORcp9_219-OMcp9_219*ORcp9_120-
 OMcp9_222*ORcp9_155-OMcp9_27*ORcp9_111-OMcp9_27*ORcp9_119+OPcp9_119*RLcp9_220+OPcp9_122*RLcp9_255+OPcp9_17*RLcp9_211+
 OPcp9_17*RLcp9_219-OPcp9_219*RLcp9_120-OPcp9_222*RLcp9_155-OPcp9_27*RLcp9_111-OPcp9_27*RLcp9_119;

// = = Block_1_0_0_10_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp9_155;
    sens->P[2] = POcp9_255;
    sens->P[3] = POcp9_355;
    sens->R[1][1] = ROcp9_122;
    sens->R[1][2] = ROcp9_222;
    sens->R[1][3] = ROcp9_322;
    sens->R[2][1] = ROcp9_422;
    sens->R[2][2] = ROcp9_522;
    sens->R[2][3] = ROcp9_622;
    sens->R[3][1] = ROcp9_721;
    sens->R[3][2] = ROcp9_821;
    sens->R[3][3] = ROcp9_921;
    sens->V[1] = VIcp9_155;
    sens->V[2] = VIcp9_255;
    sens->V[3] = VIcp9_355;
    sens->OM[1] = OMcp9_122;
    sens->OM[2] = OMcp9_222;
    sens->OM[3] = OMcp9_322;
    sens->J[1][1] = JTcp9_155_1;
    sens->J[1][2] = C1;
    sens->J[1][3] = -S1;
    sens->J[1][5] = JTcp9_155_5;
    sens->J[1][6] = JTcp9_155_6;
    sens->J[1][7] = JTcp9_155_7;
    sens->J[1][11] = JTcp9_155_8;
    sens->J[1][19] = JTcp9_155_9;
    sens->J[1][20] = JTcp9_155_10;
    sens->J[1][21] = JTcp9_155_11;
    sens->J[1][22] = JTcp9_155_12;
    sens->J[2][1] = JTcp9_255_1;
    sens->J[2][2] = S1;
    sens->J[2][3] = C1;
    sens->J[2][5] = JTcp9_255_5;
    sens->J[2][6] = JTcp9_255_6;
    sens->J[2][7] = JTcp9_255_7;
    sens->J[2][11] = JTcp9_255_8;
    sens->J[2][19] = JTcp9_255_9;
    sens->J[2][20] = JTcp9_255_10;
    sens->J[2][21] = JTcp9_255_11;
    sens->J[2][22] = JTcp9_255_12;
    sens->J[3][4] = (1.0);
    sens->J[3][6] = JTcp9_355_6;
    sens->J[3][7] = JTcp9_355_7;
    sens->J[3][11] = JTcp9_355_8;
    sens->J[3][19] = JTcp9_355_9;
    sens->J[3][20] = JTcp9_355_10;
    sens->J[3][21] = JTcp9_355_11;
    sens->J[3][22] = JTcp9_355_12;
    sens->J[4][6] = C1p5;
    sens->J[4][7] = ROcp9_46;
    sens->J[4][11] = ROcp9_46;
    sens->J[4][19] = ROcp9_111;
    sens->J[4][20] = ROcp9_111;
    sens->J[4][21] = ROcp9_420;
    sens->J[4][22] = ROcp9_721;
    sens->J[5][6] = S1p5;
    sens->J[5][7] = ROcp9_56;
    sens->J[5][11] = ROcp9_56;
    sens->J[5][19] = ROcp9_211;
    sens->J[5][20] = ROcp9_211;
    sens->J[5][21] = ROcp9_520;
    sens->J[5][22] = ROcp9_821;
    sens->J[6][1] = (1.0);
    sens->J[6][5] = (1.0);
    sens->J[6][7] = S6;
    sens->J[6][11] = S6;
    sens->J[6][19] = ROcp9_311;
    sens->J[6][20] = ROcp9_311;
    sens->J[6][21] = ROcp9_620;
    sens->J[6][22] = ROcp9_921;
    sens->A[1] = ACcp9_155;
    sens->A[2] = ACcp9_255;
    sens->A[3] = ACcp9_355;
    sens->OMP[1] = OPcp9_122;
    sens->OMP[2] = OPcp9_222;
    sens->OMP[3] = OPcp9_322;
 
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


    ROcp10_111 = ROcp10_17*C11-ROcp10_77*S11;
    ROcp10_211 = ROcp10_27*C11-ROcp10_87*S11;
    ROcp10_311 = -S11p7*C6;
    ROcp10_711 = ROcp10_17*S11+ROcp10_77*C11;
    ROcp10_811 = ROcp10_27*S11+ROcp10_87*C11;
    ROcp10_911 = C11p7*C6;
    RLcp10_111 = ROcp10_17*s->dpt[1][5]+ROcp10_77*s->dpt[3][5];
    RLcp10_211 = ROcp10_27*s->dpt[1][5]+ROcp10_87*s->dpt[3][5];
    RLcp10_311 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp10_111 = OMcp10_27*RLcp10_311-OMcp10_37*RLcp10_211;
    ORcp10_211 = -(OMcp10_17*RLcp10_311-OMcp10_37*RLcp10_111);
    ORcp10_311 = OMcp10_17*RLcp10_211-OMcp10_27*RLcp10_111;

// = = Block_1_0_0_11_0_6 = = 
 
// Sensor Kinematics 


    ROcp10_419 = ROcp10_46*C19+ROcp10_711*S19;
    ROcp10_519 = ROcp10_56*C19+ROcp10_811*S19;
    ROcp10_619 = ROcp10_911*S19+C19*S6;
    ROcp10_719 = -(ROcp10_46*S19-ROcp10_711*C19);
    ROcp10_819 = -(ROcp10_56*S19-ROcp10_811*C19);
    ROcp10_919 = ROcp10_911*C19-S19*S6;
    ROcp10_420 = ROcp10_419*C20+ROcp10_719*S20;
    ROcp10_520 = ROcp10_519*C20+ROcp10_819*S20;
    ROcp10_620 = ROcp10_619*C20+ROcp10_919*S20;
    ROcp10_720 = -(ROcp10_419*S20-ROcp10_719*C20);
    ROcp10_820 = -(ROcp10_519*S20-ROcp10_819*C20);
    ROcp10_920 = -(ROcp10_619*S20-ROcp10_919*C20);
    ROcp10_121 = ROcp10_111*C21-ROcp10_720*S21;
    ROcp10_221 = ROcp10_211*C21-ROcp10_820*S21;
    ROcp10_321 = ROcp10_311*C21-ROcp10_920*S21;
    ROcp10_721 = ROcp10_111*S21+ROcp10_720*C21;
    ROcp10_821 = ROcp10_211*S21+ROcp10_820*C21;
    ROcp10_921 = ROcp10_311*S21+ROcp10_920*C21;
    ROcp10_122 = ROcp10_121*C22+ROcp10_420*S22;
    ROcp10_222 = ROcp10_221*C22+ROcp10_520*S22;
    ROcp10_322 = ROcp10_321*C22+ROcp10_620*S22;
    ROcp10_422 = -(ROcp10_121*S22-ROcp10_420*C22);
    ROcp10_522 = -(ROcp10_221*S22-ROcp10_520*C22);
    ROcp10_622 = -(ROcp10_321*S22-ROcp10_620*C22);
    ROcp10_423 = ROcp10_422*C23+ROcp10_721*S23;
    ROcp10_523 = ROcp10_522*C23+ROcp10_821*S23;
    ROcp10_623 = ROcp10_622*C23+ROcp10_921*S23;
    ROcp10_723 = -(ROcp10_422*S23-ROcp10_721*C23);
    ROcp10_823 = -(ROcp10_522*S23-ROcp10_821*C23);
    ROcp10_923 = -(ROcp10_622*S23-ROcp10_921*C23);
    ROcp10_124 = ROcp10_122*C24-ROcp10_723*S24;
    ROcp10_224 = ROcp10_222*C24-ROcp10_823*S24;
    ROcp10_324 = ROcp10_322*C24-ROcp10_923*S24;
    ROcp10_724 = ROcp10_122*S24+ROcp10_723*C24;
    ROcp10_824 = ROcp10_222*S24+ROcp10_823*C24;
    ROcp10_924 = ROcp10_322*S24+ROcp10_923*C24;
    RLcp10_119 = ROcp10_46*s->dpt[2][13]+ROcp10_711*s->dpt[3][13];
    RLcp10_219 = ROcp10_56*s->dpt[2][13]+ROcp10_811*s->dpt[3][13];
    RLcp10_319 = ROcp10_911*s->dpt[3][13]+s->dpt[2][13]*S6;
    OMcp10_119 = OMcp10_17+ROcp10_111*qd[19];
    OMcp10_219 = OMcp10_27+ROcp10_211*qd[19];
    OMcp10_319 = OMcp10_37+ROcp10_311*qd[19];
    ORcp10_119 = OMcp10_27*RLcp10_319-OMcp10_37*RLcp10_219;
    ORcp10_219 = -(OMcp10_17*RLcp10_319-OMcp10_37*RLcp10_119);
    ORcp10_319 = OMcp10_17*RLcp10_219-OMcp10_27*RLcp10_119;
    OPcp10_119 = OPcp10_17+ROcp10_111*qdd[19]+qd[19]*(OMcp10_27*ROcp10_311-OMcp10_37*ROcp10_211);
    OPcp10_219 = OPcp10_27+ROcp10_211*qdd[19]-qd[19]*(OMcp10_17*ROcp10_311-OMcp10_37*ROcp10_111);
    OPcp10_319 = OPcp10_37+ROcp10_311*qdd[19]+qd[19]*(OMcp10_17*ROcp10_211-OMcp10_27*ROcp10_111);
    RLcp10_120 = ROcp10_419*s->dpt[2][24];
    RLcp10_220 = ROcp10_519*s->dpt[2][24];
    RLcp10_320 = ROcp10_619*s->dpt[2][24];
    OMcp10_120 = OMcp10_119+ROcp10_111*qd[20];
    OMcp10_220 = OMcp10_219+ROcp10_211*qd[20];
    OMcp10_320 = OMcp10_319+ROcp10_311*qd[20];
    ORcp10_120 = OMcp10_219*RLcp10_320-OMcp10_319*RLcp10_220;
    ORcp10_220 = -(OMcp10_119*RLcp10_320-OMcp10_319*RLcp10_120);
    ORcp10_320 = OMcp10_119*RLcp10_220-OMcp10_219*RLcp10_120;
    OMcp10_121 = OMcp10_120+ROcp10_420*qd[21];
    OMcp10_221 = OMcp10_220+ROcp10_520*qd[21];
    OMcp10_321 = OMcp10_320+ROcp10_620*qd[21];
    OMcp10_122 = OMcp10_121+ROcp10_721*qd[22];
    OMcp10_222 = OMcp10_221+ROcp10_821*qd[22];
    OMcp10_322 = OMcp10_321+ROcp10_921*qd[22];
    OPcp10_122 = OPcp10_119+ROcp10_111*qdd[20]+ROcp10_420*qdd[21]+ROcp10_721*qdd[22]+qd[20]*(OMcp10_219*ROcp10_311-
 OMcp10_319*ROcp10_211)+qd[21]*(OMcp10_220*ROcp10_620-OMcp10_320*ROcp10_520)+qd[22]*(OMcp10_221*ROcp10_921-OMcp10_321*
 ROcp10_821);
    OPcp10_222 = OPcp10_219+ROcp10_211*qdd[20]+ROcp10_520*qdd[21]+ROcp10_821*qdd[22]-qd[20]*(OMcp10_119*ROcp10_311-
 OMcp10_319*ROcp10_111)-qd[21]*(OMcp10_120*ROcp10_620-OMcp10_320*ROcp10_420)-qd[22]*(OMcp10_121*ROcp10_921-OMcp10_321*
 ROcp10_721);
    OPcp10_322 = OPcp10_319+ROcp10_311*qdd[20]+ROcp10_620*qdd[21]+ROcp10_921*qdd[22]+qd[20]*(OMcp10_119*ROcp10_211-
 OMcp10_219*ROcp10_111)+qd[21]*(OMcp10_120*ROcp10_520-OMcp10_220*ROcp10_420)+qd[22]*(OMcp10_121*ROcp10_821-OMcp10_221*
 ROcp10_721);
    RLcp10_123 = ROcp10_122*s->dpt[1][28]+ROcp10_422*s->dpt[2][28]+ROcp10_721*s->dpt[3][28];
    RLcp10_223 = ROcp10_222*s->dpt[1][28]+ROcp10_522*s->dpt[2][28]+ROcp10_821*s->dpt[3][28];
    RLcp10_323 = ROcp10_322*s->dpt[1][28]+ROcp10_622*s->dpt[2][28]+ROcp10_921*s->dpt[3][28];
    POcp10_123 = RLcp10_111+RLcp10_119+RLcp10_12+RLcp10_120+RLcp10_123+RLcp10_13;
    POcp10_223 = RLcp10_211+RLcp10_219+RLcp10_22+RLcp10_220+RLcp10_223+RLcp10_23;
    POcp10_323 = RLcp10_311+RLcp10_319+RLcp10_320+RLcp10_323+q[4];
    JTcp10_123_1 = -(RLcp10_211+RLcp10_219+RLcp10_22+RLcp10_220+RLcp10_223+RLcp10_23);
    JTcp10_223_1 = RLcp10_111+RLcp10_119+RLcp10_12+RLcp10_120+RLcp10_123+RLcp10_13;
    JTcp10_123_5 = -(RLcp10_211+RLcp10_219+RLcp10_220+RLcp10_223);
    JTcp10_223_5 = RLcp10_111+RLcp10_119+RLcp10_120+RLcp10_123;
    JTcp10_123_6 = S1p5*(RLcp10_311+RLcp10_319+RLcp10_320+RLcp10_323);
    JTcp10_223_6 = -C1p5*(RLcp10_311+RLcp10_319+RLcp10_320+RLcp10_323);
    JTcp10_323_6 = C1p5*(RLcp10_211+RLcp10_219+RLcp10_220+RLcp10_223)-S1p5*(RLcp10_111+RLcp10_119)-S1p5*(RLcp10_120+
 RLcp10_123);
    JTcp10_123_7 = ROcp10_56*(RLcp10_311+RLcp10_319+RLcp10_320+RLcp10_323)-S6*(RLcp10_211+RLcp10_219)-S6*(RLcp10_220+
 RLcp10_223);
    JTcp10_223_7 = -(ROcp10_46*(RLcp10_311+RLcp10_319+RLcp10_320+RLcp10_323)-S6*(RLcp10_111+RLcp10_119)-S6*(RLcp10_120+
 RLcp10_123));
    JTcp10_323_7 = ROcp10_46*(RLcp10_211+RLcp10_219+RLcp10_220+RLcp10_223)-ROcp10_56*(RLcp10_111+RLcp10_119)-ROcp10_56*(
 RLcp10_120+RLcp10_123);
    JTcp10_123_8 = ROcp10_56*(RLcp10_319+RLcp10_320)-S6*(RLcp10_219+RLcp10_220)-RLcp10_223*S6+RLcp10_323*ROcp10_56;
    JTcp10_223_8 = RLcp10_123*S6-RLcp10_323*ROcp10_46-ROcp10_46*(RLcp10_319+RLcp10_320)+S6*(RLcp10_119+RLcp10_120);
    JTcp10_323_8 = ROcp10_46*(RLcp10_219+RLcp10_220)-ROcp10_56*(RLcp10_119+RLcp10_120)-RLcp10_123*ROcp10_56+RLcp10_223*
 ROcp10_46;
    JTcp10_123_9 = ROcp10_211*(RLcp10_320+RLcp10_323)-ROcp10_311*(RLcp10_220+RLcp10_223);
    JTcp10_223_9 = -(ROcp10_111*(RLcp10_320+RLcp10_323)-ROcp10_311*(RLcp10_120+RLcp10_123));
    JTcp10_323_9 = ROcp10_111*(RLcp10_220+RLcp10_223)-ROcp10_211*(RLcp10_120+RLcp10_123);
    JTcp10_123_10 = -(RLcp10_223*ROcp10_311-RLcp10_323*ROcp10_211);
    JTcp10_223_10 = RLcp10_123*ROcp10_311-RLcp10_323*ROcp10_111;
    JTcp10_323_10 = -(RLcp10_123*ROcp10_211-RLcp10_223*ROcp10_111);
    JTcp10_123_11 = -(RLcp10_223*ROcp10_620-RLcp10_323*ROcp10_520);
    JTcp10_223_11 = RLcp10_123*ROcp10_620-RLcp10_323*ROcp10_420;
    JTcp10_323_11 = -(RLcp10_123*ROcp10_520-RLcp10_223*ROcp10_420);
    JTcp10_123_12 = -(RLcp10_223*ROcp10_921-RLcp10_323*ROcp10_821);
    JTcp10_223_12 = RLcp10_123*ROcp10_921-RLcp10_323*ROcp10_721;
    JTcp10_323_12 = -(RLcp10_123*ROcp10_821-RLcp10_223*ROcp10_721);
    ORcp10_123 = OMcp10_222*RLcp10_323-OMcp10_322*RLcp10_223;
    ORcp10_223 = -(OMcp10_122*RLcp10_323-OMcp10_322*RLcp10_123);
    ORcp10_323 = OMcp10_122*RLcp10_223-OMcp10_222*RLcp10_123;
    VIcp10_123 = ORcp10_111+ORcp10_119+ORcp10_12+ORcp10_120+ORcp10_123+ORcp10_13+qd[2]*C1-qd[3]*S1;
    VIcp10_223 = ORcp10_211+ORcp10_219+ORcp10_22+ORcp10_220+ORcp10_223+ORcp10_23+qd[2]*S1+qd[3]*C1;
    VIcp10_323 = ORcp10_311+ORcp10_319+ORcp10_320+ORcp10_323+qd[4];
    ACcp10_123 = OMcp10_219*ORcp10_320+OMcp10_222*ORcp10_323+OMcp10_27*(ORcp10_311+ORcp10_319)-OMcp10_319*ORcp10_220-
 OMcp10_322*ORcp10_223-OMcp10_37*ORcp10_211-OMcp10_37*ORcp10_219+OPcp10_219*RLcp10_320+OPcp10_222*RLcp10_323+OPcp10_27*
 RLcp10_311+OPcp10_27*RLcp10_319-OPcp10_319*RLcp10_220-OPcp10_322*RLcp10_223-OPcp10_37*RLcp10_211-OPcp10_37*RLcp10_219-
 ORcp10_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp10_22+(2.0)*qd[2]*S1);
    ACcp10_223 = -(OMcp10_119*ORcp10_320+OMcp10_122*ORcp10_323+OMcp10_17*(ORcp10_311+ORcp10_319)-OMcp10_319*ORcp10_120-
 OMcp10_322*ORcp10_123-OMcp10_37*ORcp10_111-OMcp10_37*ORcp10_119+OPcp10_119*RLcp10_320+OPcp10_122*RLcp10_323+OPcp10_17*
 RLcp10_311+OPcp10_17*RLcp10_319-OPcp10_319*RLcp10_120-OPcp10_322*RLcp10_123-OPcp10_37*RLcp10_111-OPcp10_37*RLcp10_119-qdd[2]
 *S1-qdd[3]*C1-qd[1]*(ORcp10_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp10_13-(2.0)*qd[3]*S1));
    ACcp10_323 = qdd[4]+OMcp10_119*ORcp10_220+OMcp10_122*ORcp10_223+OMcp10_17*ORcp10_211+OMcp10_17*ORcp10_219-OMcp10_219*
 ORcp10_120-OMcp10_222*ORcp10_123-OMcp10_27*ORcp10_111-OMcp10_27*ORcp10_119+OPcp10_119*RLcp10_220+OPcp10_122*RLcp10_223+
 OPcp10_17*RLcp10_211+OPcp10_17*RLcp10_219-OPcp10_219*RLcp10_120-OPcp10_222*RLcp10_123-OPcp10_27*RLcp10_111-OPcp10_27*
 RLcp10_119;
    OMcp10_124 = OMcp10_122+ROcp10_423*qd[24];
    OMcp10_224 = OMcp10_222+ROcp10_523*qd[24];
    OMcp10_324 = OMcp10_322+ROcp10_623*qd[24];
    OPcp10_124 = OPcp10_122+ROcp10_423*qdd[24]+qd[24]*(OMcp10_222*ROcp10_623-OMcp10_322*ROcp10_523);
    OPcp10_224 = OPcp10_222+ROcp10_523*qdd[24]-qd[24]*(OMcp10_122*ROcp10_623-OMcp10_322*ROcp10_423);
    OPcp10_324 = OPcp10_322+ROcp10_623*qdd[24]+qd[24]*(OMcp10_122*ROcp10_523-OMcp10_222*ROcp10_423);

// = = Block_1_0_0_11_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp10_123;
    sens->P[2] = POcp10_223;
    sens->P[3] = POcp10_323;
    sens->R[1][1] = ROcp10_124;
    sens->R[1][2] = ROcp10_224;
    sens->R[1][3] = ROcp10_324;
    sens->R[2][1] = ROcp10_423;
    sens->R[2][2] = ROcp10_523;
    sens->R[2][3] = ROcp10_623;
    sens->R[3][1] = ROcp10_724;
    sens->R[3][2] = ROcp10_824;
    sens->R[3][3] = ROcp10_924;
    sens->V[1] = VIcp10_123;
    sens->V[2] = VIcp10_223;
    sens->V[3] = VIcp10_323;
    sens->OM[1] = OMcp10_124;
    sens->OM[2] = OMcp10_224;
    sens->OM[3] = OMcp10_324;
    sens->J[1][1] = JTcp10_123_1;
    sens->J[1][2] = C1;
    sens->J[1][3] = -S1;
    sens->J[1][5] = JTcp10_123_5;
    sens->J[1][6] = JTcp10_123_6;
    sens->J[1][7] = JTcp10_123_7;
    sens->J[1][11] = JTcp10_123_8;
    sens->J[1][19] = JTcp10_123_9;
    sens->J[1][20] = JTcp10_123_10;
    sens->J[1][21] = JTcp10_123_11;
    sens->J[1][22] = JTcp10_123_12;
    sens->J[2][1] = JTcp10_223_1;
    sens->J[2][2] = S1;
    sens->J[2][3] = C1;
    sens->J[2][5] = JTcp10_223_5;
    sens->J[2][6] = JTcp10_223_6;
    sens->J[2][7] = JTcp10_223_7;
    sens->J[2][11] = JTcp10_223_8;
    sens->J[2][19] = JTcp10_223_9;
    sens->J[2][20] = JTcp10_223_10;
    sens->J[2][21] = JTcp10_223_11;
    sens->J[2][22] = JTcp10_223_12;
    sens->J[3][4] = (1.0);
    sens->J[3][6] = JTcp10_323_6;
    sens->J[3][7] = JTcp10_323_7;
    sens->J[3][11] = JTcp10_323_8;
    sens->J[3][19] = JTcp10_323_9;
    sens->J[3][20] = JTcp10_323_10;
    sens->J[3][21] = JTcp10_323_11;
    sens->J[3][22] = JTcp10_323_12;
    sens->J[4][6] = C1p5;
    sens->J[4][7] = ROcp10_46;
    sens->J[4][11] = ROcp10_46;
    sens->J[4][19] = ROcp10_111;
    sens->J[4][20] = ROcp10_111;
    sens->J[4][21] = ROcp10_420;
    sens->J[4][22] = ROcp10_721;
    sens->J[4][23] = ROcp10_122;
    sens->J[4][24] = ROcp10_423;
    sens->J[5][6] = S1p5;
    sens->J[5][7] = ROcp10_56;
    sens->J[5][11] = ROcp10_56;
    sens->J[5][19] = ROcp10_211;
    sens->J[5][20] = ROcp10_211;
    sens->J[5][21] = ROcp10_520;
    sens->J[5][22] = ROcp10_821;
    sens->J[5][23] = ROcp10_222;
    sens->J[5][24] = ROcp10_523;
    sens->J[6][1] = (1.0);
    sens->J[6][5] = (1.0);
    sens->J[6][7] = S6;
    sens->J[6][11] = S6;
    sens->J[6][19] = ROcp10_311;
    sens->J[6][20] = ROcp10_311;
    sens->J[6][21] = ROcp10_620;
    sens->J[6][22] = ROcp10_921;
    sens->J[6][23] = ROcp10_322;
    sens->J[6][24] = ROcp10_623;
    sens->A[1] = ACcp10_123;
    sens->A[2] = ACcp10_223;
    sens->A[3] = ACcp10_323;
    sens->OMP[1] = OPcp10_124;
    sens->OMP[2] = OPcp10_224;
    sens->OMP[3] = OPcp10_324;
 
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

// = = Block_1_0_0_13_0_5 = = 
 
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
    OMcp12_113 = OMcp12_17+ROcp12_111*qd[13];
    OMcp12_213 = OMcp12_27+ROcp12_211*qd[13];
    OMcp12_313 = OMcp12_37+ROcp12_311*qd[13];
    ORcp12_113 = OMcp12_27*RLcp12_313-OMcp12_37*RLcp12_213;
    ORcp12_213 = -(OMcp12_17*RLcp12_313-OMcp12_37*RLcp12_113);
    ORcp12_313 = OMcp12_17*RLcp12_213-OMcp12_27*RLcp12_113;
    OPcp12_113 = OPcp12_17+ROcp12_111*qdd[13]+qd[13]*(OMcp12_27*ROcp12_311-OMcp12_37*ROcp12_211);
    OPcp12_213 = OPcp12_27+ROcp12_211*qdd[13]-qd[13]*(OMcp12_17*ROcp12_311-OMcp12_37*ROcp12_111);
    OPcp12_313 = OPcp12_37+ROcp12_311*qdd[13]+qd[13]*(OMcp12_17*ROcp12_211-OMcp12_27*ROcp12_111);
    RLcp12_114 = ROcp12_413*s->dpt[2][17];
    RLcp12_214 = ROcp12_513*s->dpt[2][17];
    RLcp12_314 = ROcp12_613*s->dpt[2][17];
    OMcp12_114 = OMcp12_113+ROcp12_111*qd[14];
    OMcp12_214 = OMcp12_213+ROcp12_211*qd[14];
    OMcp12_314 = OMcp12_313+ROcp12_311*qd[14];
    ORcp12_114 = OMcp12_213*RLcp12_314-OMcp12_313*RLcp12_214;
    ORcp12_214 = -(OMcp12_113*RLcp12_314-OMcp12_313*RLcp12_114);
    ORcp12_314 = OMcp12_113*RLcp12_214-OMcp12_213*RLcp12_114;
    OMcp12_115 = OMcp12_114+ROcp12_414*qd[15];
    OMcp12_215 = OMcp12_214+ROcp12_514*qd[15];
    OMcp12_315 = OMcp12_314+ROcp12_614*qd[15];
    OMcp12_116 = OMcp12_115+ROcp12_715*qd[16];
    OMcp12_216 = OMcp12_215+ROcp12_815*qd[16];
    OMcp12_316 = OMcp12_315+ROcp12_915*qd[16];
    OPcp12_116 = OPcp12_113+ROcp12_111*qdd[14]+ROcp12_414*qdd[15]+ROcp12_715*qdd[16]+qd[14]*(OMcp12_213*ROcp12_311-
 OMcp12_313*ROcp12_211)+qd[15]*(OMcp12_214*ROcp12_614-OMcp12_314*ROcp12_514)+qd[16]*(OMcp12_215*ROcp12_915-OMcp12_315*
 ROcp12_815);
    OPcp12_216 = OPcp12_213+ROcp12_211*qdd[14]+ROcp12_514*qdd[15]+ROcp12_815*qdd[16]-qd[14]*(OMcp12_113*ROcp12_311-
 OMcp12_313*ROcp12_111)-qd[15]*(OMcp12_114*ROcp12_614-OMcp12_314*ROcp12_414)-qd[16]*(OMcp12_115*ROcp12_915-OMcp12_315*
 ROcp12_715);
    OPcp12_316 = OPcp12_313+ROcp12_311*qdd[14]+ROcp12_614*qdd[15]+ROcp12_915*qdd[16]+qd[14]*(OMcp12_113*ROcp12_211-
 OMcp12_213*ROcp12_111)+qd[15]*(OMcp12_114*ROcp12_514-OMcp12_214*ROcp12_414)+qd[16]*(OMcp12_115*ROcp12_815-OMcp12_215*
 ROcp12_715);
    RLcp12_117 = ROcp12_116*s->dpt[1][21]+ROcp12_416*s->dpt[2][21]+ROcp12_715*s->dpt[3][21];
    RLcp12_217 = ROcp12_216*s->dpt[1][21]+ROcp12_516*s->dpt[2][21]+ROcp12_815*s->dpt[3][21];
    RLcp12_317 = ROcp12_316*s->dpt[1][21]+ROcp12_616*s->dpt[2][21]+ROcp12_915*s->dpt[3][21];
    POcp12_117 = RLcp12_111+RLcp12_113+RLcp12_114+RLcp12_117+RLcp12_12+RLcp12_13;
    POcp12_217 = RLcp12_211+RLcp12_213+RLcp12_214+RLcp12_217+RLcp12_22+RLcp12_23;
    POcp12_317 = RLcp12_311+RLcp12_313+RLcp12_314+RLcp12_317+q[4];
    ORcp12_117 = OMcp12_216*RLcp12_317-OMcp12_316*RLcp12_217;
    ORcp12_217 = -(OMcp12_116*RLcp12_317-OMcp12_316*RLcp12_117);
    ORcp12_317 = OMcp12_116*RLcp12_217-OMcp12_216*RLcp12_117;
    VIcp12_117 = ORcp12_111+ORcp12_113+ORcp12_114+ORcp12_117+ORcp12_12+ORcp12_13+qd[2]*C1-qd[3]*S1;
    VIcp12_217 = ORcp12_211+ORcp12_213+ORcp12_214+ORcp12_217+ORcp12_22+ORcp12_23+qd[2]*S1+qd[3]*C1;
    VIcp12_317 = ORcp12_311+ORcp12_313+ORcp12_314+ORcp12_317+qd[4];
    ACcp12_117 = OMcp12_213*ORcp12_314+OMcp12_216*ORcp12_317+OMcp12_27*(ORcp12_311+ORcp12_313)-OMcp12_313*ORcp12_214-
 OMcp12_316*ORcp12_217-OMcp12_37*ORcp12_211-OMcp12_37*ORcp12_213+OPcp12_213*RLcp12_314+OPcp12_216*RLcp12_317+OPcp12_27*
 RLcp12_311+OPcp12_27*RLcp12_313-OPcp12_313*RLcp12_214-OPcp12_316*RLcp12_217-OPcp12_37*RLcp12_211-OPcp12_37*RLcp12_213-
 ORcp12_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp12_22+(2.0)*qd[2]*S1);
    ACcp12_217 = -(OMcp12_113*ORcp12_314+OMcp12_116*ORcp12_317+OMcp12_17*(ORcp12_311+ORcp12_313)-OMcp12_313*ORcp12_114-
 OMcp12_316*ORcp12_117-OMcp12_37*ORcp12_111-OMcp12_37*ORcp12_113+OPcp12_113*RLcp12_314+OPcp12_116*RLcp12_317+OPcp12_17*
 RLcp12_311+OPcp12_17*RLcp12_313-OPcp12_313*RLcp12_114-OPcp12_316*RLcp12_117-OPcp12_37*RLcp12_111-OPcp12_37*RLcp12_113-qdd[2]
 *S1-qdd[3]*C1-qd[1]*(ORcp12_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp12_13-(2.0)*qd[3]*S1));
    ACcp12_317 = qdd[4]+OMcp12_113*ORcp12_214+OMcp12_116*ORcp12_217+OMcp12_17*ORcp12_211+OMcp12_17*ORcp12_213-OMcp12_213*
 ORcp12_114-OMcp12_216*ORcp12_117-OMcp12_27*ORcp12_111-OMcp12_27*ORcp12_113+OPcp12_113*RLcp12_214+OPcp12_116*RLcp12_217+
 OPcp12_17*RLcp12_211+OPcp12_17*RLcp12_213-OPcp12_213*RLcp12_114-OPcp12_216*RLcp12_117-OPcp12_27*RLcp12_111-OPcp12_27*
 RLcp12_113;
    OMcp12_118 = OMcp12_116+ROcp12_417*qd[18];
    OMcp12_218 = OMcp12_216+ROcp12_517*qd[18];
    OMcp12_318 = OMcp12_316+ROcp12_617*qd[18];
    OPcp12_118 = OPcp12_116+ROcp12_417*qdd[18]+qd[18]*(OMcp12_216*ROcp12_617-OMcp12_316*ROcp12_517);
    OPcp12_218 = OPcp12_216+ROcp12_517*qdd[18]-qd[18]*(OMcp12_116*ROcp12_617-OMcp12_316*ROcp12_417);
    OPcp12_318 = OPcp12_316+ROcp12_617*qdd[18]+qd[18]*(OMcp12_116*ROcp12_517-OMcp12_216*ROcp12_417);

// = = Block_1_0_0_13_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp12_117;
    sens->P[2] = POcp12_217;
    sens->P[3] = POcp12_317;
    sens->R[1][1] = ROcp12_118;
    sens->R[1][2] = ROcp12_218;
    sens->R[1][3] = ROcp12_318;
    sens->R[2][1] = ROcp12_417;
    sens->R[2][2] = ROcp12_517;
    sens->R[2][3] = ROcp12_617;
    sens->R[3][1] = ROcp12_718;
    sens->R[3][2] = ROcp12_818;
    sens->R[3][3] = ROcp12_918;
    sens->V[1] = VIcp12_117;
    sens->V[2] = VIcp12_217;
    sens->V[3] = VIcp12_317;
    sens->OM[1] = OMcp12_118;
    sens->OM[2] = OMcp12_218;
    sens->OM[3] = OMcp12_318;
    sens->A[1] = ACcp12_117;
    sens->A[2] = ACcp12_217;
    sens->A[3] = ACcp12_317;
    sens->OMP[1] = OPcp12_118;
    sens->OMP[2] = OPcp12_218;
    sens->OMP[3] = OPcp12_318;
 
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

// = = Block_1_0_0_14_0_6 = = 
 
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
    OMcp13_119 = OMcp13_17+ROcp13_111*qd[19];
    OMcp13_219 = OMcp13_27+ROcp13_211*qd[19];
    OMcp13_319 = OMcp13_37+ROcp13_311*qd[19];
    ORcp13_119 = OMcp13_27*RLcp13_319-OMcp13_37*RLcp13_219;
    ORcp13_219 = -(OMcp13_17*RLcp13_319-OMcp13_37*RLcp13_119);
    ORcp13_319 = OMcp13_17*RLcp13_219-OMcp13_27*RLcp13_119;
    OPcp13_119 = OPcp13_17+ROcp13_111*qdd[19]+qd[19]*(OMcp13_27*ROcp13_311-OMcp13_37*ROcp13_211);
    OPcp13_219 = OPcp13_27+ROcp13_211*qdd[19]-qd[19]*(OMcp13_17*ROcp13_311-OMcp13_37*ROcp13_111);
    OPcp13_319 = OPcp13_37+ROcp13_311*qdd[19]+qd[19]*(OMcp13_17*ROcp13_211-OMcp13_27*ROcp13_111);
    RLcp13_120 = ROcp13_419*s->dpt[2][24];
    RLcp13_220 = ROcp13_519*s->dpt[2][24];
    RLcp13_320 = ROcp13_619*s->dpt[2][24];
    OMcp13_120 = OMcp13_119+ROcp13_111*qd[20];
    OMcp13_220 = OMcp13_219+ROcp13_211*qd[20];
    OMcp13_320 = OMcp13_319+ROcp13_311*qd[20];
    ORcp13_120 = OMcp13_219*RLcp13_320-OMcp13_319*RLcp13_220;
    ORcp13_220 = -(OMcp13_119*RLcp13_320-OMcp13_319*RLcp13_120);
    ORcp13_320 = OMcp13_119*RLcp13_220-OMcp13_219*RLcp13_120;
    OMcp13_121 = OMcp13_120+ROcp13_420*qd[21];
    OMcp13_221 = OMcp13_220+ROcp13_520*qd[21];
    OMcp13_321 = OMcp13_320+ROcp13_620*qd[21];
    OMcp13_122 = OMcp13_121+ROcp13_721*qd[22];
    OMcp13_222 = OMcp13_221+ROcp13_821*qd[22];
    OMcp13_322 = OMcp13_321+ROcp13_921*qd[22];
    OPcp13_122 = OPcp13_119+ROcp13_111*qdd[20]+ROcp13_420*qdd[21]+ROcp13_721*qdd[22]+qd[20]*(OMcp13_219*ROcp13_311-
 OMcp13_319*ROcp13_211)+qd[21]*(OMcp13_220*ROcp13_620-OMcp13_320*ROcp13_520)+qd[22]*(OMcp13_221*ROcp13_921-OMcp13_321*
 ROcp13_821);
    OPcp13_222 = OPcp13_219+ROcp13_211*qdd[20]+ROcp13_520*qdd[21]+ROcp13_821*qdd[22]-qd[20]*(OMcp13_119*ROcp13_311-
 OMcp13_319*ROcp13_111)-qd[21]*(OMcp13_120*ROcp13_620-OMcp13_320*ROcp13_420)-qd[22]*(OMcp13_121*ROcp13_921-OMcp13_321*
 ROcp13_721);
    OPcp13_322 = OPcp13_319+ROcp13_311*qdd[20]+ROcp13_620*qdd[21]+ROcp13_921*qdd[22]+qd[20]*(OMcp13_119*ROcp13_211-
 OMcp13_219*ROcp13_111)+qd[21]*(OMcp13_120*ROcp13_520-OMcp13_220*ROcp13_420)+qd[22]*(OMcp13_121*ROcp13_821-OMcp13_221*
 ROcp13_721);
    RLcp13_123 = ROcp13_122*s->dpt[1][28]+ROcp13_422*s->dpt[2][28]+ROcp13_721*s->dpt[3][28];
    RLcp13_223 = ROcp13_222*s->dpt[1][28]+ROcp13_522*s->dpt[2][28]+ROcp13_821*s->dpt[3][28];
    RLcp13_323 = ROcp13_322*s->dpt[1][28]+ROcp13_622*s->dpt[2][28]+ROcp13_921*s->dpt[3][28];
    POcp13_123 = RLcp13_111+RLcp13_119+RLcp13_12+RLcp13_120+RLcp13_123+RLcp13_13;
    POcp13_223 = RLcp13_211+RLcp13_219+RLcp13_22+RLcp13_220+RLcp13_223+RLcp13_23;
    POcp13_323 = RLcp13_311+RLcp13_319+RLcp13_320+RLcp13_323+q[4];
    ORcp13_123 = OMcp13_222*RLcp13_323-OMcp13_322*RLcp13_223;
    ORcp13_223 = -(OMcp13_122*RLcp13_323-OMcp13_322*RLcp13_123);
    ORcp13_323 = OMcp13_122*RLcp13_223-OMcp13_222*RLcp13_123;
    VIcp13_123 = ORcp13_111+ORcp13_119+ORcp13_12+ORcp13_120+ORcp13_123+ORcp13_13+qd[2]*C1-qd[3]*S1;
    VIcp13_223 = ORcp13_211+ORcp13_219+ORcp13_22+ORcp13_220+ORcp13_223+ORcp13_23+qd[2]*S1+qd[3]*C1;
    VIcp13_323 = ORcp13_311+ORcp13_319+ORcp13_320+ORcp13_323+qd[4];
    ACcp13_123 = OMcp13_219*ORcp13_320+OMcp13_222*ORcp13_323+OMcp13_27*(ORcp13_311+ORcp13_319)-OMcp13_319*ORcp13_220-
 OMcp13_322*ORcp13_223-OMcp13_37*ORcp13_211-OMcp13_37*ORcp13_219+OPcp13_219*RLcp13_320+OPcp13_222*RLcp13_323+OPcp13_27*
 RLcp13_311+OPcp13_27*RLcp13_319-OPcp13_319*RLcp13_220-OPcp13_322*RLcp13_223-OPcp13_37*RLcp13_211-OPcp13_37*RLcp13_219-
 ORcp13_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp13_22+(2.0)*qd[2]*S1);
    ACcp13_223 = -(OMcp13_119*ORcp13_320+OMcp13_122*ORcp13_323+OMcp13_17*(ORcp13_311+ORcp13_319)-OMcp13_319*ORcp13_120-
 OMcp13_322*ORcp13_123-OMcp13_37*ORcp13_111-OMcp13_37*ORcp13_119+OPcp13_119*RLcp13_320+OPcp13_122*RLcp13_323+OPcp13_17*
 RLcp13_311+OPcp13_17*RLcp13_319-OPcp13_319*RLcp13_120-OPcp13_322*RLcp13_123-OPcp13_37*RLcp13_111-OPcp13_37*RLcp13_119-qdd[2]
 *S1-qdd[3]*C1-qd[1]*(ORcp13_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp13_13-(2.0)*qd[3]*S1));
    ACcp13_323 = qdd[4]+OMcp13_119*ORcp13_220+OMcp13_122*ORcp13_223+OMcp13_17*ORcp13_211+OMcp13_17*ORcp13_219-OMcp13_219*
 ORcp13_120-OMcp13_222*ORcp13_123-OMcp13_27*ORcp13_111-OMcp13_27*ORcp13_119+OPcp13_119*RLcp13_220+OPcp13_122*RLcp13_223+
 OPcp13_17*RLcp13_211+OPcp13_17*RLcp13_219-OPcp13_219*RLcp13_120-OPcp13_222*RLcp13_123-OPcp13_27*RLcp13_111-OPcp13_27*
 RLcp13_119;
    OMcp13_124 = OMcp13_122+ROcp13_423*qd[24];
    OMcp13_224 = OMcp13_222+ROcp13_523*qd[24];
    OMcp13_324 = OMcp13_322+ROcp13_623*qd[24];
    OPcp13_124 = OPcp13_122+ROcp13_423*qdd[24]+qd[24]*(OMcp13_222*ROcp13_623-OMcp13_322*ROcp13_523);
    OPcp13_224 = OPcp13_222+ROcp13_523*qdd[24]-qd[24]*(OMcp13_122*ROcp13_623-OMcp13_322*ROcp13_423);
    OPcp13_324 = OPcp13_322+ROcp13_623*qdd[24]+qd[24]*(OMcp13_122*ROcp13_523-OMcp13_222*ROcp13_423);

// = = Block_1_0_0_14_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp13_123;
    sens->P[2] = POcp13_223;
    sens->P[3] = POcp13_323;
    sens->R[1][1] = ROcp13_124;
    sens->R[1][2] = ROcp13_224;
    sens->R[1][3] = ROcp13_324;
    sens->R[2][1] = ROcp13_423;
    sens->R[2][2] = ROcp13_523;
    sens->R[2][3] = ROcp13_623;
    sens->R[3][1] = ROcp13_724;
    sens->R[3][2] = ROcp13_824;
    sens->R[3][3] = ROcp13_924;
    sens->V[1] = VIcp13_123;
    sens->V[2] = VIcp13_223;
    sens->V[3] = VIcp13_323;
    sens->OM[1] = OMcp13_124;
    sens->OM[2] = OMcp13_224;
    sens->OM[3] = OMcp13_324;
    sens->A[1] = ACcp13_123;
    sens->A[2] = ACcp13_223;
    sens->A[3] = ACcp13_323;
    sens->OMP[1] = OPcp13_124;
    sens->OMP[2] = OPcp13_224;
    sens->OMP[3] = OPcp13_324;

break;
default:
break;
}


// ====== END Task 1 ====== 


}
 


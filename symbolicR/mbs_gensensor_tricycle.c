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
//	==> Function : F 6 : Sensors Kinematical Informations (sens) 
//	==> Flops complexity : 8037
//
//	==> Generation Time :  0.130 seconds
//	==> Post-Processing :  0.170 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
#include "mbs_sensor.h"
 
void  mbs_gensensor(MbsSensor *sens, 
              MbsData *s,
              int isens)
{ 
 
#include "mbs_gensensor_tricycle.h" 
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

// = = Block_0_0_0_0_0_4 = = 
 
// Trigonometric Variables  

  C11 = cos(q[11]);
  S11 = sin(q[11]);

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

// = = Block_0_0_0_0_0_7 = = 
 
// Trigonometric Variables  

  C24 = cos(q[24]);
  S24 = sin(q[24]);
  C25 = cos(q[25]);
  S25 = sin(q[25]);

// = = Block_0_0_0_0_0_8 = = 
 
// Trigonometric Variables  

  C26 = cos(q[26]);
  S26 = sin(q[26]);

// = = Block_0_0_0_0_0_9 = = 
 
// Trigonometric Variables  

  C27 = cos(q[27]);
  S27 = sin(q[27]);
  C28 = cos(q[28]);
  S28 = sin(q[28]);

// = = Block_0_0_0_0_0_10 = = 
 
// Trigonometric Variables  

  C29 = cos(q[29]);
  S29 = sin(q[29]);
  C30 = cos(q[30]);
  S30 = sin(q[30]);

// = = Block_0_0_0_5_0_1 = = 
 
// Trigonometric Variables  

  C1p5 = C1*C5-S1*S5;
  S1p5 = C1*S5+S1*C5;

// = = Block_0_0_0_8_0_2 = = 
 
// Trigonometric Variables  

  S7p8 = C7*S8+S7*C8;
  C7p8 = C7*C8-S7*S8;

// = = Block_0_0_0_9_0_2 = = 
 
// Trigonometric Variables  

  S7p8p9 = C7p8*S9+S7p8*C9;
  C7p8p9 = C7p8*C9-S7p8*S9;

// = = Block_0_0_0_10_0_3 = = 
 
// Trigonometric Variables  

  S10p7 = C10*S7+S10*C7;
  C10p7 = C10*C7-S10*S7;

// = = Block_0_0_0_24_0_7 = = 
 
// Trigonometric Variables  

  S24p7 = C24*S7+S24*C7;
  C24p7 = C24*C7-S24*S7;

// = = Block_0_0_0_26_0_8 = = 
 
// Trigonometric Variables  

  S26p7 = C26*S7+S26*C7;
  C26p7 = C26*C7-S26*S7;

// ====== END Task 0 ====== 

// ===== BEGIN task 1 ===== 
 
switch(isens)
{
 
// 
break;
case 1:
 


// = = Block_1_0_0_1_1_0 = = 
 
// Symbolic Outputs  

    sens->R[1][1] = C1;
    sens->R[1][2] = S1;
    sens->R[2][1] = -S1;
    sens->R[2][2] = C1;
    sens->R[3][3] = (1.0);
    sens->OM[3] = qd[1];
 
// 
break;
case 2:
 


// = = Block_1_0_0_2_0_1 = = 
 
// Sensor Kinematics 


    RLcp1_12 = q[2]*C1;
    RLcp1_22 = q[2]*S1;
    ORcp1_12 = -RLcp1_22*qd[1];
    ORcp1_22 = RLcp1_12*qd[1];
    VIcp1_12 = ORcp1_12+qd[2]*C1;
    VIcp1_22 = ORcp1_22+qd[2]*S1;
    ACcp1_12 = qdd[2]*C1-qd[1]*(ORcp1_22+(2.0)*qd[2]*S1);
    ACcp1_22 = qdd[2]*S1+qd[1]*(ORcp1_12+(2.0)*qd[2]*C1);

// = = Block_1_0_0_2_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = RLcp1_12;
    sens->P[2] = RLcp1_22;
    sens->R[1][1] = C1;
    sens->R[1][2] = S1;
    sens->R[2][1] = -S1;
    sens->R[2][2] = C1;
    sens->R[3][3] = (1.0);
    sens->V[1] = VIcp1_12;
    sens->V[2] = VIcp1_22;
    sens->OM[3] = qd[1];
    sens->A[1] = ACcp1_12;
    sens->A[2] = ACcp1_22;
 
// 
break;
case 3:
 


// = = Block_1_0_0_3_0_1 = = 
 
// Sensor Kinematics 


    RLcp2_12 = q[2]*C1;
    RLcp2_22 = q[2]*S1;
    ORcp2_12 = -RLcp2_22*qd[1];
    ORcp2_22 = RLcp2_12*qd[1];
    RLcp2_13 = -q[3]*S1;
    RLcp2_23 = q[3]*C1;
    POcp2_13 = RLcp2_12+RLcp2_13;
    POcp2_23 = RLcp2_22+RLcp2_23;
    ORcp2_13 = -RLcp2_23*qd[1];
    ORcp2_23 = RLcp2_13*qd[1];
    VIcp2_13 = ORcp2_12+ORcp2_13+qd[2]*C1-qd[3]*S1;
    VIcp2_23 = ORcp2_22+ORcp2_23+qd[2]*S1+qd[3]*C1;
    ACcp2_13 = -(ORcp2_23*qd[1]-qdd[2]*C1+qdd[3]*S1+(2.0)*qd[1]*qd[3]*C1+qd[1]*(ORcp2_22+(2.0)*qd[2]*S1));
    ACcp2_23 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp2_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp2_13-(2.0)*qd[3]*S1);

// = = Block_1_0_0_3_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp2_13;
    sens->P[2] = POcp2_23;
    sens->R[1][1] = C1;
    sens->R[1][2] = S1;
    sens->R[2][1] = -S1;
    sens->R[2][2] = C1;
    sens->R[3][3] = (1.0);
    sens->V[1] = VIcp2_13;
    sens->V[2] = VIcp2_23;
    sens->OM[3] = qd[1];
    sens->A[1] = ACcp2_13;
    sens->A[2] = ACcp2_23;
 
// 
break;
case 4:
 


// = = Block_1_0_0_4_0_1 = = 
 
// Sensor Kinematics 


    RLcp3_12 = q[2]*C1;
    RLcp3_22 = q[2]*S1;
    ORcp3_12 = -RLcp3_22*qd[1];
    ORcp3_22 = RLcp3_12*qd[1];
    RLcp3_13 = -q[3]*S1;
    RLcp3_23 = q[3]*C1;
    POcp3_13 = RLcp3_12+RLcp3_13;
    POcp3_23 = RLcp3_22+RLcp3_23;
    ORcp3_13 = -RLcp3_23*qd[1];
    ORcp3_23 = RLcp3_13*qd[1];
    VIcp3_13 = ORcp3_12+ORcp3_13+qd[2]*C1-qd[3]*S1;
    VIcp3_23 = ORcp3_22+ORcp3_23+qd[2]*S1+qd[3]*C1;
    ACcp3_13 = -(ORcp3_23*qd[1]-qdd[2]*C1+qdd[3]*S1+(2.0)*qd[1]*qd[3]*C1+qd[1]*(ORcp3_22+(2.0)*qd[2]*S1));
    ACcp3_23 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp3_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp3_13-(2.0)*qd[3]*S1);

// = = Block_1_0_0_4_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp3_13;
    sens->P[2] = POcp3_23;
    sens->P[3] = q[4];
    sens->R[1][1] = C1;
    sens->R[1][2] = S1;
    sens->R[2][1] = -S1;
    sens->R[2][2] = C1;
    sens->R[3][3] = (1.0);
    sens->V[1] = VIcp3_13;
    sens->V[2] = VIcp3_23;
    sens->V[3] = qd[4];
    sens->OM[3] = qd[1];
    sens->A[1] = ACcp3_13;
    sens->A[2] = ACcp3_23;
    sens->A[3] = qdd[4];
 
// 
break;
case 5:
 


// = = Block_1_0_0_5_0_1 = = 
 
// Sensor Kinematics 


    RLcp4_12 = q[2]*C1;
    RLcp4_22 = q[2]*S1;
    ORcp4_12 = -RLcp4_22*qd[1];
    ORcp4_22 = RLcp4_12*qd[1];
    RLcp4_13 = -q[3]*S1;
    RLcp4_23 = q[3]*C1;
    POcp4_13 = RLcp4_12+RLcp4_13;
    POcp4_23 = RLcp4_22+RLcp4_23;
    ORcp4_13 = -RLcp4_23*qd[1];
    ORcp4_23 = RLcp4_13*qd[1];
    VIcp4_13 = ORcp4_12+ORcp4_13+qd[2]*C1-qd[3]*S1;
    VIcp4_23 = ORcp4_22+ORcp4_23+qd[2]*S1+qd[3]*C1;
    ACcp4_13 = -(ORcp4_23*qd[1]-qdd[2]*C1+qdd[3]*S1+(2.0)*qd[1]*qd[3]*C1+qd[1]*(ORcp4_22+(2.0)*qd[2]*S1));
    ACcp4_23 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp4_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp4_13-(2.0)*qd[3]*S1);
    OMcp4_35 = qd[1]+qd[5];

// = = Block_1_0_0_5_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp4_13;
    sens->P[2] = POcp4_23;
    sens->P[3] = q[4];
    sens->R[1][1] = C1p5;
    sens->R[1][2] = S1p5;
    sens->R[2][1] = -S1p5;
    sens->R[2][2] = C1p5;
    sens->R[3][3] = (1.0);
    sens->V[1] = VIcp4_13;
    sens->V[2] = VIcp4_23;
    sens->V[3] = qd[4];
    sens->OM[3] = OMcp4_35;
    sens->A[1] = ACcp4_13;
    sens->A[2] = ACcp4_23;
    sens->A[3] = qdd[4];
    sens->OMP[3] = qdd[5];
 
// 
break;
case 6:
 


// = = Block_1_0_0_6_0_1 = = 
 
// Sensor Kinematics 


    ROcp5_46 = -S1p5*C6;
    ROcp5_56 = C1p5*C6;
    ROcp5_76 = S1p5*S6;
    ROcp5_86 = -C1p5*S6;
    RLcp5_12 = q[2]*C1;
    RLcp5_22 = q[2]*S1;
    ORcp5_12 = -RLcp5_22*qd[1];
    ORcp5_22 = RLcp5_12*qd[1];
    RLcp5_13 = -q[3]*S1;
    RLcp5_23 = q[3]*C1;
    POcp5_13 = RLcp5_12+RLcp5_13;
    POcp5_23 = RLcp5_22+RLcp5_23;
    ORcp5_13 = -RLcp5_23*qd[1];
    ORcp5_23 = RLcp5_13*qd[1];
    VIcp5_13 = ORcp5_12+ORcp5_13+qd[2]*C1-qd[3]*S1;
    VIcp5_23 = ORcp5_22+ORcp5_23+qd[2]*S1+qd[3]*C1;
    ACcp5_13 = -(ORcp5_23*qd[1]-qdd[2]*C1+qdd[3]*S1+(2.0)*qd[1]*qd[3]*C1+qd[1]*(ORcp5_22+(2.0)*qd[2]*S1));
    ACcp5_23 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp5_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp5_13-(2.0)*qd[3]*S1);
    OMcp5_35 = qd[1]+qd[5];
    OMcp5_16 = qd[6]*C1p5;
    OMcp5_26 = qd[6]*S1p5;
    OPcp5_16 = -(OMcp5_35*qd[6]*S1p5-qdd[6]*C1p5);
    OPcp5_26 = OMcp5_35*qd[6]*C1p5+qdd[6]*S1p5;

// = = Block_1_0_0_6_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp5_13;
    sens->P[2] = POcp5_23;
    sens->P[3] = q[4];
    sens->R[1][1] = C1p5;
    sens->R[1][2] = S1p5;
    sens->R[2][1] = ROcp5_46;
    sens->R[2][2] = ROcp5_56;
    sens->R[2][3] = S6;
    sens->R[3][1] = ROcp5_76;
    sens->R[3][2] = ROcp5_86;
    sens->R[3][3] = C6;
    sens->V[1] = VIcp5_13;
    sens->V[2] = VIcp5_23;
    sens->V[3] = qd[4];
    sens->OM[1] = OMcp5_16;
    sens->OM[2] = OMcp5_26;
    sens->OM[3] = OMcp5_35;
    sens->A[1] = ACcp5_13;
    sens->A[2] = ACcp5_23;
    sens->A[3] = qdd[4];
    sens->OMP[1] = OPcp5_16;
    sens->OMP[2] = OPcp5_26;
    sens->OMP[3] = qdd[5];
 
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
    ROcp6_37 = -C6*S7;
    ROcp6_77 = ROcp6_76*C7+C1p5*S7;
    ROcp6_87 = ROcp6_86*C7+S1p5*S7;
    ROcp6_97 = C6*C7;
    RLcp6_12 = q[2]*C1;
    RLcp6_22 = q[2]*S1;
    ORcp6_12 = -RLcp6_22*qd[1];
    ORcp6_22 = RLcp6_12*qd[1];
    RLcp6_13 = -q[3]*S1;
    RLcp6_23 = q[3]*C1;
    POcp6_13 = RLcp6_12+RLcp6_13;
    POcp6_23 = RLcp6_22+RLcp6_23;
    ORcp6_13 = -RLcp6_23*qd[1];
    ORcp6_23 = RLcp6_13*qd[1];
    VIcp6_13 = ORcp6_12+ORcp6_13+qd[2]*C1-qd[3]*S1;
    VIcp6_23 = ORcp6_22+ORcp6_23+qd[2]*S1+qd[3]*C1;
    ACcp6_13 = -(ORcp6_23*qd[1]-qdd[2]*C1+qdd[3]*S1+(2.0)*qd[1]*qd[3]*C1+qd[1]*(ORcp6_22+(2.0)*qd[2]*S1));
    ACcp6_23 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp6_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp6_13-(2.0)*qd[3]*S1);
    OMcp6_35 = qd[1]+qd[5];
    OMcp6_16 = qd[6]*C1p5;
    OMcp6_26 = qd[6]*S1p5;
    OMcp6_17 = OMcp6_16+ROcp6_46*qd[7];
    OMcp6_27 = OMcp6_26+ROcp6_56*qd[7];
    OMcp6_37 = OMcp6_35+qd[7]*S6;
    OPcp6_17 = -(OMcp6_35*qd[6]*S1p5-ROcp6_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp6_26*S6-OMcp6_35*ROcp6_56));
    OPcp6_27 = OMcp6_35*qd[6]*C1p5+ROcp6_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp6_16*S6-OMcp6_35*ROcp6_46);
    OPcp6_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_7_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp6_13;
    sens->P[2] = POcp6_23;
    sens->P[3] = q[4];
    sens->R[1][1] = ROcp6_17;
    sens->R[1][2] = ROcp6_27;
    sens->R[1][3] = ROcp6_37;
    sens->R[2][1] = ROcp6_46;
    sens->R[2][2] = ROcp6_56;
    sens->R[2][3] = S6;
    sens->R[3][1] = ROcp6_77;
    sens->R[3][2] = ROcp6_87;
    sens->R[3][3] = ROcp6_97;
    sens->V[1] = VIcp6_13;
    sens->V[2] = VIcp6_23;
    sens->V[3] = qd[4];
    sens->OM[1] = OMcp6_17;
    sens->OM[2] = OMcp6_27;
    sens->OM[3] = OMcp6_37;
    sens->A[1] = ACcp6_13;
    sens->A[2] = ACcp6_23;
    sens->A[3] = qdd[4];
    sens->OMP[1] = OPcp6_17;
    sens->OMP[2] = OPcp6_27;
    sens->OMP[3] = OPcp6_37;
 
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

// = = Block_1_0_0_8_0_2 = = 
 
// Sensor Kinematics 


    ROcp7_18 = ROcp7_17*C8-ROcp7_77*S8;
    ROcp7_28 = ROcp7_27*C8-ROcp7_87*S8;
    ROcp7_38 = -S7p8*C6;
    ROcp7_78 = ROcp7_17*S8+ROcp7_77*C8;
    ROcp7_88 = ROcp7_27*S8+ROcp7_87*C8;
    ROcp7_98 = C7p8*C6;
    RLcp7_18 = ROcp7_17*s->dpt[1][2]+ROcp7_77*s->dpt[3][2];
    RLcp7_28 = ROcp7_27*s->dpt[1][2]+ROcp7_87*s->dpt[3][2];
    RLcp7_38 = -C6*(s->dpt[1][2]*S7-s->dpt[3][2]*C7);
    POcp7_18 = RLcp7_12+RLcp7_13+RLcp7_18;
    POcp7_28 = RLcp7_22+RLcp7_23+RLcp7_28;
    POcp7_38 = RLcp7_38+q[4];
    OMcp7_18 = OMcp7_17+ROcp7_46*qd[8];
    OMcp7_28 = OMcp7_27+ROcp7_56*qd[8];
    OMcp7_38 = OMcp7_37+qd[8]*S6;
    ORcp7_18 = OMcp7_27*RLcp7_38-OMcp7_37*RLcp7_28;
    ORcp7_28 = -(OMcp7_17*RLcp7_38-OMcp7_37*RLcp7_18);
    ORcp7_38 = OMcp7_17*RLcp7_28-OMcp7_27*RLcp7_18;
    VIcp7_18 = ORcp7_12+ORcp7_13+ORcp7_18+qd[2]*C1-qd[3]*S1;
    VIcp7_28 = ORcp7_22+ORcp7_23+ORcp7_28+qd[2]*S1+qd[3]*C1;
    VIcp7_38 = ORcp7_38+qd[4];
    OPcp7_18 = OPcp7_17+ROcp7_46*qdd[8]+qd[8]*(OMcp7_27*S6-OMcp7_37*ROcp7_56);
    OPcp7_28 = OPcp7_27+ROcp7_56*qdd[8]-qd[8]*(OMcp7_17*S6-OMcp7_37*ROcp7_46);
    OPcp7_38 = OPcp7_37+qdd[8]*S6+qd[8]*(OMcp7_17*ROcp7_56-OMcp7_27*ROcp7_46);
    ACcp7_18 = OMcp7_27*ORcp7_38-OMcp7_37*ORcp7_28+OPcp7_27*RLcp7_38-OPcp7_37*RLcp7_28-ORcp7_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)
 *qd[1]*qd[3]*C1-qd[1]*(ORcp7_22+(2.0)*qd[2]*S1);
    ACcp7_28 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp7_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp7_13-(2.0)*qd[3]*S1)-OMcp7_17*ORcp7_38+OMcp7_37*
 ORcp7_18-OPcp7_17*RLcp7_38+OPcp7_37*RLcp7_18;
    ACcp7_38 = qdd[4]+OMcp7_17*ORcp7_28-OMcp7_27*ORcp7_18+OPcp7_17*RLcp7_28-OPcp7_27*RLcp7_18;

// = = Block_1_0_0_8_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp7_18;
    sens->P[2] = POcp7_28;
    sens->P[3] = POcp7_38;
    sens->R[1][1] = ROcp7_18;
    sens->R[1][2] = ROcp7_28;
    sens->R[1][3] = ROcp7_38;
    sens->R[2][1] = ROcp7_46;
    sens->R[2][2] = ROcp7_56;
    sens->R[2][3] = S6;
    sens->R[3][1] = ROcp7_78;
    sens->R[3][2] = ROcp7_88;
    sens->R[3][3] = ROcp7_98;
    sens->V[1] = VIcp7_18;
    sens->V[2] = VIcp7_28;
    sens->V[3] = VIcp7_38;
    sens->OM[1] = OMcp7_18;
    sens->OM[2] = OMcp7_28;
    sens->OM[3] = OMcp7_38;
    sens->A[1] = ACcp7_18;
    sens->A[2] = ACcp7_28;
    sens->A[3] = ACcp7_38;
    sens->OMP[1] = OPcp7_18;
    sens->OMP[2] = OPcp7_28;
    sens->OMP[3] = OPcp7_38;
 
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

// = = Block_1_0_0_9_0_2 = = 
 
// Sensor Kinematics 


    ROcp8_18 = ROcp8_17*C8-ROcp8_77*S8;
    ROcp8_28 = ROcp8_27*C8-ROcp8_87*S8;
    ROcp8_78 = ROcp8_17*S8+ROcp8_77*C8;
    ROcp8_88 = ROcp8_27*S8+ROcp8_87*C8;
    ROcp8_19 = ROcp8_18*C9-ROcp8_78*S9;
    ROcp8_29 = ROcp8_28*C9-ROcp8_88*S9;
    ROcp8_39 = -S7p8p9*C6;
    ROcp8_79 = ROcp8_18*S9+ROcp8_78*C9;
    ROcp8_89 = ROcp8_28*S9+ROcp8_88*C9;
    ROcp8_99 = C7p8p9*C6;
    RLcp8_18 = ROcp8_17*s->dpt[1][2]+ROcp8_77*s->dpt[3][2];
    RLcp8_28 = ROcp8_27*s->dpt[1][2]+ROcp8_87*s->dpt[3][2];
    RLcp8_38 = -C6*(s->dpt[1][2]*S7-s->dpt[3][2]*C7);
    OMcp8_18 = OMcp8_17+ROcp8_46*qd[8];
    OMcp8_28 = OMcp8_27+ROcp8_56*qd[8];
    OMcp8_38 = OMcp8_37+qd[8]*S6;
    ORcp8_18 = OMcp8_27*RLcp8_38-OMcp8_37*RLcp8_28;
    ORcp8_28 = -(OMcp8_17*RLcp8_38-OMcp8_37*RLcp8_18);
    ORcp8_38 = OMcp8_17*RLcp8_28-OMcp8_27*RLcp8_18;
    OPcp8_18 = OPcp8_17+ROcp8_46*qdd[8]+qd[8]*(OMcp8_27*S6-OMcp8_37*ROcp8_56);
    OPcp8_28 = OPcp8_27+ROcp8_56*qdd[8]-qd[8]*(OMcp8_17*S6-OMcp8_37*ROcp8_46);
    OPcp8_38 = OPcp8_37+qdd[8]*S6+qd[8]*(OMcp8_17*ROcp8_56-OMcp8_27*ROcp8_46);
    RLcp8_19 = ROcp8_18*s->dpt[1][8];
    RLcp8_29 = ROcp8_28*s->dpt[1][8];
    RLcp8_39 = -s->dpt[1][8]*S7p8*C6;
    POcp8_19 = RLcp8_12+RLcp8_13+RLcp8_18+RLcp8_19;
    POcp8_29 = RLcp8_22+RLcp8_23+RLcp8_28+RLcp8_29;
    POcp8_39 = RLcp8_38+RLcp8_39+q[4];
    OMcp8_19 = OMcp8_18+ROcp8_46*qd[9];
    OMcp8_29 = OMcp8_28+ROcp8_56*qd[9];
    OMcp8_39 = OMcp8_38+qd[9]*S6;
    ORcp8_19 = OMcp8_28*RLcp8_39-OMcp8_38*RLcp8_29;
    ORcp8_29 = -(OMcp8_18*RLcp8_39-OMcp8_38*RLcp8_19);
    ORcp8_39 = OMcp8_18*RLcp8_29-OMcp8_28*RLcp8_19;
    VIcp8_19 = ORcp8_12+ORcp8_13+ORcp8_18+ORcp8_19+qd[2]*C1-qd[3]*S1;
    VIcp8_29 = ORcp8_22+ORcp8_23+ORcp8_28+ORcp8_29+qd[2]*S1+qd[3]*C1;
    VIcp8_39 = ORcp8_38+ORcp8_39+qd[4];
    OPcp8_19 = OPcp8_18+ROcp8_46*qdd[9]+qd[9]*(OMcp8_28*S6-OMcp8_38*ROcp8_56);
    OPcp8_29 = OPcp8_28+ROcp8_56*qdd[9]-qd[9]*(OMcp8_18*S6-OMcp8_38*ROcp8_46);
    OPcp8_39 = OPcp8_38+qdd[9]*S6+qd[9]*(OMcp8_18*ROcp8_56-OMcp8_28*ROcp8_46);
    ACcp8_19 = OMcp8_27*ORcp8_38+OMcp8_28*ORcp8_39-OMcp8_37*ORcp8_28-OMcp8_38*ORcp8_29+OPcp8_27*RLcp8_38+OPcp8_28*RLcp8_39
 -OPcp8_37*RLcp8_28-OPcp8_38*RLcp8_29-ORcp8_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp8_22+(2.0)*qd[2]*S1);
    ACcp8_29 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp8_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp8_13-(2.0)*qd[3]*S1)-OMcp8_17*ORcp8_38+OMcp8_37*
 ORcp8_18-OPcp8_17*RLcp8_38+OPcp8_37*RLcp8_18-OMcp8_18*ORcp8_39+OMcp8_38*ORcp8_19-OPcp8_18*RLcp8_39+OPcp8_38*RLcp8_19;
    ACcp8_39 = qdd[4]+OMcp8_17*ORcp8_28+OMcp8_18*ORcp8_29-OMcp8_27*ORcp8_18-OMcp8_28*ORcp8_19+OPcp8_17*RLcp8_28+OPcp8_18*
 RLcp8_29-OPcp8_27*RLcp8_18-OPcp8_28*RLcp8_19;

// = = Block_1_0_0_9_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp8_19;
    sens->P[2] = POcp8_29;
    sens->P[3] = POcp8_39;
    sens->R[1][1] = ROcp8_19;
    sens->R[1][2] = ROcp8_29;
    sens->R[1][3] = ROcp8_39;
    sens->R[2][1] = ROcp8_46;
    sens->R[2][2] = ROcp8_56;
    sens->R[2][3] = S6;
    sens->R[3][1] = ROcp8_79;
    sens->R[3][2] = ROcp8_89;
    sens->R[3][3] = ROcp8_99;
    sens->V[1] = VIcp8_19;
    sens->V[2] = VIcp8_29;
    sens->V[3] = VIcp8_39;
    sens->OM[1] = OMcp8_19;
    sens->OM[2] = OMcp8_29;
    sens->OM[3] = OMcp8_39;
    sens->A[1] = ACcp8_19;
    sens->A[2] = ACcp8_29;
    sens->A[3] = ACcp8_39;
    sens->OMP[1] = OPcp8_19;
    sens->OMP[2] = OPcp8_29;
    sens->OMP[3] = OPcp8_39;
 
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
    POcp9_110 = RLcp9_110+RLcp9_12+RLcp9_13;
    POcp9_210 = RLcp9_210+RLcp9_22+RLcp9_23;
    POcp9_310 = RLcp9_310+q[4];
    ORcp9_110 = OMcp9_27*RLcp9_310-OMcp9_37*RLcp9_210;
    ORcp9_210 = -(OMcp9_17*RLcp9_310-OMcp9_37*RLcp9_110);
    ORcp9_310 = OMcp9_17*RLcp9_210-OMcp9_27*RLcp9_110;
    VIcp9_110 = ORcp9_110+ORcp9_12+ORcp9_13+qd[2]*C1-qd[3]*S1;
    VIcp9_210 = ORcp9_210+ORcp9_22+ORcp9_23+qd[2]*S1+qd[3]*C1;
    VIcp9_310 = ORcp9_310+qd[4];
    ACcp9_110 = OMcp9_27*ORcp9_310-OMcp9_37*ORcp9_210+OPcp9_27*RLcp9_310-OPcp9_37*RLcp9_210-ORcp9_23*qd[1]+qdd[2]*C1-
 qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp9_22+(2.0)*qd[2]*S1);
    ACcp9_210 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp9_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp9_13-(2.0)*qd[3]*S1)-OMcp9_17*ORcp9_310+OMcp9_37*
 ORcp9_110-OPcp9_17*RLcp9_310+OPcp9_37*RLcp9_110;
    ACcp9_310 = qdd[4]+OMcp9_17*ORcp9_210-OMcp9_27*ORcp9_110+OPcp9_17*RLcp9_210-OPcp9_27*RLcp9_110;

// = = Block_1_0_0_10_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp9_110;
    sens->P[2] = POcp9_210;
    sens->P[3] = POcp9_310;
    sens->R[1][1] = ROcp9_110;
    sens->R[1][2] = ROcp9_210;
    sens->R[1][3] = ROcp9_310;
    sens->R[2][1] = ROcp9_46;
    sens->R[2][2] = ROcp9_56;
    sens->R[2][3] = S6;
    sens->R[3][1] = ROcp9_710;
    sens->R[3][2] = ROcp9_810;
    sens->R[3][3] = ROcp9_910;
    sens->V[1] = VIcp9_110;
    sens->V[2] = VIcp9_210;
    sens->V[3] = VIcp9_310;
    sens->OM[1] = OMcp9_17;
    sens->OM[2] = OMcp9_27;
    sens->OM[3] = OMcp9_37;
    sens->A[1] = ACcp9_110;
    sens->A[2] = ACcp9_210;
    sens->A[3] = ACcp9_310;
    sens->OMP[1] = OPcp9_17;
    sens->OMP[2] = OPcp9_27;
    sens->OMP[3] = OPcp9_37;
 
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
    POcp10_110 = RLcp10_110+RLcp10_12+RLcp10_13;
    POcp10_210 = RLcp10_210+RLcp10_22+RLcp10_23;
    POcp10_310 = RLcp10_310+q[4];
    ORcp10_110 = OMcp10_27*RLcp10_310-OMcp10_37*RLcp10_210;
    ORcp10_210 = -(OMcp10_17*RLcp10_310-OMcp10_37*RLcp10_110);
    ORcp10_310 = OMcp10_17*RLcp10_210-OMcp10_27*RLcp10_110;
    VIcp10_110 = ORcp10_110+ORcp10_12+ORcp10_13+qd[2]*C1-qd[3]*S1;
    VIcp10_210 = ORcp10_210+ORcp10_22+ORcp10_23+qd[2]*S1+qd[3]*C1;
    VIcp10_310 = ORcp10_310+qd[4];
    ACcp10_110 = OMcp10_27*ORcp10_310-OMcp10_37*ORcp10_210+OPcp10_27*RLcp10_310-OPcp10_37*RLcp10_210-ORcp10_23*qd[1]+
 qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp10_22+(2.0)*qd[2]*S1);
    ACcp10_210 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp10_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp10_13-(2.0)*qd[3]*S1)-OMcp10_17*ORcp10_310+
 OMcp10_37*ORcp10_110-OPcp10_17*RLcp10_310+OPcp10_37*RLcp10_110;
    ACcp10_310 = qdd[4]+OMcp10_17*ORcp10_210-OMcp10_27*ORcp10_110+OPcp10_17*RLcp10_210-OPcp10_27*RLcp10_110;

// = = Block_1_0_0_11_0_4 = = 
 
// Sensor Kinematics 


    ROcp10_411 = ROcp10_46*C11+ROcp10_710*S11;
    ROcp10_511 = ROcp10_56*C11+ROcp10_810*S11;
    ROcp10_611 = ROcp10_910*S11+C11*S6;
    ROcp10_711 = -(ROcp10_46*S11-ROcp10_710*C11);
    ROcp10_811 = -(ROcp10_56*S11-ROcp10_810*C11);
    ROcp10_911 = ROcp10_910*C11-S11*S6;
    OMcp10_111 = OMcp10_17+ROcp10_110*qd[11];
    OMcp10_211 = OMcp10_27+ROcp10_210*qd[11];
    OMcp10_311 = OMcp10_37+ROcp10_310*qd[11];
    OPcp10_111 = OPcp10_17+ROcp10_110*qdd[11]+qd[11]*(OMcp10_27*ROcp10_310-OMcp10_37*ROcp10_210);
    OPcp10_211 = OPcp10_27+ROcp10_210*qdd[11]-qd[11]*(OMcp10_17*ROcp10_310-OMcp10_37*ROcp10_110);
    OPcp10_311 = OPcp10_37+ROcp10_310*qdd[11]+qd[11]*(OMcp10_17*ROcp10_210-OMcp10_27*ROcp10_110);

// = = Block_1_0_0_11_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp10_110;
    sens->P[2] = POcp10_210;
    sens->P[3] = POcp10_310;
    sens->R[1][1] = ROcp10_110;
    sens->R[1][2] = ROcp10_210;
    sens->R[1][3] = ROcp10_310;
    sens->R[2][1] = ROcp10_411;
    sens->R[2][2] = ROcp10_511;
    sens->R[2][3] = ROcp10_611;
    sens->R[3][1] = ROcp10_711;
    sens->R[3][2] = ROcp10_811;
    sens->R[3][3] = ROcp10_911;
    sens->V[1] = VIcp10_110;
    sens->V[2] = VIcp10_210;
    sens->V[3] = VIcp10_310;
    sens->OM[1] = OMcp10_111;
    sens->OM[2] = OMcp10_211;
    sens->OM[3] = OMcp10_311;
    sens->A[1] = ACcp10_110;
    sens->A[2] = ACcp10_210;
    sens->A[3] = ACcp10_310;
    sens->OMP[1] = OPcp10_111;
    sens->OMP[2] = OPcp10_211;
    sens->OMP[3] = OPcp10_311;
 
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

// = = Block_1_0_0_12_0_3 = = 
 
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

// = = Block_1_0_0_12_0_5 = = 
 
// Sensor Kinematics 


    ROcp11_412 = ROcp11_46*C12+ROcp11_710*S12;
    ROcp11_512 = ROcp11_56*C12+ROcp11_810*S12;
    ROcp11_612 = ROcp11_910*S12+C12*S6;
    ROcp11_712 = -(ROcp11_46*S12-ROcp11_710*C12);
    ROcp11_812 = -(ROcp11_56*S12-ROcp11_810*C12);
    ROcp11_912 = ROcp11_910*C12-S12*S6;
    RLcp11_112 = ROcp11_46*s->dpt[2][11]+ROcp11_710*s->dpt[3][11];
    RLcp11_212 = ROcp11_56*s->dpt[2][11]+ROcp11_810*s->dpt[3][11];
    RLcp11_312 = ROcp11_910*s->dpt[3][11]+s->dpt[2][11]*S6;
    POcp11_112 = RLcp11_110+RLcp11_112+RLcp11_12+RLcp11_13;
    POcp11_212 = RLcp11_210+RLcp11_212+RLcp11_22+RLcp11_23;
    POcp11_312 = RLcp11_310+RLcp11_312+q[4];
    OMcp11_112 = OMcp11_17+ROcp11_110*qd[12];
    OMcp11_212 = OMcp11_27+ROcp11_210*qd[12];
    OMcp11_312 = OMcp11_37+ROcp11_310*qd[12];
    ORcp11_112 = OMcp11_27*RLcp11_312-OMcp11_37*RLcp11_212;
    ORcp11_212 = -(OMcp11_17*RLcp11_312-OMcp11_37*RLcp11_112);
    ORcp11_312 = OMcp11_17*RLcp11_212-OMcp11_27*RLcp11_112;
    VIcp11_112 = ORcp11_110+ORcp11_112+ORcp11_12+ORcp11_13+qd[2]*C1-qd[3]*S1;
    VIcp11_212 = ORcp11_210+ORcp11_212+ORcp11_22+ORcp11_23+qd[2]*S1+qd[3]*C1;
    VIcp11_312 = ORcp11_310+ORcp11_312+qd[4];
    OPcp11_112 = OPcp11_17+ROcp11_110*qdd[12]+qd[12]*(OMcp11_27*ROcp11_310-OMcp11_37*ROcp11_210);
    OPcp11_212 = OPcp11_27+ROcp11_210*qdd[12]-qd[12]*(OMcp11_17*ROcp11_310-OMcp11_37*ROcp11_110);
    OPcp11_312 = OPcp11_37+ROcp11_310*qdd[12]+qd[12]*(OMcp11_17*ROcp11_210-OMcp11_27*ROcp11_110);
    ACcp11_112 = OMcp11_27*(ORcp11_310+ORcp11_312)-OMcp11_37*ORcp11_210-OMcp11_37*ORcp11_212+OPcp11_27*RLcp11_310+
 OPcp11_27*RLcp11_312-OPcp11_37*RLcp11_210-OPcp11_37*RLcp11_212-ORcp11_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(
 ORcp11_22+(2.0)*qd[2]*S1);
    ACcp11_212 = -(OMcp11_17*(ORcp11_310+ORcp11_312)-OMcp11_37*ORcp11_110-OMcp11_37*ORcp11_112+OPcp11_17*RLcp11_310+
 OPcp11_17*RLcp11_312-OPcp11_37*RLcp11_110-OPcp11_37*RLcp11_112-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp11_12+(2.0)*qd[2]*C1)-qd[1]*(
 ORcp11_13-(2.0)*qd[3]*S1));
    ACcp11_312 = qdd[4]+OMcp11_17*ORcp11_210+OMcp11_17*ORcp11_212-OMcp11_27*ORcp11_110-OMcp11_27*ORcp11_112+OPcp11_17*
 RLcp11_210+OPcp11_17*RLcp11_212-OPcp11_27*RLcp11_110-OPcp11_27*RLcp11_112;

// = = Block_1_0_0_12_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp11_112;
    sens->P[2] = POcp11_212;
    sens->P[3] = POcp11_312;
    sens->R[1][1] = ROcp11_110;
    sens->R[1][2] = ROcp11_210;
    sens->R[1][3] = ROcp11_310;
    sens->R[2][1] = ROcp11_412;
    sens->R[2][2] = ROcp11_512;
    sens->R[2][3] = ROcp11_612;
    sens->R[3][1] = ROcp11_712;
    sens->R[3][2] = ROcp11_812;
    sens->R[3][3] = ROcp11_912;
    sens->V[1] = VIcp11_112;
    sens->V[2] = VIcp11_212;
    sens->V[3] = VIcp11_312;
    sens->OM[1] = OMcp11_112;
    sens->OM[2] = OMcp11_212;
    sens->OM[3] = OMcp11_312;
    sens->A[1] = ACcp11_112;
    sens->A[2] = ACcp11_212;
    sens->A[3] = ACcp11_312;
    sens->OMP[1] = OPcp11_112;
    sens->OMP[2] = OPcp11_212;
    sens->OMP[3] = OPcp11_312;
 
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
    POcp12_113 = RLcp12_110+RLcp12_112+RLcp12_113+RLcp12_12+RLcp12_13;
    POcp12_213 = RLcp12_210+RLcp12_212+RLcp12_213+RLcp12_22+RLcp12_23;
    POcp12_313 = RLcp12_310+RLcp12_312+RLcp12_313+q[4];
    OMcp12_113 = OMcp12_112+ROcp12_110*qd[13];
    OMcp12_213 = OMcp12_212+ROcp12_210*qd[13];
    OMcp12_313 = OMcp12_312+ROcp12_310*qd[13];
    ORcp12_113 = OMcp12_212*RLcp12_313-OMcp12_312*RLcp12_213;
    ORcp12_213 = -(OMcp12_112*RLcp12_313-OMcp12_312*RLcp12_113);
    ORcp12_313 = OMcp12_112*RLcp12_213-OMcp12_212*RLcp12_113;
    VIcp12_113 = ORcp12_110+ORcp12_112+ORcp12_113+ORcp12_12+ORcp12_13+qd[2]*C1-qd[3]*S1;
    VIcp12_213 = ORcp12_210+ORcp12_212+ORcp12_213+ORcp12_22+ORcp12_23+qd[2]*S1+qd[3]*C1;
    VIcp12_313 = ORcp12_310+ORcp12_312+ORcp12_313+qd[4];
    OPcp12_113 = OPcp12_112+ROcp12_110*qdd[13]+qd[13]*(OMcp12_212*ROcp12_310-OMcp12_312*ROcp12_210);
    OPcp12_213 = OPcp12_212+ROcp12_210*qdd[13]-qd[13]*(OMcp12_112*ROcp12_310-OMcp12_312*ROcp12_110);
    OPcp12_313 = OPcp12_312+ROcp12_310*qdd[13]+qd[13]*(OMcp12_112*ROcp12_210-OMcp12_212*ROcp12_110);
    ACcp12_113 = OMcp12_212*ORcp12_313+OMcp12_27*(ORcp12_310+ORcp12_312)-OMcp12_312*ORcp12_213-OMcp12_37*ORcp12_210-
 OMcp12_37*ORcp12_212+OPcp12_212*RLcp12_313+OPcp12_27*RLcp12_310+OPcp12_27*RLcp12_312-OPcp12_312*RLcp12_213-OPcp12_37*
 RLcp12_210-OPcp12_37*RLcp12_212-ORcp12_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp12_22+(2.0)*qd[2]*S1);
    ACcp12_213 = -(OMcp12_112*ORcp12_313+OMcp12_17*(ORcp12_310+ORcp12_312)-OMcp12_312*ORcp12_113-OMcp12_37*ORcp12_110-
 OMcp12_37*ORcp12_112+OPcp12_112*RLcp12_313+OPcp12_17*RLcp12_310+OPcp12_17*RLcp12_312-OPcp12_312*RLcp12_113-OPcp12_37*
 RLcp12_110-OPcp12_37*RLcp12_112-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp12_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp12_13-(2.0)*qd[3]*S1));
    ACcp12_313 = qdd[4]+OMcp12_112*ORcp12_213+OMcp12_17*ORcp12_210+OMcp12_17*ORcp12_212-OMcp12_212*ORcp12_113-OMcp12_27*
 ORcp12_110-OMcp12_27*ORcp12_112+OPcp12_112*RLcp12_213+OPcp12_17*RLcp12_210+OPcp12_17*RLcp12_212-OPcp12_212*RLcp12_113-
 OPcp12_27*RLcp12_110-OPcp12_27*RLcp12_112;

// = = Block_1_0_0_13_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp12_113;
    sens->P[2] = POcp12_213;
    sens->P[3] = POcp12_313;
    sens->R[1][1] = ROcp12_110;
    sens->R[1][2] = ROcp12_210;
    sens->R[1][3] = ROcp12_310;
    sens->R[2][1] = ROcp12_413;
    sens->R[2][2] = ROcp12_513;
    sens->R[2][3] = ROcp12_613;
    sens->R[3][1] = ROcp12_713;
    sens->R[3][2] = ROcp12_813;
    sens->R[3][3] = ROcp12_913;
    sens->V[1] = VIcp12_113;
    sens->V[2] = VIcp12_213;
    sens->V[3] = VIcp12_313;
    sens->OM[1] = OMcp12_113;
    sens->OM[2] = OMcp12_213;
    sens->OM[3] = OMcp12_313;
    sens->A[1] = ACcp12_113;
    sens->A[2] = ACcp12_213;
    sens->A[3] = ACcp12_313;
    sens->OMP[1] = OPcp12_113;
    sens->OMP[2] = OPcp12_213;
    sens->OMP[3] = OPcp12_313;
 
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

// = = Block_1_0_0_14_0_5 = = 
 
// Sensor Kinematics 


    ROcp13_412 = ROcp13_46*C12+ROcp13_710*S12;
    ROcp13_512 = ROcp13_56*C12+ROcp13_810*S12;
    ROcp13_612 = ROcp13_910*S12+C12*S6;
    ROcp13_712 = -(ROcp13_46*S12-ROcp13_710*C12);
    ROcp13_812 = -(ROcp13_56*S12-ROcp13_810*C12);
    ROcp13_912 = ROcp13_910*C12-S12*S6;
    ROcp13_413 = ROcp13_412*C13+ROcp13_712*S13;
    ROcp13_513 = ROcp13_512*C13+ROcp13_812*S13;
    ROcp13_613 = ROcp13_612*C13+ROcp13_912*S13;
    ROcp13_713 = -(ROcp13_412*S13-ROcp13_712*C13);
    ROcp13_813 = -(ROcp13_512*S13-ROcp13_812*C13);
    ROcp13_913 = -(ROcp13_612*S13-ROcp13_912*C13);
    ROcp13_114 = ROcp13_110*C14-ROcp13_713*S14;
    ROcp13_214 = ROcp13_210*C14-ROcp13_813*S14;
    ROcp13_314 = ROcp13_310*C14-ROcp13_913*S14;
    ROcp13_714 = ROcp13_110*S14+ROcp13_713*C14;
    ROcp13_814 = ROcp13_210*S14+ROcp13_813*C14;
    ROcp13_914 = ROcp13_310*S14+ROcp13_913*C14;
    RLcp13_112 = ROcp13_46*s->dpt[2][11]+ROcp13_710*s->dpt[3][11];
    RLcp13_212 = ROcp13_56*s->dpt[2][11]+ROcp13_810*s->dpt[3][11];
    RLcp13_312 = ROcp13_910*s->dpt[3][11]+s->dpt[2][11]*S6;
    OMcp13_112 = OMcp13_17+ROcp13_110*qd[12];
    OMcp13_212 = OMcp13_27+ROcp13_210*qd[12];
    OMcp13_312 = OMcp13_37+ROcp13_310*qd[12];
    ORcp13_112 = OMcp13_27*RLcp13_312-OMcp13_37*RLcp13_212;
    ORcp13_212 = -(OMcp13_17*RLcp13_312-OMcp13_37*RLcp13_112);
    ORcp13_312 = OMcp13_17*RLcp13_212-OMcp13_27*RLcp13_112;
    OPcp13_112 = OPcp13_17+ROcp13_110*qdd[12]+qd[12]*(OMcp13_27*ROcp13_310-OMcp13_37*ROcp13_210);
    OPcp13_212 = OPcp13_27+ROcp13_210*qdd[12]-qd[12]*(OMcp13_17*ROcp13_310-OMcp13_37*ROcp13_110);
    OPcp13_312 = OPcp13_37+ROcp13_310*qdd[12]+qd[12]*(OMcp13_17*ROcp13_210-OMcp13_27*ROcp13_110);
    RLcp13_113 = ROcp13_412*s->dpt[2][16];
    RLcp13_213 = ROcp13_512*s->dpt[2][16];
    RLcp13_313 = ROcp13_612*s->dpt[2][16];
    POcp13_113 = RLcp13_110+RLcp13_112+RLcp13_113+RLcp13_12+RLcp13_13;
    POcp13_213 = RLcp13_210+RLcp13_212+RLcp13_213+RLcp13_22+RLcp13_23;
    POcp13_313 = RLcp13_310+RLcp13_312+RLcp13_313+q[4];
    OMcp13_113 = OMcp13_112+ROcp13_110*qd[13];
    OMcp13_213 = OMcp13_212+ROcp13_210*qd[13];
    OMcp13_313 = OMcp13_312+ROcp13_310*qd[13];
    ORcp13_113 = OMcp13_212*RLcp13_313-OMcp13_312*RLcp13_213;
    ORcp13_213 = -(OMcp13_112*RLcp13_313-OMcp13_312*RLcp13_113);
    ORcp13_313 = OMcp13_112*RLcp13_213-OMcp13_212*RLcp13_113;
    VIcp13_113 = ORcp13_110+ORcp13_112+ORcp13_113+ORcp13_12+ORcp13_13+qd[2]*C1-qd[3]*S1;
    VIcp13_213 = ORcp13_210+ORcp13_212+ORcp13_213+ORcp13_22+ORcp13_23+qd[2]*S1+qd[3]*C1;
    VIcp13_313 = ORcp13_310+ORcp13_312+ORcp13_313+qd[4];
    ACcp13_113 = OMcp13_212*ORcp13_313+OMcp13_27*(ORcp13_310+ORcp13_312)-OMcp13_312*ORcp13_213-OMcp13_37*ORcp13_210-
 OMcp13_37*ORcp13_212+OPcp13_212*RLcp13_313+OPcp13_27*RLcp13_310+OPcp13_27*RLcp13_312-OPcp13_312*RLcp13_213-OPcp13_37*
 RLcp13_210-OPcp13_37*RLcp13_212-ORcp13_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp13_22+(2.0)*qd[2]*S1);
    ACcp13_213 = -(OMcp13_112*ORcp13_313+OMcp13_17*(ORcp13_310+ORcp13_312)-OMcp13_312*ORcp13_113-OMcp13_37*ORcp13_110-
 OMcp13_37*ORcp13_112+OPcp13_112*RLcp13_313+OPcp13_17*RLcp13_310+OPcp13_17*RLcp13_312-OPcp13_312*RLcp13_113-OPcp13_37*
 RLcp13_110-OPcp13_37*RLcp13_112-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp13_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp13_13-(2.0)*qd[3]*S1));
    ACcp13_313 = qdd[4]+OMcp13_112*ORcp13_213+OMcp13_17*ORcp13_210+OMcp13_17*ORcp13_212-OMcp13_212*ORcp13_113-OMcp13_27*
 ORcp13_110-OMcp13_27*ORcp13_112+OPcp13_112*RLcp13_213+OPcp13_17*RLcp13_210+OPcp13_17*RLcp13_212-OPcp13_212*RLcp13_113-
 OPcp13_27*RLcp13_110-OPcp13_27*RLcp13_112;
    OMcp13_114 = OMcp13_113+ROcp13_413*qd[14];
    OMcp13_214 = OMcp13_213+ROcp13_513*qd[14];
    OMcp13_314 = OMcp13_313+ROcp13_613*qd[14];
    OPcp13_114 = OPcp13_112+ROcp13_110*qdd[13]+ROcp13_413*qdd[14]+qd[13]*(OMcp13_212*ROcp13_310-OMcp13_312*ROcp13_210)+
 qd[14]*(OMcp13_213*ROcp13_613-OMcp13_313*ROcp13_513);
    OPcp13_214 = OPcp13_212+ROcp13_210*qdd[13]+ROcp13_513*qdd[14]-qd[13]*(OMcp13_112*ROcp13_310-OMcp13_312*ROcp13_110)-
 qd[14]*(OMcp13_113*ROcp13_613-OMcp13_313*ROcp13_413);
    OPcp13_314 = OPcp13_312+ROcp13_310*qdd[13]+ROcp13_613*qdd[14]+qd[13]*(OMcp13_112*ROcp13_210-OMcp13_212*ROcp13_110)+
 qd[14]*(OMcp13_113*ROcp13_513-OMcp13_213*ROcp13_413);

// = = Block_1_0_0_14_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp13_113;
    sens->P[2] = POcp13_213;
    sens->P[3] = POcp13_313;
    sens->R[1][1] = ROcp13_114;
    sens->R[1][2] = ROcp13_214;
    sens->R[1][3] = ROcp13_314;
    sens->R[2][1] = ROcp13_413;
    sens->R[2][2] = ROcp13_513;
    sens->R[2][3] = ROcp13_613;
    sens->R[3][1] = ROcp13_714;
    sens->R[3][2] = ROcp13_814;
    sens->R[3][3] = ROcp13_914;
    sens->V[1] = VIcp13_113;
    sens->V[2] = VIcp13_213;
    sens->V[3] = VIcp13_313;
    sens->OM[1] = OMcp13_114;
    sens->OM[2] = OMcp13_214;
    sens->OM[3] = OMcp13_314;
    sens->A[1] = ACcp13_113;
    sens->A[2] = ACcp13_213;
    sens->A[3] = ACcp13_313;
    sens->OMP[1] = OPcp13_114;
    sens->OMP[2] = OPcp13_214;
    sens->OMP[3] = OPcp13_314;
 
// 
break;
case 15:
 


// = = Block_1_0_0_15_0_1 = = 
 
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
    ORcp14_12 = -RLcp14_22*qd[1];
    ORcp14_22 = RLcp14_12*qd[1];
    RLcp14_13 = -q[3]*S1;
    RLcp14_23 = q[3]*C1;
    ORcp14_13 = -RLcp14_23*qd[1];
    ORcp14_23 = RLcp14_13*qd[1];
    OMcp14_35 = qd[1]+qd[5];
    OMcp14_16 = qd[6]*C1p5;
    OMcp14_26 = qd[6]*S1p5;
    OMcp14_17 = OMcp14_16+ROcp14_46*qd[7];
    OMcp14_27 = OMcp14_26+ROcp14_56*qd[7];
    OMcp14_37 = OMcp14_35+qd[7]*S6;
    OPcp14_17 = -(OMcp14_35*qd[6]*S1p5-ROcp14_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp14_26*S6-OMcp14_35*ROcp14_56));
    OPcp14_27 = OMcp14_35*qd[6]*C1p5+ROcp14_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp14_16*S6-OMcp14_35*ROcp14_46);
    OPcp14_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_15_0_3 = = 
 
// Sensor Kinematics 


    ROcp14_110 = ROcp14_17*C10-ROcp14_77*S10;
    ROcp14_210 = ROcp14_27*C10-ROcp14_87*S10;
    ROcp14_310 = -S10p7*C6;
    ROcp14_710 = ROcp14_17*S10+ROcp14_77*C10;
    ROcp14_810 = ROcp14_27*S10+ROcp14_87*C10;
    ROcp14_910 = C10p7*C6;
    RLcp14_110 = ROcp14_17*s->dpt[1][5]+ROcp14_77*s->dpt[3][5];
    RLcp14_210 = ROcp14_27*s->dpt[1][5]+ROcp14_87*s->dpt[3][5];
    RLcp14_310 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp14_110 = OMcp14_27*RLcp14_310-OMcp14_37*RLcp14_210;
    ORcp14_210 = -(OMcp14_17*RLcp14_310-OMcp14_37*RLcp14_110);
    ORcp14_310 = OMcp14_17*RLcp14_210-OMcp14_27*RLcp14_110;

// = = Block_1_0_0_15_0_5 = = 
 
// Sensor Kinematics 


    ROcp14_412 = ROcp14_46*C12+ROcp14_710*S12;
    ROcp14_512 = ROcp14_56*C12+ROcp14_810*S12;
    ROcp14_612 = ROcp14_910*S12+C12*S6;
    ROcp14_712 = -(ROcp14_46*S12-ROcp14_710*C12);
    ROcp14_812 = -(ROcp14_56*S12-ROcp14_810*C12);
    ROcp14_912 = ROcp14_910*C12-S12*S6;
    ROcp14_413 = ROcp14_412*C13+ROcp14_712*S13;
    ROcp14_513 = ROcp14_512*C13+ROcp14_812*S13;
    ROcp14_613 = ROcp14_612*C13+ROcp14_912*S13;
    ROcp14_713 = -(ROcp14_412*S13-ROcp14_712*C13);
    ROcp14_813 = -(ROcp14_512*S13-ROcp14_812*C13);
    ROcp14_913 = -(ROcp14_612*S13-ROcp14_912*C13);
    ROcp14_114 = ROcp14_110*C14-ROcp14_713*S14;
    ROcp14_214 = ROcp14_210*C14-ROcp14_813*S14;
    ROcp14_314 = ROcp14_310*C14-ROcp14_913*S14;
    ROcp14_714 = ROcp14_110*S14+ROcp14_713*C14;
    ROcp14_814 = ROcp14_210*S14+ROcp14_813*C14;
    ROcp14_914 = ROcp14_310*S14+ROcp14_913*C14;
    ROcp14_115 = ROcp14_114*C15+ROcp14_413*S15;
    ROcp14_215 = ROcp14_214*C15+ROcp14_513*S15;
    ROcp14_315 = ROcp14_314*C15+ROcp14_613*S15;
    ROcp14_415 = -(ROcp14_114*S15-ROcp14_413*C15);
    ROcp14_515 = -(ROcp14_214*S15-ROcp14_513*C15);
    ROcp14_615 = -(ROcp14_314*S15-ROcp14_613*C15);
    RLcp14_112 = ROcp14_46*s->dpt[2][11]+ROcp14_710*s->dpt[3][11];
    RLcp14_212 = ROcp14_56*s->dpt[2][11]+ROcp14_810*s->dpt[3][11];
    RLcp14_312 = ROcp14_910*s->dpt[3][11]+s->dpt[2][11]*S6;
    OMcp14_112 = OMcp14_17+ROcp14_110*qd[12];
    OMcp14_212 = OMcp14_27+ROcp14_210*qd[12];
    OMcp14_312 = OMcp14_37+ROcp14_310*qd[12];
    ORcp14_112 = OMcp14_27*RLcp14_312-OMcp14_37*RLcp14_212;
    ORcp14_212 = -(OMcp14_17*RLcp14_312-OMcp14_37*RLcp14_112);
    ORcp14_312 = OMcp14_17*RLcp14_212-OMcp14_27*RLcp14_112;
    OPcp14_112 = OPcp14_17+ROcp14_110*qdd[12]+qd[12]*(OMcp14_27*ROcp14_310-OMcp14_37*ROcp14_210);
    OPcp14_212 = OPcp14_27+ROcp14_210*qdd[12]-qd[12]*(OMcp14_17*ROcp14_310-OMcp14_37*ROcp14_110);
    OPcp14_312 = OPcp14_37+ROcp14_310*qdd[12]+qd[12]*(OMcp14_17*ROcp14_210-OMcp14_27*ROcp14_110);
    RLcp14_113 = ROcp14_412*s->dpt[2][16];
    RLcp14_213 = ROcp14_512*s->dpt[2][16];
    RLcp14_313 = ROcp14_612*s->dpt[2][16];
    POcp14_113 = RLcp14_110+RLcp14_112+RLcp14_113+RLcp14_12+RLcp14_13;
    POcp14_213 = RLcp14_210+RLcp14_212+RLcp14_213+RLcp14_22+RLcp14_23;
    POcp14_313 = RLcp14_310+RLcp14_312+RLcp14_313+q[4];
    OMcp14_113 = OMcp14_112+ROcp14_110*qd[13];
    OMcp14_213 = OMcp14_212+ROcp14_210*qd[13];
    OMcp14_313 = OMcp14_312+ROcp14_310*qd[13];
    ORcp14_113 = OMcp14_212*RLcp14_313-OMcp14_312*RLcp14_213;
    ORcp14_213 = -(OMcp14_112*RLcp14_313-OMcp14_312*RLcp14_113);
    ORcp14_313 = OMcp14_112*RLcp14_213-OMcp14_212*RLcp14_113;
    VIcp14_113 = ORcp14_110+ORcp14_112+ORcp14_113+ORcp14_12+ORcp14_13+qd[2]*C1-qd[3]*S1;
    VIcp14_213 = ORcp14_210+ORcp14_212+ORcp14_213+ORcp14_22+ORcp14_23+qd[2]*S1+qd[3]*C1;
    VIcp14_313 = ORcp14_310+ORcp14_312+ORcp14_313+qd[4];
    ACcp14_113 = OMcp14_212*ORcp14_313+OMcp14_27*(ORcp14_310+ORcp14_312)-OMcp14_312*ORcp14_213-OMcp14_37*ORcp14_210-
 OMcp14_37*ORcp14_212+OPcp14_212*RLcp14_313+OPcp14_27*RLcp14_310+OPcp14_27*RLcp14_312-OPcp14_312*RLcp14_213-OPcp14_37*
 RLcp14_210-OPcp14_37*RLcp14_212-ORcp14_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp14_22+(2.0)*qd[2]*S1);
    ACcp14_213 = -(OMcp14_112*ORcp14_313+OMcp14_17*(ORcp14_310+ORcp14_312)-OMcp14_312*ORcp14_113-OMcp14_37*ORcp14_110-
 OMcp14_37*ORcp14_112+OPcp14_112*RLcp14_313+OPcp14_17*RLcp14_310+OPcp14_17*RLcp14_312-OPcp14_312*RLcp14_113-OPcp14_37*
 RLcp14_110-OPcp14_37*RLcp14_112-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp14_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp14_13-(2.0)*qd[3]*S1));
    ACcp14_313 = qdd[4]+OMcp14_112*ORcp14_213+OMcp14_17*ORcp14_210+OMcp14_17*ORcp14_212-OMcp14_212*ORcp14_113-OMcp14_27*
 ORcp14_110-OMcp14_27*ORcp14_112+OPcp14_112*RLcp14_213+OPcp14_17*RLcp14_210+OPcp14_17*RLcp14_212-OPcp14_212*RLcp14_113-
 OPcp14_27*RLcp14_110-OPcp14_27*RLcp14_112;
    OMcp14_114 = OMcp14_113+ROcp14_413*qd[14];
    OMcp14_214 = OMcp14_213+ROcp14_513*qd[14];
    OMcp14_314 = OMcp14_313+ROcp14_613*qd[14];
    OMcp14_115 = OMcp14_114+ROcp14_714*qd[15];
    OMcp14_215 = OMcp14_214+ROcp14_814*qd[15];
    OMcp14_315 = OMcp14_314+ROcp14_914*qd[15];
    OPcp14_115 = OPcp14_112+ROcp14_110*qdd[13]+ROcp14_413*qdd[14]+ROcp14_714*qdd[15]+qd[13]*(OMcp14_212*ROcp14_310-
 OMcp14_312*ROcp14_210)+qd[14]*(OMcp14_213*ROcp14_613-OMcp14_313*ROcp14_513)+qd[15]*(OMcp14_214*ROcp14_914-OMcp14_314*
 ROcp14_814);
    OPcp14_215 = OPcp14_212+ROcp14_210*qdd[13]+ROcp14_513*qdd[14]+ROcp14_814*qdd[15]-qd[13]*(OMcp14_112*ROcp14_310-
 OMcp14_312*ROcp14_110)-qd[14]*(OMcp14_113*ROcp14_613-OMcp14_313*ROcp14_413)-qd[15]*(OMcp14_114*ROcp14_914-OMcp14_314*
 ROcp14_714);
    OPcp14_315 = OPcp14_312+ROcp14_310*qdd[13]+ROcp14_613*qdd[14]+ROcp14_914*qdd[15]+qd[13]*(OMcp14_112*ROcp14_210-
 OMcp14_212*ROcp14_110)+qd[14]*(OMcp14_113*ROcp14_513-OMcp14_213*ROcp14_413)+qd[15]*(OMcp14_114*ROcp14_814-OMcp14_214*
 ROcp14_714);

// = = Block_1_0_0_15_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp14_113;
    sens->P[2] = POcp14_213;
    sens->P[3] = POcp14_313;
    sens->R[1][1] = ROcp14_115;
    sens->R[1][2] = ROcp14_215;
    sens->R[1][3] = ROcp14_315;
    sens->R[2][1] = ROcp14_415;
    sens->R[2][2] = ROcp14_515;
    sens->R[2][3] = ROcp14_615;
    sens->R[3][1] = ROcp14_714;
    sens->R[3][2] = ROcp14_814;
    sens->R[3][3] = ROcp14_914;
    sens->V[1] = VIcp14_113;
    sens->V[2] = VIcp14_213;
    sens->V[3] = VIcp14_313;
    sens->OM[1] = OMcp14_115;
    sens->OM[2] = OMcp14_215;
    sens->OM[3] = OMcp14_315;
    sens->A[1] = ACcp14_113;
    sens->A[2] = ACcp14_213;
    sens->A[3] = ACcp14_313;
    sens->OMP[1] = OPcp14_115;
    sens->OMP[2] = OPcp14_215;
    sens->OMP[3] = OPcp14_315;
 
// 
break;
case 16:
 


// = = Block_1_0_0_16_0_1 = = 
 
// Sensor Kinematics 


    ROcp15_46 = -S1p5*C6;
    ROcp15_56 = C1p5*C6;
    ROcp15_76 = S1p5*S6;
    ROcp15_86 = -C1p5*S6;
    ROcp15_17 = -(ROcp15_76*S7-C1p5*C7);
    ROcp15_27 = -(ROcp15_86*S7-S1p5*C7);
    ROcp15_77 = ROcp15_76*C7+C1p5*S7;
    ROcp15_87 = ROcp15_86*C7+S1p5*S7;
    RLcp15_12 = q[2]*C1;
    RLcp15_22 = q[2]*S1;
    ORcp15_12 = -RLcp15_22*qd[1];
    ORcp15_22 = RLcp15_12*qd[1];
    RLcp15_13 = -q[3]*S1;
    RLcp15_23 = q[3]*C1;
    ORcp15_13 = -RLcp15_23*qd[1];
    ORcp15_23 = RLcp15_13*qd[1];
    OMcp15_35 = qd[1]+qd[5];
    OMcp15_16 = qd[6]*C1p5;
    OMcp15_26 = qd[6]*S1p5;
    OMcp15_17 = OMcp15_16+ROcp15_46*qd[7];
    OMcp15_27 = OMcp15_26+ROcp15_56*qd[7];
    OMcp15_37 = OMcp15_35+qd[7]*S6;
    OPcp15_17 = -(OMcp15_35*qd[6]*S1p5-ROcp15_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp15_26*S6-OMcp15_35*ROcp15_56));
    OPcp15_27 = OMcp15_35*qd[6]*C1p5+ROcp15_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp15_16*S6-OMcp15_35*ROcp15_46);
    OPcp15_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_16_0_3 = = 
 
// Sensor Kinematics 


    ROcp15_110 = ROcp15_17*C10-ROcp15_77*S10;
    ROcp15_210 = ROcp15_27*C10-ROcp15_87*S10;
    ROcp15_310 = -S10p7*C6;
    ROcp15_710 = ROcp15_17*S10+ROcp15_77*C10;
    ROcp15_810 = ROcp15_27*S10+ROcp15_87*C10;
    ROcp15_910 = C10p7*C6;
    RLcp15_110 = ROcp15_17*s->dpt[1][5]+ROcp15_77*s->dpt[3][5];
    RLcp15_210 = ROcp15_27*s->dpt[1][5]+ROcp15_87*s->dpt[3][5];
    RLcp15_310 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp15_110 = OMcp15_27*RLcp15_310-OMcp15_37*RLcp15_210;
    ORcp15_210 = -(OMcp15_17*RLcp15_310-OMcp15_37*RLcp15_110);
    ORcp15_310 = OMcp15_17*RLcp15_210-OMcp15_27*RLcp15_110;

// = = Block_1_0_0_16_0_5 = = 
 
// Sensor Kinematics 


    ROcp15_412 = ROcp15_46*C12+ROcp15_710*S12;
    ROcp15_512 = ROcp15_56*C12+ROcp15_810*S12;
    ROcp15_612 = ROcp15_910*S12+C12*S6;
    ROcp15_712 = -(ROcp15_46*S12-ROcp15_710*C12);
    ROcp15_812 = -(ROcp15_56*S12-ROcp15_810*C12);
    ROcp15_912 = ROcp15_910*C12-S12*S6;
    ROcp15_413 = ROcp15_412*C13+ROcp15_712*S13;
    ROcp15_513 = ROcp15_512*C13+ROcp15_812*S13;
    ROcp15_613 = ROcp15_612*C13+ROcp15_912*S13;
    ROcp15_713 = -(ROcp15_412*S13-ROcp15_712*C13);
    ROcp15_813 = -(ROcp15_512*S13-ROcp15_812*C13);
    ROcp15_913 = -(ROcp15_612*S13-ROcp15_912*C13);
    ROcp15_114 = ROcp15_110*C14-ROcp15_713*S14;
    ROcp15_214 = ROcp15_210*C14-ROcp15_813*S14;
    ROcp15_314 = ROcp15_310*C14-ROcp15_913*S14;
    ROcp15_714 = ROcp15_110*S14+ROcp15_713*C14;
    ROcp15_814 = ROcp15_210*S14+ROcp15_813*C14;
    ROcp15_914 = ROcp15_310*S14+ROcp15_913*C14;
    ROcp15_115 = ROcp15_114*C15+ROcp15_413*S15;
    ROcp15_215 = ROcp15_214*C15+ROcp15_513*S15;
    ROcp15_315 = ROcp15_314*C15+ROcp15_613*S15;
    ROcp15_415 = -(ROcp15_114*S15-ROcp15_413*C15);
    ROcp15_515 = -(ROcp15_214*S15-ROcp15_513*C15);
    ROcp15_615 = -(ROcp15_314*S15-ROcp15_613*C15);
    ROcp15_416 = ROcp15_415*C16+ROcp15_714*S16;
    ROcp15_516 = ROcp15_515*C16+ROcp15_814*S16;
    ROcp15_616 = ROcp15_615*C16+ROcp15_914*S16;
    ROcp15_716 = -(ROcp15_415*S16-ROcp15_714*C16);
    ROcp15_816 = -(ROcp15_515*S16-ROcp15_814*C16);
    ROcp15_916 = -(ROcp15_615*S16-ROcp15_914*C16);
    RLcp15_112 = ROcp15_46*s->dpt[2][11]+ROcp15_710*s->dpt[3][11];
    RLcp15_212 = ROcp15_56*s->dpt[2][11]+ROcp15_810*s->dpt[3][11];
    RLcp15_312 = ROcp15_910*s->dpt[3][11]+s->dpt[2][11]*S6;
    OMcp15_112 = OMcp15_17+ROcp15_110*qd[12];
    OMcp15_212 = OMcp15_27+ROcp15_210*qd[12];
    OMcp15_312 = OMcp15_37+ROcp15_310*qd[12];
    ORcp15_112 = OMcp15_27*RLcp15_312-OMcp15_37*RLcp15_212;
    ORcp15_212 = -(OMcp15_17*RLcp15_312-OMcp15_37*RLcp15_112);
    ORcp15_312 = OMcp15_17*RLcp15_212-OMcp15_27*RLcp15_112;
    OPcp15_112 = OPcp15_17+ROcp15_110*qdd[12]+qd[12]*(OMcp15_27*ROcp15_310-OMcp15_37*ROcp15_210);
    OPcp15_212 = OPcp15_27+ROcp15_210*qdd[12]-qd[12]*(OMcp15_17*ROcp15_310-OMcp15_37*ROcp15_110);
    OPcp15_312 = OPcp15_37+ROcp15_310*qdd[12]+qd[12]*(OMcp15_17*ROcp15_210-OMcp15_27*ROcp15_110);
    RLcp15_113 = ROcp15_412*s->dpt[2][16];
    RLcp15_213 = ROcp15_512*s->dpt[2][16];
    RLcp15_313 = ROcp15_612*s->dpt[2][16];
    OMcp15_113 = OMcp15_112+ROcp15_110*qd[13];
    OMcp15_213 = OMcp15_212+ROcp15_210*qd[13];
    OMcp15_313 = OMcp15_312+ROcp15_310*qd[13];
    ORcp15_113 = OMcp15_212*RLcp15_313-OMcp15_312*RLcp15_213;
    ORcp15_213 = -(OMcp15_112*RLcp15_313-OMcp15_312*RLcp15_113);
    ORcp15_313 = OMcp15_112*RLcp15_213-OMcp15_212*RLcp15_113;
    OMcp15_114 = OMcp15_113+ROcp15_413*qd[14];
    OMcp15_214 = OMcp15_213+ROcp15_513*qd[14];
    OMcp15_314 = OMcp15_313+ROcp15_613*qd[14];
    OMcp15_115 = OMcp15_114+ROcp15_714*qd[15];
    OMcp15_215 = OMcp15_214+ROcp15_814*qd[15];
    OMcp15_315 = OMcp15_314+ROcp15_914*qd[15];
    OPcp15_115 = OPcp15_112+ROcp15_110*qdd[13]+ROcp15_413*qdd[14]+ROcp15_714*qdd[15]+qd[13]*(OMcp15_212*ROcp15_310-
 OMcp15_312*ROcp15_210)+qd[14]*(OMcp15_213*ROcp15_613-OMcp15_313*ROcp15_513)+qd[15]*(OMcp15_214*ROcp15_914-OMcp15_314*
 ROcp15_814);
    OPcp15_215 = OPcp15_212+ROcp15_210*qdd[13]+ROcp15_513*qdd[14]+ROcp15_814*qdd[15]-qd[13]*(OMcp15_112*ROcp15_310-
 OMcp15_312*ROcp15_110)-qd[14]*(OMcp15_113*ROcp15_613-OMcp15_313*ROcp15_413)-qd[15]*(OMcp15_114*ROcp15_914-OMcp15_314*
 ROcp15_714);
    OPcp15_315 = OPcp15_312+ROcp15_310*qdd[13]+ROcp15_613*qdd[14]+ROcp15_914*qdd[15]+qd[13]*(OMcp15_112*ROcp15_210-
 OMcp15_212*ROcp15_110)+qd[14]*(OMcp15_113*ROcp15_513-OMcp15_213*ROcp15_413)+qd[15]*(OMcp15_114*ROcp15_814-OMcp15_214*
 ROcp15_714);
    RLcp15_116 = ROcp15_115*s->dpt[1][20]+ROcp15_415*s->dpt[2][20]+ROcp15_714*s->dpt[3][20];
    RLcp15_216 = ROcp15_215*s->dpt[1][20]+ROcp15_515*s->dpt[2][20]+ROcp15_814*s->dpt[3][20];
    RLcp15_316 = ROcp15_315*s->dpt[1][20]+ROcp15_615*s->dpt[2][20]+ROcp15_914*s->dpt[3][20];
    POcp15_116 = RLcp15_110+RLcp15_112+RLcp15_113+RLcp15_116+RLcp15_12+RLcp15_13;
    POcp15_216 = RLcp15_210+RLcp15_212+RLcp15_213+RLcp15_216+RLcp15_22+RLcp15_23;
    POcp15_316 = RLcp15_310+RLcp15_312+RLcp15_313+RLcp15_316+q[4];
    ORcp15_116 = OMcp15_215*RLcp15_316-OMcp15_315*RLcp15_216;
    ORcp15_216 = -(OMcp15_115*RLcp15_316-OMcp15_315*RLcp15_116);
    ORcp15_316 = OMcp15_115*RLcp15_216-OMcp15_215*RLcp15_116;
    VIcp15_116 = ORcp15_110+ORcp15_112+ORcp15_113+ORcp15_116+ORcp15_12+ORcp15_13+qd[2]*C1-qd[3]*S1;
    VIcp15_216 = ORcp15_210+ORcp15_212+ORcp15_213+ORcp15_216+ORcp15_22+ORcp15_23+qd[2]*S1+qd[3]*C1;
    VIcp15_316 = ORcp15_310+ORcp15_312+ORcp15_313+ORcp15_316+qd[4];
    ACcp15_116 = OMcp15_212*ORcp15_313+OMcp15_215*ORcp15_316+OMcp15_27*(ORcp15_310+ORcp15_312)-OMcp15_312*ORcp15_213-
 OMcp15_315*ORcp15_216-OMcp15_37*ORcp15_210-OMcp15_37*ORcp15_212+OPcp15_212*RLcp15_313+OPcp15_215*RLcp15_316+OPcp15_27*
 RLcp15_310+OPcp15_27*RLcp15_312-OPcp15_312*RLcp15_213-OPcp15_315*RLcp15_216-OPcp15_37*RLcp15_210-OPcp15_37*RLcp15_212-
 ORcp15_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp15_22+(2.0)*qd[2]*S1);
    ACcp15_216 = -(OMcp15_112*ORcp15_313+OMcp15_115*ORcp15_316+OMcp15_17*(ORcp15_310+ORcp15_312)-OMcp15_312*ORcp15_113-
 OMcp15_315*ORcp15_116-OMcp15_37*ORcp15_110-OMcp15_37*ORcp15_112+OPcp15_112*RLcp15_313+OPcp15_115*RLcp15_316+OPcp15_17*
 RLcp15_310+OPcp15_17*RLcp15_312-OPcp15_312*RLcp15_113-OPcp15_315*RLcp15_116-OPcp15_37*RLcp15_110-OPcp15_37*RLcp15_112-qdd[2]
 *S1-qdd[3]*C1-qd[1]*(ORcp15_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp15_13-(2.0)*qd[3]*S1));
    ACcp15_316 = qdd[4]+OMcp15_112*ORcp15_213+OMcp15_115*ORcp15_216+OMcp15_17*ORcp15_210+OMcp15_17*ORcp15_212-OMcp15_212*
 ORcp15_113-OMcp15_215*ORcp15_116-OMcp15_27*ORcp15_110-OMcp15_27*ORcp15_112+OPcp15_112*RLcp15_213+OPcp15_115*RLcp15_216+
 OPcp15_17*RLcp15_210+OPcp15_17*RLcp15_212-OPcp15_212*RLcp15_113-OPcp15_215*RLcp15_116-OPcp15_27*RLcp15_110-OPcp15_27*
 RLcp15_112;

// = = Block_1_0_0_16_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp15_116;
    sens->P[2] = POcp15_216;
    sens->P[3] = POcp15_316;
    sens->R[1][1] = ROcp15_115;
    sens->R[1][2] = ROcp15_215;
    sens->R[1][3] = ROcp15_315;
    sens->R[2][1] = ROcp15_416;
    sens->R[2][2] = ROcp15_516;
    sens->R[2][3] = ROcp15_616;
    sens->R[3][1] = ROcp15_716;
    sens->R[3][2] = ROcp15_816;
    sens->R[3][3] = ROcp15_916;
    sens->V[1] = VIcp15_116;
    sens->V[2] = VIcp15_216;
    sens->V[3] = VIcp15_316;
    sens->OM[1] = OMcp15_115;
    sens->OM[2] = OMcp15_215;
    sens->OM[3] = OMcp15_315;
    sens->A[1] = ACcp15_116;
    sens->A[2] = ACcp15_216;
    sens->A[3] = ACcp15_316;
    sens->OMP[1] = OPcp15_115;
    sens->OMP[2] = OPcp15_215;
    sens->OMP[3] = OPcp15_315;
 
// 
break;
case 17:
 


// = = Block_1_0_0_17_0_1 = = 
 
// Sensor Kinematics 


    ROcp16_46 = -S1p5*C6;
    ROcp16_56 = C1p5*C6;
    ROcp16_76 = S1p5*S6;
    ROcp16_86 = -C1p5*S6;
    ROcp16_17 = -(ROcp16_76*S7-C1p5*C7);
    ROcp16_27 = -(ROcp16_86*S7-S1p5*C7);
    ROcp16_77 = ROcp16_76*C7+C1p5*S7;
    ROcp16_87 = ROcp16_86*C7+S1p5*S7;
    RLcp16_12 = q[2]*C1;
    RLcp16_22 = q[2]*S1;
    ORcp16_12 = -RLcp16_22*qd[1];
    ORcp16_22 = RLcp16_12*qd[1];
    RLcp16_13 = -q[3]*S1;
    RLcp16_23 = q[3]*C1;
    ORcp16_13 = -RLcp16_23*qd[1];
    ORcp16_23 = RLcp16_13*qd[1];
    OMcp16_35 = qd[1]+qd[5];
    OMcp16_16 = qd[6]*C1p5;
    OMcp16_26 = qd[6]*S1p5;
    OMcp16_17 = OMcp16_16+ROcp16_46*qd[7];
    OMcp16_27 = OMcp16_26+ROcp16_56*qd[7];
    OMcp16_37 = OMcp16_35+qd[7]*S6;
    OPcp16_17 = -(OMcp16_35*qd[6]*S1p5-ROcp16_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp16_26*S6-OMcp16_35*ROcp16_56));
    OPcp16_27 = OMcp16_35*qd[6]*C1p5+ROcp16_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp16_16*S6-OMcp16_35*ROcp16_46);
    OPcp16_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_17_0_3 = = 
 
// Sensor Kinematics 


    ROcp16_110 = ROcp16_17*C10-ROcp16_77*S10;
    ROcp16_210 = ROcp16_27*C10-ROcp16_87*S10;
    ROcp16_310 = -S10p7*C6;
    ROcp16_710 = ROcp16_17*S10+ROcp16_77*C10;
    ROcp16_810 = ROcp16_27*S10+ROcp16_87*C10;
    ROcp16_910 = C10p7*C6;
    RLcp16_110 = ROcp16_17*s->dpt[1][5]+ROcp16_77*s->dpt[3][5];
    RLcp16_210 = ROcp16_27*s->dpt[1][5]+ROcp16_87*s->dpt[3][5];
    RLcp16_310 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp16_110 = OMcp16_27*RLcp16_310-OMcp16_37*RLcp16_210;
    ORcp16_210 = -(OMcp16_17*RLcp16_310-OMcp16_37*RLcp16_110);
    ORcp16_310 = OMcp16_17*RLcp16_210-OMcp16_27*RLcp16_110;

// = = Block_1_0_0_17_0_5 = = 
 
// Sensor Kinematics 


    ROcp16_412 = ROcp16_46*C12+ROcp16_710*S12;
    ROcp16_512 = ROcp16_56*C12+ROcp16_810*S12;
    ROcp16_612 = ROcp16_910*S12+C12*S6;
    ROcp16_712 = -(ROcp16_46*S12-ROcp16_710*C12);
    ROcp16_812 = -(ROcp16_56*S12-ROcp16_810*C12);
    ROcp16_912 = ROcp16_910*C12-S12*S6;
    ROcp16_413 = ROcp16_412*C13+ROcp16_712*S13;
    ROcp16_513 = ROcp16_512*C13+ROcp16_812*S13;
    ROcp16_613 = ROcp16_612*C13+ROcp16_912*S13;
    ROcp16_713 = -(ROcp16_412*S13-ROcp16_712*C13);
    ROcp16_813 = -(ROcp16_512*S13-ROcp16_812*C13);
    ROcp16_913 = -(ROcp16_612*S13-ROcp16_912*C13);
    ROcp16_114 = ROcp16_110*C14-ROcp16_713*S14;
    ROcp16_214 = ROcp16_210*C14-ROcp16_813*S14;
    ROcp16_314 = ROcp16_310*C14-ROcp16_913*S14;
    ROcp16_714 = ROcp16_110*S14+ROcp16_713*C14;
    ROcp16_814 = ROcp16_210*S14+ROcp16_813*C14;
    ROcp16_914 = ROcp16_310*S14+ROcp16_913*C14;
    ROcp16_115 = ROcp16_114*C15+ROcp16_413*S15;
    ROcp16_215 = ROcp16_214*C15+ROcp16_513*S15;
    ROcp16_315 = ROcp16_314*C15+ROcp16_613*S15;
    ROcp16_415 = -(ROcp16_114*S15-ROcp16_413*C15);
    ROcp16_515 = -(ROcp16_214*S15-ROcp16_513*C15);
    ROcp16_615 = -(ROcp16_314*S15-ROcp16_613*C15);
    ROcp16_416 = ROcp16_415*C16+ROcp16_714*S16;
    ROcp16_516 = ROcp16_515*C16+ROcp16_814*S16;
    ROcp16_616 = ROcp16_615*C16+ROcp16_914*S16;
    ROcp16_716 = -(ROcp16_415*S16-ROcp16_714*C16);
    ROcp16_816 = -(ROcp16_515*S16-ROcp16_814*C16);
    ROcp16_916 = -(ROcp16_615*S16-ROcp16_914*C16);
    ROcp16_117 = ROcp16_115*C17-ROcp16_716*S17;
    ROcp16_217 = ROcp16_215*C17-ROcp16_816*S17;
    ROcp16_317 = ROcp16_315*C17-ROcp16_916*S17;
    ROcp16_717 = ROcp16_115*S17+ROcp16_716*C17;
    ROcp16_817 = ROcp16_215*S17+ROcp16_816*C17;
    ROcp16_917 = ROcp16_315*S17+ROcp16_916*C17;
    RLcp16_112 = ROcp16_46*s->dpt[2][11]+ROcp16_710*s->dpt[3][11];
    RLcp16_212 = ROcp16_56*s->dpt[2][11]+ROcp16_810*s->dpt[3][11];
    RLcp16_312 = ROcp16_910*s->dpt[3][11]+s->dpt[2][11]*S6;
    OMcp16_112 = OMcp16_17+ROcp16_110*qd[12];
    OMcp16_212 = OMcp16_27+ROcp16_210*qd[12];
    OMcp16_312 = OMcp16_37+ROcp16_310*qd[12];
    ORcp16_112 = OMcp16_27*RLcp16_312-OMcp16_37*RLcp16_212;
    ORcp16_212 = -(OMcp16_17*RLcp16_312-OMcp16_37*RLcp16_112);
    ORcp16_312 = OMcp16_17*RLcp16_212-OMcp16_27*RLcp16_112;
    OPcp16_112 = OPcp16_17+ROcp16_110*qdd[12]+qd[12]*(OMcp16_27*ROcp16_310-OMcp16_37*ROcp16_210);
    OPcp16_212 = OPcp16_27+ROcp16_210*qdd[12]-qd[12]*(OMcp16_17*ROcp16_310-OMcp16_37*ROcp16_110);
    OPcp16_312 = OPcp16_37+ROcp16_310*qdd[12]+qd[12]*(OMcp16_17*ROcp16_210-OMcp16_27*ROcp16_110);
    RLcp16_113 = ROcp16_412*s->dpt[2][16];
    RLcp16_213 = ROcp16_512*s->dpt[2][16];
    RLcp16_313 = ROcp16_612*s->dpt[2][16];
    OMcp16_113 = OMcp16_112+ROcp16_110*qd[13];
    OMcp16_213 = OMcp16_212+ROcp16_210*qd[13];
    OMcp16_313 = OMcp16_312+ROcp16_310*qd[13];
    ORcp16_113 = OMcp16_212*RLcp16_313-OMcp16_312*RLcp16_213;
    ORcp16_213 = -(OMcp16_112*RLcp16_313-OMcp16_312*RLcp16_113);
    ORcp16_313 = OMcp16_112*RLcp16_213-OMcp16_212*RLcp16_113;
    OMcp16_114 = OMcp16_113+ROcp16_413*qd[14];
    OMcp16_214 = OMcp16_213+ROcp16_513*qd[14];
    OMcp16_314 = OMcp16_313+ROcp16_613*qd[14];
    OMcp16_115 = OMcp16_114+ROcp16_714*qd[15];
    OMcp16_215 = OMcp16_214+ROcp16_814*qd[15];
    OMcp16_315 = OMcp16_314+ROcp16_914*qd[15];
    OPcp16_115 = OPcp16_112+ROcp16_110*qdd[13]+ROcp16_413*qdd[14]+ROcp16_714*qdd[15]+qd[13]*(OMcp16_212*ROcp16_310-
 OMcp16_312*ROcp16_210)+qd[14]*(OMcp16_213*ROcp16_613-OMcp16_313*ROcp16_513)+qd[15]*(OMcp16_214*ROcp16_914-OMcp16_314*
 ROcp16_814);
    OPcp16_215 = OPcp16_212+ROcp16_210*qdd[13]+ROcp16_513*qdd[14]+ROcp16_814*qdd[15]-qd[13]*(OMcp16_112*ROcp16_310-
 OMcp16_312*ROcp16_110)-qd[14]*(OMcp16_113*ROcp16_613-OMcp16_313*ROcp16_413)-qd[15]*(OMcp16_114*ROcp16_914-OMcp16_314*
 ROcp16_714);
    OPcp16_315 = OPcp16_312+ROcp16_310*qdd[13]+ROcp16_613*qdd[14]+ROcp16_914*qdd[15]+qd[13]*(OMcp16_112*ROcp16_210-
 OMcp16_212*ROcp16_110)+qd[14]*(OMcp16_113*ROcp16_513-OMcp16_213*ROcp16_413)+qd[15]*(OMcp16_114*ROcp16_814-OMcp16_214*
 ROcp16_714);
    RLcp16_116 = ROcp16_115*s->dpt[1][20]+ROcp16_415*s->dpt[2][20]+ROcp16_714*s->dpt[3][20];
    RLcp16_216 = ROcp16_215*s->dpt[1][20]+ROcp16_515*s->dpt[2][20]+ROcp16_814*s->dpt[3][20];
    RLcp16_316 = ROcp16_315*s->dpt[1][20]+ROcp16_615*s->dpt[2][20]+ROcp16_914*s->dpt[3][20];
    POcp16_116 = RLcp16_110+RLcp16_112+RLcp16_113+RLcp16_116+RLcp16_12+RLcp16_13;
    POcp16_216 = RLcp16_210+RLcp16_212+RLcp16_213+RLcp16_216+RLcp16_22+RLcp16_23;
    POcp16_316 = RLcp16_310+RLcp16_312+RLcp16_313+RLcp16_316+q[4];
    ORcp16_116 = OMcp16_215*RLcp16_316-OMcp16_315*RLcp16_216;
    ORcp16_216 = -(OMcp16_115*RLcp16_316-OMcp16_315*RLcp16_116);
    ORcp16_316 = OMcp16_115*RLcp16_216-OMcp16_215*RLcp16_116;
    VIcp16_116 = ORcp16_110+ORcp16_112+ORcp16_113+ORcp16_116+ORcp16_12+ORcp16_13+qd[2]*C1-qd[3]*S1;
    VIcp16_216 = ORcp16_210+ORcp16_212+ORcp16_213+ORcp16_216+ORcp16_22+ORcp16_23+qd[2]*S1+qd[3]*C1;
    VIcp16_316 = ORcp16_310+ORcp16_312+ORcp16_313+ORcp16_316+qd[4];
    ACcp16_116 = OMcp16_212*ORcp16_313+OMcp16_215*ORcp16_316+OMcp16_27*(ORcp16_310+ORcp16_312)-OMcp16_312*ORcp16_213-
 OMcp16_315*ORcp16_216-OMcp16_37*ORcp16_210-OMcp16_37*ORcp16_212+OPcp16_212*RLcp16_313+OPcp16_215*RLcp16_316+OPcp16_27*
 RLcp16_310+OPcp16_27*RLcp16_312-OPcp16_312*RLcp16_213-OPcp16_315*RLcp16_216-OPcp16_37*RLcp16_210-OPcp16_37*RLcp16_212-
 ORcp16_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp16_22+(2.0)*qd[2]*S1);
    ACcp16_216 = -(OMcp16_112*ORcp16_313+OMcp16_115*ORcp16_316+OMcp16_17*(ORcp16_310+ORcp16_312)-OMcp16_312*ORcp16_113-
 OMcp16_315*ORcp16_116-OMcp16_37*ORcp16_110-OMcp16_37*ORcp16_112+OPcp16_112*RLcp16_313+OPcp16_115*RLcp16_316+OPcp16_17*
 RLcp16_310+OPcp16_17*RLcp16_312-OPcp16_312*RLcp16_113-OPcp16_315*RLcp16_116-OPcp16_37*RLcp16_110-OPcp16_37*RLcp16_112-qdd[2]
 *S1-qdd[3]*C1-qd[1]*(ORcp16_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp16_13-(2.0)*qd[3]*S1));
    ACcp16_316 = qdd[4]+OMcp16_112*ORcp16_213+OMcp16_115*ORcp16_216+OMcp16_17*ORcp16_210+OMcp16_17*ORcp16_212-OMcp16_212*
 ORcp16_113-OMcp16_215*ORcp16_116-OMcp16_27*ORcp16_110-OMcp16_27*ORcp16_112+OPcp16_112*RLcp16_213+OPcp16_115*RLcp16_216+
 OPcp16_17*RLcp16_210+OPcp16_17*RLcp16_212-OPcp16_212*RLcp16_113-OPcp16_215*RLcp16_116-OPcp16_27*RLcp16_110-OPcp16_27*
 RLcp16_112;
    OMcp16_117 = OMcp16_115+ROcp16_416*qd[17];
    OMcp16_217 = OMcp16_215+ROcp16_516*qd[17];
    OMcp16_317 = OMcp16_315+ROcp16_616*qd[17];
    OPcp16_117 = OPcp16_115+ROcp16_416*qdd[17]+qd[17]*(OMcp16_215*ROcp16_616-OMcp16_315*ROcp16_516);
    OPcp16_217 = OPcp16_215+ROcp16_516*qdd[17]-qd[17]*(OMcp16_115*ROcp16_616-OMcp16_315*ROcp16_416);
    OPcp16_317 = OPcp16_315+ROcp16_616*qdd[17]+qd[17]*(OMcp16_115*ROcp16_516-OMcp16_215*ROcp16_416);

// = = Block_1_0_0_17_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp16_116;
    sens->P[2] = POcp16_216;
    sens->P[3] = POcp16_316;
    sens->R[1][1] = ROcp16_117;
    sens->R[1][2] = ROcp16_217;
    sens->R[1][3] = ROcp16_317;
    sens->R[2][1] = ROcp16_416;
    sens->R[2][2] = ROcp16_516;
    sens->R[2][3] = ROcp16_616;
    sens->R[3][1] = ROcp16_717;
    sens->R[3][2] = ROcp16_817;
    sens->R[3][3] = ROcp16_917;
    sens->V[1] = VIcp16_116;
    sens->V[2] = VIcp16_216;
    sens->V[3] = VIcp16_316;
    sens->OM[1] = OMcp16_117;
    sens->OM[2] = OMcp16_217;
    sens->OM[3] = OMcp16_317;
    sens->A[1] = ACcp16_116;
    sens->A[2] = ACcp16_216;
    sens->A[3] = ACcp16_316;
    sens->OMP[1] = OPcp16_117;
    sens->OMP[2] = OPcp16_217;
    sens->OMP[3] = OPcp16_317;
 
// 
break;
case 18:
 


// = = Block_1_0_0_18_0_1 = = 
 
// Sensor Kinematics 


    ROcp17_46 = -S1p5*C6;
    ROcp17_56 = C1p5*C6;
    ROcp17_76 = S1p5*S6;
    ROcp17_86 = -C1p5*S6;
    ROcp17_17 = -(ROcp17_76*S7-C1p5*C7);
    ROcp17_27 = -(ROcp17_86*S7-S1p5*C7);
    ROcp17_77 = ROcp17_76*C7+C1p5*S7;
    ROcp17_87 = ROcp17_86*C7+S1p5*S7;
    RLcp17_12 = q[2]*C1;
    RLcp17_22 = q[2]*S1;
    ORcp17_12 = -RLcp17_22*qd[1];
    ORcp17_22 = RLcp17_12*qd[1];
    RLcp17_13 = -q[3]*S1;
    RLcp17_23 = q[3]*C1;
    ORcp17_13 = -RLcp17_23*qd[1];
    ORcp17_23 = RLcp17_13*qd[1];
    OMcp17_35 = qd[1]+qd[5];
    OMcp17_16 = qd[6]*C1p5;
    OMcp17_26 = qd[6]*S1p5;
    OMcp17_17 = OMcp17_16+ROcp17_46*qd[7];
    OMcp17_27 = OMcp17_26+ROcp17_56*qd[7];
    OMcp17_37 = OMcp17_35+qd[7]*S6;
    OPcp17_17 = -(OMcp17_35*qd[6]*S1p5-ROcp17_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp17_26*S6-OMcp17_35*ROcp17_56));
    OPcp17_27 = OMcp17_35*qd[6]*C1p5+ROcp17_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp17_16*S6-OMcp17_35*ROcp17_46);
    OPcp17_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_18_0_3 = = 
 
// Sensor Kinematics 


    ROcp17_110 = ROcp17_17*C10-ROcp17_77*S10;
    ROcp17_210 = ROcp17_27*C10-ROcp17_87*S10;
    ROcp17_310 = -S10p7*C6;
    ROcp17_710 = ROcp17_17*S10+ROcp17_77*C10;
    ROcp17_810 = ROcp17_27*S10+ROcp17_87*C10;
    ROcp17_910 = C10p7*C6;
    RLcp17_110 = ROcp17_17*s->dpt[1][5]+ROcp17_77*s->dpt[3][5];
    RLcp17_210 = ROcp17_27*s->dpt[1][5]+ROcp17_87*s->dpt[3][5];
    RLcp17_310 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp17_110 = OMcp17_27*RLcp17_310-OMcp17_37*RLcp17_210;
    ORcp17_210 = -(OMcp17_17*RLcp17_310-OMcp17_37*RLcp17_110);
    ORcp17_310 = OMcp17_17*RLcp17_210-OMcp17_27*RLcp17_110;

// = = Block_1_0_0_18_0_6 = = 
 
// Sensor Kinematics 


    ROcp17_418 = ROcp17_46*C18+ROcp17_710*S18;
    ROcp17_518 = ROcp17_56*C18+ROcp17_810*S18;
    ROcp17_618 = ROcp17_910*S18+C18*S6;
    ROcp17_718 = -(ROcp17_46*S18-ROcp17_710*C18);
    ROcp17_818 = -(ROcp17_56*S18-ROcp17_810*C18);
    ROcp17_918 = ROcp17_910*C18-S18*S6;
    RLcp17_118 = ROcp17_46*s->dpt[2][12]+ROcp17_710*s->dpt[3][12];
    RLcp17_218 = ROcp17_56*s->dpt[2][12]+ROcp17_810*s->dpt[3][12];
    RLcp17_318 = ROcp17_910*s->dpt[3][12]+s->dpt[2][12]*S6;
    POcp17_118 = RLcp17_110+RLcp17_118+RLcp17_12+RLcp17_13;
    POcp17_218 = RLcp17_210+RLcp17_218+RLcp17_22+RLcp17_23;
    POcp17_318 = RLcp17_310+RLcp17_318+q[4];
    OMcp17_118 = OMcp17_17+ROcp17_110*qd[18];
    OMcp17_218 = OMcp17_27+ROcp17_210*qd[18];
    OMcp17_318 = OMcp17_37+ROcp17_310*qd[18];
    ORcp17_118 = OMcp17_27*RLcp17_318-OMcp17_37*RLcp17_218;
    ORcp17_218 = -(OMcp17_17*RLcp17_318-OMcp17_37*RLcp17_118);
    ORcp17_318 = OMcp17_17*RLcp17_218-OMcp17_27*RLcp17_118;
    VIcp17_118 = ORcp17_110+ORcp17_118+ORcp17_12+ORcp17_13+qd[2]*C1-qd[3]*S1;
    VIcp17_218 = ORcp17_210+ORcp17_218+ORcp17_22+ORcp17_23+qd[2]*S1+qd[3]*C1;
    VIcp17_318 = ORcp17_310+ORcp17_318+qd[4];
    OPcp17_118 = OPcp17_17+ROcp17_110*qdd[18]+qd[18]*(OMcp17_27*ROcp17_310-OMcp17_37*ROcp17_210);
    OPcp17_218 = OPcp17_27+ROcp17_210*qdd[18]-qd[18]*(OMcp17_17*ROcp17_310-OMcp17_37*ROcp17_110);
    OPcp17_318 = OPcp17_37+ROcp17_310*qdd[18]+qd[18]*(OMcp17_17*ROcp17_210-OMcp17_27*ROcp17_110);
    ACcp17_118 = OMcp17_27*(ORcp17_310+ORcp17_318)-OMcp17_37*ORcp17_210-OMcp17_37*ORcp17_218+OPcp17_27*RLcp17_310+
 OPcp17_27*RLcp17_318-OPcp17_37*RLcp17_210-OPcp17_37*RLcp17_218-ORcp17_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(
 ORcp17_22+(2.0)*qd[2]*S1);
    ACcp17_218 = -(OMcp17_17*(ORcp17_310+ORcp17_318)-OMcp17_37*ORcp17_110-OMcp17_37*ORcp17_118+OPcp17_17*RLcp17_310+
 OPcp17_17*RLcp17_318-OPcp17_37*RLcp17_110-OPcp17_37*RLcp17_118-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp17_12+(2.0)*qd[2]*C1)-qd[1]*(
 ORcp17_13-(2.0)*qd[3]*S1));
    ACcp17_318 = qdd[4]+OMcp17_17*ORcp17_210+OMcp17_17*ORcp17_218-OMcp17_27*ORcp17_110-OMcp17_27*ORcp17_118+OPcp17_17*
 RLcp17_210+OPcp17_17*RLcp17_218-OPcp17_27*RLcp17_110-OPcp17_27*RLcp17_118;

// = = Block_1_0_0_18_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp17_118;
    sens->P[2] = POcp17_218;
    sens->P[3] = POcp17_318;
    sens->R[1][1] = ROcp17_110;
    sens->R[1][2] = ROcp17_210;
    sens->R[1][3] = ROcp17_310;
    sens->R[2][1] = ROcp17_418;
    sens->R[2][2] = ROcp17_518;
    sens->R[2][3] = ROcp17_618;
    sens->R[3][1] = ROcp17_718;
    sens->R[3][2] = ROcp17_818;
    sens->R[3][3] = ROcp17_918;
    sens->V[1] = VIcp17_118;
    sens->V[2] = VIcp17_218;
    sens->V[3] = VIcp17_318;
    sens->OM[1] = OMcp17_118;
    sens->OM[2] = OMcp17_218;
    sens->OM[3] = OMcp17_318;
    sens->A[1] = ACcp17_118;
    sens->A[2] = ACcp17_218;
    sens->A[3] = ACcp17_318;
    sens->OMP[1] = OPcp17_118;
    sens->OMP[2] = OPcp17_218;
    sens->OMP[3] = OPcp17_318;
 
// 
break;
case 19:
 


// = = Block_1_0_0_19_0_1 = = 
 
// Sensor Kinematics 


    ROcp18_46 = -S1p5*C6;
    ROcp18_56 = C1p5*C6;
    ROcp18_76 = S1p5*S6;
    ROcp18_86 = -C1p5*S6;
    ROcp18_17 = -(ROcp18_76*S7-C1p5*C7);
    ROcp18_27 = -(ROcp18_86*S7-S1p5*C7);
    ROcp18_77 = ROcp18_76*C7+C1p5*S7;
    ROcp18_87 = ROcp18_86*C7+S1p5*S7;
    RLcp18_12 = q[2]*C1;
    RLcp18_22 = q[2]*S1;
    ORcp18_12 = -RLcp18_22*qd[1];
    ORcp18_22 = RLcp18_12*qd[1];
    RLcp18_13 = -q[3]*S1;
    RLcp18_23 = q[3]*C1;
    ORcp18_13 = -RLcp18_23*qd[1];
    ORcp18_23 = RLcp18_13*qd[1];
    OMcp18_35 = qd[1]+qd[5];
    OMcp18_16 = qd[6]*C1p5;
    OMcp18_26 = qd[6]*S1p5;
    OMcp18_17 = OMcp18_16+ROcp18_46*qd[7];
    OMcp18_27 = OMcp18_26+ROcp18_56*qd[7];
    OMcp18_37 = OMcp18_35+qd[7]*S6;
    OPcp18_17 = -(OMcp18_35*qd[6]*S1p5-ROcp18_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp18_26*S6-OMcp18_35*ROcp18_56));
    OPcp18_27 = OMcp18_35*qd[6]*C1p5+ROcp18_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp18_16*S6-OMcp18_35*ROcp18_46);
    OPcp18_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_19_0_3 = = 
 
// Sensor Kinematics 


    ROcp18_110 = ROcp18_17*C10-ROcp18_77*S10;
    ROcp18_210 = ROcp18_27*C10-ROcp18_87*S10;
    ROcp18_310 = -S10p7*C6;
    ROcp18_710 = ROcp18_17*S10+ROcp18_77*C10;
    ROcp18_810 = ROcp18_27*S10+ROcp18_87*C10;
    ROcp18_910 = C10p7*C6;
    RLcp18_110 = ROcp18_17*s->dpt[1][5]+ROcp18_77*s->dpt[3][5];
    RLcp18_210 = ROcp18_27*s->dpt[1][5]+ROcp18_87*s->dpt[3][5];
    RLcp18_310 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp18_110 = OMcp18_27*RLcp18_310-OMcp18_37*RLcp18_210;
    ORcp18_210 = -(OMcp18_17*RLcp18_310-OMcp18_37*RLcp18_110);
    ORcp18_310 = OMcp18_17*RLcp18_210-OMcp18_27*RLcp18_110;

// = = Block_1_0_0_19_0_6 = = 
 
// Sensor Kinematics 


    ROcp18_418 = ROcp18_46*C18+ROcp18_710*S18;
    ROcp18_518 = ROcp18_56*C18+ROcp18_810*S18;
    ROcp18_618 = ROcp18_910*S18+C18*S6;
    ROcp18_718 = -(ROcp18_46*S18-ROcp18_710*C18);
    ROcp18_818 = -(ROcp18_56*S18-ROcp18_810*C18);
    ROcp18_918 = ROcp18_910*C18-S18*S6;
    ROcp18_419 = ROcp18_418*C19+ROcp18_718*S19;
    ROcp18_519 = ROcp18_518*C19+ROcp18_818*S19;
    ROcp18_619 = ROcp18_618*C19+ROcp18_918*S19;
    ROcp18_719 = -(ROcp18_418*S19-ROcp18_718*C19);
    ROcp18_819 = -(ROcp18_518*S19-ROcp18_818*C19);
    ROcp18_919 = -(ROcp18_618*S19-ROcp18_918*C19);
    RLcp18_118 = ROcp18_46*s->dpt[2][12]+ROcp18_710*s->dpt[3][12];
    RLcp18_218 = ROcp18_56*s->dpt[2][12]+ROcp18_810*s->dpt[3][12];
    RLcp18_318 = ROcp18_910*s->dpt[3][12]+s->dpt[2][12]*S6;
    OMcp18_118 = OMcp18_17+ROcp18_110*qd[18];
    OMcp18_218 = OMcp18_27+ROcp18_210*qd[18];
    OMcp18_318 = OMcp18_37+ROcp18_310*qd[18];
    ORcp18_118 = OMcp18_27*RLcp18_318-OMcp18_37*RLcp18_218;
    ORcp18_218 = -(OMcp18_17*RLcp18_318-OMcp18_37*RLcp18_118);
    ORcp18_318 = OMcp18_17*RLcp18_218-OMcp18_27*RLcp18_118;
    OPcp18_118 = OPcp18_17+ROcp18_110*qdd[18]+qd[18]*(OMcp18_27*ROcp18_310-OMcp18_37*ROcp18_210);
    OPcp18_218 = OPcp18_27+ROcp18_210*qdd[18]-qd[18]*(OMcp18_17*ROcp18_310-OMcp18_37*ROcp18_110);
    OPcp18_318 = OPcp18_37+ROcp18_310*qdd[18]+qd[18]*(OMcp18_17*ROcp18_210-OMcp18_27*ROcp18_110);
    RLcp18_119 = ROcp18_418*s->dpt[2][23];
    RLcp18_219 = ROcp18_518*s->dpt[2][23];
    RLcp18_319 = ROcp18_618*s->dpt[2][23];
    POcp18_119 = RLcp18_110+RLcp18_118+RLcp18_119+RLcp18_12+RLcp18_13;
    POcp18_219 = RLcp18_210+RLcp18_218+RLcp18_219+RLcp18_22+RLcp18_23;
    POcp18_319 = RLcp18_310+RLcp18_318+RLcp18_319+q[4];
    OMcp18_119 = OMcp18_118+ROcp18_110*qd[19];
    OMcp18_219 = OMcp18_218+ROcp18_210*qd[19];
    OMcp18_319 = OMcp18_318+ROcp18_310*qd[19];
    ORcp18_119 = OMcp18_218*RLcp18_319-OMcp18_318*RLcp18_219;
    ORcp18_219 = -(OMcp18_118*RLcp18_319-OMcp18_318*RLcp18_119);
    ORcp18_319 = OMcp18_118*RLcp18_219-OMcp18_218*RLcp18_119;
    VIcp18_119 = ORcp18_110+ORcp18_118+ORcp18_119+ORcp18_12+ORcp18_13+qd[2]*C1-qd[3]*S1;
    VIcp18_219 = ORcp18_210+ORcp18_218+ORcp18_219+ORcp18_22+ORcp18_23+qd[2]*S1+qd[3]*C1;
    VIcp18_319 = ORcp18_310+ORcp18_318+ORcp18_319+qd[4];
    OPcp18_119 = OPcp18_118+ROcp18_110*qdd[19]+qd[19]*(OMcp18_218*ROcp18_310-OMcp18_318*ROcp18_210);
    OPcp18_219 = OPcp18_218+ROcp18_210*qdd[19]-qd[19]*(OMcp18_118*ROcp18_310-OMcp18_318*ROcp18_110);
    OPcp18_319 = OPcp18_318+ROcp18_310*qdd[19]+qd[19]*(OMcp18_118*ROcp18_210-OMcp18_218*ROcp18_110);
    ACcp18_119 = OMcp18_218*ORcp18_319+OMcp18_27*(ORcp18_310+ORcp18_318)-OMcp18_318*ORcp18_219-OMcp18_37*ORcp18_210-
 OMcp18_37*ORcp18_218+OPcp18_218*RLcp18_319+OPcp18_27*RLcp18_310+OPcp18_27*RLcp18_318-OPcp18_318*RLcp18_219-OPcp18_37*
 RLcp18_210-OPcp18_37*RLcp18_218-ORcp18_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp18_22+(2.0)*qd[2]*S1);
    ACcp18_219 = -(OMcp18_118*ORcp18_319+OMcp18_17*(ORcp18_310+ORcp18_318)-OMcp18_318*ORcp18_119-OMcp18_37*ORcp18_110-
 OMcp18_37*ORcp18_118+OPcp18_118*RLcp18_319+OPcp18_17*RLcp18_310+OPcp18_17*RLcp18_318-OPcp18_318*RLcp18_119-OPcp18_37*
 RLcp18_110-OPcp18_37*RLcp18_118-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp18_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp18_13-(2.0)*qd[3]*S1));
    ACcp18_319 = qdd[4]+OMcp18_118*ORcp18_219+OMcp18_17*ORcp18_210+OMcp18_17*ORcp18_218-OMcp18_218*ORcp18_119-OMcp18_27*
 ORcp18_110-OMcp18_27*ORcp18_118+OPcp18_118*RLcp18_219+OPcp18_17*RLcp18_210+OPcp18_17*RLcp18_218-OPcp18_218*RLcp18_119-
 OPcp18_27*RLcp18_110-OPcp18_27*RLcp18_118;

// = = Block_1_0_0_19_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp18_119;
    sens->P[2] = POcp18_219;
    sens->P[3] = POcp18_319;
    sens->R[1][1] = ROcp18_110;
    sens->R[1][2] = ROcp18_210;
    sens->R[1][3] = ROcp18_310;
    sens->R[2][1] = ROcp18_419;
    sens->R[2][2] = ROcp18_519;
    sens->R[2][3] = ROcp18_619;
    sens->R[3][1] = ROcp18_719;
    sens->R[3][2] = ROcp18_819;
    sens->R[3][3] = ROcp18_919;
    sens->V[1] = VIcp18_119;
    sens->V[2] = VIcp18_219;
    sens->V[3] = VIcp18_319;
    sens->OM[1] = OMcp18_119;
    sens->OM[2] = OMcp18_219;
    sens->OM[3] = OMcp18_319;
    sens->A[1] = ACcp18_119;
    sens->A[2] = ACcp18_219;
    sens->A[3] = ACcp18_319;
    sens->OMP[1] = OPcp18_119;
    sens->OMP[2] = OPcp18_219;
    sens->OMP[3] = OPcp18_319;
 
// 
break;
case 20:
 


// = = Block_1_0_0_20_0_1 = = 
 
// Sensor Kinematics 


    ROcp19_46 = -S1p5*C6;
    ROcp19_56 = C1p5*C6;
    ROcp19_76 = S1p5*S6;
    ROcp19_86 = -C1p5*S6;
    ROcp19_17 = -(ROcp19_76*S7-C1p5*C7);
    ROcp19_27 = -(ROcp19_86*S7-S1p5*C7);
    ROcp19_77 = ROcp19_76*C7+C1p5*S7;
    ROcp19_87 = ROcp19_86*C7+S1p5*S7;
    RLcp19_12 = q[2]*C1;
    RLcp19_22 = q[2]*S1;
    ORcp19_12 = -RLcp19_22*qd[1];
    ORcp19_22 = RLcp19_12*qd[1];
    RLcp19_13 = -q[3]*S1;
    RLcp19_23 = q[3]*C1;
    ORcp19_13 = -RLcp19_23*qd[1];
    ORcp19_23 = RLcp19_13*qd[1];
    OMcp19_35 = qd[1]+qd[5];
    OMcp19_16 = qd[6]*C1p5;
    OMcp19_26 = qd[6]*S1p5;
    OMcp19_17 = OMcp19_16+ROcp19_46*qd[7];
    OMcp19_27 = OMcp19_26+ROcp19_56*qd[7];
    OMcp19_37 = OMcp19_35+qd[7]*S6;
    OPcp19_17 = -(OMcp19_35*qd[6]*S1p5-ROcp19_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp19_26*S6-OMcp19_35*ROcp19_56));
    OPcp19_27 = OMcp19_35*qd[6]*C1p5+ROcp19_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp19_16*S6-OMcp19_35*ROcp19_46);
    OPcp19_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_20_0_3 = = 
 
// Sensor Kinematics 


    ROcp19_110 = ROcp19_17*C10-ROcp19_77*S10;
    ROcp19_210 = ROcp19_27*C10-ROcp19_87*S10;
    ROcp19_310 = -S10p7*C6;
    ROcp19_710 = ROcp19_17*S10+ROcp19_77*C10;
    ROcp19_810 = ROcp19_27*S10+ROcp19_87*C10;
    ROcp19_910 = C10p7*C6;
    RLcp19_110 = ROcp19_17*s->dpt[1][5]+ROcp19_77*s->dpt[3][5];
    RLcp19_210 = ROcp19_27*s->dpt[1][5]+ROcp19_87*s->dpt[3][5];
    RLcp19_310 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp19_110 = OMcp19_27*RLcp19_310-OMcp19_37*RLcp19_210;
    ORcp19_210 = -(OMcp19_17*RLcp19_310-OMcp19_37*RLcp19_110);
    ORcp19_310 = OMcp19_17*RLcp19_210-OMcp19_27*RLcp19_110;

// = = Block_1_0_0_20_0_6 = = 
 
// Sensor Kinematics 


    ROcp19_418 = ROcp19_46*C18+ROcp19_710*S18;
    ROcp19_518 = ROcp19_56*C18+ROcp19_810*S18;
    ROcp19_618 = ROcp19_910*S18+C18*S6;
    ROcp19_718 = -(ROcp19_46*S18-ROcp19_710*C18);
    ROcp19_818 = -(ROcp19_56*S18-ROcp19_810*C18);
    ROcp19_918 = ROcp19_910*C18-S18*S6;
    ROcp19_419 = ROcp19_418*C19+ROcp19_718*S19;
    ROcp19_519 = ROcp19_518*C19+ROcp19_818*S19;
    ROcp19_619 = ROcp19_618*C19+ROcp19_918*S19;
    ROcp19_719 = -(ROcp19_418*S19-ROcp19_718*C19);
    ROcp19_819 = -(ROcp19_518*S19-ROcp19_818*C19);
    ROcp19_919 = -(ROcp19_618*S19-ROcp19_918*C19);
    ROcp19_120 = ROcp19_110*C20-ROcp19_719*S20;
    ROcp19_220 = ROcp19_210*C20-ROcp19_819*S20;
    ROcp19_320 = ROcp19_310*C20-ROcp19_919*S20;
    ROcp19_720 = ROcp19_110*S20+ROcp19_719*C20;
    ROcp19_820 = ROcp19_210*S20+ROcp19_819*C20;
    ROcp19_920 = ROcp19_310*S20+ROcp19_919*C20;
    RLcp19_118 = ROcp19_46*s->dpt[2][12]+ROcp19_710*s->dpt[3][12];
    RLcp19_218 = ROcp19_56*s->dpt[2][12]+ROcp19_810*s->dpt[3][12];
    RLcp19_318 = ROcp19_910*s->dpt[3][12]+s->dpt[2][12]*S6;
    OMcp19_118 = OMcp19_17+ROcp19_110*qd[18];
    OMcp19_218 = OMcp19_27+ROcp19_210*qd[18];
    OMcp19_318 = OMcp19_37+ROcp19_310*qd[18];
    ORcp19_118 = OMcp19_27*RLcp19_318-OMcp19_37*RLcp19_218;
    ORcp19_218 = -(OMcp19_17*RLcp19_318-OMcp19_37*RLcp19_118);
    ORcp19_318 = OMcp19_17*RLcp19_218-OMcp19_27*RLcp19_118;
    OPcp19_118 = OPcp19_17+ROcp19_110*qdd[18]+qd[18]*(OMcp19_27*ROcp19_310-OMcp19_37*ROcp19_210);
    OPcp19_218 = OPcp19_27+ROcp19_210*qdd[18]-qd[18]*(OMcp19_17*ROcp19_310-OMcp19_37*ROcp19_110);
    OPcp19_318 = OPcp19_37+ROcp19_310*qdd[18]+qd[18]*(OMcp19_17*ROcp19_210-OMcp19_27*ROcp19_110);
    RLcp19_119 = ROcp19_418*s->dpt[2][23];
    RLcp19_219 = ROcp19_518*s->dpt[2][23];
    RLcp19_319 = ROcp19_618*s->dpt[2][23];
    POcp19_119 = RLcp19_110+RLcp19_118+RLcp19_119+RLcp19_12+RLcp19_13;
    POcp19_219 = RLcp19_210+RLcp19_218+RLcp19_219+RLcp19_22+RLcp19_23;
    POcp19_319 = RLcp19_310+RLcp19_318+RLcp19_319+q[4];
    OMcp19_119 = OMcp19_118+ROcp19_110*qd[19];
    OMcp19_219 = OMcp19_218+ROcp19_210*qd[19];
    OMcp19_319 = OMcp19_318+ROcp19_310*qd[19];
    ORcp19_119 = OMcp19_218*RLcp19_319-OMcp19_318*RLcp19_219;
    ORcp19_219 = -(OMcp19_118*RLcp19_319-OMcp19_318*RLcp19_119);
    ORcp19_319 = OMcp19_118*RLcp19_219-OMcp19_218*RLcp19_119;
    VIcp19_119 = ORcp19_110+ORcp19_118+ORcp19_119+ORcp19_12+ORcp19_13+qd[2]*C1-qd[3]*S1;
    VIcp19_219 = ORcp19_210+ORcp19_218+ORcp19_219+ORcp19_22+ORcp19_23+qd[2]*S1+qd[3]*C1;
    VIcp19_319 = ORcp19_310+ORcp19_318+ORcp19_319+qd[4];
    ACcp19_119 = OMcp19_218*ORcp19_319+OMcp19_27*(ORcp19_310+ORcp19_318)-OMcp19_318*ORcp19_219-OMcp19_37*ORcp19_210-
 OMcp19_37*ORcp19_218+OPcp19_218*RLcp19_319+OPcp19_27*RLcp19_310+OPcp19_27*RLcp19_318-OPcp19_318*RLcp19_219-OPcp19_37*
 RLcp19_210-OPcp19_37*RLcp19_218-ORcp19_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp19_22+(2.0)*qd[2]*S1);
    ACcp19_219 = -(OMcp19_118*ORcp19_319+OMcp19_17*(ORcp19_310+ORcp19_318)-OMcp19_318*ORcp19_119-OMcp19_37*ORcp19_110-
 OMcp19_37*ORcp19_118+OPcp19_118*RLcp19_319+OPcp19_17*RLcp19_310+OPcp19_17*RLcp19_318-OPcp19_318*RLcp19_119-OPcp19_37*
 RLcp19_110-OPcp19_37*RLcp19_118-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp19_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp19_13-(2.0)*qd[3]*S1));
    ACcp19_319 = qdd[4]+OMcp19_118*ORcp19_219+OMcp19_17*ORcp19_210+OMcp19_17*ORcp19_218-OMcp19_218*ORcp19_119-OMcp19_27*
 ORcp19_110-OMcp19_27*ORcp19_118+OPcp19_118*RLcp19_219+OPcp19_17*RLcp19_210+OPcp19_17*RLcp19_218-OPcp19_218*RLcp19_119-
 OPcp19_27*RLcp19_110-OPcp19_27*RLcp19_118;
    OMcp19_120 = OMcp19_119+ROcp19_419*qd[20];
    OMcp19_220 = OMcp19_219+ROcp19_519*qd[20];
    OMcp19_320 = OMcp19_319+ROcp19_619*qd[20];
    OPcp19_120 = OPcp19_118+ROcp19_110*qdd[19]+ROcp19_419*qdd[20]+qd[19]*(OMcp19_218*ROcp19_310-OMcp19_318*ROcp19_210)+
 qd[20]*(OMcp19_219*ROcp19_619-OMcp19_319*ROcp19_519);
    OPcp19_220 = OPcp19_218+ROcp19_210*qdd[19]+ROcp19_519*qdd[20]-qd[19]*(OMcp19_118*ROcp19_310-OMcp19_318*ROcp19_110)-
 qd[20]*(OMcp19_119*ROcp19_619-OMcp19_319*ROcp19_419);
    OPcp19_320 = OPcp19_318+ROcp19_310*qdd[19]+ROcp19_619*qdd[20]+qd[19]*(OMcp19_118*ROcp19_210-OMcp19_218*ROcp19_110)+
 qd[20]*(OMcp19_119*ROcp19_519-OMcp19_219*ROcp19_419);

// = = Block_1_0_0_20_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp19_119;
    sens->P[2] = POcp19_219;
    sens->P[3] = POcp19_319;
    sens->R[1][1] = ROcp19_120;
    sens->R[1][2] = ROcp19_220;
    sens->R[1][3] = ROcp19_320;
    sens->R[2][1] = ROcp19_419;
    sens->R[2][2] = ROcp19_519;
    sens->R[2][3] = ROcp19_619;
    sens->R[3][1] = ROcp19_720;
    sens->R[3][2] = ROcp19_820;
    sens->R[3][3] = ROcp19_920;
    sens->V[1] = VIcp19_119;
    sens->V[2] = VIcp19_219;
    sens->V[3] = VIcp19_319;
    sens->OM[1] = OMcp19_120;
    sens->OM[2] = OMcp19_220;
    sens->OM[3] = OMcp19_320;
    sens->A[1] = ACcp19_119;
    sens->A[2] = ACcp19_219;
    sens->A[3] = ACcp19_319;
    sens->OMP[1] = OPcp19_120;
    sens->OMP[2] = OPcp19_220;
    sens->OMP[3] = OPcp19_320;
 
// 
break;
case 21:
 


// = = Block_1_0_0_21_0_1 = = 
 
// Sensor Kinematics 


    ROcp20_46 = -S1p5*C6;
    ROcp20_56 = C1p5*C6;
    ROcp20_76 = S1p5*S6;
    ROcp20_86 = -C1p5*S6;
    ROcp20_17 = -(ROcp20_76*S7-C1p5*C7);
    ROcp20_27 = -(ROcp20_86*S7-S1p5*C7);
    ROcp20_77 = ROcp20_76*C7+C1p5*S7;
    ROcp20_87 = ROcp20_86*C7+S1p5*S7;
    RLcp20_12 = q[2]*C1;
    RLcp20_22 = q[2]*S1;
    ORcp20_12 = -RLcp20_22*qd[1];
    ORcp20_22 = RLcp20_12*qd[1];
    RLcp20_13 = -q[3]*S1;
    RLcp20_23 = q[3]*C1;
    ORcp20_13 = -RLcp20_23*qd[1];
    ORcp20_23 = RLcp20_13*qd[1];
    OMcp20_35 = qd[1]+qd[5];
    OMcp20_16 = qd[6]*C1p5;
    OMcp20_26 = qd[6]*S1p5;
    OMcp20_17 = OMcp20_16+ROcp20_46*qd[7];
    OMcp20_27 = OMcp20_26+ROcp20_56*qd[7];
    OMcp20_37 = OMcp20_35+qd[7]*S6;
    OPcp20_17 = -(OMcp20_35*qd[6]*S1p5-ROcp20_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp20_26*S6-OMcp20_35*ROcp20_56));
    OPcp20_27 = OMcp20_35*qd[6]*C1p5+ROcp20_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp20_16*S6-OMcp20_35*ROcp20_46);
    OPcp20_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_21_0_3 = = 
 
// Sensor Kinematics 


    ROcp20_110 = ROcp20_17*C10-ROcp20_77*S10;
    ROcp20_210 = ROcp20_27*C10-ROcp20_87*S10;
    ROcp20_310 = -S10p7*C6;
    ROcp20_710 = ROcp20_17*S10+ROcp20_77*C10;
    ROcp20_810 = ROcp20_27*S10+ROcp20_87*C10;
    ROcp20_910 = C10p7*C6;
    RLcp20_110 = ROcp20_17*s->dpt[1][5]+ROcp20_77*s->dpt[3][5];
    RLcp20_210 = ROcp20_27*s->dpt[1][5]+ROcp20_87*s->dpt[3][5];
    RLcp20_310 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp20_110 = OMcp20_27*RLcp20_310-OMcp20_37*RLcp20_210;
    ORcp20_210 = -(OMcp20_17*RLcp20_310-OMcp20_37*RLcp20_110);
    ORcp20_310 = OMcp20_17*RLcp20_210-OMcp20_27*RLcp20_110;

// = = Block_1_0_0_21_0_6 = = 
 
// Sensor Kinematics 


    ROcp20_418 = ROcp20_46*C18+ROcp20_710*S18;
    ROcp20_518 = ROcp20_56*C18+ROcp20_810*S18;
    ROcp20_618 = ROcp20_910*S18+C18*S6;
    ROcp20_718 = -(ROcp20_46*S18-ROcp20_710*C18);
    ROcp20_818 = -(ROcp20_56*S18-ROcp20_810*C18);
    ROcp20_918 = ROcp20_910*C18-S18*S6;
    ROcp20_419 = ROcp20_418*C19+ROcp20_718*S19;
    ROcp20_519 = ROcp20_518*C19+ROcp20_818*S19;
    ROcp20_619 = ROcp20_618*C19+ROcp20_918*S19;
    ROcp20_719 = -(ROcp20_418*S19-ROcp20_718*C19);
    ROcp20_819 = -(ROcp20_518*S19-ROcp20_818*C19);
    ROcp20_919 = -(ROcp20_618*S19-ROcp20_918*C19);
    ROcp20_120 = ROcp20_110*C20-ROcp20_719*S20;
    ROcp20_220 = ROcp20_210*C20-ROcp20_819*S20;
    ROcp20_320 = ROcp20_310*C20-ROcp20_919*S20;
    ROcp20_720 = ROcp20_110*S20+ROcp20_719*C20;
    ROcp20_820 = ROcp20_210*S20+ROcp20_819*C20;
    ROcp20_920 = ROcp20_310*S20+ROcp20_919*C20;
    ROcp20_121 = ROcp20_120*C21+ROcp20_419*S21;
    ROcp20_221 = ROcp20_220*C21+ROcp20_519*S21;
    ROcp20_321 = ROcp20_320*C21+ROcp20_619*S21;
    ROcp20_421 = -(ROcp20_120*S21-ROcp20_419*C21);
    ROcp20_521 = -(ROcp20_220*S21-ROcp20_519*C21);
    ROcp20_621 = -(ROcp20_320*S21-ROcp20_619*C21);
    RLcp20_118 = ROcp20_46*s->dpt[2][12]+ROcp20_710*s->dpt[3][12];
    RLcp20_218 = ROcp20_56*s->dpt[2][12]+ROcp20_810*s->dpt[3][12];
    RLcp20_318 = ROcp20_910*s->dpt[3][12]+s->dpt[2][12]*S6;
    OMcp20_118 = OMcp20_17+ROcp20_110*qd[18];
    OMcp20_218 = OMcp20_27+ROcp20_210*qd[18];
    OMcp20_318 = OMcp20_37+ROcp20_310*qd[18];
    ORcp20_118 = OMcp20_27*RLcp20_318-OMcp20_37*RLcp20_218;
    ORcp20_218 = -(OMcp20_17*RLcp20_318-OMcp20_37*RLcp20_118);
    ORcp20_318 = OMcp20_17*RLcp20_218-OMcp20_27*RLcp20_118;
    OPcp20_118 = OPcp20_17+ROcp20_110*qdd[18]+qd[18]*(OMcp20_27*ROcp20_310-OMcp20_37*ROcp20_210);
    OPcp20_218 = OPcp20_27+ROcp20_210*qdd[18]-qd[18]*(OMcp20_17*ROcp20_310-OMcp20_37*ROcp20_110);
    OPcp20_318 = OPcp20_37+ROcp20_310*qdd[18]+qd[18]*(OMcp20_17*ROcp20_210-OMcp20_27*ROcp20_110);
    RLcp20_119 = ROcp20_418*s->dpt[2][23];
    RLcp20_219 = ROcp20_518*s->dpt[2][23];
    RLcp20_319 = ROcp20_618*s->dpt[2][23];
    POcp20_119 = RLcp20_110+RLcp20_118+RLcp20_119+RLcp20_12+RLcp20_13;
    POcp20_219 = RLcp20_210+RLcp20_218+RLcp20_219+RLcp20_22+RLcp20_23;
    POcp20_319 = RLcp20_310+RLcp20_318+RLcp20_319+q[4];
    OMcp20_119 = OMcp20_118+ROcp20_110*qd[19];
    OMcp20_219 = OMcp20_218+ROcp20_210*qd[19];
    OMcp20_319 = OMcp20_318+ROcp20_310*qd[19];
    ORcp20_119 = OMcp20_218*RLcp20_319-OMcp20_318*RLcp20_219;
    ORcp20_219 = -(OMcp20_118*RLcp20_319-OMcp20_318*RLcp20_119);
    ORcp20_319 = OMcp20_118*RLcp20_219-OMcp20_218*RLcp20_119;
    VIcp20_119 = ORcp20_110+ORcp20_118+ORcp20_119+ORcp20_12+ORcp20_13+qd[2]*C1-qd[3]*S1;
    VIcp20_219 = ORcp20_210+ORcp20_218+ORcp20_219+ORcp20_22+ORcp20_23+qd[2]*S1+qd[3]*C1;
    VIcp20_319 = ORcp20_310+ORcp20_318+ORcp20_319+qd[4];
    ACcp20_119 = OMcp20_218*ORcp20_319+OMcp20_27*(ORcp20_310+ORcp20_318)-OMcp20_318*ORcp20_219-OMcp20_37*ORcp20_210-
 OMcp20_37*ORcp20_218+OPcp20_218*RLcp20_319+OPcp20_27*RLcp20_310+OPcp20_27*RLcp20_318-OPcp20_318*RLcp20_219-OPcp20_37*
 RLcp20_210-OPcp20_37*RLcp20_218-ORcp20_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp20_22+(2.0)*qd[2]*S1);
    ACcp20_219 = -(OMcp20_118*ORcp20_319+OMcp20_17*(ORcp20_310+ORcp20_318)-OMcp20_318*ORcp20_119-OMcp20_37*ORcp20_110-
 OMcp20_37*ORcp20_118+OPcp20_118*RLcp20_319+OPcp20_17*RLcp20_310+OPcp20_17*RLcp20_318-OPcp20_318*RLcp20_119-OPcp20_37*
 RLcp20_110-OPcp20_37*RLcp20_118-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp20_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp20_13-(2.0)*qd[3]*S1));
    ACcp20_319 = qdd[4]+OMcp20_118*ORcp20_219+OMcp20_17*ORcp20_210+OMcp20_17*ORcp20_218-OMcp20_218*ORcp20_119-OMcp20_27*
 ORcp20_110-OMcp20_27*ORcp20_118+OPcp20_118*RLcp20_219+OPcp20_17*RLcp20_210+OPcp20_17*RLcp20_218-OPcp20_218*RLcp20_119-
 OPcp20_27*RLcp20_110-OPcp20_27*RLcp20_118;
    OMcp20_120 = OMcp20_119+ROcp20_419*qd[20];
    OMcp20_220 = OMcp20_219+ROcp20_519*qd[20];
    OMcp20_320 = OMcp20_319+ROcp20_619*qd[20];
    OMcp20_121 = OMcp20_120+ROcp20_720*qd[21];
    OMcp20_221 = OMcp20_220+ROcp20_820*qd[21];
    OMcp20_321 = OMcp20_320+ROcp20_920*qd[21];
    OPcp20_121 = OPcp20_118+ROcp20_110*qdd[19]+ROcp20_419*qdd[20]+ROcp20_720*qdd[21]+qd[19]*(OMcp20_218*ROcp20_310-
 OMcp20_318*ROcp20_210)+qd[20]*(OMcp20_219*ROcp20_619-OMcp20_319*ROcp20_519)+qd[21]*(OMcp20_220*ROcp20_920-OMcp20_320*
 ROcp20_820);
    OPcp20_221 = OPcp20_218+ROcp20_210*qdd[19]+ROcp20_519*qdd[20]+ROcp20_820*qdd[21]-qd[19]*(OMcp20_118*ROcp20_310-
 OMcp20_318*ROcp20_110)-qd[20]*(OMcp20_119*ROcp20_619-OMcp20_319*ROcp20_419)-qd[21]*(OMcp20_120*ROcp20_920-OMcp20_320*
 ROcp20_720);
    OPcp20_321 = OPcp20_318+ROcp20_310*qdd[19]+ROcp20_619*qdd[20]+ROcp20_920*qdd[21]+qd[19]*(OMcp20_118*ROcp20_210-
 OMcp20_218*ROcp20_110)+qd[20]*(OMcp20_119*ROcp20_519-OMcp20_219*ROcp20_419)+qd[21]*(OMcp20_120*ROcp20_820-OMcp20_220*
 ROcp20_720);

// = = Block_1_0_0_21_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp20_119;
    sens->P[2] = POcp20_219;
    sens->P[3] = POcp20_319;
    sens->R[1][1] = ROcp20_121;
    sens->R[1][2] = ROcp20_221;
    sens->R[1][3] = ROcp20_321;
    sens->R[2][1] = ROcp20_421;
    sens->R[2][2] = ROcp20_521;
    sens->R[2][3] = ROcp20_621;
    sens->R[3][1] = ROcp20_720;
    sens->R[3][2] = ROcp20_820;
    sens->R[3][3] = ROcp20_920;
    sens->V[1] = VIcp20_119;
    sens->V[2] = VIcp20_219;
    sens->V[3] = VIcp20_319;
    sens->OM[1] = OMcp20_121;
    sens->OM[2] = OMcp20_221;
    sens->OM[3] = OMcp20_321;
    sens->A[1] = ACcp20_119;
    sens->A[2] = ACcp20_219;
    sens->A[3] = ACcp20_319;
    sens->OMP[1] = OPcp20_121;
    sens->OMP[2] = OPcp20_221;
    sens->OMP[3] = OPcp20_321;
 
// 
break;
case 22:
 


// = = Block_1_0_0_22_0_1 = = 
 
// Sensor Kinematics 


    ROcp21_46 = -S1p5*C6;
    ROcp21_56 = C1p5*C6;
    ROcp21_76 = S1p5*S6;
    ROcp21_86 = -C1p5*S6;
    ROcp21_17 = -(ROcp21_76*S7-C1p5*C7);
    ROcp21_27 = -(ROcp21_86*S7-S1p5*C7);
    ROcp21_77 = ROcp21_76*C7+C1p5*S7;
    ROcp21_87 = ROcp21_86*C7+S1p5*S7;
    RLcp21_12 = q[2]*C1;
    RLcp21_22 = q[2]*S1;
    ORcp21_12 = -RLcp21_22*qd[1];
    ORcp21_22 = RLcp21_12*qd[1];
    RLcp21_13 = -q[3]*S1;
    RLcp21_23 = q[3]*C1;
    ORcp21_13 = -RLcp21_23*qd[1];
    ORcp21_23 = RLcp21_13*qd[1];
    OMcp21_35 = qd[1]+qd[5];
    OMcp21_16 = qd[6]*C1p5;
    OMcp21_26 = qd[6]*S1p5;
    OMcp21_17 = OMcp21_16+ROcp21_46*qd[7];
    OMcp21_27 = OMcp21_26+ROcp21_56*qd[7];
    OMcp21_37 = OMcp21_35+qd[7]*S6;
    OPcp21_17 = -(OMcp21_35*qd[6]*S1p5-ROcp21_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp21_26*S6-OMcp21_35*ROcp21_56));
    OPcp21_27 = OMcp21_35*qd[6]*C1p5+ROcp21_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp21_16*S6-OMcp21_35*ROcp21_46);
    OPcp21_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_22_0_3 = = 
 
// Sensor Kinematics 


    ROcp21_110 = ROcp21_17*C10-ROcp21_77*S10;
    ROcp21_210 = ROcp21_27*C10-ROcp21_87*S10;
    ROcp21_310 = -S10p7*C6;
    ROcp21_710 = ROcp21_17*S10+ROcp21_77*C10;
    ROcp21_810 = ROcp21_27*S10+ROcp21_87*C10;
    ROcp21_910 = C10p7*C6;
    RLcp21_110 = ROcp21_17*s->dpt[1][5]+ROcp21_77*s->dpt[3][5];
    RLcp21_210 = ROcp21_27*s->dpt[1][5]+ROcp21_87*s->dpt[3][5];
    RLcp21_310 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp21_110 = OMcp21_27*RLcp21_310-OMcp21_37*RLcp21_210;
    ORcp21_210 = -(OMcp21_17*RLcp21_310-OMcp21_37*RLcp21_110);
    ORcp21_310 = OMcp21_17*RLcp21_210-OMcp21_27*RLcp21_110;

// = = Block_1_0_0_22_0_6 = = 
 
// Sensor Kinematics 


    ROcp21_418 = ROcp21_46*C18+ROcp21_710*S18;
    ROcp21_518 = ROcp21_56*C18+ROcp21_810*S18;
    ROcp21_618 = ROcp21_910*S18+C18*S6;
    ROcp21_718 = -(ROcp21_46*S18-ROcp21_710*C18);
    ROcp21_818 = -(ROcp21_56*S18-ROcp21_810*C18);
    ROcp21_918 = ROcp21_910*C18-S18*S6;
    ROcp21_419 = ROcp21_418*C19+ROcp21_718*S19;
    ROcp21_519 = ROcp21_518*C19+ROcp21_818*S19;
    ROcp21_619 = ROcp21_618*C19+ROcp21_918*S19;
    ROcp21_719 = -(ROcp21_418*S19-ROcp21_718*C19);
    ROcp21_819 = -(ROcp21_518*S19-ROcp21_818*C19);
    ROcp21_919 = -(ROcp21_618*S19-ROcp21_918*C19);
    ROcp21_120 = ROcp21_110*C20-ROcp21_719*S20;
    ROcp21_220 = ROcp21_210*C20-ROcp21_819*S20;
    ROcp21_320 = ROcp21_310*C20-ROcp21_919*S20;
    ROcp21_720 = ROcp21_110*S20+ROcp21_719*C20;
    ROcp21_820 = ROcp21_210*S20+ROcp21_819*C20;
    ROcp21_920 = ROcp21_310*S20+ROcp21_919*C20;
    ROcp21_121 = ROcp21_120*C21+ROcp21_419*S21;
    ROcp21_221 = ROcp21_220*C21+ROcp21_519*S21;
    ROcp21_321 = ROcp21_320*C21+ROcp21_619*S21;
    ROcp21_421 = -(ROcp21_120*S21-ROcp21_419*C21);
    ROcp21_521 = -(ROcp21_220*S21-ROcp21_519*C21);
    ROcp21_621 = -(ROcp21_320*S21-ROcp21_619*C21);
    ROcp21_422 = ROcp21_421*C22+ROcp21_720*S22;
    ROcp21_522 = ROcp21_521*C22+ROcp21_820*S22;
    ROcp21_622 = ROcp21_621*C22+ROcp21_920*S22;
    ROcp21_722 = -(ROcp21_421*S22-ROcp21_720*C22);
    ROcp21_822 = -(ROcp21_521*S22-ROcp21_820*C22);
    ROcp21_922 = -(ROcp21_621*S22-ROcp21_920*C22);
    RLcp21_118 = ROcp21_46*s->dpt[2][12]+ROcp21_710*s->dpt[3][12];
    RLcp21_218 = ROcp21_56*s->dpt[2][12]+ROcp21_810*s->dpt[3][12];
    RLcp21_318 = ROcp21_910*s->dpt[3][12]+s->dpt[2][12]*S6;
    OMcp21_118 = OMcp21_17+ROcp21_110*qd[18];
    OMcp21_218 = OMcp21_27+ROcp21_210*qd[18];
    OMcp21_318 = OMcp21_37+ROcp21_310*qd[18];
    ORcp21_118 = OMcp21_27*RLcp21_318-OMcp21_37*RLcp21_218;
    ORcp21_218 = -(OMcp21_17*RLcp21_318-OMcp21_37*RLcp21_118);
    ORcp21_318 = OMcp21_17*RLcp21_218-OMcp21_27*RLcp21_118;
    OPcp21_118 = OPcp21_17+ROcp21_110*qdd[18]+qd[18]*(OMcp21_27*ROcp21_310-OMcp21_37*ROcp21_210);
    OPcp21_218 = OPcp21_27+ROcp21_210*qdd[18]-qd[18]*(OMcp21_17*ROcp21_310-OMcp21_37*ROcp21_110);
    OPcp21_318 = OPcp21_37+ROcp21_310*qdd[18]+qd[18]*(OMcp21_17*ROcp21_210-OMcp21_27*ROcp21_110);
    RLcp21_119 = ROcp21_418*s->dpt[2][23];
    RLcp21_219 = ROcp21_518*s->dpt[2][23];
    RLcp21_319 = ROcp21_618*s->dpt[2][23];
    OMcp21_119 = OMcp21_118+ROcp21_110*qd[19];
    OMcp21_219 = OMcp21_218+ROcp21_210*qd[19];
    OMcp21_319 = OMcp21_318+ROcp21_310*qd[19];
    ORcp21_119 = OMcp21_218*RLcp21_319-OMcp21_318*RLcp21_219;
    ORcp21_219 = -(OMcp21_118*RLcp21_319-OMcp21_318*RLcp21_119);
    ORcp21_319 = OMcp21_118*RLcp21_219-OMcp21_218*RLcp21_119;
    OMcp21_120 = OMcp21_119+ROcp21_419*qd[20];
    OMcp21_220 = OMcp21_219+ROcp21_519*qd[20];
    OMcp21_320 = OMcp21_319+ROcp21_619*qd[20];
    OMcp21_121 = OMcp21_120+ROcp21_720*qd[21];
    OMcp21_221 = OMcp21_220+ROcp21_820*qd[21];
    OMcp21_321 = OMcp21_320+ROcp21_920*qd[21];
    OPcp21_121 = OPcp21_118+ROcp21_110*qdd[19]+ROcp21_419*qdd[20]+ROcp21_720*qdd[21]+qd[19]*(OMcp21_218*ROcp21_310-
 OMcp21_318*ROcp21_210)+qd[20]*(OMcp21_219*ROcp21_619-OMcp21_319*ROcp21_519)+qd[21]*(OMcp21_220*ROcp21_920-OMcp21_320*
 ROcp21_820);
    OPcp21_221 = OPcp21_218+ROcp21_210*qdd[19]+ROcp21_519*qdd[20]+ROcp21_820*qdd[21]-qd[19]*(OMcp21_118*ROcp21_310-
 OMcp21_318*ROcp21_110)-qd[20]*(OMcp21_119*ROcp21_619-OMcp21_319*ROcp21_419)-qd[21]*(OMcp21_120*ROcp21_920-OMcp21_320*
 ROcp21_720);
    OPcp21_321 = OPcp21_318+ROcp21_310*qdd[19]+ROcp21_619*qdd[20]+ROcp21_920*qdd[21]+qd[19]*(OMcp21_118*ROcp21_210-
 OMcp21_218*ROcp21_110)+qd[20]*(OMcp21_119*ROcp21_519-OMcp21_219*ROcp21_419)+qd[21]*(OMcp21_120*ROcp21_820-OMcp21_220*
 ROcp21_720);
    RLcp21_122 = ROcp21_121*s->dpt[1][27]+ROcp21_421*s->dpt[2][27]+ROcp21_720*s->dpt[3][27];
    RLcp21_222 = ROcp21_221*s->dpt[1][27]+ROcp21_521*s->dpt[2][27]+ROcp21_820*s->dpt[3][27];
    RLcp21_322 = ROcp21_321*s->dpt[1][27]+ROcp21_621*s->dpt[2][27]+ROcp21_920*s->dpt[3][27];
    POcp21_122 = RLcp21_110+RLcp21_118+RLcp21_119+RLcp21_12+RLcp21_122+RLcp21_13;
    POcp21_222 = RLcp21_210+RLcp21_218+RLcp21_219+RLcp21_22+RLcp21_222+RLcp21_23;
    POcp21_322 = RLcp21_310+RLcp21_318+RLcp21_319+RLcp21_322+q[4];
    ORcp21_122 = OMcp21_221*RLcp21_322-OMcp21_321*RLcp21_222;
    ORcp21_222 = -(OMcp21_121*RLcp21_322-OMcp21_321*RLcp21_122);
    ORcp21_322 = OMcp21_121*RLcp21_222-OMcp21_221*RLcp21_122;
    VIcp21_122 = ORcp21_110+ORcp21_118+ORcp21_119+ORcp21_12+ORcp21_122+ORcp21_13+qd[2]*C1-qd[3]*S1;
    VIcp21_222 = ORcp21_210+ORcp21_218+ORcp21_219+ORcp21_22+ORcp21_222+ORcp21_23+qd[2]*S1+qd[3]*C1;
    VIcp21_322 = ORcp21_310+ORcp21_318+ORcp21_319+ORcp21_322+qd[4];
    ACcp21_122 = OMcp21_218*ORcp21_319+OMcp21_221*ORcp21_322+OMcp21_27*(ORcp21_310+ORcp21_318)-OMcp21_318*ORcp21_219-
 OMcp21_321*ORcp21_222-OMcp21_37*ORcp21_210-OMcp21_37*ORcp21_218+OPcp21_218*RLcp21_319+OPcp21_221*RLcp21_322+OPcp21_27*
 RLcp21_310+OPcp21_27*RLcp21_318-OPcp21_318*RLcp21_219-OPcp21_321*RLcp21_222-OPcp21_37*RLcp21_210-OPcp21_37*RLcp21_218-
 ORcp21_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp21_22+(2.0)*qd[2]*S1);
    ACcp21_222 = -(OMcp21_118*ORcp21_319+OMcp21_121*ORcp21_322+OMcp21_17*(ORcp21_310+ORcp21_318)-OMcp21_318*ORcp21_119-
 OMcp21_321*ORcp21_122-OMcp21_37*ORcp21_110-OMcp21_37*ORcp21_118+OPcp21_118*RLcp21_319+OPcp21_121*RLcp21_322+OPcp21_17*
 RLcp21_310+OPcp21_17*RLcp21_318-OPcp21_318*RLcp21_119-OPcp21_321*RLcp21_122-OPcp21_37*RLcp21_110-OPcp21_37*RLcp21_118-qdd[2]
 *S1-qdd[3]*C1-qd[1]*(ORcp21_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp21_13-(2.0)*qd[3]*S1));
    ACcp21_322 = qdd[4]+OMcp21_118*ORcp21_219+OMcp21_121*ORcp21_222+OMcp21_17*ORcp21_210+OMcp21_17*ORcp21_218-OMcp21_218*
 ORcp21_119-OMcp21_221*ORcp21_122-OMcp21_27*ORcp21_110-OMcp21_27*ORcp21_118+OPcp21_118*RLcp21_219+OPcp21_121*RLcp21_222+
 OPcp21_17*RLcp21_210+OPcp21_17*RLcp21_218-OPcp21_218*RLcp21_119-OPcp21_221*RLcp21_122-OPcp21_27*RLcp21_110-OPcp21_27*
 RLcp21_118;

// = = Block_1_0_0_22_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp21_122;
    sens->P[2] = POcp21_222;
    sens->P[3] = POcp21_322;
    sens->R[1][1] = ROcp21_121;
    sens->R[1][2] = ROcp21_221;
    sens->R[1][3] = ROcp21_321;
    sens->R[2][1] = ROcp21_422;
    sens->R[2][2] = ROcp21_522;
    sens->R[2][3] = ROcp21_622;
    sens->R[3][1] = ROcp21_722;
    sens->R[3][2] = ROcp21_822;
    sens->R[3][3] = ROcp21_922;
    sens->V[1] = VIcp21_122;
    sens->V[2] = VIcp21_222;
    sens->V[3] = VIcp21_322;
    sens->OM[1] = OMcp21_121;
    sens->OM[2] = OMcp21_221;
    sens->OM[3] = OMcp21_321;
    sens->A[1] = ACcp21_122;
    sens->A[2] = ACcp21_222;
    sens->A[3] = ACcp21_322;
    sens->OMP[1] = OPcp21_121;
    sens->OMP[2] = OPcp21_221;
    sens->OMP[3] = OPcp21_321;
 
// 
break;
case 23:
 


// = = Block_1_0_0_23_0_1 = = 
 
// Sensor Kinematics 


    ROcp22_46 = -S1p5*C6;
    ROcp22_56 = C1p5*C6;
    ROcp22_76 = S1p5*S6;
    ROcp22_86 = -C1p5*S6;
    ROcp22_17 = -(ROcp22_76*S7-C1p5*C7);
    ROcp22_27 = -(ROcp22_86*S7-S1p5*C7);
    ROcp22_77 = ROcp22_76*C7+C1p5*S7;
    ROcp22_87 = ROcp22_86*C7+S1p5*S7;
    RLcp22_12 = q[2]*C1;
    RLcp22_22 = q[2]*S1;
    ORcp22_12 = -RLcp22_22*qd[1];
    ORcp22_22 = RLcp22_12*qd[1];
    RLcp22_13 = -q[3]*S1;
    RLcp22_23 = q[3]*C1;
    ORcp22_13 = -RLcp22_23*qd[1];
    ORcp22_23 = RLcp22_13*qd[1];
    OMcp22_35 = qd[1]+qd[5];
    OMcp22_16 = qd[6]*C1p5;
    OMcp22_26 = qd[6]*S1p5;
    OMcp22_17 = OMcp22_16+ROcp22_46*qd[7];
    OMcp22_27 = OMcp22_26+ROcp22_56*qd[7];
    OMcp22_37 = OMcp22_35+qd[7]*S6;
    OPcp22_17 = -(OMcp22_35*qd[6]*S1p5-ROcp22_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp22_26*S6-OMcp22_35*ROcp22_56));
    OPcp22_27 = OMcp22_35*qd[6]*C1p5+ROcp22_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp22_16*S6-OMcp22_35*ROcp22_46);
    OPcp22_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_23_0_3 = = 
 
// Sensor Kinematics 


    ROcp22_110 = ROcp22_17*C10-ROcp22_77*S10;
    ROcp22_210 = ROcp22_27*C10-ROcp22_87*S10;
    ROcp22_310 = -S10p7*C6;
    ROcp22_710 = ROcp22_17*S10+ROcp22_77*C10;
    ROcp22_810 = ROcp22_27*S10+ROcp22_87*C10;
    ROcp22_910 = C10p7*C6;
    RLcp22_110 = ROcp22_17*s->dpt[1][5]+ROcp22_77*s->dpt[3][5];
    RLcp22_210 = ROcp22_27*s->dpt[1][5]+ROcp22_87*s->dpt[3][5];
    RLcp22_310 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp22_110 = OMcp22_27*RLcp22_310-OMcp22_37*RLcp22_210;
    ORcp22_210 = -(OMcp22_17*RLcp22_310-OMcp22_37*RLcp22_110);
    ORcp22_310 = OMcp22_17*RLcp22_210-OMcp22_27*RLcp22_110;

// = = Block_1_0_0_23_0_6 = = 
 
// Sensor Kinematics 


    ROcp22_418 = ROcp22_46*C18+ROcp22_710*S18;
    ROcp22_518 = ROcp22_56*C18+ROcp22_810*S18;
    ROcp22_618 = ROcp22_910*S18+C18*S6;
    ROcp22_718 = -(ROcp22_46*S18-ROcp22_710*C18);
    ROcp22_818 = -(ROcp22_56*S18-ROcp22_810*C18);
    ROcp22_918 = ROcp22_910*C18-S18*S6;
    ROcp22_419 = ROcp22_418*C19+ROcp22_718*S19;
    ROcp22_519 = ROcp22_518*C19+ROcp22_818*S19;
    ROcp22_619 = ROcp22_618*C19+ROcp22_918*S19;
    ROcp22_719 = -(ROcp22_418*S19-ROcp22_718*C19);
    ROcp22_819 = -(ROcp22_518*S19-ROcp22_818*C19);
    ROcp22_919 = -(ROcp22_618*S19-ROcp22_918*C19);
    ROcp22_120 = ROcp22_110*C20-ROcp22_719*S20;
    ROcp22_220 = ROcp22_210*C20-ROcp22_819*S20;
    ROcp22_320 = ROcp22_310*C20-ROcp22_919*S20;
    ROcp22_720 = ROcp22_110*S20+ROcp22_719*C20;
    ROcp22_820 = ROcp22_210*S20+ROcp22_819*C20;
    ROcp22_920 = ROcp22_310*S20+ROcp22_919*C20;
    ROcp22_121 = ROcp22_120*C21+ROcp22_419*S21;
    ROcp22_221 = ROcp22_220*C21+ROcp22_519*S21;
    ROcp22_321 = ROcp22_320*C21+ROcp22_619*S21;
    ROcp22_421 = -(ROcp22_120*S21-ROcp22_419*C21);
    ROcp22_521 = -(ROcp22_220*S21-ROcp22_519*C21);
    ROcp22_621 = -(ROcp22_320*S21-ROcp22_619*C21);
    ROcp22_422 = ROcp22_421*C22+ROcp22_720*S22;
    ROcp22_522 = ROcp22_521*C22+ROcp22_820*S22;
    ROcp22_622 = ROcp22_621*C22+ROcp22_920*S22;
    ROcp22_722 = -(ROcp22_421*S22-ROcp22_720*C22);
    ROcp22_822 = -(ROcp22_521*S22-ROcp22_820*C22);
    ROcp22_922 = -(ROcp22_621*S22-ROcp22_920*C22);
    ROcp22_123 = ROcp22_121*C23-ROcp22_722*S23;
    ROcp22_223 = ROcp22_221*C23-ROcp22_822*S23;
    ROcp22_323 = ROcp22_321*C23-ROcp22_922*S23;
    ROcp22_723 = ROcp22_121*S23+ROcp22_722*C23;
    ROcp22_823 = ROcp22_221*S23+ROcp22_822*C23;
    ROcp22_923 = ROcp22_321*S23+ROcp22_922*C23;
    RLcp22_118 = ROcp22_46*s->dpt[2][12]+ROcp22_710*s->dpt[3][12];
    RLcp22_218 = ROcp22_56*s->dpt[2][12]+ROcp22_810*s->dpt[3][12];
    RLcp22_318 = ROcp22_910*s->dpt[3][12]+s->dpt[2][12]*S6;
    OMcp22_118 = OMcp22_17+ROcp22_110*qd[18];
    OMcp22_218 = OMcp22_27+ROcp22_210*qd[18];
    OMcp22_318 = OMcp22_37+ROcp22_310*qd[18];
    ORcp22_118 = OMcp22_27*RLcp22_318-OMcp22_37*RLcp22_218;
    ORcp22_218 = -(OMcp22_17*RLcp22_318-OMcp22_37*RLcp22_118);
    ORcp22_318 = OMcp22_17*RLcp22_218-OMcp22_27*RLcp22_118;
    OPcp22_118 = OPcp22_17+ROcp22_110*qdd[18]+qd[18]*(OMcp22_27*ROcp22_310-OMcp22_37*ROcp22_210);
    OPcp22_218 = OPcp22_27+ROcp22_210*qdd[18]-qd[18]*(OMcp22_17*ROcp22_310-OMcp22_37*ROcp22_110);
    OPcp22_318 = OPcp22_37+ROcp22_310*qdd[18]+qd[18]*(OMcp22_17*ROcp22_210-OMcp22_27*ROcp22_110);
    RLcp22_119 = ROcp22_418*s->dpt[2][23];
    RLcp22_219 = ROcp22_518*s->dpt[2][23];
    RLcp22_319 = ROcp22_618*s->dpt[2][23];
    OMcp22_119 = OMcp22_118+ROcp22_110*qd[19];
    OMcp22_219 = OMcp22_218+ROcp22_210*qd[19];
    OMcp22_319 = OMcp22_318+ROcp22_310*qd[19];
    ORcp22_119 = OMcp22_218*RLcp22_319-OMcp22_318*RLcp22_219;
    ORcp22_219 = -(OMcp22_118*RLcp22_319-OMcp22_318*RLcp22_119);
    ORcp22_319 = OMcp22_118*RLcp22_219-OMcp22_218*RLcp22_119;
    OMcp22_120 = OMcp22_119+ROcp22_419*qd[20];
    OMcp22_220 = OMcp22_219+ROcp22_519*qd[20];
    OMcp22_320 = OMcp22_319+ROcp22_619*qd[20];
    OMcp22_121 = OMcp22_120+ROcp22_720*qd[21];
    OMcp22_221 = OMcp22_220+ROcp22_820*qd[21];
    OMcp22_321 = OMcp22_320+ROcp22_920*qd[21];
    OPcp22_121 = OPcp22_118+ROcp22_110*qdd[19]+ROcp22_419*qdd[20]+ROcp22_720*qdd[21]+qd[19]*(OMcp22_218*ROcp22_310-
 OMcp22_318*ROcp22_210)+qd[20]*(OMcp22_219*ROcp22_619-OMcp22_319*ROcp22_519)+qd[21]*(OMcp22_220*ROcp22_920-OMcp22_320*
 ROcp22_820);
    OPcp22_221 = OPcp22_218+ROcp22_210*qdd[19]+ROcp22_519*qdd[20]+ROcp22_820*qdd[21]-qd[19]*(OMcp22_118*ROcp22_310-
 OMcp22_318*ROcp22_110)-qd[20]*(OMcp22_119*ROcp22_619-OMcp22_319*ROcp22_419)-qd[21]*(OMcp22_120*ROcp22_920-OMcp22_320*
 ROcp22_720);
    OPcp22_321 = OPcp22_318+ROcp22_310*qdd[19]+ROcp22_619*qdd[20]+ROcp22_920*qdd[21]+qd[19]*(OMcp22_118*ROcp22_210-
 OMcp22_218*ROcp22_110)+qd[20]*(OMcp22_119*ROcp22_519-OMcp22_219*ROcp22_419)+qd[21]*(OMcp22_120*ROcp22_820-OMcp22_220*
 ROcp22_720);
    RLcp22_122 = ROcp22_121*s->dpt[1][27]+ROcp22_421*s->dpt[2][27]+ROcp22_720*s->dpt[3][27];
    RLcp22_222 = ROcp22_221*s->dpt[1][27]+ROcp22_521*s->dpt[2][27]+ROcp22_820*s->dpt[3][27];
    RLcp22_322 = ROcp22_321*s->dpt[1][27]+ROcp22_621*s->dpt[2][27]+ROcp22_920*s->dpt[3][27];
    POcp22_122 = RLcp22_110+RLcp22_118+RLcp22_119+RLcp22_12+RLcp22_122+RLcp22_13;
    POcp22_222 = RLcp22_210+RLcp22_218+RLcp22_219+RLcp22_22+RLcp22_222+RLcp22_23;
    POcp22_322 = RLcp22_310+RLcp22_318+RLcp22_319+RLcp22_322+q[4];
    ORcp22_122 = OMcp22_221*RLcp22_322-OMcp22_321*RLcp22_222;
    ORcp22_222 = -(OMcp22_121*RLcp22_322-OMcp22_321*RLcp22_122);
    ORcp22_322 = OMcp22_121*RLcp22_222-OMcp22_221*RLcp22_122;
    VIcp22_122 = ORcp22_110+ORcp22_118+ORcp22_119+ORcp22_12+ORcp22_122+ORcp22_13+qd[2]*C1-qd[3]*S1;
    VIcp22_222 = ORcp22_210+ORcp22_218+ORcp22_219+ORcp22_22+ORcp22_222+ORcp22_23+qd[2]*S1+qd[3]*C1;
    VIcp22_322 = ORcp22_310+ORcp22_318+ORcp22_319+ORcp22_322+qd[4];
    ACcp22_122 = OMcp22_218*ORcp22_319+OMcp22_221*ORcp22_322+OMcp22_27*(ORcp22_310+ORcp22_318)-OMcp22_318*ORcp22_219-
 OMcp22_321*ORcp22_222-OMcp22_37*ORcp22_210-OMcp22_37*ORcp22_218+OPcp22_218*RLcp22_319+OPcp22_221*RLcp22_322+OPcp22_27*
 RLcp22_310+OPcp22_27*RLcp22_318-OPcp22_318*RLcp22_219-OPcp22_321*RLcp22_222-OPcp22_37*RLcp22_210-OPcp22_37*RLcp22_218-
 ORcp22_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp22_22+(2.0)*qd[2]*S1);
    ACcp22_222 = -(OMcp22_118*ORcp22_319+OMcp22_121*ORcp22_322+OMcp22_17*(ORcp22_310+ORcp22_318)-OMcp22_318*ORcp22_119-
 OMcp22_321*ORcp22_122-OMcp22_37*ORcp22_110-OMcp22_37*ORcp22_118+OPcp22_118*RLcp22_319+OPcp22_121*RLcp22_322+OPcp22_17*
 RLcp22_310+OPcp22_17*RLcp22_318-OPcp22_318*RLcp22_119-OPcp22_321*RLcp22_122-OPcp22_37*RLcp22_110-OPcp22_37*RLcp22_118-qdd[2]
 *S1-qdd[3]*C1-qd[1]*(ORcp22_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp22_13-(2.0)*qd[3]*S1));
    ACcp22_322 = qdd[4]+OMcp22_118*ORcp22_219+OMcp22_121*ORcp22_222+OMcp22_17*ORcp22_210+OMcp22_17*ORcp22_218-OMcp22_218*
 ORcp22_119-OMcp22_221*ORcp22_122-OMcp22_27*ORcp22_110-OMcp22_27*ORcp22_118+OPcp22_118*RLcp22_219+OPcp22_121*RLcp22_222+
 OPcp22_17*RLcp22_210+OPcp22_17*RLcp22_218-OPcp22_218*RLcp22_119-OPcp22_221*RLcp22_122-OPcp22_27*RLcp22_110-OPcp22_27*
 RLcp22_118;
    OMcp22_123 = OMcp22_121+ROcp22_422*qd[23];
    OMcp22_223 = OMcp22_221+ROcp22_522*qd[23];
    OMcp22_323 = OMcp22_321+ROcp22_622*qd[23];
    OPcp22_123 = OPcp22_121+ROcp22_422*qdd[23]+qd[23]*(OMcp22_221*ROcp22_622-OMcp22_321*ROcp22_522);
    OPcp22_223 = OPcp22_221+ROcp22_522*qdd[23]-qd[23]*(OMcp22_121*ROcp22_622-OMcp22_321*ROcp22_422);
    OPcp22_323 = OPcp22_321+ROcp22_622*qdd[23]+qd[23]*(OMcp22_121*ROcp22_522-OMcp22_221*ROcp22_422);

// = = Block_1_0_0_23_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp22_122;
    sens->P[2] = POcp22_222;
    sens->P[3] = POcp22_322;
    sens->R[1][1] = ROcp22_123;
    sens->R[1][2] = ROcp22_223;
    sens->R[1][3] = ROcp22_323;
    sens->R[2][1] = ROcp22_422;
    sens->R[2][2] = ROcp22_522;
    sens->R[2][3] = ROcp22_622;
    sens->R[3][1] = ROcp22_723;
    sens->R[3][2] = ROcp22_823;
    sens->R[3][3] = ROcp22_923;
    sens->V[1] = VIcp22_122;
    sens->V[2] = VIcp22_222;
    sens->V[3] = VIcp22_322;
    sens->OM[1] = OMcp22_123;
    sens->OM[2] = OMcp22_223;
    sens->OM[3] = OMcp22_323;
    sens->A[1] = ACcp22_122;
    sens->A[2] = ACcp22_222;
    sens->A[3] = ACcp22_322;
    sens->OMP[1] = OPcp22_123;
    sens->OMP[2] = OPcp22_223;
    sens->OMP[3] = OPcp22_323;
 
// 
break;
case 24:
 


// = = Block_1_0_0_24_0_1 = = 
 
// Sensor Kinematics 


    ROcp23_46 = -S1p5*C6;
    ROcp23_56 = C1p5*C6;
    ROcp23_76 = S1p5*S6;
    ROcp23_86 = -C1p5*S6;
    ROcp23_17 = -(ROcp23_76*S7-C1p5*C7);
    ROcp23_27 = -(ROcp23_86*S7-S1p5*C7);
    ROcp23_77 = ROcp23_76*C7+C1p5*S7;
    ROcp23_87 = ROcp23_86*C7+S1p5*S7;
    RLcp23_12 = q[2]*C1;
    RLcp23_22 = q[2]*S1;
    ORcp23_12 = -RLcp23_22*qd[1];
    ORcp23_22 = RLcp23_12*qd[1];
    RLcp23_13 = -q[3]*S1;
    RLcp23_23 = q[3]*C1;
    ORcp23_13 = -RLcp23_23*qd[1];
    ORcp23_23 = RLcp23_13*qd[1];
    OMcp23_35 = qd[1]+qd[5];
    OMcp23_16 = qd[6]*C1p5;
    OMcp23_26 = qd[6]*S1p5;
    OMcp23_17 = OMcp23_16+ROcp23_46*qd[7];
    OMcp23_27 = OMcp23_26+ROcp23_56*qd[7];
    OMcp23_37 = OMcp23_35+qd[7]*S6;
    OPcp23_17 = -(OMcp23_35*qd[6]*S1p5-ROcp23_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp23_26*S6-OMcp23_35*ROcp23_56));
    OPcp23_27 = OMcp23_35*qd[6]*C1p5+ROcp23_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp23_16*S6-OMcp23_35*ROcp23_46);
    OPcp23_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_24_0_7 = = 
 
// Sensor Kinematics 


    ROcp23_124 = ROcp23_17*C24-ROcp23_77*S24;
    ROcp23_224 = ROcp23_27*C24-ROcp23_87*S24;
    ROcp23_324 = -S24p7*C6;
    ROcp23_724 = ROcp23_17*S24+ROcp23_77*C24;
    ROcp23_824 = ROcp23_27*S24+ROcp23_87*C24;
    ROcp23_924 = C24p7*C6;
    RLcp23_124 = ROcp23_17*s->dpt[1][6]+ROcp23_77*s->dpt[3][6];
    RLcp23_224 = ROcp23_27*s->dpt[1][6]+ROcp23_87*s->dpt[3][6];
    RLcp23_324 = -C6*(s->dpt[1][6]*S7-s->dpt[3][6]*C7);
    POcp23_124 = RLcp23_12+RLcp23_124+RLcp23_13;
    POcp23_224 = RLcp23_22+RLcp23_224+RLcp23_23;
    POcp23_324 = RLcp23_324+q[4];
    ORcp23_124 = OMcp23_27*RLcp23_324-OMcp23_37*RLcp23_224;
    ORcp23_224 = -(OMcp23_17*RLcp23_324-OMcp23_37*RLcp23_124);
    ORcp23_324 = OMcp23_17*RLcp23_224-OMcp23_27*RLcp23_124;
    VIcp23_124 = ORcp23_12+ORcp23_124+ORcp23_13+qd[2]*C1-qd[3]*S1;
    VIcp23_224 = ORcp23_22+ORcp23_224+ORcp23_23+qd[2]*S1+qd[3]*C1;
    VIcp23_324 = ORcp23_324+qd[4];
    ACcp23_124 = OMcp23_27*ORcp23_324-OMcp23_37*ORcp23_224+OPcp23_27*RLcp23_324-OPcp23_37*RLcp23_224-ORcp23_23*qd[1]+
 qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp23_22+(2.0)*qd[2]*S1);
    ACcp23_224 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp23_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp23_13-(2.0)*qd[3]*S1)-OMcp23_17*ORcp23_324+
 OMcp23_37*ORcp23_124-OPcp23_17*RLcp23_324+OPcp23_37*RLcp23_124;
    ACcp23_324 = qdd[4]+OMcp23_17*ORcp23_224-OMcp23_27*ORcp23_124+OPcp23_17*RLcp23_224-OPcp23_27*RLcp23_124;

// = = Block_1_0_0_24_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp23_124;
    sens->P[2] = POcp23_224;
    sens->P[3] = POcp23_324;
    sens->R[1][1] = ROcp23_124;
    sens->R[1][2] = ROcp23_224;
    sens->R[1][3] = ROcp23_324;
    sens->R[2][1] = ROcp23_46;
    sens->R[2][2] = ROcp23_56;
    sens->R[2][3] = S6;
    sens->R[3][1] = ROcp23_724;
    sens->R[3][2] = ROcp23_824;
    sens->R[3][3] = ROcp23_924;
    sens->V[1] = VIcp23_124;
    sens->V[2] = VIcp23_224;
    sens->V[3] = VIcp23_324;
    sens->OM[1] = OMcp23_17;
    sens->OM[2] = OMcp23_27;
    sens->OM[3] = OMcp23_37;
    sens->A[1] = ACcp23_124;
    sens->A[2] = ACcp23_224;
    sens->A[3] = ACcp23_324;
    sens->OMP[1] = OPcp23_17;
    sens->OMP[2] = OPcp23_27;
    sens->OMP[3] = OPcp23_37;
 
// 
break;
case 25:
 


// = = Block_1_0_0_25_0_1 = = 
 
// Sensor Kinematics 


    ROcp24_46 = -S1p5*C6;
    ROcp24_56 = C1p5*C6;
    ROcp24_76 = S1p5*S6;
    ROcp24_86 = -C1p5*S6;
    ROcp24_17 = -(ROcp24_76*S7-C1p5*C7);
    ROcp24_27 = -(ROcp24_86*S7-S1p5*C7);
    ROcp24_77 = ROcp24_76*C7+C1p5*S7;
    ROcp24_87 = ROcp24_86*C7+S1p5*S7;
    RLcp24_12 = q[2]*C1;
    RLcp24_22 = q[2]*S1;
    ORcp24_12 = -RLcp24_22*qd[1];
    ORcp24_22 = RLcp24_12*qd[1];
    RLcp24_13 = -q[3]*S1;
    RLcp24_23 = q[3]*C1;
    ORcp24_13 = -RLcp24_23*qd[1];
    ORcp24_23 = RLcp24_13*qd[1];
    OMcp24_35 = qd[1]+qd[5];
    OMcp24_16 = qd[6]*C1p5;
    OMcp24_26 = qd[6]*S1p5;
    OMcp24_17 = OMcp24_16+ROcp24_46*qd[7];
    OMcp24_27 = OMcp24_26+ROcp24_56*qd[7];
    OMcp24_37 = OMcp24_35+qd[7]*S6;
    OPcp24_17 = -(OMcp24_35*qd[6]*S1p5-ROcp24_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp24_26*S6-OMcp24_35*ROcp24_56));
    OPcp24_27 = OMcp24_35*qd[6]*C1p5+ROcp24_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp24_16*S6-OMcp24_35*ROcp24_46);
    OPcp24_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_25_0_7 = = 
 
// Sensor Kinematics 


    ROcp24_124 = ROcp24_17*C24-ROcp24_77*S24;
    ROcp24_224 = ROcp24_27*C24-ROcp24_87*S24;
    ROcp24_324 = -S24p7*C6;
    ROcp24_724 = ROcp24_17*S24+ROcp24_77*C24;
    ROcp24_824 = ROcp24_27*S24+ROcp24_87*C24;
    ROcp24_924 = C24p7*C6;
    ROcp24_125 = ROcp24_124*C25+ROcp24_46*S25;
    ROcp24_225 = ROcp24_224*C25+ROcp24_56*S25;
    ROcp24_325 = ROcp24_324*C25+S25*S6;
    ROcp24_425 = -(ROcp24_124*S25-ROcp24_46*C25);
    ROcp24_525 = -(ROcp24_224*S25-ROcp24_56*C25);
    ROcp24_625 = -(ROcp24_324*S25-C25*S6);
    RLcp24_124 = ROcp24_17*s->dpt[1][6]+ROcp24_77*s->dpt[3][6];
    RLcp24_224 = ROcp24_27*s->dpt[1][6]+ROcp24_87*s->dpt[3][6];
    RLcp24_324 = -C6*(s->dpt[1][6]*S7-s->dpt[3][6]*C7);
    POcp24_124 = RLcp24_12+RLcp24_124+RLcp24_13;
    POcp24_224 = RLcp24_22+RLcp24_224+RLcp24_23;
    POcp24_324 = RLcp24_324+q[4];
    ORcp24_124 = OMcp24_27*RLcp24_324-OMcp24_37*RLcp24_224;
    ORcp24_224 = -(OMcp24_17*RLcp24_324-OMcp24_37*RLcp24_124);
    ORcp24_324 = OMcp24_17*RLcp24_224-OMcp24_27*RLcp24_124;
    VIcp24_124 = ORcp24_12+ORcp24_124+ORcp24_13+qd[2]*C1-qd[3]*S1;
    VIcp24_224 = ORcp24_22+ORcp24_224+ORcp24_23+qd[2]*S1+qd[3]*C1;
    VIcp24_324 = ORcp24_324+qd[4];
    ACcp24_124 = OMcp24_27*ORcp24_324-OMcp24_37*ORcp24_224+OPcp24_27*RLcp24_324-OPcp24_37*RLcp24_224-ORcp24_23*qd[1]+
 qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp24_22+(2.0)*qd[2]*S1);
    ACcp24_224 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp24_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp24_13-(2.0)*qd[3]*S1)-OMcp24_17*ORcp24_324+
 OMcp24_37*ORcp24_124-OPcp24_17*RLcp24_324+OPcp24_37*RLcp24_124;
    ACcp24_324 = qdd[4]+OMcp24_17*ORcp24_224-OMcp24_27*ORcp24_124+OPcp24_17*RLcp24_224-OPcp24_27*RLcp24_124;
    OMcp24_125 = OMcp24_17+ROcp24_724*qd[25];
    OMcp24_225 = OMcp24_27+ROcp24_824*qd[25];
    OMcp24_325 = OMcp24_37+ROcp24_924*qd[25];
    OPcp24_125 = OPcp24_17+ROcp24_724*qdd[25]+qd[25]*(OMcp24_27*ROcp24_924-OMcp24_37*ROcp24_824);
    OPcp24_225 = OPcp24_27+ROcp24_824*qdd[25]-qd[25]*(OMcp24_17*ROcp24_924-OMcp24_37*ROcp24_724);
    OPcp24_325 = OPcp24_37+ROcp24_924*qdd[25]+qd[25]*(OMcp24_17*ROcp24_824-OMcp24_27*ROcp24_724);

// = = Block_1_0_0_25_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp24_124;
    sens->P[2] = POcp24_224;
    sens->P[3] = POcp24_324;
    sens->R[1][1] = ROcp24_125;
    sens->R[1][2] = ROcp24_225;
    sens->R[1][3] = ROcp24_325;
    sens->R[2][1] = ROcp24_425;
    sens->R[2][2] = ROcp24_525;
    sens->R[2][3] = ROcp24_625;
    sens->R[3][1] = ROcp24_724;
    sens->R[3][2] = ROcp24_824;
    sens->R[3][3] = ROcp24_924;
    sens->V[1] = VIcp24_124;
    sens->V[2] = VIcp24_224;
    sens->V[3] = VIcp24_324;
    sens->OM[1] = OMcp24_125;
    sens->OM[2] = OMcp24_225;
    sens->OM[3] = OMcp24_325;
    sens->A[1] = ACcp24_124;
    sens->A[2] = ACcp24_224;
    sens->A[3] = ACcp24_324;
    sens->OMP[1] = OPcp24_125;
    sens->OMP[2] = OPcp24_225;
    sens->OMP[3] = OPcp24_325;
 
// 
break;
case 26:
 


// = = Block_1_0_0_26_0_1 = = 
 
// Sensor Kinematics 


    ROcp25_46 = -S1p5*C6;
    ROcp25_56 = C1p5*C6;
    ROcp25_76 = S1p5*S6;
    ROcp25_86 = -C1p5*S6;
    ROcp25_17 = -(ROcp25_76*S7-C1p5*C7);
    ROcp25_27 = -(ROcp25_86*S7-S1p5*C7);
    ROcp25_77 = ROcp25_76*C7+C1p5*S7;
    ROcp25_87 = ROcp25_86*C7+S1p5*S7;
    RLcp25_12 = q[2]*C1;
    RLcp25_22 = q[2]*S1;
    ORcp25_12 = -RLcp25_22*qd[1];
    ORcp25_22 = RLcp25_12*qd[1];
    RLcp25_13 = -q[3]*S1;
    RLcp25_23 = q[3]*C1;
    ORcp25_13 = -RLcp25_23*qd[1];
    ORcp25_23 = RLcp25_13*qd[1];
    OMcp25_35 = qd[1]+qd[5];
    OMcp25_16 = qd[6]*C1p5;
    OMcp25_26 = qd[6]*S1p5;
    OMcp25_17 = OMcp25_16+ROcp25_46*qd[7];
    OMcp25_27 = OMcp25_26+ROcp25_56*qd[7];
    OMcp25_37 = OMcp25_35+qd[7]*S6;
    OPcp25_17 = -(OMcp25_35*qd[6]*S1p5-ROcp25_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp25_26*S6-OMcp25_35*ROcp25_56));
    OPcp25_27 = OMcp25_35*qd[6]*C1p5+ROcp25_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp25_16*S6-OMcp25_35*ROcp25_46);
    OPcp25_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_26_0_8 = = 
 
// Sensor Kinematics 


    ROcp25_126 = ROcp25_17*C26-ROcp25_77*S26;
    ROcp25_226 = ROcp25_27*C26-ROcp25_87*S26;
    ROcp25_326 = -S26p7*C6;
    ROcp25_726 = ROcp25_17*S26+ROcp25_77*C26;
    ROcp25_826 = ROcp25_27*S26+ROcp25_87*C26;
    ROcp25_926 = C26p7*C6;
    RLcp25_126 = ROcp25_17*s->dpt[1][7]+ROcp25_77*s->dpt[3][7];
    RLcp25_226 = ROcp25_27*s->dpt[1][7]+ROcp25_87*s->dpt[3][7];
    RLcp25_326 = -C6*(s->dpt[1][7]*S7-s->dpt[3][7]*C7);
    POcp25_126 = RLcp25_12+RLcp25_126+RLcp25_13;
    POcp25_226 = RLcp25_22+RLcp25_226+RLcp25_23;
    POcp25_326 = RLcp25_326+q[4];
    ORcp25_126 = OMcp25_27*RLcp25_326-OMcp25_37*RLcp25_226;
    ORcp25_226 = -(OMcp25_17*RLcp25_326-OMcp25_37*RLcp25_126);
    ORcp25_326 = OMcp25_17*RLcp25_226-OMcp25_27*RLcp25_126;
    VIcp25_126 = ORcp25_12+ORcp25_126+ORcp25_13+qd[2]*C1-qd[3]*S1;
    VIcp25_226 = ORcp25_22+ORcp25_226+ORcp25_23+qd[2]*S1+qd[3]*C1;
    VIcp25_326 = ORcp25_326+qd[4];
    ACcp25_126 = OMcp25_27*ORcp25_326-OMcp25_37*ORcp25_226+OPcp25_27*RLcp25_326-OPcp25_37*RLcp25_226-ORcp25_23*qd[1]+
 qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp25_22+(2.0)*qd[2]*S1);
    ACcp25_226 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp25_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp25_13-(2.0)*qd[3]*S1)-OMcp25_17*ORcp25_326+
 OMcp25_37*ORcp25_126-OPcp25_17*RLcp25_326+OPcp25_37*RLcp25_126;
    ACcp25_326 = qdd[4]+OMcp25_17*ORcp25_226-OMcp25_27*ORcp25_126+OPcp25_17*RLcp25_226-OPcp25_27*RLcp25_126;

// = = Block_1_0_0_26_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp25_126;
    sens->P[2] = POcp25_226;
    sens->P[3] = POcp25_326;
    sens->R[1][1] = ROcp25_126;
    sens->R[1][2] = ROcp25_226;
    sens->R[1][3] = ROcp25_326;
    sens->R[2][1] = ROcp25_46;
    sens->R[2][2] = ROcp25_56;
    sens->R[2][3] = S6;
    sens->R[3][1] = ROcp25_726;
    sens->R[3][2] = ROcp25_826;
    sens->R[3][3] = ROcp25_926;
    sens->V[1] = VIcp25_126;
    sens->V[2] = VIcp25_226;
    sens->V[3] = VIcp25_326;
    sens->OM[1] = OMcp25_17;
    sens->OM[2] = OMcp25_27;
    sens->OM[3] = OMcp25_37;
    sens->A[1] = ACcp25_126;
    sens->A[2] = ACcp25_226;
    sens->A[3] = ACcp25_326;
    sens->OMP[1] = OPcp25_17;
    sens->OMP[2] = OPcp25_27;
    sens->OMP[3] = OPcp25_37;
 
// 
break;
case 27:
 


// = = Block_1_0_0_27_0_1 = = 
 
// Sensor Kinematics 


    ROcp26_46 = -S1p5*C6;
    ROcp26_56 = C1p5*C6;
    ROcp26_76 = S1p5*S6;
    ROcp26_86 = -C1p5*S6;
    ROcp26_17 = -(ROcp26_76*S7-C1p5*C7);
    ROcp26_27 = -(ROcp26_86*S7-S1p5*C7);
    ROcp26_77 = ROcp26_76*C7+C1p5*S7;
    ROcp26_87 = ROcp26_86*C7+S1p5*S7;
    RLcp26_12 = q[2]*C1;
    RLcp26_22 = q[2]*S1;
    ORcp26_12 = -RLcp26_22*qd[1];
    ORcp26_22 = RLcp26_12*qd[1];
    RLcp26_13 = -q[3]*S1;
    RLcp26_23 = q[3]*C1;
    ORcp26_13 = -RLcp26_23*qd[1];
    ORcp26_23 = RLcp26_13*qd[1];
    OMcp26_35 = qd[1]+qd[5];
    OMcp26_16 = qd[6]*C1p5;
    OMcp26_26 = qd[6]*S1p5;
    OMcp26_17 = OMcp26_16+ROcp26_46*qd[7];
    OMcp26_27 = OMcp26_26+ROcp26_56*qd[7];
    OMcp26_37 = OMcp26_35+qd[7]*S6;
    OPcp26_17 = -(OMcp26_35*qd[6]*S1p5-ROcp26_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp26_26*S6-OMcp26_35*ROcp26_56));
    OPcp26_27 = OMcp26_35*qd[6]*C1p5+ROcp26_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp26_16*S6-OMcp26_35*ROcp26_46);
    OPcp26_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_27_0_8 = = 
 
// Sensor Kinematics 


    ROcp26_126 = ROcp26_17*C26-ROcp26_77*S26;
    ROcp26_226 = ROcp26_27*C26-ROcp26_87*S26;
    ROcp26_326 = -S26p7*C6;
    ROcp26_726 = ROcp26_17*S26+ROcp26_77*C26;
    ROcp26_826 = ROcp26_27*S26+ROcp26_87*C26;
    ROcp26_926 = C26p7*C6;
    RLcp26_126 = ROcp26_17*s->dpt[1][7]+ROcp26_77*s->dpt[3][7];
    RLcp26_226 = ROcp26_27*s->dpt[1][7]+ROcp26_87*s->dpt[3][7];
    RLcp26_326 = -C6*(s->dpt[1][7]*S7-s->dpt[3][7]*C7);
    POcp26_126 = RLcp26_12+RLcp26_126+RLcp26_13;
    POcp26_226 = RLcp26_22+RLcp26_226+RLcp26_23;
    POcp26_326 = RLcp26_326+q[4];
    ORcp26_126 = OMcp26_27*RLcp26_326-OMcp26_37*RLcp26_226;
    ORcp26_226 = -(OMcp26_17*RLcp26_326-OMcp26_37*RLcp26_126);
    ORcp26_326 = OMcp26_17*RLcp26_226-OMcp26_27*RLcp26_126;
    VIcp26_126 = ORcp26_12+ORcp26_126+ORcp26_13+qd[2]*C1-qd[3]*S1;
    VIcp26_226 = ORcp26_22+ORcp26_226+ORcp26_23+qd[2]*S1+qd[3]*C1;
    VIcp26_326 = ORcp26_326+qd[4];
    ACcp26_126 = OMcp26_27*ORcp26_326-OMcp26_37*ORcp26_226+OPcp26_27*RLcp26_326-OPcp26_37*RLcp26_226-ORcp26_23*qd[1]+
 qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp26_22+(2.0)*qd[2]*S1);
    ACcp26_226 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp26_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp26_13-(2.0)*qd[3]*S1)-OMcp26_17*ORcp26_326+
 OMcp26_37*ORcp26_126-OPcp26_17*RLcp26_326+OPcp26_37*RLcp26_126;
    ACcp26_326 = qdd[4]+OMcp26_17*ORcp26_226-OMcp26_27*ORcp26_126+OPcp26_17*RLcp26_226-OPcp26_27*RLcp26_126;

// = = Block_1_0_0_27_0_9 = = 
 
// Sensor Kinematics 


    ROcp26_127 = ROcp26_126*C27+ROcp26_46*S27;
    ROcp26_227 = ROcp26_226*C27+ROcp26_56*S27;
    ROcp26_327 = ROcp26_326*C27+S27*S6;
    ROcp26_427 = -(ROcp26_126*S27-ROcp26_46*C27);
    ROcp26_527 = -(ROcp26_226*S27-ROcp26_56*C27);
    ROcp26_627 = -(ROcp26_326*S27-C27*S6);
    OMcp26_127 = OMcp26_17+ROcp26_726*qd[27];
    OMcp26_227 = OMcp26_27+ROcp26_826*qd[27];
    OMcp26_327 = OMcp26_37+ROcp26_926*qd[27];
    OPcp26_127 = OPcp26_17+ROcp26_726*qdd[27]+qd[27]*(OMcp26_27*ROcp26_926-OMcp26_37*ROcp26_826);
    OPcp26_227 = OPcp26_27+ROcp26_826*qdd[27]-qd[27]*(OMcp26_17*ROcp26_926-OMcp26_37*ROcp26_726);
    OPcp26_327 = OPcp26_37+ROcp26_926*qdd[27]+qd[27]*(OMcp26_17*ROcp26_826-OMcp26_27*ROcp26_726);

// = = Block_1_0_0_27_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp26_126;
    sens->P[2] = POcp26_226;
    sens->P[3] = POcp26_326;
    sens->R[1][1] = ROcp26_127;
    sens->R[1][2] = ROcp26_227;
    sens->R[1][3] = ROcp26_327;
    sens->R[2][1] = ROcp26_427;
    sens->R[2][2] = ROcp26_527;
    sens->R[2][3] = ROcp26_627;
    sens->R[3][1] = ROcp26_726;
    sens->R[3][2] = ROcp26_826;
    sens->R[3][3] = ROcp26_926;
    sens->V[1] = VIcp26_126;
    sens->V[2] = VIcp26_226;
    sens->V[3] = VIcp26_326;
    sens->OM[1] = OMcp26_127;
    sens->OM[2] = OMcp26_227;
    sens->OM[3] = OMcp26_327;
    sens->A[1] = ACcp26_126;
    sens->A[2] = ACcp26_226;
    sens->A[3] = ACcp26_326;
    sens->OMP[1] = OPcp26_127;
    sens->OMP[2] = OPcp26_227;
    sens->OMP[3] = OPcp26_327;
 
// 
break;
case 28:
 


// = = Block_1_0_0_28_0_1 = = 
 
// Sensor Kinematics 


    ROcp27_46 = -S1p5*C6;
    ROcp27_56 = C1p5*C6;
    ROcp27_76 = S1p5*S6;
    ROcp27_86 = -C1p5*S6;
    ROcp27_17 = -(ROcp27_76*S7-C1p5*C7);
    ROcp27_27 = -(ROcp27_86*S7-S1p5*C7);
    ROcp27_77 = ROcp27_76*C7+C1p5*S7;
    ROcp27_87 = ROcp27_86*C7+S1p5*S7;
    RLcp27_12 = q[2]*C1;
    RLcp27_22 = q[2]*S1;
    ORcp27_12 = -RLcp27_22*qd[1];
    ORcp27_22 = RLcp27_12*qd[1];
    RLcp27_13 = -q[3]*S1;
    RLcp27_23 = q[3]*C1;
    ORcp27_13 = -RLcp27_23*qd[1];
    ORcp27_23 = RLcp27_13*qd[1];
    OMcp27_35 = qd[1]+qd[5];
    OMcp27_16 = qd[6]*C1p5;
    OMcp27_26 = qd[6]*S1p5;
    OMcp27_17 = OMcp27_16+ROcp27_46*qd[7];
    OMcp27_27 = OMcp27_26+ROcp27_56*qd[7];
    OMcp27_37 = OMcp27_35+qd[7]*S6;
    OPcp27_17 = -(OMcp27_35*qd[6]*S1p5-ROcp27_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp27_26*S6-OMcp27_35*ROcp27_56));
    OPcp27_27 = OMcp27_35*qd[6]*C1p5+ROcp27_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp27_16*S6-OMcp27_35*ROcp27_46);
    OPcp27_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_28_0_8 = = 
 
// Sensor Kinematics 


    ROcp27_126 = ROcp27_17*C26-ROcp27_77*S26;
    ROcp27_226 = ROcp27_27*C26-ROcp27_87*S26;
    ROcp27_326 = -S26p7*C6;
    ROcp27_726 = ROcp27_17*S26+ROcp27_77*C26;
    ROcp27_826 = ROcp27_27*S26+ROcp27_87*C26;
    ROcp27_926 = C26p7*C6;
    RLcp27_126 = ROcp27_17*s->dpt[1][7]+ROcp27_77*s->dpt[3][7];
    RLcp27_226 = ROcp27_27*s->dpt[1][7]+ROcp27_87*s->dpt[3][7];
    RLcp27_326 = -C6*(s->dpt[1][7]*S7-s->dpt[3][7]*C7);
    POcp27_126 = RLcp27_12+RLcp27_126+RLcp27_13;
    POcp27_226 = RLcp27_22+RLcp27_226+RLcp27_23;
    POcp27_326 = RLcp27_326+q[4];
    ORcp27_126 = OMcp27_27*RLcp27_326-OMcp27_37*RLcp27_226;
    ORcp27_226 = -(OMcp27_17*RLcp27_326-OMcp27_37*RLcp27_126);
    ORcp27_326 = OMcp27_17*RLcp27_226-OMcp27_27*RLcp27_126;
    VIcp27_126 = ORcp27_12+ORcp27_126+ORcp27_13+qd[2]*C1-qd[3]*S1;
    VIcp27_226 = ORcp27_22+ORcp27_226+ORcp27_23+qd[2]*S1+qd[3]*C1;
    VIcp27_326 = ORcp27_326+qd[4];
    ACcp27_126 = OMcp27_27*ORcp27_326-OMcp27_37*ORcp27_226+OPcp27_27*RLcp27_326-OPcp27_37*RLcp27_226-ORcp27_23*qd[1]+
 qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp27_22+(2.0)*qd[2]*S1);
    ACcp27_226 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp27_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp27_13-(2.0)*qd[3]*S1)-OMcp27_17*ORcp27_326+
 OMcp27_37*ORcp27_126-OPcp27_17*RLcp27_326+OPcp27_37*RLcp27_126;
    ACcp27_326 = qdd[4]+OMcp27_17*ORcp27_226-OMcp27_27*ORcp27_126+OPcp27_17*RLcp27_226-OPcp27_27*RLcp27_126;

// = = Block_1_0_0_28_0_9 = = 
 
// Sensor Kinematics 


    ROcp27_127 = ROcp27_126*C27+ROcp27_46*S27;
    ROcp27_227 = ROcp27_226*C27+ROcp27_56*S27;
    ROcp27_327 = ROcp27_326*C27+S27*S6;
    ROcp27_427 = -(ROcp27_126*S27-ROcp27_46*C27);
    ROcp27_527 = -(ROcp27_226*S27-ROcp27_56*C27);
    ROcp27_627 = -(ROcp27_326*S27-C27*S6);
    ROcp27_428 = ROcp27_427*C28+ROcp27_726*S28;
    ROcp27_528 = ROcp27_527*C28+ROcp27_826*S28;
    ROcp27_628 = ROcp27_627*C28+ROcp27_926*S28;
    ROcp27_728 = -(ROcp27_427*S28-ROcp27_726*C28);
    ROcp27_828 = -(ROcp27_527*S28-ROcp27_826*C28);
    ROcp27_928 = -(ROcp27_627*S28-ROcp27_926*C28);
    OMcp27_127 = OMcp27_17+ROcp27_726*qd[27];
    OMcp27_227 = OMcp27_27+ROcp27_826*qd[27];
    OMcp27_327 = OMcp27_37+ROcp27_926*qd[27];
    OMcp27_128 = OMcp27_127+ROcp27_127*qd[28];
    OMcp27_228 = OMcp27_227+ROcp27_227*qd[28];
    OMcp27_328 = OMcp27_327+ROcp27_327*qd[28];
    OPcp27_128 = OPcp27_17+ROcp27_127*qdd[28]+ROcp27_726*qdd[27]+qd[27]*(OMcp27_27*ROcp27_926-OMcp27_37*ROcp27_826)+qd[28]
 *(OMcp27_227*ROcp27_327-OMcp27_327*ROcp27_227);
    OPcp27_228 = OPcp27_27+ROcp27_227*qdd[28]+ROcp27_826*qdd[27]-qd[27]*(OMcp27_17*ROcp27_926-OMcp27_37*ROcp27_726)-qd[28]
 *(OMcp27_127*ROcp27_327-OMcp27_327*ROcp27_127);
    OPcp27_328 = OPcp27_37+ROcp27_327*qdd[28]+ROcp27_926*qdd[27]+qd[27]*(OMcp27_17*ROcp27_826-OMcp27_27*ROcp27_726)+qd[28]
 *(OMcp27_127*ROcp27_227-OMcp27_227*ROcp27_127);

// = = Block_1_0_0_28_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp27_126;
    sens->P[2] = POcp27_226;
    sens->P[3] = POcp27_326;
    sens->R[1][1] = ROcp27_127;
    sens->R[1][2] = ROcp27_227;
    sens->R[1][3] = ROcp27_327;
    sens->R[2][1] = ROcp27_428;
    sens->R[2][2] = ROcp27_528;
    sens->R[2][3] = ROcp27_628;
    sens->R[3][1] = ROcp27_728;
    sens->R[3][2] = ROcp27_828;
    sens->R[3][3] = ROcp27_928;
    sens->V[1] = VIcp27_126;
    sens->V[2] = VIcp27_226;
    sens->V[3] = VIcp27_326;
    sens->OM[1] = OMcp27_128;
    sens->OM[2] = OMcp27_228;
    sens->OM[3] = OMcp27_328;
    sens->A[1] = ACcp27_126;
    sens->A[2] = ACcp27_226;
    sens->A[3] = ACcp27_326;
    sens->OMP[1] = OPcp27_128;
    sens->OMP[2] = OPcp27_228;
    sens->OMP[3] = OPcp27_328;
 
// 
break;
case 29:
 


// = = Block_1_0_0_29_0_1 = = 
 
// Sensor Kinematics 


    ROcp28_46 = -S1p5*C6;
    ROcp28_56 = C1p5*C6;
    ROcp28_76 = S1p5*S6;
    ROcp28_86 = -C1p5*S6;
    ROcp28_17 = -(ROcp28_76*S7-C1p5*C7);
    ROcp28_27 = -(ROcp28_86*S7-S1p5*C7);
    ROcp28_77 = ROcp28_76*C7+C1p5*S7;
    ROcp28_87 = ROcp28_86*C7+S1p5*S7;
    RLcp28_12 = q[2]*C1;
    RLcp28_22 = q[2]*S1;
    ORcp28_12 = -RLcp28_22*qd[1];
    ORcp28_22 = RLcp28_12*qd[1];
    RLcp28_13 = -q[3]*S1;
    RLcp28_23 = q[3]*C1;
    ORcp28_13 = -RLcp28_23*qd[1];
    ORcp28_23 = RLcp28_13*qd[1];
    OMcp28_35 = qd[1]+qd[5];
    OMcp28_16 = qd[6]*C1p5;
    OMcp28_26 = qd[6]*S1p5;
    OMcp28_17 = OMcp28_16+ROcp28_46*qd[7];
    OMcp28_27 = OMcp28_26+ROcp28_56*qd[7];
    OMcp28_37 = OMcp28_35+qd[7]*S6;
    OPcp28_17 = -(OMcp28_35*qd[6]*S1p5-ROcp28_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp28_26*S6-OMcp28_35*ROcp28_56));
    OPcp28_27 = OMcp28_35*qd[6]*C1p5+ROcp28_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp28_16*S6-OMcp28_35*ROcp28_46);
    OPcp28_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_29_0_8 = = 
 
// Sensor Kinematics 


    ROcp28_126 = ROcp28_17*C26-ROcp28_77*S26;
    ROcp28_226 = ROcp28_27*C26-ROcp28_87*S26;
    ROcp28_326 = -S26p7*C6;
    ROcp28_726 = ROcp28_17*S26+ROcp28_77*C26;
    ROcp28_826 = ROcp28_27*S26+ROcp28_87*C26;
    ROcp28_926 = C26p7*C6;
    RLcp28_126 = ROcp28_17*s->dpt[1][7]+ROcp28_77*s->dpt[3][7];
    RLcp28_226 = ROcp28_27*s->dpt[1][7]+ROcp28_87*s->dpt[3][7];
    RLcp28_326 = -C6*(s->dpt[1][7]*S7-s->dpt[3][7]*C7);
    POcp28_126 = RLcp28_12+RLcp28_126+RLcp28_13;
    POcp28_226 = RLcp28_22+RLcp28_226+RLcp28_23;
    POcp28_326 = RLcp28_326+q[4];
    ORcp28_126 = OMcp28_27*RLcp28_326-OMcp28_37*RLcp28_226;
    ORcp28_226 = -(OMcp28_17*RLcp28_326-OMcp28_37*RLcp28_126);
    ORcp28_326 = OMcp28_17*RLcp28_226-OMcp28_27*RLcp28_126;
    VIcp28_126 = ORcp28_12+ORcp28_126+ORcp28_13+qd[2]*C1-qd[3]*S1;
    VIcp28_226 = ORcp28_22+ORcp28_226+ORcp28_23+qd[2]*S1+qd[3]*C1;
    VIcp28_326 = ORcp28_326+qd[4];
    ACcp28_126 = OMcp28_27*ORcp28_326-OMcp28_37*ORcp28_226+OPcp28_27*RLcp28_326-OPcp28_37*RLcp28_226-ORcp28_23*qd[1]+
 qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp28_22+(2.0)*qd[2]*S1);
    ACcp28_226 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp28_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp28_13-(2.0)*qd[3]*S1)-OMcp28_17*ORcp28_326+
 OMcp28_37*ORcp28_126-OPcp28_17*RLcp28_326+OPcp28_37*RLcp28_126;
    ACcp28_326 = qdd[4]+OMcp28_17*ORcp28_226-OMcp28_27*ORcp28_126+OPcp28_17*RLcp28_226-OPcp28_27*RLcp28_126;

// = = Block_1_0_0_29_0_10 = = 
 
// Sensor Kinematics 


    ROcp28_129 = ROcp28_126*C29+ROcp28_46*S29;
    ROcp28_229 = ROcp28_226*C29+ROcp28_56*S29;
    ROcp28_329 = ROcp28_326*C29+S29*S6;
    ROcp28_429 = -(ROcp28_126*S29-ROcp28_46*C29);
    ROcp28_529 = -(ROcp28_226*S29-ROcp28_56*C29);
    ROcp28_629 = -(ROcp28_326*S29-C29*S6);
    OMcp28_129 = OMcp28_17+ROcp28_726*qd[29];
    OMcp28_229 = OMcp28_27+ROcp28_826*qd[29];
    OMcp28_329 = OMcp28_37+ROcp28_926*qd[29];
    OPcp28_129 = OPcp28_17+ROcp28_726*qdd[29]+qd[29]*(OMcp28_27*ROcp28_926-OMcp28_37*ROcp28_826);
    OPcp28_229 = OPcp28_27+ROcp28_826*qdd[29]-qd[29]*(OMcp28_17*ROcp28_926-OMcp28_37*ROcp28_726);
    OPcp28_329 = OPcp28_37+ROcp28_926*qdd[29]+qd[29]*(OMcp28_17*ROcp28_826-OMcp28_27*ROcp28_726);

// = = Block_1_0_0_29_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp28_126;
    sens->P[2] = POcp28_226;
    sens->P[3] = POcp28_326;
    sens->R[1][1] = ROcp28_129;
    sens->R[1][2] = ROcp28_229;
    sens->R[1][3] = ROcp28_329;
    sens->R[2][1] = ROcp28_429;
    sens->R[2][2] = ROcp28_529;
    sens->R[2][3] = ROcp28_629;
    sens->R[3][1] = ROcp28_726;
    sens->R[3][2] = ROcp28_826;
    sens->R[3][3] = ROcp28_926;
    sens->V[1] = VIcp28_126;
    sens->V[2] = VIcp28_226;
    sens->V[3] = VIcp28_326;
    sens->OM[1] = OMcp28_129;
    sens->OM[2] = OMcp28_229;
    sens->OM[3] = OMcp28_329;
    sens->A[1] = ACcp28_126;
    sens->A[2] = ACcp28_226;
    sens->A[3] = ACcp28_326;
    sens->OMP[1] = OPcp28_129;
    sens->OMP[2] = OPcp28_229;
    sens->OMP[3] = OPcp28_329;
 
// 
break;
case 30:
 


// = = Block_1_0_0_30_0_1 = = 
 
// Sensor Kinematics 


    ROcp29_46 = -S1p5*C6;
    ROcp29_56 = C1p5*C6;
    ROcp29_76 = S1p5*S6;
    ROcp29_86 = -C1p5*S6;
    ROcp29_17 = -(ROcp29_76*S7-C1p5*C7);
    ROcp29_27 = -(ROcp29_86*S7-S1p5*C7);
    ROcp29_77 = ROcp29_76*C7+C1p5*S7;
    ROcp29_87 = ROcp29_86*C7+S1p5*S7;
    RLcp29_12 = q[2]*C1;
    RLcp29_22 = q[2]*S1;
    ORcp29_12 = -RLcp29_22*qd[1];
    ORcp29_22 = RLcp29_12*qd[1];
    RLcp29_13 = -q[3]*S1;
    RLcp29_23 = q[3]*C1;
    ORcp29_13 = -RLcp29_23*qd[1];
    ORcp29_23 = RLcp29_13*qd[1];
    OMcp29_35 = qd[1]+qd[5];
    OMcp29_16 = qd[6]*C1p5;
    OMcp29_26 = qd[6]*S1p5;
    OMcp29_17 = OMcp29_16+ROcp29_46*qd[7];
    OMcp29_27 = OMcp29_26+ROcp29_56*qd[7];
    OMcp29_37 = OMcp29_35+qd[7]*S6;
    OPcp29_17 = -(OMcp29_35*qd[6]*S1p5-ROcp29_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp29_26*S6-OMcp29_35*ROcp29_56));
    OPcp29_27 = OMcp29_35*qd[6]*C1p5+ROcp29_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp29_16*S6-OMcp29_35*ROcp29_46);
    OPcp29_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_30_0_8 = = 
 
// Sensor Kinematics 


    ROcp29_126 = ROcp29_17*C26-ROcp29_77*S26;
    ROcp29_226 = ROcp29_27*C26-ROcp29_87*S26;
    ROcp29_326 = -S26p7*C6;
    ROcp29_726 = ROcp29_17*S26+ROcp29_77*C26;
    ROcp29_826 = ROcp29_27*S26+ROcp29_87*C26;
    ROcp29_926 = C26p7*C6;
    RLcp29_126 = ROcp29_17*s->dpt[1][7]+ROcp29_77*s->dpt[3][7];
    RLcp29_226 = ROcp29_27*s->dpt[1][7]+ROcp29_87*s->dpt[3][7];
    RLcp29_326 = -C6*(s->dpt[1][7]*S7-s->dpt[3][7]*C7);
    POcp29_126 = RLcp29_12+RLcp29_126+RLcp29_13;
    POcp29_226 = RLcp29_22+RLcp29_226+RLcp29_23;
    POcp29_326 = RLcp29_326+q[4];
    ORcp29_126 = OMcp29_27*RLcp29_326-OMcp29_37*RLcp29_226;
    ORcp29_226 = -(OMcp29_17*RLcp29_326-OMcp29_37*RLcp29_126);
    ORcp29_326 = OMcp29_17*RLcp29_226-OMcp29_27*RLcp29_126;
    VIcp29_126 = ORcp29_12+ORcp29_126+ORcp29_13+qd[2]*C1-qd[3]*S1;
    VIcp29_226 = ORcp29_22+ORcp29_226+ORcp29_23+qd[2]*S1+qd[3]*C1;
    VIcp29_326 = ORcp29_326+qd[4];
    ACcp29_126 = OMcp29_27*ORcp29_326-OMcp29_37*ORcp29_226+OPcp29_27*RLcp29_326-OPcp29_37*RLcp29_226-ORcp29_23*qd[1]+
 qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp29_22+(2.0)*qd[2]*S1);
    ACcp29_226 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp29_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp29_13-(2.0)*qd[3]*S1)-OMcp29_17*ORcp29_326+
 OMcp29_37*ORcp29_126-OPcp29_17*RLcp29_326+OPcp29_37*RLcp29_126;
    ACcp29_326 = qdd[4]+OMcp29_17*ORcp29_226-OMcp29_27*ORcp29_126+OPcp29_17*RLcp29_226-OPcp29_27*RLcp29_126;

// = = Block_1_0_0_30_0_10 = = 
 
// Sensor Kinematics 


    ROcp29_129 = ROcp29_126*C29+ROcp29_46*S29;
    ROcp29_229 = ROcp29_226*C29+ROcp29_56*S29;
    ROcp29_329 = ROcp29_326*C29+S29*S6;
    ROcp29_429 = -(ROcp29_126*S29-ROcp29_46*C29);
    ROcp29_529 = -(ROcp29_226*S29-ROcp29_56*C29);
    ROcp29_629 = -(ROcp29_326*S29-C29*S6);
    ROcp29_430 = ROcp29_429*C30+ROcp29_726*S30;
    ROcp29_530 = ROcp29_529*C30+ROcp29_826*S30;
    ROcp29_630 = ROcp29_629*C30+ROcp29_926*S30;
    ROcp29_730 = -(ROcp29_429*S30-ROcp29_726*C30);
    ROcp29_830 = -(ROcp29_529*S30-ROcp29_826*C30);
    ROcp29_930 = -(ROcp29_629*S30-ROcp29_926*C30);
    OMcp29_129 = OMcp29_17+ROcp29_726*qd[29];
    OMcp29_229 = OMcp29_27+ROcp29_826*qd[29];
    OMcp29_329 = OMcp29_37+ROcp29_926*qd[29];
    OMcp29_130 = OMcp29_129+ROcp29_129*qd[30];
    OMcp29_230 = OMcp29_229+ROcp29_229*qd[30];
    OMcp29_330 = OMcp29_329+ROcp29_329*qd[30];
    OPcp29_130 = OPcp29_17+ROcp29_129*qdd[30]+ROcp29_726*qdd[29]+qd[29]*(OMcp29_27*ROcp29_926-OMcp29_37*ROcp29_826)+qd[30]
 *(OMcp29_229*ROcp29_329-OMcp29_329*ROcp29_229);
    OPcp29_230 = OPcp29_27+ROcp29_229*qdd[30]+ROcp29_826*qdd[29]-qd[29]*(OMcp29_17*ROcp29_926-OMcp29_37*ROcp29_726)-qd[30]
 *(OMcp29_129*ROcp29_329-OMcp29_329*ROcp29_129);
    OPcp29_330 = OPcp29_37+ROcp29_329*qdd[30]+ROcp29_926*qdd[29]+qd[29]*(OMcp29_17*ROcp29_826-OMcp29_27*ROcp29_726)+qd[30]
 *(OMcp29_129*ROcp29_229-OMcp29_229*ROcp29_129);

// = = Block_1_0_0_30_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp29_126;
    sens->P[2] = POcp29_226;
    sens->P[3] = POcp29_326;
    sens->R[1][1] = ROcp29_129;
    sens->R[1][2] = ROcp29_229;
    sens->R[1][3] = ROcp29_329;
    sens->R[2][1] = ROcp29_430;
    sens->R[2][2] = ROcp29_530;
    sens->R[2][3] = ROcp29_630;
    sens->R[3][1] = ROcp29_730;
    sens->R[3][2] = ROcp29_830;
    sens->R[3][3] = ROcp29_930;
    sens->V[1] = VIcp29_126;
    sens->V[2] = VIcp29_226;
    sens->V[3] = VIcp29_326;
    sens->OM[1] = OMcp29_130;
    sens->OM[2] = OMcp29_230;
    sens->OM[3] = OMcp29_330;
    sens->A[1] = ACcp29_126;
    sens->A[2] = ACcp29_226;
    sens->A[3] = ACcp29_326;
    sens->OMP[1] = OPcp29_130;
    sens->OMP[2] = OPcp29_230;
    sens->OMP[3] = OPcp29_330;

break;
default:
break;
}


// ====== END Task 1 ====== 


}
 


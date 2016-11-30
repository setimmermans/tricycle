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
//	==> Generation Date : Mon Nov 28 17:34:57 2016
//
//	==> Project name : tricycle
//	==> using XML input file 
//
//	==> Number of joints : 31
//
//	==> Function : F 6 : Sensors Kinematical Informations (sens) 
//	==> Flops complexity : 8414
//
//	==> Generation Time :  0.120 seconds
//	==> Post-Processing :  0.190 seconds
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
  C10 = cos(q[10]);
  S10 = sin(q[10]);

// = = Block_0_0_0_0_0_3 = = 
 
// Trigonometric Variables  

  C11 = cos(q[11]);
  S11 = sin(q[11]);

// = = Block_0_0_0_0_0_4 = = 
 
// Trigonometric Variables  

  C12 = cos(q[12]);
  S12 = sin(q[12]);

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

// = = Block_0_0_0_0_0_7 = = 
 
// Trigonometric Variables  

  C25 = cos(q[25]);
  S25 = sin(q[25]);
  C26 = cos(q[26]);
  S26 = sin(q[26]);

// = = Block_0_0_0_0_0_8 = = 
 
// Trigonometric Variables  

  C27 = cos(q[27]);
  S27 = sin(q[27]);

// = = Block_0_0_0_0_0_9 = = 
 
// Trigonometric Variables  

  C28 = cos(q[28]);
  S28 = sin(q[28]);
  C29 = cos(q[29]);
  S29 = sin(q[29]);

// = = Block_0_0_0_0_0_10 = = 
 
// Trigonometric Variables  

  C30 = cos(q[30]);
  S30 = sin(q[30]);
  C31 = cos(q[31]);
  S31 = sin(q[31]);

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

// = = Block_0_0_0_11_0_3 = = 
 
// Trigonometric Variables  

  S11p7 = C11*S7+S11*C7;
  C11p7 = C11*C7-S11*S7;

// = = Block_0_0_0_25_0_7 = = 
 
// Trigonometric Variables  

  S25p7 = C25*S7+S25*C7;
  C25p7 = C25*C7-S25*S7;

// = = Block_0_0_0_27_0_8 = = 
 
// Trigonometric Variables  

  S27p7 = C27*S7+S27*C7;
  C27p7 = C27*C7-S27*S7;

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

// = = Block_1_0_0_10_0_2 = = 
 
// Sensor Kinematics 


    ROcp9_18 = ROcp9_17*C8-ROcp9_77*S8;
    ROcp9_28 = ROcp9_27*C8-ROcp9_87*S8;
    ROcp9_78 = ROcp9_17*S8+ROcp9_77*C8;
    ROcp9_88 = ROcp9_27*S8+ROcp9_87*C8;
    ROcp9_19 = ROcp9_18*C9-ROcp9_78*S9;
    ROcp9_29 = ROcp9_28*C9-ROcp9_88*S9;
    ROcp9_39 = -S7p8p9*C6;
    ROcp9_79 = ROcp9_18*S9+ROcp9_78*C9;
    ROcp9_89 = ROcp9_28*S9+ROcp9_88*C9;
    ROcp9_99 = C7p8p9*C6;
    ROcp9_110 = ROcp9_19*C10+ROcp9_46*S10;
    ROcp9_210 = ROcp9_29*C10+ROcp9_56*S10;
    ROcp9_310 = ROcp9_39*C10+S10*S6;
    ROcp9_410 = -(ROcp9_19*S10-ROcp9_46*C10);
    ROcp9_510 = -(ROcp9_29*S10-ROcp9_56*C10);
    ROcp9_610 = -(ROcp9_39*S10-C10*S6);
    RLcp9_18 = ROcp9_17*s->dpt[1][2]+ROcp9_77*s->dpt[3][2];
    RLcp9_28 = ROcp9_27*s->dpt[1][2]+ROcp9_87*s->dpt[3][2];
    RLcp9_38 = -C6*(s->dpt[1][2]*S7-s->dpt[3][2]*C7);
    OMcp9_18 = OMcp9_17+ROcp9_46*qd[8];
    OMcp9_28 = OMcp9_27+ROcp9_56*qd[8];
    OMcp9_38 = OMcp9_37+qd[8]*S6;
    ORcp9_18 = OMcp9_27*RLcp9_38-OMcp9_37*RLcp9_28;
    ORcp9_28 = -(OMcp9_17*RLcp9_38-OMcp9_37*RLcp9_18);
    ORcp9_38 = OMcp9_17*RLcp9_28-OMcp9_27*RLcp9_18;
    OPcp9_18 = OPcp9_17+ROcp9_46*qdd[8]+qd[8]*(OMcp9_27*S6-OMcp9_37*ROcp9_56);
    OPcp9_28 = OPcp9_27+ROcp9_56*qdd[8]-qd[8]*(OMcp9_17*S6-OMcp9_37*ROcp9_46);
    OPcp9_38 = OPcp9_37+qdd[8]*S6+qd[8]*(OMcp9_17*ROcp9_56-OMcp9_27*ROcp9_46);
    RLcp9_19 = ROcp9_18*s->dpt[1][8];
    RLcp9_29 = ROcp9_28*s->dpt[1][8];
    RLcp9_39 = -s->dpt[1][8]*S7p8*C6;
    OMcp9_19 = OMcp9_18+ROcp9_46*qd[9];
    OMcp9_29 = OMcp9_28+ROcp9_56*qd[9];
    OMcp9_39 = OMcp9_38+qd[9]*S6;
    ORcp9_19 = OMcp9_28*RLcp9_39-OMcp9_38*RLcp9_29;
    ORcp9_29 = -(OMcp9_18*RLcp9_39-OMcp9_38*RLcp9_19);
    ORcp9_39 = OMcp9_18*RLcp9_29-OMcp9_28*RLcp9_19;
    OPcp9_19 = OPcp9_18+ROcp9_46*qdd[9]+qd[9]*(OMcp9_28*S6-OMcp9_38*ROcp9_56);
    OPcp9_29 = OPcp9_28+ROcp9_56*qdd[9]-qd[9]*(OMcp9_18*S6-OMcp9_38*ROcp9_46);
    OPcp9_39 = OPcp9_38+qdd[9]*S6+qd[9]*(OMcp9_18*ROcp9_56-OMcp9_28*ROcp9_46);
    RLcp9_110 = ROcp9_79*s->dpt[3][12];
    RLcp9_210 = ROcp9_89*s->dpt[3][12];
    RLcp9_310 = ROcp9_99*s->dpt[3][12];
    POcp9_110 = RLcp9_110+RLcp9_12+RLcp9_13+RLcp9_18+RLcp9_19;
    POcp9_210 = RLcp9_210+RLcp9_22+RLcp9_23+RLcp9_28+RLcp9_29;
    POcp9_310 = RLcp9_310+RLcp9_38+RLcp9_39+q[4];
    OMcp9_110 = OMcp9_19+ROcp9_79*qd[10];
    OMcp9_210 = OMcp9_29+ROcp9_89*qd[10];
    OMcp9_310 = OMcp9_39+ROcp9_99*qd[10];
    ORcp9_110 = OMcp9_29*RLcp9_310-OMcp9_39*RLcp9_210;
    ORcp9_210 = -(OMcp9_19*RLcp9_310-OMcp9_39*RLcp9_110);
    ORcp9_310 = OMcp9_19*RLcp9_210-OMcp9_29*RLcp9_110;
    VIcp9_110 = ORcp9_110+ORcp9_12+ORcp9_13+ORcp9_18+ORcp9_19+qd[2]*C1-qd[3]*S1;
    VIcp9_210 = ORcp9_210+ORcp9_22+ORcp9_23+ORcp9_28+ORcp9_29+qd[2]*S1+qd[3]*C1;
    VIcp9_310 = ORcp9_310+ORcp9_38+ORcp9_39+qd[4];
    OPcp9_110 = OPcp9_19+ROcp9_79*qdd[10]+qd[10]*(OMcp9_29*ROcp9_99-OMcp9_39*ROcp9_89);
    OPcp9_210 = OPcp9_29+ROcp9_89*qdd[10]-qd[10]*(OMcp9_19*ROcp9_99-OMcp9_39*ROcp9_79);
    OPcp9_310 = OPcp9_39+ROcp9_99*qdd[10]+qd[10]*(OMcp9_19*ROcp9_89-OMcp9_29*ROcp9_79);
    ACcp9_110 = OMcp9_27*ORcp9_38+OMcp9_28*ORcp9_39+OMcp9_29*ORcp9_310-OMcp9_37*ORcp9_28-OMcp9_38*ORcp9_29-OMcp9_39*
 ORcp9_210+OPcp9_27*RLcp9_38+OPcp9_28*RLcp9_39+OPcp9_29*RLcp9_310-OPcp9_37*RLcp9_28-OPcp9_38*RLcp9_29-OPcp9_39*RLcp9_210-
 ORcp9_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp9_22+(2.0)*qd[2]*S1);
    ACcp9_210 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp9_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp9_13-(2.0)*qd[3]*S1)-OMcp9_17*ORcp9_38+OMcp9_37*
 ORcp9_18-OPcp9_17*RLcp9_38+OPcp9_37*RLcp9_18-OMcp9_18*ORcp9_39+OMcp9_38*ORcp9_19-OPcp9_18*RLcp9_39+OPcp9_38*RLcp9_19-
 OMcp9_19*ORcp9_310+OMcp9_39*ORcp9_110-OPcp9_19*RLcp9_310+OPcp9_39*RLcp9_110;
    ACcp9_310 = qdd[4]+OMcp9_17*ORcp9_28+OMcp9_18*ORcp9_29+OMcp9_19*ORcp9_210-OMcp9_27*ORcp9_18-OMcp9_28*ORcp9_19-OMcp9_29
 *ORcp9_110+OPcp9_17*RLcp9_28+OPcp9_18*RLcp9_29+OPcp9_19*RLcp9_210-OPcp9_27*RLcp9_18-OPcp9_28*RLcp9_19-OPcp9_29*RLcp9_110;

// = = Block_1_0_0_10_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp9_110;
    sens->P[2] = POcp9_210;
    sens->P[3] = POcp9_310;
    sens->R[1][1] = ROcp9_110;
    sens->R[1][2] = ROcp9_210;
    sens->R[1][3] = ROcp9_310;
    sens->R[2][1] = ROcp9_410;
    sens->R[2][2] = ROcp9_510;
    sens->R[2][3] = ROcp9_610;
    sens->R[3][1] = ROcp9_79;
    sens->R[3][2] = ROcp9_89;
    sens->R[3][3] = ROcp9_99;
    sens->V[1] = VIcp9_110;
    sens->V[2] = VIcp9_210;
    sens->V[3] = VIcp9_310;
    sens->OM[1] = OMcp9_110;
    sens->OM[2] = OMcp9_210;
    sens->OM[3] = OMcp9_310;
    sens->A[1] = ACcp9_110;
    sens->A[2] = ACcp9_210;
    sens->A[3] = ACcp9_310;
    sens->OMP[1] = OPcp9_110;
    sens->OMP[2] = OPcp9_210;
    sens->OMP[3] = OPcp9_310;
 
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
    POcp10_111 = RLcp10_111+RLcp10_12+RLcp10_13;
    POcp10_211 = RLcp10_211+RLcp10_22+RLcp10_23;
    POcp10_311 = RLcp10_311+q[4];
    ORcp10_111 = OMcp10_27*RLcp10_311-OMcp10_37*RLcp10_211;
    ORcp10_211 = -(OMcp10_17*RLcp10_311-OMcp10_37*RLcp10_111);
    ORcp10_311 = OMcp10_17*RLcp10_211-OMcp10_27*RLcp10_111;
    VIcp10_111 = ORcp10_111+ORcp10_12+ORcp10_13+qd[2]*C1-qd[3]*S1;
    VIcp10_211 = ORcp10_211+ORcp10_22+ORcp10_23+qd[2]*S1+qd[3]*C1;
    VIcp10_311 = ORcp10_311+qd[4];
    ACcp10_111 = OMcp10_27*ORcp10_311-OMcp10_37*ORcp10_211+OPcp10_27*RLcp10_311-OPcp10_37*RLcp10_211-ORcp10_23*qd[1]+
 qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp10_22+(2.0)*qd[2]*S1);
    ACcp10_211 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp10_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp10_13-(2.0)*qd[3]*S1)-OMcp10_17*ORcp10_311+
 OMcp10_37*ORcp10_111-OPcp10_17*RLcp10_311+OPcp10_37*RLcp10_111;
    ACcp10_311 = qdd[4]+OMcp10_17*ORcp10_211-OMcp10_27*ORcp10_111+OPcp10_17*RLcp10_211-OPcp10_27*RLcp10_111;

// = = Block_1_0_0_11_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp10_111;
    sens->P[2] = POcp10_211;
    sens->P[3] = POcp10_311;
    sens->R[1][1] = ROcp10_111;
    sens->R[1][2] = ROcp10_211;
    sens->R[1][3] = ROcp10_311;
    sens->R[2][1] = ROcp10_46;
    sens->R[2][2] = ROcp10_56;
    sens->R[2][3] = S6;
    sens->R[3][1] = ROcp10_711;
    sens->R[3][2] = ROcp10_811;
    sens->R[3][3] = ROcp10_911;
    sens->V[1] = VIcp10_111;
    sens->V[2] = VIcp10_211;
    sens->V[3] = VIcp10_311;
    sens->OM[1] = OMcp10_17;
    sens->OM[2] = OMcp10_27;
    sens->OM[3] = OMcp10_37;
    sens->A[1] = ACcp10_111;
    sens->A[2] = ACcp10_211;
    sens->A[3] = ACcp10_311;
    sens->OMP[1] = OPcp10_17;
    sens->OMP[2] = OPcp10_27;
    sens->OMP[3] = OPcp10_37;
 
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


    ROcp11_111 = ROcp11_17*C11-ROcp11_77*S11;
    ROcp11_211 = ROcp11_27*C11-ROcp11_87*S11;
    ROcp11_311 = -S11p7*C6;
    ROcp11_711 = ROcp11_17*S11+ROcp11_77*C11;
    ROcp11_811 = ROcp11_27*S11+ROcp11_87*C11;
    ROcp11_911 = C11p7*C6;
    RLcp11_111 = ROcp11_17*s->dpt[1][5]+ROcp11_77*s->dpt[3][5];
    RLcp11_211 = ROcp11_27*s->dpt[1][5]+ROcp11_87*s->dpt[3][5];
    RLcp11_311 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    POcp11_111 = RLcp11_111+RLcp11_12+RLcp11_13;
    POcp11_211 = RLcp11_211+RLcp11_22+RLcp11_23;
    POcp11_311 = RLcp11_311+q[4];
    ORcp11_111 = OMcp11_27*RLcp11_311-OMcp11_37*RLcp11_211;
    ORcp11_211 = -(OMcp11_17*RLcp11_311-OMcp11_37*RLcp11_111);
    ORcp11_311 = OMcp11_17*RLcp11_211-OMcp11_27*RLcp11_111;
    VIcp11_111 = ORcp11_111+ORcp11_12+ORcp11_13+qd[2]*C1-qd[3]*S1;
    VIcp11_211 = ORcp11_211+ORcp11_22+ORcp11_23+qd[2]*S1+qd[3]*C1;
    VIcp11_311 = ORcp11_311+qd[4];
    ACcp11_111 = OMcp11_27*ORcp11_311-OMcp11_37*ORcp11_211+OPcp11_27*RLcp11_311-OPcp11_37*RLcp11_211-ORcp11_23*qd[1]+
 qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp11_22+(2.0)*qd[2]*S1);
    ACcp11_211 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp11_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp11_13-(2.0)*qd[3]*S1)-OMcp11_17*ORcp11_311+
 OMcp11_37*ORcp11_111-OPcp11_17*RLcp11_311+OPcp11_37*RLcp11_111;
    ACcp11_311 = qdd[4]+OMcp11_17*ORcp11_211-OMcp11_27*ORcp11_111+OPcp11_17*RLcp11_211-OPcp11_27*RLcp11_111;

// = = Block_1_0_0_12_0_4 = = 
 
// Sensor Kinematics 


    ROcp11_412 = ROcp11_46*C12+ROcp11_711*S12;
    ROcp11_512 = ROcp11_56*C12+ROcp11_811*S12;
    ROcp11_612 = ROcp11_911*S12+C12*S6;
    ROcp11_712 = -(ROcp11_46*S12-ROcp11_711*C12);
    ROcp11_812 = -(ROcp11_56*S12-ROcp11_811*C12);
    ROcp11_912 = ROcp11_911*C12-S12*S6;
    OMcp11_112 = OMcp11_17+ROcp11_111*qd[12];
    OMcp11_212 = OMcp11_27+ROcp11_211*qd[12];
    OMcp11_312 = OMcp11_37+ROcp11_311*qd[12];
    OPcp11_112 = OPcp11_17+ROcp11_111*qdd[12]+qd[12]*(OMcp11_27*ROcp11_311-OMcp11_37*ROcp11_211);
    OPcp11_212 = OPcp11_27+ROcp11_211*qdd[12]-qd[12]*(OMcp11_17*ROcp11_311-OMcp11_37*ROcp11_111);
    OPcp11_312 = OPcp11_37+ROcp11_311*qdd[12]+qd[12]*(OMcp11_17*ROcp11_211-OMcp11_27*ROcp11_111);

// = = Block_1_0_0_12_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp11_111;
    sens->P[2] = POcp11_211;
    sens->P[3] = POcp11_311;
    sens->R[1][1] = ROcp11_111;
    sens->R[1][2] = ROcp11_211;
    sens->R[1][3] = ROcp11_311;
    sens->R[2][1] = ROcp11_412;
    sens->R[2][2] = ROcp11_512;
    sens->R[2][3] = ROcp11_612;
    sens->R[3][1] = ROcp11_712;
    sens->R[3][2] = ROcp11_812;
    sens->R[3][3] = ROcp11_912;
    sens->V[1] = VIcp11_111;
    sens->V[2] = VIcp11_211;
    sens->V[3] = VIcp11_311;
    sens->OM[1] = OMcp11_112;
    sens->OM[2] = OMcp11_212;
    sens->OM[3] = OMcp11_312;
    sens->A[1] = ACcp11_111;
    sens->A[2] = ACcp11_211;
    sens->A[3] = ACcp11_311;
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
    RLcp12_113 = ROcp12_46*s->dpt[2][13]+ROcp12_711*s->dpt[3][13];
    RLcp12_213 = ROcp12_56*s->dpt[2][13]+ROcp12_811*s->dpt[3][13];
    RLcp12_313 = ROcp12_911*s->dpt[3][13]+s->dpt[2][13]*S6;
    POcp12_113 = RLcp12_111+RLcp12_113+RLcp12_12+RLcp12_13;
    POcp12_213 = RLcp12_211+RLcp12_213+RLcp12_22+RLcp12_23;
    POcp12_313 = RLcp12_311+RLcp12_313+q[4];
    OMcp12_113 = OMcp12_17+ROcp12_111*qd[13];
    OMcp12_213 = OMcp12_27+ROcp12_211*qd[13];
    OMcp12_313 = OMcp12_37+ROcp12_311*qd[13];
    ORcp12_113 = OMcp12_27*RLcp12_313-OMcp12_37*RLcp12_213;
    ORcp12_213 = -(OMcp12_17*RLcp12_313-OMcp12_37*RLcp12_113);
    ORcp12_313 = OMcp12_17*RLcp12_213-OMcp12_27*RLcp12_113;
    VIcp12_113 = ORcp12_111+ORcp12_113+ORcp12_12+ORcp12_13+qd[2]*C1-qd[3]*S1;
    VIcp12_213 = ORcp12_211+ORcp12_213+ORcp12_22+ORcp12_23+qd[2]*S1+qd[3]*C1;
    VIcp12_313 = ORcp12_311+ORcp12_313+qd[4];
    OPcp12_113 = OPcp12_17+ROcp12_111*qdd[13]+qd[13]*(OMcp12_27*ROcp12_311-OMcp12_37*ROcp12_211);
    OPcp12_213 = OPcp12_27+ROcp12_211*qdd[13]-qd[13]*(OMcp12_17*ROcp12_311-OMcp12_37*ROcp12_111);
    OPcp12_313 = OPcp12_37+ROcp12_311*qdd[13]+qd[13]*(OMcp12_17*ROcp12_211-OMcp12_27*ROcp12_111);
    ACcp12_113 = OMcp12_27*(ORcp12_311+ORcp12_313)-OMcp12_37*ORcp12_211-OMcp12_37*ORcp12_213+OPcp12_27*RLcp12_311+
 OPcp12_27*RLcp12_313-OPcp12_37*RLcp12_211-OPcp12_37*RLcp12_213-ORcp12_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(
 ORcp12_22+(2.0)*qd[2]*S1);
    ACcp12_213 = -(OMcp12_17*(ORcp12_311+ORcp12_313)-OMcp12_37*ORcp12_111-OMcp12_37*ORcp12_113+OPcp12_17*RLcp12_311+
 OPcp12_17*RLcp12_313-OPcp12_37*RLcp12_111-OPcp12_37*RLcp12_113-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp12_12+(2.0)*qd[2]*C1)-qd[1]*(
 ORcp12_13-(2.0)*qd[3]*S1));
    ACcp12_313 = qdd[4]+OMcp12_17*ORcp12_211+OMcp12_17*ORcp12_213-OMcp12_27*ORcp12_111-OMcp12_27*ORcp12_113+OPcp12_17*
 RLcp12_211+OPcp12_17*RLcp12_213-OPcp12_27*RLcp12_111-OPcp12_27*RLcp12_113;

// = = Block_1_0_0_13_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp12_113;
    sens->P[2] = POcp12_213;
    sens->P[3] = POcp12_313;
    sens->R[1][1] = ROcp12_111;
    sens->R[1][2] = ROcp12_211;
    sens->R[1][3] = ROcp12_311;
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

// = = Block_1_0_0_14_0_5 = = 
 
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
    RLcp13_113 = ROcp13_46*s->dpt[2][13]+ROcp13_711*s->dpt[3][13];
    RLcp13_213 = ROcp13_56*s->dpt[2][13]+ROcp13_811*s->dpt[3][13];
    RLcp13_313 = ROcp13_911*s->dpt[3][13]+s->dpt[2][13]*S6;
    OMcp13_113 = OMcp13_17+ROcp13_111*qd[13];
    OMcp13_213 = OMcp13_27+ROcp13_211*qd[13];
    OMcp13_313 = OMcp13_37+ROcp13_311*qd[13];
    ORcp13_113 = OMcp13_27*RLcp13_313-OMcp13_37*RLcp13_213;
    ORcp13_213 = -(OMcp13_17*RLcp13_313-OMcp13_37*RLcp13_113);
    ORcp13_313 = OMcp13_17*RLcp13_213-OMcp13_27*RLcp13_113;
    OPcp13_113 = OPcp13_17+ROcp13_111*qdd[13]+qd[13]*(OMcp13_27*ROcp13_311-OMcp13_37*ROcp13_211);
    OPcp13_213 = OPcp13_27+ROcp13_211*qdd[13]-qd[13]*(OMcp13_17*ROcp13_311-OMcp13_37*ROcp13_111);
    OPcp13_313 = OPcp13_37+ROcp13_311*qdd[13]+qd[13]*(OMcp13_17*ROcp13_211-OMcp13_27*ROcp13_111);
    RLcp13_114 = ROcp13_413*s->dpt[2][18];
    RLcp13_214 = ROcp13_513*s->dpt[2][18];
    RLcp13_314 = ROcp13_613*s->dpt[2][18];
    POcp13_114 = RLcp13_111+RLcp13_113+RLcp13_114+RLcp13_12+RLcp13_13;
    POcp13_214 = RLcp13_211+RLcp13_213+RLcp13_214+RLcp13_22+RLcp13_23;
    POcp13_314 = RLcp13_311+RLcp13_313+RLcp13_314+q[4];
    OMcp13_114 = OMcp13_113+ROcp13_111*qd[14];
    OMcp13_214 = OMcp13_213+ROcp13_211*qd[14];
    OMcp13_314 = OMcp13_313+ROcp13_311*qd[14];
    ORcp13_114 = OMcp13_213*RLcp13_314-OMcp13_313*RLcp13_214;
    ORcp13_214 = -(OMcp13_113*RLcp13_314-OMcp13_313*RLcp13_114);
    ORcp13_314 = OMcp13_113*RLcp13_214-OMcp13_213*RLcp13_114;
    VIcp13_114 = ORcp13_111+ORcp13_113+ORcp13_114+ORcp13_12+ORcp13_13+qd[2]*C1-qd[3]*S1;
    VIcp13_214 = ORcp13_211+ORcp13_213+ORcp13_214+ORcp13_22+ORcp13_23+qd[2]*S1+qd[3]*C1;
    VIcp13_314 = ORcp13_311+ORcp13_313+ORcp13_314+qd[4];
    OPcp13_114 = OPcp13_113+ROcp13_111*qdd[14]+qd[14]*(OMcp13_213*ROcp13_311-OMcp13_313*ROcp13_211);
    OPcp13_214 = OPcp13_213+ROcp13_211*qdd[14]-qd[14]*(OMcp13_113*ROcp13_311-OMcp13_313*ROcp13_111);
    OPcp13_314 = OPcp13_313+ROcp13_311*qdd[14]+qd[14]*(OMcp13_113*ROcp13_211-OMcp13_213*ROcp13_111);
    ACcp13_114 = OMcp13_213*ORcp13_314+OMcp13_27*(ORcp13_311+ORcp13_313)-OMcp13_313*ORcp13_214-OMcp13_37*ORcp13_211-
 OMcp13_37*ORcp13_213+OPcp13_213*RLcp13_314+OPcp13_27*RLcp13_311+OPcp13_27*RLcp13_313-OPcp13_313*RLcp13_214-OPcp13_37*
 RLcp13_211-OPcp13_37*RLcp13_213-ORcp13_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp13_22+(2.0)*qd[2]*S1);
    ACcp13_214 = -(OMcp13_113*ORcp13_314+OMcp13_17*(ORcp13_311+ORcp13_313)-OMcp13_313*ORcp13_114-OMcp13_37*ORcp13_111-
 OMcp13_37*ORcp13_113+OPcp13_113*RLcp13_314+OPcp13_17*RLcp13_311+OPcp13_17*RLcp13_313-OPcp13_313*RLcp13_114-OPcp13_37*
 RLcp13_111-OPcp13_37*RLcp13_113-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp13_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp13_13-(2.0)*qd[3]*S1));
    ACcp13_314 = qdd[4]+OMcp13_113*ORcp13_214+OMcp13_17*ORcp13_211+OMcp13_17*ORcp13_213-OMcp13_213*ORcp13_114-OMcp13_27*
 ORcp13_111-OMcp13_27*ORcp13_113+OPcp13_113*RLcp13_214+OPcp13_17*RLcp13_211+OPcp13_17*RLcp13_213-OPcp13_213*RLcp13_114-
 OPcp13_27*RLcp13_111-OPcp13_27*RLcp13_113;

// = = Block_1_0_0_14_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp13_114;
    sens->P[2] = POcp13_214;
    sens->P[3] = POcp13_314;
    sens->R[1][1] = ROcp13_111;
    sens->R[1][2] = ROcp13_211;
    sens->R[1][3] = ROcp13_311;
    sens->R[2][1] = ROcp13_414;
    sens->R[2][2] = ROcp13_514;
    sens->R[2][3] = ROcp13_614;
    sens->R[3][1] = ROcp13_714;
    sens->R[3][2] = ROcp13_814;
    sens->R[3][3] = ROcp13_914;
    sens->V[1] = VIcp13_114;
    sens->V[2] = VIcp13_214;
    sens->V[3] = VIcp13_314;
    sens->OM[1] = OMcp13_114;
    sens->OM[2] = OMcp13_214;
    sens->OM[3] = OMcp13_314;
    sens->A[1] = ACcp13_114;
    sens->A[2] = ACcp13_214;
    sens->A[3] = ACcp13_314;
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

// = = Block_1_0_0_15_0_5 = = 
 
// Sensor Kinematics 


    ROcp14_413 = ROcp14_46*C13+ROcp14_711*S13;
    ROcp14_513 = ROcp14_56*C13+ROcp14_811*S13;
    ROcp14_613 = ROcp14_911*S13+C13*S6;
    ROcp14_713 = -(ROcp14_46*S13-ROcp14_711*C13);
    ROcp14_813 = -(ROcp14_56*S13-ROcp14_811*C13);
    ROcp14_913 = ROcp14_911*C13-S13*S6;
    ROcp14_414 = ROcp14_413*C14+ROcp14_713*S14;
    ROcp14_514 = ROcp14_513*C14+ROcp14_813*S14;
    ROcp14_614 = ROcp14_613*C14+ROcp14_913*S14;
    ROcp14_714 = -(ROcp14_413*S14-ROcp14_713*C14);
    ROcp14_814 = -(ROcp14_513*S14-ROcp14_813*C14);
    ROcp14_914 = -(ROcp14_613*S14-ROcp14_913*C14);
    ROcp14_115 = ROcp14_111*C15-ROcp14_714*S15;
    ROcp14_215 = ROcp14_211*C15-ROcp14_814*S15;
    ROcp14_315 = ROcp14_311*C15-ROcp14_914*S15;
    ROcp14_715 = ROcp14_111*S15+ROcp14_714*C15;
    ROcp14_815 = ROcp14_211*S15+ROcp14_814*C15;
    ROcp14_915 = ROcp14_311*S15+ROcp14_914*C15;
    RLcp14_113 = ROcp14_46*s->dpt[2][13]+ROcp14_711*s->dpt[3][13];
    RLcp14_213 = ROcp14_56*s->dpt[2][13]+ROcp14_811*s->dpt[3][13];
    RLcp14_313 = ROcp14_911*s->dpt[3][13]+s->dpt[2][13]*S6;
    OMcp14_113 = OMcp14_17+ROcp14_111*qd[13];
    OMcp14_213 = OMcp14_27+ROcp14_211*qd[13];
    OMcp14_313 = OMcp14_37+ROcp14_311*qd[13];
    ORcp14_113 = OMcp14_27*RLcp14_313-OMcp14_37*RLcp14_213;
    ORcp14_213 = -(OMcp14_17*RLcp14_313-OMcp14_37*RLcp14_113);
    ORcp14_313 = OMcp14_17*RLcp14_213-OMcp14_27*RLcp14_113;
    OPcp14_113 = OPcp14_17+ROcp14_111*qdd[13]+qd[13]*(OMcp14_27*ROcp14_311-OMcp14_37*ROcp14_211);
    OPcp14_213 = OPcp14_27+ROcp14_211*qdd[13]-qd[13]*(OMcp14_17*ROcp14_311-OMcp14_37*ROcp14_111);
    OPcp14_313 = OPcp14_37+ROcp14_311*qdd[13]+qd[13]*(OMcp14_17*ROcp14_211-OMcp14_27*ROcp14_111);
    RLcp14_114 = ROcp14_413*s->dpt[2][18];
    RLcp14_214 = ROcp14_513*s->dpt[2][18];
    RLcp14_314 = ROcp14_613*s->dpt[2][18];
    POcp14_114 = RLcp14_111+RLcp14_113+RLcp14_114+RLcp14_12+RLcp14_13;
    POcp14_214 = RLcp14_211+RLcp14_213+RLcp14_214+RLcp14_22+RLcp14_23;
    POcp14_314 = RLcp14_311+RLcp14_313+RLcp14_314+q[4];
    OMcp14_114 = OMcp14_113+ROcp14_111*qd[14];
    OMcp14_214 = OMcp14_213+ROcp14_211*qd[14];
    OMcp14_314 = OMcp14_313+ROcp14_311*qd[14];
    ORcp14_114 = OMcp14_213*RLcp14_314-OMcp14_313*RLcp14_214;
    ORcp14_214 = -(OMcp14_113*RLcp14_314-OMcp14_313*RLcp14_114);
    ORcp14_314 = OMcp14_113*RLcp14_214-OMcp14_213*RLcp14_114;
    VIcp14_114 = ORcp14_111+ORcp14_113+ORcp14_114+ORcp14_12+ORcp14_13+qd[2]*C1-qd[3]*S1;
    VIcp14_214 = ORcp14_211+ORcp14_213+ORcp14_214+ORcp14_22+ORcp14_23+qd[2]*S1+qd[3]*C1;
    VIcp14_314 = ORcp14_311+ORcp14_313+ORcp14_314+qd[4];
    ACcp14_114 = OMcp14_213*ORcp14_314+OMcp14_27*(ORcp14_311+ORcp14_313)-OMcp14_313*ORcp14_214-OMcp14_37*ORcp14_211-
 OMcp14_37*ORcp14_213+OPcp14_213*RLcp14_314+OPcp14_27*RLcp14_311+OPcp14_27*RLcp14_313-OPcp14_313*RLcp14_214-OPcp14_37*
 RLcp14_211-OPcp14_37*RLcp14_213-ORcp14_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp14_22+(2.0)*qd[2]*S1);
    ACcp14_214 = -(OMcp14_113*ORcp14_314+OMcp14_17*(ORcp14_311+ORcp14_313)-OMcp14_313*ORcp14_114-OMcp14_37*ORcp14_111-
 OMcp14_37*ORcp14_113+OPcp14_113*RLcp14_314+OPcp14_17*RLcp14_311+OPcp14_17*RLcp14_313-OPcp14_313*RLcp14_114-OPcp14_37*
 RLcp14_111-OPcp14_37*RLcp14_113-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp14_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp14_13-(2.0)*qd[3]*S1));
    ACcp14_314 = qdd[4]+OMcp14_113*ORcp14_214+OMcp14_17*ORcp14_211+OMcp14_17*ORcp14_213-OMcp14_213*ORcp14_114-OMcp14_27*
 ORcp14_111-OMcp14_27*ORcp14_113+OPcp14_113*RLcp14_214+OPcp14_17*RLcp14_211+OPcp14_17*RLcp14_213-OPcp14_213*RLcp14_114-
 OPcp14_27*RLcp14_111-OPcp14_27*RLcp14_113;
    OMcp14_115 = OMcp14_114+ROcp14_414*qd[15];
    OMcp14_215 = OMcp14_214+ROcp14_514*qd[15];
    OMcp14_315 = OMcp14_314+ROcp14_614*qd[15];
    OPcp14_115 = OPcp14_113+ROcp14_111*qdd[14]+ROcp14_414*qdd[15]+qd[14]*(OMcp14_213*ROcp14_311-OMcp14_313*ROcp14_211)+
 qd[15]*(OMcp14_214*ROcp14_614-OMcp14_314*ROcp14_514);
    OPcp14_215 = OPcp14_213+ROcp14_211*qdd[14]+ROcp14_514*qdd[15]-qd[14]*(OMcp14_113*ROcp14_311-OMcp14_313*ROcp14_111)-
 qd[15]*(OMcp14_114*ROcp14_614-OMcp14_314*ROcp14_414);
    OPcp14_315 = OPcp14_313+ROcp14_311*qdd[14]+ROcp14_614*qdd[15]+qd[14]*(OMcp14_113*ROcp14_211-OMcp14_213*ROcp14_111)+
 qd[15]*(OMcp14_114*ROcp14_514-OMcp14_214*ROcp14_414);

// = = Block_1_0_0_15_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp14_114;
    sens->P[2] = POcp14_214;
    sens->P[3] = POcp14_314;
    sens->R[1][1] = ROcp14_115;
    sens->R[1][2] = ROcp14_215;
    sens->R[1][3] = ROcp14_315;
    sens->R[2][1] = ROcp14_414;
    sens->R[2][2] = ROcp14_514;
    sens->R[2][3] = ROcp14_614;
    sens->R[3][1] = ROcp14_715;
    sens->R[3][2] = ROcp14_815;
    sens->R[3][3] = ROcp14_915;
    sens->V[1] = VIcp14_114;
    sens->V[2] = VIcp14_214;
    sens->V[3] = VIcp14_314;
    sens->OM[1] = OMcp14_115;
    sens->OM[2] = OMcp14_215;
    sens->OM[3] = OMcp14_315;
    sens->A[1] = ACcp14_114;
    sens->A[2] = ACcp14_214;
    sens->A[3] = ACcp14_314;
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


    ROcp15_111 = ROcp15_17*C11-ROcp15_77*S11;
    ROcp15_211 = ROcp15_27*C11-ROcp15_87*S11;
    ROcp15_311 = -S11p7*C6;
    ROcp15_711 = ROcp15_17*S11+ROcp15_77*C11;
    ROcp15_811 = ROcp15_27*S11+ROcp15_87*C11;
    ROcp15_911 = C11p7*C6;
    RLcp15_111 = ROcp15_17*s->dpt[1][5]+ROcp15_77*s->dpt[3][5];
    RLcp15_211 = ROcp15_27*s->dpt[1][5]+ROcp15_87*s->dpt[3][5];
    RLcp15_311 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp15_111 = OMcp15_27*RLcp15_311-OMcp15_37*RLcp15_211;
    ORcp15_211 = -(OMcp15_17*RLcp15_311-OMcp15_37*RLcp15_111);
    ORcp15_311 = OMcp15_17*RLcp15_211-OMcp15_27*RLcp15_111;

// = = Block_1_0_0_16_0_5 = = 
 
// Sensor Kinematics 


    ROcp15_413 = ROcp15_46*C13+ROcp15_711*S13;
    ROcp15_513 = ROcp15_56*C13+ROcp15_811*S13;
    ROcp15_613 = ROcp15_911*S13+C13*S6;
    ROcp15_713 = -(ROcp15_46*S13-ROcp15_711*C13);
    ROcp15_813 = -(ROcp15_56*S13-ROcp15_811*C13);
    ROcp15_913 = ROcp15_911*C13-S13*S6;
    ROcp15_414 = ROcp15_413*C14+ROcp15_713*S14;
    ROcp15_514 = ROcp15_513*C14+ROcp15_813*S14;
    ROcp15_614 = ROcp15_613*C14+ROcp15_913*S14;
    ROcp15_714 = -(ROcp15_413*S14-ROcp15_713*C14);
    ROcp15_814 = -(ROcp15_513*S14-ROcp15_813*C14);
    ROcp15_914 = -(ROcp15_613*S14-ROcp15_913*C14);
    ROcp15_115 = ROcp15_111*C15-ROcp15_714*S15;
    ROcp15_215 = ROcp15_211*C15-ROcp15_814*S15;
    ROcp15_315 = ROcp15_311*C15-ROcp15_914*S15;
    ROcp15_715 = ROcp15_111*S15+ROcp15_714*C15;
    ROcp15_815 = ROcp15_211*S15+ROcp15_814*C15;
    ROcp15_915 = ROcp15_311*S15+ROcp15_914*C15;
    ROcp15_116 = ROcp15_115*C16+ROcp15_414*S16;
    ROcp15_216 = ROcp15_215*C16+ROcp15_514*S16;
    ROcp15_316 = ROcp15_315*C16+ROcp15_614*S16;
    ROcp15_416 = -(ROcp15_115*S16-ROcp15_414*C16);
    ROcp15_516 = -(ROcp15_215*S16-ROcp15_514*C16);
    ROcp15_616 = -(ROcp15_315*S16-ROcp15_614*C16);
    RLcp15_113 = ROcp15_46*s->dpt[2][13]+ROcp15_711*s->dpt[3][13];
    RLcp15_213 = ROcp15_56*s->dpt[2][13]+ROcp15_811*s->dpt[3][13];
    RLcp15_313 = ROcp15_911*s->dpt[3][13]+s->dpt[2][13]*S6;
    OMcp15_113 = OMcp15_17+ROcp15_111*qd[13];
    OMcp15_213 = OMcp15_27+ROcp15_211*qd[13];
    OMcp15_313 = OMcp15_37+ROcp15_311*qd[13];
    ORcp15_113 = OMcp15_27*RLcp15_313-OMcp15_37*RLcp15_213;
    ORcp15_213 = -(OMcp15_17*RLcp15_313-OMcp15_37*RLcp15_113);
    ORcp15_313 = OMcp15_17*RLcp15_213-OMcp15_27*RLcp15_113;
    OPcp15_113 = OPcp15_17+ROcp15_111*qdd[13]+qd[13]*(OMcp15_27*ROcp15_311-OMcp15_37*ROcp15_211);
    OPcp15_213 = OPcp15_27+ROcp15_211*qdd[13]-qd[13]*(OMcp15_17*ROcp15_311-OMcp15_37*ROcp15_111);
    OPcp15_313 = OPcp15_37+ROcp15_311*qdd[13]+qd[13]*(OMcp15_17*ROcp15_211-OMcp15_27*ROcp15_111);
    RLcp15_114 = ROcp15_413*s->dpt[2][18];
    RLcp15_214 = ROcp15_513*s->dpt[2][18];
    RLcp15_314 = ROcp15_613*s->dpt[2][18];
    POcp15_114 = RLcp15_111+RLcp15_113+RLcp15_114+RLcp15_12+RLcp15_13;
    POcp15_214 = RLcp15_211+RLcp15_213+RLcp15_214+RLcp15_22+RLcp15_23;
    POcp15_314 = RLcp15_311+RLcp15_313+RLcp15_314+q[4];
    OMcp15_114 = OMcp15_113+ROcp15_111*qd[14];
    OMcp15_214 = OMcp15_213+ROcp15_211*qd[14];
    OMcp15_314 = OMcp15_313+ROcp15_311*qd[14];
    ORcp15_114 = OMcp15_213*RLcp15_314-OMcp15_313*RLcp15_214;
    ORcp15_214 = -(OMcp15_113*RLcp15_314-OMcp15_313*RLcp15_114);
    ORcp15_314 = OMcp15_113*RLcp15_214-OMcp15_213*RLcp15_114;
    VIcp15_114 = ORcp15_111+ORcp15_113+ORcp15_114+ORcp15_12+ORcp15_13+qd[2]*C1-qd[3]*S1;
    VIcp15_214 = ORcp15_211+ORcp15_213+ORcp15_214+ORcp15_22+ORcp15_23+qd[2]*S1+qd[3]*C1;
    VIcp15_314 = ORcp15_311+ORcp15_313+ORcp15_314+qd[4];
    ACcp15_114 = OMcp15_213*ORcp15_314+OMcp15_27*(ORcp15_311+ORcp15_313)-OMcp15_313*ORcp15_214-OMcp15_37*ORcp15_211-
 OMcp15_37*ORcp15_213+OPcp15_213*RLcp15_314+OPcp15_27*RLcp15_311+OPcp15_27*RLcp15_313-OPcp15_313*RLcp15_214-OPcp15_37*
 RLcp15_211-OPcp15_37*RLcp15_213-ORcp15_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp15_22+(2.0)*qd[2]*S1);
    ACcp15_214 = -(OMcp15_113*ORcp15_314+OMcp15_17*(ORcp15_311+ORcp15_313)-OMcp15_313*ORcp15_114-OMcp15_37*ORcp15_111-
 OMcp15_37*ORcp15_113+OPcp15_113*RLcp15_314+OPcp15_17*RLcp15_311+OPcp15_17*RLcp15_313-OPcp15_313*RLcp15_114-OPcp15_37*
 RLcp15_111-OPcp15_37*RLcp15_113-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp15_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp15_13-(2.0)*qd[3]*S1));
    ACcp15_314 = qdd[4]+OMcp15_113*ORcp15_214+OMcp15_17*ORcp15_211+OMcp15_17*ORcp15_213-OMcp15_213*ORcp15_114-OMcp15_27*
 ORcp15_111-OMcp15_27*ORcp15_113+OPcp15_113*RLcp15_214+OPcp15_17*RLcp15_211+OPcp15_17*RLcp15_213-OPcp15_213*RLcp15_114-
 OPcp15_27*RLcp15_111-OPcp15_27*RLcp15_113;
    OMcp15_115 = OMcp15_114+ROcp15_414*qd[15];
    OMcp15_215 = OMcp15_214+ROcp15_514*qd[15];
    OMcp15_315 = OMcp15_314+ROcp15_614*qd[15];
    OMcp15_116 = OMcp15_115+ROcp15_715*qd[16];
    OMcp15_216 = OMcp15_215+ROcp15_815*qd[16];
    OMcp15_316 = OMcp15_315+ROcp15_915*qd[16];
    OPcp15_116 = OPcp15_113+ROcp15_111*qdd[14]+ROcp15_414*qdd[15]+ROcp15_715*qdd[16]+qd[14]*(OMcp15_213*ROcp15_311-
 OMcp15_313*ROcp15_211)+qd[15]*(OMcp15_214*ROcp15_614-OMcp15_314*ROcp15_514)+qd[16]*(OMcp15_215*ROcp15_915-OMcp15_315*
 ROcp15_815);
    OPcp15_216 = OPcp15_213+ROcp15_211*qdd[14]+ROcp15_514*qdd[15]+ROcp15_815*qdd[16]-qd[14]*(OMcp15_113*ROcp15_311-
 OMcp15_313*ROcp15_111)-qd[15]*(OMcp15_114*ROcp15_614-OMcp15_314*ROcp15_414)-qd[16]*(OMcp15_115*ROcp15_915-OMcp15_315*
 ROcp15_715);
    OPcp15_316 = OPcp15_313+ROcp15_311*qdd[14]+ROcp15_614*qdd[15]+ROcp15_915*qdd[16]+qd[14]*(OMcp15_113*ROcp15_211-
 OMcp15_213*ROcp15_111)+qd[15]*(OMcp15_114*ROcp15_514-OMcp15_214*ROcp15_414)+qd[16]*(OMcp15_115*ROcp15_815-OMcp15_215*
 ROcp15_715);

// = = Block_1_0_0_16_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp15_114;
    sens->P[2] = POcp15_214;
    sens->P[3] = POcp15_314;
    sens->R[1][1] = ROcp15_116;
    sens->R[1][2] = ROcp15_216;
    sens->R[1][3] = ROcp15_316;
    sens->R[2][1] = ROcp15_416;
    sens->R[2][2] = ROcp15_516;
    sens->R[2][3] = ROcp15_616;
    sens->R[3][1] = ROcp15_715;
    sens->R[3][2] = ROcp15_815;
    sens->R[3][3] = ROcp15_915;
    sens->V[1] = VIcp15_114;
    sens->V[2] = VIcp15_214;
    sens->V[3] = VIcp15_314;
    sens->OM[1] = OMcp15_116;
    sens->OM[2] = OMcp15_216;
    sens->OM[3] = OMcp15_316;
    sens->A[1] = ACcp15_114;
    sens->A[2] = ACcp15_214;
    sens->A[3] = ACcp15_314;
    sens->OMP[1] = OPcp15_116;
    sens->OMP[2] = OPcp15_216;
    sens->OMP[3] = OPcp15_316;
 
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


    ROcp16_111 = ROcp16_17*C11-ROcp16_77*S11;
    ROcp16_211 = ROcp16_27*C11-ROcp16_87*S11;
    ROcp16_311 = -S11p7*C6;
    ROcp16_711 = ROcp16_17*S11+ROcp16_77*C11;
    ROcp16_811 = ROcp16_27*S11+ROcp16_87*C11;
    ROcp16_911 = C11p7*C6;
    RLcp16_111 = ROcp16_17*s->dpt[1][5]+ROcp16_77*s->dpt[3][5];
    RLcp16_211 = ROcp16_27*s->dpt[1][5]+ROcp16_87*s->dpt[3][5];
    RLcp16_311 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp16_111 = OMcp16_27*RLcp16_311-OMcp16_37*RLcp16_211;
    ORcp16_211 = -(OMcp16_17*RLcp16_311-OMcp16_37*RLcp16_111);
    ORcp16_311 = OMcp16_17*RLcp16_211-OMcp16_27*RLcp16_111;

// = = Block_1_0_0_17_0_5 = = 
 
// Sensor Kinematics 


    ROcp16_413 = ROcp16_46*C13+ROcp16_711*S13;
    ROcp16_513 = ROcp16_56*C13+ROcp16_811*S13;
    ROcp16_613 = ROcp16_911*S13+C13*S6;
    ROcp16_713 = -(ROcp16_46*S13-ROcp16_711*C13);
    ROcp16_813 = -(ROcp16_56*S13-ROcp16_811*C13);
    ROcp16_913 = ROcp16_911*C13-S13*S6;
    ROcp16_414 = ROcp16_413*C14+ROcp16_713*S14;
    ROcp16_514 = ROcp16_513*C14+ROcp16_813*S14;
    ROcp16_614 = ROcp16_613*C14+ROcp16_913*S14;
    ROcp16_714 = -(ROcp16_413*S14-ROcp16_713*C14);
    ROcp16_814 = -(ROcp16_513*S14-ROcp16_813*C14);
    ROcp16_914 = -(ROcp16_613*S14-ROcp16_913*C14);
    ROcp16_115 = ROcp16_111*C15-ROcp16_714*S15;
    ROcp16_215 = ROcp16_211*C15-ROcp16_814*S15;
    ROcp16_315 = ROcp16_311*C15-ROcp16_914*S15;
    ROcp16_715 = ROcp16_111*S15+ROcp16_714*C15;
    ROcp16_815 = ROcp16_211*S15+ROcp16_814*C15;
    ROcp16_915 = ROcp16_311*S15+ROcp16_914*C15;
    ROcp16_116 = ROcp16_115*C16+ROcp16_414*S16;
    ROcp16_216 = ROcp16_215*C16+ROcp16_514*S16;
    ROcp16_316 = ROcp16_315*C16+ROcp16_614*S16;
    ROcp16_416 = -(ROcp16_115*S16-ROcp16_414*C16);
    ROcp16_516 = -(ROcp16_215*S16-ROcp16_514*C16);
    ROcp16_616 = -(ROcp16_315*S16-ROcp16_614*C16);
    ROcp16_417 = ROcp16_416*C17+ROcp16_715*S17;
    ROcp16_517 = ROcp16_516*C17+ROcp16_815*S17;
    ROcp16_617 = ROcp16_616*C17+ROcp16_915*S17;
    ROcp16_717 = -(ROcp16_416*S17-ROcp16_715*C17);
    ROcp16_817 = -(ROcp16_516*S17-ROcp16_815*C17);
    ROcp16_917 = -(ROcp16_616*S17-ROcp16_915*C17);
    RLcp16_113 = ROcp16_46*s->dpt[2][13]+ROcp16_711*s->dpt[3][13];
    RLcp16_213 = ROcp16_56*s->dpt[2][13]+ROcp16_811*s->dpt[3][13];
    RLcp16_313 = ROcp16_911*s->dpt[3][13]+s->dpt[2][13]*S6;
    OMcp16_113 = OMcp16_17+ROcp16_111*qd[13];
    OMcp16_213 = OMcp16_27+ROcp16_211*qd[13];
    OMcp16_313 = OMcp16_37+ROcp16_311*qd[13];
    ORcp16_113 = OMcp16_27*RLcp16_313-OMcp16_37*RLcp16_213;
    ORcp16_213 = -(OMcp16_17*RLcp16_313-OMcp16_37*RLcp16_113);
    ORcp16_313 = OMcp16_17*RLcp16_213-OMcp16_27*RLcp16_113;
    OPcp16_113 = OPcp16_17+ROcp16_111*qdd[13]+qd[13]*(OMcp16_27*ROcp16_311-OMcp16_37*ROcp16_211);
    OPcp16_213 = OPcp16_27+ROcp16_211*qdd[13]-qd[13]*(OMcp16_17*ROcp16_311-OMcp16_37*ROcp16_111);
    OPcp16_313 = OPcp16_37+ROcp16_311*qdd[13]+qd[13]*(OMcp16_17*ROcp16_211-OMcp16_27*ROcp16_111);
    RLcp16_114 = ROcp16_413*s->dpt[2][18];
    RLcp16_214 = ROcp16_513*s->dpt[2][18];
    RLcp16_314 = ROcp16_613*s->dpt[2][18];
    OMcp16_114 = OMcp16_113+ROcp16_111*qd[14];
    OMcp16_214 = OMcp16_213+ROcp16_211*qd[14];
    OMcp16_314 = OMcp16_313+ROcp16_311*qd[14];
    ORcp16_114 = OMcp16_213*RLcp16_314-OMcp16_313*RLcp16_214;
    ORcp16_214 = -(OMcp16_113*RLcp16_314-OMcp16_313*RLcp16_114);
    ORcp16_314 = OMcp16_113*RLcp16_214-OMcp16_213*RLcp16_114;
    OMcp16_115 = OMcp16_114+ROcp16_414*qd[15];
    OMcp16_215 = OMcp16_214+ROcp16_514*qd[15];
    OMcp16_315 = OMcp16_314+ROcp16_614*qd[15];
    OMcp16_116 = OMcp16_115+ROcp16_715*qd[16];
    OMcp16_216 = OMcp16_215+ROcp16_815*qd[16];
    OMcp16_316 = OMcp16_315+ROcp16_915*qd[16];
    OPcp16_116 = OPcp16_113+ROcp16_111*qdd[14]+ROcp16_414*qdd[15]+ROcp16_715*qdd[16]+qd[14]*(OMcp16_213*ROcp16_311-
 OMcp16_313*ROcp16_211)+qd[15]*(OMcp16_214*ROcp16_614-OMcp16_314*ROcp16_514)+qd[16]*(OMcp16_215*ROcp16_915-OMcp16_315*
 ROcp16_815);
    OPcp16_216 = OPcp16_213+ROcp16_211*qdd[14]+ROcp16_514*qdd[15]+ROcp16_815*qdd[16]-qd[14]*(OMcp16_113*ROcp16_311-
 OMcp16_313*ROcp16_111)-qd[15]*(OMcp16_114*ROcp16_614-OMcp16_314*ROcp16_414)-qd[16]*(OMcp16_115*ROcp16_915-OMcp16_315*
 ROcp16_715);
    OPcp16_316 = OPcp16_313+ROcp16_311*qdd[14]+ROcp16_614*qdd[15]+ROcp16_915*qdd[16]+qd[14]*(OMcp16_113*ROcp16_211-
 OMcp16_213*ROcp16_111)+qd[15]*(OMcp16_114*ROcp16_514-OMcp16_214*ROcp16_414)+qd[16]*(OMcp16_115*ROcp16_815-OMcp16_215*
 ROcp16_715);
    RLcp16_117 = ROcp16_116*s->dpt[1][22]+ROcp16_416*s->dpt[2][22]+ROcp16_715*s->dpt[3][22];
    RLcp16_217 = ROcp16_216*s->dpt[1][22]+ROcp16_516*s->dpt[2][22]+ROcp16_815*s->dpt[3][22];
    RLcp16_317 = ROcp16_316*s->dpt[1][22]+ROcp16_616*s->dpt[2][22]+ROcp16_915*s->dpt[3][22];
    POcp16_117 = RLcp16_111+RLcp16_113+RLcp16_114+RLcp16_117+RLcp16_12+RLcp16_13;
    POcp16_217 = RLcp16_211+RLcp16_213+RLcp16_214+RLcp16_217+RLcp16_22+RLcp16_23;
    POcp16_317 = RLcp16_311+RLcp16_313+RLcp16_314+RLcp16_317+q[4];
    ORcp16_117 = OMcp16_216*RLcp16_317-OMcp16_316*RLcp16_217;
    ORcp16_217 = -(OMcp16_116*RLcp16_317-OMcp16_316*RLcp16_117);
    ORcp16_317 = OMcp16_116*RLcp16_217-OMcp16_216*RLcp16_117;
    VIcp16_117 = ORcp16_111+ORcp16_113+ORcp16_114+ORcp16_117+ORcp16_12+ORcp16_13+qd[2]*C1-qd[3]*S1;
    VIcp16_217 = ORcp16_211+ORcp16_213+ORcp16_214+ORcp16_217+ORcp16_22+ORcp16_23+qd[2]*S1+qd[3]*C1;
    VIcp16_317 = ORcp16_311+ORcp16_313+ORcp16_314+ORcp16_317+qd[4];
    ACcp16_117 = OMcp16_213*ORcp16_314+OMcp16_216*ORcp16_317+OMcp16_27*(ORcp16_311+ORcp16_313)-OMcp16_313*ORcp16_214-
 OMcp16_316*ORcp16_217-OMcp16_37*ORcp16_211-OMcp16_37*ORcp16_213+OPcp16_213*RLcp16_314+OPcp16_216*RLcp16_317+OPcp16_27*
 RLcp16_311+OPcp16_27*RLcp16_313-OPcp16_313*RLcp16_214-OPcp16_316*RLcp16_217-OPcp16_37*RLcp16_211-OPcp16_37*RLcp16_213-
 ORcp16_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp16_22+(2.0)*qd[2]*S1);
    ACcp16_217 = -(OMcp16_113*ORcp16_314+OMcp16_116*ORcp16_317+OMcp16_17*(ORcp16_311+ORcp16_313)-OMcp16_313*ORcp16_114-
 OMcp16_316*ORcp16_117-OMcp16_37*ORcp16_111-OMcp16_37*ORcp16_113+OPcp16_113*RLcp16_314+OPcp16_116*RLcp16_317+OPcp16_17*
 RLcp16_311+OPcp16_17*RLcp16_313-OPcp16_313*RLcp16_114-OPcp16_316*RLcp16_117-OPcp16_37*RLcp16_111-OPcp16_37*RLcp16_113-qdd[2]
 *S1-qdd[3]*C1-qd[1]*(ORcp16_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp16_13-(2.0)*qd[3]*S1));
    ACcp16_317 = qdd[4]+OMcp16_113*ORcp16_214+OMcp16_116*ORcp16_217+OMcp16_17*ORcp16_211+OMcp16_17*ORcp16_213-OMcp16_213*
 ORcp16_114-OMcp16_216*ORcp16_117-OMcp16_27*ORcp16_111-OMcp16_27*ORcp16_113+OPcp16_113*RLcp16_214+OPcp16_116*RLcp16_217+
 OPcp16_17*RLcp16_211+OPcp16_17*RLcp16_213-OPcp16_213*RLcp16_114-OPcp16_216*RLcp16_117-OPcp16_27*RLcp16_111-OPcp16_27*
 RLcp16_113;

// = = Block_1_0_0_17_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp16_117;
    sens->P[2] = POcp16_217;
    sens->P[3] = POcp16_317;
    sens->R[1][1] = ROcp16_116;
    sens->R[1][2] = ROcp16_216;
    sens->R[1][3] = ROcp16_316;
    sens->R[2][1] = ROcp16_417;
    sens->R[2][2] = ROcp16_517;
    sens->R[2][3] = ROcp16_617;
    sens->R[3][1] = ROcp16_717;
    sens->R[3][2] = ROcp16_817;
    sens->R[3][3] = ROcp16_917;
    sens->V[1] = VIcp16_117;
    sens->V[2] = VIcp16_217;
    sens->V[3] = VIcp16_317;
    sens->OM[1] = OMcp16_116;
    sens->OM[2] = OMcp16_216;
    sens->OM[3] = OMcp16_316;
    sens->A[1] = ACcp16_117;
    sens->A[2] = ACcp16_217;
    sens->A[3] = ACcp16_317;
    sens->OMP[1] = OPcp16_116;
    sens->OMP[2] = OPcp16_216;
    sens->OMP[3] = OPcp16_316;
 
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


    ROcp17_111 = ROcp17_17*C11-ROcp17_77*S11;
    ROcp17_211 = ROcp17_27*C11-ROcp17_87*S11;
    ROcp17_311 = -S11p7*C6;
    ROcp17_711 = ROcp17_17*S11+ROcp17_77*C11;
    ROcp17_811 = ROcp17_27*S11+ROcp17_87*C11;
    ROcp17_911 = C11p7*C6;
    RLcp17_111 = ROcp17_17*s->dpt[1][5]+ROcp17_77*s->dpt[3][5];
    RLcp17_211 = ROcp17_27*s->dpt[1][5]+ROcp17_87*s->dpt[3][5];
    RLcp17_311 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp17_111 = OMcp17_27*RLcp17_311-OMcp17_37*RLcp17_211;
    ORcp17_211 = -(OMcp17_17*RLcp17_311-OMcp17_37*RLcp17_111);
    ORcp17_311 = OMcp17_17*RLcp17_211-OMcp17_27*RLcp17_111;

// = = Block_1_0_0_18_0_5 = = 
 
// Sensor Kinematics 


    ROcp17_413 = ROcp17_46*C13+ROcp17_711*S13;
    ROcp17_513 = ROcp17_56*C13+ROcp17_811*S13;
    ROcp17_613 = ROcp17_911*S13+C13*S6;
    ROcp17_713 = -(ROcp17_46*S13-ROcp17_711*C13);
    ROcp17_813 = -(ROcp17_56*S13-ROcp17_811*C13);
    ROcp17_913 = ROcp17_911*C13-S13*S6;
    ROcp17_414 = ROcp17_413*C14+ROcp17_713*S14;
    ROcp17_514 = ROcp17_513*C14+ROcp17_813*S14;
    ROcp17_614 = ROcp17_613*C14+ROcp17_913*S14;
    ROcp17_714 = -(ROcp17_413*S14-ROcp17_713*C14);
    ROcp17_814 = -(ROcp17_513*S14-ROcp17_813*C14);
    ROcp17_914 = -(ROcp17_613*S14-ROcp17_913*C14);
    ROcp17_115 = ROcp17_111*C15-ROcp17_714*S15;
    ROcp17_215 = ROcp17_211*C15-ROcp17_814*S15;
    ROcp17_315 = ROcp17_311*C15-ROcp17_914*S15;
    ROcp17_715 = ROcp17_111*S15+ROcp17_714*C15;
    ROcp17_815 = ROcp17_211*S15+ROcp17_814*C15;
    ROcp17_915 = ROcp17_311*S15+ROcp17_914*C15;
    ROcp17_116 = ROcp17_115*C16+ROcp17_414*S16;
    ROcp17_216 = ROcp17_215*C16+ROcp17_514*S16;
    ROcp17_316 = ROcp17_315*C16+ROcp17_614*S16;
    ROcp17_416 = -(ROcp17_115*S16-ROcp17_414*C16);
    ROcp17_516 = -(ROcp17_215*S16-ROcp17_514*C16);
    ROcp17_616 = -(ROcp17_315*S16-ROcp17_614*C16);
    ROcp17_417 = ROcp17_416*C17+ROcp17_715*S17;
    ROcp17_517 = ROcp17_516*C17+ROcp17_815*S17;
    ROcp17_617 = ROcp17_616*C17+ROcp17_915*S17;
    ROcp17_717 = -(ROcp17_416*S17-ROcp17_715*C17);
    ROcp17_817 = -(ROcp17_516*S17-ROcp17_815*C17);
    ROcp17_917 = -(ROcp17_616*S17-ROcp17_915*C17);
    ROcp17_118 = ROcp17_116*C18-ROcp17_717*S18;
    ROcp17_218 = ROcp17_216*C18-ROcp17_817*S18;
    ROcp17_318 = ROcp17_316*C18-ROcp17_917*S18;
    ROcp17_718 = ROcp17_116*S18+ROcp17_717*C18;
    ROcp17_818 = ROcp17_216*S18+ROcp17_817*C18;
    ROcp17_918 = ROcp17_316*S18+ROcp17_917*C18;
    RLcp17_113 = ROcp17_46*s->dpt[2][13]+ROcp17_711*s->dpt[3][13];
    RLcp17_213 = ROcp17_56*s->dpt[2][13]+ROcp17_811*s->dpt[3][13];
    RLcp17_313 = ROcp17_911*s->dpt[3][13]+s->dpt[2][13]*S6;
    OMcp17_113 = OMcp17_17+ROcp17_111*qd[13];
    OMcp17_213 = OMcp17_27+ROcp17_211*qd[13];
    OMcp17_313 = OMcp17_37+ROcp17_311*qd[13];
    ORcp17_113 = OMcp17_27*RLcp17_313-OMcp17_37*RLcp17_213;
    ORcp17_213 = -(OMcp17_17*RLcp17_313-OMcp17_37*RLcp17_113);
    ORcp17_313 = OMcp17_17*RLcp17_213-OMcp17_27*RLcp17_113;
    OPcp17_113 = OPcp17_17+ROcp17_111*qdd[13]+qd[13]*(OMcp17_27*ROcp17_311-OMcp17_37*ROcp17_211);
    OPcp17_213 = OPcp17_27+ROcp17_211*qdd[13]-qd[13]*(OMcp17_17*ROcp17_311-OMcp17_37*ROcp17_111);
    OPcp17_313 = OPcp17_37+ROcp17_311*qdd[13]+qd[13]*(OMcp17_17*ROcp17_211-OMcp17_27*ROcp17_111);
    RLcp17_114 = ROcp17_413*s->dpt[2][18];
    RLcp17_214 = ROcp17_513*s->dpt[2][18];
    RLcp17_314 = ROcp17_613*s->dpt[2][18];
    OMcp17_114 = OMcp17_113+ROcp17_111*qd[14];
    OMcp17_214 = OMcp17_213+ROcp17_211*qd[14];
    OMcp17_314 = OMcp17_313+ROcp17_311*qd[14];
    ORcp17_114 = OMcp17_213*RLcp17_314-OMcp17_313*RLcp17_214;
    ORcp17_214 = -(OMcp17_113*RLcp17_314-OMcp17_313*RLcp17_114);
    ORcp17_314 = OMcp17_113*RLcp17_214-OMcp17_213*RLcp17_114;
    OMcp17_115 = OMcp17_114+ROcp17_414*qd[15];
    OMcp17_215 = OMcp17_214+ROcp17_514*qd[15];
    OMcp17_315 = OMcp17_314+ROcp17_614*qd[15];
    OMcp17_116 = OMcp17_115+ROcp17_715*qd[16];
    OMcp17_216 = OMcp17_215+ROcp17_815*qd[16];
    OMcp17_316 = OMcp17_315+ROcp17_915*qd[16];
    OPcp17_116 = OPcp17_113+ROcp17_111*qdd[14]+ROcp17_414*qdd[15]+ROcp17_715*qdd[16]+qd[14]*(OMcp17_213*ROcp17_311-
 OMcp17_313*ROcp17_211)+qd[15]*(OMcp17_214*ROcp17_614-OMcp17_314*ROcp17_514)+qd[16]*(OMcp17_215*ROcp17_915-OMcp17_315*
 ROcp17_815);
    OPcp17_216 = OPcp17_213+ROcp17_211*qdd[14]+ROcp17_514*qdd[15]+ROcp17_815*qdd[16]-qd[14]*(OMcp17_113*ROcp17_311-
 OMcp17_313*ROcp17_111)-qd[15]*(OMcp17_114*ROcp17_614-OMcp17_314*ROcp17_414)-qd[16]*(OMcp17_115*ROcp17_915-OMcp17_315*
 ROcp17_715);
    OPcp17_316 = OPcp17_313+ROcp17_311*qdd[14]+ROcp17_614*qdd[15]+ROcp17_915*qdd[16]+qd[14]*(OMcp17_113*ROcp17_211-
 OMcp17_213*ROcp17_111)+qd[15]*(OMcp17_114*ROcp17_514-OMcp17_214*ROcp17_414)+qd[16]*(OMcp17_115*ROcp17_815-OMcp17_215*
 ROcp17_715);
    RLcp17_117 = ROcp17_116*s->dpt[1][22]+ROcp17_416*s->dpt[2][22]+ROcp17_715*s->dpt[3][22];
    RLcp17_217 = ROcp17_216*s->dpt[1][22]+ROcp17_516*s->dpt[2][22]+ROcp17_815*s->dpt[3][22];
    RLcp17_317 = ROcp17_316*s->dpt[1][22]+ROcp17_616*s->dpt[2][22]+ROcp17_915*s->dpt[3][22];
    POcp17_117 = RLcp17_111+RLcp17_113+RLcp17_114+RLcp17_117+RLcp17_12+RLcp17_13;
    POcp17_217 = RLcp17_211+RLcp17_213+RLcp17_214+RLcp17_217+RLcp17_22+RLcp17_23;
    POcp17_317 = RLcp17_311+RLcp17_313+RLcp17_314+RLcp17_317+q[4];
    ORcp17_117 = OMcp17_216*RLcp17_317-OMcp17_316*RLcp17_217;
    ORcp17_217 = -(OMcp17_116*RLcp17_317-OMcp17_316*RLcp17_117);
    ORcp17_317 = OMcp17_116*RLcp17_217-OMcp17_216*RLcp17_117;
    VIcp17_117 = ORcp17_111+ORcp17_113+ORcp17_114+ORcp17_117+ORcp17_12+ORcp17_13+qd[2]*C1-qd[3]*S1;
    VIcp17_217 = ORcp17_211+ORcp17_213+ORcp17_214+ORcp17_217+ORcp17_22+ORcp17_23+qd[2]*S1+qd[3]*C1;
    VIcp17_317 = ORcp17_311+ORcp17_313+ORcp17_314+ORcp17_317+qd[4];
    ACcp17_117 = OMcp17_213*ORcp17_314+OMcp17_216*ORcp17_317+OMcp17_27*(ORcp17_311+ORcp17_313)-OMcp17_313*ORcp17_214-
 OMcp17_316*ORcp17_217-OMcp17_37*ORcp17_211-OMcp17_37*ORcp17_213+OPcp17_213*RLcp17_314+OPcp17_216*RLcp17_317+OPcp17_27*
 RLcp17_311+OPcp17_27*RLcp17_313-OPcp17_313*RLcp17_214-OPcp17_316*RLcp17_217-OPcp17_37*RLcp17_211-OPcp17_37*RLcp17_213-
 ORcp17_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp17_22+(2.0)*qd[2]*S1);
    ACcp17_217 = -(OMcp17_113*ORcp17_314+OMcp17_116*ORcp17_317+OMcp17_17*(ORcp17_311+ORcp17_313)-OMcp17_313*ORcp17_114-
 OMcp17_316*ORcp17_117-OMcp17_37*ORcp17_111-OMcp17_37*ORcp17_113+OPcp17_113*RLcp17_314+OPcp17_116*RLcp17_317+OPcp17_17*
 RLcp17_311+OPcp17_17*RLcp17_313-OPcp17_313*RLcp17_114-OPcp17_316*RLcp17_117-OPcp17_37*RLcp17_111-OPcp17_37*RLcp17_113-qdd[2]
 *S1-qdd[3]*C1-qd[1]*(ORcp17_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp17_13-(2.0)*qd[3]*S1));
    ACcp17_317 = qdd[4]+OMcp17_113*ORcp17_214+OMcp17_116*ORcp17_217+OMcp17_17*ORcp17_211+OMcp17_17*ORcp17_213-OMcp17_213*
 ORcp17_114-OMcp17_216*ORcp17_117-OMcp17_27*ORcp17_111-OMcp17_27*ORcp17_113+OPcp17_113*RLcp17_214+OPcp17_116*RLcp17_217+
 OPcp17_17*RLcp17_211+OPcp17_17*RLcp17_213-OPcp17_213*RLcp17_114-OPcp17_216*RLcp17_117-OPcp17_27*RLcp17_111-OPcp17_27*
 RLcp17_113;
    OMcp17_118 = OMcp17_116+ROcp17_417*qd[18];
    OMcp17_218 = OMcp17_216+ROcp17_517*qd[18];
    OMcp17_318 = OMcp17_316+ROcp17_617*qd[18];
    OPcp17_118 = OPcp17_116+ROcp17_417*qdd[18]+qd[18]*(OMcp17_216*ROcp17_617-OMcp17_316*ROcp17_517);
    OPcp17_218 = OPcp17_216+ROcp17_517*qdd[18]-qd[18]*(OMcp17_116*ROcp17_617-OMcp17_316*ROcp17_417);
    OPcp17_318 = OPcp17_316+ROcp17_617*qdd[18]+qd[18]*(OMcp17_116*ROcp17_517-OMcp17_216*ROcp17_417);

// = = Block_1_0_0_18_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp17_117;
    sens->P[2] = POcp17_217;
    sens->P[3] = POcp17_317;
    sens->R[1][1] = ROcp17_118;
    sens->R[1][2] = ROcp17_218;
    sens->R[1][3] = ROcp17_318;
    sens->R[2][1] = ROcp17_417;
    sens->R[2][2] = ROcp17_517;
    sens->R[2][3] = ROcp17_617;
    sens->R[3][1] = ROcp17_718;
    sens->R[3][2] = ROcp17_818;
    sens->R[3][3] = ROcp17_918;
    sens->V[1] = VIcp17_117;
    sens->V[2] = VIcp17_217;
    sens->V[3] = VIcp17_317;
    sens->OM[1] = OMcp17_118;
    sens->OM[2] = OMcp17_218;
    sens->OM[3] = OMcp17_318;
    sens->A[1] = ACcp17_117;
    sens->A[2] = ACcp17_217;
    sens->A[3] = ACcp17_317;
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


    ROcp18_111 = ROcp18_17*C11-ROcp18_77*S11;
    ROcp18_211 = ROcp18_27*C11-ROcp18_87*S11;
    ROcp18_311 = -S11p7*C6;
    ROcp18_711 = ROcp18_17*S11+ROcp18_77*C11;
    ROcp18_811 = ROcp18_27*S11+ROcp18_87*C11;
    ROcp18_911 = C11p7*C6;
    RLcp18_111 = ROcp18_17*s->dpt[1][5]+ROcp18_77*s->dpt[3][5];
    RLcp18_211 = ROcp18_27*s->dpt[1][5]+ROcp18_87*s->dpt[3][5];
    RLcp18_311 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp18_111 = OMcp18_27*RLcp18_311-OMcp18_37*RLcp18_211;
    ORcp18_211 = -(OMcp18_17*RLcp18_311-OMcp18_37*RLcp18_111);
    ORcp18_311 = OMcp18_17*RLcp18_211-OMcp18_27*RLcp18_111;

// = = Block_1_0_0_19_0_6 = = 
 
// Sensor Kinematics 


    ROcp18_419 = ROcp18_46*C19+ROcp18_711*S19;
    ROcp18_519 = ROcp18_56*C19+ROcp18_811*S19;
    ROcp18_619 = ROcp18_911*S19+C19*S6;
    ROcp18_719 = -(ROcp18_46*S19-ROcp18_711*C19);
    ROcp18_819 = -(ROcp18_56*S19-ROcp18_811*C19);
    ROcp18_919 = ROcp18_911*C19-S19*S6;
    RLcp18_119 = ROcp18_46*s->dpt[2][14]+ROcp18_711*s->dpt[3][14];
    RLcp18_219 = ROcp18_56*s->dpt[2][14]+ROcp18_811*s->dpt[3][14];
    RLcp18_319 = ROcp18_911*s->dpt[3][14]+s->dpt[2][14]*S6;
    POcp18_119 = RLcp18_111+RLcp18_119+RLcp18_12+RLcp18_13;
    POcp18_219 = RLcp18_211+RLcp18_219+RLcp18_22+RLcp18_23;
    POcp18_319 = RLcp18_311+RLcp18_319+q[4];
    OMcp18_119 = OMcp18_17+ROcp18_111*qd[19];
    OMcp18_219 = OMcp18_27+ROcp18_211*qd[19];
    OMcp18_319 = OMcp18_37+ROcp18_311*qd[19];
    ORcp18_119 = OMcp18_27*RLcp18_319-OMcp18_37*RLcp18_219;
    ORcp18_219 = -(OMcp18_17*RLcp18_319-OMcp18_37*RLcp18_119);
    ORcp18_319 = OMcp18_17*RLcp18_219-OMcp18_27*RLcp18_119;
    VIcp18_119 = ORcp18_111+ORcp18_119+ORcp18_12+ORcp18_13+qd[2]*C1-qd[3]*S1;
    VIcp18_219 = ORcp18_211+ORcp18_219+ORcp18_22+ORcp18_23+qd[2]*S1+qd[3]*C1;
    VIcp18_319 = ORcp18_311+ORcp18_319+qd[4];
    OPcp18_119 = OPcp18_17+ROcp18_111*qdd[19]+qd[19]*(OMcp18_27*ROcp18_311-OMcp18_37*ROcp18_211);
    OPcp18_219 = OPcp18_27+ROcp18_211*qdd[19]-qd[19]*(OMcp18_17*ROcp18_311-OMcp18_37*ROcp18_111);
    OPcp18_319 = OPcp18_37+ROcp18_311*qdd[19]+qd[19]*(OMcp18_17*ROcp18_211-OMcp18_27*ROcp18_111);
    ACcp18_119 = OMcp18_27*(ORcp18_311+ORcp18_319)-OMcp18_37*ORcp18_211-OMcp18_37*ORcp18_219+OPcp18_27*RLcp18_311+
 OPcp18_27*RLcp18_319-OPcp18_37*RLcp18_211-OPcp18_37*RLcp18_219-ORcp18_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(
 ORcp18_22+(2.0)*qd[2]*S1);
    ACcp18_219 = -(OMcp18_17*(ORcp18_311+ORcp18_319)-OMcp18_37*ORcp18_111-OMcp18_37*ORcp18_119+OPcp18_17*RLcp18_311+
 OPcp18_17*RLcp18_319-OPcp18_37*RLcp18_111-OPcp18_37*RLcp18_119-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp18_12+(2.0)*qd[2]*C1)-qd[1]*(
 ORcp18_13-(2.0)*qd[3]*S1));
    ACcp18_319 = qdd[4]+OMcp18_17*ORcp18_211+OMcp18_17*ORcp18_219-OMcp18_27*ORcp18_111-OMcp18_27*ORcp18_119+OPcp18_17*
 RLcp18_211+OPcp18_17*RLcp18_219-OPcp18_27*RLcp18_111-OPcp18_27*RLcp18_119;

// = = Block_1_0_0_19_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp18_119;
    sens->P[2] = POcp18_219;
    sens->P[3] = POcp18_319;
    sens->R[1][1] = ROcp18_111;
    sens->R[1][2] = ROcp18_211;
    sens->R[1][3] = ROcp18_311;
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


    ROcp19_111 = ROcp19_17*C11-ROcp19_77*S11;
    ROcp19_211 = ROcp19_27*C11-ROcp19_87*S11;
    ROcp19_311 = -S11p7*C6;
    ROcp19_711 = ROcp19_17*S11+ROcp19_77*C11;
    ROcp19_811 = ROcp19_27*S11+ROcp19_87*C11;
    ROcp19_911 = C11p7*C6;
    RLcp19_111 = ROcp19_17*s->dpt[1][5]+ROcp19_77*s->dpt[3][5];
    RLcp19_211 = ROcp19_27*s->dpt[1][5]+ROcp19_87*s->dpt[3][5];
    RLcp19_311 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp19_111 = OMcp19_27*RLcp19_311-OMcp19_37*RLcp19_211;
    ORcp19_211 = -(OMcp19_17*RLcp19_311-OMcp19_37*RLcp19_111);
    ORcp19_311 = OMcp19_17*RLcp19_211-OMcp19_27*RLcp19_111;

// = = Block_1_0_0_20_0_6 = = 
 
// Sensor Kinematics 


    ROcp19_419 = ROcp19_46*C19+ROcp19_711*S19;
    ROcp19_519 = ROcp19_56*C19+ROcp19_811*S19;
    ROcp19_619 = ROcp19_911*S19+C19*S6;
    ROcp19_719 = -(ROcp19_46*S19-ROcp19_711*C19);
    ROcp19_819 = -(ROcp19_56*S19-ROcp19_811*C19);
    ROcp19_919 = ROcp19_911*C19-S19*S6;
    ROcp19_420 = ROcp19_419*C20+ROcp19_719*S20;
    ROcp19_520 = ROcp19_519*C20+ROcp19_819*S20;
    ROcp19_620 = ROcp19_619*C20+ROcp19_919*S20;
    ROcp19_720 = -(ROcp19_419*S20-ROcp19_719*C20);
    ROcp19_820 = -(ROcp19_519*S20-ROcp19_819*C20);
    ROcp19_920 = -(ROcp19_619*S20-ROcp19_919*C20);
    RLcp19_119 = ROcp19_46*s->dpt[2][14]+ROcp19_711*s->dpt[3][14];
    RLcp19_219 = ROcp19_56*s->dpt[2][14]+ROcp19_811*s->dpt[3][14];
    RLcp19_319 = ROcp19_911*s->dpt[3][14]+s->dpt[2][14]*S6;
    OMcp19_119 = OMcp19_17+ROcp19_111*qd[19];
    OMcp19_219 = OMcp19_27+ROcp19_211*qd[19];
    OMcp19_319 = OMcp19_37+ROcp19_311*qd[19];
    ORcp19_119 = OMcp19_27*RLcp19_319-OMcp19_37*RLcp19_219;
    ORcp19_219 = -(OMcp19_17*RLcp19_319-OMcp19_37*RLcp19_119);
    ORcp19_319 = OMcp19_17*RLcp19_219-OMcp19_27*RLcp19_119;
    OPcp19_119 = OPcp19_17+ROcp19_111*qdd[19]+qd[19]*(OMcp19_27*ROcp19_311-OMcp19_37*ROcp19_211);
    OPcp19_219 = OPcp19_27+ROcp19_211*qdd[19]-qd[19]*(OMcp19_17*ROcp19_311-OMcp19_37*ROcp19_111);
    OPcp19_319 = OPcp19_37+ROcp19_311*qdd[19]+qd[19]*(OMcp19_17*ROcp19_211-OMcp19_27*ROcp19_111);
    RLcp19_120 = ROcp19_419*s->dpt[2][25];
    RLcp19_220 = ROcp19_519*s->dpt[2][25];
    RLcp19_320 = ROcp19_619*s->dpt[2][25];
    POcp19_120 = RLcp19_111+RLcp19_119+RLcp19_12+RLcp19_120+RLcp19_13;
    POcp19_220 = RLcp19_211+RLcp19_219+RLcp19_22+RLcp19_220+RLcp19_23;
    POcp19_320 = RLcp19_311+RLcp19_319+RLcp19_320+q[4];
    OMcp19_120 = OMcp19_119+ROcp19_111*qd[20];
    OMcp19_220 = OMcp19_219+ROcp19_211*qd[20];
    OMcp19_320 = OMcp19_319+ROcp19_311*qd[20];
    ORcp19_120 = OMcp19_219*RLcp19_320-OMcp19_319*RLcp19_220;
    ORcp19_220 = -(OMcp19_119*RLcp19_320-OMcp19_319*RLcp19_120);
    ORcp19_320 = OMcp19_119*RLcp19_220-OMcp19_219*RLcp19_120;
    VIcp19_120 = ORcp19_111+ORcp19_119+ORcp19_12+ORcp19_120+ORcp19_13+qd[2]*C1-qd[3]*S1;
    VIcp19_220 = ORcp19_211+ORcp19_219+ORcp19_22+ORcp19_220+ORcp19_23+qd[2]*S1+qd[3]*C1;
    VIcp19_320 = ORcp19_311+ORcp19_319+ORcp19_320+qd[4];
    OPcp19_120 = OPcp19_119+ROcp19_111*qdd[20]+qd[20]*(OMcp19_219*ROcp19_311-OMcp19_319*ROcp19_211);
    OPcp19_220 = OPcp19_219+ROcp19_211*qdd[20]-qd[20]*(OMcp19_119*ROcp19_311-OMcp19_319*ROcp19_111);
    OPcp19_320 = OPcp19_319+ROcp19_311*qdd[20]+qd[20]*(OMcp19_119*ROcp19_211-OMcp19_219*ROcp19_111);
    ACcp19_120 = OMcp19_219*ORcp19_320+OMcp19_27*(ORcp19_311+ORcp19_319)-OMcp19_319*ORcp19_220-OMcp19_37*ORcp19_211-
 OMcp19_37*ORcp19_219+OPcp19_219*RLcp19_320+OPcp19_27*RLcp19_311+OPcp19_27*RLcp19_319-OPcp19_319*RLcp19_220-OPcp19_37*
 RLcp19_211-OPcp19_37*RLcp19_219-ORcp19_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp19_22+(2.0)*qd[2]*S1);
    ACcp19_220 = -(OMcp19_119*ORcp19_320+OMcp19_17*(ORcp19_311+ORcp19_319)-OMcp19_319*ORcp19_120-OMcp19_37*ORcp19_111-
 OMcp19_37*ORcp19_119+OPcp19_119*RLcp19_320+OPcp19_17*RLcp19_311+OPcp19_17*RLcp19_319-OPcp19_319*RLcp19_120-OPcp19_37*
 RLcp19_111-OPcp19_37*RLcp19_119-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp19_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp19_13-(2.0)*qd[3]*S1));
    ACcp19_320 = qdd[4]+OMcp19_119*ORcp19_220+OMcp19_17*ORcp19_211+OMcp19_17*ORcp19_219-OMcp19_219*ORcp19_120-OMcp19_27*
 ORcp19_111-OMcp19_27*ORcp19_119+OPcp19_119*RLcp19_220+OPcp19_17*RLcp19_211+OPcp19_17*RLcp19_219-OPcp19_219*RLcp19_120-
 OPcp19_27*RLcp19_111-OPcp19_27*RLcp19_119;

// = = Block_1_0_0_20_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp19_120;
    sens->P[2] = POcp19_220;
    sens->P[3] = POcp19_320;
    sens->R[1][1] = ROcp19_111;
    sens->R[1][2] = ROcp19_211;
    sens->R[1][3] = ROcp19_311;
    sens->R[2][1] = ROcp19_420;
    sens->R[2][2] = ROcp19_520;
    sens->R[2][3] = ROcp19_620;
    sens->R[3][1] = ROcp19_720;
    sens->R[3][2] = ROcp19_820;
    sens->R[3][3] = ROcp19_920;
    sens->V[1] = VIcp19_120;
    sens->V[2] = VIcp19_220;
    sens->V[3] = VIcp19_320;
    sens->OM[1] = OMcp19_120;
    sens->OM[2] = OMcp19_220;
    sens->OM[3] = OMcp19_320;
    sens->A[1] = ACcp19_120;
    sens->A[2] = ACcp19_220;
    sens->A[3] = ACcp19_320;
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


    ROcp20_111 = ROcp20_17*C11-ROcp20_77*S11;
    ROcp20_211 = ROcp20_27*C11-ROcp20_87*S11;
    ROcp20_311 = -S11p7*C6;
    ROcp20_711 = ROcp20_17*S11+ROcp20_77*C11;
    ROcp20_811 = ROcp20_27*S11+ROcp20_87*C11;
    ROcp20_911 = C11p7*C6;
    RLcp20_111 = ROcp20_17*s->dpt[1][5]+ROcp20_77*s->dpt[3][5];
    RLcp20_211 = ROcp20_27*s->dpt[1][5]+ROcp20_87*s->dpt[3][5];
    RLcp20_311 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp20_111 = OMcp20_27*RLcp20_311-OMcp20_37*RLcp20_211;
    ORcp20_211 = -(OMcp20_17*RLcp20_311-OMcp20_37*RLcp20_111);
    ORcp20_311 = OMcp20_17*RLcp20_211-OMcp20_27*RLcp20_111;

// = = Block_1_0_0_21_0_6 = = 
 
// Sensor Kinematics 


    ROcp20_419 = ROcp20_46*C19+ROcp20_711*S19;
    ROcp20_519 = ROcp20_56*C19+ROcp20_811*S19;
    ROcp20_619 = ROcp20_911*S19+C19*S6;
    ROcp20_719 = -(ROcp20_46*S19-ROcp20_711*C19);
    ROcp20_819 = -(ROcp20_56*S19-ROcp20_811*C19);
    ROcp20_919 = ROcp20_911*C19-S19*S6;
    ROcp20_420 = ROcp20_419*C20+ROcp20_719*S20;
    ROcp20_520 = ROcp20_519*C20+ROcp20_819*S20;
    ROcp20_620 = ROcp20_619*C20+ROcp20_919*S20;
    ROcp20_720 = -(ROcp20_419*S20-ROcp20_719*C20);
    ROcp20_820 = -(ROcp20_519*S20-ROcp20_819*C20);
    ROcp20_920 = -(ROcp20_619*S20-ROcp20_919*C20);
    ROcp20_121 = ROcp20_111*C21-ROcp20_720*S21;
    ROcp20_221 = ROcp20_211*C21-ROcp20_820*S21;
    ROcp20_321 = ROcp20_311*C21-ROcp20_920*S21;
    ROcp20_721 = ROcp20_111*S21+ROcp20_720*C21;
    ROcp20_821 = ROcp20_211*S21+ROcp20_820*C21;
    ROcp20_921 = ROcp20_311*S21+ROcp20_920*C21;
    RLcp20_119 = ROcp20_46*s->dpt[2][14]+ROcp20_711*s->dpt[3][14];
    RLcp20_219 = ROcp20_56*s->dpt[2][14]+ROcp20_811*s->dpt[3][14];
    RLcp20_319 = ROcp20_911*s->dpt[3][14]+s->dpt[2][14]*S6;
    OMcp20_119 = OMcp20_17+ROcp20_111*qd[19];
    OMcp20_219 = OMcp20_27+ROcp20_211*qd[19];
    OMcp20_319 = OMcp20_37+ROcp20_311*qd[19];
    ORcp20_119 = OMcp20_27*RLcp20_319-OMcp20_37*RLcp20_219;
    ORcp20_219 = -(OMcp20_17*RLcp20_319-OMcp20_37*RLcp20_119);
    ORcp20_319 = OMcp20_17*RLcp20_219-OMcp20_27*RLcp20_119;
    OPcp20_119 = OPcp20_17+ROcp20_111*qdd[19]+qd[19]*(OMcp20_27*ROcp20_311-OMcp20_37*ROcp20_211);
    OPcp20_219 = OPcp20_27+ROcp20_211*qdd[19]-qd[19]*(OMcp20_17*ROcp20_311-OMcp20_37*ROcp20_111);
    OPcp20_319 = OPcp20_37+ROcp20_311*qdd[19]+qd[19]*(OMcp20_17*ROcp20_211-OMcp20_27*ROcp20_111);
    RLcp20_120 = ROcp20_419*s->dpt[2][25];
    RLcp20_220 = ROcp20_519*s->dpt[2][25];
    RLcp20_320 = ROcp20_619*s->dpt[2][25];
    POcp20_120 = RLcp20_111+RLcp20_119+RLcp20_12+RLcp20_120+RLcp20_13;
    POcp20_220 = RLcp20_211+RLcp20_219+RLcp20_22+RLcp20_220+RLcp20_23;
    POcp20_320 = RLcp20_311+RLcp20_319+RLcp20_320+q[4];
    OMcp20_120 = OMcp20_119+ROcp20_111*qd[20];
    OMcp20_220 = OMcp20_219+ROcp20_211*qd[20];
    OMcp20_320 = OMcp20_319+ROcp20_311*qd[20];
    ORcp20_120 = OMcp20_219*RLcp20_320-OMcp20_319*RLcp20_220;
    ORcp20_220 = -(OMcp20_119*RLcp20_320-OMcp20_319*RLcp20_120);
    ORcp20_320 = OMcp20_119*RLcp20_220-OMcp20_219*RLcp20_120;
    VIcp20_120 = ORcp20_111+ORcp20_119+ORcp20_12+ORcp20_120+ORcp20_13+qd[2]*C1-qd[3]*S1;
    VIcp20_220 = ORcp20_211+ORcp20_219+ORcp20_22+ORcp20_220+ORcp20_23+qd[2]*S1+qd[3]*C1;
    VIcp20_320 = ORcp20_311+ORcp20_319+ORcp20_320+qd[4];
    ACcp20_120 = OMcp20_219*ORcp20_320+OMcp20_27*(ORcp20_311+ORcp20_319)-OMcp20_319*ORcp20_220-OMcp20_37*ORcp20_211-
 OMcp20_37*ORcp20_219+OPcp20_219*RLcp20_320+OPcp20_27*RLcp20_311+OPcp20_27*RLcp20_319-OPcp20_319*RLcp20_220-OPcp20_37*
 RLcp20_211-OPcp20_37*RLcp20_219-ORcp20_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp20_22+(2.0)*qd[2]*S1);
    ACcp20_220 = -(OMcp20_119*ORcp20_320+OMcp20_17*(ORcp20_311+ORcp20_319)-OMcp20_319*ORcp20_120-OMcp20_37*ORcp20_111-
 OMcp20_37*ORcp20_119+OPcp20_119*RLcp20_320+OPcp20_17*RLcp20_311+OPcp20_17*RLcp20_319-OPcp20_319*RLcp20_120-OPcp20_37*
 RLcp20_111-OPcp20_37*RLcp20_119-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp20_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp20_13-(2.0)*qd[3]*S1));
    ACcp20_320 = qdd[4]+OMcp20_119*ORcp20_220+OMcp20_17*ORcp20_211+OMcp20_17*ORcp20_219-OMcp20_219*ORcp20_120-OMcp20_27*
 ORcp20_111-OMcp20_27*ORcp20_119+OPcp20_119*RLcp20_220+OPcp20_17*RLcp20_211+OPcp20_17*RLcp20_219-OPcp20_219*RLcp20_120-
 OPcp20_27*RLcp20_111-OPcp20_27*RLcp20_119;
    OMcp20_121 = OMcp20_120+ROcp20_420*qd[21];
    OMcp20_221 = OMcp20_220+ROcp20_520*qd[21];
    OMcp20_321 = OMcp20_320+ROcp20_620*qd[21];
    OPcp20_121 = OPcp20_119+ROcp20_111*qdd[20]+ROcp20_420*qdd[21]+qd[20]*(OMcp20_219*ROcp20_311-OMcp20_319*ROcp20_211)+
 qd[21]*(OMcp20_220*ROcp20_620-OMcp20_320*ROcp20_520);
    OPcp20_221 = OPcp20_219+ROcp20_211*qdd[20]+ROcp20_520*qdd[21]-qd[20]*(OMcp20_119*ROcp20_311-OMcp20_319*ROcp20_111)-
 qd[21]*(OMcp20_120*ROcp20_620-OMcp20_320*ROcp20_420);
    OPcp20_321 = OPcp20_319+ROcp20_311*qdd[20]+ROcp20_620*qdd[21]+qd[20]*(OMcp20_119*ROcp20_211-OMcp20_219*ROcp20_111)+
 qd[21]*(OMcp20_120*ROcp20_520-OMcp20_220*ROcp20_420);

// = = Block_1_0_0_21_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp20_120;
    sens->P[2] = POcp20_220;
    sens->P[3] = POcp20_320;
    sens->R[1][1] = ROcp20_121;
    sens->R[1][2] = ROcp20_221;
    sens->R[1][3] = ROcp20_321;
    sens->R[2][1] = ROcp20_420;
    sens->R[2][2] = ROcp20_520;
    sens->R[2][3] = ROcp20_620;
    sens->R[3][1] = ROcp20_721;
    sens->R[3][2] = ROcp20_821;
    sens->R[3][3] = ROcp20_921;
    sens->V[1] = VIcp20_120;
    sens->V[2] = VIcp20_220;
    sens->V[3] = VIcp20_320;
    sens->OM[1] = OMcp20_121;
    sens->OM[2] = OMcp20_221;
    sens->OM[3] = OMcp20_321;
    sens->A[1] = ACcp20_120;
    sens->A[2] = ACcp20_220;
    sens->A[3] = ACcp20_320;
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


    ROcp21_111 = ROcp21_17*C11-ROcp21_77*S11;
    ROcp21_211 = ROcp21_27*C11-ROcp21_87*S11;
    ROcp21_311 = -S11p7*C6;
    ROcp21_711 = ROcp21_17*S11+ROcp21_77*C11;
    ROcp21_811 = ROcp21_27*S11+ROcp21_87*C11;
    ROcp21_911 = C11p7*C6;
    RLcp21_111 = ROcp21_17*s->dpt[1][5]+ROcp21_77*s->dpt[3][5];
    RLcp21_211 = ROcp21_27*s->dpt[1][5]+ROcp21_87*s->dpt[3][5];
    RLcp21_311 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp21_111 = OMcp21_27*RLcp21_311-OMcp21_37*RLcp21_211;
    ORcp21_211 = -(OMcp21_17*RLcp21_311-OMcp21_37*RLcp21_111);
    ORcp21_311 = OMcp21_17*RLcp21_211-OMcp21_27*RLcp21_111;

// = = Block_1_0_0_22_0_6 = = 
 
// Sensor Kinematics 


    ROcp21_419 = ROcp21_46*C19+ROcp21_711*S19;
    ROcp21_519 = ROcp21_56*C19+ROcp21_811*S19;
    ROcp21_619 = ROcp21_911*S19+C19*S6;
    ROcp21_719 = -(ROcp21_46*S19-ROcp21_711*C19);
    ROcp21_819 = -(ROcp21_56*S19-ROcp21_811*C19);
    ROcp21_919 = ROcp21_911*C19-S19*S6;
    ROcp21_420 = ROcp21_419*C20+ROcp21_719*S20;
    ROcp21_520 = ROcp21_519*C20+ROcp21_819*S20;
    ROcp21_620 = ROcp21_619*C20+ROcp21_919*S20;
    ROcp21_720 = -(ROcp21_419*S20-ROcp21_719*C20);
    ROcp21_820 = -(ROcp21_519*S20-ROcp21_819*C20);
    ROcp21_920 = -(ROcp21_619*S20-ROcp21_919*C20);
    ROcp21_121 = ROcp21_111*C21-ROcp21_720*S21;
    ROcp21_221 = ROcp21_211*C21-ROcp21_820*S21;
    ROcp21_321 = ROcp21_311*C21-ROcp21_920*S21;
    ROcp21_721 = ROcp21_111*S21+ROcp21_720*C21;
    ROcp21_821 = ROcp21_211*S21+ROcp21_820*C21;
    ROcp21_921 = ROcp21_311*S21+ROcp21_920*C21;
    ROcp21_122 = ROcp21_121*C22+ROcp21_420*S22;
    ROcp21_222 = ROcp21_221*C22+ROcp21_520*S22;
    ROcp21_322 = ROcp21_321*C22+ROcp21_620*S22;
    ROcp21_422 = -(ROcp21_121*S22-ROcp21_420*C22);
    ROcp21_522 = -(ROcp21_221*S22-ROcp21_520*C22);
    ROcp21_622 = -(ROcp21_321*S22-ROcp21_620*C22);
    RLcp21_119 = ROcp21_46*s->dpt[2][14]+ROcp21_711*s->dpt[3][14];
    RLcp21_219 = ROcp21_56*s->dpt[2][14]+ROcp21_811*s->dpt[3][14];
    RLcp21_319 = ROcp21_911*s->dpt[3][14]+s->dpt[2][14]*S6;
    OMcp21_119 = OMcp21_17+ROcp21_111*qd[19];
    OMcp21_219 = OMcp21_27+ROcp21_211*qd[19];
    OMcp21_319 = OMcp21_37+ROcp21_311*qd[19];
    ORcp21_119 = OMcp21_27*RLcp21_319-OMcp21_37*RLcp21_219;
    ORcp21_219 = -(OMcp21_17*RLcp21_319-OMcp21_37*RLcp21_119);
    ORcp21_319 = OMcp21_17*RLcp21_219-OMcp21_27*RLcp21_119;
    OPcp21_119 = OPcp21_17+ROcp21_111*qdd[19]+qd[19]*(OMcp21_27*ROcp21_311-OMcp21_37*ROcp21_211);
    OPcp21_219 = OPcp21_27+ROcp21_211*qdd[19]-qd[19]*(OMcp21_17*ROcp21_311-OMcp21_37*ROcp21_111);
    OPcp21_319 = OPcp21_37+ROcp21_311*qdd[19]+qd[19]*(OMcp21_17*ROcp21_211-OMcp21_27*ROcp21_111);
    RLcp21_120 = ROcp21_419*s->dpt[2][25];
    RLcp21_220 = ROcp21_519*s->dpt[2][25];
    RLcp21_320 = ROcp21_619*s->dpt[2][25];
    POcp21_120 = RLcp21_111+RLcp21_119+RLcp21_12+RLcp21_120+RLcp21_13;
    POcp21_220 = RLcp21_211+RLcp21_219+RLcp21_22+RLcp21_220+RLcp21_23;
    POcp21_320 = RLcp21_311+RLcp21_319+RLcp21_320+q[4];
    OMcp21_120 = OMcp21_119+ROcp21_111*qd[20];
    OMcp21_220 = OMcp21_219+ROcp21_211*qd[20];
    OMcp21_320 = OMcp21_319+ROcp21_311*qd[20];
    ORcp21_120 = OMcp21_219*RLcp21_320-OMcp21_319*RLcp21_220;
    ORcp21_220 = -(OMcp21_119*RLcp21_320-OMcp21_319*RLcp21_120);
    ORcp21_320 = OMcp21_119*RLcp21_220-OMcp21_219*RLcp21_120;
    VIcp21_120 = ORcp21_111+ORcp21_119+ORcp21_12+ORcp21_120+ORcp21_13+qd[2]*C1-qd[3]*S1;
    VIcp21_220 = ORcp21_211+ORcp21_219+ORcp21_22+ORcp21_220+ORcp21_23+qd[2]*S1+qd[3]*C1;
    VIcp21_320 = ORcp21_311+ORcp21_319+ORcp21_320+qd[4];
    ACcp21_120 = OMcp21_219*ORcp21_320+OMcp21_27*(ORcp21_311+ORcp21_319)-OMcp21_319*ORcp21_220-OMcp21_37*ORcp21_211-
 OMcp21_37*ORcp21_219+OPcp21_219*RLcp21_320+OPcp21_27*RLcp21_311+OPcp21_27*RLcp21_319-OPcp21_319*RLcp21_220-OPcp21_37*
 RLcp21_211-OPcp21_37*RLcp21_219-ORcp21_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp21_22+(2.0)*qd[2]*S1);
    ACcp21_220 = -(OMcp21_119*ORcp21_320+OMcp21_17*(ORcp21_311+ORcp21_319)-OMcp21_319*ORcp21_120-OMcp21_37*ORcp21_111-
 OMcp21_37*ORcp21_119+OPcp21_119*RLcp21_320+OPcp21_17*RLcp21_311+OPcp21_17*RLcp21_319-OPcp21_319*RLcp21_120-OPcp21_37*
 RLcp21_111-OPcp21_37*RLcp21_119-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp21_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp21_13-(2.0)*qd[3]*S1));
    ACcp21_320 = qdd[4]+OMcp21_119*ORcp21_220+OMcp21_17*ORcp21_211+OMcp21_17*ORcp21_219-OMcp21_219*ORcp21_120-OMcp21_27*
 ORcp21_111-OMcp21_27*ORcp21_119+OPcp21_119*RLcp21_220+OPcp21_17*RLcp21_211+OPcp21_17*RLcp21_219-OPcp21_219*RLcp21_120-
 OPcp21_27*RLcp21_111-OPcp21_27*RLcp21_119;
    OMcp21_121 = OMcp21_120+ROcp21_420*qd[21];
    OMcp21_221 = OMcp21_220+ROcp21_520*qd[21];
    OMcp21_321 = OMcp21_320+ROcp21_620*qd[21];
    OMcp21_122 = OMcp21_121+ROcp21_721*qd[22];
    OMcp21_222 = OMcp21_221+ROcp21_821*qd[22];
    OMcp21_322 = OMcp21_321+ROcp21_921*qd[22];
    OPcp21_122 = OPcp21_119+ROcp21_111*qdd[20]+ROcp21_420*qdd[21]+ROcp21_721*qdd[22]+qd[20]*(OMcp21_219*ROcp21_311-
 OMcp21_319*ROcp21_211)+qd[21]*(OMcp21_220*ROcp21_620-OMcp21_320*ROcp21_520)+qd[22]*(OMcp21_221*ROcp21_921-OMcp21_321*
 ROcp21_821);
    OPcp21_222 = OPcp21_219+ROcp21_211*qdd[20]+ROcp21_520*qdd[21]+ROcp21_821*qdd[22]-qd[20]*(OMcp21_119*ROcp21_311-
 OMcp21_319*ROcp21_111)-qd[21]*(OMcp21_120*ROcp21_620-OMcp21_320*ROcp21_420)-qd[22]*(OMcp21_121*ROcp21_921-OMcp21_321*
 ROcp21_721);
    OPcp21_322 = OPcp21_319+ROcp21_311*qdd[20]+ROcp21_620*qdd[21]+ROcp21_921*qdd[22]+qd[20]*(OMcp21_119*ROcp21_211-
 OMcp21_219*ROcp21_111)+qd[21]*(OMcp21_120*ROcp21_520-OMcp21_220*ROcp21_420)+qd[22]*(OMcp21_121*ROcp21_821-OMcp21_221*
 ROcp21_721);

// = = Block_1_0_0_22_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp21_120;
    sens->P[2] = POcp21_220;
    sens->P[3] = POcp21_320;
    sens->R[1][1] = ROcp21_122;
    sens->R[1][2] = ROcp21_222;
    sens->R[1][3] = ROcp21_322;
    sens->R[2][1] = ROcp21_422;
    sens->R[2][2] = ROcp21_522;
    sens->R[2][3] = ROcp21_622;
    sens->R[3][1] = ROcp21_721;
    sens->R[3][2] = ROcp21_821;
    sens->R[3][3] = ROcp21_921;
    sens->V[1] = VIcp21_120;
    sens->V[2] = VIcp21_220;
    sens->V[3] = VIcp21_320;
    sens->OM[1] = OMcp21_122;
    sens->OM[2] = OMcp21_222;
    sens->OM[3] = OMcp21_322;
    sens->A[1] = ACcp21_120;
    sens->A[2] = ACcp21_220;
    sens->A[3] = ACcp21_320;
    sens->OMP[1] = OPcp21_122;
    sens->OMP[2] = OPcp21_222;
    sens->OMP[3] = OPcp21_322;
 
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


    ROcp22_111 = ROcp22_17*C11-ROcp22_77*S11;
    ROcp22_211 = ROcp22_27*C11-ROcp22_87*S11;
    ROcp22_311 = -S11p7*C6;
    ROcp22_711 = ROcp22_17*S11+ROcp22_77*C11;
    ROcp22_811 = ROcp22_27*S11+ROcp22_87*C11;
    ROcp22_911 = C11p7*C6;
    RLcp22_111 = ROcp22_17*s->dpt[1][5]+ROcp22_77*s->dpt[3][5];
    RLcp22_211 = ROcp22_27*s->dpt[1][5]+ROcp22_87*s->dpt[3][5];
    RLcp22_311 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp22_111 = OMcp22_27*RLcp22_311-OMcp22_37*RLcp22_211;
    ORcp22_211 = -(OMcp22_17*RLcp22_311-OMcp22_37*RLcp22_111);
    ORcp22_311 = OMcp22_17*RLcp22_211-OMcp22_27*RLcp22_111;

// = = Block_1_0_0_23_0_6 = = 
 
// Sensor Kinematics 


    ROcp22_419 = ROcp22_46*C19+ROcp22_711*S19;
    ROcp22_519 = ROcp22_56*C19+ROcp22_811*S19;
    ROcp22_619 = ROcp22_911*S19+C19*S6;
    ROcp22_719 = -(ROcp22_46*S19-ROcp22_711*C19);
    ROcp22_819 = -(ROcp22_56*S19-ROcp22_811*C19);
    ROcp22_919 = ROcp22_911*C19-S19*S6;
    ROcp22_420 = ROcp22_419*C20+ROcp22_719*S20;
    ROcp22_520 = ROcp22_519*C20+ROcp22_819*S20;
    ROcp22_620 = ROcp22_619*C20+ROcp22_919*S20;
    ROcp22_720 = -(ROcp22_419*S20-ROcp22_719*C20);
    ROcp22_820 = -(ROcp22_519*S20-ROcp22_819*C20);
    ROcp22_920 = -(ROcp22_619*S20-ROcp22_919*C20);
    ROcp22_121 = ROcp22_111*C21-ROcp22_720*S21;
    ROcp22_221 = ROcp22_211*C21-ROcp22_820*S21;
    ROcp22_321 = ROcp22_311*C21-ROcp22_920*S21;
    ROcp22_721 = ROcp22_111*S21+ROcp22_720*C21;
    ROcp22_821 = ROcp22_211*S21+ROcp22_820*C21;
    ROcp22_921 = ROcp22_311*S21+ROcp22_920*C21;
    ROcp22_122 = ROcp22_121*C22+ROcp22_420*S22;
    ROcp22_222 = ROcp22_221*C22+ROcp22_520*S22;
    ROcp22_322 = ROcp22_321*C22+ROcp22_620*S22;
    ROcp22_422 = -(ROcp22_121*S22-ROcp22_420*C22);
    ROcp22_522 = -(ROcp22_221*S22-ROcp22_520*C22);
    ROcp22_622 = -(ROcp22_321*S22-ROcp22_620*C22);
    ROcp22_423 = ROcp22_422*C23+ROcp22_721*S23;
    ROcp22_523 = ROcp22_522*C23+ROcp22_821*S23;
    ROcp22_623 = ROcp22_622*C23+ROcp22_921*S23;
    ROcp22_723 = -(ROcp22_422*S23-ROcp22_721*C23);
    ROcp22_823 = -(ROcp22_522*S23-ROcp22_821*C23);
    ROcp22_923 = -(ROcp22_622*S23-ROcp22_921*C23);
    RLcp22_119 = ROcp22_46*s->dpt[2][14]+ROcp22_711*s->dpt[3][14];
    RLcp22_219 = ROcp22_56*s->dpt[2][14]+ROcp22_811*s->dpt[3][14];
    RLcp22_319 = ROcp22_911*s->dpt[3][14]+s->dpt[2][14]*S6;
    OMcp22_119 = OMcp22_17+ROcp22_111*qd[19];
    OMcp22_219 = OMcp22_27+ROcp22_211*qd[19];
    OMcp22_319 = OMcp22_37+ROcp22_311*qd[19];
    ORcp22_119 = OMcp22_27*RLcp22_319-OMcp22_37*RLcp22_219;
    ORcp22_219 = -(OMcp22_17*RLcp22_319-OMcp22_37*RLcp22_119);
    ORcp22_319 = OMcp22_17*RLcp22_219-OMcp22_27*RLcp22_119;
    OPcp22_119 = OPcp22_17+ROcp22_111*qdd[19]+qd[19]*(OMcp22_27*ROcp22_311-OMcp22_37*ROcp22_211);
    OPcp22_219 = OPcp22_27+ROcp22_211*qdd[19]-qd[19]*(OMcp22_17*ROcp22_311-OMcp22_37*ROcp22_111);
    OPcp22_319 = OPcp22_37+ROcp22_311*qdd[19]+qd[19]*(OMcp22_17*ROcp22_211-OMcp22_27*ROcp22_111);
    RLcp22_120 = ROcp22_419*s->dpt[2][25];
    RLcp22_220 = ROcp22_519*s->dpt[2][25];
    RLcp22_320 = ROcp22_619*s->dpt[2][25];
    OMcp22_120 = OMcp22_119+ROcp22_111*qd[20];
    OMcp22_220 = OMcp22_219+ROcp22_211*qd[20];
    OMcp22_320 = OMcp22_319+ROcp22_311*qd[20];
    ORcp22_120 = OMcp22_219*RLcp22_320-OMcp22_319*RLcp22_220;
    ORcp22_220 = -(OMcp22_119*RLcp22_320-OMcp22_319*RLcp22_120);
    ORcp22_320 = OMcp22_119*RLcp22_220-OMcp22_219*RLcp22_120;
    OMcp22_121 = OMcp22_120+ROcp22_420*qd[21];
    OMcp22_221 = OMcp22_220+ROcp22_520*qd[21];
    OMcp22_321 = OMcp22_320+ROcp22_620*qd[21];
    OMcp22_122 = OMcp22_121+ROcp22_721*qd[22];
    OMcp22_222 = OMcp22_221+ROcp22_821*qd[22];
    OMcp22_322 = OMcp22_321+ROcp22_921*qd[22];
    OPcp22_122 = OPcp22_119+ROcp22_111*qdd[20]+ROcp22_420*qdd[21]+ROcp22_721*qdd[22]+qd[20]*(OMcp22_219*ROcp22_311-
 OMcp22_319*ROcp22_211)+qd[21]*(OMcp22_220*ROcp22_620-OMcp22_320*ROcp22_520)+qd[22]*(OMcp22_221*ROcp22_921-OMcp22_321*
 ROcp22_821);
    OPcp22_222 = OPcp22_219+ROcp22_211*qdd[20]+ROcp22_520*qdd[21]+ROcp22_821*qdd[22]-qd[20]*(OMcp22_119*ROcp22_311-
 OMcp22_319*ROcp22_111)-qd[21]*(OMcp22_120*ROcp22_620-OMcp22_320*ROcp22_420)-qd[22]*(OMcp22_121*ROcp22_921-OMcp22_321*
 ROcp22_721);
    OPcp22_322 = OPcp22_319+ROcp22_311*qdd[20]+ROcp22_620*qdd[21]+ROcp22_921*qdd[22]+qd[20]*(OMcp22_119*ROcp22_211-
 OMcp22_219*ROcp22_111)+qd[21]*(OMcp22_120*ROcp22_520-OMcp22_220*ROcp22_420)+qd[22]*(OMcp22_121*ROcp22_821-OMcp22_221*
 ROcp22_721);
    RLcp22_123 = ROcp22_122*s->dpt[1][29]+ROcp22_422*s->dpt[2][29]+ROcp22_721*s->dpt[3][29];
    RLcp22_223 = ROcp22_222*s->dpt[1][29]+ROcp22_522*s->dpt[2][29]+ROcp22_821*s->dpt[3][29];
    RLcp22_323 = ROcp22_322*s->dpt[1][29]+ROcp22_622*s->dpt[2][29]+ROcp22_921*s->dpt[3][29];
    POcp22_123 = RLcp22_111+RLcp22_119+RLcp22_12+RLcp22_120+RLcp22_123+RLcp22_13;
    POcp22_223 = RLcp22_211+RLcp22_219+RLcp22_22+RLcp22_220+RLcp22_223+RLcp22_23;
    POcp22_323 = RLcp22_311+RLcp22_319+RLcp22_320+RLcp22_323+q[4];
    ORcp22_123 = OMcp22_222*RLcp22_323-OMcp22_322*RLcp22_223;
    ORcp22_223 = -(OMcp22_122*RLcp22_323-OMcp22_322*RLcp22_123);
    ORcp22_323 = OMcp22_122*RLcp22_223-OMcp22_222*RLcp22_123;
    VIcp22_123 = ORcp22_111+ORcp22_119+ORcp22_12+ORcp22_120+ORcp22_123+ORcp22_13+qd[2]*C1-qd[3]*S1;
    VIcp22_223 = ORcp22_211+ORcp22_219+ORcp22_22+ORcp22_220+ORcp22_223+ORcp22_23+qd[2]*S1+qd[3]*C1;
    VIcp22_323 = ORcp22_311+ORcp22_319+ORcp22_320+ORcp22_323+qd[4];
    ACcp22_123 = OMcp22_219*ORcp22_320+OMcp22_222*ORcp22_323+OMcp22_27*(ORcp22_311+ORcp22_319)-OMcp22_319*ORcp22_220-
 OMcp22_322*ORcp22_223-OMcp22_37*ORcp22_211-OMcp22_37*ORcp22_219+OPcp22_219*RLcp22_320+OPcp22_222*RLcp22_323+OPcp22_27*
 RLcp22_311+OPcp22_27*RLcp22_319-OPcp22_319*RLcp22_220-OPcp22_322*RLcp22_223-OPcp22_37*RLcp22_211-OPcp22_37*RLcp22_219-
 ORcp22_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp22_22+(2.0)*qd[2]*S1);
    ACcp22_223 = -(OMcp22_119*ORcp22_320+OMcp22_122*ORcp22_323+OMcp22_17*(ORcp22_311+ORcp22_319)-OMcp22_319*ORcp22_120-
 OMcp22_322*ORcp22_123-OMcp22_37*ORcp22_111-OMcp22_37*ORcp22_119+OPcp22_119*RLcp22_320+OPcp22_122*RLcp22_323+OPcp22_17*
 RLcp22_311+OPcp22_17*RLcp22_319-OPcp22_319*RLcp22_120-OPcp22_322*RLcp22_123-OPcp22_37*RLcp22_111-OPcp22_37*RLcp22_119-qdd[2]
 *S1-qdd[3]*C1-qd[1]*(ORcp22_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp22_13-(2.0)*qd[3]*S1));
    ACcp22_323 = qdd[4]+OMcp22_119*ORcp22_220+OMcp22_122*ORcp22_223+OMcp22_17*ORcp22_211+OMcp22_17*ORcp22_219-OMcp22_219*
 ORcp22_120-OMcp22_222*ORcp22_123-OMcp22_27*ORcp22_111-OMcp22_27*ORcp22_119+OPcp22_119*RLcp22_220+OPcp22_122*RLcp22_223+
 OPcp22_17*RLcp22_211+OPcp22_17*RLcp22_219-OPcp22_219*RLcp22_120-OPcp22_222*RLcp22_123-OPcp22_27*RLcp22_111-OPcp22_27*
 RLcp22_119;

// = = Block_1_0_0_23_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp22_123;
    sens->P[2] = POcp22_223;
    sens->P[3] = POcp22_323;
    sens->R[1][1] = ROcp22_122;
    sens->R[1][2] = ROcp22_222;
    sens->R[1][3] = ROcp22_322;
    sens->R[2][1] = ROcp22_423;
    sens->R[2][2] = ROcp22_523;
    sens->R[2][3] = ROcp22_623;
    sens->R[3][1] = ROcp22_723;
    sens->R[3][2] = ROcp22_823;
    sens->R[3][3] = ROcp22_923;
    sens->V[1] = VIcp22_123;
    sens->V[2] = VIcp22_223;
    sens->V[3] = VIcp22_323;
    sens->OM[1] = OMcp22_122;
    sens->OM[2] = OMcp22_222;
    sens->OM[3] = OMcp22_322;
    sens->A[1] = ACcp22_123;
    sens->A[2] = ACcp22_223;
    sens->A[3] = ACcp22_323;
    sens->OMP[1] = OPcp22_122;
    sens->OMP[2] = OPcp22_222;
    sens->OMP[3] = OPcp22_322;
 
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

// = = Block_1_0_0_24_0_3 = = 
 
// Sensor Kinematics 


    ROcp23_111 = ROcp23_17*C11-ROcp23_77*S11;
    ROcp23_211 = ROcp23_27*C11-ROcp23_87*S11;
    ROcp23_311 = -S11p7*C6;
    ROcp23_711 = ROcp23_17*S11+ROcp23_77*C11;
    ROcp23_811 = ROcp23_27*S11+ROcp23_87*C11;
    ROcp23_911 = C11p7*C6;
    RLcp23_111 = ROcp23_17*s->dpt[1][5]+ROcp23_77*s->dpt[3][5];
    RLcp23_211 = ROcp23_27*s->dpt[1][5]+ROcp23_87*s->dpt[3][5];
    RLcp23_311 = -C6*(s->dpt[1][5]*S7-s->dpt[3][5]*C7);
    ORcp23_111 = OMcp23_27*RLcp23_311-OMcp23_37*RLcp23_211;
    ORcp23_211 = -(OMcp23_17*RLcp23_311-OMcp23_37*RLcp23_111);
    ORcp23_311 = OMcp23_17*RLcp23_211-OMcp23_27*RLcp23_111;

// = = Block_1_0_0_24_0_6 = = 
 
// Sensor Kinematics 


    ROcp23_419 = ROcp23_46*C19+ROcp23_711*S19;
    ROcp23_519 = ROcp23_56*C19+ROcp23_811*S19;
    ROcp23_619 = ROcp23_911*S19+C19*S6;
    ROcp23_719 = -(ROcp23_46*S19-ROcp23_711*C19);
    ROcp23_819 = -(ROcp23_56*S19-ROcp23_811*C19);
    ROcp23_919 = ROcp23_911*C19-S19*S6;
    ROcp23_420 = ROcp23_419*C20+ROcp23_719*S20;
    ROcp23_520 = ROcp23_519*C20+ROcp23_819*S20;
    ROcp23_620 = ROcp23_619*C20+ROcp23_919*S20;
    ROcp23_720 = -(ROcp23_419*S20-ROcp23_719*C20);
    ROcp23_820 = -(ROcp23_519*S20-ROcp23_819*C20);
    ROcp23_920 = -(ROcp23_619*S20-ROcp23_919*C20);
    ROcp23_121 = ROcp23_111*C21-ROcp23_720*S21;
    ROcp23_221 = ROcp23_211*C21-ROcp23_820*S21;
    ROcp23_321 = ROcp23_311*C21-ROcp23_920*S21;
    ROcp23_721 = ROcp23_111*S21+ROcp23_720*C21;
    ROcp23_821 = ROcp23_211*S21+ROcp23_820*C21;
    ROcp23_921 = ROcp23_311*S21+ROcp23_920*C21;
    ROcp23_122 = ROcp23_121*C22+ROcp23_420*S22;
    ROcp23_222 = ROcp23_221*C22+ROcp23_520*S22;
    ROcp23_322 = ROcp23_321*C22+ROcp23_620*S22;
    ROcp23_422 = -(ROcp23_121*S22-ROcp23_420*C22);
    ROcp23_522 = -(ROcp23_221*S22-ROcp23_520*C22);
    ROcp23_622 = -(ROcp23_321*S22-ROcp23_620*C22);
    ROcp23_423 = ROcp23_422*C23+ROcp23_721*S23;
    ROcp23_523 = ROcp23_522*C23+ROcp23_821*S23;
    ROcp23_623 = ROcp23_622*C23+ROcp23_921*S23;
    ROcp23_723 = -(ROcp23_422*S23-ROcp23_721*C23);
    ROcp23_823 = -(ROcp23_522*S23-ROcp23_821*C23);
    ROcp23_923 = -(ROcp23_622*S23-ROcp23_921*C23);
    ROcp23_124 = ROcp23_122*C24-ROcp23_723*S24;
    ROcp23_224 = ROcp23_222*C24-ROcp23_823*S24;
    ROcp23_324 = ROcp23_322*C24-ROcp23_923*S24;
    ROcp23_724 = ROcp23_122*S24+ROcp23_723*C24;
    ROcp23_824 = ROcp23_222*S24+ROcp23_823*C24;
    ROcp23_924 = ROcp23_322*S24+ROcp23_923*C24;
    RLcp23_119 = ROcp23_46*s->dpt[2][14]+ROcp23_711*s->dpt[3][14];
    RLcp23_219 = ROcp23_56*s->dpt[2][14]+ROcp23_811*s->dpt[3][14];
    RLcp23_319 = ROcp23_911*s->dpt[3][14]+s->dpt[2][14]*S6;
    OMcp23_119 = OMcp23_17+ROcp23_111*qd[19];
    OMcp23_219 = OMcp23_27+ROcp23_211*qd[19];
    OMcp23_319 = OMcp23_37+ROcp23_311*qd[19];
    ORcp23_119 = OMcp23_27*RLcp23_319-OMcp23_37*RLcp23_219;
    ORcp23_219 = -(OMcp23_17*RLcp23_319-OMcp23_37*RLcp23_119);
    ORcp23_319 = OMcp23_17*RLcp23_219-OMcp23_27*RLcp23_119;
    OPcp23_119 = OPcp23_17+ROcp23_111*qdd[19]+qd[19]*(OMcp23_27*ROcp23_311-OMcp23_37*ROcp23_211);
    OPcp23_219 = OPcp23_27+ROcp23_211*qdd[19]-qd[19]*(OMcp23_17*ROcp23_311-OMcp23_37*ROcp23_111);
    OPcp23_319 = OPcp23_37+ROcp23_311*qdd[19]+qd[19]*(OMcp23_17*ROcp23_211-OMcp23_27*ROcp23_111);
    RLcp23_120 = ROcp23_419*s->dpt[2][25];
    RLcp23_220 = ROcp23_519*s->dpt[2][25];
    RLcp23_320 = ROcp23_619*s->dpt[2][25];
    OMcp23_120 = OMcp23_119+ROcp23_111*qd[20];
    OMcp23_220 = OMcp23_219+ROcp23_211*qd[20];
    OMcp23_320 = OMcp23_319+ROcp23_311*qd[20];
    ORcp23_120 = OMcp23_219*RLcp23_320-OMcp23_319*RLcp23_220;
    ORcp23_220 = -(OMcp23_119*RLcp23_320-OMcp23_319*RLcp23_120);
    ORcp23_320 = OMcp23_119*RLcp23_220-OMcp23_219*RLcp23_120;
    OMcp23_121 = OMcp23_120+ROcp23_420*qd[21];
    OMcp23_221 = OMcp23_220+ROcp23_520*qd[21];
    OMcp23_321 = OMcp23_320+ROcp23_620*qd[21];
    OMcp23_122 = OMcp23_121+ROcp23_721*qd[22];
    OMcp23_222 = OMcp23_221+ROcp23_821*qd[22];
    OMcp23_322 = OMcp23_321+ROcp23_921*qd[22];
    OPcp23_122 = OPcp23_119+ROcp23_111*qdd[20]+ROcp23_420*qdd[21]+ROcp23_721*qdd[22]+qd[20]*(OMcp23_219*ROcp23_311-
 OMcp23_319*ROcp23_211)+qd[21]*(OMcp23_220*ROcp23_620-OMcp23_320*ROcp23_520)+qd[22]*(OMcp23_221*ROcp23_921-OMcp23_321*
 ROcp23_821);
    OPcp23_222 = OPcp23_219+ROcp23_211*qdd[20]+ROcp23_520*qdd[21]+ROcp23_821*qdd[22]-qd[20]*(OMcp23_119*ROcp23_311-
 OMcp23_319*ROcp23_111)-qd[21]*(OMcp23_120*ROcp23_620-OMcp23_320*ROcp23_420)-qd[22]*(OMcp23_121*ROcp23_921-OMcp23_321*
 ROcp23_721);
    OPcp23_322 = OPcp23_319+ROcp23_311*qdd[20]+ROcp23_620*qdd[21]+ROcp23_921*qdd[22]+qd[20]*(OMcp23_119*ROcp23_211-
 OMcp23_219*ROcp23_111)+qd[21]*(OMcp23_120*ROcp23_520-OMcp23_220*ROcp23_420)+qd[22]*(OMcp23_121*ROcp23_821-OMcp23_221*
 ROcp23_721);
    RLcp23_123 = ROcp23_122*s->dpt[1][29]+ROcp23_422*s->dpt[2][29]+ROcp23_721*s->dpt[3][29];
    RLcp23_223 = ROcp23_222*s->dpt[1][29]+ROcp23_522*s->dpt[2][29]+ROcp23_821*s->dpt[3][29];
    RLcp23_323 = ROcp23_322*s->dpt[1][29]+ROcp23_622*s->dpt[2][29]+ROcp23_921*s->dpt[3][29];
    POcp23_123 = RLcp23_111+RLcp23_119+RLcp23_12+RLcp23_120+RLcp23_123+RLcp23_13;
    POcp23_223 = RLcp23_211+RLcp23_219+RLcp23_22+RLcp23_220+RLcp23_223+RLcp23_23;
    POcp23_323 = RLcp23_311+RLcp23_319+RLcp23_320+RLcp23_323+q[4];
    ORcp23_123 = OMcp23_222*RLcp23_323-OMcp23_322*RLcp23_223;
    ORcp23_223 = -(OMcp23_122*RLcp23_323-OMcp23_322*RLcp23_123);
    ORcp23_323 = OMcp23_122*RLcp23_223-OMcp23_222*RLcp23_123;
    VIcp23_123 = ORcp23_111+ORcp23_119+ORcp23_12+ORcp23_120+ORcp23_123+ORcp23_13+qd[2]*C1-qd[3]*S1;
    VIcp23_223 = ORcp23_211+ORcp23_219+ORcp23_22+ORcp23_220+ORcp23_223+ORcp23_23+qd[2]*S1+qd[3]*C1;
    VIcp23_323 = ORcp23_311+ORcp23_319+ORcp23_320+ORcp23_323+qd[4];
    ACcp23_123 = OMcp23_219*ORcp23_320+OMcp23_222*ORcp23_323+OMcp23_27*(ORcp23_311+ORcp23_319)-OMcp23_319*ORcp23_220-
 OMcp23_322*ORcp23_223-OMcp23_37*ORcp23_211-OMcp23_37*ORcp23_219+OPcp23_219*RLcp23_320+OPcp23_222*RLcp23_323+OPcp23_27*
 RLcp23_311+OPcp23_27*RLcp23_319-OPcp23_319*RLcp23_220-OPcp23_322*RLcp23_223-OPcp23_37*RLcp23_211-OPcp23_37*RLcp23_219-
 ORcp23_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp23_22+(2.0)*qd[2]*S1);
    ACcp23_223 = -(OMcp23_119*ORcp23_320+OMcp23_122*ORcp23_323+OMcp23_17*(ORcp23_311+ORcp23_319)-OMcp23_319*ORcp23_120-
 OMcp23_322*ORcp23_123-OMcp23_37*ORcp23_111-OMcp23_37*ORcp23_119+OPcp23_119*RLcp23_320+OPcp23_122*RLcp23_323+OPcp23_17*
 RLcp23_311+OPcp23_17*RLcp23_319-OPcp23_319*RLcp23_120-OPcp23_322*RLcp23_123-OPcp23_37*RLcp23_111-OPcp23_37*RLcp23_119-qdd[2]
 *S1-qdd[3]*C1-qd[1]*(ORcp23_12+(2.0)*qd[2]*C1)-qd[1]*(ORcp23_13-(2.0)*qd[3]*S1));
    ACcp23_323 = qdd[4]+OMcp23_119*ORcp23_220+OMcp23_122*ORcp23_223+OMcp23_17*ORcp23_211+OMcp23_17*ORcp23_219-OMcp23_219*
 ORcp23_120-OMcp23_222*ORcp23_123-OMcp23_27*ORcp23_111-OMcp23_27*ORcp23_119+OPcp23_119*RLcp23_220+OPcp23_122*RLcp23_223+
 OPcp23_17*RLcp23_211+OPcp23_17*RLcp23_219-OPcp23_219*RLcp23_120-OPcp23_222*RLcp23_123-OPcp23_27*RLcp23_111-OPcp23_27*
 RLcp23_119;
    OMcp23_124 = OMcp23_122+ROcp23_423*qd[24];
    OMcp23_224 = OMcp23_222+ROcp23_523*qd[24];
    OMcp23_324 = OMcp23_322+ROcp23_623*qd[24];
    OPcp23_124 = OPcp23_122+ROcp23_423*qdd[24]+qd[24]*(OMcp23_222*ROcp23_623-OMcp23_322*ROcp23_523);
    OPcp23_224 = OPcp23_222+ROcp23_523*qdd[24]-qd[24]*(OMcp23_122*ROcp23_623-OMcp23_322*ROcp23_423);
    OPcp23_324 = OPcp23_322+ROcp23_623*qdd[24]+qd[24]*(OMcp23_122*ROcp23_523-OMcp23_222*ROcp23_423);

// = = Block_1_0_0_24_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp23_123;
    sens->P[2] = POcp23_223;
    sens->P[3] = POcp23_323;
    sens->R[1][1] = ROcp23_124;
    sens->R[1][2] = ROcp23_224;
    sens->R[1][3] = ROcp23_324;
    sens->R[2][1] = ROcp23_423;
    sens->R[2][2] = ROcp23_523;
    sens->R[2][3] = ROcp23_623;
    sens->R[3][1] = ROcp23_724;
    sens->R[3][2] = ROcp23_824;
    sens->R[3][3] = ROcp23_924;
    sens->V[1] = VIcp23_123;
    sens->V[2] = VIcp23_223;
    sens->V[3] = VIcp23_323;
    sens->OM[1] = OMcp23_124;
    sens->OM[2] = OMcp23_224;
    sens->OM[3] = OMcp23_324;
    sens->A[1] = ACcp23_123;
    sens->A[2] = ACcp23_223;
    sens->A[3] = ACcp23_323;
    sens->OMP[1] = OPcp23_124;
    sens->OMP[2] = OPcp23_224;
    sens->OMP[3] = OPcp23_324;
 
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


    ROcp24_125 = ROcp24_17*C25-ROcp24_77*S25;
    ROcp24_225 = ROcp24_27*C25-ROcp24_87*S25;
    ROcp24_325 = -S25p7*C6;
    ROcp24_725 = ROcp24_17*S25+ROcp24_77*C25;
    ROcp24_825 = ROcp24_27*S25+ROcp24_87*C25;
    ROcp24_925 = C25p7*C6;
    RLcp24_125 = ROcp24_17*s->dpt[1][6]+ROcp24_77*s->dpt[3][6];
    RLcp24_225 = ROcp24_27*s->dpt[1][6]+ROcp24_87*s->dpt[3][6];
    RLcp24_325 = -C6*(s->dpt[1][6]*S7-s->dpt[3][6]*C7);
    POcp24_125 = RLcp24_12+RLcp24_125+RLcp24_13;
    POcp24_225 = RLcp24_22+RLcp24_225+RLcp24_23;
    POcp24_325 = RLcp24_325+q[4];
    ORcp24_125 = OMcp24_27*RLcp24_325-OMcp24_37*RLcp24_225;
    ORcp24_225 = -(OMcp24_17*RLcp24_325-OMcp24_37*RLcp24_125);
    ORcp24_325 = OMcp24_17*RLcp24_225-OMcp24_27*RLcp24_125;
    VIcp24_125 = ORcp24_12+ORcp24_125+ORcp24_13+qd[2]*C1-qd[3]*S1;
    VIcp24_225 = ORcp24_22+ORcp24_225+ORcp24_23+qd[2]*S1+qd[3]*C1;
    VIcp24_325 = ORcp24_325+qd[4];
    ACcp24_125 = OMcp24_27*ORcp24_325-OMcp24_37*ORcp24_225+OPcp24_27*RLcp24_325-OPcp24_37*RLcp24_225-ORcp24_23*qd[1]+
 qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp24_22+(2.0)*qd[2]*S1);
    ACcp24_225 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp24_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp24_13-(2.0)*qd[3]*S1)-OMcp24_17*ORcp24_325+
 OMcp24_37*ORcp24_125-OPcp24_17*RLcp24_325+OPcp24_37*RLcp24_125;
    ACcp24_325 = qdd[4]+OMcp24_17*ORcp24_225-OMcp24_27*ORcp24_125+OPcp24_17*RLcp24_225-OPcp24_27*RLcp24_125;

// = = Block_1_0_0_25_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp24_125;
    sens->P[2] = POcp24_225;
    sens->P[3] = POcp24_325;
    sens->R[1][1] = ROcp24_125;
    sens->R[1][2] = ROcp24_225;
    sens->R[1][3] = ROcp24_325;
    sens->R[2][1] = ROcp24_46;
    sens->R[2][2] = ROcp24_56;
    sens->R[2][3] = S6;
    sens->R[3][1] = ROcp24_725;
    sens->R[3][2] = ROcp24_825;
    sens->R[3][3] = ROcp24_925;
    sens->V[1] = VIcp24_125;
    sens->V[2] = VIcp24_225;
    sens->V[3] = VIcp24_325;
    sens->OM[1] = OMcp24_17;
    sens->OM[2] = OMcp24_27;
    sens->OM[3] = OMcp24_37;
    sens->A[1] = ACcp24_125;
    sens->A[2] = ACcp24_225;
    sens->A[3] = ACcp24_325;
    sens->OMP[1] = OPcp24_17;
    sens->OMP[2] = OPcp24_27;
    sens->OMP[3] = OPcp24_37;
 
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

// = = Block_1_0_0_26_0_7 = = 
 
// Sensor Kinematics 


    ROcp25_125 = ROcp25_17*C25-ROcp25_77*S25;
    ROcp25_225 = ROcp25_27*C25-ROcp25_87*S25;
    ROcp25_325 = -S25p7*C6;
    ROcp25_725 = ROcp25_17*S25+ROcp25_77*C25;
    ROcp25_825 = ROcp25_27*S25+ROcp25_87*C25;
    ROcp25_925 = C25p7*C6;
    ROcp25_126 = ROcp25_125*C26+ROcp25_46*S26;
    ROcp25_226 = ROcp25_225*C26+ROcp25_56*S26;
    ROcp25_326 = ROcp25_325*C26+S26*S6;
    ROcp25_426 = -(ROcp25_125*S26-ROcp25_46*C26);
    ROcp25_526 = -(ROcp25_225*S26-ROcp25_56*C26);
    ROcp25_626 = -(ROcp25_325*S26-C26*S6);
    RLcp25_125 = ROcp25_17*s->dpt[1][6]+ROcp25_77*s->dpt[3][6];
    RLcp25_225 = ROcp25_27*s->dpt[1][6]+ROcp25_87*s->dpt[3][6];
    RLcp25_325 = -C6*(s->dpt[1][6]*S7-s->dpt[3][6]*C7);
    POcp25_125 = RLcp25_12+RLcp25_125+RLcp25_13;
    POcp25_225 = RLcp25_22+RLcp25_225+RLcp25_23;
    POcp25_325 = RLcp25_325+q[4];
    ORcp25_125 = OMcp25_27*RLcp25_325-OMcp25_37*RLcp25_225;
    ORcp25_225 = -(OMcp25_17*RLcp25_325-OMcp25_37*RLcp25_125);
    ORcp25_325 = OMcp25_17*RLcp25_225-OMcp25_27*RLcp25_125;
    VIcp25_125 = ORcp25_12+ORcp25_125+ORcp25_13+qd[2]*C1-qd[3]*S1;
    VIcp25_225 = ORcp25_22+ORcp25_225+ORcp25_23+qd[2]*S1+qd[3]*C1;
    VIcp25_325 = ORcp25_325+qd[4];
    ACcp25_125 = OMcp25_27*ORcp25_325-OMcp25_37*ORcp25_225+OPcp25_27*RLcp25_325-OPcp25_37*RLcp25_225-ORcp25_23*qd[1]+
 qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp25_22+(2.0)*qd[2]*S1);
    ACcp25_225 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp25_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp25_13-(2.0)*qd[3]*S1)-OMcp25_17*ORcp25_325+
 OMcp25_37*ORcp25_125-OPcp25_17*RLcp25_325+OPcp25_37*RLcp25_125;
    ACcp25_325 = qdd[4]+OMcp25_17*ORcp25_225-OMcp25_27*ORcp25_125+OPcp25_17*RLcp25_225-OPcp25_27*RLcp25_125;
    OMcp25_126 = OMcp25_17+ROcp25_725*qd[26];
    OMcp25_226 = OMcp25_27+ROcp25_825*qd[26];
    OMcp25_326 = OMcp25_37+ROcp25_925*qd[26];
    OPcp25_126 = OPcp25_17+ROcp25_725*qdd[26]+qd[26]*(OMcp25_27*ROcp25_925-OMcp25_37*ROcp25_825);
    OPcp25_226 = OPcp25_27+ROcp25_825*qdd[26]-qd[26]*(OMcp25_17*ROcp25_925-OMcp25_37*ROcp25_725);
    OPcp25_326 = OPcp25_37+ROcp25_925*qdd[26]+qd[26]*(OMcp25_17*ROcp25_825-OMcp25_27*ROcp25_725);

// = = Block_1_0_0_26_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp25_125;
    sens->P[2] = POcp25_225;
    sens->P[3] = POcp25_325;
    sens->R[1][1] = ROcp25_126;
    sens->R[1][2] = ROcp25_226;
    sens->R[1][3] = ROcp25_326;
    sens->R[2][1] = ROcp25_426;
    sens->R[2][2] = ROcp25_526;
    sens->R[2][3] = ROcp25_626;
    sens->R[3][1] = ROcp25_725;
    sens->R[3][2] = ROcp25_825;
    sens->R[3][3] = ROcp25_925;
    sens->V[1] = VIcp25_125;
    sens->V[2] = VIcp25_225;
    sens->V[3] = VIcp25_325;
    sens->OM[1] = OMcp25_126;
    sens->OM[2] = OMcp25_226;
    sens->OM[3] = OMcp25_326;
    sens->A[1] = ACcp25_125;
    sens->A[2] = ACcp25_225;
    sens->A[3] = ACcp25_325;
    sens->OMP[1] = OPcp25_126;
    sens->OMP[2] = OPcp25_226;
    sens->OMP[3] = OPcp25_326;
 
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


    ROcp26_127 = ROcp26_17*C27-ROcp26_77*S27;
    ROcp26_227 = ROcp26_27*C27-ROcp26_87*S27;
    ROcp26_327 = -S27p7*C6;
    ROcp26_727 = ROcp26_17*S27+ROcp26_77*C27;
    ROcp26_827 = ROcp26_27*S27+ROcp26_87*C27;
    ROcp26_927 = C27p7*C6;
    RLcp26_127 = ROcp26_17*s->dpt[1][7]+ROcp26_77*s->dpt[3][7];
    RLcp26_227 = ROcp26_27*s->dpt[1][7]+ROcp26_87*s->dpt[3][7];
    RLcp26_327 = -C6*(s->dpt[1][7]*S7-s->dpt[3][7]*C7);
    POcp26_127 = RLcp26_12+RLcp26_127+RLcp26_13;
    POcp26_227 = RLcp26_22+RLcp26_227+RLcp26_23;
    POcp26_327 = RLcp26_327+q[4];
    ORcp26_127 = OMcp26_27*RLcp26_327-OMcp26_37*RLcp26_227;
    ORcp26_227 = -(OMcp26_17*RLcp26_327-OMcp26_37*RLcp26_127);
    ORcp26_327 = OMcp26_17*RLcp26_227-OMcp26_27*RLcp26_127;
    VIcp26_127 = ORcp26_12+ORcp26_127+ORcp26_13+qd[2]*C1-qd[3]*S1;
    VIcp26_227 = ORcp26_22+ORcp26_227+ORcp26_23+qd[2]*S1+qd[3]*C1;
    VIcp26_327 = ORcp26_327+qd[4];
    ACcp26_127 = OMcp26_27*ORcp26_327-OMcp26_37*ORcp26_227+OPcp26_27*RLcp26_327-OPcp26_37*RLcp26_227-ORcp26_23*qd[1]+
 qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp26_22+(2.0)*qd[2]*S1);
    ACcp26_227 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp26_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp26_13-(2.0)*qd[3]*S1)-OMcp26_17*ORcp26_327+
 OMcp26_37*ORcp26_127-OPcp26_17*RLcp26_327+OPcp26_37*RLcp26_127;
    ACcp26_327 = qdd[4]+OMcp26_17*ORcp26_227-OMcp26_27*ORcp26_127+OPcp26_17*RLcp26_227-OPcp26_27*RLcp26_127;

// = = Block_1_0_0_27_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp26_127;
    sens->P[2] = POcp26_227;
    sens->P[3] = POcp26_327;
    sens->R[1][1] = ROcp26_127;
    sens->R[1][2] = ROcp26_227;
    sens->R[1][3] = ROcp26_327;
    sens->R[2][1] = ROcp26_46;
    sens->R[2][2] = ROcp26_56;
    sens->R[2][3] = S6;
    sens->R[3][1] = ROcp26_727;
    sens->R[3][2] = ROcp26_827;
    sens->R[3][3] = ROcp26_927;
    sens->V[1] = VIcp26_127;
    sens->V[2] = VIcp26_227;
    sens->V[3] = VIcp26_327;
    sens->OM[1] = OMcp26_17;
    sens->OM[2] = OMcp26_27;
    sens->OM[3] = OMcp26_37;
    sens->A[1] = ACcp26_127;
    sens->A[2] = ACcp26_227;
    sens->A[3] = ACcp26_327;
    sens->OMP[1] = OPcp26_17;
    sens->OMP[2] = OPcp26_27;
    sens->OMP[3] = OPcp26_37;
 
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


    ROcp27_127 = ROcp27_17*C27-ROcp27_77*S27;
    ROcp27_227 = ROcp27_27*C27-ROcp27_87*S27;
    ROcp27_327 = -S27p7*C6;
    ROcp27_727 = ROcp27_17*S27+ROcp27_77*C27;
    ROcp27_827 = ROcp27_27*S27+ROcp27_87*C27;
    ROcp27_927 = C27p7*C6;
    RLcp27_127 = ROcp27_17*s->dpt[1][7]+ROcp27_77*s->dpt[3][7];
    RLcp27_227 = ROcp27_27*s->dpt[1][7]+ROcp27_87*s->dpt[3][7];
    RLcp27_327 = -C6*(s->dpt[1][7]*S7-s->dpt[3][7]*C7);
    POcp27_127 = RLcp27_12+RLcp27_127+RLcp27_13;
    POcp27_227 = RLcp27_22+RLcp27_227+RLcp27_23;
    POcp27_327 = RLcp27_327+q[4];
    ORcp27_127 = OMcp27_27*RLcp27_327-OMcp27_37*RLcp27_227;
    ORcp27_227 = -(OMcp27_17*RLcp27_327-OMcp27_37*RLcp27_127);
    ORcp27_327 = OMcp27_17*RLcp27_227-OMcp27_27*RLcp27_127;
    VIcp27_127 = ORcp27_12+ORcp27_127+ORcp27_13+qd[2]*C1-qd[3]*S1;
    VIcp27_227 = ORcp27_22+ORcp27_227+ORcp27_23+qd[2]*S1+qd[3]*C1;
    VIcp27_327 = ORcp27_327+qd[4];
    ACcp27_127 = OMcp27_27*ORcp27_327-OMcp27_37*ORcp27_227+OPcp27_27*RLcp27_327-OPcp27_37*RLcp27_227-ORcp27_23*qd[1]+
 qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp27_22+(2.0)*qd[2]*S1);
    ACcp27_227 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp27_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp27_13-(2.0)*qd[3]*S1)-OMcp27_17*ORcp27_327+
 OMcp27_37*ORcp27_127-OPcp27_17*RLcp27_327+OPcp27_37*RLcp27_127;
    ACcp27_327 = qdd[4]+OMcp27_17*ORcp27_227-OMcp27_27*ORcp27_127+OPcp27_17*RLcp27_227-OPcp27_27*RLcp27_127;

// = = Block_1_0_0_28_0_9 = = 
 
// Sensor Kinematics 


    ROcp27_128 = ROcp27_127*C28+ROcp27_46*S28;
    ROcp27_228 = ROcp27_227*C28+ROcp27_56*S28;
    ROcp27_328 = ROcp27_327*C28+S28*S6;
    ROcp27_428 = -(ROcp27_127*S28-ROcp27_46*C28);
    ROcp27_528 = -(ROcp27_227*S28-ROcp27_56*C28);
    ROcp27_628 = -(ROcp27_327*S28-C28*S6);
    OMcp27_128 = OMcp27_17+ROcp27_727*qd[28];
    OMcp27_228 = OMcp27_27+ROcp27_827*qd[28];
    OMcp27_328 = OMcp27_37+ROcp27_927*qd[28];
    OPcp27_128 = OPcp27_17+ROcp27_727*qdd[28]+qd[28]*(OMcp27_27*ROcp27_927-OMcp27_37*ROcp27_827);
    OPcp27_228 = OPcp27_27+ROcp27_827*qdd[28]-qd[28]*(OMcp27_17*ROcp27_927-OMcp27_37*ROcp27_727);
    OPcp27_328 = OPcp27_37+ROcp27_927*qdd[28]+qd[28]*(OMcp27_17*ROcp27_827-OMcp27_27*ROcp27_727);

// = = Block_1_0_0_28_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp27_127;
    sens->P[2] = POcp27_227;
    sens->P[3] = POcp27_327;
    sens->R[1][1] = ROcp27_128;
    sens->R[1][2] = ROcp27_228;
    sens->R[1][3] = ROcp27_328;
    sens->R[2][1] = ROcp27_428;
    sens->R[2][2] = ROcp27_528;
    sens->R[2][3] = ROcp27_628;
    sens->R[3][1] = ROcp27_727;
    sens->R[3][2] = ROcp27_827;
    sens->R[3][3] = ROcp27_927;
    sens->V[1] = VIcp27_127;
    sens->V[2] = VIcp27_227;
    sens->V[3] = VIcp27_327;
    sens->OM[1] = OMcp27_128;
    sens->OM[2] = OMcp27_228;
    sens->OM[3] = OMcp27_328;
    sens->A[1] = ACcp27_127;
    sens->A[2] = ACcp27_227;
    sens->A[3] = ACcp27_327;
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


    ROcp28_127 = ROcp28_17*C27-ROcp28_77*S27;
    ROcp28_227 = ROcp28_27*C27-ROcp28_87*S27;
    ROcp28_327 = -S27p7*C6;
    ROcp28_727 = ROcp28_17*S27+ROcp28_77*C27;
    ROcp28_827 = ROcp28_27*S27+ROcp28_87*C27;
    ROcp28_927 = C27p7*C6;
    RLcp28_127 = ROcp28_17*s->dpt[1][7]+ROcp28_77*s->dpt[3][7];
    RLcp28_227 = ROcp28_27*s->dpt[1][7]+ROcp28_87*s->dpt[3][7];
    RLcp28_327 = -C6*(s->dpt[1][7]*S7-s->dpt[3][7]*C7);
    POcp28_127 = RLcp28_12+RLcp28_127+RLcp28_13;
    POcp28_227 = RLcp28_22+RLcp28_227+RLcp28_23;
    POcp28_327 = RLcp28_327+q[4];
    ORcp28_127 = OMcp28_27*RLcp28_327-OMcp28_37*RLcp28_227;
    ORcp28_227 = -(OMcp28_17*RLcp28_327-OMcp28_37*RLcp28_127);
    ORcp28_327 = OMcp28_17*RLcp28_227-OMcp28_27*RLcp28_127;
    VIcp28_127 = ORcp28_12+ORcp28_127+ORcp28_13+qd[2]*C1-qd[3]*S1;
    VIcp28_227 = ORcp28_22+ORcp28_227+ORcp28_23+qd[2]*S1+qd[3]*C1;
    VIcp28_327 = ORcp28_327+qd[4];
    ACcp28_127 = OMcp28_27*ORcp28_327-OMcp28_37*ORcp28_227+OPcp28_27*RLcp28_327-OPcp28_37*RLcp28_227-ORcp28_23*qd[1]+
 qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp28_22+(2.0)*qd[2]*S1);
    ACcp28_227 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp28_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp28_13-(2.0)*qd[3]*S1)-OMcp28_17*ORcp28_327+
 OMcp28_37*ORcp28_127-OPcp28_17*RLcp28_327+OPcp28_37*RLcp28_127;
    ACcp28_327 = qdd[4]+OMcp28_17*ORcp28_227-OMcp28_27*ORcp28_127+OPcp28_17*RLcp28_227-OPcp28_27*RLcp28_127;

// = = Block_1_0_0_29_0_9 = = 
 
// Sensor Kinematics 


    ROcp28_128 = ROcp28_127*C28+ROcp28_46*S28;
    ROcp28_228 = ROcp28_227*C28+ROcp28_56*S28;
    ROcp28_328 = ROcp28_327*C28+S28*S6;
    ROcp28_428 = -(ROcp28_127*S28-ROcp28_46*C28);
    ROcp28_528 = -(ROcp28_227*S28-ROcp28_56*C28);
    ROcp28_628 = -(ROcp28_327*S28-C28*S6);
    ROcp28_429 = ROcp28_428*C29+ROcp28_727*S29;
    ROcp28_529 = ROcp28_528*C29+ROcp28_827*S29;
    ROcp28_629 = ROcp28_628*C29+ROcp28_927*S29;
    ROcp28_729 = -(ROcp28_428*S29-ROcp28_727*C29);
    ROcp28_829 = -(ROcp28_528*S29-ROcp28_827*C29);
    ROcp28_929 = -(ROcp28_628*S29-ROcp28_927*C29);
    OMcp28_128 = OMcp28_17+ROcp28_727*qd[28];
    OMcp28_228 = OMcp28_27+ROcp28_827*qd[28];
    OMcp28_328 = OMcp28_37+ROcp28_927*qd[28];
    OMcp28_129 = OMcp28_128+ROcp28_128*qd[29];
    OMcp28_229 = OMcp28_228+ROcp28_228*qd[29];
    OMcp28_329 = OMcp28_328+ROcp28_328*qd[29];
    OPcp28_129 = OPcp28_17+ROcp28_128*qdd[29]+ROcp28_727*qdd[28]+qd[28]*(OMcp28_27*ROcp28_927-OMcp28_37*ROcp28_827)+qd[29]
 *(OMcp28_228*ROcp28_328-OMcp28_328*ROcp28_228);
    OPcp28_229 = OPcp28_27+ROcp28_228*qdd[29]+ROcp28_827*qdd[28]-qd[28]*(OMcp28_17*ROcp28_927-OMcp28_37*ROcp28_727)-qd[29]
 *(OMcp28_128*ROcp28_328-OMcp28_328*ROcp28_128);
    OPcp28_329 = OPcp28_37+ROcp28_328*qdd[29]+ROcp28_927*qdd[28]+qd[28]*(OMcp28_17*ROcp28_827-OMcp28_27*ROcp28_727)+qd[29]
 *(OMcp28_128*ROcp28_228-OMcp28_228*ROcp28_128);

// = = Block_1_0_0_29_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp28_127;
    sens->P[2] = POcp28_227;
    sens->P[3] = POcp28_327;
    sens->R[1][1] = ROcp28_128;
    sens->R[1][2] = ROcp28_228;
    sens->R[1][3] = ROcp28_328;
    sens->R[2][1] = ROcp28_429;
    sens->R[2][2] = ROcp28_529;
    sens->R[2][3] = ROcp28_629;
    sens->R[3][1] = ROcp28_729;
    sens->R[3][2] = ROcp28_829;
    sens->R[3][3] = ROcp28_929;
    sens->V[1] = VIcp28_127;
    sens->V[2] = VIcp28_227;
    sens->V[3] = VIcp28_327;
    sens->OM[1] = OMcp28_129;
    sens->OM[2] = OMcp28_229;
    sens->OM[3] = OMcp28_329;
    sens->A[1] = ACcp28_127;
    sens->A[2] = ACcp28_227;
    sens->A[3] = ACcp28_327;
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


    ROcp29_127 = ROcp29_17*C27-ROcp29_77*S27;
    ROcp29_227 = ROcp29_27*C27-ROcp29_87*S27;
    ROcp29_327 = -S27p7*C6;
    ROcp29_727 = ROcp29_17*S27+ROcp29_77*C27;
    ROcp29_827 = ROcp29_27*S27+ROcp29_87*C27;
    ROcp29_927 = C27p7*C6;
    RLcp29_127 = ROcp29_17*s->dpt[1][7]+ROcp29_77*s->dpt[3][7];
    RLcp29_227 = ROcp29_27*s->dpt[1][7]+ROcp29_87*s->dpt[3][7];
    RLcp29_327 = -C6*(s->dpt[1][7]*S7-s->dpt[3][7]*C7);
    POcp29_127 = RLcp29_12+RLcp29_127+RLcp29_13;
    POcp29_227 = RLcp29_22+RLcp29_227+RLcp29_23;
    POcp29_327 = RLcp29_327+q[4];
    ORcp29_127 = OMcp29_27*RLcp29_327-OMcp29_37*RLcp29_227;
    ORcp29_227 = -(OMcp29_17*RLcp29_327-OMcp29_37*RLcp29_127);
    ORcp29_327 = OMcp29_17*RLcp29_227-OMcp29_27*RLcp29_127;
    VIcp29_127 = ORcp29_12+ORcp29_127+ORcp29_13+qd[2]*C1-qd[3]*S1;
    VIcp29_227 = ORcp29_22+ORcp29_227+ORcp29_23+qd[2]*S1+qd[3]*C1;
    VIcp29_327 = ORcp29_327+qd[4];
    ACcp29_127 = OMcp29_27*ORcp29_327-OMcp29_37*ORcp29_227+OPcp29_27*RLcp29_327-OPcp29_37*RLcp29_227-ORcp29_23*qd[1]+
 qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp29_22+(2.0)*qd[2]*S1);
    ACcp29_227 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp29_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp29_13-(2.0)*qd[3]*S1)-OMcp29_17*ORcp29_327+
 OMcp29_37*ORcp29_127-OPcp29_17*RLcp29_327+OPcp29_37*RLcp29_127;
    ACcp29_327 = qdd[4]+OMcp29_17*ORcp29_227-OMcp29_27*ORcp29_127+OPcp29_17*RLcp29_227-OPcp29_27*RLcp29_127;

// = = Block_1_0_0_30_0_10 = = 
 
// Sensor Kinematics 


    ROcp29_130 = ROcp29_127*C30+ROcp29_46*S30;
    ROcp29_230 = ROcp29_227*C30+ROcp29_56*S30;
    ROcp29_330 = ROcp29_327*C30+S30*S6;
    ROcp29_430 = -(ROcp29_127*S30-ROcp29_46*C30);
    ROcp29_530 = -(ROcp29_227*S30-ROcp29_56*C30);
    ROcp29_630 = -(ROcp29_327*S30-C30*S6);
    OMcp29_130 = OMcp29_17+ROcp29_727*qd[30];
    OMcp29_230 = OMcp29_27+ROcp29_827*qd[30];
    OMcp29_330 = OMcp29_37+ROcp29_927*qd[30];
    OPcp29_130 = OPcp29_17+ROcp29_727*qdd[30]+qd[30]*(OMcp29_27*ROcp29_927-OMcp29_37*ROcp29_827);
    OPcp29_230 = OPcp29_27+ROcp29_827*qdd[30]-qd[30]*(OMcp29_17*ROcp29_927-OMcp29_37*ROcp29_727);
    OPcp29_330 = OPcp29_37+ROcp29_927*qdd[30]+qd[30]*(OMcp29_17*ROcp29_827-OMcp29_27*ROcp29_727);

// = = Block_1_0_0_30_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp29_127;
    sens->P[2] = POcp29_227;
    sens->P[3] = POcp29_327;
    sens->R[1][1] = ROcp29_130;
    sens->R[1][2] = ROcp29_230;
    sens->R[1][3] = ROcp29_330;
    sens->R[2][1] = ROcp29_430;
    sens->R[2][2] = ROcp29_530;
    sens->R[2][3] = ROcp29_630;
    sens->R[3][1] = ROcp29_727;
    sens->R[3][2] = ROcp29_827;
    sens->R[3][3] = ROcp29_927;
    sens->V[1] = VIcp29_127;
    sens->V[2] = VIcp29_227;
    sens->V[3] = VIcp29_327;
    sens->OM[1] = OMcp29_130;
    sens->OM[2] = OMcp29_230;
    sens->OM[3] = OMcp29_330;
    sens->A[1] = ACcp29_127;
    sens->A[2] = ACcp29_227;
    sens->A[3] = ACcp29_327;
    sens->OMP[1] = OPcp29_130;
    sens->OMP[2] = OPcp29_230;
    sens->OMP[3] = OPcp29_330;
 
// 
break;
case 31:
 


// = = Block_1_0_0_31_0_1 = = 
 
// Sensor Kinematics 


    ROcp30_46 = -S1p5*C6;
    ROcp30_56 = C1p5*C6;
    ROcp30_76 = S1p5*S6;
    ROcp30_86 = -C1p5*S6;
    ROcp30_17 = -(ROcp30_76*S7-C1p5*C7);
    ROcp30_27 = -(ROcp30_86*S7-S1p5*C7);
    ROcp30_77 = ROcp30_76*C7+C1p5*S7;
    ROcp30_87 = ROcp30_86*C7+S1p5*S7;
    RLcp30_12 = q[2]*C1;
    RLcp30_22 = q[2]*S1;
    ORcp30_12 = -RLcp30_22*qd[1];
    ORcp30_22 = RLcp30_12*qd[1];
    RLcp30_13 = -q[3]*S1;
    RLcp30_23 = q[3]*C1;
    ORcp30_13 = -RLcp30_23*qd[1];
    ORcp30_23 = RLcp30_13*qd[1];
    OMcp30_35 = qd[1]+qd[5];
    OMcp30_16 = qd[6]*C1p5;
    OMcp30_26 = qd[6]*S1p5;
    OMcp30_17 = OMcp30_16+ROcp30_46*qd[7];
    OMcp30_27 = OMcp30_26+ROcp30_56*qd[7];
    OMcp30_37 = OMcp30_35+qd[7]*S6;
    OPcp30_17 = -(OMcp30_35*qd[6]*S1p5-ROcp30_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp30_26*S6-OMcp30_35*ROcp30_56));
    OPcp30_27 = OMcp30_35*qd[6]*C1p5+ROcp30_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp30_16*S6-OMcp30_35*ROcp30_46);
    OPcp30_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;

// = = Block_1_0_0_31_0_8 = = 
 
// Sensor Kinematics 


    ROcp30_127 = ROcp30_17*C27-ROcp30_77*S27;
    ROcp30_227 = ROcp30_27*C27-ROcp30_87*S27;
    ROcp30_327 = -S27p7*C6;
    ROcp30_727 = ROcp30_17*S27+ROcp30_77*C27;
    ROcp30_827 = ROcp30_27*S27+ROcp30_87*C27;
    ROcp30_927 = C27p7*C6;
    RLcp30_127 = ROcp30_17*s->dpt[1][7]+ROcp30_77*s->dpt[3][7];
    RLcp30_227 = ROcp30_27*s->dpt[1][7]+ROcp30_87*s->dpt[3][7];
    RLcp30_327 = -C6*(s->dpt[1][7]*S7-s->dpt[3][7]*C7);
    POcp30_127 = RLcp30_12+RLcp30_127+RLcp30_13;
    POcp30_227 = RLcp30_22+RLcp30_227+RLcp30_23;
    POcp30_327 = RLcp30_327+q[4];
    ORcp30_127 = OMcp30_27*RLcp30_327-OMcp30_37*RLcp30_227;
    ORcp30_227 = -(OMcp30_17*RLcp30_327-OMcp30_37*RLcp30_127);
    ORcp30_327 = OMcp30_17*RLcp30_227-OMcp30_27*RLcp30_127;
    VIcp30_127 = ORcp30_12+ORcp30_127+ORcp30_13+qd[2]*C1-qd[3]*S1;
    VIcp30_227 = ORcp30_22+ORcp30_227+ORcp30_23+qd[2]*S1+qd[3]*C1;
    VIcp30_327 = ORcp30_327+qd[4];
    ACcp30_127 = OMcp30_27*ORcp30_327-OMcp30_37*ORcp30_227+OPcp30_27*RLcp30_327-OPcp30_37*RLcp30_227-ORcp30_23*qd[1]+
 qdd[2]*C1-qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp30_22+(2.0)*qd[2]*S1);
    ACcp30_227 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp30_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp30_13-(2.0)*qd[3]*S1)-OMcp30_17*ORcp30_327+
 OMcp30_37*ORcp30_127-OPcp30_17*RLcp30_327+OPcp30_37*RLcp30_127;
    ACcp30_327 = qdd[4]+OMcp30_17*ORcp30_227-OMcp30_27*ORcp30_127+OPcp30_17*RLcp30_227-OPcp30_27*RLcp30_127;

// = = Block_1_0_0_31_0_10 = = 
 
// Sensor Kinematics 


    ROcp30_130 = ROcp30_127*C30+ROcp30_46*S30;
    ROcp30_230 = ROcp30_227*C30+ROcp30_56*S30;
    ROcp30_330 = ROcp30_327*C30+S30*S6;
    ROcp30_430 = -(ROcp30_127*S30-ROcp30_46*C30);
    ROcp30_530 = -(ROcp30_227*S30-ROcp30_56*C30);
    ROcp30_630 = -(ROcp30_327*S30-C30*S6);
    ROcp30_431 = ROcp30_430*C31+ROcp30_727*S31;
    ROcp30_531 = ROcp30_530*C31+ROcp30_827*S31;
    ROcp30_631 = ROcp30_630*C31+ROcp30_927*S31;
    ROcp30_731 = -(ROcp30_430*S31-ROcp30_727*C31);
    ROcp30_831 = -(ROcp30_530*S31-ROcp30_827*C31);
    ROcp30_931 = -(ROcp30_630*S31-ROcp30_927*C31);
    OMcp30_130 = OMcp30_17+ROcp30_727*qd[30];
    OMcp30_230 = OMcp30_27+ROcp30_827*qd[30];
    OMcp30_330 = OMcp30_37+ROcp30_927*qd[30];
    OMcp30_131 = OMcp30_130+ROcp30_130*qd[31];
    OMcp30_231 = OMcp30_230+ROcp30_230*qd[31];
    OMcp30_331 = OMcp30_330+ROcp30_330*qd[31];
    OPcp30_131 = OPcp30_17+ROcp30_130*qdd[31]+ROcp30_727*qdd[30]+qd[30]*(OMcp30_27*ROcp30_927-OMcp30_37*ROcp30_827)+qd[31]
 *(OMcp30_230*ROcp30_330-OMcp30_330*ROcp30_230);
    OPcp30_231 = OPcp30_27+ROcp30_230*qdd[31]+ROcp30_827*qdd[30]-qd[30]*(OMcp30_17*ROcp30_927-OMcp30_37*ROcp30_727)-qd[31]
 *(OMcp30_130*ROcp30_330-OMcp30_330*ROcp30_130);
    OPcp30_331 = OPcp30_37+ROcp30_330*qdd[31]+ROcp30_927*qdd[30]+qd[30]*(OMcp30_17*ROcp30_827-OMcp30_27*ROcp30_727)+qd[31]
 *(OMcp30_130*ROcp30_230-OMcp30_230*ROcp30_130);

// = = Block_1_0_0_31_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp30_127;
    sens->P[2] = POcp30_227;
    sens->P[3] = POcp30_327;
    sens->R[1][1] = ROcp30_130;
    sens->R[1][2] = ROcp30_230;
    sens->R[1][3] = ROcp30_330;
    sens->R[2][1] = ROcp30_431;
    sens->R[2][2] = ROcp30_531;
    sens->R[2][3] = ROcp30_631;
    sens->R[3][1] = ROcp30_731;
    sens->R[3][2] = ROcp30_831;
    sens->R[3][3] = ROcp30_931;
    sens->V[1] = VIcp30_127;
    sens->V[2] = VIcp30_227;
    sens->V[3] = VIcp30_327;
    sens->OM[1] = OMcp30_131;
    sens->OM[2] = OMcp30_231;
    sens->OM[3] = OMcp30_331;
    sens->A[1] = ACcp30_127;
    sens->A[2] = ACcp30_227;
    sens->A[3] = ACcp30_327;
    sens->OMP[1] = OPcp30_131;
    sens->OMP[2] = OPcp30_231;
    sens->OMP[3] = OPcp30_331;

break;
default:
break;
}


// ====== END Task 1 ====== 


}
 


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
//	==> Generation Date : Mon Oct  3 17:48:57 2016
//
//	==> Project name : tricycle
//	==> using XML input file 
//
//	==> Number of joints : 30
//
//	==> Function : F 6 : Sensors Kinematical Informations (sens) 
//	==> Flops complexity : 2325
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

// = = Block_0_0_0_2_0_3 = = 
 
// Trigonometric Variables  

  S10p7 = C10*S7+S10*C7;
  C10p7 = C10*C7-S10*S7;

// = = Block_0_0_0_3_0_2 = = 
 
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
    ORcp0_13 = -RLcp0_23*qd[1];
    ORcp0_23 = RLcp0_13*qd[1];
    OMcp0_35 = qd[1]+qd[5];
    OMcp0_16 = qd[6]*C1p5;
    OMcp0_26 = qd[6]*S1p5;
    OMcp0_17 = OMcp0_16+ROcp0_46*qd[7];
    OMcp0_27 = OMcp0_26+ROcp0_56*qd[7];
    OMcp0_37 = OMcp0_35+qd[7]*S6;
    OPcp0_17 = -(OMcp0_35*qd[6]*S1p5-ROcp0_46*qdd[7]-qdd[6]*C1p5-qd[7]*(OMcp0_26*S6-OMcp0_35*ROcp0_56));
    OPcp0_27 = OMcp0_35*qd[6]*C1p5+ROcp0_56*qdd[7]+qdd[6]*S1p5-qd[7]*(OMcp0_16*S6-OMcp0_35*ROcp0_46);
    OPcp0_37 = qdd[5]+qdd[7]*S6+qd[6]*qd[7]*C6;
    RLcp0_145 = ROcp0_17*s->dpt[1][3]+ROcp0_77*s->dpt[3][3];
    RLcp0_245 = ROcp0_27*s->dpt[1][3]+ROcp0_87*s->dpt[3][3];
    RLcp0_345 = ROcp0_37*s->dpt[1][3]+ROcp0_97*s->dpt[3][3];
    POcp0_145 = RLcp0_12+RLcp0_13+RLcp0_145;
    POcp0_245 = RLcp0_22+RLcp0_23+RLcp0_245;
    POcp0_345 = RLcp0_345+q[4];
    JTcp0_145_1 = -(RLcp0_22+RLcp0_23+RLcp0_245);
    JTcp0_245_1 = RLcp0_12+RLcp0_13+RLcp0_145;
    JTcp0_145_6 = RLcp0_345*S1p5;
    JTcp0_245_6 = -RLcp0_345*C1p5;
    JTcp0_345_6 = -(RLcp0_145*S1p5-RLcp0_245*C1p5);
    JTcp0_145_7 = -(RLcp0_245*S6-RLcp0_345*ROcp0_56);
    JTcp0_245_7 = RLcp0_145*S6-RLcp0_345*ROcp0_46;
    JTcp0_345_7 = -(RLcp0_145*ROcp0_56-RLcp0_245*ROcp0_46);
    ORcp0_145 = OMcp0_27*RLcp0_345-OMcp0_37*RLcp0_245;
    ORcp0_245 = -(OMcp0_17*RLcp0_345-OMcp0_37*RLcp0_145);
    ORcp0_345 = OMcp0_17*RLcp0_245-OMcp0_27*RLcp0_145;
    VIcp0_145 = ORcp0_12+ORcp0_13+ORcp0_145+qd[2]*C1-qd[3]*S1;
    VIcp0_245 = ORcp0_22+ORcp0_23+ORcp0_245+qd[2]*S1+qd[3]*C1;
    VIcp0_345 = ORcp0_345+qd[4];
    ACcp0_145 = OMcp0_27*ORcp0_345-OMcp0_37*ORcp0_245+OPcp0_27*RLcp0_345-OPcp0_37*RLcp0_245-ORcp0_23*qd[1]+qdd[2]*C1-
 qdd[3]*S1-(2.0)*qd[1]*qd[3]*C1-qd[1]*(ORcp0_22+(2.0)*qd[2]*S1);
    ACcp0_245 = qdd[2]*S1+qdd[3]*C1+qd[1]*(ORcp0_12+(2.0)*qd[2]*C1)+qd[1]*(ORcp0_13-(2.0)*qd[3]*S1)-OMcp0_17*ORcp0_345+OMcp0_37*
 ORcp0_145-OPcp0_17*RLcp0_345+OPcp0_37*RLcp0_145;
    ACcp0_345 = qdd[4]+OMcp0_17*ORcp0_245-OMcp0_27*ORcp0_145+OPcp0_17*RLcp0_245-OPcp0_27*RLcp0_145;

// = = Block_1_0_0_1_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp0_145;
    sens->P[2] = POcp0_245;
    sens->P[3] = POcp0_345;
    sens->R[1][1] = ROcp0_17;
    sens->R[1][2] = ROcp0_27;
    sens->R[1][3] = ROcp0_37;
    sens->R[2][1] = ROcp0_46;
    sens->R[2][2] = ROcp0_56;
    sens->R[2][3] = S6;
    sens->R[3][1] = ROcp0_77;
    sens->R[3][2] = ROcp0_87;
    sens->R[3][3] = ROcp0_97;
    sens->V[1] = VIcp0_145;
    sens->V[2] = VIcp0_245;
    sens->V[3] = VIcp0_345;
    sens->OM[1] = OMcp0_17;
    sens->OM[2] = OMcp0_27;
    sens->OM[3] = OMcp0_37;
    sens->J[1][1] = JTcp0_145_1;
    sens->J[1][2] = C1;
    sens->J[1][3] = -S1;
    sens->J[1][5] = -RLcp0_245;
    sens->J[1][6] = JTcp0_145_6;
    sens->J[1][7] = JTcp0_145_7;
    sens->J[2][1] = JTcp0_245_1;
    sens->J[2][2] = S1;
    sens->J[2][3] = C1;
    sens->J[2][5] = RLcp0_145;
    sens->J[2][6] = JTcp0_245_6;
    sens->J[2][7] = JTcp0_245_7;
    sens->J[3][4] = (1.0);
    sens->J[3][6] = JTcp0_345_6;
    sens->J[3][7] = JTcp0_345_7;
    sens->J[4][6] = C1p5;
    sens->J[4][7] = ROcp0_46;
    sens->J[5][6] = S1p5;
    sens->J[5][7] = ROcp0_56;
    sens->J[6][1] = (1.0);
    sens->J[6][5] = (1.0);
    sens->J[6][7] = S6;
    sens->A[1] = ACcp0_145;
    sens->A[2] = ACcp0_245;
    sens->A[3] = ACcp0_345;
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
    ROcp1_77 = ROcp1_76*C7+C1p5*S7;
    ROcp1_87 = ROcp1_86*C7+S1p5*S7;
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

// = = Block_1_0_0_2_0_3 = = 
 
// Sensor Kinematics 


    ROcp1_110 = ROcp1_17*C10-ROcp1_77*S10;
    ROcp1_210 = ROcp1_27*C10-ROcp1_87*S10;
    ROcp1_310 = -S10p7*C6;
    ROcp1_710 = ROcp1_17*S10+ROcp1_77*C10;
    ROcp1_810 = ROcp1_27*S10+ROcp1_87*C10;
    ROcp1_910 = C10p7*C6;
    RLcp1_110 = ROcp1_17*s->dpt[1][4]+ROcp1_77*s->dpt[3][4];
    RLcp1_210 = ROcp1_27*s->dpt[1][4]+ROcp1_87*s->dpt[3][4];
    RLcp1_310 = -C6*(s->dpt[1][4]*S7-s->dpt[3][4]*C7);
    ORcp1_110 = OMcp1_27*RLcp1_310-OMcp1_37*RLcp1_210;
    ORcp1_210 = -(OMcp1_17*RLcp1_310-OMcp1_37*RLcp1_110);
    ORcp1_310 = OMcp1_17*RLcp1_210-OMcp1_27*RLcp1_110;

// = = Block_1_0_0_2_0_6 = = 
 
// Sensor Kinematics 


    ROcp1_418 = ROcp1_46*C18+ROcp1_710*S18;
    ROcp1_518 = ROcp1_56*C18+ROcp1_810*S18;
    ROcp1_618 = ROcp1_910*S18+C18*S6;
    ROcp1_718 = -(ROcp1_46*S18-ROcp1_710*C18);
    ROcp1_818 = -(ROcp1_56*S18-ROcp1_810*C18);
    ROcp1_918 = ROcp1_910*C18-S18*S6;
    ROcp1_419 = ROcp1_418*C19+ROcp1_718*S19;
    ROcp1_519 = ROcp1_518*C19+ROcp1_818*S19;
    ROcp1_619 = ROcp1_618*C19+ROcp1_918*S19;
    ROcp1_719 = -(ROcp1_418*S19-ROcp1_718*C19);
    ROcp1_819 = -(ROcp1_518*S19-ROcp1_818*C19);
    ROcp1_919 = -(ROcp1_618*S19-ROcp1_918*C19);
    ROcp1_120 = ROcp1_110*C20-ROcp1_719*S20;
    ROcp1_220 = ROcp1_210*C20-ROcp1_819*S20;
    ROcp1_320 = ROcp1_310*C20-ROcp1_919*S20;
    ROcp1_720 = ROcp1_110*S20+ROcp1_719*C20;
    ROcp1_820 = ROcp1_210*S20+ROcp1_819*C20;
    ROcp1_920 = ROcp1_310*S20+ROcp1_919*C20;
    ROcp1_121 = ROcp1_120*C21+ROcp1_419*S21;
    ROcp1_221 = ROcp1_220*C21+ROcp1_519*S21;
    ROcp1_321 = ROcp1_320*C21+ROcp1_619*S21;
    ROcp1_421 = -(ROcp1_120*S21-ROcp1_419*C21);
    ROcp1_521 = -(ROcp1_220*S21-ROcp1_519*C21);
    ROcp1_621 = -(ROcp1_320*S21-ROcp1_619*C21);
    RLcp1_118 = ROcp1_46*s->dpt[2][11]+ROcp1_710*s->dpt[3][11];
    RLcp1_218 = ROcp1_56*s->dpt[2][11]+ROcp1_810*s->dpt[3][11];
    RLcp1_318 = ROcp1_910*s->dpt[3][11]+s->dpt[2][11]*S6;
    OMcp1_118 = OMcp1_17+ROcp1_110*qd[18];
    OMcp1_218 = OMcp1_27+ROcp1_210*qd[18];
    OMcp1_318 = OMcp1_37+ROcp1_310*qd[18];
    ORcp1_118 = OMcp1_27*RLcp1_318-OMcp1_37*RLcp1_218;
    ORcp1_218 = -(OMcp1_17*RLcp1_318-OMcp1_37*RLcp1_118);
    ORcp1_318 = OMcp1_17*RLcp1_218-OMcp1_27*RLcp1_118;
    OPcp1_118 = OPcp1_17+ROcp1_110*qdd[18]+qd[18]*(OMcp1_27*ROcp1_310-OMcp1_37*ROcp1_210);
    OPcp1_218 = OPcp1_27+ROcp1_210*qdd[18]-qd[18]*(OMcp1_17*ROcp1_310-OMcp1_37*ROcp1_110);
    OPcp1_318 = OPcp1_37+ROcp1_310*qdd[18]+qd[18]*(OMcp1_17*ROcp1_210-OMcp1_27*ROcp1_110);
    RLcp1_119 = ROcp1_418*s->dpt[2][21];
    RLcp1_219 = ROcp1_518*s->dpt[2][21];
    RLcp1_319 = ROcp1_618*s->dpt[2][21];
    OMcp1_119 = OMcp1_118+ROcp1_110*qd[19];
    OMcp1_219 = OMcp1_218+ROcp1_210*qd[19];
    OMcp1_319 = OMcp1_318+ROcp1_310*qd[19];
    ORcp1_119 = OMcp1_218*RLcp1_319-OMcp1_318*RLcp1_219;
    ORcp1_219 = -(OMcp1_118*RLcp1_319-OMcp1_318*RLcp1_119);
    ORcp1_319 = OMcp1_118*RLcp1_219-OMcp1_218*RLcp1_119;
    OMcp1_120 = OMcp1_119+ROcp1_419*qd[20];
    OMcp1_220 = OMcp1_219+ROcp1_519*qd[20];
    OMcp1_320 = OMcp1_319+ROcp1_619*qd[20];
    OMcp1_121 = OMcp1_120+ROcp1_720*qd[21];
    OMcp1_221 = OMcp1_220+ROcp1_820*qd[21];
    OMcp1_321 = OMcp1_320+ROcp1_920*qd[21];
    OPcp1_121 = OPcp1_118+ROcp1_110*qdd[19]+ROcp1_419*qdd[20]+ROcp1_720*qdd[21]+qd[19]*(OMcp1_218*ROcp1_310-OMcp1_318*
 ROcp1_210)+qd[20]*(OMcp1_219*ROcp1_619-OMcp1_319*ROcp1_519)+qd[21]*(OMcp1_220*ROcp1_920-OMcp1_320*ROcp1_820);
    OPcp1_221 = OPcp1_218+ROcp1_210*qdd[19]+ROcp1_519*qdd[20]+ROcp1_820*qdd[21]-qd[19]*(OMcp1_118*ROcp1_310-OMcp1_318*
 ROcp1_110)-qd[20]*(OMcp1_119*ROcp1_619-OMcp1_319*ROcp1_419)-qd[21]*(OMcp1_120*ROcp1_920-OMcp1_320*ROcp1_720);
    OPcp1_321 = OPcp1_318+ROcp1_310*qdd[19]+ROcp1_619*qdd[20]+ROcp1_920*qdd[21]+qd[19]*(OMcp1_118*ROcp1_210-OMcp1_218*
 ROcp1_110)+qd[20]*(OMcp1_119*ROcp1_519-OMcp1_219*ROcp1_419)+qd[21]*(OMcp1_120*ROcp1_820-OMcp1_220*ROcp1_720);
    RLcp1_146 = ROcp1_121*s->dpt[1][24]+ROcp1_421*s->dpt[2][24]+ROcp1_720*s->dpt[3][24];
    RLcp1_246 = ROcp1_221*s->dpt[1][24]+ROcp1_521*s->dpt[2][24]+ROcp1_820*s->dpt[3][24];
    RLcp1_346 = ROcp1_321*s->dpt[1][24]+ROcp1_621*s->dpt[2][24]+ROcp1_920*s->dpt[3][24];
    POcp1_146 = RLcp1_110+RLcp1_118+RLcp1_119+RLcp1_12+RLcp1_13+RLcp1_146;
    POcp1_246 = RLcp1_210+RLcp1_218+RLcp1_219+RLcp1_22+RLcp1_23+RLcp1_246;
    POcp1_346 = RLcp1_310+RLcp1_318+RLcp1_319+RLcp1_346+q[4];
    JTcp1_146_1 = -(RLcp1_210+RLcp1_218+RLcp1_219+RLcp1_22+RLcp1_23+RLcp1_246);
    JTcp1_246_1 = RLcp1_110+RLcp1_118+RLcp1_119+RLcp1_12+RLcp1_13+RLcp1_146;
    JTcp1_146_5 = -(RLcp1_210+RLcp1_218+RLcp1_219+RLcp1_246);
    JTcp1_246_5 = RLcp1_110+RLcp1_118+RLcp1_119+RLcp1_146;
    JTcp1_146_6 = S1p5*(RLcp1_310+RLcp1_318+RLcp1_319+RLcp1_346);
    JTcp1_246_6 = -C1p5*(RLcp1_310+RLcp1_318+RLcp1_319+RLcp1_346);
    JTcp1_346_6 = C1p5*(RLcp1_210+RLcp1_218+RLcp1_219+RLcp1_246)-S1p5*(RLcp1_110+RLcp1_118)-S1p5*(RLcp1_119+RLcp1_146);
    JTcp1_146_7 = ROcp1_56*(RLcp1_310+RLcp1_318+RLcp1_319+RLcp1_346)-S6*(RLcp1_210+RLcp1_218)-S6*(RLcp1_219+RLcp1_246);
    JTcp1_246_7 = -(ROcp1_46*(RLcp1_310+RLcp1_318+RLcp1_319+RLcp1_346)-S6*(RLcp1_110+RLcp1_118)-S6*(RLcp1_119+RLcp1_146));
    JTcp1_346_7 = ROcp1_46*(RLcp1_210+RLcp1_218+RLcp1_219+RLcp1_246)-ROcp1_56*(RLcp1_110+RLcp1_118)-ROcp1_56*(RLcp1_119+
 RLcp1_146);
    JTcp1_146_8 = ROcp1_56*(RLcp1_318+RLcp1_319)-S6*(RLcp1_218+RLcp1_219)-RLcp1_246*S6+RLcp1_346*ROcp1_56;
    JTcp1_246_8 = RLcp1_146*S6-RLcp1_346*ROcp1_46-ROcp1_46*(RLcp1_318+RLcp1_319)+S6*(RLcp1_118+RLcp1_119);
    JTcp1_346_8 = ROcp1_46*(RLcp1_218+RLcp1_219)-ROcp1_56*(RLcp1_118+RLcp1_119)-RLcp1_146*ROcp1_56+RLcp1_246*ROcp1_46;
    JTcp1_146_9 = ROcp1_210*(RLcp1_319+RLcp1_346)-ROcp1_310*(RLcp1_219+RLcp1_246);
    JTcp1_246_9 = -(ROcp1_110*(RLcp1_319+RLcp1_346)-ROcp1_310*(RLcp1_119+RLcp1_146));
    JTcp1_346_9 = ROcp1_110*(RLcp1_219+RLcp1_246)-ROcp1_210*(RLcp1_119+RLcp1_146);
    JTcp1_146_10 = -(RLcp1_246*ROcp1_310-RLcp1_346*ROcp1_210);
    JTcp1_246_10 = RLcp1_146*ROcp1_310-RLcp1_346*ROcp1_110;
    JTcp1_346_10 = -(RLcp1_146*ROcp1_210-RLcp1_246*ROcp1_110);
    JTcp1_146_11 = -(RLcp1_246*ROcp1_619-RLcp1_346*ROcp1_519);
    JTcp1_246_11 = RLcp1_146*ROcp1_619-RLcp1_346*ROcp1_419;
    JTcp1_346_11 = -(RLcp1_146*ROcp1_519-RLcp1_246*ROcp1_419);
    JTcp1_146_12 = -(RLcp1_246*ROcp1_920-RLcp1_346*ROcp1_820);
    JTcp1_246_12 = RLcp1_146*ROcp1_920-RLcp1_346*ROcp1_720;
    JTcp1_346_12 = -(RLcp1_146*ROcp1_820-RLcp1_246*ROcp1_720);
    ORcp1_146 = OMcp1_221*RLcp1_346-OMcp1_321*RLcp1_246;
    ORcp1_246 = -(OMcp1_121*RLcp1_346-OMcp1_321*RLcp1_146);
    ORcp1_346 = OMcp1_121*RLcp1_246-OMcp1_221*RLcp1_146;
    VIcp1_146 = ORcp1_110+ORcp1_118+ORcp1_119+ORcp1_12+ORcp1_13+ORcp1_146+qd[2]*C1-qd[3]*S1;
    VIcp1_246 = ORcp1_210+ORcp1_218+ORcp1_219+ORcp1_22+ORcp1_23+ORcp1_246+qd[2]*S1+qd[3]*C1;
    VIcp1_346 = ORcp1_310+ORcp1_318+ORcp1_319+ORcp1_346+qd[4];
    ACcp1_146 = OMcp1_218*ORcp1_319+OMcp1_221*ORcp1_346+OMcp1_27*(ORcp1_310+ORcp1_318)-OMcp1_318*ORcp1_219-OMcp1_321*
 ORcp1_246-OMcp1_37*ORcp1_210-OMcp1_37*ORcp1_218+OPcp1_218*RLcp1_319+OPcp1_221*RLcp1_346+OPcp1_27*RLcp1_310+OPcp1_27*
 RLcp1_318-OPcp1_318*RLcp1_219-OPcp1_321*RLcp1_246-OPcp1_37*RLcp1_210-OPcp1_37*RLcp1_218-ORcp1_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*
 qd[1]*qd[3]*C1-qd[1]*(ORcp1_22+(2.0)*qd[2]*S1);
    ACcp1_246 = -(OMcp1_118*ORcp1_319+OMcp1_121*ORcp1_346+OMcp1_17*(ORcp1_310+ORcp1_318)-OMcp1_318*ORcp1_119-OMcp1_321*
 ORcp1_146-OMcp1_37*ORcp1_110-OMcp1_37*ORcp1_118+OPcp1_118*RLcp1_319+OPcp1_121*RLcp1_346+OPcp1_17*RLcp1_310+OPcp1_17*
 RLcp1_318-OPcp1_318*RLcp1_119-OPcp1_321*RLcp1_146-OPcp1_37*RLcp1_110-OPcp1_37*RLcp1_118-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp1_12+(2.0)
 *qd[2]*C1)-qd[1]*(ORcp1_13-(2.0)*qd[3]*S1));
    ACcp1_346 = qdd[4]+OMcp1_118*ORcp1_219+OMcp1_121*ORcp1_246+OMcp1_17*ORcp1_210+OMcp1_17*ORcp1_218-OMcp1_218*ORcp1_119-
 OMcp1_221*ORcp1_146-OMcp1_27*ORcp1_110-OMcp1_27*ORcp1_118+OPcp1_118*RLcp1_219+OPcp1_121*RLcp1_246+OPcp1_17*RLcp1_210+
 OPcp1_17*RLcp1_218-OPcp1_218*RLcp1_119-OPcp1_221*RLcp1_146-OPcp1_27*RLcp1_110-OPcp1_27*RLcp1_118;

// = = Block_1_0_0_2_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp1_146;
    sens->P[2] = POcp1_246;
    sens->P[3] = POcp1_346;
    sens->R[1][1] = ROcp1_121;
    sens->R[1][2] = ROcp1_221;
    sens->R[1][3] = ROcp1_321;
    sens->R[2][1] = ROcp1_421;
    sens->R[2][2] = ROcp1_521;
    sens->R[2][3] = ROcp1_621;
    sens->R[3][1] = ROcp1_720;
    sens->R[3][2] = ROcp1_820;
    sens->R[3][3] = ROcp1_920;
    sens->V[1] = VIcp1_146;
    sens->V[2] = VIcp1_246;
    sens->V[3] = VIcp1_346;
    sens->OM[1] = OMcp1_121;
    sens->OM[2] = OMcp1_221;
    sens->OM[3] = OMcp1_321;
    sens->J[1][1] = JTcp1_146_1;
    sens->J[1][2] = C1;
    sens->J[1][3] = -S1;
    sens->J[1][5] = JTcp1_146_5;
    sens->J[1][6] = JTcp1_146_6;
    sens->J[1][7] = JTcp1_146_7;
    sens->J[1][10] = JTcp1_146_8;
    sens->J[1][18] = JTcp1_146_9;
    sens->J[1][19] = JTcp1_146_10;
    sens->J[1][20] = JTcp1_146_11;
    sens->J[1][21] = JTcp1_146_12;
    sens->J[2][1] = JTcp1_246_1;
    sens->J[2][2] = S1;
    sens->J[2][3] = C1;
    sens->J[2][5] = JTcp1_246_5;
    sens->J[2][6] = JTcp1_246_6;
    sens->J[2][7] = JTcp1_246_7;
    sens->J[2][10] = JTcp1_246_8;
    sens->J[2][18] = JTcp1_246_9;
    sens->J[2][19] = JTcp1_246_10;
    sens->J[2][20] = JTcp1_246_11;
    sens->J[2][21] = JTcp1_246_12;
    sens->J[3][4] = (1.0);
    sens->J[3][6] = JTcp1_346_6;
    sens->J[3][7] = JTcp1_346_7;
    sens->J[3][10] = JTcp1_346_8;
    sens->J[3][18] = JTcp1_346_9;
    sens->J[3][19] = JTcp1_346_10;
    sens->J[3][20] = JTcp1_346_11;
    sens->J[3][21] = JTcp1_346_12;
    sens->J[4][6] = C1p5;
    sens->J[4][7] = ROcp1_46;
    sens->J[4][10] = ROcp1_46;
    sens->J[4][18] = ROcp1_110;
    sens->J[4][19] = ROcp1_110;
    sens->J[4][20] = ROcp1_419;
    sens->J[4][21] = ROcp1_720;
    sens->J[5][6] = S1p5;
    sens->J[5][7] = ROcp1_56;
    sens->J[5][10] = ROcp1_56;
    sens->J[5][18] = ROcp1_210;
    sens->J[5][19] = ROcp1_210;
    sens->J[5][20] = ROcp1_519;
    sens->J[5][21] = ROcp1_820;
    sens->J[6][1] = (1.0);
    sens->J[6][5] = (1.0);
    sens->J[6][7] = S6;
    sens->J[6][10] = S6;
    sens->J[6][18] = ROcp1_310;
    sens->J[6][19] = ROcp1_310;
    sens->J[6][20] = ROcp1_619;
    sens->J[6][21] = ROcp1_920;
    sens->A[1] = ACcp1_146;
    sens->A[2] = ACcp1_246;
    sens->A[3] = ACcp1_346;
    sens->OMP[1] = OPcp1_121;
    sens->OMP[2] = OPcp1_221;
    sens->OMP[3] = OPcp1_321;
 
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
    RLcp2_18 = ROcp2_17*s->dpt[1][1]+ROcp2_77*s->dpt[3][1];
    RLcp2_28 = ROcp2_27*s->dpt[1][1]+ROcp2_87*s->dpt[3][1];
    RLcp2_38 = -C6*(s->dpt[1][1]*S7-s->dpt[3][1]*C7);
    OMcp2_18 = OMcp2_17+ROcp2_46*qd[8];
    OMcp2_28 = OMcp2_27+ROcp2_56*qd[8];
    OMcp2_38 = OMcp2_37+qd[8]*S6;
    ORcp2_18 = OMcp2_27*RLcp2_38-OMcp2_37*RLcp2_28;
    ORcp2_28 = -(OMcp2_17*RLcp2_38-OMcp2_37*RLcp2_18);
    ORcp2_38 = OMcp2_17*RLcp2_28-OMcp2_27*RLcp2_18;
    OPcp2_18 = OPcp2_17+ROcp2_46*qdd[8]+qd[8]*(OMcp2_27*S6-OMcp2_37*ROcp2_56);
    OPcp2_28 = OPcp2_27+ROcp2_56*qdd[8]-qd[8]*(OMcp2_17*S6-OMcp2_37*ROcp2_46);
    OPcp2_38 = OPcp2_37+qdd[8]*S6+qd[8]*(OMcp2_17*ROcp2_56-OMcp2_27*ROcp2_46);
    RLcp2_19 = ROcp2_18*s->dpt[1][7];
    RLcp2_29 = ROcp2_28*s->dpt[1][7];
    RLcp2_39 = -s->dpt[1][7]*S7p8*C6;
    POcp2_19 = RLcp2_12+RLcp2_13+RLcp2_18+RLcp2_19;
    POcp2_29 = RLcp2_22+RLcp2_23+RLcp2_28+RLcp2_29;
    POcp2_39 = RLcp2_38+RLcp2_39+q[4];
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
    RLcp3_110 = ROcp3_17*s->dpt[1][4]+ROcp3_77*s->dpt[3][4];
    RLcp3_210 = ROcp3_27*s->dpt[1][4]+ROcp3_87*s->dpt[3][4];
    RLcp3_310 = -C6*(s->dpt[1][4]*S7-s->dpt[3][4]*C7);
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
    OMcp3_112 = OMcp3_17+ROcp3_110*qd[12];
    OMcp3_212 = OMcp3_27+ROcp3_210*qd[12];
    OMcp3_312 = OMcp3_37+ROcp3_310*qd[12];
    ORcp3_112 = OMcp3_27*RLcp3_312-OMcp3_37*RLcp3_212;
    ORcp3_212 = -(OMcp3_17*RLcp3_312-OMcp3_37*RLcp3_112);
    ORcp3_312 = OMcp3_17*RLcp3_212-OMcp3_27*RLcp3_112;
    OPcp3_112 = OPcp3_17+ROcp3_110*qdd[12]+qd[12]*(OMcp3_27*ROcp3_310-OMcp3_37*ROcp3_210);
    OPcp3_212 = OPcp3_27+ROcp3_210*qdd[12]-qd[12]*(OMcp3_17*ROcp3_310-OMcp3_37*ROcp3_110);
    OPcp3_312 = OPcp3_37+ROcp3_310*qdd[12]+qd[12]*(OMcp3_17*ROcp3_210-OMcp3_27*ROcp3_110);
    RLcp3_113 = ROcp3_412*s->dpt[2][15];
    RLcp3_213 = ROcp3_512*s->dpt[2][15];
    RLcp3_313 = ROcp3_612*s->dpt[2][15];
    OMcp3_113 = OMcp3_112+ROcp3_110*qd[13];
    OMcp3_213 = OMcp3_212+ROcp3_210*qd[13];
    OMcp3_313 = OMcp3_312+ROcp3_310*qd[13];
    ORcp3_113 = OMcp3_212*RLcp3_313-OMcp3_312*RLcp3_213;
    ORcp3_213 = -(OMcp3_112*RLcp3_313-OMcp3_312*RLcp3_113);
    ORcp3_313 = OMcp3_112*RLcp3_213-OMcp3_212*RLcp3_113;
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
    RLcp3_116 = ROcp3_115*s->dpt[1][18]+ROcp3_415*s->dpt[2][18]+ROcp3_714*s->dpt[3][18];
    RLcp3_216 = ROcp3_215*s->dpt[1][18]+ROcp3_515*s->dpt[2][18]+ROcp3_814*s->dpt[3][18];
    RLcp3_316 = ROcp3_315*s->dpt[1][18]+ROcp3_615*s->dpt[2][18]+ROcp3_914*s->dpt[3][18];
    POcp3_116 = RLcp3_110+RLcp3_112+RLcp3_113+RLcp3_116+RLcp3_12+RLcp3_13;
    POcp3_216 = RLcp3_210+RLcp3_212+RLcp3_213+RLcp3_216+RLcp3_22+RLcp3_23;
    POcp3_316 = RLcp3_310+RLcp3_312+RLcp3_313+RLcp3_316+q[4];
    ORcp3_116 = OMcp3_215*RLcp3_316-OMcp3_315*RLcp3_216;
    ORcp3_216 = -(OMcp3_115*RLcp3_316-OMcp3_315*RLcp3_116);
    ORcp3_316 = OMcp3_115*RLcp3_216-OMcp3_215*RLcp3_116;
    VIcp3_116 = ORcp3_110+ORcp3_112+ORcp3_113+ORcp3_116+ORcp3_12+ORcp3_13+qd[2]*C1-qd[3]*S1;
    VIcp3_216 = ORcp3_210+ORcp3_212+ORcp3_213+ORcp3_216+ORcp3_22+ORcp3_23+qd[2]*S1+qd[3]*C1;
    VIcp3_316 = ORcp3_310+ORcp3_312+ORcp3_313+ORcp3_316+qd[4];
    ACcp3_116 = OMcp3_212*ORcp3_313+OMcp3_215*ORcp3_316+OMcp3_27*(ORcp3_310+ORcp3_312)-OMcp3_312*ORcp3_213-OMcp3_315*
 ORcp3_216-OMcp3_37*ORcp3_210-OMcp3_37*ORcp3_212+OPcp3_212*RLcp3_313+OPcp3_215*RLcp3_316+OPcp3_27*RLcp3_310+OPcp3_27*
 RLcp3_312-OPcp3_312*RLcp3_213-OPcp3_315*RLcp3_216-OPcp3_37*RLcp3_210-OPcp3_37*RLcp3_212-ORcp3_23*qd[1]+qdd[2]*C1-qdd[3]*S1-(2.0)*
 qd[1]*qd[3]*C1-qd[1]*(ORcp3_22+(2.0)*qd[2]*S1);
    ACcp3_216 = -(OMcp3_112*ORcp3_313+OMcp3_115*ORcp3_316+OMcp3_17*(ORcp3_310+ORcp3_312)-OMcp3_312*ORcp3_113-OMcp3_315*
 ORcp3_116-OMcp3_37*ORcp3_110-OMcp3_37*ORcp3_112+OPcp3_112*RLcp3_313+OPcp3_115*RLcp3_316+OPcp3_17*RLcp3_310+OPcp3_17*
 RLcp3_312-OPcp3_312*RLcp3_113-OPcp3_315*RLcp3_116-OPcp3_37*RLcp3_110-OPcp3_37*RLcp3_112-qdd[2]*S1-qdd[3]*C1-qd[1]*(ORcp3_12+(2.0)
 *qd[2]*C1)-qd[1]*(ORcp3_13-(2.0)*qd[3]*S1));
    ACcp3_316 = qdd[4]+OMcp3_112*ORcp3_213+OMcp3_115*ORcp3_216+OMcp3_17*ORcp3_210+OMcp3_17*ORcp3_212-OMcp3_212*ORcp3_113-
 OMcp3_215*ORcp3_116-OMcp3_27*ORcp3_110-OMcp3_27*ORcp3_112+OPcp3_112*RLcp3_213+OPcp3_115*RLcp3_216+OPcp3_17*RLcp3_210+
 OPcp3_17*RLcp3_212-OPcp3_212*RLcp3_113-OPcp3_215*RLcp3_116-OPcp3_27*RLcp3_110-OPcp3_27*RLcp3_112;
    OMcp3_117 = OMcp3_115+ROcp3_416*qd[17];
    OMcp3_217 = OMcp3_215+ROcp3_516*qd[17];
    OMcp3_317 = OMcp3_315+ROcp3_616*qd[17];
    OPcp3_117 = OPcp3_115+ROcp3_416*qdd[17]+qd[17]*(OMcp3_215*ROcp3_616-OMcp3_315*ROcp3_516);
    OPcp3_217 = OPcp3_215+ROcp3_516*qdd[17]-qd[17]*(OMcp3_115*ROcp3_616-OMcp3_315*ROcp3_416);
    OPcp3_317 = OPcp3_315+ROcp3_616*qdd[17]+qd[17]*(OMcp3_115*ROcp3_516-OMcp3_215*ROcp3_416);

// = = Block_1_0_0_4_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp3_116;
    sens->P[2] = POcp3_216;
    sens->P[3] = POcp3_316;
    sens->R[1][1] = ROcp3_117;
    sens->R[1][2] = ROcp3_217;
    sens->R[1][3] = ROcp3_317;
    sens->R[2][1] = ROcp3_416;
    sens->R[2][2] = ROcp3_516;
    sens->R[2][3] = ROcp3_616;
    sens->R[3][1] = ROcp3_717;
    sens->R[3][2] = ROcp3_817;
    sens->R[3][3] = ROcp3_917;
    sens->V[1] = VIcp3_116;
    sens->V[2] = VIcp3_216;
    sens->V[3] = VIcp3_316;
    sens->OM[1] = OMcp3_117;
    sens->OM[2] = OMcp3_217;
    sens->OM[3] = OMcp3_317;
    sens->A[1] = ACcp3_116;
    sens->A[2] = ACcp3_216;
    sens->A[3] = ACcp3_316;
    sens->OMP[1] = OPcp3_117;
    sens->OMP[2] = OPcp3_217;
    sens->OMP[3] = OPcp3_317;
 
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
    RLcp4_110 = ROcp4_17*s->dpt[1][4]+ROcp4_77*s->dpt[3][4];
    RLcp4_210 = ROcp4_27*s->dpt[1][4]+ROcp4_87*s->dpt[3][4];
    RLcp4_310 = -C6*(s->dpt[1][4]*S7-s->dpt[3][4]*C7);
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
    RLcp4_118 = ROcp4_46*s->dpt[2][11]+ROcp4_710*s->dpt[3][11];
    RLcp4_218 = ROcp4_56*s->dpt[2][11]+ROcp4_810*s->dpt[3][11];
    RLcp4_318 = ROcp4_910*s->dpt[3][11]+s->dpt[2][11]*S6;
    OMcp4_118 = OMcp4_17+ROcp4_110*qd[18];
    OMcp4_218 = OMcp4_27+ROcp4_210*qd[18];
    OMcp4_318 = OMcp4_37+ROcp4_310*qd[18];
    ORcp4_118 = OMcp4_27*RLcp4_318-OMcp4_37*RLcp4_218;
    ORcp4_218 = -(OMcp4_17*RLcp4_318-OMcp4_37*RLcp4_118);
    ORcp4_318 = OMcp4_17*RLcp4_218-OMcp4_27*RLcp4_118;
    OPcp4_118 = OPcp4_17+ROcp4_110*qdd[18]+qd[18]*(OMcp4_27*ROcp4_310-OMcp4_37*ROcp4_210);
    OPcp4_218 = OPcp4_27+ROcp4_210*qdd[18]-qd[18]*(OMcp4_17*ROcp4_310-OMcp4_37*ROcp4_110);
    OPcp4_318 = OPcp4_37+ROcp4_310*qdd[18]+qd[18]*(OMcp4_17*ROcp4_210-OMcp4_27*ROcp4_110);
    RLcp4_119 = ROcp4_418*s->dpt[2][21];
    RLcp4_219 = ROcp4_518*s->dpt[2][21];
    RLcp4_319 = ROcp4_618*s->dpt[2][21];
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
    RLcp4_122 = ROcp4_121*s->dpt[1][24]+ROcp4_421*s->dpt[2][24]+ROcp4_720*s->dpt[3][24];
    RLcp4_222 = ROcp4_221*s->dpt[1][24]+ROcp4_521*s->dpt[2][24]+ROcp4_820*s->dpt[3][24];
    RLcp4_322 = ROcp4_321*s->dpt[1][24]+ROcp4_621*s->dpt[2][24]+ROcp4_920*s->dpt[3][24];
    POcp4_122 = RLcp4_110+RLcp4_118+RLcp4_119+RLcp4_12+RLcp4_122+RLcp4_13;
    POcp4_222 = RLcp4_210+RLcp4_218+RLcp4_219+RLcp4_22+RLcp4_222+RLcp4_23;
    POcp4_322 = RLcp4_310+RLcp4_318+RLcp4_319+RLcp4_322+q[4];
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
    sens->A[1] = ACcp4_122;
    sens->A[2] = ACcp4_222;
    sens->A[3] = ACcp4_322;
    sens->OMP[1] = OPcp4_123;
    sens->OMP[2] = OPcp4_223;
    sens->OMP[3] = OPcp4_323;

break;
default:
break;
}


// ====== END Task 1 ====== 


}
 


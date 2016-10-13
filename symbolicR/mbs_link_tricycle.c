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
//	==> Function : F 7 : Point to point Link Forces (frc,trq,Flnk) 
//	==> Flops complexity : 155
//
//	==> Generation Time :  0.000 seconds
//	==> Post-Processing :  0.010 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
 
void mbs_link(double **frc,double **trq,double *Flnk,double *Z,double *Zd,
MbsData *s, double tsim)

// double frc[3][30];
// double trq[3][30];
// double Flnk[3];
// double Z[3];
// double Zd[3];
{ 
 
#include "mbs_link_tricycle.h" 
#define q s->q 
#define qd s->qd 
#define qdd s->qdd 
 
 

// === begin imp_aux === 

// === end imp_aux === 

// ===== BEGIN task 0 ===== 

// = = Block_0_0_0_0_0_2 = = 
 
// Trigonometric Variables  

  C8 = cos(q[8]);
  S8 = sin(q[8]);

// = = Block_0_0_0_0_0_4 = = 
 
// Trigonometric Variables  

  C11 = cos(q[11]);
  S11 = sin(q[11]);

// = = Block_0_0_0_0_0_5 = = 
 
// Trigonometric Variables  

  C12 = cos(q[12]);
  S12 = sin(q[12]);

// = = Block_0_0_0_0_0_6 = = 
 
// Trigonometric Variables  

  C18 = cos(q[18]);
  S18 = sin(q[18]);

// = = Block_0_1_0_0_1_4 = = 
 
// Link Kinematics: Distance Z , Relative Velocity ZD 

  RLlnk1_240 = s->dpt[2][13]*C11-s->dpt[3][13]*S11;
  RLlnk1_340 = s->dpt[2][13]*S11+s->dpt[3][13]*C11;

// = = Block_0_1_0_0_1_5 = = 
 
// Link Kinematics: Distance Z , Relative Velocity ZD 

  RLlnk0_239 = s->dpt[2][17]*C12-s->dpt[3][17]*S12;
  RLlnk0_339 = s->dpt[2][17]*S12+s->dpt[3][17]*C12;

// = = Block_0_1_0_0_2_4 = = 
 
// Link Kinematics: Distance Z , Relative Velocity ZD 

  RLlnk3_242 = s->dpt[2][14]*C11-s->dpt[3][14]*S11;
  RLlnk3_342 = s->dpt[2][14]*S11+s->dpt[3][14]*C11;

// = = Block_0_1_0_0_2_6 = = 
 
// Link Kinematics: Distance Z , Relative Velocity ZD 

  RLlnk2_241 = s->dpt[2][23]*C18-s->dpt[3][23]*S18;
  RLlnk2_341 = s->dpt[2][23]*S18+s->dpt[3][23]*C18;

// = = Block_0_1_0_0_3_2 = = 
 
// Link Kinematics: Distance Z , Relative Velocity ZD 

  RLlnk4_143 = s->dpt[1][9]*C8+s->dpt[3][9]*S8;
  RLlnk4_343 = -(s->dpt[1][9]*S8-s->dpt[3][9]*C8);

// = = Block_0_1_0_1_1_4 = = 
 
// Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk21 = -(RLlnk0_239-RLlnk1_240+s->dpt[2][11]);
  Plnk31 = -(RLlnk0_339-RLlnk1_340+s->dpt[3][11]);
  Z1 = sqrt(Plnk21*Plnk21+Plnk31*Plnk31);
  e21 = Plnk21/Z1;
  e31 = Plnk31/Z1;
  Zd1 = -(e21*(qd[11]*RLlnk1_340-qd[12]*RLlnk0_339)-e31*(qd[11]*RLlnk1_240-qd[12]*RLlnk0_239));
 
// Link Force Computation 

  Flink1 = user_LinkForces(Z1,Zd1,s,tsim,1);

// = = Block_0_1_0_1_2_4 = = 
 
// Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk22 = -(RLlnk2_241-RLlnk3_242+s->dpt[2][12]);
  Plnk32 = -(RLlnk2_341-RLlnk3_342+s->dpt[3][12]);
  Z2 = sqrt(Plnk22*Plnk22+Plnk32*Plnk32);
  e22 = Plnk22/Z2;
  e32 = Plnk32/Z2;
  Zd2 = -(e22*(qd[11]*RLlnk3_342-qd[18]*RLlnk2_341)-e32*(qd[11]*RLlnk3_242-qd[18]*RLlnk2_241));
 
// Link Force Computation 

  Flink2 = user_LinkForces(Z2,Zd2,s,tsim,2);

// = = Block_0_1_0_1_3_1 = = 
 
// Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk13 = -(RLlnk4_143+s->dpt[1][2]-s->dpt[1][3]);
  Plnk33 = -(RLlnk4_343+s->dpt[3][2]-s->dpt[3][3]);
  Z3 = sqrt(Plnk13*Plnk13+Plnk33*Plnk33);
  e13 = Plnk13/Z3;
  e33 = Plnk33/Z3;
  Zd3 = qd[8]*(RLlnk4_143*e33-RLlnk4_343*e13);
 
// Link Force Computation 

  Flink3 = user_LinkForces(Z3,Zd3,s,tsim,3);

// = = Block_0_1_0_2_2_4 = = 
 
// Link Dynamics : Forces projection on body-fixed frames 

  fSlnk21 = Flink1*(e21*C11+e31*S11);
  fSlnk31 = -Flink1*(e21*S11-e31*C11);
  s->frc[2][11] = s->frc[2][11]-fSlnk21;
  s->frc[3][11] = s->frc[3][11]-fSlnk31;
  s->trq[1][11] = s->trq[1][11]+fSlnk21*(s->dpt[3][13]-s->l[3][11])-fSlnk31*s->dpt[2][13];

// = = Block_0_1_0_2_2_5 = = 
 
// Link Dynamics : Forces projection on body-fixed frames 

  fPlnk21 = Flink1*(e21*C12+e31*S12);
  fPlnk31 = -Flink1*(e21*S12-e31*C12);
  frc[2][12] = s->frc[2][12]+fPlnk21;
  frc[3][12] = s->frc[3][12]+fPlnk31;
  trq[1][12] = s->trq[1][12]-fPlnk21*s->dpt[3][17]+fPlnk31*s->dpt[2][17];

// = = Block_0_1_0_2_3_4 = = 
 
// Link Dynamics : Forces projection on body-fixed frames 

  fSlnk22 = Flink2*(e22*C11+e32*S11);
  fSlnk32 = -Flink2*(e22*S11-e32*C11);
  frc[2][11] = -(fSlnk22-s->frc[2][11]);
  frc[3][11] = -(fSlnk32-s->frc[3][11]);
  trq[1][11] = s->trq[1][11]+fSlnk22*(s->dpt[3][14]-s->l[3][11])-fSlnk32*s->dpt[2][14];

// = = Block_0_1_0_2_3_6 = = 
 
// Link Dynamics : Forces projection on body-fixed frames 

  fPlnk22 = Flink2*(e22*C18+e32*S18);
  fPlnk32 = -Flink2*(e22*S18-e32*C18);
  frc[2][18] = s->frc[2][18]+fPlnk22;
  frc[3][18] = s->frc[3][18]+fPlnk32;
  trq[1][18] = s->trq[1][18]-fPlnk22*s->dpt[3][23]+fPlnk32*s->dpt[2][23];

// = = Block_0_1_0_2_4_1 = = 
 
// Link Dynamics : Forces projection on body-fixed frames 

  fSlnk13 = Flink3*e13;
  fSlnk33 = Flink3*e33;
  frc[1][7] = s->frc[1][7]-fSlnk13;
  frc[3][7] = s->frc[3][7]-fSlnk33;
  trq[2][7] = s->trq[2][7]-fSlnk13*(s->dpt[3][3]-s->l[3][7])+fSlnk33*(s->dpt[1][3]-s->l[1][7]);

// = = Block_0_1_0_2_4_2 = = 
 
// Link Dynamics : Forces projection on body-fixed frames 

  fPlnk13 = Flink3*(e13*C8-e33*S8);
  fPlnk33 = Flink3*(e13*S8+e33*C8);
  frc[1][8] = s->frc[1][8]+fPlnk13;
  frc[3][8] = s->frc[3][8]+fPlnk33;
  trq[2][8] = s->trq[2][8]+fPlnk13*s->dpt[3][9]-fPlnk33*s->dpt[1][9];

// = = Block_0_2_0_0_0_0 = = 
 
// Symbolic Outputs  

  frc[2][7] = s->frc[2][7];
  frc[2][8] = s->frc[2][8];
  frc[1][9] = s->frc[1][9];
  frc[2][9] = s->frc[2][9];
  frc[3][9] = s->frc[3][9];
  frc[1][10] = s->frc[1][10];
  frc[2][10] = s->frc[2][10];
  frc[3][10] = s->frc[3][10];
  frc[1][11] = s->frc[1][11];
  frc[1][12] = s->frc[1][12];
  frc[1][15] = s->frc[1][15];
  frc[2][15] = s->frc[2][15];
  frc[3][15] = s->frc[3][15];
  frc[1][17] = s->frc[1][17];
  frc[2][17] = s->frc[2][17];
  frc[3][17] = s->frc[3][17];
  frc[1][18] = s->frc[1][18];
  frc[1][21] = s->frc[1][21];
  frc[2][21] = s->frc[2][21];
  frc[3][21] = s->frc[3][21];
  frc[1][23] = s->frc[1][23];
  frc[2][23] = s->frc[2][23];
  frc[3][23] = s->frc[3][23];
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
  trq[3][7] = s->trq[3][7];
  trq[1][8] = s->trq[1][8];
  trq[3][8] = s->trq[3][8];
  trq[1][9] = s->trq[1][9];
  trq[2][9] = s->trq[2][9];
  trq[3][9] = s->trq[3][9];
  trq[1][10] = s->trq[1][10];
  trq[2][10] = s->trq[2][10];
  trq[3][10] = s->trq[3][10];
  trq[2][11] = s->trq[2][11];
  trq[3][11] = s->trq[3][11];
  trq[2][12] = s->trq[2][12];
  trq[3][12] = s->trq[3][12];
  trq[1][15] = s->trq[1][15];
  trq[2][15] = s->trq[2][15];
  trq[3][15] = s->trq[3][15];
  trq[1][17] = s->trq[1][17];
  trq[2][17] = s->trq[2][17];
  trq[3][17] = s->trq[3][17];
  trq[2][18] = s->trq[2][18];
  trq[3][18] = s->trq[3][18];
  trq[1][21] = s->trq[1][21];
  trq[2][21] = s->trq[2][21];
  trq[3][21] = s->trq[3][21];
  trq[1][23] = s->trq[1][23];
  trq[2][23] = s->trq[2][23];
  trq[3][23] = s->trq[3][23];
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
  Flnk[1] = Flink1;
  Flnk[2] = Flink2;
  Flnk[3] = Flink3;
  Z[1] = Z1;
  Z[2] = Z2;
  Z[3] = Z3;
  Zd[1] = Zd1;
  Zd[2] = Zd2;
  Zd[3] = Zd3;

// ====== END Task 0 ====== 


}
 


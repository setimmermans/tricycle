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
//	==> Generation Date : Wed Dec 21 13:36:05 2016
//
//	==> Project name : tricycle
//	==> using XML input file 
//
//	==> Number of joints : 31
//
//	==> Function : F 1 : Direct Dynamics (Semi-Explicit formulation) : RNEA
//	==> Flops complexity : 6810
//
//	==> Generation Time :  0.080 seconds
//	==> Post-Processing :  0.090 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
 
void mbs_dirdyna(double **M,double *c,
MbsData *s, double tsim)

// double M[31][31];
// double c[31];
{ 
 
#include "mbs_dirdyna_tricycle.h" 
#define q s->q 
#define qd s->qd 
#define qdd s->qdd 
 
 

// === begin imp_aux === 

// === end imp_aux === 

// ===== BEGIN task 0 ===== 

// = = Block_0_0_0_0_0_1 = = 
 
// Trigonometric Variables  

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

// = = Block_0_1_0_0_0_1 = = 
 
// Forward Kinematics 

  AlF13 = -qd[1]*((2.0)*qd[3]+q[2]*qd[1]);
  AlF23 = qd[1]*((2.0)*qd[2]-q[3]*qd[1]);
  OM35 = qd[1]+qd[5];
  AlF15 = AlF13*C5+AlF23*S5;
  AlF25 = -(AlF13*S5-AlF23*C5);
  AlM15_1 = q[2]*S5-q[3]*C5;
  AlM25_1 = q[2]*C5+q[3]*S5;
  OM36 = OM35*C6;
  OpF26 = qd[6]*OM35*C6;
  OpF36 = -qd[6]*OM35*S6;
  AlF26 = AlF25*C6-s->g[3]*S6;
  AlF36 = -(AlF25*S6+s->g[3]*C6);
  AlM26_1 = AlM25_1*C6;
  AlM36_1 = -AlM25_1*S6;
  AlM26_2 = -S5*C6;
  AlM36_2 = S5*S6;
  AlM26_3 = C5*C6;
  AlM36_3 = -C5*S6;
  OM17 = qd[6]*C7-OM36*S7;
  OM27 = qd[7]+OM35*S6;
  OM37 = qd[6]*S7+OM36*C7;
  OpF17 = -(qd[7]*OM36*C7+S7*(OpF36+qd[6]*qd[7]));
  OpF37 = -(qd[7]*OM36*S7-C7*(OpF36+qd[6]*qd[7]));
  BS17 = -(OM27*OM27+OM37*OM37);
  BS37 = OM17*OM37;
  BS97 = -(OM17*OM17+OM27*OM27);
  BeF37 = BS37+OpF26;
  BeF47 = OpF37+OM17*OM27;
  BeF67 = OM27*OM37-OpF17;
  BeF77 = BS37-OpF26;
  AlF17 = AlF15*C7-AlF36*S7;
  AlF37 = AlF15*S7+AlF36*C7;
  OpM17_1 = -C6*S7;
  OpM37_1 = C6*C7;
  AlM17_1 = AlM15_1*C7-AlM36_1*S7;
  AlM37_1 = AlM15_1*S7+AlM36_1*C7;
  AlM17_2 = -(AlM36_2*S7-C5*C7);
  AlM37_2 = AlM36_2*C7+C5*S7;
  AlM17_3 = -(AlM36_3*S7-S5*C7);
  AlM37_3 = AlM36_3*C7+S5*S7;
  OpM17_5 = -C6*S7;
  OpM37_5 = C6*C7;

// = = Block_0_1_0_1_0_2 = = 
 
// Trigonometric Variables  

  S7p8 = C7*S8+S7*C8;
  C7p8 = C7*C8-S7*S8;
  S7p8p9 = C7p8*S9+S7p8*C9;
  C7p8p9 = C7p8*C9-S7p8*S9;
 
// Forward Kinematics 

  OM18 = OM17*C8-OM37*S8;
  OM28 = qd[8]+OM27;
  OM38 = OM17*S8+OM37*C8;
  OpF18 = C8*(OpF17-qd[8]*OM37)-S8*(OpF37+qd[8]*OM17);
  OpF38 = C8*(OpF37+qd[8]*OM17)+S8*(OpF17-qd[8]*OM37);
  BS18 = -(OM28*OM28+OM38*OM38);
  BeF78 = OM18*OM38-OpF26;
  AlF18 = C8*(AlF17+BS17*s->dpt[1][2]+BeF37*s->dpt[3][2])-S8*(AlF37+BS97*s->dpt[3][2]+BeF77*s->dpt[1][2]);
  AlF38 = C8*(AlF37+BS97*s->dpt[3][2]+BeF77*s->dpt[1][2])+S8*(AlF17+BS17*s->dpt[1][2]+BeF37*s->dpt[3][2]);
  AlM18_1 = C8*(AlM17_1+s->dpt[3][2]*S6)-S8*(AlM37_1-s->dpt[1][2]*S6);
  AlM38_1 = C8*(AlM37_1-s->dpt[1][2]*S6)+S8*(AlM17_1+s->dpt[3][2]*S6);
  AlM18_2 = AlM17_2*C8-AlM37_2*S8;
  AlM38_2 = AlM17_2*S8+AlM37_2*C8;
  AlM18_3 = AlM17_3*C8-AlM37_3*S8;
  AlM38_3 = AlM17_3*S8+AlM37_3*C8;
  AlM18_5 = S6*(s->dpt[1][2]*S8+s->dpt[3][2]*C8);
  AlM38_5 = -S6*(s->dpt[1][2]*C8-s->dpt[3][2]*S8);
  AlM18_7 = s->dpt[1][2]*S8+s->dpt[3][2]*C8;
  AlM38_7 = -(s->dpt[1][2]*C8-s->dpt[3][2]*S8);
  OM19 = OM18*C9-OM38*S9;
  OM29 = qd[9]+OM28;
  OM39 = OM18*S9+OM38*C9;

// = = Block_0_1_0_1_0_3 = = 
 
// Trigonometric Variables  

  S11p7 = C11*S7+S11*C7;
  C11p7 = C11*C7-S11*S7;
 
// Forward Kinematics 

  OM111 = OM17*C11-OM37*S11;
  OM311 = OM17*S11+OM37*C11;
  OpF111 = OpF17*C11-OpF37*S11;
  OpF311 = OpF17*S11+OpF37*C11;
  BS511 = -(OM111*OM111+OM311*OM311);
  BS611 = OM27*OM311;
  BS911 = -(OM111*OM111+OM27*OM27);
  BeF211 = OM111*OM27-OpF311;
  BeF311 = OpF26+OM111*OM311;
  BeF611 = BS611-OpF111;
  BeF811 = BS611+OpF111;
  AlF111 = C11*(AlF17+BS17*s->dpt[1][5]+BeF37*s->dpt[3][5])-S11*(AlF37+BS97*s->dpt[3][5]+BeF77*s->dpt[1][5]);
  AlF211 = AlF26+BeF47*s->dpt[1][5]+BeF67*s->dpt[3][5];
  AlF311 = C11*(AlF37+BS97*s->dpt[3][5]+BeF77*s->dpt[1][5])+S11*(AlF17+BS17*s->dpt[1][5]+BeF37*s->dpt[3][5]);
  OpM111_1 = -S11p7*C6;
  OpM311_1 = C11p7*C6;
  AlM111_1 = C11*(AlM17_1+s->dpt[3][5]*S6)-S11*(AlM37_1-s->dpt[1][5]*S6);
  AlM211_1 = AlM26_1-OpM17_1*s->dpt[3][5]+OpM37_1*s->dpt[1][5];
  AlM311_1 = C11*(AlM37_1-s->dpt[1][5]*S6)+S11*(AlM17_1+s->dpt[3][5]*S6);
  AlM111_2 = AlM17_2*C11-AlM37_2*S11;
  AlM311_2 = AlM17_2*S11+AlM37_2*C11;
  AlM111_3 = AlM17_3*C11-AlM37_3*S11;
  AlM311_3 = AlM17_3*S11+AlM37_3*C11;
  AlM111_4 = -S11p7*C6;
  AlM311_4 = C11p7*C6;
  OpM111_5 = -S11p7*C6;
  OpM311_5 = C11p7*C6;
  AlM111_5 = S6*(s->dpt[1][5]*S11+s->dpt[3][5]*C11);
  AlM211_5 = -(OpM17_5*s->dpt[3][5]-OpM37_5*s->dpt[1][5]);
  AlM311_5 = -S6*(s->dpt[1][5]*C11-s->dpt[3][5]*S11);
  AlM211_6 = s->dpt[1][5]*S7-s->dpt[3][5]*C7;
  AlM111_7 = s->dpt[1][5]*S11+s->dpt[3][5]*C11;
  AlM311_7 = -(s->dpt[1][5]*C11-s->dpt[3][5]*S11);

// = = Block_0_1_0_2_0_4 = = 
 
// Forward Kinematics 

  OM112 = qd[12]+OM111;
  OM212 = OM27*C12+OM311*S12;
  OM312 = -(OM27*S12-OM311*C12);
  OpF212 = C12*(OpF26+qd[12]*OM311)+S12*(OpF311-qd[12]*OM27);
  OpM212_1 = OpM311_1*S12+C12*S6;
  OpM212_5 = OpM311_5*S12+C12*S6;
  OpM212_6 = S11p7*S12;

// = = Block_0_1_0_2_0_5 = = 
 
// Trigonometric Variables  

  S13p14 = C13*S14+S13*C14;
  C13p14 = C13*C14-S13*S14;
 
// Forward Kinematics 

  OM113 = qd[13]+OM111;
  OM213 = OM27*C13+OM311*S13;
  OM313 = -(OM27*S13-OM311*C13);
  OpF213 = C13*(OpF26+qd[13]*OM311)+S13*(OpF311-qd[13]*OM27);
  OpF313 = C13*(OpF311-qd[13]*OM27)-S13*(OpF26+qd[13]*OM311);
  BS513 = -(OM113*OM113+OM313*OM313);
  BS613 = OM213*OM313;
  BS913 = -(OM113*OM113+OM213*OM213);
  BeF613 = BS613-OpF111;
  BeF813 = BS613+OpF111;
  AlF213 = C13*(AlF211+BS511*s->dpt[2][12]+BeF611*s->dpt[3][12])+S13*(AlF311+BS911*s->dpt[3][12]+BeF811*s->dpt[2][12]);
  AlF313 = C13*(AlF311+BS911*s->dpt[3][12]+BeF811*s->dpt[2][12])-S13*(AlF211+BS511*s->dpt[2][12]+BeF611*s->dpt[3][12]);
  OpM213_1 = OpM311_1*S13+C13*S6;
  OpM313_1 = OpM311_1*C13-S13*S6;
  AlM213_1 = C13*(AlM211_1-OpM111_1*s->dpt[3][12])+S13*(AlM311_1+OpM111_1*s->dpt[2][12]);
  AlM313_1 = C13*(AlM311_1+OpM111_1*s->dpt[2][12])-S13*(AlM211_1-OpM111_1*s->dpt[3][12]);
  AlM213_2 = AlM26_2*C13+AlM311_2*S13;
  AlM313_2 = -(AlM26_2*S13-AlM311_2*C13);
  AlM213_3 = AlM26_3*C13+AlM311_3*S13;
  AlM313_3 = -(AlM26_3*S13-AlM311_3*C13);
  AlM213_4 = AlM311_4*S13+C13*S6;
  AlM313_4 = AlM311_4*C13-S13*S6;
  OpM213_5 = OpM311_5*S13+C13*S6;
  OpM313_5 = OpM311_5*C13-S13*S6;
  AlM213_5 = C13*(AlM211_5-OpM111_5*s->dpt[3][12])+S13*(AlM311_5+OpM111_5*s->dpt[2][12]);
  AlM313_5 = C13*(AlM311_5+OpM111_5*s->dpt[2][12])-S13*(AlM211_5-OpM111_5*s->dpt[3][12]);
  AlM213_6 = s->dpt[2][12]*C11p7*S13+C13*(AlM211_6-s->dpt[3][12]*C11p7);
  AlM313_6 = s->dpt[2][12]*C11p7*C13-S13*(AlM211_6-s->dpt[3][12]*C11p7);
  AlM213_7 = AlM311_7*S13;
  AlM313_7 = AlM311_7*C13;
  OM114 = qd[14]+OM113;
  OM314 = -(OM213*S14-OM313*C14);
  OpF214 = C14*(OpF213+qd[14]*OM313)+S14*(OpF313-qd[14]*OM213);
  OpF314 = C14*(OpF313-qd[14]*OM213)-S14*(OpF213+qd[14]*OM313);
  AlF114 = AlF111+BeF211*s->dpt[2][12]+BeF311*s->dpt[3][12]-s->dpt[2][17]*(OpF313-OM113*OM213)+s->dpt[3][17]*(OpF213+
 OM113*OM313);
  AlF214 = C14*(AlF213+BS513*s->dpt[2][17]+BeF613*s->dpt[3][17])+S14*(AlF313+BS913*s->dpt[3][17]+BeF813*s->dpt[2][17]);
  AlF314 = C14*(AlF313+BS913*s->dpt[3][17]+BeF813*s->dpt[2][17])-S14*(AlF213+BS513*s->dpt[2][17]+BeF613*s->dpt[3][17]);
  OpM214_1 = OpM213_1*C14+OpM313_1*S14;
  OpM314_1 = -(OpM213_1*S14-OpM313_1*C14);
  AlM114_1 = AlM111_1+OpM213_1*s->dpt[3][17]-OpM311_1*s->dpt[2][12]-OpM313_1*s->dpt[2][17]+s->dpt[3][12]*S6;
  AlM214_1 = C14*(AlM213_1-OpM111_1*s->dpt[3][17])+S14*(AlM313_1+OpM111_1*s->dpt[2][17]);
  AlM314_1 = C14*(AlM313_1+OpM111_1*s->dpt[2][17])-S14*(AlM213_1-OpM111_1*s->dpt[3][17]);
  AlM214_2 = AlM213_2*C14+AlM313_2*S14;
  AlM314_2 = -(AlM213_2*S14-AlM313_2*C14);
  AlM214_3 = AlM213_3*C14+AlM313_3*S14;
  AlM314_3 = -(AlM213_3*S14-AlM313_3*C14);
  AlM214_4 = AlM213_4*C14+AlM313_4*S14;
  AlM314_4 = -(AlM213_4*S14-AlM313_4*C14);
  OpM214_5 = OpM213_5*C14+OpM313_5*S14;
  OpM314_5 = -(OpM213_5*S14-OpM313_5*C14);
  AlM114_5 = AlM111_5+OpM213_5*s->dpt[3][17]-OpM311_5*s->dpt[2][12]-OpM313_5*s->dpt[2][17]+s->dpt[3][12]*S6;
  AlM214_5 = C14*(AlM213_5-OpM111_5*s->dpt[3][17])+S14*(AlM313_5+OpM111_5*s->dpt[2][17]);
  AlM314_5 = C14*(AlM313_5+OpM111_5*s->dpt[2][17])-S14*(AlM213_5-OpM111_5*s->dpt[3][17]);
  OpM214_6 = S11p7*S13p14;
  OpM314_6 = S11p7*C13p14;
  AlM114_6 = -S11p7*(s->dpt[2][12]+s->dpt[2][17]*C13-s->dpt[3][17]*S13);
  AlM214_6 = C14*(AlM213_6-s->dpt[3][17]*C11p7)+S14*(AlM313_6+s->dpt[2][17]*C11p7);
  AlM314_6 = C14*(AlM313_6+s->dpt[2][17]*C11p7)-S14*(AlM213_6-s->dpt[3][17]*C11p7);
  AlM114_7 = AlM111_7+s->dpt[3][12]+s->dpt[2][17]*S13+s->dpt[3][17]*C13;
  AlM214_7 = AlM213_7*C14+AlM313_7*S14;
  AlM314_7 = -(AlM213_7*S14-AlM313_7*C14);
  AlM114_11 = s->dpt[3][12]+s->dpt[2][17]*S13+s->dpt[3][17]*C13;
  AlM214_13 = s->dpt[2][17]*S14-s->dpt[3][17]*C14;
  AlM314_13 = s->dpt[2][17]*C14+s->dpt[3][17]*S14;
  OM115 = OM114*C15-OM314*S15;
  OM215 = qd[15]+OM213*C14+OM313*S14;
  OpF115 = C15*(OpF111-qd[15]*OM314)-S15*(OpF314+qd[15]*OM114);
  OpF315 = C15*(OpF314+qd[15]*OM114)+S15*(OpF111-qd[15]*OM314);
  AlF115 = AlF114*C15-AlF314*S15;
  AlF315 = AlF114*S15+AlF314*C15;
  OpM115_1 = OpM111_1*C15-OpM314_1*S15;
  OpM315_1 = OpM111_1*S15+OpM314_1*C15;
  AlM115_1 = AlM114_1*C15-AlM314_1*S15;
  AlM315_1 = AlM114_1*S15+AlM314_1*C15;
  AlM115_2 = AlM111_2*C15-AlM314_2*S15;
  AlM315_2 = AlM111_2*S15+AlM314_2*C15;
  AlM115_3 = AlM111_3*C15-AlM314_3*S15;
  AlM315_3 = AlM111_3*S15+AlM314_3*C15;
  AlM115_4 = AlM111_4*C15-AlM314_4*S15;
  AlM315_4 = AlM111_4*S15+AlM314_4*C15;
  OpM115_5 = OpM111_5*C15-OpM314_5*S15;
  OpM315_5 = OpM111_5*S15+OpM314_5*C15;
  AlM115_5 = AlM114_5*C15-AlM314_5*S15;
  AlM315_5 = AlM114_5*S15+AlM314_5*C15;
  OpM115_6 = -(OpM314_6*S15-C11p7*C15);
  OpM315_6 = OpM314_6*C15+C11p7*S15;
  AlM115_6 = AlM114_6*C15-AlM314_6*S15;
  AlM315_6 = AlM114_6*S15+AlM314_6*C15;
  OpM115_7 = S13p14*S15;
  OpM315_7 = -S13p14*C15;
  AlM115_7 = AlM114_7*C15-AlM314_7*S15;
  AlM315_7 = AlM114_7*S15+AlM314_7*C15;
  OpM115_11 = S13p14*S15;
  OpM315_11 = -S13p14*C15;
  AlM115_11 = AlM114_11*C15;
  AlM315_11 = AlM114_11*S15;
  AlM115_13 = -AlM314_13*S15;
  AlM315_13 = AlM314_13*C15;
  OM116 = OM115*C16+OM215*S16;
  OM216 = -(OM115*S16-OM215*C16);
  OM316 = qd[16]+OM114*S15+OM314*C15;
  OpF116 = C16*(OpF115+qd[16]*OM215)+S16*(OpF214-qd[16]*OM115);
  OpF216 = C16*(OpF214-qd[16]*OM115)-S16*(OpF115+qd[16]*OM215);
  BS216 = OM116*OM216;
  BS316 = OM116*OM316;
  BS516 = -(OM116*OM116+OM316*OM316);
  BS616 = OM216*OM316;
  BS916 = -(OM116*OM116+OM216*OM216);
  BeF416 = BS216+OpF315;
  BeF616 = BS616-OpF116;
  BeF716 = BS316-OpF216;
  BeF816 = BS616+OpF116;
  AlF216 = -(AlF115*S16-AlF214*C16);
  OpM116_1 = OpM115_1*C16+OpM214_1*S16;
  OpM216_1 = -(OpM115_1*S16-OpM214_1*C16);
  AlM216_1 = -(AlM115_1*S16-AlM214_1*C16);
  AlM116_2 = AlM115_2*C16+AlM214_2*S16;
  AlM216_2 = -(AlM115_2*S16-AlM214_2*C16);
  AlM116_3 = AlM115_3*C16+AlM214_3*S16;
  AlM216_3 = -(AlM115_3*S16-AlM214_3*C16);
  AlM116_4 = AlM115_4*C16+AlM214_4*S16;
  AlM216_4 = -(AlM115_4*S16-AlM214_4*C16);
  OpM116_5 = OpM115_5*C16+OpM214_5*S16;
  OpM216_5 = -(OpM115_5*S16-OpM214_5*C16);
  AlM216_5 = -(AlM115_5*S16-AlM214_5*C16);
  OpM116_6 = OpM115_6*C16+OpM214_6*S16;
  OpM216_6 = -(OpM115_6*S16-OpM214_6*C16);
  AlM216_6 = -(AlM115_6*S16-AlM214_6*C16);
  OpM116_7 = OpM115_7*C16+C13p14*S16;
  OpM216_7 = -(OpM115_7*S16-C13p14*C16);
  AlM216_7 = -(AlM115_7*S16-AlM214_7*C16);
  OpM116_11 = OpM115_11*C16+C13p14*S16;
  OpM216_11 = -(OpM115_11*S16-C13p14*C16);
  AlM216_11 = -AlM115_11*S16;
  OpM116_13 = C15*C16;
  OpM216_13 = -C15*S16;
  AlM216_13 = -(AlM115_13*S16-AlM214_13*C16);
  OpM116_14 = C15*C16;
  OpM216_14 = -C15*S16;
  OM317 = -(OM216*S17-OM316*C17);
  OpF317 = -(OpF216*S17-OpF315*C17);
  AlF117 = AlF115*C16+AlF214*S16-s->dpt[1][21]*(OM216*OM216+OM316*OM316)+s->dpt[2][21]*(BS216-OpF315)+s->dpt[3][21]*(
 BS316+OpF216);
  AlF317 = C17*(AlF315+BS916*s->dpt[3][21]+BeF716*s->dpt[1][21]+BeF816*s->dpt[2][21])-S17*(AlF216+BS516*s->dpt[2][21]+
 BeF416*s->dpt[1][21]+BeF616*s->dpt[3][21]);
  OpM317_1 = -(OpM216_1*S17-OpM315_1*C17);
  AlM117_1 = AlM115_1*C16+AlM214_1*S16+OpM216_1*s->dpt[3][21]-OpM315_1*s->dpt[2][21];
  AlM317_1 = C17*(AlM315_1+OpM116_1*s->dpt[2][21]-OpM216_1*s->dpt[1][21])-S17*(AlM216_1-OpM116_1*s->dpt[3][21]+OpM315_1*
 s->dpt[1][21]);
  AlM317_2 = -(AlM216_2*S17-AlM315_2*C17);
  AlM317_3 = -(AlM216_3*S17-AlM315_3*C17);
  AlM317_4 = -(AlM216_4*S17-AlM315_4*C17);
  OpM317_5 = -(OpM216_5*S17-OpM315_5*C17);
  AlM117_5 = AlM115_5*C16+AlM214_5*S16+OpM216_5*s->dpt[3][21]-OpM315_5*s->dpt[2][21];
  AlM317_5 = C17*(AlM315_5+OpM116_5*s->dpt[2][21]-OpM216_5*s->dpt[1][21])-S17*(AlM216_5-OpM116_5*s->dpt[3][21]+OpM315_5*
 s->dpt[1][21]);
  OpM317_6 = -(OpM216_6*S17-OpM315_6*C17);
  AlM117_6 = AlM115_6*C16+AlM214_6*S16+OpM216_6*s->dpt[3][21]-OpM315_6*s->dpt[2][21];
  AlM317_6 = C17*(AlM315_6+OpM116_6*s->dpt[2][21]-OpM216_6*s->dpt[1][21])-S17*(AlM216_6-OpM116_6*s->dpt[3][21]+OpM315_6*
 s->dpt[1][21]);
  OpM317_7 = -(OpM216_7*S17-OpM315_7*C17);
  AlM117_7 = AlM115_7*C16+AlM214_7*S16+OpM216_7*s->dpt[3][21]-OpM315_7*s->dpt[2][21];
  AlM317_7 = C17*(AlM315_7+OpM116_7*s->dpt[2][21]-OpM216_7*s->dpt[1][21])-S17*(AlM216_7-OpM116_7*s->dpt[3][21]+OpM315_7*
 s->dpt[1][21]);
  OpM317_11 = -(OpM216_11*S17-OpM315_11*C17);
  AlM117_11 = AlM115_11*C16+OpM216_11*s->dpt[3][21]-OpM315_11*s->dpt[2][21];
  AlM317_11 = C17*(AlM315_11+OpM116_11*s->dpt[2][21]-OpM216_11*s->dpt[1][21])-S17*(AlM216_11-OpM116_11*s->dpt[3][21]+
 OpM315_11*s->dpt[1][21]);
  OpM317_13 = -(OpM216_13*S17-S15*C17);
  AlM117_13 = AlM115_13*C16+AlM214_13*S16+OpM216_13*s->dpt[3][21]-s->dpt[2][21]*S15;
  AlM317_13 = C17*(AlM315_13+OpM116_13*s->dpt[2][21]-OpM216_13*s->dpt[1][21])-S17*(AlM216_13-OpM116_13*s->dpt[3][21]+
 s->dpt[1][21]*S15);
  OpM317_14 = -(OpM216_14*S17-S15*C17);
  AlM117_14 = OpM216_14*s->dpt[3][21]-s->dpt[2][21]*S15;
  AlM317_14 = C17*(OpM116_14*s->dpt[2][21]-OpM216_14*s->dpt[1][21])+S17*(OpM116_14*s->dpt[3][21]-s->dpt[1][21]*S15);
  OpM317_15 = -C16*S17;
  AlM117_15 = s->dpt[3][21]*C16;
  AlM317_15 = s->dpt[3][21]*S16*S17-C17*(s->dpt[1][21]*C16-s->dpt[2][21]*S16);
  AlM317_16 = -s->dpt[1][21]*S17;
  OM118 = OM116*C18-OM317*S18;
  OM218 = qd[18]+OM216*C17+OM316*S17;
  OM318 = OM116*S18+OM317*C18;

// = = Block_0_1_0_2_0_6 = = 
 
// Trigonometric Variables  

  S19p20 = C19*S20+S19*C20;
  C19p20 = C19*C20-S19*S20;
 
// Forward Kinematics 

  OM119 = qd[19]+OM111;
  OM219 = OM27*C19+OM311*S19;
  OM319 = -(OM27*S19-OM311*C19);
  OpF219 = C19*(OpF26+qd[19]*OM311)+S19*(OpF311-qd[19]*OM27);
  OpF319 = C19*(OpF311-qd[19]*OM27)-S19*(OpF26+qd[19]*OM311);
  BS519 = -(OM119*OM119+OM319*OM319);
  BS619 = OM219*OM319;
  BS919 = -(OM119*OM119+OM219*OM219);
  BeF619 = BS619-OpF111;
  BeF819 = BS619+OpF111;
  AlF219 = C19*(AlF211+BS511*s->dpt[2][13]+BeF611*s->dpt[3][13])+S19*(AlF311+BS911*s->dpt[3][13]+BeF811*s->dpt[2][13]);
  AlF319 = C19*(AlF311+BS911*s->dpt[3][13]+BeF811*s->dpt[2][13])-S19*(AlF211+BS511*s->dpt[2][13]+BeF611*s->dpt[3][13]);
  OpM219_1 = OpM311_1*S19+C19*S6;
  OpM319_1 = OpM311_1*C19-S19*S6;
  AlM219_1 = C19*(AlM211_1-OpM111_1*s->dpt[3][13])+S19*(AlM311_1+OpM111_1*s->dpt[2][13]);
  AlM319_1 = C19*(AlM311_1+OpM111_1*s->dpt[2][13])-S19*(AlM211_1-OpM111_1*s->dpt[3][13]);
  AlM219_2 = AlM26_2*C19+AlM311_2*S19;
  AlM319_2 = -(AlM26_2*S19-AlM311_2*C19);
  AlM219_3 = AlM26_3*C19+AlM311_3*S19;
  AlM319_3 = -(AlM26_3*S19-AlM311_3*C19);
  AlM219_4 = AlM311_4*S19+C19*S6;
  AlM319_4 = AlM311_4*C19-S19*S6;
  OpM219_5 = OpM311_5*S19+C19*S6;
  OpM319_5 = OpM311_5*C19-S19*S6;
  AlM219_5 = C19*(AlM211_5-OpM111_5*s->dpt[3][13])+S19*(AlM311_5+OpM111_5*s->dpt[2][13]);
  AlM319_5 = C19*(AlM311_5+OpM111_5*s->dpt[2][13])-S19*(AlM211_5-OpM111_5*s->dpt[3][13]);
  AlM219_6 = s->dpt[2][13]*C11p7*S19+C19*(AlM211_6-s->dpt[3][13]*C11p7);
  AlM319_6 = s->dpt[2][13]*C11p7*C19-S19*(AlM211_6-s->dpt[3][13]*C11p7);
  AlM219_7 = AlM311_7*S19;
  AlM319_7 = AlM311_7*C19;
  OM120 = qd[20]+OM119;
  OM320 = -(OM219*S20-OM319*C20);
  OpF220 = C20*(OpF219+qd[20]*OM319)+S20*(OpF319-qd[20]*OM219);
  OpF320 = C20*(OpF319-qd[20]*OM219)-S20*(OpF219+qd[20]*OM319);
  AlF120 = AlF111+BeF211*s->dpt[2][13]+BeF311*s->dpt[3][13]-s->dpt[2][24]*(OpF319-OM119*OM219)+s->dpt[3][24]*(OpF219+
 OM119*OM319);
  AlF220 = C20*(AlF219+BS519*s->dpt[2][24]+BeF619*s->dpt[3][24])+S20*(AlF319+BS919*s->dpt[3][24]+BeF819*s->dpt[2][24]);
  AlF320 = C20*(AlF319+BS919*s->dpt[3][24]+BeF819*s->dpt[2][24])-S20*(AlF219+BS519*s->dpt[2][24]+BeF619*s->dpt[3][24]);
  OpM220_1 = OpM219_1*C20+OpM319_1*S20;
  OpM320_1 = -(OpM219_1*S20-OpM319_1*C20);
  AlM120_1 = AlM111_1+OpM219_1*s->dpt[3][24]-OpM311_1*s->dpt[2][13]-OpM319_1*s->dpt[2][24]+s->dpt[3][13]*S6;
  AlM220_1 = C20*(AlM219_1-OpM111_1*s->dpt[3][24])+S20*(AlM319_1+OpM111_1*s->dpt[2][24]);
  AlM320_1 = C20*(AlM319_1+OpM111_1*s->dpt[2][24])-S20*(AlM219_1-OpM111_1*s->dpt[3][24]);
  AlM220_2 = AlM219_2*C20+AlM319_2*S20;
  AlM320_2 = -(AlM219_2*S20-AlM319_2*C20);
  AlM220_3 = AlM219_3*C20+AlM319_3*S20;
  AlM320_3 = -(AlM219_3*S20-AlM319_3*C20);
  AlM220_4 = AlM219_4*C20+AlM319_4*S20;
  AlM320_4 = -(AlM219_4*S20-AlM319_4*C20);
  OpM220_5 = OpM219_5*C20+OpM319_5*S20;
  OpM320_5 = -(OpM219_5*S20-OpM319_5*C20);
  AlM120_5 = AlM111_5+OpM219_5*s->dpt[3][24]-OpM311_5*s->dpt[2][13]-OpM319_5*s->dpt[2][24]+s->dpt[3][13]*S6;
  AlM220_5 = C20*(AlM219_5-OpM111_5*s->dpt[3][24])+S20*(AlM319_5+OpM111_5*s->dpt[2][24]);
  AlM320_5 = C20*(AlM319_5+OpM111_5*s->dpt[2][24])-S20*(AlM219_5-OpM111_5*s->dpt[3][24]);
  OpM220_6 = S11p7*S19p20;
  OpM320_6 = S11p7*C19p20;
  AlM120_6 = -S11p7*(s->dpt[2][13]+s->dpt[2][24]*C19-s->dpt[3][24]*S19);
  AlM220_6 = C20*(AlM219_6-s->dpt[3][24]*C11p7)+S20*(AlM319_6+s->dpt[2][24]*C11p7);
  AlM320_6 = C20*(AlM319_6+s->dpt[2][24]*C11p7)-S20*(AlM219_6-s->dpt[3][24]*C11p7);
  AlM120_7 = AlM111_7+s->dpt[3][13]+s->dpt[2][24]*S19+s->dpt[3][24]*C19;
  AlM220_7 = AlM219_7*C20+AlM319_7*S20;
  AlM320_7 = -(AlM219_7*S20-AlM319_7*C20);
  AlM120_11 = s->dpt[3][13]+s->dpt[2][24]*S19+s->dpt[3][24]*C19;
  AlM220_19 = s->dpt[2][24]*S20-s->dpt[3][24]*C20;
  AlM320_19 = s->dpt[2][24]*C20+s->dpt[3][24]*S20;
  OM121 = OM120*C21-OM320*S21;
  OM221 = qd[21]+OM219*C20+OM319*S20;
  OpF121 = C21*(OpF111-qd[21]*OM320)-S21*(OpF320+qd[21]*OM120);
  OpF321 = C21*(OpF320+qd[21]*OM120)+S21*(OpF111-qd[21]*OM320);
  AlF121 = AlF120*C21-AlF320*S21;
  AlF321 = AlF120*S21+AlF320*C21;
  OpM121_1 = OpM111_1*C21-OpM320_1*S21;
  OpM321_1 = OpM111_1*S21+OpM320_1*C21;
  AlM121_1 = AlM120_1*C21-AlM320_1*S21;
  AlM321_1 = AlM120_1*S21+AlM320_1*C21;
  AlM121_2 = AlM111_2*C21-AlM320_2*S21;
  AlM321_2 = AlM111_2*S21+AlM320_2*C21;
  AlM121_3 = AlM111_3*C21-AlM320_3*S21;
  AlM321_3 = AlM111_3*S21+AlM320_3*C21;
  AlM121_4 = AlM111_4*C21-AlM320_4*S21;
  AlM321_4 = AlM111_4*S21+AlM320_4*C21;
  OpM121_5 = OpM111_5*C21-OpM320_5*S21;
  OpM321_5 = OpM111_5*S21+OpM320_5*C21;
  AlM121_5 = AlM120_5*C21-AlM320_5*S21;
  AlM321_5 = AlM120_5*S21+AlM320_5*C21;
  OpM121_6 = -(OpM320_6*S21-C11p7*C21);
  OpM321_6 = OpM320_6*C21+C11p7*S21;
  AlM121_6 = AlM120_6*C21-AlM320_6*S21;
  AlM321_6 = AlM120_6*S21+AlM320_6*C21;
  OpM121_7 = S19p20*S21;
  OpM321_7 = -S19p20*C21;
  AlM121_7 = AlM120_7*C21-AlM320_7*S21;
  AlM321_7 = AlM120_7*S21+AlM320_7*C21;
  OpM121_11 = S19p20*S21;
  OpM321_11 = -S19p20*C21;
  AlM121_11 = AlM120_11*C21;
  AlM321_11 = AlM120_11*S21;
  AlM121_19 = -AlM320_19*S21;
  AlM321_19 = AlM320_19*C21;
  OM122 = OM121*C22+OM221*S22;
  OM222 = -(OM121*S22-OM221*C22);
  OM322 = qd[22]+OM120*S21+OM320*C21;
  OpF122 = C22*(OpF121+qd[22]*OM221)+S22*(OpF220-qd[22]*OM121);
  OpF222 = C22*(OpF220-qd[22]*OM121)-S22*(OpF121+qd[22]*OM221);
  BS222 = OM122*OM222;
  BS322 = OM122*OM322;
  BS522 = -(OM122*OM122+OM322*OM322);
  BS622 = OM222*OM322;
  BS922 = -(OM122*OM122+OM222*OM222);
  BeF422 = BS222+OpF321;
  BeF622 = BS622-OpF122;
  BeF722 = BS322-OpF222;
  BeF822 = BS622+OpF122;
  AlF222 = -(AlF121*S22-AlF220*C22);
  OpM122_1 = OpM121_1*C22+OpM220_1*S22;
  OpM222_1 = -(OpM121_1*S22-OpM220_1*C22);
  AlM222_1 = -(AlM121_1*S22-AlM220_1*C22);
  AlM122_2 = AlM121_2*C22+AlM220_2*S22;
  AlM222_2 = -(AlM121_2*S22-AlM220_2*C22);
  AlM122_3 = AlM121_3*C22+AlM220_3*S22;
  AlM222_3 = -(AlM121_3*S22-AlM220_3*C22);
  AlM122_4 = AlM121_4*C22+AlM220_4*S22;
  AlM222_4 = -(AlM121_4*S22-AlM220_4*C22);
  OpM122_5 = OpM121_5*C22+OpM220_5*S22;
  OpM222_5 = -(OpM121_5*S22-OpM220_5*C22);
  AlM222_5 = -(AlM121_5*S22-AlM220_5*C22);
  OpM122_6 = OpM121_6*C22+OpM220_6*S22;
  OpM222_6 = -(OpM121_6*S22-OpM220_6*C22);
  AlM222_6 = -(AlM121_6*S22-AlM220_6*C22);
  OpM122_7 = OpM121_7*C22+C19p20*S22;
  OpM222_7 = -(OpM121_7*S22-C19p20*C22);
  AlM222_7 = -(AlM121_7*S22-AlM220_7*C22);
  OpM122_11 = OpM121_11*C22+C19p20*S22;
  OpM222_11 = -(OpM121_11*S22-C19p20*C22);
  AlM222_11 = -AlM121_11*S22;
  OpM122_19 = C21*C22;
  OpM222_19 = -C21*S22;
  AlM222_19 = -(AlM121_19*S22-AlM220_19*C22);
  OpM122_20 = C21*C22;
  OpM222_20 = -C21*S22;
  OM323 = -(OM222*S23-OM322*C23);
  OpF323 = -(OpF222*S23-OpF321*C23);
  AlF123 = AlF121*C22+AlF220*S22-s->dpt[1][28]*(OM222*OM222+OM322*OM322)+s->dpt[2][28]*(BS222-OpF321)+s->dpt[3][28]*(
 BS322+OpF222);
  AlF323 = C23*(AlF321+BS922*s->dpt[3][28]+BeF722*s->dpt[1][28]+BeF822*s->dpt[2][28])-S23*(AlF222+BS522*s->dpt[2][28]+
 BeF422*s->dpt[1][28]+BeF622*s->dpt[3][28]);
  OpM323_1 = -(OpM222_1*S23-OpM321_1*C23);
  AlM123_1 = AlM121_1*C22+AlM220_1*S22+OpM222_1*s->dpt[3][28]-OpM321_1*s->dpt[2][28];
  AlM323_1 = C23*(AlM321_1+OpM122_1*s->dpt[2][28]-OpM222_1*s->dpt[1][28])-S23*(AlM222_1-OpM122_1*s->dpt[3][28]+OpM321_1*
 s->dpt[1][28]);
  AlM323_2 = -(AlM222_2*S23-AlM321_2*C23);
  AlM323_3 = -(AlM222_3*S23-AlM321_3*C23);
  AlM323_4 = -(AlM222_4*S23-AlM321_4*C23);
  OpM323_5 = -(OpM222_5*S23-OpM321_5*C23);
  AlM123_5 = AlM121_5*C22+AlM220_5*S22+OpM222_5*s->dpt[3][28]-OpM321_5*s->dpt[2][28];
  AlM323_5 = C23*(AlM321_5+OpM122_5*s->dpt[2][28]-OpM222_5*s->dpt[1][28])-S23*(AlM222_5-OpM122_5*s->dpt[3][28]+OpM321_5*
 s->dpt[1][28]);
  OpM323_6 = -(OpM222_6*S23-OpM321_6*C23);
  AlM123_6 = AlM121_6*C22+AlM220_6*S22+OpM222_6*s->dpt[3][28]-OpM321_6*s->dpt[2][28];
  AlM323_6 = C23*(AlM321_6+OpM122_6*s->dpt[2][28]-OpM222_6*s->dpt[1][28])-S23*(AlM222_6-OpM122_6*s->dpt[3][28]+OpM321_6*
 s->dpt[1][28]);
  OpM323_7 = -(OpM222_7*S23-OpM321_7*C23);
  AlM123_7 = AlM121_7*C22+AlM220_7*S22+OpM222_7*s->dpt[3][28]-OpM321_7*s->dpt[2][28];
  AlM323_7 = C23*(AlM321_7+OpM122_7*s->dpt[2][28]-OpM222_7*s->dpt[1][28])-S23*(AlM222_7-OpM122_7*s->dpt[3][28]+OpM321_7*
 s->dpt[1][28]);
  OpM323_11 = -(OpM222_11*S23-OpM321_11*C23);
  AlM123_11 = AlM121_11*C22+OpM222_11*s->dpt[3][28]-OpM321_11*s->dpt[2][28];
  AlM323_11 = C23*(AlM321_11+OpM122_11*s->dpt[2][28]-OpM222_11*s->dpt[1][28])-S23*(AlM222_11-OpM122_11*s->dpt[3][28]+
 OpM321_11*s->dpt[1][28]);
  OpM323_19 = -(OpM222_19*S23-S21*C23);
  AlM123_19 = AlM121_19*C22+AlM220_19*S22+OpM222_19*s->dpt[3][28]-s->dpt[2][28]*S21;
  AlM323_19 = C23*(AlM321_19+OpM122_19*s->dpt[2][28]-OpM222_19*s->dpt[1][28])-S23*(AlM222_19-OpM122_19*s->dpt[3][28]+
 s->dpt[1][28]*S21);
  OpM323_20 = -(OpM222_20*S23-S21*C23);
  AlM123_20 = OpM222_20*s->dpt[3][28]-s->dpt[2][28]*S21;
  AlM323_20 = C23*(OpM122_20*s->dpt[2][28]-OpM222_20*s->dpt[1][28])+S23*(OpM122_20*s->dpt[3][28]-s->dpt[1][28]*S21);
  OpM323_21 = -C22*S23;
  AlM123_21 = s->dpt[3][28]*C22;
  AlM323_21 = s->dpt[3][28]*S22*S23-C23*(s->dpt[1][28]*C22-s->dpt[2][28]*S22);
  AlM323_22 = -s->dpt[1][28]*S23;
  OM124 = OM122*C24-OM323*S24;
  OM224 = qd[24]+OM222*C23+OM322*S23;
  OM324 = OM122*S24+OM323*C24;

// = = Block_0_2_0_1_0_4 = = 
 
// Backward Dynamics 

  FA112 = -(s->frc[1][12]-s->m[12]*(AlF111+s->l[3][12]*(OpF212+OM112*OM312)));
  FA212 = -(s->frc[2][12]-s->m[12]*(AlF211*C12+AlF311*S12-s->l[3][12]*(OpF111-OM212*OM312)));
  FA312 = -(s->frc[3][12]+s->m[12]*(AlF211*S12-AlF311*C12+s->l[3][12]*(OM112*OM112+OM212*OM212)));
  CF112 = -(s->trq[1][12]-s->In[1][12]*OpF111+s->In[5][12]*OM212*OM312+FA212*s->l[3][12]);
  CF212 = -(s->trq[2][12]-s->In[1][12]*OM112*OM312-s->In[5][12]*OpF212-FA112*s->l[3][12]);
  CF312 = -(s->trq[3][12]+OM112*OM212*(s->In[1][12]-s->In[5][12]));
  FB112_1 = s->m[12]*(AlM111_1+OpM212_1*s->l[3][12]);
  FB212_1 = s->m[12]*(AlM211_1*C12+AlM311_1*S12-OpM111_1*s->l[3][12]);
  FB312_1 = -s->m[12]*(AlM211_1*S12-AlM311_1*C12);
  CM112_1 = s->In[1][12]*OpM111_1-FB212_1*s->l[3][12];
  CM212_1 = s->In[5][12]*OpM212_1+FB112_1*s->l[3][12];
  FB112_2 = s->m[12]*AlM111_2;
  FB212_2 = s->m[12]*(AlM26_2*C12+AlM311_2*S12);
  FB312_2 = -s->m[12]*(AlM26_2*S12-AlM311_2*C12);
  CM112_2 = -FB212_2*s->l[3][12];
  CM212_2 = FB112_2*s->l[3][12];
  FB112_3 = s->m[12]*AlM111_3;
  FB212_3 = s->m[12]*(AlM26_3*C12+AlM311_3*S12);
  FB312_3 = -s->m[12]*(AlM26_3*S12-AlM311_3*C12);
  CM112_3 = -FB212_3*s->l[3][12];
  CM212_3 = FB112_3*s->l[3][12];
  FB112_4 = s->m[12]*AlM111_4;
  FB212_4 = s->m[12]*(AlM311_4*S12+C12*S6);
  FB312_4 = s->m[12]*(AlM311_4*C12-S12*S6);
  CM112_4 = -FB212_4*s->l[3][12];
  CM212_4 = FB112_4*s->l[3][12];
  FB112_5 = s->m[12]*(AlM111_5+OpM212_5*s->l[3][12]);
  FB212_5 = s->m[12]*(AlM211_5*C12+AlM311_5*S12-OpM111_5*s->l[3][12]);
  FB312_5 = -s->m[12]*(AlM211_5*S12-AlM311_5*C12);
  CM112_5 = s->In[1][12]*OpM111_5-FB212_5*s->l[3][12];
  CM212_5 = s->In[5][12]*OpM212_5+FB112_5*s->l[3][12];
  FB112_6 = s->m[12]*OpM212_6*s->l[3][12];
  FB212_6 = s->m[12]*(AlM211_6*C12-s->l[3][12]*C11p7);
  FB312_6 = -s->m[12]*AlM211_6*S12;
  CM112_6 = s->In[1][12]*C11p7-FB212_6*s->l[3][12];
  CM212_6 = s->In[5][12]*OpM212_6+FB112_6*s->l[3][12];
  FB112_7 = s->m[12]*(AlM111_7+s->l[3][12]*C12);
  CM112_7 = -s->m[12]*AlM311_7*s->l[3][12]*S12;
  CM112_12 = s->In[1][12]+s->m[12]*s->l[3][12]*s->l[3][12];

// = = Block_0_2_0_1_0_5 = = 
 
// Backward Dynamics 

  FA118 = -(s->frc[1][18]-s->m[18]*(AlF117*C18-AlF317*S18));
  FA218 = -(s->frc[2][18]-s->m[18]*(C17*(AlF216+BS516*s->dpt[2][21]+BeF416*s->dpt[1][21]+BeF616*s->dpt[3][21])+S17*(
 AlF315+BS916*s->dpt[3][21]+BeF716*s->dpt[1][21]+BeF816*s->dpt[2][21])));
  FA318 = -(s->frc[3][18]-s->m[18]*(AlF117*S18+AlF317*C18));
  CF118 = -(s->trq[1][18]-s->In[1][18]*(C18*(OpF116-qd[18]*OM317)-S18*(OpF317+qd[18]*OM116))+OM218*OM318*(s->In[5][18]-
 s->In[9][18]));
  CF218 = -(s->trq[2][18]-s->In[5][18]*(OpF216*C17+OpF315*S17)-OM118*OM318*(s->In[1][18]-s->In[9][18]));
  CF318 = -(s->trq[3][18]-s->In[9][18]*(C18*(OpF317+qd[18]*OM116)+S18*(OpF116-qd[18]*OM317))+OM118*OM218*(s->In[1][18]-
 s->In[5][18]));
  FB118_1 = s->m[18]*(AlM117_1*C18-AlM317_1*S18);
  FB218_1 = s->m[18]*(C17*(AlM216_1-OpM116_1*s->dpt[3][21]+OpM315_1*s->dpt[1][21])+S17*(AlM315_1+OpM116_1*s->dpt[2][21]-
 OpM216_1*s->dpt[1][21]));
  FB318_1 = s->m[18]*(AlM117_1*S18+AlM317_1*C18);
  CM118_1 = s->In[1][18]*(OpM116_1*C18-OpM317_1*S18);
  CM218_1 = s->In[5][18]*(OpM216_1*C17+OpM315_1*S17);
  CM318_1 = s->In[9][18]*(OpM116_1*S18+OpM317_1*C18);
  FB118_2 = s->m[18]*(AlM116_2*C18-AlM317_2*S18);
  FB218_2 = s->m[18]*(AlM216_2*C17+AlM315_2*S17);
  FB318_2 = s->m[18]*(AlM116_2*S18+AlM317_2*C18);
  FB118_3 = s->m[18]*(AlM116_3*C18-AlM317_3*S18);
  FB218_3 = s->m[18]*(AlM216_3*C17+AlM315_3*S17);
  FB318_3 = s->m[18]*(AlM116_3*S18+AlM317_3*C18);
  FB118_4 = s->m[18]*(AlM116_4*C18-AlM317_4*S18);
  FB218_4 = s->m[18]*(AlM216_4*C17+AlM315_4*S17);
  FB318_4 = s->m[18]*(AlM116_4*S18+AlM317_4*C18);
  FB118_5 = s->m[18]*(AlM117_5*C18-AlM317_5*S18);
  FB218_5 = s->m[18]*(C17*(AlM216_5-OpM116_5*s->dpt[3][21]+OpM315_5*s->dpt[1][21])+S17*(AlM315_5+OpM116_5*s->dpt[2][21]-
 OpM216_5*s->dpt[1][21]));
  FB318_5 = s->m[18]*(AlM117_5*S18+AlM317_5*C18);
  CM118_5 = s->In[1][18]*(OpM116_5*C18-OpM317_5*S18);
  CM218_5 = s->In[5][18]*(OpM216_5*C17+OpM315_5*S17);
  CM318_5 = s->In[9][18]*(OpM116_5*S18+OpM317_5*C18);
  FB118_6 = s->m[18]*(AlM117_6*C18-AlM317_6*S18);
  FB218_6 = s->m[18]*(C17*(AlM216_6-OpM116_6*s->dpt[3][21]+OpM315_6*s->dpt[1][21])+S17*(AlM315_6+OpM116_6*s->dpt[2][21]-
 OpM216_6*s->dpt[1][21]));
  FB318_6 = s->m[18]*(AlM117_6*S18+AlM317_6*C18);
  CM118_6 = s->In[1][18]*(OpM116_6*C18-OpM317_6*S18);
  CM218_6 = s->In[5][18]*(OpM216_6*C17+OpM315_6*S17);
  CM318_6 = s->In[9][18]*(OpM116_6*S18+OpM317_6*C18);
  FB118_7 = s->m[18]*(AlM117_7*C18-AlM317_7*S18);
  FB218_7 = s->m[18]*(C17*(AlM216_7-OpM116_7*s->dpt[3][21]+OpM315_7*s->dpt[1][21])+S17*(AlM315_7+OpM116_7*s->dpt[2][21]-
 OpM216_7*s->dpt[1][21]));
  FB318_7 = s->m[18]*(AlM117_7*S18+AlM317_7*C18);
  CM118_7 = s->In[1][18]*(OpM116_7*C18-OpM317_7*S18);
  CM218_7 = s->In[5][18]*(OpM216_7*C17+OpM315_7*S17);
  CM318_7 = s->In[9][18]*(OpM116_7*S18+OpM317_7*C18);
  FB118_11 = s->m[18]*(AlM117_11*C18-AlM317_11*S18);
  FB218_11 = s->m[18]*(C17*(AlM216_11-OpM116_11*s->dpt[3][21]+OpM315_11*s->dpt[1][21])+S17*(AlM315_11+OpM116_11*
 s->dpt[2][21]-OpM216_11*s->dpt[1][21]));
  FB318_11 = s->m[18]*(AlM117_11*S18+AlM317_11*C18);
  CM118_11 = s->In[1][18]*(OpM116_11*C18-OpM317_11*S18);
  CM218_11 = s->In[5][18]*(OpM216_11*C17+OpM315_11*S17);
  CM318_11 = s->In[9][18]*(OpM116_11*S18+OpM317_11*C18);
  FB118_13 = s->m[18]*(AlM117_13*C18-AlM317_13*S18);
  FB218_13 = s->m[18]*(C17*(AlM216_13-OpM116_13*s->dpt[3][21]+s->dpt[1][21]*S15)+S17*(AlM315_13+OpM116_13*s->dpt[2][21]-
 OpM216_13*s->dpt[1][21]));
  FB318_13 = s->m[18]*(AlM117_13*S18+AlM317_13*C18);
  CM118_13 = s->In[1][18]*(OpM116_13*C18-OpM317_13*S18);
  CM218_13 = s->In[5][18]*(OpM216_13*C17+S15*S17);
  CM318_13 = s->In[9][18]*(OpM116_13*S18+OpM317_13*C18);
  FB118_14 = s->m[18]*(AlM117_14*C18-AlM317_14*S18);
  FB218_14 = -s->m[18]*(C17*(OpM116_14*s->dpt[3][21]-s->dpt[1][21]*S15)-S17*(OpM116_14*s->dpt[2][21]-OpM216_14*
 s->dpt[1][21]));
  FB318_14 = s->m[18]*(AlM117_14*S18+AlM317_14*C18);
  CM118_14 = s->In[1][18]*(OpM116_14*C18-OpM317_14*S18);
  CM218_14 = s->In[5][18]*(OpM216_14*C17+S15*S17);
  CM318_14 = s->In[9][18]*(OpM116_14*S18+OpM317_14*C18);
  FB118_15 = s->m[18]*(AlM117_15*C18-AlM317_15*S18);
  FB218_15 = -s->m[18]*(s->dpt[3][21]*S16*C17+S17*(s->dpt[1][21]*C16-s->dpt[2][21]*S16));
  FB318_15 = s->m[18]*(AlM117_15*S18+AlM317_15*C18);
  CM118_15 = -s->In[1][18]*(OpM317_15*S18-S16*C18);
  CM218_15 = s->In[5][18]*C16*C17;
  CM318_15 = s->In[9][18]*(OpM317_15*C18+S16*S18);
  FB118_16 = -s->m[18]*(AlM317_16*S18+s->dpt[2][21]*C18);
  FB318_16 = s->m[18]*(AlM317_16*C18-s->dpt[2][21]*S18);
  CM118_16 = -s->In[1][18]*C17*S18;
  CM218_16 = s->In[5][18]*S17;
  CM318_16 = s->In[9][18]*C17*C18;
  FF17_118 = FA118*C18+FA318*S18;
  FF17_318 = -(FA118*S18-FA318*C18);
  CF17_118 = CF118*C18+CF318*S18;
  CF17_318 = -(CF118*S18-CF318*C18);
  FM171_118 = FB118_1*C18+FB318_1*S18;
  FM171_318 = -(FB118_1*S18-FB318_1*C18);
  CM171_118 = CM118_1*C18+CM318_1*S18;
  CM171_318 = -(CM118_1*S18-CM318_1*C18);
  FM172_118 = FB118_2*C18+FB318_2*S18;
  FM172_318 = -(FB118_2*S18-FB318_2*C18);
  FM173_118 = FB118_3*C18+FB318_3*S18;
  FM173_318 = -(FB118_3*S18-FB318_3*C18);
  FM174_118 = FB118_4*C18+FB318_4*S18;
  FM174_318 = -(FB118_4*S18-FB318_4*C18);
  FM175_118 = FB118_5*C18+FB318_5*S18;
  FM175_318 = -(FB118_5*S18-FB318_5*C18);
  CM175_118 = CM118_5*C18+CM318_5*S18;
  CM175_318 = -(CM118_5*S18-CM318_5*C18);
  FM176_118 = FB118_6*C18+FB318_6*S18;
  FM176_318 = -(FB118_6*S18-FB318_6*C18);
  CM176_118 = CM118_6*C18+CM318_6*S18;
  CM176_318 = -(CM118_6*S18-CM318_6*C18);
  FM177_118 = FB118_7*C18+FB318_7*S18;
  FM177_318 = -(FB118_7*S18-FB318_7*C18);
  CM177_118 = CM118_7*C18+CM318_7*S18;
  CM177_318 = -(CM118_7*S18-CM318_7*C18);
  FM1711_118 = FB118_11*C18+FB318_11*S18;
  FM1711_318 = -(FB118_11*S18-FB318_11*C18);
  CM1711_118 = CM118_11*C18+CM318_11*S18;
  CM1711_318 = -(CM118_11*S18-CM318_11*C18);
  FM1713_118 = FB118_13*C18+FB318_13*S18;
  FM1713_318 = -(FB118_13*S18-FB318_13*C18);
  CM1713_118 = CM118_13*C18+CM318_13*S18;
  CM1713_318 = -(CM118_13*S18-CM318_13*C18);
  FM1714_118 = FB118_14*C18+FB318_14*S18;
  FM1714_318 = -(FB118_14*S18-FB318_14*C18);
  CM1714_118 = CM118_14*C18+CM318_14*S18;
  CM1714_318 = -(CM118_14*S18-CM318_14*C18);
  FM1715_118 = FB118_15*C18+FB318_15*S18;
  FM1715_318 = -(FB118_15*S18-FB318_15*C18);
  CM1715_118 = CM118_15*C18+CM318_15*S18;
  CM1715_318 = -(CM118_15*S18-CM318_15*C18);
  CM1716_118 = CM118_16*C18+CM318_16*S18;
  CM1717_118 = s->In[1][18]*C18*C18+s->In[9][18]*S18*S18;
  FF116 = -(s->frc[1][16]-FF17_118);
  FF216 = -(s->frc[2][16]-FA218*C17+FF17_318*S17);
  FF316 = -(s->frc[3][16]-FA218*S17-FF17_318*C17);
  CF116 = -(s->trq[1][16]-CF17_118-s->dpt[2][21]*(FA218*S17+FF17_318*C17)+s->dpt[3][21]*(FA218*C17-FF17_318*S17));
  CF216 = -(s->trq[2][16]+CF17_318*S17-CF218*C17-FF17_118*s->dpt[3][21]+s->dpt[1][21]*(FA218*S17+FF17_318*C17));
  CF316 = -(s->trq[3][16]-CF17_318*C17-CF218*S17+FF17_118*s->dpt[2][21]-s->dpt[1][21]*(FA218*C17-FF17_318*S17));
  FM161_217 = FB218_1*C17-FM171_318*S17;
  FM161_317 = FB218_1*S17+FM171_318*C17;
  CM161_117 = CM171_118+s->dpt[2][21]*(FB218_1*S17+FM171_318*C17)-s->dpt[3][21]*(FB218_1*C17-FM171_318*S17);
  CM161_217 = -(CM171_318*S17-CM218_1*C17-FM171_118*s->dpt[3][21]+s->dpt[1][21]*(FB218_1*S17+FM171_318*C17));
  CM161_317 = CM171_318*C17+CM218_1*S17-FM171_118*s->dpt[2][21]+s->dpt[1][21]*(FB218_1*C17-FM171_318*S17);
  FM162_217 = FB218_2*C17-FM172_318*S17;
  FM162_317 = FB218_2*S17+FM172_318*C17;
  CM162_117 = s->dpt[2][21]*(FB218_2*S17+FM172_318*C17)-s->dpt[3][21]*(FB218_2*C17-FM172_318*S17);
  CM162_217 = FM172_118*s->dpt[3][21]-s->dpt[1][21]*(FB218_2*S17+FM172_318*C17);
  CM162_317 = -(FM172_118*s->dpt[2][21]-s->dpt[1][21]*(FB218_2*C17-FM172_318*S17));
  FM163_217 = FB218_3*C17-FM173_318*S17;
  FM163_317 = FB218_3*S17+FM173_318*C17;
  CM163_117 = s->dpt[2][21]*(FB218_3*S17+FM173_318*C17)-s->dpt[3][21]*(FB218_3*C17-FM173_318*S17);
  CM163_217 = FM173_118*s->dpt[3][21]-s->dpt[1][21]*(FB218_3*S17+FM173_318*C17);
  CM163_317 = -(FM173_118*s->dpt[2][21]-s->dpt[1][21]*(FB218_3*C17-FM173_318*S17));
  FM164_217 = FB218_4*C17-FM174_318*S17;
  FM164_317 = FB218_4*S17+FM174_318*C17;
  CM164_117 = s->dpt[2][21]*(FB218_4*S17+FM174_318*C17)-s->dpt[3][21]*(FB218_4*C17-FM174_318*S17);
  CM164_217 = FM174_118*s->dpt[3][21]-s->dpt[1][21]*(FB218_4*S17+FM174_318*C17);
  CM164_317 = -(FM174_118*s->dpt[2][21]-s->dpt[1][21]*(FB218_4*C17-FM174_318*S17));
  FM165_217 = FB218_5*C17-FM175_318*S17;
  FM165_317 = FB218_5*S17+FM175_318*C17;
  CM165_117 = CM175_118+s->dpt[2][21]*(FB218_5*S17+FM175_318*C17)-s->dpt[3][21]*(FB218_5*C17-FM175_318*S17);
  CM165_217 = -(CM175_318*S17-CM218_5*C17-FM175_118*s->dpt[3][21]+s->dpt[1][21]*(FB218_5*S17+FM175_318*C17));
  CM165_317 = CM175_318*C17+CM218_5*S17-FM175_118*s->dpt[2][21]+s->dpt[1][21]*(FB218_5*C17-FM175_318*S17);
  FM166_217 = FB218_6*C17-FM176_318*S17;
  FM166_317 = FB218_6*S17+FM176_318*C17;
  CM166_117 = CM176_118+s->dpt[2][21]*(FB218_6*S17+FM176_318*C17)-s->dpt[3][21]*(FB218_6*C17-FM176_318*S17);
  CM166_217 = -(CM176_318*S17-CM218_6*C17-FM176_118*s->dpt[3][21]+s->dpt[1][21]*(FB218_6*S17+FM176_318*C17));
  CM166_317 = CM176_318*C17+CM218_6*S17-FM176_118*s->dpt[2][21]+s->dpt[1][21]*(FB218_6*C17-FM176_318*S17);
  FM167_217 = FB218_7*C17-FM177_318*S17;
  FM167_317 = FB218_7*S17+FM177_318*C17;
  CM167_117 = CM177_118+s->dpt[2][21]*(FB218_7*S17+FM177_318*C17)-s->dpt[3][21]*(FB218_7*C17-FM177_318*S17);
  CM167_217 = -(CM177_318*S17-CM218_7*C17-FM177_118*s->dpt[3][21]+s->dpt[1][21]*(FB218_7*S17+FM177_318*C17));
  CM167_317 = CM177_318*C17+CM218_7*S17-FM177_118*s->dpt[2][21]+s->dpt[1][21]*(FB218_7*C17-FM177_318*S17);
  FM1611_217 = FB218_11*C17-FM1711_318*S17;
  FM1611_317 = FB218_11*S17+FM1711_318*C17;
  CM1611_117 = CM1711_118+s->dpt[2][21]*(FB218_11*S17+FM1711_318*C17)-s->dpt[3][21]*(FB218_11*C17-FM1711_318*S17);
  CM1611_217 = -(CM1711_318*S17-CM218_11*C17-FM1711_118*s->dpt[3][21]+s->dpt[1][21]*(FB218_11*S17+FM1711_318*C17));
  CM1611_317 = CM1711_318*C17+CM218_11*S17-FM1711_118*s->dpt[2][21]+s->dpt[1][21]*(FB218_11*C17-FM1711_318*S17);
  FM1613_217 = FB218_13*C17-FM1713_318*S17;
  CM1613_117 = CM1713_118+s->dpt[2][21]*(FB218_13*S17+FM1713_318*C17)-s->dpt[3][21]*(FB218_13*C17-FM1713_318*S17);
  CM1613_217 = -(CM1713_318*S17-CM218_13*C17-FM1713_118*s->dpt[3][21]+s->dpt[1][21]*(FB218_13*S17+FM1713_318*C17));
  CM1613_317 = CM1713_318*C17+CM218_13*S17-FM1713_118*s->dpt[2][21]+s->dpt[1][21]*(FB218_13*C17-FM1713_318*S17);
  CM1614_117 = CM1714_118+s->dpt[2][21]*(FB218_14*S17+FM1714_318*C17)-s->dpt[3][21]*(FB218_14*C17-FM1714_318*S17);
  CM1614_217 = -(CM1714_318*S17-CM218_14*C17-FM1714_118*s->dpt[3][21]+s->dpt[1][21]*(FB218_14*S17+FM1714_318*C17));
  CM1614_317 = CM1714_318*C17+CM218_14*S17-FM1714_118*s->dpt[2][21]+s->dpt[1][21]*(FB218_14*C17-FM1714_318*S17);
  CM1615_317 = CM1715_318*C17+CM218_15*S17-FM1715_118*s->dpt[2][21]+s->dpt[1][21]*(FB218_15*C17-FM1715_318*S17);
  CM1616_317 = CM218_16*S17+s->dpt[1][21]*(s->m[18]*s->dpt[1][21]*C17*C17+S17*(FB118_16*S18-FB318_16*C18))-s->dpt[2][21]
 *(FB118_16*C18+FB318_16*S18)-C17*(CM118_16*S18-CM318_16*C18);
  FF15_116 = FF116*C16-FF216*S16;
  FF15_216 = FF116*S16+FF216*C16;
  CF15_116 = CF116*C16-CF216*S16;
  CF15_216 = CF116*S16+CF216*C16;
  FM151_116 = -(FM161_217*S16-FM171_118*C16);
  FM151_216 = FM161_217*C16+FM171_118*S16;
  CM151_116 = CM161_117*C16-CM161_217*S16;
  CM151_216 = CM161_117*S16+CM161_217*C16;
  FM152_116 = -(FM162_217*S16-FM172_118*C16);
  FM152_216 = FM162_217*C16+FM172_118*S16;
  CM152_116 = CM162_117*C16-CM162_217*S16;
  CM152_216 = CM162_117*S16+CM162_217*C16;
  FM153_116 = -(FM163_217*S16-FM173_118*C16);
  FM153_216 = FM163_217*C16+FM173_118*S16;
  CM153_116 = CM163_117*C16-CM163_217*S16;
  CM153_216 = CM163_117*S16+CM163_217*C16;
  FM154_116 = -(FM164_217*S16-FM174_118*C16);
  FM154_216 = FM164_217*C16+FM174_118*S16;
  CM154_116 = CM164_117*C16-CM164_217*S16;
  CM154_216 = CM164_117*S16+CM164_217*C16;
  FM155_116 = -(FM165_217*S16-FM175_118*C16);
  FM155_216 = FM165_217*C16+FM175_118*S16;
  CM155_116 = CM165_117*C16-CM165_217*S16;
  CM155_216 = CM165_117*S16+CM165_217*C16;
  FM156_116 = -(FM166_217*S16-FM176_118*C16);
  FM156_216 = FM166_217*C16+FM176_118*S16;
  CM156_116 = CM166_117*C16-CM166_217*S16;
  CM156_216 = CM166_117*S16+CM166_217*C16;
  FM157_116 = -(FM167_217*S16-FM177_118*C16);
  FM157_216 = FM167_217*C16+FM177_118*S16;
  CM157_116 = CM167_117*C16-CM167_217*S16;
  CM157_216 = CM167_117*S16+CM167_217*C16;
  FM1511_116 = -(FM1611_217*S16-FM1711_118*C16);
  FM1511_216 = FM1611_217*C16+FM1711_118*S16;
  CM1511_116 = CM1611_117*C16-CM1611_217*S16;
  CM1511_216 = CM1611_117*S16+CM1611_217*C16;
  FM1513_216 = FM1613_217*C16+FM1713_118*S16;
  CM1513_216 = CM1613_117*S16+CM1613_217*C16;
  CM1514_216 = CM1614_117*S16+CM1614_217*C16;
  CM1515_216 = -(C16*(CM1715_318*S17-CM218_15*C17-FM1715_118*s->dpt[3][21]+s->dpt[1][21]*(FB218_15*S17+FM1715_318*C17))-
 S16*(CM1715_118+s->dpt[2][21]*(FB218_15*S17+FM1715_318*C17)-s->dpt[3][21]*(FB218_15*C17-FM1715_318*S17)));
  FF14_115 = FF15_116*C15+FF316*S15;
  FF14_315 = -(FF15_116*S15-FF316*C15);
  CF14_115 = CF15_116*C15+CF316*S15;
  CF14_315 = -(CF15_116*S15-CF316*C15);
  FM141_115 = FM151_116*C15+FM161_317*S15;
  FM141_315 = -(FM151_116*S15-FM161_317*C15);
  CM141_115 = CM151_116*C15+CM161_317*S15;
  CM141_315 = -(CM151_116*S15-CM161_317*C15);
  FM142_115 = FM152_116*C15+FM162_317*S15;
  FM142_315 = -(FM152_116*S15-FM162_317*C15);
  CM142_115 = CM152_116*C15+CM162_317*S15;
  CM142_315 = -(CM152_116*S15-CM162_317*C15);
  FM143_115 = FM153_116*C15+FM163_317*S15;
  FM143_315 = -(FM153_116*S15-FM163_317*C15);
  CM143_115 = CM153_116*C15+CM163_317*S15;
  CM143_315 = -(CM153_116*S15-CM163_317*C15);
  FM144_115 = FM154_116*C15+FM164_317*S15;
  FM144_315 = -(FM154_116*S15-FM164_317*C15);
  CM144_115 = CM154_116*C15+CM164_317*S15;
  CM144_315 = -(CM154_116*S15-CM164_317*C15);
  FM145_115 = FM155_116*C15+FM165_317*S15;
  FM145_315 = -(FM155_116*S15-FM165_317*C15);
  CM145_115 = CM155_116*C15+CM165_317*S15;
  CM145_315 = -(CM155_116*S15-CM165_317*C15);
  FM146_115 = FM156_116*C15+FM166_317*S15;
  FM146_315 = -(FM156_116*S15-FM166_317*C15);
  CM146_115 = CM156_116*C15+CM166_317*S15;
  CM146_315 = -(CM156_116*S15-CM166_317*C15);
  FM147_115 = FM157_116*C15+FM167_317*S15;
  FM147_315 = -(FM157_116*S15-FM167_317*C15);
  CM147_115 = CM157_116*C15+CM167_317*S15;
  CM147_315 = -(CM157_116*S15-CM167_317*C15);
  FM1411_115 = FM1511_116*C15+FM1611_317*S15;
  FM1411_315 = -(FM1511_116*S15-FM1611_317*C15);
  CM1411_115 = CM1511_116*C15+CM1611_317*S15;
  CM1411_315 = -(CM1511_116*S15-CM1611_317*C15);
  FM1413_315 = C15*(FB218_13*S17+FM1713_318*C17)+S15*(FM1613_217*S16-FM1713_118*C16);
  CM1413_115 = CM1613_317*S15+C15*(CM1613_117*C16-CM1613_217*S16);
  CM1414_115 = CM1614_317*S15+C15*(CM1614_117*C16-CM1614_217*S16);
  FF113 = -(s->frc[1][13]-FF14_115);
  FF213 = -(s->frc[2][13]+FF14_315*S14-FF15_216*C14);
  FF313 = -(s->frc[3][13]-FF14_315*C14-FF15_216*S14);
  CF113 = -(s->trq[1][13]-CF14_115-s->dpt[2][17]*(FF14_315*C14+FF15_216*S14)-s->dpt[3][17]*(FF14_315*S14-FF15_216*C14));
  CF213 = -(s->trq[2][13]+CF14_315*S14-CF15_216*C14-FF14_115*s->dpt[3][17]);
  CF313 = -(s->trq[3][13]-CF14_315*C14-CF15_216*S14+FF14_115*s->dpt[2][17]);
  FM131_214 = -(FM141_315*S14-FM151_216*C14);
  FM131_314 = FM141_315*C14+FM151_216*S14;
  CM131_114 = CM141_115+s->dpt[2][17]*(FM141_315*C14+FM151_216*S14)+s->dpt[3][17]*(FM141_315*S14-FM151_216*C14);
  CM131_214 = -(CM141_315*S14-CM151_216*C14-FM141_115*s->dpt[3][17]);
  CM131_314 = CM141_315*C14+CM151_216*S14-FM141_115*s->dpt[2][17];
  FM132_214 = -(FM142_315*S14-FM152_216*C14);
  FM132_314 = FM142_315*C14+FM152_216*S14;
  CM132_114 = CM142_115+s->dpt[2][17]*(FM142_315*C14+FM152_216*S14)+s->dpt[3][17]*(FM142_315*S14-FM152_216*C14);
  CM132_214 = -(CM142_315*S14-CM152_216*C14-FM142_115*s->dpt[3][17]);
  CM132_314 = CM142_315*C14+CM152_216*S14-FM142_115*s->dpt[2][17];
  FM133_214 = -(FM143_315*S14-FM153_216*C14);
  FM133_314 = FM143_315*C14+FM153_216*S14;
  CM133_114 = CM143_115+s->dpt[2][17]*(FM143_315*C14+FM153_216*S14)+s->dpt[3][17]*(FM143_315*S14-FM153_216*C14);
  CM133_214 = -(CM143_315*S14-CM153_216*C14-FM143_115*s->dpt[3][17]);
  CM133_314 = CM143_315*C14+CM153_216*S14-FM143_115*s->dpt[2][17];
  FM134_214 = -(FM144_315*S14-FM154_216*C14);
  FM134_314 = FM144_315*C14+FM154_216*S14;
  CM134_114 = CM144_115+s->dpt[2][17]*(FM144_315*C14+FM154_216*S14)+s->dpt[3][17]*(FM144_315*S14-FM154_216*C14);
  CM134_214 = -(CM144_315*S14-CM154_216*C14-FM144_115*s->dpt[3][17]);
  CM134_314 = CM144_315*C14+CM154_216*S14-FM144_115*s->dpt[2][17];
  FM135_214 = -(FM145_315*S14-FM155_216*C14);
  FM135_314 = FM145_315*C14+FM155_216*S14;
  CM135_114 = CM145_115+s->dpt[2][17]*(FM145_315*C14+FM155_216*S14)+s->dpt[3][17]*(FM145_315*S14-FM155_216*C14);
  CM135_214 = -(CM145_315*S14-CM155_216*C14-FM145_115*s->dpt[3][17]);
  CM135_314 = CM145_315*C14+CM155_216*S14-FM145_115*s->dpt[2][17];
  FM136_214 = -(FM146_315*S14-FM156_216*C14);
  FM136_314 = FM146_315*C14+FM156_216*S14;
  CM136_114 = CM146_115+s->dpt[2][17]*(FM146_315*C14+FM156_216*S14)+s->dpt[3][17]*(FM146_315*S14-FM156_216*C14);
  CM136_214 = -(CM146_315*S14-CM156_216*C14-FM146_115*s->dpt[3][17]);
  CM136_314 = CM146_315*C14+CM156_216*S14-FM146_115*s->dpt[2][17];
  CM137_114 = CM147_115+s->dpt[2][17]*(FM147_315*C14+FM157_216*S14)+s->dpt[3][17]*(FM147_315*S14-FM157_216*C14);
  CM1311_114 = CM1411_115+s->dpt[2][17]*(FM1411_315*C14+FM1511_216*S14)+s->dpt[3][17]*(FM1411_315*S14-FM1511_216*C14);
  CM1313_114 = CM1413_115+s->dpt[2][17]*(FM1413_315*C14+FM1513_216*S14)+s->dpt[3][17]*(FM1413_315*S14-FM1513_216*C14);

// = = Block_0_2_0_1_0_6 = = 
 
// Backward Dynamics 

  FA124 = -(s->frc[1][24]-s->m[24]*(AlF123*C24-AlF323*S24));
  FA224 = -(s->frc[2][24]-s->m[24]*(C23*(AlF222+BS522*s->dpt[2][28]+BeF422*s->dpt[1][28]+BeF622*s->dpt[3][28])+S23*(
 AlF321+BS922*s->dpt[3][28]+BeF722*s->dpt[1][28]+BeF822*s->dpt[2][28])));
  FA324 = -(s->frc[3][24]-s->m[24]*(AlF123*S24+AlF323*C24));
  CF124 = -(s->trq[1][24]-s->In[1][24]*(C24*(OpF122-qd[24]*OM323)-S24*(OpF323+qd[24]*OM122))+OM224*OM324*(s->In[5][24]-
 s->In[9][24]));
  CF224 = -(s->trq[2][24]-s->In[5][24]*(OpF222*C23+OpF321*S23)-OM124*OM324*(s->In[1][24]-s->In[9][24]));
  CF324 = -(s->trq[3][24]-s->In[9][24]*(C24*(OpF323+qd[24]*OM122)+S24*(OpF122-qd[24]*OM323))+OM124*OM224*(s->In[1][24]-
 s->In[5][24]));
  FB124_1 = s->m[24]*(AlM123_1*C24-AlM323_1*S24);
  FB224_1 = s->m[24]*(C23*(AlM222_1-OpM122_1*s->dpt[3][28]+OpM321_1*s->dpt[1][28])+S23*(AlM321_1+OpM122_1*s->dpt[2][28]-
 OpM222_1*s->dpt[1][28]));
  FB324_1 = s->m[24]*(AlM123_1*S24+AlM323_1*C24);
  CM124_1 = s->In[1][24]*(OpM122_1*C24-OpM323_1*S24);
  CM224_1 = s->In[5][24]*(OpM222_1*C23+OpM321_1*S23);
  CM324_1 = s->In[9][24]*(OpM122_1*S24+OpM323_1*C24);
  FB124_2 = s->m[24]*(AlM122_2*C24-AlM323_2*S24);
  FB224_2 = s->m[24]*(AlM222_2*C23+AlM321_2*S23);
  FB324_2 = s->m[24]*(AlM122_2*S24+AlM323_2*C24);
  FB124_3 = s->m[24]*(AlM122_3*C24-AlM323_3*S24);
  FB224_3 = s->m[24]*(AlM222_3*C23+AlM321_3*S23);
  FB324_3 = s->m[24]*(AlM122_3*S24+AlM323_3*C24);
  FB124_4 = s->m[24]*(AlM122_4*C24-AlM323_4*S24);
  FB224_4 = s->m[24]*(AlM222_4*C23+AlM321_4*S23);
  FB324_4 = s->m[24]*(AlM122_4*S24+AlM323_4*C24);
  FB124_5 = s->m[24]*(AlM123_5*C24-AlM323_5*S24);
  FB224_5 = s->m[24]*(C23*(AlM222_5-OpM122_5*s->dpt[3][28]+OpM321_5*s->dpt[1][28])+S23*(AlM321_5+OpM122_5*s->dpt[2][28]-
 OpM222_5*s->dpt[1][28]));
  FB324_5 = s->m[24]*(AlM123_5*S24+AlM323_5*C24);
  CM124_5 = s->In[1][24]*(OpM122_5*C24-OpM323_5*S24);
  CM224_5 = s->In[5][24]*(OpM222_5*C23+OpM321_5*S23);
  CM324_5 = s->In[9][24]*(OpM122_5*S24+OpM323_5*C24);
  FB124_6 = s->m[24]*(AlM123_6*C24-AlM323_6*S24);
  FB224_6 = s->m[24]*(C23*(AlM222_6-OpM122_6*s->dpt[3][28]+OpM321_6*s->dpt[1][28])+S23*(AlM321_6+OpM122_6*s->dpt[2][28]-
 OpM222_6*s->dpt[1][28]));
  FB324_6 = s->m[24]*(AlM123_6*S24+AlM323_6*C24);
  CM124_6 = s->In[1][24]*(OpM122_6*C24-OpM323_6*S24);
  CM224_6 = s->In[5][24]*(OpM222_6*C23+OpM321_6*S23);
  CM324_6 = s->In[9][24]*(OpM122_6*S24+OpM323_6*C24);
  FB124_7 = s->m[24]*(AlM123_7*C24-AlM323_7*S24);
  FB224_7 = s->m[24]*(C23*(AlM222_7-OpM122_7*s->dpt[3][28]+OpM321_7*s->dpt[1][28])+S23*(AlM321_7+OpM122_7*s->dpt[2][28]-
 OpM222_7*s->dpt[1][28]));
  FB324_7 = s->m[24]*(AlM123_7*S24+AlM323_7*C24);
  CM124_7 = s->In[1][24]*(OpM122_7*C24-OpM323_7*S24);
  CM224_7 = s->In[5][24]*(OpM222_7*C23+OpM321_7*S23);
  CM324_7 = s->In[9][24]*(OpM122_7*S24+OpM323_7*C24);
  FB124_11 = s->m[24]*(AlM123_11*C24-AlM323_11*S24);
  FB224_11 = s->m[24]*(C23*(AlM222_11-OpM122_11*s->dpt[3][28]+OpM321_11*s->dpt[1][28])+S23*(AlM321_11+OpM122_11*
 s->dpt[2][28]-OpM222_11*s->dpt[1][28]));
  FB324_11 = s->m[24]*(AlM123_11*S24+AlM323_11*C24);
  CM124_11 = s->In[1][24]*(OpM122_11*C24-OpM323_11*S24);
  CM224_11 = s->In[5][24]*(OpM222_11*C23+OpM321_11*S23);
  CM324_11 = s->In[9][24]*(OpM122_11*S24+OpM323_11*C24);
  FB124_19 = s->m[24]*(AlM123_19*C24-AlM323_19*S24);
  FB224_19 = s->m[24]*(C23*(AlM222_19-OpM122_19*s->dpt[3][28]+s->dpt[1][28]*S21)+S23*(AlM321_19+OpM122_19*s->dpt[2][28]-
 OpM222_19*s->dpt[1][28]));
  FB324_19 = s->m[24]*(AlM123_19*S24+AlM323_19*C24);
  CM124_19 = s->In[1][24]*(OpM122_19*C24-OpM323_19*S24);
  CM224_19 = s->In[5][24]*(OpM222_19*C23+S21*S23);
  CM324_19 = s->In[9][24]*(OpM122_19*S24+OpM323_19*C24);
  FB124_20 = s->m[24]*(AlM123_20*C24-AlM323_20*S24);
  FB224_20 = -s->m[24]*(C23*(OpM122_20*s->dpt[3][28]-s->dpt[1][28]*S21)-S23*(OpM122_20*s->dpt[2][28]-OpM222_20*
 s->dpt[1][28]));
  FB324_20 = s->m[24]*(AlM123_20*S24+AlM323_20*C24);
  CM124_20 = s->In[1][24]*(OpM122_20*C24-OpM323_20*S24);
  CM224_20 = s->In[5][24]*(OpM222_20*C23+S21*S23);
  CM324_20 = s->In[9][24]*(OpM122_20*S24+OpM323_20*C24);
  FB124_21 = s->m[24]*(AlM123_21*C24-AlM323_21*S24);
  FB224_21 = -s->m[24]*(s->dpt[3][28]*S22*C23+S23*(s->dpt[1][28]*C22-s->dpt[2][28]*S22));
  FB324_21 = s->m[24]*(AlM123_21*S24+AlM323_21*C24);
  CM124_21 = -s->In[1][24]*(OpM323_21*S24-S22*C24);
  CM224_21 = s->In[5][24]*C22*C23;
  CM324_21 = s->In[9][24]*(OpM323_21*C24+S22*S24);
  FB124_22 = -s->m[24]*(AlM323_22*S24+s->dpt[2][28]*C24);
  FB324_22 = s->m[24]*(AlM323_22*C24-s->dpt[2][28]*S24);
  CM124_22 = -s->In[1][24]*C23*S24;
  CM224_22 = s->In[5][24]*S23;
  CM324_22 = s->In[9][24]*C23*C24;
  FF23_124 = FA124*C24+FA324*S24;
  FF23_324 = -(FA124*S24-FA324*C24);
  CF23_124 = CF124*C24+CF324*S24;
  CF23_324 = -(CF124*S24-CF324*C24);
  FM231_124 = FB124_1*C24+FB324_1*S24;
  FM231_324 = -(FB124_1*S24-FB324_1*C24);
  CM231_124 = CM124_1*C24+CM324_1*S24;
  CM231_324 = -(CM124_1*S24-CM324_1*C24);
  FM232_124 = FB124_2*C24+FB324_2*S24;
  FM232_324 = -(FB124_2*S24-FB324_2*C24);
  FM233_124 = FB124_3*C24+FB324_3*S24;
  FM233_324 = -(FB124_3*S24-FB324_3*C24);
  FM234_124 = FB124_4*C24+FB324_4*S24;
  FM234_324 = -(FB124_4*S24-FB324_4*C24);
  FM235_124 = FB124_5*C24+FB324_5*S24;
  FM235_324 = -(FB124_5*S24-FB324_5*C24);
  CM235_124 = CM124_5*C24+CM324_5*S24;
  CM235_324 = -(CM124_5*S24-CM324_5*C24);
  FM236_124 = FB124_6*C24+FB324_6*S24;
  FM236_324 = -(FB124_6*S24-FB324_6*C24);
  CM236_124 = CM124_6*C24+CM324_6*S24;
  CM236_324 = -(CM124_6*S24-CM324_6*C24);
  FM237_124 = FB124_7*C24+FB324_7*S24;
  FM237_324 = -(FB124_7*S24-FB324_7*C24);
  CM237_124 = CM124_7*C24+CM324_7*S24;
  CM237_324 = -(CM124_7*S24-CM324_7*C24);
  FM2311_124 = FB124_11*C24+FB324_11*S24;
  FM2311_324 = -(FB124_11*S24-FB324_11*C24);
  CM2311_124 = CM124_11*C24+CM324_11*S24;
  CM2311_324 = -(CM124_11*S24-CM324_11*C24);
  FM2319_124 = FB124_19*C24+FB324_19*S24;
  FM2319_324 = -(FB124_19*S24-FB324_19*C24);
  CM2319_124 = CM124_19*C24+CM324_19*S24;
  CM2319_324 = -(CM124_19*S24-CM324_19*C24);
  FM2320_124 = FB124_20*C24+FB324_20*S24;
  FM2320_324 = -(FB124_20*S24-FB324_20*C24);
  CM2320_124 = CM124_20*C24+CM324_20*S24;
  CM2320_324 = -(CM124_20*S24-CM324_20*C24);
  FM2321_124 = FB124_21*C24+FB324_21*S24;
  FM2321_324 = -(FB124_21*S24-FB324_21*C24);
  CM2321_124 = CM124_21*C24+CM324_21*S24;
  CM2321_324 = -(CM124_21*S24-CM324_21*C24);
  CM2322_124 = CM124_22*C24+CM324_22*S24;
  CM2323_124 = s->In[1][24]*C24*C24+s->In[9][24]*S24*S24;
  FF122 = -(s->frc[1][22]-FF23_124);
  FF222 = -(s->frc[2][22]-FA224*C23+FF23_324*S23);
  FF322 = -(s->frc[3][22]-FA224*S23-FF23_324*C23);
  CF122 = -(s->trq[1][22]-CF23_124-s->dpt[2][28]*(FA224*S23+FF23_324*C23)+s->dpt[3][28]*(FA224*C23-FF23_324*S23));
  CF222 = -(s->trq[2][22]-CF224*C23+CF23_324*S23-FF23_124*s->dpt[3][28]+s->dpt[1][28]*(FA224*S23+FF23_324*C23));
  CF322 = -(s->trq[3][22]-CF224*S23-CF23_324*C23+FF23_124*s->dpt[2][28]-s->dpt[1][28]*(FA224*C23-FF23_324*S23));
  FM221_223 = FB224_1*C23-FM231_324*S23;
  FM221_323 = FB224_1*S23+FM231_324*C23;
  CM221_123 = CM231_124+s->dpt[2][28]*(FB224_1*S23+FM231_324*C23)-s->dpt[3][28]*(FB224_1*C23-FM231_324*S23);
  CM221_223 = CM224_1*C23-CM231_324*S23+FM231_124*s->dpt[3][28]-s->dpt[1][28]*(FB224_1*S23+FM231_324*C23);
  CM221_323 = CM224_1*S23+CM231_324*C23-FM231_124*s->dpt[2][28]+s->dpt[1][28]*(FB224_1*C23-FM231_324*S23);
  FM222_223 = FB224_2*C23-FM232_324*S23;
  FM222_323 = FB224_2*S23+FM232_324*C23;
  CM222_123 = s->dpt[2][28]*(FB224_2*S23+FM232_324*C23)-s->dpt[3][28]*(FB224_2*C23-FM232_324*S23);
  CM222_223 = FM232_124*s->dpt[3][28]-s->dpt[1][28]*(FB224_2*S23+FM232_324*C23);
  CM222_323 = -(FM232_124*s->dpt[2][28]-s->dpt[1][28]*(FB224_2*C23-FM232_324*S23));
  FM223_223 = FB224_3*C23-FM233_324*S23;
  FM223_323 = FB224_3*S23+FM233_324*C23;
  CM223_123 = s->dpt[2][28]*(FB224_3*S23+FM233_324*C23)-s->dpt[3][28]*(FB224_3*C23-FM233_324*S23);
  CM223_223 = FM233_124*s->dpt[3][28]-s->dpt[1][28]*(FB224_3*S23+FM233_324*C23);
  CM223_323 = -(FM233_124*s->dpt[2][28]-s->dpt[1][28]*(FB224_3*C23-FM233_324*S23));
  FM224_223 = FB224_4*C23-FM234_324*S23;
  FM224_323 = FB224_4*S23+FM234_324*C23;
  CM224_123 = s->dpt[2][28]*(FB224_4*S23+FM234_324*C23)-s->dpt[3][28]*(FB224_4*C23-FM234_324*S23);
  CM224_223 = FM234_124*s->dpt[3][28]-s->dpt[1][28]*(FB224_4*S23+FM234_324*C23);
  CM224_323 = -(FM234_124*s->dpt[2][28]-s->dpt[1][28]*(FB224_4*C23-FM234_324*S23));
  FM225_223 = FB224_5*C23-FM235_324*S23;
  FM225_323 = FB224_5*S23+FM235_324*C23;
  CM225_123 = CM235_124+s->dpt[2][28]*(FB224_5*S23+FM235_324*C23)-s->dpt[3][28]*(FB224_5*C23-FM235_324*S23);
  CM225_223 = CM224_5*C23-CM235_324*S23+FM235_124*s->dpt[3][28]-s->dpt[1][28]*(FB224_5*S23+FM235_324*C23);
  CM225_323 = CM224_5*S23+CM235_324*C23-FM235_124*s->dpt[2][28]+s->dpt[1][28]*(FB224_5*C23-FM235_324*S23);
  FM226_223 = FB224_6*C23-FM236_324*S23;
  FM226_323 = FB224_6*S23+FM236_324*C23;
  CM226_123 = CM236_124+s->dpt[2][28]*(FB224_6*S23+FM236_324*C23)-s->dpt[3][28]*(FB224_6*C23-FM236_324*S23);
  CM226_223 = CM224_6*C23-CM236_324*S23+FM236_124*s->dpt[3][28]-s->dpt[1][28]*(FB224_6*S23+FM236_324*C23);
  CM226_323 = CM224_6*S23+CM236_324*C23-FM236_124*s->dpt[2][28]+s->dpt[1][28]*(FB224_6*C23-FM236_324*S23);
  FM227_223 = FB224_7*C23-FM237_324*S23;
  FM227_323 = FB224_7*S23+FM237_324*C23;
  CM227_123 = CM237_124+s->dpt[2][28]*(FB224_7*S23+FM237_324*C23)-s->dpt[3][28]*(FB224_7*C23-FM237_324*S23);
  CM227_223 = CM224_7*C23-CM237_324*S23+FM237_124*s->dpt[3][28]-s->dpt[1][28]*(FB224_7*S23+FM237_324*C23);
  CM227_323 = CM224_7*S23+CM237_324*C23-FM237_124*s->dpt[2][28]+s->dpt[1][28]*(FB224_7*C23-FM237_324*S23);
  FM2211_223 = FB224_11*C23-FM2311_324*S23;
  FM2211_323 = FB224_11*S23+FM2311_324*C23;
  CM2211_123 = CM2311_124+s->dpt[2][28]*(FB224_11*S23+FM2311_324*C23)-s->dpt[3][28]*(FB224_11*C23-FM2311_324*S23);
  CM2211_223 = CM224_11*C23-CM2311_324*S23+FM2311_124*s->dpt[3][28]-s->dpt[1][28]*(FB224_11*S23+FM2311_324*C23);
  CM2211_323 = CM224_11*S23+CM2311_324*C23-FM2311_124*s->dpt[2][28]+s->dpt[1][28]*(FB224_11*C23-FM2311_324*S23);
  FM2219_223 = FB224_19*C23-FM2319_324*S23;
  CM2219_123 = CM2319_124+s->dpt[2][28]*(FB224_19*S23+FM2319_324*C23)-s->dpt[3][28]*(FB224_19*C23-FM2319_324*S23);
  CM2219_223 = CM224_19*C23-CM2319_324*S23+FM2319_124*s->dpt[3][28]-s->dpt[1][28]*(FB224_19*S23+FM2319_324*C23);
  CM2219_323 = CM224_19*S23+CM2319_324*C23-FM2319_124*s->dpt[2][28]+s->dpt[1][28]*(FB224_19*C23-FM2319_324*S23);
  CM2220_123 = CM2320_124+s->dpt[2][28]*(FB224_20*S23+FM2320_324*C23)-s->dpt[3][28]*(FB224_20*C23-FM2320_324*S23);
  CM2220_223 = CM224_20*C23-CM2320_324*S23+FM2320_124*s->dpt[3][28]-s->dpt[1][28]*(FB224_20*S23+FM2320_324*C23);
  CM2220_323 = CM224_20*S23+CM2320_324*C23-FM2320_124*s->dpt[2][28]+s->dpt[1][28]*(FB224_20*C23-FM2320_324*S23);
  CM2221_323 = CM224_21*S23+CM2321_324*C23-FM2321_124*s->dpt[2][28]+s->dpt[1][28]*(FB224_21*C23-FM2321_324*S23);
  CM2222_323 = CM224_22*S23+s->dpt[1][28]*(s->m[24]*s->dpt[1][28]*C23*C23+S23*(FB124_22*S24-FB324_22*C24))-s->dpt[2][28]
 *(FB124_22*C24+FB324_22*S24)-C23*(CM124_22*S24-CM324_22*C24);
  FF21_122 = FF122*C22-FF222*S22;
  FF21_222 = FF122*S22+FF222*C22;
  CF21_122 = CF122*C22-CF222*S22;
  CF21_222 = CF122*S22+CF222*C22;
  FM211_122 = -(FM221_223*S22-FM231_124*C22);
  FM211_222 = FM221_223*C22+FM231_124*S22;
  CM211_122 = CM221_123*C22-CM221_223*S22;
  CM211_222 = CM221_123*S22+CM221_223*C22;
  FM212_122 = -(FM222_223*S22-FM232_124*C22);
  FM212_222 = FM222_223*C22+FM232_124*S22;
  CM212_122 = CM222_123*C22-CM222_223*S22;
  CM212_222 = CM222_123*S22+CM222_223*C22;
  FM213_122 = -(FM223_223*S22-FM233_124*C22);
  FM213_222 = FM223_223*C22+FM233_124*S22;
  CM213_122 = CM223_123*C22-CM223_223*S22;
  CM213_222 = CM223_123*S22+CM223_223*C22;
  FM214_122 = -(FM224_223*S22-FM234_124*C22);
  FM214_222 = FM224_223*C22+FM234_124*S22;
  CM214_122 = CM224_123*C22-CM224_223*S22;
  CM214_222 = CM224_123*S22+CM224_223*C22;
  FM215_122 = -(FM225_223*S22-FM235_124*C22);
  FM215_222 = FM225_223*C22+FM235_124*S22;
  CM215_122 = CM225_123*C22-CM225_223*S22;
  CM215_222 = CM225_123*S22+CM225_223*C22;
  FM216_122 = -(FM226_223*S22-FM236_124*C22);
  FM216_222 = FM226_223*C22+FM236_124*S22;
  CM216_122 = CM226_123*C22-CM226_223*S22;
  CM216_222 = CM226_123*S22+CM226_223*C22;
  FM217_122 = -(FM227_223*S22-FM237_124*C22);
  FM217_222 = FM227_223*C22+FM237_124*S22;
  CM217_122 = CM227_123*C22-CM227_223*S22;
  CM217_222 = CM227_123*S22+CM227_223*C22;
  FM2111_122 = -(FM2211_223*S22-FM2311_124*C22);
  FM2111_222 = FM2211_223*C22+FM2311_124*S22;
  CM2111_122 = CM2211_123*C22-CM2211_223*S22;
  CM2111_222 = CM2211_123*S22+CM2211_223*C22;
  FM2119_222 = FM2219_223*C22+FM2319_124*S22;
  CM2119_222 = CM2219_123*S22+CM2219_223*C22;
  CM2120_222 = CM2220_123*S22+CM2220_223*C22;
  CM2121_222 = C22*(CM224_21*C23-CM2321_324*S23+FM2321_124*s->dpt[3][28]-s->dpt[1][28]*(FB224_21*S23+FM2321_324*C23))+
 S22*(CM2321_124+s->dpt[2][28]*(FB224_21*S23+FM2321_324*C23)-s->dpt[3][28]*(FB224_21*C23-FM2321_324*S23));
  FF20_121 = FF21_122*C21+FF322*S21;
  FF20_321 = -(FF21_122*S21-FF322*C21);
  CF20_121 = CF21_122*C21+CF322*S21;
  CF20_321 = -(CF21_122*S21-CF322*C21);
  FM201_121 = FM211_122*C21+FM221_323*S21;
  FM201_321 = -(FM211_122*S21-FM221_323*C21);
  CM201_121 = CM211_122*C21+CM221_323*S21;
  CM201_321 = -(CM211_122*S21-CM221_323*C21);
  FM202_121 = FM212_122*C21+FM222_323*S21;
  FM202_321 = -(FM212_122*S21-FM222_323*C21);
  CM202_121 = CM212_122*C21+CM222_323*S21;
  CM202_321 = -(CM212_122*S21-CM222_323*C21);
  FM203_121 = FM213_122*C21+FM223_323*S21;
  FM203_321 = -(FM213_122*S21-FM223_323*C21);
  CM203_121 = CM213_122*C21+CM223_323*S21;
  CM203_321 = -(CM213_122*S21-CM223_323*C21);
  FM204_121 = FM214_122*C21+FM224_323*S21;
  FM204_321 = -(FM214_122*S21-FM224_323*C21);
  CM204_121 = CM214_122*C21+CM224_323*S21;
  CM204_321 = -(CM214_122*S21-CM224_323*C21);
  FM205_121 = FM215_122*C21+FM225_323*S21;
  FM205_321 = -(FM215_122*S21-FM225_323*C21);
  CM205_121 = CM215_122*C21+CM225_323*S21;
  CM205_321 = -(CM215_122*S21-CM225_323*C21);
  FM206_121 = FM216_122*C21+FM226_323*S21;
  FM206_321 = -(FM216_122*S21-FM226_323*C21);
  CM206_121 = CM216_122*C21+CM226_323*S21;
  CM206_321 = -(CM216_122*S21-CM226_323*C21);
  FM207_121 = FM217_122*C21+FM227_323*S21;
  FM207_321 = -(FM217_122*S21-FM227_323*C21);
  CM207_121 = CM217_122*C21+CM227_323*S21;
  CM207_321 = -(CM217_122*S21-CM227_323*C21);
  FM2011_121 = FM2111_122*C21+FM2211_323*S21;
  FM2011_321 = -(FM2111_122*S21-FM2211_323*C21);
  CM2011_121 = CM2111_122*C21+CM2211_323*S21;
  CM2011_321 = -(CM2111_122*S21-CM2211_323*C21);
  FM2019_321 = C21*(FB224_19*S23+FM2319_324*C23)+S21*(FM2219_223*S22-FM2319_124*C22);
  CM2019_121 = CM2219_323*S21+C21*(CM2219_123*C22-CM2219_223*S22);
  CM2020_121 = CM2220_323*S21+C21*(CM2220_123*C22-CM2220_223*S22);
  FF119 = -(s->frc[1][19]-FF20_121);
  FF219 = -(s->frc[2][19]+FF20_321*S20-FF21_222*C20);
  FF319 = -(s->frc[3][19]-FF20_321*C20-FF21_222*S20);
  CF119 = -(s->trq[1][19]-CF20_121-s->dpt[2][24]*(FF20_321*C20+FF21_222*S20)-s->dpt[3][24]*(FF20_321*S20-FF21_222*C20));
  CF219 = -(s->trq[2][19]+CF20_321*S20-CF21_222*C20-FF20_121*s->dpt[3][24]);
  CF319 = -(s->trq[3][19]-CF20_321*C20-CF21_222*S20+FF20_121*s->dpt[2][24]);
  FM191_220 = -(FM201_321*S20-FM211_222*C20);
  FM191_320 = FM201_321*C20+FM211_222*S20;
  CM191_120 = CM201_121+s->dpt[2][24]*(FM201_321*C20+FM211_222*S20)+s->dpt[3][24]*(FM201_321*S20-FM211_222*C20);
  CM191_220 = -(CM201_321*S20-CM211_222*C20-FM201_121*s->dpt[3][24]);
  CM191_320 = CM201_321*C20+CM211_222*S20-FM201_121*s->dpt[2][24];
  FM192_220 = -(FM202_321*S20-FM212_222*C20);
  FM192_320 = FM202_321*C20+FM212_222*S20;
  CM192_120 = CM202_121+s->dpt[2][24]*(FM202_321*C20+FM212_222*S20)+s->dpt[3][24]*(FM202_321*S20-FM212_222*C20);
  CM192_220 = -(CM202_321*S20-CM212_222*C20-FM202_121*s->dpt[3][24]);
  CM192_320 = CM202_321*C20+CM212_222*S20-FM202_121*s->dpt[2][24];
  FM193_220 = -(FM203_321*S20-FM213_222*C20);
  FM193_320 = FM203_321*C20+FM213_222*S20;
  CM193_120 = CM203_121+s->dpt[2][24]*(FM203_321*C20+FM213_222*S20)+s->dpt[3][24]*(FM203_321*S20-FM213_222*C20);
  CM193_220 = -(CM203_321*S20-CM213_222*C20-FM203_121*s->dpt[3][24]);
  CM193_320 = CM203_321*C20+CM213_222*S20-FM203_121*s->dpt[2][24];
  FM194_220 = -(FM204_321*S20-FM214_222*C20);
  FM194_320 = FM204_321*C20+FM214_222*S20;
  CM194_120 = CM204_121+s->dpt[2][24]*(FM204_321*C20+FM214_222*S20)+s->dpt[3][24]*(FM204_321*S20-FM214_222*C20);
  CM194_220 = -(CM204_321*S20-CM214_222*C20-FM204_121*s->dpt[3][24]);
  CM194_320 = CM204_321*C20+CM214_222*S20-FM204_121*s->dpt[2][24];
  FM195_220 = -(FM205_321*S20-FM215_222*C20);
  FM195_320 = FM205_321*C20+FM215_222*S20;
  CM195_120 = CM205_121+s->dpt[2][24]*(FM205_321*C20+FM215_222*S20)+s->dpt[3][24]*(FM205_321*S20-FM215_222*C20);
  CM195_220 = -(CM205_321*S20-CM215_222*C20-FM205_121*s->dpt[3][24]);
  CM195_320 = CM205_321*C20+CM215_222*S20-FM205_121*s->dpt[2][24];
  FM196_220 = -(FM206_321*S20-FM216_222*C20);
  FM196_320 = FM206_321*C20+FM216_222*S20;
  CM196_120 = CM206_121+s->dpt[2][24]*(FM206_321*C20+FM216_222*S20)+s->dpt[3][24]*(FM206_321*S20-FM216_222*C20);
  CM196_220 = -(CM206_321*S20-CM216_222*C20-FM206_121*s->dpt[3][24]);
  CM196_320 = CM206_321*C20+CM216_222*S20-FM206_121*s->dpt[2][24];
  CM197_120 = CM207_121+s->dpt[2][24]*(FM207_321*C20+FM217_222*S20)+s->dpt[3][24]*(FM207_321*S20-FM217_222*C20);
  CM1911_120 = CM2011_121+s->dpt[2][24]*(FM2011_321*C20+FM2111_222*S20)+s->dpt[3][24]*(FM2011_321*S20-FM2111_222*C20);
  CM1919_120 = CM2019_121+s->dpt[2][24]*(FM2019_321*C20+FM2119_222*S20)+s->dpt[3][24]*(FM2019_321*S20-FM2119_222*C20);

// = = Block_0_2_0_1_0_9 = = 
 
// Backward Dynamics 

  FF28_229 = -(s->frc[2][29]*C29-s->frc[3][29]*S29);
  CF28_229 = -(s->trq[2][29]*C29-s->trq[3][29]*S29);
  CF28_329 = -(s->trq[2][29]*S29+s->trq[3][29]*C29);

// = = Block_0_2_0_1_0_10 = = 
 
// Backward Dynamics 

  FF30_231 = -(s->frc[2][31]*C31-s->frc[3][31]*S31);
  CF30_231 = -(s->trq[2][31]*C31-s->trq[3][31]*S31);
  CF30_331 = -(s->trq[2][31]*S31+s->trq[3][31]*C31);

// = = Block_0_2_0_2_0_2 = = 
 
// Backward Dynamics 

  FF19 = -(s->frc[1][9]+s->frc[1][10]*C10-s->frc[2][10]*S10-s->m[9]*(C9*(AlF18+BS18*s->dpt[1][8])-S9*(AlF38+BeF78*
 s->dpt[1][8])));
  FF29 = -(s->frc[2][9]+s->frc[1][10]*S10+s->frc[2][10]*C10-s->m[9]*(AlF26+BeF47*s->dpt[1][2]+BeF67*s->dpt[3][2]+
 s->dpt[1][8]*(OpF38+OM18*OM28)));
  FF39 = -(s->frc[3][10]+s->frc[3][9]-s->m[9]*(C9*(AlF38+BeF78*s->dpt[1][8])+S9*(AlF18+BS18*s->dpt[1][8])));
  CF19 = -(s->trq[1][9]-s->In[1][9]*(C9*(OpF18-qd[9]*OM38)-S9*(OpF38+qd[9]*OM18))+s->trq[1][10]*C10-s->trq[2][10]*S10+
 OM29*OM39*(s->In[5][9]-s->In[9][9])-s->dpt[3][11]*(s->frc[1][10]*S10+s->frc[2][10]*C10));
  CF29 = -(s->trq[2][9]-s->In[5][9]*OpF26+s->trq[1][10]*S10+s->trq[2][10]*C10-OM19*OM39*(s->In[1][9]-s->In[9][9])+
 s->dpt[3][11]*(s->frc[1][10]*C10-s->frc[2][10]*S10));
  CF39 = -(s->trq[3][10]+s->trq[3][9]-s->In[9][9]*(C9*(OpF38+qd[9]*OM18)+S9*(OpF18-qd[9]*OM38))+OM19*OM29*(s->In[1][9]-
 s->In[5][9]));
  FB19_1 = s->m[9]*(AlM18_1*C9-S9*(AlM38_1-s->dpt[1][8]*S6));
  FB29_1 = s->m[9]*(AlM26_1-OpM17_1*s->dpt[3][2]+OpM37_1*s->dpt[1][2]+s->dpt[1][8]*C7p8*C6);
  FB39_1 = s->m[9]*(AlM18_1*S9+C9*(AlM38_1-s->dpt[1][8]*S6));
  CM19_1 = -s->In[1][9]*S7p8p9*C6;
  CM29_1 = s->In[5][9]*S6;
  CM39_1 = s->In[9][9]*C7p8p9*C6;
  FB19_2 = s->m[9]*(AlM18_2*C9-AlM38_2*S9);
  FB29_2 = s->m[9]*AlM26_2;
  FB39_2 = s->m[9]*(AlM18_2*S9+AlM38_2*C9);
  FB19_3 = s->m[9]*(AlM18_3*C9-AlM38_3*S9);
  FB29_3 = s->m[9]*AlM26_3;
  FB39_3 = s->m[9]*(AlM18_3*S9+AlM38_3*C9);
  FB29_4 = s->m[9]*S6;
  FB19_5 = s->m[9]*(AlM18_5*C9-S9*(AlM38_5-s->dpt[1][8]*S6));
  FB29_5 = -s->m[9]*(OpM17_5*s->dpt[3][2]-OpM37_5*s->dpt[1][2]-s->dpt[1][8]*C7p8*C6);
  FB39_5 = s->m[9]*(AlM18_5*S9+C9*(AlM38_5-s->dpt[1][8]*S6));
  CM19_5 = -s->In[1][9]*S7p8p9*C6;
  CM29_5 = s->In[5][9]*S6;
  CM39_5 = s->In[9][9]*C7p8p9*C6;
  FB29_6 = s->m[9]*(s->dpt[1][2]*S7+s->dpt[1][8]*S7p8-s->dpt[3][2]*C7);
  CM19_6 = s->In[1][9]*C7p8p9;
  CM39_6 = s->In[9][9]*S7p8p9;
  FB19_7 = s->m[9]*(AlM18_7*C9-S9*(AlM38_7-s->dpt[1][8]));
  FB39_7 = s->m[9]*(AlM18_7*S9+C9*(AlM38_7-s->dpt[1][8]));
  FF18 = -(s->frc[1][8]-FF19*C9-FF39*S9);
  FF28 = -(s->frc[2][8]-FF29);
  FF38 = -(s->frc[3][8]+FF19*S9-FF39*C9);
  CF18 = -(s->trq[1][8]-CF19*C9-CF39*S9);
  CF28 = -(s->trq[2][8]-CF29-s->dpt[1][8]*(FF19*S9-FF39*C9));
  CF38 = -(s->trq[3][8]+CF19*S9-CF39*C9-FF29*s->dpt[1][8]);
  FM81_19 = FB19_1*C9+FB39_1*S9;
  FM81_39 = -(FB19_1*S9-FB39_1*C9);
  CM81_19 = CM19_1*C9+CM39_1*S9;
  CM81_29 = CM29_1+s->dpt[1][8]*(FB19_1*S9-FB39_1*C9);
  CM81_39 = -(CM19_1*S9-CM39_1*C9-FB29_1*s->dpt[1][8]);
  FM82_19 = FB19_2*C9+FB39_2*S9;
  FM82_39 = -(FB19_2*S9-FB39_2*C9);
  CM82_29 = s->dpt[1][8]*(FB19_2*S9-FB39_2*C9);
  CM82_39 = FB29_2*s->dpt[1][8];
  FM83_19 = FB19_3*C9+FB39_3*S9;
  FM83_39 = -(FB19_3*S9-FB39_3*C9);
  CM83_29 = s->dpt[1][8]*(FB19_3*S9-FB39_3*C9);
  CM83_39 = FB29_3*s->dpt[1][8];
  CM84_29 = -s->m[9]*s->dpt[1][8]*C7p8*C6;
  CM84_39 = FB29_4*s->dpt[1][8];
  FM85_19 = FB19_5*C9+FB39_5*S9;
  FM85_39 = -(FB19_5*S9-FB39_5*C9);
  CM85_19 = CM19_5*C9+CM39_5*S9;
  CM85_29 = CM29_5+s->dpt[1][8]*(FB19_5*S9-FB39_5*C9);
  CM85_39 = -(CM19_5*S9-CM39_5*C9-FB29_5*s->dpt[1][8]);
  CM86_19 = CM19_6*C9+CM39_6*S9;
  CM86_39 = -(CM19_6*S9-CM39_6*C9-FB29_6*s->dpt[1][8]);
  FM87_19 = FB19_7*C9+FB39_7*S9;
  FM87_39 = -(FB19_7*S9-FB39_7*C9);
  CM87_29 = s->In[5][9]+s->dpt[1][8]*(FB19_7*S9-FB39_7*C9);
  CM88_29 = s->In[5][9]+s->m[9]*s->dpt[1][8]*s->dpt[1][8];

// = = Block_0_2_0_2_0_3 = = 
 
// Backward Dynamics 

  FF111 = -(s->frc[1][11]-FA112-FF113-FF119);
  FF211 = -(s->frc[2][11]-FA212*C12+FA312*S12-FF213*C13-FF219*C19+FF313*S13+FF319*S19);
  FF311 = -(s->frc[3][11]-FA212*S12-FA312*C12-FF213*S13-FF219*S19-FF313*C13-FF319*C19);
  CF111 = -(s->trq[1][11]-CF112-CF113-CF119-s->dpt[2][12]*(FF213*S13+FF313*C13)-s->dpt[2][13]*(FF219*S19+FF319*C19)+
 s->dpt[3][12]*(FF213*C13-FF313*S13)+s->dpt[3][13]*(FF219*C19-FF319*S19));
  CF211 = -(s->trq[2][11]-CF212*C12-CF213*C13-CF219*C19+CF312*S12+CF313*S13+CF319*S19-FF113*s->dpt[3][12]-FF119*
 s->dpt[3][13]);
  CF311 = -(s->trq[3][11]-CF212*S12-CF213*S13-CF219*S19-CF312*C12-CF313*C13-CF319*C19+FF113*s->dpt[2][12]+FF119*
 s->dpt[2][13]);
  FM111_1 = FB112_1+FM141_115+FM201_121;
  FM211_1 = FB212_1*C12-FB312_1*S12+FM131_214*C13-FM131_314*S13+FM191_220*C19-FM191_320*S19;
  FM311_1 = FB212_1*S12+FB312_1*C12+FM131_214*S13+FM131_314*C13+FM191_220*S19+FM191_320*C19;
  CM111_1 = CM112_1+CM131_114+CM191_120+s->dpt[2][12]*(FM131_214*S13+FM131_314*C13)+s->dpt[2][13]*(FM191_220*S19+
 FM191_320*C19)-s->dpt[3][12]*(FM131_214*C13-FM131_314*S13)-s->dpt[3][13]*(FM191_220*C19-FM191_320*S19);
  CM211_1 = CM131_214*C13-CM131_314*S13+CM191_220*C19-CM191_320*S19+CM212_1*C12+FM141_115*s->dpt[3][12]+FM201_121*
 s->dpt[3][13];
  CM311_1 = CM131_214*S13+CM131_314*C13+CM191_220*S19+CM191_320*C19+CM212_1*S12-FM141_115*s->dpt[2][12]-FM201_121*
 s->dpt[2][13];
  FM111_2 = FB112_2+FM142_115+FM202_121;
  FM211_2 = FB212_2*C12-FB312_2*S12+FM132_214*C13-FM132_314*S13+FM192_220*C19-FM192_320*S19;
  FM311_2 = FB212_2*S12+FB312_2*C12+FM132_214*S13+FM132_314*C13+FM192_220*S19+FM192_320*C19;
  CM111_2 = CM112_2+CM132_114+CM192_120+s->dpt[2][12]*(FM132_214*S13+FM132_314*C13)+s->dpt[2][13]*(FM192_220*S19+
 FM192_320*C19)-s->dpt[3][12]*(FM132_214*C13-FM132_314*S13)-s->dpt[3][13]*(FM192_220*C19-FM192_320*S19);
  CM211_2 = CM132_214*C13-CM132_314*S13+CM192_220*C19-CM192_320*S19+CM212_2*C12+FM142_115*s->dpt[3][12]+FM202_121*
 s->dpt[3][13];
  CM311_2 = CM132_214*S13+CM132_314*C13+CM192_220*S19+CM192_320*C19+CM212_2*S12-FM142_115*s->dpt[2][12]-FM202_121*
 s->dpt[2][13];
  FM111_3 = FB112_3+FM143_115+FM203_121;
  FM211_3 = FB212_3*C12-FB312_3*S12+FM133_214*C13-FM133_314*S13+FM193_220*C19-FM193_320*S19;
  FM311_3 = FB212_3*S12+FB312_3*C12+FM133_214*S13+FM133_314*C13+FM193_220*S19+FM193_320*C19;
  CM111_3 = CM112_3+CM133_114+CM193_120+s->dpt[2][12]*(FM133_214*S13+FM133_314*C13)+s->dpt[2][13]*(FM193_220*S19+
 FM193_320*C19)-s->dpt[3][12]*(FM133_214*C13-FM133_314*S13)-s->dpt[3][13]*(FM193_220*C19-FM193_320*S19);
  CM211_3 = CM133_214*C13-CM133_314*S13+CM193_220*C19-CM193_320*S19+CM212_3*C12+FM143_115*s->dpt[3][12]+FM203_121*
 s->dpt[3][13];
  CM311_3 = CM133_214*S13+CM133_314*C13+CM193_220*S19+CM193_320*C19+CM212_3*S12-FM143_115*s->dpt[2][12]-FM203_121*
 s->dpt[2][13];
  FM111_4 = FB112_4+FM144_115+FM204_121;
  FM211_4 = FB212_4*C12-FB312_4*S12+FM134_214*C13-FM134_314*S13+FM194_220*C19-FM194_320*S19;
  FM311_4 = FB212_4*S12+FB312_4*C12+FM134_214*S13+FM134_314*C13+FM194_220*S19+FM194_320*C19;
  CM111_4 = CM112_4+CM134_114+CM194_120+s->dpt[2][12]*(FM134_214*S13+FM134_314*C13)+s->dpt[2][13]*(FM194_220*S19+
 FM194_320*C19)-s->dpt[3][12]*(FM134_214*C13-FM134_314*S13)-s->dpt[3][13]*(FM194_220*C19-FM194_320*S19);
  CM211_4 = CM134_214*C13-CM134_314*S13+CM194_220*C19-CM194_320*S19+CM212_4*C12+FM144_115*s->dpt[3][12]+FM204_121*
 s->dpt[3][13];
  CM311_4 = CM134_214*S13+CM134_314*C13+CM194_220*S19+CM194_320*C19+CM212_4*S12-FM144_115*s->dpt[2][12]-FM204_121*
 s->dpt[2][13];
  FM111_5 = FB112_5+FM145_115+FM205_121;
  FM211_5 = FB212_5*C12-FB312_5*S12+FM135_214*C13-FM135_314*S13+FM195_220*C19-FM195_320*S19;
  FM311_5 = FB212_5*S12+FB312_5*C12+FM135_214*S13+FM135_314*C13+FM195_220*S19+FM195_320*C19;
  CM111_5 = CM112_5+CM135_114+CM195_120+s->dpt[2][12]*(FM135_214*S13+FM135_314*C13)+s->dpt[2][13]*(FM195_220*S19+
 FM195_320*C19)-s->dpt[3][12]*(FM135_214*C13-FM135_314*S13)-s->dpt[3][13]*(FM195_220*C19-FM195_320*S19);
  CM211_5 = CM135_214*C13-CM135_314*S13+CM195_220*C19-CM195_320*S19+CM212_5*C12+FM145_115*s->dpt[3][12]+FM205_121*
 s->dpt[3][13];
  CM311_5 = CM135_214*S13+CM135_314*C13+CM195_220*S19+CM195_320*C19+CM212_5*S12-FM145_115*s->dpt[2][12]-FM205_121*
 s->dpt[2][13];
  FM111_6 = FB112_6+FM146_115+FM206_121;
  FM211_6 = FB212_6*C12-FB312_6*S12+FM136_214*C13-FM136_314*S13+FM196_220*C19-FM196_320*S19;
  FM311_6 = FB212_6*S12+FB312_6*C12+FM136_214*S13+FM136_314*C13+FM196_220*S19+FM196_320*C19;
  CM111_6 = CM112_6+CM136_114+CM196_120+s->dpt[2][12]*(FM136_214*S13+FM136_314*C13)+s->dpt[2][13]*(FM196_220*S19+
 FM196_320*C19)-s->dpt[3][12]*(FM136_214*C13-FM136_314*S13)-s->dpt[3][13]*(FM196_220*C19-FM196_320*S19);
  CM211_6 = CM136_214*C13-CM136_314*S13+CM196_220*C19-CM196_320*S19+CM212_6*C12+FM146_115*s->dpt[3][12]+FM206_121*
 s->dpt[3][13];
  CM311_6 = CM136_214*S13+CM136_314*C13+CM196_220*S19+CM196_320*C19+CM212_6*S12-FM146_115*s->dpt[2][12]-FM206_121*
 s->dpt[2][13];
  FM111_7 = FB112_7+FM147_115+FM207_121;
  FM311_7 = s->m[12]*AlM311_7+C13*(FM147_315*C14+FM157_216*S14)-S13*(FM147_315*S14-FM157_216*C14)+C19*(FM207_321*C20+
 FM217_222*S20)-S19*(FM207_321*S20-FM217_222*C20);
  CM211_7 = FM147_115*s->dpt[3][12]+FM207_121*s->dpt[3][13]+C12*(s->In[5][12]*C12+FB112_7*s->l[3][12])-C13*(CM147_315*
 S14-CM157_216*C14-FM147_115*s->dpt[3][17])-S13*(CM147_315*C14+CM157_216*S14-FM147_115*s->dpt[2][17])-C19*(CM207_321*S20-
 CM217_222*C20-FM207_121*s->dpt[3][24])-S19*(CM207_321*C20+CM217_222*S20-FM207_121*s->dpt[2][24]);
  CM211_11 = FM1411_115*s->dpt[3][12]+FM2011_121*s->dpt[3][13]+C12*C12*(s->In[5][12]+s->m[12]*s->l[3][12]*s->l[3][12])-
 C13*(CM1411_315*S14-CM1511_216*C14-FM1411_115*s->dpt[3][17])-S13*(CM1411_315*C14+CM1511_216*S14-FM1411_115*s->dpt[2][17])-
 C19*(CM2011_321*S20-CM2111_222*C20-FM2011_121*s->dpt[3][24])-S19*(CM2011_321*C20+CM2111_222*S20-FM2011_121*s->dpt[2][24]);
  CM76_211 = CM211_6+s->dpt[1][5]*(FM111_6*S11-FM311_6*C11)+s->dpt[3][5]*(FM111_6*C11+FM311_6*S11);

// = = Block_0_2_0_2_0_7 = = 
 
// Backward Dynamics 

  FF25_126 = -(s->frc[1][26]*C26-s->frc[2][26]*S26);
  FF25_226 = -(s->frc[1][26]*S26+s->frc[2][26]*C26);
  CF25_126 = -(s->trq[1][26]*C26-s->trq[2][26]*S26);
  CF25_226 = -(s->trq[1][26]*S26+s->trq[2][26]*C26);

// = = Block_0_2_0_2_0_8 = = 
 
// Backward Dynamics 

  FF127 = -(s->frc[1][27]+s->frc[1][29]*C28+s->frc[1][31]*C30+FF28_229*S28+FF30_231*S30);
  FF227 = -(s->frc[2][27]+s->frc[1][29]*S28+s->frc[1][31]*S30-FF28_229*C28-FF30_231*C30);
  FF327 = -(s->frc[3][27]+s->frc[2][29]*S29+s->frc[2][31]*S31+s->frc[3][29]*C29+s->frc[3][31]*C31);
  CF127 = -(s->trq[1][27]+s->trq[1][29]*C28+s->trq[1][31]*C30+CF28_229*S28+CF30_231*S30);
  CF227 = -(s->trq[2][27]+s->trq[1][29]*S28+s->trq[1][31]*S30-CF28_229*C28-CF30_231*C30);
  CF327 = -(s->trq[3][27]-CF28_329-CF30_331);

// = = Block_0_2_0_3_0_1 = = 
 
// Backward Dynamics 

  FA17 = -(s->frc[1][7]-s->m[7]*(AlF17+BS17*s->l[1][7]+BeF37*s->l[3][7]));
  FA27 = -(s->frc[2][7]-s->m[7]*(AlF26+BeF47*s->l[1][7]+BeF67*s->l[3][7]));
  FA37 = -(s->frc[3][7]-s->m[7]*(AlF37+BS97*s->l[3][7]+BeF77*s->l[1][7]));
  FF17 = FA17-s->frc[3][26]*S25+FF111*C11+FF127*C27+FF18*C8+FF25_126*C25+FF311*S11+FF327*S27+FF38*S8;
  FF27 = FA27+FF211+FF227+FF25_226+FF28;
  FF37 = FA37-s->frc[3][26]*C25-FF111*S11-FF127*S27-FF18*S8-FF25_126*S25+FF311*C11+FF327*C27+FF38*C8;
  CF17 = -(s->trq[1][7]-s->In[1][7]*OpF17-s->In[3][7]*OpF37+s->trq[3][26]*S25-CF111*C11-CF127*C27-CF18*C8-CF25_126*C25-
 CF311*S11-CF327*S27-CF38*S8+FA27*s->l[3][7]+FF211*s->dpt[3][5]+FF227*s->dpt[3][7]+FF25_226*s->dpt[3][6]+FF28*s->dpt[3][2]-
 OM27*(s->In[3][7]*OM17-s->In[5][7]*OM37+s->In[9][7]*OM37));
  CF27 = -(s->trq[2][7]-CF211-CF227-CF25_226-CF28-s->In[5][7]*OpF26-FA17*s->l[3][7]+FA37*s->l[1][7]+OM17*(s->In[3][7]*
 OM17+s->In[9][7]*OM37)-OM37*(s->In[1][7]*OM17+s->In[3][7]*OM37)-s->dpt[1][2]*(FF18*S8-FF38*C8)-s->dpt[1][5]*(FF111*S11-FF311
 *C11)-s->dpt[1][6]*(s->frc[3][26]*C25+FF25_126*S25)-s->dpt[1][7]*(FF127*S27-FF327*C27)-s->dpt[3][2]*(FF18*C8+FF38*S8)-
 s->dpt[3][5]*(FF111*C11+FF311*S11)+s->dpt[3][6]*(s->frc[3][26]*S25-FF25_126*C25)-s->dpt[3][7]*(FF127*C27+FF327*S27));
  CF37 = -(s->trq[3][7]-s->In[3][7]*OpF17-s->In[9][7]*OpF37+s->trq[3][26]*C25+CF111*S11+CF127*S27+CF18*S8+CF25_126*S25-
 CF311*C11-CF327*C27-CF38*C8-FA27*s->l[1][7]-FF211*s->dpt[1][5]-FF227*s->dpt[1][7]-FF25_226*s->dpt[1][6]-FF28*s->dpt[1][2]+
 OM27*(s->In[1][7]*OM17+s->In[3][7]*OM37-s->In[5][7]*OM17));
  FB17_1 = s->m[7]*(AlM17_1+s->l[3][7]*S6);
  FB27_1 = s->m[7]*(AlM26_1-OpM17_1*s->l[3][7]+OpM37_1*s->l[1][7]);
  FB37_1 = s->m[7]*(AlM37_1-s->l[1][7]*S6);
  FM17_1 = FB17_1+FM111_1*C11+FM311_1*S11+FM81_19*C8+FM81_39*S8;
  FM27_1 = FB27_1+FB29_1+FM211_1;
  FM37_1 = FB37_1-FM111_1*S11+FM311_1*C11-FM81_19*S8+FM81_39*C8;
  CM17_1 = s->In[1][7]*OpM17_1+s->In[3][7]*OpM37_1+CM111_1*C11+CM311_1*S11+CM81_19*C8+CM81_39*S8-FB27_1*s->l[3][7]-
 FB29_1*s->dpt[3][2]-FM211_1*s->dpt[3][5];
  CM27_1 = CM211_1+CM81_29+s->In[5][7]*S6+FB17_1*s->l[3][7]-FB37_1*s->l[1][7]+s->dpt[1][2]*(FM81_19*S8-FM81_39*C8)+
 s->dpt[1][5]*(FM111_1*S11-FM311_1*C11)+s->dpt[3][2]*(FM81_19*C8+FM81_39*S8)+s->dpt[3][5]*(FM111_1*C11+FM311_1*S11);
  CM37_1 = s->In[3][7]*OpM17_1+s->In[9][7]*OpM37_1-CM111_1*S11+CM311_1*C11-CM81_19*S8+CM81_39*C8+FB27_1*s->l[1][7]+
 FB29_1*s->dpt[1][2]+FM211_1*s->dpt[1][5];
  FB17_2 = s->m[7]*AlM17_2;
  FB27_2 = s->m[7]*AlM26_2;
  FB37_2 = s->m[7]*AlM37_2;
  FM17_2 = FB17_2+FM111_2*C11+FM311_2*S11+FM82_19*C8+FM82_39*S8;
  FM27_2 = FB27_2+FB29_2+FM211_2;
  FM37_2 = FB37_2-FM111_2*S11+FM311_2*C11-FM82_19*S8+FM82_39*C8;
  CM17_2 = CM111_2*C11+CM311_2*S11+CM82_39*S8-FB27_2*s->l[3][7]-FB29_2*s->dpt[3][2]-FM211_2*s->dpt[3][5];
  CM27_2 = CM211_2+CM82_29+FB17_2*s->l[3][7]-FB37_2*s->l[1][7]+s->dpt[1][2]*(FM82_19*S8-FM82_39*C8)+s->dpt[1][5]*(
 FM111_2*S11-FM311_2*C11)+s->dpt[3][2]*(FM82_19*C8+FM82_39*S8)+s->dpt[3][5]*(FM111_2*C11+FM311_2*S11);
  CM37_2 = -(CM111_2*S11-CM311_2*C11-CM82_39*C8-FB27_2*s->l[1][7]-FB29_2*s->dpt[1][2]-FM211_2*s->dpt[1][5]);
  FB17_3 = s->m[7]*AlM17_3;
  FB27_3 = s->m[7]*AlM26_3;
  FB37_3 = s->m[7]*AlM37_3;
  FM17_3 = FB17_3+FM111_3*C11+FM311_3*S11+FM83_19*C8+FM83_39*S8;
  FM27_3 = FB27_3+FB29_3+FM211_3;
  FM37_3 = FB37_3-FM111_3*S11+FM311_3*C11-FM83_19*S8+FM83_39*C8;
  CM17_3 = CM111_3*C11+CM311_3*S11+CM83_39*S8-FB27_3*s->l[3][7]-FB29_3*s->dpt[3][2]-FM211_3*s->dpt[3][5];
  CM27_3 = CM211_3+CM83_29+FB17_3*s->l[3][7]-FB37_3*s->l[1][7]+s->dpt[1][2]*(FM83_19*S8-FM83_39*C8)+s->dpt[1][5]*(
 FM111_3*S11-FM311_3*C11)+s->dpt[3][2]*(FM83_19*C8+FM83_39*S8)+s->dpt[3][5]*(FM111_3*C11+FM311_3*S11);
  CM37_3 = -(CM111_3*S11-CM311_3*C11-CM83_39*C8-FB27_3*s->l[1][7]-FB29_3*s->dpt[1][2]-FM211_3*s->dpt[1][5]);
  FB17_4 = -s->m[7]*C6*S7;
  FB27_4 = s->m[7]*S6;
  FB37_4 = s->m[7]*C6*C7;
  CM17_4 = CM111_4*C11+CM311_4*S11+CM84_39*S8-FB27_4*s->l[3][7]-FB29_4*s->dpt[3][2]-FM211_4*s->dpt[3][5];
  CM27_4 = CM211_4-s->m[9]*C6*(s->dpt[1][2]*C7+s->dpt[1][8]*C7p8+s->dpt[3][2]*S7)+FB17_4*s->l[3][7]-FB37_4*s->l[1][7]+
 s->dpt[1][5]*(FM111_4*S11-FM311_4*C11)+s->dpt[3][5]*(FM111_4*C11+FM311_4*S11);
  CM37_4 = -(CM111_4*S11-CM311_4*C11-CM84_39*C8-FB27_4*s->l[1][7]-FB29_4*s->dpt[1][2]-FM211_4*s->dpt[1][5]);
  FB27_5 = -s->m[7]*(OpM17_5*s->l[3][7]-OpM37_5*s->l[1][7]);
  CM17_5 = s->In[1][7]*OpM17_5+s->In[3][7]*OpM37_5+CM111_5*C11+CM311_5*S11+CM85_19*C8+CM85_39*S8-FB27_5*s->l[3][7]-
 FB29_5*s->dpt[3][2]-FM211_5*s->dpt[3][5];
  CM27_5 = CM211_5+CM85_29+s->In[5][7]*S6+s->m[7]*s->l[1][7]*s->l[1][7]*S6+s->m[7]*s->l[3][7]*s->l[3][7]*S6+s->dpt[1][2]
 *(FM85_19*S8-FM85_39*C8)+s->dpt[1][5]*(FM111_5*S11-FM311_5*C11)+s->dpt[3][2]*(FM85_19*C8+FM85_39*S8)+s->dpt[3][5]*(FM111_5*
 C11+FM311_5*S11);
  CM37_5 = s->In[3][7]*OpM17_5+s->In[9][7]*OpM37_5-CM111_5*S11+CM311_5*C11-CM85_19*S8+CM85_39*C8+FB27_5*s->l[1][7]+
 FB29_5*s->dpt[1][2]+FM211_5*s->dpt[1][5];
  FB27_6 = s->m[7]*(s->l[1][7]*S7-s->l[3][7]*C7);
  CM27_7 = s->In[5][7]+CM211_7+CM87_29+s->m[7]*s->l[1][7]*s->l[1][7]+s->m[7]*s->l[3][7]*s->l[3][7]+s->dpt[1][2]*(FM87_19
 *S8-FM87_39*C8)+s->dpt[1][5]*(FM111_7*S11-FM311_7*C11)+s->dpt[3][2]*(FM87_19*C8+FM87_39*S8)+s->dpt[3][5]*(FM111_7*C11+
 FM311_7*S11);
  FF6_17 = FF17*C7+FF37*S7;
  FF6_37 = -(FF17*S7-FF37*C7);
  CF6_17 = CF17*C7+CF37*S7;
  FM61_17 = FM17_1*C7+FM37_1*S7;
  FM61_37 = -(FM17_1*S7-FM37_1*C7);
  CM61_17 = CM17_1*C7+CM37_1*S7;
  FM62_17 = FM17_2*C7+FM37_2*S7;
  FM62_37 = -(FM17_2*S7-FM37_2*C7);
  CM62_17 = CM17_2*C7+CM37_2*S7;
  FM63_37 = -(FM17_3*S7-FM37_3*C7);
  CM63_17 = CM17_3*C7+CM37_3*S7;
  CM64_17 = CM17_4*C7+CM37_4*S7;
  CM65_17 = CM17_5*C7+CM37_5*S7;
  CM66_17 = C7*(s->In[1][7]*C7+s->In[3][7]*S7+CM111_6*C11+CM311_6*S11+CM86_19*C8+CM86_39*S8-FB27_6*s->l[3][7]-FB29_6*
 s->dpt[3][2]-FM211_6*s->dpt[3][5])+S7*(s->In[3][7]*C7+s->In[9][7]*S7-CM111_6*S11+CM311_6*C11-CM86_19*S8+CM86_39*C8+FB27_6*
 s->l[1][7]+FB29_6*s->dpt[1][2]+FM211_6*s->dpt[1][5]);
  FF5_26 = FF27*C6-FF6_37*S6;
  FF5_36 = FF27*S6+FF6_37*C6;
  CF5_36 = CF27*S6-C6*(CF17*S7-CF37*C7);
  FM51_26 = FM27_1*C6-FM61_37*S6;
  FM51_36 = FM27_1*S6+FM61_37*C6;
  CM51_36 = CM27_1*S6-C6*(CM17_1*S7-CM37_1*C7);
  FM52_26 = FM27_2*C6-FM62_37*S6;
  FM52_36 = FM27_2*S6+FM62_37*C6;
  CM52_36 = CM27_2*S6-C6*(CM17_2*S7-CM37_2*C7);
  FM53_36 = FM27_3*S6+FM63_37*C6;
  CM53_36 = CM27_3*S6-C6*(CM17_3*S7-CM37_3*C7);
  FM54_36 = C6*(C7*(FB37_4+s->m[9]*C6*C7-FM111_4*S11+FM311_4*C11)-S7*(FB17_4-s->m[9]*C6*S7+FM111_4*C11+FM311_4*S11))+S6*
 (FB27_4+FB29_4+FM211_4);
  CM54_36 = CM27_4*S6-C6*(CM17_4*S7-CM37_4*C7);
  CM55_36 = CM27_5*S6-C6*(CM17_5*S7-CM37_5*C7);
  FF4_15 = -(FF5_26*S5-FF6_17*C5);
  FF4_25 = FF5_26*C5+FF6_17*S5;
  FM41_15 = -(FM51_26*S5-FM61_17*C5);
  FM41_25 = FM51_26*C5+FM61_17*S5;
  FM42_15 = -(FM52_26*S5-FM62_17*C5);
  FM42_25 = FM52_26*C5+FM62_17*S5;
  FM43_25 = C5*(FM27_3*C6-FM63_37*S6)+S5*(FM17_3*C7+FM37_3*S7);
  CF1_32 = CF5_36+q[2]*FF4_25-q[3]*FF4_15;
  CM11_32 = CM51_36+q[2]*FM41_25-q[3]*FM41_15;

// = = Block_0_3_0_0_0_0 = = 
 
// Symbolic Outputs  

  M[1][1] = CM11_32;
  M[1][2] = FM41_15;
  M[1][3] = FM41_25;
  M[1][4] = FM51_36;
  M[1][5] = CM51_36;
  M[1][6] = CM61_17;
  M[1][7] = CM27_1;
  M[1][8] = CM81_29;
  M[1][9] = CM29_1;
  M[1][11] = CM211_1;
  M[1][12] = CM112_1;
  M[1][13] = CM131_114;
  M[1][14] = CM141_115;
  M[1][15] = CM151_216;
  M[1][16] = CM161_317;
  M[1][17] = CM171_118;
  M[1][18] = CM218_1;
  M[1][19] = CM191_120;
  M[1][20] = CM201_121;
  M[1][21] = CM211_222;
  M[1][22] = CM221_323;
  M[1][23] = CM231_124;
  M[1][24] = CM224_1;
  M[2][1] = FM41_15;
  M[2][2] = FM42_15;
  M[2][3] = FM42_25;
  M[2][4] = FM52_36;
  M[2][5] = CM52_36;
  M[2][6] = CM62_17;
  M[2][7] = CM27_2;
  M[2][8] = CM82_29;
  M[2][11] = CM211_2;
  M[2][12] = CM112_2;
  M[2][13] = CM132_114;
  M[2][14] = CM142_115;
  M[2][15] = CM152_216;
  M[2][16] = CM162_317;
  M[2][19] = CM192_120;
  M[2][20] = CM202_121;
  M[2][21] = CM212_222;
  M[2][22] = CM222_323;
  M[3][1] = FM41_25;
  M[3][2] = FM42_25;
  M[3][3] = FM43_25;
  M[3][4] = FM53_36;
  M[3][5] = CM53_36;
  M[3][6] = CM63_17;
  M[3][7] = CM27_3;
  M[3][8] = CM83_29;
  M[3][11] = CM211_3;
  M[3][12] = CM112_3;
  M[3][13] = CM133_114;
  M[3][14] = CM143_115;
  M[3][15] = CM153_216;
  M[3][16] = CM163_317;
  M[3][19] = CM193_120;
  M[3][20] = CM203_121;
  M[3][21] = CM213_222;
  M[3][22] = CM223_323;
  M[4][1] = FM51_36;
  M[4][2] = FM52_36;
  M[4][3] = FM53_36;
  M[4][4] = FM54_36;
  M[4][5] = CM54_36;
  M[4][6] = CM64_17;
  M[4][7] = CM27_4;
  M[4][8] = CM84_29;
  M[4][11] = CM211_4;
  M[4][12] = CM112_4;
  M[4][13] = CM134_114;
  M[4][14] = CM144_115;
  M[4][15] = CM154_216;
  M[4][16] = CM164_317;
  M[4][19] = CM194_120;
  M[4][20] = CM204_121;
  M[4][21] = CM214_222;
  M[4][22] = CM224_323;
  M[5][1] = CM51_36;
  M[5][2] = CM52_36;
  M[5][3] = CM53_36;
  M[5][4] = CM54_36;
  M[5][5] = CM55_36;
  M[5][6] = CM65_17;
  M[5][7] = CM27_5;
  M[5][8] = CM85_29;
  M[5][9] = CM29_5;
  M[5][11] = CM211_5;
  M[5][12] = CM112_5;
  M[5][13] = CM135_114;
  M[5][14] = CM145_115;
  M[5][15] = CM155_216;
  M[5][16] = CM165_317;
  M[5][17] = CM175_118;
  M[5][18] = CM218_5;
  M[5][19] = CM195_120;
  M[5][20] = CM205_121;
  M[5][21] = CM215_222;
  M[5][22] = CM225_323;
  M[5][23] = CM235_124;
  M[5][24] = CM224_5;
  M[6][1] = CM61_17;
  M[6][2] = CM62_17;
  M[6][3] = CM63_17;
  M[6][4] = CM64_17;
  M[6][5] = CM65_17;
  M[6][6] = CM66_17;
  M[6][7] = CM76_211;
  M[6][11] = CM211_6;
  M[6][12] = CM112_6;
  M[6][13] = CM136_114;
  M[6][14] = CM146_115;
  M[6][15] = CM156_216;
  M[6][16] = CM166_317;
  M[6][17] = CM176_118;
  M[6][18] = CM218_6;
  M[6][19] = CM196_120;
  M[6][20] = CM206_121;
  M[6][21] = CM216_222;
  M[6][22] = CM226_323;
  M[6][23] = CM236_124;
  M[6][24] = CM224_6;
  M[7][1] = CM27_1;
  M[7][2] = CM27_2;
  M[7][3] = CM27_3;
  M[7][4] = CM27_4;
  M[7][5] = CM27_5;
  M[7][6] = CM76_211;
  M[7][7] = CM27_7;
  M[7][8] = CM87_29;
  M[7][9] = s->In[5][9];
  M[7][11] = CM211_7;
  M[7][12] = CM112_7;
  M[7][13] = CM137_114;
  M[7][14] = CM147_115;
  M[7][15] = CM157_216;
  M[7][16] = CM167_317;
  M[7][17] = CM177_118;
  M[7][18] = CM218_7;
  M[7][19] = CM197_120;
  M[7][20] = CM207_121;
  M[7][21] = CM217_222;
  M[7][22] = CM227_323;
  M[7][23] = CM237_124;
  M[7][24] = CM224_7;
  M[8][1] = CM81_29;
  M[8][2] = CM82_29;
  M[8][3] = CM83_29;
  M[8][4] = CM84_29;
  M[8][5] = CM85_29;
  M[8][7] = CM87_29;
  M[8][8] = CM88_29;
  M[8][9] = s->In[5][9];
  M[9][1] = CM29_1;
  M[9][5] = CM29_5;
  M[9][7] = s->In[5][9];
  M[9][8] = s->In[5][9];
  M[9][9] = s->In[5][9];
  M[11][1] = CM211_1;
  M[11][2] = CM211_2;
  M[11][3] = CM211_3;
  M[11][4] = CM211_4;
  M[11][5] = CM211_5;
  M[11][6] = CM211_6;
  M[11][7] = CM211_7;
  M[11][11] = CM211_11;
  M[11][13] = CM1311_114;
  M[11][14] = CM1411_115;
  M[11][15] = CM1511_216;
  M[11][16] = CM1611_317;
  M[11][17] = CM1711_118;
  M[11][18] = CM218_11;
  M[11][19] = CM1911_120;
  M[11][20] = CM2011_121;
  M[11][21] = CM2111_222;
  M[11][22] = CM2211_323;
  M[11][23] = CM2311_124;
  M[11][24] = CM224_11;
  M[12][1] = CM112_1;
  M[12][2] = CM112_2;
  M[12][3] = CM112_3;
  M[12][4] = CM112_4;
  M[12][5] = CM112_5;
  M[12][6] = CM112_6;
  M[12][7] = CM112_7;
  M[12][12] = CM112_12;
  M[13][1] = CM131_114;
  M[13][2] = CM132_114;
  M[13][3] = CM133_114;
  M[13][4] = CM134_114;
  M[13][5] = CM135_114;
  M[13][6] = CM136_114;
  M[13][7] = CM137_114;
  M[13][11] = CM1311_114;
  M[13][13] = CM1313_114;
  M[13][14] = CM1413_115;
  M[13][15] = CM1513_216;
  M[13][16] = CM1613_317;
  M[13][17] = CM1713_118;
  M[13][18] = CM218_13;
  M[14][1] = CM141_115;
  M[14][2] = CM142_115;
  M[14][3] = CM143_115;
  M[14][4] = CM144_115;
  M[14][5] = CM145_115;
  M[14][6] = CM146_115;
  M[14][7] = CM147_115;
  M[14][11] = CM1411_115;
  M[14][13] = CM1413_115;
  M[14][14] = CM1414_115;
  M[14][15] = CM1514_216;
  M[14][16] = CM1614_317;
  M[14][17] = CM1714_118;
  M[14][18] = CM218_14;
  M[15][1] = CM151_216;
  M[15][2] = CM152_216;
  M[15][3] = CM153_216;
  M[15][4] = CM154_216;
  M[15][5] = CM155_216;
  M[15][6] = CM156_216;
  M[15][7] = CM157_216;
  M[15][11] = CM1511_216;
  M[15][13] = CM1513_216;
  M[15][14] = CM1514_216;
  M[15][15] = CM1515_216;
  M[15][16] = CM1615_317;
  M[15][17] = CM1715_118;
  M[15][18] = CM218_15;
  M[16][1] = CM161_317;
  M[16][2] = CM162_317;
  M[16][3] = CM163_317;
  M[16][4] = CM164_317;
  M[16][5] = CM165_317;
  M[16][6] = CM166_317;
  M[16][7] = CM167_317;
  M[16][11] = CM1611_317;
  M[16][13] = CM1613_317;
  M[16][14] = CM1614_317;
  M[16][15] = CM1615_317;
  M[16][16] = CM1616_317;
  M[16][17] = CM1716_118;
  M[16][18] = CM218_16;
  M[17][1] = CM171_118;
  M[17][5] = CM175_118;
  M[17][6] = CM176_118;
  M[17][7] = CM177_118;
  M[17][11] = CM1711_118;
  M[17][13] = CM1713_118;
  M[17][14] = CM1714_118;
  M[17][15] = CM1715_118;
  M[17][16] = CM1716_118;
  M[17][17] = CM1717_118;
  M[18][1] = CM218_1;
  M[18][5] = CM218_5;
  M[18][6] = CM218_6;
  M[18][7] = CM218_7;
  M[18][11] = CM218_11;
  M[18][13] = CM218_13;
  M[18][14] = CM218_14;
  M[18][15] = CM218_15;
  M[18][16] = CM218_16;
  M[18][18] = s->In[5][18];
  M[19][1] = CM191_120;
  M[19][2] = CM192_120;
  M[19][3] = CM193_120;
  M[19][4] = CM194_120;
  M[19][5] = CM195_120;
  M[19][6] = CM196_120;
  M[19][7] = CM197_120;
  M[19][11] = CM1911_120;
  M[19][19] = CM1919_120;
  M[19][20] = CM2019_121;
  M[19][21] = CM2119_222;
  M[19][22] = CM2219_323;
  M[19][23] = CM2319_124;
  M[19][24] = CM224_19;
  M[20][1] = CM201_121;
  M[20][2] = CM202_121;
  M[20][3] = CM203_121;
  M[20][4] = CM204_121;
  M[20][5] = CM205_121;
  M[20][6] = CM206_121;
  M[20][7] = CM207_121;
  M[20][11] = CM2011_121;
  M[20][19] = CM2019_121;
  M[20][20] = CM2020_121;
  M[20][21] = CM2120_222;
  M[20][22] = CM2220_323;
  M[20][23] = CM2320_124;
  M[20][24] = CM224_20;
  M[21][1] = CM211_222;
  M[21][2] = CM212_222;
  M[21][3] = CM213_222;
  M[21][4] = CM214_222;
  M[21][5] = CM215_222;
  M[21][6] = CM216_222;
  M[21][7] = CM217_222;
  M[21][11] = CM2111_222;
  M[21][19] = CM2119_222;
  M[21][20] = CM2120_222;
  M[21][21] = CM2121_222;
  M[21][22] = CM2221_323;
  M[21][23] = CM2321_124;
  M[21][24] = CM224_21;
  M[22][1] = CM221_323;
  M[22][2] = CM222_323;
  M[22][3] = CM223_323;
  M[22][4] = CM224_323;
  M[22][5] = CM225_323;
  M[22][6] = CM226_323;
  M[22][7] = CM227_323;
  M[22][11] = CM2211_323;
  M[22][19] = CM2219_323;
  M[22][20] = CM2220_323;
  M[22][21] = CM2221_323;
  M[22][22] = CM2222_323;
  M[22][23] = CM2322_124;
  M[22][24] = CM224_22;
  M[23][1] = CM231_124;
  M[23][5] = CM235_124;
  M[23][6] = CM236_124;
  M[23][7] = CM237_124;
  M[23][11] = CM2311_124;
  M[23][19] = CM2319_124;
  M[23][20] = CM2320_124;
  M[23][21] = CM2321_124;
  M[23][22] = CM2322_124;
  M[23][23] = CM2323_124;
  M[24][1] = CM224_1;
  M[24][5] = CM224_5;
  M[24][6] = CM224_6;
  M[24][7] = CM224_7;
  M[24][11] = CM224_11;
  M[24][19] = CM224_19;
  M[24][20] = CM224_20;
  M[24][21] = CM224_21;
  M[24][22] = CM224_22;
  M[24][24] = s->In[5][24];
  c[1] = CF1_32;
  c[2] = FF4_15;
  c[3] = FF4_25;
  c[4] = FF5_36;
  c[5] = CF5_36;
  c[6] = CF6_17;
  c[7] = CF27;
  c[8] = CF28;
  c[9] = CF29;
  c[10] = -s->trq[3][10];
  c[11] = CF211;
  c[12] = CF112;
  c[13] = CF113;
  c[14] = CF14_115;
  c[15] = CF15_216;
  c[16] = CF316;
  c[17] = CF17_118;
  c[18] = CF218;
  c[19] = CF119;
  c[20] = CF20_121;
  c[21] = CF21_222;
  c[22] = CF322;
  c[23] = CF23_124;
  c[24] = CF224;
  c[25] = CF25_226;
  c[26] = -s->trq[3][26];
  c[27] = CF227;
  c[28] = CF28_329;
  c[29] = -s->trq[1][29];
  c[30] = CF30_331;
  c[31] = -s->trq[1][31];

// ====== END Task 0 ====== 


}
 


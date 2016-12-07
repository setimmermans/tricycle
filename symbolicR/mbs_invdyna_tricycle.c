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
//	==> Function : F 2 : Inverse Dynamics : RNEA
//	==> Flops complexity : 1641
//
//	==> Generation Time :  0.020 seconds
//	==> Post-Processing :  0.020 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
 
void mbs_invdyna(double *Qq,
MbsData *s, double tsim)

// double Qq[31];
{ 
 
#include "mbs_invdyna_tricycle.h" 
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

// = = Block_0_1_0_0_0_0 = = 
 
// Forward Kinematics 

  ALPHA13 = qdd[2]-q[2]*qd[1]*qd[1]-(2.0)*qd[1]*qd[3];
  ALPHA23 = qdd[3]-q[3]*qd[1]*qd[1]+(2.0)*qd[1]*qd[2];
  ALPHA34 = qdd[4]-s->g[3];
  OM35 = qd[1]+qd[5];
  ALPHA15 = ALPHA13*C5+ALPHA23*S5;
  ALPHA25 = -(ALPHA13*S5-ALPHA23*C5);
  OM36 = OM35*C6;
  OMp36 = -(qd[6]*OM35*S6-qdd[5]*C6);
  ALPHA26 = ALPHA25*C6+ALPHA34*S6;
  ALPHA36 = -(ALPHA25*S6-ALPHA34*C6);
  OM17 = qd[6]*C7-OM36*S7;
  OM27 = qd[7]+OM35*S6;
  OM37 = qd[6]*S7+OM36*C7;
  OMp17 = C7*(qdd[6]-qd[7]*OM36)-S7*(OMp36+qd[6]*qd[7]);
  OMp27 = qdd[7]+qd[6]*OM35*C6+qdd[5]*S6;
  OMp37 = C7*(OMp36+qd[6]*qd[7])+S7*(qdd[6]-qd[7]*OM36);
  BS17 = -(OM27*OM27+OM37*OM37);
  BS37 = OM17*OM37;
  BS97 = -(OM17*OM17+OM27*OM27);
  BETA37 = BS37+OMp27;
  BETA47 = OMp37+OM17*OM27;
  BETA67 = OM27*OM37-OMp17;
  BETA77 = BS37-OMp27;
  ALPHA17 = ALPHA15*C7-ALPHA36*S7;
  ALPHA37 = ALPHA15*S7+ALPHA36*C7;
  OM18 = OM17*C8-OM37*S8;
  OM28 = qd[8]+OM27;
  OM38 = OM17*S8+OM37*C8;
  OMp18 = C8*(OMp17-qd[8]*OM37)-S8*(OMp37+qd[8]*OM17);
  OMp28 = qdd[8]+OMp27;
  OMp38 = C8*(OMp37+qd[8]*OM17)+S8*(OMp17-qd[8]*OM37);
  BS18 = -(OM28*OM28+OM38*OM38);
  BETA78 = OM18*OM38-OMp28;
  ALPHA18 = C8*(ALPHA17+BETA37*s->dpt[3][2]+BS17*s->dpt[1][2])-S8*(ALPHA37+BETA77*s->dpt[1][2]+BS97*s->dpt[3][2]);
  ALPHA38 = C8*(ALPHA37+BETA77*s->dpt[1][2]+BS97*s->dpt[3][2])+S8*(ALPHA17+BETA37*s->dpt[3][2]+BS17*s->dpt[1][2]);
  OM19 = OM18*C9-OM38*S9;
  OM29 = qd[9]+OM28;
  OM39 = OM18*S9+OM38*C9;
  OM111 = OM17*C11-OM37*S11;
  OM311 = OM17*S11+OM37*C11;
  OMp111 = OMp17*C11-OMp37*S11;
  OMp311 = OMp17*S11+OMp37*C11;
  BS511 = -(OM111*OM111+OM311*OM311);
  BS611 = OM27*OM311;
  BS911 = -(OM111*OM111+OM27*OM27);
  BETA211 = OM111*OM27-OMp311;
  BETA311 = OMp27+OM111*OM311;
  BETA611 = BS611-OMp111;
  BETA811 = BS611+OMp111;
  ALPHA111 = C11*(ALPHA17+BETA37*s->dpt[3][5]+BS17*s->dpt[1][5])-S11*(ALPHA37+BETA77*s->dpt[1][5]+BS97*s->dpt[3][5]);
  ALPHA211 = ALPHA26+BETA47*s->dpt[1][5]+BETA67*s->dpt[3][5];
  ALPHA311 = C11*(ALPHA37+BETA77*s->dpt[1][5]+BS97*s->dpt[3][5])+S11*(ALPHA17+BETA37*s->dpt[3][5]+BS17*s->dpt[1][5]);
  OM112 = qd[12]+OM111;
  OM212 = OM27*C12+OM311*S12;
  OM312 = -(OM27*S12-OM311*C12);
  OMp112 = qdd[12]+OMp111;
  OMp212 = C12*(OMp27+qd[12]*OM311)+S12*(OMp311-qd[12]*OM27);
  OM113 = qd[13]+OM111;
  OM213 = OM27*C13+OM311*S13;
  OM313 = -(OM27*S13-OM311*C13);
  OMp113 = qdd[13]+OMp111;
  OMp213 = C13*(OMp27+qd[13]*OM311)+S13*(OMp311-qd[13]*OM27);
  OMp313 = C13*(OMp311-qd[13]*OM27)-S13*(OMp27+qd[13]*OM311);
  BS513 = -(OM113*OM113+OM313*OM313);
  BETA813 = OMp113+OM213*OM313;
  ALPHA213 = C13*(ALPHA211+BETA611*s->dpt[3][12]+BS511*s->dpt[2][12])+S13*(ALPHA311+BETA811*s->dpt[2][12]+BS911*
 s->dpt[3][12]);
  ALPHA313 = C13*(ALPHA311+BETA811*s->dpt[2][12]+BS911*s->dpt[3][12])-S13*(ALPHA211+BETA611*s->dpt[3][12]+BS511*
 s->dpt[2][12]);
  OM114 = qd[14]+OM113;
  OM314 = -(OM213*S14-OM313*C14);
  OMp114 = qdd[14]+OMp113;
  OMp314 = C14*(OMp313-qd[14]*OM213)-S14*(OMp213+qd[14]*OM313);
  ALPHA114 = ALPHA111+BETA211*s->dpt[2][12]+BETA311*s->dpt[3][12]-s->dpt[2][17]*(OMp313-OM113*OM213);
  ALPHA214 = C14*(ALPHA213+BS513*s->dpt[2][17])+S14*(ALPHA313+BETA813*s->dpt[2][17]);
  ALPHA314 = C14*(ALPHA313+BETA813*s->dpt[2][17])-S14*(ALPHA213+BS513*s->dpt[2][17]);
  OM115 = OM114*C15-OM314*S15;
  OM215 = qd[15]+OM213*C14+OM313*S14;
  OMp115 = C15*(OMp114-qd[15]*OM314)-S15*(OMp314+qd[15]*OM114);
  OMp215 = qdd[15]+C14*(OMp213+qd[14]*OM313)+S14*(OMp313-qd[14]*OM213);
  ALPHA115 = ALPHA114*C15-ALPHA314*S15;
  ALPHA315 = ALPHA114*S15+ALPHA314*C15;
  OM116 = OM115*C16+OM215*S16;
  OM216 = -(OM115*S16-OM215*C16);
  OM316 = qd[16]+OM114*S15+OM314*C15;
  OMp116 = C16*(OMp115+qd[16]*OM215)+S16*(OMp215-qd[16]*OM115);
  OMp216 = C16*(OMp215-qd[16]*OM115)-S16*(OMp115+qd[16]*OM215);
  OMp316 = qdd[16]+C15*(OMp314+qd[15]*OM114)+S15*(OMp114-qd[15]*OM314);
  BS216 = OM116*OM216;
  BS316 = OM116*OM316;
  BS516 = -(OM116*OM116+OM316*OM316);
  BS616 = OM216*OM316;
  BS916 = -(OM116*OM116+OM216*OM216);
  BETA416 = BS216+OMp316;
  BETA616 = BS616-OMp116;
  BETA716 = BS316-OMp216;
  BETA816 = BS616+OMp116;
  ALPHA216 = -(ALPHA115*S16-ALPHA214*C16);
  OM317 = -(OM216*S17-OM316*C17);
  OMp317 = -(OMp216*S17-OMp316*C17);
  ALPHA117 = ALPHA115*C16+ALPHA214*S16-s->dpt[1][21]*(OM216*OM216+OM316*OM316)+s->dpt[2][21]*(BS216-OMp316)+
 s->dpt[3][21]*(BS316+OMp216);
  ALPHA317 = C17*(ALPHA315+BETA716*s->dpt[1][21]+BETA816*s->dpt[2][21]+BS916*s->dpt[3][21])-S17*(ALPHA216+BETA416*
 s->dpt[1][21]+BETA616*s->dpt[3][21]+BS516*s->dpt[2][21]);
  OM118 = OM116*C18-OM317*S18;
  OM218 = qd[18]+OM216*C17+OM316*S17;
  OM318 = OM116*S18+OM317*C18;
  OM119 = qd[19]+OM111;
  OM219 = OM27*C19+OM311*S19;
  OM319 = -(OM27*S19-OM311*C19);
  OMp119 = qdd[19]+OMp111;
  OMp219 = C19*(OMp27+qd[19]*OM311)+S19*(OMp311-qd[19]*OM27);
  OMp319 = C19*(OMp311-qd[19]*OM27)-S19*(OMp27+qd[19]*OM311);
  BS519 = -(OM119*OM119+OM319*OM319);
  BETA819 = OMp119+OM219*OM319;
  ALPHA219 = C19*(ALPHA211+BETA611*s->dpt[3][13]+BS511*s->dpt[2][13])+S19*(ALPHA311+BETA811*s->dpt[2][13]+BS911*
 s->dpt[3][13]);
  ALPHA319 = C19*(ALPHA311+BETA811*s->dpt[2][13]+BS911*s->dpt[3][13])-S19*(ALPHA211+BETA611*s->dpt[3][13]+BS511*
 s->dpt[2][13]);
  OM120 = qd[20]+OM119;
  OM320 = -(OM219*S20-OM319*C20);
  OMp120 = qdd[20]+OMp119;
  OMp320 = C20*(OMp319-qd[20]*OM219)-S20*(OMp219+qd[20]*OM319);
  ALPHA120 = ALPHA111+BETA211*s->dpt[2][13]+BETA311*s->dpt[3][13]-s->dpt[2][24]*(OMp319-OM119*OM219);
  ALPHA220 = C20*(ALPHA219+BS519*s->dpt[2][24])+S20*(ALPHA319+BETA819*s->dpt[2][24]);
  ALPHA320 = C20*(ALPHA319+BETA819*s->dpt[2][24])-S20*(ALPHA219+BS519*s->dpt[2][24]);
  OM121 = OM120*C21-OM320*S21;
  OM221 = qd[21]+OM219*C20+OM319*S20;
  OMp121 = C21*(OMp120-qd[21]*OM320)-S21*(OMp320+qd[21]*OM120);
  OMp221 = qdd[21]+C20*(OMp219+qd[20]*OM319)+S20*(OMp319-qd[20]*OM219);
  ALPHA121 = ALPHA120*C21-ALPHA320*S21;
  ALPHA321 = ALPHA120*S21+ALPHA320*C21;
  OM122 = OM121*C22+OM221*S22;
  OM222 = -(OM121*S22-OM221*C22);
  OM322 = qd[22]+OM120*S21+OM320*C21;
  OMp122 = C22*(OMp121+qd[22]*OM221)+S22*(OMp221-qd[22]*OM121);
  OMp222 = C22*(OMp221-qd[22]*OM121)-S22*(OMp121+qd[22]*OM221);
  OMp322 = qdd[22]+C21*(OMp320+qd[21]*OM120)+S21*(OMp120-qd[21]*OM320);
  BS222 = OM122*OM222;
  BS322 = OM122*OM322;
  BS522 = -(OM122*OM122+OM322*OM322);
  BS622 = OM222*OM322;
  BS922 = -(OM122*OM122+OM222*OM222);
  BETA422 = BS222+OMp322;
  BETA622 = BS622-OMp122;
  BETA722 = BS322-OMp222;
  BETA822 = BS622+OMp122;
  ALPHA222 = -(ALPHA121*S22-ALPHA220*C22);
  OM323 = -(OM222*S23-OM322*C23);
  OMp323 = -(OMp222*S23-OMp322*C23);
  ALPHA123 = ALPHA121*C22+ALPHA220*S22-s->dpt[1][28]*(OM222*OM222+OM322*OM322)+s->dpt[2][28]*(BS222-OMp322)+
 s->dpt[3][28]*(BS322+OMp222);
  ALPHA323 = C23*(ALPHA321+BETA722*s->dpt[1][28]+BETA822*s->dpt[2][28]+BS922*s->dpt[3][28])-S23*(ALPHA222+BETA422*
 s->dpt[1][28]+BETA622*s->dpt[3][28]+BS522*s->dpt[2][28]);
  OM124 = OM122*C24-OM323*S24;
  OM224 = qd[24]+OM222*C23+OM322*S23;
  OM324 = OM122*S24+OM323*C24;
 
// Backward Dynamics 

  Fq230 = -(s->frc[2][31]*C31-s->frc[3][31]*S31);
  Cq230 = -(s->trq[2][31]*C31-s->trq[3][31]*S31);
  Cq330 = -(s->trq[2][31]*S31+s->trq[3][31]*C31);
  Fq228 = -(s->frc[2][29]*C29-s->frc[3][29]*S29);
  Cq228 = -(s->trq[2][29]*C29-s->trq[3][29]*S29);
  Cq328 = -(s->trq[2][29]*S29+s->trq[3][29]*C29);
  Fq127 = -(s->frc[1][27]+s->frc[1][29]*C28+s->frc[1][31]*C30+Fq228*S28+Fq230*S30);
  Fq227 = -(s->frc[2][27]+s->frc[1][29]*S28+s->frc[1][31]*S30-Fq228*C28-Fq230*C30);
  Fq327 = -(s->frc[3][27]+s->frc[2][29]*S29+s->frc[2][31]*S31+s->frc[3][29]*C29+s->frc[3][31]*C31);
  Cq127 = -(s->trq[1][27]+s->trq[1][29]*C28+s->trq[1][31]*C30+Cq228*S28+Cq230*S30);
  Cq227 = -(s->trq[2][27]+s->trq[1][29]*S28+s->trq[1][31]*S30-Cq228*C28-Cq230*C30);
  Cq327 = -(s->trq[3][27]-Cq328-Cq330);
  Fq125 = -(s->frc[1][26]*C26-s->frc[2][26]*S26);
  Fq225 = -(s->frc[1][26]*S26+s->frc[2][26]*C26);
  Cq125 = -(s->trq[1][26]*C26-s->trq[2][26]*S26);
  Cq225 = -(s->trq[1][26]*S26+s->trq[2][26]*C26);
  Fs124 = -(s->frc[1][24]-s->m[24]*(ALPHA123*C24-ALPHA323*S24));
  Fs224 = -(s->frc[2][24]-s->m[24]*(C23*(ALPHA222+BETA422*s->dpt[1][28]+BETA622*s->dpt[3][28]+BS522*s->dpt[2][28])+S23*(
 ALPHA321+BETA722*s->dpt[1][28]+BETA822*s->dpt[2][28]+BS922*s->dpt[3][28])));
  Fs324 = -(s->frc[3][24]-s->m[24]*(ALPHA123*S24+ALPHA323*C24));
  Cq124 = -(s->trq[1][24]-s->In[1][24]*(C24*(OMp122-qd[24]*OM323)-S24*(OMp323+qd[24]*OM122))+OM224*OM324*(s->In[5][24]-
 s->In[9][24]));
  Cq224 = -(s->trq[2][24]-s->In[5][24]*(qdd[24]+OMp222*C23+OMp322*S23)-OM124*OM324*(s->In[1][24]-s->In[9][24]));
  Cq324 = -(s->trq[3][24]-s->In[9][24]*(C24*(OMp323+qd[24]*OM122)+S24*(OMp122-qd[24]*OM323))+OM124*OM224*(s->In[1][24]-
 s->In[5][24]));
  Fq123 = Fs124*C24+Fs324*S24;
  Fq323 = -(Fs124*S24-Fs324*C24);
  Cq123 = Cq124*C24+Cq324*S24;
  Cq323 = -(Cq124*S24-Cq324*C24);
  Fq122 = -(s->frc[1][22]-Fq123);
  Fq222 = -(s->frc[2][22]+Fq323*S23-Fs224*C23);
  Fq322 = -(s->frc[3][22]-Fq323*C23-Fs224*S23);
  Cq122 = -(s->trq[1][22]-Cq123-s->dpt[2][28]*(Fq323*C23+Fs224*S23)-s->dpt[3][28]*(Fq323*S23-Fs224*C23));
  Cq222 = -(s->trq[2][22]-Cq224*C23+Cq323*S23-Fq123*s->dpt[3][28]+s->dpt[1][28]*(Fq323*C23+Fs224*S23));
  Cq322 = -(s->trq[3][22]-Cq224*S23-Cq323*C23+Fq123*s->dpt[2][28]+s->dpt[1][28]*(Fq323*S23-Fs224*C23));
  Fq121 = Fq122*C22-Fq222*S22;
  Fq221 = Fq122*S22+Fq222*C22;
  Cq121 = Cq122*C22-Cq222*S22;
  Cq221 = Cq122*S22+Cq222*C22;
  Fq120 = Fq121*C21+Fq322*S21;
  Fq320 = -(Fq121*S21-Fq322*C21);
  Cq120 = Cq121*C21+Cq322*S21;
  Cq320 = -(Cq121*S21-Cq322*C21);
  Fq119 = -(s->frc[1][19]-Fq120);
  Fq219 = -(s->frc[2][19]-Fq221*C20+Fq320*S20);
  Fq319 = -(s->frc[3][19]-Fq221*S20-Fq320*C20);
  Cq119 = -(s->trq[1][19]-Cq120-s->dpt[2][24]*(Fq221*S20+Fq320*C20));
  Cq219 = -(s->trq[2][19]-Cq221*C20+Cq320*S20);
  Cq319 = -(s->trq[3][19]-Cq221*S20-Cq320*C20+Fq120*s->dpt[2][24]);
  Fs118 = -(s->frc[1][18]-s->m[18]*(ALPHA117*C18-ALPHA317*S18));
  Fs218 = -(s->frc[2][18]-s->m[18]*(C17*(ALPHA216+BETA416*s->dpt[1][21]+BETA616*s->dpt[3][21]+BS516*s->dpt[2][21])+S17*(
 ALPHA315+BETA716*s->dpt[1][21]+BETA816*s->dpt[2][21]+BS916*s->dpt[3][21])));
  Fs318 = -(s->frc[3][18]-s->m[18]*(ALPHA117*S18+ALPHA317*C18));
  Cq118 = -(s->trq[1][18]-s->In[1][18]*(C18*(OMp116-qd[18]*OM317)-S18*(OMp317+qd[18]*OM116))+OM218*OM318*(s->In[5][18]-
 s->In[9][18]));
  Cq218 = -(s->trq[2][18]-s->In[5][18]*(qdd[18]+OMp216*C17+OMp316*S17)-OM118*OM318*(s->In[1][18]-s->In[9][18]));
  Cq318 = -(s->trq[3][18]-s->In[9][18]*(C18*(OMp317+qd[18]*OM116)+S18*(OMp116-qd[18]*OM317))+OM118*OM218*(s->In[1][18]-
 s->In[5][18]));
  Fq117 = Fs118*C18+Fs318*S18;
  Fq317 = -(Fs118*S18-Fs318*C18);
  Cq117 = Cq118*C18+Cq318*S18;
  Cq317 = -(Cq118*S18-Cq318*C18);
  Fq116 = -(s->frc[1][16]-Fq117);
  Fq216 = -(s->frc[2][16]+Fq317*S17-Fs218*C17);
  Fq316 = -(s->frc[3][16]-Fq317*C17-Fs218*S17);
  Cq116 = -(s->trq[1][16]-Cq117-s->dpt[2][21]*(Fq317*C17+Fs218*S17)-s->dpt[3][21]*(Fq317*S17-Fs218*C17));
  Cq216 = -(s->trq[2][16]-Cq218*C17+Cq317*S17-Fq117*s->dpt[3][21]+s->dpt[1][21]*(Fq317*C17+Fs218*S17));
  Cq316 = -(s->trq[3][16]-Cq218*S17-Cq317*C17+Fq117*s->dpt[2][21]+s->dpt[1][21]*(Fq317*S17-Fs218*C17));
  Fq115 = Fq116*C16-Fq216*S16;
  Fq215 = Fq116*S16+Fq216*C16;
  Cq115 = Cq116*C16-Cq216*S16;
  Cq215 = Cq116*S16+Cq216*C16;
  Fq114 = Fq115*C15+Fq316*S15;
  Fq314 = -(Fq115*S15-Fq316*C15);
  Cq114 = Cq115*C15+Cq316*S15;
  Cq314 = -(Cq115*S15-Cq316*C15);
  Fq113 = -(s->frc[1][13]-Fq114);
  Fq213 = -(s->frc[2][13]-Fq215*C14+Fq314*S14);
  Fq313 = -(s->frc[3][13]-Fq215*S14-Fq314*C14);
  Cq113 = -(s->trq[1][13]-Cq114-s->dpt[2][17]*(Fq215*S14+Fq314*C14));
  Cq213 = -(s->trq[2][13]-Cq215*C14+Cq314*S14);
  Cq313 = -(s->trq[3][13]-Cq215*S14-Cq314*C14+Fq114*s->dpt[2][17]);
  Fs112 = -(s->frc[1][12]-s->m[12]*(ALPHA111+s->l[3][12]*(OMp212+OM112*OM312)));
  Fs212 = -(s->frc[2][12]-s->m[12]*(ALPHA211*C12+ALPHA311*S12-s->l[3][12]*(OMp112-OM212*OM312)));
  Fs312 = -(s->frc[3][12]+s->m[12]*(ALPHA211*S12-ALPHA311*C12+s->l[3][12]*(OM112*OM112+OM212*OM212)));
  Cq112 = -(s->trq[1][12]-s->In[1][12]*OMp112+s->In[5][12]*OM212*OM312+Fs212*s->l[3][12]);
  Cq212 = -(s->trq[2][12]-s->In[1][12]*OM112*OM312-s->In[5][12]*OMp212-Fs112*s->l[3][12]);
  Cq312 = -(s->trq[3][12]+OM112*OM212*(s->In[1][12]-s->In[5][12]));
  Fq111 = -(s->frc[1][11]-Fq113-Fq119-Fs112);
  Fq211 = -(s->frc[2][11]-Fq213*C13-Fq219*C19+Fq313*S13+Fq319*S19-Fs212*C12+Fs312*S12);
  Fq311 = -(s->frc[3][11]-Fq213*S13-Fq219*S19-Fq313*C13-Fq319*C19-Fs212*S12-Fs312*C12);
  Cq111 = -(s->trq[1][11]-Cq112-Cq113-Cq119-s->dpt[2][12]*(Fq213*S13+Fq313*C13)-s->dpt[2][13]*(Fq219*S19+Fq319*C19)+
 s->dpt[3][12]*(Fq213*C13-Fq313*S13)+s->dpt[3][13]*(Fq219*C19-Fq319*S19));
  Cq211 = -(s->trq[2][11]-Cq212*C12-Cq213*C13-Cq219*C19+Cq312*S12+Cq313*S13+Cq319*S19-Fq113*s->dpt[3][12]-Fq119*
 s->dpt[3][13]);
  Cq311 = -(s->trq[3][11]-Cq212*S12-Cq213*S13-Cq219*S19-Cq312*C12-Cq313*C13-Cq319*C19+Fq113*s->dpt[2][12]+Fq119*
 s->dpt[2][13]);
  Fq19 = -(s->frc[1][9]+s->frc[1][10]*C10-s->frc[2][10]*S10-s->m[9]*(C9*(ALPHA18+BS18*s->dpt[1][8])-S9*(ALPHA38+BETA78*
 s->dpt[1][8])));
  Fq29 = -(s->frc[2][9]+s->frc[1][10]*S10+s->frc[2][10]*C10-s->m[9]*(ALPHA26+BETA47*s->dpt[1][2]+BETA67*s->dpt[3][2]+
 s->dpt[1][8]*(OMp38+OM18*OM28)));
  Fq39 = -(s->frc[3][10]+s->frc[3][9]-s->m[9]*(C9*(ALPHA38+BETA78*s->dpt[1][8])+S9*(ALPHA18+BS18*s->dpt[1][8])));
  Cq19 = -(s->trq[1][9]-s->In[1][9]*(C9*(OMp18-qd[9]*OM38)-S9*(OMp38+qd[9]*OM18))+s->trq[1][10]*C10-s->trq[2][10]*S10+
 OM29*OM39*(s->In[5][9]-s->In[9][9])-s->dpt[3][11]*(s->frc[1][10]*S10+s->frc[2][10]*C10));
  Cq29 = -(s->trq[2][9]-s->In[5][9]*(qdd[9]+OMp28)+s->trq[1][10]*S10+s->trq[2][10]*C10-OM19*OM39*(s->In[1][9]-
 s->In[9][9])+s->dpt[3][11]*(s->frc[1][10]*C10-s->frc[2][10]*S10));
  Cq39 = -(s->trq[3][10]+s->trq[3][9]-s->In[9][9]*(C9*(OMp38+qd[9]*OM18)+S9*(OMp18-qd[9]*OM38))+OM19*OM29*(s->In[1][9]-
 s->In[5][9]));
  Fq18 = -(s->frc[1][8]-Fq19*C9-Fq39*S9);
  Fq28 = -(s->frc[2][8]-Fq29);
  Fq38 = -(s->frc[3][8]+Fq19*S9-Fq39*C9);
  Cq18 = -(s->trq[1][8]-Cq19*C9-Cq39*S9);
  Cq28 = -(s->trq[2][8]-Cq29-s->dpt[1][8]*(Fq19*S9-Fq39*C9));
  Cq38 = -(s->trq[3][8]+Cq19*S9-Cq39*C9-Fq29*s->dpt[1][8]);
  Fs17 = -(s->frc[1][7]-s->m[7]*(ALPHA17+BETA37*s->l[3][7]+BS17*s->l[1][7]));
  Fs27 = -(s->frc[2][7]-s->m[7]*(ALPHA26+BETA47*s->l[1][7]+BETA67*s->l[3][7]));
  Fs37 = -(s->frc[3][7]-s->m[7]*(ALPHA37+BETA77*s->l[1][7]+BS97*s->l[3][7]));
  Fq17 = Fs17-s->frc[3][26]*S25+Fq111*C11+Fq125*C25+Fq127*C27+Fq18*C8+Fq311*S11+Fq327*S27+Fq38*S8;
  Fq27 = Fq211+Fq225+Fq227+Fq28+Fs27;
  Fq37 = Fs37-s->frc[3][26]*C25-Fq111*S11-Fq125*S25-Fq127*S27-Fq18*S8+Fq311*C11+Fq327*C27+Fq38*C8;
  Cq17 = -(s->trq[1][7]-s->In[1][7]*OMp17-s->In[3][7]*OMp37+s->trq[3][26]*S25-Cq111*C11-Cq125*C25-Cq127*C27-Cq18*C8-
 Cq311*S11-Cq327*S27-Cq38*S8+Fq211*s->dpt[3][5]+Fq225*s->dpt[3][6]+Fq227*s->dpt[3][7]+Fq28*s->dpt[3][2]+Fs27*s->l[3][7]-OM27*
 (s->In[3][7]*OM17-s->In[5][7]*OM37+s->In[9][7]*OM37));
  Cq27 = -(s->trq[2][7]-Cq211-Cq225-Cq227-Cq28-s->In[5][7]*OMp27-Fs17*s->l[3][7]+Fs37*s->l[1][7]+OM17*(s->In[3][7]*OM17+
 s->In[9][7]*OM37)-OM37*(s->In[1][7]*OM17+s->In[3][7]*OM37)-s->dpt[1][2]*(Fq18*S8-Fq38*C8)-s->dpt[1][5]*(Fq111*S11-Fq311*C11)
 -s->dpt[1][6]*(s->frc[3][26]*C25+Fq125*S25)-s->dpt[1][7]*(Fq127*S27-Fq327*C27)-s->dpt[3][2]*(Fq18*C8+Fq38*S8)-s->dpt[3][5]*(
 Fq111*C11+Fq311*S11)+s->dpt[3][6]*(s->frc[3][26]*S25-Fq125*C25)-s->dpt[3][7]*(Fq127*C27+Fq327*S27));
  Cq37 = -(s->trq[3][7]-s->In[3][7]*OMp17-s->In[9][7]*OMp37+s->trq[3][26]*C25+Cq111*S11+Cq125*S25+Cq127*S27+Cq18*S8-
 Cq311*C11-Cq327*C27-Cq38*C8-Fq211*s->dpt[1][5]-Fq225*s->dpt[1][6]-Fq227*s->dpt[1][7]-Fq28*s->dpt[1][2]-Fs27*s->l[1][7]+OM27*
 (s->In[1][7]*OM17+s->In[3][7]*OM37-s->In[5][7]*OM17));
  Fq16 = Fq17*C7+Fq37*S7;
  Fq36 = -(Fq17*S7-Fq37*C7);
  Cq16 = Cq17*C7+Cq37*S7;
  Fq25 = Fq27*C6-Fq36*S6;
  Fq35 = Fq27*S6+Fq36*C6;
  Cq35 = Cq27*S6-C6*(Cq17*S7-Cq37*C7);
  Fq14 = Fq16*C5-Fq25*S5;
  Fq24 = Fq16*S5+Fq25*C5;
  Cq31 = Cq35+q[2]*Fq24-q[3]*Fq14;

// = = Block_0_2_0_0_0_0 = = 
 
// Symbolic Outputs  

  Qq[1] = Cq31;
  Qq[2] = Fq14;
  Qq[3] = Fq24;
  Qq[4] = Fq35;
  Qq[5] = Cq35;
  Qq[6] = Cq16;
  Qq[7] = Cq27;
  Qq[8] = Cq28;
  Qq[9] = Cq29;
  Qq[10] = -s->trq[3][10];
  Qq[11] = Cq211;
  Qq[12] = Cq112;
  Qq[13] = Cq113;
  Qq[14] = Cq114;
  Qq[15] = Cq215;
  Qq[16] = Cq316;
  Qq[17] = Cq117;
  Qq[18] = Cq218;
  Qq[19] = Cq119;
  Qq[20] = Cq120;
  Qq[21] = Cq221;
  Qq[22] = Cq322;
  Qq[23] = Cq123;
  Qq[24] = Cq224;
  Qq[25] = Cq225;
  Qq[26] = -s->trq[3][26];
  Qq[27] = Cq227;
  Qq[28] = Cq328;
  Qq[29] = -s->trq[1][29];
  Qq[30] = Cq330;
  Qq[31] = -s->trq[1][31];

// ====== END Task 0 ====== 


}
 


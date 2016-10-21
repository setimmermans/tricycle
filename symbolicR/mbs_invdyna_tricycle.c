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
//	==> Generation Date : Fri Oct 21 09:51:50 2016
//
//	==> Project name : tricycle
//	==> using XML input file 
//
//	==> Number of joints : 30
//
//	==> Function : F 2 : Inverse Dynamics : RNEA
//	==> Flops complexity : 1611
//
//	==> Generation Time :  0.030 seconds
//	==> Post-Processing :  0.010 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
 
void mbs_invdyna(double *Qq,
MbsData *s, double tsim)

// double Qq[30];
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
  OM110 = OM17*C10-OM37*S10;
  OM310 = OM17*S10+OM37*C10;
  OMp110 = OMp17*C10-OMp37*S10;
  OMp310 = OMp17*S10+OMp37*C10;
  BS510 = -(OM110*OM110+OM310*OM310);
  BS610 = OM27*OM310;
  BS910 = -(OM110*OM110+OM27*OM27);
  BETA210 = OM110*OM27-OMp310;
  BETA310 = OMp27+OM110*OM310;
  BETA610 = BS610-OMp110;
  BETA810 = BS610+OMp110;
  ALPHA110 = C10*(ALPHA17+BETA37*s->dpt[3][5]+BS17*s->dpt[1][5])-S10*(ALPHA37+BETA77*s->dpt[1][5]+BS97*s->dpt[3][5]);
  ALPHA210 = ALPHA26+BETA47*s->dpt[1][5]+BETA67*s->dpt[3][5];
  ALPHA310 = C10*(ALPHA37+BETA77*s->dpt[1][5]+BS97*s->dpt[3][5])+S10*(ALPHA17+BETA37*s->dpt[3][5]+BS17*s->dpt[1][5]);
  OM111 = qd[11]+OM110;
  OM211 = OM27*C11+OM310*S11;
  OM311 = -(OM27*S11-OM310*C11);
  OMp111 = qdd[11]+OMp110;
  OMp211 = C11*(OMp27+qd[11]*OM310)+S11*(OMp310-qd[11]*OM27);
  OM112 = qd[12]+OM110;
  OM212 = OM27*C12+OM310*S12;
  OM312 = -(OM27*S12-OM310*C12);
  OMp112 = qdd[12]+OMp110;
  OMp212 = C12*(OMp27+qd[12]*OM310)+S12*(OMp310-qd[12]*OM27);
  OMp312 = C12*(OMp310-qd[12]*OM27)-S12*(OMp27+qd[12]*OM310);
  BS512 = -(OM112*OM112+OM312*OM312);
  BETA812 = OMp112+OM212*OM312;
  ALPHA212 = C12*(ALPHA210+BETA610*s->dpt[3][11]+BS510*s->dpt[2][11])+S12*(ALPHA310+BETA810*s->dpt[2][11]+BS910*
 s->dpt[3][11]);
  ALPHA312 = C12*(ALPHA310+BETA810*s->dpt[2][11]+BS910*s->dpt[3][11])-S12*(ALPHA210+BETA610*s->dpt[3][11]+BS510*
 s->dpt[2][11]);
  OM113 = qd[13]+OM112;
  OM313 = -(OM212*S13-OM312*C13);
  OMp113 = qdd[13]+OMp112;
  OMp313 = C13*(OMp312-qd[13]*OM212)-S13*(OMp212+qd[13]*OM312);
  ALPHA113 = ALPHA110+BETA210*s->dpt[2][11]+BETA310*s->dpt[3][11]-s->dpt[2][16]*(OMp312-OM112*OM212);
  ALPHA213 = C13*(ALPHA212+BS512*s->dpt[2][16])+S13*(ALPHA312+BETA812*s->dpt[2][16]);
  ALPHA313 = C13*(ALPHA312+BETA812*s->dpt[2][16])-S13*(ALPHA212+BS512*s->dpt[2][16]);
  OM114 = OM113*C14-OM313*S14;
  OM214 = qd[14]+OM212*C13+OM312*S13;
  OMp114 = C14*(OMp113-qd[14]*OM313)-S14*(OMp313+qd[14]*OM113);
  OMp214 = qdd[14]+C13*(OMp212+qd[13]*OM312)+S13*(OMp312-qd[13]*OM212);
  ALPHA114 = ALPHA113*C14-ALPHA313*S14;
  ALPHA314 = ALPHA113*S14+ALPHA313*C14;
  OM115 = OM114*C15+OM214*S15;
  OM215 = -(OM114*S15-OM214*C15);
  OM315 = qd[15]+OM113*S14+OM313*C14;
  OMp115 = C15*(OMp114+qd[15]*OM214)+S15*(OMp214-qd[15]*OM114);
  OMp215 = C15*(OMp214-qd[15]*OM114)-S15*(OMp114+qd[15]*OM214);
  OMp315 = qdd[15]+C14*(OMp313+qd[14]*OM113)+S14*(OMp113-qd[14]*OM313);
  BS215 = OM115*OM215;
  BS315 = OM115*OM315;
  BS515 = -(OM115*OM115+OM315*OM315);
  BS615 = OM215*OM315;
  BS915 = -(OM115*OM115+OM215*OM215);
  BETA415 = BS215+OMp315;
  BETA615 = BS615-OMp115;
  BETA715 = BS315-OMp215;
  BETA815 = BS615+OMp115;
  ALPHA215 = -(ALPHA114*S15-ALPHA213*C15);
  OM316 = -(OM215*S16-OM315*C16);
  OMp316 = -(OMp215*S16-OMp315*C16);
  ALPHA116 = ALPHA114*C15+ALPHA213*S15-s->dpt[1][20]*(OM215*OM215+OM315*OM315)+s->dpt[2][20]*(BS215-OMp315)+
 s->dpt[3][20]*(BS315+OMp215);
  ALPHA316 = C16*(ALPHA314+BETA715*s->dpt[1][20]+BETA815*s->dpt[2][20]+BS915*s->dpt[3][20])-S16*(ALPHA215+BETA415*
 s->dpt[1][20]+BETA615*s->dpt[3][20]+BS515*s->dpt[2][20]);
  OM117 = OM115*C17-OM316*S17;
  OM217 = qd[17]+OM215*C16+OM315*S16;
  OM317 = OM115*S17+OM316*C17;
  OM118 = qd[18]+OM110;
  OM218 = OM27*C18+OM310*S18;
  OM318 = -(OM27*S18-OM310*C18);
  OMp118 = qdd[18]+OMp110;
  OMp218 = C18*(OMp27+qd[18]*OM310)+S18*(OMp310-qd[18]*OM27);
  OMp318 = C18*(OMp310-qd[18]*OM27)-S18*(OMp27+qd[18]*OM310);
  BS518 = -(OM118*OM118+OM318*OM318);
  BETA818 = OMp118+OM218*OM318;
  ALPHA218 = C18*(ALPHA210+BETA610*s->dpt[3][12]+BS510*s->dpt[2][12])+S18*(ALPHA310+BETA810*s->dpt[2][12]+BS910*
 s->dpt[3][12]);
  ALPHA318 = C18*(ALPHA310+BETA810*s->dpt[2][12]+BS910*s->dpt[3][12])-S18*(ALPHA210+BETA610*s->dpt[3][12]+BS510*
 s->dpt[2][12]);
  OM119 = qd[19]+OM118;
  OM319 = -(OM218*S19-OM318*C19);
  OMp119 = qdd[19]+OMp118;
  OMp319 = C19*(OMp318-qd[19]*OM218)-S19*(OMp218+qd[19]*OM318);
  ALPHA119 = ALPHA110+BETA210*s->dpt[2][12]+BETA310*s->dpt[3][12]-s->dpt[2][23]*(OMp318-OM118*OM218);
  ALPHA219 = C19*(ALPHA218+BS518*s->dpt[2][23])+S19*(ALPHA318+BETA818*s->dpt[2][23]);
  ALPHA319 = C19*(ALPHA318+BETA818*s->dpt[2][23])-S19*(ALPHA218+BS518*s->dpt[2][23]);
  OM120 = OM119*C20-OM319*S20;
  OM220 = qd[20]+OM218*C19+OM318*S19;
  OMp120 = C20*(OMp119-qd[20]*OM319)-S20*(OMp319+qd[20]*OM119);
  OMp220 = qdd[20]+C19*(OMp218+qd[19]*OM318)+S19*(OMp318-qd[19]*OM218);
  ALPHA120 = ALPHA119*C20-ALPHA319*S20;
  ALPHA320 = ALPHA119*S20+ALPHA319*C20;
  OM121 = OM120*C21+OM220*S21;
  OM221 = -(OM120*S21-OM220*C21);
  OM321 = qd[21]+OM119*S20+OM319*C20;
  OMp121 = C21*(OMp120+qd[21]*OM220)+S21*(OMp220-qd[21]*OM120);
  OMp221 = C21*(OMp220-qd[21]*OM120)-S21*(OMp120+qd[21]*OM220);
  OMp321 = qdd[21]+C20*(OMp319+qd[20]*OM119)+S20*(OMp119-qd[20]*OM319);
  BS221 = OM121*OM221;
  BS321 = OM121*OM321;
  BS521 = -(OM121*OM121+OM321*OM321);
  BS621 = OM221*OM321;
  BS921 = -(OM121*OM121+OM221*OM221);
  BETA421 = BS221+OMp321;
  BETA621 = BS621-OMp121;
  BETA721 = BS321-OMp221;
  BETA821 = BS621+OMp121;
  ALPHA221 = -(ALPHA120*S21-ALPHA219*C21);
  OM322 = -(OM221*S22-OM321*C22);
  OMp322 = -(OMp221*S22-OMp321*C22);
  ALPHA122 = ALPHA120*C21+ALPHA219*S21-s->dpt[1][27]*(OM221*OM221+OM321*OM321)+s->dpt[2][27]*(BS221-OMp321)+
 s->dpt[3][27]*(BS321+OMp221);
  ALPHA322 = C22*(ALPHA320+BETA721*s->dpt[1][27]+BETA821*s->dpt[2][27]+BS921*s->dpt[3][27])-S22*(ALPHA221+BETA421*
 s->dpt[1][27]+BETA621*s->dpt[3][27]+BS521*s->dpt[2][27]);
  OM123 = OM121*C23-OM322*S23;
  OM223 = qd[23]+OM221*C22+OM321*S22;
  OM323 = OM121*S23+OM322*C23;
 
// Backward Dynamics 

  Fq229 = -(s->frc[2][30]*C30-s->frc[3][30]*S30);
  Cq229 = -(s->trq[2][30]*C30-s->trq[3][30]*S30);
  Cq329 = -(s->trq[2][30]*S30+s->trq[3][30]*C30);
  Fq227 = -(s->frc[2][28]*C28-s->frc[3][28]*S28);
  Cq227 = -(s->trq[2][28]*C28-s->trq[3][28]*S28);
  Cq327 = -(s->trq[2][28]*S28+s->trq[3][28]*C28);
  Fq126 = -(s->frc[1][26]+s->frc[1][28]*C27+s->frc[1][30]*C29+Fq227*S27+Fq229*S29);
  Fq226 = -(s->frc[2][26]+s->frc[1][28]*S27+s->frc[1][30]*S29-Fq227*C27-Fq229*C29);
  Fq326 = -(s->frc[3][26]+s->frc[2][28]*S28+s->frc[2][30]*S30+s->frc[3][28]*C28+s->frc[3][30]*C30);
  Cq126 = -(s->trq[1][26]+s->trq[1][28]*C27+s->trq[1][30]*C29+Cq227*S27+Cq229*S29);
  Cq226 = -(s->trq[2][26]+s->trq[1][28]*S27+s->trq[1][30]*S29-Cq227*C27-Cq229*C29);
  Cq326 = -(s->trq[3][26]-Cq327-Cq329);
  Fq124 = -(s->frc[1][25]*C25-s->frc[2][25]*S25);
  Fq224 = -(s->frc[1][25]*S25+s->frc[2][25]*C25);
  Cq124 = -(s->trq[1][25]*C25-s->trq[2][25]*S25);
  Cq224 = -(s->trq[1][25]*S25+s->trq[2][25]*C25);
  Fs123 = -(s->frc[1][23]-s->m[23]*(ALPHA122*C23-ALPHA322*S23));
  Fs223 = -(s->frc[2][23]-s->m[23]*(C22*(ALPHA221+BETA421*s->dpt[1][27]+BETA621*s->dpt[3][27]+BS521*s->dpt[2][27])+S22*(
 ALPHA320+BETA721*s->dpt[1][27]+BETA821*s->dpt[2][27]+BS921*s->dpt[3][27])));
  Fs323 = -(s->frc[3][23]-s->m[23]*(ALPHA122*S23+ALPHA322*C23));
  Cq123 = -(s->trq[1][23]-s->In[1][23]*(C23*(OMp121-qd[23]*OM322)-S23*(OMp322+qd[23]*OM121))+OM223*OM323*(s->In[5][23]-
 s->In[9][23]));
  Cq223 = -(s->trq[2][23]-s->In[5][23]*(qdd[23]+OMp221*C22+OMp321*S22)-OM123*OM323*(s->In[1][23]-s->In[9][23]));
  Cq323 = -(s->trq[3][23]-s->In[9][23]*(C23*(OMp322+qd[23]*OM121)+S23*(OMp121-qd[23]*OM322))+OM123*OM223*(s->In[1][23]-
 s->In[5][23]));
  Fq122 = Fs123*C23+Fs323*S23;
  Fq322 = -(Fs123*S23-Fs323*C23);
  Cq122 = Cq123*C23+Cq323*S23;
  Cq322 = -(Cq123*S23-Cq323*C23);
  Fq121 = -(s->frc[1][21]-Fq122);
  Fq221 = -(s->frc[2][21]+Fq322*S22-Fs223*C22);
  Fq321 = -(s->frc[3][21]-Fq322*C22-Fs223*S22);
  Cq121 = -(s->trq[1][21]-Cq122-s->dpt[2][27]*(Fq322*C22+Fs223*S22)-s->dpt[3][27]*(Fq322*S22-Fs223*C22));
  Cq221 = -(s->trq[2][21]-Cq223*C22+Cq322*S22-Fq122*s->dpt[3][27]+s->dpt[1][27]*(Fq322*C22+Fs223*S22));
  Cq321 = -(s->trq[3][21]-Cq223*S22-Cq322*C22+Fq122*s->dpt[2][27]+s->dpt[1][27]*(Fq322*S22-Fs223*C22));
  Fq120 = Fq121*C21-Fq221*S21;
  Fq220 = Fq121*S21+Fq221*C21;
  Cq120 = Cq121*C21-Cq221*S21;
  Cq220 = Cq121*S21+Cq221*C21;
  Fq119 = Fq120*C20+Fq321*S20;
  Fq319 = -(Fq120*S20-Fq321*C20);
  Cq119 = Cq120*C20+Cq321*S20;
  Cq319 = -(Cq120*S20-Cq321*C20);
  Fq118 = -(s->frc[1][18]-Fq119);
  Fq218 = -(s->frc[2][18]-Fq220*C19+Fq319*S19);
  Fq318 = -(s->frc[3][18]-Fq220*S19-Fq319*C19);
  Cq118 = -(s->trq[1][18]-Cq119-s->dpt[2][23]*(Fq220*S19+Fq319*C19));
  Cq218 = -(s->trq[2][18]-Cq220*C19+Cq319*S19);
  Cq318 = -(s->trq[3][18]-Cq220*S19-Cq319*C19+Fq119*s->dpt[2][23]);
  Fs117 = -(s->frc[1][17]-s->m[17]*(ALPHA116*C17-ALPHA316*S17));
  Fs217 = -(s->frc[2][17]-s->m[17]*(C16*(ALPHA215+BETA415*s->dpt[1][20]+BETA615*s->dpt[3][20]+BS515*s->dpt[2][20])+S16*(
 ALPHA314+BETA715*s->dpt[1][20]+BETA815*s->dpt[2][20]+BS915*s->dpt[3][20])));
  Fs317 = -(s->frc[3][17]-s->m[17]*(ALPHA116*S17+ALPHA316*C17));
  Cq117 = -(s->trq[1][17]-s->In[1][17]*(C17*(OMp115-qd[17]*OM316)-S17*(OMp316+qd[17]*OM115))+OM217*OM317*(s->In[5][17]-
 s->In[9][17]));
  Cq217 = -(s->trq[2][17]-s->In[5][17]*(qdd[17]+OMp215*C16+OMp315*S16)-OM117*OM317*(s->In[1][17]-s->In[9][17]));
  Cq317 = -(s->trq[3][17]-s->In[9][17]*(C17*(OMp316+qd[17]*OM115)+S17*(OMp115-qd[17]*OM316))+OM117*OM217*(s->In[1][17]-
 s->In[5][17]));
  Fq116 = Fs117*C17+Fs317*S17;
  Fq316 = -(Fs117*S17-Fs317*C17);
  Cq116 = Cq117*C17+Cq317*S17;
  Cq316 = -(Cq117*S17-Cq317*C17);
  Fq115 = -(s->frc[1][15]-Fq116);
  Fq215 = -(s->frc[2][15]+Fq316*S16-Fs217*C16);
  Fq315 = -(s->frc[3][15]-Fq316*C16-Fs217*S16);
  Cq115 = -(s->trq[1][15]-Cq116-s->dpt[2][20]*(Fq316*C16+Fs217*S16)-s->dpt[3][20]*(Fq316*S16-Fs217*C16));
  Cq215 = -(s->trq[2][15]-Cq217*C16+Cq316*S16-Fq116*s->dpt[3][20]+s->dpt[1][20]*(Fq316*C16+Fs217*S16));
  Cq315 = -(s->trq[3][15]-Cq217*S16-Cq316*C16+Fq116*s->dpt[2][20]+s->dpt[1][20]*(Fq316*S16-Fs217*C16));
  Fq114 = Fq115*C15-Fq215*S15;
  Fq214 = Fq115*S15+Fq215*C15;
  Cq114 = Cq115*C15-Cq215*S15;
  Cq214 = Cq115*S15+Cq215*C15;
  Fq113 = Fq114*C14+Fq315*S14;
  Fq313 = -(Fq114*S14-Fq315*C14);
  Cq113 = Cq114*C14+Cq315*S14;
  Cq313 = -(Cq114*S14-Cq315*C14);
  Fq112 = -(s->frc[1][12]-Fq113);
  Fq212 = -(s->frc[2][12]-Fq214*C13+Fq313*S13);
  Fq312 = -(s->frc[3][12]-Fq214*S13-Fq313*C13);
  Cq112 = -(s->trq[1][12]-Cq113-s->dpt[2][16]*(Fq214*S13+Fq313*C13));
  Cq212 = -(s->trq[2][12]-Cq214*C13+Cq313*S13);
  Cq312 = -(s->trq[3][12]-Cq214*S13-Cq313*C13+Fq113*s->dpt[2][16]);
  Fs111 = -(s->frc[1][11]-s->m[11]*(ALPHA110+s->l[3][11]*(OMp211+OM111*OM311)));
  Fs211 = -(s->frc[2][11]-s->m[11]*(ALPHA210*C11+ALPHA310*S11-s->l[3][11]*(OMp111-OM211*OM311)));
  Fs311 = -(s->frc[3][11]+s->m[11]*(ALPHA210*S11-ALPHA310*C11+s->l[3][11]*(OM111*OM111+OM211*OM211)));
  Cq111 = -(s->trq[1][11]-s->In[1][11]*OMp111+s->In[5][11]*OM211*OM311+Fs211*s->l[3][11]);
  Cq211 = -(s->trq[2][11]-s->In[1][11]*OM111*OM311-s->In[5][11]*OMp211-Fs111*s->l[3][11]);
  Cq311 = -(s->trq[3][11]+OM111*OM211*(s->In[1][11]-s->In[5][11]));
  Fq110 = -(s->frc[1][10]-Fq112-Fq118-Fs111);
  Fq210 = -(s->frc[2][10]-Fq212*C12-Fq218*C18+Fq312*S12+Fq318*S18-Fs211*C11+Fs311*S11);
  Fq310 = -(s->frc[3][10]-Fq212*S12-Fq218*S18-Fq312*C12-Fq318*C18-Fs211*S11-Fs311*C11);
  Cq110 = -(s->trq[1][10]-Cq111-Cq112-Cq118-s->dpt[2][11]*(Fq212*S12+Fq312*C12)-s->dpt[2][12]*(Fq218*S18+Fq318*C18)+
 s->dpt[3][11]*(Fq212*C12-Fq312*S12)+s->dpt[3][12]*(Fq218*C18-Fq318*S18));
  Cq210 = -(s->trq[2][10]-Cq211*C11-Cq212*C12-Cq218*C18+Cq311*S11+Cq312*S12+Cq318*S18-Fq112*s->dpt[3][11]-Fq118*
 s->dpt[3][12]);
  Cq310 = -(s->trq[3][10]-Cq211*S11-Cq212*S12-Cq218*S18-Cq311*C11-Cq312*C12-Cq318*C18+Fq112*s->dpt[2][11]+Fq118*
 s->dpt[2][12]);
  Fs19 = -(s->frc[1][9]-s->m[9]*(C9*(ALPHA18+BS18*s->dpt[1][8])-S9*(ALPHA38+BETA78*s->dpt[1][8])));
  Fs29 = -(s->frc[2][9]-s->m[9]*(ALPHA26+BETA47*s->dpt[1][2]+BETA67*s->dpt[3][2]+s->dpt[1][8]*(OMp38+OM18*OM28)));
  Fs39 = -(s->frc[3][9]-s->m[9]*(C9*(ALPHA38+BETA78*s->dpt[1][8])+S9*(ALPHA18+BS18*s->dpt[1][8])));
  Cq19 = -(s->trq[1][9]-s->In[1][9]*(C9*(OMp18-qd[9]*OM38)-S9*(OMp38+qd[9]*OM18))+OM29*OM39*(s->In[5][9]-s->In[9][9]));
  Cq29 = -(s->trq[2][9]-s->In[5][9]*(qdd[9]+OMp28)-OM19*OM39*(s->In[1][9]-s->In[9][9]));
  Cq39 = -(s->trq[3][9]-s->In[9][9]*(C9*(OMp38+qd[9]*OM18)+S9*(OMp18-qd[9]*OM38))+OM19*OM29*(s->In[1][9]-s->In[5][9]));
  Fq18 = -(s->frc[1][8]-Fs19*C9-Fs39*S9);
  Fq28 = -(s->frc[2][8]-Fs29);
  Fq38 = -(s->frc[3][8]+Fs19*S9-Fs39*C9);
  Cq18 = -(s->trq[1][8]-Cq19*C9-Cq39*S9);
  Cq28 = -(s->trq[2][8]-Cq29-s->dpt[1][8]*(Fs19*S9-Fs39*C9));
  Cq38 = -(s->trq[3][8]+Cq19*S9-Cq39*C9-Fs29*s->dpt[1][8]);
  Fs17 = -(s->frc[1][7]-s->m[7]*(ALPHA17+BETA37*s->l[3][7]+BS17*s->l[1][7]));
  Fs27 = -(s->frc[2][7]-s->m[7]*(ALPHA26+BETA47*s->l[1][7]+BETA67*s->l[3][7]));
  Fs37 = -(s->frc[3][7]-s->m[7]*(ALPHA37+BETA77*s->l[1][7]+BS97*s->l[3][7]));
  Fq17 = Fs17-s->frc[3][25]*S24+Fq110*C10+Fq124*C24+Fq126*C26+Fq18*C8+Fq310*S10+Fq326*S26+Fq38*S8;
  Fq27 = Fq210+Fq224+Fq226+Fq28+Fs27;
  Fq37 = Fs37-s->frc[3][25]*C24-Fq110*S10-Fq124*S24-Fq126*S26-Fq18*S8+Fq310*C10+Fq326*C26+Fq38*C8;
  Cq17 = -(s->trq[1][7]-s->In[1][7]*OMp17-s->In[3][7]*OMp37+s->trq[3][25]*S24-Cq110*C10-Cq124*C24-Cq126*C26-Cq18*C8-
 Cq310*S10-Cq326*S26-Cq38*S8+Fq210*s->dpt[3][5]+Fq224*s->dpt[3][6]+Fq226*s->dpt[3][7]+Fq28*s->dpt[3][2]+Fs27*s->l[3][7]-OM27*
 (s->In[3][7]*OM17-s->In[5][7]*OM37+s->In[9][7]*OM37));
  Cq27 = -(s->trq[2][7]-Cq210-Cq224-Cq226-Cq28-s->In[5][7]*OMp27-Fs17*s->l[3][7]+Fs37*s->l[1][7]+OM17*(s->In[3][7]*OM17+
 s->In[9][7]*OM37)-OM37*(s->In[1][7]*OM17+s->In[3][7]*OM37)-s->dpt[1][2]*(Fq18*S8-Fq38*C8)-s->dpt[1][5]*(Fq110*S10-Fq310*C10)
 -s->dpt[1][6]*(s->frc[3][25]*C24+Fq124*S24)-s->dpt[1][7]*(Fq126*S26-Fq326*C26)-s->dpt[3][2]*(Fq18*C8+Fq38*S8)-s->dpt[3][5]*(
 Fq110*C10+Fq310*S10)+s->dpt[3][6]*(s->frc[3][25]*S24-Fq124*C24)-s->dpt[3][7]*(Fq126*C26+Fq326*S26));
  Cq37 = -(s->trq[3][7]-s->In[3][7]*OMp17-s->In[9][7]*OMp37+s->trq[3][25]*C24+Cq110*S10+Cq124*S24+Cq126*S26+Cq18*S8-
 Cq310*C10-Cq326*C26-Cq38*C8-Fq210*s->dpt[1][5]-Fq224*s->dpt[1][6]-Fq226*s->dpt[1][7]-Fq28*s->dpt[1][2]-Fs27*s->l[1][7]+OM27*
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
  Qq[10] = Cq210;
  Qq[11] = Cq111;
  Qq[12] = Cq112;
  Qq[13] = Cq113;
  Qq[14] = Cq214;
  Qq[15] = Cq315;
  Qq[16] = Cq116;
  Qq[17] = Cq217;
  Qq[18] = Cq118;
  Qq[19] = Cq119;
  Qq[20] = Cq220;
  Qq[21] = Cq321;
  Qq[22] = Cq122;
  Qq[23] = Cq223;
  Qq[24] = Cq224;
  Qq[25] = -s->trq[3][25];
  Qq[26] = Cq226;
  Qq[27] = Cq327;
  Qq[28] = -s->trq[1][28];
  Qq[29] = Cq329;
  Qq[30] = -s->trq[1][30];

// ====== END Task 0 ====== 


}
 


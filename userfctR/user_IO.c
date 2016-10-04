/*===========================================================================*
  *
  *  user_sf_IO.c
  *
  *  Project:    PendulumSpringC
  *
  *  Generation date: 14-Nov-2014 18:28:15
  *
  *  (c) Universite catholique de Louvain
  *      2, Place du Levant
  *      1348 Louvain-la-Neuve
  *  http://www.robotran.be//
  *
 /*===========================================================================*/

#include <stdlib.h>
#include "user_IO.h"


UserIO* mbs_new_user_IO(UserIoInfo* ioInfo)
{
    UserIO *uvs;


	int i=0;
    //
    uvs = (UserIO*) malloc(sizeof(UserIO));

	// controll
	uvs->deltaFD = 0.0;
	uvs->deltaFD_old = 0.0;
	uvs->deltaFD_deri = 0.0;

	uvs->thetaD=0.0;
	uvs->thetaD_theo = 0.0;
	uvs->dpsiD = 0.0;
	uvs->QsteerD = 0.0;

	uvs->delta_sw_FF = 0.0;
	uvs->delta_sw_FB = 0.0;

	uvs->dirdyn_ft_lt_steer = 0.0;
	uvs->dirdyn_ft_rt_steer = 0.0;
	uvs->deltaF= 0.0;


	// integration/ derivative variables
	uvs->thetaD_deri=0.0;
	uvs->thetaD_old=0.0;




	uvs->a_lat=0.0;

	uvs->EnergyC=0.0;
	uvs->e=0.0;
	uvs->e_d=0.0;
	uvs->e_old=0.0;
	uvs->curve_f=0.0;

	uvs->vD=0.0;
	uvs->steeringTorque=0.0;

	uvs->steering_d=0.0;
	uvs->steering_old=0.0;

	uvs->equil_ft_lt_camber = 0.0;
	uvs->equil_ft_lt_toe = 0.0;
	uvs->equil_ft_rt_camber = 0.0;
	uvs->equil_ft_rt_toe = 0.0;


	uvs->distance = 0.0;
	uvs->distance_old = 0.0;
	uvs->distance_deri = 0.0;




	uvs->TC=0; // 1=DTC, 2=STC 3=DSTC

	// state-feedback controll



	// uvs->lpk = (MbsLpk*)malloc(sizeof(MbsLpk)); Done in the new_lpk function... !
	uvs->Kr = NULL;
	uvs->Gr = NULL;
	uvs->Mr = NULL;

	uvs->PtrSensor=(MbsSensor*) malloc(sizeof(MbsSensor));

	uvs->lqc = (LQController*)malloc(sizeof(LQController));

	uvs->lqc->lq_K = NULL;
	//uvs->lqc->lq_filename = NULL;
	uvs->lqc->a = 0.0;
	uvs->lqc->b = 0.0;
	uvs->lqc->c = 0.0;

	uvs->qstc = (Quasistatic*)malloc(sizeof(Quasistatic));

	uvs->qstc->Qsteer = NULL;
	uvs->qstc->R = NULL;
	uvs->qstc->theta = NULL;
	uvs->qstc->V = NULL;
	uvs->qstc->Vstart = 0.0;
	uvs->qstc->Rstart = 0.0;
	uvs->qstc->deltaV = 0.0;
	uvs->qstc->deltaR = 0.0;
	uvs->qstc->Vend = 0.0;
	uvs->qstc->Rend = 0.0;

	uvs->cvs=(Controller*) malloc(sizeof(Controller));

	uvs->cvs->P=0.0;
	uvs->cvs->I=0.0;
	uvs->cvs->D=0.0;

	return uvs;
}

void mbs_delete_user_IO(UserIO *uvs)
{
	free(uvs->cvs);
    free(uvs);
}




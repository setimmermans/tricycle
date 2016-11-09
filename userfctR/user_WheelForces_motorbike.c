//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------

#include "math.h"

#include "MBSdef.h"
#include "MBSfun.h"
#include "mbs_data.h"
#include "user_model.h"
#include "mbs_motorbike_contact.h"
#include "user_all_id.h"

#include <stdio.h>

//#include "userDef.h" //ob1

void user_WheelForces_motorbike(double pen, double rz, double anglis, double angcamb,
	                  double gliss, double Vctz, double dxF[4],
					  MbsData *mbs_data, double tsim, int iwhl,
					  double* SWr)
{
	TIRE_param_strct *TIRE_param;
	double Fwhl[4]={0.0, 0.0, 0.0, 0.0};
	double Mwhl[4]={0.0, 0.0, 0.0, 0.0};
	int res_1;


	double Fwhl_bis[4]={0.0, 0.0, 0.0, 0.0};
	double Mwhl_bis[4]={0.0, 0.0, 0.0, 0.0};

	double K ;//rnom, ;
	//K=mbs_data->user_model->Param_pneu.k;

	/*switch(iwhl){

		case sensorGround_FL_id+1:
		case sensorGround_FR_id+1:
		case sensorGround_RL_id+1:
		case sensorGround_RR_id+1:
			 K    = MBSdata->user_model->wheel.tyreK;
		break;
	}*/
	K = mbs_data->user_model->wheel_ft.K_tire;
	if (iwhl == F_wheel_ft_lt_id || F_wheel_ft_rt_id == iwhl)
	{
		TIRE_param = init_TIRE_param_strct_120_70();
	}
	else if (iwhl == F_wheel_rr_id)
	{
		TIRE_param = init_TIRE_param_strct_140_70_beta();
	}





	//printf( "F1 : bakker  %f \n",pen);
	///system("pause");

	if (mbs_data->process == 2) // equil static 
	{
		Fwhl[3] = K * pen; 
	}
	//else if(mbs_data->process == 12) // equil quasistatic
	//{
	//	//printf("F1 : bakker  %f \n", pen); 
	//	Fwhl[3] = K * pen;
	//	TIRE_param=init_TIRE_param_strct_120_70();
	//	res_1 = mbs_motorbike_contact(Fwhl,Mwhl,anglis,angcamb,gliss,TIRE_param);
	//}
	else // supposed dirdyn 
	{
		
		if (pen > 0.0)
		{
			Fwhl[3] = K*pen;

			// modèle Calspan
			//mbs_calspan(Fwhl,Mwhl, anglis,angcamb);

			// modèle Bakker
			//	mbs_bakker(Fwhl,Mwhl,anglis,angcamb,gliss);
			//Fwhl_bis[3] = Fwhl[3];

			TIRE_param = init_TIRE_param_strct_120_70();
			res_1 = mbs_motorbike_contact(Fwhl, Mwhl, anglis, angcamb, gliss, TIRE_param);
			

		}
	}
	/*
	switch(iwhl)
	{
		case F_wheel_ft_lt_id+1: 
			MBSdata->user_IO->Fn_wheel_ft_lt = Fwhl[3];
			MBSdata->user_IO->Flat_wheel_ft_lt = Fwhl[2];
			break;
		case F_wheel_ft_rt_id+1:
			MBSdata->user_IO->Fn_wheel_ft_rt = Fwhl[3];
			MBSdata->user_IO->Flat_wheel_ft_rt = Fwhl[2];
			break;
		case F_wheel_rr_id+1:
			MBSdata->user_IO->Fn_wheel_rr = Fwhl[3];
			MBSdata->user_IO->Flat_wheel_rr = Fwhl[2];
			break;
	}
	*/


	SWr[1] = Fwhl[1];// /pen*MBSdata->user_model->tyreModelRatio;
	SWr[2] = Fwhl[2]; // /pen*MBSdata->user_model->tyreModelRatio;
	SWr[3] = Fwhl[3];
	SWr[4] = Mwhl[1];
	SWr[5] = Mwhl[2];
	SWr[6] = Mwhl[3]; // /pen*MBSdata->user_model->tyreModelRatio;
	SWr[7] = dxF[1];
	SWr[8] = dxF[2];
	SWr[9] = dxF[3];

		
	/*
    printf( "F1 : bakker  %f , my %f\n",Fwhl[1],Fwhl_bis[1]);
	printf( "F2 : bakker  %f , my %f\n",Fwhl[2],Fwhl_bis[2]);
	printf( "F3 : bakker  %f , my %f\n",Fwhl[3],Fwhl_bis[3]);

	printf( "M1 : bakker  %f , my %f\n",Mwhl[1],Mwhl_bis[1]);
	printf( "M2 : bakker  %f , my %f\n",Mwhl[2],Mwhl_bis[2]);
	printf( "M3 : bakker  %f , my %f\n",Mwhl[3],Mwhl_bis[3]);

	system("pause");//*/


	// Sample code:
	// MBSdata->user_var.FWheel_rad[iwhl] = whlWr[3];
	// MBSdata->user_var.FWheel_lat[iwhl] = whlWr[2];
	// MBSdata->user_var.FWheel_long[iwhl] = whlWr[1];
}
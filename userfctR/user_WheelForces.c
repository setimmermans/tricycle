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
#include "user_all_id.h"
#include "mbs_motorbike_contact.h"

#include <stdio.h>


void user_WheelForces(double rz, double anglis, double angcamb,
	                  double gliss, double Vctz, double dxF[4],
					  MbsData *mbs_data, double tsim, int iwhl,
					  double* SWr)
{
	// double Fwhl[4]={0.0, 0.0, 0.0, 0.0};
	// double Mwhl[4]={0.0, 0.0, 0.0, 0.0};
	// int res_1;

	// double Fwhl_bis[4]={0.0, 0.0, 0.0, 0.0};
	// double Mwhl_bis[4]={0.0, 0.0, 0.0, 0.0};


	// double rnom, K;
	
// //	K=mbs_data->user_model->Param_pneu.k;
// //	rnom=mbs_data->user_model->Param_pneu.R0;

	// /*switch(iwhl)
	// {

		// case sensorGround_FL_id+1:
		// case sensorGround_FR_id+1:
		// case sensorGround_RL_id+1:
		// case sensorGround_RR_id+1:
			 // K    = MBSdata->user_model->wheel.tyreK;
			 // rnom = MBSdata->user_model->wheel.wheelR;
		// break;
	// }*/

	// //rnom = mbs_data->user_model->wheel.wheelR;
	// if (mbs_data->process == 2) // equil
	// {
		
		// Fwhl[3] = K *(rnom-rz);
		// printf("Froce wtf =%f \n", Fwhl[3]);
	// }else
	// {
	// if(rnom > rz)
	// {
	// // modèle ressort
	// //	Fwhl[3] = K * (rnom - rz)*cos(angcamb);
	// // modèle Calspan
	// //mbs_calspan(Fwhl,Mwhl, anglis,angcamb);
	// // modèle Bakker
		// //mbs_bakker(Fwhl,Mwhl,anglis,angcamb,gliss);
		// //Fwhl_bis[3] = Fwhl[3];
		// //res_1 = mbs_motorbike_contact(Fwhl_bis,Mwhl_bis,anglis,angcamb,gliss);
	// }
	// }

	// SWr[1] = Fwhl[1];
	// SWr[2] = Fwhl[2];
	// SWr[3] = Fwhl[3];
	// SWr[4] = Mwhl[1];
	// SWr[5] = Mwhl[2];
	// SWr[6] = Mwhl[3];
	// SWr[7] = dxF[1];
	// SWr[8] = dxF[2];
	// SWr[9] = dxF[3];	
	// /*
    // printf( "F1 : bakker  %f , my %f\n",Fwhl[1],Fwhl_bis[1]);
	// printf( "F2 : bakker  %f , my %f\n",Fwhl[2],Fwhl_bis[2]);
	// printf( "F3 : bakker  %f , my %f\n",Fwhl[3],Fwhl_bis[3]);

	// printf( "M1 : bakker  %f , my %f\n",Mwhl[1],Mwhl_bis[1]);
	// printf( "M2 : bakker  %f , my %f\n",Mwhl[2],Mwhl_bis[2]);
	// printf( "M3 : bakker  %f , my %f\n",Mwhl[3],Mwhl_bis[3]);

	// system("pause");//*/


	// // Sample code:
	// // MBSdata->user_var.FWheel_rad[iwhl] = whlWr[3];
	// // MBSdata->user_var.FWheel_lat[iwhl] = whlWr[2];
	// // MBSdata->user_var.FWheel_long[iwhl] = whlWr[1];
}
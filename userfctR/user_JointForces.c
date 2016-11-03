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
#include "mbs_data.h"
#include "user_model.h"
#include "user_all_id.h"
#include "user_IO.h"
#include "set_output.h"
#include "Controleur.h"

double* user_JointForces(MbsData *mbs_data, double tsim)
{
	
		if(mbs_data->user_IO->modeTC == 2) //DTC
		{// controleur sur le tilt
		mbs_data->Qq[R1_pendulum_id] = my_controleur(mbs_data, tsim, mbs_data->speed_ref, mbs_data->q[R3_steering_fork_id]);
		}
		else if (mbs_data->user_IO->modeTC == 1) //STC
		{
			mbs_data->Qq[R3_steering_fork_id] = my_controleur_stc(mbs_data, tsim, mbs_data->speed_ref, mbs_data->q[R3_steering_fork_id]);
			printf("my steering controle = %f \n ", mbs_data->Qq[R3_steering_fork_id]);
		}
		else
		{//printf("no joint force \n");
			// no torque control 
		}
	//perturbation
	/*if (tsim > 5.0 && tsim < 5.03)
	{
		mbs_data->Qq[R3_steering_fork_id] = -1;
	}*/


	/*-- End of user code --*/
	return mbs_data->Qq;
}






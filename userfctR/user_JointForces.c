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
	
		double my_speed;
		my_speed = mbs_data->qd[T1_body_id] * mbs_data->qd[T1_body_id] + mbs_data->qd[T2_body_id] * mbs_data->qd[T2_body_id];

		if(mbs_data->user_IO->modeTC == 2) //DTC
		{// controleur sur le tilt
		mbs_data->Qq[R1_pendulum_id] = my_controleur(mbs_data, tsim, my_speed, mbs_data->q[R3_steering_fork_id]);
		}
		else
		{
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






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


double smoothstep(double p0, double pstep, double t0, double tstep, double t)
{
	double x;
	x = (t - t0) / tstep;
	// Evaluate polynomial
	return p0 + (x*x*(3 - 2 * x))*pstep;
}
int	signQ(double d)
{
	if (d > 0.0)
	{
		return 1;
	}
	else if (d == 0)
	{
		return 0;
	}
	else
	{
		return -1;
	}
}


double* user_JointForces(MbsData *mbs_data, double tsim)
{

	int i, j, k;
	double *ref;
	int nqu;

	int iV;
	double V;


	// feedbackloop			

	// linearization, real time  
	if (mbs_data->process == 8)
	{

		
		// no joint force or maybe a controller on the wheel rotational speed
	}
	// control, real time 
	else if (mbs_data->process == 3)
	{
		//	mbs_data->user_IO->qd_lin[0] = 0.0;
		//mbs_data->user_IO->qd_lin[R1_body_id-1] = 0.0;
		//	mbs_data->user_IO->q_lin[0] = 0.0;
		mbs_data->Qq[R3_steering_fork_id] = 0.0;
		
		if (mbs_data->user_IO->modeTC == 1)
		{
		//	uqd_apply_LQI(mbs_data, lqr, lqr->indRobControlled, mbs_data->ux[mbs_data->user_model->lqi.e[1]]);
		}
		else
		{
	
		//	uqd_apply_LQI(mbs_data, lqr, lqr->indRobControlled, mbs_data->ux[mbs_data->user_model->lqi.e[1]]);
		}
	
		//uqd_DTC_PID(mbs_data);
		// mbs_data->Qq[R3_steering_fork_id] = mbs_data->user_IO->dummy_Qsteer;

		mbs_data->Qq[R2_wheel_rr_id] = 0.0;// mbs_data->user_IO->inter->Qqu[8 - 1];//mbs_data->user_IO->dummy_Qqrear;

		// DO NOT FORGET TO APPLY OTHER JOINT FORCES !

	}
	// perturbation 
	if (tsim > 1 && tsim < 1.1)
	{
		mbs_data->Qq[R1_body_id] =0.0;
	}
	else
	{
		mbs_data->Qq[R1_body_id] = 0.0;
	}
	/*-- End of user code --*/

	return mbs_data->Qq;
}






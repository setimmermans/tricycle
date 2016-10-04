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



double smoothStep(double t1, double t, double t2, double v1, double v2)
{
	double x, y;
	x = (t - t1) / (t2 - t1);
	if (x < 0.0) { x = 0.0; }
	if (x > 1.0) { x = 1.0; }


	y = 3 * x*x - 2 * x*x*x;

	return  (v2 - v1)*y + v1;
}

double smoothStep_d(double t1, double t, double t2, double v1, double v2)
{
	double x, y;
	x = (t - t1) / (t2 - t1);
	y = 3 * x*x - 2 * x*x*x;

	return  (v2 - v1)*y + v1;
}

double smoothStep_dd(double t1, double t, double t2, double v1, double v2)
{
	double x, y;
	x = (t - t1) / (t2 - t1);
	y = 3 * x*x - 2 * x*x*x;

	return  (v2 - v1)*y + v1;
}



void user_DrivenJoints(MbsData *mbs_data, double tsim)
{
	if (tsim > 0.0)
	{
		if (mbs_data->user_IO->modeTC == 2) // DTC
		{
			mbs_data->q[R3_steering_fork_id] = smoothStep(2.0, tsim, 3.0, 0.0, mbs_data->user_IO->steer);
		}
		else
		{
			// no control of variables ! 
		}
	}


	/*
	double omega = _2pi / 3.0;
	double A=0.0;

	if (mbs_data->process == 3)
	{
		mbs_data->q[R3_steering_fork_id] = A*sin(tsim*omega);
		mbs_data->qd[R3_steering_fork_id] = A*omega*cos(tsim*omega);
		mbs_data->qdd[R3_steering_fork_id] = -A*omega*omega*sin(tsim*omega);
	}
	*/
}



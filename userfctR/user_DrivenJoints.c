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


void user_DrivenJoints(MbsData *mbs_data, double tsim)
{
	if (tsim > 0.0)
	{
		if (mbs_data->user_IO->modeTC == 2) // DTC
		{
			mbs_data->q[R3_steering_fork_id] = 0.0;
		}
		else
		{
			// no control of variables ! 
		}
	}

}



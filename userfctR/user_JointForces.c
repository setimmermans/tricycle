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

double* user_JointForces(MbsData *mbs_data, double tsim)
{

	
	//mbs_data->Qq[R3_steering_fork_id] = 0.0;
	//mbs_data->Qc[R3_steering_fork_id] = 0.0;
	


	//perturbation
	//if (tsim > 30.0 && tsim < 30.03)
	//{
	//	mbs_data->Qq[R3_steering_fork_id] = -0.5;
	//}

	/*-- End of user code --*/
	return mbs_data->Qq;
}






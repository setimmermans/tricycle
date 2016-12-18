	/* Modified by Sebastien, on the basis of Quentin's work 

	Objective: static and quasi static equilibrium
	+ modal analysis
*/


#include "ModalAnalysis.h"


void ModalAnalysis(MbsData *mbs_data, double V, char *filename_modal, double front_radius, double rear_radius)
{
	mbs_data->process = 4; // modal!
	MbsModal *mbs_modal = mbs_new_modal(mbs_data);

	//mbs_data->qd[T1_body_id] = V;
	//mbs_data->q[T1_body_id] = 0.0;
	//mbs_data->q[T2_body_id] = 0.0;

	//mbs_data->qd[R2_wheel_ft_lt_id] = V / front_radius;
	//mbs_data->qd[R2_wheel_ft_rt_id] = V / front_radius;
	//mbs_data->qd[R2_wheel_rr_id] = V / rear_radius; // very sensitive (need to take static eq value for nominal radii)

	// modal options (see documentations for additional options) ? 
	mbs_modal->options->verbose = 0;
	mbs_run_modal(mbs_modal, mbs_data);
		
	mbs_modal_save_result(mbs_modal, mbs_data, filename_modal);
	mbs_delete_modal(mbs_modal, mbs_data);
	
	}

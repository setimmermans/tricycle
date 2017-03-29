	/* Modified by Sebastien, on the basis of Quentin's work 

	Objective: static and quasi static equilibrium
	+ modal analysis
*/


#include "ModalAnalysis.h"


void ModalAnalysis(MbsData *mbs_data, double V, char *filename_modal, double front_radius, double rear_radius)
{
	mbs_data->process = 4; // modal!
	MbsModal *mbs_modal = mbs_new_modal(mbs_data);

	// modal options (see documentations for additional options) ? 
	mbs_modal->options->verbose = 0;
	mbs_run_modal(mbs_modal, mbs_data);
		
	mbs_modal_save_result(mbs_modal, mbs_data, filename_modal);
	mbs_delete_modal(mbs_modal, mbs_data);
	
}

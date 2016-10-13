	/* Modified by Sebastien, on the basis of Quentin's work 

	Objective: static and quasi static equilibrium
	+ modal analysis
*/

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif
#include<windows.h>
#include "mbs_part.h"
#include "mbs_load_xml.h"
#include "mbs_equil.h"
#include "mbs_modal.h"
#include "mbs_dirdyn.h"
#include "mbs_data.h"
#include "realtime.h"
#include "cmake_config.h"
#include "user_all_id.h"
#include "user_model.h"
#include "mbs_set.h"
#include "user_IO.h"
#include "mbs_sensor.h"
#include "mbs_project_interface.h"
#include "mbs_linearipk.h"
#include<stdio.h>
#include<conio.h>
#include<process.h>


void ModalAnalysis(MbsData *mbs_data, double V, char *filename_modal, double front_radius, double rear_radius)
{

	MbsModal *mbs_modal = mbs_new_modal(mbs_data);

	mbs_data->qd[T1_body_id] = V;
	mbs_data->q[T1_body_id] = 0.0;
	mbs_data->q[T2_body_id] = 0.0;

	mbs_data->qd[R2_wheel_ft_lt_id] = V / front_radius;
	mbs_data->qd[R2_wheel_ft_rt_id] = V / front_radius;
	mbs_data->qd[R2_wheel_rr_id] = V / rear_radius; // very sensitive (need to take static eq value for nominal radii)

	// modal options (see documentations for additional options) ? 

	//printf("\n\n Run modal Analysis \n");
	mbs_run_modal(mbs_modal, mbs_data);
	//printf("Print modal Result on the file for V  = %f \n", V);
	
	mbs_modal_save_result(mbs_modal, mbs_data, filename_modal);
	mbs_delete_modal(mbs_modal, mbs_data);
	
	}
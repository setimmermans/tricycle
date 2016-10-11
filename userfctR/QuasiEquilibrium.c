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


void QuasiEquilibrium(MbsData *mbs_data, double V)
{

	mbs_data->process = 1; // equil !
	
	mbs_data->qd[T1_body_id] = V;
	mbs_data->q[T1_body_id] = 0.0;
	mbs_data->q[T2_body_id] = 0.0;

	mbs_data->qd[R2_wheel_ft_lt_id] = V / 0.258591;
	mbs_data->qd[R2_wheel_ft_rt_id] = V / 0.258591;
	mbs_data->qd[R2_wheel_rr_id] = V / 0.255193; // very sensitive (need to take static eq value for nominal radii)

	MbsEquil *mbs_equil = mbs_new_equil(mbs_data);

	// equil options (see documentations for additional options)
	mbs_equil->options->senstol = 1e-01;
	mbs_equil->options->verbose = 0;
	mbs_equil->options->quasistatic = 1;
	mbs_equil->options->nquch = 4;
	mbs_equil_exchange(mbs_equil->options);
	mbs_equil->options->quch[1] = T1_body_id;
	mbs_equil->options->quch[2] = R2_wheel_ft_lt_id;
	mbs_equil->options->quch[3] = R2_wheel_rr_id;
	mbs_equil->options->quch[4] = R2_wheel_ft_rt_id;
	mbs_equil->options->xch_ptr[1] = &(mbs_data->Qq[R2_wheel_rr_id]);
	mbs_equil->options->xch_ptr[2] = &(mbs_data->qd[R2_wheel_ft_lt_id]);
	mbs_equil->options->xch_ptr[3] = &(mbs_data->qd[R2_wheel_rr_id]);
	mbs_equil->options->xch_ptr[4] = &(mbs_data->qd[R2_wheel_ft_rt_id]);
	mbs_run_equil(mbs_equil, mbs_data);

	//mbs_print_equil(mbs_equil);
	//printf("my speed = %f \n", mbs_data->qd[R2_wheel_ft_lt_id]/(3.1415*2));
	mbs_delete_equil(mbs_equil, mbs_data);
	
	}
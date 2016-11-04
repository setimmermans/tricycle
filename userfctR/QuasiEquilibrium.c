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


void QuasiEquilibrium(MbsData *mbs_data, double V, double front_radius, double rear_radius, int Toprint, double steer)
{
	MbsEquil *mbs_equil = mbs_new_equil(mbs_data);
	mbs_data->process = 5; // equil quasi ? !

	if(mbs_data->tourne == 1)
	{
		//  curve equilibrium
		printf("Curve equilibrium (tourne ==1 )\n");
		mbs_data->q[T1_body_id] = 0.0;
		mbs_data->q[T2_body_id] =  -mbs_data->Rayon;
		mbs_data->qd[R3_body_qs_id] =    V / (mbs_data->Rayon);//tourne
		printf("rayon = %f et omega = %f \n", mbs_data->Rayon, mbs_data->qd[R3_body_qs_id]);
		mbs_data->qd[T1_body_id] = 0.0; // vitesse relative par rapport a la carotte :)
		mbs_data->qd[T2_body_id] = 0.0;
		mbs_data->q[R3_body_qs_id] = 0.0;
		mbs_data->q[R3_body_id] = 0.0;

		mbs_data->qd[R2_wheel_ft_lt_id] = (V / front_radius);
		mbs_data->qd[R2_wheel_ft_rt_id] = (V / front_radius);
		mbs_data->qd[R2_wheel_rr_id] = V / rear_radius; // very sensitive (need to take static eq value for nominal radii)

		// equil options (see documentations for additional options)
		mbs_equil->options->senstol = 1e-7;
		mbs_equil->options->verbose = Toprint;
		mbs_equil->options->quasistatic = 1;
		mbs_equil->options->nquch = 5;
		mbs_equil->options->equitol = 1e-8;
		mbs_equil->options->smooth = 0.8;
		mbs_equil->options->itermax = 30;
		mbs_equil_exchange(mbs_equil->options);
		mbs_equil->options->quch[1] = T1_body_id;
		mbs_equil->options->quch[2] = R2_wheel_rr_id;
		mbs_equil->options->quch[3] = R2_wheel_ft_rt_id;
		mbs_equil->options->quch[4] = R2_wheel_ft_lt_id;
		mbs_equil->options->quch[5] = T2_body_id;


		mbs_equil->options->xch_ptr[1] = &(mbs_data->Qq[R2_wheel_rr_id]);
		mbs_equil->options->xch_ptr[2] = &(mbs_data->qd[R2_wheel_rr_id]);
		mbs_equil->options->xch_ptr[3] = &(mbs_data->qd[R2_wheel_ft_rt_id]);
		mbs_equil->options->xch_ptr[4] = &(mbs_data->qd[R2_wheel_ft_lt_id]);
		mbs_equil->options->xch_ptr[5] = &(mbs_data->Qq[R3_steering_fork_id]);


		mbs_run_equil(mbs_equil, mbs_data);
		if (Toprint == 1)
		{
			printf("q  : "); print_dvec_0(mbs_data->q, mbs_data->njoint);
			printf("qd : "); print_dvec_0(mbs_data->qd, mbs_data->njoint);
			printf("qdd: "); print_dvec_0(mbs_data->qdd, mbs_data->njoint);
			printf("Qq : "); print_dvec_0(mbs_data->Qq, mbs_data->njoint);
			printf("Tilt_ref = %f \n", mbs_data->q[R1_body_id]);
			mbs_print_equil(mbs_equil);
		}
	}
	else
	{
		printf("Tourne !=1 ==> equilibre ligne droite  \n");
		mbs_data->qd[T1_body_id] = V;
		mbs_data->q[T1_body_id] = 0.0;
		mbs_data->q[T2_body_id] = 0.0;

		mbs_data->qd[R2_wheel_ft_lt_id] = V / front_radius;
		mbs_data->qd[R2_wheel_ft_rt_id] = V / front_radius;
		mbs_data->qd[R2_wheel_rr_id] = V / rear_radius; // very sensitive (need to take static eq value for nominal radii)

	

		// equil options (see documentations for additional options)
		mbs_equil->options->senstol = 1e-1;
		mbs_equil->options->verbose = Toprint;
		mbs_equil->options->quasistatic = 1;
		mbs_equil->options->nquch = 4;
		mbs_equil->options->equitol = 1e-4;
		mbs_equil->options->itermax = 30;
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

		if (Toprint == 1)
		{
			mbs_print_equil(mbs_equil);
		}
	}
	mbs_delete_equil(mbs_equil, mbs_data);

}
/**
*
*   Universite catholique de Louvain
*   CEREM : Centre for research in mechatronics
*   http://www.robotran.be
*   Contact : info@robotran.be
*
*
*   MBsysC main script template for simple model:
*   -----------------------------------------------
*    This template loads the data file *.mbs and execute:
*      - the coordinate partitioning module
*      - the direct dynamic module (time integration of
*        equations of motion).
*    It may be adapted and completed by the user.
*
*    (c) Universite catholique de Louvain
*
* To turn this file as a C++ file, just change its extension to .cc (or .cpp).
* If you plan to use some C++ files, it is usually better that the main is compiled as a C++ function.
* Currently, most compilers do not require this, but it is a safer approach to port your code to other computers.
*/

/* Given by Quentin, modified by Sebastien 

Objective: static and quasi static equilibrium
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




int main(int argc, char const *argv[])
{

	double *q_saved_dir, *qd_saved_dir, *Qq_saved_dir;
	MbsData *mbs_data;
	MbsPart *mbs_part;
	MbsDirdyn *mbs_dirdyn;
	double simu_t;
	double V, Rmin, steer;

	UserIO *uIO;
	MbsEquil *mbs_equil;
	MbsModal *mbs_modal;

	simu_t = 1.5;
	V = 10;

	char *filename;
	char *filename_modal;

	printf("Hello tricycle MBS!\n"); 
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                     LOADING                               *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	printf("Loading the tricycle data file !\n"); 
	mbs_data = mbs_load(PROJECT_SOURCE_DIR"/../dataR/tricycle.mbs", BUILD_PATH); 
	printf("*.mbs file loaded!\n");
	uIO = mbs_data->user_IO;

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*              COORDINATE PARTITIONING                      *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	mbs_part = mbs_new_part(mbs_data);
	mbs_part->options->rowperm = 1;
	mbs_part->options->verbose = 1;
	mbs_run_part(mbs_part, mbs_data);
	mbs_delete_part(mbs_part);

	//mbs_print_data(mbs_data);
	printf("test after coordinate partionning \n");
	system("pause");

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*					 STATIC EQUILIBRIUM	                     *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	mbs_data->process = 2; // equil !
	mbs_equil = mbs_new_equil(mbs_data);

	// equil options (see documentations for additional options)
	mbs_equil->options->method = 1; //?Q

	mbs_equil->options->senstol = 1e-06;
	mbs_equil->options->relax = 0.6;
	mbs_equil->options->smooth = 1;
	mbs_equil->options->verbose = 0;

	// 1. Variable exchange quch->xch
	/*mbs_equil->options->nquch=2; // nquch = nxch
	mbs_equil_substitution(mbs_equil->options);
	mbs_equil->options->quch[1]=R2_body_id+1;
	mbs_equil->options->quch[2]=T3_body_id+1;
	mbs_equil->options->xch_ptr[1]=&(mbs_data->user_model->shock_rr.Z_0);
	mbs_equil->options->xch_ptr[2]=&(mbs_data->user_model->shock_ft.Z_0);*/

	// 2. Added variables and equations !
	/*mbs_equil->options->nxe=2;
	mbs_equil_addition(mbs_equil->options);
	mbs_equil->options->xe_ptr[1]=&(mbs_data->lrod[1]);
	mbs_equil->options->xe_ptr[2]=&(mbs_data->lrod[2]);*/

	printf("run equil\n");
	mbs_run_equil(mbs_equil, mbs_data);
	printf("print equil \n");
	mbs_print_equil(mbs_equil);
	mbs_delete_equil(mbs_equil, mbs_data);

	q_saved_dir = get_dvec_0(mbs_data->njoint + 1);
	qd_saved_dir = get_dvec_0(mbs_data->njoint + 1);
	Qq_saved_dir = get_dvec_0(mbs_data->njoint + 1);
	copy_dvec_0(&(mbs_data->q[1]), q_saved_dir, mbs_data->njoint);
	copy_dvec_0(&(mbs_data->qd[1]), qd_saved_dir, mbs_data->njoint);
	copy_dvec_0(&(mbs_data->Qq[1]), Qq_saved_dir, mbs_data->njoint);


	system("pause");



	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*					 QUASI STATIC EQUILIBRIUM	                     *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	mbs_data->process = 2; // equil !


	mbs_data->qd[T1_body_id] = V;
	mbs_data->q[T1_body_id] = 0.0;
	mbs_data->q[T2_body_id] = 0.0;

	mbs_data->qd[R2_wheel_ft_lt_id] = V / 0.258591;
	mbs_data->qd[R2_wheel_ft_rt_id] = V / 0.258591;
	mbs_data->qd[R2_wheel_rr_id] = V / 0.255193; // very sensitive (need to take static eq value for nominal radii)

	mbs_equil = mbs_new_equil(mbs_data);

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

	mbs_print_equil(mbs_equil);
	mbs_delete_equil(mbs_equil, mbs_data);
	system("pause");


	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*					 MODAL ANALYSIS                     *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	mbs_data->process = 18; // modal
	mbs_modal = mbs_new_modal(mbs_data);

	// modal options (see documentations for additional options)

//	mbs_modal->options->verbose = 0;


	printf("run modal\n");
	mbs_run_modal(mbs_modal, mbs_data);
	printf("print modal \n");
	
	mbs_modal_save_result(mbs_modal, mbs_data, "Analyse_modale\My_Modal_Analysis.txt");
	mbs_delete_modal(mbs_modal, mbs_data);


	system("pause");


	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*					LOOPS MODAL ANALYSIS ON V			             *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	double steps, speed;
	steps = 1;
	speed = 0.0;

	char *path = PROJECT_SOURCE_DIR"/../resultsR/AnalyseModale";
	filename_modal =  (char *)malloc(1 + strlen(path) + strlen("/") + 30);//"Analyse_modale\My_Modal_Analysis"; // 
	printf("\n\n Ready for LOOPS \n");
	system("pause");

	while (speed<V)
	{
		mbs_data->qd[T1_body_id] = speed;
		mbs_data->q[T1_body_id] = 0.0;
		mbs_data->q[T2_body_id] = 0.0;

		mbs_data->qd[R2_wheel_ft_lt_id] = speed / 0.258591;
		mbs_data->qd[R2_wheel_ft_rt_id] = speed / 0.258591;
		mbs_data->qd[R2_wheel_rr_id] = speed / 0.255193; // very sensitive (need to take static eq value for nominal radii)

		sprintf(filename_modal, "%s/V%3.1f.txt", path, speed);

		mbs_modal = mbs_new_modal(mbs_data);
		mbs_run_modal(mbs_modal, mbs_data);
		mbs_modal_save_result(mbs_modal, mbs_data, filename_modal);
		mbs_delete_modal(mbs_modal, mbs_data);


		speed = speed + steps;
	}



	system("pause");


	
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                   DIRECT DYNANMICS                        *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	mbs_data->user_IO->modeTC = 2;

	// initialize dirdyn with slane equilibrium
	copy_dvec_0(q_saved_dir, &(mbs_data->q[1]), mbs_data->njoint);
	copy_dvec_0(qd_saved_dir, &(mbs_data->qd[1]), mbs_data->njoint);
	copy_dvec_0(Qq_saved_dir, &(mbs_data->Qq[1]), mbs_data->njoint);
	
	if (mbs_data->user_IO->modeTC == 2) // only for DTC control where the steering is driven (to get a given curve)
	{
		mbs_set_qdriven(mbs_data, R3_steering_fork_id);
	}

	//uqd_slane_equil(mbs_data, V);
	

	mbs_data->q[T3_body_id] = 0.18;

	//------------------------------------------

	// initialize dirdyn with curve equilibrium
	//V = 10.0;
	//R = 30.0;
	//uqd_curve_equil(mbs_data, V, R, 1);
	mbs_data->qd[R3_body_id] = mbs_data->qd[R3_body_qs_id];
	mbs_data->qd[R3_body_qs_id] = 0.0;
	mbs_data->qd[T1_body_id] = V;
	mbs_data->q[T1_body_id] = 0.0;
	mbs_data->q[T2_body_id] = 0.0;
	mbs_data->user_IO->dummy_Qsteer = mbs_data->Qq[R3_steering_fork_id];
	mbs_data->user_IO->dummy_Qqrear = mbs_data->Qq[R2_wheel_rr_id];
	//------------------------------------------

	printf("\n\n Ready for dirdyn \n");
	printf("q  : "); print_dvec_0(mbs_data->q, mbs_data->njoint);
	printf("qd : "); print_dvec_0(mbs_data->qd, mbs_data->njoint);
	printf("qdd: "); print_dvec_0(mbs_data->qdd, mbs_data->njoint);
	printf("Qq : "); print_dvec_0(mbs_data->Qq, mbs_data->njoint);

	
	mbs_data->process = 3;
	mbs_dirdyn = mbs_new_dirdyn(mbs_data);

	// dirdyn options (see documentations for additional options)
	mbs_dirdyn->options->dt0 = 1e-3;
	mbs_dirdyn->options->tf = simu_t;
	mbs_dirdyn->options->save2file = 1;
	mbs_dirdyn->options->animpath = PROJECT_SOURCE_DIR"/../animationR";
	
	//mbs_dirdyn->options->realtime = 1;
	mbs_dirdyn->options->saveperiod = 10;


	//filename_dirdyn = (char *)malloc( 30); // pour save les result en fct de son Input ///////////////seb

	//char *path = PROJECT_SOURCE_DIR"/../resultsR/dirdyn";
	//filename_dirdyn = (char *)malloc(1 + strlen(path) + strlen("/") + 30);
	//
	//if (mbs_data->user_IO->modeTC == 1)
	//{
	//	sprintf(filename_dirdyn, "%s/V%3.1f/STC", path, V);
	//	mbs_dirdyn->options->respath =filename_dirdyn;
	//}
	//else
	//{
	//	sprintf(filename_dirdyn, "%s/V%3.1f/DTC", path, V);
	//	mbs_dirdyn->options->respath = filename_dirdyn;
	//}
	//

	mbs_dirdyn->options->respath = PROJECT_SOURCE_DIR"/../resultsR/dirdyn";
	mbs_run_dirdyn(mbs_dirdyn, mbs_data);
	printf("dirdyn done \n");
	mbs_delete_dirdyn(mbs_dirdyn, mbs_data);

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                   CLOSING OPERATIONS                      *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	mbs_delete_data(mbs_data);
	free(filename_modal);
	return 0;
}


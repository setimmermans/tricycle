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




int main(int argc, char const *argv[])
{

	double *q_saved_dir, *qd_saved_dir, *Qq_saved_dir;
	MbsData *mbs_data;
	MbsPart *mbs_part;
	MbsDirdyn *mbs_dirdyn;
	double simu_t;
	double V, Rmin, steer;
	double max_V;

	UserIO *uIO;
	MbsEquil *mbs_equil;
	MbsModal *mbs_modal;

	char *filename;
	char *filename_modal;


	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                    PARAMETERS                              *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	simu_t = 10.0;
	V = 6.0; // en m/s
	max_V = 30; 


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
	printf("\n\n Coordinate partionning done \n");
	system("pause");

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*					 STATIC EQUILIBRIUM	                     *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


	mbs_data->process = 2; // equil !
	mbs_equil = mbs_new_equil(mbs_data);

	// equil options (see documentations for additional options)
	mbs_equil->options->method = 1; 

	mbs_equil->options->senstol = 1e-02;
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

	printf("\n\n Run equilibrium \n");
	mbs_run_equil(mbs_equil, mbs_data);
	printf("Print equilibrium	 \n");
	mbs_print_equil(mbs_equil);
	mbs_delete_equil(mbs_equil, mbs_data);

	// store results from equil
	//q_saved_dir = get_dvec_0(mbs_data->njoint + 1);
	//qd_saved_dir = get_dvec_0(mbs_data->njoint + 1);
	//Qq_saved_dir = get_dvec_0(mbs_data->njoint + 1);
	//copy_dvec_0(&(mbs_data->q[1]), q_saved_dir, mbs_data->njoint);
	//copy_dvec_0(&(mbs_data->qd[1]), qd_saved_dir, mbs_data->njoint);
	//copy_dvec_0(&(mbs_data->Qq[1]), Qq_saved_dir, mbs_data->njoint);


	system("pause");

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*					 QUASI STATIC EQUILIBRIUM	                     *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	
	QuasiEquilibrium(mbs_data, V);
	
	system("pause");

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*					 MODAL ANALYSIS                      *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	printf("\n\n Run modal Analysis \n");
	ModalAnalysis(mbs_data, V, "Analyse_modale\My_Modal_Analysis.txt");
	printf("Print modal Result on the file for V  = %f \n", V);

	system("pause");


	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*					LOOPS MODAL ANALYSIS ON V			             *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	double steps, speed;
	steps = 0.1;
	speed = 0.1;

	char *path = PROJECT_SOURCE_DIR"/../resultsR/AnalyseModale";
	filename_modal =  (char *)malloc(1 + strlen(path) + strlen("/") + 30);
	printf("\n\n Ready for LOOPS on Modal Analysis \n");
	system("pause");

	while (speed<max_V)
	{
		QuasiEquilibrium(mbs_data, speed);
		sprintf(filename_modal, "%s/V%3.1f.txt", path, speed);
		ModalAnalysis(mbs_data, speed, filename_modal); // Analyse Modale  

		speed = speed + steps;
	}

	printf("qdd: "); print_dvec_0(mbs_data->qdd, mbs_data->njoint);
	system("pause");
		
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                   DIRECT DYNANMICS                        *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	mbs_data->user_IO->modeTC = 2;


	//------------------------------------------

	// initialize dirdyn with equilibrium
	QuasiEquilibrium(mbs_data, V);

	system("pause");
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
	
	mbs_dirdyn->options->realtime = 1;
	mbs_dirdyn->options->saveperiod = 10;


	mbs_dirdyn->options->respath = PROJECT_SOURCE_DIR"/../resultsR/dirdyn";
	mbs_run_dirdyn(mbs_dirdyn, mbs_data);
	printf(" Dirdyn done \n");
	mbs_delete_dirdyn(mbs_dirdyn, mbs_data);

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                   CLOSING OPERATIONS                      *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	mbs_delete_data(mbs_data);
	free(filename_modal);
	return 0;
}


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
#include "mbs_sensor.h"
#include<process.h>
#include "coordRobotran.h"


//#define Scaled	 
#define Normal	 

#define Tourne
#define Dirdyn	
#define DTC
//#define LoopModal	
//#define Printcoord

	

int main(int argc, char const *argv[])
{

	MbsData *mbs_data;
	MbsPart *mbs_part;
	MbsDirdyn *mbs_dirdyn;
	double simu_t;
	double V, Rmin, steer, front_radius, rear_radius, Rayon;
	double max_V;
	int Toprint;
	UserIO *uIO;
	MbsEquil *mbs_equil;
	MbsModal *mbs_modal;

	char *filename;
	char *path_modal, *path_K;
	char *filename_modal, *filename_K;
	double K_factor_init, K_factor_max;
	double steps, speed, scaling_factor, manual_scaling;


	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                    PARAMETERS                              *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	simu_t = 4;
	V = 1; // en m/s
	max_V = 10; 
	steps = 0.01;
	speed = 0.1;
	Toprint = 0;
	K_factor_init = 1.0;
	K_factor_max = 1.0;
	scaling_factor = 1.0;
	manual_scaling = 0.4;
	steer = 0.01;
	Rayon = 1;

	front_radius = 0.258591;
	rear_radius = 0.255193;

#ifdef Scaled
	front_radius =  0.104104;// 0.258591 *manual_scaling = 0.1036; //
	rear_radius =  0.10359;// 0.255193 * manual_scaling; // very sensitive (need to take static eq value for nominal radii)
#endif

	printf("Hello tricycle MBS!\n"); 
	//printf(" Nominal Radius = %f et %f \n \n", front_radius, rear_radius);

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                     LOADING                               *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	printf("Loading the tricycle data file !\n"); 
	mbs_data = mbs_load(PROJECT_SOURCE_DIR"/../dataR/tricycle.mbs", BUILD_PATH); 
	path_modal = PROJECT_SOURCE_DIR"/../resultsR/AnalyseModale";
	path_K = PROJECT_SOURCE_DIR"/../resultsR/ResultsK";
	filename_modal = (char *)malloc(1 + strlen(path_modal) + strlen("/") + 30);
	filename_K = (char *)malloc(1 + strlen(path_K) + strlen("/") + 30 + strlen("/") + 30 );
	printf("*.mbs file loaded!\n");
	uIO = mbs_data->user_IO;
	mbs_data->K_factor = K_factor_init;
	mbs_data->scaling_factor = scaling_factor;
	mbs_data->ErrorTot = 0.0;
	mbs_data->user_IO->steer = steer;
	mbs_data->speed_ref = V;
	mbs_data->tourne = 0;
	mbs_data->user_IO->modeTC = 1;
	mbs_data->Rayon = 100000; 


	mbs_set_qdriven(mbs_data, R3_steering_fork_id);

#ifdef Tourne
	mbs_data->tourne = 1;
	mbs_data->Rayon = Rayon;
#endif


	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*              SCALING					                      *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	//
	//printf("\n\n Ready to scale the system \n");
	//system("pause");
	//Scale_data(mbs_data, scaling_factor);
	//system("pause");
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
	printf("q  : "); print_dvec_0(mbs_data->q, mbs_data->njoint);
	printf("qd : "); print_dvec_0(mbs_data->qd, mbs_data->njoint);
	printf("qdd: "); print_dvec_0(mbs_data->qdd, mbs_data->njoint);
	printf("Qq : "); print_dvec_0(mbs_data->Qq, mbs_data->njoint);


	mbs_data->process = 2; // equil !
	mbs_equil = mbs_new_equil(mbs_data);
	// equil options (see documentations for additional options)
	mbs_equil->options->method = 1; 
	mbs_equil->options->senstol = 1e-01;
	mbs_equil->options->relax = 0.6;
	mbs_equil->options->smooth = 1;
	mbs_equil->options->verbose = 0;
	mbs_equil->options->equitol = 1e-2;
	printf("\n\n Run equilibrium \n");
	mbs_run_equil(mbs_equil, mbs_data);
	mbs_print_equil(mbs_equil);
	mbs_delete_equil(mbs_equil, mbs_data);


	
	// !!! add equation in user_equil_fxe fct !!!!!!!!!
	// 1. Variable exchange quch->xch 
	/*mbs_equil->options->nquch=2; // nquch = nxch
	mbs_equil_substitution(mbs_equil->options);
	mbs_equil->options->quch[1]=R2_body_id+1;
	mbs_equil->options->quch[2]=T3_body_id+1;
	mbs_equil->options->xch_ptr[1]=&(mbs_data->user_model->shock_rr.Z_0);
	mbs_equil->options->xch_ptr[2]=&(mbs_data->user_model->shock_ft.Z_0);*/
	// 2. Added variables and equations !
	//mbs_equil->options->nxe = 1;
	//mbs_equil_addition(mbs_equil->options);
	//mbs_equil->options->xe_ptr[1] = &(mbs_data->lrod[4]);

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*					ANGLES	             *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	printf("\n\n Angles Calculus \n");
	Angles(mbs_data);
	
	system("pause");

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*					 QUASI STATIC EQUILIBRIUM	             *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	#ifdef DTC
	mbs_data->user_IO->modeTC = 2;
	#endif
	
	Toprint = 1;
	printf("\n\n Run Quasistatic Equilibrium \n");
	printf("q  : "); print_dvec_0(mbs_data->q, mbs_data->njoint);
	printf("qd : "); print_dvec_0(mbs_data->qd, mbs_data->njoint);
	printf("qdd: "); print_dvec_0(mbs_data->qdd, mbs_data->njoint);
	printf("Qq : "); print_dvec_0(mbs_data->Qq, mbs_data->njoint);
	QuasiEquilibrium(mbs_data, V, front_radius, rear_radius, Toprint,steer);
	printf("q  : "); print_dvec_0(mbs_data->q, mbs_data->njoint);
	printf("qd : "); print_dvec_0(mbs_data->qd, mbs_data->njoint);
	printf("qdd: "); print_dvec_0(mbs_data->qdd, mbs_data->njoint);
	printf("Qq : "); print_dvec_0(mbs_data->Qq, mbs_data->njoint); 
	system("pause");

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*					 MODAL ANALYSIS                      *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	mbs_data->process = 4; // modal !

	printf("\n\n Run modal Analysis for V  = %f \n", V );
	ModalAnalysis(mbs_data, V, "Analyse_modale\My_Modal_Analysis.txt", front_radius, rear_radius);
	printf("q  : "); print_dvec_0(mbs_data->q, mbs_data->njoint);
	printf("qd : "); print_dvec_0(mbs_data->qd, mbs_data->njoint);
	printf("qdd: "); print_dvec_0(mbs_data->qdd, mbs_data->njoint);
	printf("Qq : "); print_dvec_0(mbs_data->Qq, mbs_data->njoint);
	system("pause");


	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*					LOOPS MODAL ANALYSIS ON  V AND K       *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

		//printf("\n\n Ready for LOOPS on Modal Analysis for V and K \n");

		//mbs_data->K_factor = K_factor_init;
		//system("pause");
		//Toprint = 0;
		//while (mbs_data->K_factor < K_factor_max)
		//{
		//	printf("  K  = %f \n", mbs_data->K_factor);
		//	while (speed < max_V)
		//	{
	//mbs_data->process = 2; // equil quasi !

		//		QuasiEquilibrium(mbs_data, speed, front_radius, rear_radius, Toprint,steer);
		//		sprintf(filename_K, "%s/K%3.1f/V%3.1f.txt", path_K,  mbs_data->K_factor, speed);
	//mbs_data->process = 4; // equil quasi !

		//		ModalAnalysis(mbs_data, speed, filename_K, front_radius, rear_radius); // Analyse Modale  

		//		speed = speed + steps;
		//	}
		//	mbs_data->K_factor = 2.0* mbs_data->K_factor;
		//	speed = 0.1;
		//}
		//printf("qdd: "); print_dvec_0(mbs_data->qdd, mbs_data->njoint);
		//system("pause");

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*					LOOPS MODAL ANALYSIS ON V		          *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifdef LoopModal

	printf("\n\n Ready for LOOPS on Modal Analysis for V \n");
	system("pause");
	Toprint = 0;
	while (speed<max_V)
	{
		mbs_data->process = 2; // equil quasi !
		QuasiEquilibrium(mbs_data, speed, front_radius, rear_radius, Toprint,steer);
		sprintf(filename_modal, "%s/V%9.2f.txt", path_modal, speed);
		mbs_data->process = 4; // modal!
		ModalAnalysis(mbs_data, speed, filename_modal, front_radius, rear_radius); // Analyse Modale  
		printf("filename = %s \n", filename_modal);
		speed = speed + steps;
	}

	printf("qdd: "); print_dvec_0(mbs_data->qdd, mbs_data->njoint);
	system("pause");

#endif
		
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                   DIRECT DYNANMICS                        *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifdef Dirdyn
	mbs_data->process = 3; // dirdyn!

	Toprint = 1;
	// initialize dirdyn with equilibrium
	QuasiEquilibrium(mbs_data, V, front_radius, rear_radius,Toprint,steer);
	
	printf("\n\n Ready for dirdyn \n");
	
	//mbs_data->q[T3_body_id] = 0.2; // impose une hauteur
	printf("q  : "); print_dvec_0(mbs_data->q, mbs_data->njoint);
	printf("qd : "); print_dvec_0(mbs_data->qd, mbs_data->njoint);
	printf("qdd: "); print_dvec_0(mbs_data->qdd, mbs_data->njoint);
	printf("Qq : "); print_dvec_0(mbs_data->Qq, mbs_data->njoint);

	system("pause");

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
#endif

	///* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	///*                   CLOSING OPERATIONS                      *
	///* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	mbs_delete_data(mbs_data);
	free(filename_modal);
	free(filename_K);


	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                   ANCHOR POINT : PRINT TO A FILE           *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifdef Printcoord
	anchor_points_coord();
#endif
	

	return 0;
}


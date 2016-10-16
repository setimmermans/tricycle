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




int main(int argc, char const *argv[])
{

	MbsData *mbs_data;
	MbsPart *mbs_part;
	MbsDirdyn *mbs_dirdyn;
	double simu_t;
	double V, Rmin, steer, front_radius, rear_radius;
	double max_V;
	int Toprint;
	UserIO *uIO;
	MbsEquil *mbs_equil;
	MbsModal *mbs_modal;

	char *filename;
	char *path_modal, *path_K;
	char *filename_modal, *filename_K;
	double K_factor_init, K_factor_max;
	double steps, speed;




	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                    PARAMETERS                              *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	simu_t = 10.0;
	V = 8.0; // en m/s
	max_V = 10; 
	front_radius = 0.258591; // a changer quand on fait le 1/3
	rear_radius = 0.255193; // very sensitive (need to take static eq value for nominal radii)
	steps = 0.1;
	speed = 0.1;
	Toprint = 0;
	K_factor_init = 1.0;
	K_factor_max = 1.0;

	printf("Hello tricycle MBS!\n"); 
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
	mbs_print_equil(mbs_equil);
	mbs_delete_equil(mbs_equil, mbs_data);


	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*					ANGLES	             *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


	double X_wheel_left, X_wheel_right, Z_wheel_left, Z_wheel_right; // wheel
	X_wheel_left = 0.0;
	X_wheel_right = 0.0;
	Z_wheel_left = 0.0;
	Z_wheel_right = 0.0;

	double X_carrrier_left, X_carrrier_right, Z_carrrier_left, Z_carrrier_right; // carrier
	X_carrrier_left = 0.0;
	X_carrrier_right = 0.0;
	Z_carrrier_left = 0.0;
	Z_carrrier_right = 0.0;

	MbsSensor psens[1];                        // Creation of a pointer to a sensor struct.
	allocate_sensor(psens, mbs_data->njoint);  // Allocate the Jacobian at the correct dimension
	init_sensor(psens, mbs_data->njoint);      // Initialize all value to zero
	

	mbs_sensor(psens, mbs_data, Sensor_wheel_ft_rt_id); // Compute the sensor wheel right
	X_wheel_right = psens->P[1];
	Z_wheel_right = psens->P[3]; 

	printf("roue droite :  X = %f,  Z = %f \n", psens->P[1], psens->P[3]);

	mbs_sensor(psens, mbs_data, Sensor_wheel_ft_lt_id); // Compute the sensor wheel left
	X_wheel_left = psens->P[1];
	Z_wheel_left = psens->P[3];
	printf("roue gauche :  X = %f,  Z = %f \n", psens->P[1], psens->P[3]);



	mbs_sensor(psens, mbs_data, sens_ft_lf_id); // Compute the sensor carrier left
	X_carrrier_left = psens->P[1];
	Z_carrrier_left = psens->P[3]; 
	printf("carrier gauche :  X = %f,  Z = %f \n", psens->P[1], psens->P[3]);

	mbs_sensor(psens, mbs_data, sens_ft_rt_id); // Compute the sensor carrier right
	X_carrrier_right = psens->P[1];
	Z_carrrier_right = psens->P[3];
	printf("carrier droit :  X = %f,  Z = %f \n", psens->P[1], psens->P[3]);

	printf("\n\n gauche : Delta X = %f, Delta Z = %f \n", X_carrrier_left - X_wheel_left, Z_carrrier_left - Z_wheel_left);   
	printf("droite : Delta X = %f, Delta Z = %f \n\n ", X_carrrier_right - X_wheel_right, Z_carrrier_right - Z_wheel_right);

	mbs_data->user_IO->equil_ft_lt_caster = atan2((Z_carrrier_left - Z_wheel_left), (X_carrrier_left - X_wheel_left));
	mbs_data->user_IO->equil_ft_rt_caster = atan2((Z_carrrier_right - Z_wheel_right), (X_carrrier_right - X_wheel_right));
	//MbsSensor *PtrSensor = mbs_dd->mbs_aux->psens;

	//// compute the sensor (position, velocity...)
	//mbs_sensor(PtrSensor, mbs_data, id);



	printf("\n\n Angles : camber gauche (carrossage) = %f , camber droite (carrossage) = %f \n", mbs_data->user_IO->equil_ft_lt_camber * 180 / 3.1415, mbs_data->user_IO->equil_ft_rt_camber * 180 / 3.1415);
	printf(" Angles : caster gauche (chasse) = %f , caster droite (chasse) = %f  \n", mbs_data->user_IO->equil_ft_lt_caster * 180 / 3.1415, mbs_data->user_IO->equil_ft_rt_caster * 180 / 3.1415);
	printf(" Angles : toe gauche (pinçage) = %f, toe droite (pinçage) = %f and steer = %f  \n \n", mbs_data->user_IO->equil_ft_rt_toe * 180 / 3.1415, mbs_data->user_IO->equil_ft_rt_toe * 180 / 3.1415, mbs_data->user_IO->dirdyn_ft_rt_steer);
	
	free_sensor(psens);                        // Free the memory (always better)
	
	system("pause");

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*					 QUASI STATIC EQUILIBRIUM	             *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	Toprint = 1;
	printf("\n\n Run Quasistatic Equilibrium \n");
	QuasiEquilibrium(mbs_data, V, front_radius, rear_radius, Toprint);
	system("pause");

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*					 MODAL ANALYSIS                      *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	printf("\n\n Run modal Analysis for V  = %f \n", V );
	ModalAnalysis(mbs_data, V, "Analyse_modale\My_Modal_Analysis.txt", front_radius, rear_radius);

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
		//		QuasiEquilibrium(mbs_data, speed, front_radius, rear_radius, Toprint);
		//		sprintf(filename_K, "%s/K%3.1f/V%3.1f.txt", path_K,  mbs_data->K_factor, speed);
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

	printf("\n\n Ready for LOOPS on Modal Analysis for V \n");
	system("pause");
	Toprint = 0;
	while (speed<max_V)
	{
		QuasiEquilibrium(mbs_data, speed, front_radius, rear_radius, Toprint);
		sprintf(filename_modal, "%s/V%3.1f.txt", path_modal, speed);
		ModalAnalysis(mbs_data, speed, filename_modal, front_radius, rear_radius); // Analyse Modale  

		speed = speed + steps;
	}

	printf("qdd: "); print_dvec_0(mbs_data->qdd, mbs_data->njoint);
	system("pause");


		
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                   DIRECT DYNANMICS                        *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	mbs_data->user_IO->modeTC = 2;
	Toprint = 1;
	// initialize dirdyn with equilibrium
	QuasiEquilibrium(mbs_data, V, front_radius, rear_radius,Toprint);

	printf("\n\n Ready for dirdyn \n");

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


	
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                   CLOSING OPERATIONS                      *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	mbs_delete_data(mbs_data);
	free(filename_modal);
	free(filename_K);


	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                   ANCHOR POINT : PRINT TO A FILE           *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	anchor_points_coord();

	return 0;
}


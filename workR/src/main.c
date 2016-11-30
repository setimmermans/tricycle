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
#include "mbs_sensor.h"
#include "Controleur.h"
#include "coordRobotran.h"
#include "MyPause.c"
#include "Angles_calculus.h"
#include "ModalAnalysis.h"
#include "QuasiEquilibrium.h"
#include "useful_functions.h"

#define Scaled	 
//#define Normal	 
	
//#define DTC
//#define STC

//#define ChgmntVariables

//#define curveEq

//#define ModalAnalysis

//#define LoopModal	
//#define LoopQuasi

//#define EntreCourbe
//#define DoubleBand
#define Dirdyn	

//#define Printcoord


	

int main(int argc, char const *argv[])
{

	MbsData *mbs_data;
	MbsPart *mbs_part;
	MbsDirdyn *mbs_dirdyn;
	double simu_t,t_start;
	double V,  steer, front_radius, rear_radius, Rayon;
	double max_V;
	int Toprint;
	UserIO *uIO;
	MbsEquil *mbs_equil;
	//MbsModal *mbs_modal;
	FILE* writing_file_R_loop = NULL;

	//char *filename;
	char *path_modal, *path_K;
	char *filename_modal, *filename_K;
	double K_factor_init, K_factor_max;
	double steps, speed_init, scaling_factor, manual_scaling;
	double R_loop_max, R_loop_init,  R_increment;

	// for curve quasistatic !
	//double *q_saved, *qd_saved;
	double *q_saved_dir, *qd_saved_dir, *Qq_saved_dir;

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                    PARAMETERS                              *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	simu_t =10; //time total simu
	t_start = 2; // tournant
	V = 1; // vitesse de simu et d'eq quasi statique en m/s
	Rayon = -100; //STC
	steer =  -(V*t_start) / (Rayon * 8); //DTC
	

	// boucle en vitesse
	max_V = 6; 
	steps = 0.01;
	speed_init = 0.1;

	Toprint = 0;

	// boucle en raideur
	K_factor_init = 1.0;
	K_factor_max = 1.0;
	scaling_factor = 1.0;
	manual_scaling = 0.4;



	//boucle Rayon et vitesse
	R_loop_max = 100; // 200 deja trop selon quentin rmin 15 pour  10m/s
	R_increment = 1;
	R_loop_init = 97;

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
	printf("steer =>%f \n", steer);
	uIO = mbs_data->user_IO;
	mbs_data->K_factor = K_factor_init;
	mbs_data->scaling_factor = scaling_factor;
	mbs_data->ErrorTot = 0.0;
	mbs_data->user_IO->steer = steer;
	mbs_data->speed_ref = V;
	mbs_data->tourne = 0;
	mbs_data->user_IO->modeTC = 0; //no control
	mbs_data->Rayon = Rayon; 
	mbs_data->EntreEnCourbe = 0;
	mbs_data->EstEnCourbe = 0;
	mbs_data->DoubleBande = 0;
	mbs_data->t_start = t_start;
	
	//printf("\n\n nJoints =%d \n", mbs_data->njoint);

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*              SCALING					                      *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	//
	//printf("\n\n Ready to scale the system \n");
	//mypause();
	//Scale_data(mbs_data, scaling_factor);
	//mypause();


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
	mypause();

	///* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	///*					 STATIC EQUILIBRIUM	                     *
	///* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	Print_q_qd_qdd_Qq(mbs_data); // Print current value of joints


	mbs_data->process = 2; // equil static //
	mbs_equil = mbs_new_equil(mbs_data);
	// equil options (see documentations for additional options)
	mbs_equil->options->method = 1; 
	mbs_equil->options->senstol = 1e-01;
	mbs_equil->options->relax = 0.6;
	mbs_equil->options->smooth = 1;
	mbs_equil->options->verbose = 1;
	mbs_equil->options->equitol = 1e-8;

	printf("\n\n Run equilibrium \n");
	mbs_run_equil(mbs_equil, mbs_data);
	mbs_print_equil(mbs_equil);
	mbs_delete_equil(mbs_equil, mbs_data);
	mypause();

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*					 STATIC EQUILIBRIUM AVEC CHGMT VARIABLES                *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifdef ChgmntVariables

	Print_q_qd_qdd_Qq(mbs_data); // Print current value of joints


	mbs_data->process = 2; // equil static //
	mbs_equil = mbs_new_equil(mbs_data);
	// equil options (see documentations for additional options)
	mbs_equil->options->method = 1;
	mbs_equil->options->senstol = 1e-01;
	mbs_equil->options->relax = 0.6;
	mbs_equil->options->smooth = 1;
	mbs_equil->options->verbose = 1;
	mbs_equil->options->equitol = 1e-8;


	// !!! add equation in user_equil_fxe fct !!!!!!!!!
	// 1. Variable exchange quch->xch 
	//mbs_equil->options->nquch=2; // nquch = nxch
	//mbs_equil_substitution(mbs_equil->options);
	//mbs_equil->options->quch[1]=R2_body_id+1;
	//mbs_equil->options->quch[2]=T3_body_id+1;
	//mbs_equil->options->xch_ptr[1]=&(mbs_data->user_model->shock_rr.Z_0);
	//mbs_equil->options->xch_ptr[2]=&(mbs_data->user_model->shock_ft.Z_0);
	// 2. Added variables and equations !
	mbs_equil->options->nxe = 3;
	mbs_equil_addition(mbs_equil->options);
	// ok to find 5 rods
	//mbs_equil->options->xe_ptr[1] = &(mbs_data->lrod[1]);
	//mbs_equil->options->xe_ptr[2] = &(mbs_data->lrod[2]);
	//mbs_equil->options->xe_ptr[3] = &(mbs_data->lrod[4]);
	//mbs_equil->options->xe_ptr[4] = &(mbs_data->lrod[5]);
	//mbs_equil->options->xe_ptr[5] = &(mbs_data->lrod[3]); 
	//
	mbs_equil->options->xe_ptr[1] = &(mbs_data->lrod[4]);
	mbs_equil->options->xe_ptr[2] = &(mbs_data->lrod[5]);
	mbs_equil->options->xe_ptr[3] = &(mbs_data->lrod[3]); // OK pour trouver R1=R=02 rt T3 = 8cm
	//

	printf("\n\n \t Run equilibrium AVEC CHGMENT DE VARIABLE \n");
	mbs_run_equil(mbs_equil, mbs_data);
	mbs_print_equil(mbs_equil);
	mbs_delete_equil(mbs_equil, mbs_data);
	Print_q_qd_qdd_Qq(mbs_data); // Print current value of joints
	mypause();
#endif
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*					ANGLES	             *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	
	printf("\n\n Angles Calculus \n");
	Angles(mbs_data);

	//printf("my_force = %f \n", mbs_data->Qq[R1_pendulum_id]);
	mypause();

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*					 QUASI STATIC EQUILIBRIUM	STRAIGTH             *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	Toprint = 1;
	printf("\n\n Run Quasistatic Equilibrium STRAIGTH \n");
	Print_q_qd_qdd_Qq(mbs_data); // Print current value of joints
	mypause();
	mbs_data->tourne = 0;
	QuasiEquilibrium(mbs_data, V, front_radius, rear_radius, Toprint, steer);
	Print_q_qd_qdd_Qq(mbs_data); // Print current value of joints
	mypause();

	// saved the quasi static equilibrium

	q_saved_dir = get_dvec_0(mbs_data->njoint + 1);
	qd_saved_dir = get_dvec_0(mbs_data->njoint + 1);
	Qq_saved_dir = get_dvec_0(mbs_data->njoint + 1);
	copy_dvec_0(&(mbs_data->q[1]), q_saved_dir, mbs_data->njoint);
	copy_dvec_0(&(mbs_data->qd[1]), qd_saved_dir, mbs_data->njoint);
	copy_dvec_0(&(mbs_data->Qq[1]), Qq_saved_dir, mbs_data->njoint);

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*					 MODAL ANALYSIS                      *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifdef ModalAnalysis

	mbs_data->process = 4; // modal !

	printf("\n\n Run modal Analysis for V  = %f \n", V);
	ModalAnalysis(mbs_data, V, "Analyse_modale\My_Modal_Analysis.txt", front_radius, rear_radius);
	Print_q_qd_qdd_Qq(mbs_data); // Print current value of joints
	mypause();

#endif 

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*					 QUASI STATIC EQUILIBRIUM	CURVE             *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifdef curveEq
	Toprint = 1;
	mbs_data->EstEnCourbe = 1;
	printf("\n\n Run Quasistatic Equilibrium  CURVE V = %f et R = %f \n", V, mbs_data->Rayon);
	Print_q_qd_qdd_Qq(mbs_data); // Print current value of joints
	mbs_data->tourne = 1;
	QuasiEquilibrium(mbs_data, V, front_radius, rear_radius, Toprint,steer);
	Print_q_qd_qdd_Qq(mbs_data); // Print current value of joints
	mbs_data->EstEnCourbe = 0;
	printf("my couple steering =%f my couple rear = %f et my autre couple = %f \n", mbs_data->Qq[R3_steering_fork_id], mbs_data->Qq[R2_wheel_rr_id], mbs_data->Qq[R1_pendulum_id]);
	mypause();

#endif
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*					LOOPS QUASISTATIC EQUILIBRIUM R AND V       *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifdef LoopQuasi
	printf("\n\n Ready for LOOPS on quasiStatique for R and V \n");
	mbs_data->process = 12; // quasi
	mbs_data->tourne = 1; // tourne
	mbs_data->EstEnCourbe = 1;
	Toprint = 0;
	mbs_data->Rayon = R_loop_max;

	Print_q_qd_qdd_Qq(mbs_data); // Print current value of joints
	mypause();


	writing_file_R_loop = fopen("CurveEquilibrium.txt", "w+");

	if (writing_file_R_loop != NULL)
	{
		printf("Fichier d'ecriture a bien ete ouvert\n");
		fprintf(writing_file_R_loop, "CurveEquilibrium\n");

		while (mbs_data->Rayon > R_loop_init)
		{
			mbs_data->speed_ref = speed_init;
		
			while (mbs_data->speed_ref < max_V)
			{
				printf("Rayon = %f \n", mbs_data->Rayon);
				//printf("speed = %f et R = %f\n", speed, mbs_data->R_loop);
				QuasiEquilibrium(mbs_data, mbs_data->speed_ref, front_radius, rear_radius, Toprint,steer);
				//if (abs(mbs_data->q[R3_steering_fork_id]) >3.1415 || abs(mbs_data->q[R1_body_id])>2.0);
				//{
				//	mbs_data->q[R3_steering_fork_id] = -10;
				//	mbs_data->q[R1_body_id] = -15;

				//}
				fprintf(writing_file_R_loop, "R %f V %f SteerAngle %f TiltRef %f \n", mbs_data->Rayon, mbs_data->speed_ref, mbs_data->q[R3_steering_fork_id], mbs_data->q[R1_body_id]);
				
				mbs_data->speed_ref = mbs_data->speed_ref + steps;
			}
			mbs_data->Rayon = mbs_data->Rayon - R_increment;
			
		}
	fclose(writing_file_R_loop);
	printf("Fichier d'ecriture a bien ete ferme\n");
	}
	else
	{
		printf("Impossible d'ecrire dans le fichier CurveEquilibrium.txt \n");
	}
	printf("qdd: "); print_dvec_0(mbs_data->qdd, mbs_data->njoint);
	mypause();
	mbs_data->Rayon = R_loop_init;
	mbs_data->speed_ref = V;
	mbs_data->EstEnCourbe = 0;
#endif




	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*					LOOPS MODAL ANALYSIS ON  V AND K       *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

		//printf("\n\n Ready for LOOPS on Modal Analysis for V and K \n");

		//mbs_data->K_factor = K_factor_init;
		//mypause();
		//Toprint = 0;
		//while (mbs_data->K_factor < K_factor_max)
		//{
		//	printf("  K  = %f \n", mbs_data->K_factor);
		//	while (speed < max_V)
		//	{
	//mbs_data->process = 12; // equil quasi !

		//		QuasiEquilibrium(mbs_data, speed, front_radius, rear_radius, Toprint,steer);
		//		sprintf(filename_K, "%s/K%3.1f/V%3.1f.txt", path_K,  mbs_data->K_factor, speed);
	//mbs_data->process = 4; // modal

		//		ModalAnalysis(mbs_data, speed, filename_K, front_radius, rear_radius); // Analyse Modale  

		//		speed = speed + steps;
		//	}
		//	mbs_data->K_factor = 2.0* mbs_data->K_factor;
		//	speed = 0.1;
		//}
		//printf("qdd: "); print_dvec_0(mbs_data->qdd, mbs_data->njoint);
		//mypause();

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*					LOOPS MODAL ANALYSIS ON V		          *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifdef LoopModal

	printf("\n\n Ready for LOOPS on Modal Analysis for V \n");
	mypause();
	Toprint = 0;
	mbs_data->speed_ref = speed_init;
	while (mbs_data->speed_ref<max_V)
	{
		QuasiEquilibrium(mbs_data, mbs_data->speed_ref, front_radius, rear_radius, Toprint,steer);
		sprintf(filename_modal, "%s/V%9.2f.txt", path_modal, mbs_data->speed_ref);
		 mbs_data->process = 4; // modal
		ModalAnalysis(mbs_data, mbs_data->speed_ref, filename_modal, front_radius, rear_radius); // Analyse Modale  
		printf("filename = %s \n", filename_modal);
		mbs_data->speed_ref = mbs_data->speed_ref + steps;
	}
	mbs_data->speed_ref = V;
	printf("qdd: "); print_dvec_0(mbs_data->qdd, mbs_data->njoint);
	mypause();

#endif
		
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                   DIRECT DYNANMICS                        *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifdef Dirdyn

	Toprint = 1;
	// initialize dirdyn with straigth line equilibrium

	//copy_dvec_0(q_saved_dir, &(mbs_data->q[1]), mbs_data->njoint);
	//copy_dvec_0(qd_saved_dir, &(mbs_data->qd[1]), mbs_data->njoint);
	//copy_dvec_0(Qq_saved_dir, &(mbs_data->Qq[1]), mbs_data->njoint);
	
	printf("\n\n Ready for dirdyn \n");

	mbs_data->process = 3; // dirdyn!
	mbs_data->speed_ref = V;
#ifdef STC
	mbs_data->user_IO->modeTC = 1;
#endif


#ifdef DTC
	mbs_data->user_IO->modeTC = 2;
	mbs_set_qdriven(mbs_data, R3_steering_fork_id);
#endif

#ifdef EntreCourbe
	mbs_data->EntreEnCourbe = 1;
#endif
#ifdef DoubleBand
	mbs_data->DoubleBande = 1;
#endif

	//mbs_data->q[T3_body_id] = 0.2; // impose une hauteur
	Print_q_qd_qdd_Qq(mbs_data); // Print current value of joints
	mypause();

	mbs_dirdyn = mbs_new_dirdyn(mbs_data);

	// dirdyn options (see documentations for additional options)
	mbs_dirdyn->options->dt0 = 1e-3;
	mbs_dirdyn->options->tf = simu_t;
	mbs_dirdyn->options->save2file = 1;	
	mbs_dirdyn->options->animpath = PROJECT_SOURCE_DIR"/../animationR";
	
	mbs_dirdyn->options->realtime = 0;
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


	

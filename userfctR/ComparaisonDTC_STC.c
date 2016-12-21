//---------------------------
// author: Seb tims
// date: 16/10/16
// version 5.2
//
// COmparae le control STC et DTC, sotcke dans deux fichiers different
//---------------------------


#include "ComparaisonDTC_STC.h"



void ComparaisonDTC_STC(MbsData *mbs_data, double *q_saved_dir, double *qd_saved_dir, double *Qq_saved_dir, double *qdd_saved_dir,  double V,double simu_t)
{
	MbsDirdyn *mbs_dirdyn;
	mbs_data->ErrorTot = 0.0;
	mbs_data->last_tilt_torque = 0.0;
	mbs_data->EstEnCourbe = 0;
	// initialize dirdyn with straigth line equilibrium
	copy_dvec_0(q_saved_dir, &(mbs_data->q[1]), mbs_data->njoint);
	copy_dvec_0(qd_saved_dir, &(mbs_data->qd[1]), mbs_data->njoint);
	copy_dvec_0(Qq_saved_dir, &(mbs_data->Qq[1]), mbs_data->njoint);
	copy_dvec_0(qdd_saved_dir, &(mbs_data->qdd[1]), mbs_data->njoint);
	mbs_data->Qq[R2_wheel_rr_id] = mbs_data->q_rr_ref;
	mbs_data->q[R1_body_id] = 0.0;

	mbs_data->last_pen_ft_lt = 0.0;
	mbs_data->last_pen_ft_rt = 0.0;
	mbs_data->last_pen_rr = 0.0;
	mbs_data->ErrorTot_y = 0.0;
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                   DIRECT DYNANMICS    STC                    *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	printf("\n\n Ready for dirdyn STC \n");

	mbs_data->process = 3; // dirdyn!
	mbs_data->speed_ref = V;

	mbs_data->user_IO->modeTC = 1; //STC


	Print_q_qd_qdd_Qq(mbs_data); // Print current value of joints
	mypause();

	mbs_dirdyn = mbs_new_dirdyn(mbs_data);

	// dirdyn options (see documentations for additional options)
	mbs_dirdyn->options->dt0 = 1e-3;
	mbs_dirdyn->options->tf = simu_t;
	mbs_dirdyn->options->save2file = 1;
	mbs_dirdyn->options->animpath = "C:/Users/sebtims/Documents/MBProjects/tricycle/animationR/STC";

	mbs_dirdyn->options->realtime = 0;
	mbs_dirdyn->options->saveperiod = 1;


	mbs_dirdyn->options->respath = "C:/Users/sebtims/Documents/MBProjects/tricycle/resultsR/STC/dirdyn";
	mbs_run_dirdyn(mbs_dirdyn, mbs_data);
	printf(" Dirdyn STC done \n");
	mbs_delete_dirdyn(mbs_dirdyn, mbs_data);


	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                   DIRECT DYNANMICS    DTC                    *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


	mbs_data->EntreEnCourbe = mbs_data->EntreEnCourbe;
	mbs_data->DoubleBande = mbs_data->DoubleBande;
	mbs_data->ErrorTot = 0.0;
	mbs_data->last_tilt_torque = 0.0;
	mbs_data->EstEnCourbe = 0;
	mbs_data->ErrorTot_y = 0.0;
	mbs_data->last_pen_ft_lt = 0.0;
	mbs_data->last_pen_ft_rt = 0.0;
	mbs_data->last_pen_rr = 0.0;
	// initialize dirdyn with straigth line equilibrium
	copy_dvec_0(q_saved_dir, &(mbs_data->q[1]), mbs_data->njoint);
	copy_dvec_0(qd_saved_dir, &(mbs_data->qd[1]), mbs_data->njoint);
	copy_dvec_0(qdd_saved_dir, &(mbs_data->qdd[1]), mbs_data->njoint);
	copy_dvec_0(Qq_saved_dir, &(mbs_data->Qq[1]), mbs_data->njoint);
	mbs_data->Qq[R2_wheel_rr_id] = mbs_data->q_rr_ref;
	mbs_data->q[R1_body_id] = 0.0;

	
	printf("\n\n Ready for dirdyn DTC \n");

	mbs_data->process = 3; // dirdyn!
	mbs_data->speed_ref = V;

	mbs_data->user_IO->modeTC = 2; //DTC
	mbs_set_qdriven(mbs_data, R3_steering_fork_id);


	Print_q_qd_qdd_Qq(mbs_data); // Print current value of joints


	mbs_dirdyn = mbs_new_dirdyn(mbs_data);

	// dirdyn options (see documentations for additional options)
	mbs_dirdyn->options->dt0 = 1e-3;
	mbs_dirdyn->options->tf = simu_t;
	mbs_dirdyn->options->save2file = 1;
	mbs_dirdyn->options->animpath = "C:/Users/sebtims/Documents/MBProjects/tricycle/animationR/DTC";

	mbs_dirdyn->options->realtime = 0;
	mbs_dirdyn->options->saveperiod = 1;


	mbs_dirdyn->options->respath = "C:/Users/sebtims/Documents/MBProjects/tricycle/resultsR/DTC/dirdyn";
	mbs_run_dirdyn(mbs_dirdyn, mbs_data);
	printf(" Dirdyn DTC done \n");
	mbs_delete_dirdyn(mbs_dirdyn, mbs_data);



	



}


//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------

#include "math.h" 

#include "MBSdef.h"
#include "mbs_data.h"
#include "user_model.h"
#include "user_all_id.h"
#include "user_IO.h"
#include "set_output.h"
#include "Controleur.h"

double* user_JointForces(MbsData *mbs_data, double tsim)
{


	// // perturbation  control vitesse
	//if (tsim > 2.0 && tsim < 2.0 + 0.2)
	//{
	//	mbs_data->speed_ref = 4.5;
	//	mbs_data->Qq[T1_body_id] = -5* mbs_data->q_rr_ref;
	//	mbs_data->qd[T1_body_id] = mbs_data->speed_ref*0.7;
	//}

	double R1_perturb;
	if (mbs_data->speed_ref > 2)
	{
		R1_perturb = 2.5;
	}
	else
	{
		R1_perturb =2.5;
	}


	double t_pertub,sigma, mu,my_dt;
	t_pertub = 150;
	my_dt = 0.5;
	sigma = 1 / (R1_perturb*sqrt(2 * 3.1415));
	mu = t_pertub + my_dt;
	if (mbs_data->process == 3) //dirdyn controle de vitesse / couple
	{
		double Kp_rr, V;
		V = sqrt(mbs_data->qd[T1_body_id] * mbs_data->qd[T1_body_id] + mbs_data->qd[T2_body_id] * mbs_data->qd[T2_body_id]);

		Kp_rr = 10;
		//printf(" vitesse  de dedans =%f , vit demande =%f  et diff =%f\n", V, mbs_data->speed_ref, (mbs_data->qd[T1_body_id] - mbs_data->speed_ref));
		mbs_data->Qq[R2_wheel_rr_id] = mbs_data->q_rr_ref - Kp_rr * (V - mbs_data->speed_ref); // - Kp_rr * (mbs_data->Qq[R2_wheel_rr_id] - mbs_data->q_rr_ref);//
	}


	if (mbs_data->process == 3) //dirdyn controle de vitesse / couple
	{
		if (mbs_data->user_IO->modeTC == 2) //DTC
		{
			////perturbation  control DTC
			mbs_data->Qq[R1_body_id] = 0.0;
			if (tsim > t_pertub && tsim < t_pertub + my_dt)
			{
				mbs_data->Qq[R1_body_id] = R1_perturb * exp(-(tsim - (t_pertub + my_dt/2))*(tsim - (t_pertub + my_dt/2)) / (2.0*sigma *sigma)); //gaussienne
			}
			// controleur sur le tilt
			mbs_data->Qq[R1_pendulum_id] = my_controleur(mbs_data, tsim, mbs_data->speed_ref, mbs_data->q[R3_steering_fork_id]);
			//printf("my tilting controle torque DTC = %f \n ", mbs_data->Qq[R1_pendulum_id]);

		} //DTC
		else if (mbs_data->user_IO->modeTC == 1) //STC
		{
			mbs_data->Qq[R3_steering_fork_id] = my_controleur_stc(mbs_data, tsim, mbs_data->speed_ref, mbs_data->q[R3_steering_fork_id]);
			//printf("my steering controle torque STC = %f \n ", mbs_data->Qq[R3_steering_fork_id]);

			//perturbation  control STC
			mbs_data->Qq[R1_body_id] = 0.0;
			if (tsim > t_pertub && tsim < t_pertub + my_dt)
			{
				mbs_data->Qq[R1_body_id] = R1_perturb * exp(-(tsim - (t_pertub + my_dt/2))*(tsim - (t_pertub + my_dt/2)) / (2.0*sigma *sigma)); //gaussienne
			}

		} //STC

		else
		{
			//	printf("no joint force \n");
				// no torque control 

				//perturbation no control
			if (tsim > t_pertub && tsim < t_pertub + my_dt)
			{
				//printf("if\n");
				//mbs_data->Qq[R1_body_id] = R1_perturb * exp(-(tsim - (t_pertub + my_dt / 2))*(tsim - (t_pertub + my_dt / 2)) / (2.0*sigma *sigma)); //gaussienne
				mbs_data->Qq[R3_steering_fork_id] = 0.01;
					//printf("torque steer = %f \n", mbs_data->Qq[R3_steering_fork_id]);
			}
		} // no control
	}

	/*-- End of user code --*/
	return mbs_data->Qq;
}






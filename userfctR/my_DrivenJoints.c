//---------------------------
// author: Tims seb
// date: 21/11/16
// version 1.0
//
// my_DrivenJoints fonctions 
//--------------------------


#include "my_DrivenJoints.h"



void my_DrivenJoints_EnCourbe(MbsData *mbs_data, double tsim)
{
	double t_f, t_c, a_c;
	double t_start;
	t_start = mbs_data->t_start; //Temps d'initiation du tournant

	if (mbs_data->user_IO->modeTC == 2) //DTC 
	{
		if (mbs_data->DoubleBande == 1)
		{
			//mbs_data->q[R3_steering_fork_id] =  my_DrivenJoints_double_bande(mbs_data, tsim, t_start);
			mbs_data->q[R3_steering_fork_id] = my_DrivenJoints_controleur_y(mbs_data, tsim, t_start);

			//printf("my_steering_consigne = %f\n", mbs_data->q[R3_steering_fork_id]);
			//if (tsim > t_start)
			//{
			//	my_command = ang_speed* sin(4*ang_speed*(tsim - t_start))* sin(4 * ang_speed*(tsim - t_start))* sin(4 * ang_speed*(tsim - t_start));
			//	if (my_command >= 0)
			//	{
			//		mbs_data->q[R3_steering_fork_id] = fmin(my_command, 0.5);
			//	}
			//	else
			//	{
			//		mbs_data->q[R3_steering_fork_id] = fmax(my_command, -0.5);
			//	}
			//	printf(" sin(ang_speed*(tsim - t_start)) = %f et q3 =%f et diff time = %f \n ", my_command, mbs_data->q[R3_steering_fork_id], (tsim - t_start) );
			//}
			//else
			//{
			//	mbs_data->q[R3_steering_fork_id] = 0.0;
			//}
		}
		else
		{
			t_f = mbs_data->speed_ref;
			t_c = mbs_data->speed_ref / 2;
			a_c = (-4 * mbs_data->user_IO->steer) / (((0.5*t_f - t_c) * 2)*((0.5*t_f - t_c) * 2) - t_f*t_f);
			mbs_data->q[R3_steering_fork_id] = 0.0;// my_DrivenJoints_Steering_smooth_consigne(mbs_data, tsim, t_f, t_c, a_c, t_start, 0.0, mbs_data->user_IO->steer);

		}
	}
	else if (mbs_data->user_IO->modeTC == 1)  //STC 
	{
		//printf("entre en courbe STC \n");
		if (tsim < t_start)
		{
			mbs_data->EstEnCourbe = 0;
		}
		else if (tsim >= t_start)
		{
			mbs_data->EstEnCourbe = 1;
		}
	}
}

double my_DrivenJoints_controleur_y(MbsData *mbs_data, double tsim, double t_start)
{
	double Kp_y_ref, my_steering_consigne, y_consigne, Kd_y_ref,  Ki_y_ref, y_consigne_vit, y_error_tot, delta_err_y;
	Kp_y_ref = 0.1;
	Ki_y_ref = 0.01;
	Kd_y_ref = 0.1;
	my_steering_consigne = 0.0;
	y_consigne = my_DrivenJoints_y_ref_DTC(mbs_data, tsim, t_start);
	y_consigne_vit = my_DrivenJoints_Steering_smooth_consigne_vit(mbs_data, tsim, 0.0, 0.0, 0.0, t_start, 0.0, 0.0);
	delta_err_y = (mbs_data->q[T2_body_id] - y_consigne);
	if (fabs(delta_err_y)>0.0001)
	{
		mbs_data->ErrorTot_y += delta_err_y * 0.001; // time step
	}
	if (tsim < t_start)
	{
		my_steering_consigne = 0.0;

	}
	else if (tsim>t_start)
	{
		my_steering_consigne = Kp_y_ref * delta_err_y - Kd_y_ref * (mbs_data->qd[T2_body_id] - y_consigne_vit) - Ki_y_ref *mbs_data->ErrorTot_y;
	}
	printf("my_steering_consigne = %f\n", my_steering_consigne);
	return  my_steering_consigne;
}


double my_DrivenJoints_y_ref_DTC(MbsData *mbs_data, double tsim, double t_start)
{
	double delta_y, y_ref;
	double t_f, t_c, a_c;
	y_ref = 0.0;

	delta_y = 1.0;
	t_f = 5.0;
	t_c = t_f / 2;
	a_c = (-delta_y) / (((0.5*t_f - t_c) * 2)*((0.5*t_f - t_c) * 2) - t_f*t_f);
	if (tsim > t_start)
	{	y_ref = my_DrivenJoints_Steering_smooth_consigne(mbs_data, tsim, t_f, t_c, a_c, t_start, 0.0, delta_y);
}
	printf("ac = %f\n", a_c);
	return y_ref;
}





double my_DrivenJoints_double_bande(MbsData *mbs_data, double tsim, double t_start)
{
	double consigne_double_bande, t_mid, t_mid_end;
	consigne_double_bande = 0.0;
	t_mid = t_start + 5.0;
	t_mid_end = t_mid + 5.0;
	if (tsim < t_start)
	{
		consigne_double_bande = 0.0;

	}
	else if (tsim > t_start && tsim < t_mid)
	{
		consigne_double_bande = my_DrivenJoints_changement_bande_simple(mbs_data, tsim, t_start, 1, 0.0);
	}
	else if (tsim > t_mid && tsim < t_mid_end)
	{
		//consigne_double_bande = my_DrivenJoints_changement_bande_simple(mbs_data, tsim, 0, -1,t_mid);
	}

	return  consigne_double_bande;
}

double my_DrivenJoints_changement_bande_simple(MbsData *mbs_data, double tsim, double t_start, int signe, double t_mid)
{
	double t_f, t_c, a_c, consigne_simple_bande, t_end1, t_end2, delta_t, t_end3, t_end4, facteur;
	delta_t = 1.0;
	t_end1 = t_mid + t_start + delta_t;
	t_end2 = t_end1 + delta_t;
	t_end3 = t_end2 + delta_t;
	t_end4 = t_end3 + delta_t;
	facteur = 2.0;
	t_f = 1.0;// mbs_data->speed_ref;
	t_c = 0.5;// (mbs_data->speed_ref) / 2;
	a_c = signe * (-4 * mbs_data->user_IO->steer) / (((0.5*t_f - t_c) * 2)*((0.5*t_f - t_c) * 2) - t_f*t_f);
	if (signe <= 0)
	{
		if (tsim <t_mid + t_start)
		{
			consigne_simple_bande = 0.0;
		}
		else if (tsim > t_mid + t_start && tsim < t_end1)
		{
			consigne_simple_bande = my_DrivenJoints_Steering_smooth_consigne(mbs_data, tsim, t_f, t_c, 2 * a_c, t_start + t_mid, 0.0, -2 * mbs_data->user_IO->steer);

		}
		else if (tsim > t_end1 && tsim < t_end2)
		{
			consigne_simple_bande = my_DrivenJoints_Steering_smooth_consigne(mbs_data, tsim, t_f, t_c, -2 * a_c, t_end1, -2 * mbs_data->user_IO->steer, 0.0);
		}
		else if (tsim > t_end2 && tsim < t_end3)
		{
			consigne_simple_bande = my_DrivenJoints_Steering_smooth_consigne(mbs_data, tsim, t_f, t_c, -2 * a_c, t_end2, 0.0, 2 * mbs_data->user_IO->steer);
		}
		else if (tsim > t_end3 && tsim < t_end4)
		{
			consigne_simple_bande = my_DrivenJoints_Steering_smooth_consigne(mbs_data, tsim, t_f, t_c, 2 * a_c, t_end3, 2 * mbs_data->user_IO->steer, 0.0);// 0.1*mbs_data->user_IO->steer);
		}
		else
		{
			consigne_simple_bande = 0.0;
		}
	}
	else
	{
		if (tsim < t_start)
		{
			consigne_simple_bande = 0.0;
		}
		else if (tsim > t_start && tsim < t_end1)
		{
			consigne_simple_bande = my_DrivenJoints_Steering_smooth_consigne(mbs_data, tsim, t_f, t_c, a_c, t_start, 0.0, mbs_data->user_IO->steer);

		}
		else if (tsim > t_end1 && tsim < t_end2)
		{
			consigne_simple_bande = my_DrivenJoints_Steering_smooth_consigne(mbs_data, tsim, t_f, t_c, -a_c, t_end1, mbs_data->user_IO->steer, 0.0);
		}
		else if (tsim > t_end2 && tsim < t_end3)
		{
			consigne_simple_bande = my_DrivenJoints_Steering_smooth_consigne(mbs_data, tsim, t_f, t_c, -a_c, t_end2, 0.0, -mbs_data->user_IO->steer);
		}
		else if (tsim > t_end3 && tsim < t_end4)
		{
			consigne_simple_bande = my_DrivenJoints_Steering_smooth_consigne(mbs_data, tsim, t_f, t_c, a_c, t_end3, -mbs_data->user_IO->steer, 0.0);// 0.1*mbs_data->user_IO->steer);
		}
		else
		{
			consigne_simple_bande = 0.0;
		}
	}
	return consigne_simple_bande;

}

double my_DrivenJoints_Steering_smooth_consigne(MbsData *mbs_data, double tsim, double t_f, double t_c, double a_c, double t_start, double q_i, double q_f)
{
	double consigne_smooth;
	consigne_smooth = 0.0;


	//printf("my ac = %f \n", a_c);
	//if (tsim < t_start)
	//{
	//	consigne_smooth = 0.0;
	//}
	//else
	//{
	mbs_data->EstEnCourbe = 1;
	if (tsim <= t_start + t_c)
	{
		consigne_smooth = q_i + 0.5*a_c* (tsim - t_start)*(tsim - t_start);
	}
	else if (tsim > (t_start + t_c) && tsim <= t_start + (t_f - t_c))
	{
		consigne_smooth = a_c * t_c * (tsim - t_start - 0.5*t_c);
	}
	else if (tsim < (t_start + t_f) && tsim >= t_start + (t_f - t_c))
	{
		consigne_smooth = q_f - 0.5 *	a_c *((tsim - t_start) - t_f) *((tsim - t_start) - t_f);
	}
	else
	{
		if (mbs_data->DoubleBande == 0)
		{

			consigne_smooth = mbs_data->user_IO->steer; // ???????????? consigne_smooth;
			//printf("consigne smoothe = %f \n", consigne_smooth);
		}
		else
		{
			consigne_smooth = q_f;// mbs_data->user_IO->steer;
		//	printf("consigne smoothe = %f \n", consigne_smooth);
		}

	}

	//	}
	return consigne_smooth;

}

double my_DrivenJoints_Steering_smooth_consigne_vit(MbsData *mbs_data, double tsim, double t_f, double t_c, double a_c, double t_start, double q_i, double q_f)
{
	double consigne_smooth_vit;
	consigne_smooth_vit = 0.0;



	if (tsim <= t_start + t_c)
	{
		consigne_smooth_vit = a_c* (tsim - t_start);
	}
	else if (tsim > (t_start + t_c) && tsim <= t_start + (t_f - t_c))
	{
		consigne_smooth_vit = a_c * t_c;
	}
	else if (tsim < (t_start + t_f) && tsim >= t_start + (t_f - t_c))
	{
		consigne_smooth_vit = -a_c *((tsim - t_start) - t_f) ;
	}
	else
	{
		consigne_smooth_vit = 0.0;

	}

	return consigne_smooth_vit;

}

void my_DrivenJoints_LigneDroite(MbsData *mbs_data, double tsim)
{
	double t_pertub;
	//printf("straight line  time= %f \n", tsim);
	// pas de perturb ici, voir dans joint forces !!!! R1_body
	if (mbs_data->user_IO->modeTC == 1) //STC
	{
		// straigth line with STC
		//	printf("straight line with STC time= %f \n", tsim);
		t_pertub = 1.0;
		//perturb STC
		//if (tsim > t_pertub && tsim < t_pertub + 0.2)
		//{
		//	//	pertub position:
		//	//mbs_data->q[R3_steering_fork_id] = 0.01;
		//	//	//printf("torque steer = %f \n", mbs_data->Qq[R3_steering_fork_id]);

		//	//	pertub vitesse:
		//	//mbs_data->qd[T1_body_id] = mbs_data->qd[T1_body_id] + 0.5;

		//}
		//else if (tsim > t_pertub + 0.2)
		//{
		//	mbs_data->EstEnCourbe = 1;
		//	mbs_data->EntreEnCourbe = 1;
		//	mbs_data->Rayon = 10;
		//}
	}
	else if (mbs_data->user_IO->modeTC == 2) //DTC
	{
		// straigth line with DTC
		mbs_data->q[R3_steering_fork_id] = 0.0;
		//mbs_data->qd[R3_steering_fork_id] = 0.0;
		//mbs_data->qdd[R3_steering_fork_id] = 0.0;
		//printf("straight line with DTC time= %f \n", tsim);
		//t_pertub = 1.5;
		////pertub DTC
		//if (tsim > t_pertub && tsim < t_pertub + 0.2)
		//{
		//	// perturb position
		//	mbs_data->q[R3_steering_fork_id] = 0.02;

		//	//	pertub vitesse:
		//	//mbs_data->qd[T1_body_id] = mbs_data->qd[T1_body_id] + 0.5;
		//	//printf("torque steer = %f \n", mbs_data->Qq[R3_steering_fork_id]);
		//}

	}


}


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


	double t_pertub, t_f, t_c, a_c;

	//printf("entre en courbe \n");
	double thePathX, thePathY, speedX, speedY, dirX, dirY, distanceX, distanceY, errorX, errorY, Z_cross, errorTot;
	double t_start, R, vmax, Xbegin, ang_speed, K;

	// The path
	t_start = mbs_data->t_start; //Temps d'initiation du tournant
	R = mbs_data->Rayon; // Rayon du tourant [m]
	vmax = mbs_data->speed_ref; // Vitesse dans le tournant(vtot = vmax + vstart)
	Xbegin = vmax * t_start;
	ang_speed = vmax / R; // Vitesse angulaire dans le tournant

	if (tsim < t_start)
	{
		thePathX = vmax*tsim;
		thePathY = 0;
		speedX = vmax;
		speedY = 0;
	}
	else if (tsim >= t_start)
	{
		mbs_data->EstEnCourbe = 1;
		thePathX = Xbegin + (vmax / ang_speed)*sin(ang_speed*(tsim - t_start));
		thePathY = -(vmax / ang_speed)*cos(ang_speed*(tsim - t_start)) + R;
		speedX = vmax*cos(ang_speed*(tsim - t_start));
		speedY = vmax*sin(ang_speed*(tsim - t_start));
	}

	dirX = speedX / sqrt(speedX*speedX + speedY*speedY);
	dirY = speedY / sqrt(speedX*speedX + speedY*speedY);

	distanceX = thePathX - mbs_data->q[T1_body_id];
	distanceY = thePathY - mbs_data->q[T2_body_id];
	errorX = mbs_data->q[T1_body_id] - (thePathX - distanceX*dirX);
	errorY = mbs_data->q[T2_body_id] - (thePathY - distanceY*dirY);
	errorTot = sqrt(errorX*errorX + errorY *errorY);
	Z_cross = distanceX*dirY - distanceY*dirX;
	K = 0.1;

	if (mbs_data->user_IO->modeTC == 2) //DTC 
	{
		if (mbs_data->DoubleBande == 1)
		{
			if (Z_cross >= 0)
			{
				mbs_data->q[R3_steering_fork_id] = fmax(-0.05, -K*errorTot);
			}
			else
			{
				mbs_data->q[R3_steering_fork_id] = fmin(0.05, K*errorTot);
			}
		}
		else
		{
			t_f = mbs_data->speed_ref;
			t_c = mbs_data->speed_ref / 2;
			a_c = (-4 * mbs_data->user_IO->steer) / (((0.5*t_f - t_c) * 2)*((0.5*t_f - t_c) * 2) - t_f*t_f);
			//printf("my ac = %f \n", a_c);
			if (tsim < t_start)
			{
				mbs_data->q[R3_steering_fork_id] = 0.0;
			}
			else
			{
				mbs_data->EstEnCourbe = 1;
				if (tsim <= t_start + t_c)
				{
					mbs_data->q[R3_steering_fork_id] = 0.5*a_c* (tsim - t_start)*(tsim - t_start);
				}
				else if (tsim > (t_start + t_c) && tsim <= t_start + (t_f - t_c))
				{
					mbs_data->q[R3_steering_fork_id] = a_c * t_c * (tsim - t_start - 0.5*t_c);
				}
				else if (tsim < (t_start + t_f) && tsim >= t_start + (t_f - t_c))
				{
					mbs_data->q[R3_steering_fork_id] = mbs_data->user_IO->steer - 0.5 *	a_c *((tsim - t_start) - t_f) *((tsim - t_start) - t_f);
				}
				else
				{
					mbs_data->q[R3_steering_fork_id] = mbs_data->q[R3_steering_fork_id];
				}

			}
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
			//printf("mbs_data->q[R3_steering_fork_id]  = %f \n", mbs_data->q[R3_steering_fork_id]);
		}
	}
}

void my_DrivenJoints_LigneDroite(MbsData *mbs_data, double tsim)
{
	double t_pertub;
	//printf("straight line  time= %f \n", tsim);
	if (mbs_data->user_IO->modeTC == 1) //STC
	{
		// straigth line with STC
		//	printf("straight line with STC time= %f \n", tsim);
		t_pertub = 1.0;
		//perturb STC
		if (tsim > t_pertub && tsim < t_pertub + 0.2)
		{
			//	//printf("if\n");
			mbs_data->q[R3_steering_fork_id] = 0.01;
			//	//printf("torque steer = %f \n", mbs_data->Qq[R3_steering_fork_id]);

		}
		else if (tsim > t_pertub + 0.2)
		{
			mbs_data->EstEnCourbe = 1;
			mbs_data->EntreEnCourbe = 1;
			mbs_data->Rayon = 10;
		}
	}
	else if (mbs_data->user_IO->modeTC == 2) //DTC
	{
		// straigth line with DTC
		mbs_data->q[R3_steering_fork_id] = 0.0;
		//mbs_data->qd[R3_steering_fork_id] = 0.0;
		//mbs_data->qdd[R3_steering_fork_id] = 0.0;
		//printf("straight line with DTC time= %f \n", tsim);
		t_pertub = 2.0;
		//pertub DTC
		if (tsim > t_pertub && tsim < t_pertub + 0.2)
		{
			//printf("if\n");
			mbs_data->q[R3_steering_fork_id] = 0.02;
			//printf("torque steer = %f \n", mbs_data->Qq[R3_steering_fork_id]);
		}

	}


}


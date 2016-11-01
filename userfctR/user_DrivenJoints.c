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


void user_DrivenJoints(MbsData *mbs_data, double tsim)
{

	if (mbs_data->tourne == 1)
	{ 
		if (mbs_data->user_IO->modeTC== 2) //DTC
		{
			double thePathX, thePathY, speedX, speedY, dirX, dirY, distanceX, distanceY, errorX, errorY, Z_cross, errorTot;
			double t_start, R, vmax, Xbegin, ang_speed, K;

			// The path
			t_start = 2; //Temps d'initiation du tournant
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
			if (Z_cross >= 0)
			{
				mbs_data->q[R3_steering_fork_id] = max(-0.05, -K*errorTot);
			}
			else
			{
				mbs_data->q[R3_steering_fork_id] = min(0.05, K*errorTot);
			}
		}
		else
		{ // STC

		}
	}
	else
	{
		// quand meme driven !?  
		mbs_data->q[R3_steering_fork_id] = 0.0;
		mbs_data->qd[R3_steering_fork_id] = 0.0;
		mbs_data->qdd[R3_steering_fork_id] = 0.0;
	}
}



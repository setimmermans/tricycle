//---------------------------
// author: Tims seb
// date: 29/10/16
// version 1.0
//
// Controleur du tilt  
//--------------------------


#include "Controleur.h"


//#define Scaled	 
#define Normal	 

double my_controleur(MbsData *mbs_data, double tsim, double speed, double steering)
{
	double My_torque, delta_err, tilt_ref, max_torque, speed_tilt_ref, delta_speed;
	mbs_data->Kp = 1000.0;
	mbs_data->Ki = 1;
	mbs_data->Kd = 10;
	max_torque = 100.0;

	tilt_ref = tilt_reference(mbs_data, tsim, speed, steering);
	speed_tilt_ref = 0.0;
	delta_err = tilt_ref - mbs_data->q[R1_body_id];
	delta_speed = speed_tilt_ref - mbs_data->qd[R1_body_id];
	My_torque = - mbs_data->Kp * delta_err + mbs_data->Ki * mbs_data->ErrorTot + mbs_data->Kd *  delta_speed;
	//if (abs(mbs_data->q[R1_body_id]) > 0.0001)
	//{
		//printf("Delta err %f et  My_torque = %f et ErrorTot = %f et tilt ref = %f\n", delta_err, My_torque, mbs_data->ErrorTot, tilt_ref);
	//}
	mbs_data->ErrorTot += delta_err * 0.001; // time step

	if(abs(My_torque) > max_torque)
	{
		return max_torque*sign(My_torque);
	}
	else
	{
		return My_torque;
	}
}

double tilt_reference(MbsData *mbs_data, double tsim, double speed, double steering)
{
	double the_tilt;
	the_tilt = atan(speed*speed / mbs_data->Rayon);
	return the_tilt;

}


double my_controleur_stc(MbsData *mbs_data, double tsim, double speed, double steering)
{

	return 0.0;
}

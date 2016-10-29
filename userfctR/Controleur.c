//---------------------------
// author: Tims seb
// date: 29/10/16
// version 1.0
//
// Controleur du tilt  
//--------------------------


#include "Controleur.h"



double my_controleur(MbsData *mbs_data, double tsim)
{
	double My_torque;
	mbs_data->Kp = 1000.0;
	mbs_data->Ki = 1.0;
	mbs_data->Kd = 1.0;
	My_torque = -mbs_data->Kp * mbs_data->q[R1_pendulum_id]-mbs_data->Ki * mbs_data->qPrevious + mbs_data->Kd *  mbs_data->qd[R1_pendulum_id];
	printf("R1_body = %f R1 pendule = % et  My_torque = %f et previous = %f et time = %f\n", mbs_data->q[R1_body_id], mbs_data->q[R1_pendulum_id],My_torque, mbs_data->qPrevious, tsim);
	mbs_data->qPrevious += mbs_data->q[R1_pendulum_id]*tsim;
	return My_torque;
}


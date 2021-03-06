//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//
//
//---------------------------

#include "math.h"

#include "MBSdef.h"
#include "MBSfun.h"
#include "mbs_data.h"
#include "mbs_project_interface.h"
#include "user_model.h"
#include "user_all_id.h"
#include "mbs_motorbike_contact.h"
#include "mbs_matrix.h"
#include "wheel.h"
#include "user_IO.h"


double* user_ExtForces(double PxF[4], double RxF[4][4],
	double VxF[4], double OMxF[4],
	double AxF[4], double OMPxF[4],
	MbsData *mbs_data, double tsim, int ixF)
{

	
	double Fx = 0.0, Fy = 0.0, Fz = 0.0;
	double Mx = 0.0, My = 0.0, Mz = 0.0;
	double dxF[4] = { 0.0, 0.0, 0.0, 0.0 };
	double vec_1[4] = { 0.0, 0.0, 0.0, 0.0 };
	double vec_2[4] = { 0.0, 0.0, 0.0, 0.0 };

	double  RxF_t[4][4];
	int idpt;

	// wheel parameters...
	double pen, rz, anglis, angcamb, gliss, Vct[4], Rtsol[4][4], Ftemp[4]; // , steering_Angle;

	double *SWr = mbs_data->SWr[ixF];
	double r_rim, r_tire;



	if (ixF == F_wheel_ft_lt_id || F_wheel_ft_rt_id == ixF)
	{
		r_tire = mbs_data->user_model->wheel_ft.r_t_tire; // adapter !!!!!!!!!!!!!!!!!!!! =>ok
		r_rim = mbs_data->user_model->wheel_ft.r_n_tire - mbs_data->user_model->wheel_ft.r_t_tire;
	}
	else if (ixF == F_wheel_rr_id)
	{
		r_tire = mbs_data->user_model->wheel_rr.r_t_tire;
		r_rim = mbs_data->user_model->wheel_rr.r_n_tire - mbs_data->user_model->wheel_rr.r_t_tire;
	}


	// default application point of the force: anchor point to which it is attached

	idpt = mbs_data->xfidpt[ixF];
	dxF[1] = mbs_data->dpt[1][idpt];
	dxF[2] = mbs_data->dpt[2][idpt];
	dxF[3] = mbs_data->dpt[3][idpt];

	mbs_kine_wheel_motorbike(PxF, RxF, VxF, OMxF, tsim, ixF, r_rim, r_tire, &pen, &rz, &anglis, &angcamb, &gliss, Vct, Rtsol, dxF);
	user_WheelForces_motorbike(pen, rz, anglis, angcamb, gliss, Vct[3], dxF, mbs_data, tsim, ixF, SWr);
	//printf("Calcul Forces wheel ok \n");


	// camber and toe
	vec_1[0] = 0.0; vec_1[1] = 0.0; vec_1[2] = 1.0; vec_1[3] = 0.0;
	transpose(RxF, RxF_t);
	matrix_product(RxF_t, vec_1, vec_2);
	if (ixF == F_wheel_ft_lt_id)
	{

		mbs_data->user_IO->equil_ft_lt_camber = atan2(vec_2[3], vec_2[2]);
		mbs_data->user_IO->equil_ft_lt_toe = -atan2(vec_2[1], vec_2[2]);
		mbs_data->user_IO->dirdyn_ft_lt_steer = -atan2(vec_2[1], vec_2[2]) - mbs_data->q[R3_body_id];


		mbs_data->equil_ft_lt_camber = mbs_data->user_IO->equil_ft_lt_camber;
		mbs_data->toe_lft = mbs_data->user_IO->equil_ft_lt_toe; // to use in  equil
	}
	else if (ixF == F_wheel_ft_rt_id)
	{
		mbs_data->user_IO->equil_ft_rt_camber = atan2(vec_2[3], vec_2[2]);
		mbs_data->user_IO->equil_ft_rt_toe = -atan2(vec_2[1], vec_2[2]);
		mbs_data->user_IO->dirdyn_ft_rt_steer = -atan2(vec_2[1], vec_2[2]) - mbs_data->q[R3_body_id];

		mbs_data->equil_ft_rgt_camber = mbs_data->user_IO->equil_ft_rt_camber;
		mbs_data->toe_rgt = mbs_data->user_IO->equil_ft_rt_toe;// to use in  equil
	}


	if (ixF == F_wheel_ft_lt_id)
	{
		//printf("delta front left  et phi : %f  %f \n",delta,phi);
	}



	// transformation from Rsol to inertial frame

	matrix_product(Rtsol, SWr, Ftemp);
	SWr[1] = Ftemp[1];
	SWr[2] = Ftemp[2];
	SWr[3] = Ftemp[3];


	if (ixF == F_wheel_ft_lt_id)
	{
		mbs_data->user_IO->Fn_lft = SWr[3];
	}
	else if (ixF == F_wheel_ft_rt_id)
	{
		mbs_data->user_IO->Fn_rgt = SWr[3];
	}


	matrix_product(Rtsol, &(SWr[3]), Ftemp);
	SWr[4] = Ftemp[1];
	SWr[5] = Ftemp[2];
	SWr[6] = Ftemp[3];

	//printf("rear wheel Fn:%f , Flat:%f et autres= %f  \n", SWr[3], SWr[2], SWr[1]);

	switch (ixF)
	{
		/* Begin of user code */

	case my_ext_force_lat_id:

		if (tsim>10.0 &&tsim<11.0)
		{
			//Fy = 10;
			//SWr[1] = 0.0;
			//SWr[2] = Fy;
			//SWr[3] = 0.0;
			//SWr[4] = 0.0;
			//SWr[5] = 0.0;
			//SWr[6] = 0.0;
		}
		break;

		/* End of user code */
	}

	return SWr;
}
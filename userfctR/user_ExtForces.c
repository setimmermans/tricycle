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
#include "Tyre\mbs_motorbike_contact.h"
#include "mbs_matrix.h"
#include "Tyre\wheel.h"
#include "user_IO.h"


double* user_ExtForces(double PxF[4], double RxF[4][4], 
                       double VxF[4], double OMxF[4], 
                       double AxF[4], double OMPxF[4], 
                       MbsData *mbs_data, double tsim,int ixF)
{
    double Fx=0.0, Fy=0.0, Fz=0.0;
    double Mx=0.0, My=0.0, Mz=0.0;
    double dxF[4] ={0.0, 0.0, 0.0, 0.0};
	double vec_1[4] = { 0.0, 0.0, 0.0, 0.0 };
	double vec_2[4] = { 0.0, 0.0, 0.0, 0.0 };

	double  RxF_t[4][4];
	//double delta, phi, Rphi[4][4],Rphi_T[4][4], eRb[4], eRturn[4], Rturn[4][4];
	//int i,j;

	// quasistatic analysis
	double Fn_inner, Flat_inner, Fn_outer, Flat_outer;



	int idpt;

	// wheel parameters...
	double pen, rz, anglis, angcamb, gliss, Vct[4], Rtsol[4][4], Ftemp[4],steering_Angle;
	

    double *SWr = mbs_data->SWr[ixF];

	double r_rim, r_tire;




	if(ixF==F_wheel_ft_lt_id || F_wheel_ft_rt_id==ixF)
	{
		r_tire = mbs_data->user_model->wheel_ft.r_t_tire;
		r_rim = mbs_data->user_model->wheel_ft.r_n_tire-mbs_data->user_model->wheel_ft.r_t_tire;
	}
	else if(ixF==F_wheel_rr_id)
	{
		r_tire = mbs_data->user_model->wheel_rr.r_t_tire;
		r_rim = mbs_data->user_model->wheel_rr.r_n_tire-mbs_data->user_model->wheel_rr.r_t_tire;
	}
	
	
    // default application point of the force: anchor point to which it is attached
    
	idpt = mbs_data->xfidpt[ixF];
	dxF[1] = mbs_data->dpt[1][idpt];
	dxF[2] = mbs_data->dpt[2][idpt];
	dxF[3] = mbs_data->dpt[3][idpt];
	
	mbs_kine_wheel_motorbike(PxF,RxF, VxF, OMxF, tsim, ixF,r_rim,r_tire,&pen,&rz,&anglis,&angcamb,&gliss,Vct,Rtsol,dxF);
	user_WheelForces_motorbike(pen,rz,anglis,angcamb,gliss, Vct[3], dxF, mbs_data, tsim, ixF, SWr);
	//printf("Calcul Forces wheel ok \n");
	
	
	
	// calculation of delta 1,2,3
    
	//mbs_sensor(mbs_data->user_IO->PtrSensor, mbs_data, 2);
    // compute the sensor (position, velocity...)
	/*for(i=1;i<=3;i++)
	{
		for(j=1;j<=3;j++)
		{
			Rturn[i][j]=mbs_data->user_IO->PtrSensor->R[i][j];
			//printf("R[%d][%d] : %f ",i,j,Rturn[i][j]);
		}
		//printf(" \n ");
	}*/



	// camber and toe
	vec_1[0] = 0.0; vec_1[1] = 0.0; vec_1[2] = 1.0; vec_1[3] = 0.0;
	transpose(RxF, RxF_t);
	matrix_product(RxF_t, vec_1, vec_2);
	if (ixF == F_wheel_ft_lt_id)
	{
		mbs_data->user_IO->equil_ft_lt_camber = atan2(vec_2[3], vec_2[2]);
		mbs_data->user_IO->equil_ft_lt_toe = -atan2(vec_2[1], vec_2[2]);
		mbs_data->user_IO->dirdyn_ft_lt_steer = -atan2(vec_2[1], vec_2[2]) - mbs_data->q[R3_body_id];
	}
	else if (ixF == F_wheel_ft_rt_id)
	{
		mbs_data->user_IO->equil_ft_rt_camber = atan2(vec_2[3], vec_2[2]);
		mbs_data->user_IO->equil_ft_rt_toe = -atan2(vec_2[1], vec_2[2]);
		mbs_data->user_IO->dirdyn_ft_rt_steer = -atan2(vec_2[1], vec_2[2]) - mbs_data->q[R3_body_id];
	}


	if(ixF==F_wheel_ft_lt_id )
	{
		//printf("delta front left  et phi : %f  %f \n",delta,phi);
	}


	//mbs_bakker(SWr ,&(SWr[3]),anglis,angcamb,gliss);

	// transformation from Rsol to inertial frame
	
	matrix_product(Rtsol,SWr,Ftemp);
	SWr[1] = Ftemp[1];
	SWr[2] = Ftemp[2];
	SWr[3] = Ftemp[3];


	if (ixF == F_wheel_ft_lt_id)
	{
		mbs_data->user_IO->Fn_lft= SWr[3];
	}
	else if (ixF == F_wheel_ft_rt_id)
	{
		mbs_data->user_IO->Fn_rgt= SWr[3];
	}

	
	matrix_product(Rtsol,&(SWr[3]),Ftemp);
	SWr[4] = Ftemp[1];
	SWr[5] = Ftemp[2];
	SWr[6] = Ftemp[3];








	// proof of quasistatic equilibirum
	// center of mass
	/*if (ixF == F_wheel_ft_lt_id)
	{
		Flat_outer= SWr[2];
		Fn_outer = SWr[3];
		printf("outer wheel Fn:%f , Flat:%f  \n", SWr[3],SWr[2]);
	}
	else if (F_wheel_ft_rt_id == ixF)
	{
		printf("inner wheel Fn:%f , Flat:%f  \n", SWr[3], SWr[2]);
	}
	else if (ixF == F_wheel_rr_id)
	{
		printf("rear wheel Fn:%f , Flat:%f  \n", SWr[3], SWr[2]);
	}$/





	/*
    SWr[1]=Fx;
    SWr[2]=Fy;
    SWr[3]=Fz;
    SWr[4]=Mx;
    SWr[5]=My;
    SWr[6]=Mz;
    SWr[7]=dxF[1];
    SWr[8]=dxF[2];
    SWr[9]=dxF[3];
	*/
    return SWr;
}

 

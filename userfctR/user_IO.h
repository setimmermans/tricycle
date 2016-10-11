/*===========================================================================*
  *
  *  user_sf_IO.h
  *	
  *  Project:	PendulumSpringC
  * 
  *  Generation date: 14-Nov-2014 18:28:15
  * 
  *  (c) Universite catholique de Louvain
  *      D�partement de M�canique 
  *      Unit� de Production M�canique et Machines 
  *      2, Place du Levant 
  *      1348 Louvain-la-Neuve 
  *  http://www.robotran.be// 
  *  
 /*===========================================================================*/

#ifndef UsersfIO_h
#define UsersfIO_h
/*--------------------*/
 
#ifdef ACCELRED 
#define S_FUNCTION_NAME  mbs_sf_accelred_PendulumSpringC 
#elif defined DIRDYNARED 
#define S_FUNCTION_NAME  mbs_sf_dirdynared_PendulumSpringC 
#elif defined INVDYNARED 
#define S_FUNCTION_NAME  mbs_sf_invdynared_PendulumSpringC 
#elif defined SENSORKIN 
#define S_FUNCTION_NAME  mbs_sf_sensorkin_PendulumSpringC 
#endif 
 
#define SF_N_USER_INPUT 0 
#define SF_N_USER_OUTPUT 0 

#include "mbs_user_interface.h"
#include "mbs_sensor.h"
#include "mbs_linearipk.h"

typedef struct Controller 
{
	double P;
	double I;
	double D;
} Controller;




typedef struct Quasistatic 
{
	double **theta;
	double *R;
	double *V;
	double **Qsteer;

	double **qu_slane;
	double **qud_slane;
	double **Qq_slane;
	int nV_slane;
	double deltaV_slane;

	double Vstart;
	double Rstart;
	double deltaV;
	double deltaR;
	double Vend;
	double Rend;
	int nV;
	int nR;

} Quasistatic;

typedef struct LQController 
{
	// state-feedback control
	double *lq_K;
	const char lq_filename[500];
	double a, b, c;

} LQController ;







typedef struct UserIO 
{
	// computation of front axle steering angle. 
	double delta_sw_FF, delta_sw_FB;
	double dirdyn_ft_lt_steer;
	double dirdyn_ft_rt_steer;
	double deltaF;
	double deltaF_old;
	double deltaF_deri;

	
	// desired quantities for control... !
	double deltaFD;
	double deltaFD_deri;
	double deltaFD_old;
	double thetaD;
	double QsteerD;
	double dpsiD;

	double thetaD_theo;

		double thetaD_old;
	double thetaD_deri;
	double a_lat;

	double equil_ft_lt_camber;
	double equil_ft_lt_toe;
	double equil_ft_rt_camber;
	double equil_ft_rt_toe;

	double EnergyC;

	double aI_pilot[4];

	int TC;
	double vD;
	double steeringTorque;

	double steering_d;
	double steering_old;

	double e;
	double e_d;
	double e_old;
	double curve_f;







	double distance;
	double distance_old;
	double distance_deri;

	MbsSensor *PtrSensor;
	Controller *cvs;

	double **Mr;
	double **Gr;
	double **Kr;
	MbsLpk *lpk;
	
	LQController *lqc; // for Linear quadratic Controller (either used to control lean angle or yaw rate)
	Quasistatic *qstc; // for quasistatic mapping (for determining thetaD)




	//--------------------------------------------------------------

	double V;
	double steer;

	int modeTC;

	double reference;
	double r;

	// save the quasistatic values:   the eigenvalues for modal analysis
	double **qstc_vec_a;
	double **qstc_vec_b;
	double **qstc_val_a;
	double **qstc_val_b;

	double *q_equil;
	double *qd_equil;
	double *qdd_equil;
	double *Qq_equil;
	
	double dummy_Qqrear;
	double dummy_Qsteer;

	double *Qq_lin;
	double *q_lin;
	double *qd_lin;
	double *qdd_lin;





	// saving of dirdyn variables... ! 
	double Fn_lft;
	double Fn_rgt;


} UserIO ;





/*--------------------*/
#endif

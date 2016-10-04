#ifndef MBS_MOTORBIKE_CONTACT_h
#define MBS_MOTORBIKE_CONTACT_h

#include "mbs_data.h"
/*
// Longitudinal parameter for 160/70 tire

#define C_x 1.6064

#define p_Dx1 1.2017
#define p_Dx2 -0.0922

#define p_Ex1 0.0263
#define p_Ex2 0.27056
#define p_Ex3 -0.0769
#define p_Ex4 1.1268

#define p_Kx1 25.94
#define p_Kx2 -4.233
#define p_Kx3 0.3369


#define F_z0 1600.0

// Lateral parameter for 160/70 tire

#define C_y 0.93921

#define p_Dy1 1.1524
#define p_Dy2 -0.01794
#define p_Dy3 -0.06531

#define p_Ey1 -0.94635
#define p_Ey2 -0.09845
#define p_Ey4 -1.6416

#define p_Ky1 26.601
#define p_Ky2 1.0167
#define p_Ky3 1.4989
#define p_Ky4 0.52567
#define p_Ky5 -0.24064
#define p_Ky6 0.7667
#define p_Ky7 0.0

#define C_gamma 0.50732
#define E_gamma -4.7481

// Aligning parameter for 160/70 tire

#define C_t 1.3115

#define q_Bz1 10.354
#define q_Bz2 4.3004
#define q_Bz5 -0.34033
#define q_Bz6 -0.13202
#define q_Bz9 10.118
#define q_Bz10 -1.0508

#define q_Dz1 0.20059
#define q_Dz2 0.05282
#define q_Dz3 -0.21116
#define q_Dz4 -0.15941
#define q_Dz8 0.30941
#define q_Dz9 0.0
#define q_Dz10 0.10037
#define q_Dz11 0.0

#define q_Ez1 -0.39247 // caution: fault in article
#define q_Ez2 0.10809  // caution: fault in article
#define q_Ez5 0.9836

#define q_Hz3 -0.04908
#define q_Hz4 0.0

#define R_0 0.080 // CAUTION ONLY FOR 160/70 TIRE

// Combined slip parameter

#define r_Bx1 13.476
#define r_Bx2 11.354
#define C_xalpha 1.1231

#define r_By1 7.7856
#define r_By2 8.1697
#define r_By3 -0.05914
#define C_ykappa 1.0533
*/

typedef struct TIRE_param_strct
{
	// Longitudinal parameter for the tire

	double C_x;

	double p_Dx1;
	double p_Dx2;

	double p_Ex1;
	double p_Ex2;
	double p_Ex3;
	double p_Ex4;

	double p_Kx1;
	double p_Kx2;
	double p_Kx3;

	double F_z0;

	// Lateral parameter for the tire

	double C_y;

	double p_Dy1;
	double p_Dy2;
	double p_Dy3;

	double p_Ey1;
	double p_Ey2;
	double p_Ey4;

	double p_Ky1;
	double p_Ky2;
	double p_Ky3;
	double p_Ky4;
	double p_Ky5;
	double p_Ky6;
	double p_Ky7;

	double C_gamma;
	double E_gamma;

	// Aligning parameter for the tire

	double C_t;

	double q_Bz1;
	double q_Bz2;
	double q_Bz5;
	double q_Bz6;
	double q_Bz9;
	double q_Bz10;

	double q_Dz1;
	double q_Dz2;
	double q_Dz3;
	double q_Dz4;
	double q_Dz8;
	double q_Dz9;
	double q_Dz10;
	double q_Dz11;

	double q_Ez1;
	double q_Ez2;
	double q_Ez5;

	double q_Hz3;
	double q_Hz4;

	double R_0;

	// Combined slip parameter

	double r_Bx1;
	double r_Bx2;
	double C_xalpha;

	double r_By1;
	double r_By2;
	double r_By3;
	double C_ykappa;

} TIRE_param_strct;

int mbs_motorbike_contact(double F[4], double M[4], double beta, double gamma, double kappa, TIRE_param_strct* tire_param_strct);

void mbs_kine_wheel_motorbike(double Pw[4],double Rw[4][4],
					   double Vw[4],double OMw[4],
					   double tsim,int iwhl, double r_rim, double r_t_tire,
					   double *penp, double *rzp, double *anglisp, double *angcambp,
					   double *glissp, double Vct[4], double Rtsol[4][4], double dxF[4]);

void user_WheelForces_motorbike(double pen, double rz, double anglis, double angcamb,
	                  double gliss, double Vctz, double dxF[4],
					  MbsData *mbs_data, double tsim, int iwhl,
					  double* SWr);

TIRE_param_strct* init_TIRE_param_strct_120_70();
TIRE_param_strct* init_TIRE_param_strct_140_70_beta();
TIRE_param_strct* init_TIRE_param_strct_160_70();

void free_TIRE_param_strct(TIRE_param_strct* tire_param_strct);

#endif
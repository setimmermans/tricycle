#ifndef WHEEL_h
#define WHEEL_h

#include "user_wheel.h"
#include "mbs_motorbike_contact.h"

#include "useful_functions.h"
#include "mbs_matrix.h"

#define TM_n_in 4
#define TM_n_out 3

typedef void (*TIRE_model_ptr)(double*, double*, double, double, double, TIRE_param_strct*);

typedef struct WHEEL_param_strct
{
    double K_tire;
	double D_tire;
    double r_n_tire;
	double r_t_tire;

} WHEEL_param_strct;

typedef struct WHEEL_strct
{
	int ext_force_id;
	TIRE_model_ptr tire_model_ptr;
	TIRE_param_strct* tire_param_strct;

	WHEEL_param_strct* param;

	double pen;
	double rz;
	double gliss;
	double anglis; 
	double angcamb;

	double Vct[4], Rtsol[4][4], dxF[4];
	double Fwhl[4], Mwhl[4];

	double TM_in_static_equil[TM_n_in];
	double TM_in_qs_equil[TM_n_in];

	double LTM_in_0[TM_n_in];
	double LTM_out_0[TM_n_out];

	double K_LTM[TM_n_out][TM_n_in]; // jacobian matrix for Linear Tire Model

} WHEEL_strct;

typedef struct WHEEL_vhcl_strct
{
	int n;
	WHEEL_strct** wheel_list;

} WHEEL_vhcl_strct;

WHEEL_param_strct* init_WHEEL_param_strct();
void free_WHEEL_param_strct(WHEEL_param_strct* wheel_param_strct);

WHEEL_strct* init_WHEEL_strct(int ext_force_id, void* tire_model_ptr, TIRE_param_strct* tire_param_strct, WHEEL_param_strct* wheel_param_strct);
void free_WHEEL_strct(WHEEL_strct* wheel_strct); // use only free_WHEEL_vhcl_strct(WHEEL_vhcl_strct* wheel_vhcl_strct);

WHEEL_vhcl_strct* init_WHEEL_vhcl_strct(int n);
void free_WHEEL_vhcl_strct(WHEEL_vhcl_strct* wheel_vhcl_strct);

void cmpt_wheel_force(WHEEL_vhcl_strct* wheel_vhcl_strct, 
					   double PxF[4], double RxF[4][4], 
					   double VxF[4], double OMxF[4], 
					   double AxF[4], double OMPxF[4], 
					   double tsim, int ixF, int mode, double* SWr);

void set_TM_in(WHEEL_strct* wheel_strct, double* TM_in);
void get_TM_in(WHEEL_strct* wheel_strct, double* TM_in);
void set_TM_out(WHEEL_strct* wheel_strct, double* TM_out);
void get_TM_out(WHEEL_strct* wheel_strct, double* TM_out);

void update_LTM_WHEEL_strct(WHEEL_strct* wheel_strct);
void update_LTM_WHEEL_vhcl_strct(WHEEL_vhcl_strct* wheel_vhcl_strct);

void set_TM_in_static_equil_WHEEL_strct(WHEEL_strct* whl);
void reset_TM_in_static_equil_WHEEL_strct(WHEEL_strct* whl);
void set_TM_in_static_equil_WHEEL_vhcl_strct(WHEEL_vhcl_strct* wheel_vhcl_strct);
void reset_TM_in_static_equil_WHEEL_vhcl_strct(WHEEL_vhcl_strct* wheel_vhcl_strct);

void set_TM_in_qs_equil_WHEEL_strct(WHEEL_strct* whl);
void reset_TM_in_qs_equil_WHEEL_strct(WHEEL_strct* whl);
void set_TM_in_qs_equil_WHEEL_vhcl_strct(WHEEL_vhcl_strct* wheel_vhcl_strct);
void reset_TM_in_qs_equil_WHEEL_vhcl_strct(WHEEL_vhcl_strct* wheel_vhcl_strct);

void cmpt_LTM_WHEEL(WHEEL_strct* whl);

WHEEL_strct* find_WHEEL_in_vhcl(WHEEL_vhcl_strct* wheel_vhcl_strct, int ext_force_id);



void mbs_kine_wheel(double Pw[4],double Rw[4][4],
					   double Vw[4],double OMw[4],
					   double tsim,int iwhl,
					   double *rzp, double *anglisp, double *angcambp,
					   double *glissp, double Vct[4], double Rtsol[4][4], double dxF[4]);

void mbs_kine_wheel_motorbike(double Pw[4],double Rw[4][4],
					   double Vw[4],double OMw[4],
					   double tsim, int iwhl, double r_rim, double r_t_tire,
					   double *penp, double *rzp, double *anglisp, double *angcambp,
					   double *glissp, double Vct[4], double Rtsol[4][4], double dxF[4]);

void mbs_bakker(double *Fwhl,double *Mwhl,
			 double anglis, double ancamb, double gliss);
void mbs_bakker_modif(double *Fwhl,double *Mwhl,
			 double anglis, double ancamb, double gliss);
void mbs_bakker_lin_0(double *Fwhl,double *Mwhl,
			 double anglis, double ancamb, double gliss);

void mbs_calspan(double *Fwhl,double *Mwhl,
			 double anglis, double ancamb);



#endif
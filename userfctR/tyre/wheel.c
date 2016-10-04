#include "wheel.h"


WHEEL_param_strct* init_WHEEL_param_strct()
{
	WHEEL_param_strct* wheel_param_strct;

	wheel_param_strct = (WHEEL_param_strct*)malloc(sizeof(WHEEL_param_strct));

	wheel_param_strct->D_tire = 0.0; 
	wheel_param_strct->K_tire = 0.0;
	wheel_param_strct->r_n_tire = 0.0;
	wheel_param_strct->r_t_tire = 0.0;

	return wheel_param_strct;
}
void free_WHEEL_param_strct(WHEEL_param_strct* wheel_param_strct)
{
	free(wheel_param_strct);
}

WHEEL_strct* init_WHEEL_strct(int ext_force_id, 
							  void* tire_model_ptr, 
							  TIRE_param_strct* tire_param_strct, 
							  WHEEL_param_strct* wheel_param_strct)
{
	WHEEL_strct* wheel_strct;

	wheel_strct = (WHEEL_strct*)malloc(sizeof(WHEEL_strct));

	wheel_strct->ext_force_id = ext_force_id;
	wheel_strct->tire_model_ptr = tire_model_ptr;
	wheel_strct->tire_param_strct = tire_param_strct;
	wheel_strct->param = wheel_param_strct;

	wheel_strct->pen = 0.0;
	wheel_strct->rz = 0.0;
	wheel_strct->gliss = 0.0;
	wheel_strct->anglis = 0.0;
	wheel_strct->angcamb = 0.0;

	zeros_double_vec(wheel_strct->Fwhl, 4);
	zeros_double_vec(wheel_strct->Mwhl, 4);

	return wheel_strct;
}
void free_WHEEL_strct(WHEEL_strct* wheel_strct)
{
	free_TIRE_param_strct(wheel_strct->tire_param_strct);
	free(wheel_strct);
}

WHEEL_vhcl_strct* init_WHEEL_vhcl_strct(int n)
{
	int i; 
	WHEEL_vhcl_strct* wheel_vhcl_strct;

	wheel_vhcl_strct = (WHEEL_vhcl_strct*)malloc(sizeof(WHEEL_vhcl_strct));

	wheel_vhcl_strct->n = n;
	
	wheel_vhcl_strct->wheel_list = (WHEEL_strct**)malloc(n*sizeof(WHEEL_strct*));

	return wheel_vhcl_strct;
}
void free_WHEEL_vhcl_strct(WHEEL_vhcl_strct* wheel_vhcl_strct)
{
	int i; 
	for(i=0; i<wheel_vhcl_strct->n; i++)
	{
		free_WHEEL_strct(wheel_vhcl_strct->wheel_list[i]);
	}
	free(wheel_vhcl_strct->wheel_list); 
	free(wheel_vhcl_strct);
}


void cmpt_wheel_force(WHEEL_vhcl_strct* wheel_vhcl_strct, 
					   double PxF[4], double RxF[4][4], 
					   double VxF[4], double OMxF[4], 
					   double AxF[4], double OMPxF[4], 
					   double tsim, int ixF, int mode, double* SWr)
{
	int i, ind_wheel;
	WHEEL_strct* whl;

	ind_wheel = 0;
	while(wheel_vhcl_strct->wheel_list[ind_wheel]->ext_force_id != ixF-1)
	{
		ind_wheel++;
	}
	whl = wheel_vhcl_strct->wheel_list[ind_wheel];

	#ifdef AUTOMOTIVE_KINEWHEEL
		mbs_kine_wheel(PxF,RxF, VxF, OMxF, tsim, ixF, &whl->rz, &whl->anglis, &whl->angcamb, &whl->gliss, whl->Vct, whl->Rtsol, whl->dxF);
		whl->pen = whl->param->r_n_tire - whl->rz * cos(whl->angcamb);
	#endif // AUTOMOTIVE_KINEWHEEL
	#ifdef MOTORBIKE_KINEWHEEL
		mbs_kine_wheel_motorbike(PxF,RxF, VxF, OMxF, tsim, ixF, whl->param->r_n_tire - whl->param->r_t_tire , whl->param->r_t_tire, &whl->pen, &whl->rz, &whl->anglis, &whl->angcamb, &whl->gliss, whl->Vct, whl->Rtsol, whl->dxF);
	#endif // MOTORBIKE_KINEWHEEL
	
	zeros_double_vec(whl->Fwhl, 4);
	zeros_double_vec(whl->Mwhl, 4);
	switch(mode)
	{
		case 1:
			whl->Fwhl[3] = whl->param->K_tire * whl->pen;
		break;
		case 2:
			whl->Fwhl[3] = whl->param->K_tire * whl->pen;
			cmpt_LTM_WHEEL(whl);
		break;
		default:
			if(whl->pen>0.0)
			{
				whl->Fwhl[3] = whl->param->K_tire * whl->pen;	
				whl->tire_model_ptr(whl->Fwhl,whl->Mwhl,whl->anglis,whl->angcamb,whl->gliss, whl->tire_param_strct);


				//cmpt_LTM_WHEEL(whl);
				// mbs_bakker(whl->Fwhl,whl->Mwhl,whl->anglis,whl->angcamb,whl->gliss); // caution !!!
			}
		break;
	}
	/*
	printf("\n");
	print_double_vec(&(whl->Fwhl[1]),3);
	print_double_vec(&(whl->Mwhl[1]),3);*/

	matrix_product(whl->Rtsol, whl->Fwhl, SWr);	
	matrix_product(whl->Rtsol, whl->Mwhl, &(SWr[3]));
	copy_double_vec(&(whl->dxF[1]), &(SWr[7]), 3);  // need to be tcheked!
}



void set_TM_in(WHEEL_strct* whl, double* TM_in)
{
	whl->Fwhl[3] = TM_in[0];
	whl->gliss = TM_in[1];
	whl->anglis = TM_in[2];
	whl->angcamb = TM_in[3];
}
void get_TM_in(WHEEL_strct* whl, double* TM_in)
{
	TM_in[0] = whl->Fwhl[3];
	TM_in[1] = whl->gliss;
	TM_in[2] = whl->anglis;
	TM_in[3] = whl->angcamb;
}

void set_TM_out(WHEEL_strct* whl, double* TM_out)
{
	whl->Fwhl[1] = TM_out[0];
	whl->Fwhl[2] = TM_out[1];
	whl->Mwhl[3] = TM_out[2];
}
void get_TM_out(WHEEL_strct* whl, double* TM_out)
{
	TM_out[0] = whl->Fwhl[1];
	TM_out[1] = whl->Fwhl[2];
	TM_out[2] = whl->Mwhl[3];
}

#define delta_F_n 50.0
#define delta_gliss 0.02 //0.02
#define delta_anglis 0.02 //0.02
#define delta_angcamb 0.01 //0.01

void update_LTM_WHEEL_strct(WHEEL_strct* whl) // can be improve with ptr !!
{
	int i,j;
	double delta_TM_in[TM_n_in] = {delta_F_n, delta_gliss, delta_anglis, delta_angcamb};
	double TM_in_cur[TM_n_in];
	double TM_out_dp[TM_n_out];
	double TM_out_dm[TM_n_out];
	
	get_TM_in(whl, whl->LTM_in_0);
	whl->tire_model_ptr(whl->Fwhl,whl->Mwhl,whl->anglis,whl->angcamb,whl->gliss, whl->tire_param_strct);
	get_TM_out(whl, whl->LTM_out_0);

	copy_double_vec(whl->LTM_in_0,TM_in_cur,TM_n_in);
	for(i=0; i<TM_n_in; i++)
	{
		TM_in_cur[i] += delta_TM_in[i];
		set_TM_in(whl, TM_in_cur);
		whl->tire_model_ptr(whl->Fwhl,whl->Mwhl,whl->anglis,whl->angcamb,whl->gliss, whl->tire_param_strct);
		get_TM_out(whl, TM_out_dp);

		TM_in_cur[i] -= 2.0 * delta_TM_in[i];
		set_TM_in(whl, TM_in_cur);
		TM_in_cur[i] += delta_TM_in[i];
		whl->tire_model_ptr(whl->Fwhl,whl->Mwhl,whl->anglis,whl->angcamb,whl->gliss, whl->tire_param_strct);
		get_TM_out(whl, TM_out_dm);

		for(j=0; j<TM_n_out; j++)
		{
			whl->K_LTM[j][i] = (TM_out_dp[j]-TM_out_dm[j]) / (2.0 * delta_TM_in[i]);
		}
	}
	/*
	printf("\n");
	for(i=0; i<TM_n_out; i++)
	{
		for(j=0; j<TM_n_in; j++)
		{
			printf("	%f",whl->K_LTM[i][j]);
		}
		printf("\n");
	}
	//*/
}
void update_LTM_WHEEL_vhcl_strct(WHEEL_vhcl_strct* wheel_vhcl_strct)
{
	int i; 
	for(i=0; i<wheel_vhcl_strct->n; i++)
	{
		update_LTM_WHEEL_strct(wheel_vhcl_strct->wheel_list[i]);
	}
}

void set_TM_in_static_equil_WHEEL_strct(WHEEL_strct* whl)
{
	get_TM_in(whl, whl->TM_in_static_equil);
	//print_double_vec(whl->TM_in_static_equil,4);
}
void reset_TM_in_static_equil_WHEEL_strct(WHEEL_strct* whl)
{
	set_TM_in(whl, whl->TM_in_static_equil);
}
void set_TM_in_static_equil_WHEEL_vhcl_strct(WHEEL_vhcl_strct* wheel_vhcl_strct)
{
	int i; 
	for(i=0; i<wheel_vhcl_strct->n; i++)
	{
		set_TM_in_static_equil_WHEEL_strct(wheel_vhcl_strct->wheel_list[i]);
	}
}
void reset_TM_in_static_equil_WHEEL_vhcl_strct(WHEEL_vhcl_strct* wheel_vhcl_strct)
{
	int i; 
	for(i=0; i<wheel_vhcl_strct->n; i++)
	{
		reset_TM_in_static_equil_WHEEL_strct(wheel_vhcl_strct->wheel_list[i]);
	}
}

void set_TM_in_qs_equil_WHEEL_strct(WHEEL_strct* whl)
{
	get_TM_in(whl, whl->TM_in_qs_equil);
	//print_double_vec(whl->TM_in_qs_equil,4);
}
void reset_TM_in_qs_equil_WHEEL_strct(WHEEL_strct* whl)
{
	set_TM_in(whl, whl->TM_in_qs_equil);
}
void set_TM_in_qs_equil_WHEEL_vhcl_strct(WHEEL_vhcl_strct* wheel_vhcl_strct)
{
	int i; 
	for(i=0; i<wheel_vhcl_strct->n; i++)
	{
		set_TM_in_qs_equil_WHEEL_strct(wheel_vhcl_strct->wheel_list[i]);
	}
}
void reset_TM_in_qs_equil_WHEEL_vhcl_strct(WHEEL_vhcl_strct* wheel_vhcl_strct)
{
	int i; 
	for(i=0; i<wheel_vhcl_strct->n; i++)
	{
		reset_TM_in_qs_equil_WHEEL_strct(wheel_vhcl_strct->wheel_list[i]);
	}
}

void cmpt_LTM_WHEEL(WHEEL_strct* whl)
{
	int i,j;
	double TM_in_cur[TM_n_in];
	double TM_out_cur[TM_n_out];

	get_TM_in(whl, TM_in_cur);

	for(i=0; i<TM_n_out; i++)
	{
		TM_out_cur[i] =  whl->LTM_out_0[i];
		for(j=1; j<TM_n_in; j++)/////////////////////////////////////////////////////////////////////////////////////////////////////////////// caution ; 
		{
			TM_out_cur[i] += whl->K_LTM[i][j] * (TM_in_cur[j]-whl->LTM_in_0[j]);
		}
		TM_out_cur[i] *= TM_in_cur[0]/whl->LTM_in_0[0]; /////////////////////////////////////////////////////////////////////////////////////////////////////////////// caution ; 
	}
	set_TM_out(whl, TM_out_cur);
}


WHEEL_strct* find_WHEEL_in_vhcl(WHEEL_vhcl_strct* wheel_vhcl_strct, int ext_force_id)
{
	int i=0; 
	while (wheel_vhcl_strct->wheel_list[i]->ext_force_id != ext_force_id)
	{
		i++;
	}
	return wheel_vhcl_strct->wheel_list[i];
}

void mbs_bakker_modif(double *Fwhl,double *Mwhl,
			 double anglis, double ancamb, double gliss)
{
	double R = 1.6;
	mbs_bakker(Fwhl,Mwhl, anglis, ancamb, gliss);
	Fwhl[1] = R *Fwhl[1];
	Fwhl[2] = R *Fwhl[2];
	Mwhl[3] = R *Mwhl[3];
}
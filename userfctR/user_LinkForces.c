//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------

#include "math.h"

#include "mbs_data.h"
#include "user_IO.h"
#include "user_all_id.h"
#include "user_model.h"


//#define Scaled	 
#define Normal	 

double user_LinkForces(double Z, double Zd, MbsData *mbs_data, double tsim, int ilnk)
{
    double Flink, K,K_stop, C, Z_0,Z_max,Z_min;
    UserModel *um = mbs_data->user_model;
/*-- Begin of user code --*/


	Flink=0.0;
#ifdef Normal
        if(ilnk==shock_ft_rt_id|| ilnk==shock_ft_lt_id)
		{
			K= um->shock_ft.K;
			K_stop=um->shock_ft.K_stop;
			C = um->shock_ft.D;
			Z_0 = um->shock_ft.Z_0;
			Z_max=um->shock_ft.Z_max;
			Z_min=um->shock_ft.Z_min;
		}
		else if(ilnk==shock_rr_id)
		{
			K= um->shock_rr.K;
			K_stop=um->shock_rr.K_stop;
			C = um->shock_rr.D;
			Z_0 = um->shock_rr.Z_0;
			Z_max=um->shock_rr.Z_max;
			Z_min=um->shock_rr.Z_min;
		}
		// calculation
		if(Z>Z_max || Z<Z_min)
		{
			K=K_stop;
        }
        Flink = mbs_data->K_factor*K*(Z-Z_0) + C*Zd;
		Flink = Flink * mbs_data->scaling_factor;
#endif

/*-- End of user code --*/    
  
    return  Flink;
}

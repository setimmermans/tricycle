//---------------------------
// author: Timmermans Sebastien
// date: 17/10/16
// version 1.0
//
// Scale les donnees de mbs_data
//---------------------------


#include "Scaling.h"


void Scale_data(MbsData *mbs_data, double scaling_factor)	
{
	//
	//int i, j;

	//// boucle sur les anchor points
	//for (i = 1; i <= mbs_data->npt; i++) {
	//	printf("anchor point %d \n", i);
	//	// boucle  sur x, y , z
	//	for (j = 1; j <= 3; j++) {
	//		
	//		mbs_data->dpt[j][i] = mbs_data->dpt[j][i] * scaling_factor;
	//		printf("anchor point %d = %f \n",j,  mbs_data->dpt[j][i]);
	//	}
	//}

	//int ids[15];
	//int my_id;
	//my_id = 1;
	//ids[1] = body_id;
	//ids[2] = fork_id;
	//ids[3] = wheel_rr_id;
	//ids[4] = body_noze_dn_id;
	//ids[5] = pendulum_id;
	//ids[6] = wishbone_ft_rt_dn_id;
	//ids[7] = carrier_ft_rt_id;
	//ids[8] = wheel_ft_rt_id;
	//ids[9] = wishbone_ft_lt_dn_id;
	//ids[10] = carrier_ft_lt_id;
	//ids[11] = wheel_ft_lt_id;
	//ids[12] = steering_fork_id;
	//ids[13] = body_noze_up_id;
	//ids[14] = wishbone_ft_rt_up_id;
	//ids[15] = wishbone_ft_lt_up_id;
	//
	////boucle sur les rod
	//for (j = 1; j <= mbs_data->Ncons; j++) { // ne sait pas ou trouver le nombre de rod :(

	//	mbs_data->lrod[j] = mbs_data->lrod[j] * scaling_factor;
	//	printf("lrod %d = %f \n", j, mbs_data->lrod[j]);
	//}

	//////boucle sur les joint
	////for (j = 1; j <= mbs_data->njoint; j++) { 

	////	mbs_data->q[j] = mbs_data->q[j] * scaling_factor;
	////	mbs_data->qd[j] = mbs_data->qd[j] * scaling_factor;
	////	mbs_data->qdd[j] = mbs_data->qdd[j] * scaling_factor;
	////	printf("q[ %d] = %f \n", j, mbs_data->q[j]);
	////}



	//// boucle sur les inerties et masses + center of mass
	//for (i = 1; i<mbs_data->nbody; i++)
	//{
	//	my_id = ids[i];
	//	for (j = 1; j<=3; j++)
	//	{
	//		mbs_data->l[j][my_id] = mbs_data->l[j][my_id] * scaling_factor;
	//	}

	//	mbs_data->m[my_id] = mbs_data->m[my_id] * scaling_factor * scaling_factor * scaling_factor;
	//	printf("masse %d = %f \n", i, mbs_data->m[my_id]);

	//	for (j = 1; j <= 9; j++)
	//	{
	//		mbs_data->In[j][my_id] = mbs_data->In[j][my_id] * scaling_factor * scaling_factor;
	//		printf("inertie %d = %f \n", my_id, mbs_data->In[j][my_id]);
	//	}

	//}
}


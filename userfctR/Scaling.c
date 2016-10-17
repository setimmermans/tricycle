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
	
	int i, j;

	// boucle sur les anchor points
	for (i = 1; i <= mbs_data->npt; i++) {
		printf("anchor point %d \n", i);
		// boucle  sur x, y , z
		for (j = 1; j <= 3; j++) {
			
			mbs_data->dpt[j][i] = mbs_data->dpt[j][i] * scaling_factor;
			printf("anchor point %d = %f \n",j,  mbs_data->dpt[j][i]);
		}
	}

	//boucle sur les rod
	for (j = 1; j <= mbs_data->Ncons; j++) { // ne sait pas ou trouver le nombre de rod :(

		mbs_data->lrod[j] = mbs_data->lrod[j] * scaling_factor;
		printf("lrod %d = %f \n", j, mbs_data->lrod[j]);
	}

	//boucle sur les joint
	for (j = 1; j <= mbs_data->njoint; j++) { // ne sait pas ou trouver le nombre de rod :(

		mbs_data->q[j] = mbs_data->q[j] * scaling_factor;
		mbs_data->qd[j] = mbs_data->qd[j] * scaling_factor;
		mbs_data->qdd[j] = mbs_data->qdd[j] * scaling_factor;
		printf("q[ %d] = %f \n", j, mbs_data->q[j]);
	}



	// boucle sur les inerties et masses + center of mass
	for (i = 1; i<mbs_data->nbody; i++)
	{
		for (j = 1; j<=3; j++)
		{
			mbs_data->l[j][i] = mbs_data->l[j][i] * scaling_factor;
		}

		mbs_data->m[i] = mbs_data->m[i] * scaling_factor;
		//printf("masse %d = %f \n", i, mbs_data->m[j]);

		for (j = 1; j <= 9; j++)
		{
			mbs_data->In[j][i] = mbs_data->In[j][i] * scaling_factor * scaling_factor;
			//printf("inertie %d = %f \n", j, mbs_data->In[j][i]);
		}

	}
}


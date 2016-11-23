//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------

#include "math.h"

#include "MBSdef.h"
#include "mbs_data.h"
#include "user_model.h"
#include "user_all_id.h"
#include "user_IO.h"
#include "my_DrivenJoints.h"

void user_DrivenJoints(MbsData *mbs_data, double tsim)
{
	if (mbs_data->process == 3) //Dirdyn
	{
		//		printf("Dirdyn \n");
		if (mbs_data->EntreEnCourbe == 1) // Entre en courbe
		{
			my_DrivenJoints_EnCourbe(mbs_data, tsim);
			
		}
		else // ligne droite
		{
			my_DrivenJoints_LigneDroite(mbs_data, tsim);
		}
	}
}

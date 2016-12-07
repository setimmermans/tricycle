/** ---------------------------
  * Robotran - MBsysC
  * 
  * Template file for direct dynamics module
  * 
  * This files enable the user to call custom at
  * specific places in the time simulation. It is a template
  * file that can be edited by the user.
  * 
  * (c) Universite catholique de Louvain
  *     
  */

#include "math.h"

#include "mbs_data.h"
#include "mbs_dirdyn_struct.h"
#include "set_output.h"
#include "user_all_id.h"
#include "mbs_project_interface.h"
#include "user_IO.h"

/*! \brief user own initialization functions
 *
 * \param[in,out] mbs_data data structure of the model
 * \param[in,out] mbs_dd general structure of the direct dynamic module (for advance users)
 *
 * For beginners, it is advised to only use the MbsData structure.
 * The field MbsDirdyn is provided for more advance users.
 */
void user_dirdyn_init(MbsData *mbs_data, MbsDirdyn *mbs_dd)
{

}

/*! \brief user own loop functions
 *
 * \param[in,out] mbs_data data structure of the model
 * \param[in,out] mbs_dd general structure of the direct dynamic module (for advance users)
 *
 * For beginners, it is advised to only use the MbsData structure.
 * The field MbsDirdyn is provided for more advance users.
 */
void user_dirdyn_loop(MbsData *mbs_data, MbsDirdyn *mbs_dd)
{

	mbs_data->last_tilt_torque = mbs_data->Qq[R1_pendulum_id];
	//printf("mbs_data->last_tilt_torque = %f \n", mbs_data->last_tilt_torque);
	
	
	// sensors
	int id = sensor_body_bottom_id;

	// retrieve the pointer to the sensor structure defined in mbs_aux
	MbsSensor *PtrSensor = mbs_dd->mbs_aux->psens;
	
	// compute the sensor (position, velocity...)
	mbs_sensor(PtrSensor, mbs_data, id);

	// save the vertical acceleration
	set_output(PtrSensor->A[3], "Sensor_pilot_AccelerationZ");
	// save the position
	set_output(PtrSensor->P[1], "Sensor_pilot_PositionX");
	set_output(PtrSensor->P[2], "Sensor_pilot_PositionY");
	set_output(PtrSensor->P[3], "Sensor_pilot_PositionZ");


	// ForcesNormales
	set_output(mbs_data->user_IO->Fn_lft, "Fn_lft");
	set_output(mbs_data->user_IO->Fn_rgt, "Fn_rgt");

}

/*! \brief user own finishing functions
 *
 * \param[in,out] mbs_data data structure of the model
 * \param[in,out] mbs_dd general structure of the direct dynamic module (for advance users)
 *
 * For beginners, it is advised to only use the MbsData structure.
 * The field MbsDirdyn is provided for more advance users.
 */
void user_dirdyn_finish(MbsData *mbs_data, MbsDirdyn *mbs_dd)
{

}

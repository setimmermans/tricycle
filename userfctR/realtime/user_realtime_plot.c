/*! 
 * \author Nicolas Van der Noot
 * \file user_realtime_plot.c
 * \brief Configure the SDL screen to plot simulations values
 *
 * In order to use C++ features, you just need to change the extension of this file (.c) to .cc (or .cpp).
 */

#ifdef SDL


#include "mbs_data.h"
#include "user_realtime.h"
#include "user_all_id.h"

/*! \brief assign values for the SDL functions
 *
 * \param[in] mbs_data Robotran main structure
 *
 * To plot a curve, use 'set_plot(value, label)' where 'value' is the value you want to plot
 *     and 'label' is a string corresponding to the curve label. You can use the 'set_plot' function
 *     in the following function ('user_realtime_plot') or anywhere in your code, provided you add
 *     the include '#include "user_realtime.h"' in the corresponding file. Using 'user_realtime_plot' is
 *     still useful to structure the code, avoiding to put the 'set_plot' function everywhere in the code.
 *     However, using 'set_plot' in other files can be faster and is especially relevant fot C++ code
 *     where some internal variables are private and cannot be used outside the corresponding class.
 *     To plot different curves, you must use different labels (otherwise, some curves won't be plotted).
 *
 * example:
 *   set_plot(mbs_data->q[4], "q4 [rad]");
 */
void user_realtime_plot(MbsData* mbs_data)
{




	set_plot(mbs_data->q[R1_body_id], "R1 body");
	set_plot(mbs_data->q[R3_steering_fork_id], "Steering Fork");
	set_plot(mbs_data->Qq[R1_pendulum_id], "Couple Pendule");
	set_plot(mbs_data->Qq[R3_steering_fork_id], "Couple steering Fork");

	//double V;
	//V = sqrt(  mbs_data->qd[T1_body_id]* mbs_data->qd[T1_body_id] + mbs_data->qd[T2_body_id]* mbs_data->qd[T2_body_id]);

	//set_plot(mbs_data->Qq[R2_wheel_rr_id], "Couple rr");
	//set_plot(mbs_data->Qc[R2_wheel_rr_id], "Couple rr Qc");
	//set_plot(mbs_data->qd[T1_body_id], "Vitesse X");
	//set_plot(V, "Vitesse abs");
}

#endif

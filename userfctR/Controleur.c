//---------------------------
// author: Tims seb
// date: 29/10/16
// version 1.0
//
// Controleur du tilt  
//--------------------------


#include "Controleur.h"


#define Scaled	 
//#define Normal	 

double my_controleur(MbsData *mbs_data, double tsim, double speed, double steering)
{
	double My_torque, delta_err, tilt_ref, max_torque, speed_tilt_ref, delta_speed, diff_max_torque, torque_step_max;
	mbs_data->Kp =  100.0;// *(mbs_data->speed_ref*mbs_data->speed_ref / mbs_data->Rayon);//100
	mbs_data->Ki = 10;// 1.0;//1
	mbs_data->Kd = 10;// 20;//100
	max_torque = 50.0;


	tilt_ref = tilt_reference(mbs_data, tsim, speed, steering);
	speed_tilt_ref = -fabs((mbs_data->last_tilt_ref - tilt_ref) / 0.001);
	if (tsim > mbs_data->t_start && mbs_data->EntreEnCourbe==1)
	{
		speed_tilt_ref =  EntreEnCourbe_DTC_tilt_ref_vitesse(mbs_data, tsim);
	}

	delta_err = tilt_ref - mbs_data->q[R1_body_id];
	delta_speed = speed_tilt_ref - mbs_data->qd[R1_body_id];
	My_torque = (-mbs_data->Kp * delta_err - mbs_data->Ki * mbs_data->ErrorTot - mbs_data->Kd *  delta_speed);
	diff_max_torque = fabs(My_torque - mbs_data->last_tilt_torque);
	torque_step_max = 1;
	if (speed_tilt_ref >= 0.001)
	{
		//printf("Delta err %f et  My_torque = %f et ErrorTot = %f et tilt ref = %f et 	mbs_data->last_tilt_ref =%f et speed_tilt_ref =%f et tsim %f \n", delta_err, My_torque, mbs_data->ErrorTot, tilt_ref, mbs_data->last_tilt_ref, speed_tilt_ref, tsim);
	}
	mbs_data->last_tilt_ref = tilt_ref;

	if (fabs(mbs_data->q[R1_body_id]) > 1)
	{
		printf("Delta err %f et  My_torque = %f et ErrorTot = %f et tilt ref = %f et speed_tilt_ref =%f \n", delta_err, My_torque, mbs_data->ErrorTot, tilt_ref, speed_tilt_ref);
	}
	if (fabs(delta_err) > 0.001)
	{
		mbs_data->ErrorTot += delta_err * 0.001; // time step
	}
	if (fabs(diff_max_torque) > torque_step_max)
	{
		if (fabs(My_torque) > max_torque)
		{
			return max_torque*sign(My_torque);
		}
		else
		{
			return mbs_data->last_tilt_torque + torque_step_max*sign(My_torque);
		}
	}
	else
	{
		if (fabs(My_torque) > max_torque)
		{
			return max_torque*sign(My_torque);
		}
		else
		{
			return My_torque;
		}

	}
}
//DTC

double my_controleur_stc(MbsData *mbs_data, double tsim, double speed, double steering) //STC
{
	double My_torque_tilt, delta_err, tilt_ref, max_torque, speed_tilt_ref, delta_speed, My_torque, torque_step_max, diff_max_torque;
	mbs_data->Kp = 10;//*mbs_data->speed_ref;//200 normal //5 avant chgmt
	mbs_data->Ki = 1.0;// 10;// 10;// 10;//100 normal // 100avant chgmt
	mbs_data->Kd = 5.0;// 5.0;// *sqrt(mbs_data->speed_ref / 2);// 10;//100 normal // 10 avant chgmt
	max_torque = 2.0;

	tilt_ref = tilt_reference(mbs_data, tsim, speed, steering);
	speed_tilt_ref = 0.0;// fabs((mbs_data->last_tilt_ref - tilt_ref) / 0.001);
	if (tsim > mbs_data->t_start &&  mbs_data->EntreEnCourbe == 1)
	{
		speed_tilt_ref = EntreEnCourbe_STC_tilt_ref_vitesse(mbs_data, tsim);  //(mbs_data->last_tilt_ref - tilt_ref) / 0.001;
		//printf("speed_tilt_ref = %f  car veut etre  en courbe \n",speed_tilt_ref);
	}
	
	mbs_data->last_tilt_ref = tilt_ref;
	delta_err = tilt_ref - mbs_data->q[R1_body_id];
	delta_speed = speed_tilt_ref - mbs_data->qd[R1_body_id];

	My_torque_tilt = mbs_data->Kp * delta_err + mbs_data->Kd *  delta_speed + mbs_data->Ki * mbs_data->ErrorTot;
	diff_max_torque = fabs(My_torque_tilt - mbs_data->last_tilt_torque);
	torque_step_max = 0.5;

	//My_torque_steer = mbs_data->Qq[R3_steering_fork_id];
	if (mbs_data->EstEnCourbe == 1)
	{
		My_torque = (My_torque_tilt);// -My_torque_steer);//
	}
	else
	{
		My_torque = (My_torque_tilt);// -My_torque_steer );//
	}

	if (fabs(mbs_data->q[R1_body_id]) > 0.1)
	{
		printf("Delta err*Kp %f et  My_torque = %f et Ki* ErrorTot = %f et vit R1 body = %f \n", mbs_data->Kp *delta_err, My_torque, mbs_data->Ki * mbs_data->ErrorTot, delta_speed);
	}

	if (fabs(delta_err) > 0.0001)
	{
		mbs_data->ErrorTot += delta_err * 0.001; // time step
	}


	if (fabs(diff_max_torque) > torque_step_max)
	{
		if (fabs(My_torque) > max_torque)
		{
			return max_torque*sign(My_torque);
		}
		else
		{
			return mbs_data->last_tilt_torque + torque_step_max*sign(My_torque);
		}
	}
	else
	{
		if (fabs(My_torque) > max_torque)
		{
			return max_torque*sign(My_torque);
		}
		else
		{
			return My_torque;
		}

	}
}

double tilt_reference(MbsData *mbs_data, double tsim, double speed, double steering)
{
	double the_tilt, my_tilt_control, translation_acc, acc_gyro, vit_lacet, Rayon_instantane;
	the_tilt = 0.0;
	my_tilt_control = 0.0;
	// look up table CurveEquilibrium.txt
	//FILE *reading_file_SteerAngle = NULL;
	//char chaine[1000] = "";
	//int flag = 0;
	//const char separation_term[2] = " ";
	//char *sub_chaine;
	//double Rayon_file, speed_file, steer_file;
	//Rayon_file = 0.0;
	//speed_file = 0.0;
	//steer_file = 0.0;
	//reading_file_SteerAngle = fopen(PROJECT_BINARY_DIR"/CurveEquilibrium.txt", "r");
	//if (reading_file_SteerAngle != NULL)
	//{
	//	//printf("Fichier CurveEquilibrium.txt a bien ete ouvert\n");
	//	while (fgets(chaine, 1000, reading_file_SteerAngle) != NULL) // On lit le fichier ligne par ligne tant qu'on ne re�oit pas d'erreur (NULL)
	//	{
	//		if (flag == 0 && strcmp(chaine, "CurveEquilibrium\n") == 0)	// Une fois arriv� � la ligne contenant "CurveEquilibrium"
	//		{
	//			fseek(reading_file_SteerAngle, 2, SEEK_CUR);		// avance le curseur de deux caracteres pour passer le \n
	//			flag = 1;											// mnt qu'on a trouve le debut du fichier, on mais le flag ON
	//		}
	//		else if (flag == 1 && strcmp(chaine, "\n") == 0)	// � la fin du fichier on stoppe la lecture
	//		{
	//			break;
	//		}
	//		else if (flag == 1) {
	//			// on separe la ligne en 6 parties
	//			sub_chaine = strtok(chaine, separation_term); //R
	//			sub_chaine = strtok(NULL, separation_term); 
	//			Rayon_file = atof( sub_chaine );
	//			sub_chaine = strtok(NULL, separation_term); //V
	//			sub_chaine = strtok(NULL, separation_term); 
	//			speed_file = atof(sub_chaine);
	//			sub_chaine = strtok(NULL, separation_term); //SteerAngle
	//			//printf("test: %s\n ", sub_chaine);
	//			sub_chaine = strtok(NULL, separation_term);
	//			steer_file = atof(sub_chaine);
	//			//printf("R = %f et V = %f et steer = %f \n", Rayon_file, speed_file, steer_file);
	//		}
	//	}
	//	fclose(reading_file_SteerAngle);
	//	//printf("Fichier de lecture CurveEquilibrium a bien ete ferme\n");
	//}
	//else
	//{
	//	printf("Impossible d'ouvrir le fichier CurveEquilibrium.txt \n");
	//} 
	translation_acc = 0.0;

	// lecture fichier database tilt et v et R=> fonctionne pas
	double I_wheel_lat, I_wheel_rot, w_rot, h, m, g;
	if (tsim > 0.1)

	{
		translation_acc = 0.0;// mbs_data->qdd[T1_body_id];
	}
	vit_lacet = mbs_data->qd[T2_body_id] / mbs_data->Rayon;
	I_wheel_lat = 0.0176;
	I_wheel_rot = 0.0352;
	w_rot = speed / mbs_data->Rayon;
	h = 0.2;
	m = 15;
	g = 9.81;

	acc_gyro = 2.0 * ((I_wheel_lat - I_wheel_rot) * w_rot * vit_lacet) / (h*m*g);

	//printf(" 2 translation acc =%f \n ", translation_acc);

	if (mbs_data->tourne == 0 && mbs_data->EntreEnCourbe != 1) //ligne droite 
	{
		the_tilt = 0.0;
	}
	else
	{
		if (mbs_data->EstEnCourbe == 1)
		{
			the_tilt = atan(speed*speed / (mbs_data->Rayon *g));// +translation_acc / g - acc_gyro;

			if (mbs_data->user_IO->modeTC == 2) // DTC
			{
				the_tilt = atan(speed*speed * (-mbs_data->q[R3_steering_fork_id]) / (0.35*g)) + translation_acc / g + acc_gyro; ///////////::quidddddddddddddddddddddd
				//the_tilt = -0.064;
			}

			//
			//if (mbs_data->DoubleBande == 1)
			//{
			//	the_tilt = atan(speed*speed *mbs_data->q[R3_steering_fork_id] / (g* 0.35))  - acc_gyro;
			//}

		}
		else // ligne droite avant pertubation?
		{
			the_tilt = 0.0;
		}
	}
	my_tilt_control = EntreEnCourbe_STC_tilt_ref(mbs_data, tsim, the_tilt);

	//printf("the tilt =%f et gyro =%f  \n ", the_tilt, 1000000.0 * acc_gyro);

	double f_p, f_pp, my_value,y_ref_stc;
	f_p = 0.0;
	f_pp = 0.0;
	Rayon_instantane = mbs_data->Rayon;
	//y_ref_stc = my_DrivenJoints_controleur_y(mbs_data, tsim, mbs_data->t_start);
	//if (y_ref_stc - mbs_data->last_f > 0.00001)
	//{
	//	f_p = (y_ref_stc - mbs_data->last_f) / 0.001; //vit de courbure

	//	f_pp = (f_p - mbs_data->last_fp) / 0.001; // acc de courbure}
	//}
	//mbs_data->last_f = y_ref_stc;
	//my_value = (1 + f_p*f_p) * sqrt(1 + f_p*f_p) / f_pp;
	//mbs_data->last_fp = f_p;



	////double t_f, t_c, a_c, consigne_simple_bande;

	////t_f = 1.0;// mbs_data->speed_ref;
	////t_c = 0.5;// (mbs_data->speed_ref) / 2;
	////a_c = (-4 * mbs_data->user_IO->steer) / (((0.5*t_f - t_c) * 2)*((0.5*t_f - t_c) * 2) - t_f*t_f);

	////my_value = my_DrivenJoints_Steering_smooth_consigne(mbs_data, tsim, t_f, t_c, 2 * a_c, mbs_data->t_start, 0.0, -2 * mbs_data->user_IO->steer);

	//if (fabs(f_pp) >= 0.1)
	//{
	//	
	//	Rayon_instantane = my_value;
	//	//printf("the tilt control =%f et Rayon_instantane = %f et f_p =%f et f_pp =%f  et  time =%f et y_ref =%f \n ", my_tilt_control, Rayon_instantane, f_p, f_pp, tsim, y_ref_stc);
	//
	//}
	

	
	if (mbs_data->user_IO->modeTC == 1) //STC
	{
		//	my_tilt_control = EntreEnCourbe_STC_tilt_ref(mbs_data, tsim, the_tilt); 
			//printf("the tilt control =%f et time =%f  \n ", my_tilt_control, tsim);
		if (tsim > 19.99)
		{
			printf("the tilt control  =%f \n ", my_tilt_control);
		}

		if (mbs_data->DoubleBande == 1)
		{
			my_tilt_control = atan(speed*speed / (Rayon_instantane *g));
		}
		//return my_tilt_control;
		return my_tilt_control;
		//return the_tilt;
	}
	else
	{
		if (tsim > 14.99)
		{
			printf("the tilt  =%f \n ", the_tilt);
		}
		//return my_tilt_control;
		return the_tilt;
	}


}


double EntreEnCourbe_DTC_tilt_ref_vitesse(MbsData *mbs_data, double tsim)
{
	// entre en courbe
	double t_f, t_c, a_c, t_start, the_tilt;

	the_tilt = EntreEnCourbe_STC_tilt_ref(mbs_data, tsim, atan(mbs_data->speed_ref* mbs_data->speed_ref / (mbs_data->Rayon *9.81))); // atan(mbs_data->speed_ref* mbs_data->speed_ref * (-mbs_data->q[R3_steering_fork_id]) / (0.35*9.81)); ///////////::quidddddddddddddddddddddd
	t_f = mbs_data->speed_ref;
	t_c = mbs_data->speed_ref / 2;
	a_c = (-4 * the_tilt) / (((0.5*t_f - t_c) * 2)*((0.5*t_f - t_c) * 2) - t_f*t_f);
	t_start = mbs_data->t_start; //Temps d'initiation du tournant

	double consigne_smooth_vit;
	consigne_smooth_vit = 0.0;

	if (tsim <= t_start + t_c)
	{
		consigne_smooth_vit = a_c* (tsim - t_start);
	}
	else if (tsim > (t_start + t_c) && tsim <= t_start + (t_f - t_c))
	{
		consigne_smooth_vit = a_c * t_c;
	}
	else if (tsim < (t_start + t_f) && tsim >= t_start + (t_f - t_c))
	{
		consigne_smooth_vit = -a_c *((tsim - t_start) - t_f);
	}
	else
	{
		consigne_smooth_vit = 0.0;

	}

	return consigne_smooth_vit;

}




double EntreEnCourbe_STC_tilt_ref(MbsData *mbs_data, double tsim, double the_tilt) //STC
{
	// entre en courbe
	double t_f, t_c, a_c, my_tilt_control, t_start;

	t_f = 0.5;// mbs_data->speed_ref;
	t_c = 0.25;// mbs_data->speed_ref / 2;
	a_c = (-4 * the_tilt) / (((0.5*t_f - t_c) * 2)*((0.5*t_f - t_c) * 2) - t_f*t_f);
	t_start = mbs_data->t_start; //Temps d'initiation du tournant


	if (tsim < t_start)
	{
		my_tilt_control = 0.0;
	}
	else
	{
		mbs_data->EstEnCourbe = 1;
		if (tsim <= t_start + t_c)
		{
			my_tilt_control = 0.5*a_c* (tsim - t_start)*(tsim - t_start);
		}
		else if (tsim > (t_start + t_c) && tsim <= t_start + (t_f - t_c))
		{
			my_tilt_control = a_c * t_c * (tsim - t_start - 0.5*t_c);
		}
		else if (tsim < (t_start + t_f) && tsim >= t_start + (t_f - t_c))
		{
			my_tilt_control = the_tilt - 0.5 *	a_c *((tsim - t_start) - t_f) *((tsim - t_start) - t_f);
		}
		else
		{
			my_tilt_control = the_tilt;

		}

	}
	//printf("the tilt control= %f \n", my_tilt_control);
	return my_tilt_control;

}


double EntreEnCourbe_STC_tilt_ref_vitesse(MbsData *mbs_data, double tsim)
{
	// entre en courbe
	double t_f, t_c, a_c, t_start, the_tilt;

	the_tilt = atan(mbs_data->speed_ref* mbs_data->speed_ref / (mbs_data->Rayon *9.81));

	t_f = mbs_data->speed_ref;
	t_c = mbs_data->speed_ref / 2;
	a_c = (-4 * the_tilt) / (((0.5*t_f - t_c) * 2)*((0.5*t_f - t_c) * 2) - t_f*t_f);
	t_start = mbs_data->t_start; //Temps d'initiation du tournant

	double consigne_smooth_vit;
	consigne_smooth_vit = 0.0;

	if (tsim <= t_start + t_c)
	{
		consigne_smooth_vit = a_c* (tsim - t_start);
	}
	else if (tsim > (t_start + t_c) && tsim <= t_start + (t_f - t_c))
	{
		consigne_smooth_vit = a_c * t_c;
	}
	else if (tsim < (t_start + t_f) && tsim >= t_start + (t_f - t_c))
	{
		consigne_smooth_vit = -a_c *((tsim - t_start) - t_f);
	}
	else
	{
		consigne_smooth_vit = 0.0;

	}

	return consigne_smooth_vit;

}

void Print_q_qd_qdd_Qq(MbsData *mbs_data)
{


	printf("q  : "); print_dvec_0(mbs_data->q, mbs_data->njoint);
	printf("qd : "); print_dvec_0(mbs_data->qd, mbs_data->njoint);
	printf("qdd: "); print_dvec_0(mbs_data->qdd, mbs_data->njoint);
	printf("Qq : "); print_dvec_0(mbs_data->Qq, mbs_data->njoint);
}
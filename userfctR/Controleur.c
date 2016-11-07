//---------------------------
// author: Tims seb
// date: 29/10/16
// version 1.0
//
// Controleur du tilt  
//--------------------------


#include "Controleur.h"


//#define Scaled	 
#define Normal	 

double my_controleur(MbsData *mbs_data, double tsim, double speed, double steering)
{
	double My_torque, delta_err, tilt_ref, max_torque, speed_tilt_ref, delta_speed;
	mbs_data->Kp = 1000.0;
	mbs_data->Ki = 1;
	mbs_data->Kd = 100;
	max_torque = 100.0;

	tilt_ref = tilt_reference(mbs_data, tsim, speed, steering);
	speed_tilt_ref = 0.0;
	delta_err = tilt_ref - mbs_data->q[R1_body_id];
	delta_speed = speed_tilt_ref - mbs_data->qd[R1_body_id];
	My_torque = -mbs_data->Kp * delta_err + mbs_data->Ki * mbs_data->ErrorTot - mbs_data->Kd *  delta_speed;
	if (abs(mbs_data->q[R1_body_id]) > 0.0001)
	{
		printf("Delta err %f et  My_torque = %f et ErrorTot = %f et tilt ref = %f\n", delta_err, My_torque, mbs_data->ErrorTot, tilt_ref);
	}
	mbs_data->ErrorTot += delta_err * 0.001; // time step

	if (abs(My_torque) > max_torque)
	{
		return max_torque*sign(My_torque);
	}
	else
	{
		return My_torque;
	}
}

double tilt_reference(MbsData *mbs_data, double tsim, double speed, double steering)
{
	double the_tilt;
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
	//	while (fgets(chaine, 1000, reading_file_SteerAngle) != NULL) // On lit le fichier ligne par ligne tant qu'on ne reçoit pas d'erreur (NULL)
	//	{
	//		if (flag == 0 && strcmp(chaine, "CurveEquilibrium\n") == 0)	// Une fois arrivé à la ligne contenant "CurveEquilibrium"
	//		{
	//			fseek(reading_file_SteerAngle, 2, SEEK_CUR);		// avance le curseur de deux caracteres pour passer le \n
	//			flag = 1;											// mnt qu'on a trouve le debut du fichier, on mais le flag ON
	//		}
	//		else if (flag == 1 && strcmp(chaine, "\n") == 0)	// à la fin du fichier on stoppe la lecture
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
	if (mbs_data->tourne == 0  && mbs_data->EntreEnCourbe !=1) //ligne droite 
	{
		the_tilt = 0.0;
	}
	else
	{	if(mbs_data->EstEnCourbe == 1) 
		{ 
			the_tilt = atan(speed*speed / (mbs_data->Rayon*9.81));
		//	printf("the tilt =%f \n ", the_tilt);
		}
		else // ligne droite avant et après pertubation?
		{
			the_tilt = 0.0;
		}
	}
	return the_tilt;



}


double my_controleur_stc(MbsData *mbs_data, double tsim, double speed, double steering) //STC
{
	double My_torque_tilt, delta_err, tilt_ref, max_torque, speed_tilt_ref, delta_speed, My_torque_steer, My_torque;
	mbs_data->Kp = 20;//200 normal
	mbs_data->Ki = 10;//100 normal
	mbs_data->Kd = 10;//100 normal
	max_torque = 2.0;

	tilt_ref = tilt_reference(mbs_data, tsim, speed, steering);
	speed_tilt_ref = 0.0;
	delta_err = tilt_ref - mbs_data->q[R1_body_id];
	delta_speed = speed_tilt_ref - mbs_data->qd[R1_body_id];
	My_torque_tilt = mbs_data->Kp * delta_err + mbs_data->Kd *  delta_speed  -mbs_data->Ki * mbs_data->ErrorTot;
	My_torque_steer = mbs_data->Qq[R3_steering_fork_id];
	if (mbs_data->EstEnCourbe == 1)
	{
		My_torque = (My_torque_tilt- My_torque_steer );//
	}
	else
	{
		My_torque = (My_torque_tilt -My_torque_steer );//
	}

	/*if (abs(mbs_data->q[R1_body_id]) > 0.00001)
	{*/
	//printf("Delta err*Kp %f et  My_torque = %f et Ki* ErrorTot = %f et vit R1 body = %f \n", mbs_data->Kp *delta_err, My_torque, mbs_data->Ki * mbs_data->ErrorTot, delta_speed);
	//}
	if (abs(delta_err)>0.0001)
	{
		mbs_data->ErrorTot += delta_err * 0.001; // time step
	}
	

	if (abs(My_torque) > max_torque)
	{
		return max_torque*sign(My_torque);
	}
	else
	{
		return My_torque;
	}

}

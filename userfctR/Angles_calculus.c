//---------------------------
// author: Timmermans Sebastien
// date: 17/10/16
// version 1..0
//
// Calcul des angles camber, caster et toe
//---------------------------


#include "Angles_calculus.h"
#include "user_all_id.h"



void Angles(MbsData *mbs_data)
{
	double X_carrrier_up_left, X_carrrier_up_right, Z_carrrier_up_left, Z_carrrier_up_right; // carrier up
	X_carrrier_up_left = 0.0;
	X_carrrier_up_right = 0.0;
	Z_carrrier_up_left = 0.0;
	Z_carrrier_up_right = 0.0;
	 
	double X_carrrier_down_left, X_carrrier_down_right, Z_carrrier_down_left, Z_carrrier_down_right; // carrier down
	X_carrrier_down_left = 0.0;
	X_carrrier_down_right = 0.0;
	Z_carrrier_down_left = 0.0;
	Z_carrrier_down_right = 0.0;

	MbsSensor psens[1];                        // Creation of a pointer to a sensor struct.
	allocate_sensor(psens, mbs_data->njoint);  // Allocate the Jacobian at the correct dimension
	init_sensor(psens, mbs_data->njoint);      // Initialize all value to zero


	mbs_sensor(psens, mbs_data, sens_car_down_lt_id); // Compute the sensor carrier down left
	X_carrrier_down_left = psens->P[1];
	Z_carrrier_down_left = psens->P[3];

	printf("carrier down left :  X = %f,  Z = %f \n", psens->P[1], psens->P[3]);

	mbs_sensor(psens, mbs_data, sens_car_up_lt_id); // Compute the sensor carrier up left
	X_carrrier_up_left = psens->P[1];
	Z_carrrier_up_left = psens->P[3];
	printf("carrier up left :  X = %f,  Z = %f \n", psens->P[1], psens->P[3]);



	mbs_sensor(psens, mbs_data, sens_car_down_rt_id); // Compute the sensor carrier down right
	X_carrrier_down_right = psens->P[1];
	Z_carrrier_down_right = psens->P[3];
	printf(" carrier down right :  X = %f,  Z = %f \n", psens->P[1], psens->P[3]);

	mbs_sensor(psens, mbs_data, sens_car_up_rt_id); // Compute the sensor carrier up right
	X_carrrier_up_right = psens->P[1];
	Z_carrrier_up_right = psens->P[3];
	printf(" carrier up right :  X = %f,  Z = %f \n", psens->P[1], psens->P[3]);

	printf("\n\n gauche : Delta X = %f, Delta Z = %f \n",  X_carrrier_up_left - X_carrrier_down_left, Z_carrrier_up_left - Z_carrrier_down_left);
	printf("droite : Delta X = %f, Delta Z = %f \n\n ",  X_carrrier_up_right - X_carrrier_down_right, Z_carrrier_up_right - Z_carrrier_down_right);


	mbs_data->user_IO->equil_ft_lt_caster = atan2(  X_carrrier_up_left - X_carrrier_down_left, Z_carrrier_up_left - Z_carrrier_down_left);
	mbs_data->user_IO->equil_ft_rt_caster = atan2((X_carrrier_up_right - X_carrrier_down_right), (Z_carrrier_up_right - Z_carrrier_down_right));


	printf("\n\n Angles : camber gauche (carrossage) = %f , camber droite (carrossage) = %f \n", mbs_data->user_IO->equil_ft_lt_camber * 180 / 3.1415, mbs_data->user_IO->equil_ft_rt_camber * 180 / 3.1415);
	printf(" Angles : caster gauche (chasse) = %f , caster droite (chasse) = %f  \n", mbs_data->user_IO->equil_ft_lt_caster * 180 / 3.1415, mbs_data->user_IO->equil_ft_rt_caster * 180 / 3.1415);
	printf(" Angles : toe gauche (pin�age) = %f, toe droite (pin�age) = %f and steer = %f  \n \n", mbs_data->user_IO->equil_ft_rt_toe * 180 / 3.1415, mbs_data->user_IO->equil_ft_rt_toe * 180 / 3.1415, mbs_data->user_IO->dirdyn_ft_rt_steer);

	free_sensor(psens);                        // Free the memory (always better)
   

}


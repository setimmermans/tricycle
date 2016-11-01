/* --------------------------------------------------------
 * This code was generated automatically by MBsysC modules.
 * MBsysC modules are distributed as part of the ROBOTRAN 
 * software. They provides functionalities for dealing with
 * symbolic equations generated by ROBOTRAN. 
 *
 * More info on www.robotran.be 
 *
 * Universite catholique de Louvain, Belgium 
 *
 * Last update : Tue Nov  1 11:13:49 2016
 * --------------------------------------------------------
 *
 */
#ifndef USER_ALL_ID_h
#define USER_ALL_ID_h

// ============================================================ //


// joint

#define R3_body_qs_id 1
#define T1_body_id 2
#define T2_body_id 3
#define T3_body_id 4
#define R3_body_id 5
#define R1_body_id 6
#define R2_body_id 7
#define R2_fork_id 8
#define R2_wheel_rr_id 9
#define R2_body_noze_dn_id 10
#define R1_pendulum_id 11
#define R1_wishbone_ft_rt_dn_id 12
#define R1_carrier_ft_rt_id 13
#define R2_carrier_ft_rt_id 14
#define R3_carrier_ft_rt_id 15
#define R1_wheel_ft_rt_id 16
#define R2_wheel_ft_rt_id 17
#define R1_wishbone_ft_lt_dn_id 18
#define R1_carrier_ft_lt_id 19
#define R2_carrier_ft_lt_id 20
#define R3_carrier_ft_lt_id 21
#define R1_wheel_ft_lt_id 22
#define R2_wheel_ft_lt_id 23
#define R2_streering_fork_id 24
#define R3_steering_fork_id 25
#define R2_body_noze_up_id 26
#define R3_wishbone_ft_rt_up_id 27
#define R1_wishbone_ft_rt_up_id 28
#define R3_wishbone_ft_lt_up_id 29
#define R1_wishbone_ft_lt_up_id 30


// body

#define body_id 7
#define fork_id 8
#define wheel_rr_id 9
#define body_noze_dn_id 10
#define pendulum_id 11
#define wishbone_ft_rt_dn_id 12
#define carrier_ft_rt_id 15
#define wheel_ft_rt_id 17
#define wishbone_ft_lt_dn_id 18
#define carrier_ft_lt_id 21
#define wheel_ft_lt_id 23
#define steering_fork_id 25
#define body_noze_up_id 26
#define wishbone_ft_rt_up_id 28
#define wishbone_ft_lt_up_id 30


// point

//#define origin_id 0
#define body_to_fork_id 1
#define body_to_shock_rr_id 2
#define body_to_sensor_id 3
#define body_to_body_noze_dn_id 4
#define body_to_steering_fork_id 5
#define body_to_body_noze_up_id 6
#define fork_to_wheel_rr_id 7
#define fork_to_shock_rr_id 8
//#define origin_id 9
#define body_noze_dn_to_wishbone_ft_rt_dn_id 10
#define body_noze_dn_to_wishbone_ft_lt_dn_id 11
#define pendulum_to_shock_ft_rt_id 12
#define pendulum_to_shock_ft_lt_id 13
#define Point_0_id 14
#define wishbone_ft_rt_dn_to_carrier_ft_rt_id 15
#define wishbone_ft_rt_dn_to_shock_ft_rt_id 16
//#define origin_id 17
#define carrier_ft_rt_to_wishbone_ft_rt_up_id 18
#define carrier_ft_rt_to_wheel_ft_rt_id 19
#define carrier_ft_rt_to_sterring_rod_ft_rt_id 20
//#define origin_id 21
#define wishbone_ft_lt_dn_to_carrier_ft_lt_id 22
#define wishbone_ft_lt_dn_to_shock_ft_lt_id 23
//#define origin_id 24
#define carrier_ft_lt_to_wishbone_ft_lt_up_id 25
#define carrier_ft_lt_to_wheel_ft_lt_id 26
#define carrier_ft_lt_to_sterring_rod_ft_lt_id 27
//#define origin_id 28
#define steering_fork_to_steering_rod_rt_id 29
#define steering_fork_to_steering_rod_lt_id 30
#define body_noze_up_to_wishbone_ft_rt_up_id 31
#define body_noze_up_to_wishbone_ft_lt_up_id 32
#define wishbone_ft_rt_up_to_carrier_ft_rt_id 33
#define wishbone_ft_lt_up_to_carrier_ft_lt_id 34


// sensor

#define sensor_body_bottom_id 1
#define sensor_pilot_id 2
#define sens_car_down_rt_id 3
#define sens_car_up_rt_id 4
#define sens_ft_rt_id 5
#define Sensor_wheel_ft_rt_id 6
#define sens_car_down_lt_id 7
#define sens_car_up_lt_id 8
#define sens_ft_lf_id 9
#define Sensor_wheel_ft_lt_id 10


// extforce

#define F_wheel_rr_id 1
#define F_wheel_ft_rt_id 2
#define F_wheel_ft_lt_id 3


// links

#define shock_ft_rt_id 1
#define shock_ft_lt_id 2
#define shock_rr_id 3


// ============================================================ //

# endif

/*! 
 * \author Tims seb
 * \file my_DrivenJoints.h
 * \brief header of my_DrivenJoints.c
 */

#include "set_output.h"
#include "user_all_id.h"
#include "mbs_project_interface.h"
#include "mbs_load_xml.h"
#include "mbs_equil.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "mbs_data.h"
#include "mbs_dirdyn.h"
#include "mbs_part.h"
#include "realtime.h"
#include "mbs_set.h"
#include "mbs_load_xml.h"
#include "cmake_config.h"
#include "user_IO.h"

#ifndef _MY_DRIVENJOINTS_H_
#define _MY_DRIVENJOINTS_H_


void my_DrivenJoints_EnCourbe(MbsData *mbs_data, double tsim);	
double my_DrivenJoints_controleur_y(MbsData *mbs_data, double tsim, double t_start);
void my_DrivenJoints_LigneDroite(MbsData *mbs_data, double tsim);	
double my_DrivenJoints_Steering_smooth_consigne(MbsData *mbs_data, double tsim, double t_f, double t_c, double a_c, double t_start, double q_i, double q_f);
double my_DrivenJoints_Steering_smooth_consigne_vit(MbsData *mbs_data, double tsim, double t_f, double t_c, double a_c, double t_start, double q_i, double q_f);
double my_DrivenJoints_double_bande(MbsData *mbs_data, double tsim, double t_start);
double my_DrivenJoints_changement_bande_simple(MbsData *mbs_data, double tsim, double t_start, int signe, double t_mid);


#endif

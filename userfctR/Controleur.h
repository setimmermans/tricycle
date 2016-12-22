/*! 
 * \author Tims seb
 * \file Controleur.h
 * \brief header of Controleur.c
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

#ifndef _CONTROLEUR_H_
#define _CONTROLEUR_H_


double my_controleur(MbsData *mbs_data, double tsim, double speed, double steering);
double tilt_reference(MbsData *mbs_data, double tsim, double speed, double steering);
double my_controleur_stc(MbsData *mbs_data, double tsim, double speed, double steering);
void Print_q_qd_qdd_Qq(MbsData *mbs_data);
double EntreEnCourbe_STC_tilt_ref(MbsData *mbs_data, double tsim, double the_tilt);
double EntreEnCourbe_STC_tilt_ref_vitesse(MbsData *mbs_data, double tsim);
double EntreEnCourbe_DTC_tilt_ref_vitesse(MbsData *mbs_data, double tsim);
#endif

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

#ifndef _CONTROLEUR_H_
#define _CONTROLEUR_H_


double my_controleur(MbsData *mbs_data, double tsim, double speed, double steering);
double tilt_reference(MbsData *mbs_data, double tsim, double speed, double steering);

#endif

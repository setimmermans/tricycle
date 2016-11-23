/*! 
 * \author Seb tims
 * \file QuasiEquilibrium.h
 * \brief header of QuasiEquilibrium.c
 */

#include <stdio.h>
#include "mbs_part.h"
#include "mbs_load_xml.h"
#include "mbs_equil.h"
#include "mbs_modal.h"
#include "mbs_dirdyn.h"
#include "mbs_data.h"
#include "realtime.h"
#include "cmake_config.h"
#include "user_all_id.h"
#include "user_model.h"
#include "mbs_set.h"
#include "user_IO.h"
#include "mbs_sensor.h"
#include "mbs_project_interface.h"
#include "mbs_linearipk.h"
#include<stdio.h>
#include "Controleur.h"

#ifndef _QUASIEQUILIBRIUM_H_
#define _QUASIEQUILIBRIUM_H_


void QuasiEquilibrium(MbsData *mbs_data, double V, double front_radius, double rear_radius, int Toprint, double steer);

#endif

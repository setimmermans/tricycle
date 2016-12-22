/*! 
 * \author Seb tims
 * \file ComparaisonDTC_STC.h
 * \brief header of ComparaisonDTC_STC.c
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

#ifndef _COMPARAISONDTC_STC_H_
#define _COMPARAISONDTC_STC_H_


void ComparaisonDTC_STC(MbsData *mbs_data, double *q_saved_dir, double *qd_saved_dir, double *Qq_saved_dir, double *qdd_saved_dir, double V, double simu_t, int my_mode);

#endif

/*! 
 * \author Tims seb
 * \file Equil_eq.h
 * \brief header of Equil_eq.c
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

#ifndef _EQUIL_EQ_H_
#define _EQUIL_EQ_H_


void my_user_equil_fxe(MbsData *mbs_data, double* f);	

#endif

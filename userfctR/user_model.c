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
 * Last update : Fri Oct 14 16:28:14 2016
 * --------------------------------------------------------
 *
 */
#include "mbs_path.h"
#include "user_model.h"
#include "mbs_xml_reader.h"
#include "mbs_load_xml.h"
#include "useful_functions.h"

// ============================================================ //


UserModel* mbs_new_user_model() 
{
    UserModel* um;
    um = (UserModel*)malloc(sizeof(UserModel));
    um->wheel_ft.K_tire = 0.0;
    um->wheel_ft.D_tire = 0.0;
    um->wheel_ft.r_n_tire = 0.0;
    um->wheel_ft.r_t_tire = 0.0;
 
    um->wheel_rr.K_tire = 0.0;
    um->wheel_rr.D_tire = 0.0;
    um->wheel_rr.r_n_tire = 0.0;
    um->wheel_rr.r_t_tire = 0.0;
 
    um->shock_ft.K = 0.0;
    um->shock_ft.D = 0.0;
    um->shock_ft.Z_0 = 0.0;
    um->shock_ft.Z_min = 0.0;
    um->shock_ft.Z_max = 0.0;
    um->shock_ft.K_stop = 0.0;
 
    um->shock_rr.K = 0.0;
    um->shock_rr.D = 0.0;
    um->shock_rr.Z_0 = 0.0;
    um->shock_rr.Z_min = 0.0;
    um->shock_rr.Z_max = 0.0;
    um->shock_rr.K_stop = 0.0;
 
    um->tilting_ctrl.K = 0.0;
    um->tilting_ctrl.Ki = 0.0;
    um->tilting_ctrl.Kd = 0.0;
 
    um->steering_box.K = 0.0;
 
    um->BGC.carrier_ft_lt_to_sterring_rod_ft_lt_width = 0.0;
 
    um->pincage.pin = get_dvec_1(4);
    um->pincage.pin[0] = 4;
 
    um->lqi.e = get_ivec_1(2);
    um->lqi.e[0] = 2;
 
    um->energy.energyE = get_ivec_1(2);
    um->energy.energyE[0] = 2;
    um->energy.energyM = get_ivec_1(2);
    um->energy.energyM[0] = 2;
 
    return um;
}

void mbs_delete_user_model(UserModel* um) 
{
    free_dvec_0(um->pincage.pin);
    free_ivec_0(um->lqi.e);
    free_ivec_0(um->energy.energyE);
    free_ivec_0(um->energy.energyM);
    free(um);
}

 void mbs_load_user_model_xml(MDS_gen_strct* gen, UserModel* um) 
{

    int ind;
    int ind_state_value = 1;

    um->wheel_ft.K_tire = gen->user_models->user_model_list[0]->parameter_list[0]->value_list[0];
    um->wheel_ft.D_tire = gen->user_models->user_model_list[0]->parameter_list[1]->value_list[0];
    um->wheel_ft.r_n_tire = gen->user_models->user_model_list[0]->parameter_list[2]->value_list[0];
    um->wheel_ft.r_t_tire = gen->user_models->user_model_list[0]->parameter_list[3]->value_list[0];
 
    um->wheel_rr.K_tire = gen->user_models->user_model_list[1]->parameter_list[0]->value_list[0];
    um->wheel_rr.D_tire = gen->user_models->user_model_list[1]->parameter_list[1]->value_list[0];
    um->wheel_rr.r_n_tire = gen->user_models->user_model_list[1]->parameter_list[2]->value_list[0];
    um->wheel_rr.r_t_tire = gen->user_models->user_model_list[1]->parameter_list[3]->value_list[0];
 
    um->shock_ft.K = gen->user_models->user_model_list[2]->parameter_list[0]->value_list[0];
    um->shock_ft.D = gen->user_models->user_model_list[2]->parameter_list[1]->value_list[0];
    um->shock_ft.Z_0 = gen->user_models->user_model_list[2]->parameter_list[2]->value_list[0];
    um->shock_ft.Z_min = gen->user_models->user_model_list[2]->parameter_list[3]->value_list[0];
    um->shock_ft.Z_max = gen->user_models->user_model_list[2]->parameter_list[4]->value_list[0];
    um->shock_ft.K_stop = gen->user_models->user_model_list[2]->parameter_list[5]->value_list[0];
 
    um->shock_rr.K = gen->user_models->user_model_list[3]->parameter_list[0]->value_list[0];
    um->shock_rr.D = gen->user_models->user_model_list[3]->parameter_list[1]->value_list[0];
    um->shock_rr.Z_0 = gen->user_models->user_model_list[3]->parameter_list[2]->value_list[0];
    um->shock_rr.Z_min = gen->user_models->user_model_list[3]->parameter_list[3]->value_list[0];
    um->shock_rr.Z_max = gen->user_models->user_model_list[3]->parameter_list[4]->value_list[0];
    um->shock_rr.K_stop = gen->user_models->user_model_list[3]->parameter_list[5]->value_list[0];
 
    um->tilting_ctrl.K = gen->user_models->user_model_list[4]->parameter_list[0]->value_list[0];
    um->tilting_ctrl.Ki = gen->user_models->user_model_list[4]->parameter_list[1]->value_list[0];
    um->tilting_ctrl.Kd = gen->user_models->user_model_list[4]->parameter_list[2]->value_list[0];
 
    um->steering_box.K = gen->user_models->user_model_list[5]->parameter_list[0]->value_list[0];
 
    um->BGC.carrier_ft_lt_to_sterring_rod_ft_lt_width = gen->user_models->user_model_list[6]->parameter_list[0]->value_list[0];
 
    for(ind=0; ind<gen->user_models->user_model_list[7]->parameter_list[0]->n_value; ind++)
    {
        um->pincage.pin[ind] = gen->user_models->user_model_list[7]->parameter_list[0]->value_list[ind];
    }
 
    for(ind=0; ind<gen->user_models->user_model_list[8]->parameter_list[0]->n_value; ind++)
    {
        um->lqi.e[ind+1] = ind_state_value;
        ind_state_value++;
    }
 
    for(ind=0; ind<gen->user_models->user_model_list[9]->parameter_list[0]->n_value; ind++)
    {
        um->energy.energyE[ind+1] = ind_state_value;
        ind_state_value++;
    }
    for(ind=0; ind<gen->user_models->user_model_list[9]->parameter_list[1]->n_value; ind++)
    {
        um->energy.energyM[ind+1] = ind_state_value;
        ind_state_value++;
    }
 
}

// ============================================================ //
 

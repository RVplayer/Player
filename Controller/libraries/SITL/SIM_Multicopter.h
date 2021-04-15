/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  multicopter simulator class
*/

#pragma once

#include "SIM_Aircraft.h"
#include "SIM_Motor.h"
#include "SIM_Frame.h"
#include "fileOperation.h"
// #include "../ArduCopter/Copter.h"
#define RERUN_SIMQUAD 0
#define RERUN_SOLO 1
#define RERUN_SIM_FRAME RERUN_SOLO

namespace SITL {

/*
  a multicopter simulator
 */
class MultiCopter : public Aircraft {
public:
    MultiCopter(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new MultiCopter(frame_str);
    }

    void add_disturb_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel);
    bool get_pos_based_disturb(float lin_dist[3], float rot_dist[3], float pos_x, float pos_y, float pos_z) const;
    void add_model_disturbance(uint64_t time_from_armed);
    void sync_model(uint64_t time_from_armed);
    void replace_model_states(uint64_t time_from_armed);

protected:
    // calculate rotational and linear accelerations
    void calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel);
    Frame *frame;
    std::vector<std::vector<double>> disturb_data_lin;
    std::vector<std::vector<double>> disturb_data_rot;
    std::vector<std::vector<double>> disturb_data_arr[2];
    std::vector<std::vector<double>> sync_data;
    std::vector<std::vector<double>> config_data;
    bool is_origin_model = true;
    bool is_add_disturb = false;
    uint64_t arm_time;
    bool armed = false;
    bool is_last_origin = true;
    bool is_pos_disturb = false; //position based disturbance
    bool is_log_SimStates = false;
    bool is_replace_euler = false;
    bool is_replace_gyro = false;
    float replace_start = 0;
    bool is_sync_states = false;
    float  sync_end = -1;
private:
    //parameters
    float x[12] = {0}; //in ENU Frame
    float x_NED[12] = {};
    float dx[12] = {0};
    float dx_NED[12] = {0};
    float y_out[12] = {0};
    float u[4] = {0};
    //========for states log============
    float state_log_dt_us = 1.0F / 400.0F * 1e6;
    float last_log_time_us = 0;
    std::fstream states_output;
    //==================================

#if RERUN_SIM_FRAME == RERUN_SIMQUAD
    //=========SimQuad Parameters===========
    float a = 0.128364;
    float b = 0.128364;
    float c = 0.128364;
    float d = 0.128364;
    float m = 1.5;
    float I_x = 0.015;
    float I_y = 0.015;
    float I_z = 0.015;
    float K_T = 7.21077; 
    float K_Q = 0.10472; 
#elif RERUN_SIM_FRAME == RERUN_SOLO
    //===========Solo Parameters============
    float a = 0.15;
    float b = 0.15;
    float c = 0.15;
    float d = 0.15;
    float m = 1.5;
    float I_x = 0.037;
    float I_y = 0.038;
    float I_z = 0.04;
    float K_T = 8.35; 
    float K_Q = 0.3187; 
#endif
    void new_model_step(const struct sitl_input &input);
    void state_sycn_origin2new();
    void state_sycn_new2origin();
};

}

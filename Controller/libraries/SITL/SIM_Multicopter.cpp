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

#include "SIM_Multicopter.h"
#include <AP_Motors/AP_Motors.h>
#include <AP_LogCompression/AP_LogCompression.h>

#include <stdio.h>
// #define USE_SCYN_SIM 1
using namespace SITL;

MultiCopter::MultiCopter(const char *frame_str) :
    Aircraft(frame_str),
    frame(nullptr)
{
    mass = 1.5f;

    frame = Frame::find_frame(frame_str);
    if (frame == nullptr) {
        printf("Frame '%s' not found", frame_str);
        exit(1);
    }
    // initial mass is passed through to Frame for it to calculate a
    // hover thrust requirement.
    if (strstr(frame_str, "-fast")) {
        frame->init(gross_mass(), 0.5, 85, 4*radians(360));
    } else {
        frame->init(gross_mass(), 0.51, 15, 4*radians(360));
    }
    frame_height = 0.1;
    ground_behavior = GROUND_BEHAVIOR_NO_MOVEMENT;

    std::string config_filePath = "/home/bob/ardupilot/libraries/SITL/sim_rerun/config.csv";
    readCSV(config_filePath, config_data);
    if(config_data.size() > 0){
        printf("Config data: ");
        for(double cfg: config_data[0]){
            printf("%d, ", (int)cfg);
        }
        printf("\n");
        is_origin_model = (int)config_data[0][0] == 1;
        is_add_disturb = (int)config_data[0][1] == 1;
        if(is_add_disturb){
            is_pos_disturb = (int)config_data[0][2] == 1;
        }
        is_log_SimStates = (int)config_data[0][3] == 1;
        is_replace_euler = (int)config_data[0][4] == 1;
        is_replace_gyro = (int)config_data[0][5] == 1;
        replace_start = (float)config_data[0][6];
        is_sync_states = (int)config_data[0][7] == 1;
        sync_end = (float)config_data[0][8];
    }

    std::string fileNo ="159"; // "00000284";
    std::string data_folder = "/home/bob/ardupilot/libraries/SITL/sim_rerun/MultiCopter/";
    
    std::string lin_disturb_filePath = data_folder + fileNo + "_disturb_lin.csv";
    std::string rot_disturb_filePath = data_folder + fileNo + "_disturb_rot.csv";

    if(is_add_disturb){
        readCSV(lin_disturb_filePath, disturb_data_lin);
        readCSV(rot_disturb_filePath, disturb_data_rot);
        disturb_data_arr[0] = disturb_data_lin;
        disturb_data_arr[1] = disturb_data_rot;
        if(disturb_data_lin.size() > 0)
            printf("linear disturb data lines: (%d, %d), %f\n", (int)disturb_data_lin.size(), (int) disturb_data_lin[0].size(), disturb_data_lin[0][0]);
        if(disturb_data_rot.size() > 0)
            printf("rotation disturb data lines: (%d, %d), %f\n", (int)disturb_data_rot.size(), (int) disturb_data_rot[0].size(), disturb_data_rot[0][0]);
        if(is_pos_disturb && (int) disturb_data_lin[0].size() < 5){
            is_pos_disturb = false;
        }
    }

    if(is_add_disturb || is_replace_gyro || is_replace_euler){
        std::string syn_filePath = data_folder + fileNo + "_syn.csv";
        readCSV(syn_filePath, sync_data);
        if(sync_data.size() > 0)
            printf("sycn data lines: (%d, %d), %f\n", (int)sync_data.size(), (int) sync_data[0].size(), sync_data[0][0]);
    }
    

    if(!is_origin_model){
        x[5] = M_PI/2; // in ENU frame
    }
    printf("is position based disturbance? %d\n", is_pos_disturb);

    if(is_log_SimStates){
        std::string log_folder = "/home/bob/ardupilot/libraries/SITL/sim_rerun/MultiCopter/log/";
        states_output.open(log_folder + "_SimData.csv",std::fstream::out);
    }
}

void MultiCopter::add_disturb_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel){
    if(!armed){
        return;
    }
    uint64_t time_from_armed = time_now_us - arm_time;
    static uint idxs_dis[2] = {0};
    for (int i = 0; i < 2; i++)
    {
        if(idxs_dis[i] < disturb_data_arr[i].size() && time_from_armed > (uint64_t) disturb_data_arr[i][idxs_dis[i]][0]){
            while((uint64_t) disturb_data_arr[i][idxs_dis[i]][0] < time_from_armed){
                idxs_dis[i]++;
                if(idxs_dis[i] >= disturb_data_arr[i].size()){
                    break;
                }
            }
            if(idxs_dis[i] < disturb_data_arr[i].size()){
                idxs_dis[i]--;// find the closest disturbance time less than time_now_us
                //data in file is in NED frame, apply according to ENU frame
                if(i == 0){
                    Vector3f lin_accel_ef = Vector3f((float)disturb_data_arr[i][idxs_dis[i]][1], 
                                                    (float)disturb_data_arr[i][idxs_dis[i]][2], 
                                                    (float)disturb_data_arr[i][idxs_dis[i]][3]);
                    body_accel += get_dcm().transposed() * lin_accel_ef;
                }else{
                    rot_accel.x += (float)disturb_data_arr[i][idxs_dis[i]][1];
                    rot_accel.y += (float)disturb_data_arr[i][idxs_dis[i]][2];
                    rot_accel.z += (float)disturb_data_arr[i][idxs_dis[i]][3];
                }
                
            }
        }
    }
}

// calculate rotational and linear accelerations
void MultiCopter::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel)
{
    frame->calculate_forces(*this, input, rot_accel, body_accel);
    if(is_add_disturb){
        add_disturb_forces(input, rot_accel, body_accel);
    }
    add_shove_forces(rot_accel, body_accel);
    add_twist_forces(rot_accel);
}

void MultiCopter::state_sycn_origin2new(){
    float r, p, y;
    dcm.to_euler(&r, &p, &y);

    x_NED[0] = position.x;  
    x_NED[1] = position.y;  
    x_NED[2] = position.z;
    x_NED[3] = r;           
    x_NED[4] = p;           
    x_NED[5] = y;
    x_NED[6] = velocity_ef.x;
    x_NED[7] = velocity_ef.y;
    x_NED[8] = velocity_ef.z;
    x_NED[9] = gyro.x;
    x_NED[10] = gyro.y;
    x_NED[11] = gyro.z;

    AP_LOGC::transfromNED2ENU(x_NED, x);
    for(int i = 0; i < 12; i++){
        dx[i] = 0;
    }
}

void MultiCopter::state_sycn_new2origin(){
    AP_LOGC::transfromENU2NED(x, x_NED);
    AP_LOGC::transfromENU2NED(dx, dx_NED);

    if(is_log_SimStates){
        uint64_t time_from_arm = time_now_us - arm_time;
        if(time_from_arm - last_log_time_us > state_log_dt_us){
            writeCSV(states_output, time_from_arm, x_NED, 12);
            last_log_time_us = time_from_arm;
        }
    }

    position.x = x_NED[0];
    position.y = x_NED[1];
    position.z = x_NED[2];
    dcm.from_euler(x_NED[3], x_NED[4], x_NED[5]);
    dcm.normalize();
    velocity_ef.x = x_NED[6];
    velocity_ef.y = x_NED[7];
    velocity_ef.z = x_NED[8];
    gyro.x = x_NED[9];
    gyro.y = x_NED[10];
    gyro.z = x_NED[11];

    gyro_prev = gyro;

    ang_accel.x = dx_NED[9];
    ang_accel.y = dx_NED[10];
    ang_accel.z = dx_NED[11];

    Vector3f accel_earth(dx_NED[6], dx_NED[7], dx_NED[8]);

    // work out acceleration as seen by the accelerometers. It sees the kinematic
    // acceleration (ie. real movement), plus gravity
    accel_body = dcm.transposed() * (accel_earth + Vector3f(0.0f, 0.0f, -GRAVITY_MSS));

    // velocity relative to air mass, in earth frame
    velocity_air_ef = velocity_ef + wind_ef;

    // velocity relative to airmass in body frame
    velocity_air_bf = dcm.transposed() * velocity_air_ef;

    // airspeed
    airspeed = velocity_air_ef.length();

    // airspeed as seen by a fwd pitot tube (limited to 120m/s)
    airspeed_pitot = constrain_float(velocity_air_bf * Vector3f(1.0f, 0.0f, 0.0f), 0.0f, 120.0f);


}

bool MultiCopter::get_pos_based_disturb(float lin_dist[3], float rot_dist[3], float pos_x, float pos_y, float pos_z) const {
    float pre_cut_range = 3;
    for (int i = 0; i < 2; i++){
        float nearest_square_dist = 1000000;
        float nearest_disturb[3] = {0};
        for(std::vector<double> row_data : disturb_data_arr[i]){
            if( abs((float)row_data[4] - pos_x) <= pre_cut_range &&
                abs((float)row_data[5] - pos_y) <= pre_cut_range &&
                abs((float)row_data[6] - pos_z) <= pre_cut_range){
                float x_dist, y_dist, z_dist;
                x_dist = ((float)row_data[4] - pos_x);
                y_dist = ((float)row_data[5] - pos_y);
                z_dist = ((float)row_data[6] - pos_z);
                float square_dist =  x_dist * x_dist + y_dist * y_dist + z_dist * z_dist;
                if(square_dist < nearest_square_dist){
                    nearest_square_dist = square_dist;
                    nearest_disturb[0] = row_data[1];
                    nearest_disturb[1] = row_data[2];
                    nearest_disturb[2] = row_data[3];
                }
                                   
            }
        }
        if((int)nearest_square_dist == 1000000){
            printf("No Disturbance data found in range %f meters\n", pre_cut_range);
            return false;
        }
        if(i == 0){
            lin_dist[0] = nearest_disturb[0];
            lin_dist[1] = nearest_disturb[1];
            lin_dist[2] = nearest_disturb[2];
        }else{
            rot_dist[0] = nearest_disturb[0];
            rot_dist[1] = nearest_disturb[1];
            rot_dist[2] = nearest_disturb[2];
        }
    }
    return true;

}

void MultiCopter::new_model_step(const struct sitl_input &input){
    if(is_last_origin){
        state_sycn_origin2new();
        is_last_origin = false;
    }

    //prepare dt, unit: s
    const float dt = frame_time_us * 1.0e-6f;
    
    //prepare input
    u[0] = AP_LOGC::transformInput(input.servos[0]);
    u[1] = AP_LOGC::transformInput(input.servos[1]);
    u[2] = AP_LOGC::transformInput(input.servos[2]);
    u[3] = AP_LOGC::transformInput(input.servos[3]);

    //1. calculate dx
    AP_LOGC::quadrotor_m(0.0, x, u, a, b, c, d, m, I_x, I_y, I_z, K_T, K_Q, dx, y_out);
#if RERUN_SIM_FRAME == RERUN_SIMQUAD
    // add static air resistence (wind parameter won't have any effect)
    for (int i = 0; i < 3; i++)
    {
        dx[6 + i] += -x[6 + i] * (GRAVITY_MSS/frame->terminal_velocity);
        dx[9 + i] += -x[9 + i] * radians(400.0) / frame->terminal_rotation_rate;
    }
#endif    
    
    uint64_t time_from_armed = time_now_us - arm_time;
    //2. add disturbance
    add_model_disturbance(time_from_armed);
    
    //3. update old states.
    AP_LOGC::updateState(x, dx, dt); 

//#if USE_SCYN_SIM == 1
    //4. test if we need synchronization. If so, synchronize.
    if(is_sync_states){
        if(sync_end <=0 || time_from_armed < sync_end){
            sync_model(time_from_armed);
        }
    }
    
//#endif

    //5. test if we need states replacement, if so, replace.
    if (is_replace_euler || is_replace_gyro){
        replace_model_states(time_from_armed);
    }
    
    //6. sycn with origin model variables
    state_sycn_new2origin();

}

void MultiCopter::replace_model_states(uint64_t time_from_armed){
    float skip_time = replace_start * 1e6;
    if(time_from_armed < skip_time){
        return;
    }
    static uint idx = 0;
    if(idx < sync_data.size() && time_from_armed >= (uint64_t)sync_data[idx][0] ){
        while(time_from_armed >= (uint64_t)sync_data[idx][0]){
            idx++;
            if(idx >= sync_data.size()){
                return;
            }
        }
        idx--;
        for (size_t i = 0; i < 12; i++)
        {
            x_NED[i] = sync_data[idx][i+1];
        }
        float x_ENU[12] = {0};
        AP_LOGC::transfromNED2ENU(x_NED, x_ENU);
        if(is_replace_euler){
            for (size_t i = 3; i < 6; i++)
            {
                x[i] = x_ENU[i]; // synchronize roll pitch yaw
            }
            if(!(sitl->is_start_eurle_sync)){
                sitl->is_start_eurle_sync = true; //make ekf in controller to sync angles
            }
            //copter.ahrs.get_NavEKF3().updateStatesFromEurle(x_NED[3], x_NED[4], x_NED[5]);
        }

        if(is_replace_gyro){
            for (size_t i = 9; i < 12; i++)
            {
                x[i] = x_ENU[i]; // synchronize roll pitch yaw rate
            }
        }
        
        // for (size_t i = 0; i < 3; i++)
        // {
        //     x[i] = x_ENU[i]; // synchronize x,y,z value
        // }
    }
}

void MultiCopter::sync_model(uint64_t time_from_armed){
    static uint idx = 0;
    static uint next_sync_time_us = 0;
    if(idx < sync_data.size() 
        && time_from_armed >= (uint64_t)sync_data[idx][0] 
            &&time_from_armed >= next_sync_time_us){

        while(time_from_armed >= (uint64_t)sync_data[idx][0]){
            idx++;
            if(idx >= sync_data.size()){
                return;
            }
        }
        idx--;
        for (size_t i = 0; i < 12; i++)
        {
            x_NED[i] = sync_data[idx][i+1];
        }

        float x_ENU[12] = {0};
        AP_LOGC::transfromNED2ENU(x_NED, x_ENU);

        for (size_t i = 0; i < 3; i++)
        {
            x[i] = x_ENU[i]; // synchronize x,y,z value
        }

        // for (size_t i = 3; i < 6; i++)
        // {
        //     x[i] = x_ENU[i]; // synchronize roll pitch yaw
        // }

        // copter.ahrs.get_NavEKF3().updateStatesFromEurle(x_NED[3], x_NED[4], x_NED[5]);

        // for (size_t i = 9; i < 12; i++)
        // {
        //     x[i] = x_ENU[i]; // synchronize roll pitch yaw rate
        // }

        float sync_interval = 1; //Unit: s, min: 0.1s

        next_sync_time_us = time_from_armed + sync_interval * 1e6;
        idx++; //this one has been used, skip

        //idx += (int)(sync_interval * 10); // sync every 1s
    }
}

void MultiCopter::add_model_disturbance(uint64_t time_from_armed){
    if(is_pos_disturb){
        //position based disturbance
        float skip_time = 1e6; //default 3s
        if(time_from_armed > skip_time){
            float lin_dist_temp[3], rot_dist_temp[3];
            bool is_distb_found = get_pos_based_disturb(lin_dist_temp, rot_dist_temp, x[0], x[1], x[2]);
            if(is_distb_found){
                dx[6] +=   lin_dist_temp[1]; // acc_y
                dx[7] +=   lin_dist_temp[0]; // acc_x
                dx[8] += (-lin_dist_temp[2]); // -acc_z
                dx[9] +=    rot_dist_temp[0]; // ang_acc_x
                dx[10] += (-rot_dist_temp[1]); // -ang_acc_y
                dx[11] += (-rot_dist_temp[2]); // -ang_acc_z
            }
        }
    }else{
        //Time based disturbance
        static uint idxs_dis[2] = {0};
        for (int i = 0; i < 2; i++)
        {
            if(idxs_dis[i] < disturb_data_arr[i].size() && time_from_armed > (uint64_t) disturb_data_arr[i][idxs_dis[i]][0]){
                while((uint64_t) disturb_data_arr[i][idxs_dis[i]][0] < time_from_armed){
                    idxs_dis[i]++;
                    if(idxs_dis[i] >= disturb_data_arr[i].size()){
                        break;
                    }
                }
                if(idxs_dis[i] < disturb_data_arr[i].size()){
                    idxs_dis[i]--;// find the closest disturbance time less than time_now_us
                    //data in file is in NED frame, apply according to ENU frame
                    if(i == 0){
                        dx[6] +=   disturb_data_arr[i][idxs_dis[i]][2]; // acc_y
                        dx[7] +=   disturb_data_arr[i][idxs_dis[i]][1]; // acc_x
                        dx[8] += (-disturb_data_arr[i][idxs_dis[i]][3]); // -acc_z
                    }else{
                        dx[9] +=    disturb_data_arr[i][idxs_dis[i]][1]; // ang_acc_x
                        dx[10] += (-disturb_data_arr[i][idxs_dis[i]][2]); // -ang_acc_y
                        dx[11] += (-disturb_data_arr[i][idxs_dis[i]][3]); // -ang_acc_z
                    }
                    
                }
            }
        }
    }
}
    
/*
  update the multicopter simulation by one time step
 */
void MultiCopter::update(const struct sitl_input &input)
{
    if(!armed && input.servos[0] >= 1300){
        armed = true;
        arm_time = time_now_us;
    }
    if(on_ground()|| !armed || is_origin_model){
        is_last_origin = true;

        // get wind vector setup
        update_wind(input);

        Vector3f rot_accel;

        calculate_forces(input, rot_accel, accel_body);

        // estimate voltage and current
        frame->current_and_voltage(input, battery_voltage, battery_current);

        update_dynamics(rot_accel);
        update_external_payload(input);

        // update lat/lon/altitude
        update_position();
        time_advance();

        // update magnetic field
        update_mag_field_bf();
    }else{
        use_smoothing = false;

        // get wind vector setup
        update_wind(input);

        frame->current_and_voltage(input, battery_voltage, battery_current);

        new_model_step(input); // main model code

        update_external_payload(input);

        // update lat/lon/altitude
        update_position();
        time_advance();

        // update magnetic field
        update_mag_field_bf();
        
    }
    
}

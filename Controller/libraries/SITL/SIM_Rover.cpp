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
  rover simulator class
*/

#include "SIM_Rover.h"

#include <string.h>
#include <stdio.h>
#include "fileOperation.h"
#include <AP_LogCompression/AP_LogCompression.h>
#include <sys/stat.h>

namespace SITL {

SimRover::SimRover(const char *frame_str) :
    Aircraft(frame_str),
    max_speed(20),
    max_accel(10),
    max_wheel_turn(35),
    turning_circle(1.8),
    skid_turn_rate(140), // degrees/sec
    skid_steering(false)
{
    skid_steering = strstr(frame_str, "skid") != nullptr;

    if (skid_steering) {
        printf("SKID Steering Rover Simulation Started\n");
        // these are taken from a 6V wild thumper with skid steering,
        // with a sabertooth controller
        max_accel = 14;
        max_speed = 4;
    }

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

    printf("frame time in us: %d\n", (int)frame_time_us);
    
    if(is_add_disturb){
        std::string fileNo = "00000166";
#if RERUN_SIM_FRAME == RERUN_SIMROVER        
        std::string data_folder = "/home/bob/ardupilot/libraries/SITL/sim_rerun/Rover/";
#elif RERUN_SIM_FRAME == RERUN_ERLEROVER
        std::string data_folder = "/home/bob/ardupilot/libraries/SITL/sim_rerun/Rover/Erle/";
#endif
        std::string lin_disturb_filePath = data_folder + fileNo + "_disturb_lin.csv";
        std::string rot_disturb_filePath = data_folder + fileNo + "_disturb_rot.csv";
        std::string syn_filePath = data_folder + fileNo + "_syn.csv";
        readCSV(lin_disturb_filePath, disturb_data_lin);
        readCSV(rot_disturb_filePath, disturb_data_rot);
        disturb_data_arr[0] = disturb_data_lin;
        disturb_data_arr[1] = disturb_data_rot;

        readCSV(syn_filePath, sync_data);
        if(disturb_data_lin.size() > 0)
            printf("linear disturb data lines: (%d, %d), %f\n", (int)disturb_data_lin.size(), (int) disturb_data_lin[0].size(), disturb_data_lin[0][0]);
        if(disturb_data_rot.size() > 0)
            printf("rotation disturb data lines: (%d, %d), %f\n", (int)disturb_data_rot.size(), (int) disturb_data_rot[0].size(), disturb_data_rot[0][0]);
        if(sync_data.size() > 0)
            printf("sycn data lines: (%d, %d), %f\n", (int)sync_data.size(), (int) sync_data[0].size(), sync_data[0][0]);
    }

    
    // std::string curr_time_str = getTimeStr();
    std::string log_folder = "/home/bob/ardupilot/libraries/SITL/sim_rerun/Rover/log/";
    // mkdir(log_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    // std::string filename_data = log_folder + "/data.csv";
    // printf("%s\n", filename_data.c_str());
    data_output.open(log_folder + "data.csv",std::fstream::out);
    info_output.open(log_folder + "info.txt", std::fstream::out);
}



/*
  return turning circle (diameter) in meters for steering angle proportion in degrees
*/
float SimRover::turn_circle(float steering)
{
    if (fabsf(steering) < 1.0e-6) {
        return 0;
    }
    return turning_circle * sinf(radians(max_wheel_turn)) / sinf(radians(steering*max_wheel_turn));
}

/*
   return yaw rate in degrees/second given steering_angle and speed
*/
float SimRover::calc_yaw_rate(float steering, float speed)
{
    if (skid_steering) {
        return steering * skid_turn_rate;
    }
    if (fabsf(steering) < 1.0e-6 or fabsf(speed) < 1.0e-6) {
        return 0;
    }
    float d = turn_circle(steering);
    float c = M_PI * d;
    float t = c / speed;
    float rate = 360.0f / t;
    return rate;
}

/*
  return lateral acceleration in m/s/s
*/
float SimRover::calc_lat_accel(float steering_angle, float speed)
{
    float yaw_rate = calc_yaw_rate(steering_angle, speed);
    float accel = radians(yaw_rate) * speed;
    return accel;
}

/*
  update the rover simulation by one time step
 */
void SimRover::update(const struct sitl_input &input)
{

    if(!armed && abs(input.servos[2] - 1500) >= arm_ref){
        armed = true;
        arm_time = time_now_us;
    }

    if(!armed || is_origin_model){
        is_last_origin = true;
        float steering, throttle;
        // if in skid steering mode the steering and throttle values are used for motor1 and motor2
        if (skid_steering) {
            float motor1 = 2*((input.servos[0]-1000)/1000.0f - 0.5f);
            float motor2 = 2*((input.servos[2]-1000)/1000.0f - 0.5f);
            steering = motor1 - motor2;
            throttle = 0.5*(motor1 + motor2);
            printf("skid steering mode!!");
        } else {
            steering = 2*((input.servos[0]-1000)/1000.0f - 0.5f);
            throttle = 2*((input.servos[2]-1000)/1000.0f - 0.5f);
        }

        // how much time has passed?
        float delta_time = frame_time_us * 1.0e-6f;
        
        // speed in m/s in body frame
        Vector3f velocity_body = dcm.transposed() * velocity_ef;

        // speed along x axis, +ve is forward
        float speed = velocity_body.x;

        // yaw rate in degrees/s
        float yaw_rate = calc_yaw_rate(steering, speed);

        // target speed with current throttle
        float target_speed = throttle * max_speed;

        // linear acceleration in m/s/s - very crude model
        float accel = max_accel * (target_speed - speed) / max_speed;

        //============== Add wind effect for rover ===============
        if(armed){
            Vector3f air_resistance = -velocity_air_ef * (GRAVITY_MSS/15);
            Vector3f air_resis_bf = get_dcm().transposed() * air_resistance;
            accel += air_resis_bf.x;
        }
        //========================================================

        gyro = Vector3f(0,0,radians(yaw_rate));

        // update attitude
        dcm.rotate(gyro * delta_time);
        dcm.normalize();

        // accel in body frame due to motor
        accel_body = Vector3f(accel, 0, 0);

        // add in accel due to direction change
        accel_body.y += radians(yaw_rate) * speed;

        // now in earth frame
        Vector3f accel_earth = dcm * accel_body;
        accel_earth += Vector3f(0, 0, GRAVITY_MSS);

        // we are on the ground, so our vertical accel is zero
        accel_earth.z = 0;

        // work out acceleration as seen by the accelerometers. It sees the kinematic
        // acceleration (ie. real movement), plus gravity
        accel_body = dcm.transposed() * (accel_earth + Vector3f(0, 0, -GRAVITY_MSS));

        // new velocity vector
        velocity_ef += accel_earth * delta_time;

        // new position vector
        position += velocity_ef * delta_time;

        //============== Add wind effect for rover ===============
        update_wind(input);

        // velocity relative to air mass, in earth frame
        velocity_air_ef = velocity_ef + wind_ef;

        // velocity relative to airmass in body frame
        velocity_air_bf = dcm.transposed() * velocity_air_ef;

        // airspeed
        airspeed = velocity_air_ef.length();
        //========================================================

    }
    else{
        use_smoothing = true;

        // get wind vector setup
        update_wind(input);

        new_model_step(input);
    }

    
    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

void SimRover::new_model_step(const struct sitl_input &input){
    if(is_last_origin){
        state_sycn_origin2new();
        is_last_origin = false;
    }
    float steering=0, throttle=0;
    // if in skid steering mode the steering and throttle values are used for motor1 and motor2
    if (skid_steering) {
        printf("doesn't support skid steering\n");
    } else {
        steering = max_turn_ang * ((input.servos[0]-1000)/1000.0f - 0.5f);
        throttle = ((input.servos[2]-1000)/1000.0f - 0.5f);
    }

    //prepare input
    u[0] = steering;
    u[1] = 0;//front wheel
    u[2] = throttle; //rear wheel

    // how much time has passed?
    float dt = frame_time_us * 1.0e-6f;

    //1. calculate dx
    if(std::abs(x[3]) < 0.1){
        printf("======warning: longtitorial speed too slow\n");
        x[3] = x[3] > 0 ? 0.1f : -0.1f;
    }
    AP_LOGC::rover_m(0, x, u, m, a, b, Cx, Cy, CA, dx, y_out);
#if RERUN_SIM_FRAME == RERUN_SIMROVER
    // add static air resistence (wind parameter won't have any effect)
    dx[3] += -x[3] * 9.80665F / 15.0F;
#endif

    

    //2. add disturbance
    uint64_t time_from_armed = time_now_us - arm_time;

    static uint idxs_dis[2] = {0};
    for (int i = 0; i < 2; i++)
    {
        if(time_from_armed > 3 * 1e6 && 
                idxs_dis[i] < disturb_data_arr[i].size() && 
                    time_from_armed > (uint64_t) disturb_data_arr[i][idxs_dis[i]][0]){
            while((uint64_t) disturb_data_arr[i][idxs_dis[i]][0] < time_from_armed){
                idxs_dis[i]++;
                if(idxs_dis[i] >= disturb_data_arr[i].size()){
                    break;
                }
            }
            if(idxs_dis[i] < disturb_data_arr[i].size()){
                idxs_dis[i]--;
                if(i == 0){
                    // writeInfo(info_output, time_now_us, std::to_string(disturb_data_arr[i][idxs_dis[i]][0]));
                    //disturbance data is in NED frame, rover use NED frame for model
                    dx[3] +=  disturb_data_arr[i][idxs_dis[i]][1]; // acc_x
                    dx[4] +=   disturb_data_arr[i][idxs_dis[i]][2]; // acc_y
                }else{
                    //dx[5] +=   disturb_data_arr[i][idxs_dis[i]][1]; // yaw_rate
                }
                
            }
        }
    }

    if(abs(dx[3]) > 10){
        printf("!!!warning: foward acceleration too fast\n");
        writeInfo(info_output, time_now_us, "!!!warning: foward acceleration too fast");
        // dx[3] = dx[3] > 0 ? 4 : -4;
    }

    if(abs(dx[4]) > 10){
        printf("!!!warning: side acceleration too fast\n");
        writeInfo(info_output, time_now_us,  "!!!warning: side acceleration too fast");
        // dx[4] = dx[4] > 0 ? 1.5 : -1.5;
    }

    if(abs(dx[5]) > 10){
        printf("!!!warning: yaw acceleration too fast\n");
        writeInfo(info_output, time_now_us,  "!!!warning: yaw acceleration too fast");
        // dx[5] = dx[5] > 0 ? 4 : -4;
    }

    //3. update old states.
    float x_last[6] = {0};
    for (int i = 0; i < 6; i++)
    {
        x_last[i] = x[i];
        x[i] += dx[i] * dt;
    }
    x[2] = wrap_2PI(x[2]);

    //anormly velocity checks
    float vel_thresh[3] = {4, 4, 1}; //body x, y, yaw velocity
    for(int i = 3; i < 6; i++){
        float thres =  vel_thresh[i-3];
        if(abs(x[i]) > thres){
            printf("warning: states %d exceed threshold\n", (i+1));
            // writeInfo(info_output, time_now_us,  "warning: foward speed too fast");
            x[i] = x[i] > 0 ? thres : -thres;
            dx[i] = (x[i] - x_last[i])/dt;
        }
    }


    //4. test if we need synchronization. If so, synchronize.

    if(is_sync_states){
        static uint idx = 0;

        if(idx < sync_data.size() && time_from_armed >= (uint64_t)sync_data[idx][0] ){
            while(time_from_armed >= (uint64_t)sync_data[idx][0]){
                idx++;
                if(idx >= sync_data.size()){
                    break;
                }
            }
            idx--;
            for (size_t i = 0; i < 6; i++)
            {
                y_out[i] = sync_data[idx][i+1];
            }

            float x_bf[6] = {0};
            AP_LOGC::transfromef2bf_rover(y_out, x_bf);

            for (size_t i = 0; i < 2; i++)
            {
                x[i] = x_bf[i]; //only synchronize x,y, yaw value
            }
            
            float sync_interval = 0.1; //Unit: s, min: 0.1s
            
            idx += (int)(sync_interval * 10); // default: sync every 1s
        }
    }
    

    //5. update output values
    AP_LOGC::transfrombf2ef_rover(x, y_out);

    float combined_data[20];
    memcpy(combined_data, x, 6 * sizeof(float));
    memcpy(&combined_data[6], dx, 6 * sizeof(float));
    memcpy(&combined_data[12], y_out, 6 * sizeof(float));
    combined_data[18] = steering/M_PI;
    combined_data[19] = throttle;

    writeCSV(data_output, time_now_us, combined_data, 20);

    //6. sycn with origin model variables
    state_sycn_new2origin();

}

void SimRover::state_sycn_new2origin(){

    gyro = Vector3f(0, 0, y_out[5]);

    // update attitude
    // dcm.rotate(gyro * frame_time_us * 1.0e-6f);
    dcm.from_euler(0, 0, y_out[2]);
    dcm.normalize();

    accel_body = Vector3f(dx[3], dx[4], 0);
    ang_accel = Vector3f(0, 0, dx[5]);

    // now in earth frame
    // Vector3f accel_earth = dcm * accel_body;
    // float dx_ef[6] = {0};
    // AP_LOGC::transfrombf2ef_rover(dx, dx_ef);
    // Vector3f accel_earth = Vector3f(dx_ef[3], dx_ef[4], 0);
    // accel_earth += Vector3f(0, 0, GRAVITY_MSS);

    // // we are on the ground, so our vertical accel is zero
    // accel_earth.z = 0;

    // work out acceleration as seen by the accelerometers. It sees the kinematic
    // acceleration (ie. real movement), plus gravity
    // accel_body = dcm.transposed() * (accel_earth + Vector3f(0, 0, -GRAVITY_MSS));
    accel_body += Vector3f(0, 0, -GRAVITY_MSS);

    // new velocity vector
    velocity_ef.x = y_out[3];
    velocity_ef.y = y_out[4];
    velocity_ef.z = 0;

    // new position vector
    position.x = y_out[0];
    position.y = y_out[1];
    position.z = 0;

    //Wind effect
    // velocity relative to air mass, in earth frame
    velocity_air_ef = velocity_ef + wind_ef;

    // velocity relative to airmass in body frame
    velocity_air_bf = dcm.transposed() * velocity_air_ef;

    // airspeed
    airspeed = velocity_air_ef.length();

}

void SimRover::state_sycn_origin2new(){
    float r, p, y;
    dcm.to_euler(&r, &p, &y);

    y_out[0] = position.x;
    y_out[1] = position.y;
    y_out[2] = y;
    y_out[3] = velocity_ef.x;
    y_out[4] = velocity_ef.y;
    y_out[5] = gyro.z;

    AP_LOGC::transfromef2bf_rover(y_out, x);
    for(int i = 0; i < 6; i++){
        dx[i] = 0;
    }
}

} // namespace SITL
#pragma once

#include <cstdlib>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>


namespace AP_LOGC{
    void quadrotor_m(float, const float x[12], const float u[4], float a, float
                 b, float c, float d, float m, float I_x, float I_y, float
                 I_z, float K_T, float K_Q, float dx[12], float y[12]);
    void rover_m(float, const float x[6], const float i[3], float m, float
                    a, float b, float Cx, float Cy, float CA, float dx[6], float
                    y[6]);
    void transfromNED2ENU(float x[12]);
    void transfromNED2ENU(float in[12], float out[12]);
    void transfromef2bf_rover(float in[6], float out[6]);
    void transfrombf2ef_rover(float in[6], float out[6]);
    void transfromENU2NED(float in[12], float out[12]);
    void updateState(float x[12], float dx[12], float dt);
    void compressionLog(const struct log_Bob_EKF1 & sensor_pkt, const struct log_motors & motor_pkt);
    bool is_log(float error, float error_max, int last_log_loop, int current_loop, int max_freq);
    float transformInput(uint16_t pwm);
}
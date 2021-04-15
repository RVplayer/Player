#include "AP_LogCompression.h"
#include <AP_Logger/AP_Logger.h>

void AP_LOGC::rover_m(float, const float x[6], const float i[3], float m, float a, float
             b, float Cx, float Cy, float CA, float dx[6], float y[6])
{
  float c;

  // IDNLGREY model file (discrete-time nonlinear model) :
  // xn = x(t+Ts) : state update values in discrete-time case (A column vector with Nx entries) 
  // y : outputs values (A column vector with Ny entries)
  //  NED frame
  // x(1): Postion X
  // x(2): Position Y
  // x(3): Yaw [-pi, pi]
  // x(4): Longitudinal vehicle velocity. (body frame)
  // x(5): Lateral velocity. (body frame)
  // x(6): Yaw rate.
  //      m = p(1);
  //      a = p(2);
  //      b = p(3);
  //      Cx = p(4);
  //      Cy = p(5);
  //      CA = p(6);
  //      %[x y yaw vx vy r]
  dx[0] = x[3] * cosf(x[2]) - x[4] * sinf(x[2]);
  dx[1] = x[3] * sinf(x[2]) + x[4] * cosf(x[2]);
  dx[2] = x[5];
  dx[3] = (x[4] * x[5] + 1.0F / m * (((Cx * (i[1] + i[1]) * cosf(i[0]) -
            2.0F * Cy * (i[0] - atanf((x[4] + a * x[5]) / x[3])) * sinf(i[0])) +
            Cx * (i[2] + i[2])) - CA * (x[3] * x[3])));
  dx[4] = -x[3] * x[5] + 1.0F / m * ((Cx * (i[1] + i[1]) * sinf(i[0]) + 2.0F
    * Cy * (i[0] - atanf((x[4] + a * x[5]) / x[3])) * cosf(i[0])) + 2.0F * Cy * atanf((b * x[5] - x[4]) / x[3]));
  c = (a + b) / 2.0F;
  dx[5] = 1.0F / (c * c * m) * (a * (Cx * (i[1] + i[1]) * sinf(i[0]) + 2.0F *
    Cy * (i[0] - atanf((x[4] + a * x[5]) / x[3])) * cosf(i[0])) - 2.0F * b * Cy *
    atanf((b * x[5] - x[4]) / x[3]));

  
  //  air resistance
  //  *0.6538
//   dx[3] += -x[3] * 9.80665F / 15.0F;

  // (NED world frame)
  // y(1): Postion X
  // y(2): Position Y
  // y(3): Yaw
  // y(4): North velocity.
  // y(5): East velocity.
  // y(6): Yaw rate.
  // y(7): Lateral vehicle acceleration.
  y[0] = x[0];
  y[1] = x[1];
  y[2] = x[2];
  y[3] = x[3] * cosf(x[2]) - x[4] * sinf(x[2]);
  y[4] = x[3] * sinf(x[2]) + x[4] * cosf(x[2]);
  y[5] = x[5];


}

void AP_LOGC::quadrotor_m(float, const float x[12], const float u[4], float a, float b,
                          float c, float d, float m, float I_x, float I_y, float I_z,
                          float K_T, float K_Q, float dx[12], float y[12])
{
    int i;
    float b_x[16];

    // IDNLGREY model file (discrete-time nonlinear model) :
    // xn = x(t+Ts) : state update values in discrete-time case (A column vector with Nx entries)
    // y : outputs values (A column vector with Ny entries)
    //  gravity acceleration constant (m/s^2)
    //  inputs
    //      x(13:16)=u;
    for (i = 0; i < 12; i++)
    {
        b_x[i] = x[i];
    }

    for (i = 0; i < 4; i++)
    {
        b_x[i + 12] = u[i];
    }

    // -------------------------------------------------
    dx[0] = b_x[6];
    dx[1] = b_x[7];
    dx[2] = b_x[8];
    dx[3] = (b_x[9] + sinf(b_x[3]) * tanf(b_x[4]) * b_x[10]) + cosf(b_x[3]) * tanf(b_x[4]) * b_x[11];
    dx[4] = cosf(b_x[3]) * b_x[10] - sinf(b_x[3]) * b_x[11];
    dx[5] = sinf(b_x[3]) / cosf(b_x[4]) * b_x[10] + cosf(b_x[3]) / cosf(b_x[4]) * b_x[11];
    dx[6] = K_T / m * (((b_x[12] + b_x[13]) + b_x[14]) + b_x[15]) * (cosf(b_x[3]) * sinf(b_x[4]) * cosf(b_x[5]) + sinf(b_x[3]) * sinf(b_x[5]));
    dx[7] = K_T / m * (((b_x[12] + b_x[13]) + b_x[14]) + b_x[15]) * (cosf(b_x[3]) * sinf(b_x[4]) * sinf(b_x[5]) - sinf(b_x[3]) * cosf(b_x[5]));
    dx[8] = K_T / m * (((b_x[12] + b_x[13]) + b_x[14]) + b_x[15]) * (cosf(b_x[3]) * cosf(b_x[4])) - 9.80665F;
    dx[9] = (I_y - I_z) / I_x * b_x[10] * b_x[11] + K_T / I_x * (((-a * b_x[12] + d * b_x[13]) + a * b_x[14]) - d * b_x[15]);
    dx[10] = (I_x - I_z) / I_y * b_x[9] * b_x[11] + K_T / I_y * (((-b * b_x[12] + c * b_x[13]) - b * b_x[14]) + c * b_x[15]);
    dx[11] = (I_x - I_y) / I_z * b_x[9] * b_x[10] + K_Q / I_z * (((-b_x[12] - b_x[13]) + b_x[14]) + b_x[15]);

    //
    for (i = 0; i < 12; i++)
    {
        y[i] = x[i];
    }

    //  update outputs
}

void AP_LOGC::transfromNED2ENU(float state[12])
{
    float x[12] = {0};
    x[0] = state[1];
    x[1] = state[0];
    x[2] = -state[2];
    x[3] = state[3];
    x[4] = -state[4];
    x[5] = wrap_2PI(-state[5] + M_PI / 2);
    x[6] = state[7];
    x[7] = state[6];
    x[8] = -state[8];
    x[9] = state[9];
    x[10] = -state[10];
    x[11] = -state[11];
    for (int i = 0; i < 12; i++)
    {
        state[i] = x[i];
    }
}

void AP_LOGC::transfromNED2ENU(float state[12], float x[12])
{
    x[0] = state[1];
    x[1] = state[0];
    x[2] = -state[2];
    x[3] = state[3];
    x[4] = -state[4];
    x[5] = wrap_2PI(-state[5] + M_PI / 2);
    x[6] = state[7];
    x[7] = state[6];
    x[8] = -state[8];
    x[9] = state[9];
    x[10] = -state[10];
    x[11] = -state[11];
}

void AP_LOGC::transfromef2bf_rover(float in[6], float out[6]){
    out[0] = in[0];
    out[1] = in[1];
    out[2] = in[2];
    out[5] = in[5];

    out[3] = cosf(in[2]) * in[3] + sinf(in[2]) * in[4];
    out[4] = -sinf(in[2]) * in[3] + cosf(in[2]) * in[4];
}

void AP_LOGC::transfrombf2ef_rover(float in[6], float out[6]){
    out[0] = in[0];
    out[1] = in[1];
    out[2] = in[2];
    out[5] = in[5];

    out[3] = cosf(in[2]) * in[3] - sinf(in[2]) * in[4];
    out[4] = sinf(in[2]) * in[3] + cosf(in[2]) * in[4];
}

void AP_LOGC::transfromENU2NED(float in[12], float out[12])
{
    out[0] = in[1];
    out[1] = in[0];
    out[2] = -in[2];
    out[3] = in[3];
    out[4] = -in[4];
    out[5] = wrap_2PI(-(in[5] - M_PI / 2));
    out[6] = in[7];
    out[7] = in[6];
    out[8] = -in[8];
    out[9] = in[9];
    out[10] = -in[10];
    out[11] = -in[11];
}

void AP_LOGC::updateState(float x[12], float dx[12], float dt)
{
    for (int i = 0; i < 12; i++)
    {
        x[i] += dx[i] * dt;
    }
    x[5] = wrap_2PI(x[5]);
}

void AP_LOGC::compressionLog(const struct log_Bob_EKF1 &sensor_pkt, const struct log_motors &motor_pkt)
{
    //parameters
    static float x[12] = {0};
    static float true_x[12] = {0};
    static float dx[12] = {0};
    static float y[12] = {0};
    static float u[4] = {0};
    static float a = 0.128364;
    static float b = 0.128364;
    static float c = 0.128364;
    static float d = 0.128364;
    static float m = 1.5;
    static float I_x = 0.015;
    static float I_y = 0.015;
    static float I_z = 0.015;
    static float K_T = 7.21077;
    static float K_Q = 0.10472;
    static uint64_t last_time_us = 0;
    static int K_stone = 400;
    static int loopCount = -1;
    static int max_freq = 400;

    static float error_thre[12] = {0.139337476507485, 0.124396874563097, 0.0147272946765464,
                                   0.00372856443706101, 0.00390555433968539, 0.00589400093019354,
                                   0.0696205367431640, 0.0815225658620695, 0.0264938590124031,
                                   0.00822311394035166, 0.00840920644259765, 0.0101633185357302};
    static int last_log_loop[12] = {0};

    uint64_t time_us = sensor_pkt.time_us;

    //prepare dt, unit: s
    float dt = (float)((double)(time_us - last_time_us) * 1e-6);
    last_time_us = time_us;

    //prepare input
    u[0] = transformInput(motor_pkt.motor1);
    u[1] = transformInput(motor_pkt.motor2);
    u[2] = transformInput(motor_pkt.motor3);
    u[3] = transformInput(motor_pkt.motor4);

    //1. update old states to predict current states.
    AP_LOGC::updateState(x, dx, dt);

    true_x[0] = sensor_pkt.posN;
    true_x[1] = sensor_pkt.posE;
    true_x[2] = sensor_pkt.posD;
    true_x[3] = sensor_pkt.roll;
    true_x[4] = sensor_pkt.pitch;
    true_x[5] = wrap_2PI(sensor_pkt.yaw);
    true_x[6] = sensor_pkt.velN;
    true_x[7] = sensor_pkt.velE;
    true_x[8] = sensor_pkt.velD;
    true_x[9] = sensor_pkt.gyrX;
    true_x[10] = sensor_pkt.gyrY;
    true_x[11] = sensor_pkt.gyrZ;
    AP_LOGC::transfromNED2ENU(true_x);

    //2. test if we need synchronization. If so, synchronize.
    loopCount++; //loopCount change from [0, K_stone)
    if (loopCount % K_stone == 0)
    { //2.1 K-milestone sychronization
        loopCount = 0;
        for (int i = 0; i < 12; i++)
        {
            x[i] = true_x[i];
            last_log_loop[i] = loopCount;
        }
        struct log_Bob_EKF1 log_sycn = sensor_pkt;
        log_sycn.msgid = LOG_CLOG_SYN_MSG;
        if (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
        { //only do actual log in sitl
            AP::logger().WriteCriticalBlock(&log_sycn, sizeof(log_sycn));
        }
    }

    //3. get current output and calculate dx for next time.
    AP_LOGC::quadrotor_m(0.0, x, u, a, b, c, d, m, I_x, I_y, I_z, K_T, K_Q, dx, y);
    //air resistence in static air
    for (int i = 0; i < 3; i++)
    {
        dx[6 + i] += -x[6 + i] * 9.80665F / 15.0F;
        dx[9 + i] += -x[9 + i] * 6.98131704F / 25.1327419F;
    }
    //on ground check
    float frame_height = 0.1;
    if ((x[2] - frame_height < 0.001F) && (dx[8] <= 0.0F))
    {
        //  on ground
        dx[8] = 0.0F;
    }

    //compression log
    for (int i = 0; i < 12; i++)
    {
        float error = abs(true_x[i] - y[i]);
        if (is_log(error, error_thre[i], last_log_loop[i], loopCount, max_freq))
        {
            struct log_clog_gt clog_data = {
                LOG_PACKET_HEADER_INIT(LOG_CLOG_GT_MSG),
                time_us : time_us,
                stateNo : (int8_t)(i + 1),
                value : (float)true_x[i]
            };
            if (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
            { //only do actual log in sitl
                AP::logger().WriteCriticalBlock(&clog_data, sizeof(clog_data));
            }
            last_log_loop[i] = loopCount;
        }
    }
}

bool AP_LOGC::is_log(float error, float error_max, int last_log_loop, int current_loop, int max_freq)
{
    float scale_factor = 1;
    if (error < 1e-20 || last_log_loop == current_loop)
    {
        return false;
    }
    if (error >= error_max)
    {
        return true;
    }
    else
    {
        float desired_freq = scale_factor * (error / error_max) * max_freq;
        float current_freq = max_freq * (1 / (float)(current_loop - last_log_loop));
        if (desired_freq > current_freq)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}

float AP_LOGC::transformInput(uint16_t pwm)
{
    return constrain_float(((float)(pwm - 1100)) / 900.0, 0.0f, 1.0f);
}

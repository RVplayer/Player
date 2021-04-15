#include "Rover.h"

/*****************************************
    Set the flight control servos based on the current calculated values
*****************************************/
void Rover::set_servos(void)
{
    // send output signals to motors
    if (motor_test) {
        motor_test_output();
    } else {
        // get ground speed
        float speed;
        if (!g2.attitude_control.get_forward_speed(speed)) {
            speed = 0.0f;
        }

        g2.motors.output(arming.is_armed(), speed, G_Dt);

        //==========compressed log ==================
        static bool delay_satrt = true;
        if(!delay_satrt || scheduler.ticks() > 150){
            delay_satrt = false;
            const uint64_t start_evaluation = AP_HAL::micros64();
            //Bob: log ekf data, -1 means the primary core;
            AP::ahrs_navekf().Log_Write_BKF1_rover(-1, AP_HAL::micros64());
            const uint64_t end_evaluation = AP_HAL::micros64();
            if(scheduler.ticks() % 50 == 0){
                AP::logger().Write_BOBL(19, (int)(end_evaluation-start_evaluation));
            }
        }
        //============================================
    }
}

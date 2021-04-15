#include "AP_InertialSensor.h"
#include <AP_Scheduler/AP_Scheduler.h>
#include <GCS_MAVLink/GCS.h>

#define SHOULD_INT_SMP_ATK (check_inter_sample_atk() && AP_Scheduler::get_singleton()->inter_sample_atk)

const Vector3f &AP_InertialSensor::get_accel(uint8_t i) const{
    // if(SHOULD_INT_SMP_ATK){
    //     gcs().send_text(MAV_SEVERITY_INFO, "Inter Sample Attack Launched!!");
    //     return get_fake_accel(_accel[i]);
    // }

    return _accel[i];
}

const Vector3f &AP_InertialSensor::get_gyro(uint8_t i) const{
    // if(SHOULD_INT_SMP_ATK){
    //     //gcs().send_text(MAV_SEVERITY_INFO, "Inter Sample Attack Launched!!");
    //     return get_fake_gyro(_gyro[i]);
    // }

    return _gyro[i];
}

// bool AP_InertialSensor::get_delta_angle(uint8_t i, Vector3f &delta_angle) const
// {
//     // if(SHOULD_INT_SMP_ATK){
//     //     delta_angle = get_fake_delta_angle(_delta_angle[i]);
//     //     return true;
//     // }

//     if (_delta_angle_valid[i]) {
//         delta_angle = _delta_angle[i];
//         return true;
//     } else if (get_gyro_health(i)) {
//         // provide delta angle from raw gyro, so we use the same code
//         // at higher level
//         delta_angle = get_gyro(i) * get_delta_time();
//         return true;
//     }
//     return false;
// }

// const Vector3f     &AP_InertialSensor::get_fake_accel(const Vector3f & true_accel) const{
//         (*fake_accel_ptr) = true_accel;
//         fake_accel_ptr->x += 10;
//         fake_accel_ptr->z -= 15;
//         return (*fake_accel_ptr);
// }



// const Vector3f     &AP_InertialSensor::get_fake_gyro(const Vector3f & true_gyro) const{
//         (*fake_gyro_ptr) = true_gyro;
//         fake_gyro_ptr->x += 2;
//         fake_gyro_ptr->z += 2;
//         return (*fake_gyro_ptr);
// }




// Vector3f & AP_InertialSensor::get_fake_delta_angle(const Vector3f &true_delta_angle) const
// {
//     (*fake_delta_angle) = true_delta_angle;
//     fake_delta_angle->x = 1;
//     return (*fake_delta_angle);
// }

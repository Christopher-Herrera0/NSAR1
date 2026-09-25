#include "main.h"
#include "EZ-Template/api.hpp"

void intertial_check(){
    auto check_motors = [&](std::vector<pros::Motor>& motors) {
        for (auto& m : motors) {
            if (m.get_actual_velocity() > 1){
                printf("[IC]: Robot in motion");
            }
        }
    }
    if (chassis.imu == nullptr || !chassis.imu->is_installed()) {
        printf("[IC]: IMU on port %d not responding\n", chassis.imu != nullptr ? chassis.imu->get_port() : -1);
    }
}
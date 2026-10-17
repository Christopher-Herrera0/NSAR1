#include "main.h"
#include "EZ-Template/api.hpp"

volatile bool cascade_resetting = false;

void intertial_check(){
    /*
    auto check_motors = [&](std::vector<pros::Motor>& motors) {
        for (auto& m : motors) {
            if (m.get_actual_velocity() > 1){
                printf("[IC]: Robot in motion");
                break;
            } else {
                continue;
            }
        }
    }
    if (chassis.imu == nullptr || !chassis.imu->is_installed()) {
        printf("[IC]: IMU on port %d not responding\n", chassis.imu != nullptr ? chassis.imu->get_port() : -1);
    }
    */
}

void cascade_reset() {
    while (true) {
        if (master.get_digital_new_release(DIGITAL_RIGHT)) {
            cascade_resetting = true;
            lift.move_absolute(0, 127);
            wrist.move_absolute(0, 70);

            int timeout = 0;
            while (!((lift.get_position() < 5) && (lift.get_position() > -5) &&
                     (wrist.get_position() < 5) && (wrist.get_position() > -5)) &&
                   timeout < 1500) {
                pros::delay(2);
                timeout++;
            }
            printf("Cascade is reset.\n");
            cascade_resetting = false;
        }
        pros::delay(10);
    }
}
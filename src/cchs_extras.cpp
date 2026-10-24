#include "main.h"
#include "EZ-Template/api.hpp"

volatile bool cascade_resetting = false;
volatile bool intake_jammed = false;

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

void brain_log(){

    while(true){
        std::cout << "Efficiency: " << intake.get_efficiency() << std::endl;

        pros::delay(250);
    }
}

void intake_antijam(){
    while (true){
        if (intake.get_efficiency() < 5){
            intake_jammed = true;
            intake.move_relative(-3000, 127);
            intake_jammed = false;
        }
        pros::delay(10);
    }
}
#include "main.h"
#include "EZ-Template/api.hpp"

volatile bool cascade_resetting = false;
volatile bool intake_jammed = false;
volatile bool scoring_block = false;

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

void stack_action(){
    double targetGoalDistance = 400; // Change 400 to whatever distance away from goal is ideal IN MILIMETERS
    double wristHorizPos = 400; // Change 400 to however many revs is perfectly horizontal IN REVOLUTIONS
    double bansheeReading = 0; // For use in loops below
    while (true){
        if (master.get_digital_new_press(DIGITAL_Y)){
            scoring_block = true;
            if (goal_distance.get_distance() > 1000){
                master.rumble("_._");
                scoring_block = false;
                break;
            } else {
                double distanceTargetError = (targetGoalDistance - goal_distance.get_distance());
                chassis.pid_drive_set((distanceTargetError * okapi::millimeter), 127);
                chassis.pid_wait();
                wrist.move_absolute(wristHorizPos, 127);
                bansheeReading = banshee_distance.get_distance();
                lift.move(127); // Reverse if necessary, not sure if pos or neg 127 makes lift go up.
                while ((bansheeReading >= bansheeReading - 10) && (bansheeReading <= bansheeReading + 10)){
                    pros::delay(10);
                }
                
            }
        }
    }
}
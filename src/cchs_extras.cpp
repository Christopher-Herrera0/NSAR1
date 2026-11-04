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
        //std::cout << "Efficiency: " << intake.get_efficiency() << std::endl;

        pros::delay(5000);
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
    // Constants
    double scoringDistance = 400; // Ideal distance from goal to score in milimeters.
    double wristLength = 400; // Distance of banshee edge furthest from wrist joint in milimeters
    double approachDistance = (scoringDistance + wristLength); // Distance the robot needs to be in order for lift to rise without hitting stack in milimeters.
    double pincupHeight = 0; // Distance lift needs to raise to clear stack in revolutions.
    okapi::QLength castingDoubleMM = 1_mm; // Used for converting double into okapi units. Constant.
    //

    while (true){
        if (master.get_digital_new_press(DIGITAL_Y)){
            scoring_block = true; // Disables driver controls.
            if (goal_distance.get_distance() > 1000){
                master.rumble("_._"); // Tells driver that stack is not in range.
                scoring_block = false; // Enables driver controls.
                break;
            } else {
                double distanceTargetError = (goal_distance.get_distance() - approachDistance); // Used to automate the robot's ideal approach to stack.
                wrist.move_absolute(800, 127); // Perfectly horizontal banshee position.

                // Approaching stack...
                chassis.pid_drive_set((distanceTargetError * castingDoubleMM), 127); 
                chassis.pid_wait();

                // Raising lift...
                double bansheeReading = banshee_distance.get_distance(); // Gets current distance between banshee and stack
                lift.move(127); // Reverse if necessary, not sure if pos or neg 127 makes lift go up.
                while (!(banshee_distance.get_distance() >= bansheeReading - 10) && !(banshee_distance.get_distance() <= bansheeReading + 10)){
                    // As soon as the most recent reading is outside +/- 10mm of initial bansheeReading, loop breaks.
                    // Until loop breaks, lift continues rising at 127 velocity.
                    pros::delay(10);
                }
                lift.move(0); 
                lift.move_relative(pincupHeight, 127); // Raises lift to clear stack

                // Getting pin-cup over stack...
                chassis.pid_drive_set((wristLength * castingDoubleMM), 60);
                chassis.pid_wait();
                
                // Scoring motion
                lift.move_relative((-1 * pincupHeight), 127); // Lowers lift over stack.
                pros::delay(100); // Gives time to settle
                claw.move(-127); // Spit out pincup
                wrist.move_absolute(2000, 127); // Raises banshee to clear stacked pincup.
                lift.move_relative(pincupHeight, 127); // Raises lift to clear stack.
                claw.move(0); // Stop spinning
                pros::delay(500); // Gives time to settle
                chassis.pid_drive_set((-1 * (wristLength * castingDoubleMM)), 60); // Brings robot away from stack to lower lift
                chassis.pid_wait();

                // Cascade Reset Function Code
                lift.move_absolute(0, 127);
                wrist.move_absolute(0, 70);
                int timeout = 0;
                while (!((lift.get_position() < 5) && (lift.get_position() > -5) &&
                         (wrist.get_position() < 5) && (wrist.get_position() > -5)) && timeout < 1500) {
                pros::delay(2);
                timeout++;
                }

                scoring_block = false; // Enable driver controls
            }
        }
    }
}
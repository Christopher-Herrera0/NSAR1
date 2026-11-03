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

void drive_to_distance(pros::Distance* distance_sensor, double target_in, int max_speed) {
  if (distance_sensor == nullptr) return;

  ez::PID::Constants consts = chassis.pid_drive_constants_get();
  ez::PID pid(consts.kp, consts.ki, consts.kd, consts.start_i, "Distance Drive");

  pid.target_set(0.0);
  pid.timers_reset();

  // 1. FASTER EXIT TIMERS: Reduced settlement check from 100ms/250ms down to 40ms/80ms
  // Exits quickly when within 0.4 inches rather than waiting a quarter of a second
  pid.exit_condition_set(40, 0.4, 80, 0.8, 0, 0);

  int timeout_timer = 0;
  const int max_timeout = 2000; // Reduced safety timeout to 2.0s

  while (timeout_timer < max_timeout) {
    int32_t dist_mm = distance_sensor->get();
    int32_t confidence = distance_sensor->get_confidence();

    if (dist_mm == PROS_ERR || dist_mm == INT32_MAX || confidence < 10) {
      chassis.drive_set(0, 0);
      pros::delay(ez::util::DELAY_TIME);
      timeout_timer += ez::util::DELAY_TIME;
      continue;
    }

    double current_dist_in = dist_mm / 25.4;
    double error_in = current_dist_in - target_in;

    // 2. INSTANT CHAINING BREAK: Exit immediately once inside tolerance (0.4 in)
    // Prevents the robot from coming to a complete dead stop if chaining into another auton move
    if (std::abs(error_in) <= 0.4) {
      break;
    }

    double power = pid.compute_error(error_in, current_dist_in);
    int clamped_power = static_cast<int>(ez::util::clamp(power, max_speed, -max_speed));

    // 3. HIGHER MINIMUM POWER FLOOR: Increased from 25 to 38
    // Eliminates slow crawling near the end of the move to overcome motor friction & brake hold instantly
    if (std::abs(clamped_power) < 38) {
      clamped_power = (error_in > 0) ? 38 : -38;
    }

    chassis.drive_set(clamped_power, clamped_power);

    if (pid.exit_condition() != ez::RUNNING) break;

    pros::delay(ez::util::DELAY_TIME);
    timeout_timer += ez::util::DELAY_TIME;
  }

  // 4. Removed high-frequency printf & fflush (serial latency killer)
}

void drive_to_distance_rear(pros::Distance* distance_sensor, double target_in, int max_speed) {
  if (distance_sensor == nullptr) return;

  ez::PID::Constants consts = chassis.pid_drive_constants_get();
  ez::PID pid(consts.kp, consts.ki, consts.kd, consts.start_i, "Distance Drive Rear");

  pid.target_set(0.0);
  pid.timers_reset();
  pid.exit_condition_set(40, 0.4, 80, 0.8, 0, 0);

  int timeout_timer = 0;
  const int max_timeout = 2000;

  while (timeout_timer < max_timeout) {
    int32_t dist_mm = distance_sensor->get();
    int32_t confidence = distance_sensor->get_confidence();

    if (dist_mm == PROS_ERR || dist_mm == INT32_MAX || dist_mm >= 9990 || confidence < 5) {
      chassis.drive_set(0, 0);
      pros::delay(ez::util::DELAY_TIME);
      timeout_timer += ez::util::DELAY_TIME;
      continue;
    }

    double current_dist_in = dist_mm / 25.4;
    double error_in = current_dist_in - target_in;

    if (std::abs(error_in) <= 0.4) break;

    double power = pid.compute_error(error_in, current_dist_in);
    int clamped_power = static_cast<int>(ez::util::clamp(power, max_speed, -max_speed));

    if (std::abs(clamped_power) < 38) {
      clamped_power = (error_in > 0) ? 38 : -38;
    }

    // INVERT MOTOR POWER: Drives FORWARD to increase distance from rear wall
    chassis.drive_set(-clamped_power, -clamped_power);

    if (pid.exit_condition() != ez::RUNNING) break;

    pros::delay(ez::util::DELAY_TIME);
    timeout_timer += ez::util::DELAY_TIME;
  }

  chassis.drive_set(0, 0);
}
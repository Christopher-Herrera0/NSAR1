#include "main.h"
#include "EZ-Template/api.hpp"

volatile bool cascade_resetting = false;
volatile bool intake_jammed = false;
volatile bool scoring_block = false;
static double goal_distance_offset = 0.0;
double distance_baseline_bias = 0.0;

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

void tare_goal_distance(pros::Distance* sensor) {
  if (sensor == nullptr) return;
  int32_t dist_mm = sensor->get();
  if (dist_mm != PROS_ERR && dist_mm < 9990) {
    goal_distance_offset = dist_mm / 25.4;
  }
}

double get_tared_distance(pros::Distance* sensor) {
  if (sensor == nullptr) return -1.0;
  int32_t dist_mm = sensor->get();
  if (dist_mm == PROS_ERR || dist_mm >= 9990 || sensor->get_confidence() < 5) return -1.0;
  return (dist_mm / 25.4) - goal_distance_offset;
}

void calibrate_distance_baseline(pros::Distance* sensor) {
  if (sensor == nullptr) return;
  double sum = 0.0;
  int valid_samples = 0;

  for (int i = 0; i < 10; i++) {
    int32_t raw = sensor->get();
    if (raw != PROS_ERR && raw < 9990) {
      sum += (raw / 25.4);
      valid_samples++;
    }
    pros::delay(10);
  }

  if (valid_samples > 0) {
    distance_baseline_bias = sum / valid_samples;
  }
}

void drive_to_distance(pros::Distance* distance_sensor, double target_in, int max_speed) {
  if (distance_sensor == nullptr) return;

  ez::PID::Constants consts = chassis.pid_drive_constants_get();
  ez::PID pid(consts.kp, consts.ki, consts.kd, consts.start_i, "Distance Drive");

  pid.target_set(0.0);
  pid.timers_reset();
  pid.exit_condition_set(40, 0.4, 80, 0.8, 0, 0);

  int timeout_timer = 0;
  const int max_timeout = 2000;

  while (timeout_timer < max_timeout) {
    // FIXED: Use tared distance to incorporate zero-calibration offset
    double current_dist_in = get_tared_distance(distance_sensor);

    if (current_dist_in < 0) { // Invalid reading guard
      chassis.drive_set(0, 0);
      pros::delay(ez::util::DELAY_TIME);
      timeout_timer += ez::util::DELAY_TIME;
      continue;
    }

    double error_in = current_dist_in - target_in;

    if (std::abs(error_in) <= 0.4) break;

    double power = pid.compute_error(error_in, current_dist_in);
    int clamped_power = static_cast<int>(ez::util::clamp(power, max_speed, -max_speed));

    if (std::abs(clamped_power) < 38) {
      clamped_power = (error_in > 0) ? 38 : -38;
    }

    chassis.drive_set(clamped_power, clamped_power);

    if (pid.exit_condition() != ez::RUNNING) break;

    pros::delay(ez::util::DELAY_TIME);
    timeout_timer += ez::util::DELAY_TIME;
  }
  chassis.drive_set(0, 0);
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
    // FIXED: Use tared distance calculation
    double current_dist_in = get_tared_distance(distance_sensor);

    if (current_dist_in < 0) { // Invalid reading guard
      chassis.drive_set(0, 0);
      pros::delay(ez::util::DELAY_TIME);
      timeout_timer += ez::util::DELAY_TIME;
      continue;
    }

    double error_in = current_dist_in - target_in;

    if (std::abs(error_in) <= 0.4) break;

    double power = pid.compute_error(error_in, current_dist_in);
    int clamped_power = static_cast<int>(ez::util::clamp(power, max_speed, -max_speed));

    if (std::abs(clamped_power) < 38) {
      clamped_power = (error_in > 0) ? 38 : -38;
    }

    chassis.drive_set(-clamped_power, -clamped_power);

    if (pid.exit_condition() != ez::RUNNING) break;

    pros::delay(ez::util::DELAY_TIME);
    timeout_timer += ez::util::DELAY_TIME;
  }

  chassis.drive_set(0, 0);
}
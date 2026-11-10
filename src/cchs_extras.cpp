#include "main.h"
#include "EZ-Template/api.hpp"
#include <algorithm>
#include <cmath>

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

// One valid reading in inches, or -1 if the sensor sees nothing usable.
static double sample_in(pros::Distance& s) {
  int32_t mm = s.get();
  if (mm == PROS_ERR || mm >= 2000 || s.get_confidence() < 5) return -1.0;
  return mm / 25.4;
}
 
static double med3(double a, double b, double c) {
  return std::max(std::min(a, b), std::min(std::max(a, b), c));
}
 
static void drive_to_distance_impl(pros::Distance& sensor, double target_in, int max_speed, bool reverse, int timeout_ms = 2000, int no_signal_ms = 400) {
  // The PID task keeps holding the last pid_drive_set target and would fight the
  // drive_set() calls below. Disable it; the next pid_*_set call re-enables it.
  chassis.drive_mode_set(ez::DISABLE);
 
  ez::PID::Constants c = chassis.pid_drive_constants_get();
  ez::PID pid(c.kp, c.ki, c.kd, c.start_i, "Distance Drive");
  pid.target_set(0.0);
  pid.timers_reset();
  pid.exit_condition_set(40, 0.4, 80, 0.8, 0, 0);
 
  const int dir = reverse ? -1 : 1;
  const uint32_t start = pros::millis();
  uint32_t last_valid = start;
  double w[3] = {0, 0, 0};
  int n = 0;
 
  while (pros::millis() - start < static_cast<uint32_t>(timeout_ms)) {
    double s = sample_in(sensor);
    if (s < 0) {
      chassis.drive_set(0, 0);
      if (pros::millis() - last_valid > static_cast<uint32_t>(no_signal_ms)) {
        printf("drive_to_distance: no valid sensor reading, giving up\n");
        break;  // don't burn the full 2 s staring at nothing
      }
      pros::delay(20);
      continue;
    }
    last_valid = pros::millis();
 
    w[n % 3] = s;
    n++;
    if (n < 3) { pros::delay(20); continue; }  // fill the median window first
 
    double d = med3(w[0], w[1], w[2]);
    double err = d - target_in;  // + means too far from the wall/goal
    if (std::abs(err) <= 0.4) break;
 
    double power = pid.compute_error(err, d);
    int p = static_cast<int>(ez::util::clamp(power, max_speed, -max_speed));
    int min_p = std::abs(err) > 2.0 ? 38 : 25;  // taper so it doesn't overshoot the 0.4" band
    if (std::abs(p) < min_p) p = (err > 0) ? min_p : -min_p;
 
    chassis.drive_set(dir * p, dir * p);
    if (pid.exit_condition() != ez::RUNNING) break;
    pros::delay(20);  // sensor updates at roughly 50 Hz
  }
  chassis.drive_set(0, 0);
}
 
void drive_to_distance(pros::Distance* s, double target_in, int max_speed) {
  if (s) drive_to_distance_impl(*s, target_in, max_speed, false);
}
 
void drive_to_distance_rear(pros::Distance* s, double target_in, int max_speed) {
  if (s) drive_to_distance_impl(*s, target_in, max_speed, true);
}

void wait_until_ms(uint32_t t0, uint32_t ms) {
  uint32_t elapsed = pros::millis() - t0;
  if (elapsed < ms) pros::delay(ms - elapsed);
}

bool drive_to_pickup(pros::Distance& front_sensor, double target_in, double max_travel_in, int max_speed, bool reverse, int timeout_ms) {
  chassis.drive_mode_set(ez::DISABLE);

  ez::PID::Constants c = chassis.pid_drive_constants_get();
  ez::PID pid(c.kp, c.ki, c.kd, c.start_i, "Pickup Drive");
  pid.target_set(0.0);
  pid.timers_reset();
  pid.exit_condition_set(90, 1.0, 250, 3.0, 500, 500);

  const int dir = reverse ? -1 : 1;
  const uint32_t start = pros::millis();
  const double start_l = chassis.drive_sensor_left();
  const double start_r = chassis.drive_sensor_right();
  double w[3] = {0, 0, 0};
  int n = 0;
  bool found = false;
  int loop_count = 0;

  while (pros::millis() - start < static_cast<uint32_t>(timeout_ms)) {
    double traveled = std::abs(((chassis.drive_sensor_left() - start_l) +
                                (chassis.drive_sensor_right() - start_r)) / 2.0);
    if (traveled >= max_travel_in) {
      printf("drive_to_pickup: hit max_travel_in safety cap\n");
      break;
    }

    double s = sample_in(front_sensor);
    double err;
    if (s > 0) {
      w[n % 3] = s; n++;
      if (n >= 3) {
        found = true;
        double d = med3(w[0], w[1], w[2]);
        err = d - target_in;
        if (std::abs(err) <= 0.4) {
          printf("drive_to_pickup: within tolerance, err=%.2f, d=%.2f, traveled=%.2f\n", err, d, traveled);
          break;
        }
      } else {
        err = max_travel_in - traveled;
      }
    } else {
      err = max_travel_in - traveled;
    }

    double power = pid.compute_error(err, traveled);
    int p = static_cast<int>(ez::util::clamp(power, max_speed, -max_speed));
    chassis.drive_set(dir * p, dir * p);

    if (loop_count % 10 == 0) {
      printf("loop=%d s=%.2f n=%d found=%d err=%.2f power=%.2f p=%d traveled=%.2f dir=%d\n",
             loop_count, s, n, found, err, power, p, traveled, dir);
    }
    loop_count++;

    if (found && pid.exit_condition() != ez::RUNNING) {
      printf("drive_to_pickup: pid exit_condition fired\n");
      break;
    }
    pros::delay(20);
  }
  chassis.drive_set(0, 0);
  printf("drive_to_pickup: returning found=%d\n", found);
  return found;
}

bool align_to_goal_dual(pros::Distance& sensor_a, pros::Distance& sensor_b, double spacing_in, bool a_is_left, int turn_speed, double heading_bias_in, int samples_per_side, double max_reasonable_deg) {
  double sum_a = 0, sum_b = 0;
  int valid_a = 0, valid_b = 0;

  for (int i = 0; i < samples_per_side; i++) {
    double a = sample_in(sensor_a);
    double b = sample_in(sensor_b);
    if (a > 0) { sum_a += a; valid_a++; }
    if (b > 0) { sum_b += b; valid_b++; }
    pros::delay(5);
  }

  if (valid_a < samples_per_side * 0.6 || valid_b < samples_per_side * 0.6) {
    printf("align_to_goal_dual: too many invalid samples (a:%d/%d b:%d/%d), aborting\n",
           valid_a, samples_per_side, valid_b, samples_per_side);
    return false;
  }

  double a_avg = sum_a / valid_a;
  double b_avg = sum_b / valid_b;

  double diff = (a_is_left ? (a_avg - b_avg) : (b_avg - a_avg)) - heading_bias_in;
  double angle_deg = std::atan2(diff, spacing_in) * 180.0 / M_PI;

  if (std::abs(angle_deg) > max_reasonable_deg) {
    printf("align_to_goal_dual: computed %.2f deg exceeds sanity limit (a_avg=%.2f b_avg=%.2f), aborting\n",
           angle_deg, a_avg, b_avg);
    return false;
  }

  if (std::abs(angle_deg) < 1.0) return true;

  printf("align_to_goal_dual: off by %.2f deg (a_avg=%.2f b_avg=%.2f)\n", angle_deg, a_avg, b_avg);
  chassis.pid_turn_relative_set(angle_deg, turn_speed);
  chassis.pid_wait();
  return true;
}

// Backs toward a sensor target WHILE turning to a target heading, combining
// what are currently two sequential motions into one. Both a distance-error
// PID and a heading-error PID run every loop iteration, and their outputs
// are combined into differential left/right power.
bool drive_to_distance_turn(pros::Distance& sensor, double target_in, double target_heading_deg, int max_speed, int max_turn_power, int timeout_ms) {
  chassis.drive_mode_set(ez::DISABLE);

  ez::PID::Constants dc = chassis.pid_drive_constants_get();
  ez::PID drive_pid(dc.kp, dc.ki, dc.kd, dc.start_i, "DistTurn Drive");
  drive_pid.target_set(0.0);
  drive_pid.timers_reset();
  drive_pid.exit_condition_set(40, 0.4, 80, 0.8, 0, 0);

  ez::PID::Constants hc = chassis.pid_heading_constants_get();
  ez::PID heading_pid(hc.kp, hc.ki, hc.kd, hc.start_i, "DistTurn Heading");
  heading_pid.timers_reset();

  const uint32_t start = pros::millis();
  uint32_t last_valid = start;
  double w[3] = {0, 0, 0};
  int n = 0;
  int settled_count = 0;

  while (pros::millis() - start < static_cast<uint32_t>(timeout_ms)) {
    double s = sample_in(sensor);
    double drive_power = 0;
    bool have_dist_err = false;
    double drive_err = 0;

    if (s < 0) {
      if (pros::millis() - last_valid > 400) {
        printf("drive_to_distance_turn: no valid sensor reading, giving up\n");
        break;
      }
    } else {
      last_valid = pros::millis();
      w[n % 3] = s; n++;
      if (n >= 3) {
        double d = med3(w[0], w[1], w[2]);
        drive_err = d - target_in;
        have_dist_err = true;
        drive_power = drive_pid.compute_error(drive_err, d);
      }
    }

    double heading_now = chassis.drive_imu_get();
    double heading_err = target_heading_deg - heading_now;
    double heading_power = heading_pid.compute_error(heading_err, heading_now);

    int base = static_cast<int>(ez::util::clamp(drive_power, max_speed, -max_speed));
    int turn = static_cast<int>(ez::util::clamp(heading_power, max_turn_power, -max_turn_power));

    // Backing up (negative base direction) -- verify this sign on the robot.
    chassis.drive_set(-base + turn, -base - turn);

    if (have_dist_err && std::abs(drive_err) <= 0.4 && std::abs(heading_err) <= 1.0) {
      if (++settled_count >= 4) break;
    } else {
      settled_count = 0;
    }

    pros::delay(20);
  }
  chassis.drive_set(0, 0);
  return true;
}

void goal_dual_bias_diagnostic() {
  const int SAMPLES = 30;
  double sum_a = 0, sum_b = 0, sum_diff = 0;
  int valid = 0;

  printf("=== goal_dual_bias_diagnostic: robot must be squared to goal NOW ===\n");

  for (int i = 0; i < SAMPLES; i++) {
    double a = sample_in(goal_distance_left);
    double b = sample_in(goal_distance);

    if (a > 0 && b > 0) {
      double diff = a - b;  // matches a_is_left = true convention; flip if you use false
      sum_a += a;
      sum_b += b;
      sum_diff += diff;
      valid++;
      printf("[%2d] a=%.3f  b=%.3f  diff=%.3f\n", i, a, b, diff);
    } else {
      printf("[%2d] invalid read (a=%.3f b=%.3f)\n", i, a, b);
    }
    pros::delay(50);
  }

  if (valid < SAMPLES / 2) {
    printf("=== too many invalid reads (%d/%d) -- check mounting/aim before trusting any number below ===\n",
           valid, SAMPLES);
    return;
  }

  printf("=== AVERAGE a=%.3f  b=%.3f  diff(a-b)=%.3f  <- use this as heading_bias_in ===\n",
         sum_a / valid, sum_b / valid, sum_diff / valid);
}

void dist_turn_test() {
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  pros::delay(50);
  bool ok = drive_to_distance_turn(goal_distance, 6.5, 34.0, 110, 40, 2000);
  printf("dist_turn_test finished, ok=%d, final heading=%.2f\n", ok, chassis.drive_imu_get());
}
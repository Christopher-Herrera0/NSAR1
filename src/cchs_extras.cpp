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

  const int dir = reverse ? -1 : 1;
  const uint32_t start = pros::millis();
  const double start_l = chassis.drive_sensor_left();
  const double start_r = chassis.drive_sensor_right();
  double w[3] = {0, 0, 0};
  int n = 0;
  bool found = false;
  int loop_count = 0;
  double prev_err = 0;
  bool have_prev_err = false;

  while (pros::millis() - start < static_cast<uint32_t>(timeout_ms)) {
    double traveled = std::abs(((chassis.drive_sensor_left() - start_l) +
                                (chassis.drive_sensor_right() - start_r)) / 2.0);
    if (traveled >= max_travel_in) {
      printf("drive_to_pickup: hit max_travel_in safety cap\n");
      break;
    }

    double s = sample_in(front_sensor);
    int p;

    if (s > 0) {
      w[n % 3] = s; n++;
    }

    if (n >= 3) {
      found = true;
      double d = med3(w[0], w[1], w[2]);
      double err = d - target_in;

      if (std::abs(err) <= 0.4) {
        printf("drive_to_pickup: within tolerance, err=%.2f, d=%.2f, traveled=%.2f\n", err, d, traveled);
        break;
      }

      // Manual PD, computed directly -- no library call whose behavior
      // we can't verify. derivative = how fast the error is changing
      // per loop (20ms), scaled by kd.
      double derivative = have_prev_err ? (err - prev_err) / 0.02 : 0.0;
      double power = c.kp * err + c.kd * derivative;
      prev_err = err;
      have_prev_err = true;

      p = static_cast<int>(ez::util::clamp(power, max_speed, -max_speed));
      int min_p = std::abs(err) > 2.0 ? 38 : 25;
      if (std::abs(p) < min_p) p = (err > 0) ? min_p : -min_p;

      if (loop_count % 10 == 0) {
        printf("loop=%d PHASE=sensor d=%.2f err=%.2f deriv=%.2f power=%.2f p=%d traveled=%.2f\n",
               loop_count, d, err, derivative, power, p, traveled);
      }
    } else {
      double travel_err = max_travel_in - traveled;
      p = static_cast<int>(ez::util::clamp(c.kp * travel_err, max_speed, -max_speed));
      if (std::abs(p) < 40) p = (travel_err > 0) ? 40 : -40;

      if (loop_count % 10 == 0) {
        printf("loop=%d PHASE=filling s=%.2f n=%d p=%d traveled=%.2f\n", loop_count, s, n, p, traveled);
      }
    }
    loop_count++;

    chassis.drive_set(dir * p, dir * p);
    pros::delay(20);
  }
  chassis.drive_set(0, 0);
  printf("drive_to_pickup: returning found=%d\n", found);
  return found;
}

bool align_to_goal_dual(pros::Distance& sensor_straight, pros::Distance& sensor_toed,
                        double spacing_in, double toe_angle_deg, bool straight_is_left,
                        int turn_speed, double heading_bias_in,
                        int samples_per_side, double max_reasonable_deg) {
  int groups = samples_per_side / 3;
  if (groups < 1) groups = 1;

  double sum_s = 0, sum_t = 0;
  int valid_groups_s = 0, valid_groups_t = 0;

  for (int g = 0; g < groups; g++) {
    double ws[3], wt[3];
    int ns = 0, nt = 0;

    for (int i = 0; i < 3; i++) {
      double s = sample_in(sensor_straight);
      double t = sample_in(sensor_toed);
      if (s > 0) ws[ns++] = s;
      if (t > 0) wt[nt++] = t;
      pros::delay(5);
    }

    if (ns == 3) { sum_s += med3(ws[0], ws[1], ws[2]); valid_groups_s++; }
    if (nt == 3) { sum_t += med3(wt[0], wt[1], wt[2]); valid_groups_t++; }
  }

  if (valid_groups_s < groups * 0.6 || valid_groups_t < groups * 0.6) {
    printf("align_to_goal_dual: too many invalid groups (s:%d/%d t:%d/%d), aborting\n",
           valid_groups_s, groups, valid_groups_t, groups);
    return false;
  }

  double D = sum_s / valid_groups_s;
  double t_avg = sum_t / valid_groups_t;

  double phi = toe_angle_deg * M_PI / 180.0;
  double sec_phi = 1.0 / std::cos(phi);
  double tan_phi = std::tan(phi);

  double expected_square_offset = D * (1.0 - sec_phi);

  double raw_diff = straight_is_left ? (D - t_avg) : (t_avg - D);
  double expected_offset_signed = straight_is_left ? -expected_square_offset : expected_square_offset;

  double diff = (raw_diff - expected_offset_signed) - heading_bias_in;

  double gain = (spacing_in / 2.0) * (1.0 + sec_phi) + D * tan_phi * sec_phi;
  if (gain < 0.01) {
    printf("align_to_goal_dual: degenerate gain (%.4f), aborting\n", gain);
    return false;
  }

  double angle_deg = (diff / gain) * 180.0 / M_PI;

  printf("D=%.3f t_avg=%.3f raw_diff=%.4f expected_offset=%.4f gain=%.4f angle=%.4f\n",
         D, t_avg, raw_diff, expected_offset_signed, gain, angle_deg);

  if (std::abs(angle_deg) > max_reasonable_deg) {
    printf("align_to_goal_dual: computed %.2f deg exceeds sanity limit (D=%.2f t_avg=%.2f), aborting\n",
           angle_deg, D, t_avg);
    return false;
  }

  if (std::abs(angle_deg) < 1.0) return true;

  printf("align_to_goal_dual: off by %.2f deg (D=%.2f t_avg=%.2f)\n", angle_deg, D, t_avg);
  chassis.pid_turn_relative_set(angle_deg, turn_speed);
  chassis.pid_wait();
  return true;
}

void goal_dual_bias_diagnostic() {
  const int SAMPLES = 30;
  double sum_a = 0, sum_b = 0, sum_diff = 0;
  int valid = 0;

  printf("=== goal_dual_bias_diagnostic: robot must be squared to goal NOW ===\n");

  for (int i = 0; i < SAMPLES; i++) {
    double a = sample_in(goal_distance_toed);
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
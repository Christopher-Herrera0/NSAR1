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
 
static void drive_to_distance_impl(pros::Distance& sensor, double target_in, int max_speed,
                                   bool reverse, int timeout_ms = 2000, int no_signal_ms = 400) {
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

// Small corrective turn (robot barely translates) so `side` reads target_in
// from whatever surface it's currently facing. Call this right before the
// pid_swing_set/pid_turn_set that aims you into the goal.
// target_in: measure this by hand once, with the robot placed exactly
// square to that surface during practice, then hardcode it.
bool align_heading_to_side_wall(pros::Distance& side, double target_in,
                                int timeout_ms = 600, int max_power = 35) {
  uint32_t start = pros::millis();
  int settled = 0;
  while (pros::millis() - start < static_cast<uint32_t>(timeout_ms)) {
    int32_t mm = side.get();
    if (mm == PROS_ERR || mm >= 2000 || side.get_confidence() < 5) {
      chassis.drive_set(0, 0);
      pros::delay(15);
      continue;
    }
    double reading = mm / 25.4;
    double err = reading - target_in;

    if (std::abs(err) <= 0.3) {
      if (++settled >= 4) break;
    } else {
      settled = 0;
    }

    int p = static_cast<int>(ez::util::clamp(4.0 * err, max_power, -max_power));
    if (std::abs(p) < 15 && std::abs(err) > 0.3) p = (err > 0) ? 15 : -15;

    chassis.drive_set(p, -p);  // pure rotation, no net forward travel
    pros::delay(15);
  }
  chassis.drive_set(0, 0);
  return settled >= 4;
}
#pragma once

void cascade_reset();
void brain_log();
void intake_antijam();
void stack_action();
void drive_to_distance(pros::Distance* distance_sensor, double target_in, int max_speed = 127);
void drive_to_distance_rear(pros::Distance* distance_sensor, double target_in, int max_speed);
void wait_until_ms(uint32_t t0, uint32_t ms);
bool drive_to_pickup(pros::Distance& front_sensor, double target_in, double max_travel_in, int max_speed, bool reverse, int timeout_ms = 2500);
bool align_to_goal_dual(pros::Distance& sensor_a, pros::Distance& sensor_b, double spacing_in, bool a_is_left, int turn_speed, double heading_bias_in = 0.0, int samples_per_side = 15, double max_reasonable_deg = 20.0);
bool drive_to_distance_turn(pros::Distance& sensor, double target_in, double target_heading_deg, int max_speed, int max_turn_power = 40, int timeout_ms = 2000);
void dist_turn_test();
void goal_dual_bias_diagnostic();
void stack_action();
extern volatile bool cascade_resetting;
extern volatile bool intake_jammed;
extern volatile bool scoring_block;


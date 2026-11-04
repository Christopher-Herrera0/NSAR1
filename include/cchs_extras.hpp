#pragma once

void cascade_reset();
void brain_log();
void intake_antijam();
void stack_action();
void drive_to_distance(pros::Distance* distance_sensor, double target_in, int max_speed = 127);
void drive_to_distance_rear(pros::Distance* distance_sensor, double target_in, int max_speed);
void tare_goal_distance(pros::Distance* sensor);
void calibrate_distance_baseline(pros::Distance* sensor);
void stack_action();
extern volatile bool cascade_resetting;
extern volatile bool intake_jammed;
extern volatile bool scoring_block;


#pragma once

void cascade_reset();
void brain_log();
void intake_antijam();
void stack_action();
void drive_to_distance(pros::Distance* distance_sensor, double target_in, int max_speed = 127);
void drive_to_distance_rear(pros::Distance* distance_sensor, double target_in, int max_speed);
void wait_until_ms(uint32_t t0, uint32_t ms);
bool align_heading_to_side_wall(pros::Distance& side, double target_in, int timeout_ms = 600, int max_power = 35);
void stack_action();
extern volatile bool cascade_resetting;
extern volatile bool intake_jammed;
extern volatile bool scoring_block;


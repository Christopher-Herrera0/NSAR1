#pragma once

void cascade_reset();
void brain_log();
void intake_antijam();
void stack_action();
extern volatile bool cascade_resetting;
extern volatile bool intake_jammed;
extern volatile bool scoring_block;
void distance_sensor(pros::Distance distance_sensor1, pros::Distance distance_sensor2, 
                        okapi::QLength targetDistance1, okapi::QLength targetDistance2);


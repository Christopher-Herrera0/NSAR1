#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern ez::Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

inline pros::MotorGroup lift({-18, 20});
inline pros::Motor intake(19);
inline pros::Motor wrist(-15);
inline pros::Motor claw(-11);

inline pros::Distance left_distance(17);
inline pros::Distance right_distance(6);
inline pros::Distance back_distance(10);
inline pros::Distance goal_distance(16);
inline pros::Distance banshee_distance(15);
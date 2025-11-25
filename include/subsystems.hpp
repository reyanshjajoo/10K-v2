#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor intake(5);
inline pros::Motor hood(6);

enum class IntakeState {
    idle,
    midGoal,
    highGoal,
    reverse,
    intake
};

extern IntakeState intakeState;

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');
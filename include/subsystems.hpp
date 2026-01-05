#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::MotorGroup intake({3, -8});

inline pros::adi::DigitalOut midGoalPiston('F');
inline pros::adi::DigitalOut blockerPiston('B');
inline pros::adi::DigitalOut matchload('C');
inline pros::adi::DigitalOut horn('A');

enum class IntakeState {
    idle,
    midGoal,
    midGoalAuto,
    highGoal,
    reverse,
    intake
};

extern IntakeState intakeState;

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');
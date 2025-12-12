#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 90;
const int DRIVE_SPEED_AWP = 110;
const int MATCHLOAD_SPEED = 50;
const int TURN_SPEED = 95;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(16.7, 0.0, 106.5);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 30.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// Drive Example
///

void right(){
  chassis.odom_xyt_set(48_in, 11_in, 270_deg);
  chassis.pid_turn_set({22, 20}, fwd, TURN_SPEED);
  chassis.pid_wait_quick();
  intakeState = IntakeState::intake;
  chassis.pid_odom_set({{22, 20}, fwd, DRIVE_SPEED-20});
  chassis.pid_wait_quick();
  chassis.pid_turn_set({53, 45}, fwd, TURN_SPEED);
  chassis.pid_wait_quick();
  matchload.set_value(true);
  chassis.pid_odom_set({{53, 45}, fwd, DRIVE_SPEED});
  chassis.pid_wait_quick();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(16_in, MATCHLOAD_SPEED, true); //TODO: change to 14
  pros::delay(0);
  chassis.pid_wait_quick();
  intakeState = IntakeState::idle;
  chassis.pid_drive_set(-28_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-5_in, DRIVE_SPEED-30, true);
  intakeState = IntakeState::highGoal;
  pros::delay(2000);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(5_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();
}

void right_7ball(){
  right();
  chassis.pid_drive_set(-10_in, DRIVE_SPEED+20, true);
  chassis.pid_wait_quick();
  matchloadDown = true;
}

void right_horn(){
  chassis.pid_drive_set(7_in, DRIVE_SPEED+20, true);
  chassis.pid_wait_quick();
  chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, SWING_SPEED, 10, ez::ccw);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(40_in, DRIVE_SPEED+20, true);

  // right();
  // chassis.pid_drive_set(-10_in, DRIVE_SPEED+20, true);
  // chassis.

}

void awp() {
  chassis.odom_xyt_set(-48_in, 16_in, 0_deg);
  chassis.pid_drive_set(29.5_in, DRIVE_SPEED_AWP-10, true);
  matchload.set_value(true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set({-60, 45.5}, fwd, TURN_SPEED);
  chassis.pid_wait_quick();
  intakeState = IntakeState::intake;
  chassis.pid_drive_set(9.5_in, MATCHLOAD_SPEED, true);
  chassis.pid_wait_quick();
  pros::delay(150);//mathcload delay
  chassis.pid_drive_set(-28_in, DRIVE_SPEED_AWP, true);
  matchload.set_value(false);
  chassis.pid_wait_quick_chain();
  intakeState = IntakeState::highGoal;
  chassis.pid_drive_set(-5_in, DRIVE_SPEED_AWP-30, true);
  pros::delay(1000);//score high goal
  //intakeState = IntakeState::idle;
  chassis.pid_drive_set(12_in, DRIVE_SPEED_AWP, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set({-24, 21}, fwd, TURN_SPEED);
  intakeState = IntakeState::intake;
  chassis.pid_odom_set({{-24, 21}, fwd, DRIVE_SPEED_AWP-10});//go to 3 ball
  chassis.pid_wait();
  //pros::delay(200);
  chassis.pid_turn_set({-6, 3}, rev, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_odom_set({{-6, 3}, rev, DRIVE_SPEED});//to mid goal
  chassis.pid_wait_quick();
  intakeState = IntakeState::midGoal;//score mid goal
  pros::delay(400);
  intakeState = IntakeState::idle;
  chassis.pid_drive_set(18_in, DRIVE_SPEED_AWP, true);//back to -23,23
  chassis.pid_wait_quick();
  intakeState = IntakeState::intake;
  //chassis.pid_turn_set({-24, -24}, fwd, TURN_SPEED);//next 3 ball
  //chassis.pid_wait_quick();
  //chassis.pid_drive_set(48_in, DRIVE_SPEED_AWP, true);
  //chassis.pid_wait_quick();
  chassis.pid_turn_set({-45, -55}, fwd, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(60_in, DRIVE_SPEED_AWP, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{-45, -55}, fwd, DRIVE_SPEED_AWP+30});
  matchload.set_value(true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set({-60,-55}, fwd, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(17_in, MATCHLOAD_SPEED, true);
  chassis.pid_wait();
  pros::delay(50);//matchload delay
  chassis.pid_drive_set(-29.5_in, DRIVE_SPEED_AWP, true);
  chassis.pid_wait_quick_chain();
  intakeState = IntakeState::highGoal;
  chassis.pid_drive_set(-5_in, DRIVE_SPEED_AWP-30, true);


}
void left_7ball(){
  chassis.odom_xyt_set(48_in, -11_in, 270_deg);
  chassis.pid_turn_set({22, -20}, fwd, TURN_SPEED);
  chassis.pid_wait();
  intakeState = IntakeState::intake;
  chassis.pid_odom_set({{22, -20}, fwd, DRIVE_SPEED-20});
  chassis.pid_wait();
  chassis.pid_turn_set({53, -45}, fwd, TURN_SPEED);
  chassis.pid_wait();
  matchload.set_value(true);
  chassis.pid_odom_set({{53, -45}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(300);
  chassis.pid_drive_set(19_in, MATCHLOAD_SPEED, true);
  chassis.pid_wait();
  pros::delay(400);
  intakeState = IntakeState::idle;
  chassis.pid_drive_set(-36_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-5_in, DRIVE_SPEED-30, true);
  intakeState = IntakeState::highGoal;
  pros::delay(3000);
  chassis.pid_wait();
  chassis.pid_drive_set(5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-10_in, DRIVE_SPEED+20, true);
  chassis.pid_wait();
  matchloadDown = true;
}

void left_horn(){

}

void skills(){
  chassis.odom_xyt_set(-48_in, 16_in, 0_deg);
  chassis.pid_drive_set(30.5_in, DRIVE_SPEED_AWP, true);
  matchload.set_value(true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set({-60, 46.5}, fwd, TURN_SPEED);
  chassis.pid_wait_quick();
  intakeState = IntakeState::intake;
  chassis.pid_drive_set(11_in, DRIVE_SPEED_AWP, true);
  chassis.pid_wait_quick();
  pros::delay(300);//mathcload delay
  chassis.pid_drive_set(-10_in, DRIVE_SPEED_AWP, true);
  intakeState = IntakeState::idle;
  chassis.pid_wait_quick();
  chassis.pid_turn_set({-24, 58}, rev, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_odom_set({{-24, 58}, rev, DRIVE_SPEED_AWP});//to alley pose
  chassis.pid_wait_quick(); 
  chassis.pid_turn_set({34, 58}, rev, TURN_SPEED);
  chassis.pid_wait_quick(); 
  chassis.pid_drive_set(-50, DRIVE_SPEED_AWP, true);//to matchload 1
  chassis.pid_wait_quick(); 
  chassis.pid_turn_set({34,48}, rev, TURN_SPEED);//90 deg turn 
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-10_in, MATCHLOAD_SPEED, true); 
  chassis.pid_wait_quick();
  chassis.pid_turn_set({27, 48}, rev, TURN_SPEED);//turn to scoring 
  chassis.pid_drive_set(-40_in, DRIVE_SPEED_AWP, true);//scoring pose 

  

}

void skills_matchload(){
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  matchload.set_value(true);
  intakeState = IntakeState::intake;
  chassis.pid_drive_set(35_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

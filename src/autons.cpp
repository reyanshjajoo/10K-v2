#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 90;
const int DRIVE_SPEED_AWP = 120;
const int MATCHLOAD_SPEED = 50;
const int TURN_SPEED = 95;
const int SWING_SPEED = 110;
const int DRIVE_SPEED_SKILLS = 75;

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
  right();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.pid_drive_set(7_in, DRIVE_SPEED+20, true);
  chassis.pid_wait_quick();
  chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, SWING_SPEED, 5, ez::ccw);
  chassis.pid_wait_quick();
  horn.set_value(true);
  chassis.pid_drive_set(33, DRIVE_SPEED+20, true);


  // right();
  // chassis.pid_drive_set(-10_in, DRIVE_SPEED+20, true);
  // chassis.

}

void awp() {
  chassis.odom_xyt_set(-48_in, 16_in, 0_deg);
  chassis.pid_drive_set(29.5_in, DRIVE_SPEED_AWP-10, true);
  matchload.set_value(true);
   intakeState = IntakeState::intake;
  chassis.pid_wait_quick();
  chassis.pid_turn_set({-60, 45.5}, fwd, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(10.5_in, MATCHLOAD_SPEED+20, true);
  chassis.pid_wait_quick();
  pros::delay(110);//mathcload delay
  chassis.pid_drive_set(-28_in, DRIVE_SPEED_AWP-10, true);
    intakeState = IntakeState::idle;
  matchload.set_value(false);
  chassis.pid_wait_quick_chain();
  intakeState = IntakeState::highGoal;
  chassis.pid_drive_set(-5_in, DRIVE_SPEED_AWP-30, true);
  pros::delay(1000);//score high goal
  //intakeState = IntakeState::idle;
  chassis.pid_drive_set(12_in, DRIVE_SPEED_AWP, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set({-22, 21}, fwd, TURN_SPEED);
  intakeState = IntakeState::intake;
  chassis.pid_odom_set({{-22, 21}, fwd, DRIVE_SPEED_AWP-10});//go to 3 ball
  pros::delay(900);
  matchload.set_value(true);
  chassis.pid_wait();
  //pros::delay(200);
  chassis.pid_turn_set({-6, 3}, rev, TURN_SPEED);
  chassis.pid_wait_quick();
  matchload.set_value(false);
  chassis.pid_odom_set({{-6, 3}, rev, DRIVE_SPEED});//to mid goal
  intakeState = IntakeState::midGoal;//score mid goal
  pros::delay(1000);
  chassis.pid_wait_quick();
  intakeState = IntakeState::idle;
  chassis.pid_drive_set(18_in, DRIVE_SPEED_AWP, true);//back to -23,23
  chassis.pid_wait_quick();
  intakeState = IntakeState::intake;
  //chassis.pid_turn_set({-24, -24}, fwd, TURN_SPEED);//next 3 ball
  //chassis.pid_wait_quick();
  //chassis.pid_drive_set(48_in, DRIVE_SPEED_AWP, true);
  //chassis.pid_wait_quick();
  chassis.pid_turn_set({-45, -54}, fwd, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(59_in, DRIVE_SPEED_AWP, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{-45, -54}, fwd, DRIVE_SPEED_AWP});
  matchload.set_value(true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set({-60,-54}, fwd, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(17_in, MATCHLOAD_SPEED+20, true);
  chassis.pid_wait();
  pros::delay(50);//matchload delay
  chassis.pid_drive_set(-29.5_in, DRIVE_SPEED_AWP, true);
  chassis.pid_wait_quick();
  intakeState = IntakeState::highGoal;
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
  chassis.pid_drive_set(30.5_in, DRIVE_SPEED_SKILLS, true);
  matchload.set_value(true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set({-60, 46.5}, fwd, TURN_SPEED);
  chassis.pid_wait_quick();
  intakeState = IntakeState::intake;
  chassis.pid_drive_set(11_in, MATCHLOAD_SPEED, true);
  chassis.pid_wait_quick();
  pros::delay(1500);//mathcload delay
  chassis.pid_drive_set(-10_in, DRIVE_SPEED_SKILLS, true);
  intakeState = IntakeState::idle;
  matchload.set_value(false);
  chassis.pid_wait_quick();
  chassis.pid_turn_set({-24, 57}, rev, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_odom_set({{-24, 57}, rev, DRIVE_SPEED_SKILLS});//to alley pose
  chassis.pid_wait_quick(); 
  chassis.pid_turn_set({34, 57}, rev, TURN_SPEED);
  chassis.pid_wait_quick(); 
  chassis.pid_drive_set(-63_in, DRIVE_SPEED_SKILLS, true);//alley
  chassis.pid_wait_quick(); 
  chassis.pid_swing_set(ez::RIGHT_SWING, 90_deg, SWING_SPEED, 0);
  chassis.pid_wait_quick();
  //chassis.pid_turn_set({27, 48}, rev, TURN_SPEED);//turn to scoring 
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-15_in, DRIVE_SPEED_SKILLS, true);//scoring pose 
  chassis.pid_wait_quick(); 
  intakeState = IntakeState::highGoal;
  pros::delay(2000);
  chassis.pid_wait_quick();
  matchload.set_value(true);
  intakeState = IntakeState::intake;
  chassis.pid_drive_set(32_in, MATCHLOAD_SPEED, true);//matchload next 
  chassis.pid_wait_quick();
  pros::delay(300);
  chassis.pid_drive_set(-32_in, DRIVE_SPEED_SKILLS, true);
  chassis.pid_wait_quick();
  intakeState = IntakeState::highGoal;
  pros::delay(2000);
  chassis.pid_drive_set(16_in, DRIVE_SPEED_SKILLS, true);
  intakeState = IntakeState::intake;
  matchload.set_value(false);
  chassis.pid_wait_quick();
  chassis.pid_turn_set({45, -52}, fwd, DRIVE_SPEED_SKILLS);//turn to next side
  chassis.pid_wait_quick();
  chassis.pid_odom_set({{45, -52}, fwd, DRIVE_SPEED_SKILLS});//move to next side 
  chassis.pid_wait_quick();
  matchload.set_value(true);
  chassis.pid_turn_set({62, -52}, fwd, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_odom_set({{62, -52}, fwd, MATCHLOAD_SPEED});//matchload
  chassis.pid_wait_quick();
  pros::delay(1000);//intake matchload
  chassis.pid_drive_set(-18, DRIVE_SPEED_SKILLS, true);//back from matchload
  matchload.set_value(false);
  chassis.pid_wait_quick();
  chassis.odom_theta_set(0_deg);//reset pose at scoring position
  chassis.pid_swing_set(ez::LEFT_SWING, 180_deg, SWING_SPEED, 8);
  chassis.pid_wait_quick();  
  chassis.pid_odom_set({{-36_in, -60_in}, fwd, DRIVE_SPEED});//alley
  chassis.pid_wait_quick();

  
  
}

void skills_matchload(){
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  matchload.set_value(true);
  intakeState = IntakeState::intake;
  chassis.pid_drive_set(35_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}



#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 85;
const int DRIVE_SPEED_AWP = 100;
const int MATCHLOAD_SPEED = 50;
const int TURN_SPEED = 80;
const int SWING_SPEED = 110;
const int DRIVE_SPEED_SKILLS = 75;

///
// Constants
///

void default_constants()
{

  //pid
  chassis.pid_drive_constants_set(17.0, 0.0, 120.0);
  //chassis.pid_drive_constants_forward_set(17.0, 0.0, 120.0);
  //chassis.pid_drive_constants_backward_set(17.0, 0.0, 100.0);
  chassis.pid_heading_constants_set(11.0, 0.0, 30.0);       // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(2.5, 0.05, 20, 4.7);    // Turn in place constant
  chassis.pid_swing_constants_set(6.0, 0.0, 50.0);          // Swing constants
  chassis.pid_odom_angular_constants_set(2.83, 0.0, 25.8, 4.7);   // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5); // TODO: TUNE THIS (idt we are using boomerang)Angular control for boomerang motions
  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);


  chassis.odom_look_ahead_set(7_in);          // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in); // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);    // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest); // Changes the default behavior for turning, this defaults it to the shortest path there
}

void setMatchload(bool value) { 
  matchload.set_value(value); 
  // apply_profile(value ? ChassisProfile::MatchloadDown : ChassisProfile::Normal);
}

// ----------------------------------------------------------------

pros::Distance dist_back(13);
pros::Distance dist_right(14);

constexpr double FIELD_SIZE_IN = 144.0;
constexpr double FIELD_HALF_IN = FIELD_SIZE_IN/2;

constexpr double BACK_SENSOR_OFFSET_IN = 5.0;
constexpr double RIGHT_SENSOR_OFFSET_IN = 5.667;

inline double mm_to_in(double mm){
    return mm / 25.4;
}

void distance_reset(){
    double d_back_mm = dist_back.get();
    double d_right_mm = dist_right.get();

    double d_back_in = mm_to_in(d_back_mm);
    double d_right_in = mm_to_in(d_right_mm);

    double y_sensor = -FIELD_HALF_IN + d_back_in;
    double x_sensor = FIELD_HALF_IN - d_right_in;

    double y_center = y_sensor + BACK_SENSOR_OFFSET_IN;
    double x_center = x_sensor + RIGHT_SENSOR_OFFSET_IN;

    chassis.odom_xy_set(x_center * 1_in, y_center * 1_in);
}

// -----------------------------------------------------------------------------------------------------

void left7()
{
  chassis.odom_xyt_set(48_in, -11_in, 270_deg);
  chassis.pid_turn_set({22, -20}, fwd, TURN_SPEED);
  chassis.pid_wait_quick();
  intakeState = IntakeState::intake;
  chassis.pid_odom_set({{22, -20}, fwd, DRIVE_SPEED - 20});
  pros::delay(380);
  matchload.set_value(true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set({53, -46}, fwd, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_odom_set({{53, -46}, fwd, DRIVE_SPEED});
  chassis.pid_wait_quick();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(9.5_in, MATCHLOAD_SPEED, true);
  pros::delay(100);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-28.5_in, DRIVE_SPEED, true);
  matchload.set_value(false);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-5_in, DRIVE_SPEED, true);
  intakeState = IntakeState::highGoal;
  pros::delay(1500);
  chassis.pid_wait_quick();
}

void left_7ball()
{
  left7();
  chassis.pid_drive_set(5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-10_in, DRIVE_SPEED + 20, true);
  chassis.pid_wait();
  matchloadDown = true;
}

void left_horn()
{
  left7();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.pid_drive_set(7_in, DRIVE_SPEED + 20, true);
  chassis.pid_wait_quick();
  chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, SWING_SPEED, 5, ez::ccw);
  chassis.pid_wait_quick();
  horn.set_value(true);
  chassis.pid_drive_set(33, DRIVE_SPEED + 20, true);
}


void left_3_4(){
  chassis.odom_xyt_set(-48_in, 16_in, 0_deg);
  chassis.pid_drive_set(29.5_in, DRIVE_SPEED_AWP - 10, true);
  setMatchload(true);
  intakeState = IntakeState::intake;
  chassis.pid_wait_quick();
  chassis.pid_turn_set({-60, 45.5}, fwd, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(10_in, MATCHLOAD_SPEED, true);
  chassis.pid_wait_quick();
  pros::delay(160); // mathcload delay
  chassis.pid_drive_set(-28_in, DRIVE_SPEED_AWP - 10, true);
  setMatchload(false);
  chassis.pid_wait_quick_chain();
  intakeState = IntakeState::highGoal;
  chassis.pid_drive_set(-5_in, DRIVE_SPEED_AWP - 30, true);
  pros::delay(1010); // score high goal
  chassis.pid_drive_set(12_in, DRIVE_SPEED_AWP, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set({-22, 21}, fwd, TURN_SPEED);
  intakeState = IntakeState::intake;
  chassis.pid_odom_set({{-22, 21}, fwd, DRIVE_SPEED_AWP - 10}); // go to 3 ball
  pros::delay(900);
  setMatchload(true);
  chassis.pid_wait();
  chassis.pid_turn_set({-6.5, 3.5}, rev, TURN_SPEED);
  chassis.pid_wait_quick();
  setMatchload(false);
  chassis.pid_odom_set({{-6.5, 3.5}, rev, 90}); // to mid goal
  intakeState = IntakeState::midGoal;                // score mid goal
  pros::delay(10000);
}

void right7()
{
  chassis.odom_xyt_set(48_in, 11_in, 270_deg);
  chassis.pid_turn_set({22, 20}, fwd, TURN_SPEED);
  chassis.pid_wait_quick();
  intakeState = IntakeState::intake;
  chassis.pid_odom_set({{22, 20}, fwd, DRIVE_SPEED - 20});
  pros::delay(380);
  matchload.set_value(true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set({53, 45.5}, fwd, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_odom_set({{53, 45.5}, fwd, DRIVE_SPEED});
  chassis.pid_wait_quick();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(14.5_in, MATCHLOAD_SPEED+10, true);
  pros::delay(60);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-30_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-5_in, DRIVE_SPEED - 30, true);
  intakeState = IntakeState::highGoal;
  pros::delay(1500);
  chassis.pid_wait_quick();
}

void right_7ball()
{
  right7();
  chassis.pid_drive_set(-10_in, DRIVE_SPEED + 20, true);
  chassis.pid_wait_quick();
  setMatchload(true);
}

void right_horn()
{
  right7();
  chassis.odom_theta_set(0_deg);
  setMatchload(false);
  chassis.pid_drive_set(5_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, SWING_SPEED, 5, ez::ccw);
  chassis.pid_wait_quick_chain();
  horn.set_value(true);
  chassis.pid_drive_set(33, DRIVE_SPEED + 20, true);
  pros::delay(10000);
}

void awp()
{
   chassis.odom_xyt_set(48_in, 11_in, 0_deg);
  intakeState = IntakeState::intake;  
  chassis.pid_drive_set(30.5_in, DRIVE_SPEED+10, true);
  chassis.pid_wait_quick();
  matchload.set_value(true); 
  chassis.pid_turn_exit_condition_set(20_ms, 3_deg, 100_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_turn_set(90_deg, TURN_SPEED+10);
  chassis.pid_wait_quick();
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_set(10.5_in, DRIVE_SPEED, true);//intake matchload 
  chassis.pid_wait_quick(); 
  pros::delay(100); //matchlaod delay
  chassis.pid_drive_set(-30_in, DRIVE_SPEED, true);//back from matchload  
  pros::delay(600);
  intakeState = IntakeState::highGoal;
  chassis.pid_wait_quick();
  pros::delay(400);
  setMatchload(false);
  chassis.odom_xyt_set(29_in, 48_in, 90_deg);
  intakeState = IntakeState::intake;
  chassis.pid_drive_set(6_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::LEFT_SWING, 225_deg, SWING_SPEED, -20, ez::cw);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(25_in, DRIVE_SPEED, true); 
  chassis.pid_wait_until(19_in);
  matchload.set_value(true);
  pros::delay(200);
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  matchload.set_value(false);
  chassis.pid_drive_set(51_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(40_in);
  matchload.set_value(true);//next 3 ball
  chassis.pid_wait_quick();
  chassis.pid_turn_set(135, TURN_SPEED);//mid goal 
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-18_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();
  intakeState = IntakeState::midGoal;
  pros::delay(500);//score mid goal
  intakeState = IntakeState::intake;
  /*
  chassis.pid_turn_set({24, 24}, fwd, TURN_SPEED);
  chassis.pid_wait_quick(); 
  intakeState = IntakeState::intake;
  chassis.pid_odom_set({{24, 24}, fwd, DRIVE_SPEED_AWP});//go to 3 ball 
  chassis.pid_drive_set(29_in, DRIVE_SPEED, true);//3 ball faster
  pros::delay(500);
  setMatchload(true); 
  chassis.pid_wait_quick(); 
  chassis.pid_turn_set({24, -24}, fwd, TURN_SPEED);//next 3 ball
  chassis.pid_wait_quick(); 
  setMatchload(false);
  chassis.pid_drive_set(48_in, DRIVE_SPEED, true);
  pros::delay(500);//must tune delay for matchload 
  setMatchload(true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set({9, -9}, rev, TURN_SPEED);
  chassis.pid_wait_quick(); 
  setMatchload(false);
  chassis.pid_drive_set(-21_in, DRIVE_SPEED, true);//to mid goal 
  intakeState = IntakeState::midGoal;
  pros::delay(500);//score mid goal
  chassis.pid_wait_quick();
  //-------------------- distance reset 
  chassis.pid_turn_set({55, -40}, fwd, TURN_SPEED); 
  chassis.pid_wait_quick(); 
  chassis.pid_turn_set(0_deg, TURN_SPEED); //line up to wall for reset
  chassis.pid_wait_quick(); 
  distance_reset();
  setMatchload(true);
  chassis.pid_odom_set({{55, -47}, rev, DRIVE_SPEED}); //matchload at 48,-47
  chassis.pid_wait_quick();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait_quick();
    //--------------------
  chassis.pid_drive_set(14_in, MATCHLOAD_SPEED, true);//intake matchload  
  pros::delay(300); //matchlaod delay
  chassis.pid_drive_set(-35_in, DRIVE_SPEED, true);//back from matchload  
  chassis.pid_wait_quick_chain();
  intakeState = IntakeState::highGoal;
  chassis.pid_drive_set(-10_in, DRIVE_SPEED, true);//press against goal while scoring 
  chassis.pid_wait_quick();
  */
}

void skills()
{
  chassis.odom_xyt_set(-56, 0, 270);
  intakeState = IntakeState::intake;
  chassis.pid_drive_set(10_in, DRIVE_SPEED_SKILLS);  // clear park zone
  chassis.pid_wait();
  pros::delay(1000);
  chassis.pid_drive_set(-15_in, DRIVE_SPEED_SKILLS); // back out and turn 180 to reset
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED); // distance reset
  chassis.pid_wait();
  distance_reset();
  chassis.pid_odom_set({{-19, 0}, fwd, DRIVE_SPEED_SKILLS}); //move back
  chassis.pid_turn_set({-21, 21}, fwd, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(21_in, DRIVE_SPEED_SKILLS); // intake single blue block
  chassis.pid_wait();
  chassis.pid_turn_set({-9, 9}, rev, TURN_SPEED); // move to mid goal
  chassis.pid_wait();
  chassis.pid_drive_set(-17_in, DRIVE_SPEED_SKILLS);
  chassis.pid_wait_quick();
  intakeState = IntakeState::midGoal; // score in mid goal
  pros::delay(2000);
  intakeState = IntakeState::intake;
  chassis.pid_drive_set(50_in, DRIVE_SPEED); // move to matchload
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{-47, 47}, fwd, DRIVE_SPEED_SKILLS});
  chassis.pid_wait();
  chassis.pid_turn_set(270_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(19_in, MATCHLOAD_SPEED); // matchload
  chassis.pid_wait_quick();
  pros::delay(200);
  chassis.pid_drive_set(-26_in, MATCHLOAD_SPEED); // back out
  chassis.pid_wait_quick();
}
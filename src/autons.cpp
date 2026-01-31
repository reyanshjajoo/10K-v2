

#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 85;
const int MATCHLOAD_SPEED = 50;
const int TURN_SPEED = 80;
const int SWING_SPEED = 110;

///
// Constants
///

void default_constants()
{

  // pid
  chassis.pid_drive_constants_set(17.0, 0.0, 120.0);
  // chassis.pid_drive_constants_forward_set(17.0, 0.0, 120.0);
  // chassis.pid_drive_constants_backward_set(17.0, 0.0, 100.0);
  chassis.pid_heading_constants_set(11.0, 0.0, 30.0);           // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(2.5, 0.05, 20, 4.7);           // Turn in place constant
  chassis.pid_swing_constants_set(6.0, 0.0, 50.0);              // Swing constants
  chassis.pid_odom_angular_constants_set(2.83, 0.0, 25.8, 4.7); // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);     // TODO: TUNE THIS (idt we are using boomerang)Angular control for boomerang motions
  // Exit conditions

  chassis.pid_turn_exit_condition_set(80_ms, 3_deg, 200_ms, 7_deg, 400_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(20_ms, 3_deg, 250_ms, 7_deg, 300_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(70_ms, 1_in, 200_ms, 3_in, 400_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(65_ms, 3_deg, 200_ms, 7_deg, 400_ms, 500_ms);
  chassis.pid_odom_drive_exit_condition_set(70_ms, 1_in, 200_ms, 3_in, 400_ms, 500_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(4_in);

  chassis.odom_look_ahead_set(7_in);          // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in); // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);    // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest); // Changes the default behavior for turning, this defaults it to the shortest path there
}

void setMatchload(bool value)
{
  matchload.set_value(value);
  // apply_profile(value ? ChassisProfile::MatchloadDown : ChassisProfile::Normal);
}

// ----------------------------------------------------------------

pros::Distance dist_back(17);
pros::Distance dist_right(18);

constexpr double FIELD_SIZE_IN = 144.0;
constexpr double FIELD_HALF_IN = FIELD_SIZE_IN / 2;

constexpr double BACK_SENSOR_OFFSET_IN = 5.0;
constexpr double RIGHT_SENSOR_OFFSET_IN = 5.667;

inline double mm_to_in(double mm)
{
  return mm / 25.4;
}

void distance_reset()
{
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

void go_forward(){
  wing.set_value(false);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.pid_drive_set(5_in, DRIVE_SPEED);
}

void kaihan_counter()
{

chassis.odom_xyt_set(-48_in, 11_in, 0_deg);
  wing.set_value(true);
  intakeState = IntakeState::intake;
  matchload.set_value(true);
  chassis.pid_drive_set(34_in, DRIVE_SPEED, true);
  pros::delay(1000);
  chassis.pid_turn_exit_condition_set(20_ms, 3_deg, 100_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_exit_condition_set(80_ms, 3_deg, 200_ms, 7_deg, 400_ms, 500_ms);
  chassis.pid_drive_set(8.7_in, 55); // intake matchload
  chassis.pid_wait();
  pros::delay(140);                             // matchlaod delay
  chassis.pid_drive_set(-33_in, DRIVE_SPEED); // back from matchload
    pros::delay(900);
  intakeState = IntakeState::highGoal;
  matchload.set_value(false);
  pros::delay(400);
  intakeState = IntakeState::intake;
  pros::delay(400);
  chassis.pid_swing_set(ez::RIGHT_SWING, 145_deg, SWING_SPEED, -60, ez::ccw);
  pros::delay(800);
  chassis.pid_wait();
  intakeState = IntakeState::intake; // later to outtake balls
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  pros::delay(200);
  chassis.pid_drive_set(14.5_in, DRIVE_SPEED, true);
  pros::delay(150);
  matchload.set_value(true);
  pros::delay(500);
  chassis.pid_turn_set(-48,TURN_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-21.5_in, DRIVE_SPEED, true);
  pros::delay(600);
  intakeState = IntakeState::midGoal;           // score mid goal
  pros::delay(2000);
  intakeState = IntakeState::intake;
  wing.set_value(false);
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
  pros::delay(1000);
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_BRAKE);
  matchload.set_value(false);

  chassis.pid_drive_set(4_in, DRIVE_SPEED);
  pros::delay(200);
  chassis.pid_drive_set(-6_in, DRIVE_SPEED);
  pros::delay(400);


  chassis.pid_drive_set(42_in, DRIVE_SPEED, true);
  pros::delay(1200);
  chassis.pid_turn_set(85_deg, TURN_SPEED);
  pros::delay(500);
  wing.set_value(false);
  chassis.pid_drive_set(29_in, DRIVE_SPEED, true);
  pros::delay(1000);
}


void left_3_4()
{

  chassis.odom_xyt_set(-48_in, 11_in, 0_deg);
  wing.set_value(true);
  intakeState = IntakeState::intake;
  matchload.set_value(true);
  chassis.pid_drive_set(34_in, DRIVE_SPEED, true);
  pros::delay(1000);
  chassis.pid_turn_exit_condition_set(20_ms, 3_deg, 100_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_exit_condition_set(80_ms, 3_deg, 200_ms, 7_deg, 400_ms, 500_ms);
  chassis.pid_drive_set(8.7_in, 55); // intake matchload
  chassis.pid_wait();
  pros::delay(140);                             // matchlaod delay
  chassis.pid_drive_set(-33_in, DRIVE_SPEED); // back from matchload
    pros::delay(900);
  intakeState = IntakeState::highGoal;
  matchload.set_value(false);
  pros::delay(400);
 pros::delay(550);
  intakeState = IntakeState::intake;
  chassis.pid_swing_set(ez::RIGHT_SWING, 145_deg, SWING_SPEED, -60, ez::ccw);
  pros::delay(800);
  chassis.pid_wait();
  intakeState = IntakeState::intake; // later to outtake balls
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  pros::delay(200);
  chassis.pid_drive_set(14.5_in, DRIVE_SPEED, true);
  pros::delay(150);
  matchload.set_value(true);
  pros::delay(500);
  chassis.pid_turn_set(-48,TURN_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-21.5_in, DRIVE_SPEED, true);
  pros::delay(600);
  intakeState = IntakeState::midGoal;           // score mid goal
  pros::delay(1000);
  intakeState = IntakeState::intake;
    matchload.set_value(false);

  chassis.pid_drive_set(4_in, DRIVE_SPEED);
  pros::delay(200);
  chassis.pid_drive_set(-6_in, DRIVE_SPEED);
  pros::delay(400);


  chassis.pid_drive_set(42_in, DRIVE_SPEED, true);
  pros::delay(1200);
  chassis.pid_turn_set(85_deg, TURN_SPEED);
  pros::delay(500);
  wing.set_value(false);
  chassis.pid_drive_set(29_in, DRIVE_SPEED, true);
  pros::delay(1000);
  /*
    chassis.pid_turn_set(120_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  */

  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);  
  

  
}

void right7()
{
  chassis.odom_xyt_set(48_in, 11_in, 270_deg);
  chassis.pid_turn_set({20, 24}, fwd, TURN_SPEED);
  //pros::delay(400);
  chassis.pid_wait();
  intakeState = IntakeState::intake;
  chassis.pid_odom_ptp_set({{20, 24}, fwd, DRIVE_SPEED});
  chassis.pid_wait_until(16_in);
  matchload.set_value(true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set({50, 45}, fwd, TURN_SPEED);
  //chassis.pid_wait_quick();
  chassis.pid_odom_set({{50, 45}, fwd, DRIVE_SPEED});
  chassis.pid_wait_quick();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  pros::delay(400);
  chassis.pid_drive_set(12_in, DRIVE_SPEED);
  pros::delay(900);
  intakeState = IntakeState::idle;
  chassis.pid_drive_set(-33_in, DRIVE_SPEED);
  pros::delay(800);
  chassis.drive_set(-127, -127);
  matchload.set_value(false);
  intakeState = IntakeState::highGoal;
  pros::delay(1700);
  chassis.pid_wait_quick();
  intakeState = IntakeState::intake;
}

void right_7ball()
{
  right7();
  chassis.pid_drive_set(-10_in, DRIVE_SPEED + 20, true);
  chassis.pid_wait_quick();
  setMatchload(true);
}

void right_wing()
{
  right7();
  chassis.odom_theta_set(0_deg);
  setMatchload(false);
  chassis.pid_drive_set(2_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, SWING_SPEED+40, 0, ez::ccw);
  chassis.pid_wait_quick_chain();
  wing.set_value(false);
  chassis.pid_drive_set(26_in, DRIVE_SPEED-30, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(225_deg, TURN_SPEED);
  pros::delay(500);
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
}

void right_six_ball_rush()
{
  wing.set_value(true);
  chassis.odom_xyt_set(48_in, 11_in, 270_deg);
  chassis.pid_turn_set({20, 24}, fwd, TURN_SPEED);
  //pros::delay(300);
  intakeState = IntakeState::intake;
  chassis.pid_odom_ptp_set({{20, 24}, fwd, DRIVE_SPEED-20});
  chassis.pid_wait_until(16_in);
  matchload.set_value(true);
  pros::delay(300);
  chassis.pid_turn_set({2, 47}, fwd, TURN_SPEED);
  chassis.pid_drive_set(20_in, DRIVE_SPEED);
  chassis.pid_wait_until(2_in);
  matchload.set_value(false);
  pros::delay(600);
  chassis.pid_turn_set(265_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.drive_set(-127, -127);
  pros::delay(1000);
  intakeState = IntakeState::highGoal;
  pros::delay(1700);
  chassis.odom_theta_set(0_deg);
  setMatchload(false);
  chassis.pid_drive_set(2_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, SWING_SPEED+40, 0, ez::ccw);
  chassis.pid_wait_quick_chain();
  wing.set_value(false);
  chassis.pid_drive_set(26_in, DRIVE_SPEED-30, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(225_deg, TURN_SPEED);
  pros::delay(500);
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
}

void awp()
{
  chassis.pid_turn_exit_condition_set(20_ms, 3_deg, 100_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(20_ms, 1_in, 100_ms, 4_in, 400_ms, 500_ms);
  chassis.odom_xyt_set(48_in, 11_in, 0_deg);
  intakeState = IntakeState::intake;
  matchload.set_value(true);
  wing.set_value(true);
  chassis.pid_drive_set(33_in, DRIVE_SPEED , true);
  pros::delay(1100);
  chassis.pid_turn_exit_condition_set(20_ms, 3_deg, 100_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_turn_set(90_deg, TURN_SPEED + 10);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_exit_condition_set(80_ms, 3_deg, 200_ms, 7_deg, 400_ms, 500_ms);
  chassis.pid_drive_set(8.5_in, 60, true); // intake matchload
  chassis.pid_wait();
  pros::delay(100);                             // matchlaod delay
  chassis.pid_drive_set(-31_in, DRIVE_SPEED); // back from matchload
  pros::delay(900);
  intakeState = IntakeState::highGoal;
  chassis.drive_set(-127, -127);
  pros::delay(700);

  chassis.odom_xyt_set(29_in, 48_in,90_deg);

  matchload.set_value(false);
  wing.set_value(true);
  chassis.pid_turn_set(215_deg, TURN_SPEED);
  pros::delay(1000);
  intakeState = IntakeState::intake; // later to outtake balls
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  pros::delay(300);
  chassis.pid_drive_set(62.5_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(40_in);
  matchload.set_value(true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(135, TURN_SPEED); // mid goal
  pros::delay(400);
  chassis.pid_drive_set(-22_in, DRIVE_SPEED-10, true);
  pros::delay(500); // tuen
  intakeState = IntakeState::midGoal;
  pros::delay(1100); // score mid goal
  intakeState = IntakeState::idle;
  pros::delay(100);
  matchload.set_value(false);
  chassis.pid_turn_set(140, TURN_SPEED);
  
  chassis.pid_drive_set(53_in, DRIVE_SPEED, true);
  matchload.set_value(true);
  chassis.pid_wait_quick();
  
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  intakeState = IntakeState::intake;
  pros::delay(350); 

  chassis.pid_drive_set(15_in, 55, true); // matchload
  pros::delay(1000);                           // matchlaod delay
  chassis.pid_drive_set(-36_in, DRIVE_SPEED); // back from matchload
  pros::delay(800);
  intakeState = IntakeState::highGoal;
  matchload.set_value(false);
  

  
  
  
}

void skills()
{
  // ? MID GOAL
  chassis.pid_turn_exit_condition_set(20_ms, 3_deg, 100_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(20_ms, 1_in, 100_ms, 4_in, 400_ms, 500_ms);
  chassis.odom_xyt_set(-44, 0, 270);
  wing.set_value(true);
  int rightDistance = mm_to_in(dist_right.get());
  intakeState = IntakeState::intake;
  chassis.pid_drive_set(6_in, DRIVE_SPEED-30);
  pros::delay(400);
   chassis.pid_drive_set(-3_in, DRIVE_SPEED-20);
  pros::delay(400);
   chassis.pid_drive_set(7_in, DRIVE_SPEED);
  pros::delay(600);
  chassis.pid_drive_set(-24_in, 127);
  chassis.pid_wait();
  chassis.pid_drive_set(9_in, 30);
  chassis.pid_wait_quick();
  
  int finalDistance = mm_to_in(dist_right.get());
  int distanceDiff = rightDistance - finalDistance;
  chassis.odom_xyt_set(-44, distanceDiff, 270);//:)
  intakeState = IntakeState::intake;
  
  chassis.pid_drive_set(-5_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set({{-17, 0}, rev, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_turn_set(0, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(18_in, 40); // intake single blue block
  chassis.pid_wait();
  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-14.5_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-48_deg, TURN_SPEED);
  pros::delay(50);
  chassis.pid_turn_set(-43_deg, TURN_SPEED);
  pros::delay(50);
  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.drive_set(-35, -35);

  intakeState = IntakeState::midGoalSkills;
  pros::delay(700);

  intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  intakeState = IntakeState::idle;
  pros::delay(200);

  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intakeState = IntakeState::midGoalAuto;
  pros::delay(1700);

  intakeState = IntakeState::midGoalSkills;
  pros::delay(300);

  chassis.pid_drive_set(57_in, DRIVE_SPEED); // move to matchload
  pros::delay(300);
  intakeState = IntakeState::intake;
  chassis.pid_wait();
  chassis.pid_turn_set(270_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  matchload.set_value(true);
  chassis.pid_drive_set(-19_in, DRIVE_SPEED); // matchload
  pros::delay(600);
  intakeState = IntakeState::highGoal;
  chassis.drive_set(-127, -127);
  pros::delay(1000);
  // ? FIRST QUADRANT
  intakeState = IntakeState::intake;
  wing.set_value(true);
  matchload.set_value(true);
  chassis.odom_xyt_set(-29_in, 48_in, 270_deg);
  chassis.pid_drive_set(33_in, 55); // intake matchload
  chassis.pid_wait();
  chassis.drive_set(30, 30);
  pros::delay(1000);                             // matchlaod delay
  chassis.pid_drive_set(-10.5_in, DRIVE_SPEED); // back from matchload
  chassis.pid_wait();
  chassis.pid_turn_set({-30, 60}, rev, TURN_SPEED);
  pros::delay(400);
  chassis.pid_drive_set(-22_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  pros::delay(400);
  chassis.pid_drive_set(-66_in, DRIVE_SPEED);
  pros::delay(200);
  matchload.set_value(false);
  chassis.pid_wait();
  // chassis.pid_turn_set(0_deg, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // chassis.pid_turn_set(90_deg, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-20_in, DRIVE_SPEED);
  // pros::delay(500);
  chassis.pid_swing_set(ez::RIGHT_SWING, 90_deg, -SWING_SPEED, 5, ez::cw);
  chassis.pid_wait();
  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  pros::delay(300);
  intakeState = IntakeState::highGoal;
  chassis.drive_set(-127, -127);
  pros::delay(1700);
  intakeState = IntakeState::intake;
  matchload.set_value(true);
  chassis.pid_drive_set(34_in, 55);
  chassis.pid_wait_quick();
  chassis.drive_set(30, 30);
  pros::delay(700);                             // matchlaod delay
  chassis.pid_drive_set(-32_in, DRIVE_SPEED-20);//too much delay not 33
  pros::delay(600);
  matchload.set_value(false);
  intakeState = IntakeState::highGoal;
  chassis.drive_set(-127, -127);
  pros::delay(2000);
  // ? SECOND QUADRANT
  chassis.odom_xyt_set(29_in, 48_in, 90_deg);
  intakeState = IntakeState::intake;
  wing.set_value(true);
  chassis.pid_drive_set(5_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set({61, 26}, fwd, TURN_SPEED);
  chassis.pid_odom_ptp_set({{59, 26}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_swing_set(ez::LEFT_SWING, 180_deg, SWING_SPEED, 40, ez::cw);
  matchload.set_value(true);
  chassis.pid_wait();
  chassis.drive_set(127, 127);
  pros::delay(550);
  matchload.set_value(false);
  pros::delay(800);
  chassis.pid_drive_set(-17_in, 40);
  chassis.pid_wait();
  matchload.set_value(false);
  chassis.pid_drive_set(5_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_swing_set(ez::LEFT_SWING, 210_deg, SWING_SPEED, 15, ez::cw);
  chassis.pid_wait();
  chassis.pid_drive_set(23_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-25_in, DRIVE_SPEED);
  pros::delay(400);
  intakeState = IntakeState::highGoal;//2nd high goal 
  chassis.drive_set(-127, -127);
  pros::delay(1200);
  wing.set_value(true);
  intakeState = IntakeState::intake;
  chassis.odom_theta_set(90_deg);
  matchload.set_value(true);
  chassis.pid_drive_set(33_in, 55);
  chassis.pid_wait();
  chassis.drive_set(30, 30);
  pros::delay(1400);                             // matchlaod delay
  chassis.pid_drive_set(-10_in, DRIVE_SPEED); // back from matchload
  chassis.pid_wait();
  chassis.pid_turn_set(50_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-17_in, DRIVE_SPEED);//
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-70_in, DRIVE_SPEED);
  pros::delay(200);
  matchload.set_value(false);
  chassis.pid_wait();
  // chassis.pid_turn_set(180_deg, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // chassis.pid_turn_set(-90_deg, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-15_in, DRIVE_SPEED);
  // chassis.pid_wait();
  chassis.pid_swing_set(ez::RIGHT_SWING, 270_deg, -SWING_SPEED, 5, ez::cw);
  chassis.pid_wait();
  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  pros::delay(300);
  intakeState = IntakeState::highGoal;
  chassis.drive_set(-127, -127);
  pros::delay(2300);
  intakeState = IntakeState::intake;
  matchload.set_value(true);
  chassis.pid_drive_set(33.7_in, 55);
  chassis.pid_wait();
  chassis.drive_set(30, 30);
  pros::delay(1000);
  chassis.pid_drive_set(-33_in, DRIVE_SPEED);
  pros::delay(650);
  matchload.set_value(false);
  intakeState = IntakeState::highGoal;
  chassis.drive_set(-127, -127);
  pros::delay(1800);
  chassis.odom_theta_set(-90_deg);
  intakeState = IntakeState::highGoal;
  chassis.pid_drive_set(5_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(46_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.drive_set(127, 127);
  pros::delay(1000);
  chassis.drive_set(0, 0);
  intakeState = IntakeState::idle;

  // ? old below
  // pros::delay(600);
  // chassis.pid_drive_set(-16_in, MATCHLOAD_SPEED); // back out
  // chassis.pid_wait_quick();
  // matchload.set_value(false);
  // chassis.pid_turn_set(-150_deg, TURN_SPEED);
  // chassis.pid_wait_quick();
  // chassis.pid_drive_set(-14_in, DRIVE_SPEED);
  // chassis.pid_wait_quick();
  // chassis.pid_turn_set(270_deg, TURN_SPEED);
  // chassis.pid_wait_quick();
  // chassis.pid_drive_set(-80_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // chassis.pid_swing_set(ez::RIGHT_SWING, -270_deg, SWING_SPEED, -29, ez::cw);//tune this swing 
  // chassis.pid_wait_quick();
  // chassis.pid_wait(); 
  // chassis.pid_drive_set(-24_in, 127);
  // pros::delay(400);
  // intakeState = IntakeState::highGoal;
  // pros::delay(1000);

  // // p2
  // chassis.odom_xyt_set(29_in, 48_in, 90_deg); 
  // intakeState = IntakeState::intake;
  // matchload.set_value(true);
  // chassis.pid_drive_set(30_in, DRIVE_SPEED-20);
  // chassis.pid_wait();
  // pros::delay(900);
  // chassis.pid_drive_set(-30_in, DRIVE_SPEED+30);
  // chassis.pid_wait();
  // matchload.set_value(false);
  // intakeState = IntakeState::highGoal;
  // pros::delay(1500);
  // intakeState = IntakeState::intake;
  // chassis.odom_xyt_set(29_in, 48_in, 90_deg);//done 2nd quad 


  // intakeState = IntakeState::intake;
  // chassis.pid_drive_set(12_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // chassis.pid_turn_set(180_deg, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(66_in, DRIVE_SPEED, true);
  // pros::delay(1000);
  // chassis.pid_odom_set({{ 41, -50.5}, fwd, DRIVE_SPEED}, true);//go to quad 3 


  // chassis.pid_wait();
  // chassis.pid_turn_set(90_deg, TURN_SPEED);
  // chassis.pid_wait();
  // matchload.set_value(true);
  // pros::delay(200);
  // chassis.pid_drive_set(25_in, MATCHLOAD_SPEED);  //60 x 
  // chassis.pid_wait();
  // pros::delay(1000);//matchload delay 
  // chassis.pid_drive_set(-31_in, MATCHLOAD_SPEED); // back out
  // chassis.pid_wait();
  // intakeState = IntakeState::highGoal;
  // matchload.set_value(false);
  // pros::delay(1000);//scoring
  // intakeState = IntakeState::idle;    
  // chassis.pid_swing_set(ez::LEFT_SWING, -90_deg, SWING_SPEED, -29, ez::cw);//to alley swing 
  // chassis.pid_wait(); 
  // /*
  //   chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 67, ez::cw);//position clear zone 
  // chassis.pid_drive_set(69_in, 127);
  // chassis.pid_wait_quick();
  // chassis.pid_swing_set(ez::LEFT_SWING, 40_deg, SWING_SPEED, 10, ez::cw);
  // chassis.pid_wait_quick(); 
  // */


  // stupid ass temp skills
  // wing.set_value(true);
  // chassis.odom_xyt_set(-48_in, 11_in, 0_deg);
  // intakeState = IntakeState::intake;
  // matchload.set_value(true);
  // chassis.pid_drive_set(31.5_in, DRIVE_SPEED, true);
  // chassis.pid_wait();
  // chassis.pid_turn_set(-90_deg, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(15_in, 55); // intake matchload
  // chassis.pid_wait();
  // chassis.drive_set(30, 30);
  // pros::delay(1800);                             // matchlaod delay
  // chassis.pid_drive_set(-10.5_in, DRIVE_SPEED); // back from matchload
  // chassis.pid_wait();
  // chassis.pid_turn_set({-27, 58}, rev, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-22_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // chassis.pid_turn_set(-90_deg, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-75_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // matchload.set_value(false);
  // pros::delay(500);
  // chassis.pid_turn_set(0_deg, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // chassis.pid_turn_set(90_deg, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-15_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // intakeState = IntakeState::highGoal;
  // chassis.drive_set(-127, -127);
  // pros::delay(2500);
  // intakeState = IntakeState::intake;
  // matchload.set_value(true);
  // chassis.pid_drive_set(33_in, 55);
  // chassis.pid_wait();
  // chassis.drive_set(30, 30);
  // pros::delay(1800);
  // chassis.pid_drive_set(-33_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // matchload.set_value(false);
  // intakeState = IntakeState::highGoal;
  // chassis.drive_set(-127, -127);
  // pros::delay(2500);
  // wing.set_value(true);
  // intakeState = IntakeState::intake;
  // chassis.odom_theta_set(90_deg);
  // chassis.pid_drive_set(5_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-5_in, 40);
  // chassis.pid_wait();
  // chassis.pid_drive_set(15_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // chassis.pid_turn_set(180_deg, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(98_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // chassis.pid_turn_set(90_deg, TURN_SPEED);
  // chassis.pid_wait();
  // matchload.set_value(true);
  // chassis.pid_drive_set(25_in, 55);
  // chassis.pid_wait();
  // chassis.drive_set(30, 30);
  // //??
  // pros::delay(1800);                             // matchlaod delay
  // chassis.pid_drive_set(-10_in, DRIVE_SPEED); // back from matchload
  // chassis.pid_wait();
  // chassis.pid_turn_set(50_deg, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-24_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // chassis.pid_turn_set(90_deg, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-75_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // matchload.set_value(false);
  // pros::delay(500);
  // chassis.pid_turn_set(180_deg, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-13_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // chassis.pid_turn_set(-90_deg, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-15_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // intakeState = IntakeState::highGoal;
  // chassis.drive_set(-127, -127);
  // pros::delay(2500);
  // intakeState = IntakeState::intake;
  // matchload.set_value(true);
  // chassis.pid_drive_set(33_in, 55);
  // chassis.pid_wait();
  // chassis.drive_set(30, 30);
  // pros::delay(1800);
  // chassis.pid_drive_set(-33_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // matchload.set_value(false);
  // intakeState = IntakeState::highGoal;
  // chassis.drive_set(-127, -127);
  // pros::delay(2500);
  // intakeState = IntakeState::idle;
  // chassis.pid_drive_set(5_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-5_in, 40);
  // chassis.pid_wait();
  // chassis.odom_theta_set(-90_deg);
  // chassis.pid_drive_set(8_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // chassis.pid_turn_set(-40_deg, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(40_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // //chassis.pid_turn_set(-5_deg, TURN_SPEED);
  // chassis.pid_swing_set(ez::LEFT_SWING, -5_deg, SWING_SPEED, 0, ez::cw);
  // chassis.pid_wait();
  // matchload.set_value(true);
  // pros::delay(500);
  // intakeState = IntakeState::highGoal;
  // chassis.drive_set(127, 127);
  // pros::delay(700);
  // chassis.drive_set(0, 0);
  // matchload.set_value(false);
  // chassis.pid_drive_set(3_in, 55);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-3_in, 55);
}
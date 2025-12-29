

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
enum class ChassisProfile { Normal, MatchloadDown };

static ChassisProfile currentProfile = ChassisProfile::Normal;

void default_constants()
{
  // Exit conditions
  chassis.pid_turn_exit_condition_set(40_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set (90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  chassis.odom_look_ahead_set(7_in);          // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in); // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);    // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest); // Changes the default behavior for turning, this defaults it to the shortest path there
}

void matchload_constants() { 
  chassis.pid_drive_constants_set(16.7, 0.0, 106.5);
  chassis.pid_heading_constants_set(11.0, 0.0, 30.0);       // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0); 
}

void apply_profile(ChassisProfile p) {
  if (p == currentProfile) return;   // dont reapply 
  switch (p) {
    case ChassisProfile::Normal: 
      // P, I, D, and Start I
      //chassis.pid_drive_constants_set(16.7, 0.0, 103);        // Fwd/rev constants, used for odom and non odom motions
      chassis.pid_drive_constants_forward_set(17.0, 0.0, 120.0);
      chassis.pid_drive_constants_backward_set(17.0, 0.0, 100.0);
      chassis.pid_heading_constants_set(11.0, 0.0, 30.0);       // Holds the robot straight while going forward without odom
      chassis.pid_turn_constants_set(2.83, 0.0, 25.8, 4.7);    // Turn in place constant
      chassis.pid_swing_constants_set(6.0, 0.0, 50.0);          // Swing constants
      chassis.pid_odom_angular_constants_set(2.83, 0.0, 25.8, 4.7);   // Angular control for odom motions
      chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5); // TODO: TUNE THIS (idt we are using boomerang)Angular control for boomerang motions
      break;

    case ChassisProfile::MatchloadDown: //only need to change drive forward and turn
        // P, I, D, and Start I
      chassis.pid_drive_constants_forward_set(17.0, 0.0, 120.0);//TODO tune matchload
      chassis.pid_drive_constants_backward_set(17.0, 0.0, 100.0);
      chassis.pid_turn_constants_set(2.83, 0.0, 25.8, 4.7);  
      chassis.pid_odom_angular_constants_set(2.83, 0.0, 25.8, 4.7); 
      break;
  }

  currentProfile = p;
}
void setMatchload(bool value) { 
  matchload.set_value(value); 
  apply_profile(value ? ChassisProfile::MatchloadDown : ChassisProfile::Normal);
}

// ----------------------------------------------------------------

pros::Distance dist_back(13);
pros::Distance dist_right(14);

constexpr double FIELD_SIZE_IN = 144.0;
constexpr double FIELD_HALF_IN = FIELD_SIZE_IN/2;

constexpr double BACK_SENSOR_OFFSET_IN = 7.0;
constexpr double RIGHT_SENSOR_OFFSET_IN = 6.0;

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
  chassis.odom_xyt_set(48_in, 11_in, 270_deg);
  chassis.pid_drive_set(7_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set({48, 46}, fwd, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(36_in, DRIVE_SPEED, true);//drive to 48,46 matchload 
  chassis.pid_wait_quick();
  chassis.pid_turn_set(90_deg, fwd, TURN_SPEED);
  intakeState = IntakeState::intake;  
  chassis.pid_wait_quick();
  setMatchload(true);
  chassis.pid_drive_set(14_in, MATCHLOAD_SPEED, true);//intake matchload  
  pros::delay(300); //matchlaod delay
  chassis.pid_drive_set(-35_in, DRIVE_SPEED, true);//back from matchload  
  chassis.pid_wait_quick_chain();
  intakeState = IntakeState::highGoal;
  chassis.pid_drive_set(-10_in, DRIVE_SPEED, true);//press against goal while scoring 
  chassis.pid_wait_quick();
  setMatchload(false);
  pros::delay(1000);
  chassis.pid_drive_set(15_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick(); 
  chassis.pid_turn_set({24, 24}, fwd, TURN_SPEED);
  chassis.pid_wait_quick(); 
  intakeState = IntakeState::intake;
  //chassis.pid_odom_set({{24, 24}, fwd, DRIVE_SPEED_AWP});//go to 3 ball 
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
  chassis.pid_drive_set(54_in, DRIVE_SPEED, true);//cordinate 48,-47
  chassis.pid_wait_quick(); 
  chassis.pid_turn_set(90_deg, fwd, TURN_SPEED);
  setMatchload(true);
  chassis.pid_drive_set(14_in, MATCHLOAD_SPEED, true);//intake matchload  
  pros::delay(300); //matchlaod delay
  chassis.pid_drive_set(-35_in, DRIVE_SPEED, true);//back from matchload  
  chassis.pid_wait_quick_chain();
  intakeState = IntakeState::highGoal;
  chassis.pid_drive_set(-10_in, DRIVE_SPEED, true);//press against goal while scoring 
  chassis.pid_wait_quick();

}

void skills()
{
  chassis.odom_xyt_set(-48_in, 16_in, 0_deg);
  chassis.pid_drive_set(30.5_in, DRIVE_SPEED_SKILLS, true);
  setMatchload(true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set({-60, 46.5}, fwd, TURN_SPEED);
  chassis.pid_wait_quick();
  intakeState = IntakeState::intake;
  chassis.pid_drive_set(11_in, MATCHLOAD_SPEED, true);
  chassis.pid_wait_quick();
  pros::delay(1500); // mathcload delay
  chassis.pid_drive_set(-10_in, DRIVE_SPEED_SKILLS, true);
  intakeState = IntakeState::idle;
  setMatchload(false);
  chassis.pid_wait_quick();
  chassis.pid_turn_set({-24, 57}, rev, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_odom_set({{-24, 57}, rev, DRIVE_SPEED_SKILLS}); // to alley pose
  chassis.pid_wait_quick();
  chassis.pid_turn_set({34, 57}, rev, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-63_in, DRIVE_SPEED_SKILLS, true); // alley
  chassis.pid_wait_quick();
  chassis.pid_swing_set(ez::RIGHT_SWING, 90_deg, SWING_SPEED, 0);
  chassis.pid_wait_quick();
  // chassis.pid_turn_set({27, 48}, rev, TURN_SPEED);//turn to scoring
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-15_in, DRIVE_SPEED_SKILLS, true); // scoring pose
  chassis.pid_wait_quick();
  intakeState = IntakeState::highGoal;
  pros::delay(2000);
  chassis.pid_wait_quick();
  setMatchload(true);
  intakeState = IntakeState::intake;
  chassis.pid_drive_set(32_in, MATCHLOAD_SPEED, true); // matchload next
  chassis.pid_wait_quick();
  pros::delay(300);
  chassis.pid_drive_set(-32_in, DRIVE_SPEED_SKILLS, true);
  chassis.pid_wait_quick();
  intakeState = IntakeState::highGoal;
  pros::delay(2000);
  chassis.pid_drive_set(16_in, DRIVE_SPEED_SKILLS, true);
  intakeState = IntakeState::intake;
  setMatchload(false);
  chassis.pid_wait_quick();
  chassis.pid_turn_set({45, -52}, fwd, DRIVE_SPEED_SKILLS); // turn to next side
  chassis.pid_wait_quick();
  chassis.pid_odom_set({{45, -52}, fwd, DRIVE_SPEED_SKILLS}); // move to next side
  chassis.pid_wait_quick();
  setMatchload(true);
  chassis.pid_turn_set({62, -52}, fwd, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_odom_set({{62, -52}, fwd, MATCHLOAD_SPEED}); // matchload
  chassis.pid_wait_quick();
  pros::delay(1000);                                    // intake matchload
  chassis.pid_drive_set(-18, DRIVE_SPEED_SKILLS, true); // back from matchload
  setMatchload(false);
  chassis.pid_wait_quick();
  chassis.odom_theta_set(0_deg); // reset pose at scoring position
  chassis.pid_swing_set(ez::LEFT_SWING, 180_deg, SWING_SPEED, 8);
  chassis.pid_wait_quick();
  chassis.pid_odom_set({{-36_in, -60_in}, fwd, DRIVE_SPEED}); // alley
  chassis.pid_wait_quick();
    chassis.pid_turn_set(270_deg, fwd, DRIVE_SPEED_SKILLS);
    chassis.pid_wait_quick();
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(180_deg, fwd);
  chassis.pid_drive_set(-10_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  intakeState = IntakeState::highGoal;//score second high goal 
  chassis.pid_drive_set(-5, DRIVE_SPEED - 30, true);
  pros::delay(2000);
  intakeState = IntakeState::intake;
  chassis.pid_wait_quick();
  setMatchload(true);
  chassis.pid_drive_set(32_in, MATCHLOAD_SPEED, true);
  chassis.pid_wait_quick();
  pros::delay(1000);//intake matchload
  chassis.pid_drive_set(-32_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  intakeState = IntakeState::highGoal;
  chassis.pid_drive_set(-5, DRIVE_SPEED - 30, true);
  pros::delay(5000);
  
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
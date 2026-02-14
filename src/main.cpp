#include "main.h"

// init vars
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {1, -2, -11}, // Left Chassis Ports (negative port will reverse it!)
    {9, -10, 20}, // Right Chassis Ports (negative port will reverse it!)

    21,   // IMU Port
    3.25, // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450); // Wheel RPM = cartridge * (motor gear / wheel gear)

IntakeState intakeState = IntakeState::idle;
bool drive_arcade = false;

void drive_mode_task()
{
  while (true)
  {
    if (master.get_digital_new_press(DIGITAL_UP))
    {
      drive_arcade = !drive_arcade;
      // master.rumble(drive_arcade ? "." : "..");

      // Update controller screen when drive mode changes. Controller updates are slow,
      // so only update on changes (we're doing that here).
      // master.print(0, 0, "%s", drive_arcade ? "Drive: Arcade" : "Drive: Tank");
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}

/**
 * Printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line)
{
  std::string tracker_value = "", tracker_width = "";
  if (tracker != nullptr)
  {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());            // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get()); // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line); // Print final tracker text
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task()
{
  while (true)
  {
    if (!pros::competition::is_connected())
    {
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled())
      {
        if (ez::as::page_blank_is_on(0))
        {
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    else
    {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing UP and X together
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras()
{
  if (!pros::competition::is_connected())
  {
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    if (master.get_digital(DIGITAL_A) && master.get_digital(DIGITAL_LEFT))
    {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }
    chassis.pid_tuner_iterate();
  }
  else
  {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

void shooter_task()
{
  while (true)
  {
    switch (intakeState)
    {
    case IntakeState::midGoal:
      intake.move(127); // intake forward //TODO FOR SKILLS 80 //TODO FOR MATCH 127
      midGoalPiston.set_value(true);
      blockerPiston.set_value(true);
      break;

    case IntakeState::midGoalSkills:
      intake.move(95);
      midGoalPiston.set_value(true);
      blockerPiston.set_value(true);
      break;

    case IntakeState::midGoalMax:
      intake.move(127);
      midGoalPiston.set_value(true);
      blockerPiston.set_value(true);
      break;

    case IntakeState::midGoalAuto:
      intake.move(75);
      midGoalPiston.set_value(true);
      blockerPiston.set_value(true);
      break;

    case IntakeState::highGoal:
      intake.move(127); // intake forward
      midGoalPiston.set_value(false);
      blockerPiston.set_value(false);
      break;

    case IntakeState::intake:
      intake.move(127); // intake forward
      midGoalPiston.set_value(false);
      blockerPiston.set_value(true);

      break;

    case IntakeState::reverse:
      intake.move(-65); // intake backward
      midGoalPiston.set_value(false);
      blockerPiston.set_value(true);

      break;
    case IntakeState::outtakeMid:
      intake.move(-65); // intake backward
      midGoalPiston.set_value(true);
      blockerPiston.set_value(false);
      break;
    case IntakeState::idle:
    default:
      intake.move(0);
      midGoalPiston.set_value(false);
      blockerPiston.set_value(true);
      break;
    }

    pros::delay(10);
  }
}

// void shooter_task()
// {
//   static bool antiJamActive = false;
//   static uint32_t antiJamEnd = 0;

//   static bool wasHighGoal = false;
//   static uint32_t highGoalStart = 0;
//   static int lowRpmMs = 0;

//   while (true)
//   {
//     int intakeCmd = 0;
//     bool midPiston = false;
//     bool blocker = true;

//     switch (intakeState)
//     {
//       case IntakeState::highGoal:
//         intakeCmd = 127;
//         midPiston = false;
//         blocker = false;
//         break;

//       case IntakeState::midGoal:
//         intakeCmd = 80;
//         midPiston = true;
//         blocker = true;
//         break;

//       case IntakeState::midGoalSkills:
//         intakeCmd = 95;
//         midPiston = true;
//         blocker = true;
//         break;

//       case IntakeState::midGoalMax:
//         intakeCmd = 127;
//         midPiston = true;
//         blocker = true;
//         break;

//       case IntakeState::midGoalAuto:
//         intakeCmd = 75;
//         midPiston = true;
//         blocker = true;
//         break;

//       case IntakeState::intake:
//         intakeCmd = 127;
//         midPiston = false;
//         blocker = true;
//         break;

//       case IntakeState::reverse:
//         intakeCmd = -65;
//         midPiston = false;
//         blocker = true;
//         break;

//       case IntakeState::outtakeMid:
//         intakeCmd = -65;
//         midPiston = true;
//         blocker = false;
//         break;

//       case IntakeState::idle:
//       default:
//         intakeCmd = 0;
//         midPiston = false;
//         blocker = true;
//         break;
//     }

//     midGoalPiston.set_value(midPiston);
//     blockerPiston.set_value(blocker);

//     uint32_t now = pros::millis();
//     bool isHighGoal = (intakeState == IntakeState::highGoal);

//     // Entering high goal
//     if (isHighGoal && !wasHighGoal)
//     {
//       highGoalStart = now;
//       lowRpmMs = 0;
//     }

//     if (!isHighGoal)
//       lowRpmMs = 0;

//     // -------- High goal anti-jam --------
//     if (isHighGoal && !antiJamActive && (now - highGoalStart) > 500)
//     {
//       int rpm = std::abs(intake.get_actual_velocity());

//       if (rpm < 70) lowRpmMs += 10;
//       else lowRpmMs = 0;

//       if (lowRpmMs >= 200)
//       {
//         antiJamActive = true;
//         antiJamEnd = now + 150;
//         lowRpmMs = 0;
//       }
//     }
//     // -----------------------------------

//     wasHighGoal = isHighGoal;

//     if (antiJamActive)
//     {
//       intake.move(-80);

//       if (now >= antiJamEnd)
//       {
//         antiJamActive = false;

//         highGoalStart = now;
//         lowRpmMs = 0;
//       }
//     }
//     else
//     {
//       intake.move(intakeCmd);
//     }

//     pros::delay(10);
//   }
// }

void disabled()
{
  // . . .
}

void competition_initialize()
{

  // . . .
}

bool midGoalMacroRunning = false;
bool midGoalMacroRequest = false;
bool toggleMid = false;
bool toggleHigh = false;
bool toggleIntake = true;
bool toggleReverse = false;
bool matchloadDown = false;
bool wingControlEnabled = false;
bool driveMacroRunning = false;
bool driveMacroRequest = false;

void mid_goal_macro_task()
{
  while (true)
  {
    if (midGoalMacroRequest && !midGoalMacroRunning)
    {
      midGoalMacroRunning = true;
      midGoalMacroRequest = false;
      wing.set_value(true);

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

      midGoalMacroRunning = false;
      toggleMid = true;
      toggleHigh = toggleIntake = toggleReverse = false;
    }

    pros::delay(10);
  }
}

void drive_macro_task()
{
  while (true)
  {
    if (driveMacroRequest && !driveMacroRunning)
    {
      driveMacroRunning = true;
      driveMacroRequest = false;

      chassis.odom_xyt_set(0_in, 0_in, 0_deg);
      chassis.pid_turn_set(100_deg, 127);
      pros::delay(500);
      chassis.pid_drive_set(2_in, 90);
      pros::delay(100);
      chassis.pid_turn_set(-10_deg, 127);
      pros::delay(500);
    }
    driveMacroRunning = false;
    pros::delay(10);
  }
}

void autonomous()
{
  chassis.pid_targets_reset();               // Resets PID targets to 0
  chassis.drive_imu_reset();                 // Reset gyro position to 0
  chassis.drive_sensor_reset();              // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);   // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency

  ez::as::auton_selector.selected_auton_call(); // Calls selected auton from autonomous selector
}

void initialize()
{
  ez::ez_template_print();
  pros::Task ezScreenTask(ez_screen_task);
  pros::Task driveModeTask(drive_mode_task);
  pros::Task shooterTask(shooter_task);
  pros::Task midGoalMacroTask(mid_goal_macro_task);
  pros::Task driveMacroTask(drive_macro_task);

  pros::delay(500);
  wing.set_value(false);
  chassis.opcontrol_curve_buttons_toggle(true);  // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);  // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTRO LLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  ez::as::auton_selector.autons_add({
      {"AWP", awp},
      {"Left 7", left7},
      {"Left 4", left4},
      {"New Left 3-4", left_3_4_new},
      {"Right 5 Ball Rush", right_5ball_rush},
      {"Right 7 Ball Wing", right_wing},
      //{"Kaihan Counter", kaihan_counter},
      {"Go Forward", go_forward},
      {"Right 7 Ball Push", right_7ball},
      {"Old Left 3-4", left_3_4},
      {"Skills", skills},
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  pros::lcd::initialize();
  // pros::Task screenTask([&](){
  //       while (true) {
  //           pros::lcd::print(4, "X: %f", chassis.odom_x_get()); // x
  //           pros::lcd::print(5, "Y: %f", chassis.odom_y_get()); // y
  //           pros::lcd::print(6, "Theta: %f", chassis.odom_theta_get()); // heading
  //           }
  // });
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
  // Show initial drive mode on the controller screen
  master.set_text(0, 0, drive_arcade ? "Drive: Arcade" : "Drive: Tank");
}

void opcontrol()
{
  while (true)
  {
    ez_template_extras();
    chassis.drive_brake_set(MOTOR_BRAKE_COAST);

    if (!driveMacroRunning)
    {
      if (drive_arcade)
        chassis.opcontrol_arcade_standard(ez::SPLIT);
      else
        chassis.opcontrol_tank();
    }

    bool l2Held = master.get_digital(DIGITAL_L2);

    // !!! ONLY ENABLE FOR MATCH
    if (master.get_digital_new_press(DIGITAL_B))
    {
      wingControlEnabled = true;
      toggleMid = !toggleMid;
      toggleHigh = toggleIntake = toggleReverse = false;
    }

    // !!! ONLY ENABLE FOR SKILLS
    if (master.get_digital_new_press(DIGITAL_Y) && !midGoalMacroRunning)
    {
      toggleMid = toggleHigh = toggleIntake = toggleReverse = false;
      midGoalMacroRequest = true;
    }

    if (master.get_digital_new_press(DIGITAL_RIGHT) && !driveMacroRunning)
    {
      driveMacroRequest = true;
    }

    if (master.get_digital_new_press(DIGITAL_R2))
    {
      toggleHigh = !toggleHigh;
      toggleMid = toggleIntake = toggleReverse = false;
    }

    if (master.get_digital_new_press(DIGITAL_L1))
    {
      if (toggleMid || toggleHigh || toggleReverse)
      {
        toggleIntake = toggleMid = toggleHigh = toggleReverse = false;
      }
      else
      {
        toggleIntake = !toggleIntake;
        toggleMid = toggleHigh = toggleReverse = false;
      }
    }

    if (l2Held)
    {
      toggleReverse = false;
      toggleMid = toggleHigh = toggleIntake = false;
    }
    else
    {
      toggleReverse = false;
    }

    if (!wingControlEnabled && master.get_digital(DIGITAL_R1))
      wingControlEnabled = true;
    if (wingControlEnabled)
    {
      wing.set_value(!master.get_digital(DIGITAL_R1));
    }
    if (master.get_digital_new_press(DIGITAL_DOWN))
    {
      matchloadDown = !matchloadDown;
      matchload.set_value(matchloadDown);
    }

    if (!midGoalMacroRunning && !midGoalMacroRequest)
    {
      if (toggleMid)
        intakeState = IntakeState::midGoal;
      else if (toggleHigh)
        intakeState = IntakeState::highGoal;
      else if (toggleIntake)
        intakeState = IntakeState::intake;
      else if (l2Held)
        intakeState = IntakeState::reverse;
      else
        intakeState = IntakeState::idle;
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}

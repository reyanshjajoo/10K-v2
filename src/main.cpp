#include "main.h"

#include "mcl_localizer.hpp"

extern ez::Drive chassis;
extern MCLLocalizer mcl;

// init vars
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {1, -2, -11},     // Left Chassis Ports (negative port will reverse it!)
    {9, -10, 20}, // Right Chassis Ports (negative port will reverse it!)

    21,   // IMU Port
    3.25, // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450); // Wheel RPM = cartridge * (motor gear / wheel gear)

// field 12x12 ft, 20 percent blend, 10 degree heading gate

// 12x12 field (144"), 200 particles
MCLLocalizer mcl(chassis, 144.0, 200);
bool usingMCL = true;

void initialize_mcl() {

    mcl.addSensor(7, -4.0, 4.0, M_PI/2); // right-facing
    mcl.addSensor(8, -4.0, 4.0, M_PI);   // back-facing

    mcl.initializeAroundOdom();
    mcl.setDebug(true);

    pros::Task mclTask([] {
        pros::lcd::initialize();
        while (true) {
            if (usingMCL){
              mcl.update();
              mcl.debugPrintBrain();
            }
            pros::delay(20);
        }
    });
}

IntakeState intakeState = IntakeState::idle;
bool drive_arcade = false;

void drive_mode_task()
{
  while (true)
  {
    if (master.get_digital_new_press(DIGITAL_Y))
    {
      drive_arcade = !drive_arcade;
      master.rumble(drive_arcade ? "." : "..");

      // Update controller screen when drive mode changes. Controller updates are slow,
      // so only update on changes (we're doing that here).
      master.set_text(0, 0, drive_arcade ? "Drive: Arcade" : "Drive: Tank");
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
 * - run your autonomous routine in opcontrol by pressing DOWN and B
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

    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN))
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

void shooter_task() {
  while (true) {

    switch (intakeState) {

      case IntakeState::midGoal:
        intake.move(127);    // intake forward
        hood.move(-127);     // hood backward
        break;

      case IntakeState::highGoal:
        intake.move(127);    // intake forward
        hood.move(127);      // hood forward
        break;

      case IntakeState::intake:
        intake.move(127);    // intake forward
        hood.brake();        // hood braked
        break;

      case IntakeState::reverse:
        intake.move(-127);   // intake backward
        hood.move(-127);     // hood backward
        break;

      case IntakeState::idle:
      default:
        intake.move(0);
        hood.move(0);
        break;
    }

    pros::delay(10);
  }
}

void disabled()
{
  // . . .
}

void competition_initialize()
{
  // . . .
}

void autonomous()
{
  chassis.pid_targets_reset();               // Resets PID targets to 0
  chassis.drive_imu_reset();                 // Reset gyro position to 0
  chassis.drive_sensor_reset();              // Reset drive sensors to 0
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);   // Set the current position, you can start at a specific position with this
  chassis.odom_xyt_set(-48_in, -13_in, 90_deg);   // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency

  ez::as::auton_selector.selected_auton_call(); // Calls selected auton from autonomous selector
}


void initialize()
{
  ez::ez_template_print();
  pros::Task ezScreenTask(ez_screen_task);
  pros::Task driveModeTask(drive_mode_task);
  pros::Task shooterTask(shooter_task);

  pros::delay(500);

  chassis.opcontrol_curve_buttons_toggle(true);  // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);  // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  ez::as::auton_selector.autons_add({
      {"Drive\n\nDrive forward and come back", drive_example},
      {"Turn\n\nTurn 3 times.", turn_example},
      {"Drive and Turn\n\nDrive forward, turn, come back", drive_and_turn},
      {"Drive and Turn\n\nSlow down during drive", wait_until_change_speed},
      {"Swing Turn\n\nSwing in an 'S' curve", swing_example},
      {"Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining},
      {"Combine all 3 movements", combining_movements},
      {"Interference\n\nAfter driving forward, robot performs differently if interfered or not", interfered_example},
      {"Simple Odom\n\nThis is the same as the drive example, but it uses odom instead!", odom_drive_example},
      {"Pure Pursuit\n\nGo to (0, 30) and pass through (6, 10) on the way.  Come back to (0, 0)", odom_pure_pursuit_example},
      {"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
      {"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", odom_boomerang_example},
      {"Boomerang Pure Pursuit\n\nGo to (0, 24, 45) on the way to (24, 24) then come back to (0, 0, 0)", odom_boomerang_injected_pure_pursuit_example},
      {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets},
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  chassis.odom_xyt_set(-48_in, -13_in, 90_deg); //TODO: set starting position for each auton
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");

  initialize_mcl(); // TODO: only initialize in skills runs

  // Show initial drive mode on the controller screen
  master.set_text(0, 0, drive_arcade ? "Drive: Arcade" : "Drive: Tank");
}

void opcontrol()
{
  //chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  while (true)
  {
    ez_template_extras();

    if (drive_arcade)
    {
      chassis.opcontrol_arcade_standard(ez::SPLIT);
    } else
    {
      chassis.opcontrol_tank();
    }


    if (master.get_digital(DIGITAL_L1)) {
        intakeState = IntakeState::midGoal;
    }
    else if (master.get_digital(DIGITAL_L2)) {
        intakeState = IntakeState::highGoal;
    }
    else if (master.get_digital(DIGITAL_R1)) {
        intakeState = IntakeState::intake;
    } else if (master.get_digital(DIGITAL_R2)) {
        intakeState = IntakeState::reverse;
    }
    else {
        intakeState = IntakeState::idle;
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}

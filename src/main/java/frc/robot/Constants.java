// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  ////////////////////////////////////////
  //                 OI                 //
  ////////////////////////////////////////

  public static final double JOYSTICK_X_DEADZONE = 0.15;
  public static final double JOYSTICK_Y_DEADZONE = 0.15;
  public static final double JOYSTICK_Z_DEADZONE = 0.15;
  public static final double JOYSTICK_Z2_DEADZONE = 0.15;
  public static final double JOYSTICK_X_SLEW_RATE = 14.0;
  public static final double JOYSTICK_Y_SLEW_RATE = 14.0;

  public static final int DRIVE_CONTROLLER_PORT = 2;
  public static final int STEER_CONTROLLER_PORT = 0;
  public static final double SLOW_SPEED = 0.5;
  public static final double FAST_SPEED = 1.0;

  public static final double AUTON_1_DISTANCE = 1.5; //meters
  public static final double AUTON_2_DISTANCE = 1.5; //meters
  public static final double AUTON_BALANCE_DISTANCE = 0.4;

  public static final double[] AUTO_BALANCE_PID = {0.003, 4.0e-5, 2.0e-3}; //6.4e-4

  public static final int LIGHT_RELAY_PORT = 0; //port for Spike controlling lights

  public static final double GRID_X_SETPOINT = 0.7;
  public static final double GRID_Y_SETPOINT = 0.8;
  public static final double RIGHT_SUBSTATION_X = 0.5;
  public static final double LEFT_SUBSTATION_X = -0.5;
  public static final double SUBSTATION_Y = 1.1;

  ////////////////////////////////////////
  //               Swerve               //
  ////////////////////////////////////////

  /*
   * Swerve module motor and encoder ids
   * { Front Right, Back Right, Back Left, Front Left }
   */
  public static final int[] SWERVE_DRIVE_MOTOR_IDS = {22, 23, 20, 21};
  public static final int[] SWERVE_STEER_MOTOR_IDS = {32, 33, 30, 31};
  public static final int[] SWERVE_ENCODER_IDS = {42, 43, 40, 41};

  public static final int swerveModuleNumber = 4;

  public static final double[] SWERVE_SETPOINT_OFFSET = {
    // must be between 0 & 360 degrees
    0,//88.7, // Front Right
    0,//96.5, // Rear Right
    0,//360.0 - 126.2, // Rear Left
    0,//360.0 - 58.0 // Front Left
  };

  public static final double[][] SWERVE_STEER_PID_CONSTANTS = {
    // kP   kI   kD  kIz  kFF  kMn  kMx
		{ 1.7e-2, 0.0, 2.8e-4, 0.0, 0.0, -1.0, 1.0 }, //Front Right
		{ 1.7e-2, 0.0, 2.8e-4, 0.0, 0.0, -1.0, 1.0 }, //Rear Right
		{ 1.7e-2, 0.0, 2.8e-4, 0.0, 0.0, -1.0, 1.0 }, //Rear Left
		{ 1.7e-2, 0.0, 2.8e-4, 0.0, 0.0, -1.0, 1.0 }  //Front Left
	};

  public static double[][] SWERVE_DRIVE_PID_CONSTANTS = { 
		// kP   kI   kD  kIz  kFF  kMn  kMx
		{ 1.0e-2, 0.0, 2.0e-2, 0.0, 0.2, -1.0, 1.0 }, //Front Right
		{ 1.0e-2, 0.0, 2.0e-2, 0.0, 0.2, -1.0, 1.0 }, //Rear Right
		{ 1.0e-2, 0.0, 2.0e-2, 0.0, 0.2, -1.0, 1.0 }, //Rear Left
		{ 1.0e-2, 0.0, 2.0e-2, 0.0, 0.2, -1.0, 1.0 }  //Front Left
	};

  public static final double MAX_DRIVETRAIN_SPEED = 5820;

  public static final double[] TAG_ALIGN_STR_PID = {0.4, 0.0, 0.006};
  public static final double[] TAG_ALIGN_ROT_PID = {0.01, 5.0e-2, 0.002};
  public static final double[] TAG_ALIGN_FWD_PID = {0.4, 0.0, 0.006};

  public static final boolean[] STEER_MOTOR_INVERTED = {false, false, false, false};
  public static final boolean[] DRIVE_MOTOR_INVERTED = {false, false, false, false};

  /*
   * Swerve constants for swerve module calculations
   */
  public static final double SWERVE_FRAME_LENGTH = 25.5;
  public static final double SWERVE_FRAME_WIDTH = 20.5;
  public static final double SWERVE_RADIUS = Math.sqrt(Math.pow(SWERVE_FRAME_LENGTH, 2) + Math.pow(SWERVE_FRAME_WIDTH, 2));
  public static final double SWERVE_PID_TOLERANCE = 2.8e-4;
  public static final double DRIVE_POSITION_CONVERSION = (4.0 * 0.0254 * Math.PI) / (6.75);
  public static final double DRIVE_VELOCITY_FACTOR = DRIVE_POSITION_CONVERSION / 60;
  public static final double STEER_POSITION_FACTOR = 2.0 * Math.PI / 12.8; 
  public static final double STEER_VELOCITY_FACTOR = STEER_POSITION_FACTOR / 60;
  public static final double MAX_SPEED_MperS = 4.17;

  public static final double[] DRIVE_FF = {
  // kS  kP   kV
    0.0, 0.0, 1.0 / (MAX_DRIVETRAIN_SPEED * DRIVE_VELOCITY_FACTOR)
  };
  //                                      (wheel circum / (encppr * swerve Ratio)

  public static final int TOP_SHOOT_ID = 19;
  public static final int BOT_SHOOT_ID = 18;
  public static final int INTAKE_ID = 12;
  public static final int CLIMB_LEFT = 50;
  public static final int CLIMB_RIGHT = 51;

  public static final double MAX_CLIMB_REVS = 4.5;
  public static final double SWERVE_VOLT_COMP = 12.6;
  public static double[][] SHOOT_PID = { 
		// kP   kI   kD  kIz  kFF  kMn  kMx
		{ 1.0e-4, 0.0, 2.0e-4, 0.0, 1.65e-4, -1.0, 1.0 }, //Front Right
		{ 1.0e-4, 0.0, 2.0e-4, 0.0, 1.65e-4, -1.0, 1.0 }, //Rear Right
		{ 1.0e-4, 0.0, 2.0e-4, 0.0, 1.65e-4, -1.0, 1.0 }, //Rear Left
		{ 1.0e-4, 0.0, 2.0e-4, 0.0, 1.65e-4, -1.0, 1.0 }  //Front Left
	};
  public static double kRotTransFactor = 0.045; 
}

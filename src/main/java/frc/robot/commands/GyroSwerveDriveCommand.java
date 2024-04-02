package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.GyroSwerveDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class GyroSwerveDriveCommand extends Command {
  DoubleSupplier dX, dY, dZ, dZ2;
  double headingCorrection;
  IntSupplier povHat;
  boolean driveHeading;
  ADIS16470_IMU m_gyro;
  GyroSwerveDrive drivetrain;
  XboxController driveStick;
  double steerAngle;
  BooleanSupplier triggerPressR;
  BooleanSupplier triggerPressL;

  public GyroSwerveDriveCommand(
      DoubleSupplier dX,
      DoubleSupplier dY,
      DoubleSupplier dZ,
      DoubleSupplier dZ2,
      IntSupplier povHat,
      BooleanSupplier triggerPressR,
      BooleanSupplier triggerPressL,
      ADIS16470_IMU imu,
      GyroSwerveDrive gyroSwerveDrive
      ) {
    this.dX = dX;
    this.dY = dY;
    this.dZ = dZ;
    this.dZ2 = dZ2;
    this.povHat = povHat;
    m_gyro = imu;
    drivetrain = gyroSwerveDrive;
    this.triggerPressR = triggerPressR;
    this.triggerPressL = triggerPressL;

    addRequirements(drivetrain);
  }

  private double applyDeadzone(double input, double deadzone) {
    if (Math.abs(input) < deadzone) return 0.0;
    double result = (Math.abs(input) - deadzone) / (1.0 - deadzone);
    return (input < 0.0 ? -result : result);
  }

  private double expo(double input){
    double unsignIn = Math.abs(input);
    double d = Constants.MAX_SPEED_MperS / 2.0;
    double f = Constants.MAX_SPEED_MperS;
    double g = 0.7;
    double h = unsignIn * (Math.pow(unsignIn, 5.0) * g + unsignIn * (1 - g));
    double curved = ((d * unsignIn) + ((f - d) * h));
    return Math.signum(input) * curved;
  }

  @Override
  public void execute() {
    //*
    steerAngle = Math.atan2(applyDeadzone(-dZ.getAsDouble(), Constants.JOYSTICK_Z_DEADZONE), applyDeadzone(dZ2.getAsDouble(), Constants.JOYSTICK_Z2_DEADZONE)) / Math.PI * 180.0;
    driveHeading = (0.5625 < dZ.getAsDouble() * dZ.getAsDouble() + dZ2.getAsDouble() * dZ2.getAsDouble());
    steerAngle = steerAngle < 0.0 ? 360 + steerAngle : steerAngle;
    drivetrain.drive(
        expo(applyDeadzone(-dY.getAsDouble(), Constants.JOYSTICK_X_DEADZONE)),
          expo(applyDeadzone(-dX.getAsDouble(), Constants.JOYSTICK_X_DEADZONE)),
            steerAngle,
              driveHeading,
                triggerPressR.getAsBoolean()
    );
    //*/
    //System.out.println(dX.getAsDouble() * Constants.MAX_SPEED_MperS);
    //drivetrain.setModuleStates(new ChassisSpeeds(applyDeadzone(dY.getAsDouble(), Constants.JOYSTICK_X_DEADZONE) * Constants.MAX_SPEED_MperS, applyDeadzone(dX.getAsDouble(), Constants.JOYSTICK_X_DEADZONE) * Constants.MAX_SPEED_MperS, applyDeadzone(dZ.getAsDouble(), Constants.JOYSTICK_X_DEADZONE)));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}

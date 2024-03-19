package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
  PIDController turnController = new PIDController(0.02, 0.1, 0.001);
  double setAngle;
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
    turnController.reset();
    turnController.setIntegratorRange(-0.2, 0.2);
    turnController.enableContinuousInput(0.0, 360.0);
    turnController.setTolerance(Math.toRadians(0.2));
    this.triggerPressR = triggerPressR;
    this.triggerPressL = triggerPressL;

    addRequirements(drivetrain);
  }

  private double applyDeadzone(double input, double deadzone) {
    if (Math.abs(input) < deadzone) return 0.0;
    double result = (Math.abs(input) - deadzone) / (1.0 - deadzone);
    return (input < 0.0 ? -result : result);
  }

  @Override
  public void execute() {
    double getY = drivetrain.getPose().getY();
    double getX = drivetrain.getPose().getX();
    setAngle = Math.atan2(applyDeadzone(dZ.getAsDouble(), Constants.JOYSTICK_Z_DEADZONE), applyDeadzone(dZ2.getAsDouble(), Constants.JOYSTICK_Z2_DEADZONE)) / Math.PI * 180.0;
    if(triggerPressR.getAsBoolean()){setAngle = -Math.toDegrees(Math.atan2(getY - 5.45, getX));}
    //if(triggerPressL.getAsBoolean()){setAngle = -Math.toDegrees(Math.atan2(getX - 7.8, getY - 1.78));}
    //if(triggerPressL){setAngle = Math.atan2(applyDeadzone(dZ.getAsDouble(), Constants.JOYSTICK_Z_DEADZONE), applyDeadzone(dZ2.getAsDouble(), Constants.JOYSTICK_Z2_DEADZONE)) / Math.PI * 180.0;}

    driveHeading = triggerPressR.getAsBoolean() || triggerPressL.getAsBoolean() || (0.75 < Math.sqrt(dZ.getAsDouble() * dZ.getAsDouble() + dZ2.getAsDouble() * dZ2.getAsDouble()));
    setAngle = setAngle < 0.0 ? 360 + setAngle : setAngle;
    if(driveHeading){
      headingCorrection =  MathUtil.clamp(turnController.calculate(360.0 - (m_gyro.getAngle(m_gyro.getYawAxis()) % 360.0), setAngle), -1.0, 1.0);
    }
    drivetrain.alteredGyroDrive(
        dX.getAsDouble(),
          dY.getAsDouble(),
            driveHeading ? headingCorrection : 0.0,
              Math.toRadians(m_gyro.getAngle(m_gyro.getYawAxis()))
    );
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}

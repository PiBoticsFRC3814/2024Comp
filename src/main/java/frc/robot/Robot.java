// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.GyroSwerveDriveCommand;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private int autonNumber;

  private RobotContainer m_robotContainer;
  private final Field2d m_field = new Field2d();
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  // Store what the last hue of the first pixel is
  private int m_rainbowFirstPixelHue;
  //private final PowerDistribution m_pdp = new PowerDistribution(1, ModuleType.kRev);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_led = new AddressableLED(9);

    autonNumber = 0;

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
    rainbow();
    // Set the LEDs
    m_led.setData(m_ledBuffer);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putData("First", m_robotContainer.chooserFirst);
    //SmartDashboard.putNumber("Voltage", m_pdp.getVoltage());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = new PathPlannerAuto("AutoShoot");
    autonNumber = 0;
    m_autonomousCommand.isFinished();
    m_robotContainer.m_robotStates.autonomous = true;
    LimelightHelpers.setPipelineIndex("limelight", 0);
    m_robotContainer.m_gyroSwerveDrive.removeDefaultCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if(m_autonomousCommand.isFinished()){
      if(autonNumber == 1) m_autonomousCommand = m_robotContainer.chooserFirst.getSelected();
      if(autonNumber == 2) m_autonomousCommand = m_robotContainer.chooserSecond.getSelected();
      if(autonNumber == 2) m_autonomousCommand = m_robotContainer.chooserThird.getSelected();
      autonNumber++;
      if (m_autonomousCommand != null) {
        m_autonomousCommand.schedule();
      }
    }
    //SmartDashboard.putNumber("Voltage", m_pdp.getVoltage());
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.m_gyroSwerveDrive.setDefaultCommand(
        new GyroSwerveDriveCommand(
            () -> m_robotContainer.driveStick.getLeftX(),
            () -> m_robotContainer.driveStick.getLeftY(),
            () -> m_robotContainer.driveStick.getRightX(),
            () -> -m_robotContainer.driveStick.getRightY(),
            () -> m_robotContainer.driveStick.getPOV(0),
            () -> m_robotContainer.driveStick.getRightTriggerAxis() >= 0.8,
            () -> m_robotContainer.driveStick.getLeftTriggerAxis() >= 0.8,
            m_robotContainer.m_gyro,
            m_robotContainer.m_gyroSwerveDrive));
    LimelightHelpers.setPipelineIndex("limelight", 1);
    m_robotContainer.m_robotStates.autonomous = false;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putData("Field", m_field);
    m_field.setRobotPose(m_robotContainer.m_gyroSwerveDrive.getPose());
    SmartDashboard.putNumber("Gyro", m_robotContainer.m_gyro.getAngle(m_robotContainer.m_gyro.getYawAxis()));
    SmartDashboard.putBoolean("Note", m_robotContainer.m_intake.gotNote);
    //SmartDashboard.putNumber("speakerAngle", Math.toDegrees(Math.atan2(m_robotContainer.m_gyroSwerveDrive.getPose().getY() - 5.56, m_robotContainer.m_gyroSwerveDrive.getPose().getX())));
    SmartDashboard.putNumber("ampSpeed", m_robotContainer.m_robotStates.ampSpeed);
    SmartDashboard.putNumber("ampDist", m_robotContainer.m_robotStates.speakDist);
    //SmartDashboard.putNumber("Voltage", m_pdp.getVoltage());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  private void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 255 / m_ledBuffer.getLength())) % 255;
      // Set the value
      m_ledBuffer.setHSV(i, 55, 255, hue);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 255;
  }
}

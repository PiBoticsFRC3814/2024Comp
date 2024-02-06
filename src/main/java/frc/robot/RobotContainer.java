// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.DriveFast;
import frc.robot.commands.DriveSlow;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.GyroReset;
import frc.robot.commands.GyroSwerveDriveCommand;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.Outake;
import frc.robot.commands.ShootAmp;
import frc.robot.commands.ShootSpeaker;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FlywheelShooter;
import frc.robot.subsystems.GyroSwerveDrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RobotStates;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final FlywheelShooter m_shooter = new FlywheelShooter();
  private final Intake m_intake = new Intake();
  private final RobotStates m_robotStates = new RobotStates();
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final GyroSwerveDrive m_gyroSwerveDrive = new GyroSwerveDrive(m_robotStates, m_gyro);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  XboxController driveStick = new XboxController(2);
  XboxController controlStick = new XboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //*
    m_gyroSwerveDrive.setDefaultCommand(
        new GyroSwerveDriveCommand(
            () -> driveStick.getLeftX(),
            () -> driveStick.getLeftY(),
            () -> driveStick.getRightX(),
            () -> -driveStick.getRightY(),
            () -> driveStick.getPOV(0),
            m_gyro,
            m_gyroSwerveDrive,
            driveStick));
    // Configure the trigger bindings
    double speed = 1300;//SmartDashboard.getNumber("Speed", 0.0);
    //m_shooter.setDefaultCommand(new ShootWithSlider(m_shooter, () -> speed, () -> m_driverController.getThrottle()));
    configureBindings();
  }

  //now fixed

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the-
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new JoystickButton(controlStick, Button.kRightBumper.value).whileTrue(new IntakeRun(m_intake));
    new JoystickButton(controlStick, Button.kRightBumper.value).whileFalse(new IntakeStop(m_intake));
    new JoystickButton(controlStick, Button.kLeftBumper.value).whileTrue(new Outake(m_intake));
    new JoystickButton(driveStick, Button.kX.value).whileTrue(new GyroReset(m_gyro, m_gyroSwerveDrive));
    new JoystickButton(controlStick, Button.kX.value).whileTrue(new ShootAmp(m_shooter, m_intake));
    new JoystickButton(controlStick, Button.kA.value).whileTrue(new ShootSpeaker(m_shooter, m_intake));
    new JoystickButton(driveStick, Button.kRightBumper.value).whileTrue(new DriveFast(m_robotStates));
    new JoystickButton(driveStick, Button.kRightBumper.value).whileFalse(new DriveSlow(m_robotStates));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}

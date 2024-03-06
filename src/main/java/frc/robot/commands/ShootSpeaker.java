// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelShooter;
import frc.robot.subsystems.GyroSwerveDrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RobotStates;

public class ShootSpeaker extends Command {
  /** Creates a new ShootAmp. */
  private FlywheelShooter shooter;
  private Intake intake;
  Timer timer;
  private int count;
  RobotStates robotState;
  double speed;
  GyroSwerveDrive drivetrain;
  public ShootSpeaker(FlywheelShooter shooter, Intake intake, RobotStates robotState) {
    this.shooter = shooter;
    this.intake = intake;
    timer = new Timer();
    this.robotState = robotState;
    addRequirements(shooter, intake, robotState);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    intake.outtakeShoot();
    intake.brake();
    count = 0;
    //speed = 4100;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() >= 0.5) shooter.fireDifference(robotState.speakSpeed, robotState.speakSpeed >= 4200 ? 0.0 : -0.1);
    if(((!intake.gotNote) || timer.get() >= 0.3) && shooter.speed.getVelocity() <= robotState.speakSpeed) intake.stop();
    if((shooter.speed.getVelocity() >= (robotState.speakSpeed - robotState.speakSpeed * 0.1) * 0.9)) intake.shoot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() >= 1.5 && robotState.autonomous);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelShooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RobotStates;

public class ShootAmp extends Command {
  /** Creates a new ShootAmp. */
  private FlywheelShooter shooter;
  private Intake intake;
  Timer timer;
  RobotStates robotState;
  public ShootAmp(FlywheelShooter shooter, Intake intake, RobotStates robotState) {
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
    intake.outtake();
    intake.brake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() >= 0.3) shooter.fireDifference(1700, 0.08);
    if(((!intake.gotNote) && (shooter.speed.getVelocity() <= 1700)) || timer.get() >= 0.3) intake.stop();
    if(shooter.speed.getVelocity() >= 1600) intake.shoot();
  }

  //0 1700 0.08
  //6 1900 0.2
  //12 2100 0.3

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RobotStates;

public class IntakeRun extends Command {
  /** Creates a new IntakeRun. */
  Intake intake;
  Timer timer;
  Timer timeOut;
  RobotStates robotState;
  public IntakeRun(Intake intake, RobotStates robotState) {
    this.intake = intake;
    timer = new Timer();
    timeOut = new Timer();
    this.robotState = robotState;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timeOut.reset();
    timeOut.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.intake();
    if(intake.gotNote) timer.start();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((intake.gotNote && (timer.get() >= 0.2)) || (robotState.autonomous && timeOut.get() >= 4.0));
  }
}

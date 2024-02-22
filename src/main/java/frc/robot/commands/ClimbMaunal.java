// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimbMaunal extends Command {
  /** Creates a new ClimbMaunal. */
  DoubleSupplier left;
  DoubleSupplier right;
  Climber climber;
  public ClimbMaunal(Climber climber, DoubleSupplier left, DoubleSupplier right) {
    this.climber = climber;
    this.left = left;
    this.right = right;
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  private double applyDeadzone(double input, double deadzone) {
    if (Math.abs(input) < deadzone) return 0.0;
    double result = (Math.abs(input) - deadzone) / (1.0 - deadzone);
    return (input < 0.0 ? -result : result);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.control(applyDeadzone(left.getAsDouble(), 0.2), applyDeadzone(right.getAsDouble(), 0.2));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

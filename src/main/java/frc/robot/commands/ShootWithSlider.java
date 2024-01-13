// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelShooter;

public class ShootWithSlider extends Command {
  /** Creates a new ShootWithSlider. */
  FlywheelShooter shooter;
  DoubleSupplier speed;
  DoubleSupplier factor;
  public ShootWithSlider(FlywheelShooter shooter, DoubleSupplier speed, DoubleSupplier factor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.speed = speed;
    this.factor = factor;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.fireDifference(speed.getAsDouble(), factor.getAsDouble());
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

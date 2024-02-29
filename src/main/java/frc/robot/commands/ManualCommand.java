// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelShooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RobotStates;

public class ManualCommand extends Command {
  /** Creates a new ShootAmp. */
  private FlywheelShooter shooter;
  Timer timer;
  DoubleSupplier speed;
  DoubleSupplier factor;
  public ManualCommand(FlywheelShooter shooter, DoubleSupplier speed, DoubleSupplier factor) {
    this.shooter = shooter;
    timer = new Timer();
    this.factor = factor;
    this.speed = speed;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.fireDifference(speed.getAsDouble() * 5000, factor.getAsDouble() * 0.2 - 0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

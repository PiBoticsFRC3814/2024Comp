// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelShooter;
import frc.robot.subsystems.Intake;

public class ShootSpeaker extends Command {
  /** Creates a new ShootAmp. */
  private FlywheelShooter shooter;
  private Intake intake;
  Timer timer;
  public ShootSpeaker(FlywheelShooter shooter, Intake intake) {
    this.shooter = shooter;
    this.intake = intake;
    timer = new Timer();
    addRequirements(shooter, intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    intake.outtake();
    intake.coast();
    shooter.fireDifference(4200, 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((!intake.gotNote) && (shooter.speed.getVelocity() <= 4200) || timer.get() >= 0.2) intake.stop();
    if(shooter.speed.getVelocity() >= 4200) intake.shoot();
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
    return false;
  }
}

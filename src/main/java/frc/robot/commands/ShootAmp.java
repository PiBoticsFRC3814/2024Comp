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
  double speed;
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
    double distance = 1;
    //values from linear regression given datapoints causes I'm too lazy
    //0 1750
    //3 1800
    //6 1900
    //12 2000
    speed = 740.16 * Math.log(186.236 * distance + 5334.16) - 4607.85;
    System.out.println(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    if(timer.get() >= 0.3) shooter.fireDifference(speed, 0.30);
    if(((!intake.gotNote) && (shooter.speed.getVelocity() <= speed)) || timer.get() >= 0.3) intake.stop();
    if(shooter.speed.getVelocity() >= speed) intake.shoot();
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

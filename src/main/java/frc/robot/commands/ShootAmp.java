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

public class ShootAmp extends Command {
  /** Creates a new ShootAmp. */
  private FlywheelShooter shooter;
  private Intake intake;
  Timer timer;
  RobotStates robotState;
  double speed;
  GyroSwerveDrive drivetrain;
  public ShootAmp(FlywheelShooter shooter, Intake intake, RobotStates robotState, GyroSwerveDrive drivetrain) {
    this.shooter = shooter;
    this.intake = intake;
    timer = new Timer();
    this.robotState = robotState;
    this.drivetrain = drivetrain;
    addRequirements(shooter, intake, robotState, drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    intake.outtake();
    intake.brake();
    System.out.println(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    double distance = 230 - Math.sqrt(Math.pow(Math.abs(drivetrain.getPose().getX()) - 6.429375,2.0) + Math.pow(drivetrain.getPose().getY() - 4.098925,2.0)) * 1000 / 25.4;
    //values from linear regression given datapoints causes I'm too lazy
    //0 1750
    //3 1800
    //6 1900
    //12 2000
    distance = distance >= 0.0 ? distance : 0.0;
    speed = distance >= 1 ? 740.16 * Math.log(186.236 * distance + 5334.16) - 4607.85 : 1700;
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

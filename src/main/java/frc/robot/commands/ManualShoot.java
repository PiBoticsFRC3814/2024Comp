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

public class ManualShoot extends Command {
  /** Creates a new ShootAmp. */
  private FlywheelShooter shooter;
  private Intake intake;
  Timer timerSpeak;
  private int count;
  RobotStates robotState;
  double speed;
  GyroSwerveDrive drivetrain;
  double timeSinceShoot;
  public ManualShoot(FlywheelShooter shooter, Intake intake, RobotStates robotState) {
    this.shooter = shooter;
    this.intake = intake;
    timerSpeak = new Timer();
    this.robotState = robotState;
    addRequirements(shooter, intake, robotState);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timerSpeak.reset();
    timerSpeak.start();
    intake.outtakeShoot();
    intake.brake();
    count = 0;
    timeSinceShoot = 0;
    //speed = 4100;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timerSpeak.get() >= 0.3) shooter.fireDifference(4200, 0.0);
    if(((!intake.gotNote) || timerSpeak.get() >= 0.3) && shooter.speed.getVelocity() <= (4200 * 0.95)){
      intake.stop();
    }
    if((shooter.speed.getVelocity() >= 4200  * 0.95) && (!robotState.autonomous || robotState.inSpeaker)){
      if(timeSinceShoot == 0) timeSinceShoot = Timer.getFPGATimestamp();
      intake.shoot();
    }
    //System.out.println(Timer.getFPGATimestamp() - timeSinceShoot);
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
    return (((Timer.getFPGATimestamp() - timeSinceShoot >= 1.3) && timeSinceShoot > 0) && robotState.autonomous);
  }
}

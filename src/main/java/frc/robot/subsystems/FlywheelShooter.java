// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class FlywheelShooter extends SubsystemBase {
  /** Creates a new FlywheelShooter. */
  private CANSparkMax topShooter;
  private SparkPIDController topPIDShoot;
  private CANSparkMax botShooter;
  private SparkPIDController botPIDShoot;
  public RelativeEncoder speed;
  public FlywheelShooter() {
    topShooter = new CANSparkMax(Constants.TOP_SHOOT_ID, MotorType.kBrushless);
    topPIDShoot = topShooter.getPIDController();
		topPIDShoot.setP(Constants.SHOOT_PID[0][0]);
		topPIDShoot.setI(Constants.SHOOT_PID[0][1]);
		topPIDShoot.setD(Constants.SHOOT_PID[0][2]);
		topPIDShoot.setIZone(Constants.SHOOT_PID[0][3]);
		topPIDShoot.setFF(Constants.SHOOT_PID[0][4]);
    topShooter.setOpenLoopRampRate( 0.2 );
		topShooter.setSmartCurrentLimit(50, 40);
    topShooter.setInverted(true);

    botShooter = new CANSparkMax(Constants.BOT_SHOOT_ID, MotorType.kBrushless);
    botPIDShoot = botShooter.getPIDController();
		botPIDShoot.setP(Constants.SHOOT_PID[1][0]);
		botPIDShoot.setI(Constants.SHOOT_PID[1][1]);
		botPIDShoot.setD(Constants.SHOOT_PID[1][2]);
		botPIDShoot.setIZone(Constants.SHOOT_PID[1][3]);
		botPIDShoot.setFF(Constants.SHOOT_PID[1][4]);
    botShooter.setOpenLoopRampRate( 0.2 );
		botShooter.setSmartCurrentLimit(50, 40);

    speed = botShooter.getEncoder();
  }

  public void stopShooter(){
    topPIDShoot.setReference(0, CANSparkMax.ControlType.kVelocity);
    botPIDShoot.setReference(0, CANSparkMax.ControlType.kVelocity);
    topShooter.stopMotor();
    botShooter.stopMotor();
  }

  public void fireDifference(double speed, double factor){
    topPIDShoot.setReference(speed - speed * factor, CANSparkMax.ControlType.kVelocity);
    botPIDShoot.setReference(speed + speed * factor, CANSparkMax.ControlType.kVelocity);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
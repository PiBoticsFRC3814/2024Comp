// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  WPI_TalonSRX climbLeft = new WPI_TalonSRX(Constants.CLIMB_LEFT);
  WPI_TalonSRX climbRight = new WPI_TalonSRX(Constants.CLIMB_RIGHT);

  private DutyCycleEncoder leftEncoder = new DutyCycleEncoder(6);
  private DutyCycleEncoder rightEncoder = new DutyCycleEncoder(7);

  private double leftRotation = 0;
  private double rightRotation = 0;
  
  public Climber() {
    climbLeft.configOpenloopRamp(0.2);
    climbRight.configOpenloopRamp(0.2);
    climbLeft.configPeakCurrentLimit(5);
    climbRight.configPeakCurrentLimit(5);
    climbLeft.setNeutralMode(NeutralMode.Brake);
    climbRight.setNeutralMode(NeutralMode.Brake);
  }

  public void control(double driveLeft, double driveRight){
    //if(leftEncoder.getDistance() <= Constants.MAX_CLIMB_REVS || driveLeft <= 0.0) climbLeft.set(driveLeft); else climbLeft.set(0.0);
    //if(rightEncoder.getDistance() <= Constants.MAX_CLIMB_REVS || driveRight <= 0.0) climbRight.set(driveRight); else climbRight.set(0.0);
    climbRight.set(driveRight);
    climbLeft.set(driveLeft);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  WPI_TalonSRX climbLeft = new WPI_TalonSRX(Constants.CLIMB_LEFT);
  WPI_TalonSRX climbRight = new WPI_TalonSRX(Constants.CLIMB_RIGHT);
  
  public Climber() {
    climbLeft.configOpenloopRamp(0.2);
    climbRight.configOpenloopRamp(0.2);
    climbLeft.configPeakCurrentLimit(5);
    climbRight.configPeakCurrentLimit(5);
    climbLeft.setNeutralMode(NeutralMode.Brake);
    climbRight.setNeutralMode(NeutralMode.Brake);
  }

  public void control(double driveLeft, double driveRight){
    climbLeft.set(driveLeft);
    climbRight.set(driveRight);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

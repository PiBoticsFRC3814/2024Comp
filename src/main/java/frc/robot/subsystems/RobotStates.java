// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RobotStates extends SubsystemBase {
  /** Creates a new RobotStates. */
  public boolean inFrontOfCubeStation;
  public int moveFromLastAlign;
  public boolean autonomous;
  public double driveMultiplier;
  public double speakSpeed;
  public double speakDist;
  public double ampSpeed;
  public boolean inAmp;
  public boolean inSpeaker;
  public int gyroReset;

  public RobotStates() {
    inFrontOfCubeStation = false;
    moveFromLastAlign = 0;
    autonomous = false;
    driveMultiplier = Constants.SLOW_SPEED;
    speakSpeed = 4500;
    ampSpeed = 1750;
    inAmp = true;
    inSpeaker = true;
    gyroReset = 0;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
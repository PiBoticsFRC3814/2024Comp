// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private WPI_TalonSRX intakeMotor;
  public boolean gotNote;
  private DigitalInput sensor;
  public Intake() {
    intakeMotor = new WPI_TalonSRX(Constants.INTAKE_ID);
    sensor = new DigitalInput(8);
    intakeMotor.configPeakCurrentLimit(50);
  }

  public void intake(){
    intakeMotor.set(0.75);
  }

  public void shoot(){
    intakeMotor.set(1.0);
  }

  public void outtake(){
    intakeMotor.set(-0.4);
  }

  public void outtakeShoot(){
    intakeMotor.set(-1.0);
  }

  public void stop(){
    intakeMotor.set(0.0);
  }

  public void brake(){
    intakeMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void coast(){
    intakeMotor.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {
    gotNote = !sensor.get();
    SmartDashboard.putBoolean("Note", gotNote);
    // This method will be called once per scheduler run
  }
}

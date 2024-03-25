package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
	public CANSparkMax            driveMotor;
	private SparkPIDController driveVelocityPIDController;
	private SparkPIDController steerPIDController;

	public CANSparkMax            steerMotor;
	private CoreCANcoder              steerAngleEncoder;
	private SimpleMotorFeedforward driveFF;

	public double                 position;
	private int 				  index;
	public RelativeEncoder 		  driveEncoder;
	public RelativeEncoder 		  steerEncoder;
	
	/* the SwerveModule subsystem */
	public SwerveModule( int swerveModIndex ) {
		driveMotor = new CANSparkMax( Constants.SWERVE_DRIVE_MOTOR_IDS[ swerveModIndex ], MotorType.kBrushless );
		driveMotor.setIdleMode(IdleMode.kBrake);
		driveMotor.enableVoltageCompensation(Constants.SWERVE_VOLT_COMP);
		driveMotor.setInverted( Constants.DRIVE_MOTOR_INVERTED[swerveModIndex] );
		driveMotor.setOpenLoopRampRate( 0.2 );
		driveMotor.setSmartCurrentLimit(50, 55);

		driveVelocityPIDController = driveMotor.getPIDController();
		driveVelocityPIDController.setP(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][0]);
		driveVelocityPIDController.setI(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][1]);
		driveVelocityPIDController.setD(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][2]);
		driveVelocityPIDController.setIZone(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][3]); 

		driveEncoder = driveMotor.getEncoder();
		driveEncoder.setPositionConversionFactor(Constants.DRIVE_POSITION_CONVERSION);
		driveEncoder.setVelocityConversionFactor(Constants.DRIVE_VELOCITY_FACTOR);
		driveEncoder.setAverageDepth(4);
		driveEncoder.setMeasurementPeriod(16);
		driveFF = new SimpleMotorFeedforward(Constants.DRIVE_FF[0],Constants.DRIVE_FF[1],Constants.DRIVE_FF[2]);
		configureCANStatusFrames(10, 20, 20, 500, 500, 200, 200, driveMotor);
		driveMotor.burnFlash();
		
		steerMotor = new CANSparkMax( Constants.SWERVE_STEER_MOTOR_IDS[swerveModIndex], MotorType.kBrushless );
		steerMotor.enableVoltageCompensation(Constants.SWERVE_VOLT_COMP);
		steerMotor.setIdleMode(IdleMode.kCoast);
		steerMotor.setInverted( Constants.STEER_MOTOR_INVERTED[swerveModIndex] );
		steerMotor.setSmartCurrentLimit(50, 40);

		steerPIDController = steerMotor.getPIDController();
		steerPIDController.setP(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][0]);
		steerPIDController.setI(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][1]);
		steerPIDController.setD(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][2]);
		steerPIDController.setIZone(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][3]); 
		steerPIDController.setPositionPIDWrappingEnabled(true);
		
		steerEncoder = steerMotor.getEncoder();
		steerEncoder.setPositionConversionFactor(Constants.STEER_POSITION_FACTOR);
		steerEncoder.setVelocityConversionFactor(Constants.STEER_VELOCITY_FACTOR);
		steerEncoder.setAverageDepth(4);
		steerEncoder.setMeasurementPeriod(16);

		steerAngleEncoder = new CANcoder( Constants.SWERVE_ENCODER_IDS[swerveModIndex] );
		steerEncoder.setPosition(steerAngleEncoder.getAbsolutePosition().getValue());

		steerMotor.burnFlash();

		index = swerveModIndex;
	}

	public void configureCANStatusFrames(
      int CANStatus0, int CANStatus1, int CANStatus2, int CANStatus3, int CANStatus4, int CANStatus5, int CANStatus6, CANSparkMax motor)
  {
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, CANStatus0);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, CANStatus1);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, CANStatus2);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, CANStatus3);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, CANStatus4);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, CANStatus5);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, CANStatus6);
    //  https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
  }

	public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(getStateAngle()));
    }

	public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(getStateAngle()));
    }

	public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getStateAngle()));

        driveVelocityPIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity, 0, driveFF.calculate(state.speedMetersPerSecond) * Constants.SWERVE_VOLT_COMP);
        setReferenceAngle(state.angle.getRadians());
    }

	public void setReferenceAngle(double referenceAngleRadians) {
        double currentAngleRadians = steerEncoder.getPosition();

        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI;
        }

        // The reference angle has the range [0, 2pi) but the Neo's encoder can go above
        // that
        double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }
        steerPIDController.setReference(adjustedReferenceAngleRadians, ControlType.kPosition);
    }

	public double getStateAngle() {
        double motorAngleRadians = steerEncoder.getPosition();
        motorAngleRadians %= 2.0 * Math.PI;
        if (motorAngleRadians < 0.0) {
            motorAngleRadians += 2.0 * Math.PI;
        }

        return motorAngleRadians;
    }

  public void initDefaultCommand() {
    // NOTE: no default command unless running swerve modules seperately
  }
}

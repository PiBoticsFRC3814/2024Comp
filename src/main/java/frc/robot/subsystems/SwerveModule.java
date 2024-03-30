package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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
	private CANcoder              steerAngleEncoder;
	private SimpleMotorFeedforward driveFF;

	public double                 position;
	private int 				  index;
	public RelativeEncoder 		  driveEncoder;
	public RelativeEncoder 		  steerEncoder;

	public static double   STATUS_TIMEOUT_SECONDS = 0.02;
	
	/* the SwerveModule subsystem */
	public SwerveModule( int swerveModIndex ) {
		driveMotor = new CANSparkMax( Constants.SWERVE_DRIVE_MOTOR_IDS[ swerveModIndex ], MotorType.kBrushless );
		driveMotor.setIdleMode(IdleMode.kBrake);
		driveMotor.enableVoltageCompensation(Constants.SWERVE_VOLT_COMP);
		driveMotor.setInverted( Constants.DRIVE_MOTOR_INVERTED[swerveModIndex] );
		driveMotor.setOpenLoopRampRate( 0.4 );
		driveMotor.setSmartCurrentLimit(50, 40);

		driveVelocityPIDController = driveMotor.getPIDController();
		driveVelocityPIDController.setP(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][0]);
		driveVelocityPIDController.setI(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][1]);
		driveVelocityPIDController.setD(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][2]);
		driveVelocityPIDController.setIZone(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][3]); 
		driveVelocityPIDController.setFF(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][4]);
		driveVelocityPIDController.setOutputRange(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][5], Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][6]);

		driveEncoder = driveMotor.getEncoder();
		driveEncoder.setPositionConversionFactor(Constants.DRIVE_POSITION_CONVERSION);
		driveEncoder.setVelocityConversionFactor(Constants.DRIVE_VELOCITY_FACTOR);
		driveEncoder.setAverageDepth(4);
		driveEncoder.setMeasurementPeriod(16);
		configureCANStatusFrames(10, 20, 20, 500, 500, 200, 200, driveMotor);
		driveMotor.burnFlash();
		
		steerMotor = new CANSparkMax( Constants.SWERVE_STEER_MOTOR_IDS[swerveModIndex], MotorType.kBrushless );
		steerMotor.enableVoltageCompensation(Constants.SWERVE_VOLT_COMP);
		steerMotor.setIdleMode(IdleMode.kBrake);
		steerMotor.setInverted( Constants.STEER_MOTOR_INVERTED[swerveModIndex] );
		steerMotor.setSmartCurrentLimit(40, 20);

		steerPIDController = steerMotor.getPIDController();
		steerPIDController.setP(Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex][0]);
		steerPIDController.setI(Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex][1]);
		steerPIDController.setD(Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex][2]);
		steerPIDController.setIZone(Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex][3]); 
		steerPIDController.setOutputRange(Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex][5], Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex][6]);
		steerPIDController.setPositionPIDWrappingEnabled(true);
		steerPIDController.setPositionPIDWrappingMinInput(0);
		steerPIDController.setPositionPIDWrappingMaxInput(2 * Math.PI);
		
		steerEncoder = steerMotor.getEncoder();
		steerEncoder.setPositionConversionFactor(Constants.STEER_POSITION_FACTOR);
		steerEncoder.setVelocityConversionFactor(Constants.STEER_VELOCITY_FACTOR);
		steerEncoder.setAverageDepth(4);
		steerEncoder.setMeasurementPeriod(16);
		configureCANStatusFrames(10, 20, 20, 500, 500, 200, 200, steerMotor);
		steerMotor.burnFlash();

		steerAngleEncoder = new CANcoder( Constants.SWERVE_ENCODER_IDS[swerveModIndex] );
		steerEncoder.setPosition(getAbsolutePosition());

		index = swerveModIndex;
	}
	public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(getStateAngle()));
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


	public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(getStateAngle()));
    }

	public void output(){
		SmartDashboard.putNumber("speed mod " + index, driveEncoder.getVelocity());
		SmartDashboard.putNumber("angle mod " + index, new Rotation2d(getStateAngle()).getDegrees());
	}

	public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getStateAngle()));
	if(((Math.abs(state.speedMetersPerSecond) < 0.05) && Math.abs(state.angle.getRadians() - getStateAngle()) < 0.02 ) && steerEncoder.getVelocity() < 0.1){
		driveMotor.set(0);
		steerMotor.set(0);
	} else {
		double velocity = getCosineCompensatedVelocity(state);

        	driveVelocityPIDController.setReference(velocity, ControlType.kVelocity);
        	//driveVelocityPIDController.setReference(Constants.MAX_SPEED_MperS, ControlType.kVelocity);
        	setReferenceAngle(state.angle.getRadians());
	}
    	}
	
	private double getCosineCompensatedVelocity(SwerveModuleState desiredState){
    double cosineScalar = 1.0;
    // Taken from the CTRE SwerveModule class.
    // https://api.ctr-electronics.com/phoenix6/release/java/src-html/com/ctre/phoenix6/mechanisms/swerve/SwerveModule.html#line.46
    /* From FRC 900's whitepaper, we add a cosine compensator to the applied drive velocity */
    /* To reduce the "skew" that occurs when changing direction */
    /* If error is close to 0 rotations, we're already there, so apply full power */
    /* If the error is close to 0.25 rotations, then we're 90 degrees, so movement doesn't help us at all */
    cosineScalar = Rotation2d.fromDegrees(desiredState.angle.getDegrees())
                             .minus(new Rotation2d(getStateAngle()))
                             .getCos(); // TODO: Investigate angle modulus by 180.
    /* Make sure we don't invert our drive, even though we shouldn't ever target over 90 degrees anyway */
    if (cosineScalar < 0.0)
    {
      cosineScalar = 1;
    }

    return desiredState.speedMetersPerSecond * (cosineScalar);
  }

	public void resetModule(){
		steerEncoder.setPosition(getAbsolutePosition());
		setDesiredState(new SwerveModuleState(0.0, new Rotation2d(0)));
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
        //steerPIDController.setReference(0.0, ControlType.kPosition);
    }

	public double getStateAngle() {
        double motorAngleRadians = steerEncoder.getPosition();
        motorAngleRadians %= 2.0 * Math.PI;
        if (motorAngleRadians < 0.0) {
            motorAngleRadians += 2.0 * Math.PI;
        }

        return motorAngleRadians;
    }

public double getAbsolutePosition()
  {

    StatusSignal<Double> angle = steerAngleEncoder.getAbsolutePosition();

    // Taken from democat's library.
    // Source: https://github.com/democat3457/swerve-lib/blob/7c03126b8c22f23a501b2c2742f9d173a5bcbc40/src/main/java/com/swervedrivespecialties/swervelib/ctre/CanCoderFactoryBuilder.java#L51-L74
    for (int i = 0; i < 10; i++)
    {
      if (angle.getStatus() == StatusCode.OK)
      {
        break;
      }
      angle = angle.waitForUpdate(STATUS_TIMEOUT_SECONDS);
    }

    return angle.getValue() * Math.PI * 2.0;
  }

  public void initDefaultCommand() {
    // NOTE: no default command unless running swerve modules seperately
  }
}

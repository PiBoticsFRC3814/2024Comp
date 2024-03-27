package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class GyroSwerveDrive extends SubsystemBase {
  private double[] speed = {0.0, 0.0, 0.0, 0.0};
  private double[] angle = {0.0, 0.0, 0.0, 0.0};
  private RobotStates m_RobotStates;

  private SlewRateLimiter slewX = new SlewRateLimiter(Constants.JOYSTICK_X_SLEW_RATE);
  private SlewRateLimiter slewY = new SlewRateLimiter(Constants.JOYSTICK_Y_SLEW_RATE);

  private SwerveDriveKinematics kinematics;
  private SwerveDrivePoseEstimator poseEstimator;
  private ADIS16470_IMU gyro;
  PIDController turnController = new PIDController(0.02, 0.1, 0.001);

  public boolean trustVision;
  public Pose2d currentPose;

  private SwerveModule[] swerveMod = {
    new SwerveModule(0), new SwerveModule(1), new SwerveModule(2), new SwerveModule(3)
  };

  public GyroSwerveDrive(RobotStates robotStates, ADIS16470_IMU gyro) {
    m_RobotStates = robotStates;
    this.gyro = gyro;

    turnController.setIntegratorRange(-0.2, 0.2);
    turnController.enableContinuousInput(0.0, 360.0);
    turnController.setTolerance(Math.toRadians(0.2));

    kinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.SWERVE_FRAME_LENGTH / 2.0 * 0.0254, Constants.SWERVE_FRAME_WIDTH / 2.0 * 0.0254),
       new Translation2d(Constants.SWERVE_FRAME_LENGTH / 2.0 * 0.0254, -Constants.SWERVE_FRAME_WIDTH / 2.0 * 0.0254),
        new Translation2d(-Constants.SWERVE_FRAME_LENGTH / 2.0 * 0.0254, Constants.SWERVE_FRAME_WIDTH / 2.0 * 0.0254),
         new Translation2d(-Constants.SWERVE_FRAME_LENGTH / 2.0 * 0.0254, -Constants.SWERVE_FRAME_WIDTH / 2.0 * 0.0254)
    );
    poseEstimator = new SwerveDrivePoseEstimator(
      kinematics, 
      Rotation2d.fromDegrees(gyro.getAngle(gyro.getYawAxis())),
       getModulePositions(),
        new Pose2d(),
          VecBuilder.fill(0.01, 0.01, 0.05),
            VecBuilder.fill(0.05, 0.05, 1.0));

    trustVision = false;

    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetOdometry,
      this::getChassisSpeed,
      this::setModuleStates,
      new HolonomicPathFollowerConfig(
        new PIDConstants(28, 0.05, 0.0),
        new PIDConstants(1.4, 0.005, 1.4),
        Constants.MAX_DRIVETRAIN_SPEED * Constants.DRIVE_POSITION_CONVERSION / 60.0,
        Constants.SWERVE_RADIUS / 25.4 / 1000.0,
        new ReplanningConfig()
      ),
      () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
    );
  }

  //help

  public ChassisSpeeds getChassisSpeed() {
    return kinematics.toChassisSpeeds(
      swerveMod[0].getState(),
      swerveMod[1].getState(),
      swerveMod[2].getState(),
      swerveMod[3].getState());
  }

  @Override
  public void periodic() {
    if(LimelightHelpers.getTV("limelight")){
      LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      if(!m_RobotStates.autonomous && DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
        limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight");
      }
      if(limelightMeasurement.avgTagDist <= 4)updateVisionPoseEstimator(limelightMeasurement.pose, limelightMeasurement.timestampSeconds, limelightMeasurement.tagCount);
    }
    for(int i = 0; i<4; i++){
      swerveMod[i].output();
    }

    poseEstimator.updateWithTime(
      Timer.getFPGATimestamp(),
       Rotation2d.fromDegrees(gyro.getAngle(gyro.getYawAxis()) % 360.0),
        getModulePositions()
    );

    double ampdistance = (Math.sqrt(Math.pow(Math.abs(getPose().getX()) - 1.65,2.0) + Math.pow(getPose().getY() - 7.54,2.0)) * 1000.0 / 25.4);
    m_RobotStates.speakDist = ampdistance;
    //values from linear regression given datapoints causes I'm too lazy
    //0 1750
    //3 1800
    //6 1900
    //12 2000
    ampdistance = ampdistance >= 0.0 ? ampdistance : 0.0;
    m_RobotStates.inAmp = ampdistance <= 12;
    m_RobotStates.ampSpeed = (ampdistance >= 1.0 ? 740.16 * Math.log(186.236 * ampdistance + 5334.16) - 4607.85 : 1750.0);
    double Speakerdistance;
    try{
      Speakerdistance = 325 - Math.sqrt(Math.pow(Math.abs(getPose().getX() - 8.308975),2.0) + Math.pow(getPose().getY() - (!m_RobotStates.autonomous && DriverStation.getAlliance().get() == DriverStation.Alliance.Red  ? -1.442593 : 1.442593),2.0)) * 1000 / 25.4;
    } catch(Exception e){
      Speakerdistance = 0;
      System.out.println(e);
    }
    //values from linear regression given datapoints causes I'm too lazy
    //28 3650 -0.1
    //10 3900 -0.1
    //0 4500 0.0
    Speakerdistance = Speakerdistance >= 0.0 ? Speakerdistance : 0.0;
    m_RobotStates.inSpeaker = Speakerdistance <= 20;
    m_RobotStates.speakSpeed = Speakerdistance >= 3 ? 1.01 * (-259.36 * Math.log(0.00721146 * (Speakerdistance) + 0.00791717) + 3245.03) : 4800;
  }

  public Pose2d getPose(){
    return poseEstimator.getEstimatedPosition();
  }

  private SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = swerveMod[0].getPosition();
    positions[1] = swerveMod[1].getPosition();
    positions[2] = swerveMod[2].getPosition();
    positions[3] = swerveMod[3].getPosition();
    return positions;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAX_SPEED_MperS);
    swerveMod[0].setDesiredState(desiredStates[0]);
    swerveMod[1].setDesiredState(desiredStates[1]);
    swerveMod[2].setDesiredState(desiredStates[2]);
    swerveMod[3].setDesiredState(desiredStates[3]);
    //m_FLModule.setDesiredState(desiredStates[0]);
    //m_FRModule.setDesiredState(desiredStates[1]);
    //m_RLModule.setDesiredState(desiredStates[2]);
    //m_RRModule.setDesiredState(desiredStates[3]);
  }

  public void setModuleStates(ChassisSpeeds chassisSpeeds) {
    //*
    SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(secondOrderKinematics(chassisSpeeds));
    //*/SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(chassisSpeeds, 0.02));
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAX_SPEED_MperS);
    swerveMod[0].setDesiredState(desiredStates[0]);
    swerveMod[1].setDesiredState(desiredStates[1]);
    swerveMod[2].setDesiredState(desiredStates[2]);
    swerveMod[3].setDesiredState(desiredStates[3]);
  }

  public ChassisSpeeds secondOrderKinematics(ChassisSpeeds chassisSpeeds) {
    Translation2d translation = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    Translation2d rotAdj = translation.rotateBy(new Rotation2d(-Math.PI / 2.0))
        .times(chassisSpeeds.omegaRadiansPerSecond * Constants.kRotTransFactor);

    translation = translation.plus(rotAdj);

    return new ChassisSpeeds(translation.getX(), translation.getY(), chassisSpeeds.omegaRadiansPerSecond);
  }


  public void resetOdometry(Pose2d pose){
    poseEstimator.resetPosition(
      Rotation2d.fromDegrees(gyro.getAngle(gyro.getYawAxis()) % 360.0),
       getModulePositions(),
        pose
    );
  }

  public void drive(double xSpeed, double ySpeed, double setAngle, boolean lock, boolean speakerLock) {
    //xSpeed = slewX.calculate(xSpeed);
    //ySpeed = slewY.calculate(ySpeed);

    Pose2d position = getPose();
    if(speakerLock){setAngle = -Math.toDegrees(Math.atan2(position.getY() - 5.45, position.getY()));}
    double rot = 0.0;
    if(lock) rot = turnController.calculate(setAngle, position.getRotation().getDegrees());
    rot *= Constants.MAX_SPEED_MperS / new Rotation2d(Constants.SWERVE_FRAME_LENGTH / 2.0 * 0.0254, Constants.SWERVE_FRAME_WIDTH / 2.0 * 0.0254).getRadians();
    setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, position.getRotation()));
  }


  public void resetGyro(){
    gyro.reset();
    poseEstimator.resetPosition(Rotation2d.fromDegrees(0), getModulePositions(), getPose());
    m_RobotStates.gyroReset++;
  }

  public void updateVisionPoseEstimator(Pose2d visionEstimate, double timestamp, int tagNumber){
    //ramp measurement trust based on robot distance
    //poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.1 * Math.pow(15, distance), 0.1 * Math.pow(15, distance), Units.degreesToRadians(20)));
    //if(tagNumber == 1)poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.8,.8, Math.toRadians(20)));
    //if(tagNumber > 1)poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.1,.1,Math.toRadians(5)));
    //0.04 0.04 5
    //0.8 0.8 20
    poseEstimator.addVisionMeasurement(visionEstimate, timestamp);
  }

  private double applyDeadzone(double input, double deadzone) {
    if (Math.abs(input) < deadzone) return 0.0;
    double result = (Math.abs(input) - deadzone) / (1.0 - deadzone);
    return (input < 0.0 ? -result : result);
  }

  public void motorZero(){
    for (int i = 0; i < 4; i++) {
      swerveMod[i].driveMotor.set(0.0);
      swerveMod[i].steerMotor.set(0.0);
    }
  }
}

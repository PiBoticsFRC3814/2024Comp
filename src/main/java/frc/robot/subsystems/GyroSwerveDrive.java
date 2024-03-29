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

  private SwerveDriveKinematics kinematics;
  private SwerveDrivePoseEstimator poseEstimator;
  private ADIS16470_IMU gyro;

  public boolean trustVision;
  public Pose2d currentPose;

  private SwerveModule[] swerveMod = {
    new SwerveModule(0), new SwerveModule(1), new SwerveModule(2), new SwerveModule(3)
  };

  public GyroSwerveDrive(RobotStates robotStates, ADIS16470_IMU gyro) {
    m_RobotStates = robotStates;
    this.gyro = gyro;

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
      this::getSpeeds,
      this::driveUnits,
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

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleState());
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
    positions[1] = swerveMod[3].getPosition();
    positions[2] = swerveMod[1].getPosition();
    positions[3] = swerveMod[2].getPosition();
    return positions;
  }

  public void resetOdometry(Pose2d pose){
    for (int i = 0; i < 4; i++) {
      swerveMod[i].resetModule();
    }
    poseEstimator.resetPosition(
      Rotation2d.fromDegrees(-gyro.getAngle(gyro.getYawAxis()) % 360.0),
       getModulePositions(),
        pose
    );
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

  public void alteredGyroDrive(double dX, double dY, double dZ, double gyroAngle){
    dX = -applyDeadzone(dX, Constants.JOYSTICK_X_DEADZONE);
    dY = -applyDeadzone(dY, Constants.JOYSTICK_Y_DEADZONE);
    //dZ = -applyDeadzone(dZ, Constants.JOYSTICK_Z_DEADZONE);
    if ((dX != 0.0) || (dY != 0.0) || (dZ != 0.0)) {
      gyroDrive(
         dX * m_RobotStates.driveMultiplier,
         dY * m_RobotStates.driveMultiplier,
         -dZ,
          gyroAngle
      );
      m_RobotStates.inFrontOfCubeStation = false;
    } else{
      speed[0] = 0.0;
      speed[1] = 0.0;
      speed[2] = 0.0;
      speed[3] = 0.0;
      setSetpoints();
    }
  }

  public void gyroDrive(double str, double fwd, double rot, double gyroAngle) {
    double angle = m_RobotStates.gyroReset >= 2 ? gyroAngle: poseEstimator.getEstimatedPosition().getRotation().getRadians();
    double intermediary = fwd * Math.cos(angle) + str * Math.sin(angle);
    str = -fwd * Math.sin(angle) + str * Math.cos(angle);
    drive(str, intermediary, rot);
    setSetpoints();
  }

  public void drive(double str, double fwd, double rot) {
    if(str != 0.0 || fwd != 0.0 || rot != 0.0) computeSwerveInputs(str, fwd, rot);
    else{
      speed[0] = 0.0;
      speed[1] = 0.0;
      speed[2] = 0.0;
      speed[3] = 0.0;
    }
    setSetpoints();
  }

  public void driveUnits(ChassisSpeeds driveSpeeds) {
    //driveSpeeds = ChassisSpeeds.discretize(driveSpeeds, 0.2); 
    double str = driveSpeeds.vyMetersPerSecond;
    double fwd = driveSpeeds.vxMetersPerSecond;
    double rot = driveSpeeds.omegaRadiansPerSecond;
    double meterSecToRPM = (1 / Constants.DRIVE_POSITION_CONVERSION * 60.0);
    //System.out.println(fwd);
    drive(str * meterSecToRPM / Constants.MAX_DRIVETRAIN_SPEED, fwd * meterSecToRPM / Constants.MAX_DRIVETRAIN_SPEED, rot);
  }

  public SwerveModuleState[] getModuleState(){
    SwerveModuleState[] positions = new SwerveModuleState[4];
    positions[0] = swerveMod[0].getState();
    positions[1] = swerveMod[3].getState();
    positions[2] = swerveMod[1].getState();
    positions[3] = swerveMod[2].getState();
    return positions;
  }

  public void drive(double[] inputs) {
    drive(inputs[0], inputs[1], inputs[2]);
  }

  /*
   * Brake system
   */
  public void brakeAngle() {
    angle[0] = -0.25;
    angle[1] = 0.25;
    angle[2] = -0.25;
    angle[3] = 0.25;
    speed[0] = 0.0;
    speed[1] = 0.0;
    speed[2] = 0.0;
    speed[3] = 0.0;
    setSetpoints();
  }

  private double getDeltaAngle(double alpha, double beta) {
    return 1.0 - Math.abs(Math.abs(alpha - beta) % 2.0 - 1.0);
  }

  private void computeSwerveInputs(double str, double fwd, double rot) {
    double a = str - rot * (Constants.SWERVE_FRAME_LENGTH / Constants.SWERVE_RADIUS);
    double b = str + rot * (Constants.SWERVE_FRAME_LENGTH / Constants.SWERVE_RADIUS);
    double c = fwd - rot * (Constants.SWERVE_FRAME_WIDTH / Constants.SWERVE_RADIUS);
    double d = fwd + rot * (Constants.SWERVE_FRAME_WIDTH / Constants.SWERVE_RADIUS);

    speed[1] = Math.sqrt((a * a) + (d * d));
    speed[2] = Math.sqrt((a * a) + (c * c));
    speed[0] = Math.sqrt((b * b) + (d * d));
    speed[3] = Math.sqrt((b * b) + (c * c));

    angle[1] = Math.atan2(a, d) / Math.PI;
    angle[2] = Math.atan2(a, c) / Math.PI;
    angle[0] = Math.atan2(b, d) / Math.PI;
    angle[3] = Math.atan2(b, c) / Math.PI;
  }

  private void setSetpoints() {
    for (int i = 0; i < 4; i++) {
      double steerAngle = swerveMod[i].getSteerAngle();
      if (getDeltaAngle(angle[i], steerAngle) > 0.5) {
        angle[i] = Math.abs(Math.abs(angle[i] + 2.0) % 2.0) - 1.0;
        speed[i] = -speed[i];
      }
      swerveMod[i].drive(speed[i], angle[i]);
    }
  }

  public void WheelToCoast() {
    for (int i = 0; i < 4; i++) {
      swerveMod[i].driveMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  public void WheelToBrake() {
    for (int i = 0; i < 4; i++) {
      swerveMod[i].driveMotor.setIdleMode(IdleMode.kBrake);
    }
  }

  public void outputEncoderPos() {
    SmartDashboard.putNumber("Module 1 encoder", swerveMod[0].getSteerAngle());
    SmartDashboard.putNumber("Module 2 encoder", swerveMod[1].getSteerAngle());
    SmartDashboard.putNumber("Module 3 encoder", swerveMod[2].getSteerAngle());
    SmartDashboard.putNumber("Module 4 encoder", swerveMod[3].getSteerAngle());
  }

  public void motorZero(){
    for (int i = 0; i < 4; i++) {
      swerveMod[i].driveMotor.set(0.0);
      swerveMod[i].steerMotor.set(0.0);
    }
  }
}
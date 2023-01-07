package frc.robot.SubSystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//MOTORS
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//UTIL
import frc.robot.Util.Swerve;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.fasterxml.jackson.databind.PropertyNamingStrategies.LowerCamelCaseStrategy;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;

//MATHU
import java.math.*;
import java.util.ArrayList;

public class Drive extends SubsystemBase {
  // TODO ADD TRAJECTORY
  // https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/trajectory-tutorial/index.html
  private static Drive drive;
  public static double frontLeftSwerveSpeed;
  public static double frontLeftSwerveAngle;
  public static double frontRightSwerveSpeed;
  public static double frontRightSwerveAngle;
  public static double backLeftSwerveSpeed;
  public static double backLeftSwerveAngle;
  public static double backRightSwerveSpeed;
  public static double backRightSwerveAngle;
  public static PIDController PIDDFL;
  public static PIDController PIDAFL;
  public static PIDController PIDDFR;
  public static PIDController PIDAFR;
  public static PIDController PIDDBL;
  public static PIDController PIDABL;
  public static PIDController PIDDBR;
  public static PIDController PIDABR;
  public static PIDController angleRatePID;
  public static PIDController toAnglePID;
  public static PIDController smallAngle;
  public AHRS navX;
  public Swerve frontLeft;
  public Swerve frontRight;
  public Swerve backRight;
  public Swerve backLeft;
  public double gyroAngle;
  public AnalogGyro gyro;
  public SwerveDriveOdometry m_odometry;
  public double maxTurnSpeed;
  public double angleRateVector;
  double targetAngleRate;
  public boolean angle = false;
  public boolean flipHeading = false;

  // Swerve drive auton
  SwerveDriveKinematics m_kinematics;
  public double velX;
  public double velY;
  public double omega;
  Pose2d swervePose;
  Translation2d m_frontLeftLocation;
  Translation2d m_frontRightLocation;
  Translation2d m_backLeftLocation;
  Translation2d m_backRightLocation;
  ChassisSpeeds swerveSpeed;

  Timer autoTimer;
  double prevTime;
  public static Drive get_Instance() {

    if (drive == null) {
      drive = new Drive();
    }
    return drive;
  }

  public Drive() {
    navX = new AHRS(SPI.Port.kMXP, (byte) 200);
    navX.calibrate();
    navX.resetDisplacement();
    gyro = new AnalogGyro(0);
    autoTimer = new Timer();
    angleRatePID = new PIDController(Constants.aRP, Constants.aRI, Constants.aRD);

    toAnglePID = new PIDController(Constants.kPTA, Constants.kITA, Constants.kDTA);
    smallAngle = new PIDController(Constants.smallP, Constants.smallI, Constants.smallD);

    PIDDFL = new PIDController(Constants.dPFL, Constants.dIFL, Constants.dDFL);
    PIDAFL = new PIDController(Constants.aPFL, Constants.aIFL, Constants.aDFL);
    PIDDFR = new PIDController(Constants.dPFR, Constants.dIFR, Constants.dDFR);
    PIDAFR = new PIDController(Constants.aPFR, Constants.aIFR, Constants.aDFR);
    PIDDBL = new PIDController(Constants.dPBL, Constants.dIBL, Constants.dDBL);
    PIDABL = new PIDController(Constants.aPBL, Constants.aIBL, Constants.aDBL);
    PIDDBR = new PIDController(Constants.dPBR, Constants.dIBR, Constants.dDBR);
    PIDABR = new PIDController(Constants.aPBR, Constants.aIBR, Constants.aDBR);
    
    frontLeft = new Swerve(Constants.motorIdDriveFrontLeft, Constants.motorIdAngleFrontLeft,
        Constants.encoderIdFrontLeft, PIDDFL, PIDAFL, Constants.flHome);
    frontRight = new Swerve(Constants.motorIdDriveFrontRight, Constants.motorIdAngleFrontRight,
        Constants.encoderIdFrontRight, PIDDFR, PIDAFR, Constants.frHome);
    backRight = new Swerve(Constants.motorIdDriveBackRight, Constants.motorIdAngleBackRight,
        Constants.encoderIdBackRight, PIDDBR, PIDABR, Constants.brHome);
    backLeft = new Swerve(Constants.motorIdDriveBackLeft, Constants.motorIdAngleBackLeft, Constants.encoderIdBackLeft,
        PIDDBL, PIDABL, Constants.blHome);
    
    frontLeft.resetDriveEnc();
    frontRight.resetDriveEnc();
    backLeft.resetDriveEnc();
    backRight.resetDriveEnc();
  }

  public void autonInit() {
    prevTime = 0;
    m_frontLeftLocation = new Translation2d(-Constants.modLoc, -Constants.modLoc);
    m_frontRightLocation = new Translation2d(Constants.modLoc, -Constants.modLoc);
    m_backLeftLocation = new Translation2d(-Constants.modLoc, Constants.modLoc);
    m_backRightLocation = new Translation2d(Constants.modLoc, Constants.modLoc);

    m_kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    
    m_odometry = new SwerveDriveOdometry(m_kinematics,
        new Rotation2d(Math.toRadians(-navX.getYaw())));//, new Pose2d(0, 0, new Rotation2d(Math.toRadians(-navX.getYaw()))));

  }

  public void manualSpeeds(double x, double y, double omega) {
    velY = y;
    velX = x;
    this.omega = omega;
  }

  public void autonUpdate() {
    autoTimer.start();
    double currTime = autoTimer.get();
    SmartDashboard.putNumber("period", currTime - prevTime);
    prevTime = currTime;

    SmartDashboard.putNumber("odo theta", m_odometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("Gyro Angle", navX.getYaw());
    if(autoTimer.get() > 0){
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(velY, velX, omega, Rotation2d.fromDegrees(-navX.getYaw()));
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    // Front left module state
    SwerveModuleState backRightState = moduleStates[0];

    // Front right module state
    SwerveModuleState frontRightState = moduleStates[1];

    // Back left module state
    SwerveModuleState backLeftState = moduleStates[2];

    // Back right module state
    SwerveModuleState frontLeftState = moduleStates[3];

    frontRight.modState(frontRightState);
    frontLeft.modState(frontLeftState);
    backRight.modState(backRightState);
    backLeft.modState(backLeftState);
    }

    var gyroAngle = Rotation2d.fromDegrees(-navX.getYaw());

    SmartDashboard.putNumber("odo gyroangle", gyroAngle.getDegrees());

    swervePose = m_odometry.updateWithTime(autoTimer.get(), gyroAngle, frontLeft.getState(), frontRight.getState(),
        backLeft.getState(), backRight.getState());

    SmartDashboard.putNumber("fr Vel a", frontRight.getState().angle.getDegrees());
    SmartDashboard.putNumber("fl Vel a", frontLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("br Vel a", backRight.getState().angle.getDegrees());
    SmartDashboard.putNumber("bl Vel a", backLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("fr Vel", frontRight.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("fl Vel", frontLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("br Vel", backRight.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("bl Vel", backLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("fr distance", frontRight.getDriveEncoder() * Constants.velocityToMeters / 10);
    SmartDashboard.putNumber("fl distance", frontLeft.getDriveEncoder() * Constants.velocityToMeters / 10);
    SmartDashboard.putNumber("br distance", backRight.getDriveEncoder() * Constants.velocityToMeters / 10);
    SmartDashboard.putNumber("bl distance", backLeft.getDriveEncoder() * Constants.velocityToMeters / 10);

  }

  public double properSanitize(double tarTheta, double currTheta){
    double lowError = Math.abs((tarTheta - 2 * Math.PI) - currTheta);
    SmartDashboard.putNumber("low", lowError);
    double highError = Math.abs((tarTheta + 2 * Math.PI) - currTheta);
    double error = Math.abs((tarTheta) - currTheta);
    SmartDashboard.putNumber("High", highError);
    SmartDashboard.putNumber("er", error);
    SmartDashboard.putNumber("angle rad", currTheta);
    if(lowError < error && lowError < highError){
      return tarTheta - 2 * Math.PI;
    }else if(highError < error && highError < lowError){
      return tarTheta + 2 * Math.PI;
    }else{
      return tarTheta;
    }


  }
  public void resetSwerveOdometry() {
    m_odometry.resetPosition(new Pose2d(0, 0, new Rotation2d(navX.getYaw())), new Rotation2d(navX.getYaw()));
  }

  public void homeSwerve() {
    drive.setVector(180, 0, 0);
  }

  public void flipFusedHeading() {
    flipHeading = !flipHeading;
  }

  public void setSwerve(double angleVectorX, double angleVectorY, double rotationVectorX) {

    boolean isDeadZone = Math.abs(angleVectorX) < .15 &&
        Math.abs(angleVectorY) < .15 &&
        Math.abs(rotationVectorX) < 1;
    if (!isDeadZone) {
      double rotationVectorY = rotationVectorX;
      double A = angleVectorX + rotationVectorX;// THE PLUS AND MINUS MAY BE FLIPPED
      double B = angleVectorX - rotationVectorX;
      double C = angleVectorY + rotationVectorY;
      double D = angleVectorY - rotationVectorY;

      frontLeftSwerveSpeed = Math.sqrt(Math.pow(A, 2.0) + Math.pow(C, 2.0));
      frontLeftSwerveAngle = Math.atan2(A, C) * 180 / Math.PI;
      backLeftSwerveSpeed = Math.sqrt(Math.pow(A, 2.0) + Math.pow(D, 2.0));
      backLeftSwerveAngle = Math.atan2(A, D) * 180 / Math.PI;
      frontRightSwerveSpeed = Math.sqrt(Math.pow(B, 2.0) + Math.pow(C, 2.0));
      frontRightSwerveAngle = Math.atan2(B, C) * 180 / Math.PI;
      backRightSwerveSpeed = Math.sqrt(Math.pow(B, 2.0) + Math.pow(D, 2.0));
      backRightSwerveAngle = Math.atan2(B, D) * 180 / Math.PI;
      this.angle = false;
    } else {
      frontLeftSwerveSpeed = 0;
      frontRightSwerveSpeed = 0;
      backLeftSwerveSpeed = 0;
      backLeftSwerveSpeed = 0;
    }
    // SET ALL OF THE NUMBERS FOR THE SWERVE VARS
  }

  public void breakMode() {
    frontRight.setDriveModeBrake();
    frontLeft.setDriveModeBrake();
    backRight.setDriveModeBrake();
    backLeft.setDriveModeBrake();
  }

  public void coastMode() {
    frontRight.setDriveModeCoast();
    frontLeft.setDriveModeCoast();
    backRight.setDriveModeCoast();
    backLeft.setDriveModeCoast();
  }

  public void setVector(double angle, double mag, double rotationVectorX) {
    SmartDashboard.putNumber("anlge 12", angle);
    double angleVX = Math.cos((angle - gyroAngle) * Math.PI / 180) * 180 / Math.PI * mag;// TODO CHECK about RAD VS DEG
    double angleVY = Math.sin((angle - gyroAngle) * Math.PI / 180) * 180 / Math.PI * mag;
    targetAngleRate = rotationVectorX;
    setSwerve(angleVX, angleVY, angleRateVector);
  }

  public void sanitizeAngle() {
    gyroAngle = navX.getYaw();
    if (flipHeading) {
      gyroAngle = gyroAngle + 180;
    }
    while (gyroAngle > 360) {
      gyroAngle = gyroAngle - 360;
    }
    while (gyroAngle < 0) {
      gyroAngle = gyroAngle + 360;
    }
    SmartDashboard.putNumber("fused heading", gyroAngle);

  }

  double xDis;

  public void xDisplacement() {
    xDis = xDis + 0.05 * navX.getVelocityX();
  }

  public void angleRatePID(double target) {
    target = target * 30;// Takes in value -1 - 1 and turns it into max / min 30/-30
    angleRateVector = -1 * angleRatePID.calculate(navX.getRate(), target);
  }

  public double anglePID(double target) {
    SmartDashboard.putNumber("calculated", toAnglePID.calculate(gyroAngle, target));
    if (Math.abs(target - gyroAngle) < 20) {
      return smallAngle.calculate(gyroAngle, target);
    } else {
      return toAnglePID.calculate(gyroAngle, target);
    }
  }

  public void modToAngle(double angle) {
    this.angle = true;
    angle = angle + 90;
    frontLeftSwerveAngle = angle;
    frontRightSwerveAngle = angle;
    backLeftSwerveAngle = angle;
    backRightSwerveAngle = angle;
  }

  public void setManual(double speed, double angle) {
    frontRight.setAngleSpeed(frontRight.anglePIDCalcABS(angle));
    frontRight.setMetersPerSec(speed);
    frontLeft.setAngleSpeed(frontLeft.anglePIDCalcABS(angle));
    frontLeft.setMetersPerSec(speed);
    backRight.setAngleSpeed(backRight.anglePIDCalcABS(angle));
    backRight.setMetersPerSec(speed);
    backLeft.setAngleSpeed(backLeft.anglePIDCalcABS(angle));
    backLeft.setMetersPerSec(speed);
    SmartDashboard.putNumber("fr Vel", frontRight.getMetersPerSec());
    SmartDashboard.putNumber("fl Vel", frontLeft.getMetersPerSec());
    SmartDashboard.putNumber("br Vel", backRight.getMetersPerSec());
    SmartDashboard.putNumber("bl Vel", backLeft.getMetersPerSec());
    SmartDashboard.putNumber("fr", frontRight.getEncoder());
    SmartDashboard.putNumber("fl", frontLeft.getEncoder());
    SmartDashboard.putNumber("br", backRight.getEncoder());
    SmartDashboard.putNumber("bl", backLeft.getEncoder());
  }

  public void periodicTele() {
    /*
     * //System.out.println(frontLeftSwerveSpeed/300);
     * //+180+Constants.flHome
     * //System.out.println(frontLeft.getEncoder() + "FL");
     * //System.out.println(frontRight.getEncoder() + "FR");
     * //System.out.println(backLeft.getEncoder() + "BL");
     * //System.out.println(backRight.getEncoder() + "BR");
     */sanitizeAngle();
    angleRatePID(targetAngleRate);
    SmartDashboard.putNumber("pitch", navX.getPitch());
    SmartDashboard.putNumber("fr Vel", frontRight.getMetersPerSec());
    SmartDashboard.putNumber("fl Vel", frontLeft.getMetersPerSec());
    SmartDashboard.putNumber("br Vel", backRight.getMetersPerSec());
    SmartDashboard.putNumber("bl Vel", backLeft.getMetersPerSec());
    SmartDashboard.putNumber("fr", frontRight.getEncoder());
    SmartDashboard.putNumber("fl", frontLeft.getEncoder());
    SmartDashboard.putNumber("br", backRight.getEncoder());
    SmartDashboard.putNumber("bl", backLeft.getEncoder());
    /*
     * 
     * //SmartDashboard.putNumber("input", navX.getFusedHeading());
     * //SmartDashboard.putNumber("output", navX.getRate());
     * //SmartDashboard.putNumber("c", targetAngleRate/navX.getRate());
     * /*System.out.println(backRightSwerveAngle);
     * System.out.println(backRightSwerveSpeed);
     * System.out.println(backLeftSwerveAngle);
     * System.out.println(backLeftSwerveSpeed);
     * System.out.println(frontRightSwerveAngle);
     * System.out.println(frontRightSwerveSpeed);
     * System.out.println(frontLeftSwerveAngle);
     * System.out.println(frontLeftSwerveSpeed);
     * //Get my gyro angle. We are negating the value because gyros return positive
     * //values as the robot turns clockwise. This is not standard convention that
     * is
     * //used by the WPILib classes.
     * //Update the pose
     */
    if (angle) {
      frontLeft.setSpeed(0, frontLeft.anglePIDCalcABS(frontLeftSwerveAngle));
      frontRight.setSpeed(0, frontRight.anglePIDCalcABS(frontRightSwerveAngle));
      backLeft.setSpeed(0, backLeft.anglePIDCalcABS(backLeftSwerveAngle));
      backRight.setSpeed(0, backRight.anglePIDCalcABS(backRightSwerveAngle));
    } else {
      frontLeft.setSpeed(frontLeftSwerveSpeed / 75, frontLeft.anglePIDCalcABS(frontLeftSwerveAngle));
      frontRight.setSpeed(frontRightSwerveSpeed / 75, frontRight.anglePIDCalcABS(frontRightSwerveAngle));
      backLeft.setSpeed(backLeftSwerveSpeed / 75, backLeft.anglePIDCalcABS(backLeftSwerveAngle));
      backRight.setSpeed(backRightSwerveSpeed / 75, backRight.anglePIDCalcABS(backRightSwerveAngle));
    }

    SmartDashboard.putNumber("fr", frontRight.getEncoder());
    SmartDashboard.putNumber("fl", frontLeft.getEncoder());
    SmartDashboard.putNumber("br", backRight.getEncoder());
    SmartDashboard.putNumber("bl", backLeft.getEncoder());
    SmartDashboard.putNumber("fr angle", frontRightSwerveAngle);

  }

}
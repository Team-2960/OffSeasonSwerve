package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

public class Constants{
    public double robotWidth = 0;
    public double robotLength = 0;


    //OI Constants
    public static int driverControl = 1;
    public static int operatorControl =2;
  

    //Shooter Constants
    public static double uWP = 0.550585/16;//0.68585
    public static double uWI = 4.5529 * Math.pow(10, -5);
    public static double uWD = 2.1277* Math.pow(10, -6);
    public static double lWP = 0.550587/16;//0.51587
    public static double lWI = 4.5532*Math.pow(10, -5);
    public static double lWD = 2 * Math.pow(10, -6);
    public static double lowGoalRPM = 7500;
    public static double edgeTarmacRPM = 11500;

    public static double uWPP = 0.00012;//0.68585
    public static double uWII = 0;
    public static double uWDD = 0.0000016;//0.0000013
    public static double lWPP = 0.0002;//0.51587
    public static double lWII = 0;
    public static double lWDD = 0.0000014;//0.0000013

    public static double uTarmacRPM = 1;
    public static double lTarmacRPM = 1;
    public static int mUpperShooter = 16;
    public static int mLowerShooter = 17;



    //Drive Constants
    public static double modLoc = 11.5  * 0.0254;
    public static double autoSwerveTol = 1;

    public static double thresholdX1 = 0.03;
    public static double xVel1 = 0;

    public static double thresholdX2 = 0.3;
    public static double xVel2 = 0.8;

    public static double thresholdX3 = 1;
    public static double xVel3 = 2;

    public static double xVelOutside = 2.3;


    public static double thresholdY1 = 0.03;
    public static double yVel1 = 0;

    public static double thresholdY2 = 0.3;
    public static double yVel2 = 0.8;

    public static double thresholdY3 = 1;
    public static double yVel3 = 2;

    public static double yVelOutside = 2.6;


    public static double thresholdT1 = 0.1;
    public static double tVel1 = 0;

    public static double thresholdT2 = 0.4;
    public static double tVel2 = 1.5;

    public static double thresholdT3 = 0.7;
    public static double tVel3 = 2.1;

    public static double tVelOutside = 1.5 * Math.PI;

    public static double aRP = .3;
    public static double aRI = 0;
    public static double aRD = 0;
    public static double dPFL = 0.0008;
    public static double dIFL = 0;
    public static double dDFL = 0;
    public static double aPFL = 0.007;
    public static double aIFL = 0;
    public static double aDFL = 0;
    public static double dPFR = 0.0008;
    public static double dIFR = 0;
    public static double dDFR = 0;
    public static double aPFR = 0.007;
    public static double aIFR = 0;
    public static double aDFR = 0;
    public static double dPBL = 0.0008;
    public static double dIBL = 0;
    public static double dDBL = 0;
    public static double aPBL = 0.007;
    public static double aIBL = 0;
    public static double aDBL = 0.000000;
    public static double dPBR = 0.0008;
    public static double dIBR = 0;
    public static double dDBR = 0;
    public static double aPBR = 0.007;
    public static double aIBR = 0;
    public static double aDBR = 0;
    public static double flHome = -6.5;//-7.27;
    public static double frHome = -70;//-269.19;
    public static double blHome = -28;//-73.81;
    public static double brHome = -21;//-157.03;

    public static int motorIdDriveFrontLeft = 5;
    public static int motorIdAngleFrontLeft = 6;
    public static int motorIdDriveFrontRight = 7;
    public static int motorIdAngleFrontRight = 8;
    public static int motorIdDriveBackRight = 0;
    public static int motorIdAngleBackRight = 2;
    public static int motorIdDriveBackLeft = 3;
    public static int motorIdAngleBackLeft = 4;
    public static int encoderIdFrontLeft = 12;
    public static int encoderIdFrontRight = 11;
    public static int encoderIdBackLeft = 9;
    public static int encoderIdBackRight = 10;

    //toAnglePID
    public static double kPTA = 0.013;
    public static double kITA = 0;
    public static double kDTA = 0;
    public static double smallP = 0.1;
    public static double smallI = 0;
    public static double smallD = 0;
    public static double angleTolerance = 1;




    //ADD PIDs FOR SWERVE

    //CAMERA CONSTANTS
    public final static int cameraPort = 0;
    public final static int cWidth = 640;
    public final static int cHeight = 480;
    public final static double horizontalViewAngle = 61;
    public final static double verticalViewAngle = 20.55;
    public final static double deg_per_px = verticalViewAngle / cHeight;

    //Climber Constants
    public final static int climbRArmSolenoid1 = 2;
    public final static int climbRArmSolenoid2 = 3;
    public final static int climbLArmSolenoid1 = 4;
    public final static int climbLArmSolenoid2 = 5;
    public final static int climbHookSolenoid1 = 6;
    public final static int climbHookSolenoid2 = 7;
    public final static int mClimbL = 13;
    public final static int mClimbR = 14;
    public final static double winchExtendPos = 11500;
    public final static double winchExtendLimit = 490000;
    //17
    public final static double winchContractPos = 1500;
    public final static double winchLvl3Pos = 5000;

    public final static int rHallEffectSensor = 2;
    public final static int lHallEffectSensor = 3;
    public final static int limitSwitchPort = 4;

    public final static double climbPitchLow = 5;
    public final static double climbPitchHigh = 0;

    public final static int climbEncoderPortA = 5;
    public final static int climbEncoderPortB = 6;

    public final static int sPusher = 8;

    //Intake Constants
    public final static int intakeMotor = 18;
    public final static int intakeSolenoid1 = 0;
    public final static int intakeSolendoid2 = 1;

    //Index Constants
    public final static int photoeye1 = 0;
    public final static int photoeye2 = 1;
    public final static int indexMotor = 15;

    //Limelight Constants
    public final static double h1 = 32;//HEIGHT OF LIMELIGHT
    public final static double a1 = 26;//ANGLE FROM HORIZONTAL
    public final static double h2 = 104;//HEIGHT OF TARGET

    //AUTON CONSTANTS
    public final static double autoSpeed = 0.3;
    public final static double sensorConv = 1;

    public final static double xToPosTolerance = 0.03;
    public final static double yToPosTolerance = 0.03;
    public final static double thetaToPosTolerance = 0.1;

    public final static double velocityToMeters = (10/8.16*Math.PI*3.9*0.0254)/(2048);

    public final static double autonCamera = Math.PI/22;

}
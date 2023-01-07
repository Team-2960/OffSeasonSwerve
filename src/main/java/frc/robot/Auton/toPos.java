package frc.robot.Auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubSystems.*;
import frc.robot.*;

public class toPos extends CommandBase{
    //shoot the ball
   
    Drive drive = Drive.get_Instance();
    OI oi = new OI();
   
    double y;
    double x;
    double theta;
    double wheelVelX = 0;
    double wheelVelY = 0;
    double initTheta = 0;
    double deltaTheta = 0;
    double omega;
    Timer timer = new Timer();
    //OFF OF POS VARS
    double tarX = 1000000;
    double tarY = 10000000;
    double tarTheta = 1000000;
    //x and y are relative to the starting location and time is relative to the start of the command
    //Meters and degrees are the units
    public toPos(double x, double y, double theta){
        this.x = x;
        this.y = y;
        this.theta = Math.toRadians(theta);
    }

    @Override
    public void initialize() {
        drive.resetSwerveOdometry();
    }

    /**
     * Returns true if all the commands in this group have been started and have
     * finished.
     * <p>
     * <p>
     * Teams may override this method, although they should probably reference
     * super.isFinished() if they do.
     * </p>
     *
     * @return whether this {@link CommandGroup} is finished
     */
    @Override
    public boolean isFinished() {
        //IF ALL OF THE TARGET VALUES ARE LESS THAN TOLERANCE THEN RETURN TRUE
        if (Math.abs(tarX) < Constants.xToPosTolerance && Math.abs(tarY) < Constants.yToPosTolerance && Math.abs(tarTheta) < Constants.thetaToPosTolerance)
            return true;
        else 
            return false;
    }

    @Override
    public void execute() {
        double currX = drive.m_odometry.getPoseMeters().getY();
        double currY = drive.m_odometry.getPoseMeters().getX();
        SmartDashboard.putNumber("odo X", currX);
        SmartDashboard.putNumber("odo Y", currY);
        double currTheta = Math.toRadians(drive.navX.getYaw());
        tarX = x - currX;
        tarY = y - currY;
        double sanTheta = drive.properSanitize(theta, currTheta);
        tarTheta = sanTheta - currTheta;
        SmartDashboard.putNumber("sanitize angle", Math.toDegrees(sanTheta));
        SmartDashboard.putNumber("angle", Math.toDegrees(theta));
        double finalVelX = 0;
        double finalVelY = 0;
        double finalOmega = 0;

        //SETS X VELOCITY BASED ON HOW FAR AWAY FROM THE DESIRED X COORD WE ARE
        if(Math.abs(tarX) < Constants.thresholdX1){
            finalVelX = Constants.xVel1;
        }else if(Math.abs(tarX) <Constants.thresholdX2){
            finalVelX = Constants.xVel2;
        }else if(Math.abs(tarX) < Constants.thresholdX3){
            finalVelX = Constants.xVel3;
        }else{
            finalVelX = Constants.xVelOutside;
        }

        //THE SIGNS MIGHT NEED TO BE FLIPPED
        //THIS PART OF THE CODE ADJUSTS THE DIRECTION OF THE VELOCITY SO THAT IT GOES THE CORRECT DIRECTION BASED ON WHETHER OUR ERROR IS POSITIVE OR NEGATIVE
        if(tarX < 0){
            drive.velX = -1 * finalVelX;
        }else{
            drive.velX = finalVelX;
        }


        //SETS THE Y VELOCITY BASED ON HOW CLOSE WE ARE TO THE DESIRED Y COORD
        if(Math.abs(tarY) < Constants.thresholdY1){
            finalVelY = Constants.yVel1;
        }else if(Math.abs(tarY) <Constants.thresholdY2){
            finalVelY = Constants.yVel2;
        }else if(Math.abs(tarY) < Constants.thresholdY3){
            finalVelY = Constants.yVel3;
        }else{
            finalVelY = Constants.yVelOutside;
        }
        //THE SIGNS MIGHT NEED TO BE FLIPPED
        //THIS PART OF THE CODE ADJUSTS THE DIRECTION OF THE VELOCITY SO THAT IT GOES THE CORRECT DIRECTION BASED ON WHETHER OUR ERROR IS POSITIVE OR NEGATIVE
        if(tarY < 0){
            drive.velY = -1 * finalVelY;
        }else{
            drive.velY = finalVelY;
        }
        //NOTE tarTheta IS IN DEGREES BUT OUTPUT WILL BE IN RAD/SEC
        //This part sets the omega based on how far we are from the desired theta
        if(Math.abs(tarTheta) < Constants.thresholdT1){
            finalOmega = Constants.tVel1;
        }else if(Math.abs(tarTheta) <Constants.thresholdT2){
            finalOmega = Constants.tVel2;
        }else if(Math.abs(tarTheta) < Constants.thresholdT3){
            finalOmega = Constants.tVel3;
        }else{
            finalOmega = Constants.tVelOutside;
        }      

        //THE SIGNS MIGHT NEED TO BE FLIPPED
        //THIS PART OF THE CODE ADJUSTS THE DIRECTION OF THE OMEGA SO THAT IT GOES THE CORRECT DIRECTION BASED ON WHETHER OUR ERROR IS POSITIVE OR NEGATIVE
        if(tarTheta < 0){
            drive.omega = -1 * finalOmega;
        }else{
            drive.omega = finalOmega;
        }
    }

   
    /**
     * @param interrupte
     */
    @Override
    public void end(boolean interrupte) {
        System.out.println("finished com");
        drive.omega = 0;
        drive.velX = 0;
        drive.velY = 0;
    }
    
}
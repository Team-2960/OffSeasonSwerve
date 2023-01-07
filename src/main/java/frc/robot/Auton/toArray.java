package frc.robot.Auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubSystems.*;
import frc.robot.Util.autoPathPoint;
import frc.robot.*;


public class toArray extends CommandBase{
    //shoot the ball
   
    Drive drive = Drive.get_Instance();
    OI oi = new OI();
    Lime lime = Lime.get_Instance();
    MegaShooter2PointO mega = MegaShooter2PointO.get_Instance();
   
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
    Pose2d currPose;
    autoPathPoint array[];
    int index = 0;
    //x and y are relative to the starting location and time is relative to the start of the command
    //Meters and degrees are the units
    public toArray(autoPathPoint array[]){
        this.array = array;
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
        return (index == array.length);
    }

    @Override
    public void execute() {
        if(index < array.length){
            x = array[index].pose.getX();
            y = array[index].pose.getY();
            theta = array[index].pose.getRotation().getRadians();
        }
        if(array[index].isIntakeOn){
            mega.intakeOn();
        }else{
            mega.intakeOff();
        }
        
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
        if(index != array.length-1){
        //SETS X VELOCITY BASED ON HOW FAR AWAY FROM THE DESIRED X COORD WE ARE
        if(Math.abs(tarX) < Constants.thresholdX1){
            finalVelX = Constants.xVel2-0.5;
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
            finalVelY = Constants.yVel2-0.5;
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
    if(index == array.length - 1){
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
        if(array[index].isCameraTracking && lime.isSeeTar()){
            drive.omega = -1 * lime.getHorOffset() * Constants.autonCamera;
        }

        if (Math.abs(tarX) < array[index].posTolerance && Math.abs(tarY) < array[index].posTolerance && (Math.abs(tarTheta) < array[index].angleTolerance || (array[index].isCameraTracking && Math.abs(lime.getHorOffset()) < 1))){
            index++;
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
        mega.intakeOff();
    }
}
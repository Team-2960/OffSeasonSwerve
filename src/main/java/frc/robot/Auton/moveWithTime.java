package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubSystems.Drive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;


public class moveWithTime extends CommandBase{

    
    private boolean isFinish = false;
    private double distance;
    private double theta;
    private Drive drive;
    private double speed;
    public double x = 0;
    private double time;
    private Timer timer;
    public boolean reversed = false;
    public 
    moveWithTime(double time, double theta, double speed){//Distannce in feet
        this.theta = theta + 90;
        drive = Drive.get_Instance();
        this.speed = speed;
        this.time = time;
        timer = new Timer();
        timer.start();
        if(speed > 0){
            reversed = true;
        }else{
            reversed = false;
        }
    }

    @Override
    public void initialize() {
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
        return timer.get() > time;
    }

    @Override
    public void execute() {
        drive.backLeftSwerveAngle = theta;
        drive.backRightSwerveAngle = theta;
        drive.frontLeftSwerveAngle = theta;
        drive.frontRightSwerveAngle = theta;
        drive.frontLeftSwerveSpeed = speed * -75;
        drive.frontRightSwerveSpeed = speed * -75;
        drive.backLeftSwerveSpeed = speed * -75;
        drive.backRightSwerveSpeed = speed * -75;
        SmartDashboard.putNumber("fr ticks", drive.frontRight.getDriveEncoder());
        SmartDashboard.putNumber("distance auto", distance);

    }

    
    /** 
     * @param interrupte
     */
    @Override
    public void end(boolean interrupte) {
        drive.frontLeftSwerveSpeed = 0;
        drive.frontRightSwerveSpeed = 0;
        drive.backLeftSwerveSpeed = 0;
        drive.backRightSwerveSpeed = 0;
    }
}
package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubSystems.Drive;
import frc.robot.SubSystems.Lime;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;


public class turnTillTarget extends CommandBase{
    //shoot the ball
    
    
    private boolean isFinish = false;
    private double theta;
    private Drive drive;
    private TrapezoidProfile trapProfile;
    private boolean isLeft = false;
    private Lime lime;

    Timer timer;
    private double time;
    public turnTillTarget(){
        this.time = time;
        isLeft = true;
        drive = Drive.get_Instance();
        this.theta = theta + drive.navX.getFusedHeading();
        while(this.theta > 360){
          this.theta = this.theta - 360;
        }
        while(this.theta < 0){
          this.theta = this.theta + 360;
        }
        timer = new Timer();
        trapProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(1, 1),
                                           new TrapezoidProfile.State(theta, 0),
                                           new TrapezoidProfile.State(drive.navX.getFusedHeading(), 0));
        lime = Lime.get_Instance();

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
        return timer.get() > 0.2;
        //return Math.abs(theta - drive.gyroAngle) < Constants.angleTolerance; //|| trapProfile.totalTime() < timer.get();
    }

    @Override
    public void execute() {
    boolean isSeetarget = lime.isSeeTar();
    if(isSeetarget){
        timer.start();
    }else{
        timer.stop();
    }

    if(isLeft){
        drive.setVector(0, 0, -2);
    }else{
     drive.setVector(0, 0, 2);
        }
        drive.periodic();
        SmartDashboard.putNumber("Timer", timer.get());
        SmartDashboard.putBoolean("isSeetarget", isSeetarget);
    }

    
    /** 
     * @param interrupte
     */
    @Override
    public void end(boolean interrupte) {
        drive.setVector(0, 0, 0);
        drive.frontLeftSwerveSpeed = 0;
        drive.frontRightSwerveSpeed = 0;
        drive.backLeftSwerveSpeed = 0;
        drive.backRightSwerveSpeed = 0;
        drive.periodic();

    }
}
package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubSystems.Drive;
import frc.robot.SubSystems.Lime;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.*;


public class turnToTarget extends CommandBase{
    //shoot the ball
    
    
    private boolean isFinish = false;
    private double theta;
    private Drive drive;
    private TrapezoidProfile trapProfile;
    private boolean isLeft = false;
    public Lime lime;

    Timer timer;
    private double time;
    public turnToTarget(){
        
        drive = Drive.get_Instance();
        timer = new Timer();
        lime = Lime.get_Instance();
    }

    @Override
    public void initialize() {
        timer.start();
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
        return lime.isSeeTar();
        //return Math.abs(theta - drive.gyroAngle) < Constants.angleTolerance; //|| trapProfile.totalTime() < timer.get();
    }

    @Override
    public void execute() {
        var setpoint = trapProfile.calculate(timer.get());
        if(isLeft){
            drive.setVector(0, 0, -1);
        }else{
            drive.setVector(0, 0, 2);
        }
        drive.periodic();
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
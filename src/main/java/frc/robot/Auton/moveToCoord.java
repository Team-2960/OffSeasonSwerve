package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubSystems.Drive;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.*;


public class moveToCoord extends CommandBase{
    //shoot the ball
    
    Drive drive = Drive.get_Instance();
    OI oi = new OI();
    
    private boolean isFinish = false;
    double y;
    double x;
    double theta;
    double mag;
    Timer timer = new Timer();
    public moveToCoord(double x, double y){
        this.x = x;
        this.y = y;
    }

    @Override
    public void initialize() {

        theta = Math.atan2(x,y) + 2*Math.PI;
        mag = Math.sqrt(x*x + y*y);
        super.initialize();
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
        if (isFinish)
            return true;
        else
            return false;
    }

    @Override
    public void execute() {
        drive.setVector(oi.driveAngle(0.00001, 1), 0.5, 0);
        drive.periodic();

    }

    
    /** 
     * @param interrupte
     */
    @Override
    public void end(boolean interrupte) {
        //drive.setVector(0, 0, 0);
    }
}
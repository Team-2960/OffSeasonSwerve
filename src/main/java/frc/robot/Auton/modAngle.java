package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubSystems.Drive;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.*;

public class modAngle extends CommandBase {
    // shoot the ball

    private boolean isFinish = false;
    private double theta;
    private Drive drive;
    Timer timer;

    public modAngle(double theta) {
        this.theta = theta + 90;
        drive = Drive.get_Instance();
        timer = new Timer();
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
        return timer.get() > 1;
    }

    @Override
    public void execute() {
        drive.backLeftSwerveAngle = theta;
        drive.backRightSwerveAngle = theta;
        drive.frontLeftSwerveAngle = theta;
        drive.frontRightSwerveAngle = theta;
        drive.frontLeftSwerveSpeed = 0;
        drive.frontRightSwerveSpeed = 0;
        drive.backLeftSwerveSpeed = 0;
        drive.backRightSwerveSpeed = 0;
    }

    /**
     * @param interrupte
     */
    @Override
    public void end(boolean interrupte) {
        System.out.println("fin");
    }

}
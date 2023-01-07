package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubSystems.Drive;
import frc.robot.SubSystems.MegaShooter2PointO;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.*;

public class intakeUp extends CommandBase {
    // shoot the ball

    private boolean isFinish = false;
    private double theta;
    private Drive drive;
    private TrapezoidProfile trapProfile;
    private MegaShooter2PointO mega;
    private double wait;
    private double intake;
    private double delay;

    Timer timer;

    public intakeUp() {
        mega = MegaShooter2PointO.get_Instance();
        timer = new Timer();
        delay = 0.5;
    }

    public intakeUp(double delay){
        mega = MegaShooter2PointO.get_Instance();
        timer = new Timer();
        this.delay = delay;
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
        return timer.get() > delay;
    }

    @Override
    public void execute() {
        mega.intakeUp();
    }

    /**
     * @param interrupte
     */
    @Override
    public void end(boolean interrupte) {
    }
}
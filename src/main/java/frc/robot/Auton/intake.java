package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubSystems.Drive;
import frc.robot.SubSystems.MegaShooter2PointO;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.*;

public class intake extends CommandBase {
    // shoot the ball

    private boolean isFinish = false;
    private double theta;
    private Drive drive;
    private TrapezoidProfile trapProfile;
    private MegaShooter2PointO mega;
    private double wait;
    private double intake;

    Timer timer;

    public intake(double wait, double intake) {
        mega = MegaShooter2PointO.get_Instance();
        timer = new Timer();
        this.wait = wait;
        this.intake = intake;
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
        return timer.get() > (wait + intake);
    }

    @Override
    public void execute() {
        if (timer.get() < wait) {
            // Doing Nothing Intentionally
        } else if (timer.get() > wait && timer.get() < intake + wait) {
            mega.indexing();
            mega.intakeOn();
        }
    }

    /**
     * @param interrupte
     */
    @Override
    public void end(boolean interrupte) {
        mega.intakeOff();
        timer.stop();
        timer.reset();

    }
}
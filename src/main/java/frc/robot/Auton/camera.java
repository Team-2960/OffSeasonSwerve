package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubSystems.Drive;
import frc.robot.SubSystems.Lime;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;

public class camera extends CommandBase {
    // Auton Camera Alignment

    private double theta;
    private Drive drive;

    Timer timer;
    Lime lime;

    public camera() {

        drive = Drive.get_Instance();
        lime = Lime.get_Instance();
        this.theta = theta + drive.navX.getYaw();
        while (this.theta > 360) {
            this.theta = this.theta - 360;
        }
        while (this.theta < 0) {
            this.theta = this.theta + 360;
        }
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
        return lime.getHorOffset() < Constants.angleTolerance && timer.get() > 3;
    }

    @Override
    public void execute() {
        drive.setVector(0, 0, -0.13 * lime.getHorOffset());
        drive.periodic();
    }

    /**
     * @param interrupte
     */
    @Override
    public void end(boolean interrupte) {
        drive.setVector(0, 0, 0);
        Drive.frontLeftSwerveSpeed = 0;
        Drive.frontRightSwerveSpeed = 0;
        Drive.backLeftSwerveSpeed = 0;
        Drive.backRightSwerveSpeed = 0;
        drive.periodic();
        System.out.println("cam fin");

    }
}
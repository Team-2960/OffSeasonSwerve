package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubSystems.Drive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;

public class moveInDir extends CommandBase {

    private boolean isFinish = false;
    private double distance;
    private double theta;
    private Drive drive;
    private double speed;
    public double x = 0;
    public boolean reversed = false;

    public moveInDir(double distance, double theta, double speed) {// Distannce in feet
        this.distance = distance * (2048 * 8.16) / (4 * Math.PI);
        this.theta = theta + 90;
        drive = Drive.get_Instance();
        this.speed = speed;
        if (speed > 0) {
            reversed = true;
        } else {
            reversed = false;
        }
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("x", x);
        x = drive.frontRight.getDriveEncoder();
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
        if (reversed) {
            return drive.frontRight.getDriveEncoder() < -distance + x;
        } else {
            return drive.frontRight.getDriveEncoder() > distance + x;
        }
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
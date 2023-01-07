package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubSystems.Drive;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.*;

// Turn to an angle at continuous power
public class conTurn extends CommandBase {
  // Turn to an angle at continuous power

  private double theta;
  private Drive drive;

  Timer timer;

  public conTurn(double theta) {

    drive = Drive.get_Instance();
    this.theta = theta;
    timer = new Timer();
  }

  @Override
  public void initialize() {
    timer.start();
    drive.periodic();
    theta += drive.gyroAngle;

    while (theta > 360) {
      theta = theta - 360;
    }

    while (theta < 0) {
      theta = theta + 360;
    }
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
    return Math.abs(theta - Math.abs(drive.gyroAngle)) < 15;
  }

  @Override
  public void execute() {
    if (Math.abs(theta - Math.abs(drive.gyroAngle)) > 15) {
      drive.setVector(0, 0, -1);
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
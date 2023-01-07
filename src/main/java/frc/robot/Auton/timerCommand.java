package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubSystems.Drive;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;

// Turn to an angle at continuous power
public class timerCommand extends CommandBase {
  // Turn to an angle at continuous power

  Timer timer;

  public timerCommand() {
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
    return false;
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("auto timer", timer.get());
  }

  /**
   * @param interrupte
   */
  @Override
  public void end(boolean interrupte) {
    timer.stop();

  }
}
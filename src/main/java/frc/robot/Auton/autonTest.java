package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.SubSystems.Lime;

public class autonTest extends SequentialCommandGroup {
    public autonTest() {
        Lime lime = new Lime();
        addCommands(
                new ParallelCommandGroup(
                        new intake(0, 0.8),
                        new SequentialCommandGroup(
                                new intakeUp(),
                                new intakeDown())));

    }
}
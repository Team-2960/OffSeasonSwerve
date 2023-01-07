package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.SubSystems.Lime;

public class twoBallAuto extends SequentialCommandGroup {
    public twoBallAuto() {
        Lime lime = new Lime();
        addCommands(
                new intakeDown(),
                new modAngle(180),
                new ParallelRaceGroup(new moveInDir(42, 180, -0.3),
                        new intake(0, 2)),
                new ParallelRaceGroup(new turnWithTime(2.5, true),
                        new intake(0, 4)),
                
                //new conTurn(-180),
                new camera(),
                new shoot(11500)
        );

    }
}

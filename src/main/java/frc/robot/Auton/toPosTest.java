package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.SubSystems.Lime;

public class toPosTest extends SequentialCommandGroup {
    public toPosTest() {
        Lime lime = new Lime();
        addCommands(
                new toPos(0.9, 0, -225),
                //new toPos(0, 0, 120),
                new toPos(0.1,1,125),
                new toPos(0.2,2.3,-90),
                new toPos(-1.9, 0.5,-45),
                //new toPos(-1.4,-1.2 , -45),
                new toPos(1.3, 3.7, 135),
                new toPos(-0.8, -3.7, -45)
                //new toPos(-0.3, 0,-45)
        );

    }
}
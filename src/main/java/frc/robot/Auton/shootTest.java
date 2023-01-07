package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class shootTest extends SequentialCommandGroup {
    public shootTest() {
        addCommands(
                new shoot(11500),
                new waitTilBall(),
                new shoot(11500));
    }

}

package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.SubSystems.Lime;

public class oneBallWithCamera extends SequentialCommandGroup {
        public oneBallWithCamera() {
                Lime lime = new Lime();
                addCommands(
                                new ParallelRaceGroup(
                                                new SequentialCommandGroup(
                                                                new setShooterRPM(11500),
                                                                new modAngle(180),
                                                                new ParallelRaceGroup(new moveInDir(42, 180, -0.3),
                                                                                new intake(10, 0)),
                                                                new ParallelRaceGroup(new turnTillTarget(),
                                                                                new intake(10, 0)),
                                                                new camera(),
                                                                new shoot(11500),
                                                                new waitTilBall(),
                                                                new shoot(11500)),
                                                new intake(13, 0)),
                                new runIndex(11500)

                );

        }
}

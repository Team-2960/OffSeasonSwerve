package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.SubSystems.Lime;

public class twoBallAutoCamera extends SequentialCommandGroup {
        public twoBallAutoCamera() {
                Lime lime = new Lime();
                addCommands(new ParallelRaceGroup(

                                new SequentialCommandGroup(
                                                new intakeDown(),
                                                new setShooterRPM(11500),
                                                new modAngle(180),
                                                new ParallelRaceGroup(new moveInDir(42, 180, -0.3),
                                                                new intake(0, 2)),
                                                new ParallelRaceGroup(new turnTillTarget(),
                                                                new intake(0, 4)),

                                                // new conTurn(-180),
                                                new camera(),
                                                new shoot(11500),
                                                new waitTilBall(),
                                                new shoot(11500)),
                                new intake(13, 0)), new runIndex(11500)

                );

        }
}

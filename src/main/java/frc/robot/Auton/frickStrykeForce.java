package frc.robot.Auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Util.autoPathPoint;

public class frickStrykeForce extends SequentialCommandGroup {
    public frickStrykeForce() {
        autoPathPoint[] positions1 = new autoPathPoint[] {
                new autoPathPoint(new Pose2d(0, 3, Rotation2d.fromDegrees(-225)), false, true), // 3.15
                new autoPathPoint(new Pose2d(1.141, 3.2, Rotation2d.fromDegrees(-225)), false, true)
        };

        autoPathPoint[] positions2 = new autoPathPoint[] {
                new autoPathPoint(new Pose2d(-1.141, -2.2, Rotation2d.fromDegrees(0)), false, true),
                new autoPathPoint(new Pose2d(-1.141, -2.2, Rotation2d.fromDegrees(0)), true, true)        };

        addCommands(
                new intakeDown(),
                new toArray(positions1),
                new intake(0, 3),
                new setShooterRPM(11500),
                new toArray(positions2),
                new setShooterRPM(11500),
                new shoot(11500),
                new setShooterRPM(11500),
                new waitTilBall(),
                new shoot(11500)
        );
    }

}

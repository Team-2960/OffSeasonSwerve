package frc.robot.Auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.SubSystems.Lime;
import frc.robot.Util.autoPathPoint;

public class toPosTest2 extends SequentialCommandGroup {
        public toPosTest2() {
                autoPathPoint[] positions1 = new autoPathPoint[] {
                                new autoPathPoint(new Pose2d(0.9, 0, Rotation2d.fromDegrees(-90)), true, false),
                };

                autoPathPoint[] positions2 = new autoPathPoint[] {
                                new autoPathPoint(new Pose2d(0, 0, Rotation2d.fromDegrees(-225)), false, false),
                                new autoPathPoint(new Pose2d(0, 1.5, Rotation2d.fromDegrees(-225)), false, true, 0.4,
                                                Constants.thetaToPosTolerance),
                                new autoPathPoint(new Pose2d(0, 2.4, Rotation2d.fromDegrees(-180)), false, true, 0.4,
                                                Constants.thetaToPosTolerance),
                                new autoPathPoint(new Pose2d(0.3, 3.9, Rotation2d.fromDegrees(-90)), false, true, 0.4,//0.1, 4.1
                                                Constants.thetaToPosTolerance),
                                new autoPathPoint(new Pose2d(-1.3, 3.7, Rotation2d.fromDegrees(-55)), true, true, 0.4,
                                                Constants.thetaToPosTolerance),
                                new autoPathPoint(new Pose2d(-1.4, 2.7, Rotation2d.fromDegrees(-55)), true, true),//3.7
                                
                };

                autoPathPoint[] positions3 = new autoPathPoint[] {
                                new autoPathPoint(new Pose2d(0.75, 3.85, Rotation2d.fromDegrees(-215)), false, true),//3.15
                };

                autoPathPoint[] positions4 = new autoPathPoint[] {
                                new autoPathPoint(new Pose2d(-0.9, -3.95, Rotation2d.fromDegrees(-35)), false, true),//-3.55
                                new autoPathPoint(new Pose2d(-0.9, -3.95, Rotation2d.fromDegrees(-35)), true, true),
                };
                System.out.println("Hello!");
                addCommands(
                                new ParallelRaceGroup(
                                                new SequentialCommandGroup(
                                                                new intakeDownInstant(),
                                                                new setShooterRPM(11500),
                                                                new toArray(positions1),
                                                                new shoot(11500),
                                                                new setShooterRPM(12050),
                                                                new toArray(positions2),
                                                                new setShooterRPM(12050),
                                                                new ParallelRaceGroup(
                                                                                new SequentialCommandGroup(
                                                                                                new setShooterRPM(
                                                                                                                12050),
                                                                                                new shoot(12050)),
                                                                                new intake(0, 10)),
                                                                new waitTilBall(),
                                                                new shoot(12050),
                                                                new setShooterRPM(11500),
                                                                new toArray(positions3),
                                                                new ParallelCommandGroup(
                                                                                new intake(0, 0.8),
                                                                                new SequentialCommandGroup(
                                                                                                new intakeUp(),
                                                                                                new intakeDown())),
                                                                new toArray(positions4),
                                                                new setShooterRPM(11500),
                                                                new shoot(11500),
                                                                new setShooterRPM(11500),
                                                                new waitTilBall(),
                                                                new shoot(11500)),
                                                new timerCommand())

                );

        }
}
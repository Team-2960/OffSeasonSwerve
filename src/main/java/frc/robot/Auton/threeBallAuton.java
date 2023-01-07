package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SubSystems.Lime;

public class threeBallAuton extends SequentialCommandGroup{
    public threeBallAuton(){
        Lime lime = new Lime();
        addCommands(
            /*new intakeDown(),
            new modAngle(180),
            new ParallelCommandGroup(new moveInDir(34,180,-0.3),
                                     new intake(0, 4)
            ),
            //new moveInDir(0.01,180, 0.3),
            new toAngle(180),
            new camera(),
            new shoot(2, 4000),*/
            new turnWithTime(1.1, true),
            new ParallelCommandGroup(new moveWithTime(3.5,180, -0.3),
                                     new intake(0, 3.5)
            ),
            new intakeDown(),
            new turnWithTime2(1.15, true),
            new camera(),
            new shoot(4000)

           // new moveInDir(10, 0, 0.3)
            //new toAngle(180),
            //new shoot(1, 3000),
            //new moveInDir(48,180,0.3)

            //new toAngle(90)
            //new DriveAngle(1, 90, 0.75),
            //new moveToCoord(1, 1)
        );

    }
}

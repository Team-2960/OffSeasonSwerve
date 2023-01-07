package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auton.*;

public class oneBallAuton extends SequentialCommandGroup{
    public oneBallAuton(){
        addCommands(
            new intakeDown(),
            new modAngle(180),
            new moveInDir(36,180,0.3),
           // new moveInDir(10, 0, 0.3)
            //new toAngle(180),
            new shoot(11500),
            new moveInDir(48,180,0.3)

            //new toAngle(90)
            //new DriveAngle(1, 90, 0.75),
            //new moveToCoord(1, 1)
        );



    }
}
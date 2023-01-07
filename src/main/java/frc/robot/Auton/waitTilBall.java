package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubSystems.Drive;
import frc.robot.SubSystems.Index;
import frc.robot.SubSystems.MegaShooter2PointO;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;


public class waitTilBall extends CommandBase{

    public  MegaShooter2PointO megashooter2pointo;
    private int numBalls = 0;
    private int totalBalls;
    private Timer timer;
    private double speed;
    private Index index;
    public waitTilBall(){
        megashooter2pointo = MegaShooter2PointO.get_Instance();
        index = Index.get_Instance();
        

    }

    @Override
    public void initialize() {
    }

    /**
     * Returns true if all the commands in this group have been started and have
     * finished.
     * <p>
     * <p>
     * Teams may override this method, although they should probably reference
     * super.isFinished() if they do.
     * </p>
     *
     * @return whether this {@link CommandGroup} is finished
     */
    @Override
    public boolean isFinished() {
        return index.getUpperPhotoeye();//(numBalls == totalBalls);
    }

    @Override
    public void execute() {
        megashooter2pointo.indexing();
    }

    
    /** 
     * @param interrupte
     */
    @Override
    public void end(boolean interrupte) {
    }
}
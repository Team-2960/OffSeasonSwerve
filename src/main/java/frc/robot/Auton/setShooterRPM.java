package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubSystems.MegaShooter2PointO;



public class setShooterRPM extends CommandBase{

    public  MegaShooter2PointO megashooter2pointo;

    private double rpm;
    public setShooterRPM(double rpm){
        this.rpm = rpm;
        megashooter2pointo = MegaShooter2PointO.get_Instance();

    }

    @Override
    public void initialize() {
        megashooter2pointo.setShooterRPM(rpm, rpm);
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
        return true;
    }

    @Override
    public void execute() {
    }

    
    /** 
     * @param interrupte
     */
    @Override
    public void end(boolean interrupte) {
    }
}
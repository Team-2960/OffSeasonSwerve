package frc.robot.SubSystems;

import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.*;

public class Lime {
    private static Lime lime;
    public Lime(){

    }
    public static Lime get_Instance(){
        if(lime == null){
            lime = new Lime();
        }
        return lime;
    }
    public double getHorOffset(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }
    public double calcDistance(){
        return (Constants.h2-Constants.h1)/Math.tan((Constants.a1 - NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0))*Math.PI/180)-31;
    }
    public boolean isSeeTar(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1;
    }
}

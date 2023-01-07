package frc.robot.SubSystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Index extends SubsystemBase {
    private static Index index;
    //Photoeye
    private DigitalInput lowerPhotoeye;
    private DigitalInput upperPhotoeye;

    //Photoeye
    private TalonFX iMotor;

    //In Transit Boolean
    private boolean ballInTransit = false;

    //Previous photoeye state
    private boolean prevPhotoeye = false;
    private boolean prevUpperPhotoeye = false;

    public static Index get_Instance(){
    
        if(index == null){
          index = new Index();
        } 
        return index;
      }

      Index(){
        lowerPhotoeye = new DigitalInput(Constants.photoeye1);
        upperPhotoeye = new DigitalInput(Constants.photoeye2);
        iMotor = new TalonFX(Constants.indexMotor);
        iMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
        iMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);
        
        iMotor.setNeutralMode(NeutralMode.Brake);
      }
      public boolean getUpperPhotoeye(){
        return !upperPhotoeye.get();
      }

      public boolean getLowerPhotoeye(){
        return !lowerPhotoeye.get();
      }

      public void setSpeed(double speed){
        iMotor.set(ControlMode.PercentOutput, speed);
      }

      public boolean isInTransit(){
        return ballInTransit;
      }

      public void inTransit(){
        if(!getLowerPhotoeye() && prevPhotoeye){
          ballInTransit = true;
        }
        if(ballInTransit && getUpperPhotoeye()){
          ballInTransit = false;
        }
        prevPhotoeye = getLowerPhotoeye();
      }

      public boolean fallingEdgeUpper(){
        boolean bool  = (!getUpperPhotoeye() && prevUpperPhotoeye);
        prevUpperPhotoeye = getUpperPhotoeye();
        return bool;
      }
}
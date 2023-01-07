package frc.robot.SubSystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.fasterxml.jackson.annotation.JsonCreator.Mode;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climb extends SubsystemBase {
    private static Climb climb;

    //Solenoids
    private DoubleSolenoid sClimbLArm;
    private DoubleSolenoid sClimbRArm;
    private DoubleSolenoid sClimbHook;

    //Motors
    public TalonFX mLeftClimb;
    public TalonFX mRightClimb;


    //Hall Effect Sensors
    private DigitalInput rHallEffect;
    private DigitalInput lHallEffect;

    //Limit Switch
    private DigitalInput limitSwitch;

    //Encoder
    public Encoder climbEncoder;

    //Pnuematic Pushers
    public Solenoid sPusher;



    public static Climb get_Instance(){
    
        if(climb == null){
          climb = new Climb();
        } 
        return climb;
      }

      Climb(){
        sClimbLArm = new DoubleSolenoid(20, PneumaticsModuleType.REVPH, Constants.climbLArmSolenoid1, Constants.climbLArmSolenoid2);
        sClimbRArm = new DoubleSolenoid(20, PneumaticsModuleType.REVPH, Constants.climbRArmSolenoid1, Constants.climbRArmSolenoid2);
        sClimbHook = new DoubleSolenoid(20, PneumaticsModuleType.REVPH, Constants.climbHookSolenoid1, Constants.climbHookSolenoid2);
        mLeftClimb = new TalonFX(Constants.mClimbL);
        mRightClimb = new TalonFX(Constants.mClimbR);
        mLeftClimb.setNeutralMode(NeutralMode.Brake);
        mRightClimb.setNeutralMode(NeutralMode.Brake);
        mLeftClimb.configForwardSoftLimitEnable(true);
        mRightClimb.configForwardSoftLimitEnable(true);
        mLeftClimb.configForwardSoftLimitThreshold(Constants.winchExtendLimit);
        mRightClimb.configForwardSoftLimitThreshold(Constants.winchExtendLimit);
        mRightClimb.setInverted(true);
        
        mRightClimb.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
        mRightClimb.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);
        mLeftClimb.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
        mLeftClimb.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        
        rHallEffect = new DigitalInput(Constants.rHallEffectSensor);
        lHallEffect = new DigitalInput(Constants.lHallEffectSensor);
        limitSwitch = new DigitalInput(Constants.limitSwitchPort);

        climbEncoder = new Encoder(Constants.climbEncoderPortA, Constants.climbEncoderPortB);

        sPusher = new Solenoid(20, PneumaticsModuleType.REVPH, Constants.sPusher);

      }

      public void setPositionArm(int state){//0 = down 1 = up
        if(state == 0){
          sClimbLArm.set(Value.kForward);
          sClimbRArm.set(Value.kForward);
        }
        else if(state == 1){
          sClimbLArm.set(Value.kReverse);
          sClimbRArm.set(Value.kReverse);
        }
      }

      public double getClimbEncoder(){
        return climbEncoder.get();
      }

      public void setPositionHook(int state){//0 = down 1 = up
        SmartDashboard.putNumber("state", state);
        if(state == 0){
          sClimbHook.set(Value.kForward);
        }
        else if(state == 1){
          sClimbHook.set(Value.kReverse);
        }
      }

      public void setWinchSpeed(double left, double right){
        if((limitSwitch.get() && (left < 0 || right < 0))){
          mLeftClimb.set(ControlMode.PercentOutput, 0);
          mRightClimb.set(ControlMode.PercentOutput, 0);
        }
        mLeftClimb.set(ControlMode.PercentOutput, left);
        mRightClimb.set(ControlMode.PercentOutput, right);
        if(right > 0 && left > 0){
          sPusher.set(false);
        }else if(right < 0 && left < 0){
          sPusher.set(true);
        }
      }

      public double getWinchPos(){
        return Math.abs(climbEncoder.get());
      }

      public boolean isAttached(){
        return (rHallEffect.get() && lHallEffect.get());
      }

      public boolean isArmsExtended(){
        return !(sClimbLArm.get() == Value.kForward && sClimbRArm.get() == Value.kForward);
      }

      public boolean getLimitSwitch(){
        return !limitSwitch.get();
      }

      public void resetWinchPos(){
        if(getLimitSwitch() && getWinchPos() < Constants.winchContractPos){
          climbEncoder.reset(); 
        }
      }
}

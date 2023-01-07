package frc.robot.SubSystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.math.controller.PIDController;

import javax.naming.ldap.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.BangBangController;

public class Hood extends SubsystemBase {
  private static final String Controlmode = null;
  private static Hood hood;

  private SimpleMotorFeedforward upperWheelFF;
  private double upperWheelVel;
  private TalonFX mUpperWheel;
  private PIDController upperWheelPID;

  private SimpleMotorFeedforward lowerWheelFF;
  private double lowerWheelVel;
  private TalonFX mLowerWheel;
  private PIDController lowerWheelPID;

  // bang bang controllers
  private BangBangController bangL;
  private BangBangController bangU;

  private double shooterToleranceLower = 100;
  private double shooterToleranceUpper = 250;

  private int shootFrames = 0;
  private double prevRPM = 0;

  private Timer timerMan;
  public static Hood get_Instance() {
    if (hood == null) {
      hood = new Hood();
    }
    return hood;
  }

  Hood() {
    // Motors
    mUpperWheel = new TalonFX(Constants.mUpperShooter);
    mLowerWheel = new TalonFX(Constants.mLowerShooter);
    mLowerWheel.setInverted(true);

    // BANG BANG
    bangU = new BangBangController();
    bangL = new BangBangController();
    // PID STUFF
    upperWheelPID = new PIDController(Constants.uWPP, Constants.uWII, Constants.uWDD);
    lowerWheelPID = new PIDController(Constants.lWPP, Constants.lWII, Constants.lWDD);
    // FF
    lowerWheelFF = new SimpleMotorFeedforward(Constants.lWP, Constants.lWI, Constants.lWD);
    upperWheelFF = new SimpleMotorFeedforward(Constants.uWP, Constants.uWI, Constants.uWD);

    timerMan = new Timer();
    timerMan.start();
  }

  public void setSpeed(double powerL, double powerU) {
    mUpperWheel.set(ControlMode.PercentOutput, powerU);
    mLowerWheel.set(ControlMode.PercentOutput, powerL);
  }

  public void hoodPID() {
    setSpeed(lowerWheelFF.calculate(mLowerWheel.getSelectedSensorVelocity(), lowerWheelVel),
        upperWheelFF.calculate(mUpperWheel.getSelectedSensorVelocity(), upperWheelVel));
  }

  public void setWheelSpeedVel(double upper, double lower) {// IN SENSOR UNITS/ 100 MS
    upperWheelVel = upper;
    lowerWheelVel = lower;
  }

  public void calcWheelSpeed() {
    double upPower = upperWheelFF.calculate(upperWheelVel)
        + upperWheelPID.calculate(mUpperWheel.getSelectedSensorVelocity(), upperWheelVel);
    double lowPower = lowerWheelFF.calculate(lowerWheelVel)
        + lowerWheelPID.calculate(mLowerWheel.getSelectedSensorVelocity(), lowerWheelVel);
    SmartDashboard.putNumber("loPwr", lowPower);
    SmartDashboard.putNumber("upPwr", upPower);
    SmartDashboard.putNumber("up PID", upperWheelPID.calculate(mUpperWheel.getSelectedSensorVelocity(), upperWheelVel));
    SmartDashboard.putNumber("low PID",
        lowerWheelPID.calculate(mLowerWheel.getSelectedSensorVelocity(), lowerWheelVel));
    if (!(upperWheelVel == 0 || lowerWheelVel == 0)) {
      if (upperWheelVel < 9500 && lowerWheelVel < 9500) {
        setSpeed(upperWheelFF.calculate(upperWheelVel), upperWheelFF.calculate(upperWheelVel));
      } else {
        setSpeed(lowPower, upPower);
      }
    } else {
      hood.setSpeed(0, 0);
    }
  }

  public boolean isWheelAtVel() {
    if (Math.abs(lowerWheelVel) > 9500 && Math.abs(lowerWheelVel) > 9500) {
      double low_error = Math.abs(lowerWheelVel - Math.abs(mLowerWheel.getSelectedSensorVelocity()));
      double high_error = Math.abs(upperWheelVel - Math.abs(mUpperWheel.getSelectedSensorVelocity()));

      boolean lower_limit = low_error < shooterToleranceLower;
      boolean upper_limit = high_error < shooterToleranceUpper;

      SmartDashboard.putBoolean("lower limit", lower_limit);
      SmartDashboard.putBoolean("upper limit", upper_limit);

      SmartDashboard.putNumber("lower error", low_error);
      SmartDashboard.putNumber("upper error", high_error);

      return lower_limit && upper_limit;
    } else {
      double low_error = Math.abs(lowerWheelVel - Math.abs(mLowerWheel.getSelectedSensorVelocity()));
      double high_error = Math.abs(upperWheelVel - Math.abs(mUpperWheel.getSelectedSensorVelocity()));

      boolean lower_limit = low_error < 1000;
      boolean upper_limit = high_error < 1000;

      SmartDashboard.putBoolean("lower limit", lower_limit);
      SmartDashboard.putBoolean("upper limit", upper_limit);

      SmartDashboard.putNumber("lower error", low_error);
      SmartDashboard.putNumber("upper error", high_error);

      return lower_limit && upper_limit;
    }
  }

  public void printRPM() {
    SmartDashboard.putNumber("up vel", mUpperWheel.getSelectedSensorVelocity());
    SmartDashboard.putNumber("low vel", mLowerWheel.getSelectedSensorVelocity());
  }

  public double RPMCalc(double distance) {
    return (41.666666666) * distance + 7583.33;
  }

  public boolean isShootReady() {
    if (isWheelAtVel()) {
      shootFrames = 0;
    } else {
      shootFrames++;
    }
    return shootFrames < 9 && timerMan.get() > 0.25;
  }

  public double setWheelWithByDistance(double distance) {
    double x = distance;
    double rpm = 0.05996 * Math.pow(x, 3) + -17.62 * Math.pow(x, 2) + 1705.97 * x - 43888.9;
    return rpm;
  }

  public void shootTimeManager(){
    if(upperWheelVel != prevRPM){
      timerMan.reset();
    }

    prevRPM = upperWheelVel;

  }

}
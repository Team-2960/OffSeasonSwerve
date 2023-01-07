/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.Auton.*;
import frc.robot.Constants;
import frc.robot.SubSystems.*;
import frc.robot.Util.LEDs;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Compressor;



  //private Camera camera = Camera.get_Instance();


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public OI oi;
  private Command autonCommand = null;
  private PowerDistribution pdp;
  private Compressor comp;
  private LEDs leds;
  private Drive drive;
  private MegaShooter2PointO mega;
  private UsbCamera UsbCamera;


  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    UsbCamera = CameraServer.getInstance().startAutomaticCapture(0);

    PortForwarder.add(5801, "limelight.local", 5801);
    oi = new OI();
    UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
    MjpegServer mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181);
    mjpegServer1.setSource(usbCamera);

    // Creates the CvSink and connects it to the UsbCamera
    CvSink cvSink = new CvSink("opencv_USB Camera 0");
    cvSink.setSource(usbCamera);

    // Creates the CvSource and MjpegServer [2] and connects them
    CvSource outputStream = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 30);
    MjpegServer mjpegServer2 = new MjpegServer("serve_Blur", 1182);
    mjpegServer2.setSource(outputStream);
    pdp = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
    pdp.setSwitchableChannel(true);
    comp = new Compressor(20, PneumaticsModuleType.REVPH);
    leds = new LEDs();
    drive = Drive.get_Instance();
    mega = MegaShooter2PointO.get_Instance();

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  public void autonomousInit() {
    drive.autonInit();
    //drive.manualSpeeds(0, 1.5, 0);
    drive.breakMode();
    autonCommand = new twoBallAutoCamera();
    if(autonCommand != null) autonCommand.schedule();
  }

  @Override
  public void autonomousPeriodic() {
    drive.periodicTele();
    //drive.autonUpdate();
  }

  @Override
  public void teleopInit() {
    drive.coastMode();
  }

  @Override
  public void teleopPeriodic() {
    oi.oiRun();
    SmartDashboard.putNumber("winch", mega.climb.getClimbEncoder());

  }

  @Override
  public void disabledInit() {
    mega.disableClimb();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}

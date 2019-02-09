/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;

import java.io.IOException;

import com.kauailabs.navx.frc.AHRS;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.Compressor;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static DriveSubsystem DriveSub = new DriveSubsystem();
  /**
   *
   */

  private static final DriveSubsystem driveSub = DriveSub;
  public static OI m_oi;
  Solenoid PistonTestOne = new Solenoid(RobotMap.TOP_SOLENOID_ID);
  Solenoid PistonTestTwo = new Solenoid(RobotMap.BOTTOM_SOLENOID_ID);
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  public static AHRS navXGyro;
 
 // PowerDistributionPanel theOnlyPDP = new PowerDistributionPanel();
 // The the above at some point please. It keeps throwing an error

  // Stuff we don't need TODO
  public static Compressor Comp0 = new Compressor(0);


 // implement the above at some point please. It keeps throwing an error
 // TODO
  
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

  @Override
  public void robotInit() {
    ///
    

    UsbCamera camera0 = CameraServer.getInstance().startAutomaticCapture();
    UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture();

    MjpegServer mServer0 = CameraServer.getInstance().addServer("Another_Server0");
    MjpegServer mServer1 = CameraServer.getInstance().addServer("Another_Server_1");

   mServer0.setSource(camera0);
   mServer1.setSource(camera1);

   

// NT: server: client CONNECTED: 10.15.57.180 port 61914


    // TODO TEST!!
    //
    /*
=======
  
  UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
  MjpegServer mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181);
  mjpegServer1.setSource(usbCamera); CvSink cvSink = new CvSink("opencv_USB Camera 0");
  cvSink.setSource(usbCamera);
  CvSource outputStream = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 30);
  MjpegServer mjpegServer2 = new MjpegServer("serve_Blur", 1182);
  mjpegServer2.setSource(outputStream);



/* Test Test Test TODO
>>>>>>> 087ab63f0f1a94b07ff512af8a060d8ef8c0c7a4
    UsbCamera visionTapeCamera = new UsbCamera("VisionTapeCamera", 0);
    MjpegServer visionTapeMJpeg = new MjpegServer("THE_VISION_TAPE 1182", 1182);
    CvSink VisionTapeCvSink = new CvSink("Vision-Tape-Camera-Cv-Sink");

   // VisionTapeCvSink.setSource(visionTapeCamera);
    visionTapeMJpeg.setSource(visionTapeCamera);
   // CvSource outputStreamVisionTape = new CvSource("Vision_Tape_Output_Stream_Thing", PixelFormat.kMJPEG, 640, 480, 30);

     

=======
   Test Test Test TODO
   
    
    MjpegServer theSecondMJepServer = new MjpegServer("Serve_Vision_Tape_Output_Stream_Thing", 1182);
    theSecondMJepServer.setSource(outputStreamVisionTape);
   
    CameraServer.getInstance().addCamera(visionTapeCamera);
    CameraServer.getInstance().startAutomaticCapture();
    CameraServer.getInstance().getVideo(visionTapeCamera);
    CameraServer.getInstance().putVideo("Vision-Tape", 480, 640);
    SmartDashboard.putNumber("port Number", visionTapeMJpeg.getPort());
    SmartDashboard.putNumber("Handle Number", visionTapeMJpeg.getHandle());
    SmartDashboard.putString("get Listen Address", visionTapeMJpeg.getListenAddress());
    SmartDashboard.putString("get Listen Description", visionTapeMJpeg.getDescription());
*/



/*
    /// Camera 1

    UsbCamera usbCamera0 = CameraServer.getInstance().startAutomaticCapture();
    usbCamera0.setResolution(320, 240);
    usbCamera0.setFPS(10);
    CvSink cvSink0 = CameraServer.getInstance().getVideo();

    /// Camera 1

    /// Camera 2

    UsbCamera usbCamera1 = CameraServer.getInstance().startAutomaticCapture(1);
    usbCamera1.setResolution(320, 240);
    usbCamera1.setFPS(10);
    CvSink cvSink1 = CameraServer.getInstance().getVideo();

    // Camera 2

    ////////////
   MjpegServer visionTapeSense = new MjpegServer("THE_VISION_TAPE", 1181);
   visionTapeSense.setSource(usbCamera0);

   
    Mat visionTarget = new Mat();

    cvSink0.grabFrame(visionTarget);

    cvSink.Jpeg
    ////////////

  */

  
// Testing Angles


//


    /// Camera 2


    SmartDashboard.putData("Auto mode", m_chooser);
    m_chooser.setDefaultOption("Default Auto", new ExampleCommand());

    try {
      navXGyro = new AHRS(SPI.Port.kMXP);

    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating NAV-X Gyro (MXP)", true);
    }
   
    
    // chooser.addOption("My Auto", new MyAutoCommand());
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */

  @Override
  public void robotPeriodic() {
              SmartDashboard.putNumber("Gyro angle", Robot.navXGyro.getAngle());
              SmartDashboard.putNumber("POV", OI.zeroSlotController.getPOV());

    
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
    /* remember - no breaks! */
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {

    ////// Jamie G testing
    DriveSub.UpdateDriveTurn_to_angle(90, OI.zeroSlotController.getX(Hand.kLeft), OI.zeroSlotController.getY(Hand.kLeft));
    /////
   
   ////// The Jamie G test site
    DriveSub.UpdateDriveTurn_to_angle(90, OI.zeroSlotController.getX(Hand.kLeft), OI.zeroSlotController.getY(Hand.kLeft));

   //////
    try {
      Client client = new Client("127.0.0.1", 5000);
    } catch (IOException e) {
      // TODO Auto-generated catch block
		e.printStackTrace();
	}
    SmartDashboard.putNumber("I Am THE GYRO", navXGyro.getAngle());
    SmartDashboard.putNumber("Gyro Yaw", navXGyro.getYaw());
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
     * switch(autoSelected) { case "My Auto": autonomousCommand = new
     * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
     * ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  
  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    new DefaultDriveCommand().start();

    SmartDashboard.putData(DriveSub.frontRight);
    SmartDashboard.putData(DriveSub.rearLeft);
    SmartDashboard.putData(DriveSub.frontLeft);
    SmartDashboard.putData(DriveSub.rearRight);


    SmartDashboard.putData("Mecamum Drive", driveSub.mecDrive);
    SmartDashboard.putData("Turn Controller ", driveSub.turnController);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    Scheduler.getInstance().run();
    
    m_autonomousCommand = m_chooser.getSelected();



   

  }
}


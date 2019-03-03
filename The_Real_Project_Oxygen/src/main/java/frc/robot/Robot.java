/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoSource;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;

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
import frc.robot.commands.CameraServerStartInstantCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.FCDDriveCommand;
import frc.robot.commands.NonFCDDriveCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.frontLifterCommand;
import frc.robot.commands.getBottomCamCommand;
import frc.robot.commands.getTopCamCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FrontLiftSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.RearLiftSubsystem;
import frc.robot.subsystems.TopRailSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.OI;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static DriveSubsystem driveSub;
  public static VisionSubsystem visionSub;
  public static FrontLiftSubsystem frontLifterSub;
  public static RearLiftSubsystem rearLiftSub;
  public static TopRailSubsystem topLiftSub;

  /**
   *
   */

  public static OI m_oi;
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  public static AHRS navXGyro;

  // PowerDistributionPanel theOnlyPDP = new PowerDistributionPanel();
  // The the above at some point please. It keeps throwing an error

  // implement the above at some point please. It keeps throwing an error
  // TODO

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */


  public static VisionMath vMath;

  @Override
  public void robotInit() {
    try {
      navXGyro = new AHRS(SPI.Port.kMXP);

    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating NAV-X Gyro (MXP)", true);
    }
    
    m_oi = new OI();
    visionSub = new VisionSubsystem();
    frontLifterSub = new FrontLiftSubsystem();
    rearLiftSub = new RearLiftSubsystem();
    topLiftSub = new TopRailSubsystem();
    driveSub = new DriveSubsystem();

    
    // VERY IMPORTANT
    navXGyro.reset();
    // VERY IMPORTANT

  /*
    

    SmartDashboard.putData("Auto mode", m_chooser);
    // m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
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
 }

  @Override
  public void robotPeriodic() {
    // SmartDashboard Data
    SmartDashboard.putNumber("Gyro angle", Robot.navXGyro.getAngle());
    SmartDashboard.putNumber("POV", OI.zeroSlotController.getPOV());
    SmartDashboard.putNumber("Controller X", OI.zeroSlotController.getX(Hand.kLeft));
    SmartDashboard.putNumber("Controller Y", OI.zeroSlotController.getY(Hand.kLeft));
    SmartDashboard.putNumber("Controller Z", OI.zeroSlotController.getX(Hand.kRight));
    SmartDashboard.putNumber("NewZero", driveSub.newZero);
    SmartDashboard.putNumber("Rotation Speed", driveSub.rotationSpeed);
    SmartDashboard.putNumber("Angle Off", driveSub.angleOff);
    SmartDashboard.putNumber("PID-Average Error ", driveSub.turnController.getAvgError());
    SmartDashboard.putNumber("PID-Setpoint ", driveSub.turnController.getSetpoint());
    SmartDashboard.putNumber("PID-Delta (Change in) Setpoint  ", driveSub.turnController.getDeltaSetpoint());
    SmartDashboard.putNumber("PID-  P", driveSub.turnController.getP());
    SmartDashboard.putNumber("PID-  I", driveSub.turnController.getI());
    SmartDashboard.putNumber("PID-  D", driveSub.turnController.getD());
    SmartDashboard.putNumber("PID-  F", driveSub.turnController.getF());
    SmartDashboard.putData(driveSub.frontRight);
    SmartDashboard.putData(driveSub.rearLeft);
    SmartDashboard.putData(driveSub.frontLeft);
    SmartDashboard.putData(driveSub.rearRight);
    SmartDashboard.putData(driveSub.turnController);
    SmartDashboard.putData("Mecanum Drive", driveSub.mecDrive);
    SmartDashboard.putData("Turn Controller", driveSub.turnController);
    SmartDashboard.putNumber("PID ERROR",driveSub.turnController.getError());
    SmartDashboard.putBoolean("is the drive turn controller on target", driveSub.turnController.onTarget());
    // SmartDashboard Data

   visionSub.updateVisionObject();

   if(OI.allButtonComboPressesd(OI.zeroSlotController)) {
     navXGyro.reset();
   }
  
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

  private static int testCount = 0;

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
   new NonFCDDriveCommand().start();

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

    SmartDashboard.putBoolean("Is Collision Detected:",driveSub.collisionDetected);
  
    new FCDDriveCommand().start();

    
   

    if(OI.zeroSlotController.getXButtonPressed()) {
      rearLiftSub.liftMethod();
    }

    if(OI.zeroSlotController.getAButtonPressed()) {
      topLiftSub.liftMethod();

    }

    if(OI.zeroSlotController.getBButtonPressed()) {
      frontLifterSub.liftMethod();
    }

    if (OI.zeroSlotController.getAButtonPressed()) {

    }

  Timer.delay(0.02);

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    
  }

}

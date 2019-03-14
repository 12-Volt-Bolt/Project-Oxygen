/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.FCDDriveCommand;
import frc.robot.commands.CMDButtonCommand;
import frc.robot.commands.DriveFCDStrafeCommand;
import frc.robot.commands.DriveMecanumPIDCommand;
import frc.robot.commands.DriveNonFCDDriveCommand;
import frc.robot.commands.DriveWithVisionCommand;
import frc.robot.commands.CameraServerStartInstantCommand;
import frc.robot.commands.FCDDriveCommand;
import frc.robot.commands.HatchDefaultPositionCommand;
import frc.robot.commands.HatchObtainPositionCommand;
import frc.robot.commands.HatchPlacementHeightCommand;
import frc.robot.commands.lifter_commands.ManualLifterCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.lifter_commands.frontLifterCommand;
import frc.robot.subsystems.DiskUnitSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FrontLiftSubsystem;
import frc.robot.subsystems.LifterSubsystem;
import frc.robot.subsystems.RearLiftSubsystem;
import frc.robot.subsystems.TopRailSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.GenericLiftSubsystem.LiftID;
import frc.robot.statics_and_classes.Constants_And_Equations.AxisNames;
import frc.robot.OI;
import frc.robot.statics_and_classes.VisionMath;
import frc.robot.statics_and_classes.Constants_And_Equations;
import frc.robot.statics_and_classes.ControllerFunctions;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 * 
 * @param <topLiftSub>
 */
public class Robot<topLiftSub> extends TimedRobot {
  public static DriveSubsystem driveSub;
  public static VisionSubsystem visionSub;
  public static FrontLiftSubsystem frontLiftSub;
  public static RearLiftSubsystem rearLiftSub;
  public static TopRailSubsystem topLiftSub;
  public static LifterSubsystem liftSub;
  public static DiskUnitSubsystem DiskSub;

  /**
   *
   */

  public static OI m_oi;
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  public static AHRS navXGyro;

  // PowerDistributionPanel theOnlyPDP = new PowerDistributionPanel();
  // The the above at some point please. It keeps throwing an error


  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

 
  public static int liftSafteyMode;
  public static boolean isProcessCmdBool;


  public static VisionMath vMath;

  @Override
  public void robotInit() {
    try {
      navXGyro = new AHRS(SPI.Port.kMXP);

    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating NAV-X Gyro (MXP)", true);
    }

    driveSub = new DriveSubsystem();
    m_oi = new OI();
    visionSub = new VisionSubsystem();
   // frontLiftSub = new FrontLiftSubsystem();
   // rearLiftSub = new RearLiftSubsystem();
   // topLiftSub = new TopRailSubsystem();
   // DiskSub = new DiskUnitSubsystem();

   

    // VERY IMPORTANT
    navXGyro.reset();
    // VERY IMPORTANT


    /*
     * 
     * 
     * SmartDashboard.putData("Auto mode", m_chooser); //
     * m_chooser.setDefaultOption("Default Auto", new ExampleCommand()); //
     * chooser.addOption("My Auto", new MyAutoCommand());
     * 
     * }
     * 
     * /** This function is called every robot packet, no matter the mode. Use this
     * for items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p> This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
  }

  @Override
  public void robotPeriodic() {
    /*
    SmartDashboard.putNumber("Gyro angle", Robot.navXGyro.getAngle());
    SmartDashboard.putNumber("POV", OI.zeroSlotController.getPOV());
    
    SmartDashboard.putNumber("Controller X", OI.zeroSlotController.getX(Hand.kLeft));
    SmartDashboard.putNumber("Controller Y", OI.zeroSlotController.getY(Hand.kLeft));
    SmartDashboard.putNumber("Controller Z", OI.zeroSlotController.getX(Hand.kRight));
 
    // SmartDashboard.putNumber("NewZero", driveSub.newZero);
    // SmartDashboard.putNumber("Rotation Speed", driveSub.rotationSpeed);
    // SmartDashboard.putNumber("Angle Off", driveSub.angleOff);
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
    SmartDashboard.putNumber("PID ERROR", driveSub.turnController.getError());
    SmartDashboard.putBoolean("is the drive turn controller on target", driveSub.turnController.onTarget());
    // SmartDashboard Data
    */
    visionSub.updateVisionVariables();
   
    if (OI.allButtonComboPressesd(OI.zeroSlotController)) {
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

    if(OI.visionStartCombo()) {
      new CMDButtonCommand().start();
    //new DriveWithVisionCommand().start();
    }
    else {
   // new DriveFCDStrafeCommand().start();
    new DriveNonFCDDriveCommand().start();
    }
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

    ///navXGyro.reset();

    driveSub.turnController.disable();

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();  

    SmartDashboard.putNumber("Distance From Target (ROBO TELE)", visionSub.distanceFromCamToTargetInCM() /* * (0.6)*/);
    SmartDashboard.putNumber("lateral Offset To Target (ROBO TELE 1)", visionSub.lateralOffsetToTargetInCM());
    SmartDashboard.putNumber("Motor Adjust for Vision y axis", -Robot.visionSub.runForwardController(Robot.visionSub.distanceFromCamToTargetInCM()) );
    SmartDashboard.putNumber("Lateral Motor speed", visionSub.runStrafeController(visionSub.lateralOffsetToTargetInCM()));
    new DriveMecanumPIDCommand().start();
    /*
    if(OI.visionStartCombo()) {
     new CMDButtonCommand().start();
    Robot.driveSub.updateDriveCartesian(visionSub.runStrafeController(visionSub.lateralOffsetToTargetInCM()),
          -Robot.visionSub.runForwardController(Robot.visionSub.distanceFromCamToTargetInCM()), 0);

    }
    */

    liftSafteyMode = LifterSubsystem.checkLiftSaftey();

    switch (liftSafteyMode) {
    case 1:
      new ManualLifterCommand().start();
      break;

    default:
      break; // do nothing, saftey on break;
    }

  
    SmartDashboard.putNumber("measCenterXPixels", visionSub.measCenterXPixels);
    SmartDashboard.putNumber("Meas Separation", visionSub.measSeparationPixels);
    SmartDashboard.putNumber("Meas AL Angle",visionSub.measAlAngleDegrees);
    SmartDashboard.putNumber("Meas AL Angle Center Pixels",visionSub.measAlCenterXPixels);
  }

  private double testValue;

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
   
   
    if (OI.zeroSlotController.getBumper(Hand.kRight) == true) {
      testValue = Constants_And_Equations.Clamp(-1, 1, testValue + 0.1);
    }

    else if (OI.zeroSlotController.getBumper(Hand.kLeft) == true) {
      testValue = Constants_And_Equations.Clamp(-1, 1, testValue - 0.1);
    }

    driveSub.frontLeft.set(testValue);

    SmartDashboard.putData(new CameraServerStartInstantCommand());
    SmartDashboard.putData(new CMDButtonCommand());
    SmartDashboard.putData(new FCDDriveCommand());
    SmartDashboard.putData(new frontLifterCommand());
    SmartDashboard.putData(new HatchDefaultPositionCommand());
    SmartDashboard.putData(new HatchObtainPositionCommand());
    SmartDashboard.putData(new HatchPlacementHeightCommand());
    SmartDashboard.putData(new ManualLifterCommand());
  }

}

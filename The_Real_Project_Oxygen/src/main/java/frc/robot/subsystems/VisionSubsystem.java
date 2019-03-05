/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import javax.swing.GroupLayout.Alignment;
import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants_And_Equations;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.VisionMath;
import edu.wpi.cscore.VideoSource;

/**
 * This class holds all methods and objects related to the cameras. We use both
 * the Logitech 930e (Vision Tape) and the Microsoft LifeCam 3000 (Alignment
 * Tape)
 */

public class VisionSubsystem extends Subsystem {

  // USB Camera Objects
  public static UsbCamera topCam, bottomCam;

  // Camera Servers
  public static CameraServer camServerTop, camServerBottom;

  // VideoSink Server
  public static VideoSink theOnlyCamServer;

  // M-JPEG Servers
  public static MjpegServer mjpegServerTop, mjpegServerBottom;

  // Camera names
  public static final String TOP_CAM_NAME = "Top Camera";
  public static final String BOTTOM_CAM_NAME = "Bottom Camera";

  // Camera frame rate
  public static final int TOP_CAM_FPS = 5;
  public static final int BOTTOM_CAM_FPS = 5;

  // Total number of pixel columns
  public static final int TOP_CAM_COL_PIXEL_NUM = 480;
  public static final int BOTTOM_CAM_COL_PIXEL_NUM = 240;

  // Total number of pixel rows
  public static final int TOP_CAM_ROW_PIXEL_NUM = 640;
  public static final int BOTTOM_CAM_ROW_PIXEL_NUM = 320;

  // Smartdashboard variables
  public static int measCenterXPixels;
  public static int measSeparationPixels;
  public static int measAlAngleDegrees;
  public static int measAlCenterXPixels;
  public static boolean isProcessCmdBool;

  // Smartdashboard Network Table Strings
  public static final String measCenterXString = "DB/Slider 0";
  public static final String measSeparationString = "DB/Slider 1";
  public static final String measAlCenterXString = "DB/Slider 2";
  public static final String measAlAngleDegreesString = "DB/Slider 3";
  public static final String isProcessCMDString = "DB/Button 0";

  // Data obtained from LabVIEW vision program
  public static double targetCenterXInPixels;
  public static double vtSeparationInPixels;
  public static double alCenterXinPixels;
  public static double alAngleInDegrees;

  // Calibration Data:
  public static final int caliVTDistanceCm = 1;
  public static final int caliVTCenterInPixels = 1;
  public static final int caliVTSeparationInPixels = 1;
  public static final int caliAlCenterInPixels = 1;
  public static final int caliAlAngleInDegrees = 1;

  // PID Controllers
  public PIDController rotationController;
  public PIDController strafeController;
  public PIDController verticalController;

  // PID Rotation Constants
  private final float RotationP = 0.0f;
  private final float RotationI = 0.0f;
  private final float RotationD = 0.0f;
  private final float RotationF = 0.0f;
  // The minimum value required to rotation
  private final float MIN_ROTATION_VALUE = 0.1f;

  // PID Strafing Constants
  private final float StrafeP = 0.0f;
  private final float StrafeI = 0.0f;
  private final float StrafeD = 0.0f;
  private final float StrafeF = 0.0f;
  // The minimum value required to strafe
  private final float MIN_STRAFE_VALUE = 0.1f;

  // PID Vertical Movement Constants
  private final float verticalP = 0.0f;
  private final float verticalI = 0.0f;
  private final float verticalD = 0.0f;
  private final float verticalF = 0.0f;
  // The minimum value required to vertical
  private final float VERTICAL_VALUE = 0.1f;

  // Constants
  private static final int NO_DATA = -888;

  // Timer
  private double cmdTimer;

  public VisionSubsystem() {
    topCam = CameraServer.getInstance().startAutomaticCapture(TOP_CAM_NAME, RobotMap.CAMERA_ZERO_ID);
    topCam.setResolution(TOP_CAM_ROW_PIXEL_NUM, TOP_CAM_COL_PIXEL_NUM);
    topCam.setFPS(TOP_CAM_FPS);

    bottomCam = CameraServer.getInstance().startAutomaticCapture(BOTTOM_CAM_NAME, RobotMap.CAMERA_ONE_ID);
    bottomCam.setResolution(BOTTOM_CAM_ROW_PIXEL_NUM, BOTTOM_CAM_COL_PIXEL_NUM);
    bottomCam.setFPS(BOTTOM_CAM_FPS);

    bottomCam.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
    topCam.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

    targetCenterXInPixels = NO_DATA;
    vtSeparationInPixels = NO_DATA;
    alCenterXinPixels = NO_DATA;
    alAngleInDegrees = NO_DATA;

    // rotationController = new PIDController(Kp, Ki, Kd, source, output)
    // rotationController = new PIDController(Kp, Ki, Kd, source, output);
    // strafeController = new PIDController(Kp, Ki, Kd, source, output);
    // verticalController = new PIDController(Kp, Ki, Kd, source, output);

  }

  @Override
  public void initDefaultCommand() {

  }

  public int distanceFromCamToTargetInCM() {
    return ((caliVTSeparationInPixels) / (measSeparationPixels)) * (caliVTDistanceCm);
  }

  public int lateralOffsetToTargetInCM() {
    return (int) ((int) (caliVTCenterInPixels / measSeparationPixels) * (20.32 / vtSeparationInPixels));
  }

  // If positive, strafe right
  // If negative, strafe left
  public int aLineCamOffset() {
    return (int) (((caliAlCenterInPixels) - measCenterXPixels) * (20.32 / vtSeparationInPixels));
  }

  // If positive, rotate Left
  // If negative, rotate Right
  public int aLineAngleOffset() {
    return (measAlAngleDegrees - caliAlAngleInDegrees);
  }

  public void updateVisionVariables() {
    if (OI.visionStartCombo()) {
      SmartDashboard.putBoolean(isProcessCMDString, true);
    } else {
      SmartDashboard.putBoolean(isProcessCMDString, false);

    }

    measCenterXPixels = (int) SmartDashboard.getNumber(measCenterXString, NO_DATA);
    measSeparationPixels = (int) SmartDashboard.getNumber(measSeparationString, NO_DATA);
    measAlAngleDegrees = (int) SmartDashboard.getNumber(measAlAngleDegreesString, NO_DATA);
    measAlCenterXPixels = (int) SmartDashboard.getNumber(measAlCenterXString, NO_DATA);
  }

  public void runRotationController() {
    float headingError = (float) -aLineAngleOffset();
    float rotationAdjust = 0.0f;

    // rotate Right
    if (aLineAngleOffset() > 0) {
      rotationAdjust = RotationP * headingError - MIN_ROTATION_VALUE;
    }

    // rotate Left
    else if (aLineCamOffset() < 0) {
      rotationAdjust = RotationP * headingError + (MIN_ROTATION_VALUE);
    }

    // Add CW rotation method here
    // Add CCW rotation method here

  }

  public void runStrafeController() {
    float strafeError = (float) -lateralOffsetToTargetInCM();
    float strafeAdjust = 0.0f;

    // strafe left
    if (0 > lateralOffsetToTargetInCM()) {
      strafeAdjust = StrafeP * strafeError - MIN_STRAFE_VALUE;
    }
    // strafe Right
    else if (0 < lateralOffsetToTargetInCM()) {
      strafeAdjust = StrafeP * strafeError + MIN_STRAFE_VALUE;
    }

    // Add left Strafe method
    // Add Right strafe method

  }

  public void runVerticalController() {
    float distanceError = (float) -distanceFromCamToTargetInCM();
    float verticalAdjust = 0.0f;

    // Alter Talon SRX loops here
    distanceFromCamToTargetInCM();

  }

  public void positionRobotPhase1() {

    runRotationController();
    runStrafeController();
  }

  public void stopThePresses() {

  }

  public void activateVisionDriveMode() {
      
  }

  // untested method below
  // The following method updates the vision variables values
  public void getCMDData() {
    if ((System.currentTimeMillis() % 1000) == 0) {
      SmartDashboard.putBoolean(isProcessCMDString, true);
    } else if ((System.currentTimeMillis() % 2000) == 0 && SmartDashboard.getBoolean(isProcessCMDString, false)) {
      if (SmartDashboard.putBoolean(isProcessCMDString, true)) {
        SmartDashboard.putBoolean(isProcessCMDString, false);
      }

    }

  }

  public void smartdashboardTestingMethod() {

  }

}

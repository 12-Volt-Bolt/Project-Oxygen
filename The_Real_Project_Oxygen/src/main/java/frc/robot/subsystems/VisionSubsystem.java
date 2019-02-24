/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;
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
 * This class holds all methods and objects related to the camera We use both
 * the Logitech 930e and the Microsoft LifeCam 3000
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
  public static int measCenterPixels;
  public static int measSeparationPixels;
  public static int measAngleDegrees;
  public static boolean isProcessCmdBool;

  // Smartdashboard Network Table Strings
  public static final String measCenterString = "DB/Slider 0";
  public static final String measSeparationString = "DB/Slider 1";
  public static final String measAngleDegreesString = "DB/Slider 2";
  public static final String isProcessCMDString = "DB/Button 0";

  public VisionSubsystem() {
    topCam = CameraServer.getInstance().startAutomaticCapture(RobotMap.CAMERA_ZERO_ID);
    topCam.setResolution(TOP_CAM_ROW_PIXEL_NUM, TOP_CAM_COL_PIXEL_NUM);
    topCam.setFPS(TOP_CAM_FPS);

    bottomCam = CameraServer.getInstance().startAutomaticCapture(BOTTOM_CAM_NAME, RobotMap.CAMERA_ONE_ID);
    bottomCam.setResolution(BOTTOM_CAM_ROW_PIXEL_NUM, BOTTOM_CAM_COL_PIXEL_NUM);
    bottomCam.setFPS(BOTTOM_CAM_FPS);

    bottomCam.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
    topCam.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
  }

  @Override
  public void initDefaultCommand() {
  }

  public void updateVisionObject(VisionMath vMath) {
    if (OI.zeroSlotController.getTriggerAxis(Hand.kLeft) > 0.5
        && OI.zeroSlotController.getTriggerAxis(Hand.kRight) > 0.5) {
        SmartDashboard.putBoolean(isProcessCMDString, true);
    } else {
       SmartDashboard.putBoolean(isProcessCMDString, false);

    }

    
    measCenterPixels = (int) SmartDashboard.getNumber(measCenterString, Constants_And_Equations.NT_Table_Constant);
    measSeparationPixels = (int) SmartDashboard.getNumber(measSeparationString,
        Constants_And_Equations.NT_Table_Constant);
    measAngleDegrees = (int) SmartDashboard.getNumber(measAngleDegreesString,
        Constants_And_Equations.NT_Table_Constant);

    if (isProcessCmdBool) {
      vMath = new VisionMath(measCenterPixels, measSeparationPixels, measAngleDegrees);
    }

  }

}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.cscore.VideoSource;

/**
 * This class holds all methods and objects related to the camera
 * We use both the Logitech 930e and the Microsoft LifeCam 3000
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
public static final int TOP_CAM_FPS = 10;
public static final int BOTTOM_CAM_FPS = 10;

// Total number of pixel columns
public static final int TOP_CAM_COL_PIXEL_NUM = 480;
public static final int BOTTOM_CAM_COL_PIXEL_NUM = 480;

// Total number of pixel rows
public static final int TOP_CAM_ROW_PIXEL_NUM = 640;
public static final int BOTTOM_CAM_ROW_PIXEL_NUM = 640;

   
public VisionSubsystem() {
  topCam = CameraServer.getInstance().startAutomaticCapture(RobotMap.CAMERA_ZERO_ID);
 // topCam = new UsbCamera("TOP_CAM", RobotMap.CAMERA_ZERO_ID);
  topCam.setResolution(TOP_CAM_ROW_PIXEL_NUM, TOP_CAM_COL_PIXEL_NUM);
  topCam.setFPS(TOP_CAM_FPS);

  bottomCam = CameraServer.getInstance().startAutomaticCapture(RobotMap.CAMERA_ONE_ID);
 // bottomCam = new UsbCamera("BOTTOM_CAM", RobotMap.CAMERA_ONE_ID);
  bottomCam.setResolution(BOTTOM_CAM_ROW_PIXEL_NUM, BOTTOM_CAM_COL_PIXEL_NUM);
  bottomCam.setFPS(BOTTOM_CAM_FPS);

  // The following lines of code will keep the cameras from shutting down, thus decreasing lag
  // Remember, we just want to stop sending the streams, not shut the cameras down.
  // kKeepOpen is equaled to 2
  bottomCam.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
  topCam.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

   //theOnlyCamServer = CameraServer.getInstance().getServer();
   theOnlyCamServer = CameraServer.getInstance().getServer();



}
  @Override
  public void initDefaultCommand() {
  }


}

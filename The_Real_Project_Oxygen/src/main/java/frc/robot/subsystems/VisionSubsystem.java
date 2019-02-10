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
import frc.robot.RobotMap;
import edu.wpi.cscore.VideoSource;

/**
 * This class holds all methods and objects related to vision tracking
 */

 
public class VisionSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
 // setDefaultCommand(new MySpecialCommand());

public static UsbCamera topCam, bottomCam;
public static CameraServer camServerTop, camServerBottom;
public static VideoSink theOnlyCamServer;
public static MjpegServer mjpegServerTop, mjpegServerBottom;

public static final String TOP_CAM_NAME = "Top Camera";
public static final String BOTTOM_CAM_NAME = "Bottom Camera";

public static final int TOP_CAM_FPS = 10;
public static final int BOTTOM_CAM_FPS = 10;

public static final int TOP_CAM_COL_PIXEL_NUM = 480;
public static final int BOTTOM_CAM_COL_PIXEL_NUM = 480;

public static final int TOP_CAM_ROW_PIXEL_NUM = 640;
public static final int BOTTOM_CAM_ROW_PIXEL_NUM = 640;

   

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    
    topCam = CameraServer.getInstance().startAutomaticCapture(RobotMap.CAMERA_ZERO_ID);
    bottomCam = CameraServer.getInstance().startAutomaticCapture(RobotMap.CAMERA_ONE_ID);

  


/*
    topCam = CameraServer.getInstance().startAutomaticCapture(RobotMap.CAMERA_ZERO_ID);
    topCam = new UsbCamera("TOP_CAM", RobotMap.CAMERA_ZERO_ID);
    topCam.setResolution(640, 480);
    topCam.setFPS(10);
    This works!
    */

   /*
    bottomCam = CameraServer.getInstance().startAutomaticCapture(RobotMap.CAMERA_ONE_ID);
    bottomCam = new UsbCamera("BOTTOM_CAM", RobotMap.CAMERA_ONE_ID);
    bottomCam.setResolution(640, 480);
    bottomCam.setFPS(10);
    This Works
    */
    

   // bottomCam.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
   // topCam.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);


    theOnlyCamServer = CameraServer.getInstance().getServer();


    // The following lines of code will keep the cameras from shutting down, thus decreasing lag
    // Remember, we just want to stop sending the streams, not shut the cameras down.
    
    
  }



}

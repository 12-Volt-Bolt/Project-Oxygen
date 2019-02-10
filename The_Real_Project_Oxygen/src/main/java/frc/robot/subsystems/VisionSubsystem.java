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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This class holds all methods and objects related to vision tracking
 */

public static UsbCamera topCam, bottomCam;
public static CameraServer camServerTop, camServerBottom;
public static MjpegServer mjpegServerTop, mjpegServerBottom;

public static final String TOP_CAM_NAME = "Top Camera";
public static final String BOTTOM_CAM_NAME = "Bottom Camera";

public static final int TOP_CAM_FPS = 10;
public static final int BOTTOM_CAM_FPS = 10;

public static final int TOP_CAM_COL_PIXEL_NUM = 480;
public static final int BOTTOM_CAM_COL_PIXEL_NUM = 480;

public static final int TOP_CAM_ROW_PIXEL_NUM = 640;
public static final int BOTTOM_CAM_ROW_PIXEL_NUM = 640;



 
 
public class VisionSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
 // setDefaultCommand(new MySpecialCommand());
   

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }



}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Add your docs here.
 */
public class CameraServerStartInstantCommand extends InstantCommand {
  /**
   * Add your docs here.
   */
  public CameraServerStartInstantCommand() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    UsbCamera topCam = new UsbCamera("Top Camera", 0); 
    MjpegServer mJPEGServer0 = CameraServer.getInstance().addServer("Another_Server0", 1181);
    CvSink sink0 = new CvSink("Open_0");
    mJPEGServer0.setSource(topCam);
    sink0.setSource(topCam);
    CameraServer.getInstance().addCamera(topCam);
    CameraServer.getInstance().startAutomaticCapture();
    CameraServer.getInstance().getVideo(topCam);
    CameraServer.getInstance().putVideo("Top_Cam_PutVideo()_Method", 640, 480);
  }

}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
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
    UsbCamera usbCamera0 = CameraServer.getInstance().startAutomaticCapture();
    usbCamera0.setResolution(320, 240);
    usbCamera0.setFPS(10);
    CvSink cvSink0 = CameraServer.getInstance().getVideo();

    UsbCamera usbCamera1 = CameraServer.getInstance().startAutomaticCapture(1);
    usbCamera1.setResolution(320, 240);
    usbCamera1.setFPS(10);
    CvSink cvSink1 = CameraServer.getInstance().getVideo();
  }

}

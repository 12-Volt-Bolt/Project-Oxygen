/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveWithVisionCommand extends Command {
  
  private double meanMotorSpeeds;

  public DriveWithVisionCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveSub);
    meanMotorSpeeds = 0;
  }
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Robot.driveSub.setTurnControllerSetpointDeg(Robot.navXGyro.getAngle());
    Robot.visionSub.motorControllerRampForVision(true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    meanMotorSpeeds = Robot.driveSub.meanOfDriveMotorSpeeds();
    Robot.visionSub.updateVisionVariables();
    Robot.driveSub.updateDriveCartesian(Robot.visionSub.distanceFromCamToTargetInCM(), 0,0);// Robot.visionSub.distanceFromCamToTargetInCM(),// Robot.driveSub.rotateToAngleRate);
    if(meanMotorSpeeds > Robot.driveSub.meanOfDriveMotorSpeeds()) {
      Robot.visionSub.motorControllerRampForVision(false);
    }           
    else {
      Robot.visionSub.motorControllerRampForVision(true);
    }                                                                           
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.visionSub.motorControllerRampForVision(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.visionSub.motorControllerRampForVision(false);
  }
}

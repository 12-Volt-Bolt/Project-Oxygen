/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class FCDDriveCommand extends Command {
  public FCDDriveCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveSub);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (OI.zeroSlotController.getRawButtonPressed(RobotMap.LEFT_STICK_ID)) {
      Robot.driveSub.driveRampFCD(OI.zeroSlotController.getRawAxis(RobotMap.RIGHT_X_AXIS_ID));
    } 
    else if(!Robot.navXGyro.isConnected()) {
      Robot.driveSub.updateDriveCartesian(OI.zeroSlotController.getX(Hand.kLeft), OI.zeroSlotController.getY(Hand.kLeft), OI.zeroSlotController.getX(Hand.kRight));
    } 
    else {
      Robot.driveSub.updateDriveCartesian(
          OI.zeroSlotController.getX(Hand.kLeft),
          OI.zeroSlotController.getY(Hand.kLeft), 
          OI.zeroSlotController.getX(Hand.kRight), true);
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
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

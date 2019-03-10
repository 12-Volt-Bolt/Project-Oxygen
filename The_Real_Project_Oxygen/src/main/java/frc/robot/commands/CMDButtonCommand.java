/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

public class CMDButtonCommand extends Command {
  public CMDButtonCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.visionSub.CMDButtonOn(true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.visionSub.CMDButtonOn(true);
    Timer.delay(2);
    Robot.visionSub.updateVisionVariables();
    Robot.visionSub.CMDButtonOn(false);
    Timer.delay(0.2);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(!OI.visionStartCombo()) {
      Robot.visionSub.CMDButtonOn(false);
      return false;
      }
      return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.visionSub.CMDButtonOn(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.HatchArm;

public class HatchArmAlternate extends Command {
  private HatchArm hatchArm = new HatchArm();
  private long time;

  public HatchArmAlternate() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(hatchArm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.DiskSub.setSpeed(-0.1);

    if (time > System.currentTimeMillis()) {
      time = System.currentTimeMillis() + 1000;
      if (OI.zeroSlotController.getAButton() == true) {
        hatchArm.ArmUp();
      } else if (OI.zeroSlotController.getBButton() == true) {
        hatchArm.ArmDown();
      }
    }
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    /*
    if(Robot.DiskSub.hatchMotor.getOutputCurrent() > Robot.DiskSub.currentlimit) {
      return true;
    }
    */
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  // Robot.DiskSub.stopThePresses();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
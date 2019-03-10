/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.lifter_commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.statics_and_classes.Constants_And_Equations;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.GenericLiftSubsystem;
import frc.robot.subsystems.LifterSubsystem;
import frc.robot.subsystems.GenericLiftSubsystem.LiftID;

public class ManualLifterCommand extends Command {

  private static LifterSubsystem liftSub;
  private static Constants_And_Equations cAndE;
  private static GenericLiftSubsystem genLift;
  
  public ManualLifterCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    requires(Robot.frontLiftSub);
    requires(Robot.rearLiftSub);
    requires(Robot.topLiftSub);
    requires(liftSub);
    requires(genLift);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //liftSub.moveAllLifters(cAndE.deadzone(OI.oneSlotController.getY(Hand.kRight)), cAndE.deadzone(OI.oneSlotController.getY(Hand.kLeft)), cAndE.deadzone(cAndE.triggersAsJoy()));

    genLift.liftMethod(cAndE.deadzone(OI.oneSlotController.getY(Hand.kRight)), LiftID.frontLift);
    genLift.liftMethod(cAndE.deadzone(OI.oneSlotController.getY(Hand.kLeft)), LiftID.rearLift);
    genLift.liftMethod(cAndE.deadzone(cAndE.triggersAsJoy()), LiftID.topLift);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (Robot.liftSafteyMode != 1) {
      return true;
    } else {
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    genLift.StopThePresses();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    genLift.StopThePresses();
  }
}
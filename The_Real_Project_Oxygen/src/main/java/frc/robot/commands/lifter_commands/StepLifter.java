/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.lifter_commands;
import frc.robot.subsystems.GenericLiftSubsystem;
import frc.robot.Robot;
import frc.robot.statics_and_classes.ClimbSteps;
import frc.robot.statics_and_classes.ClimbSteps.ClimbStep;
import frc.robot.statics_and_classes.ClimbSteps.ClimbSubstep;

import edu.wpi.first.wpilibj.command.Command;

public class StepLifter extends Command {

  private static GenericLiftSubsystem genLift;
  private static ClimbStep[] climbsteps = Robot.climbSteps;
  private static ClimbSubstep[] currentSubstep;
  private static int currentStep = 0;
  private static int lastStep;

  public StepLifter() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(genLift);
  }
  

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (currentStep != lastStep) {
      ClimbStep currentClimbStep = ClimbSteps.GetStep(currentStep);
      currentSubstep[currentClimbStep.substeps.length] = currentClimbStep.substeps[currentClimbStep.substeps.length];
      for (int i = 0; i < currentSubstep.length; i++) {
        genLift.stopRunning(currentSubstep[i].liftID);
      }
      Thread.sleep(100);
    }
    for (int i = 0; i < currentSubstep.length; i++) {
      genLift.StepLift(currentSubstep[i].liftID, currentSubstep[i].lockPos, currentSubstep[i].speed, currentSubstep[i].distance);
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

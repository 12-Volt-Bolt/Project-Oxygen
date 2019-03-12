/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.lifter_commands;
import frc.robot.subsystems.GenericLiftSubsystem;
import frc.robot.subsystems.GenericTestFunctions;
import frc.robot.Robot;
import frc.robot.statics_and_classes.ClimbData;
import frc.robot.statics_and_classes.ClimbSteps;
import frc.robot.statics_and_classes.DataParser;
import frc.robot.statics_and_classes.ClimbSteps.ClimbStep;
import frc.robot.statics_and_classes.ClimbSteps.ClimbSubstep;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Arrays;

public class StepLifter extends Command {

  private static GenericLiftSubsystem genLift;
  private static ClimbStep[] climbsteps = Robot.climbSteps;
  private static ArrayList<ClimbSubstep> currentSubsteps = new ArrayList<ClimbSubstep>();
  private static int currentStep = 0;
  private static int lastStep = 1;
  private static long waitTime;

  private static DataParser parser;
  
  public static ClimbStep[] LV_3_CLIMB_STEPS;

  public StepLifter() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.genericLiftSub);
  }
  

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    LV_3_CLIMB_STEPS = parser.ParseClimbData(ClimbData.LV_3_CLIMB_DATA);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    genLift.UpdateLiftLocation();

    if (currentStep != lastStep) 
    {
      lastStep = currentStep;
      ClimbStep currentClimbStep = LV_3_CLIMB_STEPS[currentStep];
      currentSubsteps = new ArrayList<ClimbSubstep>(Arrays.asList(currentClimbStep.substeps));
      SmartDashboard.putString("liftID", currentSubsteps.get(0).liftID.toString());
      SmartDashboard.putBoolean("locPos", currentSubsteps.get(0).lockPos);
      SmartDashboard.putNumber("speed", currentSubsteps.get(0).speed);
      SmartDashboard.putNumber("distance", currentSubsteps.get(0).distance);

      for (int i = 0; i < currentSubsteps.size(); i++) {
        //genLift.stopRunning(currentSubsteps.get(i).liftID);
      }
    } else 
    {
      for (int i = 0; i < currentSubsteps.size(); i++) {
        genLift.StepLift(currentSubsteps.get(i).liftID, currentSubsteps.get(i).lockPos, currentSubsteps.get(i).speed, currentSubsteps.get(i).distance);
      }
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

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drive_commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants_And_Equations;
import frc.robot.OI;
import frc.robot.Constants_And_Equations.AxisNames;
import frc.robot.subsystems.ControllerFunctions;
import frc.robot.subsystems.DriveSubsystem;

public class BasicDrive extends Command {

  private static DriveSubsystem driveSub;
  private long tempTime = 0;

  public BasicDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(driveSub);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    driveSub.updateDriveCartesian(ControllerFunctions.RollingAverage(AxisNames.leftX, OI.zeroSlotController.getY(Hand.kLeft)),OI.zeroSlotController.getX(Hand.kLeft), ControllerFunctions.RollingAverage(AxisNames.rightX, OI.getZeroZ()));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (ControllerFunctions.CheckJoysticks() == true) {
      tempTime = System.currentTimeMillis() + 100;
      return false;
    } else if(System.currentTimeMillis() > tempTime == true){
      return true;
    } else {
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    driveSub.StopThePresses();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

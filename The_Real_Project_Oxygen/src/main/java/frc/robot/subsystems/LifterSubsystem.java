/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.OI;
import frc.robot.commands.frontLifterCommand;

/**
 * Add your docs here.
 */
public class LifterSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // global variables
  public static int mode = 0;

  // private variables
  private static FrontLiftSubsystem frontLiftSub;
  private static RearLiftSubsystem rearLiftSub;
  private static TopRailSubsystem topLiftSub;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  // checks to see if the current lift saftey mode needs updating and returns the new mode
  // 0 = safety on
  // 1 = manual mode
  // 2 = step mode
  // 3 = reset lifter
  public static int checkLiftSaftey(){

    if (OI.oneSlotController.getBumper(Hand.kLeft) == true && OI.oneSlotController.getBumper(Hand.kRight) == true) {
      if (OI.oneSlotController.getBButton() == true){
        mode = 0; // saftey on
      } else if (OI.oneSlotController.getX(Hand.kLeft) > 0.9 && OI.oneSlotController.getX(Hand.kRight) < -0.9) {
        mode = 1; // manual mode
      } else if (OI.oneSlotController.getX(Hand.kLeft) < -0.9 && OI.oneSlotController.getX(Hand.kRight) > 0.9) {
        mode = 2; // step mode
      } else if (OI.oneSlotController.getStickButton(Hand.kLeft) == true && OI.oneSlotController.getStickButton(Hand.kRight) == true) {
        mode = 3; // reset lifter
      }
    } // else returns mode's (global variable) previous value
    return mode;
  }

  public static void moveAllLifters(double front, double rear, double top) {
    frontLiftSub.liftMethod(front);
    rearLiftSub.liftMethod(rear);
    topLiftSub.liftMethod(top);
  }

  public static void stopAllPresses() {
    frontLiftSub.stopThePresses();
    rearLiftSub.stopThePresses();
    topLiftSub.stopThePresses();
  }

  public static void stopAndStay(boolean hight) {
    
  }
}

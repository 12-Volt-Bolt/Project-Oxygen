/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*---------------------i-------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants_And_Equations.AxisNames;

/**
 * Add your docs here.
 */
public class ControllerFunctions extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static final double[] rollingArray = new double[] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  private static final int raLength = rollingArray.length - 1;

  private static double[] raLeftX = rollingArray;
  public static double[] raLeftY = rollingArray;
  private static double[] raRightX = rollingArray;
  private static double[] raRightY = rollingArray;
  private static double rolledAverageLeftX;
  public static double rolledAverageLeftY;
  private static double rolledAverageRightX;
  private static double rolledAverageRightY;

  private static long savedTimeMili;
  private static final int waitTimeMili = 100;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());\
  }

  public static double[] CallArray(AxisNames whichAxis) {
    switch (whichAxis) {
    case leftX:
      return raLeftX;

    case leftY:
      return raLeftY;

    case rightX:
      return raRightX;

    case rightY:
      return raRightY;

    default:
      return rollingArray;
    }
  }

  public static void SetArray(AxisNames whichAxis, double[] newValue) {
    switch (whichAxis) {
    case leftX:
      raLeftX = newValue;
      break;

    case leftY:
      raLeftY = newValue;
      break;

    case rightX:
      raRightX = newValue;
      break;

    case rightY:
      raRightY = newValue;
      break;

    default:
      break;
    }
  }

  public static double AverageArray(AxisNames whichAxis, double arraySum) {
    switch (whichAxis) {
      case leftX:
        rolledAverageLeftX = arraySum/raLength;
        return rolledAverageLeftX;
  
      case leftY:
        rolledAverageLeftY = arraySum/raLength;
        return rolledAverageLeftY;
  
      case rightX:
        rolledAverageRightX = arraySum/raLength;
        return rolledAverageRightX;
  
      case rightY:
        rolledAverageRightY = arraySum/raLength;
        return rolledAverageRightY;
  
      default:
        return 0;
      }
  }

  public static double GetRolledAverage(AxisNames whichAxis) {
    switch (whichAxis) {
      case leftX:
        return rolledAverageLeftX;
  
      case leftY:
        return rolledAverageLeftY;
  
      case rightX:
        return rolledAverageRightX;
  
      case rightY:
        return rolledAverageRightY;
  
      default:
        return 0;
      }
  } 

  public static double RollingAverage(AxisNames whichAxis, double newInput) {
    
    double raSum = 0;
    double[] oldArray = CallArray(whichAxis);
    double[] tempRollingArray = rollingArray;

    if (Math.abs(newInput) < Math.abs(GetRolledAverage(whichAxis)) == true) {
      for (int i = 0; i < raLength; i++) {
        tempRollingArray[i] = newInput;
      }
      SetArray(whichAxis, tempRollingArray);
      return newInput;
    }
    else if (System.currentTimeMillis() - savedTimeMili > waitTimeMili == true) {
      for (int i = 0; i < raLength; i++) {
        tempRollingArray[i] = oldArray[i + 1];
      }
      tempRollingArray[raLength] = newInput;
      SetArray(whichAxis, tempRollingArray);

      for (int i = 0; i < raLength; i++) {
        raSum += tempRollingArray[i];
      }
      
      savedTimeMili = System.currentTimeMillis();
      return AverageArray(whichAxis, raSum);

    } else {
      return GetRolledAverage(whichAxis);
    }
  }



}

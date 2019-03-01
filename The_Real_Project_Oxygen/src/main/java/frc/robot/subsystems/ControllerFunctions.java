/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*---------------------i-------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class ControllerFunctions extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private double rolledAverage;
  private final double[] rollingArray = new double[] { 0,0,0,0,0,0,0,0,0,0 };
  private double[] raLeftX = rollingArray;
  private double[] raLeftY = rollingArray;
  private double[] raRightX = rollingArray;
  private double[] raRightY = rollingArray;
  private int raLength = rollingArray.length;

  public enum AxisNames
  {
    leftX, leftY, rightX, rightY;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());\
  }

  public double[] CallArray(AxisNames whichAxis){
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

  public void SetArray(AxisNames whichAxis, double[] newValue){
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

  public double RollingAverage(AxisNames whichAxis,  double newInput) {
    double[] oldArray = CallArray(whichAxis);
    double[] tempRollingArray = new double[raLength];
    double raSum = 0;

    for (int i = 0; i < raLength - 1; i++){
      tempRollingArray[i] = oldArray[i++];
    }

    tempRollingArray[raLength] = newInput;
    SetArray(whichAxis, tempRollingArray);

    for (int i = 0; i < raLength; i++) {
      raSum += tempRollingArray[i];
    }

    rolledAverage = raSum/(raLength++);
    return rolledAverage;
  }
}

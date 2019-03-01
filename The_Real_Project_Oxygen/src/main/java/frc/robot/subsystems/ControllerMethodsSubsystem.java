/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class ControllerMethodsSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  double[] leftXJoyArray, rightXJoyArray, leftYJoyArray, rightYJoyArray = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  public enum JoyID{
    leftXJoy
    ,rightXJoy
    ,leftYJoy
    ,rightYJoy
  }

  public double RollingAverage(JoyID joyID, double controllerInput){
    switch (joyID) {
      case leftXJoy:
        leftXJoyArray = moveAverage(controllerInput, leftXJoyArray);
        return makeAverage(leftXJoyArray);
      case leftYJoy:
        leftYJoyArray = moveAverage(controllerInput, leftYJoyArray);
        return makeAverage(leftYJoyArray);
      case rightYJoy:
        rightYJoyArray = moveAverage(controllerInput, rightYJoyArray);
        return makeAverage(rightYJoyArray);
      case rightXJoy:
        rightXJoyArray = moveAverage(controllerInput, rightXJoyArray);
        return makeAverage(rightXJoyArray);
      default:
        return 0.0;
    }
  }

  public static double[] moveAverage(double newNum, double[] array){
    double[] firstArray = array;
    double[] secondArray = array;
    secondArray[0] = newNum;
    int i; 
    for (i = 1; i < array.length; i++){
        secondArray[i] = firstArray[i-1];
    }
    return secondArray;
  }

  public static double makeAverage(double[] array){
    double total = 0;
    for(int i = 0; i < array.length; i++){
        total += array[i];
    }
    return total/array.length;
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.statics_and_classes;

import frc.robot.statics_and_classes.ClimbSteps;
import frc.robot.statics_and_classes.ClimbSteps.ClimbStep;
import frc.robot.statics_and_classes.ClimbSteps.ClimbSubstep;
import frc.robot.subsystems.GenericLiftSubsystem.LiftID;

/**
 * Add your docs here.
 */
public class DataParser {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static <T> T ParseObject (String stringInput, String objectType) {
    

    String stringToParse = stringInput.replaceAll("\\s+", "");

    switch (objectType) {
      case "ClimbStep[]":
        return (T) ParseClimbData(stringInput);

      default:
        return null;
    }
  }

  public static ClimbStep[] ParseClimbData(String stringInput) {
    String[] steps = stringInput.split(":", 0);
    ClimbStep[] climbSteps = new ClimbStep[steps.length];
    for (int i = 0; i < steps.length; i++) {
      climbSteps[i] = ParseClimbSteps(steps[i]);
    }

    return climbSteps;
  }

  public static ClimbStep ParseClimbSteps(String stringInput) {
    String[] stringSubsteps = stringInput.split(";", 0);
    ClimbSubstep[] substeps = new ClimbSubstep[stringSubsteps.length];

    for (int i = 0; i < stringSubsteps.length; i++) {
      String[] values = (stringSubsteps[i].replaceAll("\\s", "")).split(",", 0);
      substeps[i] = new ClimbSteps.ClimbSubstep(LiftID.valueOf(values[0]), Boolean.getBoolean(values[1]), Double.parseDouble(values[2]), Integer.parseInt(values[3]));
    }


    return new ClimbStep(substeps);
  }
}

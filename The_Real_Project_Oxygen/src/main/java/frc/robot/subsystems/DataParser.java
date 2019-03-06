/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.ClimbSteps;
import frc.robot.ClimbSteps.ClimbStep;
import frc.robot.ClimbSteps.ClimbSubstep;
import frc.robot.subsystems.GenericLiftSubsystem.LiftID;

/**
 * Add your docs here.
 */
public class DataParser extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public <T> T ParseObject (String stringInput, T objectType) {
    

    String stringToParse = stringInput.replaceAll("\\s", "");

    switch (objectType.getClass().getName()) {
      case "ClimbStep":
        return (T) ParseClimbStep(stringToParse);

      case "ClimbSubstep":
        return (T) ParseClimbSubstep(stringInput);
    
      default:
        return null;
    }
  }

  public ClimbStep ParseClimbStep (String stringInput) {
      long count = stringInput.chars().filter(ch -> ch == ';').count();
      String[] values = stringInput.split(";", 0);
      ClimbStep temp = new ClimbStep(new ClimbSteps.ClimbSubstep[Math.toIntExact(count)]);

      for (int i = 0; i > count; i++){
          temp.substeps[i] = ParseClimbSubstep(values[i]);
      }

      return temp;
  }

  public ClimbSubstep ParseClimbSubstep (String stringInput) {

      String[] values = stringInput.split(",", 0);
      return new ClimbSteps.ClimbSubstep(LiftID.valueOf(values[0]), Boolean.getBoolean(values[1]), Double.parseDouble(values[2]), Double.parseDouble(values[3]));
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.statics_and_classes;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class ClimbData extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static final String LV_3_CLIMB_DATA = "testLift, false, -0.1, 20: testLift, false, 0.1, 20: testLift, true, 0.2, 0";

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}

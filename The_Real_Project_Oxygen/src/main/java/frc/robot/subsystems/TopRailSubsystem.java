/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */

public class TopRailSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public WPI_TalonSRX topRailMotor;

 public TopRailSubsystem() {
  topRailMotor = new WPI_TalonSRX(RobotMap.TOP_RAIL_MOTOR_ID);
 }
 
  @Override
  public void initDefaultCommand() {
   
  }

  // TODO: Implement the following methods:
  // A method that sets the speed of the motor controller with a parameter
  // 3 different  methods that set a LOW, MED, and HIGH speed just by calling
  // A "stop the presses" method
  
}

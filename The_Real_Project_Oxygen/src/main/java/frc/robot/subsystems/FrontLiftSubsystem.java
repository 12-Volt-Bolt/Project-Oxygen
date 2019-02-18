/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.WPI_MotorSafetyImplem;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class FrontLiftSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public WPI_TalonSRX frontLifterMotor;
  public static double maxLifterCurrent;
  public static double speed;

  
  public FrontLiftSubsystem() {
    WPI_TalonSRX frontLifterMotor = new WPI_TalonSRX(RobotMap.FRONT_RAIL_MOTOR_ID);
  }
// TODO: Implement the following methods:
  // A method that sets the speed of the motor controller with a parameter
  // 3 different  methods that set a LOW, MED, and HIGH speed just by calling them
  // A "stop the presses" method. set motor to zero
  // frontLifterMotor.configOpenloopRamp(secondsFromNeutralToFull, timeoutMs)
  // read documentation about the open loop ramp function
  // The motors can not just go from 0 to 50 in an instant
  // only power, or .set() motor, "IF" its current is below the motor current limit variable
  // only "IF" that is true and zero otherwise
  
  @Override
  public void initDefaultCommand() {
     // frontLifterMotor.configClosedloopRamp(0.5);
     // frontLifterMotor.configOpenloopRamp(secondsFromNeutralToFull, timeoutMs)
    //  frontLifterMotor.configClosedloopRamp(secondsFromNeutralToFull)
    }


}

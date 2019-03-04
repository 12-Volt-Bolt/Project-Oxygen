/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants_And_Equations;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class FrontLiftSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private WPI_TalonSRX motor;
  public static double maxLifterCurrent = 1;
  public static int conCurrentLimit = 88;
  public static double rampTimeInSecs = 0.2;
  public static double off = 0;
  public static double lowSpeed = 0.1;
  public static double medSpeed = 0.5;
  public static double highSpeed = 0.7;
  private int positionOption;

  public FrontLiftSubsystem() {
    motor = new WPI_TalonSRX(RobotMap.FRONT_RAIL_MOTOR_ID);
    motor.configOpenloopRamp(rampTimeInSecs);
    positionOption = 0;
  }

  // TODO: Implement the following methods:
  // A method that sets the speed of the motor controller with a parameter
  // 3 different methods that set a LOW, MED, and HIGH speed just by calling them
  // A "stop the presses" method. set motor to zero
  // frontLifterMotor.configOpenloopRamp(secondsFromNeutralToFull, timeoutMs)
  // read documentation about the open loop ramp function
  // The motors can not just go from 0 to 50 in an instant
  // only power, or .set() motor, "IF" its current is below the motor current
  // limit variable
  // only "IF" that is true and zero otherwise

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  public void liftMethod() {

    this.positionOption++;
    switch (this.positionOption) {
    case 1:
      setSpeed(highSpeed);
      break;
    case 2:
      setSpeed(off);
      break;
    case 3:
      setSpeed(-highSpeed);
      break;
    default:
      setSpeed(off);
      this.positionOption = 0;
      break;
    }

  }

  // moves lifter at speed given in paramaters
  public void liftMethod(double speed) {
    setSpeed(Constants_And_Equations.deadzone(speed));
  }

  // Will move at given speed and use encoder to stay at set position
  public void liftMethod(double speed, boolean lock) {

  }

  public void stopThePresses() {
    setSpeed(Constants_And_Equations.zero);
  }

  @Override
  public void initDefaultCommand() {

  }
}

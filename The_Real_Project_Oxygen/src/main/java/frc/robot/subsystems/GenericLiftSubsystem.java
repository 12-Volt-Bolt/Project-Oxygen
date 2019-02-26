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
public class GenericLiftSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public enum LiftIds {
    frontLift(7)
    ,rearLift(6)
    ,topLift(5);

    private int value;

    private LiftIds(int value) {
      this.value = value;
    }
    public int getValue() {
      return value;
    }
  }

  public int frontLift = RobotMap.FRONT_RAIL_MOTOR_ID;
  public int rearLift = RobotMap.REAR_RAIL_MOTOR_ID;
  public int topLift = RobotMap.TOP_RAIL_MOTOR_ID;

  public void liftMethod(double speed, String lift) {

  }

  public void liftMotor(int liftID) {
    
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    liftMethod(1, "frontLift");
  }
}

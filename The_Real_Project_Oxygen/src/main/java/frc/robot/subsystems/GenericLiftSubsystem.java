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
public class GenericLiftSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private WPI_TalonSRX frontLift = new WPI_TalonSRX(RobotMap.FRONT_RAIL_MOTOR_ID);
  private WPI_TalonSRX rearLift = new WPI_TalonSRX(RobotMap.REAR_RAIL_MOTOR_ID);
  private WPI_TalonSRX topLift = new WPI_TalonSRX(RobotMap.TOP_RAIL_MOTOR_ID);

  private Constants_And_Equations cAndE;

  //private int[] liftID = new int[] { 7,6,5 };
  
  public enum LiftID {
    frontLift(7)
    ,rearLift(6)
    ,topLift(5);

    private final int id;

    LiftID(int id) {
      this.id = id;
    }
    public int getID() {
      return id;
    }
  }

  //public int frontLift = RobotMap.FRONT_RAIL_MOTOR_ID;
  //public int rearLift = RobotMap.REAR_RAIL_MOTOR_ID;
  //public int topLift = RobotMap.TOP_RAIL_MOTOR_ID;

  public GenericLiftSubsystem() {
    frontLift.configOpenloopRamp(cAndE.rampTimeInSecs);
    rearLift.configOpenloopRamp(cAndE.rampTimeInSecs);
    topLift.configOpenloopRamp(cAndE.rampTimeInSecs);
  }

  public void liftMethod(double speed, LiftID lifterName) {
    setMotor(speed, lifterName);
  }

  public void setMotor(double speed, LiftID lifterID) {
    switch (lifterID) {
      case frontLift:
        frontLift.set(speed);
        break;
      case rearLift:
        rearLift.set(speed);
        break;
      case topLift:
        topLift.set(speed);
        break;
      default:
        String message = "Motor ID '" + lifterID + "' is not valid!";
        System.out.print(message);
        break;
    }
  }

  public void StopThePresses() {
    frontLift.set(0);
    rearLift.set(0);
    topLift.set(0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    StopThePresses();
  }
}
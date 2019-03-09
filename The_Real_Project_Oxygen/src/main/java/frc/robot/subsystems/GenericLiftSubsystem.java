/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.statics_and_classes.Constants_And_Equations;
import frc.robot.statics_and_classes.RobotMap;

/**
 * Add your docs here.
 */
public class GenericLiftSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static WPI_TalonSRX frontLift = new WPI_TalonSRX(RobotMap.FRONT_RAIL_MOTOR_ID);
  private static WPI_TalonSRX rearLift = new WPI_TalonSRX(RobotMap.REAR_RAIL_MOTOR_ID);
  private static WPI_TalonSRX topLift = new WPI_TalonSRX(RobotMap.TOP_RAIL_MOTOR_ID);

  private Constants_And_Equations cAndE;

  private static double[] liftLocation = new double[3];
  private static Boolean[] liftRunning = new Boolean[3];

  //private int[] liftID = new int[] { 7,6,5 };
  
  public enum LiftID {
    frontLift
    ,rearLift
    ,topLift;
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

  public static void setMotor(double speed, LiftID lifterID) {
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

  public void stopRunning(LiftID liftID) {
    liftRunning[liftID.ordinal()] = false;
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

  public static void StepLift(LiftID liftID, Boolean locPos, double speed, double distance) {
    if (locPos == true) {
      LiftAndStay(liftID, speed, distance);
    } else {
      LiftAndCoast(liftID, speed, distance);
    }
  }

  private static void LiftAndStay(LiftID liftID, double speed, double distance) {
    liftRunning[liftID.ordinal()] = true;

    double currentLiftLocation = liftLocation[liftID.ordinal()];
    double realSpeed = speed; 
    double tempSpeed;

    if (distance < currentLiftLocation) {
      realSpeed = -realSpeed;
    }

    while (liftRunning[liftID.ordinal()] == true) {
      if (Constants_And_Equations.WithinRange(distance, currentLiftLocation, 5) == true) {
        tempSpeed = 0;
      } else if (distance > currentLiftLocation == true) {
        tempSpeed = - realSpeed;
      } else {
        tempSpeed = realSpeed;
      }
      
      setMotor(tempSpeed, liftID);
    }
  }

  private static void LiftAndCoast(LiftID liftID, double speed, double distance) {
    double currentLiftLocation = liftLocation[liftID.ordinal()];
    double realSpeed = speed;

    if (distance < currentLiftLocation) {
      realSpeed = -realSpeed;
    }

    while (Constants_And_Equations.WithinRange(distance, currentLiftLocation, 5) == true || liftRunning[liftID.ordinal()] == true) {
      setMotor(realSpeed, liftID);
    }
    liftRunning[liftID.ordinal()] = false;
  }
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.statics_and_classes.Constants_And_Equations;
import frc.robot.statics_and_classes.RobotMap;

/**
 * Add your docs here.
 */
public class GenericLiftSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static WPI_TalonSRX frontLifter = new WPI_TalonSRX(RobotMap.FRONT_RAIL_MOTOR_ID);
  private static WPI_TalonSRX rearLifter = new WPI_TalonSRX(RobotMap.REAR_RAIL_MOTOR_ID);
  private static WPI_TalonSRX topLifter = new WPI_TalonSRX(RobotMap.TOP_RAIL_MOTOR_ID);
  private static WPI_TalonSRX testLifter = new WPI_TalonSRX(1);

  //private static Encoder frontEncoder = new Encoder(0, 1);
  //private static Encoder rearEncoder = new Encoder(2, 3);
  //private static Encoder topEncoder = new Encoder(4, 5);
  private static Encoder testEncoder = new Encoder(0, 1);

  private static Constants_And_Equations cAndE;

  private static int[] liftLocation = new int[] { 0,0,0,0 };
  public static Boolean[] liftRunning = new Boolean[] { false,false,false,false };
  
  public enum LiftID {
    frontLift
    ,rearLift
    ,topLift
    ,testLift;
  }

  public GenericLiftSubsystem() {
    frontLifter.configOpenloopRamp(cAndE.rampTimeInSecs);
    rearLifter.configOpenloopRamp(cAndE.rampTimeInSecs);
    topLifter.configOpenloopRamp(cAndE.rampTimeInSecs);
    testLifter.configOpenloopRamp(cAndE.rampTimeInSecs);
    
  }

  public void liftMethod(double speed, LiftID lifterName) {
    setMotor(speed, lifterName);
  }

  public static void setMotor(double speed, LiftID liftID) {
    switch (liftID) {
      case frontLift:
        frontLifter.set(speed);
        break;
      case rearLift:
        rearLifter.set(speed);
        break;
      case topLift:
        topLifter.set(speed);
        break;
      case testLift:
        testLifter.set(speed);
        break;
      default:
        String message = "Motor ID '" + liftID + "' is not valid!";
        System.out.print(message);
        break;
    }
  }

  public static void StopRunning(LiftID liftID) {
    liftRunning[liftID.ordinal()] = false;
    setMotor(0, liftID);
  }

  public void StopThePresses() {
    frontLifter.set(0);
    rearLifter.set(0);
    topLifter.set(0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    StopThePresses();
  }

  public static void StepLift(LiftID liftID, Boolean locPos, double speed, int distance) {
    liftRunning[liftID.ordinal()] = true;

    if (locPos == true) {
      LiftAndStay(liftID, speed, distance);
    } else {
      LiftAndCoast(liftID, speed, distance);
    }
  }

  private static void LiftAndStay(LiftID liftID, double speed, int distance) {
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

  private static void LiftAndCoast(LiftID liftID, double speed, int distance) {
    double realSpeed = speed;

    if (distance < liftLocation[liftID.ordinal()]) {
      realSpeed = -realSpeed;
    }

    while (Constants_And_Equations.WithinRange(distance, liftLocation[liftID.ordinal()], 5) != true && liftRunning[liftID.ordinal()] == true) {
      setMotor(realSpeed, liftID);
    }
    liftRunning[liftID.ordinal()] = false;
  }

  private static void LiftTillStopped(LiftID liftID, double speed) {

    while (liftRunning[liftID.ordinal()] == true) {
      setMotor(speed, liftID);
    }
  }

  public static void LiftTimed(LiftID liftID, double speed, int distance) {

    long time = System.currentTimeMillis() + (Math.abs(distance) * 100);
    while (System.currentTimeMillis() < time && liftRunning[liftID.ordinal()] == true) {
      setMotor(speed, liftID);
    }
    StopRunning(liftID);
    System.out.println("motor off");
  }

  public static void UpdateLiftLocation() {
    liftLocation[3] = (int) Math.round(liftLocation[3] / 5.7);
  }

  public static void ResetEncoder(LiftID liftID) {
    switch (liftID) {
      case frontLift:
        
        break;
      case rearLift:

        break;
      case topLift:
        break;
      case testLift:
        testEncoder.reset();
        break;
    
      default:
        break;
    }
  }

  public static void ResetLift(LiftID liftID) {
    long time = System.currentTimeMillis() + 5000;
    liftRunning[liftID.ordinal()] = true;
    switch (liftID) {
      case testLift:
        LiftTimed(LiftID.testLift, -0.3, 50);
        break;
    
      default:
        break;

      }

    while (time > System.currentTimeMillis()) {}
    liftRunning[liftID.ordinal()] = false;
    ResetEncoder(liftID);
  }


}
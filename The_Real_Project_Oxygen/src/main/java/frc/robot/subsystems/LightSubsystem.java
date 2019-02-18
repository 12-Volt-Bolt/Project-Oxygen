/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class LightSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  static Spark lightCon = new Spark(0);
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public static void letThereBeLight() {
    lightCon.set(0.67);
    Timer.delay(5);
  }

  public static void CycleTheColors(){
    double i;
    for (i=-0.87; i<0.99; i+=0.02){
      lightCon.set(i);
      Timer.delay(5);
    }
  }

  public static void regularRed(){
    lightCon.set(0.61);
    Timer.delay(5);
  }

  public static void turnEmOff(){
    lightCon.set(0);
    Timer.delay(5);
  }

  public static void redStrobe(){
    lightCon.set(-0.11);
    Timer.delay(5);
  }

  public static void blueStrobe(){
    lightCon.set(-0.09);
    Timer.delay(5);
  }

  public static void whiteStrobe(){
    lightCon.set(-0.05);
    Timer.delay(5);
  }

  public static void redHeartbeat(){
    lightCon.set(-0.25);
    Timer.delay(5);
  }

  public static void redBreath(){
    lightCon.set(-0.15);
    Timer.delay(5);
  }
}

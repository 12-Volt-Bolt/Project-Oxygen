/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.OI;
import frc.robot.statics_and_classes.RobotMap;

/**
 * Add your docs here.
 */
public class GenericTestFunctions extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static WPI_TalonSRX hatchMotor = new WPI_TalonSRX(1);
  public static double currentlimit = 25;

  public static int hatchLocationEnc;
  public static int hatchlocationTime;

  public GenericTestFunctions() {
    hatchMotor.set(0);
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public static void ChangeMotorSpeed() {
    if(OI.zeroSlotController.getPOV() == 0) {
     changeMotorSpeedBy(0.1);
    }

    if(OI.zeroSlotController.getPOV() == 180) {
      changeMotorSpeedBy(-0.1);
    }
    
    if(OI.zeroSlotController.getBButton() == true) {
      stopThePresses();
    }
  }

  public static void SetSpeed(double speed) {
    hatchMotor.set(speed);
  }


  public static void changeMotorSpeedBy(double speedChange) {
    hatchMotor.set(hatchMotor.get() + speedChange); 
  }

  public static void stopThePresses() {
    hatchMotor.set(0);
  }
  
  
}

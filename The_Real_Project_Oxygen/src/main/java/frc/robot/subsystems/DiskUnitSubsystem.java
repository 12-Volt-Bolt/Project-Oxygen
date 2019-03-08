/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.OI;
import frc.robot.statics_and_classes.RobotMap;

/**
 * Add your docs here.
 */
public class DiskUnitSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public WPI_VictorSPX hatchMotor;
  public static double currentlimit = 25;

  public static int hatchLocationEnc;
  public static int hatchlocationTime;
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    hatchMotor = new WPI_VictorSPX(RobotMap.HATCH_MOTOR_ID);
    hatchMotor.set(0);
  }

  public void hatchStepSpeed(boolean buttonPos, boolean buttonNeg, boolean offButton) {
    if(buttonPos) {
     changeMotorSpeedBy(0.1);
    }

    if(buttonNeg) {
      changeMotorSpeedBy(-0.1);
    }
    
    if(offButton) {
      stopThePresses();
    }
  }

  public void setSpeed(double speed) {
    hatchMotor.set(speed);
  }


  public void changeMotorSpeedBy(double speedChange) {
    hatchMotor.set(hatchMotor.get() + speedChange); 
  }

  public void stopThePresses() {
    hatchMotor.set(0);
  }
  
  
}

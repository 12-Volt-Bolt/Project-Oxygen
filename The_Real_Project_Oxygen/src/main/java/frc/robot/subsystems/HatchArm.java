/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.statics_and_classes.RobotMap;

/**
 * Add your docs here.
 */
public class HatchArm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static WPI_TalonSRX armMotor = new WPI_TalonSRX(RobotMap.HATCH_MOTOR_ID);
  private static Encoder armEncoder = new Encoder(8, 9);
  
  private static final int upPosition = 350;
  private static final int downPosition = 0;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public static void ArmUp() {
    while (armEncoder.get() < upPosition) {
      armMotor.set(0.05);
    }
    armMotor.set(0);
  }

  public static void ArmDown() {
    while (armEncoder.get() > downPosition) {
      armMotor.set(-0.05);
    }
    armMotor.set(0);
  }
}

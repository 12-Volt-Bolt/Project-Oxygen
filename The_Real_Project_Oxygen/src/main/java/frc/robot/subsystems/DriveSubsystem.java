/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.OI;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

WPI_TalonSRX frontLeft = new WPI_TalonSRX(RobotMap.FRONT_LEFT_MOTOR_ID);
WPI_TalonSRX frontRight = new WPI_TalonSRX(RobotMap.FRONT_RIGHT_MOTOR_ID);
WPI_TalonSRX rearLeft = new WPI_TalonSRX(RobotMap.REAR_LEFT_MOTOR_ID);
WPI_TalonSRX rearRight = new WPI_TalonSRX(RobotMap.REAR_RIGHT_MOTOR_ID);

public MecanumDrive mecDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void executeMecanumDrive() {
    mecDrive.setSafetyEnabled(true);
    // Look at this and fix it!!!!!!!!!!!!!!!
  mecDrive.driveCartesian(OI.zeroSlotController.getX(Hand.kLeft),
        OI.zeroSlotController.getY(Hand.kRight),
        -OI.zeroSlotController.getRawAxis(RobotMap.RIGHT_X_AXIS_ID));

        mecDrive.driveCartesian(OI.zeroSlotController.getY(Hand.kLeft), OI.zeroSlotController.getX(Hand.kLeft), OI.zeroSlotController.getY(Hand.kRight));
} 

  public void CorrectMotorDirectionForMecanumDrive() {
    // Please do not edit this unless you know the purpose of it.

   //  frontRight.setInverted(true);
    // frontLeft.setInverted(true);
    // rearLeft.setInverted(true);
   // rearRight.setInverted(true);


}
public void StopThePresses() {
  mecDrive.driveCartesian(RobotMap.MOTOR_OFF, RobotMap.MOTOR_OFF, RobotMap.MOTOR_OFF);
}
}

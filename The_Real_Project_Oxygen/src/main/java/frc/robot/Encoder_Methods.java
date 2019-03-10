/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.statics_and_classes.RobotMap;

import edu.wpi.first.wpilibj.Encoder;

/**
 * Add your docs here.
 */
public class Encoder_Methods {
    // Overload- Turn 2 specified wheels, specified distance, specified speed, in
  // centimeters
  public void inputTurnCm(WPI_TalonSRX esc, WPI_TalonSRX esc2, Encoder encoder, int distance, double speed) {
    encoder.reset();
    esc.set(speed);
    esc2.set(speed);
    while (Math.abs(updateEncoderReadCm(encoder)) < Math.abs(distance)) {
    }
    esc.set(RobotMap.MOTOR_OFF);
    esc2.set(RobotMap.MOTOR_OFF);
  }

  // Turn specified wheel, specified distance, specified speed, in inches
  public void inputTurnIn(WPI_TalonSRX esc, Encoder encoder, int distance, double speed) {
    encoder.reset();
    esc.set(speed);
    while (Math.abs(updateEncoderReadIn(encoder)) < Math.abs(distance)) {
    }
    esc.set(RobotMap.MOTOR_OFF);
  }

  // Overload- Turn 2 specified wheels, specified distance, specified speed, in
  // inches
  public void inputTurnIn(WPI_TalonSRX esc, WPI_TalonSRX esc2, Encoder encoder, int distance, double speed) {
    encoder.reset();
    esc.set(speed);
    esc2.set(speed);
    while (Math.abs(updateEncoderReadIn(encoder)) < Math.abs(distance)) {
    }
    esc.set(RobotMap.MOTOR_OFF);
    esc2.set(RobotMap.MOTOR_OFF);
  }

  // Move entire robot Specified distance, specified speed, in centimeters
  public void inputMoveCm(Encoder encoder, int distance, double speed) {
    updateEncoderReset(encoder);
    Robot.driveSub.updateMoveSet(speed);
    while (Math.abs(updateEncoderReadCm(encoder)) < Math.abs(distance)) {
    }
    Robot.driveSub.StopThePresses();
  }

  // Move entire robot Specified distance, specified speed, in inches
  public void inputMoveIn(Encoder encoder, int distance, double speed) {
    updateEncoderReset(encoder);
    Robot.driveSub.updateMoveSet(speed);
    while (Math.abs(updateEncoderReadIn(encoder)) < Math.abs(distance)) {
    }
    Robot.driveSub.StopThePresses();
  }

  /*
   * //Updates PID rotateToAngleRate to input specified in paramaters public void
   * UpdatePidSet(double output) { rotateToAngleRate = output; }
   */

  /// Encoder methods should go here. Please make sure to have an encoder object
  /// as a parameter.
  // Wheels have an 8 in diameter
  // Methods we need: Drive "x" distance, zero encoder each encoder, zero both
  /// enocoders

  // Reset one encoder
  public void updateEncoderReset(Encoder encoder) {
    encoder.reset();
  }

  // Overload- Reset both encoders
  public void updateEncoderReset(Encoder encoder1, Encoder encoder2) {
    encoder1.reset();
    encoder2.reset();
  }

  // Read encoder and calculate distance turned in Centimeters
  public int updateEncoderReadCm(Encoder encoder) {
    return (int) Math.round(encoder.get() * 63.84);
  }

  // Read encoder and calculate distance turned in inches
  public int updateEncoderReadIn(Encoder encoder) {
    return (int) Math.round(encoder.get() * 25.13);

  }

  // Turn specified wheel, specified distance, specified speed, in centimeters
  public void inputTurnCm(WPI_TalonSRX esc, Encoder encoder, int distance, double speed) {
    encoder.reset();
    esc.set(speed);
    while (Math.abs(updateEncoderReadCm(encoder)) < Math.abs(distance)) {
    }
    esc.set(RobotMap.MOTOR_OFF);
  }

  // Move entire robot Specified distance, specified speed, in centimeters
  public void moveDistanceCm(Encoder encoder, int distance, double speed) {
    encoderReset(encoder);
    Robot.driveSub.setAllMotors(speed);
    while (Math.abs(encoderReadCm(encoder)) < Math.abs(distance)) {
    }
    Robot.driveSub.StopThePresses();
  }

  // Move entire robot Specified distance, specified speed, in inches
  public void moveDistanceIn(Encoder encoder, int distance, double speed) {
    encoderReset(encoder);
    Robot.driveSub.setAllMotors(speed);
    while (Math.abs(encoderReadIn(encoder)) < Math.abs(distance)) {
    }
    Robot.driveSub.StopThePresses();
  }

  // Reset one encoder
  public void encoderReset(Encoder encoder) {
    encoder.reset();
  }

  // Reset both encoders
  public void encoderReset(Encoder encoder1, Encoder encoder2) {
    encoder1.reset();
    encoder2.reset();
  }

  // Read encoder and calculate distance turned in Centimeters
  public int encoderReadCm(Encoder encoder) {
    return (int) Math.round(encoder.get() * 63.84);
  }

  // Read encoder and calculate distance turned in inches
  public int encoderReadIn(Encoder encoder) {
    return (int) Math.round(encoder.get() * 25.13);

  }

  // Turn specified wheel, specified distance, specified speed, in centimeters
  public void moveWheelDistanceCm(WPI_TalonSRX esc, Encoder encoder, int distance, double speed) {
    encoder.reset();
    esc.set(speed);
    while (Math.abs(encoderReadCm(encoder)) < Math.abs(distance)) {
    }
    esc.set(RobotMap.MOTOR_OFF);
  }
    
}

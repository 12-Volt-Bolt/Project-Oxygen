/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants_And_Equations;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.DefaultDriveCommand;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem implements PIDOutput {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static WPI_TalonSRX frontLeft = new WPI_TalonSRX(RobotMap.FRONT_LEFT_MOTOR_ID);
  public static WPI_TalonSRX frontRight = new WPI_TalonSRX(RobotMap.FRONT_RIGHT_MOTOR_ID);
  public static WPI_TalonSRX rearLeft = new WPI_TalonSRX(RobotMap.REAR_LEFT_MOTOR_ID);
  public static WPI_TalonSRX rearRight = new WPI_TalonSRX(RobotMap.REAR_RIGHT_MOTOR_ID);

  // PID, YAY!!!
  public static PIDController turnController;
  public static double rotateToAngleRate;

  static final double kP = 0.03;
  static final double kI = 0.00;
  static final double kD = 0.00;
  static final double kF = 0.00;

  // variables for tank driving
  private static double newZero = 0.00;
  private static boolean newNewZero = true;

  /* This tuning parameter indicates how close to "on target" the */
  /* PID Controller will attempt to get. */

  static final double kToleranceDegrees = 2.0f;

  public static MecanumDrive mecDrive = new MecanumDrive(frontLeft, rearRight, frontRight, rearLeft);

  public DriveSubsystem() {
    super();

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DefaultDriveCommand());
    turnController = new PIDController(kP, kI, kD, kF, Robot.navXGyro, this);
    turnController.setInputRange(-180.0f, 180.0f);
    turnController.setOutputRange(-1.0, 1.0);
    turnController.setAbsoluteTolerance(kToleranceDegrees);
    turnController.setContinuous(true);

  }

  public void executeMecanumDrive() {
    mecDrive.setSafetyEnabled(false);

    boolean rotateToAngle = false;
    double currentRotationRate;

    switch (OI.zeroSlotController.getPOV()) {
    case 0:
      SmartDashboard.putBoolean("Can you see this (POV turn 0)", rotateToAngle);
      turnController.setSetpoint(0.0f);
      rotateToAngle = true;
      break;
    case 45:
      turnController.setSetpoint(45.0f);
      rotateToAngle = true;
      break;
    case 90:
      turnController.setSetpoint(90.0f);
      rotateToAngle = true;
      break;
    case 135:
      turnController.setSetpoint(135.0f);
      rotateToAngle = true;
      break;
    case 180:
      turnController.setSetpoint(179.9f);
      rotateToAngle = true;
      break;
    case 225:
      turnController.setSetpoint(225.0f);
      rotateToAngle = true;
      break;
    case 270:
      turnController.setSetpoint(270.0f);
      rotateToAngle = true;
      break;
    case 315:
      turnController.setSetpoint(315.0f);
      rotateToAngle = true;
      break;
    }
    if (rotateToAngle) {
      turnController.enable();
      currentRotationRate = rotateToAngleRate;
    } else {
      turnController.disable();
      // I don't know why getX has to be negitive, but let's just go with it
      currentRotationRate = Constants_And_Equations.deadzone(-OI.zeroSlotController.getX(Hand.kRight), 0.1);
      turnController.disable();
    }
    mecDrive.driveCartesian(Constants_And_Equations.deadzone(-OI.zeroSlotController.getX(Hand.kLeft), 0.1),
        -Constants_And_Equations.deadzone(-OI.zeroSlotController.getY(Hand.kLeft), 0.1), currentRotationRate,
        -Robot.navXGyro.getAngle());
  }

  public void driveRamp(double twist) {
    mecDrive.driveCartesian(
        Constants_And_Equations
            .parabola(Constants_And_Equations.deadzone(-OI.zeroSlotController.getX(Hand.kLeft), 0.1)),
        Constants_And_Equations
            .parabola(-Constants_And_Equations.deadzone(-OI.zeroSlotController.getY(Hand.kLeft), 0.1)),
        twist, -Robot.navXGyro.getAngle());
  }

  public static void turnToAngle(double angle) {
    turnController.setSetpoint(angle);
    double currentRotationRate;
    currentRotationRate = rotateToAngleRate;
    mecDrive.driveCartesian(Constants_And_Equations.deadzone(-OI.zeroSlotController.getX(Hand.kLeft), 0.1),
        -Constants_And_Equations.deadzone(-OI.zeroSlotController.getY(Hand.kLeft), 0.1), currentRotationRate,
        -Robot.navXGyro.getAngle());

  }

  // We may make our own mecamum method someday
  public void homeBrewMecanumMethod() {

  }

  public void CorrectMotorDirectionForMecanumDrive() {
    // Please do not edit this unless you know the purpose of it.
    frontRight.setInverted(true);
    frontLeft.setInverted(true);
    rearLeft.setInverted(true);
    rearRight.setInverted(true);

  }

  public static void StopThePresses() {
    mecDrive.driveCartesian(RobotMap.MOTOR_OFF, RobotMap.MOTOR_OFF, RobotMap.MOTOR_OFF);
    mecDrive.setSafetyEnabled(true);
  }

  public static void setAllMotors(double speed) {
    frontRight.set(speed);
    frontLeft.set(speed);
    rearRight.set(speed);
    rearLeft.set(speed);

  }

  public void pidWrite(double output) {
    rotateToAngleRate = output;
  }

  /// Encoder methods should go here. Please make sure to have an encoder object
  /// as a parameter.
  // Wheels have an 8 in diameter
  // Methods we need: Drive "x" distance, zero encoder each encoder, zero both
  /// enocoders

  // Reset one encoder
  public static void EncoderReset(Encoder encoder) {
    encoder.reset();
  }

  // Reset both encoders
  public static void EncoderReset(Encoder encoder1, Encoder encoder2) {
    encoder1.reset();
    encoder2.reset();
  }

  // Read encoder and calculate distance turned in Centimeters
  public static int EncoderReadCm(Encoder encoder) {
    return (int) Math.round(encoder.get() * 63.84);
  }

  // Read encoder and calculate distance turned in inches
  public static int EncoderReadIn(Encoder encoder) {
    return (int) Math.round(encoder.get() * 25.13);

  }

  // Turn specified wheel, specified distance, specified speed, in centimeters
  public static void MoveWheelDistanceCm(WPI_TalonSRX esc, Encoder encoder, int distance, double speed) {
    encoder.reset();
    esc.set(speed);
    while (Math.abs(EncoderReadCm(encoder)) < Math.abs(distance)) {
    }
    esc.set(RobotMap.MOTOR_OFF);
  }

  // Turn specified wheel, specified distance, specified speed, in inches
  public static void MoveWheelDistanceIn(WPI_TalonSRX esc, Encoder encoder, int distance, double speed) {
    encoder.reset();
    esc.set(speed);
    while (Math.abs(EncoderReadIn(encoder)) < Math.abs(distance)) {
    }
    esc.set(RobotMap.MOTOR_OFF);
  }

  // Move entire robot Specified distance, specified speed, in centimeters
  public static void MoveDistanceCm(Encoder encoder, int distance, double speed) {
    EncoderReset(encoder);
    setAllMotors(speed);
    while (Math.abs(EncoderReadCm(encoder)) < Math.abs(distance)) {
    }
    StopThePresses();
  }

  // Move entire robot Specified distance, specified speed, in inches
  public static void MoveDistanceIn(Encoder encoder, int distance, double speed) {
    EncoderReset(encoder);
    setAllMotors(speed);
    while (Math.abs(EncoderReadIn(encoder)) < Math.abs(distance)) {
    }
    StopThePresses();
  }

  // Move robot forward/backwards without rotation drifting
  // Strafe robot
  // Robot can still turn without inturupting movement
  // Relies on newZero to work
  public static void MoveMecanumStraight() {
    mecDrive.setSafetyEnabled(false);

    // if turning, reset newZero
    if (Constants_And_Equations.deadzone(OI.zeroSlotController.getX(Hand.kRight)) != 0) {
      newZero = Robot.navXGyro.getAngle();
    }
    // drive forward based on leftJoyX, Strafe based on LeftJoyY, turn based on
    // AmountDrifted+RightJoyX
    mecDrive.driveCartesian(Constants_And_Equations.deadzone(OI.zeroSlotController.getY(Hand.kLeft)),
        Constants_And_Equations.deadzone(OI.zeroSlotController.getX(Hand.kLeft)),
        Math.round(Robot.navXGyro.getAngle() - newZero)
            + Constants_And_Equations.deadzone(OI.zeroSlotController.getX(Hand.kRight)));
  }

  ///

}

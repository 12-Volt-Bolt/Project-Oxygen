/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*
Drive subsystem formating:
  Functions use a BlankBlankBlank format
    Blank 1 determins what type of update loop the code contains
      ValueBlankBlank: runs until the paramater-set conditions have been fulfilled
      UpdateBlankBlank: needs to be called every update and turns things on OR off

    Blank 2 determins what is being run/set
      BlankMoveBlank: effects all wheels 
      BlankTurnBlank: effects fewer than all the wheels
      BlankDriveBlank: effects all wheels dynamically (IE: turn while driving)
      BlankEncoderBlank: reads/sets encoder values

    Blank 3 determins what specific input/output the function requires/returns (One function can have multiple Blank 3s)
      BlankBlankIn: input/output in inches
      BlankBlankCm: input/output in centimeters
      BlankBlankCartesian: input/output in cartesian coordinates
      BlankBlankPolar: input/output in polar coordinates
      BlankBlankSet: Sets a value dictated in paramaters/constants
      Ect...

*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
//import com.sun.tools.classfile.StackMapTable_attribute.stack_map_frame;

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
//import jdk.javadoc.internal.doclets.toolkit.resources.doclets;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem implements PIDOutput {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public  WPI_TalonSRX frontLeft = new WPI_TalonSRX(RobotMap.FRONT_LEFT_MOTOR_ID);
  public  WPI_TalonSRX frontRight = new WPI_TalonSRX(RobotMap.FRONT_RIGHT_MOTOR_ID);
  public  WPI_TalonSRX rearLeft = new WPI_TalonSRX(RobotMap.REAR_LEFT_MOTOR_ID);
  public  WPI_TalonSRX rearRight = new WPI_TalonSRX(RobotMap.REAR_RIGHT_MOTOR_ID);


  public  PIDController turnController;
  public  double rotateToAngleRate;

   final double kP = 0.05;
   final double kI = 0.0002;
   final double kD = 0.08;
   final double kF = 0.00;

  // variables for tank driving
  public  double newZero = 0.00;
  public  double rotationSpeed = 0.00;
  public  double angleOff = 0.00;

  /* This tuning parameter indicates how close to "on target" the */
  /* PID Controller will attempt to get. */

   final double kToleranceDegrees = 5.0f;

  //Variables for collision detection
   double last_world_linear_accel_x;
   double last_world_linear_accel_y;

  public boolean collisionDetected = false;

  // Collision detection threshold
  final double kCollisionThreshold_DeltaG = 0.5f;


  public MecanumDrive mecDrive = new MecanumDrive(frontLeft, rearRight, frontRight, rearLeft);

  public DriveSubsystem() {
    super();
    correctMotorDirectionForMecanumDrive();

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
    turnController.setName("Drive PID Controller");

  }
  
  public void updateDriveCartesian(double xLeft, double yLeft, double xRight) {
    mecDrive.driveCartesian(Constants_And_Equations.deadzone(OI.zeroSlotController.getX(Hand.kLeft), 0.1),
    -Constants_And_Equations.deadzone(OI.zeroSlotController.getY(Hand.kLeft), 0.1), Constants_And_Equations.deadzone(OI.zeroSlotController.getX(Hand.kRight), 0.1));  
  }

  public void driveRampNonFCD(double twist) {
    mecDrive.driveCartesian(
        Constants_And_Equations
            .parabola(Constants_And_Equations.deadzone(-OI.zeroSlotController.getX(Hand.kLeft), 0.1)),
        Constants_And_Equations
            .parabola(-Constants_And_Equations.deadzone(-OI.zeroSlotController.getY(Hand.kLeft), 0.1)),
        twist);
  }

  public void executeMecanumDrive(double ySpeed, double xSpeed, double rotation) {
    mecDrive.setSafetyEnabled(false);
     mecDrive.driveCartesian(ySpeed, xSpeed, rotation);
    } 

  public void updateDriveCartesian(double xLeft, double yLeft, double xRight, Boolean locked) {
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
      turnController.setSetpoint(-135f);
      rotateToAngle = true;
      break;
    case 270:
      turnController.setSetpoint(-90f);
      rotateToAngle = true;
      break;
    case 315:
      turnController.setSetpoint(-45f);
      rotateToAngle = true;
      break;
    }
    if (rotateToAngle) {
      turnController.enable();
      currentRotationRate = rotateToAngleRate;
    } else {
      currentRotationRate = Constants_And_Equations.deadzone(OI.zeroSlotController.getX(Hand.kRight), 0.1);
      turnController.disable();
    }
    mecDrive.driveCartesian(Constants_And_Equations.deadzone(OI.zeroSlotController.getX(Hand.kLeft), 0.1),
        -Constants_And_Equations.deadzone(OI.zeroSlotController.getY(Hand.kLeft), 0.1), currentRotationRate,
        -Robot.navXGyro.getAngle());
  }

  public void driveRampFCD(double twist) {
    mecDrive.driveCartesian(
        Constants_And_Equations
            .parabola(Constants_And_Equations.deadzone(-OI.zeroSlotController.getX(Hand.kLeft), 0.1)),
        Constants_And_Equations
            .parabola(-Constants_And_Equations.deadzone(-OI.zeroSlotController.getY(Hand.kLeft), 0.1)),
        twist, -Robot.navXGyro.getAngle());
  }

  public void turnToAngle(double angle) {
    turnController.enable();
    turnController.setSetpoint(angle);
    double currentRotationRate;
    currentRotationRate = rotateToAngleRate;
    mecDrive.driveCartesian(Constants_And_Equations.deadzone(-OI.zeroSlotController.getX(Hand.kLeft), 0.1),
        -Constants_And_Equations.deadzone(-OI.zeroSlotController.getY(Hand.kLeft), 0.1), currentRotationRate,
        -Robot.navXGyro.getAngle());

  }

  public void updateDriveRamp(double twist, double xLeft, double yLeft) {
    mecDrive.driveCartesian(Constants_And_Equations.parabola(Constants_And_Equations.deadzone(-xLeft)),
        Constants_And_Equations.parabola(-Constants_And_Equations.deadzone(-yLeft)), twist, -Robot.navXGyro.getAngle());
  }

  public void updateDriveTurn_to_angle(double angle, double xLeft, double yLeft) {
    turnController.setSetpoint(angle);
    double currentRotationRate;
    currentRotationRate = rotateToAngleRate;
    mecDrive.driveCartesian(Constants_And_Equations.deadzone(xLeft), -Constants_And_Equations.deadzone(yLeft),
        currentRotationRate, -Robot.navXGyro.getAngle());

  }

  // We may make our own mecanum method someday
  public void homeBrewMecanumMethod() {

  }

  // This method is used to rectify wheel rotation directions
  public void correctMotorDirectionForMecanumDrive() {
    // Please do not edit this unless you know the purpose of it.
    frontRight.setInverted(true);
    frontLeft.setInverted(true);
    rearLeft.setInverted(true);
    rearRight.setInverted(true);

  }

  public void setAllMotors(double speed) {
    frontRight.set(speed);
    frontLeft.set(speed);
    rearRight.set(speed);
    rearLeft.set(speed);
  }

  /// Encoder methods should go here. Please make sure to have an encoder object
  /// as a parameter.
  // Wheels have an 8 in diameter
  // Methods we need: Drive "x" distance, zero encoder each encoder, zero both
  /// enocoders

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

  // Stops all motors
  public void StopThePresses() {
    mecDrive.driveCartesian(RobotMap.MOTOR_OFF, RobotMap.MOTOR_OFF, RobotMap.MOTOR_OFF);
    mecDrive.setSafetyEnabled(true);
  }

  // Sets all motors to the speed specified in paramaters
  public void updateMoveSet(double speed) {
    frontRight.set(speed);
    frontLeft.set(speed);
    rearRight.set(speed);
    rearLeft.set(speed);

  }

  // Updates PID rotateToAngleRate to input specified in paramaters
  @Override
  public void pidWrite(double output) {
    rotateToAngleRate = output;
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
  public void InputTurnCm(WPI_TalonSRX esc, Encoder encoder, int distance, double speed) {
    encoder.reset();
    esc.set(speed);
    while (Math.abs(updateEncoderReadCm(encoder)) < Math.abs(distance)) {
    }
    esc.set(RobotMap.MOTOR_OFF);
  }

  // Move entire robot Specified distance, specified speed, in centimeters
  public void MoveDistanceCm(Encoder encoder, int distance, double speed) {
    encoderReset(encoder);
    setAllMotors(speed);
    while (Math.abs(encoderReadCm(encoder)) < Math.abs(distance)) {
    }
    StopThePresses();
  }

  // Move entire robot Specified distance, specified speed, in inches
  public void MoveDistanceIn(Encoder encoder, int distance, double speed) {
    encoderReset(encoder);
    setAllMotors(speed);
    while (Math.abs(encoderReadIn(encoder)) < Math.abs(distance)) {
    }
    StopThePresses();
  }

  // Move robot forward/backwards without rotation drifting
  // Strafe robot
  // Robot can still turn without inturupting movement
  // Relies on newZero to work
  public void MoveMecanumStraight() {
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

  // Overload- Turn 2 specified wheels, specified distance, specified speed, in
  // centimeters
  public void InputTurnCm(WPI_TalonSRX esc, WPI_TalonSRX esc2, Encoder encoder, int distance, double speed) {
    encoder.reset();
    esc.set(speed);
    esc2.set(speed);
    while (Math.abs(updateEncoderReadCm(encoder)) < Math.abs(distance)) {
    }
    esc.set(RobotMap.MOTOR_OFF);
    esc2.set(RobotMap.MOTOR_OFF);
  }

  // Turn specified wheel, specified distance, specified speed, in inches
  public void InputTurnIn(WPI_TalonSRX esc, Encoder encoder, int distance, double speed) {
    encoder.reset();
    esc.set(speed);
    while (Math.abs(updateEncoderReadIn(encoder)) < Math.abs(distance)) {
    }
    esc.set(RobotMap.MOTOR_OFF);
  }

  // Overload- Turn 2 specified wheels, specified distance, specified speed, in
  // inches
  public void InputTurnIn(WPI_TalonSRX esc, WPI_TalonSRX esc2, Encoder encoder, int distance, double speed) {
    encoder.reset();
    esc.set(speed);
    esc2.set(speed);
    while (Math.abs(updateEncoderReadIn(encoder)) < Math.abs(distance)) {
    }
    esc.set(RobotMap.MOTOR_OFF);
    esc2.set(RobotMap.MOTOR_OFF);
  }

  // Move entire robot Specified distance, specified speed, in centimeters
  public void InputMoveCm(Encoder encoder, int distance, double speed) {
    updateEncoderReset(encoder);
    updateMoveSet(speed);
    while (Math.abs(updateEncoderReadCm(encoder)) < Math.abs(distance)) {
    }
    StopThePresses();
  }

  // Move entire robot Specified distance, specified speed, in inches
  public void InputMoveIn(Encoder encoder, int distance, double speed) {
    updateEncoderReset(encoder);
    updateMoveSet(speed);
    while (Math.abs(updateEncoderReadIn(encoder)) < Math.abs(distance)) {
    }
    StopThePresses();
  }

  // Move robot forward/backwards without rotation drifting by asigning a local
  // north and turning towards that
  // Strafe robot
  // Robot can still turn without inturupting movement
  // Relies on newZero to work
  public void updateDriveLocal(double xLeft, double yLeft, double xRight) {
    mecDrive.setSafetyEnabled(false);
    // Resets local north if turning

    rotationSpeed = locRotationLock(xLeft, xRight);

    // newZero = Robot.navXGyro.getAngle();
    mecDrive.driveCartesian(Constants_And_Equations.deadzone(yLeft), Constants_And_Equations.deadzone(xLeft),
        rotationSpeed);
  }

  public double locRotationLock(double xInput, double zInput) {
    if (Constants_And_Equations.deadzone(zInput) != 0) {
      newZero = Robot.navXGyro.getAngle();
      rotationSpeed = zInput;
    } else {
      angleOff = Robot.navXGyro.getAngle() - newZero;
      if (Math.abs(angleOff) < kToleranceDegrees) {
        rotationSpeed = 0;
      } else {
        if (xInput == 0) {
          angleOff = Constants_And_Equations.deadzoneSet(Constants_And_Equations.parabola((angleOff) / 180), 0.35);
        } else {
          angleOff = Constants_And_Equations.deadzoneSet(Constants_And_Equations.parabola((angleOff) / 180), 0.2);
        }
        rotationSpeed = Constants_And_Equations.Clamp(-1, 1, angleOff);
      }
    }

    return rotationSpeed;
  }

public void collisionDetection() {
    double curr_world_linear_accel_x = Robot.navXGyro.getWorldLinearAccelX();
    double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
    last_world_linear_accel_x = curr_world_linear_accel_x;
    
    double curr_world_linear_accel_y = Robot.navXGyro.getWorldLinearAccelY();
    double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
    last_world_linear_accel_y = curr_world_linear_accel_y;
    
    if ( ( Math.abs(currentJerkX) > kCollisionThreshold_DeltaG ) ||
         ( Math.abs(currentJerkY) > kCollisionThreshold_DeltaG) ) {
        collisionDetected = true;
    }
    SmartDashboard.putBoolean("CollisionDetected", collisionDetected);

  }

}

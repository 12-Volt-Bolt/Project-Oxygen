/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*
/*  Drive subsystem formating:
/*    Functions use a BlankBlankBlank format
/*      Blank 1 determins what type of update loop the code contains
/*        ValueBlankBlank: runs until the paramater-set conditions have been fulfilled
/*        UpdateBlankBlank: needs to be called every update and turns things on OR off
/*  
/*      Blank 2 determins what is being run/set
/*        BlankMoveBlank: effects all wheels 
/*        BlankTurnBlank: effects fewer than all the wheels
/*        BlankDriveBlank: effects all wheels dynamically (IE: turn while driving)
/*        BlankEncoderBlank: reads/sets encoder values
/*  
/*      Blank 3 determins what specific input/output the function requires/returns (One function can have multiple Blank 3s)
/*        BlankBlankIn: input/output in inches
/*        BlankBlankCm: input/output in centimeters
/*        BlankBlankCartesian: input/output in cartesian coordinates
/*        BlankBlankPolar: input/output in polar coordinates
/*        BlankBlankSet: Sets a value dictated in paramaters/constants
/*        Ect...
/*  
*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
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

  public WPI_TalonSRX frontLeft;
  public WPI_TalonSRX frontRight;
  public WPI_TalonSRX rearLeft;
  public WPI_TalonSRX rearRight;

  public PIDController turnController;
  public static double rotateToAngleRate;
  private double currentRotationRate = 0;

  final double kP = 0.05;
  final double kI = 0.0002;
  final double kD = 0.08;
  final double kF = 0.00;


  // variables for LocRot driving
  private double newZero = 0.00;
  private double rotationSpeed = 0.00;
  private double angleOff = 0.00;
  private double locRotDelay; // is measured in milliseconds

  /* This tuning parameter indicates how close to "on target" the PID Controller will attempt to get. */

  final double kToleranceDegrees = 5.0f;

  // Variables for collision detection
  double last_world_linear_accel_x;
  double last_world_linear_accel_y;

  public boolean collisionDetected = false;

  // Collision detection threshold
  final double kCollisionThreshold_DeltaG = 0.5f;

  public MecanumDrive mecDrive;

  boolean isTurnControllerOn = false;

  public DriveSubsystem() {
    super();
    frontLeft = new WPI_TalonSRX(RobotMap.FRONT_LEFT_MOTOR_ID);
    frontRight = new WPI_TalonSRX(RobotMap.FRONT_RIGHT_MOTOR_ID);
    rearLeft = new WPI_TalonSRX(RobotMap.REAR_LEFT_MOTOR_ID);
    rearRight = new WPI_TalonSRX(RobotMap.REAR_RIGHT_MOTOR_ID);

    mecDrive = new MecanumDrive(frontLeft, rearRight, frontRight, rearLeft);

    correctMotorDirectionForMecanumDrive();

    turnController = new PIDController(kP, kI, kD, kF, Robot.navXGyro, this);
    turnController.setInputRange(-180.0f, 180.0f);
    turnController.setOutputRange(-0.7, 0.7);
    // Please set output range to less than 80%
    turnController.setAbsoluteTolerance(kToleranceDegrees);
    turnController.setContinuous(true);
    turnController.setName("Drive PID Controller");

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DefaultDriveCommand());
  }

  public void updateDriveCartesian(double xLeft, double yLeft, double xRight) {
    mecDrive.setSafetyEnabled(false);
    mecDrive.driveCartesian(Constants_And_Equations.deadzone(xLeft, 0.1), -Constants_And_Equations.deadzone(yLeft, 0.1),
        Constants_And_Equations.deadzone(xRight, 0.1));
  }

  public void driveRampNonFCD(double twist) {
    mecDrive.driveCartesian(
        Constants_And_Equations
            .parabola(Constants_And_Equations.deadzone(-OI.zeroSlotController.getX(Hand.kLeft), 0.1)),
        Constants_And_Equations
            .parabola(-Constants_And_Equations.deadzone(-OI.zeroSlotController.getY(Hand.kLeft), 0.1)),
        twist);
  }

  public void updateDriveCartesian(double xLeft, double yLeft, double xRight, Boolean locked) {
    mecDrive.setSafetyEnabled(false);

    if (OI.zeroSlotController.getPOV() != -1) {
      setTurnControllerSetpointDeg(OI.zeroSlotController.getPOV());
    }

    if (Math.abs(OI.zeroSlotController.getX(Hand.kRight)) > 0.2) {
      enableTurnController(false);
    }

    if (!turnController.isEnabled()) {
      currentRotationRate = Constants_And_Equations.deadzone(OI.zeroSlotController.getX(Hand.kRight), 0.1);
      enableTurnController(false);
    } else {
      currentRotationRate = rotateToAngleRate;
    }
    mecDrive.driveCartesian(Constants_And_Equations.deadzone(OI.zeroSlotController.getX(Hand.kLeft), 0.1),
        -Constants_And_Equations.deadzone(OI.zeroSlotController.getY(Hand.kLeft), 0.1), currentRotationRate,
        -Robot.navXGyro.getAngle());
  }

  public void driveRampFCD(double twist) {
    mecDrive.driveCartesian(
        Constants_And_Equations.parabola(Constants_And_Equations.deadzone(-OI.zeroSlotController.getX(Hand.kLeft), 0.1)),
        Constants_And_Equations.parabola(-Constants_And_Equations.deadzone(-OI.zeroSlotController.getX(Hand.kLeft), 0.1)),
        twist, -Robot.navXGyro.getAngle());
  }

  // "angle" is used to set a set point for the turn controller
  // The parameter should be 0 - 360 (inclusive)
  // Enables the turn controller
  public void setTurnControllerSetpointDeg(double angle) {
    enableTurnController(true);
    turnController.setSetpoint((float) Constants_And_Equations.gyroAngleForPIDLoop(angle));
  }

  // Turns robot to a specified angle, using the "angle" parameter.
  // The parameter should be 0 - 360 (inclusive)
  // Ensures that the variable "angle" is usable in the
  // setTurnControllerSetpointMethod
  // Uses obtained PID write method data
  public void turnToAngleDeg(double angle) {
    setTurnControllerSetpointDeg(Constants_And_Equations.gyroAngleForPIDLoop(angle));
    if (turnController.onTarget() || Math.abs(OI.zeroSlotController.getX(Hand.kRight)) > 0.2) {
      enableTurnController(false);
    } else {
      enableTurnController(true);
      currentRotationRate = rotateToAngleRate;
    }
    setMecanumRotationSpeedWithoutJoy(currentRotationRate);
  }

  public void setMecanumRotationSpeedWithJoy(double speed, double yLeft, double xLeft) {
    mecDrive.driveCartesian(Constants_And_Equations.deadzone(yLeft, 0.1), -Constants_And_Equations.deadzone(xLeft, 0.1), speed, -Robot.navXGyro.getAngle());
  }

  public void setMecanumRotationSpeedWithoutJoy(double speed) {
    mecDrive.driveCartesian(0, 0, speed, -Robot.navXGyro.getAngle());
  }

  public void setMecanumStrafeSpeedWithJoy(double speed, double yLeft, double xRight) {
    updateDriveLocalStrafe(yLeft, speed, xRight);
  }

  public void setMecanumStrafeSpeedWithoutJoy(double speed) {
    updateDriveLocalStrafe(0, speed, 0);
  }

  public void setMecanumVerticalSpeedWithJoy(double speed, double xLeft, double xRight) {
    if(!turnController.isEnabled()){
      turnController.setSetpoint(Robot.navXGyro.getYaw());
    }
    mecDrive.driveCartesian(speed, -Constants_And_Equations.deadzone(xLeft, 0.1), Constants_And_Equations.deadzone(xRight, 0.1));
  }

  public void setMecanumVerticalSpeedWithoutJoy(double speed) {
  /*  if(!turnController.isEnabled()){
      turnController.setSetpoint(Robot.navXGyro.getYaw());
    }
    */
    mecDrive.driveCartesian(speed, 0, 0);
  }


  public void enableTurnController(boolean trueOrFalse) {
    if (trueOrFalse) {
      turnController.enable();
    } else {
      turnController.disable();
    }
  }


  public void updateDriveRamp(double xLeft, double yLeft, double twist) {
    mecDrive.driveCartesian(Constants_And_Equations.parabola(Constants_And_Equations.deadzone(-xLeft)),
        Constants_And_Equations.parabola(-Constants_And_Equations.deadzone(-yLeft)), twist, -Robot.navXGyro.getAngle());
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

  // Updates rotateToAngleRate to input specified in parameters
  @Override
  public void pidWrite(double output) {
    rotateToAngleRate = output;
  }

  // Move robot forward/backwards without rotation drifting
  // Strafe robot
  // Robot can still turn without inturupting movement
  // Relies on newZero to work
  public void moveMecanumStraight() {
    mecDrive.setSafetyEnabled(false);

    // if turning, reset newZero
    if (Constants_And_Equations.deadzone(OI.zeroSlotController.getX(Hand.kRight)) != 0) {
      newZero = Robot.navXGyro.getAngle();
    }
    // drive forward based on leftJoyX, Strafe based on LeftJoyY, turn based on
    // AmountDrifted+RightJoyX
    mecDrive.driveCartesian(Constants_And_Equations.deadzone(OI.zeroSlotController.getY(Hand.kLeft)),
        Constants_And_Equations.deadzone(OI.zeroSlotController.getX(Hand.kLeft)), Math.round(Robot.navXGyro.getAngle() - newZero) + Constants_And_Equations.deadzone(OI.zeroSlotController.getX(Hand.kRight)));
  }

  public void moveMecanumStraight(double speed) {
    mecDrive.setSafetyEnabled(false);

    // if turning, reset newZero
    if (Constants_And_Equations.deadzone(OI.zeroSlotController.getX(Hand.kRight)) != 0) {
      newZero = Robot.navXGyro.getAngle();
    }
    // drive forward based on leftJoyX, Strafe based on LeftJoyY, turn based on
    // AmountDrifted+RightJoyX
    mecDrive.driveCartesian(Constants_And_Equations.deadzone(OI.zeroSlotController.getY(Hand.kLeft)), speed, Math.round(Robot.navXGyro.getAngle() - newZero) + Constants_And_Equations.deadzone(OI.zeroSlotController.getX(Hand.kRight)));
  }
  
  // Move robot forward/backwards without rotation drifting by asigning a local
  // north and turning towards that
  // Strafe robot
  // Robot can still turn without inturupting movement
  // Relies on newZero to work
  public void updateDriveLocalStrafe(double yLeft, double xLeft, double xRight) {
    mecDrive.setSafetyEnabled(false);
    // Resets local north if turning

    rotationSpeed = locRotationLock(xLeft, -xRight);

    // newZero = Robot.navXGyro.getAngle();
    mecDrive.driveCartesian(Constants_And_Equations.deadzone(xLeft), Constants_And_Equations.deadzone(-yLeft), rotationSpeed);
  }

  public double locRotationLock(double xInput, double zInput) {
    // if Z axis joystick is moving or has moved within the past 0.4 seconds set doLocRot to "true", else leave as "false"
    boolean doLocRot = false;
    if (Constants_And_Equations.deadzone(zInput) != 0) {
      locRotDelay = System.currentTimeMillis();
    } else if (System.currentTimeMillis() - locRotDelay > 400) {
      doLocRot = true;
    }

    // if not locking rotation, roationSpeed equals Z axis input
    if (doLocRot == false) {
      newZero = Robot.navXGyro.getAngle();
      rotationSpeed = zInput;
    } else {
      angleOff = Robot.navXGyro.getAngle() - newZero;
      // if "angleOff" is greater than kToleranceDegrees, rotationSpeed equals zero and robot does not turn
      if (Math.abs(angleOff) < kToleranceDegrees) {
        rotationSpeed = 0;
      } else {
        // if strafing, set locRot power lower beacuse the wheels are already turning, they dont need to pass the minimum required torque
        double turnPower;
        if (xInput != 0) {
          turnPower = Constants_And_Equations.deadzoneSet(Constants_And_Equations.parabola((angleOff) / 180), 0.25); // turnPower equals 
        } else {
          turnPower = Constants_And_Equations.deadzoneSet(Constants_And_Equations.parabola((angleOff) / 180), 0.35);
        }
        rotationSpeed = Constants_And_Equations.Clamp(-1, 1, turnPower);
      }
    }
    SmartDashboard.putNumber("newZero", newZero);
    SmartDashboard.putNumber("Rotation speed", -rotationSpeed);

    return -rotationSpeed;
  }

  public void collisionDetection() {
    double curr_world_linear_accel_x = Robot.navXGyro.getWorldLinearAccelX();
    double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
    last_world_linear_accel_x = curr_world_linear_accel_x;

    double curr_world_linear_accel_y = Robot.navXGyro.getWorldLinearAccelY();
    double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
    last_world_linear_accel_y = curr_world_linear_accel_y;

    if ((Math.abs(currentJerkX) > kCollisionThreshold_DeltaG)
        || (Math.abs(currentJerkY) > kCollisionThreshold_DeltaG)) {
      collisionDetected = true;
    }
    SmartDashboard.putBoolean("CollisionDetected", collisionDetected);
  }


  
}

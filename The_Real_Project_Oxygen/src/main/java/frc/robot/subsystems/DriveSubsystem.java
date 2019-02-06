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
  
  //PID, YAY!!!
  public static PIDController turnController;
  public static double rotateToAngleRate;

  static final double kP = 0.03;
  static final double kI = 0.00;
  static final double kD = 0.00;
  static final double kF = 0.00;

  /* This tuning parameter indicates how close to "on target" the    */
/* PID Controller will attempt to get.                             */

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
      turnController.setInputRange(-180.0f,  180.0f);
      turnController.setOutputRange(-1.0, 1.0);
      turnController.setAbsoluteTolerance(kToleranceDegrees);
      turnController.setContinuous(true);
      
  }

  public void executeMecanumDrive() {
    mecDrive.setSafetyEnabled(false);
     
    boolean rotateToAngle = false;
          
          if ( OI.zeroSlotController.getPOV() == 0) {
              turnController.setSetpoint( 0.0f);
              rotateToAngle = true;
          } 
          else if ( OI.zeroSlotController.getPOV() == 45) {
            turnController.setSetpoint(45.0f);
            rotateToAngle = true;
          }
          
          else if ( OI.zeroSlotController.getPOV() == 90) {
              turnController.setSetpoint(90.0f);
              rotateToAngle = true;
          } 
          else if ( OI.zeroSlotController.getPOV() == 135) {
            turnController.setSetpoint(135.0f);
            rotateToAngle = true;
          }
          
          else if ( OI.zeroSlotController.getPOV() == 180) {
              turnController.setSetpoint(179.9f);
              rotateToAngle = true;
          } 
          else if ( OI.zeroSlotController.getPOV() == 225) {
            turnController.setSetpoint(225.0f);
            rotateToAngle = true;
          }
          else if ( OI.zeroSlotController.getPOV() == 270) {
              turnController.setSetpoint(270.0f);
              rotateToAngle = true;
          }
          else if ( OI.zeroSlotController.getPOV() == 315) {
            turnController.setSetpoint(315.0f);
            rotateToAngle = true;
          }
          
          double currentRotationRate;
        
          if ( rotateToAngle ) {
              turnController.enable();
              currentRotationRate = rotateToAngleRate;
          } else {
              turnController.disable();
              currentRotationRate = Constants_And_Equations.deadzone(-OI.zeroSlotController.getX(Hand.kRight), 0.1);
            }
              /* Use the joystick X axis for lateral movement,          */
              /* Y axis for forward movement, and the current           */
              /* calculated rotation rate (or joystick Z axis),         */
              /* depending upon whether "rotate to angle" is active.    */
    
              mecDrive.driveCartesian(Constants_And_Equations.deadzone(-OI.zeroSlotController.getX(Hand.kLeft), 0.1), -Constants_And_Equations.deadzone(-OI.zeroSlotController.getY(Hand.kLeft), 0.1), currentRotationRate, -Robot.navXGyro.getAngle());
   
    } 

    public void driveRamp(){
      mecDrive.driveCartesian(Constants_And_Equations.parabola(Constants_And_Equations.deadzone(-OI.zeroSlotController.getX(Hand.kLeft), 0.1)), Constants_And_Equations.parabola(-Constants_And_Equations.deadzone(-OI.zeroSlotController.getY(Hand.kLeft), 0.1)), currentRotationRate, -Robot.navXGyro.getAngle());
    }
 
    public static void turnToAngle(double angle) {
      turnController.setSetpoint(angle);
      turnController.enable();
      double currentRotationRate;
      currentRotationRate = rotateToAngleRate;
      mecDrive.driveCartesian(Constants_And_Equations.deadzone(-OI.zeroSlotController.getX(Hand.kLeft), 0.1), -Constants_And_Equations.deadzone(-OI.zeroSlotController.getY(Hand.kLeft), 0.1), currentRotationRate, -Robot.navXGyro.getAngle());

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

/// Encoder methods should go here. Please make sure to have an encoder object as a parameter.
// Wheels have an 8 in diameter
// Methods we need: Drive "x" distance, zero encoder each encoder, zero both enocoders

// Reset one encoder
public static void EncoderReset(Encoder encoder) 
{
  encoder.reset();
}

// Reset both encoders
public static void EncoderReset(Encoder encoder1, Encoder encoder2)
{
  encoder1.reset();
  encoder2.reset();
}

// Read encoder and calculate distance turned in Centimeters
public static int EncoderReadCm(Encoder encoder)
{
  return (int) Math.round(encoder.get() * 63.84);
}

// Read encoder and calculate distance turned in inches
public static int EncoderReadIn(Encoder encoder)
{
  return (int) Math.round(encoder.get() * 25.13);

}

// Turn specified wheel, specified distance, specified speed, in centimeters
public static void TurnWheelDistanceCm(WPI_TalonSRX esc, Encoder encoder, int distance, double speed)
{
  encoder.reset();
  esc.set(speed);
  while(Math.abs(EncoderReadCm(encoder)) < Math.abs(distance)){
  }
  esc.set(RobotMap.MOTOR_OFF);
}

// Turn specified wheel, specified distance, specified speed, in inches
public static void TurnWheelDistanceIn(WPI_TalonSRX esc, Encoder encoder, int distance, double speed)
{
  encoder.reset();
  esc.set(speed);
  while(Math.abs(EncoderReadIn(encoder)) < Math.abs(distance)){
  }
  esc.set(RobotMap.MOTOR_OFF);
}

// Move entire robot Specified distance, specified speed, in centimeters
public static void MoveDistanceCm(Encoder encoder, int distance, double speed)
{
  EncoderReset(encoder);
  setAllMotors(speed);
  while(Math.abs(EncoderReadCm(encoder)) < Math.abs(distance)){
  }
  StopThePresses();
}

// Move entire robot Specified distance, specified speed, in inches
public static void MoveDistanceIn(Encoder encoder, int distance, double speed)
{
  EncoderReset(encoder);
  setAllMotors(speed);
  while(Math.abs(EncoderReadIn(encoder)) < Math.abs(distance)){
  }
  StopThePresses();
}

///

}

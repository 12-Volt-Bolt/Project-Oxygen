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
public static PIDController turnController;
  /**
   *
   */

public static double rotateToAngleRate;

  static final double kP = 0.03;
  static final double kI = 0.00;
  static final double kD = 0.00;
  static final double kF = 0.00;

  /* This tuning parameter indicates how close to "on target" the    */
/* PID Controller will attempt to get.                             */

static final double kToleranceDegrees = 2.0f;

// Remember that you switched the front wheels
public static MecanumDrive mecDrive = new MecanumDrive(frontLeft, rearRight, frontRight, rearLeft);
//public static MecanumDrive fsdaf = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor)
  
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

      // Motor ramping
      
  }

  public void executeMecanumDrive() {
    mecDrive.setSafetyEnabled(false);
     
    

    boolean rotateToAngle = false;
          
   SmartDashboard.putNumber("POV", OI.zeroSlotController.getPOV());
          if (OI.zeroSlotController.getPOV() == 0) {
              turnController.setSetpoint( 0.0f);
              rotateToAngle = true;
          } 
          else if ( OI.zeroSlotController.getPOV() == 45) {
            turnController.setSetpoint(45.0f);
            rotateToAngle = true;
          }
          
          else if (OI.zeroSlotController.getPOV() == 90) {
              turnController.setSetpoint(90.0f);
              rotateToAngle = true;
          } 
          else if ( OI.zeroSlotController.getPOV() == 135) {
            turnController.setSetpoint(135.0f);
            rotateToAngle = true;
          }
          
          else if (OI.zeroSlotController.getPOV() == 180) {
              turnController.setSetpoint(179.9f);
              rotateToAngle = true;
          } 
          else if (OI.zeroSlotController.getPOV() == 225) {
            turnController.setSetpoint(225.0f);
            rotateToAngle = true;
          }
          else if (OI.zeroSlotController.getPOV() == 270) {
              turnController.setSetpoint(270.0f);
              rotateToAngle = true;
          }
          else if ( OI.zeroSlotController.getPOV() == 315) {
            turnController.setSetpoint(315.0f);
            rotateToAngle = true;
    }
          else if(OI.zeroSlotController.getYButtonPressed() == true) {
           setAllMotors(0.5);

           if(Robot.navXGyro.getAngle() != 0 ){
            turnController.setSetpoint(0f);
             rotateToAngle = true;
           }

          }
          double currentRotationRate;

          if ( rotateToAngle ) {
            turnController.enable();
              currentRotationRate = rotateToAngleRate;
          } else {
            turnController.disable();
              // I don't know why getX has to be negitive, but let's just go with it
              if(Math.abs(OI.zeroSlotController.getX(Hand.kRight)) > 0.05) {
                currentRotationRate = Constants_And_Equations.deadzone(-OI.zeroSlotController.getX(Hand.kRight), 0.1);
              }
              else {
                currentRotationRate = 0;
              }
                  
              

            }
              /* Use the joystick X axis for lateral movement,          */
              /* Y axis for forward movement, and the current           */
              /* calculated rotation rate (or joystick Z axis),         */
              /* depending upon whether "rotate to angle" is active.    */
    
              mecDrive.driveCartesian(Constants_And_Equations.deadzone(-OI.zeroSlotController.getX(Hand.kLeft), 0.1), -Constants_And_Equations.deadzone(-OI.zeroSlotController.getY(Hand.kLeft), 0.1), currentRotationRate, -Robot.navXGyro.getAngle());
              SmartDashboard.putNumber("Gyro angle", Robot.navXGyro.getAngle());
         
     /////
     
     ////


    } 
 
    public static void turnToAngle(double angle) {
      turnController.setSetpoint(angle);
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
public void StopThePresses() {
  mecDrive.driveCartesian(RobotMap.MOTOR_OFF, RobotMap.MOTOR_OFF, RobotMap.MOTOR_OFF);
}

public void setAllMotors(double speed) {
frontRight.set(speed);
frontLeft.set(speed);
rearRight.set(speed);
rearLeft.set(speed);

}

public void pidWrite(double output) {
  rotateToAngleRate = output;
}

/////////
// Please make it so motor speed will "ramp" if the Cantalon motor controller speed is less than 15%
// Please make it so 
//       fron./tLeft.configOpenloopRamp(0.15);
//  
////////




}

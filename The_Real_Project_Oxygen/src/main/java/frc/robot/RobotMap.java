/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2
  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

  public static final int CONTROLLER_ONE_ID = 0;
  public static final int CONTROLLER_TWO_ID = 1;
  
  // These are the wheel motor controller IDs
  public static final int FRONT_LEFT_MOTOR_ID = 2;
  public static final int FRONT_RIGHT_MOTOR_ID = 1;
  public static final int REAR_LEFT_MOTOR_ID = 4;
  public static final int REAR_RIGHT_MOTOR_ID = 3;

  // Solenoid IDs
  public static final int TOP_SOLENOID_ID = 1;
  public static final int BOTTOM_SOLENOID_ID = 2;

  // MAGIC NUMBERS
  public final static int MOTOR_OFF = 0;

  public static int fillIn = 0;
 
  // The PDP
  public static final int PDP_ID = 0;	
	// The below would the IDs for buttons on any HID controller (basically any USB controller)
  // If using 
  
	public static int LEFT_X_AXIS_ID = 0;
	public static int LEFT_Y_AXIS_ID = 1;

	public static int LEFT_TRIGGER_AXIS_ID = 2; 
	public static int RIGHT_TRIGGER_AXIS_ID = 3; 

	public static int RIGHT_X_AXIS_ID = 4;
	public static int RIGHT_Y_AXIS_ID = 5;

	public static int A_BUTTON_ID = 1;
	public static int B_BUTTON_ID = 2; 
	public static int X_BUTTON_ID = 3; 
	public static int Y_BUTTON_ID = 4;
	public static int LEFT_BUMPER_ID = 5;
	public static int RIGHT_BUMPER_ID = 6; 
}

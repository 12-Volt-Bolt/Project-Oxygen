/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  public static XboxController zeroSlotController = new XboxController(RobotMap.CONTROLLER_ZERO_ID);
  public static XboxController oneSlotController = new XboxController(RobotMap.CONTROLLER_ONE_ID);

  JoystickButton zeroXConJoyYButton = new JoystickButton(zeroSlotController, RobotMap.Y_BUTTON_ID),
      zeroXJoyJoyBButton = new JoystickButton(zeroSlotController, RobotMap.B_BUTTON_ID),
      zeroXJoyAButton = new JoystickButton(zeroSlotController, RobotMap.A_BUTTON_ID),
      zeroXJoyXButton = new JoystickButton(zeroSlotController, RobotMap.X_BUTTON_ID),
      zeroXJoyRBButton = new JoystickButton(zeroSlotController, RobotMap.RIGHT_BUMPER_ID),
      zeroXJoyLBButton = new JoystickButton(zeroSlotController, RobotMap.LEFT_BUMPER_ID),
      zeroXJoyRTButton = new JoystickButton(zeroSlotController, RobotMap.RIGHT_TRIGGER_AXIS_ID),
      zeroXJoyLTButton = new JoystickButton(zeroSlotController, RobotMap.LEFT_TRIGGER_AXIS_ID),
      zeroXJoyLeftStickButton = new JoystickButton(zeroSlotController, RobotMap.LEFT_STICK_ID),
      zeroXJoyRightStickButton = new JoystickButton(zeroSlotController, RobotMap.RIGHT_STICK_ID),
      zeroXJoyMenuButton = new JoystickButton(zeroSlotController, RobotMap.MENU_BUTTON_ID),
      zeroXJoyStartButton = new JoystickButton(zeroSlotController, RobotMap.START_BUTTON_ID);

  JoystickButton xJoyYButton = new JoystickButton(oneSlotController, RobotMap.Y_BUTTON_ID),
      oneXJoyBButton = new JoystickButton(oneSlotController, RobotMap.B_BUTTON_ID),
      oneXJoyAButton = new JoystickButton(oneSlotController, RobotMap.A_BUTTON_ID),
      oneXJoyXButton = new JoystickButton(oneSlotController, RobotMap.X_BUTTON_ID),
      oneXJoyRBButton = new JoystickButton(oneSlotController, RobotMap.RIGHT_BUMPER_ID),
      oneXJoyRTButton = new JoystickButton(oneSlotController, RobotMap.RIGHT_TRIGGER_AXIS_ID),
      oneXJoyLBButton = new JoystickButton(oneSlotController, RobotMap.LEFT_BUMPER_ID),
      oneXJoyLTButton = new JoystickButton(oneSlotController, RobotMap.LEFT_TRIGGER_AXIS_ID),
      oneXJoyLeftStickButton = new JoystickButton(oneSlotController, RobotMap.LEFT_STICK_ID),
      oneXJoyRightStickButton = new JoystickButton(oneSlotController, RobotMap.RIGHT_STICK_ID),
      oneXJoyMenuButton = new JoystickButton(oneSlotController, RobotMap.MENU_BUTTON_ID),
      oneXJoyStartButton = new JoystickButton(oneSlotController, RobotMap.START_BUTTON_ID);

  
  public static boolean visionStartCombo() {
     if(OI.zeroSlotController.getTriggerAxis(Hand.kLeft) > 0.5 && OI.zeroSlotController.getTriggerAxis(Hand.kRight) > 0.5) {
         return true;
     }
     return false;
  }

  public static boolean allButtonComboPressesd(XboxController xJoy) {
    return xJoy.getAButtonPressed() && xJoy.getBButtonPressed() && xJoy.getXButtonPressed() && xJoy.getYButtonPressed();
  }

  public static void leftRumbleMethod(XboxController xCon, double power) {
    xCon.setRumble(RumbleType.kLeftRumble, power);
  }

  public static void leftRumbleMethod(XboxController xCon, double power, double delay) {
    int powerOff = 0;
    leftRumbleMethod(xCon, power);
    Timer.delay(delay);
    leftRumbleMethod(xCon, powerOff);
  }

  public static void rightRumbleMethod(XboxController xCon, double power) {
    xCon.setRumble(RumbleType.kRightRumble, power);
  }

  public static void rightRumbleMethod(XboxController xCon, double power, double delay) {
    rightRumbleMethod(xCon, power);
    Timer.delay(delay);
    rightRumbleMethod(xCon, 0);
  }

  public static void doubleRumbleMethod(XboxController xCon, double leftPower, double rightPower) {
    leftRumbleMethod(xCon, leftPower);
    rightRumbleMethod(xCon, rightPower);

  }

  public static double getZeroZ() {
    return zeroSlotController.getX(Hand.kRight);
  }
  
  public static double getOneZ() {
    return oneSlotController.getX(Hand.kRight);
  }

}

//// CREATING BUTTONS
// One type of button is a joystick button which is any button on a
//// joystick.
// You create one by telling it which joystick it's on and which button
// number it is.
// Joystick stick = new Joystick(port);
// Button button = new JoystickButton(stick, buttonNumber);

// There are a few additional built in buttons you can use. Additionally,
// by subclassing Button you can create custom triggers and bind those to
// commands the same as any other Button.

//// TRIGGERING COMMANDS WITH BUTTONS
// Once you have a button, it's trivial to bind it to a button in one of
// three ways:

// Start the command when the button is pressed and let it run the command
// until it is finished as determined by it's isFinished method.
// button.whenPressed(new ExampleCommand());

// Run the command while the button is being held down and interrupt it once
// the button is released.
// button.whileHeld(new ExampleCommand());

// Start the command when the button is released and let it run the command
// until it is finished as determined by it's isFinished method.
// button.whenReleased(new ExampleCommand());
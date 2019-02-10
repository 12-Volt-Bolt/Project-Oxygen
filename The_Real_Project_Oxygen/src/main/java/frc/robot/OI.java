/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
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
  public static XboxController OneSlotController = new XboxController(RobotMap.CONTROLLER_ONE_ID);
  
  JoystickButton zeroXConJoyYButton = new JoystickButton(zeroSlotController, RobotMap.Y_BUTTON_ID),
                 zeroXConJoyBButton = new JoystickButton(zeroSlotController, RobotMap.B_BUTTON_ID),
                 zeroXConAButton = new JoystickButton(zeroSlotController, RobotMap.A_BUTTON_ID),
                 zeroXConXButton = new JoystickButton(zeroSlotController, RobotMap.X_BUTTON_ID),
                 zeroXConRBButton = new JoystickButton(zeroSlotController, RobotMap.RIGHT_BUMPER_ID),
                 zeroXConRTButton = new JoystickButton(zeroSlotController, RobotMap.RIGHT_TRIGGER_AXIS_ID),
                 zeroXConLBButton = new JoystickButton(zeroSlotController, RobotMap.LEFT_BUMPER_ID),
                 zeroXConLTButton = new JoystickButton(zeroSlotController, RobotMap.LEFT_TRIGGER_AXIS_ID),
                 zeroXConLeftStickButton = new JoystickButton(zeroSlotController, RobotMap.LEFT_STICK_ID),
                 zeroXConRightStickButton = new JoystickButton(zeroSlotController, RobotMap.RIGHT_STICK_ID),
                 zeroXConMenuButton = new JoystickButton(zeroSlotController, RobotMap.MENU_BUTTON_ID),
                 zeroXConStartButton = new JoystickButton(zeroSlotController, RobotMap.START_BUTTON_ID);
                 

  
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

public static void leftRumbleMethod(XboxController xCon, double power) {
  xCon.setRumble(RumbleType.kLeftRumble, power);
}
public static void leftRumbleMethod(XboxController xCon, double power, double delay) {
  leftRumbleMethod(power);
  Timer.delay(delay);
  leftRumbleMethod(0);
}

public static void rightRumbleMethod((XboxController xCon, double power) {
  xCon.setRumble(RumbleType.kRightRumble, power);
}

public static void rightRumbleMethod(double power, double delay) {
  rightRumbleMethod(power);
  Timer.delay(delay);
  rightRumbleMethod(0);
}


public static void doubleRumbleMethod( double leftPower, double rightPower) {
leftRumbleMethod(leftPower);
rightRumbleMethod(rightPower);
}


public static void doubleRumbleMethod( double leftPower, double leftDelay, double rightPower, double rightDelay) {
  leftRumbleMethod(leftPower, leftDelay);
  rightRumbleMethod(rightPower, rightDelay);
  }

}
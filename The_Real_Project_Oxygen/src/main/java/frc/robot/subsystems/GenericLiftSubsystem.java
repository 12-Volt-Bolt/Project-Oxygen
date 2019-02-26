/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class GenericLiftSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private WPI_TalonSRX frontLift = new WPI_TalonSRX(RobotMap.FRONT_RAIL_MOTOR_ID);
  private WPI_TalonSRX rearLift = new WPI_TalonSRX(RobotMap.REAR_RAIL_MOTOR_ID);
  private WPI_TalonSRX topLift = new WPI_TalonSRX(RobotMap.TOP_RAIL_MOTOR_ID);

  private int[] liftID = new int[] { 7,6,5 };

  enum LiftId {
    frontLift(7)
    ,rearLift(6)
    ,topLift(5);

    private int value;

    private void LiftIds(int value) {
      this.value = value;
    }
    public int getValue() {
      return value;
    }
  }

  //public int frontLift = RobotMap.FRONT_RAIL_MOTOR_ID;
  //public int rearLift = RobotMap.REAR_RAIL_MOTOR_ID;
  //public int topLift = RobotMap.TOP_RAIL_MOTOR_ID;

  public GenericLiftSubsystem() {
  }

  public void liftMethod(double speed, String liftID) {

  }

  public void setMotor(double speed, int liftID) {
    switch (liftID) {
      case 7:
        
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}

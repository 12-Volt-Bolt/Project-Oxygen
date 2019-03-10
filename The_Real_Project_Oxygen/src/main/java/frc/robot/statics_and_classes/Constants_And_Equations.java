/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.statics_and_classes;

import frc.robot.OI;

import java.sql.Time;

//import com.sun.tools.classfile.StackMapTable_attribute.stack_map_frame;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * Add your docs here.
 */
public class Constants_And_Equations {

    public enum AxisNames {
        leftX, leftY, rightX, rightY;
    }

    public static final int zero = 0;
    public static final double rampTimeInSecs = 0.2;

    public static double a_Vision_Variable;
    public static final int NT_Table_Constant = 999999;
    public static final int visionCaliConstant = 1;

    public static double Clamp(double min, double max, double value) {
        if (value > max) {
            return max;
        } else if (value < min) {
            return min;
        } else {
            return value;
        }
    }

    public static double deadzone(double d) {
        return Math.abs(d) > 0.1 ? d : 0;
    }

    // Overload-
    public static double deadzone(double d, double deadzone) {
        return Math.abs(d) > Math.abs(deadzone) ? d : 0;
    }

    public static double deadzoneSet(double d, double deadzone) {
        if (Math.abs(d) > Math.abs(deadzone)) {
            return d;
        } else {
            if (d > 0) {
                return deadzone;
            } else {
                return -deadzone;
            }
        }
    }

    public static double parabola(double d) {
        double dten = d * 10;
        if (dten > 0) {
            return Math.pow(dten, 2) / 10;
        } else {
            return (Math.pow(dten, 2) / 10) * -1;
        }
    }

    public static double triggersAsJoy() {
        double leftTrig = deadzone(OI.oneSlotController.getTriggerAxis(Hand.kLeft));
        double rightTrig = deadzone(OI.oneSlotController.getTriggerAxis(Hand.kRight));
        if (leftTrig != 0 && rightTrig != 0) {
            return 0;
        } else if (leftTrig > 0) {
            return leftTrig;
        } else if (rightTrig > 0) {
            return -rightTrig;
        } else {
            return 0;
        }
    }

    public static float gyroAngleForPIDLoop(double angle) {
        if (angle > 180) {
            angle -= 360;
        } else if ((int) angle == 180) {
            angle = 179.9;
        }
        return (float) angle;
    }

    // The following method returns a scaled version of currentValue.
    // scale is the maximum and minimum, inclusive
    public static double scaleValue(double value, double currentValue, double scale) {
        return (scale / value) * currentValue;
    }
    
    // The method takes in two doubles as parameters and uses them as positive
    // and negative values for an axis
    // deadzone1 and deadzone2 are thresholds
    public static double turnIntoAxis(double axisPos, double axisNeg, double deadzone1, double deadzone2) {
        return (Constants_And_Equations.deadzone(axisPos, deadzone1) + Constants_And_Equations.deadzone(-axisNeg, deadzone2));
    }
}

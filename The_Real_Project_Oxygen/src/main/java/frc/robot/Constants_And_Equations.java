/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.sql.Time;

//import com.sun.tools.classfile.StackMapTable_attribute.stack_map_frame;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * Add your docs here.
 */
public class Constants_And_Equations {

    public static double Clamp(double min, double max, double value)
    {
        if (value > max){
            return max;
        }
        else if (value < min){
            return min;
        }
        else{
            return value;
        }
    }

    public static double deadzone(double d){
        return Math.abs(d) > 0.1 ? d : 0;
    }

    // Overload- 
    public static double deadzone(double d, double deadzone){
        return Math.abs(d) > Math.abs(deadzone) ? d : 0;
    }

    public static double deadzoneSet(double d, double deadzone){
        if (Math.abs(d) > Math.abs(deadzone)){
            return d;
        }
        else{
            if (d > 0){
                return deadzone;
            }
            else{
                return -deadzone;
            }
        }
    }

    public static double parabola(double d){
        double dten = d*10;
        if (dten > 0) {
            return Math.pow(dten, 2)/10;
        } else {
            return (Math.pow(dten, 2)/10)*-1;
        }
    }
}

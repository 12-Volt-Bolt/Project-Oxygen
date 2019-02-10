/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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

    public static double parabola(double d){
        double dten = d*10;
        return dten;
    }

    public static double powNoSignChange(double d){
        return powNoSignChange(d, 2);
    }

    public static double powNoSignChange(double d, double pow){
        double dten = d*10;
        double temp = Math.pow(dten, pow);
        if (checkValuePositive(dten) == checkValuePositive(temp)){
            return temp;
        } else {
            return temp * -1;
        }
    }

    public static boolean checkValuePositive(double value){
        if (value > 0){
            return true;
        }
        else{
            return false;
        }
    }



}
